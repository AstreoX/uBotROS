#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "Containers/ArrayView.h"
#include "Engine/Engine.h"
#include "Engine/EngineBaseTypes.h"
#include "Engine/World.h"
#include "EngineUtils.h"
#include "GameFramework/Actor.h"
#include "HAL/PlatformMisc.h"
#include "HAL/PlatformProcess.h"
#include "HAL/PlatformTime.h"
#include "Misc/ScopeExit.h"
#include "UBotROS.h"
#include "UBotROSBridgeSubsystem.h"
#include "UBotROSConverters.h"
#include "UBotROSMessageTypes.h"
#include "UBotROSAttitudeCommandComponent.h"
#include "UBotROSFlightCommandTestTypes.h"
#include "UBotROSMotorCommandComponent.h"
#include "UBotROSSettings.h"
#include "UBotROSTypes.h"
#include "UBotRightHandedConventions.h"
#include "UBotSensorTypes.h"
#include "UBotTypes.h"

// End-to-end check against a real ubot_ros_bridge. The ROS side
// is ros2/ubot_ros_bridge/test/e2e_check.py, which verifies what this test publishes and publishes the
// cmd_vel Twist this test waits for. Run it with:
//   WSL:  source /opt/ros/humble/setup.bash
//         cd ros2/ubot_ros_bridge && python3 -m ubot_ros_bridge.bridge_node --port 8268 &
//         python3 test/e2e_check.py
//   UE:   set UBOT_ROS_E2E=1, then Automation RunTests UBotROS.EndToEnd.Bridge
namespace UBotROSEndToEndTests
{
    const TCHAR* const EnableVariable = TEXT("UBOT_ROS_E2E");
    const TCHAR* const BridgeHost = TEXT("127.0.0.1");
    constexpr int32 BridgePort = 8268;

    const TCHAR* const ImuTopic = TEXT("/ubot_e2e/imu");
    const TCHAR* const PointsTopic = TEXT("/ubot_e2e/points");
    const TCHAR* const CmdVelTopic = TEXT("/ubot_e2e/cmd_vel");
    const TCHAR* const ImuFrameId = TEXT("e2e_imu");
    const TCHAR* const LidarFrameId = TEXT("e2e_lidar");
    const TCHAR* const BaseFrameId = TEXT("base_link");

    constexpr double FixtureStampSeconds = 12.5;
    constexpr double PublishPeriodSeconds = 0.1;     // 10 Hz
    constexpr double MinimumPublishSeconds = 5.0;    // keep publishing for the checker even after cmd_vel arrived
    constexpr double TimeoutSeconds = 20.0;
    constexpr float LoopSleepSeconds = 0.005f;

    // ROS values (section 12).
    const FVector ImuAngularVelocityROS(0.1, 0.2, 0.3);
    const FVector ImuLinearAccelerationROS(0.0, 0.0, 9.81);
    const FVector CmdVelLinear(1.5, 0.0, 0.0);
    const FVector CmdVelAngular(0.0, 0.0, 0.25);

    struct FCloudPoint
    {
        float X;
        float Y;
        float Z;
        float Intensity;
        uint16 Ring;
        float Time;
    };

    const FCloudPoint CloudPoints[] =
    {
        { 1.0f, 2.0f, 3.0f, 10.0f, 0, 0.0f },
        { 4.0f, 5.0f, 6.0f, 20.0f, 1, 0.0f },
        { 7.0f, 8.0f, 9.0f, 30.0f, 2, 0.0f },
    };

    /** State shared with the cmd_vel callback (runs on the game thread inside the subsystem tick). */
    struct FCommandState
    {
        int32 MessagesReceived = 0;
        int32 UndecodableMessages = 0;
        bool bMatched = false;
        bool bHasLast = false;
        FUBotRosTwist Last;
    };

    /** Points the bridge settings at the fixture for the scope (no auto-connect, clock on); restores them afterwards. */
    class FScopedE2ESettings
    {
    public:
        UE_NONCOPYABLE(FScopedE2ESettings);

        FScopedE2ESettings()
            : Settings(GetMutableDefault<UUBotROSSettings>())
        {
            if (Settings)
            {
                SavedHost = Settings->BridgeHost;
                SavedPort = Settings->BridgePort;
                bSavedAutoConnect = Settings->bAutoConnect;
                bSavedPublishClock = Settings->bPublishClock;
                SavedClockTopic = Settings->ClockTopic;

                Settings->BridgeHost = BridgeHost;
                Settings->BridgePort = BridgePort;
                // The test connects explicitly with ConnectTo once everything is set up.
                Settings->bAutoConnect = false;
                Settings->bPublishClock = true;
                Settings->ClockTopic = TEXT("/clock");
            }
        }

        ~FScopedE2ESettings()
        {
            if (Settings)
            {
                Settings->BridgeHost = SavedHost;
                Settings->BridgePort = SavedPort;
                Settings->bAutoConnect = bSavedAutoConnect;
                Settings->bPublishClock = bSavedPublishClock;
                Settings->ClockTopic = SavedClockTopic;
            }
        }

    private:
        UUBotROSSettings* Settings = nullptr;
        FString SavedHost;
        int32 SavedPort = 0;
        bool bSavedAutoConnect = true;
        bool bSavedPublishClock = true;
        FString SavedClockTopic;
    };

    /** Transient game world ticked by hand (the editor engine does not tick Game world contexts it did not create). */
    class FScopedE2EWorld
    {
    public:
        UE_NONCOPYABLE(FScopedE2EWorld);

        FScopedE2EWorld()
            : World(UWorld::CreateWorld(EWorldType::Game, false))
            , CachedFrameCounter(GFrameCounter)
        {
            if (World && GEngine)
            {
                World->AddToRoot();
                GEngine->CreateNewWorldContext(EWorldType::Game).SetCurrentWorld(World);
                World->InitializeActorsForPlay(FURL());
            }
        }

        ~FScopedE2EWorld()
        {
            if (!World)
            {
                return;
            }
            if (World->AreActorsInitialized())
            {
                for (TActorIterator<AActor> It(World); It; ++It)
                {
                    It->RouteEndPlay(EEndPlayReason::LevelTransition);
                }
            }
            if (GEngine)
            {
                GEngine->DestroyWorldContext(World);
            }
            World->DestroyWorld(false);
            World->RemoveFromRoot();
            GFrameCounter = CachedFrameCounter;
        }

        UWorld* Get() const
        {
            return World;
        }

        /** World tick: runs the bridge subsystem tick (clock, static TF, MESSAGE dispatch on the game thread). */
        void Tick(float DeltaSeconds) const
        {
            World->Tick(LEVELTICK_All, DeltaSeconds);
            ++GFrameCounter;
        }

    private:
        UWorld* World = nullptr;
        uint64 CachedFrameCounter = 0;
    };

    FString FormatVector(const FUBotRosVector3& Value)
    {
        return FString::Printf(TEXT("(%.9g, %.9g, %.9g)"), Value.x, Value.y, Value.z);
    }

    bool VectorEquals(const FUBotRosVector3& Actual, const FVector& Expected)
    {
        return Actual.x == Expected.X && Actual.y == Expected.Y && Actual.z == Expected.Z;
    }

    /**
     * The fixture IMU, produced the way a sensor sample is published: uBot (UE-axes) data through
     * UBot::ROSConvert::MakeImu. Fails when the converter output is not exactly the section 12 values.
     */
    bool MakeFixtureImu(FUBotRosImu& OutImu, FString& OutError)
    {
        FUBotImuSensorData Data;
        Data.Header.SensorId = ImuFrameId;
        Data.Header.FrameId = ImuFrameId;
        Data.Header.TimestampSeconds = FixtureStampSeconds;
        Data.Header.bValid = true;
        Data.Header.Status = EUBotSampleStatus::Valid;
        Data.OrientationQuaternion = FQuat::Identity;
        Data.AngularVelocityRadPerSecond = UBot::RightHanded::AngularVelocityToUE(ImuAngularVelocityROS);
        Data.LinearAccelerationMetersPerSecondSquared = UBot::RightHanded::VectorToUE(ImuLinearAccelerationROS);

        OutImu = UBot::ROSConvert::MakeImu(Data, ImuFrameId);

        TArray<FString> Problems;
        if (!OutImu.header.frame_id.Equals(ImuFrameId, ESearchCase::CaseSensitive))
        {
            Problems.Add(FString::Printf(TEXT("frame_id '%s'"), *OutImu.header.frame_id));
        }
        if (OutImu.header.stamp.sec != 12 || OutImu.header.stamp.nanosec != 500000000u)
        {
            Problems.Add(FString::Printf(TEXT("stamp %d.%09u"), OutImu.header.stamp.sec, OutImu.header.stamp.nanosec));
        }
        const FUBotRosQuaternion& Q = OutImu.orientation;
        if (Q.x != 0.0 || Q.y != 0.0 || Q.z != 0.0 || Q.w != 1.0)
        {
            Problems.Add(FString::Printf(TEXT("orientation (%.9g, %.9g, %.9g, %.9g)"), Q.x, Q.y, Q.z, Q.w));
        }
        if (!VectorEquals(OutImu.angular_velocity, ImuAngularVelocityROS))
        {
            Problems.Add(TEXT("angular_velocity ") + FormatVector(OutImu.angular_velocity));
        }
        if (!VectorEquals(OutImu.linear_acceleration, ImuLinearAccelerationROS))
        {
            Problems.Add(TEXT("linear_acceleration ") + FormatVector(OutImu.linear_acceleration));
        }
        if (!Problems.IsEmpty())
        {
            OutError = TEXT("UBot::ROSConvert::MakeImu does not reproduce the section 12 IMU fixture: ") + FString::Join(Problems, TEXT(", "));
            return false;
        }
        return true;
    }

    void AppendLittleEndian32(TArray<uint8>& Data, uint32 Bits)
    {
        Data.Add(static_cast<uint8>(Bits & 0xFFu));
        Data.Add(static_cast<uint8>((Bits >> 8) & 0xFFu));
        Data.Add(static_cast<uint8>((Bits >> 16) & 0xFFu));
        Data.Add(static_cast<uint8>((Bits >> 24) & 0xFFu));
    }

    void AppendFloat(TArray<uint8>& Data, float Value)
    {
        uint32 Bits = 0;
        FMemory::Memcpy(&Bits, &Value, sizeof(Bits));
        AppendLittleEndian32(Data, Bits);
    }

    void AppendUInt16(TArray<uint8>& Data, uint16 Value)
    {
        Data.Add(static_cast<uint8>(Value & 0xFFu));
        Data.Add(static_cast<uint8>((Value >> 8) & 0xFFu));
    }

    /** The fixture cloud in the uBot PointCloud2 layout (x, y, z, intensity float32, ring uint16, time float32; 22 bytes). */
    FUBotRosPointCloud2 MakeFixturePointCloud()
    {
        FUBotRosPointCloud2 Cloud;
        Cloud.header.stamp = FUBotRosTime::FromSeconds(FixtureStampSeconds);
        Cloud.header.frame_id = LidarFrameId;
        Cloud.height = 1;
        Cloud.width = static_cast<uint32>(UE_ARRAY_COUNT(CloudPoints));
        Cloud.fields.Add(FUBotRosPointField::Make(TEXT("x"), UBot::ROSConvert::PointCloudOffsetX, FUBotRosPointField::FLOAT32));
        Cloud.fields.Add(FUBotRosPointField::Make(TEXT("y"), UBot::ROSConvert::PointCloudOffsetY, FUBotRosPointField::FLOAT32));
        Cloud.fields.Add(FUBotRosPointField::Make(TEXT("z"), UBot::ROSConvert::PointCloudOffsetZ, FUBotRosPointField::FLOAT32));
        Cloud.fields.Add(FUBotRosPointField::Make(TEXT("intensity"), UBot::ROSConvert::PointCloudOffsetIntensity, FUBotRosPointField::FLOAT32));
        Cloud.fields.Add(FUBotRosPointField::Make(TEXT("ring"), UBot::ROSConvert::PointCloudOffsetRing, FUBotRosPointField::UINT16));
        Cloud.fields.Add(FUBotRosPointField::Make(TEXT("time"), UBot::ROSConvert::PointCloudOffsetTime, FUBotRosPointField::FLOAT32));
        Cloud.is_bigendian = false;
        Cloud.point_step = UBot::ROSConvert::PointCloudPointStep;
        Cloud.row_step = Cloud.point_step * Cloud.width;
        Cloud.data.Reserve(static_cast<int32>(Cloud.row_step));
        for (const FCloudPoint& Point : CloudPoints)
        {
            AppendFloat(Cloud.data, Point.X);
            AppendFloat(Cloud.data, Point.Y);
            AppendFloat(Cloud.data, Point.Z);
            AppendFloat(Cloud.data, Point.Intensity);
            AppendUInt16(Cloud.data, Point.Ring);
            AppendFloat(Cloud.data, Point.Time);
        }
        Cloud.is_dense = true;
        return Cloud;
    }

    /** base_link -> e2e_imu, translation (1, 2, 3) m, identity rotation (ROS values). */
    FUBotRosTransformStamped MakeFixtureStaticTransform()
    {
        FUBotRosTransformStamped Transform;
        Transform.header.stamp = FUBotRosTime::FromSeconds(FixtureStampSeconds);
        Transform.header.frame_id = BaseFrameId;
        Transform.child_frame_id = ImuFrameId;
        Transform.transform.translation.x = 1.0;
        Transform.transform.translation.y = 2.0;
        Transform.transform.translation.z = 3.0;
        Transform.transform.rotation.x = 0.0;
        Transform.transform.rotation.y = 0.0;
        Transform.transform.rotation.z = 0.0;
        Transform.transform.rotation.w = 1.0;
        return Transform;
    }
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotROSEndToEndBridgeTest, "UBotROS.EndToEnd.Bridge", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotROSEndToEndBridgeTest::RunTest(const FString& Parameters)
{
    using namespace UBotROSEndToEndTests;

    if (FPlatformMisc::GetEnvironmentVariable(EnableVariable).TrimStartAndEnd() != TEXT("1"))
    {
        const FString Message = FString::Printf(TEXT("UBotROS.EndToEnd.Bridge skipped: set %s=1 and start ubot_ros_bridge (port %d) and test/e2e_check.py in WSL to run it."),
            EnableVariable, BridgePort);
        UE_LOG(LogUBotROS, Display, TEXT("%s"), *Message);
        AddInfo(Message);
        return true;
    }

    FScopedE2ESettings Settings;
    FScopedE2EWorld Scope;
    UWorld* World = Scope.Get();
    if (!TestNotNull(TEXT("Transient game world"), World))
    {
        return false;
    }
    UUBotROSBridgeSubsystem* Bridge = UUBotROSBridgeSubsystem::Get(World);
    if (!TestNotNull(TEXT("uBot ROS bridge subsystem of the game world"), Bridge))
    {
        return false;
    }

    FUBotRosImu ImuMessage;
    FString FixtureError;
    if (!MakeFixtureImu(ImuMessage, FixtureError))
    {
        AddError(FixtureError);
        return false;
    }
    const TArray<uint8> ImuCdr = UBot::Ros::Encode(ImuMessage);
    const TArray<uint8> CloudCdr = UBot::Ros::Encode(MakeFixturePointCloud());

    // Subscribe before publishing so the checker's cmd_vel publisher is matched as early as possible.
    TSharedRef<FCommandState> CommandState = MakeShared<FCommandState>();
    const int32 SubscriptionId = Bridge->Subscribe(CmdVelTopic, FUBotRosTwist::TypeName(),
        FUBotROSQos(EUBotROSReliability::BestEffort, EUBotROSDurability::Volatile, 10),
        [CommandState](TConstArrayView<uint8> Cdr)
        {
            ++CommandState->MessagesReceived;
            FUBotRosTwist Twist;
            if (!UBot::Ros::Decode(Cdr, Twist))
            {
                ++CommandState->UndecodableMessages;
                return;
            }
            CommandState->Last = Twist;
            CommandState->bHasLast = true;
            if (VectorEquals(Twist.linear, CmdVelLinear) && VectorEquals(Twist.angular, CmdVelAngular))
            {
                CommandState->bMatched = true;
            }
        });
    const int32 ImuPublisherId = Bridge->Advertise(ImuTopic, FUBotRosImu::TypeName(), FUBotROSQos::SensorData());
    const int32 PointsPublisherId = Bridge->Advertise(PointsTopic, FUBotRosPointCloud2::TypeName(), FUBotROSQos::SensorData());

    ON_SCOPE_EXIT
    {
        Bridge->Unsubscribe(SubscriptionId);
        Bridge->Unadvertise(ImuPublisherId);
        Bridge->Unadvertise(PointsPublisherId);
        Bridge->ClearStaticTransforms(World);
        Bridge->Disconnect();
    };

    if (!TestTrue(TEXT("cmd_vel subscription created"), SubscriptionId > 0)
        || !TestTrue(TEXT("IMU publisher advertised"), ImuPublisherId > 0)
        || !TestTrue(TEXT("Point cloud publisher advertised"), PointsPublisherId > 0))
    {
        return false;
    }
    Bridge->SetStaticTransforms(World, { MakeFixtureStaticTransform() });
    Bridge->ConnectTo(BridgeHost, BridgePort);

    const double StartSeconds = FPlatformTime::Seconds();
    double LastTickSeconds = StartSeconds;
    double NextPublishSeconds = StartSeconds;
    double ConnectedAfterSeconds = -1.0;
    double MatchedAfterSeconds = -1.0;
    int32 PublishedImu = 0;
    int32 PublishedClouds = 0;
    int32 RefusedPublishes = 0;
    for (;;)
    {
        const double NowSeconds = FPlatformTime::Seconds();
        const double ElapsedSeconds = NowSeconds - StartSeconds;
        if (CommandState->bMatched && MatchedAfterSeconds < 0.0)
        {
            MatchedAfterSeconds = ElapsedSeconds;
        }
        if ((CommandState->bMatched && ElapsedSeconds >= MinimumPublishSeconds) || ElapsedSeconds >= TimeoutSeconds)
        {
            break;
        }
        if (ConnectedAfterSeconds < 0.0 && Bridge->IsBridgeConnected())
        {
            ConnectedAfterSeconds = ElapsedSeconds;
        }

        if (NowSeconds >= NextPublishSeconds)
        {
            NextPublishSeconds = FMath::Max(NextPublishSeconds + PublishPeriodSeconds, NowSeconds);
            TArray<uint8> ImuFrame = ImuCdr;
            TArray<uint8> CloudFrame = CloudCdr;
            if (Bridge->Publish(ImuPublisherId, MoveTemp(ImuFrame)))
            {
                ++PublishedImu;
            }
            else
            {
                ++RefusedPublishes;
            }
            if (Bridge->Publish(PointsPublisherId, MoveTemp(CloudFrame)))
            {
                ++PublishedClouds;
            }
            else
            {
                ++RefusedPublishes;
            }
        }

        // The world tick runs the subsystem tick: /clock, /tf_static and the cmd_vel dispatch on the game thread.
        Scope.Tick(static_cast<float>(FMath::Clamp(NowSeconds - LastTickSeconds, 0.001, 0.1)));
        LastTickSeconds = NowSeconds;
        FPlatformProcess::Sleep(LoopSleepSeconds);
    }

    const FUBotROSBridgeStats Stats = Bridge->GetStats();
    AddInfo(FString::Printf(TEXT("Bridge %s: connected after %.2f s, cmd_vel matched after %.2f s; published %d IMU / %d clouds (%d refused); frames sent %lld, received %lld, dropped %lld, sessions %d."),
        *Bridge->GetBridgeEndpoint(), ConnectedAfterSeconds, MatchedAfterSeconds, PublishedImu, PublishedClouds, RefusedPublishes,
        Stats.FramesSent, Stats.FramesReceived, Stats.DroppedMessages, Stats.SessionCount));

    if (!CommandState->bMatched)
    {
        const FString LastCommand = CommandState->bHasLast
            ? FString::Printf(TEXT("; last Twist linear %s angular %s"), *FormatVector(CommandState->Last.linear), *FormatVector(CommandState->Last.angular))
            : FString();
        AddError(FString::Printf(
            TEXT("No geometry_msgs/msg/Twist with linear (1.5, 0, 0) and angular (0, 0, 0.25) arrived on %s within %.0f s. ")
            TEXT("Bridge %s:%d status %s, %s, last error '%s'; cmd_vel messages received %d (undecodable %d)%s. ")
            TEXT("Start ubot_ros_bridge (python3 -m ubot_ros_bridge.bridge_node --port %d) and test/e2e_check.py in WSL before running this test."),
            CmdVelTopic, TimeoutSeconds, BridgeHost, BridgePort, *UEnum::GetValueAsString(Bridge->GetBridgeStatus()),
            ConnectedAfterSeconds < 0.0 ? TEXT("never connected") : TEXT("was connected"), *Bridge->GetLastBridgeError(),
            CommandState->MessagesReceived, CommandState->UndecodableMessages, *LastCommand, BridgePort));
        return false;
    }

    TestTrue(TEXT("The bridge connected"), ConnectedAfterSeconds >= 0.0);
    TestEqual(TEXT("No publish was refused"), RefusedPublishes, 0);
    TestEqual(TEXT("Every cmd_vel message decoded"), CommandState->UndecodableMessages, 0);
    return true;
}

// Flight commands through a real ubot_ros_bridge: the attitude and motor command components subscribe, and ROS
// publishes from the command line. The bridge needs mavros_msgs and actuator_msgs. Run it with:
//   WSL:  source /opt/ros/humble/setup.bash
//         cd ros2/ubot_ros_bridge && python3 -m ubot_ros_bridge.bridge_node --port 8268 &
//         ros2 topic pub -r 10 /ubot_e2e/cmd_attitude mavros_msgs/msg/AttitudeTarget "{orientation: {z: 0.7071067811865476, w: 0.7071067811865476}, thrust: 0.6}" &
//         ros2 topic pub -r 10 /ubot_e2e/cmd_motors actuator_msgs/msg/Actuators "{velocity: [800.0, 805.0, 810.0, 815.0]}"
//   UE:   set UBOT_ROS_E2E_FLIGHT=1, then Automation RunTests UBotROS.EndToEnd.FlightCommands
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotROSEndToEndFlightCommandsTest, "UBotROS.EndToEnd.FlightCommands", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotROSEndToEndFlightCommandsTest::RunTest(const FString& Parameters)
{
    using namespace UBotROSEndToEndTests;
    const TCHAR* const FlightVariable = TEXT("UBOT_ROS_E2E_FLIGHT");
    if (FPlatformMisc::GetEnvironmentVariable(FlightVariable).TrimStartAndEnd() != TEXT("1"))
    {
        const FString Message = FString::Printf(TEXT("UBotROS.EndToEnd.FlightCommands skipped: set %s=1, start ubot_ros_bridge (port %d) and publish the commands in WSL to run it."),
            FlightVariable, BridgePort);
        UE_LOG(LogUBotROS, Display, TEXT("%s"), *Message);
        AddInfo(Message);
        return true;
    }

    FScopedE2ESettings Settings;
    FScopedE2EWorld Scope;
    UWorld* World = Scope.Get();
    UUBotROSBridgeSubsystem* Bridge = World ? UUBotROSBridgeSubsystem::Get(World) : nullptr;
    if (!TestNotNull(TEXT("uBot ROS bridge subsystem of the game world"), Bridge))
    {
        return false;
    }

    AActor* Vehicle = World->SpawnActor<AActor>();
    UUBotROSFlightCommandTestTarget* Target = NewObject<UUBotROSFlightCommandTestTarget>(Vehicle, TEXT("Vehicle"));
    Target->RegisterComponent();
    UUBotROSAttitudeCommandComponent* Attitude = NewObject<UUBotROSAttitudeCommandComponent>(Vehicle, TEXT("Attitude"));
    Attitude->CommandTopicOverride = TEXT("/ubot_e2e/cmd_attitude");
    Attitude->RegisterComponent();
    UUBotROSMotorCommandComponent* Motors = NewObject<UUBotROSMotorCommandComponent>(Vehicle, TEXT("Motors"));
    Motors->CommandTopicOverride = TEXT("/ubot_e2e/cmd_motors");
    Motors->RegisterComponent();
    Vehicle->DispatchBeginPlay();
    TestTrue(TEXT("Attitude receiver subscribed"), Attitude->GetResolvedCommandTopic() == TEXT("/ubot_e2e/cmd_attitude"));

    Bridge->ConnectTo(BridgeHost, BridgePort);
    const double StartSeconds = FPlatformTime::Seconds();
    double LastTickSeconds = StartSeconds;
    while (FPlatformTime::Seconds() - StartSeconds < TimeoutSeconds
        && (Target->Attitudes.IsEmpty() || Target->MotorCommands.IsEmpty()))
    {
        const double NowSeconds = FPlatformTime::Seconds();
        Scope.Tick(static_cast<float>(FMath::Clamp(NowSeconds - LastTickSeconds, 0.001, 0.1)));
        LastTickSeconds = NowSeconds;
        FPlatformProcess::Sleep(LoopSleepSeconds);
    }

    if (!TestTrue(FString::Printf(TEXT("An attitude command arrived (bridge error '%s', receiver error '%s')"), *Bridge->GetLastBridgeError(), *Attitude->GetCommandError()),
            !Target->Attitudes.IsEmpty())
        || !TestTrue(FString::Printf(TEXT("A motor command arrived (receiver error '%s')"), *Motors->GetCommandError()), !Target->MotorCommands.IsEmpty()))
    {
        return false;
    }
    // ROS yaw +90 degrees (nose to the left) is the nose along UE -Y.
    TestTrue(TEXT("Attitude converted"), Target->Attitudes.Last().Orientation.GetAxisX().Equals(FVector(0.0, -1.0, 0.0), 1e-6));
    TestEqual(TEXT("Thrust"), Target->Attitudes.Last().NormalizedThrust, 0.6, 1e-6);
    TestTrue(TEXT("Motor speeds"), Target->MotorCommands.Last().Values == TArray<double>({ 800.0, 805.0, 810.0, 815.0 }));
    TestEqual(TEXT("Motor command type"), Target->MotorCommands.Last().Type, EUBotMotorCommandType::AngularVelocity);
    return true;
}

#endif
