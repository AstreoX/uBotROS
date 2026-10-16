#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "Components/SceneComponent.h"
#include "Engine/Engine.h"
#include "Engine/EngineBaseTypes.h"
#include "Engine/World.h"
#include "EngineUtils.h"
#include "GameFramework/Actor.h"
#include "Misc/ScopeExit.h"
#include "Serialization/MemoryReader.h"
#include "Serialization/MemoryWriter.h"
#include "UBot2DLidarSensorComponent.h"
#include "UBot3DLidarSensorComponent.h"
#include "UBotDepthCameraSensorComponent.h"
#include "UBotExtensionRegistry.h"
#include "UBotIdentityComponent.h"
#include "UBotImuSensorComponent.h"
#include "UBotOdometrySensorComponent.h"
#include "UBotPoseSensorComponent.h"
#include "UBotRgbCameraSensorComponent.h"
#include "UBotROSBridgeSubsystem.h"
#include "UBotROSBuiltinPublishers.h"
#include "UBotROSConventionHelpers.h"
#include "UBotROSLegacyTestTypes.h"
#include "UBotROSMessageTypes.h"
#include "UBotROSSensorBridgeComponent.h"
#include "UBotROSSensorPublisher.h"
#include "UBotROSSettings.h"
#include "UBotROSTypes.h"
#include "UBotSensorComponent.h"
#include "UObject/CoreRedirects.h"
#include "UObject/Package.h"
#include "UObject/UnrealType.h"

namespace UBotROSFactoryTests
{
    const TCHAR* const TestOwnerPackage = TEXT("UBotROSTests");

    struct FSensorCase
    {
        const TCHAR* Label;
        UClass* SensorClass;
        UClass* PublisherClass;
        const TCHAR* FactoryName;
        bool bOptical;
    };

    TArray<FSensorCase> GetSensorCases()
    {
        return {
            { TEXT("Pose"), UUBotPoseSensorComponent::StaticClass(), UUBotROSPosePublisher::StaticClass(), TEXT("UBotROS.Pose"), false },
            { TEXT("Odometry"), UUBotOdometrySensorComponent::StaticClass(), UUBotROSOdometryPublisher::StaticClass(), TEXT("UBotROS.Odometry"), false },
            { TEXT("IMU"), UUBotImuSensorComponent::StaticClass(), UUBotROSImuPublisher::StaticClass(), TEXT("UBotROS.IMU"), false },
            { TEXT("Lidar2D"), UUBot2DLidarSensorComponent::StaticClass(), UUBotROSLidar2DPublisher::StaticClass(), TEXT("UBotROS.Lidar2D"), false },
            { TEXT("Lidar3D"), UUBot3DLidarSensorComponent::StaticClass(), UUBotROSLidar3DPublisher::StaticClass(), TEXT("UBotROS.Lidar3D"), false },
            { TEXT("RGBCamera"), UUBotRgbCameraSensorComponent::StaticClass(), UUBotROSRgbCameraPublisher::StaticClass(), TEXT("UBotROS.RGBCamera"), true },
            { TEXT("DepthCamera"), UUBotDepthCameraSensorComponent::StaticClass(), UUBotROSDepthCameraPublisher::StaticClass(), TEXT("UBotROS.DepthCamera"), true },
        };
    }

    /** Factories registered at the sensor publisher point by the UBotROS package itself. */
    TArray<TSharedRef<IUBotROSSensorPublisherFactory>> GetBuiltinFactories()
    {
        TArray<TSharedRef<IUBotROSSensorPublisherFactory>> Result;
        const FName PointName = UBot::ROSPublishing::GetExtensionPointName();
        const FName Owner(UBot::ROSPublishing::OwnerPackageName);
        for (const FUBotExtensionRecord& Record : FUBotExtensionRegistry::Get().GetAllExtensions())
        {
            if (Record.ExtensionPoint == PointName && Record.OwnerPackage == Owner && Record.Extension.IsValid())
            {
                Result.Add(StaticCastSharedRef<IUBotROSSensorPublisherFactory>(Record.Extension.ToSharedRef()));
            }
        }
        return Result;
    }

    /** Test factory for IMU sensors with a configurable priority; counts the publishers it creates. */
    class FTestImuPublisherFactory final : public IUBotROSSensorPublisherFactory
    {
    public:
        FTestImuPublisherFactory(FName InName, int32 InPriority)
            : Name(InName)
            , Priority(InPriority)
        {
        }

        virtual FName GetExtensionName() const override { return Name; }
        virtual int32 GetPriority() const override { return Priority; }

        virtual bool SupportsSensor(const UObject* Sensor) const override
        {
            return Sensor && Sensor->IsA<UUBotImuSensorComponent>();
        }

        virtual UUBotROSSensorPublisher* CreatePublisher(UObject* Outer, UObject* Sensor) const override
        {
            ++CreatedPublishers;
            return NewObject<UUBotROSImuPublisher>(Outer ? Outer : GetTransientPackage(), NAME_None, RF_Transient);
        }

        mutable int32 CreatedPublishers = 0;

    private:
        FName Name;
        int32 Priority = 0;
    };

    /** Disables auto-connect for the scope so component tests never open a socket; restores the settings afterwards. */
    class FScopedNoAutoConnect
    {
    public:
        UE_NONCOPYABLE(FScopedNoAutoConnect);

        FScopedNoAutoConnect()
            : Settings(GetMutableDefault<UUBotROSSettings>())
        {
            if (Settings)
            {
                bSavedAutoConnect = Settings->bAutoConnect;
                Settings->bAutoConnect = false;
            }
        }

        ~FScopedNoAutoConnect()
        {
            if (Settings)
            {
                Settings->bAutoConnect = bSavedAutoConnect;
            }
        }

    private:
        UUBotROSSettings* Settings = nullptr;
        bool bSavedAutoConnect = true;
    };

    /** Transient game world (has the bridge subsystem) ticked by hand; torn down when the scope ends. */
    class FScopedBridgeTestWorld
    {
    public:
        UE_NONCOPYABLE(FScopedBridgeTestWorld);

        FScopedBridgeTestWorld()
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

        ~FScopedBridgeTestWorld()
        {
            if (!World)
            {
                return;
            }
            for (TActorIterator<AActor> It(World); It; ++It)
            {
                for (UActorComponent* Component : It->GetComponents())
                {
                    if (UUBotROSSensorBridgeComponent* BridgeComponent = Cast<UUBotROSSensorBridgeComponent>(Component))
                    {
                        BridgeComponent->StopROSBridge();
                    }
                }
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

        void Tick(float DeltaSeconds) const
        {
            World->Tick(LEVELTICK_All, DeltaSeconds);
            ++GFrameCounter;
        }

    private:
        UWorld* World = nullptr;
        uint64 CachedFrameCounter = 0;
    };

    /** Robot actor: a scene root plus sensors and a bridge component, registered but not begun play (sensors stay idle). */
    struct FTestRobot
    {
        AActor* Actor = nullptr;
        USceneComponent* Root = nullptr;
        UUBotROSSensorBridgeComponent* BridgeComponent = nullptr;

        static FTestRobot Spawn(UWorld* World, const TCHAR* Namespace)
        {
            FTestRobot Robot;
            Robot.Actor = World ? World->SpawnActor<AActor>() : nullptr;
            if (!Robot.Actor)
            {
                return Robot;
            }
            Robot.Root = NewObject<USceneComponent>(Robot.Actor, TEXT("Root"));
            Robot.Actor->SetRootComponent(Robot.Root);
            Robot.Root->RegisterComponent();

            Robot.BridgeComponent = NewObject<UUBotROSSensorBridgeComponent>(Robot.Actor, TEXT("RosSensorBridge"));
            Robot.BridgeComponent->bAutoInitialize = false;
            Robot.BridgeComponent->RobotNamespaceOverride = Namespace;
            Robot.BridgeComponent->RegisterComponent();
            return Robot;
        }

        template <typename TSensor>
        TSensor* AddSensor(const TCHAR* ComponentName, const TCHAR* SensorId, const FVector& RelativeLocationCm = FVector::ZeroVector)
        {
            TSensor* Sensor = NewObject<TSensor>(Actor, ComponentName);
            Sensor->SensorId = SensorId;
            // Every UBotSensor component defaults FrameId to its type's default id (for example "lidar_2d"), which
            // would collide between sensors of one type. An empty FrameId falls back to the sensor id.
            Sensor->FrameId.Reset();
            Sensor->bAutoStart = false;
            Sensor->SetupAttachment(Root);
            Sensor->SetRelativeLocation(RelativeLocationCm);
            Sensor->RegisterComponent();
            Sensor->UpdateComponentToWorld();
            return Sensor;
        }
    };

    FString JoinTopics(const TArray<FString>& Topics)
    {
        return FString::Join(Topics, TEXT(", "));
    }

    const FUBotRosTransformStamped* FindStaticTransform(const TArray<FUBotRosTransformStamped>& Transforms, const FString& ChildFrame)
    {
        return Transforms.FindByPredicate([&ChildFrame](const FUBotRosTransformStamped& Transform)
        {
            return Transform.child_frame_id.Equals(ChildFrame, ESearchCase::CaseSensitive);
        });
    }

    FString Redirect(ECoreRedirectFlags Type, const FString& OldName)
    {
        return FCoreRedirects::GetRedirectedName(Type, FCoreRedirectObjectName(OldName)).ToString();
    }
}

// -------------------------------------------------------------------------------------------------

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotROSFactoriesExtensionPointTest, "UBotROS.Factories.ExtensionPoint", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotROSFactoriesExtensionPointTest::RunTest(const FString& Parameters)
{
    using namespace UBotROSFactoryTests;

    FUBotExtensionRegistry& Registry = FUBotExtensionRegistry::Get();
    const FName PointName = UBot::ROSPublishing::GetExtensionPointName();
    TestEqual(TEXT("Extension point name"), *PointName.ToString(), TEXT("UBotROS.SensorPublisher"));
    if (!TestTrue(TEXT("UBotROS declares the sensor publisher extension point"), Registry.IsExtensionPointDeclared(PointName)))
    {
        return false;
    }

    bool bFoundPoint = false;
    for (const FUBotExtensionPointInfo& Point : Registry.GetExtensionPoints())
    {
        if (Point.Name == PointName)
        {
            bFoundPoint = true;
            TestEqual(TEXT("The point is owned by UBotROS"), *Point.OwnerPackage.ToString(), UBot::ROSPublishing::OwnerPackageName);
            TestFalse(TEXT("The point has a description"), Point.Description.IsEmpty());
        }
    }
    TestTrue(TEXT("The point is listed"), bFoundPoint);

    const TArray<TSharedRef<IUBotROSSensorPublisherFactory>> Builtins = GetBuiltinFactories();
    TestEqual(TEXT("Seven built-in factories are registered by UBotROS"), Builtins.Num(), 7);

    TArray<FName> RegisteredNames;
    for (const TSharedRef<IUBotROSSensorPublisherFactory>& Factory : Builtins)
    {
        RegisteredNames.Add(Factory->GetExtensionName());
        TestEqual(*FString::Printf(TEXT("Built-in factory %s has priority 0"), *Factory->GetExtensionName().ToString()), Factory->GetPriority(), 0);
    }
    for (const FName& ExpectedName : UBot::ROSBuiltin::GetBuiltinFactoryNames())
    {
        TestTrue(*FString::Printf(TEXT("Built-in factory %s is registered"), *ExpectedName.ToString()), RegisteredNames.Contains(ExpectedName));
    }
    TestTrue(TEXT("Every built-in factory is reachable through the point"),
        Registry.GetExtensionsAs<IUBotROSSensorPublisherFactory>(PointName).Num() >= Builtins.Num());
    return true;
}

// -------------------------------------------------------------------------------------------------

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotROSFactoriesBuiltinCoverageTest, "UBotROS.Factories.BuiltinCoverage", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotROSFactoriesBuiltinCoverageTest::RunTest(const FString& Parameters)
{
    using namespace UBotROSFactoryTests;

    const TArray<TSharedRef<IUBotROSSensorPublisherFactory>> Builtins = GetBuiltinFactories();
    if (!TestEqual(TEXT("Seven built-in factories"), Builtins.Num(), 7))
    {
        return false;
    }

    for (const FSensorCase& Case : GetSensorCases())
    {
        UUBotSensorComponent* Sensor = NewObject<UUBotSensorComponent>(GetTransientPackage(), Case.SensorClass);
        if (!TestNotNull(*FString::Printf(TEXT("%s sensor created"), Case.Label), Sensor))
        {
            continue;
        }

        TArray<TSharedRef<IUBotROSSensorPublisherFactory>> Matching;
        for (const TSharedRef<IUBotROSSensorPublisherFactory>& Factory : Builtins)
        {
            if (Factory->SupportsSensor(Sensor))
            {
                Matching.Add(Factory);
            }
        }
        if (!TestEqual(*FString::Printf(TEXT("%s is matched by exactly one built-in factory"), Case.Label), Matching.Num(), 1))
        {
            continue;
        }
        const TSharedRef<IUBotROSSensorPublisherFactory>& Factory = Matching[0];
        TestEqual(*FString::Printf(TEXT("%s factory name"), Case.Label), *Factory->GetExtensionName().ToString(), Case.FactoryName);

        // Only another package's factory may take precedence over the built-in one.
        const TArray<TSharedRef<IUBotROSSensorPublisherFactory>> Supporting = UBot::ROSPublishing::FindFactories(Sensor);
        const bool bOnlyBuiltinsSupport = Supporting.Num() == 1;
        if (bOnlyBuiltinsSupport)
        {
            const TSharedPtr<IUBotROSSensorPublisherFactory> Selected = UBot::ROSPublishing::FindFactory(Sensor);
            TestTrue(*FString::Printf(TEXT("%s selects the built-in factory"), Case.Label), Selected.Get() == &Factory.Get());
        }
        else
        {
            AddInfo(FString::Printf(TEXT("%s: %d factories support it (other packages registered some); selection not checked."), Case.Label, Supporting.Num()));
        }

        UUBotROSSensorPublisher* Publisher = Factory->CreatePublisher(GetTransientPackage(), Sensor);
        if (TestNotNull(*FString::Printf(TEXT("%s publisher created"), Case.Label), Publisher))
        {
            TestTrue(*FString::Printf(TEXT("%s publisher class is %s"), Case.Label, *Case.PublisherClass->GetName()), Publisher->GetClass() == Case.PublisherClass);
            TestTrue(*FString::Printf(TEXT("%s publisher uses the optical frame only for cameras"), Case.Label), Publisher->UsesOpticalFrame() == Case.bOptical);
            TestFalse(*FString::Printf(TEXT("%s publisher is not initialized before Initialize"), Case.Label), Publisher->IsPublisherInitialized());
            TestEqual(*FString::Printf(TEXT("%s publisher has no topics before Initialize"), Case.Label), Publisher->GetTopics().Num(), 0);
            Publisher->Shutdown();
        }

        // A factory refuses sensors of other types.
        for (const TSharedRef<IUBotROSSensorPublisherFactory>& Other : Builtins)
        {
            if (&Other.Get() != &Factory.Get())
            {
                TestNull(*FString::Printf(TEXT("%s refuses a %s sensor"), *Other->GetExtensionName().ToString(), Case.Label), Other->CreatePublisher(GetTransientPackage(), Sensor));
            }
        }
    }

    // Objects that are not uBot sensors are not matched.
    const UObject* NotASensor = GetDefault<AActor>();
    for (const TSharedRef<IUBotROSSensorPublisherFactory>& Factory : Builtins)
    {
        TestFalse(*FString::Printf(TEXT("%s does not support an actor"), *Factory->GetExtensionName().ToString()), Factory->SupportsSensor(NotASensor));
        TestFalse(*FString::Printf(TEXT("%s does not support null"), *Factory->GetExtensionName().ToString()), Factory->SupportsSensor(nullptr));
    }
    TestFalse(TEXT("FindFactory(null) is null"), UBot::ROSPublishing::FindFactory(nullptr).IsValid());
    return true;
}

// -------------------------------------------------------------------------------------------------

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotROSFactoriesPriorityTest, "UBotROS.Factories.PriorityOverride", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotROSFactoriesPriorityTest::RunTest(const FString& Parameters)
{
    using namespace UBotROSFactoryTests;

    FUBotExtensionRegistry& Registry = FUBotExtensionRegistry::Get();
    const FName PointName = UBot::ROSPublishing::GetExtensionPointName();
    const FName TestOwner(TestOwnerPackage);
    ON_SCOPE_EXIT
    {
        FUBotExtensionRegistry::Get().UnregisterAllFromPackage(FName(UBotROSFactoryTests::TestOwnerPackage));
    };

    UUBotImuSensorComponent* Imu = NewObject<UUBotImuSensorComponent>(GetTransientPackage());
    UUBot2DLidarSensorComponent* Lidar = NewObject<UUBot2DLidarSensorComponent>(GetTransientPackage());
    const TSharedPtr<IUBotROSSensorPublisherFactory> BuiltinImu = UBot::ROSPublishing::FindFactory(Imu);
    const TSharedPtr<IUBotROSSensorPublisherFactory> BuiltinLidar = UBot::ROSPublishing::FindFactory(Lidar);
    if (!TestTrue(TEXT("IMU has a factory"), BuiltinImu.IsValid()) || !TestTrue(TEXT("2D LiDAR has a factory"), BuiltinLidar.IsValid()))
    {
        return false;
    }

    // Equal priority: registration order wins, so the built-in factory registered at startup stays selected.
    const TSharedRef<FTestImuPublisherFactory> SamePriority = MakeShared<FTestImuPublisherFactory>(FName(TEXT("UBotROSTests.SamePriorityImu")), BuiltinImu->GetPriority());
    TestTrue(TEXT("Same-priority factory registered"), Registry.RegisterExtension(PointName, TestOwner, SamePriority));
    TestTrue(TEXT("An equal priority does not displace the earlier factory"), UBot::ROSPublishing::FindFactory(Imu) == BuiltinImu);

    // Lower priority never wins.
    const TSharedRef<FTestImuPublisherFactory> LowPriority = MakeShared<FTestImuPublisherFactory>(FName(TEXT("UBotROSTests.LowPriorityImu")), BuiltinImu->GetPriority() - 5);
    TestTrue(TEXT("Low-priority factory registered"), Registry.RegisterExtension(PointName, TestOwner, LowPriority));
    TestTrue(TEXT("A lower priority does not win"), UBot::ROSPublishing::FindFactory(Imu) == BuiltinImu);

    // Higher priority wins, for the sensors it supports only.
    const TSharedRef<FTestImuPublisherFactory> HighPriority = MakeShared<FTestImuPublisherFactory>(FName(TEXT("UBotROSTests.HighPriorityImu")), BuiltinImu->GetPriority() + 10);
    TestTrue(TEXT("High-priority factory registered"), Registry.RegisterExtension(PointName, TestOwner, HighPriority));
    const TSharedPtr<IUBotROSSensorPublisherFactory> Selected = UBot::ROSPublishing::FindFactory(Imu);
    TestTrue(TEXT("The higher-priority factory wins"), Selected.Get() == &HighPriority.Get());
    TestTrue(TEXT("Other sensor types keep their built-in factory"), UBot::ROSPublishing::FindFactory(Lidar) == BuiltinLidar);

    const TArray<TSharedRef<IUBotROSSensorPublisherFactory>> Ordered = UBot::ROSPublishing::FindFactories(Imu);
    if (TestTrue(TEXT("Four factories support the IMU"), Ordered.Num() >= 4))
    {
        TestTrue(TEXT("Order[0] is the highest priority"), &Ordered[0].Get() == &HighPriority.Get());
        TestTrue(TEXT("Order[last] is the lowest priority"), &Ordered.Last().Get() == &LowPriority.Get());
        const int32 BuiltinIndex = Ordered.IndexOfByPredicate([&BuiltinImu](const TSharedRef<IUBotROSSensorPublisherFactory>& Factory) { return &Factory.Get() == BuiltinImu.Get(); });
        const int32 SameIndex = Ordered.IndexOfByPredicate([&SamePriority](const TSharedRef<IUBotROSSensorPublisherFactory>& Factory) { return &Factory.Get() == &SamePriority.Get(); });
        TestTrue(TEXT("Ties keep registration order"), BuiltinIndex != INDEX_NONE && SameIndex != INDEX_NONE && BuiltinIndex < SameIndex);
    }

    // The bridge component uses the selected factory.
    {
        FScopedNoAutoConnect NoAutoConnect;
        FScopedBridgeTestWorld Scope;
        FTestRobot Robot = FTestRobot::Spawn(Scope.Get(), TEXT("priority_bot"));
        if (TestNotNull(TEXT("Robot spawned"), Robot.BridgeComponent))
        {
            UUBotImuSensorComponent* RobotImu = Robot.AddSensor<UUBotImuSensorComponent>(TEXT("Imu"), TEXT("imu"));
            TestTrue(TEXT("Bridge component ready"), Robot.BridgeComponent->RestartROSBridge());
            TestEqual(TEXT("The high-priority factory created the IMU publisher"), HighPriority->CreatedPublishers, 1);
            TestEqual(TEXT("The other test factories created nothing"), SamePriority->CreatedPublishers + LowPriority->CreatedPublishers, 0);
            TestNotNull(TEXT("The IMU has a publisher"), Robot.BridgeComponent->FindPublisherForSensor(RobotImu));
        }
    }

    Registry.UnregisterAllFromPackage(TestOwner);
    TestTrue(TEXT("Removing the test factories restores the built-in selection"), UBot::ROSPublishing::FindFactory(Imu) == BuiltinImu);
    return true;
}

// -------------------------------------------------------------------------------------------------

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotROSSensorBridgeTopicsTest, "UBotROS.SensorBridge.TopicsAndTF", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotROSSensorBridgeTopicsTest::RunTest(const FString& Parameters)
{
    using namespace UBotROSFactoryTests;

    FScopedNoAutoConnect NoAutoConnect;
    FScopedBridgeTestWorld Scope;
    FTestRobot Robot = FTestRobot::Spawn(Scope.Get(), TEXT("test_bot"));
    if (!TestNotNull(TEXT("Robot spawned"), Robot.BridgeComponent))
    {
        return false;
    }

    UUBotImuSensorComponent* Imu = Robot.AddSensor<UUBotImuSensorComponent>(TEXT("Imu"), TEXT("imu_main"), FVector(10.0, 20.0, 30.0));
    Robot.AddSensor<UUBot2DLidarSensorComponent>(TEXT("FrontLidar"), TEXT("front"));
    Robot.AddSensor<UUBot2DLidarSensorComponent>(TEXT("RearLidar"), TEXT("rear"));
    UUBotRgbCameraSensorComponent* Camera = Robot.AddSensor<UUBotRgbCameraSensorComponent>(TEXT("Camera"), TEXT("cam"), FVector(50.0, 0.0, 100.0));
    // An odometry sensor with its default FrameId "odom": the odometry frame, which is the PARENT of base_link.
    UUBotOdometrySensorComponent* Odometry = Robot.AddSensor<UUBotOdometrySensorComponent>(TEXT("Odometry"), TEXT("odometry_main"));
    Odometry->FrameId = TEXT("odom");

    UUBotROSSensorBridgeComponent* Bridge = Robot.BridgeComponent;
    // best_effort state QoS: while the bridge is not connected the odom topic itself is skipped, so the only
    // message the odometry sample below can publish is the dynamic /tf transform.
    Bridge->StateQosOverride.bOverride = true;
    Bridge->StateQosOverride.Qos = FUBotROSQos::SensorData();
    TestTrue(TEXT("Initially uninitialized"), Bridge->GetROSBridgeStatus() == EUBotROSSensorBridgeStatus::Uninitialized);
    if (!TestTrue(*FString::Printf(TEXT("Bridge becomes ready without a connection (error: %s)"), *Bridge->GetROSBridgeError()), Bridge->RestartROSBridge()))
    {
        return false;
    }
    TestTrue(TEXT("Status Ready"), Bridge->IsROSBridgeReady());
    TestEqual(TEXT("Discovered sensors"), Bridge->GetDiscoveredSensorCount(), 5);
    TestEqual(TEXT("Namespace"), *Bridge->GetResolvedNamespace(), TEXT("test_bot"));

    // Legacy layout: single IMU and camera use the fixed paths, the two 2D lidars use their sensor ids.
    const TArray<FString> ExpectedTopics =
    {
        TEXT("/test_bot/camera/color/camera_info"),
        TEXT("/test_bot/camera/color/image_raw"),
        TEXT("/test_bot/front"),
        TEXT("/test_bot/imu/data"),
        TEXT("/test_bot/odom"),
        TEXT("/test_bot/rear"),
    };
    TestEqualSensitive(TEXT("Default topics"), *JoinTopics(Bridge->GetActivePublisherTopics()), *JoinTopics(ExpectedTopics));

    const UUBotROSSensorPublisher* ImuPublisher = Bridge->FindPublisherForSensor(Imu);
    const UUBotROSSensorPublisher* CameraPublisher = Bridge->FindPublisherForSensor(Camera);
    if (TestNotNull(TEXT("IMU publisher"), ImuPublisher) && TestNotNull(TEXT("Camera publisher"), CameraPublisher))
    {
        TestEqualSensitive(TEXT("IMU frame (empty FrameId -> sensor id)"), *ImuPublisher->GetSensorFrameId(), TEXT("imu_main"));
        TestEqualSensitive(TEXT("Camera optical frame"), *CameraPublisher->GetOpticalFrameId(), TEXT("cam_optical"));
        TestTrue(TEXT("Camera publisher uses the optical frame"), CameraPublisher->UsesOpticalFrame());
    }

    // Static TF: base -> each sensor, camera -> optical, aggregated by the subsystem.
    UUBotROSBridgeSubsystem* Subsystem = UUBotROSBridgeSubsystem::Get(Scope.Get());
    if (TestNotNull(TEXT("Bridge subsystem"), Subsystem))
    {
        Scope.Tick(0.01f);
        const TArray<FUBotRosTransformStamped>& Transforms = Subsystem->GetMergedStaticTransforms();
        TestEqual(TEXT("Static transforms: 4 physical sensors + 1 optical frame (the odometry sensor has none)"), Transforms.Num(), 5);
        // Regression: base_link -> odom next to the dynamic odom -> base_link made a TF loop (and collided with
        // map -> odom of SLAM / AMCL).
        TestNull(TEXT("No static transform to the odometry frame"), FindStaticTransform(Transforms, TEXT("odom")));
        for (const FUBotRosTransformStamped& Transform : Transforms)
        {
            TestFalse(*FString::Printf(TEXT("No static transform is published with the odometry frame as parent or child (%s -> %s)"), *Transform.header.frame_id, *Transform.child_frame_id),
                Transform.header.frame_id.Equals(TEXT("odom"), ESearchCase::CaseSensitive) || Transform.child_frame_id.Equals(TEXT("odom"), ESearchCase::CaseSensitive));
        }
        if (const FUBotRosTransformStamped* ImuTf = FindStaticTransform(Transforms, TEXT("imu_main")))
        {
            TestEqualSensitive(TEXT("IMU TF parent"), *ImuTf->header.frame_id, TEXT("base_link"));
            TestEqual(TEXT("IMU TF x (m)"), ImuTf->transform.translation.x, 0.1, 1.e-9);
            TestEqual(TEXT("IMU TF y (m, mirrored)"), ImuTf->transform.translation.y, -0.2, 1.e-9);
            TestEqual(TEXT("IMU TF z (m)"), ImuTf->transform.translation.z, 0.3, 1.e-9);
            TestEqual(TEXT("IMU TF rotation w"), ImuTf->transform.rotation.w, 1.0, 1.e-9);
        }
        else
        {
            AddError(TEXT("No static TF base_link -> imu_main"));
        }
        if (const FUBotRosTransformStamped* OpticalTf = FindStaticTransform(Transforms, TEXT("cam_optical")))
        {
            const FQuat Expected = UBot::ROSConventions::CameraLinkToOpticalRotation();
            TestEqualSensitive(TEXT("Optical TF parent"), *OpticalTf->header.frame_id, TEXT("cam"));
            TestEqual(TEXT("Optical TF qx"), OpticalTf->transform.rotation.x, Expected.X, 1.e-9);
            TestEqual(TEXT("Optical TF qy"), OpticalTf->transform.rotation.y, Expected.Y, 1.e-9);
            TestEqual(TEXT("Optical TF qz"), OpticalTf->transform.rotation.z, Expected.Z, 1.e-9);
            TestEqual(TEXT("Optical TF qw"), OpticalTf->transform.rotation.w, Expected.W, 1.e-9);
        }
        else
        {
            AddError(TEXT("No static TF cam -> cam_optical"));
        }
        TestNotNull(TEXT("Static TF base_link -> cam"), FindStaticTransform(Transforms, TEXT("cam")));
        TestNotNull(TEXT("Static TF base_link -> front"), FindStaticTransform(Transforms, TEXT("front")));
        TestNotNull(TEXT("Static TF base_link -> rear"), FindStaticTransform(Transforms, TEXT("rear")));
    }

    // The dynamic odom -> base_link transform is still published for the odometry sensor.
    TestTrue(TEXT("The odometry sensor drives the dynamic TF"), Bridge->GetDynamicTFOdometrySensor() == Odometry);
    if (Subsystem)
    {
        FUBotOdometrySensorData Sample;
        Sample.Header.bValid = true;
        Sample.Header.TimestampSeconds = 1.0;
        Sample.OdometryFrameId = TEXT("odom");
        Sample.ChildFrameId = TEXT("base_link");
        Sample.PositionMeters = FVector(1.0, 2.0, 0.0);
        const int64 DroppedBefore = Subsystem->GetStats().DroppedMessages;
        Odometry->OnOdometryDataUpdated.Broadcast(Sample);
        // Not connected: the published /tf message is counted as dropped (the best_effort odom topic is skipped).
        TestEqual(TEXT("A valid odometry sample publishes exactly one dynamic /tf message"), Subsystem->GetStats().DroppedMessages - DroppedBefore, int64(1));
    }

    // Overrides and frame prefixes apply on refresh.
    FUBotROSTopicOverride ImuOverride;
    ImuOverride.bOverride = true;
    ImuOverride.Topic = TEXT("custom/imu");
    Bridge->TopicOverrides.Add(TEXT("imu"), ImuOverride);
    FUBotROSTopicOverride RearOverride;
    RearOverride.bOverride = true;
    RearOverride.Topic = TEXT("/rear_scan");
    Bridge->TopicOverrides.Add(TEXT("rear"), RearOverride);
    Bridge->bPrefixFramesWithNamespace = true;
    if (TestTrue(*FString::Printf(TEXT("Refresh with overrides (error: %s)"), *Bridge->GetROSBridgeError()), Bridge->RefreshSensorPublishers()))
    {
        const TArray<FString> Topics = Bridge->GetActivePublisherTopics();
        TestTrue(TEXT("Override without leading slash becomes absolute"), Topics.Contains(TEXT("/custom/imu")));
        TestTrue(TEXT("Override keyed by sensor id"), Topics.Contains(TEXT("/rear_scan")));
        TestFalse(TEXT("Default IMU topic is gone"), Topics.Contains(TEXT("/test_bot/imu/data")));
        TestEqualSensitive(TEXT("Prefixed base frame"), *Bridge->GetResolvedBaseFrameId(), TEXT("test_bot/base_link"));
        if (const UUBotROSSensorPublisher* Prefixed = Bridge->FindPublisherForSensor(Imu))
        {
            TestEqualSensitive(TEXT("Prefixed IMU frame"), *Prefixed->GetSensorFrameId(), TEXT("test_bot/imu_main"));
        }
    }

    // Stop releases everything.
    Bridge->StopROSBridge();
    TestTrue(TEXT("Status Stopped"), Bridge->GetROSBridgeStatus() == EUBotROSSensorBridgeStatus::Stopped);
    TestEqual(TEXT("No topics after stop"), Bridge->GetActivePublisherTopics().Num(), 0);
    TestEqual(TEXT("No sensors after stop"), Bridge->GetDiscoveredSensorCount(), 0);
    if (Subsystem)
    {
        Scope.Tick(0.01f);
        TestEqual(TEXT("Static transforms cleared after stop"), Subsystem->GetMergedStaticTransforms().Num(), 0);
    }
    return true;
}

// -------------------------------------------------------------------------------------------------

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotROSSensorBridgeBaseComponentTest, "UBotROS.SensorBridge.BaseComponent", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotROSSensorBridgeBaseComponentTest::RunTest(const FString& Parameters)
{
    using namespace UBotROSFactoryTests;

    FScopedNoAutoConnect NoAutoConnect;
    FScopedBridgeTestWorld Scope;
    FTestRobot Robot = FTestRobot::Spawn(Scope.Get(), TEXT("base_bot"));
    if (!TestNotNull(TEXT("Robot spawned"), Robot.BridgeComponent))
    {
        return false;
    }

    auto AddScene = [&Robot](const TCHAR* Name, const FVector& RelativeLocationCm)
    {
        USceneComponent* Component = NewObject<USceneComponent>(Robot.Actor, Name);
        Component->SetupAttachment(Robot.Root);
        Component->SetRelativeLocation(RelativeLocationCm);
        Component->RegisterComponent();
        Component->UpdateComponentToWorld();
        return Component;
    };
    USceneComponent* Chassis = AddScene(TEXT("Chassis"), FVector(200.0, 0.0, 0.0));
    USceneComponent* Mast = AddScene(TEXT("Mast"), FVector(-100.0, 0.0, 0.0));

    // The uBot identity names the chassis (not the actor root) as the base of the robot.
    UUBotIdentityComponent* Identity = NewObject<UUBotIdentityComponent>(Robot.Actor, TEXT("Identity"));
    Identity->BaseComponent.PathToComponent = Chassis->GetName();
    Identity->RegisterComponent();

    UUBotImuSensorComponent* Imu = Robot.AddSensor<UUBotImuSensorComponent>(TEXT("Imu"), TEXT("imu"), FVector(300.0, 0.0, 0.0));
    UUBotOdometrySensorComponent* Odometry = Robot.AddSensor<UUBotOdometrySensorComponent>(TEXT("Odometry"), TEXT("odometry"));
    Odometry->FrameId = TEXT("odom");
    TestNotNull(TEXT("IMU created"), Imu);

    UUBotROSSensorBridgeComponent* Bridge = Robot.BridgeComponent;
    UUBotROSBridgeSubsystem* Subsystem = UUBotROSBridgeSubsystem::Get(Scope.Get());
    if (!TestNotNull(TEXT("Bridge subsystem"), Subsystem))
    {
        return false;
    }

    // Static TF base_link -> imu in meters along ROS x: the IMU sits 300 cm from the root.
    auto ImuTransformX = [&]() -> TOptional<double>
    {
        Scope.Tick(0.01f);
        const FUBotRosTransformStamped* ImuTf = FindStaticTransform(Subsystem->GetMergedStaticTransforms(), TEXT("imu"));
        return ImuTf ? TOptional<double>(ImuTf->transform.translation.x) : TOptional<double>();
    };

    // 1. BaseLinkComponent left empty: the identity's base component (the chassis, 200 cm from the root) is the base.
    //    An empty FComponentReference resolves to the root component, which used to hide the identity completely.
    Odometry->CustomTrackedComponent.PathToComponent = Chassis->GetName();
    if (TestTrue(*FString::Printf(TEXT("Bridge ready with an empty BaseLinkComponent (error: %s)"), *Bridge->GetROSBridgeError()), Bridge->RestartROSBridge()))
    {
        const TOptional<double> X = ImuTransformX();
        if (TestTrue(TEXT("Static TF base_link -> imu exists"), X.IsSet()))
        {
            TestEqual(TEXT("Identity base: IMU is 100 cm in front of the chassis"), X.GetValue(), 1.0, 1.e-9);
        }
        TestTrue(TEXT("Dynamic TF enabled: the odometry sensor tracks the identity's base component"), Bridge->GetDynamicTFOdometrySensor() == Odometry);
    }

    // 2. An explicit BaseLinkComponent wins over the identity.
    Bridge->BaseLinkComponent.PathToComponent = Mast->GetName();
    Odometry->CustomTrackedComponent.PathToComponent = Mast->GetName();
    if (TestTrue(*FString::Printf(TEXT("Refresh with an explicit BaseLinkComponent (error: %s)"), *Bridge->GetROSBridgeError()), Bridge->RefreshSensorPublishers()))
    {
        const TOptional<double> X = ImuTransformX();
        if (TestTrue(TEXT("Static TF base_link -> imu exists (explicit base)"), X.IsSet()))
        {
            TestEqual(TEXT("Explicit base: IMU is 400 cm in front of the mast"), X.GetValue(), 4.0, 1.e-9);
        }
        TestTrue(TEXT("Dynamic TF follows the explicit base component"), Bridge->GetDynamicTFOdometrySensor() == Odometry);
    }

    // 3. A BaseLinkComponent that does not resolve falls back to the identity's base component.
    Bridge->BaseLinkComponent.PathToComponent = TEXT("NoSuchComponent");
    Odometry->CustomTrackedComponent.PathToComponent = Chassis->GetName();
    if (TestTrue(*FString::Printf(TEXT("Refresh with an unresolved BaseLinkComponent (error: %s)"), *Bridge->GetROSBridgeError()), Bridge->RefreshSensorPublishers()))
    {
        const TOptional<double> X = ImuTransformX();
        if (TestTrue(TEXT("Static TF base_link -> imu exists (unresolved base)"), X.IsSet()))
        {
            TestEqual(TEXT("Unresolved base: identity base again"), X.GetValue(), 1.0, 1.e-9);
        }
        TestTrue(TEXT("Dynamic TF enabled again"), Bridge->GetDynamicTFOdometrySensor() == Odometry);
    }

    // 4. Without any identity component and without a reference the actor root is the base.
    Identity->DestroyComponent();
    Bridge->BaseLinkComponent = FComponentReference();
    Odometry->CustomTrackedComponent = FComponentReference();
    if (TestTrue(*FString::Printf(TEXT("Refresh without an identity (error: %s)"), *Bridge->GetROSBridgeError()), Bridge->RefreshSensorPublishers()))
    {
        const TOptional<double> X = ImuTransformX();
        if (TestTrue(TEXT("Static TF base_link -> imu exists (root base)"), X.IsSet()))
        {
            TestEqual(TEXT("No identity: the root is the base"), X.GetValue(), 3.0, 1.e-9);
        }
        TestTrue(TEXT("Dynamic TF enabled for the root"), Bridge->GetDynamicTFOdometrySensor() == Odometry);
    }
    return true;
}

// -------------------------------------------------------------------------------------------------

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotROSSensorBridgeValidationTest, "UBotROS.SensorBridge.Validation", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotROSSensorBridgeValidationTest::RunTest(const FString& Parameters)
{
    using namespace UBotROSFactoryTests;

    FScopedNoAutoConnect NoAutoConnect;
    FScopedBridgeTestWorld Scope;

    // Duplicate sensor id within one type.
    {
        FTestRobot Robot = FTestRobot::Spawn(Scope.Get(), TEXT("dup_bot"));
        if (TestNotNull(TEXT("Robot spawned"), Robot.BridgeComponent))
        {
            UUBot2DLidarSensorComponent* First = Robot.AddSensor<UUBot2DLidarSensorComponent>(TEXT("LidarA"), TEXT("same"));
            UUBot2DLidarSensorComponent* Second = Robot.AddSensor<UUBot2DLidarSensorComponent>(TEXT("LidarB"), TEXT("same"));
            First->FrameId = TEXT("frame_a");
            Second->FrameId = TEXT("frame_b");
            AddExpectedErrorPlain(TEXT("Duplicate SensorId 'same'"), EAutomationExpectedErrorFlags::Contains, 1);
            TestFalse(TEXT("Duplicate sensor ids fail"), Robot.BridgeComponent->RestartROSBridge());
            TestTrue(TEXT("Status Error"), Robot.BridgeComponent->GetROSBridgeStatus() == EUBotROSSensorBridgeStatus::Error);
            TestTrue(TEXT("Error names the duplicate"), Robot.BridgeComponent->GetROSBridgeError().Contains(TEXT("Duplicate SensorId")));
            TestEqual(TEXT("No publisher survives a failed configuration"), Robot.BridgeComponent->GetActivePublisherTopics().Num(), 0);
        }
    }

    // The same sensor id in different types is fine, a duplicate frame id is not.
    {
        FTestRobot Robot = FTestRobot::Spawn(Scope.Get(), TEXT("frame_bot"));
        if (TestNotNull(TEXT("Robot spawned"), Robot.BridgeComponent))
        {
            UUBotImuSensorComponent* Imu = Robot.AddSensor<UUBotImuSensorComponent>(TEXT("Imu"), TEXT("unit"));
            UUBot2DLidarSensorComponent* Lidar = Robot.AddSensor<UUBot2DLidarSensorComponent>(TEXT("Lidar"), TEXT("unit"));
            Imu->FrameId = TEXT("shared_frame");
            Lidar->FrameId = TEXT("shared_frame");
            AddExpectedErrorPlain(TEXT("Duplicate sensor FrameId 'shared_frame'"), EAutomationExpectedErrorFlags::Contains, 1);
            TestFalse(TEXT("Duplicate frame ids fail"), Robot.BridgeComponent->RestartROSBridge());

            Lidar->FrameId = TEXT("lidar_frame");
            TestTrue(*FString::Printf(TEXT("Same id in different types is accepted (error: %s)"), *Robot.BridgeComponent->GetROSBridgeError()), Robot.BridgeComponent->RestartROSBridge());
        }
    }

    // Two publishers resolving to the same topic.
    {
        FTestRobot Robot = FTestRobot::Spawn(Scope.Get(), TEXT("conflict_bot"));
        if (TestNotNull(TEXT("Robot spawned"), Robot.BridgeComponent))
        {
            Robot.AddSensor<UUBotImuSensorComponent>(TEXT("Imu"), TEXT("imu"));
            Robot.AddSensor<UUBot2DLidarSensorComponent>(TEXT("Lidar"), TEXT("lidar"));
            FUBotROSTopicOverride Override;
            Override.bOverride = true;
            Override.Topic = TEXT("/conflict_bot/scan");
            Robot.BridgeComponent->TopicOverrides.Add(TEXT("imu"), Override);
            AddExpectedErrorPlain(TEXT("ROS topic conflict: '/conflict_bot/scan'"), EAutomationExpectedErrorFlags::Contains, 1);
            TestFalse(TEXT("Topic conflicts fail"), Robot.BridgeComponent->RestartROSBridge());
            TestTrue(TEXT("Status Error after a conflict"), Robot.BridgeComponent->GetROSBridgeStatus() == EUBotROSSensorBridgeStatus::Error);
        }
    }

    // Invalid override topic.
    {
        FTestRobot Robot = FTestRobot::Spawn(Scope.Get(), TEXT("invalid_bot"));
        if (TestNotNull(TEXT("Robot spawned"), Robot.BridgeComponent))
        {
            Robot.AddSensor<UUBotImuSensorComponent>(TEXT("Imu"), TEXT("imu"));
            FUBotROSTopicOverride Override;
            Override.bOverride = true;
            Override.Topic = TEXT("/bad topic");
            Robot.BridgeComponent->TopicOverrides.Add(TEXT("imu"), Override);
            AddExpectedErrorPlain(TEXT("is not a valid absolute topic"), EAutomationExpectedErrorFlags::Contains, 1);
            TestFalse(TEXT("Invalid topics fail"), Robot.BridgeComponent->RestartROSBridge());

            // Refresh after fixing the configuration recovers.
            Robot.BridgeComponent->TopicOverrides.Reset();
            TestTrue(*FString::Printf(TEXT("Refresh after the fix succeeds (error: %s)"), *Robot.BridgeComponent->GetROSBridgeError()), Robot.BridgeComponent->RefreshSensorPublishers());
            TestTrue(TEXT("Default topic restored"), Robot.BridgeComponent->GetActivePublisherTopics().Contains(TEXT("/invalid_bot/imu/data")));
        }
    }

    // "/" selects the root namespace (same rule as the velocity command component).
    {
        FTestRobot Robot = FTestRobot::Spawn(Scope.Get(), TEXT("/"));
        if (TestNotNull(TEXT("Robot spawned"), Robot.BridgeComponent))
        {
            Robot.AddSensor<UUBotImuSensorComponent>(TEXT("Imu"), TEXT("imu"));
            TestTrue(*FString::Printf(TEXT("Root namespace is accepted (error: %s)"), *Robot.BridgeComponent->GetROSBridgeError()), Robot.BridgeComponent->RestartROSBridge());
            TestEqualSensitive(TEXT("Root namespace topics"), *JoinTopics(Robot.BridgeComponent->GetActivePublisherTopics()), TEXT("/imu/data"));
            TestEqualSensitive(TEXT("Root namespace base frame is not prefixed"), *Robot.BridgeComponent->GetResolvedBaseFrameId(), TEXT("base_link"));
        }
    }

    // A blank override falls back to the owner's uBot identity (actor name without a uBot Identity component).
    {
        FTestRobot Robot = FTestRobot::Spawn(Scope.Get(), TEXT("  "));
        if (TestNotNull(TEXT("Robot spawned"), Robot.BridgeComponent))
        {
            TestEqualSensitive(TEXT("Identity namespace"), *Robot.BridgeComponent->GetResolvedNamespace(),
                *UBot::ROSPublishing::SanitizeRosPath(UBot::ROSConventions::SanitizeRosName(Robot.Actor->GetName())));
        }
    }

    // An actor without supported sensors is Ready with no topics (legacy behaviour).
    {
        FTestRobot Robot = FTestRobot::Spawn(Scope.Get(), TEXT("empty_bot"));
        if (TestNotNull(TEXT("Robot spawned"), Robot.BridgeComponent))
        {
            TestTrue(TEXT("No sensors is not an error"), Robot.BridgeComponent->RestartROSBridge());
            TestEqual(TEXT("No topics"), Robot.BridgeComponent->GetActivePublisherTopics().Num(), 0);
        }
    }
    return true;
}

// -------------------------------------------------------------------------------------------------

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotROSRedirectsResolveTest, "UBotROS.Redirects.Resolve", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotROSRedirectsResolveTest::RunTest(const FString& Parameters)
{
    using namespace UBotROSFactoryTests;

    const FString LegacyPackage = TEXT("/Script/AgentSensorROSBridge");

    TestEqual(TEXT("Component class redirect"),
        *Redirect(ECoreRedirectFlags::Type_Class, LegacyPackage + TEXT(".AgentROSSensorSuitePublisherComponent")),
        *UUBotROSSensorBridgeComponent::StaticClass()->GetPathName());
    TestEqual(TEXT("Script package redirect"), *Redirect(ECoreRedirectFlags::Type_Package, LegacyPackage), TEXT("/Script/UBotROS"));
    TestEqual(TEXT("Topic override struct redirect"),
        *Redirect(ECoreRedirectFlags::Type_Struct, LegacyPackage + TEXT(".AgentROSTopicOverride")),
        *FUBotROSTopicOverride::StaticStruct()->GetPathName());
    TestEqual(TEXT("QoS override struct redirect"),
        *Redirect(ECoreRedirectFlags::Type_Struct, LegacyPackage + TEXT(".AgentROSQOSOverride")),
        *FUBotROSQosOverride::StaticStruct()->GetPathName());
    TestEqual(TEXT("Status enum redirect"),
        *Redirect(ECoreRedirectFlags::Type_Enum, LegacyPackage + TEXT(".EAgentROSBridgeStatus")),
        *StaticEnum<EUBotROSSensorBridgeStatus>()->GetPathName());
    TestEqual(TEXT("Reliability enum redirect"),
        *Redirect(ECoreRedirectFlags::Type_Enum, LegacyPackage + TEXT(".EAgentROSReliability")),
        *StaticEnum<EUBotROSReliability>()->GetPathName());
    TestEqual(TEXT("Legacy status value names resolve"),
        StaticEnum<EUBotROSSensorBridgeStatus>()->GetValueByName(FName(TEXT("EAgentROSBridgeStatus::Ready"))),
        static_cast<int64>(EUBotROSSensorBridgeStatus::Ready));
    TestEqual(TEXT("Legacy reliability value names resolve"),
        StaticEnum<EUBotROSReliability>()->GetValueByName(FName(TEXT("EAgentROSReliability::Reliable"))),
        static_cast<int64>(EUBotROSReliability::Reliable));

    // The legacy QoS values load into the load-only members (reflected under the legacy names), which
    // PostSerialize moves into Qos; a property redirect would break that, so there must be none.
    const UScriptStruct* QosOverrideStruct = FUBotROSQosOverride::StaticStruct();
    for (const TCHAR* LegacyName : { TEXT("Reliability"), TEXT("QueueSize") })
    {
        const FProperty* LoadOnly = QosOverrideStruct->FindPropertyByName(FName(LegacyName));
        if (TestNotNull(*FString::Printf(TEXT("QoS override has a load-only member reflected as %s"), LegacyName), LoadOnly))
        {
            TestTrue(*FString::Printf(TEXT("%s is flagged deprecated (never saved)"), LegacyName), LoadOnly->HasAnyPropertyFlags(CPF_Deprecated));
        }
        TestTrue(*FString::Printf(TEXT("%s has no property redirect"), LegacyName), FProperty::FindRedirectedPropertyName(QosOverrideStruct, FName(LegacyName)).IsNone());
    }

    // Property redirects are looked up with the class the data loads into.
    const UClass* ComponentClass = UUBotROSSensorBridgeComponent::StaticClass();
    TestEqual(TEXT("RobotNamespace -> RobotNamespaceOverride"),
        *FProperty::FindRedirectedPropertyName(ComponentClass, FName(TEXT("RobotNamespace"))).ToString(), TEXT("RobotNamespaceOverride"));
    TestEqual(TEXT("BaseLinkFrame -> BaseFrameOverride"),
        *FProperty::FindRedirectedPropertyName(ComponentClass, FName(TEXT("BaseLinkFrame"))).ToString(), TEXT("BaseFrameOverride"));
    TestNotNull(TEXT("Legacy QoS override property names still match (FName is case-insensitive)"),
        ComponentClass->FindPropertyByName(FName(TEXT("SensorDataQOSOverride"))));
    for (const TCHAR* Unchanged : { TEXT("bAutoInitialize"), TEXT("bPublishInvalidSamples"), TEXT("bPublishDynamicTF"), TEXT("bPublishStaticSensorTF"),
        TEXT("OdometryFrame"), TEXT("BaseLinkComponent"), TEXT("TopicOverrides"), TEXT("StateQOSOverride"),
        TEXT("OnROSBridgeReady"), TEXT("OnROSBridgeError"), TEXT("OnROSPublishersRefreshed") })
    {
        TestNotNull(*FString::Printf(TEXT("Legacy property %s exists"), Unchanged), ComponentClass->FindPropertyByName(FName(Unchanged)));
    }
    for (const TCHAR* Function : { TEXT("IsROSBridgeReady"), TEXT("GetROSBridgeStatus"), TEXT("GetROSBridgeError"), TEXT("GetDiscoveredSensorCount"),
        TEXT("GetActivePublisherTopics"), TEXT("RefreshSensorPublishers"), TEXT("RestartROSBridge"), TEXT("StopROSBridge") })
    {
        TestNotNull(*FString::Printf(TEXT("Legacy Blueprint function %s exists"), Function), ComponentClass->FindFunctionByName(FName(Function)));
    }

    // Delegate signatures (linker resolves DelegateFunction imports with Type_Object).
    for (const TCHAR* Delegate : { TEXT("AgentROSBridgeReadyEvent"), TEXT("AgentROSBridgeErrorEvent"), TEXT("AgentROSPublishersRefreshedEvent") })
    {
        const FString NewPath = Redirect(ECoreRedirectFlags::Type_Object, FString::Printf(TEXT("%s.%s__DelegateSignature"), *LegacyPackage, Delegate));
        TestNotNull(*FString::Printf(TEXT("Delegate signature %s -> %s exists"), Delegate, *NewPath), FindObject<UFunction>(nullptr, *NewPath));
    }

    // The vehicle / drone command components are deliberately not redirected.
    TestEqual(TEXT("Vehicle command component is not redirected"),
        *Redirect(ECoreRedirectFlags::Type_Class, TEXT("/Script/AgentVehicleROSBridge.AgentROSVehicleCommandComponent")),
        TEXT("/Script/AgentVehicleROSBridge.AgentROSVehicleCommandComponent"));
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotROSRedirectsQosOverrideValuesTest, "UBotROS.Redirects.QosOverrideValues", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotROSRedirectsQosOverrideValuesTest::RunTest(const FString& Parameters)
{
    using namespace UBotROSFactoryTests;

    // Legacy data = tagged properties with the legacy names; loading goes through UScriptStruct::SerializeItem
    // (property redirects, then PostSerialize), exactly like the struct member of a loaded component.
    auto LoadLegacy = [](bool bOverride, EUBotROSReliability Reliability, int32 QueueSize) -> FUBotROSQosOverride
    {
        FUBotROSLegacyQosOverrideMirror Legacy;
        Legacy.bOverride = bOverride;
        Legacy.Reliability = Reliability;
        Legacy.QueueSize = QueueSize;

        TArray<uint8> Bytes;
        FMemoryWriter Writer(Bytes, /*bIsPersistent*/ true);
        FUBotROSLegacyQosOverrideMirror::StaticStruct()->SerializeItem(Writer, &Legacy, nullptr);

        FUBotROSQosOverride Loaded;
        FMemoryReader Reader(Bytes, /*bIsPersistent*/ true);
        FUBotROSQosOverride::StaticStruct()->SerializeItem(Reader, &Loaded, nullptr);
        return Loaded;
    };

    // {true, Reliable, 20}: the values survive instead of falling back to best_effort / 5.
    AddExpectedErrorPlain(TEXT("Migrated a legacy QoS override (reliability reliable, depth 20)"), EAutomationExpectedErrorFlags::Contains, 1);
    const FUBotROSQosOverride Reliable = LoadLegacy(true, EUBotROSReliability::Reliable, 20);
    TestTrue(TEXT("bOverride kept"), Reliable.bOverride);
    TestTrue(TEXT("Reliability migrated"), Reliable.Qos.Reliability == EUBotROSReliability::Reliable);
    TestEqual(TEXT("QueueSize migrated to Depth"), Reliable.Qos.Depth, 20);
    TestTrue(TEXT("Legacy durability was always volatile"), Reliable.Qos.Durability == EUBotROSDurability::Volatile);
    TestEqual(TEXT("Load-only reliability cleared"), static_cast<int32>(Reliable.Reliability_DEPRECATED), static_cast<int32>(FUBotROSQosOverride::LegacyReliabilityAbsent));
    TestEqual(TEXT("Load-only queue size cleared"), Reliable.QueueSize_DEPRECATED, 0);

    AddExpectedErrorPlain(TEXT("Migrated a legacy QoS override (reliability best_effort, depth 250)"), EAutomationExpectedErrorFlags::Contains, 1);
    const FUBotROSQosOverride BestEffort = LoadLegacy(true, EUBotROSReliability::BestEffort, 250);
    TestTrue(TEXT("best_effort kept"), BestEffort.Qos.Reliability == EUBotROSReliability::BestEffort);
    TestEqual(TEXT("Depth 250"), BestEffort.Qos.Depth, 250);

    // A disabled override carries its values too, silently (nothing is applied to the publishers).
    const FUBotROSQosOverride Disabled = LoadLegacy(false, EUBotROSReliability::Reliable, 7);
    TestFalse(TEXT("Disabled override stays disabled"), Disabled.bOverride);
    TestTrue(TEXT("Disabled override still carries the legacy reliability"), Disabled.Qos.Reliability == EUBotROSReliability::Reliable);
    TestEqual(TEXT("Disabled override still carries the legacy depth"), Disabled.Qos.Depth, 7);

    // The legacy defaults map to the new defaults.
    const FUBotROSQosOverride Defaults = LoadLegacy(false, EUBotROSReliability::BestEffort, 5);
    TestTrue(TEXT("Legacy defaults = FUBotROSQos defaults"), Defaults.Qos == FUBotROSQos::SensorData());

    // Data in the new format is not touched: the load-only members are never written, so nothing migrates.
    FUBotROSQosOverride Current;
    Current.bOverride = true;
    Current.Qos = FUBotROSQos(EUBotROSReliability::Reliable, EUBotROSDurability::TransientLocal, 7);
    Current.Reliability_DEPRECATED = EUBotROSReliability::BestEffort;
    Current.QueueSize_DEPRECATED = 99;
    TArray<uint8> Bytes;
    FMemoryWriter Writer(Bytes, /*bIsPersistent*/ true);
    FUBotROSQosOverride::StaticStruct()->SerializeItem(Writer, &Current, nullptr);
    FUBotROSQosOverride Loaded;
    FMemoryReader Reader(Bytes, /*bIsPersistent*/ true);
    FUBotROSQosOverride::StaticStruct()->SerializeItem(Reader, &Loaded, nullptr);
    TestTrue(TEXT("New-format override loads unchanged"), Loaded.bOverride && Loaded.Qos == Current.Qos);
    TestTrue(TEXT("Load-only members are not saved"), static_cast<uint8>(Loaded.Reliability_DEPRECATED) == FUBotROSQosOverride::LegacyReliabilityAbsent && Loaded.QueueSize_DEPRECATED == 0);
    return true;
}

#endif
