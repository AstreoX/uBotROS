#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "Components/SceneComponent.h"
#include "Engine/Engine.h"
#include "Engine/EngineBaseTypes.h"
#include "Engine/World.h"
#include "EngineUtils.h"
#include "GameFramework/Actor.h"
#include "UBotIdentityComponent.h"
#include "UBotROSAttitudeCommandComponent.h"
#include "UBotROSFlightCommandTestTypes.h"
#include "UBotROSMessageTypes.h"
#include "UBotROSMotorCommandComponent.h"
#include "UObject/StrongObjectPtr.h"

namespace UBotROSFlightCommandTests
{
    constexpr float FrameSeconds = 0.1f;
    constexpr EAutomationTestFlags TestFlags = EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter;

    // Transient game world ticked by hand (same setup as the velocity command tests).
    class FScopedWorld
    {
    public:
        UE_NONCOPYABLE(FScopedWorld);

        FScopedWorld()
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

        ~FScopedWorld()
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

        UWorld* Get() const { return World; }

        void TickFrames(int32 Count) const
        {
            for (int32 Frame = 0; Frame < Count; ++Frame)
            {
                World->Tick(LEVELTICK_All, FrameSeconds);
                ++GFrameCounter;
            }
        }

    private:
        UWorld* World = nullptr;
        uint64 CachedFrameCounter = 0;
    };

    template <typename TComponent>
    struct FFixture
    {
        AActor* Owner = nullptr;
        TComponent* Command = nullptr;
        UUBotROSFlightCommandTestTarget* Target = nullptr;
        TStrongObjectPtr<UUBotROSFlightCommandTestListener> Listener;
    };

    /** An actor with an optional recording target and a command component that does not subscribe (payloads are injected). */
    template <typename TComponent>
    FFixture<TComponent> Spawn(const FScopedWorld& Scope, bool bWithTarget, const TCHAR* RobotId = nullptr)
    {
        FFixture<TComponent> Fixture;
        Fixture.Owner = Scope.Get()->SpawnActor<AActor>();
        if (RobotId)
        {
            UUBotIdentityComponent* Identity = NewObject<UUBotIdentityComponent>(Fixture.Owner, TEXT("Identity"));
            Identity->RobotId = RobotId;
            Identity->RegisterComponent();
        }
        if (bWithTarget)
        {
            Fixture.Target = NewObject<UUBotROSFlightCommandTestTarget>(Fixture.Owner, TEXT("TestVehicle"));
            Fixture.Target->RegisterComponent();
        }
        Fixture.Command = NewObject<TComponent>(Fixture.Owner, TEXT("RosCommand"));
        Fixture.Command->bAutoStartReceiver = false;
        Fixture.Command->RegisterComponent();
        Fixture.Listener.Reset(NewObject<UUBotROSFlightCommandTestListener>());
        Fixture.Listener->Bind(*Fixture.Command);
        Fixture.Owner->DispatchBeginPlay();
        return Fixture;
    }

    template <typename TMessage>
    void Inject(UUBotROSCommandReceiverComponent& Component, const TMessage& Message)
    {
        const TArray<uint8> Cdr = UBot::Ros::Encode(Message);
        FUBotROSCommandReceiverTestAccess::InjectPayload(Component, Cdr);
    }

    FUBotRosAttitudeTarget MakeAttitude(double RosYaw, float Thrust)
    {
        FUBotRosAttitudeTarget Message;
        Message.orientation = FUBotRosQuaternion::FromQuat(FQuat(FVector::UpVector, RosYaw));
        Message.thrust = Thrust;
        return Message;
    }

    template <typename TComponent, typename TMessage>
    bool TestOwnership(FAutomationTestBase& Test, const TMessage& Message,
        int32 UUBotROSFlightCommandTestTarget::*ReleaseCount)
    {
        FScopedWorld Scope;
        auto Fixture = Spawn<TComponent>(Scope, true);
        auto* Second = NewObject<UUBotROSFlightCommandTestTarget>(Fixture.Owner);
        Second->RegisterComponent();

        Inject(*Fixture.Command, Message);
        Fixture.Command->SetTargetComponent(Fixture.Target);
        Test.TestEqual(TEXT("Same target retains accepted command"), Fixture.Target->*ReleaseCount, 0);
        Fixture.Command->SetTargetComponent(Second);
        Test.TestEqual(TEXT("Switch releases actual old target immediately"), Fixture.Target->*ReleaseCount, 1);
        Test.TestTrue(TEXT("Switch preserves received-command timeout"), Fixture.Command->IsCommandActive());
        Scope.TickFrames(6);
        Test.TestEqual(TEXT("Uncommanded replacement is not released by timeout"), Second->*ReleaseCount, 0);
        Test.TestEqual(TEXT("Old target not released twice"), Fixture.Target->*ReleaseCount, 1);
        Test.TestEqual(TEXT("Timeout event remains armed"), Fixture.Listener->TimeoutCount, 1);

        Second->bAccept = false;
        Inject(*Fixture.Command, Message);
        Scope.TickFrames(6);
        Test.TestEqual(TEXT("Rejected command does not acquire ownership"), Second->*ReleaseCount, 0);
        Second->bAccept = true;
        Inject(*Fixture.Command, Message);
        Fixture.Command->TargetComponent.OverrideComponent = Fixture.Target;
        Scope.TickFrames(6);
        Test.TestEqual(TEXT("Timeout releases accepted target after direct reference edit"), Second->*ReleaseCount, 1);
        Test.TestEqual(TEXT("Timeout does not release newly resolved target"), Fixture.Target->*ReleaseCount, 1);

        Inject(*Fixture.Command, Message);
        auto* Invalid = NewObject<USceneComponent>(Fixture.Owner);
        Invalid->RegisterComponent();
        Fixture.Command->SetTargetComponent(Invalid);
        Test.TestEqual(TEXT("Invalid replacement also releases old target"), Fixture.Target->*ReleaseCount, 2);
        Scope.TickFrames(6);
        Test.TestEqual(TEXT("Invalid replacement cannot duplicate the release"), Fixture.Target->*ReleaseCount, 2);

        Fixture.Command->SetTargetComponent(nullptr);
        Fixture.Command->CommandTimeoutSeconds = 0.0f;
        Inject(*Fixture.Command, Message);
        Fixture.Command->SetTargetComponent(Second);
        Test.TestEqual(TEXT("Switch releases with timeout disabled"), Fixture.Target->*ReleaseCount, 3);
        Inject(*Fixture.Command, Message);
        Fixture.Command->TargetComponent.OverrideComponent = Fixture.Target;
        Fixture.Owner->RouteEndPlay(EEndPlayReason::Destroyed);
        Test.TestEqual(TEXT("EndPlay releases actual accepted target"), Second->*ReleaseCount, 2);
        Test.TestEqual(TEXT("EndPlay leaves uncommanded resolved target alone"), Fixture.Target->*ReleaseCount, 3);
        return true;
    }

    template <typename TComponent, typename TMessage>
    bool TestReentry(FAutomationTestBase& Test, const TMessage& Message,
        int32 UUBotROSFlightCommandTestTarget::*ReleaseCount)
    {
        FScopedWorld Scope;
        auto Fixture = Spawn<TComponent>(Scope, true);
        auto* Second = NewObject<UUBotROSFlightCommandTestTarget>(Fixture.Owner);
        Second->RegisterComponent();
        Fixture.Target->OnApply = [&]() { Fixture.Command->SetTargetComponent(Second); };
        Inject(*Fixture.Command, Message);
        Fixture.Target->OnApply = nullptr;
        Test.TestEqual(TEXT("Target changed inside Apply is released after accepting"), Fixture.Target->*ReleaseCount, 1);

        Inject(*Fixture.Command, Message);
        Second->OnRelease = [&]()
        {
            Fixture.Command->SetTargetComponent(Fixture.Target);
            Inject(*Fixture.Command, Message);
        };
        Fixture.Command->SetTargetComponent(nullptr);
        Second->OnRelease = nullptr;
        Test.TestEqual(TEXT("Release callback does not recurse"), Second->*ReleaseCount, 1);
        Scope.TickFrames(6);
        Test.TestEqual(TEXT("Reentrant command gets its own release"), Fixture.Target->*ReleaseCount, 2);

        Inject(*Fixture.Command, Message);
        Fixture.Target->DestroyComponent();
        Fixture.Command->SetTargetComponent(Second);
        Scope.TickFrames(6);
        Test.TestEqual(TEXT("Weak destroyed target cannot release replacement"), Second->*ReleaseCount, 1);

        Inject(*Fixture.Command, Message);
        Second->OnRelease = [&]() { Inject(*Fixture.Command, Message); };
        Fixture.Owner->RouteEndPlay(EEndPlayReason::Destroyed);
        Second->OnRelease = nullptr;
        Test.TestEqual(TEXT("EndPlay releases once despite reentrant input"), Second->*ReleaseCount, 2);
        Test.TestFalse(TEXT("EndPlay cannot rearm command"), Fixture.Command->IsCommandActive());
        return true;
    }
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotROSAttitudeTargetOwnershipTest, "UBotROS.AttitudeCommand.AcceptedTargetOwnership", UBotROSFlightCommandTests::TestFlags)

bool FUBotROSAttitudeTargetOwnershipTest::RunTest(const FString& Parameters)
{
    return UBotROSFlightCommandTests::TestOwnership<UUBotROSAttitudeCommandComponent>(*this,
        UBotROSFlightCommandTests::MakeAttitude(0.0, 0.5f), &UUBotROSFlightCommandTestTarget::AttitudeReleases);
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotROSAttitudeTargetReentryTest, "UBotROS.AttitudeCommand.TargetSwitchReentry", UBotROSFlightCommandTests::TestFlags)

bool FUBotROSAttitudeTargetReentryTest::RunTest(const FString& Parameters)
{
    // The same ownership path also covers body-rate commands.
    FUBotRosAttitudeTarget Message;
    Message.type_mask = FUBotRosAttitudeTarget::IGNORE_ATTITUDE;
    Message.thrust = 0.5f;
    return UBotROSFlightCommandTests::TestReentry<UUBotROSAttitudeCommandComponent>(*this,
        Message, &UUBotROSFlightCommandTestTarget::AttitudeReleases);
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotROSMotorTargetOwnershipTest, "UBotROS.MotorCommand.AcceptedTargetOwnership", UBotROSFlightCommandTests::TestFlags)

bool FUBotROSMotorTargetOwnershipTest::RunTest(const FString& Parameters)
{
    FUBotRosActuators Message;
    Message.normalized = { 0.5, 0.5, 0.5, 0.5 };
    return UBotROSFlightCommandTests::TestOwnership<UUBotROSMotorCommandComponent>(*this,
        Message, &UUBotROSFlightCommandTestTarget::MotorReleases);
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotROSMotorTargetReentryTest, "UBotROS.MotorCommand.TargetSwitchReentry", UBotROSFlightCommandTests::TestFlags)

bool FUBotROSMotorTargetReentryTest::RunTest(const FString& Parameters)
{
    FUBotRosActuators Message;
    Message.normalized = { 0.5, 0.5, 0.5, 0.5 };
    return UBotROSFlightCommandTests::TestReentry<UUBotROSMotorCommandComponent>(*this,
        Message, &UUBotROSFlightCommandTestTarget::MotorReleases);
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotROSAttitudeCommandDispatchTest, "UBotROS.AttitudeCommand.Dispatch", UBotROSFlightCommandTests::TestFlags)

bool FUBotROSAttitudeCommandDispatchTest::RunTest(const FString& Parameters)
{
    using namespace UBotROSFlightCommandTests;
    FScopedWorld Scope;
    auto Fixture = Spawn<UUBotROSAttitudeCommandComponent>(Scope, true, TEXT("drone_1"));
    TestEqual(TEXT("Default topic"), Fixture.Command->GetResolvedCommandTopic(), FString(TEXT("/drone_1/cmd_attitude")));
    TestTrue(TEXT("Target found on the owner"), Fixture.Command->GetCommandTarget() == Fixture.Target);

    Scope.TickFrames(2);
    Inject(*Fixture.Command, MakeAttitude(UE_DOUBLE_HALF_PI, 0.6f));
    if (TestEqual(TEXT("Attitude dispatched"), Fixture.Target->Attitudes.Num(), 1))
    {
        const FUBotAttitudeCommand& Applied = Fixture.Target->Attitudes[0];
        TestTrue(TEXT("ROS yaw left is UE nose along -Y"), Applied.Orientation.GetAxisX().Equals(FVector(0.0, -1.0, 0.0), 1e-9));
        TestEqual(TEXT("Thrust"), Applied.NormalizedThrust, 0.6, 1e-6);
        TestEqual(TEXT("Stamped with sim time"), Applied.TimestampSeconds, static_cast<double>(Scope.Get()->GetTimeSeconds()), 1e-9);
    }
    TestTrue(TEXT("Command active"), Fixture.Command->IsCommandActive());

    FUBotRosAttitudeTarget Rates;
    Rates.type_mask = FUBotRosAttitudeTarget::IGNORE_ATTITUDE | FUBotRosAttitudeTarget::IGNORE_THRUST;
    Rates.body_rate = FUBotRosVector3::FromVector(FVector(0.0, 0.0, 1.0));
    Inject(*Fixture.Command, Rates);
    if (TestEqual(TEXT("Body rate dispatched"), Fixture.Target->BodyRates.Num(), 1))
    {
        TestTrue(TEXT("ROS yaw left rate is UE -Z"), Fixture.Target->BodyRates[0].BodyRateRadiansPerSecond.Equals(FVector(0.0, 0.0, -1.0), 1e-12));
        TestFalse(TEXT("Thrust ignored"), Fixture.Target->BodyRates[0].bUseThrust);
    }

    // Malformed and invalid messages are dropped with an error.
    const TArray<uint8> Garbage = { 0x00, 0x01, 0x00, 0x00, 0x01 };
    FUBotROSCommandReceiverTestAccess::InjectPayload(*Fixture.Command, Garbage);
    FUBotRosAttitudeTarget Zero;
    Zero.orientation = FUBotRosQuaternion::FromQuat(FQuat(0.0, 0.0, 0.0, 0.0));
    Inject(*Fixture.Command, Zero);
    TestEqual(TEXT("Nothing more dispatched"), Fixture.Target->Attitudes.Num(), 1);
    TestEqual(TEXT("Two errors"), Fixture.Listener->Errors.Num(), 2);

    // A rejection is reported once until a command is accepted again.
    Fixture.Target->bAccept = false;
    Inject(*Fixture.Command, MakeAttitude(0.0, 0.5f));
    Inject(*Fixture.Command, MakeAttitude(0.0, 0.5f));
    TestEqual(TEXT("Rejection reported once"), Fixture.Listener->Errors.Num(), 3);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotROSAttitudeCommandTimeoutTest, "UBotROS.AttitudeCommand.TimeoutAndEndPlay", UBotROSFlightCommandTests::TestFlags)

bool FUBotROSAttitudeCommandTimeoutTest::RunTest(const FString& Parameters)
{
    using namespace UBotROSFlightCommandTests;
    FScopedWorld Scope;
    auto Fixture = Spawn<UUBotROSAttitudeCommandComponent>(Scope, true);
    Inject(*Fixture.Command, MakeAttitude(0.0, 0.5f));
    Scope.TickFrames(4);
    TestEqual(TEXT("Not released before the timeout"), Fixture.Target->AttitudeReleases, 0);
    Scope.TickFrames(2);
    TestEqual(TEXT("Released once after 0.5 s"), Fixture.Target->AttitudeReleases, 1);
    TestEqual(TEXT("Timeout event once"), Fixture.Listener->TimeoutCount, 1);
    TestFalse(TEXT("No longer active"), Fixture.Command->IsCommandActive());
    Scope.TickFrames(10);
    TestEqual(TEXT("Still released only once"), Fixture.Target->AttitudeReleases, 1);

    // An active command is released when the component ends play.
    Inject(*Fixture.Command, MakeAttitude(0.0, 0.5f));
    Fixture.Owner->RouteEndPlay(EEndPlayReason::Destroyed);
    TestEqual(TEXT("Released at end of play"), Fixture.Target->AttitudeReleases, 2);

    // Without a target the component still decodes and times out.
    auto Lonely = Spawn<UUBotROSAttitudeCommandComponent>(Scope, false);
    Inject(*Lonely.Command, MakeAttitude(0.0, 0.5f));
    TestEqual(TEXT("Last attitude kept without a target"), Lonely.Command->GetLastAttitudeCommand().NormalizedThrust, 0.5, 1e-6);
    Scope.TickFrames(6);
    TestEqual(TEXT("Timeout without a target"), Lonely.Listener->TimeoutCount, 1);
    TestEqual(TEXT("No error without a target"), Lonely.Listener->Errors.Num(), 0);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotROSMotorCommandTest, "UBotROS.MotorCommand.DispatchAndTimeout", UBotROSFlightCommandTests::TestFlags)

bool FUBotROSMotorCommandTest::RunTest(const FString& Parameters)
{
    using namespace UBotROSFlightCommandTests;
    FScopedWorld Scope;
    auto Fixture = Spawn<UUBotROSMotorCommandComponent>(Scope, true, TEXT("drone_2"));
    TestEqual(TEXT("Default topic"), Fixture.Command->GetResolvedCommandTopic(), FString(TEXT("/drone_2/cmd_motors")));

    FUBotRosActuators Message;
    Message.velocity = { 800.0, 805.0, 810.0, 815.0 };
    Inject(*Fixture.Command, Message);
    if (TestEqual(TEXT("Motor command dispatched"), Fixture.Target->MotorCommands.Num(), 1))
    {
        TestEqual(TEXT("Speeds"), Fixture.Target->MotorCommands[0].Type, EUBotMotorCommandType::AngularVelocity);
        TestTrue(TEXT("Values kept"), Fixture.Target->MotorCommands[0].Values == Message.velocity);
    }

    // A count that differs from the vehicle's motors is not dispatched.
    FUBotRosActuators Three;
    Three.normalized = { 0.5, 0.5, 0.5 };
    Inject(*Fixture.Command, Three);
    TestEqual(TEXT("Wrong count not dispatched"), Fixture.Target->MotorCommands.Num(), 1);
    TestEqual(TEXT("Wrong count reported"), Fixture.Listener->Errors.Num(), 1);
    TestEqual(TEXT("But the event carries it"), Fixture.Command->GetLastMotorCommand().Values.Num(), 3);

    FUBotRosActuators Empty;
    Empty.position = { 1.0 };
    Inject(*Fixture.Command, Empty);
    TestEqual(TEXT("Neither velocity nor normalized reported"), Fixture.Listener->Errors.Num(), 2);

    Scope.TickFrames(6);
    TestEqual(TEXT("Motor command released after the timeout"), Fixture.Target->MotorReleases, 1);
    TestEqual(TEXT("Attitude interface untouched"), Fixture.Target->AttitudeReleases, 0);
    return true;
}

#endif
