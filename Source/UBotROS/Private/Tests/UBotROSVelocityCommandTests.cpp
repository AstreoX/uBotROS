#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "Components/SceneComponent.h"
#include "Engine/Engine.h"
#include "Engine/EngineBaseTypes.h"
#include "Engine/World.h"
#include "EngineUtils.h"
#include "GameFramework/Actor.h"
#include "UBotActuatorInterface.h"
#include "UBotIdentityComponent.h"
#include "UBotROSBridgeSubsystem.h"
#include "UBotROSVelocityCommandComponent.h"
#include "UBotROSVelocityCommandTestTypes.h"
#include "UObject/StrongObjectPtr.h"

namespace UBotROSVelocityCommandTests
{
    constexpr float FrameSeconds = 0.1f;
    constexpr float VectorTolerance = 1.e-6f;

    static_assert(PLATFORM_LITTLE_ENDIAN, "MakeTwistCdr copies doubles in host byte order; CDR here is little endian.");

    /**
     * geometry_msgs/msg/Twist linear (1.5, 0.5, -0.25), angular (0.1, -0.2, 0.3), serialized in WSL with
     * rclpy.serialization.serialize_message (ROS 2 Humble): encapsulation header 00 01 00 00, then six float64.
     */
    const uint8 GoldenTwistCdr[] = {
        0x00, 0x01, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x3f,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xe0, 0x3f,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xd0, 0xbf,
        0x9a, 0x99, 0x99, 0x99, 0x99, 0x99, 0xb9, 0x3f,
        0x9a, 0x99, 0x99, 0x99, 0x99, 0x99, 0xc9, 0xbf,
        0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0xd3, 0x3f,
    };

    TArray<uint8> GoldenTwist()
    {
        return TArray<uint8>(GoldenTwistCdr, static_cast<int32>(UE_ARRAY_COUNT(GoldenTwistCdr)));
    }

    /** IEEE 754 double from its bit pattern (NaN and infinity without constant-folding warnings). */
    double DoubleFromBits(uint64 Bits)
    {
        double Value = 0.0;
        FMemory::Memcpy(&Value, &Bits, sizeof(Value));
        return Value;
    }

    /** Same wire layout as GoldenTwistCdr for arbitrary ROS-frame values. */
    TArray<uint8> MakeTwistCdr(const FVector& ROSLinear, const FVector& ROSAngular)
    {
        TArray<uint8> Bytes = { 0x00, 0x01, 0x00, 0x00 };
        const double Values[6] = { ROSLinear.X, ROSLinear.Y, ROSLinear.Z, ROSAngular.X, ROSAngular.Y, ROSAngular.Z };
        for (const double Value : Values)
        {
            uint8 Raw[sizeof(double)];
            FMemory::Memcpy(Raw, &Value, sizeof(double));
            Bytes.Append(Raw, static_cast<int32>(sizeof(double)));
        }
        return Bytes;
    }

    void Inject(UUBotROSVelocityCommandComponent& Component, const TArray<uint8>& Cdr)
    {
        FUBotROSVelocityCommandTestAccess::InjectTwistPayload(Component, TConstArrayView<uint8>(Cdr));
    }

    // Transient game world, initialized for play and ticked by hand; torn down when the scope ends.
    // Same setup as the UBotSensor component tests.
    class FScopedCommandTestWorld
    {
    public:
        UE_NONCOPYABLE(FScopedCommandTestWorld);

        FScopedCommandTestWorld()
            : World(UWorld::CreateWorld(EWorldType::Game, false))
            , CachedFrameCounter(GFrameCounter)
        {
            if (World && GEngine)
            {
                // UWorld::Tick may run a conditional garbage collection.
                World->AddToRoot();
                // UWorld::Tick looks its world context up in the engine.
                GEngine->CreateNewWorldContext(EWorldType::Game).SetCurrentWorld(World);
                // Spawned actors are initialized, so EndPlay is routed when they are destroyed.
                World->InitializeActorsForPlay(FURL());
            }
        }

        ~FScopedCommandTestWorld()
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

        void Tick(float DeltaSeconds) const
        {
            World->Tick(LEVELTICK_All, DeltaSeconds);
            // A tick function runs at most once per GFrameCounter value (FTickFunction::QueueTickFunction),
            // and the engine loop is not advancing the counter while a test ticks the world by hand.
            ++GFrameCounter;
        }

        void TickFrames(int32 FrameCount) const
        {
            for (int32 Frame = 0; Frame < FrameCount; ++Frame)
            {
                Tick(FrameSeconds);
            }
        }

        AActor* SpawnActor(const TCHAR* Name = nullptr) const
        {
            if (!World)
            {
                return nullptr;
            }
            FActorSpawnParameters Parameters;
            if (Name)
            {
                Parameters.Name = FName(Name);
            }
            return World->SpawnActor<AActor>(Parameters);
        }

    private:
        UWorld* World = nullptr;
        uint64 CachedFrameCounter = 0;
    };

    struct FCommandFixture
    {
        AActor* Owner = nullptr;
        UUBotROSVelocityCommandComponent* Command = nullptr;
        UUBotROSVelocityCommandTestTarget* Target = nullptr;
        TStrongObjectPtr<UUBotROSVelocityCommandTestListener> Listener;

        bool IsSpawned() const
        {
            return Owner && Command && Listener.IsValid();
        }
    };

    /**
     * Spawns an actor with an optional recording target and a velocity command component, binds a listener and
     * begins play. The component does not subscribe (bAutoStartReceiver = false): the tests feed it bridge
     * payloads directly, so they neither need nor touch a bridge connection. BeginPlay registers the tick.
     */
    FCommandFixture SpawnFixture(const FScopedCommandTestWorld& Scope, bool bWithTarget)
    {
        FCommandFixture Fixture;
        Fixture.Owner = Scope.SpawnActor();
        if (!Fixture.Owner)
        {
            return Fixture;
        }
        if (bWithTarget)
        {
            Fixture.Target = NewObject<UUBotROSVelocityCommandTestTarget>(Fixture.Owner, TEXT("TestDrive"));
            Fixture.Target->RegisterComponent();
        }
        Fixture.Command = NewObject<UUBotROSVelocityCommandComponent>(Fixture.Owner, TEXT("RosVelocityCommand"));
        Fixture.Command->bAutoStartReceiver = false;
        Fixture.Command->RegisterComponent();

        Fixture.Listener.Reset(NewObject<UUBotROSVelocityCommandTestListener>());
        Fixture.Listener->Bind(*Fixture.Command);

        Fixture.Owner->DispatchBeginPlay();
        return Fixture;
    }
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotROSVelocityCommandConversionTest, "UBotROS.VelocityCommand.ConversionAndDispatch", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotROSVelocityCommandConversionTest::RunTest(const FString& Parameters)
{
    using namespace UBotROSVelocityCommandTests;

    TestTrue(TEXT("MakeTwistCdr reproduces the rclpy bytes"),
        MakeTwistCdr(FVector(1.5, 0.5, -0.25), FVector(0.1, -0.2, 0.3)) == GoldenTwist());

    FScopedCommandTestWorld Scope;
    if (!TestNotNull(TEXT("World"), Scope.Get()))
    {
        return false;
    }
    FCommandFixture Fixture = SpawnFixture(Scope, true);
    if (!TestTrue(TEXT("Fixture"), Fixture.IsSpawned() && Fixture.Target != nullptr))
    {
        return false;
    }
    TestTrue(TEXT("The owner's IUBotVelocityCommandable component is found automatically"), Fixture.Command->GetCommandTarget() == Fixture.Target);
    TestFalse(TEXT("No command before the first message"), Fixture.Command->IsCommandActive());

    // Advance sim time so the timestamp check cannot pass by accident with 0.
    Scope.TickFrames(3);
    const double ReceiveTime = Scope.Get()->GetTimeSeconds();
    TestTrue(TEXT("Sim time advanced"), ReceiveTime > 0.0);

    Inject(*Fixture.Command, GoldenTwist());

    if (TestEqual(TEXT("One command dispatched"), Fixture.Target->AppliedCommands.Num(), 1))
    {
        const FUBotVelocityCommand& Applied = Fixture.Target->AppliedCommands[0];
        // ROS (x fwd, y left, z up; CCW positive) -> UE (x fwd, y right, z up; yaw right positive).
        TestEqual(TEXT("Linear: (x, -y, z)"), Applied.LinearVelocityMetersPerSecond, FVector(1.5, -0.5, -0.25), VectorTolerance);
        TestEqual(TEXT("Angular: (-x, y, -z)"), Applied.AngularVelocityRadiansPerSecond, FVector(-0.1, -0.2, -0.3), VectorTolerance);
        TestEqual(TEXT("Stamped with the sim time of reception"), Applied.TimestampSeconds, ReceiveTime, 1.e-9);
        TestTrue(TEXT("Source is ROS"), Applied.Source == FName(TEXT("ROS")));
    }
    if (TestEqual(TEXT("OnVelocityCommand fired once"), Fixture.Listener->Commands.Num(), 1))
    {
        TestEqual(TEXT("Event carries the dispatched command"), Fixture.Listener->Commands[0].LinearVelocityMetersPerSecond, FVector(1.5, -0.5, -0.25), VectorTolerance);
    }
    TestEqual(TEXT("GetLastCommand linear"), Fixture.Command->GetLastCommand().LinearVelocityMetersPerSecond, FVector(1.5, -0.5, -0.25), VectorTolerance);
    TestEqual(TEXT("GetLastCommand angular"), Fixture.Command->GetLastCommand().AngularVelocityRadiansPerSecond, FVector(-0.1, -0.2, -0.3), VectorTolerance);
    TestTrue(TEXT("Command active"), Fixture.Command->IsCommandActive());
    TestEqual(TEXT("No errors"), Fixture.Listener->Errors.Num(), 0);

    // A pure yaw command: ROS +z (counter-clockwise) is a UE turn to the left, i.e. negative UE Z.
    Inject(*Fixture.Command, MakeTwistCdr(FVector(2.0, 0.0, 0.0), FVector(0.0, 0.0, 1.0)));
    if (TestEqual(TEXT("Second command dispatched"), Fixture.Target->AppliedCommands.Num(), 2))
    {
        TestEqual(TEXT("Forward stays forward"), Fixture.Target->AppliedCommands[1].LinearVelocityMetersPerSecond, FVector(2.0, 0.0, 0.0), VectorTolerance);
        TestEqual(TEXT("CCW yaw -> negative UE yaw rate"), Fixture.Target->AppliedCommands[1].AngularVelocityRadiansPerSecond, FVector(0.0, 0.0, -1.0), VectorTolerance);
    }
    TestEqual(TEXT("Still no StopMotion"), Fixture.Target->StopMotionCount, 0);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotROSVelocityCommandTargetSelectionTest, "UBotROS.VelocityCommand.TargetSelection", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotROSVelocityCommandTargetSelectionTest::RunTest(const FString& Parameters)
{
    using namespace UBotROSVelocityCommandTests;

    FScopedCommandTestWorld Scope;
    if (!TestNotNull(TEXT("World"), Scope.Get()))
    {
        return false;
    }
    FCommandFixture Fixture = SpawnFixture(Scope, true);
    if (!TestTrue(TEXT("Fixture"), Fixture.IsSpawned() && Fixture.Target != nullptr))
    {
        return false;
    }

    // An explicit target wins over the automatic lookup (which would return the first implementer, TestDrive).
    UUBotROSVelocityCommandTestTarget* Second = NewObject<UUBotROSVelocityCommandTestTarget>(Fixture.Owner, TEXT("SecondDrive"));
    Second->RegisterComponent();
    Fixture.Command->SetTargetComponent(Second);
    TestTrue(TEXT("Explicit target resolved"), Fixture.Command->GetCommandTarget() == Second);
    Inject(*Fixture.Command, GoldenTwist());
    TestEqual(TEXT("Explicit target receives the command"), Second->AppliedCommands.Num(), 1);
    TestEqual(TEXT("Automatic target is bypassed"), Fixture.Target->AppliedCommands.Num(), 0);

    // An explicit target without the interface is a configuration error: reported once, events keep firing.
    USceneComponent* NotCommandable = NewObject<USceneComponent>(Fixture.Owner, TEXT("NotCommandable"));
    NotCommandable->RegisterComponent();
    Fixture.Command->SetTargetComponent(NotCommandable);
    TestNull(TEXT("Unusable explicit target is not returned"), Fixture.Command->GetCommandTarget());
    Inject(*Fixture.Command, GoldenTwist());
    Inject(*Fixture.Command, GoldenTwist());
    TestEqual(TEXT("Unusable target reported once"), Fixture.Listener->Errors.Num(), 1);
    if (Fixture.Listener->Errors.Num() > 0)
    {
        TestTrue(TEXT("Error names the missing interface"), Fixture.Listener->Errors[0].Contains(TEXT("IUBotVelocityCommandable")));
    }
    TestEqual(TEXT("No fallback dispatch to the automatic target"), Fixture.Target->AppliedCommands.Num(), 0);
    TestEqual(TEXT("No dispatch to the previous target"), Second->AppliedCommands.Num(), 1);
    TestEqual(TEXT("OnVelocityCommand still fires"), Fixture.Listener->Commands.Num(), 3);

    // nullptr restores the automatic lookup.
    Fixture.Command->SetTargetComponent(nullptr);
    TestTrue(TEXT("Automatic lookup restored"), Fixture.Command->GetCommandTarget() == Fixture.Target);

    // A rejecting target is reported once until it accepts again; the command still counts as received.
    Fixture.Target->bAcceptCommands = false;
    Inject(*Fixture.Command, GoldenTwist());
    Inject(*Fixture.Command, GoldenTwist());
    TestEqual(TEXT("Rejected commands were offered"), Fixture.Target->AppliedCommands.Num(), 2);
    TestEqual(TEXT("Rejection reported once"), Fixture.Listener->Errors.Num(), 2);
    if (Fixture.Listener->Errors.Num() > 1)
    {
        TestTrue(TEXT("Error mentions the rejection"), Fixture.Listener->Errors[1].Contains(TEXT("rejected")));
    }
    TestEqual(TEXT("Rejected commands are still broadcast"), Fixture.Listener->Commands.Num(), 5);
    TestTrue(TEXT("Rejected command still arms the timeout"), Fixture.Command->IsCommandActive());

    Fixture.Target->bAcceptCommands = true;
    Inject(*Fixture.Command, GoldenTwist());
    Fixture.Target->bAcceptCommands = false;
    Inject(*Fixture.Command, GoldenTwist());
    TestEqual(TEXT("A rejection after an accepted command is reported again"), Fixture.Listener->Errors.Num(), 3);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotROSVelocityCommandClampTest, "UBotROS.VelocityCommand.Clamp", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotROSVelocityCommandClampTest::RunTest(const FString& Parameters)
{
    using namespace UBotROSVelocityCommandTests;

    FScopedCommandTestWorld Scope;
    if (!TestNotNull(TEXT("World"), Scope.Get()))
    {
        return false;
    }
    FCommandFixture Fixture = SpawnFixture(Scope, true);
    if (!TestTrue(TEXT("Fixture"), Fixture.IsSpawned() && Fixture.Target != nullptr))
    {
        return false;
    }
    TArray<FUBotVelocityCommand>& Applied = Fixture.Target->AppliedCommands;

    // Defaults are unlimited.
    TestEqual(TEXT("Default linear limit"), Fixture.Command->MaxLinearSpeedMetersPerSecond, 0.0f);
    TestEqual(TEXT("Default angular limit"), Fixture.Command->MaxAngularSpeedRadiansPerSecond, 0.0f);
    Inject(*Fixture.Command, MakeTwistCdr(FVector(3.0, 4.0, 0.0), FVector(0.0, 0.0, 2.0)));
    if (TestEqual(TEXT("Unlimited command dispatched"), Applied.Num(), 1))
    {
        TestEqual(TEXT("0 = unlimited (linear)"), Applied[0].LinearVelocityMetersPerSecond, FVector(3.0, -4.0, 0.0), VectorTolerance);
        TestEqual(TEXT("0 = unlimited (angular)"), Applied[0].AngularVelocityRadiansPerSecond, FVector(0.0, 0.0, -2.0), VectorTolerance);
    }

    // Norm clamp keeps the direction: |(3, -4, 0)| = 5 -> 1 m/s gives (0.6, -0.8, 0).
    Fixture.Command->MaxLinearSpeedMetersPerSecond = 1.0f;
    Fixture.Command->MaxAngularSpeedRadiansPerSecond = 0.5f;
    Inject(*Fixture.Command, MakeTwistCdr(FVector(3.0, 4.0, 0.0), FVector(0.0, 0.0, 2.0)));
    if (TestEqual(TEXT("Clamped command dispatched"), Applied.Num(), 2))
    {
        TestEqual(TEXT("Linear clamped along its direction"), Applied[1].LinearVelocityMetersPerSecond, FVector(0.6, -0.8, 0.0), VectorTolerance);
        TestEqual(TEXT("Angular clamped along its axis"), Applied[1].AngularVelocityRadiansPerSecond, FVector(0.0, 0.0, -0.5), VectorTolerance);
    }
    if (TestEqual(TEXT("Event fired for the clamped command"), Fixture.Listener->Commands.Num(), 2))
    {
        TestEqual(TEXT("Event carries the clamped linear velocity"), Fixture.Listener->Commands[1].LinearVelocityMetersPerSecond, FVector(0.6, -0.8, 0.0), VectorTolerance);
    }
    TestEqual(TEXT("GetLastCommand is clamped"), Fixture.Command->GetLastCommand().AngularVelocityRadiansPerSecond, FVector(0.0, 0.0, -0.5), VectorTolerance);

    // Commands within the limits pass unchanged, including the reverse direction.
    Inject(*Fixture.Command, MakeTwistCdr(FVector(-0.3, 0.2, 0.1), FVector(0.1, 0.0, -0.2)));
    if (TestEqual(TEXT("Small command dispatched"), Applied.Num(), 3))
    {
        TestEqual(TEXT("Linear within the limit unchanged"), Applied[2].LinearVelocityMetersPerSecond, FVector(-0.3, -0.2, 0.1), VectorTolerance);
        TestEqual(TEXT("Angular within the limit unchanged"), Applied[2].AngularVelocityRadiansPerSecond, FVector(-0.1, 0.0, 0.2), VectorTolerance);
    }

    // Each limit applies to its own vector only.
    Fixture.Command->MaxLinearSpeedMetersPerSecond = 0.0f;
    Inject(*Fixture.Command, MakeTwistCdr(FVector(3.0, 4.0, 0.0), FVector(0.0, 0.0, 2.0)));
    if (TestEqual(TEXT("Mixed-limit command dispatched"), Applied.Num(), 4))
    {
        TestEqual(TEXT("Linear unlimited again"), Applied[3].LinearVelocityMetersPerSecond, FVector(3.0, -4.0, 0.0), VectorTolerance);
        TestEqual(TEXT("Angular still clamped"), Applied[3].AngularVelocityRadiansPerSecond, FVector(0.0, 0.0, -0.5), VectorTolerance);
    }
    TestEqual(TEXT("No errors"), Fixture.Listener->Errors.Num(), 0);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotROSVelocityCommandTimeoutTest, "UBotROS.VelocityCommand.Timeout", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotROSVelocityCommandTimeoutTest::RunTest(const FString& Parameters)
{
    using namespace UBotROSVelocityCommandTests;

    FScopedCommandTestWorld Scope;
    if (!TestNotNull(TEXT("World"), Scope.Get()))
    {
        return false;
    }
    FCommandFixture Fixture = SpawnFixture(Scope, true);
    if (!TestTrue(TEXT("Fixture"), Fixture.IsSpawned() && Fixture.Target != nullptr))
    {
        return false;
    }
    UUBotROSVelocityCommandComponent& Command = *Fixture.Command;
    UUBotROSVelocityCommandTestTarget& Target = *Fixture.Target;
    TestEqual(TEXT("Default timeout"), Command.CommandTimeoutSeconds, 0.5f);

    // Nothing to stop before the first command; the timeout tick is off.
    TestFalse(TEXT("Tick disabled while idle"), Command.IsComponentTickEnabled());
    Scope.TickFrames(10);
    TestEqual(TEXT("No StopMotion without a command"), Target.StopMotionCount, 0);
    TestEqual(TEXT("No timeout without a command"), Fixture.Listener->TimeoutCount, 0);

    // t = 1.0: command. 0.3 s later it is still active.
    Inject(Command, GoldenTwist());
    TestTrue(TEXT("Tick enabled by a command"), Command.IsComponentTickEnabled());
    Scope.TickFrames(3);
    TestEqual(TEXT("No StopMotion before the timeout"), Target.StopMotionCount, 0);
    TestTrue(TEXT("Still active before the timeout"), Command.IsCommandActive());

    // A new command restarts the timeout: 0.6 s after the first, 0.3 s after the second.
    Inject(Command, GoldenTwist());
    Scope.TickFrames(3);
    TestEqual(TEXT("A fresh command keeps the target moving"), Target.StopMotionCount, 0);
    TestEqual(TEXT("No timeout event while commands arrive"), Fixture.Listener->TimeoutCount, 0);

    // 0.6 s after the second command: stopped exactly once.
    Scope.TickFrames(3);
    TestEqual(TEXT("StopMotion once at the timeout"), Target.StopMotionCount, 1);
    TestEqual(TEXT("OnCommandTimeout once"), Fixture.Listener->TimeoutCount, 1);
    TestFalse(TEXT("Inactive after the timeout"), Command.IsCommandActive());
    TestFalse(TEXT("Tick disabled after the timeout"), Command.IsComponentTickEnabled());

    Scope.TickFrames(10);
    TestEqual(TEXT("StopMotion is not repeated"), Target.StopMotionCount, 1);
    TestEqual(TEXT("OnCommandTimeout is not repeated"), Fixture.Listener->TimeoutCount, 1);

    // Every timeout stops once.
    Inject(Command, GoldenTwist());
    Scope.TickFrames(7);
    TestEqual(TEXT("Second timeout stops once more"), Target.StopMotionCount, 2);
    TestEqual(TEXT("Second timeout event"), Fixture.Listener->TimeoutCount, 2);

    // The timeout follows sim time, not the number of ticks: one long frame is enough.
    Inject(Command, GoldenTwist());
    Scope.Tick(0.3f);
    TestEqual(TEXT("0.3 s frame: no timeout"), Target.StopMotionCount, 2);
    Scope.Tick(0.3f);
    TestEqual(TEXT("0.6 s elapsed: stopped"), Target.StopMotionCount, 3);

    // CommandTimeoutSeconds = 0 disables the timeout.
    Command.CommandTimeoutSeconds = 0.0f;
    Inject(Command, GoldenTwist());
    Scope.TickFrames(10);
    TestEqual(TEXT("Disabled timeout never stops"), Target.StopMotionCount, 3);
    TestTrue(TEXT("Command stays active without a timeout"), Command.IsCommandActive());
    TestEqual(TEXT("No errors"), Fixture.Listener->Errors.Num(), 0);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotROSVelocityCommandMalformedTest, "UBotROS.VelocityCommand.MalformedPayload", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotROSVelocityCommandMalformedTest::RunTest(const FString& Parameters)
{
    using namespace UBotROSVelocityCommandTests;

    FScopedCommandTestWorld Scope;
    if (!TestNotNull(TEXT("World"), Scope.Get()))
    {
        return false;
    }
    FCommandFixture Fixture = SpawnFixture(Scope, true);
    if (!TestTrue(TEXT("Fixture"), Fixture.IsSpawned() && Fixture.Target != nullptr))
    {
        return false;
    }

    TArray<uint8> Truncated = GoldenTwist();
    Truncated.Pop();
    const TArray<uint8> HeaderOnly = { 0x00, 0x01, 0x00, 0x00 };
    const TArray<uint8> Empty;

    Inject(*Fixture.Command, Empty);
    Inject(*Fixture.Command, HeaderOnly);
    Inject(*Fixture.Command, Truncated);
    TestEqual(TEXT("Every malformed payload fires OnCommandError"), Fixture.Listener->Errors.Num(), 3);
    for (const FString& Error : Fixture.Listener->Errors)
    {
        TestTrue(FString::Printf(TEXT("Error describes the drop: %s"), *Error), Error.Contains(TEXT("malformed")));
    }

    const double QuietNaN = DoubleFromBits(0x7FF8000000000000ull);
    const double PositiveInfinity = DoubleFromBits(0x7FF0000000000000ull);
    Inject(*Fixture.Command, MakeTwistCdr(FVector(QuietNaN, 0.0, 0.0), FVector::ZeroVector));
    Inject(*Fixture.Command, MakeTwistCdr(FVector::ZeroVector, FVector(0.0, 0.0, PositiveInfinity)));
    TestEqual(TEXT("Non-finite values fire OnCommandError"), Fixture.Listener->Errors.Num(), 5);
    if (Fixture.Listener->Errors.Num() == 5)
    {
        TestTrue(TEXT("NaN reported as non-finite"), Fixture.Listener->Errors[3].Contains(TEXT("non-finite")));
        TestTrue(TEXT("Inf reported as non-finite"), Fixture.Listener->Errors[4].Contains(TEXT("non-finite")));
    }

    TestEqual(TEXT("Nothing dispatched"), Fixture.Target->AppliedCommands.Num(), 0);
    TestEqual(TEXT("No OnVelocityCommand"), Fixture.Listener->Commands.Num(), 0);
    TestFalse(TEXT("No active command"), Fixture.Command->IsCommandActive());
    TestTrue(TEXT("GetCommandError returns the last error"), Fixture.Command->GetCommandError().Contains(TEXT("non-finite")));

    Scope.TickFrames(10);
    TestEqual(TEXT("Malformed payloads do not arm the timeout"), Fixture.Target->StopMotionCount, 0);
    TestEqual(TEXT("No timeout event"), Fixture.Listener->TimeoutCount, 0);

    // The receiver keeps working after bad input.
    Inject(*Fixture.Command, GoldenTwist());
    TestEqual(TEXT("Valid payload after errors is dispatched"), Fixture.Target->AppliedCommands.Num(), 1);
    TestEqual(TEXT("No further errors"), Fixture.Listener->Errors.Num(), 5);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotROSVelocityCommandNoTargetTest, "UBotROS.VelocityCommand.NoTarget", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotROSVelocityCommandNoTargetTest::RunTest(const FString& Parameters)
{
    using namespace UBotROSVelocityCommandTests;

    FScopedCommandTestWorld Scope;
    if (!TestNotNull(TEXT("World"), Scope.Get()))
    {
        return false;
    }
    FCommandFixture Fixture = SpawnFixture(Scope, false);
    if (!TestTrue(TEXT("Fixture"), Fixture.IsSpawned()))
    {
        return false;
    }
    TestNull(TEXT("No target on the owner"), Fixture.Command->GetCommandTarget());

    Inject(*Fixture.Command, GoldenTwist());
    if (TestEqual(TEXT("OnVelocityCommand fires without a target"), Fixture.Listener->Commands.Num(), 1))
    {
        TestEqual(TEXT("Converted linear"), Fixture.Listener->Commands[0].LinearVelocityMetersPerSecond, FVector(1.5, -0.5, -0.25), VectorTolerance);
        TestEqual(TEXT("Converted angular"), Fixture.Listener->Commands[0].AngularVelocityRadiansPerSecond, FVector(-0.1, -0.2, -0.3), VectorTolerance);
    }
    TestTrue(TEXT("Command active"), Fixture.Command->IsCommandActive());
    TestEqual(TEXT("A missing target is not an error"), Fixture.Listener->Errors.Num(), 0);

    Scope.TickFrames(7);
    TestEqual(TEXT("OnCommandTimeout fires without a target"), Fixture.Listener->TimeoutCount, 1);
    TestFalse(TEXT("Inactive after the timeout"), Fixture.Command->IsCommandActive());
    Scope.TickFrames(10);
    TestEqual(TEXT("Timeout fires once"), Fixture.Listener->TimeoutCount, 1);

    // A target added later is picked up by the next command.
    UUBotROSVelocityCommandTestTarget* LateTarget = NewObject<UUBotROSVelocityCommandTestTarget>(Fixture.Owner, TEXT("LateDrive"));
    LateTarget->RegisterComponent();
    Inject(*Fixture.Command, GoldenTwist());
    TestEqual(TEXT("Late target receives the next command"), LateTarget->AppliedCommands.Num(), 1);
    Scope.TickFrames(7);
    TestEqual(TEXT("Late target stopped once"), LateTarget->StopMotionCount, 1);
    TestEqual(TEXT("No errors"), Fixture.Listener->Errors.Num(), 0);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotROSVelocityCommandEndPlayTest, "UBotROS.VelocityCommand.EndPlayStopsMotion", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotROSVelocityCommandEndPlayTest::RunTest(const FString& Parameters)
{
    using namespace UBotROSVelocityCommandTests;

    FScopedCommandTestWorld Scope;
    if (!TestNotNull(TEXT("World"), Scope.Get()))
    {
        return false;
    }
    FCommandFixture Fixture = SpawnFixture(Scope, true);
    if (!TestTrue(TEXT("Fixture"), Fixture.IsSpawned() && Fixture.Target != nullptr))
    {
        return false;
    }

    // EndPlay without an active command does not touch the target.
    FCommandFixture Idle = SpawnFixture(Scope, true);
    if (TestTrue(TEXT("Idle fixture"), Idle.IsSpawned() && Idle.Target != nullptr))
    {
        Idle.Owner->RouteEndPlay(EEndPlayReason::Destroyed);
        TestEqual(TEXT("Idle EndPlay sends no StopMotion"), Idle.Target->StopMotionCount, 0);
    }

    // EndPlay with an active command stops the target once; it is not a timeout.
    Inject(*Fixture.Command, GoldenTwist());
    Fixture.Owner->RouteEndPlay(EEndPlayReason::Destroyed);
    TestEqual(TEXT("EndPlay stops an active command"), Fixture.Target->StopMotionCount, 1);
    TestEqual(TEXT("EndPlay is not reported as a timeout"), Fixture.Listener->TimeoutCount, 0);
    TestFalse(TEXT("Inactive after EndPlay"), Fixture.Command->IsCommandActive());

    Scope.TickFrames(10);
    TestEqual(TEXT("No StopMotion after EndPlay"), Fixture.Target->StopMotionCount, 1);
    TestEqual(TEXT("No timeout after EndPlay"), Fixture.Listener->TimeoutCount, 0);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotROSVelocityCommandTopicTest, "UBotROS.VelocityCommand.TopicResolution", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotROSVelocityCommandTopicTest::RunTest(const FString& Parameters)
{
    using namespace UBotROSVelocityCommandTests;

    FScopedCommandTestWorld Scope;
    if (!TestNotNull(TEXT("World"), Scope.Get()))
    {
        return false;
    }

    // No identity component: the sanitized actor name.
    AActor* Robot = Scope.SpawnActor(TEXT("UBotTest-Robot7"));
    if (!TestNotNull(TEXT("Robot"), Robot))
    {
        return false;
    }
    UUBotROSVelocityCommandComponent* Command = NewObject<UUBotROSVelocityCommandComponent>(Robot, TEXT("RosVelocityCommand"));
    TestEqual(TEXT("Namespace from the actor name"), Command->GetResolvedCommandTopic(), FString(TEXT("/UBotTest_Robot7/cmd_vel")));

    // Identity component namespace (leading/trailing '/' ignored, multi-level kept).
    UUBotIdentityComponent* Identity = NewObject<UUBotIdentityComponent>(Robot, TEXT("Identity"));
    Identity->Namespace = TEXT("/fleet/robot_7/");
    TestEqual(TEXT("Namespace from UBot::Identity"), Command->GetResolvedCommandTopic(), FString(TEXT("/fleet/robot_7/cmd_vel")));

    // RobotNamespaceOverride wins over the identity and is sanitized per segment.
    Command->RobotNamespaceOverride = TEXT(" /my robot/ ");
    TestEqual(TEXT("Namespace override sanitized"), Command->GetResolvedCommandTopic(), FString(TEXT("/my_robot/cmd_vel")));
    Command->RobotNamespaceOverride = TEXT("fleet/7");
    TestEqual(TEXT("Digit-first segment prefixed"), Command->GetResolvedCommandTopic(), FString(TEXT("/fleet/_7/cmd_vel")));
    Command->RobotNamespaceOverride = TEXT("/");
    TestEqual(TEXT("'/' selects the root namespace"), Command->GetResolvedCommandTopic(), FString(TEXT("/cmd_vel")));
    Command->RobotNamespaceOverride = TEXT("   ");
    TestEqual(TEXT("Whitespace-only override is unset"), Command->GetResolvedCommandTopic(), FString(TEXT("/fleet/robot_7/cmd_vel")));

    // CommandTopicOverride replaces the whole topic; a missing leading '/' is added (legacy behaviour).
    Command->CommandTopicOverride = TEXT(" teleop/cmd_vel ");
    TestEqual(TEXT("Relative override made absolute"), Command->GetResolvedCommandTopic(), FString(TEXT("/teleop/cmd_vel")));
    Command->CommandTopicOverride = TEXT("/abs/cmd");
    TestEqual(TEXT("Absolute override kept"), Command->GetResolvedCommandTopic(), FString(TEXT("/abs/cmd")));

    // Invalid topics are rejected before the bridge is contacted.
    TStrongObjectPtr<UUBotROSVelocityCommandTestListener> Listener(NewObject<UUBotROSVelocityCommandTestListener>());
    Listener->Bind(*Command);
    const TCHAR* const InvalidTopics[] = { TEXT("/bad//topic"), TEXT("/fleet/7robot/cmd_vel"), TEXT("/with space/cmd_vel"), TEXT("/trailing/") };
    constexpr int32 InvalidTopicCount = static_cast<int32>(UE_ARRAY_COUNT(InvalidTopics));
    AddExpectedErrorPlain(TEXT("is not a valid absolute ROS 2 topic"), EAutomationExpectedErrorFlags::Contains, InvalidTopicCount);
    for (const TCHAR* const InvalidTopic : InvalidTopics)
    {
        Command->CommandTopicOverride = InvalidTopic;
        TestFalse(FString::Printf(TEXT("'%s' is rejected"), InvalidTopic), Command->RestartCommandReceiver());
        TestFalse(FString::Printf(TEXT("'%s': receiver not ready"), InvalidTopic), Command->IsCommandReceiverReady());
    }
    TestEqual(TEXT("Each rejection fires OnCommandError"), Listener->Errors.Num(), InvalidTopicCount);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotROSVelocityCommandReceiverTest, "UBotROS.VelocityCommand.Receiver", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotROSVelocityCommandReceiverTest::RunTest(const FString& Parameters)
{
    using namespace UBotROSVelocityCommandTests;

    // The test world never begins play, so its bridge subsystem registers subscriptions without connecting.
    FScopedCommandTestWorld Scope;
    if (!TestNotNull(TEXT("World"), Scope.Get()))
    {
        return false;
    }
    UUBotROSBridgeSubsystem* Bridge = UUBotROSBridgeSubsystem::Get(Scope.Get());
    if (!TestNotNull(TEXT("Bridge subsystem of a game world"), Bridge))
    {
        return false;
    }
    FCommandFixture Fixture = SpawnFixture(Scope, true);
    if (!TestTrue(TEXT("Fixture"), Fixture.IsSpawned()))
    {
        return false;
    }
    UUBotROSVelocityCommandComponent& Command = *Fixture.Command;
    TestFalse(TEXT("Not subscribed with bAutoStartReceiver = false"), Command.IsCommandReceiverReady());
    TestEqual(TEXT("No subscription id"), FUBotROSVelocityCommandTestAccess::GetSubscriptionId(Command), 0);

    Command.RobotNamespaceOverride = TEXT("rx_robot");
    TestTrue(TEXT("RestartCommandReceiver subscribes"), Command.RestartCommandReceiver());
    const int32 FirstId = FUBotROSVelocityCommandTestAccess::GetSubscriptionId(Command);
    TestTrue(TEXT("Subscription id assigned"), FirstId > 0);
    TestTrue(TEXT("Bridge knows the subscription"), Bridge->IsSubscribed(FirstId));
    TestTrue(TEXT("Receiver ready while the bridge is not connected"), Command.IsCommandReceiverReady());
    TestEqual(TEXT("Subscribed topic"), Command.GetResolvedCommandTopic(), FString(TEXT("/rx_robot/cmd_vel")));

    // While running, the resolved topic is the subscribed one; a restart applies changed settings.
    Command.RobotNamespaceOverride = TEXT("other_robot");
    TestEqual(TEXT("Running receiver reports its subscribed topic"), Command.GetResolvedCommandTopic(), FString(TEXT("/rx_robot/cmd_vel")));
    TestTrue(TEXT("Restart with new settings"), Command.RestartCommandReceiver());
    const int32 SecondId = FUBotROSVelocityCommandTestAccess::GetSubscriptionId(Command);
    TestFalse(TEXT("Restart removes the old subscription"), Bridge->IsSubscribed(FirstId));
    TestTrue(TEXT("Restart creates a new subscription"), SecondId > 0 && SecondId != FirstId && Bridge->IsSubscribed(SecondId));
    TestEqual(TEXT("Restarted topic"), Command.GetResolvedCommandTopic(), FString(TEXT("/other_robot/cmd_vel")));

    // Stopping the receiver does not cancel an active command: it still times out once.
    Inject(Command, GoldenTwist());
    Command.StopCommandReceiver();
    TestFalse(TEXT("Stopped receiver is not ready"), Command.IsCommandReceiverReady());
    TestFalse(TEXT("Stop removes the subscription"), Bridge->IsSubscribed(SecondId));
    Scope.TickFrames(7);
    TestEqual(TEXT("Active command still times out after stop"), Fixture.Target->StopMotionCount, 1);
    TestEqual(TEXT("No errors"), Fixture.Listener->Errors.Num(), 0);

    // bAutoStartReceiver (default true) subscribes in BeginPlay; EndPlay unsubscribes.
    AActor* AutoOwner = Scope.SpawnActor();
    if (!TestNotNull(TEXT("Auto owner"), AutoOwner))
    {
        return false;
    }
    UUBotROSVelocityCommandComponent* AutoCommand = NewObject<UUBotROSVelocityCommandComponent>(AutoOwner, TEXT("RosVelocityCommand"));
    TestTrue(TEXT("Auto start is the default"), AutoCommand->bAutoStartReceiver);
    AutoCommand->RegisterComponent();
    AutoOwner->DispatchBeginPlay();
    const int32 AutoId = FUBotROSVelocityCommandTestAccess::GetSubscriptionId(*AutoCommand);
    TestTrue(TEXT("BeginPlay subscribes"), AutoCommand->IsCommandReceiverReady() && Bridge->IsSubscribed(AutoId));
    AutoOwner->RouteEndPlay(EEndPlayReason::Destroyed);
    TestFalse(TEXT("EndPlay unsubscribes"), Bridge->IsSubscribed(AutoId));
    TestFalse(TEXT("Not ready after EndPlay"), AutoCommand->IsCommandReceiverReady());
    return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
