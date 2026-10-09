#include "UBotROSVelocityCommandComponent.h"

#include "Components/ActorComponent.h"
#include "GameFramework/Actor.h"
#include "UBotClock.h"
#include "UBotIdentityComponent.h"
#include "UBotROSBridgeSubsystem.h"
#include "UBotROSConverters.h"
#include "UBotROSMessageTypes.h"
#include "UBotROSSensorPublisher.h"
#include "UBotROSTypes.h"

DEFINE_LOG_CATEGORY_STATIC(LogUBotROSVelocityCommand, Log, All);

namespace UBot::ROSVelocityCommand::Private
{
    const TCHAR* const CommandTopicLeaf = TEXT("cmd_vel");
    constexpr int32 CommandQueueDepth = 10;

    bool ImplementsVelocityCommandable(const UObject* Object)
    {
        return Object != nullptr && Object->GetClass()->ImplementsInterface(UUBotVelocityCommandable::StaticClass());
    }

    /** Norm clamp that keeps the direction. MaxNorm <= 0 means unlimited. */
    FVector ClampNorm(const FVector& Value, float MaxNorm)
    {
        return MaxNorm > 0.0f ? Value.GetClampedToMaxSize(MaxNorm) : Value;
    }
}

UUBotROSVelocityCommandComponent::UUBotROSVelocityCommandComponent()
{
    // The tick only checks the command timeout; it is enabled while a command is active.
    PrimaryComponentTick.bCanEverTick = true;
    PrimaryComponentTick.bStartWithTickEnabled = false;
}

void UUBotROSVelocityCommandComponent::BeginPlay()
{
    Super::BeginPlay();
    if (bAutoStartReceiver)
    {
        StartReceiver();
    }
}

void UUBotROSVelocityCommandComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    StopReceiver();

    // The timeout tick stops with the component, so stop the target now instead of leaving it moving.
    if (bCommandActive)
    {
        bCommandActive = false;
        SetComponentTickEnabled(false);
        if (UObject* Target = ResolveCommandTarget(nullptr); IsValid(Target))
        {
            IUBotVelocityCommandable::Execute_StopMotion(Target);
        }
    }

    Super::EndPlay(EndPlayReason);
}

void UUBotROSVelocityCommandComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
    Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
    CheckCommandTimeout();
}

bool UUBotROSVelocityCommandComponent::IsCommandReceiverReady() const
{
    const UUBotROSBridgeSubsystem* Bridge = BridgeSubsystem.Get();
    return SubscriptionId > 0 && Bridge != nullptr && Bridge->IsSubscribed(SubscriptionId);
}

FString UUBotROSVelocityCommandComponent::GetResolvedCommandTopic() const
{
    return SubscriptionId > 0 ? ActiveCommandTopic : ResolveCommandTopic();
}

UObject* UUBotROSVelocityCommandComponent::GetCommandTarget() const
{
    return ResolveCommandTarget(nullptr);
}

void UUBotROSVelocityCommandComponent::SetTargetComponent(UActorComponent* NewTarget)
{
    TargetComponent = FComponentReference();
    TargetComponent.OverrideComponent = NewTarget;
    bTargetErrorReported = false;
    bRejectionReported = false;
}

bool UUBotROSVelocityCommandComponent::RestartCommandReceiver()
{
    StopReceiver();
    return StartReceiver();
}

void UUBotROSVelocityCommandComponent::StopCommandReceiver()
{
    StopReceiver();
}

bool UUBotROSVelocityCommandComponent::StartReceiver()
{
    namespace Local = UBot::ROSVelocityCommand::Private;

    LastError.Reset();
    LastLoggedError.Reset();
    bTargetErrorReported = false;
    bRejectionReported = false;

    // Validate before touching the bridge, like the legacy command components.
    const FString Topic = ResolveCommandTopic();
    FString Reason;
    if (!UBot::ROSPublishing::IsValidAbsoluteTopic(Topic, &Reason))
    {
        ReportError(FString::Printf(TEXT("Velocity command topic '%s' is not a valid absolute ROS 2 topic: %s."), *Topic, *Reason), true);
        return false;
    }

    UUBotROSBridgeSubsystem* Bridge = UUBotROSBridgeSubsystem::Get(this);
    if (Bridge == nullptr)
    {
        ReportError(FString::Printf(TEXT("No uBot ROS bridge subsystem for '%s' (game and PIE worlds only); cannot subscribe to %s."), *GetPathNameSafe(this), *Topic), true);
        return false;
    }

    FUBotROSQos Qos;
    Qos.Reliability = EUBotROSReliability::BestEffort;
    Qos.Durability = EUBotROSDurability::Volatile;
    Qos.Depth = Local::CommandQueueDepth;

    TWeakObjectPtr<UUBotROSVelocityCommandComponent> WeakThis(this);
    const int32 NewSubscriptionId = Bridge->Subscribe(Topic, FUBotRosTwist::TypeName(), Qos,
        [WeakThis](TConstArrayView<uint8> Cdr)
        {
            if (UUBotROSVelocityCommandComponent* Self = WeakThis.Get())
            {
                Self->HandleTwistPayload(Cdr);
            }
        });
    if (NewSubscriptionId <= 0)
    {
        ReportError(FString::Printf(TEXT("The uBot ROS bridge refused the subscription to %s (%s)."), *Topic, FUBotRosTwist::TypeName()), true);
        return false;
    }

    BridgeSubsystem = Bridge;
    SubscriptionId = NewSubscriptionId;
    ActiveCommandTopic = Topic;

    FString TargetError;
    const UObject* Target = ResolveCommandTarget(&TargetError);
    if (!TargetError.IsEmpty())
    {
        // Not fatal: the receiver runs and broadcasts; the target may still appear later.
        bTargetErrorReported = true;
        ReportError(TargetError, false);
    }
    UE_LOG(LogUBotROSVelocityCommand, Log, TEXT("%s: listening for velocity commands on %s; target %s."),
        *GetPathNameSafe(this), *Topic, Target ? *GetPathNameSafe(Target) : TEXT("none (events only)"));
    return true;
}

void UUBotROSVelocityCommandComponent::StopReceiver()
{
    if (SubscriptionId > 0)
    {
        if (UUBotROSBridgeSubsystem* Bridge = BridgeSubsystem.Get())
        {
            Bridge->Unsubscribe(SubscriptionId);
        }
    }
    SubscriptionId = 0;
    BridgeSubsystem.Reset();
    ActiveCommandTopic.Reset();
}

FString UUBotROSVelocityCommandComponent::ResolveCommandTopic() const
{
    namespace Local = UBot::ROSVelocityCommand::Private;

    const FString TopicOverride = CommandTopicOverride.TrimStartAndEnd();
    if (!TopicOverride.IsEmpty())
    {
        return TopicOverride.StartsWith(TEXT("/"), ESearchCase::CaseSensitive) ? TopicOverride : TEXT("/") + TopicOverride;
    }

    // Same namespace rules as the sensor bridge: segments sanitized, '/' only -> root namespace.
    const FString NamespaceOverride = RobotNamespaceOverride.TrimStartAndEnd();
    const FString Namespace = NamespaceOverride.IsEmpty() ? UBot::Identity::ResolveNamespace(GetOwner()) : NamespaceOverride;
    return UBot::ROSPublishing::JoinTopic(Namespace, Local::CommandTopicLeaf);
}

UObject* UUBotROSVelocityCommandComponent::ResolveCommandTarget(FString* OutError) const
{
    namespace Local = UBot::ROSVelocityCommand::Private;

    // An empty FComponentReference resolves to the root component, so "unset" must be detected explicitly.
    AActor* Owner = GetOwner();
    const bool bComponentConfigured = TargetComponent.OverrideComponent.IsValid()
        || !TargetComponent.ComponentProperty.IsNone()
        || !TargetComponent.PathToComponent.IsEmpty();
    if (bComponentConfigured)
    {
        UActorComponent* Component = TargetComponent.GetComponent(Owner);
        if (Component == nullptr)
        {
            if (OutError)
            {
                const FString Reference = !TargetComponent.ComponentProperty.IsNone() ? TargetComponent.ComponentProperty.ToString()
                    : !TargetComponent.PathToComponent.IsEmpty() ? TargetComponent.PathToComponent
                    : FString(TEXT("<destroyed component>"));
                *OutError = FString::Printf(TEXT("%s: TargetComponent '%s' could not be resolved; commands are only broadcast."),
                    *GetPathNameSafe(this), *Reference);
            }
            return nullptr;
        }
        if (!Local::ImplementsVelocityCommandable(Component))
        {
            if (OutError)
            {
                *OutError = FString::Printf(TEXT("%s: TargetComponent %s does not implement IUBotVelocityCommandable; commands are only broadcast."),
                    *GetPathNameSafe(this), *GetPathNameSafe(Component));
            }
            return nullptr;
        }
        return Component;
    }

    if (const AActor* ReferencedActor = TargetComponent.OtherActor.Get())
    {
        UObject* Found = UBot::Actuation::FindVelocityCommandable(ReferencedActor);
        if (Found == nullptr && OutError)
        {
            *OutError = FString::Printf(TEXT("%s: referenced actor %s has no IUBotVelocityCommandable; commands are only broadcast."),
                *GetPathNameSafe(this), *GetPathNameSafe(ReferencedActor));
        }
        return Found;
    }

    // No target on the owner is a supported configuration (events only), not an error.
    return UBot::Actuation::FindVelocityCommandable(Owner);
}

void UUBotROSVelocityCommandComponent::HandleTwistPayload(TConstArrayView<uint8> Cdr)
{
    namespace Local = UBot::ROSVelocityCommand::Private;

    // UBot::Ros::Decode runs FUBotRosTwist::Deserialize on a bounds-checked FUBotCdrReader.
    FUBotRosTwist Twist;
    if (!UBot::Ros::Decode(Cdr, Twist))
    {
        ReportError(FString::Printf(TEXT("%s: dropped a malformed %s message (%d bytes) on %s."),
            *GetPathNameSafe(this), FUBotRosTwist::TypeName(), Cdr.Num(), *GetResolvedCommandTopic()), false);
        return;
    }

    const double NowSeconds = UBot::Clock::GetSimTimeSeconds(this);
    FUBotVelocityCommand Command = UBot::ROSConvert::MakeVelocityCommand(Twist, NowSeconds);
    if (Command.LinearVelocityMetersPerSecond.ContainsNaN() || Command.AngularVelocityRadiansPerSecond.ContainsNaN())
    {
        ReportError(FString::Printf(TEXT("%s: dropped a %s message with non-finite values on %s."),
            *GetPathNameSafe(this), FUBotRosTwist::TypeName(), *GetResolvedCommandTopic()), false);
        return;
    }

    Command.LinearVelocityMetersPerSecond = Local::ClampNorm(Command.LinearVelocityMetersPerSecond, MaxLinearSpeedMetersPerSecond);
    Command.AngularVelocityRadiansPerSecond = Local::ClampNorm(Command.AngularVelocityRadiansPerSecond, MaxAngularSpeedRadiansPerSecond);

    // A valid command re-arms both the timeout and the error log de-duplication.
    LastCommand = Command;
    LastCommandSimTimeSeconds = NowSeconds;
    bCommandActive = true;
    LastLoggedError.Reset();
    if (CommandTimeoutSeconds > 0.0f)
    {
        SetComponentTickEnabled(true);
    }

    DispatchCommand(Command);
    OnVelocityCommand.Broadcast(Command);
}

void UUBotROSVelocityCommandComponent::DispatchCommand(const FUBotVelocityCommand& Command)
{
    FString TargetError;
    UObject* Target = ResolveCommandTarget(&TargetError);
    if (!TargetError.IsEmpty())
    {
        // Configuration problems are reported once per start, not once per message.
        if (!bTargetErrorReported)
        {
            bTargetErrorReported = true;
            ReportError(TargetError, false);
        }
        return;
    }
    bTargetErrorReported = false;
    if (Target == nullptr)
    {
        return;
    }

    if (IUBotVelocityCommandable::Execute_ApplyVelocityCommand(Target, Command))
    {
        bRejectionReported = false;
        return;
    }
    if (!bRejectionReported)
    {
        bRejectionReported = true;
        ReportError(FString::Printf(TEXT("%s: %s rejected the velocity command."), *GetPathNameSafe(this), *GetPathNameSafe(Target)), false);
    }
}

void UUBotROSVelocityCommandComponent::CheckCommandTimeout()
{
    if (!bCommandActive || CommandTimeoutSeconds <= 0.0f)
    {
        SetComponentTickEnabled(false);
        return;
    }

    const double ElapsedSeconds = UBot::Clock::GetSimTimeSeconds(this) - LastCommandSimTimeSeconds;
    if (ElapsedSeconds < CommandTimeoutSeconds)
    {
        return;
    }

    // Clear the state first so a handler that sends a new command re-arms a fresh timeout.
    bCommandActive = false;
    SetComponentTickEnabled(false);
    if (UObject* Target = ResolveCommandTarget(nullptr))
    {
        IUBotVelocityCommandable::Execute_StopMotion(Target);
    }
    UE_LOG(LogUBotROSVelocityCommand, Verbose, TEXT("%s: no velocity command for %.3f s; motion stopped."), *GetPathNameSafe(this), ElapsedSeconds);
    OnCommandTimeout.Broadcast();
}

void UUBotROSVelocityCommandComponent::ReportError(const FString& Message, bool bSetupError)
{
    LastError = Message;
    // Every occurrence is broadcast; identical consecutive messages are logged once.
    if (LastLoggedError != Message)
    {
        LastLoggedError = Message;
        if (bSetupError)
        {
            UE_LOG(LogUBotROSVelocityCommand, Error, TEXT("%s"), *Message);
        }
        else
        {
            UE_LOG(LogUBotROSVelocityCommand, Warning, TEXT("%s"), *Message);
        }
    }
    OnCommandError.Broadcast(Message);
}
