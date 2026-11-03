#include "UBotROSCommandReceiverComponent.h"

#include "GameFramework/Actor.h"
#include "UBotClock.h"
#include "UBotIdentityComponent.h"
#include "UBotROSBridgeSubsystem.h"
#include "UBotROSSensorPublisher.h"
#include "UBotROSTypes.h"

DEFINE_LOG_CATEGORY_STATIC(LogUBotROSCommand, Log, All);

namespace UBot::ROSCommandReceiver::Private
{
    constexpr int32 CommandQueueDepth = 10;
}

UUBotROSCommandReceiverComponent::UUBotROSCommandReceiverComponent()
{
    // The tick only checks the command timeout; it is enabled while a command is active.
    PrimaryComponentTick.bCanEverTick = true;
    PrimaryComponentTick.bStartWithTickEnabled = false;
}

void UUBotROSCommandReceiverComponent::BeginPlay()
{
    Super::BeginPlay();
    bEndingPlay = false;
    if (bAutoStartReceiver)
    {
        StartReceiver();
    }
}

void UUBotROSCommandReceiverComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    bEndingPlay = true;
    ++CommandRevision;
    StopReceiver();

    // The timeout tick stops with the component, so release the target now instead of leaving it on the last command.
    if (bCommandActive)
    {
        bCommandActive = false;
        SetComponentTickEnabled(false);
        ReleaseCommandTarget();
    }

    Super::EndPlay(EndPlayReason);
}

void UUBotROSCommandReceiverComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
    Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
    CheckCommandTimeout();
}

bool UUBotROSCommandReceiverComponent::IsCommandReceiverReady() const
{
    const UUBotROSBridgeSubsystem* Bridge = BridgeSubsystem.Get();
    return SubscriptionId > 0 && Bridge != nullptr && Bridge->IsSubscribed(SubscriptionId);
}

FString UUBotROSCommandReceiverComponent::GetResolvedCommandTopic() const
{
    return SubscriptionId > 0 ? ActiveCommandTopic : ResolveCommandTopic();
}

UObject* UUBotROSCommandReceiverComponent::GetCommandTarget() const
{
    return ResolveCommandTarget(nullptr);
}

void UUBotROSCommandReceiverComponent::SetTargetComponent(UActorComponent* NewTarget)
{
    UObject* PreviousTarget = ResolveCommandTarget(nullptr);
    TargetComponent = FComponentReference();
    TargetComponent.OverrideComponent = NewTarget;
    bTargetErrorReported = false;
    bRejectionReported = false;
    UObject* ResolvedTarget = ResolveCommandTarget(nullptr);
    if (PreviousTarget != ResolvedTarget)
    {
        ++CommandRevision;
    }
    if (AcceptedCommandTarget.Get() != ResolvedTarget)
    {
        ReleaseCommandTarget();
    }
}

bool UUBotROSCommandReceiverComponent::RestartCommandReceiver()
{
    StopReceiver();
    return StartReceiver();
}

void UUBotROSCommandReceiverComponent::StopCommandReceiver()
{
    StopReceiver();
}

bool UUBotROSCommandReceiverComponent::StartReceiver()
{
    namespace Local = UBot::ROSCommandReceiver::Private;

    LastError.Reset();
    LastLoggedError.Reset();
    bTargetErrorReported = false;
    bRejectionReported = false;

    const FString Topic = ResolveCommandTopic();
    FString Reason;
    if (!UBot::ROSPublishing::IsValidAbsoluteTopic(Topic, &Reason))
    {
        ReportError(FString::Printf(TEXT("Command topic '%s' is not a valid absolute ROS 2 topic: %s."), *Topic, *Reason), true);
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

    TWeakObjectPtr<UUBotROSCommandReceiverComponent> WeakThis(this);
    const int32 NewSubscriptionId = Bridge->Subscribe(Topic, GetMessageTypeName(), Qos,
        [WeakThis](TConstArrayView<uint8> Cdr)
        {
            if (UUBotROSCommandReceiverComponent* Self = WeakThis.Get())
            {
                Self->HandlePayload(Cdr);
            }
        });
    if (NewSubscriptionId <= 0)
    {
        ReportError(FString::Printf(TEXT("The uBot ROS bridge refused the subscription to %s (%s)."), *Topic, GetMessageTypeName()), true);
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
    UE_LOG(LogUBotROSCommand, Log, TEXT("%s: listening for %s on %s; target %s."), *GetPathNameSafe(this), GetMessageTypeName(), *Topic,
        Target ? *GetPathNameSafe(Target) : TEXT("none (events only)"));
    return true;
}

void UUBotROSCommandReceiverComponent::StopReceiver()
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

FString UUBotROSCommandReceiverComponent::ResolveCommandTopic() const
{
    const FString TopicOverride = CommandTopicOverride.TrimStartAndEnd();
    if (!TopicOverride.IsEmpty())
    {
        return TopicOverride.StartsWith(TEXT("/"), ESearchCase::CaseSensitive) ? TopicOverride : TEXT("/") + TopicOverride;
    }
    const FString NamespaceOverride = RobotNamespaceOverride.TrimStartAndEnd();
    const FString Namespace = NamespaceOverride.IsEmpty() ? UBot::Identity::ResolveNamespace(GetOwner()) : NamespaceOverride;
    return UBot::ROSPublishing::JoinTopic(Namespace, GetTopicLeaf());
}

UObject* UUBotROSCommandReceiverComponent::ResolveCommandTarget(FString* OutError) const
{
    // An empty FComponentReference resolves to the root component, so "unset" must be detected explicitly.
    AActor* Owner = GetOwner();
    const UClass* Interface = GetTargetInterface();
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
        if (!Interface || !Component->GetClass()->ImplementsInterface(Interface))
        {
            if (OutError)
            {
                *OutError = FString::Printf(TEXT("%s: TargetComponent %s does not implement %s; commands are only broadcast."),
                    *GetPathNameSafe(this), *GetPathNameSafe(Component), *GetNameSafe(Interface));
            }
            return nullptr;
        }
        return Component;
    }

    if (const AActor* ReferencedActor = TargetComponent.OtherActor.Get())
    {
        UObject* Found = FindTargetOn(ReferencedActor);
        if (Found == nullptr && OutError)
        {
            *OutError = FString::Printf(TEXT("%s: referenced actor %s has no %s; commands are only broadcast."),
                *GetPathNameSafe(this), *GetPathNameSafe(ReferencedActor), *GetNameSafe(Interface));
        }
        return Found;
    }

    // No target on the owner is a supported configuration (events only), not an error.
    return FindTargetOn(Owner);
}

UObject* UUBotROSCommandReceiverComponent::ResolveTargetForDispatch()
{
    const uint64 DispatchRevision = CommandRevision;
    FString TargetError;
    UObject* Target = ResolveCommandTarget(&TargetError);
    if (AcceptedCommandTarget.Get() != Target)
    {
        ReleaseCommandTarget();
    }
    // Releasing a target can invoke user code that changes the target or dispatches a newer command.
    if (DispatchRevision != CommandRevision || bEndingPlay || (Target && !IsValid(Target)))
    {
        return nullptr;
    }
    if (!TargetError.IsEmpty())
    {
        // Configuration problems are reported once per start, not once per message.
        if (!bTargetErrorReported)
        {
            bTargetErrorReported = true;
            ReportError(TargetError, false);
        }
        return nullptr;
    }
    bTargetErrorReported = false;
    return Target;
}

void UUBotROSCommandReceiverComponent::NoteValidCommand()
{
    ++CommandRevision;
    LastCommandSimTimeSeconds = UBot::Clock::GetSimTimeSeconds(this);
    bCommandActive = true;
    LastLoggedError.Reset();
    if (CommandTimeoutSeconds > 0.0f)
    {
        SetComponentTickEnabled(true);
    }
}

void UUBotROSCommandReceiverComponent::NoteDispatchResult(const UObject* Target, bool bAccepted, const TCHAR* What)
{
    NoteDispatchResult(const_cast<UObject*>(Target), bAccepted, What, CommandRevision);
}

void UUBotROSCommandReceiverComponent::NoteDispatchResult(UObject* Target, bool bAccepted, const TCHAR* What, uint64 DispatchRevision)
{
    if (bAccepted)
    {
        if (DispatchRevision != CommandRevision || bEndingPlay || ResolveCommandTarget(nullptr) != Target)
        {
            // A setter, EndPlay or another command ran inside Apply*. Do not orphan that accepted
            // command, or release a newer command already held by the same target.
            if (IsValid(Target) && AcceptedCommandTarget.Get() != Target)
            {
                ReleaseTarget(Target);
            }
            return;
        }
        AcceptedCommandTarget = Target;
        bRejectionReported = false;
        return;
    }
    if (DispatchRevision == CommandRevision && !bEndingPlay && !bRejectionReported)
    {
        bRejectionReported = true;
        ReportError(FString::Printf(TEXT("%s: %s rejected the %s."), *GetPathNameSafe(this), *GetPathNameSafe(Target), What), false);
    }
}

void UUBotROSCommandReceiverComponent::CheckCommandTimeout()
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
    ++CommandRevision;
    ReleaseCommandTarget();
    UE_LOG(LogUBotROSCommand, Verbose, TEXT("%s: no command for %.3f s; target released."), *GetPathNameSafe(this), ElapsedSeconds);
    OnCommandTimeout.Broadcast();
}

void UUBotROSCommandReceiverComponent::ReleaseCommandTarget()
{
    UObject* Target = AcceptedCommandTarget.Get();
    // Clear ownership before invoking a Blueprint/native callback, which may reenter this receiver.
    AcceptedCommandTarget.Reset();
    if (IsValid(Target))
    {
        ReleaseTarget(Target);
    }
}

void UUBotROSCommandReceiverComponent::ReportError(const FString& Message, bool bSetupError)
{
    LastError = Message;
    // Every occurrence is broadcast; identical consecutive messages are logged once.
    if (LastLoggedError != Message)
    {
        LastLoggedError = Message;
        if (bSetupError)
        {
            UE_LOG(LogUBotROSCommand, Error, TEXT("%s"), *Message);
        }
        else
        {
            UE_LOG(LogUBotROSCommand, Warning, TEXT("%s"), *Message);
        }
    }
    OnCommandError.Broadcast(Message);
}
