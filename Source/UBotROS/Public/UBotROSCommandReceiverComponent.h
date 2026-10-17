#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "Containers/ArrayView.h"
#include "Engine/EngineTypes.h"
#include "UBotROSCommandReceiverComponent.generated.h"

class UUBotROSBridgeSubsystem;

DECLARE_DYNAMIC_MULTICAST_DELEGATE(FUBotROSCommandTimeoutEvent);
DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FUBotROSCommandErrorEvent, const FString&, ErrorMessage);

/**
 * Base of the components that receive commands from ROS 2 through the uBot ROS bridge and forward them to a
 * target that implements a uBotCore actuation interface. Contains no control logic.
 *
 * - Topic: /<namespace>/<leaf of the subclass> (best_effort / volatile / depth 10). The namespace comes from
 *   UBot::Identity unless Robot Namespace Override is set; Command Topic Override replaces the whole topic.
 * - Target: Target Component when set, else the owner actor or its first component that implements the
 *   subclass' interface. Without a target the component still decodes commands, fires its events and runs
 *   the timeout.
 * - Timeout: when no command arrived for Command Timeout Seconds of sim time, the target is released exactly
 *   once (it falls back to its own behavior, such as a hover) and On Command Timeout fires. Ending play
 *   releases an active command too.
 */
UCLASS(Abstract, ClassGroup = (uBot), meta = (DisplayName = "uBot ROS Command Receiver"))
class UBOTROS_API UUBotROSCommandReceiverComponent : public UActorComponent
{
    GENERATED_BODY()

    friend struct FUBotROSCommandReceiverTestAccess;

public:
    UUBotROSCommandReceiverComponent();

    /** Subscribe in BeginPlay. When false, call Restart Command Receiver to start receiving. */
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "ROS|Command")
    bool bAutoStartReceiver = true;

    /**
     * Empty -> the namespace resolved by UBot::Identity (identity component, else the sanitized actor name).
     * Leading/trailing '/' are ignored and every segment is sanitized; a value made only of '/' selects the
     * root namespace.
     */
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "ROS|Command|Identity")
    FString RobotNamespaceOverride;

    /** Non-empty -> used instead of /<namespace>/<leaf>. A missing leading '/' is added. */
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "ROS|Command|Topic")
    FString CommandTopicOverride;

    /**
     * Component that receives the commands; it must implement the subclass' interface. Unset -> the owner actor
     * or its first implementing component. With only "Referenced Actor" set, that actor is searched the same way.
     */
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "ROS|Command|Target", meta = (UseComponentPicker))
    FComponentReference TargetComponent;

    /** Sim seconds without a command before the target is released. 0 disables the timeout. */
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "ROS|Command|Safety", meta = (ClampMin = "0.0", Units = "s"))
    float CommandTimeoutSeconds = 0.5f;

    /** Fires once per timeout, after the target (if any) was released. */
    UPROPERTY(BlueprintAssignable, Category = "uBot|ROS|Command")
    FUBotROSCommandTimeoutEvent OnCommandTimeout;

    /** Setup failures, malformed or invalid messages, an unusable configured target, rejected commands. */
    UPROPERTY(BlueprintAssignable, Category = "uBot|ROS|Command")
    FUBotROSCommandErrorEvent OnCommandError;

    /** True while subscribed to the bridge (independent of the bridge connection state). */
    UFUNCTION(BlueprintPure, Category = "uBot|ROS|Command")
    bool IsCommandReceiverReady() const;

    /** True from a valid command until its timeout (always true after a command when the timeout is disabled). */
    UFUNCTION(BlueprintPure, Category = "uBot|ROS|Command")
    bool IsCommandActive() const { return bCommandActive; }

    /** The subscribed topic while the receiver runs, else the topic the next start would use. */
    UFUNCTION(BlueprintPure, Category = "uBot|ROS|Command")
    FString GetResolvedCommandTopic() const;

    /** The most recent error message (empty when none since the last start). */
    UFUNCTION(BlueprintPure, Category = "uBot|ROS|Command")
    FString GetCommandError() const { return LastError; }

    /** The object commands are dispatched to, or nullptr (events only). */
    UFUNCTION(BlueprintPure, Category = "uBot|ROS|Command")
    UObject* GetCommandTarget() const;

    /** Replaces Target Component by a direct reference; nullptr restores the automatic lookup on the owner. */
    UFUNCTION(BlueprintCallable, Category = "uBot|ROS|Command")
    void SetTargetComponent(UActorComponent* NewTarget);

    /** Unsubscribes and subscribes again with the current settings. The command timeout keeps running. */
    UFUNCTION(BlueprintCallable, Category = "uBot|ROS|Command")
    bool RestartCommandReceiver();

    /** Unsubscribes. An active command still times out normally. */
    UFUNCTION(BlueprintCallable, Category = "uBot|ROS|Command")
    void StopCommandReceiver();

    virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

protected:
    virtual void BeginPlay() override;
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

    /** Last segment of the default topic, e.g. "cmd_attitude". */
    virtual const TCHAR* GetTopicLeaf() const PURE_VIRTUAL(UUBotROSCommandReceiverComponent::GetTopicLeaf, return TEXT("cmd"););

    /** ROS 2 type of the subscription, e.g. "mavros_msgs/msg/AttitudeTarget". */
    virtual const TCHAR* GetMessageTypeName() const PURE_VIRTUAL(UUBotROSCommandReceiverComponent::GetMessageTypeName, return TEXT(""););

    /** The uBotCore interface a target must implement (its UInterface class). */
    virtual UClass* GetTargetInterface() const PURE_VIRTUAL(UUBotROSCommandReceiverComponent::GetTargetInterface, return nullptr;);

    /** The implementer on Actor (the actor itself, else its first component), or nullptr. */
    virtual UObject* FindTargetOn(const AActor* Actor) const PURE_VIRTUAL(UUBotROSCommandReceiverComponent::FindTargetOn, return nullptr;);

    /** Decodes one complete serialized message (game thread) and dispatches it. Tests call it directly. */
    virtual void HandlePayload(TConstArrayView<uint8> Cdr) PURE_VIRTUAL(UUBotROSCommandReceiverComponent::HandlePayload, );

    /** Tells the target its sender stopped (timeout, end of play). */
    virtual void ReleaseTarget(UObject* Target) PURE_VIRTUAL(UUBotROSCommandReceiverComponent::ReleaseTarget, );

    /** The target for a dispatch, reporting configuration problems once per start; nullptr when there is none. */
    UObject* ResolveTargetForDispatch();

    /** Re-arms the timeout after a valid command. */
    void NoteValidCommand();

    /** Reports a rejected command once until the target accepts one again. */
    void NoteDispatchResult(const UObject* Target, bool bAccepted, const TCHAR* What);

    /** Broadcasts OnCommandError; identical consecutive messages are logged once. */
    void ReportError(const FString& Message, bool bSetupError);

private:
    bool StartReceiver();
    void StopReceiver();
    FString ResolveCommandTopic() const;
    UObject* ResolveCommandTarget(FString* OutError) const;
    void CheckCommandTimeout();

    TWeakObjectPtr<UUBotROSBridgeSubsystem> BridgeSubsystem;
    int32 SubscriptionId = 0;
    FString ActiveCommandTopic;
    double LastCommandSimTimeSeconds = 0.0;
    bool bCommandActive = false;
    FString LastError;
    FString LastLoggedError;
    bool bTargetErrorReported = false;
    bool bRejectionReported = false;
};
