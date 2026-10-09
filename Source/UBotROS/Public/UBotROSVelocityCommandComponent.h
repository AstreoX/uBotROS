#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "Containers/ArrayView.h"
#include "Engine/EngineTypes.h"
#include "UBotActuatorInterface.h"
#include "UBotROSVelocityCommandComponent.generated.h"

class UUBotROSBridgeSubsystem;

DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FUBotROSVelocityCommandReceivedEvent, const FUBotVelocityCommand&, Command);
DECLARE_DYNAMIC_MULTICAST_DELEGATE(FUBotROSVelocityCommandTimeoutEvent);
DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FUBotROSVelocityCommandErrorEvent, const FString&, ErrorMessage);

/**
 * Receives geometry_msgs/msg/Twist velocity commands from ROS 2 through the uBot ROS bridge and forwards
 * them to an IUBotVelocityCommandable target. Contains no control logic.
 *
 * - Topic: /<namespace>/cmd_vel (best_effort / volatile / depth 10). The namespace comes from UBot::Identity
 *   unless RobotNamespaceOverride is set; CommandTopicOverride replaces the whole topic.
 * - Conversion: ROS body frame -> UE body axes through UBot::ROSConvert::MakeVelocityCommand (linear (x, -y, z),
 *   angular (-x, y, -z), Source "ROS", timestamp = uBot sim time at reception), then the optional speed limits.
 * - Target: TargetComponent when set, else UBot::Actuation::FindVelocityCommandable(owner). Without a target the
 *   component still decodes commands, fires its events and runs the timeout, so Blueprints can consume them.
 * - Timeout: when no command arrived for CommandTimeoutSeconds of sim time (checked in TickComponent, which only
 *   runs while a command is active), the target receives StopMotion exactly once and OnCommandTimeout fires.
 */
UCLASS(ClassGroup = (uBot), meta = (BlueprintSpawnableComponent, DisplayName = "uBot ROS Velocity Command"))
class UBOTROS_API UUBotROSVelocityCommandComponent : public UActorComponent
{
    GENERATED_BODY()

    friend struct FUBotROSVelocityCommandTestAccess;

public:
    UUBotROSVelocityCommandComponent();

    /** Subscribe in BeginPlay. When false, call RestartCommandReceiver to start receiving. */
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "uBot|ROS|Velocity Command")
    bool bAutoStartReceiver = true;

    /**
     * Empty -> the namespace resolved by UBot::Identity (identity component, else the sanitized actor name).
     * Leading/trailing '/' are ignored and every segment is sanitized; a value made only of '/' selects the
     * root namespace (topic /cmd_vel).
     */
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "uBot|ROS|Velocity Command|Identity")
    FString RobotNamespaceOverride;

    /** Non-empty -> used instead of /<namespace>/cmd_vel. A missing leading '/' is added. */
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "uBot|ROS|Velocity Command|Topic")
    FString CommandTopicOverride;

    /**
     * Component that receives the commands; it must implement IUBotVelocityCommandable. Unset -> the owner actor
     * or its first implementing component. With only "Referenced Actor" set, that actor is searched the same way.
     */
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "uBot|ROS|Velocity Command|Target", meta = (UseComponentPicker))
    FComponentReference TargetComponent;

    /** Sim seconds without a command before the target is stopped. 0 disables the timeout. */
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "uBot|ROS|Velocity Command|Safety", meta = (ClampMin = "0.0", Units = "s"))
    float CommandTimeoutSeconds = 0.5f;

    /** Upper bound of the linear speed (vector norm, direction preserved). 0 = unlimited. */
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "uBot|ROS|Velocity Command|Limits", meta = (ClampMin = "0.0"))
    float MaxLinearSpeedMetersPerSecond = 0.0f;

    /** Upper bound of the angular speed (vector norm, axis preserved). 0 = unlimited. */
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "uBot|ROS|Velocity Command|Limits", meta = (ClampMin = "0.0"))
    float MaxAngularSpeedRadiansPerSecond = 0.0f;

    /** Fires for every valid command after conversion and clamping, whether or not a target exists. */
    UPROPERTY(BlueprintAssignable, Category = "uBot|ROS|Velocity Command")
    FUBotROSVelocityCommandReceivedEvent OnVelocityCommand;

    /** Fires once per timeout, after StopMotion was sent to the target (if any). */
    UPROPERTY(BlueprintAssignable, Category = "uBot|ROS|Velocity Command")
    FUBotROSVelocityCommandTimeoutEvent OnCommandTimeout;

    /** Setup failures, malformed or non-finite messages, an unusable configured target, rejected commands. */
    UPROPERTY(BlueprintAssignable, Category = "uBot|ROS|Velocity Command")
    FUBotROSVelocityCommandErrorEvent OnCommandError;

    /** True while subscribed to the bridge (independent of the bridge connection state). */
    UFUNCTION(BlueprintPure, Category = "uBot|ROS|Velocity Command")
    bool IsCommandReceiverReady() const;

    /** True from a valid command until its timeout (always true after a command when the timeout is disabled). */
    UFUNCTION(BlueprintPure, Category = "uBot|ROS|Velocity Command")
    bool IsCommandActive() const { return bCommandActive; }

    /** The most recent valid command after clamping (zero before the first one). */
    UFUNCTION(BlueprintPure, Category = "uBot|ROS|Velocity Command")
    FUBotVelocityCommand GetLastCommand() const { return LastCommand; }

    /** The subscribed topic while the receiver runs, else the topic the next start would use. */
    UFUNCTION(BlueprintPure, Category = "uBot|ROS|Velocity Command")
    FString GetResolvedCommandTopic() const;

    /** The most recent error message (empty when none since the last start). */
    UFUNCTION(BlueprintPure, Category = "uBot|ROS|Velocity Command")
    FString GetCommandError() const { return LastError; }

    /** The object commands are dispatched to, or nullptr (events only). */
    UFUNCTION(BlueprintPure, Category = "uBot|ROS|Velocity Command")
    UObject* GetCommandTarget() const;

    /** Replaces TargetComponent by a direct reference; nullptr restores the automatic lookup on the owner. */
    UFUNCTION(BlueprintCallable, Category = "uBot|ROS|Velocity Command")
    void SetTargetComponent(UActorComponent* NewTarget);

    /** Unsubscribes and subscribes again with the current settings. The command timeout keeps running. */
    UFUNCTION(BlueprintCallable, Category = "uBot|ROS|Velocity Command")
    bool RestartCommandReceiver();

    /** Unsubscribes. An active command still times out normally. */
    UFUNCTION(BlueprintCallable, Category = "uBot|ROS|Velocity Command")
    void StopCommandReceiver();

    virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

protected:
    virtual void BeginPlay() override;
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

    /**
     * Entry point of every bridge MESSAGE on the command subscription (game thread): Cdr is the complete
     * serialized geometry_msgs/msg/Twist including the 4-byte encapsulation header. Tests call it directly.
     */
    void HandleTwistPayload(TConstArrayView<uint8> Cdr);

private:
    bool StartReceiver();
    void StopReceiver();
    FString ResolveCommandTopic() const;
    UObject* ResolveCommandTarget(FString* OutError) const;
    void DispatchCommand(const FUBotVelocityCommand& Command);
    void CheckCommandTimeout();
    void ReportError(const FString& Message, bool bSetupError);

    TWeakObjectPtr<UUBotROSBridgeSubsystem> BridgeSubsystem;
    int32 SubscriptionId = 0;
    FString ActiveCommandTopic;

    FUBotVelocityCommand LastCommand;
    double LastCommandSimTimeSeconds = 0.0;
    bool bCommandActive = false;

    FString LastError;
    FString LastLoggedError;
    bool bTargetErrorReported = false;
    bool bRejectionReported = false;
};
