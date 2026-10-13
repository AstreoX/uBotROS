#pragma once

#include "CoreMinimal.h"
#include "UBotActuatorInterface.h"
#include "UBotROSCommandReceiverComponent.h"
#include "UBotROSAttitudeCommandComponent.generated.h"

DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FUBotROSAttitudeCommandReceivedEvent, const FUBotAttitudeCommand&, Command);
DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FUBotROSBodyRateCommandReceivedEvent, const FUBotBodyRateCommand&, Command);

/**
 * Receives mavros_msgs/msg/AttitudeTarget on /<namespace>/cmd_attitude and forwards it to an
 * IUBotAttitudeCommandable target (an aerial vehicle, such as a drone with a uBot Multirotor Dynamics component).
 *
 * - type_mask without IGNORE_ATTITUDE (128): an attitude command; the orientation is converted from the ROS world
 *   frame to UE and body_rate is not used.
 * - type_mask with IGNORE_ATTITUDE: a body-rate command from body_rate (FLU -> UE body); axes with
 *   IGNORE_ROLL_RATE (1), IGNORE_PITCH_RATE (2) or IGNORE_YAW_RATE (4) are 0.
 * - thrust (0..1 of the vehicle's maximum) is used unless IGNORE_THRUST (64) is set; then the vehicle holds its
 *   altitude.
 *
 * The bridge needs the mavros_msgs package (ros-humble-mavros-msgs).
 */
UCLASS(ClassGroup = (uBot), meta = (BlueprintSpawnableComponent, DisplayName = "uBot ROS Attitude Command"))
class UBOTROS_API UUBotROSAttitudeCommandComponent : public UUBotROSCommandReceiverComponent
{
    GENERATED_BODY()

public:
    /** Fires for every valid attitude command after conversion, whether or not a target exists. */
    UPROPERTY(BlueprintAssignable, Category = "uBot|ROS|Command")
    FUBotROSAttitudeCommandReceivedEvent OnAttitudeCommand;

    /** Fires for every valid body-rate command after conversion, whether or not a target exists. */
    UPROPERTY(BlueprintAssignable, Category = "uBot|ROS|Command")
    FUBotROSBodyRateCommandReceivedEvent OnBodyRateCommand;

    /** The most recent valid attitude command (identity before the first one). */
    UFUNCTION(BlueprintPure, Category = "uBot|ROS|Command")
    FUBotAttitudeCommand GetLastAttitudeCommand() const { return LastAttitudeCommand; }

    /** The most recent valid body-rate command (zero before the first one). */
    UFUNCTION(BlueprintPure, Category = "uBot|ROS|Command")
    FUBotBodyRateCommand GetLastBodyRateCommand() const { return LastBodyRateCommand; }

protected:
    virtual const TCHAR* GetTopicLeaf() const override;
    virtual const TCHAR* GetMessageTypeName() const override;
    virtual UClass* GetTargetInterface() const override;
    virtual UObject* FindTargetOn(const AActor* Actor) const override;
    virtual void HandlePayload(TConstArrayView<uint8> Cdr) override;
    virtual void ReleaseTarget(UObject* Target) override;

private:
    FUBotAttitudeCommand LastAttitudeCommand;
    FUBotBodyRateCommand LastBodyRateCommand;
};
