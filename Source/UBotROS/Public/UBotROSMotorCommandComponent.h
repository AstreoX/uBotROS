#pragma once

#include "CoreMinimal.h"
#include "UBotActuatorInterface.h"
#include "UBotROSCommandReceiverComponent.h"
#include "UBotROSMotorCommandComponent.generated.h"

DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FUBotROSMotorCommandReceivedEvent, const FUBotMotorCommand&, Command);

/**
 * Receives actuator_msgs/msg/Actuators on /<namespace>/cmd_motors and forwards it to an IUBotMotorCommandable
 * target (for example a drone's uBot Multirotor Dynamics component). velocity[] (rad/s) is used when it has
 * values, else normalized[] (0..1); position[] is ignored. A command whose length differs from the target's
 * motor count is not dispatched.
 *
 * The bridge needs the actuator_msgs package (ros-humble-actuator-msgs).
 */
UCLASS(ClassGroup = (uBot), meta = (BlueprintSpawnableComponent, DisplayName = "uBot ROS Motor Command"))
class UBOTROS_API UUBotROSMotorCommandComponent : public UUBotROSCommandReceiverComponent
{
    GENERATED_BODY()

public:
    /** Fires for every valid motor command after conversion, whether or not a target exists. */
    UPROPERTY(BlueprintAssignable, Category = "uBot|ROS|Command")
    FUBotROSMotorCommandReceivedEvent OnMotorCommand;

    /** The most recent valid motor command (empty before the first one). */
    UFUNCTION(BlueprintPure, Category = "uBot|ROS|Command")
    FUBotMotorCommand GetLastMotorCommand() const { return LastMotorCommand; }

protected:
    virtual const TCHAR* GetTopicLeaf() const override;
    virtual const TCHAR* GetMessageTypeName() const override;
    virtual UClass* GetTargetInterface() const override;
    virtual UObject* FindTargetOn(const AActor* Actor) const override;
    virtual void HandlePayload(TConstArrayView<uint8> Cdr) override;
    virtual void ReleaseTarget(UObject* Target) override;

private:
    FUBotMotorCommand LastMotorCommand;
};
