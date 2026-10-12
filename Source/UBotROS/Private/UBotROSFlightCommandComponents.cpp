#include "UBotROSAttitudeCommandComponent.h"
#include "UBotROSMotorCommandComponent.h"

#include "UBotClock.h"
#include "UBotROSConverters.h"
#include "UBotROSMessageTypes.h"

// ---------------------------------------------------------------------------------------------------------------
// uBot ROS Attitude Command

const TCHAR* UUBotROSAttitudeCommandComponent::GetTopicLeaf() const
{
    return TEXT("cmd_attitude");
}

const TCHAR* UUBotROSAttitudeCommandComponent::GetMessageTypeName() const
{
    return FUBotRosAttitudeTarget::TypeName();
}

UClass* UUBotROSAttitudeCommandComponent::GetTargetInterface() const
{
    return UUBotAttitudeCommandable::StaticClass();
}

UObject* UUBotROSAttitudeCommandComponent::FindTargetOn(const AActor* Actor) const
{
    return UBot::Actuation::FindAttitudeCommandable(Actor);
}

void UUBotROSAttitudeCommandComponent::HandlePayload(TConstArrayView<uint8> Cdr)
{
    FUBotRosAttitudeTarget Message;
    if (!UBot::Ros::Decode(Cdr, Message))
    {
        ReportError(FString::Printf(TEXT("%s: dropped a malformed %s message (%d bytes) on %s."),
            *GetPathNameSafe(this), FUBotRosAttitudeTarget::TypeName(), Cdr.Num(), *GetResolvedCommandTopic()), false);
        return;
    }

    bool bBodyRate = false;
    FUBotAttitudeCommand Attitude;
    FUBotBodyRateCommand BodyRate;
    FString Error;
    if (!UBot::ROSConvert::MakeAttitudeTargetCommand(Message, UBot::Clock::GetSimTimeSeconds(this), bBodyRate, Attitude, BodyRate, &Error))
    {
        ReportError(FString::Printf(TEXT("%s: dropped a %s message on %s: %s."),
            *GetPathNameSafe(this), FUBotRosAttitudeTarget::TypeName(), *GetResolvedCommandTopic(), *Error), false);
        return;
    }

    NoteValidCommand();
    UObject* Target = ResolveTargetForDispatch();
    if (bBodyRate)
    {
        LastBodyRateCommand = BodyRate;
        if (Target)
        {
            NoteDispatchResult(Target, IUBotAttitudeCommandable::Execute_ApplyBodyRateCommand(Target, BodyRate), TEXT("body-rate command"));
        }
        OnBodyRateCommand.Broadcast(BodyRate);
    }
    else
    {
        LastAttitudeCommand = Attitude;
        if (Target)
        {
            NoteDispatchResult(Target, IUBotAttitudeCommandable::Execute_ApplyAttitudeCommand(Target, Attitude), TEXT("attitude command"));
        }
        OnAttitudeCommand.Broadcast(Attitude);
    }
}

void UUBotROSAttitudeCommandComponent::ReleaseTarget(UObject* Target)
{
    IUBotAttitudeCommandable::Execute_ReleaseAttitudeCommand(Target);
}

// ---------------------------------------------------------------------------------------------------------------
// uBot ROS Motor Command

const TCHAR* UUBotROSMotorCommandComponent::GetTopicLeaf() const
{
    return TEXT("cmd_motors");
}

const TCHAR* UUBotROSMotorCommandComponent::GetMessageTypeName() const
{
    return FUBotRosActuators::TypeName();
}

UClass* UUBotROSMotorCommandComponent::GetTargetInterface() const
{
    return UUBotMotorCommandable::StaticClass();
}

UObject* UUBotROSMotorCommandComponent::FindTargetOn(const AActor* Actor) const
{
    return UBot::Actuation::FindMotorCommandable(Actor);
}

void UUBotROSMotorCommandComponent::HandlePayload(TConstArrayView<uint8> Cdr)
{
    FUBotRosActuators Message;
    if (!UBot::Ros::Decode(Cdr, Message))
    {
        ReportError(FString::Printf(TEXT("%s: dropped a malformed %s message (%d bytes) on %s."),
            *GetPathNameSafe(this), FUBotRosActuators::TypeName(), Cdr.Num(), *GetResolvedCommandTopic()), false);
        return;
    }

    FUBotMotorCommand Command;
    FString Error;
    if (!UBot::ROSConvert::MakeMotorCommand(Message, UBot::Clock::GetSimTimeSeconds(this), Command, &Error))
    {
        ReportError(FString::Printf(TEXT("%s: dropped a %s message on %s: %s."),
            *GetPathNameSafe(this), FUBotRosActuators::TypeName(), *GetResolvedCommandTopic(), *Error), false);
        return;
    }

    NoteValidCommand();
    LastMotorCommand = Command;
    if (UObject* Target = ResolveTargetForDispatch())
    {
        const int32 MotorCount = IUBotMotorCommandable::Execute_GetMotorCount(Target);
        if (MotorCount != Command.Values.Num())
        {
            ReportError(FString::Printf(TEXT("%s: the motor command has %d values but %s has %d motors; not dispatched."),
                *GetPathNameSafe(this), Command.Values.Num(), *GetPathNameSafe(Target), MotorCount), false);
        }
        else
        {
            NoteDispatchResult(Target, IUBotMotorCommandable::Execute_ApplyMotorCommand(Target, Command), TEXT("motor command"));
        }
    }
    OnMotorCommand.Broadcast(Command);
}

void UUBotROSMotorCommandComponent::ReleaseTarget(UObject* Target)
{
    IUBotMotorCommandable::Execute_ReleaseMotorCommand(Target);
}
