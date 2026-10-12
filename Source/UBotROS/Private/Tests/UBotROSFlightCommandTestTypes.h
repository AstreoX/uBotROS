#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "Containers/ArrayView.h"
#include "UBotActuatorInterface.h"
#include "UBotROSCommandReceiverComponent.h"
#include "UObject/Object.h"
#include "UBotROSFlightCommandTestTypes.generated.h"

// Test-only reflected types for UBotROS.AttitudeCommand.* and UBotROS.MotorCommand.* (compiled in every
// configuration because UHT does not allow WITH_DEV_AUTOMATION_TESTS guards).

// Befriended by UUBotROSCommandReceiverComponent: feeds a raw payload into a command component exactly as the
// bridge subsystem does for a MESSAGE frame on its subscription.
struct FUBotROSCommandReceiverTestAccess
{
    static void InjectPayload(UUBotROSCommandReceiverComponent& Component, TConstArrayView<uint8> Cdr)
    {
        Component.HandlePayload(Cdr);
    }
};

/** Native aerial vehicle stand-in: records every attitude, body-rate and motor call. */
UCLASS(Transient, NotBlueprintable, HideDropdown)
class UUBotROSFlightCommandTestTarget : public UActorComponent, public IUBotAttitudeCommandable, public IUBotMotorCommandable
{
    GENERATED_BODY()

public:
    TArray<FUBotAttitudeCommand> Attitudes;
    TArray<FUBotBodyRateCommand> BodyRates;
    TArray<FUBotMotorCommand> MotorCommands;
    int32 AttitudeReleases = 0;
    int32 MotorReleases = 0;
    int32 MotorCount = 4;
    bool bAccept = true;

    virtual bool ApplyAttitudeCommand_Implementation(const FUBotAttitudeCommand& Command) override
    {
        Attitudes.Add(Command);
        return bAccept;
    }

    virtual bool ApplyBodyRateCommand_Implementation(const FUBotBodyRateCommand& Command) override
    {
        BodyRates.Add(Command);
        return bAccept;
    }

    virtual void ReleaseAttitudeCommand_Implementation() override
    {
        ++AttitudeReleases;
    }

    virtual bool ApplyMotorCommand_Implementation(const FUBotMotorCommand& Command) override
    {
        MotorCommands.Add(Command);
        return bAccept;
    }

    virtual int32 GetMotorCount_Implementation() const override
    {
        return MotorCount;
    }

    virtual void ReleaseMotorCommand_Implementation() override
    {
        ++MotorReleases;
    }
};

/** Records timeouts and errors of a command component. */
UCLASS(Transient, NotBlueprintable, HideDropdown)
class UUBotROSFlightCommandTestListener : public UObject
{
    GENERATED_BODY()

public:
    int32 TimeoutCount = 0;
    TArray<FString> Errors;

    void Bind(UUBotROSCommandReceiverComponent& Component)
    {
        Component.OnCommandTimeout.AddDynamic(this, &ThisClass::HandleTimeout);
        Component.OnCommandError.AddDynamic(this, &ThisClass::HandleError);
    }

    UFUNCTION()
    void HandleTimeout()
    {
        ++TimeoutCount;
    }

    UFUNCTION()
    void HandleError(const FString& ErrorMessage)
    {
        Errors.Add(ErrorMessage);
    }
};
