#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "Containers/ArrayView.h"
#include "UBotActuatorInterface.h"
#include "UBotROSVelocityCommandComponent.h"
#include "UObject/Object.h"
#include "UBotROSVelocityCommandTestTypes.generated.h"

// Test-only reflected types for UBotROS.VelocityCommand.* (UHT cannot see types declared in a .cpp, and it does
// not allow WITH_DEV_AUTOMATION_TESTS guards, so these are compiled in every configuration).

// Befriended by UUBotROSVelocityCommandComponent: feeds a raw payload into the component exactly as the bridge
// subsystem does for a MESSAGE frame on the command subscription.
struct FUBotROSVelocityCommandTestAccess
{
    static void InjectTwistPayload(UUBotROSVelocityCommandComponent& Component, TConstArrayView<uint8> Cdr)
    {
        Component.HandleTwistPayload(Cdr);
    }

    /** Bridge subscription id of the running receiver (0 when stopped). */
    static int32 GetSubscriptionId(const UUBotROSVelocityCommandComponent& Component)
    {
        return Component.SubscriptionId;
    }
};

/** Native IUBotVelocityCommandable that records every call. */
UCLASS(Transient, NotBlueprintable, HideDropdown)
class UUBotROSVelocityCommandTestTarget : public UActorComponent, public IUBotVelocityCommandable
{
    GENERATED_BODY()

public:
    TArray<FUBotVelocityCommand> AppliedCommands;
    int32 StopMotionCount = 0;

    /** Return value of ApplyVelocityCommand. */
    bool bAcceptCommands = true;

    virtual bool ApplyVelocityCommand_Implementation(const FUBotVelocityCommand& Command) override
    {
        AppliedCommands.Add(Command);
        return bAcceptCommands;
    }

    virtual void StopMotion_Implementation() override
    {
        ++StopMotionCount;
    }
};

/** Records the Blueprint events of a UUBotROSVelocityCommandComponent. */
UCLASS(Transient, NotBlueprintable, HideDropdown)
class UUBotROSVelocityCommandTestListener : public UObject
{
    GENERATED_BODY()

public:
    TArray<FUBotVelocityCommand> Commands;
    int32 TimeoutCount = 0;
    TArray<FString> Errors;

    void Bind(UUBotROSVelocityCommandComponent& Component)
    {
        Component.OnVelocityCommand.AddDynamic(this, &ThisClass::HandleVelocityCommand);
        Component.OnCommandTimeout.AddDynamic(this, &ThisClass::HandleCommandTimeout);
        Component.OnCommandError.AddDynamic(this, &ThisClass::HandleCommandError);
    }

    UFUNCTION()
    void HandleVelocityCommand(const FUBotVelocityCommand& Command)
    {
        Commands.Add(Command);
    }

    UFUNCTION()
    void HandleCommandTimeout()
    {
        ++TimeoutCount;
    }

    UFUNCTION()
    void HandleCommandError(const FString& ErrorMessage)
    {
        Errors.Add(ErrorMessage);
    }
};
