#pragma once

#include "CoreMinimal.h"
#include "Modules/ModuleInterface.h"

UBOTROS_API DECLARE_LOG_CATEGORY_EXTERN(LogUBotROS, Log, All);

/**
 * UBotROS runtime module. StartupModule declares the "UBotROS.SensorPublisher" extension point and
 * registers the seven built-in sensor publisher factories; ShutdownModule removes everything the
 * package registered (UnregisterAllFromPackage("UBotROS")).
 */
class FUBotROSModule final : public IModuleInterface
{
public:
    virtual void StartupModule() override;
    virtual void ShutdownModule() override;
};
