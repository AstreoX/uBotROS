#include "UBotROS.h"

#include "Modules/ModuleManager.h"
#include "UBotExtensionRegistry.h"
#include "UBotROSBuiltinPublishers.h"
#include "UBotROSSensorPublisher.h"

DEFINE_LOG_CATEGORY(LogUBotROS);

void FUBotROSModule::StartupModule()
{
    FUBotExtensionRegistry& Registry = FUBotExtensionRegistry::Get();
    const FName OwnerPackage(UBot::ROSPublishing::OwnerPackageName);

    const FText Description = NSLOCTEXT("UBotROS", "SensorPublisherPointDescription",
        "ROS 2 publishers for uBot sensors. Register an IUBotROSSensorPublisherFactory (UBotROSSensorPublisher.h); "
        "the uBot ROS Sensor Bridge uses the supporting factory with the highest priority for each sensor.");
    if (!Registry.RegisterExtensionPoint(UBot::ROSPublishing::GetExtensionPointName(), OwnerPackage, Description))
    {
        UE_LOG(LogUBotROS, Error, TEXT("Extension point '%s' is already declared by another package; uBot ROS sensor publishing is unavailable."),
            UBot::ROSPublishing::ExtensionPointName);
    }

    UBot::ROSBuiltin::RegisterBuiltinFactories(Registry, OwnerPackage);
}

void FUBotROSModule::ShutdownModule()
{
    FUBotExtensionRegistry::Get().UnregisterAllFromPackage(FName(UBot::ROSPublishing::OwnerPackageName));
}

IMPLEMENT_MODULE(FUBotROSModule, UBotROS)
