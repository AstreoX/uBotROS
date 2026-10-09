using UnrealBuildTool;

public class UBotROS : ModuleRules
{
    public UBotROS(ReadOnlyTargetRules Target) : base(Target)
    {
        PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;

        PublicDependencyModuleNames.AddRange(new[]
        {
            "Core",
            "CoreUObject",
            "Engine",
            "DeveloperSettings",
            "UBotCore",
            "UBotSensor"
        });

        PrivateDependencyModuleNames.AddRange(new[]
        {
            "Sockets",
            "Networking",
            "Json"
        });
    }
}
