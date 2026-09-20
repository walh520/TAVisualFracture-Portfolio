using UnrealBuildTool;

public class TAVisualFractureRuntime : ModuleRules
{
    public TAVisualFractureRuntime(ReadOnlyTargetRules Target) : base(Target)
    {
        PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;
        PublicDependencyModuleNames.AddRange(new[]
        {
            "Core",
            "CoreUObject",
            "Engine",
            "ProceduralMeshComponent",
            "TAVisualFractureCore"
        });
    }
}
