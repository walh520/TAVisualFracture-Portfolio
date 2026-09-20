using UnrealBuildTool;

public class TAVisualFractureEditor : ModuleRules
{
    public TAVisualFractureEditor(ReadOnlyTargetRules Target) : base(Target)
    {
        PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;
        CppStandard = CppStandardVersion.Cpp20;
        bEnableExceptions = true; // Converts core validation exceptions into Editor bake errors.

        PublicDependencyModuleNames.AddRange(new[]
        {
            "Core",
            "CoreUObject",
            "Engine",
            "TAVisualFractureCore",
            "TAVisualFractureRuntime"
        });

        PrivateDependencyModuleNames.AddRange(new[]
        {
            "AssetRegistry",
            "AssetTools",
            "GeometryCore",
            "Landscape",
            "LevelEditor",
            "MeshDescription",
            "ProceduralMeshComponent",
            "PropertyEditor",
            "Slate",
            "SlateCore",
            "StaticMeshDescription",
            "UnrealEd"
        });
    }
}
