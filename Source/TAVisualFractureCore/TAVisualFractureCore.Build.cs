using UnrealBuildTool;

// Integration scaffold. Standalone C++ is verified separately from UnrealBuildTool.
public class TAVisualFractureCore : ModuleRules
{
    public TAVisualFractureCore(ReadOnlyTargetRules Target) : base(Target)
    {
        PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;
        CppStandard = CppStandardVersion.Cpp20;
        bUseUnity = false;
        bEnableExceptions = true; // Reference baker reports invalid input with std::exception.
        PublicDependencyModuleNames.Add("Core");
        PrivateDependencyModuleNames.Add("GeometryCore");
    }
}
