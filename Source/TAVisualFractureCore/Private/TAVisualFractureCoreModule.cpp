#include "CompGeom/ExactPredicates.h"
#include "Modules/ModuleManager.h"

class FTAVisualFractureCoreModule final : public IModuleInterface
{
public:
    virtual void StartupModule() override
    {
        UE::Geometry::ExactPredicates::GlobalInit();
    }
};

IMPLEMENT_MODULE(FTAVisualFractureCoreModule, TAVisualFractureCore)
