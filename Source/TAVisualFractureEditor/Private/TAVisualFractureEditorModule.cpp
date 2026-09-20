#include "Modules/ModuleManager.h"

#include "TAVisualFractureBakeService.h"
#include "TAVisualFractureComponent.h"
#include "TAVisualFractureComponentDetails.h"
#include "TAVisualFracturePreviewManager.h"

#include "PropertyEditorModule.h"

class FTAVisualFractureEditorModule : public IModuleInterface
{
public:
    virtual void StartupModule() override
    {
        FPropertyEditorModule& PropertyEditor = FModuleManager::LoadModuleChecked<FPropertyEditorModule>(TEXT("PropertyEditor"));
        PropertyEditor.RegisterCustomClassLayout(UTAVisualFractureComponent::StaticClass()->GetFName(),
            FOnGetDetailCustomizationInstance::CreateStatic(&FTAVisualFractureComponentDetails::MakeInstance));
        PropertyEditor.NotifyCustomizationModuleChanged();
        FTAVisualFracturePreviewManager::Get().Startup();
    }

    virtual void ShutdownModule() override
    {
        FTAVisualFracturePreviewManager::Get().Shutdown();
        FTAVisualFractureBakeService::Get().CancelAllAndWait();
        if (FModuleManager::Get().IsModuleLoaded(TEXT("PropertyEditor")))
        {
            FPropertyEditorModule& PropertyEditor = FModuleManager::GetModuleChecked<FPropertyEditorModule>(TEXT("PropertyEditor"));
            PropertyEditor.UnregisterCustomClassLayout(UTAVisualFractureComponent::StaticClass()->GetFName());
            PropertyEditor.NotifyCustomizationModuleChanged();
        }
    }
};

IMPLEMENT_MODULE(FTAVisualFractureEditorModule, TAVisualFractureEditor)
