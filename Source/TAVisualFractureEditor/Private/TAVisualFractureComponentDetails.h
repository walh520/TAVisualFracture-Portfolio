#pragma once

#include "IDetailCustomization.h"

class UTAVisualFractureComponent;

class FTAVisualFractureComponentDetails : public IDetailCustomization
{
public:
    static TSharedRef<IDetailCustomization> MakeInstance();
    virtual void CustomizeDetails(IDetailLayoutBuilder& DetailBuilder) override;

private:
    TWeakObjectPtr<UTAVisualFractureComponent> Component;
};
