#pragma once

#include "CoreMinimal.h"

class UTAVisualFractureComponent;

class FTAVisualFractureLandscapeService
{
public:
    static bool Capture(UTAVisualFractureComponent* Component);
    static bool ValidateCurrent(const UTAVisualFractureComponent* Component, FString& OutError);
};
