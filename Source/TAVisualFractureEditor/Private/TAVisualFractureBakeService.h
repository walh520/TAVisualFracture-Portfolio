#pragma once

#include "CoreMinimal.h"

class UTAVisualFractureComponent;

class FTAVisualFractureBakeService
{
public:
    static FTAVisualFractureBakeService& Get();

    void StartBake(UTAVisualFractureComponent* Component);
    void CancelBake(UTAVisualFractureComponent* Component);
    void CancelAllAndWait();

    static FString BuildBakeSignature(const UTAVisualFractureComponent* Component, FString* OutError = nullptr);

private:
    struct FBakeJob;
    TMap<TWeakObjectPtr<UTAVisualFractureComponent>, TSharedPtr<FBakeJob, ESPMode::ThreadSafe>> ActiveJobs;
};
