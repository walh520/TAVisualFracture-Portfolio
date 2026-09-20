#pragma once

#include "CoreMinimal.h"
#include "Containers/Ticker.h"

class UTAVisualFractureComponent;
class FObjectPreSaveContext;

class FTAVisualFracturePreviewManager
{
public:
    static FTAVisualFracturePreviewManager& Get();

    void Startup();
    void Shutdown();
    bool ApplyPreset(UTAVisualFractureComponent* Component, uint8 PresetValue, FString& OutError);
    bool Break(UTAVisualFractureComponent* Component, FString& OutError);
    void Reset(UTAVisualFractureComponent* Component = nullptr);
    bool PrepareCollision(UTAVisualFractureComponent* Component, FString& OutError);
    bool ToggleCollisionProxies(UTAVisualFractureComponent* Component, FString& OutError);

private:
    struct FSession;
    FTAVisualFracturePreviewManager() = default;
    ~FTAVisualFracturePreviewManager();

    bool CreateSession(UTAVisualFractureComponent* Component, FString& OutError);
    void UpdateVisuals();
    bool Tick(float DeltaSeconds);
    void OnPreBeginPIE(bool bIsSimulating);
    void OnMapOpened(const FString& Filename, bool bAsTemplate);
    void OnMapChanged(uint32 MapChangeFlags);
    void OnPreSaveWorld(UWorld* World, FObjectPreSaveContext SaveContext);
    void OnPostUndoRedo();

    TUniquePtr<FSession> ActiveSession;
    FTSTicker::FDelegateHandle TickerHandle;
    FDelegateHandle PreBeginPIEHandle;
    FDelegateHandle MapOpenedHandle;
    FDelegateHandle MapChangedHandle;
    FDelegateHandle PreSaveWorldHandle;
    FDelegateHandle PostUndoRedoHandle;
    uint64 NextEventId = 1;
};
