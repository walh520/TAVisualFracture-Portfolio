#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "Materials/MaterialInterface.h"
#include "TAVisualFractureAsset.h"
#include "TAVisualFractureCollisionTypes.h"
#include "TAVisualFractureComponent.generated.h"

class UStaticMeshComponent;
class AActor;

UCLASS(ClassGroup=(Rendering), meta=(BlueprintSpawnableComponent))
class TAVISUALFRACTURERUNTIME_API UTAVisualFractureComponent : public UActorComponent
{
    GENERATED_BODY()

public:
    UTAVisualFractureComponent();

    UPROPERTY(EditInstanceOnly, Category="Target", meta=(ToolTip="要破碎的 Static Mesh Component；留空时自动使用 Actor 上第一个静态网格组件。"))
    TObjectPtr<UStaticMeshComponent> TargetMeshComponent;

    UPROPERTY(EditAnywhere, Category="Bake", meta=(ToolTip="保存烘焙出的碎块拓扑。首次烘焙会自动创建，之后会更新同一资产。"))
    TObjectPtr<UTAVisualFractureAsset> BakedAsset;

    UPROPERTY(EditAnywhere, Category="Bake", meta=(ToolTip="控制碎块数量、形状和随机性；修改后需要重新烘焙。"))
    FTVFBakeSettings BakeSettings;

    UPROPERTY(EditAnywhere, Category="Response", meta=(ToolTip="碎块质量、Bond 阻力、重力、接触和休眠参数。"))
    FTVFObjectResponseSettings ObjectResponse;

    UPROPERTY(EditAnywhere, Category="Visual Collision", meta=(ToolTip="可选视觉凸代理接触，不改变切割烘焙。准备失败只阻止碰撞预览。"))
    FTVFVisualCollisionSettings VisualCollision;

    UPROPERTY(VisibleInstanceOnly, Transient, Category="Status")
    FString LastCollisionStatus;

    UPROPERTY(EditAnywhere, Category="Performance", meta=(ToolTip="只约束编辑器运动预览，不影响烘焙是否成功。"))
    FTVFPhysicsPerformanceBudget PhysicsBudget;

    UPROPERTY(EditAnywhere, Category="Performance", meta=(ToolTip="只约束当前每 Chunk 一个 PMC 的兼容预览，不影响烘焙是否成功。"))
    FTVFRenderPerformanceBudget RenderBudget;

    UPROPERTY(EditAnywhere, Category="Landscape", meta=(ToolTip="LandscapeAnchored 只释放与支撑断开的碎块；FreeObject 第一次冲击会释放全部碎块。"))
    ETVFSupportMode SupportMode = ETVFSupportMode::LandscapeAnchored;

    // Kept as AActor so the Runtime module has no Landscape dependency. The Editor service validates the class.
    UPROPERTY(EditInstanceOnly, Category="Landscape", meta=(ToolTip="要采样的 Landscape。未指定时，仅在捕获范围内唯一的 Landscape 会被自动使用。"))
    TObjectPtr<AActor> ExplicitLandscape;

    UPROPERTY(EditAnywhere, Category="Landscape", meta=(ClampMin="0.0", ToolTip="Landscape 捕获半范围，单位 cm。0 表示按目标尺寸自动计算。"))
    double CaptureHalfExtentOverrideCm = 0.0;

    UPROPERTY(VisibleInstanceOnly, Category="Landscape", meta=(ToolTip="已捕获的局部地形高度快照。目标变换、范围或 Landscape 改变后会失效。"))
    FTVFLandscapeSnapshot LandscapeSnapshot;

    UPROPERTY(EditAnywhere, Category="Impact", meta=(ToolTip="单次破碎事件的冲击位置、半径、冲量和方向权重。重复点击破碎会累积 Bond 损伤。"))
    FTVFImpactSettings Impact;

    UPROPERTY(EditAnywhere, Category="Rendering", meta=(ToolTip="覆盖碎块外表材质；未指定时使用调试外表材质。"))
    TObjectPtr<UMaterialInterface> ExteriorMaterialOverride;

    UPROPERTY(EditAnywhere, Category="Rendering", meta=(ToolTip="覆盖碎块断面材质；未指定时使用调试断面材质。"))
    TObjectPtr<UMaterialInterface> InteriorMaterialOverride;

    UPROPERTY(VisibleInstanceOnly, Category="Status", meta=(ToolTip="组件实例的唯一标识，用于生成并关联其独立烘焙资产。"))
    FGuid InstanceGuid;

    UPROPERTY(VisibleInstanceOnly, Transient, Category="Status", meta=(ToolTip="是否正在后台计算碎块拓扑。"))
    bool bBakeInProgress = false;

    UPROPERTY(VisibleInstanceOnly, Transient, Category="Status", meta=(ClampMin="0.0", ClampMax="1.0", ToolTip="当前烘焙进度，范围 0 到 1。"))
    float BakeProgress = 0.0f;

    UPROPERTY(VisibleInstanceOnly, Transient, Category="Status", meta=(ToolTip="最近一次烘焙的结果或失败原因。"))
    FString LastBakeStatus;

    UPROPERTY(VisibleInstanceOnly, Transient, Category="Status", meta=(ToolTip="最近一次 Landscape 捕获的结果或失败原因。"))
    FString LastCaptureStatus;

    UPROPERTY(VisibleInstanceOnly, Transient, Category="Status", meta=(ToolTip="当前预览状态或最近一次预览错误。"))
    FString LastPreviewStatus;

    UPROPERTY(VisibleInstanceOnly, Transient, Category="Status", meta=(ToolTip="当前预览中已与支撑断开的碎块数。"))
    int32 PreviewDetachedChunks = 0;

    UPROPERTY(VisibleInstanceOnly, Transient, Category="Status", meta=(ToolTip="当前预览中已断开的 Bond 数量。"))
    int32 PreviewBrokenBonds = 0;

    UStaticMeshComponent* ResolveTargetMeshComponent() const;
    double ResolveCaptureHalfExtentCm() const;
    bool IsBakeCurrent(FString* OutReason = nullptr) const;
    bool IsLandscapeCaptureCurrent(FString* OutReason = nullptr) const;
    void InitializeImpactRadiiIfNeeded();

protected:
    virtual void OnRegister() override;
    virtual void PostDuplicate(bool bDuplicateForPIE) override;
#if WITH_EDITOR
    virtual void PostEditImport() override;
    virtual void PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent) override;
#endif

private:
    void ResetDuplicatedAuthoringIdentity();
};
