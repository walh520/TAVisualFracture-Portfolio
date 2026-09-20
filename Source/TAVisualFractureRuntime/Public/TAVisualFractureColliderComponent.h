#pragma once
#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "TAVisualFractureCollisionTypes.h"
#include "TAVisualFractureColliderComponent.generated.h"
class UStaticMeshComponent;

UCLASS(ClassGroup=(Rendering), meta=(BlueprintSpawnableComponent))
class TAVISUALFRACTURERUNTIME_API UTAVisualFractureColliderComponent : public UActorComponent
{
    GENERATED_BODY()
public:
    UPROPERTY(EditInstanceOnly, Category="Collision", meta=(ToolTip="静态目标；留空自动选择本 Actor 的第一个 StaticMeshComponent。仅编辑器视觉预览使用，不修改 Chaos 碰撞。"))
    TObjectPtr<UStaticMeshComponent> TargetMeshComponent;
    UPROPERTY(EditAnywhere, Category="Collision") bool bEnabled = true;
    UPROPERTY(EditAnywhere, Category="Collision", meta=(ToolTip="关闭时只读取资源 BoxElems；没有 Box 必须配置，不使用整 Mesh 包围盒回退。")) bool bUseManualBoxes = false;
    UPROPERTY(EditAnywhere, Category="Collision", meta=(EditCondition="bUseManualBoxes")) TArray<FTVFManualCollisionBox> ManualBoxes;
    UPROPERTY(EditAnywhere, Category="Collision") int32 CollisionMask = -1;
    UPROPERTY(EditAnywhere, Category="Collision", meta=(ClampMin="0.0",ClampMax="1.0")) double Friction = 0.4;
    UPROPERTY(EditAnywhere, Category="Collision", meta=(ClampMin="0.0",ClampMax="1.0")) double Restitution = 0.15;
};
