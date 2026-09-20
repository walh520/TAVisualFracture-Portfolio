#pragma once
#include "CoreMinimal.h"
#include "TAVisualFractureCollisionTypes.generated.h"

USTRUCT()
struct TAVISUALFRACTURERUNTIME_API FTVFVisualCollisionSettings
{
    GENERATED_BODY()
    UPROPERTY(EditAnywhere, Category="Collision", meta=(DisplayName="落地后 XY 避让", ToolTip="空中不做自碰撞或外部碰撞；落地后圆形占位避让，外部 Box 简化为 XY 矩形。不堆叠，不修改烘焙。")) bool bSimpleVisual = true;
    UPROPERTY(EditAnywhere, Category="Ground Avoidance", meta=(ClampMin="1",EditCondition="bSimpleVisual")) int32 GroundAvoidanceIterations = 4;
    UPROPERTY(EditAnywhere, Category="Ground Avoidance", meta=(DisplayName="避让修正速度",ClampMin="0.0",EditCondition="bSimpleVisual",ToolTip="单位 cm/s。0 为立即分离；正值限制每块每步累计推开距离。不限制物体真实速度，不会触发预算暂停。旧实例已保存的值保留。")) double GroundAvoidanceSpeedCmPerSecond = 0.0;
    UPROPERTY(EditAnywhere, Category="Ground Avoidance", meta=(DisplayName="碎块 XY 碰撞大小",ClampMin="0.05",EditCondition="bSimpleVisual",ToolTip="缩放碎块圆形占位，减小间距；不改变地面高度或外部 Box 本身。缩小可能造成视觉网格相交。")) double XYFootprintScale = 1.0;
    UPROPERTY(EditAnywhere, Category="Ground Avoidance", meta=(DisplayName="地面视觉滚动",EditCondition="bSimpleVisual")) bool bGroundVisualRoll = true;
    UPROPERTY(EditAnywhere, Category="Ground Avoidance", meta=(DisplayName="地面滚动强度",ClampMin="0.0",EditCondition="bSimpleVisual && bGroundVisualRoll",ToolTip="沿真实切向速度附加姿态翻滚，不写入角速度。0 关闭；低速和休眠停止。")) double GroundRollStrength = 0.35;
    UPROPERTY(EditAnywhere, Category="Ground Avoidance", meta=(DisplayName="外部撞击速度传递系数",ClampMin="0.0",EditCondition="bSimpleVisual")) double ExternalVelocityTransfer = 1.0;
    UPROPERTY(EditAnywhere, Category="Ground Avoidance", meta=(DisplayName="外部撞击弹起比例",ClampMin="0.0",EditCondition="bSimpleVisual")) double ExternalLiftRatio = 0.15;
    UPROPERTY(EditAnywhere, Category="Ground Avoidance", meta=(DisplayName="外部撞击弹起阈值",ClampMin="0.0",Units="cm/s",EditCondition="bSimpleVisual")) double ExternalLiftThreshold = 20.0;
    UPROPERTY(EditAnywhere, Category="Ground Avoidance", meta=(DisplayName="地面摩擦衰减率",ClampMin="0.0",EditCondition="bSimpleVisual",ToolTip="每秒衰减率，与摩擦系数相乘。越小滑行越远。")) double GroundFrictionDecay = 6.0;
    UPROPERTY(EditAnywhere, Category="Collision") bool bSelfCollision = false;
    UPROPERTY(EditAnywhere, Category="Collision") bool bExternalBoxCollision = false;
    UPROPERTY(EditAnywhere, Category="Proxy", meta=(ClampMin="1", EditCondition="!bSimpleVisual", ToolTip="仅严格模式使用多凸体分解；落地避让模式不使用凸体。")) int32 MaxHullsPerChunk = 8;
    UPROPERTY(EditAnywhere, Category="Proxy", meta=(EditCondition="!bSimpleVisual",ClampMin="4", ToolTip="仅严格模式使用此凸包顶点预算；落地避让不生成凸包。")) int32 MaxVerticesPerHull = 64;
    UPROPERTY(EditAnywhere, Category="Proxy", meta=(EditCondition="!bSimpleVisual",ClampMin="0.0")) double SurfaceToleranceCm = 0.2;
    UPROPERTY(EditAnywhere, Category="Proxy", meta=(EditCondition="!bSimpleVisual",ClampMin="0.0")) double RelativeSurfaceTolerance = 0.01;
    UPROPERTY(EditAnywhere, Category="Proxy", meta=(EditCondition="!bSimpleVisual",ClampMin="0.0")) double RelativeVolumeTolerance = 0.05;
    UPROPERTY(EditAnywhere, Category="Solver", meta=(EditCondition="!bSimpleVisual",ClampMin="1",ClampMax="8")) int32 Substeps = 4;
    UPROPERTY(EditAnywhere, Category="Solver", meta=(EditCondition="!bSimpleVisual",ClampMin="1")) int32 PositionIterations = 8;
    UPROPERTY(EditAnywhere, Category="Solver", meta=(EditCondition="!bSimpleVisual",ClampMin="1")) int32 VelocityIterations = 2;
    UPROPERTY(EditAnywhere, Category="Solver", meta=(EditCondition="!bSimpleVisual",ClampMin="0.0")) double ContactOffsetCm = 0.2;
    UPROPERTY(EditAnywhere, Category="Solver", meta=(EditCondition="!bSimpleVisual",ClampMin="0.0")) double RestOffsetCm = 0.0;
    UPROPERTY(EditAnywhere, Category="Solver", meta=(EditCondition="!bSimpleVisual",ClampMin="0.0")) double AllowedPenetrationCm = 0.2;
    UPROPERTY(EditAnywhere, Category="Solver", meta=(EditCondition="!bSimpleVisual",ClampMin="0.0")) double Compliance = 0.0;
    UPROPERTY(EditAnywhere, Category="Budget", meta=(EditCondition="!bSimpleVisual",ClampMin="1")) int32 MaxCandidatePairs = 100000;
    UPROPERTY(EditAnywhere, Category="Budget", meta=(EditCondition="!bSimpleVisual",ClampMin="1")) int32 MaxShapePairs = 100000;
    UPROPERTY(EditAnywhere, Category="Budget", meta=(EditCondition="!bSimpleVisual",ClampMin="1")) int32 MaxContactPoints = 16384;
    UPROPERTY(EditAnywhere, Category="Filter") int32 CollisionMask = -1;
};

USTRUCT()
struct TAVISUALFRACTURERUNTIME_API FTVFManualCollisionBox
{
    GENERATED_BODY()
    UPROPERTY(EditAnywhere, Category="Box") FVector Center = FVector::ZeroVector;
    UPROPERTY(EditAnywhere, Category="Box") FRotator Rotation = FRotator::ZeroRotator;
    UPROPERTY(EditAnywhere, Category="Box", meta=(ClampMin="0.001")) FVector HalfExtent = FVector(50.0);
};
