#pragma once

#include "CoreMinimal.h"
#include "Engine/StaticMesh.h"
#include "TAVisualFractureTypes.h"
#include "TAVisualFractureAsset.generated.h"

UCLASS(BlueprintType)
class TAVISUALFRACTURERUNTIME_API UTAVisualFractureAsset : public UObject
{
    GENERATED_BODY()

public:
    virtual void Serialize(FArchive& Ar) override;
    virtual void PostLoad() override;

    UPROPERTY(VisibleAnywhere, Category="Version")
    int32 BakeVersion = 3;

    UPROPERTY(VisibleAnywhere, Category="Version")
    bool bLegacyRebakeRequired = false;

    UPROPERTY(VisibleAnywhere, Category="Source")
    TSoftObjectPtr<UStaticMesh> SourceMesh;

    UPROPERTY(VisibleAnywhere, Category="Source")
    FString BakeSignature;

    UPROPERTY(VisibleAnywhere, Category="Source")
    FString SourceGeometryHash;

    UPROPERTY(VisibleAnywhere, Category="Source")
    FString TopologySettingsHash;

    UPROPERTY(VisibleAnywhere, Category="Source")
    FVector AppliedPositiveScale = FVector::OneVector;

    UPROPERTY(VisibleAnywhere, Category="Source")
    int32 SourceTriangleCount = 0;

    UPROPERTY(VisibleAnywhere, Category="Bake")
    FTVFBakeSettings BakeSettings;

    UPROPERTY(VisibleAnywhere, Category="Bake")
    FTVFCoordinateContract CoordinateContract;

    UPROPERTY(VisibleAnywhere, Category="Bake")
    int32 FieldGridNx = 0;

    UPROPERTY(VisibleAnywhere, Category="Bake")
    int32 FieldGridNy = 0;

    UPROPERTY(VisibleAnywhere, Category="Bake")
    int32 FieldGridNz = 0;

    UPROPERTY(VisibleAnywhere, Category="Bake")
    double FieldGridStepCm = 0.0;

    UPROPERTY(VisibleAnywhere, Category="Bake")
    TArray<FVector> SeedPositions;

    UPROPERTY(VisibleAnywhere, Category="Bake")
    TArray<double> SeedWeights;

    UPROPERTY(VisibleAnywhere, Category="Bake")
    TArray<FTVFChunkRenderData> Chunks;

    UPROPERTY(VisibleAnywhere, Category="Bake")
    TArray<FTVFBondData> Bonds;

    UPROPERTY(VisibleAnywhere, Category="Bake")
    TArray<FTVFCanonicalVertexData> CanonicalVertices;

    UPROPERTY(VisibleAnywhere, Category="Bake")
    TArray<FTVFInterfacePatchData> InterfacePatches;

    UPROPERTY(VisibleAnywhere, Category="Diagnostics")
    TArray<FTVFDiagnostic> Diagnostics;

    UPROPERTY(VisibleAnywhere, Category="Statistics")
    FTVFBakePerformanceStats PerformanceStats;

    UPROPERTY(VisibleAnywhere, Category="Bake")
    bool bHasCenterInteriorPoint = false;

    UPROPERTY(VisibleAnywhere, Category="Bake")
    FVector CenterInteriorPoint = FVector::ZeroVector;

    UPROPERTY(VisibleAnywhere, Category="Statistics")
    double InputVolumeCm3 = 0.0;

    UPROPERTY(VisibleAnywhere, Category="Statistics")
    double OutputVolumeCm3 = 0.0;

    UPROPERTY(VisibleAnywhere, Category="Statistics")
    double DiscardedVolumeCm3 = 0.0;

    UPROPERTY(VisibleAnywhere, Category="Statistics")
    int32 BakedTriangleCount = 0;

    bool HasUsableTopology(FString* OutReason = nullptr) const;
};
