#include "TAVisualFractureComponent.h"

#include "Components/StaticMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "GameFramework/Actor.h"
#if WITH_EDITOR
#include "UObject/UnrealType.h"
#endif

UTAVisualFractureComponent::UTAVisualFractureComponent()
{
    PrimaryComponentTick.bCanEverTick = false;
    bTickInEditor = false;
}

void UTAVisualFractureComponent::OnRegister()
{
    Super::OnRegister();
    if (!InstanceGuid.IsValid() && !HasAnyFlags(RF_ClassDefaultObject))
    {
        InstanceGuid = FGuid::NewGuid();
    }
}

void UTAVisualFractureComponent::PostDuplicate(const bool bDuplicateForPIE)
{
    Super::PostDuplicate(bDuplicateForPIE);
    if (!bDuplicateForPIE) ResetDuplicatedAuthoringIdentity();
}

#if WITH_EDITOR
void UTAVisualFractureComponent::PostEditImport()
{
    Super::PostEditImport();
    ResetDuplicatedAuthoringIdentity();
}

void UTAVisualFractureComponent::PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent)
{
    const FName PropertyName = PropertyChangedEvent.GetPropertyName();
    if (PropertyName == GET_MEMBER_NAME_CHECKED(FTVFImpactSettings, ImpactPointLocal) ||
        PropertyName == GET_MEMBER_NAME_CHECKED(FTVFImpactSettings, DirectionLocal))
    {
        Impact.Preset = ETVFImpactLocationPreset::Custom;
    }
    Super::PostEditChangeProperty(PropertyChangedEvent);
}
#endif

void UTAVisualFractureComponent::ResetDuplicatedAuthoringIdentity()
{
    InstanceGuid = FGuid::NewGuid();
    BakedAsset = nullptr;
    LandscapeSnapshot = {};
    Impact.ImpactRadiusCm = 0.0;
    Impact.BondRadiusCm = 0.0;
    Impact.ReferenceRadiusCm = 0.0;
    bBakeInProgress = false;
    BakeProgress = 0.0f;
    PreviewDetachedChunks = 0;
    PreviewBrokenBonds = 0;
    LastBakeStatus.Reset();
    LastCaptureStatus.Reset();
    LastPreviewStatus.Reset();
}

UStaticMeshComponent* UTAVisualFractureComponent::ResolveTargetMeshComponent() const
{
    if (IsValid(TargetMeshComponent))
    {
        return TargetMeshComponent;
    }
    return GetOwner() ? GetOwner()->FindComponentByClass<UStaticMeshComponent>() : nullptr;
}

double UTAVisualFractureComponent::ResolveCaptureHalfExtentCm() const
{
    if (CaptureHalfExtentOverrideCm > 0.0)
    {
        return CaptureHalfExtentOverrideCm;
    }
    const UStaticMeshComponent* Target = ResolveTargetMeshComponent();
    if (!Target)
    {
        return 1000.0;
    }
    const FVector Extent = Target->Bounds.BoxExtent;
    return FMath::Max(FMath::Max(Extent.X, Extent.Y) * 4.0, 1000.0);
}

bool UTAVisualFractureComponent::IsBakeCurrent(FString* OutReason) const
{
    const UStaticMeshComponent* Target = ResolveTargetMeshComponent();
    if (!Target || !Target->GetStaticMesh())
    {
        if (OutReason) *OutReason = TEXT("No target Static Mesh Component or Static Mesh.");
        return false;
    }
    if (!BakedAsset || !BakedAsset->HasUsableTopology(OutReason))
    {
        return false;
    }
    const FVector Scale = Target->GetComponentScale();
    if (Scale.X <= 0.0 || Scale.Y <= 0.0 || Scale.Z <= 0.0 ||
        !Scale.Equals(BakedAsset->AppliedPositiveScale, KINDA_SMALL_NUMBER) ||
        BakedAsset->SourceMesh.ToSoftObjectPath() != FSoftObjectPath(Target->GetStaticMesh()))
    {
        if (OutReason) *OutReason = TEXT("Target mesh or positive component scale differs from the bake signature.");
        return false;
    }
    return true;
}

bool UTAVisualFractureComponent::IsLandscapeCaptureCurrent(FString* OutReason) const
{
    const UStaticMeshComponent* Target = ResolveTargetMeshComponent();
    if (!Target || !LandscapeSnapshot.IsValid(OutReason))
    {
        return false;
    }
    if (!Target->GetComponentTransform().Equals(LandscapeSnapshot.CapturedTargetTransform, 0.01) ||
        !FMath::IsNearlyEqual(ResolveCaptureHalfExtentCm(), LandscapeSnapshot.CapturedHalfExtentCm, 0.01))
    {
        if (OutReason) *OutReason = TEXT("Target transform, scale or capture range changed after Landscape capture.");
        return false;
    }
    if (ExplicitLandscape && FSoftObjectPath(ExplicitLandscape) != LandscapeSnapshot.LandscapePath)
    {
        if (OutReason) *OutReason = TEXT("The selected Landscape differs from the captured Landscape.");
        return false;
    }
    return true;
}

void UTAVisualFractureComponent::InitializeImpactRadiiIfNeeded()
{
    if (!BakedAsset || BakedAsset->Chunks.IsEmpty())
    {
        return;
    }
    FBox Bounds(ForceInit);
    for (const FTVFChunkRenderData& Chunk : BakedAsset->Chunks)
    {
        for (const FVector& Vertex : Chunk.Exterior.Vertices) Bounds += Vertex + Chunk.Centroid;
        for (const FVector& Vertex : Chunk.Interior.Vertices) Bounds += Vertex + Chunk.Centroid;
    }
    const double Scale = FMath::Max3(Bounds.GetSize().X, Bounds.GetSize().Y, Bounds.GetSize().Z) / 100.0;
    if (Impact.ImpactRadiusCm <= 0.0) Impact.ImpactRadiusCm = 160.0 * Scale;
    if (Impact.BondRadiusCm <= 0.0) Impact.BondRadiusCm = 160.0 * Scale;
    if (Impact.ReferenceRadiusCm <= 0.0) Impact.ReferenceRadiusCm = 20.0 * Scale;
}
