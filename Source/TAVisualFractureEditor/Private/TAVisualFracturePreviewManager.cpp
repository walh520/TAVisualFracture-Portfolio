#include "TAVisualFracturePreviewManager.h"

#include "TAVisualFractureBakeService.h"
#include "TAVisualFractureComponent.h"
#include "TAVisualFractureLandscapeService.h"
#include "TVFMotion.h"
#include "TAVisualFractureCollisionService.h"

#include "Components/StaticMeshComponent.h"
#include "Editor.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "Materials/Material.h"
#include "ProceduralMeshComponent.h"
#include "ScopedTransaction.h"
#include "UObject/ObjectSaveContext.h"
#include "UObject/StrongObjectPtr.h"

#include <memory>
#include <vector>

namespace
{
tvf::Vec3 ToPreviewTVF(const FVector& V)
{
    return {V.X, V.Y, V.Z};
}

tvf::Quat ToPreviewTVF(const FQuat& Q)
{
    return {Q.X, Q.Y, Q.Z, Q.W};
}

FVector ToPreviewUE(const tvf::Vec3& V)
{
    return FVector(V.x, V.y, V.z);
}

FQuat ToPreviewUE(const tvf::Quat& Q)
{
    return FQuat(Q.x, Q.y, Q.z, Q.w);
}

void BuildRestUVs(const FTVFChunkMeshSection& Section, TArray<FVector2D>& UV1,
                  TArray<FVector2D>& UV2, TArray<FVector2D>& UV3)
{
    UV1.Reserve(Section.RestSpaceData.Num());
    UV2.Reserve(Section.RestSpaceData.Num());
    UV3.Reserve(Section.RestSpaceData.Num());
    for (const FVector4& Rest : Section.RestSpaceData)
    {
        UV1.Add(FVector2D(Rest.X, Rest.Y));
        UV2.Add(FVector2D(Rest.X, Rest.Z));
        UV3.Add(FVector2D(Rest.Y, Rest.Z));
    }
}

void CreateSection(UProceduralMeshComponent* PMC, const int32 SectionIndex,
                   const FTVFChunkMeshSection& Section)
{
    if (!PMC || Section.Vertices.IsEmpty())
    {
        return;
    }
    TArray<FProcMeshTangent> Tangents;
    Tangents.Reserve(Section.Tangents.Num());
    for (const FVector& Tangent : Section.Tangents)
    {
        Tangents.Emplace(Tangent, false);
    }
    TArray<FVector2D> UV1, UV2, UV3;
    BuildRestUVs(Section, UV1, UV2, UV3);
    PMC->CreateMeshSection_LinearColor(SectionIndex, Section.Vertices, Section.Triangles,
        Section.Normals, Section.UV0, UV1, UV2, UV3, Section.VertexColors, Tangents,
        false, false);
}

UMaterialInterface* ResolveDebugMaterial()
{
    static TWeakObjectPtr<UMaterialInterface> Cached;
    if (!Cached.IsValid())
    {
        Cached = LoadObject<UMaterialInterface>(nullptr,
            TEXT("/Engine/EngineDebugMaterials/VertexColorMaterial.VertexColorMaterial"));
    }
    return Cached.IsValid() ? Cached.Get() : UMaterial::GetDefaultMaterial(MD_Surface);
}

bool RayTriangle(const FVector& Origin, const FVector& Direction, const FVector& A,
                 const FVector& B, const FVector& C, double& OutDistance)
{
    const FVector E1 = B - A;
    const FVector E2 = C - A;
    const FVector H = FVector::CrossProduct(Direction, E2);
    const double Determinant = FVector::DotProduct(E1, H);
    if (FMath::Abs(Determinant) <= UE_DOUBLE_SMALL_NUMBER) return false;
    const double Inv = 1.0 / Determinant;
    const FVector S = Origin - A;
    const double U = Inv * FVector::DotProduct(S, H);
    if (U < 0.0 || U > 1.0) return false;
    const FVector Q = FVector::CrossProduct(S, E1);
    const double V = Inv * FVector::DotProduct(Direction, Q);
    if (V < 0.0 || U + V > 1.0) return false;
    const double T = Inv * FVector::DotProduct(E2, Q);
    if (T <= UE_DOUBLE_SMALL_NUMBER) return false;
    OutDistance = T;
    return true;
}

bool FindFirstExteriorHit(const UTAVisualFractureAsset* Asset, const FVector& Origin,
                          const FVector& Direction, FVector& OutPoint)
{
    double Best = TNumericLimits<double>::Max();
    bool bFound = false;
    for (const FTVFChunkRenderData& Chunk : Asset->Chunks)
    {
        const FTVFChunkMeshSection& Section = Chunk.Exterior;
        for (int32 Index = 0; Index + 2 < Section.Triangles.Num(); Index += 3)
        {
            const int32 I0 = Section.Triangles[Index];
            const int32 I1 = Section.Triangles[Index + 1];
            const int32 I2 = Section.Triangles[Index + 2];
            if (!Section.Vertices.IsValidIndex(I0) || !Section.Vertices.IsValidIndex(I1) ||
                !Section.Vertices.IsValidIndex(I2)) continue;
            double Distance = 0.0;
            if (RayTriangle(Origin, Direction, Section.Vertices[I0] + Chunk.Centroid,
                Section.Vertices[I1] + Chunk.Centroid, Section.Vertices[I2] + Chunk.Centroid, Distance) &&
                Distance < Best)
            {
                Best = Distance;
                bFound = true;
            }
        }
    }
    if (bFound) OutPoint = Origin + Direction * Best;
    return bFound;
}

FBox BuildAssetBounds(const UTAVisualFractureAsset* Asset)
{
    FBox Bounds(ForceInit);
    for (const FTVFChunkRenderData& Chunk : Asset->Chunks)
    {
        for (const FVector& P : Chunk.Exterior.Vertices) Bounds += P + Chunk.Centroid;
        for (const FVector& P : Chunk.Interior.Vertices) Bounds += P + Chunk.Centroid;
    }
    return Bounds;
}

bool ResponseEquals(const FTVFObjectResponseSettings& A, const FTVFObjectResponseSettings& B)
{
    return A.MassPerCm3 == B.MassPerCm3 && A.BondResistancePerCm2 == B.BondResistancePerCm2 &&
        A.LinearDragPerSecond == B.LinearDragPerSecond && A.AngularDragPerSecond == B.AngularDragPerSecond &&
        A.GravityWorld == B.GravityWorld && A.Restitution == B.Restitution && A.Friction == B.Friction &&
        A.ContactAngularDampingPerSecond == B.ContactAngularDampingPerSecond &&
        A.RestitutionVelocityThresholdCmPerSecond == B.RestitutionVelocityThresholdCmPerSecond &&
        A.SleepLinearSpeedCmPerSecond == B.SleepLinearSpeedCmPerSecond &&
        A.SleepAngularSpeedRadPerSecond == B.SleepAngularSpeedRadPerSecond &&
        A.SleepDelaySeconds == B.SleepDelaySeconds && A.FixedTimeStepSeconds == B.FixedTimeStepSeconds;
}

bool ValidateResponse(const FTVFObjectResponseSettings& R, FString& OutError)
{
    const bool bFinite = FMath::IsFinite(R.MassPerCm3) && FMath::IsFinite(R.BondResistancePerCm2) &&
        FMath::IsFinite(R.LinearDragPerSecond) && FMath::IsFinite(R.AngularDragPerSecond) &&
        !R.GravityWorld.ContainsNaN() && FMath::IsFinite(R.Restitution) && FMath::IsFinite(R.Friction) &&
        FMath::IsFinite(R.ContactAngularDampingPerSecond) &&
        FMath::IsFinite(R.RestitutionVelocityThresholdCmPerSecond) &&
        FMath::IsFinite(R.SleepLinearSpeedCmPerSecond) && FMath::IsFinite(R.SleepAngularSpeedRadPerSecond) &&
        FMath::IsFinite(R.SleepDelaySeconds) && FMath::IsFinite(R.FixedTimeStepSeconds);
    const bool bRange = R.MassPerCm3 > 0.0 && R.BondResistancePerCm2 > 0.0 &&
        R.LinearDragPerSecond >= 0.0 && R.AngularDragPerSecond >= 0.0 &&
        R.Restitution >= 0.0 && R.Restitution <= 1.0 && R.Friction >= 0.0 && R.Friction <= 1.0 &&
        R.ContactAngularDampingPerSecond >= 0.0 && R.RestitutionVelocityThresholdCmPerSecond >= 0.0 &&
        R.SleepLinearSpeedCmPerSecond >= 0.0 && R.SleepAngularSpeedRadPerSecond >= 0.0 &&
        R.SleepDelaySeconds >= 0.0 && R.FixedTimeStepSeconds > 0.0;
    if (!bFinite || !bRange)
    {
        OutError = TEXT("Object response contains non-finite or out-of-range mass, Bond, motion, contact or sleep values.");
        return false;
    }
    return true;
}

bool ValidateImpact(const FTVFImpactSettings& I, FString& OutError)
{
    const bool bFinite = !I.ImpactPointLocal.ContainsNaN() && !I.DirectionLocal.ContainsNaN() &&
        FMath::IsFinite(I.ImpactRadiusCm) && FMath::IsFinite(I.BondRadiusCm) && FMath::IsFinite(I.Impulse) &&
        FMath::IsFinite(I.BondDamage) && FMath::IsFinite(I.FalloffExponent) &&
        FMath::IsFinite(I.ReferenceRadiusCm) && FMath::IsFinite(I.SizeSpeedBias) &&
        FMath::IsFinite(I.RadialWeight) && FMath::IsFinite(I.DirectionalWeight) &&
        FMath::IsFinite(I.UpwardWeight) && FMath::IsFinite(I.RandomWeight);
    if (!bFinite || I.ImpactRadiusCm <= 0.0 || I.BondRadiusCm < 0.0 || I.Impulse < 0.0 ||
        I.BondDamage < 0.0 || I.FalloffExponent < 0.0 || I.ReferenceRadiusCm <= 0.0)
    {
        OutError = TEXT("Impact contains non-finite or out-of-range radii, impulse, damage, falloff or direction weights.");
        return false;
    }
    return true;
}
}

struct FTAVisualFracturePreviewManager::FSession
{
    TWeakObjectPtr<UTAVisualFractureComponent> Component;
    TWeakObjectPtr<UStaticMeshComponent> Target;
    TArray<TStrongObjectPtr<UProceduralMeshComponent>> ChunkComponents;
    std::unique_ptr<tvf::MotionSystem> Motion;
    TUniquePtr<FTVFCollisionPreparation> Collision;
    FString CollisionSettingsText;
    bool bShowCollisionProxies = false;
    bool bFrozenCollisionPreview = false;
    FString LastLoggedCollisionReason;
    FTransform InitialTargetTransform = FTransform::Identity;
    ETVFSupportMode SupportMode = ETVFSupportMode::LandscapeAnchored;
    FTVFObjectResponseSettings Response;
    TWeakObjectPtr<UTAVisualFractureAsset> Asset;
    TWeakObjectPtr<UMaterialInterface> ExteriorMaterial;
    TWeakObjectPtr<UMaterialInterface> InteriorMaterial;
    bool bOriginalVisible = true;
    bool bOriginalHiddenInGame = false;
    bool bSwitchedToChunks = false;
};

FTAVisualFracturePreviewManager& FTAVisualFracturePreviewManager::Get()
{
    static FTAVisualFracturePreviewManager Instance;
    return Instance;
}

FTAVisualFracturePreviewManager::~FTAVisualFracturePreviewManager() = default;

void FTAVisualFracturePreviewManager::Startup()
{
    TickerHandle = FTSTicker::GetCoreTicker().AddTicker(
        FTickerDelegate::CreateRaw(this, &FTAVisualFracturePreviewManager::Tick));
    PreBeginPIEHandle = FEditorDelegates::PreBeginPIE.AddRaw(this, &FTAVisualFracturePreviewManager::OnPreBeginPIE);
    MapOpenedHandle = FEditorDelegates::OnMapOpened.AddRaw(this, &FTAVisualFracturePreviewManager::OnMapOpened);
    MapChangedHandle = FEditorDelegates::MapChange.AddRaw(this, &FTAVisualFracturePreviewManager::OnMapChanged);
    PreSaveWorldHandle = FEditorDelegates::PreSaveWorldWithContext.AddRaw(this, &FTAVisualFracturePreviewManager::OnPreSaveWorld);
    PostUndoRedoHandle = FEditorDelegates::PostUndoRedo.AddRaw(this, &FTAVisualFracturePreviewManager::OnPostUndoRedo);
}

void FTAVisualFracturePreviewManager::Shutdown()
{
    Reset();
    if (TickerHandle.IsValid()) FTSTicker::GetCoreTicker().RemoveTicker(TickerHandle);
    FEditorDelegates::PreBeginPIE.Remove(PreBeginPIEHandle);
    FEditorDelegates::OnMapOpened.Remove(MapOpenedHandle);
    FEditorDelegates::MapChange.Remove(MapChangedHandle);
    FEditorDelegates::PreSaveWorldWithContext.Remove(PreSaveWorldHandle);
    FEditorDelegates::PostUndoRedo.Remove(PostUndoRedoHandle);
}

bool FTAVisualFracturePreviewManager::ApplyPreset(UTAVisualFractureComponent* Component,
                                                  const uint8 PresetValue, FString& OutError)
{
    if (!IsValid(Component) || !Component->BakedAsset || !Component->IsBakeCurrent(&OutError) ||
        !Component->BakedAsset->HasUsableTopology(&OutError) ||
        FTAVisualFractureBakeService::BuildBakeSignature(Component, &OutError) != Component->BakedAsset->BakeSignature)
    {
        if (OutError.IsEmpty()) OutError = TEXT("Mesh content or bake settings changed; rebake before selecting a preset.");
        return false;
    }
    const ETVFImpactLocationPreset Preset = static_cast<ETVFImpactLocationPreset>(PresetValue);
    const FBox Bounds = BuildAssetBounds(Component->BakedAsset);
    if (!Bounds.IsValid)
    {
        OutError = TEXT("Baked chunk bounds are invalid.");
        return false;
    }

    FVector Point = FVector::ZeroVector;
    FVector Direction = FVector::ForwardVector;
    const FVector Center = Bounds.GetCenter();
    const double Margin = FMath::Max3(Bounds.GetSize().X, Bounds.GetSize().Y, Bounds.GetSize().Z) + 1.0;
    if (Preset == ETVFImpactLocationPreset::Top)
    {
        const FVector Origin(Center.X, Center.Y, Bounds.Max.Z + Margin);
        Direction = FVector(0, 0, -1);
        if (!FindFirstExteriorHit(Component->BakedAsset, Origin, Direction, Point))
        {
            OutError = TEXT("Top preset ray did not hit the baked exterior; no fallback point was fabricated.");
            return false;
        }
    }
    else if (Preset == ETVFImpactLocationPreset::SideMiddle)
    {
        FVector Origin = Center;
        switch (Component->Impact.SideAxis)
        {
        case ETVFSideAxis::PositiveX: Origin.X = Bounds.Max.X + Margin; Direction = FVector(-1, 0, 0); break;
        case ETVFSideAxis::NegativeX: Origin.X = Bounds.Min.X - Margin; Direction = FVector(1, 0, 0); break;
        case ETVFSideAxis::PositiveY: Origin.Y = Bounds.Max.Y + Margin; Direction = FVector(0, -1, 0); break;
        case ETVFSideAxis::NegativeY: Origin.Y = Bounds.Min.Y - Margin; Direction = FVector(0, 1, 0); break;
        }
        if (!FindFirstExteriorHit(Component->BakedAsset, Origin, Direction, Point))
        {
            OutError = TEXT("Side-middle preset ray did not hit the baked exterior; no fallback point was fabricated.");
            return false;
        }
    }
    else if (Preset == ETVFImpactLocationPreset::CenterInterior)
    {
        if (!Component->BakedAsset->bHasCenterInteriorPoint)
        {
            OutError = TEXT("Bake asset has no valid negative-SDF center sample.");
            return false;
        }
        Point = Component->BakedAsset->CenterInteriorPoint;
        Direction = FVector::ForwardVector;
    }
    else
    {
        OutError = TEXT("Custom is edited directly and has no preset button.");
        return false;
    }

    const FScopedTransaction Transaction(NSLOCTEXT("TAVisualFracture", "ApplyImpactPreset", "Apply TA Visual Fracture Impact Preset"));
    Component->Modify();
    Component->Impact.Preset = Preset;
    Component->Impact.ImpactPointLocal = Point;
    Component->Impact.DirectionLocal = Direction;
    if (Preset == ETVFImpactLocationPreset::Top)
    {
        Component->Impact.RadialWeight = 0.35;
        Component->Impact.DirectionalWeight = 1.0;
        Component->Impact.UpwardWeight = 0.0;
        Component->Impact.RandomWeight = 0.10;
    }
    else if (Preset == ETVFImpactLocationPreset::SideMiddle)
    {
        Component->Impact.RadialWeight = 0.70;
        Component->Impact.DirectionalWeight = 0.80;
        Component->Impact.UpwardWeight = 0.10;
        Component->Impact.RandomWeight = 0.15;
    }
    else
    {
        Component->Impact.RadialWeight = 1.0;
        Component->Impact.DirectionalWeight = 0.0;
        Component->Impact.UpwardWeight = 0.15;
        Component->Impact.RandomWeight = 0.18;
    }
    Component->PostEditChange();
    Component->MarkPackageDirty();
    return true;
}

bool FTAVisualFracturePreviewManager::CreateSession(UTAVisualFractureComponent* Component,
                                                    FString& OutError)
{
    if (!Component->IsBakeCurrent(&OutError) || !Component->BakedAsset)
    {
        return false;
    }
    if (FTAVisualFractureBakeService::BuildBakeSignature(Component, &OutError) != Component->BakedAsset->BakeSignature)
    {
        OutError = TEXT("Mesh content or bake settings changed; rebake before preview.");
        return false;
    }
    if (!FTAVisualFractureLandscapeService::ValidateCurrent(Component, OutError))
    {
        return false;
    }
    if (!ValidateResponse(Component->ObjectResponse, OutError))
    {
        return false;
    }
    const int32 ChunkCount = Component->BakedAsset->Chunks.Num();
    const int64 TriangleCount = Component->BakedAsset->BakedTriangleCount;
    if (ChunkCount > Component->PhysicsBudget.MaxActiveChunkStates ||
        ChunkCount > Component->PhysicsBudget.MaxGroundQueriesPerStep)
    {
        OutError = FString::Printf(TEXT("Bake is valid, but the current CPU preview physics budget is too small: Chunks=%d, ActiveStates=%d, GroundQueries=%d. Raise the preview budget or wait for the Phase C batched preview."),
            ChunkCount, Component->PhysicsBudget.MaxActiveChunkStates, Component->PhysicsBudget.MaxGroundQueriesPerStep);
        return false;
    }
    if (ChunkCount > Component->RenderBudget.MaxProceduralMeshComponents ||
        TriangleCount > Component->RenderBudget.MaxPreviewTriangles)
    {
        OutError = FString::Printf(TEXT("Bake is valid, but the compatibility PMC render budget is exceeded: Components=%d/%d, Triangles=%lld/%lld. This does not invalidate or delete the baked asset."),
            ChunkCount, Component->RenderBudget.MaxProceduralMeshComponents,
            TriangleCount, Component->RenderBudget.MaxPreviewTriangles);
        return false;
    }
    UStaticMeshComponent* Target = Component->ResolveTargetMeshComponent();
    AActor* Owner = Component->GetOwner();
    if (!Target || !Owner || !Target->GetWorld() || !Target->IsRegistered() || Target->GetOwner() != Owner)
    {
        OutError = TEXT("Target component is not registered in an Editor world.");
        return false;
    }

    TSet<int32> SupportedChunks;
    if (Component->SupportMode == ETVFSupportMode::LandscapeAnchored)
    {
        const double DistanceThreshold = FMath::Max(2.0 * Component->LandscapeSnapshot.GridStepCm, 5.0);
        const FQuat Rotation = Target->GetComponentQuat();
        const FVector Translation = Target->GetComponentLocation();
        for (const FTVFChunkRenderData& Chunk : Component->BakedAsset->Chunks)
        {
            bool bSupported = false;
            for (const FVector& LocalVertex : Chunk.Exterior.Vertices)
            {
                const FVector WorldPoint = Translation + Rotation.RotateVector(LocalVertex + Chunk.Centroid);
                double Height = 0.0;
                FVector Normal;
                if (!Component->LandscapeSnapshot.SampleHeightAndNormal(FVector2D(WorldPoint.X, WorldPoint.Y), Height, Normal))
                {
                    continue;
                }
                const FVector GroundPoint(WorldPoint.X, WorldPoint.Y, Height);
                if (FMath::Abs(FVector::DotProduct(Normal, WorldPoint - GroundPoint)) <= DistanceThreshold)
                {
                    bSupported = true;
                    break;
                }
            }
            if (bSupported) SupportedChunks.Add(Chunk.ChunkId);
        }
        if (SupportedChunks.IsEmpty())
        {
            OutError = TEXT("LandscapeAnchored found no exterior vertex close enough to the captured terrain.");
            return false;
        }
    }

    ActiveSession = MakeUnique<FSession>();
    FSession& Session = *ActiveSession;
    Session.Component = Component;
    Session.Target = Target;
    Session.InitialTargetTransform = Target->GetComponentTransform();
    Session.SupportMode = Component->SupportMode;
    Session.Response = Component->ObjectResponse;
    FTVFVisualCollisionSettings::StaticStruct()->ExportText(Session.CollisionSettingsText,
        &Component->VisualCollision, nullptr, nullptr, PPF_None, nullptr);
    Session.Asset = Component->BakedAsset;
    Session.ExteriorMaterial = Component->ExteriorMaterialOverride;
    Session.InteriorMaterial = Component->InteriorMaterialOverride;
    Session.bOriginalVisible = Target->GetVisibleFlag();
    Session.bOriginalHiddenInGame = Target->bHiddenInGame;
    Session.Motion = std::make_unique<tvf::MotionSystem>();

    tvf::MotionSettings MotionSettings;
    MotionSettings.fixedTimeStep = Component->ObjectResponse.FixedTimeStepSeconds;
    if (Component->VisualCollision.bSelfCollision || Component->VisualCollision.bExternalBoxCollision)
        MotionSettings.fixedTimeStep = 1.0 / 60.0;
    MotionSettings.linearDrag = Component->ObjectResponse.LinearDragPerSecond;
    MotionSettings.angularDrag = Component->ObjectResponse.AngularDragPerSecond;
    MotionSettings.sleepLinearSpeed = Component->ObjectResponse.SleepLinearSpeedCmPerSecond;
    MotionSettings.sleepAngularSpeed = Component->ObjectResponse.SleepAngularSpeedRadPerSecond;
    MotionSettings.sleepDelay = Component->ObjectResponse.SleepDelaySeconds;
    MotionSettings.initialWorldOffset = ToPreviewTVF(Target->GetComponentLocation());
    MotionSettings.initialWorldRotation = ToPreviewTVF(Target->GetComponentQuat());
    std::vector<tvf::ChunkMotionDesc> MotionChunks;
    MotionChunks.reserve(Component->BakedAsset->Chunks.Num());
    for (const FTVFChunkRenderData& Chunk : Component->BakedAsset->Chunks)
    {
        tvf::ChunkMotionDesc Desc;
        Desc.id = static_cast<uint32_t>(Chunk.ChunkId);
        Desc.restCentroid = ToPreviewTVF(Chunk.Centroid);
        Desc.radius = FMath::Max(Chunk.RadiusCm, UE_DOUBLE_SMALL_NUMBER);
        Desc.mass = FMath::Max(Chunk.VolumeCm3 * Component->ObjectResponse.MassPerCm3, UE_DOUBLE_SMALL_NUMBER);
        Desc.supported = SupportedChunks.Contains(Chunk.ChunkId);
        MotionChunks.push_back(Desc);
    }
    std::vector<tvf::BondMotionDesc> MotionBonds;
    MotionBonds.reserve(Component->BakedAsset->Bonds.Num());
    for (const FTVFBondData& Bond : Component->BakedAsset->Bonds)
    {
        if (!Component->BakedAsset->Chunks.IsValidIndex(Bond.ChunkA) ||
            !Component->BakedAsset->Chunks.IsValidIndex(Bond.ChunkB) || Bond.ChunkA == Bond.ChunkB)
        {
            OutError = TEXT("Baked asset contains an invalid Bond endpoint.");
            Reset();
            return false;
        }
        tvf::BondMotionDesc Desc;
        Desc.chunkA = static_cast<uint32_t>(Bond.ChunkA);
        Desc.chunkB = static_cast<uint32_t>(Bond.ChunkB);
        Desc.centroid = ToPreviewTVF(Bond.Centroid);
        Desc.resistance = FMath::Max(Bond.AreaCm2 * Component->ObjectResponse.BondResistancePerCm2,
            UE_DOUBLE_SMALL_NUMBER);
        MotionBonds.push_back(Desc);
    }
    if (!Session.Motion->Initialize(MotionSettings, MotionChunks, MotionBonds))
    {
        OutError = TEXT("MotionSystem rejected response, chunk or Bond parameters.");
        Reset();
        return false;
    }

    if (Component->VisualCollision.bSelfCollision || Component->VisualCollision.bExternalBoxCollision)
    {
        Session.Collision = MakeUnique<FTVFCollisionPreparation>();
        if (!TVFCollisionEditor::Prepare(Component, *Session.Collision, OutError))
        {
            Component->LastCollisionStatus = OutError;
            Reset();
            return false;
        }
        Session.Motion->SetCollisionScene(&Session.Collision->Scene);
        Component->LastCollisionStatus = FString::Printf(TEXT("Prepared %d proxies/footprints in %.3f s; RenderSeamProxies=%d; no Bake changes."),
            Session.Collision->HullCount, Session.Collision->PreparationSeconds, Session.Collision->RenderSeamProxyCount);
        Component->LastCollisionStatus += Session.Collision->VisualWarnings;
    }

    UMaterialInterface* DebugMaterial = ResolveDebugMaterial();
    for (const FTVFChunkRenderData& Chunk : Component->BakedAsset->Chunks)
    {
        UProceduralMeshComponent* PMC = NewObject<UProceduralMeshComponent>(Owner, NAME_None,
            RF_Transient | RF_DuplicateTransient | RF_TextExportTransient);
        if (!PMC)
        {
            OutError = TEXT("Failed to allocate a transient chunk ProceduralMeshComponent.");
            Reset();
            return false;
        }
        PMC->SetMobility(EComponentMobility::Movable);
        PMC->SetCollisionEnabled(ECollisionEnabled::NoCollision);
        PMC->SetCanEverAffectNavigation(false);
        PMC->bUseAsyncCooking = false;
        Owner->AddOwnedComponent(PMC); // GC/registration ownership only; never add to persistent InstanceComponents.
        PMC->RegisterComponentWithWorld(Target->GetWorld());
        CreateSection(PMC, 0, Chunk.Exterior);
        CreateSection(PMC, 1, Chunk.Interior);
        PMC->SetMaterial(0, Component->ExteriorMaterialOverride ? Component->ExteriorMaterialOverride.Get() : DebugMaterial);
        PMC->SetMaterial(1, Component->InteriorMaterialOverride ? Component->InteriorMaterialOverride.Get() : DebugMaterial);
        PMC->SetWorldTransform(FTransform(Target->GetComponentQuat(),
            Target->GetComponentLocation() + Target->GetComponentQuat().RotateVector(Chunk.Centroid), FVector::OneVector));
        PMC->SetVisibility(false, true);
        PMC->SetHiddenInGame(false, true);
        Session.ChunkComponents.Emplace(PMC);
    }
    return true;
}

bool FTAVisualFracturePreviewManager::PrepareCollision(UTAVisualFractureComponent* Component, FString& OutError)
{
    Reset();
    if (!Component || (!Component->VisualCollision.bSelfCollision && !Component->VisualCollision.bExternalBoxCollision))
    {
        OutError = TEXT("Enable visual self collision or external Box collision first.");
        return false;
    }
    if (!CreateSession(Component, OutError)) return false;
    ActiveSession->bFrozenCollisionPreview = true;
    ActiveSession->bShowCollisionProxies = true;
    return true;
}

bool FTAVisualFracturePreviewManager::ToggleCollisionProxies(UTAVisualFractureComponent* Component, FString& OutError)
{
    if (!ActiveSession || ActiveSession->Component.Get() != Component)
        return PrepareCollision(Component, OutError);
    if (!ActiveSession->Collision) { OutError = TEXT("No collision proxies prepared."); return false; }
    ActiveSession->bShowCollisionProxies = !ActiveSession->bShowCollisionProxies;
    return true;
}

bool FTAVisualFracturePreviewManager::Break(UTAVisualFractureComponent* Component, FString& OutError)
{
    check(IsInGameThread());
    if (!IsValid(Component))
    {
        OutError = TEXT("Invalid fracture component.");
        return false;
    }
    Component->InitializeImpactRadiiIfNeeded();
    if (ActiveSession && ActiveSession->Component.Get() != Component)
    {
        Reset();
    }
    if (!ActiveSession && !CreateSession(Component, OutError))
    {
        return false;
    }
    if (!ActiveSession || !ActiveSession->Motion)
    {
        OutError = TEXT("Preview session was not initialized.");
        return false;
    }
    if (ActiveSession->Collision && ActiveSession->Collision->Scene.statistics.paused)
    {
        OutError = FString::Printf(TEXT("Collision preview paused: %s. Reset after correcting the invalid state."),UTF8_TO_TCHAR(ActiveSession->Collision->Scene.statistics.reason.c_str()));
        return false;
    }
    FString CurrentError;
    if (!Component->IsBakeCurrent(&CurrentError) ||
        FTAVisualFractureBakeService::BuildBakeSignature(Component, &CurrentError) != Component->BakedAsset->BakeSignature ||
        !FTAVisualFractureLandscapeService::ValidateCurrent(Component, CurrentError))
    {
        OutError = CurrentError.IsEmpty() ? TEXT("Bake or Landscape capture became stale.") : CurrentError;
        Reset();
        return false;
    }
    if (!ValidateImpact(Component->Impact, OutError))
    {
        return false;
    }

    const UStaticMeshComponent* Target = ActiveSession->Target.Get();
    const FQuat Rotation = Target->GetComponentQuat();
    tvf::ImpactEvent Impact;
    Impact.eventId = NextEventId++;
    if (Impact.eventId == 0) Impact.eventId = NextEventId++;
    Impact.randomSeed = static_cast<uint32_t>(Component->Impact.RandomSeed);
    Impact.pointWorld = ToPreviewTVF(Target->GetComponentLocation() + Rotation.RotateVector(Component->Impact.ImpactPointLocal));
    Impact.directionWorld = ToPreviewTVF(Rotation.RotateVector(Component->Impact.DirectionLocal));
    Impact.upWorld = ToPreviewTVF(Rotation.RotateVector(FVector::UpVector));
    Impact.radius = Component->Impact.ImpactRadiusCm;
    Impact.impulse = Component->Impact.Impulse;
    Impact.radialWeight = Component->Impact.RadialWeight;
    Impact.directionalWeight = Component->Impact.DirectionalWeight;
    Impact.upwardWeight = Component->Impact.UpwardWeight;
    Impact.randomWeight = Component->Impact.RandomWeight;
    Impact.falloffExponent = Component->Impact.FalloffExponent;
    Impact.referenceRadius = Component->Impact.ReferenceRadiusCm;
    Impact.sizeSpeedBias = Component->Impact.SizeSpeedBias;
    Impact.bondDamage = Component->Impact.BondDamage;
    Impact.bondRadius = Component->Impact.BondRadiusCm;
    if (!ActiveSession->Motion->ApplyImpact(Impact))
    {
        OutError = TEXT("MotionSystem rejected the one-shot impact event.");
        return false;
    }
    Component->LastPreviewStatus = FString::Printf(TEXT("Submitted one-shot eventId=%llu."), Impact.eventId);
    ActiveSession->bFrozenCollisionPreview = false;
    UpdateVisuals();
    return true;
}

void FTAVisualFracturePreviewManager::UpdateVisuals()
{
    if (!ActiveSession || !ActiveSession->Motion)
    {
        return;
    }
    UStaticMeshComponent* Target = ActiveSession->Target.Get();
    if (!Target)
    {
        return;
    }
    const std::vector<tvf::ChunkMotionState>& States = ActiveSession->Motion->States();
    for (const tvf::ChunkMotionState& State : States)
    {
        if (State.detached)
        {
            ActiveSession->bSwitchedToChunks = true;
            break;
        }
    }
    int32 DetachedCount = 0;
    for (const tvf::ChunkMotionState& State : States) DetachedCount += State.detached ? 1 : 0;
    int32 BrokenCount = 0;
    for (const tvf::BondMotionState& Bond : ActiveSession->Motion->BondStates()) BrokenCount += Bond.broken ? 1 : 0;
    if (UTAVisualFractureComponent* Component = ActiveSession->Component.Get())
    {
        Component->PreviewDetachedChunks = DetachedCount;
        Component->PreviewBrokenBonds = BrokenCount;
    }
    Target->SetVisibility(ActiveSession->bSwitchedToChunks ? false : ActiveSession->bOriginalVisible, false);
    Target->SetHiddenInGame(ActiveSession->bSwitchedToChunks ? true : ActiveSession->bOriginalHiddenInGame, false);
    for (int32 Index = 0; Index < ActiveSession->ChunkComponents.Num(); ++Index)
    {
        UProceduralMeshComponent* PMC = ActiveSession->ChunkComponents[Index].Get();
        if (!PMC) continue;
        if (Index >= static_cast<int32>(States.size())) continue;
        const tvf::ChunkMotionState& State = States[Index];
        PMC->SetWorldLocationAndRotation(ToPreviewUE(State.positionWorld), ToPreviewUE(State.rotationWorld), false, nullptr,
            ETeleportType::TeleportPhysics);
        PMC->SetVisibility(ActiveSession->bSwitchedToChunks, true);
    }
    if (GEditor) GEditor->RedrawLevelEditingViewports(false);
}

bool FTAVisualFracturePreviewManager::Tick(const float DeltaSeconds)
{
    if (!ActiveSession)
    {
        return true;
    }
    UTAVisualFractureComponent* Component = ActiveSession->Component.Get();
    UStaticMeshComponent* Target = ActiveSession->Target.Get();
    if (!Component || !Target || !ActiveSession->Motion || Component->ResolveTargetMeshComponent() != Target ||
        !Target->GetComponentTransform().Equals(ActiveSession->InitialTargetTransform, 0.01) ||
        Component->SupportMode != ActiveSession->SupportMode ||
        Component->BakedAsset != ActiveSession->Asset.Get() ||
        Component->ExteriorMaterialOverride != ActiveSession->ExteriorMaterial.Get() ||
        Component->InteriorMaterialOverride != ActiveSession->InteriorMaterial.Get() ||
        !ResponseEquals(Component->ObjectResponse, ActiveSession->Response))
    {
        if (Component) Component->LastPreviewStatus = TEXT("Preview reset because target, asset, materials, support mode or response settings changed.");
        Reset();
        return true;
    }

    FString Error;
    if (!Component->IsBakeCurrent(&Error) ||
        FTAVisualFractureBakeService::BuildBakeSignature(Component, &Error) != Component->BakedAsset->BakeSignature ||
        !FTAVisualFractureLandscapeService::ValidateCurrent(Component, Error))
    {
        Component->LastPreviewStatus = Error.IsEmpty() ? TEXT("Preview reset because bake or Landscape capture became stale.") : Error;
        Reset();
        return true;
    }

    tvf::MotionStepInput Input;
    Input.gravityWorld = ToPreviewTVF(Component->ObjectResponse.GravityWorld);
    const FTVFLandscapeSnapshot* Snapshot = &Component->LandscapeSnapshot;
    const FTVFObjectResponseSettings Response = Component->ObjectResponse;
    const tvf::GroundPlaneQuery Ground =
        tvf::GroundPlaneQuery([Snapshot, Response](uint32_t, const tvf::Vec3& Center, tvf::PlaneCollider& Out)
        {
            double Height = 0.0;
            FVector Normal;
            if (!Snapshot->SampleHeightAndNormal(FVector2D(Center.x, Center.y), Height, Normal))
            {
                return false;
            }
            Out.normal = ToPreviewTVF(Normal);
            Out.offset = FVector::DotProduct(Normal, FVector(Center.x, Center.y, Height));
            Out.restitution = Response.Restitution;
            Out.friction = Response.Friction;
            Out.angularDamping = Response.ContactAngularDampingPerSecond;
            Out.restitutionVelocityThreshold = Response.RestitutionVelocityThresholdCmPerSecond;
            return true;
        });
    FString CurrentCollisionSettings;
    FTVFVisualCollisionSettings::StaticStruct()->ExportText(CurrentCollisionSettings,
        &Component->VisualCollision, nullptr, nullptr, PPF_None, nullptr);
    if (CurrentCollisionSettings != ActiveSession->CollisionSettingsText)
    {
        Component->LastCollisionStatus = TEXT("Collision settings changed; preview reset.");
        Reset(); return true;
    }
    FString ExternalRefreshError;
    if (ActiveSession->Collision)
    {
        // Invalid external registrations disable only external obstacles until
        // repaired; the fractured object and its ground simulation stay alive.
        TVFCollisionEditor::RefreshExternal(Component,*ActiveSession->Collision,ExternalRefreshError);
        FString Signature;
        if (!TVFCollisionEditor::Signature(Component, Signature, Error) || Signature != ActiveSession->Collision->Signature)
        {
            Component->LastCollisionStatus = Error.IsEmpty() ? TEXT("Registered collider geometry/transform/settings changed; preview reset.") : Error;
            Reset(); return true;
        }
        if (ActiveSession->bShowCollisionProxies)
            TVFCollisionEditor::Draw(Component, *ActiveSession->Collision, ActiveSession->Motion->States());
    }
    if (!ActiveSession->bFrozenCollisionPreview) ActiveSession->Motion->Step(DeltaSeconds, Input, Ground);
    if (ActiveSession->Collision)
    {
        const auto& S = ActiveSession->Collision->Scene.statistics;
        const FString Reason=FString(UTF8_TO_TCHAR(S.reason.c_str()))+(ExternalRefreshError.IsEmpty()?FString():TEXT(" External collisions temporarily disabled: ")+ExternalRefreshError);
        if(!Reason.IsEmpty()&&Reason!=ActiveSession->LastLoggedCollisionReason){
            UE_LOG(LogTemp,Warning,TEXT("TAVisualFracture %s: %s (pairs=%llu shapes=%llu contacts=%llu groundQueries=%llu)"),*Component->GetPathName(),*Reason,S.pairs,S.shapePairs,S.contacts,S.groundQueries);
            ActiveSession->LastLoggedCollisionReason=Reason;
        }
        Component->LastCollisionStatus = FString::Printf(TEXT("Proxies=%d Prepare=%.3fs Pairs=%llu Shapes=%llu Contacts=%llu MaxPenetration=%.4fcm Sleeping=%llu Substeps=%u %s"),
            ActiveSession->Collision->HullCount, ActiveSession->Collision->PreparationSeconds,
            S.pairs, S.shapePairs, S.contacts, S.maxPenetration, S.sleeping, S.substeps, UTF8_TO_TCHAR(S.reason.c_str()));
        Component->LastCollisionStatus += FString::Printf(TEXT(" RenderSeamProxies=%d"),ActiveSession->Collision->RenderSeamProxyCount);
        Component->LastCollisionStatus += ActiveSession->Collision->VisualWarnings;
        if(!ExternalRefreshError.IsEmpty())Component->LastCollisionStatus+=TEXT(" External collisions temporarily disabled: ")+ExternalRefreshError;
    }
    UpdateVisuals();
    return true;
}

void FTAVisualFracturePreviewManager::Reset(UTAVisualFractureComponent* Component)
{
    check(IsInGameThread());
    if (!ActiveSession || (Component && ActiveSession->Component.Get() != Component))
    {
        return;
    }
    if (UStaticMeshComponent* Target = ActiveSession->Target.Get())
    {
        Target->SetVisibility(ActiveSession->bOriginalVisible, false);
        Target->SetHiddenInGame(ActiveSession->bOriginalHiddenInGame, false);
    }
    if (UTAVisualFractureComponent* ActiveComponent = ActiveSession->Component.Get())
    {
        ActiveComponent->PreviewDetachedChunks = 0;
        ActiveComponent->PreviewBrokenBonds = 0;
        if (ActiveComponent->LastPreviewStatus.IsEmpty()) ActiveComponent->LastPreviewStatus = TEXT("Preview reset; source mesh restored.");
    }
    for (const TStrongObjectPtr<UProceduralMeshComponent>& StrongPMC : ActiveSession->ChunkComponents)
    {
        if (UProceduralMeshComponent* PMC = StrongPMC.Get())
        {
            PMC->DestroyComponent();
        }
    }
    ActiveSession.Reset();
    NextEventId = 1;
    if (GEditor) GEditor->RedrawLevelEditingViewports(false);
}

void FTAVisualFracturePreviewManager::OnPreBeginPIE(bool)
{
    Reset();
}

void FTAVisualFracturePreviewManager::OnMapOpened(const FString&, bool)
{
    Reset();
}

void FTAVisualFracturePreviewManager::OnMapChanged(uint32)
{
    Reset();
}

void FTAVisualFracturePreviewManager::OnPreSaveWorld(UWorld* World, FObjectPreSaveContext)
{
    if (!ActiveSession || !World || !ActiveSession->Target.IsValid() || ActiveSession->Target->GetWorld() == World)
    {
        Reset();
    }
}

void FTAVisualFracturePreviewManager::OnPostUndoRedo()
{
    Reset();
}
