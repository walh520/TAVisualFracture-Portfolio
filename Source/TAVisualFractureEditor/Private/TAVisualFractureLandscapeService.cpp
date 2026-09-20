#include "TAVisualFractureLandscapeService.h"

#include "TAVisualFractureComponent.h"

#include "Components/StaticMeshComponent.h"
#include "Editor.h"
#include "EngineUtils.h"
#include "LandscapeProxy.h"
#include "ScopedTransaction.h"

namespace
{
constexpr int32 SnapshotResolution = 129;

struct FCandidateLandscape
{
    FGuid Guid;
    TArray<ALandscapeProxy*> Proxies;
    TArray<double> Heights;
    TArray<uint8> Valid;
};

bool SampleCandidate(FCandidateLandscape& Candidate, const FVector2D& Origin, const double Step,
                     const double ProbeZ)
{
    const int32 Count = SnapshotResolution * SnapshotResolution;
    Candidate.Heights.SetNumUninitialized(Count);
    Candidate.Valid.Init(0, Count);
    for (int32 Y = 0; Y < SnapshotResolution; ++Y)
    {
        for (int32 X = 0; X < SnapshotResolution; ++X)
        {
            const FVector Probe(Origin.X + X * Step, Origin.Y + Y * Step, ProbeZ);
            TOptional<float> Height;
            for (const ALandscapeProxy* Proxy : Candidate.Proxies)
            {
                Height = Proxy->GetHeightAtLocation(Probe, EHeightfieldSource::Editor);
                if (Height.IsSet()) break;
            }
            const int32 Index = Y * SnapshotResolution + X;
            if (!Height.IsSet() || !FMath::IsFinite(Height.GetValue()))
            {
                return false;
            }
            Candidate.Heights[Index] = Height.GetValue();
            Candidate.Valid[Index] = 1;
        }
    }
    return true;
}
}

bool FTAVisualFractureLandscapeService::Capture(UTAVisualFractureComponent* Component)
{
    check(IsInGameThread());
    if (!IsValid(Component))
    {
        return false;
    }
    UStaticMeshComponent* Target = Component->ResolveTargetMeshComponent();
    UWorld* World = Target ? Target->GetWorld() : nullptr;
    if (!Target || !World || !Target->IsRegistered() || Target->GetOwner() != Component->GetOwner())
    {
        Component->LastCaptureStatus = TEXT("Landscape capture requires a registered target mesh in an Editor world.");
        return false;
    }

    ALandscapeProxy* Explicit = Cast<ALandscapeProxy>(Component->ExplicitLandscape);
    if (Component->ExplicitLandscape && !Explicit)
    {
        Component->LastCaptureStatus = TEXT("Explicit Landscape must reference a Landscape or Landscape Streaming Proxy.");
        return false;
    }
    if (Explicit && Explicit->GetWorld() != World)
    {
        Component->LastCaptureStatus = TEXT("Explicit Landscape must belong to the target mesh's Editor world.");
        return false;
    }

    TMap<FGuid, FCandidateLandscape> Groups;
    for (TActorIterator<ALandscapeProxy> It(World); It; ++It)
    {
        ALandscapeProxy* Proxy = *It;
        const FGuid Guid = Proxy->GetLandscapeGuid();
        if (!Guid.IsValid() || (Explicit && Guid != Explicit->GetLandscapeGuid()))
        {
            continue;
        }
        FCandidateLandscape& Group = Groups.FindOrAdd(Guid);
        Group.Guid = Guid;
        Group.Proxies.Add(Proxy);
    }
    if (Groups.IsEmpty())
    {
        Component->LastCaptureStatus = TEXT("No Landscape candidate exists in the target world.");
        return false;
    }

    const double HalfExtent = Component->ResolveCaptureHalfExtentCm();
    const FVector Center = Target->GetComponentLocation();
    const FVector2D Origin(Center.X - HalfExtent, Center.Y - HalfExtent);
    const double Step = (HalfExtent * 2.0) / (SnapshotResolution - 1);
    TArray<FCandidateLandscape*> CompleteCandidates;
    for (TPair<FGuid, FCandidateLandscape>& Pair : Groups)
    {
        if (SampleCandidate(Pair.Value, Origin, Step, Center.Z))
        {
            CompleteCandidates.Add(&Pair.Value);
        }
    }
    if (CompleteCandidates.Num() != 1)
    {
        Component->LastCaptureStatus = CompleteCandidates.IsEmpty() ?
            TEXT("The requested 129x129 capture contains invalid Landscape heights.") :
            TEXT("More than one Landscape fully covers the capture range; assign Explicit Landscape.");
        return false;
    }

    FCandidateLandscape& Chosen = *CompleteCandidates[0];
    ALandscapeProxy* Representative = Explicit ? Explicit : Chosen.Proxies[0];
    const FScopedTransaction Transaction(NSLOCTEXT("TAVisualFracture", "CaptureLandscape", "Capture TA Visual Fracture Landscape"));
    Component->Modify();
    FTVFLandscapeSnapshot Snapshot;
    Snapshot.LandscapePath = FSoftObjectPath(Representative);
    Snapshot.LandscapeGuid = Chosen.Guid;
    Snapshot.CapturedTargetTransform = Target->GetComponentTransform();
    Snapshot.OriginWorldXY = Origin;
    Snapshot.GridStepCm = Step;
    Snapshot.Resolution = SnapshotResolution;
    Snapshot.CapturedHalfExtentCm = HalfExtent;
    Snapshot.WorldHeightsCm = MoveTemp(Chosen.Heights);
    Snapshot.ValidMask = MoveTemp(Chosen.Valid);
    Component->LandscapeSnapshot = MoveTemp(Snapshot);
    Component->LastCaptureStatus = FString::Printf(TEXT("Captured Landscape %s: 129x129, step %.3f cm, half range %.3f cm."),
        *Chosen.Guid.ToString(EGuidFormats::DigitsWithHyphens), Step, HalfExtent);
    Component->PostEditChange();
    Component->MarkPackageDirty();
    return true;
}

bool FTAVisualFractureLandscapeService::ValidateCurrent(const UTAVisualFractureComponent* Component,
                                                        FString& OutError)
{
    if (!Component || !Component->IsLandscapeCaptureCurrent(&OutError))
    {
        return false;
    }
    const ALandscapeProxy* Landscape = Cast<ALandscapeProxy>(Component->LandscapeSnapshot.LandscapePath.ResolveObject());
    if (!Landscape || Landscape->GetLandscapeGuid() != Component->LandscapeSnapshot.LandscapeGuid)
    {
        OutError = TEXT("Captured Landscape is unloaded, replaced or has a different GUID.");
        return false;
    }
    return true;
}
