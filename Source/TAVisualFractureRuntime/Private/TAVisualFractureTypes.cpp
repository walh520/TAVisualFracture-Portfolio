#include "TAVisualFractureTypes.h"

namespace
{
bool SampleHeightOnly(const FTVFLandscapeSnapshot& Snapshot, const FVector2D& WorldXY, double& OutHeight)
{
    if (Snapshot.Resolution < 2 || Snapshot.GridStepCm <= 0.0)
    {
        return false;
    }

    const double GridX = (WorldXY.X - Snapshot.OriginWorldXY.X) / Snapshot.GridStepCm;
    const double GridY = (WorldXY.Y - Snapshot.OriginWorldXY.Y) / Snapshot.GridStepCm;
    if (GridX < 0.0 || GridY < 0.0 || GridX > Snapshot.Resolution - 1 || GridY > Snapshot.Resolution - 1)
    {
        return false;
    }

    const int32 X0 = FMath::Clamp(FMath::FloorToInt(GridX), 0, Snapshot.Resolution - 2);
    const int32 Y0 = FMath::Clamp(FMath::FloorToInt(GridY), 0, Snapshot.Resolution - 2);
    const int32 X1 = X0 + 1;
    const int32 Y1 = Y0 + 1;
    const int32 Indices[4] =
    {
        Y0 * Snapshot.Resolution + X0,
        Y0 * Snapshot.Resolution + X1,
        Y1 * Snapshot.Resolution + X0,
        Y1 * Snapshot.Resolution + X1
    };
    for (const int32 Index : Indices)
    {
        if (!Snapshot.ValidMask.IsValidIndex(Index) || Snapshot.ValidMask[Index] == 0 ||
            !Snapshot.WorldHeightsCm.IsValidIndex(Index) || !FMath::IsFinite(Snapshot.WorldHeightsCm[Index]))
        {
            return false;
        }
    }

    const double Tx = FMath::Clamp(GridX - X0, 0.0, 1.0);
    const double Ty = FMath::Clamp(GridY - Y0, 0.0, 1.0);
    const double H0 = FMath::Lerp(Snapshot.WorldHeightsCm[Indices[0]], Snapshot.WorldHeightsCm[Indices[1]], Tx);
    const double H1 = FMath::Lerp(Snapshot.WorldHeightsCm[Indices[2]], Snapshot.WorldHeightsCm[Indices[3]], Tx);
    OutHeight = FMath::Lerp(H0, H1, Ty);
    return FMath::IsFinite(OutHeight);
}
}

bool FTVFLandscapeSnapshot::IsValid(FString* OutReason) const
{
    const int64 ExpectedCount = static_cast<int64>(Resolution) * Resolution;
    const bool bValid = LandscapeGuid.IsValid() && Resolution >= 2 && GridStepCm > 0.0 &&
        WorldHeightsCm.Num() == ExpectedCount && ValidMask.Num() == ExpectedCount &&
        !ValidMask.Contains(0);
    if (!bValid && OutReason)
    {
        *OutReason = TEXT("Landscape snapshot is incomplete or contains invalid height samples.");
    }
    return bValid;
}

bool FTVFLandscapeSnapshot::SampleHeightAndNormal(const FVector2D& WorldXY, double& OutHeightCm,
                                                   FVector& OutNormal) const
{
    if (!SampleHeightOnly(*this, WorldXY, OutHeightCm))
    {
        return false;
    }

    const double MinX = OriginWorldXY.X;
    const double MinY = OriginWorldXY.Y;
    const double MaxX = MinX + GridStepCm * (Resolution - 1);
    const double MaxY = MinY + GridStepCm * (Resolution - 1);
    const FVector2D XMinus(FMath::Max(MinX, WorldXY.X - GridStepCm), WorldXY.Y);
    const FVector2D XPlus(FMath::Min(MaxX, WorldXY.X + GridStepCm), WorldXY.Y);
    const FVector2D YMinus(WorldXY.X, FMath::Max(MinY, WorldXY.Y - GridStepCm));
    const FVector2D YPlus(WorldXY.X, FMath::Min(MaxY, WorldXY.Y + GridStepCm));
    double HX0 = 0.0, HX1 = 0.0, HY0 = 0.0, HY1 = 0.0;
    if (!SampleHeightOnly(*this, XMinus, HX0) || !SampleHeightOnly(*this, XPlus, HX1) ||
        !SampleHeightOnly(*this, YMinus, HY0) || !SampleHeightOnly(*this, YPlus, HY1))
    {
        return false;
    }

    const double DX = FMath::Max(XPlus.X - XMinus.X, UE_DOUBLE_SMALL_NUMBER);
    const double DY = FMath::Max(YPlus.Y - YMinus.Y, UE_DOUBLE_SMALL_NUMBER);
    OutNormal = FVector(-(HX1 - HX0) / DX, -(HY1 - HY0) / DY, 1.0).GetSafeNormal();
    return !OutNormal.IsNearlyZero();
}
