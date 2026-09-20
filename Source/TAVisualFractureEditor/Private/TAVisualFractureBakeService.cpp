#include "TAVisualFractureBakeService.h"

#include "TAVisualFractureAsset.h"
#include "TAVisualFractureComponent.h"
#include "TAVisualFractureMeshAdapter.h"
#include "TVFFracture.h"

#include "AssetRegistry/AssetRegistryModule.h"
#include "AssetToolsModule.h"
#include "Async/Async.h"
#include "Async/TaskGraphInterfaces.h"
#include "Components/StaticMeshComponent.h"
#include "DynamicMesh/DynamicMesh3.h"
#include "DynamicMesh/DynamicMeshAABBTree3.h"
#include "Engine/StaticMesh.h"
#include "HAL/PlatformProcess.h"
#include "Implicit/Solidify.h"
#include "MeshDescription.h"
#include "Misc/PackageName.h"
#include "Misc/SecureHash.h"
#include "Spatial/FastWinding.h"
#include "StaticMeshAttributes.h"
#include "UObject/Package.h"

#include <algorithm>
#include <cmath>
#include <exception>
#include <limits>

namespace
{
constexpr uint64 BytesPerMegabyte = 1024ull * 1024ull;
constexpr uint64 TestsPerMillion = 1000000ull;

struct FPreparedBakeInput
{
    tvf::TriangleMesh Mesh;
    FTVFBakeSettings Settings;
    FSoftObjectPath SourceMeshPath;
    FVector PositiveScale = FVector::OneVector;
    FString Signature;
    FString SourceGeometryHash;
    FString TopologySettingsHash;
    int32 SourceTriangleCount = 0;
    int32 ProcessingTriangleCount = 0;
    std::vector<tvf::Diagnostic> Diagnostics;
};

struct FBakeWorkerResult
{
    bool bSucceeded = false;
    bool bCancelled = false;
    FString Error;
    FPreparedBakeInput Input;
    tvf::BakeResult Bake;
    std::vector<tvf::Seed> Seeds;
    uint32 GridNx = 0;
    uint32 GridNy = 0;
    uint32 GridNz = 0;
    double GridStepCm = 0.0;
    bool bHasCenterInteriorPoint = false;
    tvf::Vec3 CenterInteriorPoint;
    int32 BakedTriangleCount = 0;
    uint64 PeakEstimatedBytes = 0;
    std::vector<tvf::Diagnostic> InputDiagnostics;
};

tvf::Vec3 ToTVF(const FVector& V)
{
    return {V.X, V.Y, V.Z};
}

FVector ToUE(const tvf::Vec3& V)
{
    return FVector(V.x, V.y, V.z);
}

uint64 SaturatingMultiply(const uint64 A, const uint64 B)
{
    return A == 0 || B == 0 ? 0 : (A > TNumericLimits<uint64>::Max() / B ? TNumericLimits<uint64>::Max() : A * B);
}

uint64 GetWorkBudget(const FTVFBakeSettings& Settings)
{
    return SaturatingMultiply(static_cast<uint64>(Settings.MaxBakeWorkMillions), TestsPerMillion);
}

uint64 GetMemoryBudget(const FTVFBakeSettings& Settings)
{
    return SaturatingMultiply(static_cast<uint64>(Settings.WorkingMemoryBudgetMB), BytesPerMegabyte);
}

FString FormatDiagnostics(const std::vector<tvf::Diagnostic>& Diagnostics, const bool bFatalOnly)
{
    FString Text;
    for (const tvf::Diagnostic& Diagnostic : Diagnostics)
    {
        if (bFatalOnly && Diagnostic.severity != tvf::DiagnosticSeverity::FatalGeometry) continue;
        const TCHAR* Severity = Diagnostic.severity == tvf::DiagnosticSeverity::FatalGeometry ? TEXT("FatalGeometry") :
            (Diagnostic.severity == tvf::DiagnosticSeverity::Recovered ? TEXT("Recovered") : TEXT("Quality"));
        Text += FString::Printf(TEXT("[%s][Stage=%d Code=%d] %s"), Severity,
            static_cast<int32>(Diagnostic.stage), static_cast<int32>(Diagnostic.code), UTF8_TO_TCHAR(Diagnostic.message.c_str()));
        if (Diagnostic.seedId != tvf::Diagnostic::InvalidId) Text += FString::Printf(TEXT(" Seed=%llu"), Diagnostic.seedId);
        if (Diagnostic.tetId != tvf::Diagnostic::InvalidId) Text += FString::Printf(TEXT(" Tet=%llu"), Diagnostic.tetId);
        if (Diagnostic.patchId != tvf::Diagnostic::InvalidId) Text += FString::Printf(TEXT(" Patch=%llu"), Diagnostic.patchId);
        if (Diagnostic.measuredValue != 0.0 || Diagnostic.limit != 0.0)
        {
            Text += FString::Printf(TEXT(" Measured=%.9g Limit=%.9g"), Diagnostic.measuredValue, Diagnostic.limit);
        }
        if (Diagnostic.beforeCount != 0 || Diagnostic.afterCount != 0)
        {
            Text += FString::Printf(TEXT(" Before=%llu After=%llu"), Diagnostic.beforeCount, Diagnostic.afterCount);
        }
        Text += LINE_TERMINATOR;
    }
    return Text;
}

ETVFDiagnosticSeverity ToRuntimeSeverity(const tvf::DiagnosticSeverity Severity)
{
    if (Severity == tvf::DiagnosticSeverity::FatalGeometry) return ETVFDiagnosticSeverity::FatalGeometry;
    if (Severity == tvf::DiagnosticSeverity::Recovered) return ETVFDiagnosticSeverity::Recovered;
    return ETVFDiagnosticSeverity::Quality;
}

ETVFDiagnosticStage ToRuntimeStage(const tvf::DiagnosticStage Stage)
{
    return static_cast<ETVFDiagnosticStage>(static_cast<uint8>(Stage));
}

FName DiagnosticCodeName(const tvf::DiagnosticCode Code)
{
    static const TCHAR* Names[] = {
        TEXT("InvalidInput"), TEXT("BoundaryEdge"), TEXT("NonManifoldEdge"), TEXT("InconsistentWinding"), TEXT("DegenerateTriangle"),
        TEXT("DisconnectedVertexLink"), TEXT("BowTieVertex"), TEXT("SelfIntersection"), TEXT("CoplanarOverlap"), TEXT("ShellIntersection"),
        TEXT("AmbiguousShellNesting"), TEXT("RepairChangedTopology"), TEXT("RepairSurfaceDeviation"), TEXT("ShellSurfaceFusion"),
        TEXT("NonFiniteValue"), TEXT("DegenerateTetra"), TEXT("DegenerateFieldPlane"), TEXT("DuplicateSeedResampled"),
        TEXT("InsufficientSeeds"), TEXT("UnexpectedEmptyCell"), TEXT("BudgetExceeded"), TEXT("UnmatchedInterface"),
        TEXT("NonManifoldOutput"), TEXT("InternalInterface"), TEXT("VolumeResidual"), TEXT("CoverageGap"), TEXT("CoverageOverlap")
    };
    const int32 Index = static_cast<int32>(Code);
    return FName(Index >= 0 && Index < UE_ARRAY_COUNT(Names) ? Names[Index] : TEXT("Unknown"));
}

FString ComputeGeometryHash(const FMeshDescription& Description, const FVector& Scale)
{
    FMD5 Hasher;
    const FStaticMeshConstAttributes Attributes(Description);
    const TVertexAttributesConstRef<FVector3f> Positions = Attributes.GetVertexPositions();
    for (const FVertexID VertexId : Description.Vertices().GetElementIDs())
    {
        const FVector3f P = Positions[VertexId];
        const double Scaled[3] = {P.X * Scale.X, P.Y * Scale.Y, P.Z * Scale.Z};
        Hasher.Update(reinterpret_cast<const uint8*>(Scaled), sizeof(Scaled));
    }
    for (const FTriangleID TriangleId : Description.Triangles().GetElementIDs())
    {
        const TArrayView<const FVertexInstanceID> Corners = Description.GetTriangleVertexInstances(TriangleId);
        for (const FVertexInstanceID Corner : Corners)
        {
            const int32 VertexValue = Description.GetVertexInstanceVertex(Corner).GetValue();
            Hasher.Update(reinterpret_cast<const uint8*>(&VertexValue), sizeof(VertexValue));
        }
    }
    uint8 Digest[16];
    Hasher.Final(Digest);
    return BytesToHex(Digest, UE_ARRAY_COUNT(Digest));
}

FString ComputeTopologySettingsHash(const FTVFBakeSettings& B)
{
    const FString Contract = FString::Printf(
        TEXT("TVFTopologyV3-WindingUECore1-Clip2-SeedMix1|Mode=%d|Shell=%.17g|Solidify=%d,%.17g|Count=%d|Grid=%d|Seed=%d|Irregularity=%.17g|Variation=%.17g|Metric=%.17g,%.17g,%.17g,%.17g|Focus=%.17g,%.17g,%.17g,%.17g,%.17g|Merge=%.17g"),
        static_cast<int32>(B.InputMode), B.OpenShellThicknessCm, B.SolidifyVoxelResolution, B.SolidifyWindingThreshold,
        B.TargetChunkCount, B.LongestAxisCells, B.RandomSeed, B.Irregularity, B.SizeVariation,
        B.MetricAxis.X, B.MetricAxis.Y, B.MetricAxis.Z, B.MetricAxisScale,
        B.DensityFocusPointLocal.X, B.DensityFocusPointLocal.Y, B.DensityFocusPointLocal.Z,
        B.DensityFocusRadiusCm, B.DensityFocusConcentration, B.MinFinalChunkVolumeFraction);
    return FMD5::HashAnsiString(*Contract);
}

bool EstimateGridVertexCount(const tvf::TriangleMesh& Mesh, const int32 Cells, uint64& OutCount)
{
    if (Mesh.positions.empty() || Cells < 2)
    {
        return false;
    }
    const tvf::Bounds Bounds = tvf::MeshBounds(Mesh);
    const tvf::Vec3 Size = Bounds.Size();
    const double Extent = std::max({Size.x, Size.y, Size.z});
    if (!(Extent > 0.0) || !std::isfinite(Extent))
    {
        return false;
    }
    const double Pad = Extent * 2.0 / Cells;
    const tvf::Vec3 DomainSize = Size + tvf::Vec3(Pad * 2.0, Pad * 2.0, Pad * 2.0);
    const double Step = (Extent + Pad * 2.0) / Cells;
    const uint64 Nx = static_cast<uint64>(std::ceil(DomainSize.x / Step)) + 1;
    const uint64 Ny = static_cast<uint64>(std::ceil(DomainSize.y / Step)) + 1;
    const uint64 Nz = static_cast<uint64>(std::ceil(DomainSize.z / Step)) + 1;
    OutCount = SaturatingMultiply(SaturatingMultiply(Nx, Ny), Nz);
    return OutCount != TNumericLimits<uint64>::Max();
}

bool ConvertSolidMeshToCore(const UE::Geometry::FDynamicMesh3& Source, tvf::TriangleMesh& OutMesh, FString& OutError)
{
    OutMesh = {};
    TMap<int32, uint32> VertexToIndex;
    OutMesh.positions.reserve(Source.VertexCount());
    for (const int32 VertexId : Source.VertexIndicesItr())
    {
        const FVector3d P = Source.GetVertex(VertexId);
        if (P.ContainsNaN())
        {
            OutError = TEXT("VisualSolidify generated a non-finite vertex.");
            return false;
        }
        const uint32 Index = static_cast<uint32>(OutMesh.positions.size());
        VertexToIndex.Add(VertexId, Index);
        OutMesh.positions.push_back({P.X, P.Y, P.Z});
    }
    OutMesh.triangles.reserve(Source.TriangleCount());
    for (const int32 TriangleId : Source.TriangleIndicesItr())
    {
        const UE::Geometry::FIndex3i Triangle = Source.GetTriangle(TriangleId);
        const uint32* A = VertexToIndex.Find(Triangle.A);
        const uint32* B = VertexToIndex.Find(Triangle.B);
        const uint32* C = VertexToIndex.Find(Triangle.C);
        if (!A || !B || !C)
        {
            OutError = TEXT("VisualSolidify generated an invalid triangle reference.");
            return false;
        }
        OutMesh.triangles.push_back(TVFMeshAdapter::ReverseTriangle(*A, *B, *C));
    }
    return !OutMesh.positions.empty() && !OutMesh.triangles.empty();
}

bool BuildDynamicMeshFromCore(const tvf::TriangleMesh& Input, const bool bAllowTriangleSoup,
                              UE::Geometry::FDynamicMesh3& OutMesh, FString& OutError)
{
    using namespace UE::Geometry;
    TArray<int32> SharedVertexIds;
    SharedVertexIds.Reserve(static_cast<int32>(Input.positions.size()));
    for (const tvf::Vec3& Position : Input.positions)
    {
        SharedVertexIds.Add(OutMesh.AppendVertex(FVector3d(Position.x, Position.y, Position.z)));
    }
    for (const std::array<uint32_t, 3>& Triangle : Input.triangles)
    {
        if (Triangle[0] >= Input.positions.size() || Triangle[1] >= Input.positions.size() || Triangle[2] >= Input.positions.size())
        {
            OutError = TEXT("Input triangle index is out of range.");
            return false;
        }
        const tvf::Vec3& A = Input.positions[Triangle[0]];
        const tvf::Vec3& B = Input.positions[Triangle[1]];
        const tvf::Vec3& C = Input.positions[Triangle[2]];
        if (tvf::LengthSquared(tvf::Cross(B - A, C - A)) <= 1e-24)
        {
            continue;
        }
        const std::array<int32, 3> UnrealTriangle = TVFMeshAdapter::ReverseTriangle(
            SharedVertexIds[Triangle[0]], SharedVertexIds[Triangle[1]], SharedVertexIds[Triangle[2]]);
        if (OutMesh.AppendTriangle(FIndex3i(UnrealTriangle[0], UnrealTriangle[1], UnrealTriangle[2])) < 0)
        {
            if (!bAllowTriangleSoup)
            {
                OutError = TEXT("Closed Volume could not be represented as a manifold DynamicMesh.");
                return false;
            }
            const int32 VA = OutMesh.AppendVertex(FVector3d(A.x, A.y, A.z));
            const int32 VB = OutMesh.AppendVertex(FVector3d(B.x, B.y, B.z));
            const int32 VC = OutMesh.AppendVertex(FVector3d(C.x, C.y, C.z));
            const std::array<int32, 3> UnrealSoupTriangle = TVFMeshAdapter::ReverseTriangle(VA, VB, VC);
            if (OutMesh.AppendTriangle(FIndex3i(UnrealSoupTriangle[0], UnrealSoupTriangle[1], UnrealSoupTriangle[2])) < 0)
            {
                OutError = TEXT("Triangle-soup mode could not append an isolated source face.");
                return false;
            }
        }
    }
    if (OutMesh.TriangleCount() == 0)
    {
        OutError = TEXT("Input contains no non-degenerate triangles.");
        return false;
    }
    return true;
}

bool BuildAcceleratedGrid(const UE::Geometry::FDynamicMesh3& Mesh, const int32 Cells,
                          const uint64 MemoryBudget, const uint64 WorkBudget, TAtomic<bool>& bCancel,
                          tvf::ScalarGrid& OutGrid, uint64& InOutPeakBytes, FString& OutError,
                          const double ShellHalfThicknessCm = 0.0)
{
    using namespace UE::Geometry;
    FDynamicMeshAABBTree3 Spatial(&Mesh);
    TFastWindingTree<FDynamicMesh3> Winding(&Spatial);
    const FAxisAlignedBox3d MeshBounds = Spatial.GetBoundingBox();
    const FVector3d MeshSize = MeshBounds.Diagonal();
    const double Extent = MeshSize.GetMax();
    if (!(Extent > 0.0) || !FMath::IsFinite(Extent))
    {
        OutError = TEXT("VisualSolidify output has invalid bounds.");
        return false;
    }
    const double Pad = Extent * 2.0 / Cells + FMath::Max(0.0, ShellHalfThicknessCm);
    const FAxisAlignedBox3d Domain(MeshBounds.Min - FVector3d(Pad, Pad, Pad), MeshBounds.Max + FVector3d(Pad, Pad, Pad));
    const FVector3d DomainSize = Domain.Diagonal();
    const double Step = DomainSize.GetMax() / Cells;
    OutGrid.origin = {Domain.Min.X, Domain.Min.Y, Domain.Min.Z};
    OutGrid.step = Step;
    OutGrid.nx = static_cast<uint32>(std::ceil(DomainSize.X / Step)) + 1;
    OutGrid.ny = static_cast<uint32>(std::ceil(DomainSize.Y / Step)) + 1;
    OutGrid.nz = static_cast<uint32>(std::ceil(DomainSize.Z / Step)) + 1;
    const uint64 GridVertexCount = SaturatingMultiply(SaturatingMultiply(OutGrid.nx, OutGrid.ny), OutGrid.nz);
    const uint64 GridBytes = SaturatingMultiply(GridVertexCount, sizeof(double));
    InOutPeakBytes = FMath::Max(InOutPeakBytes, GridBytes);
    if (GridVertexCount == TNumericLimits<uint64>::Max() || GridBytes > MemoryBudget)
    {
        OutError = TEXT("Scalar-field grid exceeds the configured working-memory budget.");
        return false;
    }
    const uint64 QueryWeight = FMath::Max<uint64>(8ull,
        static_cast<uint64>(std::ceil(std::log2(static_cast<double>(FMath::Max(2, Mesh.TriangleCount()))))) * 4ull);
    if (SaturatingMultiply(GridVertexCount, QueryWeight) > WorkBudget)
    {
        OutError = TEXT("Accelerated scalar-field estimate exceeds the configured work budget.");
        return false;
    }
    OutGrid.phi.resize(static_cast<size_t>(GridVertexCount));
    bool bHasInterior = false;
    for (uint32 Z = 0; Z < OutGrid.nz; ++Z)
    {
        if (bCancel.Load())
        {
            return false;
        }
        for (uint32 Y = 0; Y < OutGrid.ny; ++Y)
        for (uint32 X = 0; X < OutGrid.nx; ++X)
        {
            const tvf::Vec3 CorePosition = OutGrid.Position(X, Y, Z);
            const FVector3d Position(CorePosition.x, CorePosition.y, CorePosition.z);
            double DistanceSquared = TNumericLimits<double>::Max();
            if (Spatial.FindNearestTriangle(Position, DistanceSquared) < 0 || !FMath::IsFinite(DistanceSquared))
            {
                OutError = TEXT("Accelerated scalar field could not query its source surface.");
                return false;
            }
            double Distance = FMath::Sqrt(FMath::Max(0.0, DistanceSquared));
            if (ShellHalfThicknessCm > 0.0)
            {
                Distance -= ShellHalfThicknessCm;
            }
            else
            {
                const double WindingNumber = Winding.FastWindingNumber(Position);
                if (!FMath::IsFinite(WindingNumber))
                {
                    OutError = TEXT("Accelerated winding query returned a non-finite value.");
                    return false;
                }
                if (FMath::Abs(WindingNumber) > 0.5)
                {
                    Distance = -Distance;
                }
            }
            bHasInterior |= Distance < 0.0;
            OutGrid.phi[OutGrid.Index(X, Y, Z)] = Distance;
        }
    }
    if (!bHasInterior)
    {
        OutError = TEXT("Scalar field contains no resolved interior sample; increase grid resolution or physical thickness.");
        return false;
    }
    tvf::AnalyzeScalarGrid(OutGrid);
    return true;
}

bool BuildVisualSolidifiedInput(FPreparedBakeInput& Input, TAtomic<bool>& bCancel,
                                tvf::ScalarGrid& OutGrid, uint64& OutPeakBytes, FString& OutError)
{
    using namespace UE::Geometry;
    const tvf::Bounds SourceCoreBounds = tvf::MeshBounds(Input.Mesh);
    const int32 SourceTriangleCount = static_cast<int32>(Input.Mesh.triangles.size());
    const tvf::MeshDiagnostics SourceTopology = tvf::ValidateClosedMesh(Input.Mesh, 1e-9, [&bCancel]() { return bCancel.Load(); });
    FDynamicMesh3 SourceMesh;
    if (!BuildDynamicMeshFromCore(Input.Mesh, true, SourceMesh, OutError))
    {
        return false;
    }

    const uint64 MemoryBudget = GetMemoryBudget(Input.Settings);
    const uint64 WorkBudget = GetWorkBudget(Input.Settings);
    const uint64 SourceBytes = SaturatingMultiply(static_cast<uint64>(SourceMesh.VertexCount()), sizeof(FVector3d) * 4ull) +
        SaturatingMultiply(static_cast<uint64>(SourceMesh.TriangleCount()), 128ull);
    OutPeakBytes = FMath::Max(OutPeakBytes, SourceBytes);
    if (SourceBytes > MemoryBudget)
    {
        OutError = TEXT("VisualSolidify triangle-soup input exceeds the configured working-memory budget.");
        return false;
    }

    const uint64 SolidifyCellEstimate = SaturatingMultiply(SaturatingMultiply(
        static_cast<uint64>(Input.Settings.SolidifyVoxelResolution),
        static_cast<uint64>(Input.Settings.SolidifyVoxelResolution)),
        static_cast<uint64>(Input.Settings.SolidifyVoxelResolution));
    const uint64 SolidifyQueryWeight = FMath::Max<uint64>(8ull,
        static_cast<uint64>(std::ceil(std::log2(static_cast<double>(FMath::Max(2, SourceMesh.TriangleCount()))))) * 4ull);
    if (SaturatingMultiply(SolidifyCellEstimate, SolidifyQueryWeight) > WorkBudget)
    {
        OutError = TEXT("VisualSolidify estimate exceeds the configured work budget; lower Solidify resolution or raise the budget.");
        return false;
    }

    FDynamicMeshAABBTree3 SourceSpatial(&SourceMesh);
    TFastWindingTree<FDynamicMesh3> SourceWinding(&SourceSpatial);
    const FAxisAlignedBox3d Bounds = SourceSpatial.GetBoundingBox();
    const double Extent = Bounds.MaxDim();
    if (!(Extent > 0.0) || !FMath::IsFinite(Extent))
    {
        OutError = TEXT("VisualSolidify input has invalid bounds.");
        return false;
    }
    const double CellSize = Extent / Input.Settings.SolidifyVoxelResolution;
    TImplicitSolidify<FDynamicMesh3> Solidify(&SourceMesh, &SourceSpatial, &SourceWinding);
    Solidify.MeshCellSize = CellSize;
    Solidify.ExtendBounds = CellSize * 2.0;
    Solidify.WindingThreshold = Input.Settings.SolidifyWindingThreshold;
    Solidify.SurfaceSearchSteps = 5;
    Solidify.bSolidAtBoundaries = true;
    Solidify.CancelF = [&bCancel]() { return bCancel.Load(); };
    FDynamicMesh3 SolidMesh(&Solidify.Generate());
    if (bCancel.Load())
    {
        return false;
    }
    if (SolidMesh.TriangleCount() == 0)
    {
        OutError = TEXT("VisualSolidify produced an empty shell; lower the winding threshold or improve the source silhouette.");
        return false;
    }
    const uint64 SolidBytes = SaturatingMultiply(static_cast<uint64>(SolidMesh.VertexCount()), sizeof(FVector3d) * 4ull) +
        SaturatingMultiply(static_cast<uint64>(SolidMesh.TriangleCount()), 160ull);
    OutPeakBytes = FMath::Max(OutPeakBytes, SourceBytes + SolidBytes);
    if (SourceBytes + SolidBytes > MemoryBudget)
    {
        OutError = TEXT("VisualSolidify output exceeds the configured working-memory budget.");
        return false;
    }

    if (!ConvertSolidMeshToCore(SolidMesh, Input.Mesh, OutError))
    {
        return false;
    }
    Input.ProcessingTriangleCount = static_cast<int32>(Input.Mesh.triangles.size());
    const tvf::MeshDiagnostics Diagnostics = tvf::ValidateClosedMesh(Input.Mesh, 1e-9, [&bCancel]() { return bCancel.Load(); });
    Input.Diagnostics.insert(Input.Diagnostics.end(), Diagnostics.diagnostics.begin(), Diagnostics.diagnostics.end());
    const tvf::Bounds RepairedBounds = tvf::MeshBounds(Input.Mesh);
    const double SourceExtent = std::max({SourceCoreBounds.Size().x, SourceCoreBounds.Size().y, SourceCoreBounds.Size().z, 1e-12});
    const double BoundsDeviation = std::max(tvf::Length(RepairedBounds.min - SourceCoreBounds.min),
        tvf::Length(RepairedBounds.max - SourceCoreBounds.max)) / SourceExtent;
    tvf::Diagnostic RepairTopology;
    RepairTopology.severity = tvf::DiagnosticSeverity::Quality;
    RepairTopology.stage = tvf::DiagnosticStage::SolidDomain;
    RepairTopology.code = tvf::DiagnosticCode::RepairChangedTopology;
    RepairTopology.message = TCHAR_TO_UTF8(*FString::Printf(TEXT("Visual Union Repair replaced source triangles %d -> %d; shell/cavity counts are recorded separately from repaired volume truth."),
        SourceTriangleCount, Input.ProcessingTriangleCount));
    RepairTopology.beforeCount = static_cast<uint64>(SourceTopology.connectedShells + SourceTopology.nestedCavities);
    RepairTopology.afterCount = static_cast<uint64>(Diagnostics.connectedShells + Diagnostics.nestedCavities);
    Input.Diagnostics.push_back(std::move(RepairTopology));
    tvf::Diagnostic RepairBounds;
    RepairBounds.severity = tvf::DiagnosticSeverity::Quality;
    RepairBounds.stage = tvf::DiagnosticStage::SolidDomain;
    RepairBounds.code = tvf::DiagnosticCode::RepairSurfaceDeviation;
    RepairBounds.message = "Visual Union Repair changed the source bounds; the repaired volume is the bake truth.";
    RepairBounds.measuredValue = BoundsDeviation;
    Input.Diagnostics.push_back(std::move(RepairBounds));
    if (!Diagnostics.valid)
    {
        OutError = FString::Printf(TEXT("VisualSolidify output validation failed: %s [boundary=%llu, non-manifold=%llu, winding=%llu, degenerate=%llu]"),
            UTF8_TO_TCHAR(Diagnostics.message.c_str()), static_cast<uint64>(Diagnostics.boundaryEdges),
            static_cast<uint64>(Diagnostics.nonManifoldEdges), static_cast<uint64>(Diagnostics.inconsistentEdges),
            static_cast<uint64>(Diagnostics.degenerateTriangles));
        OutError += FString(LINE_TERMINATOR) + FormatDiagnostics(Input.Diagnostics, true);
        return false;
    }
    return BuildAcceleratedGrid(SolidMesh, Input.Settings.LongestAxisCells, MemoryBudget, WorkBudget, bCancel,
        OutGrid, OutPeakBytes, OutError);
}

bool BuildOpenShellInput(FPreparedBakeInput& Input, TAtomic<bool>& bCancel,
                         tvf::ScalarGrid& OutGrid, uint64& OutPeakBytes, FString& OutError)
{
    UE::Geometry::FDynamicMesh3 SourceMesh;
    if (!BuildDynamicMeshFromCore(Input.Mesh, true, SourceMesh, OutError))
    {
        return false;
    }
    const double HalfThickness = Input.Settings.OpenShellThicknessCm * 0.5;
    const bool bBuilt = BuildAcceleratedGrid(SourceMesh, Input.Settings.LongestAxisCells,
        GetMemoryBudget(Input.Settings), GetWorkBudget(Input.Settings), bCancel,
        OutGrid, OutPeakBytes, OutError, HalfThickness);
    return bBuilt;
}

void SetStatus(UTAVisualFractureComponent* Component, const FString& Text, const bool bBusy, const float Progress)
{
    if (!IsValid(Component))
    {
        return;
    }
    Component->bBakeInProgress = bBusy;
    Component->BakeProgress = Progress;
    Component->LastBakeStatus = Text;
}

bool PrepareInput(UTAVisualFractureComponent* Component, FPreparedBakeInput& Out, FString& OutError)
{
    UStaticMeshComponent* Target = Component ? Component->ResolveTargetMeshComponent() : nullptr;
    UStaticMesh* Mesh = Target ? Target->GetStaticMesh() : nullptr;
    if (!Target || !Mesh)
    {
        OutError = TEXT("Bake requires a target Static Mesh Component with a Static Mesh.");
        return false;
    }
    if (Target->GetOwner() != Component->GetOwner())
    {
        OutError = TEXT("Target Static Mesh Component must belong to the fracture component's Actor.");
        return false;
    }

    const FVector Scale = Target->GetComponentScale();
    if (!FMath::IsFinite(Scale.X) || !FMath::IsFinite(Scale.Y) || !FMath::IsFinite(Scale.Z) ||
        Scale.X <= 0.0 || Scale.Y <= 0.0 || Scale.Z <= 0.0)
    {
        OutError = TEXT("Bake rejects zero, negative or non-finite component scale.");
        return false;
    }
    if (Component->BakeSettings.TargetChunkCount < 2)
    {
        OutError = TEXT("Target chunk count must be at least 2.");
        return false;
    }
    const FTVFBakeSettings& Settings = Component->BakeSettings;
    const bool bValidMode = Settings.InputMode == ETVFBakeInputMode::StrictClosed ||
        Settings.InputMode == ETVFBakeInputMode::VisualSolidify ||
        Settings.InputMode == ETVFBakeInputMode::OpenShellThickness;
    if (!bValidMode || Settings.LongestAxisCells < 8 || Settings.LongestAxisCells > 256 ||
        Settings.SolidifyVoxelResolution < 16 || Settings.SolidifyVoxelResolution > 256 ||
        !FMath::IsFinite(Settings.SolidifyWindingThreshold) || Settings.SolidifyWindingThreshold < 0.01 ||
        Settings.SolidifyWindingThreshold > 0.99 || Settings.MaxBakeWorkMillions < 100 ||
        static_cast<uint64>(Settings.MaxBakeWorkMillions) > TNumericLimits<uint64>::Max() / TestsPerMillion ||
        Settings.WorkingMemoryBudgetMB < 512 ||
        !FMath::IsFinite(Settings.Irregularity) || Settings.Irregularity < 0.0 || Settings.Irregularity > 1.0 ||
        !FMath::IsFinite(Settings.SizeVariation) || Settings.SizeVariation < 0.0 || Settings.SizeVariation > 1.0 ||
        Settings.MetricAxis.ContainsNaN() || Settings.MetricAxis.IsNearlyZero() ||
        !FMath::IsFinite(Settings.MetricAxisScale) || Settings.MetricAxisScale < 0.125 || Settings.MetricAxisScale > 8.0 ||
        Settings.DensityFocusPointLocal.ContainsNaN() || !FMath::IsFinite(Settings.DensityFocusRadiusCm) ||
        Settings.DensityFocusRadiusCm < 0.0 || !FMath::IsFinite(Settings.DensityFocusConcentration) ||
        Settings.DensityFocusConcentration < 0.0 || Settings.DensityFocusConcentration > 1.0 ||
        (Settings.DensityFocusConcentration > 0.0 && Settings.DensityFocusRadiusCm <= 0.0) ||
        !FMath::IsFinite(Settings.OpenShellThicknessCm) || Settings.OpenShellThicknessCm <= 0.0 ||
        Settings.MaxOutputTriangles < 10000 || !FMath::IsFinite(Settings.MinFinalChunkVolumeFraction) ||
        Settings.MinFinalChunkVolumeFraction < 0.0 || Settings.MinFinalChunkVolumeFraction > 0.1)
    {
        OutError = TEXT("Bake settings contain a non-finite or out-of-range geometry, Solidify, seed or performance-budget value.");
        return false;
    }

    const FMeshDescription* Description = Mesh->GetMeshDescription(0);
    if (!Description || Description->Triangles().Num() == 0)
    {
        OutError = TEXT("LOD0 MeshDescription is unavailable or empty; render buffers are not used as a fallback.");
        return false;
    }
    const uint64 EstimatedSourceBytes = SaturatingMultiply(static_cast<uint64>(Description->Vertices().Num()), 96ull) +
        SaturatingMultiply(static_cast<uint64>(Description->Triangles().Num()), 192ull);
    if (EstimatedSourceBytes > GetMemoryBudget(Settings))
    {
        OutError = TEXT("LOD0 estimated working set exceeds the configured memory budget; raise the budget or use a lighter source LOD.");
        return false;
    }

    const FStaticMeshConstAttributes Attributes(*Description);
    const TVertexAttributesConstRef<FVector3f> Positions = Attributes.GetVertexPositions();
    TMap<FVertexID, uint32> VertexToIndex;
    Out.Mesh.positions.reserve(Description->Vertices().Num());
    for (const FVertexID VertexId : Description->Vertices().GetElementIDs())
    {
        const FVector3f P = Positions[VertexId];
        const uint32 Index = static_cast<uint32>(Out.Mesh.positions.size());
        VertexToIndex.Add(VertexId, Index);
        Out.Mesh.positions.push_back({P.X * Scale.X, P.Y * Scale.Y, P.Z * Scale.Z});
    }
    Out.Mesh.triangles.reserve(Description->Triangles().Num());
    for (const FTriangleID TriangleId : Description->Triangles().GetElementIDs())
    {
        const TArrayView<const FVertexInstanceID> Corners = Description->GetTriangleVertexInstances(TriangleId);
        if (Corners.Num() != 3)
        {
            OutError = TEXT("LOD0 contains a non-triangle polygon after triangulation.");
            return false;
        }
        std::array<uint32_t, 3> Triangle{};
        for (int32 Corner = 0; Corner < 3; ++Corner)
        {
            const FVertexID VertexId = Description->GetVertexInstanceVertex(Corners[Corner]);
            const uint32* Index = VertexToIndex.Find(VertexId);
            if (!Index)
            {
                OutError = TEXT("LOD0 MeshDescription has an invalid vertex-instance reference.");
                return false;
            }
            Triangle[Corner] = *Index;
        }
        Out.Mesh.triangles.push_back(TVFMeshAdapter::ReverseTriangle(Triangle[0], Triangle[1], Triangle[2]));
    }

    Out.Settings = Component->BakeSettings;
    Out.SourceMeshPath = FSoftObjectPath(Mesh);
    Out.PositiveScale = Scale;
    Out.SourceTriangleCount = static_cast<int32>(Out.Mesh.triangles.size());
    Out.ProcessingTriangleCount = Out.SourceTriangleCount;
    Out.SourceGeometryHash = ComputeGeometryHash(*Description, Scale);
    Out.TopologySettingsHash = ComputeTopologySettingsHash(Out.Settings);
    Out.Signature = FTAVisualFractureBakeService::BuildBakeSignature(Component, &OutError);
    return !Out.Signature.IsEmpty();
}

void AppendTriangle(FTVFChunkMeshSection& Section, const tvf::SurfaceTriangle& Triangle,
                    const tvf::Vec3& Centroid, const bool bInterior)
{
    const FVector P0 = ToUE(Triangle.p[0]);
    const FVector P1 = ToUE(Triangle.p[1]);
    const FVector P2 = ToUE(Triangle.p[2]);
    const FVector Normal = FVector::CrossProduct(P1 - P0, P2 - P0).GetSafeNormal();
    FVector Tangent = (P1 - P0).GetSafeNormal();
    if (Tangent.IsNearlyZero())
    {
        Tangent = FVector::CrossProduct(FVector::UpVector, Normal).GetSafeNormal();
        if (Tangent.IsNearlyZero()) Tangent = FVector::ForwardVector;
    }
    const FVector AbsNormal = Normal.GetAbs();
    const FLinearColor Color = bInterior ? FLinearColor(1.0f, 0.45f, 0.03f, 1.0f) :
                                           FLinearColor(0.03f, 0.20f, 1.0f, 1.0f);
    const FVector Points[3] = {P0, P1, P2};
    const int32 First = Section.Vertices.Num();
    for (const FVector& Point : Points)
    {
        Section.Vertices.Add(Point - ToUE(Centroid));
        Section.Normals.Add(Normal);
        Section.Tangents.Add(Tangent);
        if (AbsNormal.Z >= AbsNormal.X && AbsNormal.Z >= AbsNormal.Y)
        {
            Section.UV0.Add(FVector2D(Point.X, Point.Y));
        }
        else if (AbsNormal.X >= AbsNormal.Y)
        {
            Section.UV0.Add(FVector2D(Point.Y, Point.Z));
        }
        else
        {
            Section.UV0.Add(FVector2D(Point.X, Point.Z));
        }
        Section.RestSpaceData.Add(FVector4(Point.X, Point.Y, Point.Z, bInterior ? 1.0 : 0.0));
        Section.VertexColors.Add(Color);
    }
    const std::array<int32, 3> UnrealTriangle = TVFMeshAdapter::ReverseTriangle(First, First + 1, First + 2);
    Section.Triangles.Add(UnrealTriangle[0]);
    Section.Triangles.Add(UnrealTriangle[1]);
    Section.Triangles.Add(UnrealTriangle[2]);
}

void PopulateAsset(UTAVisualFractureAsset* Asset, const FBakeWorkerResult& Result)
{
    Asset->BakeVersion = 3;
    Asset->bLegacyRebakeRequired = false;
    Asset->SourceMesh = TSoftObjectPtr<UStaticMesh>(Result.Input.SourceMeshPath);
    Asset->BakeSignature = Result.Input.Signature;
    Asset->SourceGeometryHash = Result.Input.SourceGeometryHash;
    Asset->TopologySettingsHash = Result.Input.TopologySettingsHash;
    Asset->AppliedPositiveScale = Result.Input.PositiveScale;
    Asset->SourceTriangleCount = Result.Input.SourceTriangleCount;
    Asset->BakeSettings = Result.Input.Settings;
    Asset->CoordinateContract.Version = static_cast<int32>(Result.Bake.coordinateContract.version);
    Asset->CoordinateContract.OriginCm = ToUE(Result.Bake.coordinateContract.originCm);
    Asset->CoordinateContract.ScaleCm = Result.Bake.coordinateContract.scaleCm;
    Asset->CoordinateContract.MetricAxis = ToUE(Result.Bake.coordinateContract.metric.axis);
    Asset->CoordinateContract.MetricAxisScale = Result.Bake.coordinateContract.metric.axisScale;
    Asset->FieldGridNx = static_cast<int32>(Result.GridNx);
    Asset->FieldGridNy = static_cast<int32>(Result.GridNy);
    Asset->FieldGridNz = static_cast<int32>(Result.GridNz);
    Asset->FieldGridStepCm = Result.GridStepCm;
    Asset->SeedPositions.Reset(Result.Seeds.size());
    Asset->SeedWeights.Reset(Result.Seeds.size());
    for (const tvf::Seed& Seed : Result.Seeds)
    {
        Asset->SeedPositions.Add(ToUE(Seed.position));
        Asset->SeedWeights.Add(Seed.weight);
    }
    Asset->InputVolumeCm3 = Result.Bake.inputVolume;
    Asset->OutputVolumeCm3 = Result.Bake.outputVolume;
    Asset->DiscardedVolumeCm3 = Result.Bake.discardedVolume;
    Asset->BakedTriangleCount = Result.BakedTriangleCount;
    Asset->bHasCenterInteriorPoint = Result.bHasCenterInteriorPoint;
    Asset->CenterInteriorPoint = ToUE(Result.CenterInteriorPoint);
    Asset->Chunks.Reset(Result.Bake.chunks.size());
    for (const tvf::Chunk& Source : Result.Bake.chunks)
    {
        FTVFChunkRenderData& Chunk = Asset->Chunks.AddDefaulted_GetRef();
        Chunk.ChunkId = static_cast<int32>(Source.id);
        Chunk.Centroid = ToUE(Source.centroid);
        Chunk.RadiusCm = Source.radius;
        Chunk.VolumeCm3 = Source.volume;
        for (const tvf::SurfaceTriangle& Triangle : Source.surface)
        {
            const bool bInterior = Triangle.kind == tvf::FaceKind::Interior;
            AppendTriangle(bInterior ? Chunk.Interior : Chunk.Exterior, Triangle, Source.centroid, bInterior);
        }
    }
    Asset->Bonds.Reset(Result.Bake.bonds.size());
    for (const tvf::Bond& Source : Result.Bake.bonds)
    {
        FTVFBondData& Bond = Asset->Bonds.AddDefaulted_GetRef();
        Bond.ChunkA = static_cast<int32>(Source.chunkA);
        Bond.ChunkB = static_cast<int32>(Source.chunkB);
        Bond.AreaCm2 = Source.area;
        Bond.Centroid = ToUE(Source.centroid);
        Bond.NormalAToB = ToUE(Source.normalAToB);
    }
    Asset->CanonicalVertices.Reset(Result.Bake.canonicalVertices.size());
    for (const tvf::CanonicalVertex& Source : Result.Bake.canonicalVertices)
    {
        FTVFCanonicalVertexData& Vertex = Asset->CanonicalVertices.AddDefaulted_GetRef();
        Vertex.VertexId = static_cast<int64>(Source.id);
        Vertex.Position = ToUE(Source.position);
        Vertex.SupportIds.Reserve(Source.supportIds.size());
        for (const uint64 Support : Source.supportIds) Vertex.SupportIds.Add(static_cast<int64>(Support));
    }
    Asset->InterfacePatches.Reset(Result.Bake.interfacePatches.size());
    for (const tvf::InterfacePatch& Source : Result.Bake.interfacePatches)
    {
        FTVFInterfacePatchData& Patch = Asset->InterfacePatches.AddDefaulted_GetRef();
        Patch.PatchId = static_cast<int64>(Source.id);
        Patch.ChunkA = static_cast<int32>(Source.chunkA);
        Patch.ChunkB = static_cast<int32>(Source.chunkB);
        Patch.AreaCm2 = Source.area;
        Patch.CanonicalVertexIds.Reserve(Source.canonicalVertexIds.size());
        for (const uint64 VertexId : Source.canonicalVertexIds) Patch.CanonicalVertexIds.Add(static_cast<int64>(VertexId));
        Patch.Triangles.Reserve(Source.triangles.size());
        for (const uint32 Index : Source.triangles) Patch.Triangles.Add(static_cast<int32>(Index));
    }
    Asset->Diagnostics.Reset(Result.Bake.diagnostics.size());
    for (const tvf::Diagnostic& Source : Result.Bake.diagnostics)
    {
        FTVFDiagnostic& Diagnostic = Asset->Diagnostics.AddDefaulted_GetRef();
        Diagnostic.Severity = ToRuntimeSeverity(Source.severity);
        Diagnostic.Stage = ToRuntimeStage(Source.stage);
        Diagnostic.Code = DiagnosticCodeName(Source.code);
        Diagnostic.Message = UTF8_TO_TCHAR(Source.message.c_str());
        Diagnostic.SeedId = Source.seedId == tvf::Diagnostic::InvalidId ? INDEX_NONE : static_cast<int64>(Source.seedId);
        Diagnostic.TetId = Source.tetId == tvf::Diagnostic::InvalidId ? INDEX_NONE : static_cast<int64>(Source.tetId);
        Diagnostic.TriangleId = Source.triangleId == tvf::Diagnostic::InvalidId ? INDEX_NONE : static_cast<int64>(Source.triangleId);
        Diagnostic.PatchId = Source.patchId == tvf::Diagnostic::InvalidId ? INDEX_NONE : static_cast<int64>(Source.patchId);
        Diagnostic.MeasuredValue = Source.measuredValue;
        Diagnostic.Limit = Source.limit;
        Diagnostic.BeforeCount = static_cast<int64>(Source.beforeCount);
        Diagnostic.AfterCount = static_cast<int64>(Source.afterCount);
    }
    const tvf::BakeStatistics& Stats = Result.Bake.statistics;
    Asset->PerformanceStats.ActiveTets = static_cast<int64>(Stats.activeTets);
    Asset->PerformanceStats.CandidateSitesP50 = static_cast<int64>(Stats.candidateSitesP50);
    Asset->PerformanceStats.CandidateSitesP95 = static_cast<int64>(Stats.candidateSitesP95);
    Asset->PerformanceStats.CandidateSitesMax = static_cast<int64>(Stats.candidateSitesMax);
    Asset->PerformanceStats.ActualCellTetPairs = static_cast<int64>(Stats.actualCellTetPairs);
    Asset->PerformanceStats.PlaneTests = static_cast<int64>(Stats.planeTests);
    Asset->PerformanceStats.CanonicalVertices = static_cast<int64>(Stats.canonicalVertices);
    Asset->PerformanceStats.SharedFacets = static_cast<int64>(Stats.sharedFacets);
    Asset->PerformanceStats.InterfacePatches = static_cast<int64>(Stats.interfacePatches);
    Asset->PerformanceStats.PeakTemporaryBytes = static_cast<int64>(FMath::Max<uint64>(Stats.peakTemporaryBytes, Result.PeakEstimatedBytes));
    Asset->PerformanceStats.OutputTriangles = static_cast<int64>(Stats.outputTriangles);
    Asset->PerformanceStats.TargetSeeds = static_cast<int32>(Stats.targetSeeds);
    Asset->PerformanceStats.ValidSeeds = static_cast<int32>(Stats.validSeeds);
    Asset->PerformanceStats.FinalChunks = static_cast<int32>(Stats.finalChunks);
    Asset->PerformanceStats.VisualDebris = static_cast<int32>(Stats.visualDebris);
}

UTAVisualFractureAsset* ResolveDestinationAsset(UTAVisualFractureComponent* Component, const FSoftObjectPath& SourcePath)
{
    if (IsValid(Component->BakedAsset))
    {
        return Component->BakedAsset;
    }
    const FString MeshName = SourcePath.GetAssetName();
    const FString BasePackage = FString::Printf(TEXT("/Game/TAVisualFracture/Baked/TVF_%s_%s"), *MeshName,
        *Component->InstanceGuid.ToString(EGuidFormats::Digits));
    FString PackageName;
    FString AssetName;
    FAssetToolsModule::GetModule().Get().CreateUniqueAssetName(BasePackage, TEXT(""), PackageName, AssetName);
    UPackage* Package = CreatePackage(*PackageName);
    if (!Package)
    {
        return nullptr;
    }
    UTAVisualFractureAsset* Asset = NewObject<UTAVisualFractureAsset>(Package, *AssetName,
        RF_Public | RF_Standalone | RF_Transactional);
    if (Asset)
    {
        FAssetRegistryModule::AssetCreated(Asset);
    }
    return Asset;
}
}

struct FTAVisualFractureBakeService::FBakeJob
{
    TAtomic<bool> bCancel{false};
    TFuture<void> Future;
};

FTAVisualFractureBakeService& FTAVisualFractureBakeService::Get()
{
    static FTAVisualFractureBakeService Instance;
    return Instance;
}

FString FTAVisualFractureBakeService::BuildBakeSignature(const UTAVisualFractureComponent* Component,
                                                         FString* OutError)
{
    const UStaticMeshComponent* Target = Component ? Component->ResolveTargetMeshComponent() : nullptr;
    const UStaticMesh* Mesh = Target ? Target->GetStaticMesh() : nullptr;
    if (!Target || !Mesh)
    {
        if (OutError) *OutError = TEXT("Cannot build a signature without a target mesh.");
        return {};
    }
    const FVector S = Target->GetComponentScale();
    const FTVFBakeSettings& B = Component->BakeSettings;
    const FMeshDescription* Description = Mesh->GetMeshDescription(0);
    if (!Description)
    {
        if (OutError) *OutError = TEXT("Cannot build a geometry signature without LOD0 MeshDescription.");
        return {};
    }
    const FString GeometryHash = ComputeGeometryHash(*Description, S);
    const FString TopologyHash = ComputeTopologySettingsHash(B);
    return FString::Printf(TEXT("TVFv3|%s|Geometry=%s|Topology=%s|BakeBudget=%lld,%d,%lld"),
        *FSoftObjectPath(Mesh).ToString(), *GeometryHash, *TopologyHash,
        B.MaxBakeWorkMillions, B.WorkingMemoryBudgetMB, B.MaxOutputTriangles);
}

void FTAVisualFractureBakeService::StartBake(UTAVisualFractureComponent* Component)
{
    check(IsInGameThread());
    if (!IsValid(Component))
    {
        return;
    }
    if (ActiveJobs.Contains(TWeakObjectPtr<UTAVisualFractureComponent>(Component)))
    {
        SetStatus(Component, TEXT("A bake is already active; cancel it before starting another."), true, Component->BakeProgress);
        return;
    }

    FPreparedBakeInput Prepared;
    FString Error;
    if (!PrepareInput(Component, Prepared, Error))
    {
        SetStatus(Component, Error, false, 0.0f);
        return;
    }

    TSharedPtr<FBakeJob, ESPMode::ThreadSafe> Job = MakeShared<FBakeJob, ESPMode::ThreadSafe>();
    TWeakObjectPtr<UTAVisualFractureComponent> WeakComponent(Component);
    ActiveJobs.Add(WeakComponent, Job);
    const TCHAR* PreparationText = Prepared.Settings.InputMode == ETVFBakeInputMode::VisualSolidify ?
        TEXT("Prepared LOD0 clone; building a Visual Union Repair shell on a worker thread...") :
        (Prepared.Settings.InputMode == ETVFBakeInputMode::OpenShellThickness ?
            TEXT("Prepared LOD0 clone; building a two-sided physical shell field on a worker thread...") :
            TEXT("Prepared LOD0 clone; validating the Closed Volume on a worker thread..."));
    SetStatus(Component, PreparationText, true, 0.10f);

    Job->Future = Async(EAsyncExecution::ThreadPool, [this, WeakComponent, Job, Prepared = MoveTemp(Prepared)]() mutable
    {
        TSharedRef<FBakeWorkerResult, ESPMode::ThreadSafe> Result = MakeShared<FBakeWorkerResult, ESPMode::ThreadSafe>();
        Result->Input = MoveTemp(Prepared);
        Result->PeakEstimatedBytes = SaturatingMultiply(static_cast<uint64>(Result->Input.Mesh.positions.size()), 96ull) +
            SaturatingMultiply(static_cast<uint64>(Result->Input.Mesh.triangles.size()), 192ull);
        try
        {
            if (Job->bCancel.Load())
            {
                Result->bCancelled = true;
            }
            else
            {
                tvf::ScalarGrid Grid;
                bool bGridReady = false;
                if (Result->Input.Settings.InputMode == ETVFBakeInputMode::VisualSolidify)
                {
                    bGridReady = BuildVisualSolidifiedInput(Result->Input, Job->bCancel, Grid,
                        Result->PeakEstimatedBytes, Result->Error);
                    if (Job->bCancel.Load())
                    {
                        Result->bCancelled = true;
                        bGridReady = false;
                    }
                }
                else if (Result->Input.Settings.InputMode == ETVFBakeInputMode::OpenShellThickness)
                {
                    bGridReady = BuildOpenShellInput(Result->Input, Job->bCancel, Grid,
                        Result->PeakEstimatedBytes, Result->Error);
                    tvf::Diagnostic ShellDiagnostic;
                    ShellDiagnostic.severity = tvf::DiagnosticSeverity::Quality;
                    ShellDiagnostic.stage = tvf::DiagnosticStage::SolidDomain;
                    ShellDiagnostic.code = tvf::DiagnosticCode::ShellSurfaceFusion;
                    ShellDiagnostic.message = "Open Shell uses symmetric unsigned-distance thickness; surfaces closer than the requested thickness can merge.";
                    ShellDiagnostic.measuredValue = Result->Input.Settings.OpenShellThicknessCm;
                    Result->InputDiagnostics.push_back(std::move(ShellDiagnostic));
                    if (Job->bCancel.Load())
                    {
                        Result->bCancelled = true;
                        bGridReady = false;
                    }
                }
                else
                {
                    const tvf::MeshDiagnostics Diagnostics = tvf::ValidateClosedMesh(Result->Input.Mesh, 1e-9, [Job]() { return Job->bCancel.Load(); });
                    Result->InputDiagnostics.insert(Result->InputDiagnostics.end(), Diagnostics.diagnostics.begin(), Diagnostics.diagnostics.end());
                    if (!Diagnostics.valid)
                    {
                        Result->Error = FString::Printf(TEXT("Closed Volume validation failed: %s [boundary=%llu, non-manifold=%llu, winding=%llu, degenerate=%llu, vertex-link=%llu, intersections=%llu, coplanar=%llu]. Select Visual Union Repair or Open Shell Thickness only when their changed solid semantics are acceptable."),
                            UTF8_TO_TCHAR(Diagnostics.message.c_str()), static_cast<uint64>(Diagnostics.boundaryEdges),
                            static_cast<uint64>(Diagnostics.nonManifoldEdges), static_cast<uint64>(Diagnostics.inconsistentEdges),
                            static_cast<uint64>(Diagnostics.degenerateTriangles), static_cast<uint64>(Diagnostics.disconnectedVertexLinks),
                            static_cast<uint64>(Diagnostics.selfIntersections), static_cast<uint64>(Diagnostics.coplanarOverlaps));
                        Result->Error += FString(LINE_TERMINATOR) + FormatDiagnostics(Result->InputDiagnostics, true);
                    }
                    else
                    {
                        UE::Geometry::FDynamicMesh3 StrictMesh;
                        if (!BuildDynamicMeshFromCore(Result->Input.Mesh, false, StrictMesh, Result->Error))
                        {
                            bGridReady = false;
                        }
                        else
                        {
                            bGridReady = BuildAcceleratedGrid(StrictMesh, Result->Input.Settings.LongestAxisCells,
                                GetMemoryBudget(Result->Input.Settings), GetWorkBudget(Result->Input.Settings),
                                Job->bCancel, Grid, Result->PeakEstimatedBytes, Result->Error);
                        }
                    }
                }

                if (bGridReady)
                {
                    Result->GridNx = Grid.nx;
                    Result->GridNy = Grid.ny;
                    Result->GridNz = Grid.nz;
                    Result->GridStepCm = Grid.step;
                    Result->InputDiagnostics.insert(Result->InputDiagnostics.end(),
                        Result->Input.Diagnostics.begin(), Result->Input.Diagnostics.end());
                    AsyncTask(ENamedThreads::GameThread, [WeakComponent]()
                    {
                        if (WeakComponent.IsValid()) SetStatus(WeakComponent.Get(), TEXT("SDF complete; generating deterministic seeds..."), true, 0.45f);
                    });
                    if (Job->bCancel.Load())
                    {
                        Result->bCancelled = true;
                    }
                    else
                    {
                        const tvf::Bounds SourceBounds = tvf::MeshBounds(Result->Input.Mesh);
                        const tvf::Vec3 BoundsCenter = (SourceBounds.min + SourceBounds.max) * 0.5;
                        double BestDistanceSquared = TNumericLimits<double>::Max();
                        for (uint32_t Z = 0; Z < Grid.nz; ++Z)
                        for (uint32_t Y = 0; Y < Grid.ny; ++Y)
                        for (uint32_t X = 0; X < Grid.nx; ++X)
                        {
                            if (Grid.phi[Grid.Index(X, Y, Z)] >= 0.0) continue;
                            const tvf::Vec3 P = Grid.Position(X, Y, Z);
                            const double DistanceSquared = tvf::LengthSquared(P - BoundsCenter);
                            if (DistanceSquared < BestDistanceSquared)
                            {
                                BestDistanceSquared = DistanceSquared;
                                Result->CenterInteriorPoint = P;
                                Result->bHasCenterInteriorPoint = true;
                            }
                        }

                        tvf::SolidDomainSettings DomainSettings;
                        DomainSettings.maxPieces = FMath::Max<uint64>(4096ull, GetMemoryBudget(Result->Input.Settings) / 1024ull);
                        DomainSettings.maxTemporaryBytes = GetMemoryBudget(Result->Input.Settings);
                        DomainSettings.shouldCancel = [Job]() { return Job->bCancel.Load(); };
                        tvf::SolidDomain Domain = tvf::BuildSolidDomain(Grid, DomainSettings);
                        Result->PeakEstimatedBytes = FMath::Max(Result->PeakEstimatedBytes, Domain.peakTemporaryBytes);
                        if (Domain.HasFatal())
                        {
                            Result->InputDiagnostics.insert(Result->InputDiagnostics.end(), Domain.diagnostics.begin(), Domain.diagnostics.end());
                            Result->Bake.diagnostics = Result->InputDiagnostics;
                            Result->Error = FString(TEXT("Solid-domain construction stopped on fatal diagnostics:")) + LINE_TERMINATOR +
                                FormatDiagnostics(Result->InputDiagnostics, true);
                        }
                        else
                        {
                            tvf::SeedSettings Seeds;
                            Seeds.targetCount = static_cast<uint32_t>(Result->Input.Settings.TargetChunkCount);
                            Seeds.randomSeed = static_cast<uint32_t>(Result->Input.Settings.RandomSeed);
                            Seeds.irregularity = Result->Input.Settings.Irregularity;
                            Seeds.sizeVariation = Result->Input.Settings.SizeVariation;
                            Seeds.metric.axis = ToTVF(Result->Input.Settings.MetricAxis);
                            Seeds.metric.axisScale = Result->Input.Settings.MetricAxisScale;
                            Seeds.impactPoint = ToTVF(Result->Input.Settings.DensityFocusPointLocal * Result->Input.PositiveScale);
                            Seeds.impactRadius = Result->Input.Settings.DensityFocusRadiusCm;
                            Seeds.impactConcentration = Result->Input.Settings.DensityFocusConcentration;
                            Seeds.maxSelectionTests = GetWorkBudget(Result->Input.Settings);
                            Seeds.shouldCancel = [Job]() { return Job->bCancel.Load(); };
                            const std::vector<tvf::Seed> GeneratedSeeds = tvf::GenerateSeeds(Domain, Seeds, &Result->InputDiagnostics);
                            Result->Seeds = GeneratedSeeds;
                            AsyncTask(ENamedThreads::GameThread, [WeakComponent]()
                            {
                                if (WeakComponent.IsValid()) SetStatus(WeakComponent.Get(), TEXT("Cached solid pieces and volume Seeds complete; building certified Power cells..."), true, 0.65f);
                            });
                            if (Job->bCancel.Load())
                            {
                                Result->bCancelled = true;
                            }
                            else
                            {
                                tvf::BakeSettings Settings;
                                Settings.metric = Seeds.metric;
                                Settings.maxPlaneTests = GetWorkBudget(Result->Input.Settings);
                                Settings.maxPieces = FMath::Max<uint64>(4096ull, GetMemoryBudget(Result->Input.Settings) / 2048ull);
                                Settings.maxTemporaryBytes = GetMemoryBudget(Result->Input.Settings);
                                Settings.maxOutputTriangles = static_cast<uint64>(Result->Input.Settings.MaxOutputTriangles);
                                Settings.minFinalChunkVolumeFraction = Result->Input.Settings.MinFinalChunkVolumeFraction;
                                Settings.shouldCancel = [Job]() { return Job->bCancel.Load(); };
                                Result->Bake = tvf::Bake(Domain, GeneratedSeeds, Settings);
                                Result->Bake.diagnostics.insert(Result->Bake.diagnostics.begin(),
                                    Result->InputDiagnostics.begin(), Result->InputDiagnostics.end());
                                for (const tvf::Chunk& Chunk : Result->Bake.chunks)
                                {
                                    Result->BakedTriangleCount += static_cast<int32>(Chunk.surface.size());
                                }
                                if (Job->bCancel.Load())
                                {
                                    Result->bCancelled = true;
                                }
                                else if (Result->Bake.HasFatal())
                                {
                                    Result->Error = FString(TEXT("Bake stopped on fatal diagnostics:")) + LINE_TERMINATOR +
                                        FormatDiagnostics(Result->Bake.diagnostics, true);
                                }
                                else if (Result->Bake.chunks.empty() || Result->BakedTriangleCount <= 0)
                                {
                                    Result->Error = TEXT("Bake returned no chunks or triangles.");
                                }
                                else if (!Result->bHasCenterInteriorPoint)
                                {
                                    Result->Error = TEXT("SDF contains no valid negative sample for the center-interior preset.");
                                }
                                else
                                {
                                    Result->bSucceeded = true;
                                }
                            }
                        }
                    }
                }
            }
        }
        catch (const std::exception& Exception)
        {
            tvf::Diagnostic Diagnostic;
            Diagnostic.severity = tvf::DiagnosticSeverity::FatalGeometry;
            Diagnostic.stage = tvf::DiagnosticStage::Seed;
            Diagnostic.code = FString(UTF8_TO_TCHAR(Exception.what())).Contains(TEXT("budget"), ESearchCase::IgnoreCase) ?
                tvf::DiagnosticCode::BudgetExceeded : tvf::DiagnosticCode::InsufficientSeeds;
            Diagnostic.message = Exception.what();
            Result->Bake.diagnostics = Result->InputDiagnostics;
            Result->Bake.diagnostics.push_back(std::move(Diagnostic));
            Result->Error = FString(TEXT("Bake stopped on a structured core failure:")) + LINE_TERMINATOR +
                FormatDiagnostics(Result->Bake.diagnostics, true);
        }
        catch (...)
        {
            tvf::Diagnostic Diagnostic;
            Diagnostic.severity = tvf::DiagnosticSeverity::FatalGeometry;
            Diagnostic.stage = tvf::DiagnosticStage::Validation;
            Diagnostic.code = tvf::DiagnosticCode::InvalidInput;
            Diagnostic.message = "Bake failed with an unknown core exception.";
            Result->Bake.diagnostics = Result->InputDiagnostics;
            Result->Bake.diagnostics.push_back(std::move(Diagnostic));
            Result->Error = FString(TEXT("Bake stopped on a structured core failure:")) + LINE_TERMINATOR +
                FormatDiagnostics(Result->Bake.diagnostics, true);
        }

        AsyncTask(ENamedThreads::GameThread, [this, WeakComponent, Job, Result]()
        {
            const TSharedPtr<FBakeJob, ESPMode::ThreadSafe>* Current = ActiveJobs.Find(WeakComponent);
            if (!Current || *Current != Job)
            {
                return;
            }
            ActiveJobs.Remove(WeakComponent);
            UTAVisualFractureComponent* Component = WeakComponent.Get();
            if (!Component)
            {
                return;
            }
            if (Result->bCancelled || Job->bCancel.Load())
            {
                SetStatus(Component, TEXT("Bake cancelled; no asset was written."), false, 0.0f);
                return;
            }
            FString CurrentError;
            if (BuildBakeSignature(Component, &CurrentError) != Result->Input.Signature)
            {
                SetStatus(Component, TEXT("Target mesh, scale or bake settings changed while baking; result discarded."), false, 0.0f);
                return;
            }
            if (!Result->bSucceeded)
            {
                SetStatus(Component, Result->Error.IsEmpty() ? TEXT("Bake failed without producing an asset.") : Result->Error, false, 0.0f);
                return;
            }

            UTAVisualFractureAsset* Asset = ResolveDestinationAsset(Component, Result->Input.SourceMeshPath);
            if (!Asset)
            {
                SetStatus(Component, TEXT("Bake succeeded, but the destination asset could not be created; no partial asset was written."), false, 0.0f);
                return;
            }
            Component->Modify();
            Asset->Modify();
            PopulateAsset(Asset, *Result);
            Component->BakedAsset = Asset;
            Component->InitializeImpactRadiiIfNeeded();
            Asset->PostEditChange();
            Asset->MarkPackageDirty();
            Component->PostEditChange();
            Component->MarkPackageDirty();
            const uint64 WorkBudget = GetWorkBudget(Result->Input.Settings);
            const uint64 MemoryBudget = GetMemoryBudget(Result->Input.Settings);
            const bool bPerformanceWarning = Result->Bake.planeTestCount > WorkBudget - WorkBudget / 4ull ||
                Result->PeakEstimatedBytes > MemoryBudget - MemoryBudget / 4ull;
            const FString Prefix = bPerformanceWarning ? TEXT("Bake complete (performance-budget warning).") : TEXT("Bake complete.");
            const TCHAR* ModeName = Result->Input.Settings.InputMode == ETVFBakeInputMode::VisualSolidify ?
                TEXT("VisualUnionRepair") : (Result->Input.Settings.InputMode == ETVFBakeInputMode::OpenShellThickness ?
                    TEXT("OpenShellThickness") : TEXT("ClosedVolume"));
            FString Completion = FString::Printf(TEXT("%s Mode=%s Seeds=%d/%d Chunks=%d Bonds=%d Triangles=%d Source/Processed=%d/%d ActiveTets=%lld Candidates(P50/P95/Max)=%lld/%lld/%lld CellTet=%lld PlaneTests=%.3fM Canonical(V/P)=%lld/%lld Peak=%.1fMB VolumeResidual=%.9g cm^3"),
                *Prefix, ModeName, Asset->PerformanceStats.TargetSeeds, Asset->PerformanceStats.ValidSeeds,
                Asset->Chunks.Num(), Asset->Bonds.Num(), Asset->BakedTriangleCount,
                Result->Input.SourceTriangleCount, Result->Input.ProcessingTriangleCount,
                Asset->PerformanceStats.ActiveTets,
                Asset->PerformanceStats.CandidateSitesP50, Asset->PerformanceStats.CandidateSitesP95,
                Asset->PerformanceStats.CandidateSitesMax, Asset->PerformanceStats.ActualCellTetPairs,
                static_cast<double>(Result->Bake.planeTestCount) / TestsPerMillion,
                Asset->PerformanceStats.CanonicalVertices, Asset->PerformanceStats.InterfacePatches,
                static_cast<double>(Asset->PerformanceStats.PeakTemporaryBytes) / BytesPerMegabyte,
                Asset->InputVolumeCm3 - Asset->OutputVolumeCm3 - Asset->DiscardedVolumeCm3);
            const FString DiagnosticText = FormatDiagnostics(Result->Bake.diagnostics, false);
            if (!DiagnosticText.IsEmpty()) Completion += LINE_TERMINATOR TEXT("Diagnostics:") LINE_TERMINATOR + DiagnosticText;
            SetStatus(Component, Completion, false, 1.0f);
        });
    });
}

void FTAVisualFractureBakeService::CancelBake(UTAVisualFractureComponent* Component)
{
    check(IsInGameThread());
    if (const TSharedPtr<FBakeJob, ESPMode::ThreadSafe>* Job = ActiveJobs.Find(TWeakObjectPtr<UTAVisualFractureComponent>(Component)))
    {
        (*Job)->bCancel.Store(true);
        SetStatus(Component, TEXT("Cancellation requested; waiting for the current pure-math stage boundary..."), true,
            Component ? Component->BakeProgress : 0.0f);
    }
}

void FTAVisualFractureBakeService::CancelAllAndWait()
{
    check(IsInGameThread());
    TArray<TSharedPtr<FBakeJob, ESPMode::ThreadSafe>> Jobs;
    ActiveJobs.GenerateValueArray(Jobs);
    for (const TSharedPtr<FBakeJob, ESPMode::ThreadSafe>& Job : Jobs)
    {
        Job->bCancel.Store(true);
    }
    for (const TSharedPtr<FBakeJob, ESPMode::ThreadSafe>& Job : Jobs)
    {
        if (Job->Future.IsValid()) Job->Future.Wait();
    }
    FTaskGraphInterface::Get().ProcessThreadUntilIdle(ENamedThreads::GameThread);
    ActiveJobs.Empty();
}
