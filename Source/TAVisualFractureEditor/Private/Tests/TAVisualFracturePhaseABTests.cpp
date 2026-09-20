#include "Misc/AutomationTest.h"
#include "TAVisualFractureAsset.h"
#include "TAVisualFractureMeshAdapter.h"
#include "TAVisualFractureTypes.h"
#include "TVFFracture.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <exception>

#if WITH_DEV_AUTOMATION_TESTS
namespace
{
tvf::ScalarGrid MakeCubeField(const uint32 Cells)
{
    return tvf::SampleField({{-55, -55, -55}, {55, 55, 55}}, Cells, [](const tvf::Vec3 P)
    {
        return std::max({std::abs(P.x), std::abs(P.y), std::abs(P.z)}) - 50.0;
    });
}

tvf::TriangleMesh MakeCubeMesh(const double HalfExtent, const tvf::Vec3 Offset = {}, const bool bReverse = false)
{
    tvf::TriangleMesh Mesh;
    Mesh.positions = {
        Offset + tvf::Vec3(-HalfExtent, -HalfExtent, -HalfExtent), Offset + tvf::Vec3(HalfExtent, -HalfExtent, -HalfExtent),
        Offset + tvf::Vec3(HalfExtent, HalfExtent, -HalfExtent), Offset + tvf::Vec3(-HalfExtent, HalfExtent, -HalfExtent),
        Offset + tvf::Vec3(-HalfExtent, -HalfExtent, HalfExtent), Offset + tvf::Vec3(HalfExtent, -HalfExtent, HalfExtent),
        Offset + tvf::Vec3(HalfExtent, HalfExtent, HalfExtent), Offset + tvf::Vec3(-HalfExtent, HalfExtent, HalfExtent)};
    Mesh.triangles = {{0,2,1},{0,3,2},{4,5,6},{4,6,7},{0,1,5},{0,5,4},
        {1,2,6},{1,6,5},{2,3,7},{2,7,6},{3,0,4},{3,4,7}};
    if (bReverse) for (std::array<uint32_t, 3>& Triangle : Mesh.triangles) std::swap(Triangle[1], Triangle[2]);
    return Mesh;
}

void AppendMesh(tvf::TriangleMesh& Destination, const tvf::TriangleMesh& Source)
{
    const uint32 Base = static_cast<uint32>(Destination.positions.size());
    Destination.positions.insert(Destination.positions.end(), Source.positions.begin(), Source.positions.end());
    for (std::array<uint32_t, 3> Triangle : Source.triangles)
    {
        for (uint32_t& Index : Triangle) Index += Base;
        Destination.triangles.push_back(Triangle);
    }
}

bool HasCode(const std::vector<tvf::Diagnostic>& Diagnostics, const tvf::DiagnosticCode Code)
{
    return std::any_of(Diagnostics.begin(), Diagnostics.end(), [Code](const tvf::Diagnostic& Diagnostic)
    {
        return Diagnostic.code == Code;
    });
}

uint64 TopologyHash(const tvf::BakeResult& Bake)
{
    uint64 Hash = 1469598103934665603ull;
    auto Add = [&Hash](const uint64 Value) { Hash = (Hash ^ Value) * 1099511628211ull; };
    Add(Bake.chunks.size());
    Add(Bake.bonds.size());
    for (const tvf::CanonicalVertex& Vertex : Bake.canonicalVertices)
    {
        Add(Vertex.id);
        Add(static_cast<uint64>(std::llround(Vertex.position.x * 1e9)));
        Add(static_cast<uint64>(std::llround(Vertex.position.y * 1e9)));
        Add(static_cast<uint64>(std::llround(Vertex.position.z * 1e9)));
        for (const uint64 Support : Vertex.supportIds) Add(Support);
    }
    return Hash;
}

tvf::BakeResult BakeCount(const tvf::SolidDomain& Domain, const uint32 Count, const uint32 RandomSeed)
{
    tvf::SeedSettings Seeds;
    Seeds.targetCount = Count;
    Seeds.randomSeed = RandomSeed;
    Seeds.impactConcentration = 0.0;
    Seeds.impactRadius = 0.0;
    Seeds.maxSelectionTests = 1000000000ull;
    std::vector<tvf::Diagnostic> Diagnostics;
    const std::vector<tvf::Seed> Generated = tvf::GenerateSeeds(Domain, Seeds, &Diagnostics);
    tvf::BakeSettings Settings;
    Settings.maxPlaneTests = 5000000000ull;
    Settings.maxPieces = 8000000ull;
    Settings.maxOutputTriangles = 5000000ull;
    tvf::BakeResult Result = tvf::Bake(Domain, Generated, Settings);
    Result.diagnostics.insert(Result.diagnostics.begin(), Diagnostics.begin(), Diagnostics.end());
    return Result;
}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTVFPhaseABContractTest,
    "TA.VisualFracture.PhaseAB.ContractsAndCanonicalInterfaces",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTVFPhaseABContractTest::RunTest(const FString& Parameters)
{
    TestEqual(TEXT("StrictClosed serialized value"), static_cast<uint8>(ETVFBakeInputMode::StrictClosed), uint8(0));
    TestEqual(TEXT("VisualSolidify serialized value"), static_cast<uint8>(ETVFBakeInputMode::VisualSolidify), uint8(1));
    TestEqual(TEXT("OpenShellThickness appended value"), static_cast<uint8>(ETVFBakeInputMode::OpenShellThickness), uint8(2));

    tvf::TriangleMesh UnrealWindingCube = MakeCubeMesh(10.0);
    for (std::array<uint32_t, 3>& Triangle : UnrealWindingCube.triangles)
    {
        Triangle = TVFMeshAdapter::ReverseTriangle(Triangle[0], Triangle[1], Triangle[2]);
    }
    TestFalse(TEXT("Raw Unreal winding is negative in the right-handed Core contract"),
        tvf::ValidateClosedMesh(UnrealWindingCube).valid);
    for (std::array<uint32_t, 3>& Triangle : UnrealWindingCube.triangles)
    {
        Triangle = TVFMeshAdapter::ReverseTriangle(Triangle[0], Triangle[1], Triangle[2]);
    }
    const tvf::MeshDiagnostics ConvertedUnrealCube = tvf::ValidateClosedMesh(UnrealWindingCube);
    TestTrue(TEXT("UE-to-Core winding conversion restores one positive closed shell"),
        ConvertedUnrealCube.valid && ConvertedUnrealCube.connectedShells == 1 && ConvertedUnrealCube.nestedCavities == 0);

    tvf::ScalarGrid Grid = MakeCubeField(12);
    tvf::SolidDomain Domain = tvf::BuildSolidDomain(Grid);
    TestFalse(TEXT("Cube SolidDomain has no fatal diagnostic"), Domain.HasFatal());
    TestTrue(TEXT("Cube has cached SolidTetPieces"), !Domain.pieces.empty());

    tvf::TriangleMesh OpenCube = MakeCubeMesh(10.0);
    OpenCube.triangles.resize(10);
    TestTrue(TEXT("Open Cube is rejected"), !tvf::ValidateClosedMesh(OpenCube).valid);
    tvf::TriangleMesh Plane;
    Plane.positions = {{0, 0, 0}, {10, 0, 0}, {0, 10, 0}};
    Plane.triangles = {{0, 1, 2}};
    TestEqual(TEXT("Plane exposes three boundary edges"),
        static_cast<uint64>(tvf::ValidateClosedMesh(Plane).boundaryEdges), uint64(3));

    tvf::TriangleMesh Intersecting;
    AppendMesh(Intersecting, MakeCubeMesh(20.0, {-10, 0, 0}));
    AppendMesh(Intersecting, MakeCubeMesh(20.0, {10, 0, 0}));
    const tvf::MeshDiagnostics IntersectionDiagnostics = tvf::ValidateClosedMesh(Intersecting);
    TestTrue(TEXT("Intersecting shells are rejected"), !IntersectionDiagnostics.valid && IntersectionDiagnostics.selfIntersections > 0);

    tvf::TriangleMesh Nested = MakeCubeMesh(40.0);
    AppendMesh(Nested, MakeCubeMesh(12.0, {}, true));
    const tvf::MeshDiagnostics NestedDiagnostics = tvf::ValidateClosedMesh(Nested);
    TestTrue(TEXT("Nested reversed shell is recognized as a cavity"), NestedDiagnostics.valid && NestedDiagnostics.nestedCavities == 1);

    const tvf::ScalarGrid Torus = tvf::SampleField({{-60, -60, -30}, {60, 60, 30}}, 20, [](const tvf::Vec3 P)
    {
        const double Ring = std::sqrt(P.x * P.x + P.y * P.y) - 35.0;
        return std::sqrt(Ring * Ring + P.z * P.z) - 12.0;
    });
    TestTrue(TEXT("Torus center remains outside"), tvf::SampleGrid(Torus, {}) > 0.0);
    TestFalse(TEXT("Torus SolidDomain is valid"), tvf::BuildSolidDomain(Torus).HasFatal());

    const tvf::ScalarGrid ThickPlane = tvf::SampleField({{-30, -30, -10}, {30, 30, 10}}, 16, [](const tvf::Vec3 P)
    {
        return std::abs(P.z) - 2.5;
    });
    TestFalse(TEXT("Open-shell thickness field creates a solid domain"), tvf::BuildSolidDomain(ThickPlane).HasFatal());

    tvf::BakeResult A = BakeCount(Domain, 64, 9137);
    tvf::BakeResult B = BakeCount(Domain, 64, 9137);
    TestFalse(TEXT("64 Seed canonical bake has no fatal diagnostic"), A.HasFatal());
    TestEqual(TEXT("Deterministic topology hash"), TopologyHash(A), TopologyHash(B));
    TestFalse(TEXT("No unmatched interface"), HasCode(A.diagnostics, tvf::DiagnosticCode::UnmatchedInterface));
    TestFalse(TEXT("No non-manifold output"), HasCode(A.diagnostics, tvf::DiagnosticCode::NonManifoldOutput));
    TestFalse(TEXT("No unexpected empty Cell"), HasCode(A.diagnostics, tvf::DiagnosticCode::UnexpectedEmptyCell));
    TestTrue(TEXT("Canonical interfaces emitted"), !A.interfacePatches.empty());
    for (const tvf::InterfacePatch& Patch : A.interfacePatches)
    {
        TestTrue(TEXT("Interface Patch has two different owners"), Patch.chunkA != Patch.chunkB);
        TestTrue(TEXT("Interface Patch has triangulation"), !Patch.triangles.empty() && Patch.triangles.size() % 3 == 0);
    }

    tvf::SeedSettings FocusOff;
    FocusOff.targetCount = 8;
    FocusOff.impactConcentration = 0.0;
    FocusOff.impactRadius = 0.0;
    TestEqual(TEXT("Disabled Density Focus permits zero radius"),
        static_cast<uint64>(tvf::GenerateSeeds(Domain, FocusOff).size()), uint64(8));

    UTAVisualFractureAsset* NewAsset = NewObject<UTAVisualFractureAsset>();
    TestEqual(TEXT("New asset uses v3 contract"), NewAsset->BakeVersion, 3);
    UTAVisualFractureAsset* LegacyAsset = NewObject<UTAVisualFractureAsset>();
    LegacyAsset->BakeVersion = 2;
    LegacyAsset->PostLoad();
    TestTrue(TEXT("Legacy v2 asset is marked for rebake"), LegacyAsset->bLegacyRebakeRequired);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTVFBakeGeometryRegressionTest,
    "TA.VisualFracture.PhaseAB.DefaultBakeAndOrientedVolume",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTVFBakeGeometryRegressionTest::RunTest(const FString& Parameters)
{
    // Six Freudenthal tetrahedra in [-1,1]^3, clipped at x=1/4.
    // Analytic volume = 1.25*2*2 = 5. This catches mixed inward tetra
    // faces/outward caps even when an absolute signed-volume sum looks valid.
    tvf::ScalarGrid CutGrid;
    CutGrid.origin={-1,-1,-1};CutGrid.step=2;CutGrid.nx=CutGrid.ny=CutGrid.nz=2;
    for(uint32 Z=0;Z<2;++Z)for(uint32 Y=0;Y<2;++Y)for(uint32 X=0;X<2;++X)
        CutGrid.phi.push_back(CutGrid.Position(X,Y,Z).x-0.25);
    const tvf::SolidDomain CutDomain=tvf::BuildSolidDomain(CutGrid);
    TestFalse(TEXT("Analytic clipped domain has no Fatal"),CutDomain.HasFatal());
    TestTrue(TEXT("Clipped volume agrees with analytic 5 cm3"),std::abs(CutDomain.volume-5.0)<1e-10);
    for(const tvf::SolidTetPiece& Piece:CutDomain.pieces){
        double DecomposedVolume=0;
        for(const tvf::SolidSimplex& Simplex:Piece.samplingTetrahedra)DecomposedVolume+=Simplex.volume;
        TestTrue(TEXT("Sampling decomposition preserves piece volume"),std::abs(DecomposedVolume-Piece.volume)<1e-11);
        for(const tvf::ConvexFace& Face:Piece.faces){
            const tvf::Vec3 Normal=tvf::Cross(Face.vertices[1]-Face.vertices[0],Face.vertices[2]-Face.vertices[0]);
            TestTrue(TEXT("Every cached face points away from the interior"),tvf::Dot(Normal,Piece.centroid-Face.vertices[0])<=1e-12);
        }
    }

    tvf::ScalarGrid FullGrid=CutGrid;FullGrid.phi.assign(8,-1.0);
    const tvf::SolidDomain FullDomain=tvf::BuildSolidDomain(FullGrid);
    // The Power bisector x=y is exactly a shared Freudenthal plane.
    const std::vector<tvf::Seed> CoplanarSeeds={{{-0.5,0.5,0},0},{{0.5,-0.5,0},0}};
    const tvf::BakeResult Coplanar=tvf::Bake(FullDomain,CoplanarSeeds);
    TestFalse(TEXT("Coplanar two-owner split has no Fatal"),Coplanar.HasFatal());
    TestEqual(TEXT("Coplanar split retains both chunks"),static_cast<int32>(Coplanar.chunks.size()),2);
    TestTrue(TEXT("Coplanar split preserves 8 cm3"),std::abs(Coplanar.outputVolume-8.0)<1e-9);

    const FTVFBakeSettings Defaults;
    const tvf::SolidDomain Domain=tvf::BuildSolidDomain(MakeCubeField(Defaults.LongestAxisCells));
    tvf::SeedSettings Seeds;
    Seeds.targetCount=Defaults.TargetChunkCount;Seeds.randomSeed=Defaults.RandomSeed;
    Seeds.irregularity=Defaults.Irregularity;Seeds.sizeVariation=Defaults.SizeVariation;
    Seeds.impactConcentration=Defaults.DensityFocusConcentration;Seeds.impactRadius=Defaults.DensityFocusRadiusCm;
    const auto Sites=tvf::GenerateSeeds(Domain,Seeds);
    const tvf::BakeResult Result=tvf::Bake(Domain,Sites);
    for(const tvf::Diagnostic& Diagnostic:Result.diagnostics)
        if(Diagnostic.severity==tvf::DiagnosticSeverity::FatalGeometry)AddError(UTF8_TO_TCHAR(Diagnostic.message.c_str()));
    TestFalse(TEXT("Default 24 seeds / 24 cells / variation 0.75 bake succeeds"),Result.HasFatal());
    TestEqual(TEXT("No positive local volume discarded"),Result.discardedVolume,0.0);
    TestTrue(TEXT("Default output volume conserved"),std::abs(Result.outputVolume-Domain.volume)<Domain.volume*1e-8);
    for(const tvf::Chunk& Chunk:Result.chunks){
        double SurfaceVolume=0;tvf::Vec3 AreaVector{};
        for(const tvf::SurfaceTriangle& Triangle:Chunk.surface){
            const tvf::Vec3 A=Triangle.p[0]-Chunk.centroid,B=Triangle.p[1]-Chunk.centroid,C=Triangle.p[2]-Chunk.centroid;
            SurfaceVolume+=tvf::Dot(A,tvf::Cross(B,C))/6.0;
            AreaVector+=tvf::Cross(B-A,C-A)*0.5;
        }
        TestTrue(TEXT("Rendered shell has positive volume matching its chunk"),SurfaceVolume>0&&std::abs(SurfaceVolume-Chunk.volume)<std::max(1e-7,Chunk.volume*1e-7));
        TestTrue(TEXT("Rendered shell has zero area-vector residual"),tvf::Length(AreaVector)<std::max(1e-7,Chunk.radius*Chunk.radius*1e-7));
    }
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTVFSeedArtControlsTest,
    "TA.VisualFracture.PhaseAB.SeedArtControls",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTVFSeedArtControlsTest::RunTest(const FString& Parameters)
{
    const tvf::SolidDomain Domain=tvf::BuildSolidDomain(MakeCubeField(16));
    tvf::SeedSettings Settings;Settings.targetCount=24;Settings.randomSeed=1337;
    Settings.irregularity=0;Settings.sizeVariation=0;
    const auto Regular=tvf::GenerateSeeds(Domain,Settings);
    Settings.irregularity=1;
    const auto Random=tvf::GenerateSeeds(Domain,Settings);
    bool PositionsDiffer=false;
    for(size_t I=0;I<Regular.size();++I)
        PositionsDiffer|=tvf::LengthSquared(Regular[I].position-Random[I].position)>1e-16;
    TestTrue(TEXT("Irregularity changes Seed positions at a fixed random seed"),PositionsDiffer);
    Settings.irregularity=0.5;
    const auto Mixed=tvf::GenerateSeeds(Domain,Settings);
    const auto Repeat=tvf::GenerateSeeds(Domain,Settings);
    Settings.sizeVariation=1;
    const auto Varied=tvf::GenerateSeeds(Domain,Settings);
    bool WeightsDiffer=false;
    for(size_t I=0;I<Mixed.size();++I){
        TestEqual(TEXT("Repeat position is deterministic"),tvf::LengthSquared(Mixed[I].position-Repeat[I].position),0.0);
        TestEqual(TEXT("Size Variation does not move Seeds"),tvf::LengthSquared(Mixed[I].position-Varied[I].position),0.0);
        TestEqual(TEXT("Zero Size Variation has zero weight"),Mixed[I].weight,0.0);
        TestTrue(TEXT("Seed stays in the same piecewise-affine solid field"),tvf::SampleGrid(Domain.grid,Mixed[I].position)<0);
        WeightsDiffer|=Varied[I].weight!=0;
        for(size_t J=0;J<Varied.size();++J)if(I!=J){
            const double D2=Settings.metric.DistanceSquared(Varied[I].position-Varied[J].position);
            TestTrue(TEXT("Weight bound retains each Seed's ownership"),std::abs(Varied[I].weight-Varied[J].weight)<=0.4*D2+1e-12);
        }
    }
    TestTrue(TEXT("Size Variation changes Power weights"),WeightsDiffer);
    const auto UnweightedBake=tvf::Bake(Domain,Mixed);
    const auto WeightedBake=tvf::Bake(Domain,Varied);
    TestFalse(TEXT("Mixed Seed unweighted bake has no Fatal"),UnweightedBake.HasFatal());
    TestFalse(TEXT("Mixed Seed weighted bake has no Fatal"),WeightedBake.HasFatal());
    if(!UnweightedBake.HasFatal()&&!WeightedBake.HasFatal())
        TestTrue(TEXT("Size Variation changes baked geometry"),TopologyHash(UnweightedBake)!=TopologyHash(WeightedBake));
    Settings.shouldCancel=[](){return true;};
    bool Cancelled=false;
    try{tvf::GenerateSeeds(Domain,Settings);}catch(const std::exception&){Cancelled=true;}
    TestTrue(TEXT("Seed selection honors cancellation"),Cancelled);
    Settings.shouldCancel={};Settings.maxSelectionTests=1;
    bool BudgetStopped=false;
    try{tvf::GenerateSeeds(Domain,Settings);}catch(const std::exception&){BudgetStopped=true;}
    TestTrue(TEXT("Seed selection honors work budget"),BudgetStopped);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTVFPhaseABScaleGradientTest,
    "TA.VisualFracture.PhaseAB.ScaleGradient",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::PerfFilter)

bool FTVFPhaseABScaleGradientTest::RunTest(const FString& Parameters)
{
    const tvf::SolidDomain Domain = tvf::BuildSolidDomain(MakeCubeField(16));
    const uint32 Counts[] = {64, 100, 244, 256, 1024, 4096};
    for (const uint32 Count : Counts)
    {
        const tvf::BakeResult Result = BakeCount(Domain, Count, 1337);
        AddInfo(FString::Printf(TEXT("Seeds=%u Chunks=%u P50/P95/Max=%llu/%llu/%llu PlaneTests=%llu PeakBytes=%llu Triangles=%llu"),
            Count, static_cast<uint32>(Result.chunks.size()), Result.statistics.candidateSitesP50,
            Result.statistics.candidateSitesP95, Result.statistics.candidateSitesMax,
            Result.statistics.planeTests, Result.statistics.peakTemporaryBytes,
            Result.statistics.outputTriangles));
        TestFalse(FString::Printf(TEXT("Seeds=%u has no fatal diagnostic"), Count), Result.HasFatal());
        TestFalse(FString::Printf(TEXT("Seeds=%u has no unmatched interface"), Count),
            HasCode(Result.diagnostics, tvf::DiagnosticCode::UnmatchedInterface));
        TestFalse(FString::Printf(TEXT("Seeds=%u has no coverage gap"), Count),
            HasCode(Result.diagnostics, tvf::DiagnosticCode::CoverageGap));
    }
    return true;
}
#endif
