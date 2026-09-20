#pragma once
#include "TVFMath.h"
#include <algorithm>
#include <array>
#include <cstdint>
#include <functional>
#include <limits>
#include <string>
#include <vector>

namespace tvf {
// Geometry: centimeters, double precision, object-local right-handed algebra.
struct TriangleMesh {std::vector<Vec3> positions;std::vector<std::array<uint32_t,3>> triangles;};

enum class DiagnosticSeverity:uint8_t {FatalGeometry=0,Recovered=1,Quality=2};
enum class DiagnosticStage:uint8_t {Input=0,SolidDomain=1,Seed=2,PowerPartition=3,CanonicalTopology=4,Validation=5,Performance=6};
enum class DiagnosticCode:uint16_t {
    InvalidInput,BoundaryEdge,NonManifoldEdge,InconsistentWinding,DegenerateTriangle,
    DisconnectedVertexLink,BowTieVertex,SelfIntersection,CoplanarOverlap,ShellIntersection,
    AmbiguousShellNesting,RepairChangedTopology,RepairSurfaceDeviation,ShellSurfaceFusion,
    NonFiniteValue,DegenerateTetra,DegenerateFieldPlane,DuplicateSeedResampled,
    InsufficientSeeds,UnexpectedEmptyCell,BudgetExceeded,UnmatchedInterface,
    NonManifoldOutput,InternalInterface,VolumeResidual,CoverageGap,CoverageOverlap
};

struct Diagnostic {
    static constexpr uint64_t InvalidId=std::numeric_limits<uint64_t>::max();
    DiagnosticSeverity severity=DiagnosticSeverity::Quality;
    DiagnosticStage stage=DiagnosticStage::Input;
    DiagnosticCode code=DiagnosticCode::InvalidInput;
    std::string message;
    uint64_t seedId=InvalidId,tetId=InvalidId,triangleId=InvalidId,patchId=InvalidId;
    double measuredValue=0,limit=0;
    uint64_t beforeCount=0,afterCount=0;
};

struct MeshDiagnostics {
    bool valid=false;
    size_t boundaryEdges=0,nonManifoldEdges=0,inconsistentEdges=0,degenerateTriangles=0;
    size_t disconnectedVertexLinks=0,bowTieVertices=0,selfIntersections=0,coplanarOverlaps=0;
    size_t connectedShells=0,nestedCavities=0;
    std::string message;
    std::vector<Diagnostic> diagnostics;
};
TAVISUALFRACTURECORE_API MeshDiagnostics ValidateClosedMesh(const TriangleMesh& mesh,double relativeWeldTolerance=1e-9,const std::function<bool()>& shouldCancel={});
TAVISUALFRACTURECORE_API Bounds MeshBounds(const TriangleMesh& mesh);
TAVISUALFRACTURECORE_API double SignedDistanceToMesh(const TriangleMesh& mesh,Vec3 point); // winding sign, exact triangle distance; O(triangles)

struct ScalarGrid {
    Vec3 origin;double step=1;uint32_t nx=0,ny=0,nz=0; // vertex counts, NOT cell counts
    std::vector<double> phi; // negative = solid, x fastest
    double maxSamplingError=0,minResolvedNegativeThickness=0;
    uint32_t negativeComponentCount=0;
    size_t Index(uint32_t x,uint32_t y,uint32_t z)const{return (size_t(z)*ny+y)*nx+x;}
    Vec3 Position(uint32_t x,uint32_t y,uint32_t z)const{return origin+Vec3(x,y,z)*step;}
    Bounds GetBounds()const{return {origin,Position(nx-1,ny-1,nz-1)};}
};
TAVISUALFRACTURECORE_API ScalarGrid SampleField(Bounds domain,uint32_t longestAxisCells,const std::function<double(Vec3)>& signedField);
TAVISUALFRACTURECORE_API void AnalyzeScalarGrid(ScalarGrid& grid);
TAVISUALFRACTURECORE_API ScalarGrid MeshToGrid(const TriangleMesh& mesh,uint32_t longestAxisCells); // validates closed oriented mesh, adds exterior padding
TAVISUALFRACTURECORE_API double SampleGrid(const ScalarGrid& grid,Vec3 point); // same piecewise affine Freudenthal field as Bake; outside = positive

struct Seed {Vec3 position;double weight=0;}; // weight: cm^2 in shared metric
struct SeedSettings {
    uint32_t targetCount=24,randomSeed=1337;
    double irregularity=0.55,sizeVariation=0.4;
    Metric metric;
    Vec3 impactPoint={0,0,0};double impactRadius=0,impactConcentration=0;
    uint64_t maxCandidateCount=8000000; // legacy wrapper: maximum cached SolidTetPiece count; 0 = unlimited
    uint64_t maxSelectionTests=5000000000ull; // actual seed-selection work budget; 0 = unlimited
    std::function<bool()> shouldCancel;
};
TAVISUALFRACTURECORE_API std::vector<Seed> GenerateSeeds(const ScalarGrid& grid,const SeedSettings& settings);

struct ConvexFace {
    std::vector<Vec3> vertices;
    std::vector<std::vector<uint64_t>> vertexSupportIds;
    uint64_t sourcePlaneId=0;
};
struct SolidSimplex {std::array<Vec3,4> vertices;double volume=0;};
struct SolidTetPiece {
    uint64_t tetraId=0;
    uint32_t componentId=0;
    std::vector<ConvexFace> faces;
    std::vector<SolidSimplex> samplingTetrahedra;
    double volume=0;
    Vec3 centroid;
    Bounds bounds;
};
struct SolidDomain {
    ScalarGrid grid;
    std::vector<SolidTetPiece> pieces;
    double volume=0;
    uint64_t activeTets=0;
    uint64_t peakTemporaryBytes=0;
    std::vector<Diagnostic> diagnostics;
    bool HasFatal()const
    {
        return std::any_of(diagnostics.begin(), diagnostics.end(), [](const Diagnostic& diagnostic)
        {
            return diagnostic.severity == DiagnosticSeverity::FatalGeometry;
        });
    }
};
struct SolidDomainSettings {
    double relativeTolerance=1e-8;
    uint64_t maxPieces=8000000ull;
    uint64_t maxTemporaryBytes=8ull*1024ull*1024ull*1024ull;
    std::function<bool()> shouldCancel;
};
TAVISUALFRACTURECORE_API SolidDomain BuildSolidDomain(const ScalarGrid& grid,const SolidDomainSettings& settings={});
TAVISUALFRACTURECORE_API std::vector<Seed> GenerateSeeds(const SolidDomain& domain,const SeedSettings& settings,std::vector<Diagnostic>* diagnostics=nullptr);

enum class FaceKind:uint32_t {Exterior=0,Interior=1};
struct SurfaceTriangle {
    std::array<Vec3,3> p;
    std::array<uint64_t,3> canonicalVertexIds{Diagnostic::InvalidId,Diagnostic::InvalidId,Diagnostic::InvalidId};
    FaceKind kind=FaceKind::Exterior;
    int32_t neighborSeed=-1; // site id, NOT chunk id; exterior = -1
    uint64_t patchId=Diagnostic::InvalidId;
};
struct Chunk {
    uint32_t id=0,seedId=0,componentId=0;
    std::vector<SurfaceTriangle> surface; // object-local cm, outward winding
    double volume=0; // cm^3
    Vec3 centroid;
    double radius=0; // containing sphere about centroid, cm
    Bounds bounds;
};
struct Bond {uint32_t chunkA=0,chunkB=0;double area=0;Vec3 centroid,normalAToB;};
struct CoordinateContract {uint32_t version=2;Vec3 originCm;double scaleCm=1;Metric metric;};
struct CanonicalVertex {uint64_t id=0;Vec3 position;std::vector<uint64_t> supportIds;};
struct InterfacePatch {
    uint64_t id=0;
    uint32_t chunkA=0,chunkB=0;
    std::vector<uint64_t> canonicalVertexIds;
    std::vector<uint32_t> triangles;
    double area=0;
};
struct BakeStatistics {
    uint64_t activeTets=0,candidateSitesP50=0,candidateSitesP95=0,candidateSitesMax=0;
    uint64_t actualCellTetPairs=0,planeTests=0,canonicalVertices=0,sharedFacets=0,interfacePatches=0;
    uint64_t peakTemporaryBytes=0,outputTriangles=0;
    uint32_t targetSeeds=0,validSeeds=0,finalChunks=0,visualDebris=0;
};
struct BakeSettings {
    Metric metric;
    double relativeTolerance=1e-8;
    double minVolume=1e-12; // numerical cull only, cm^3; report every removed volume
    uint64_t maxPlaneTests=5000000000ull; // actual clipping work budget; 0 = unlimited
    uint64_t maxPieces=8000000ull; // temporary topology/memory budget; 0 = unlimited
    uint64_t maxTemporaryBytes=8ull*1024ull*1024ull*1024ull;
    uint64_t maxOutputTriangles=5000000ull;
    double minFinalChunkVolumeFraction=0;
    std::function<bool()> shouldCancel;
};
struct BakeResult {
    std::vector<Chunk> chunks;
    std::vector<Bond> bonds;
    double inputVolume=0,outputVolume=0,discardedVolume=0;
    uint32_t emptySeedCount=0;
    uint64_t planeTestCount=0,pieceCount=0;
    std::vector<std::string> warnings;
    std::vector<Diagnostic> diagnostics;
    std::vector<CanonicalVertex> canonicalVertices;
    std::vector<InterfacePatch> interfacePatches;
    CoordinateContract coordinateContract;
    BakeStatistics statistics;
    bool HasFatal()const
    {
        return std::any_of(diagnostics.begin(), diagnostics.end(), [](const Diagnostic& diagnostic)
        {
            return diagnostic.severity == DiagnosticSeverity::FatalGeometry;
        });
    }
};
// Shared Freudenthal tetrahedra: clip affine phi<=0, then shared metric power cells.
// Cancel shared tetra faces; split each seed into volume-connected components.
TAVISUALFRACTURECORE_API BakeResult Bake(const ScalarGrid& grid,const std::vector<Seed>& seeds,const BakeSettings& settings={});
TAVISUALFRACTURECORE_API BakeResult Bake(const SolidDomain& domain,const std::vector<Seed>& seeds,const BakeSettings& settings={});
}
