#include "TAVisualFractureAsset.h"
#include "Serialization/CustomVersion.h"

namespace
{
const FGuid GTAVisualFractureAssetVersionGuid(0xDB21E4A7, 0x56BC4D50, 0xA839620E, 0x128EEC53);
enum class ETAVisualFractureAssetVersion : int32
{
    BeforeCustomVersion = 0,
    CanonicalTopologyV3 = 3,
    Latest = CanonicalTopologyV3
};
FCustomVersionRegistration GTAVisualFractureAssetVersionRegistration(
    GTAVisualFractureAssetVersionGuid,
    static_cast<int32>(ETAVisualFractureAssetVersion::Latest),
    TEXT("TAVisualFractureAssetVersion"));
}

void UTAVisualFractureAsset::Serialize(FArchive& Ar)
{
    Super::Serialize(Ar);
    Ar.UsingCustomVersion(GTAVisualFractureAssetVersionGuid);
}

void UTAVisualFractureAsset::PostLoad()
{
    Super::PostLoad();
    const int32 Version = GetLinkerCustomVersion(GTAVisualFractureAssetVersionGuid);
    bLegacyRebakeRequired = Version < static_cast<int32>(ETAVisualFractureAssetVersion::CanonicalTopologyV3) ||
        BakeVersion < 3 || CanonicalVertices.IsEmpty();
}

bool UTAVisualFractureAsset::HasUsableTopology(FString* OutReason) const
{
    auto SectionIsValid = [](const FTVFChunkMeshSection& Section)
    {
        const int32 VertexCount = Section.Vertices.Num();
        if (Section.Triangles.Num() % 3 != 0 || Section.Normals.Num() != VertexCount ||
            Section.UV0.Num() != VertexCount || Section.RestSpaceData.Num() != VertexCount ||
            Section.VertexColors.Num() != VertexCount || Section.Tangents.Num() != VertexCount)
        {
            return false;
        }
        for (const int32 Index : Section.Triangles)
        {
            if (Index < 0 || Index >= VertexCount) return false;
        }
        return true;
    };

    bool bValid = !SourceMesh.IsNull() && !AppliedPositiveScale.ContainsNaN() && AppliedPositiveScale.X > 0.0 &&
        AppliedPositiveScale.Y > 0.0 && AppliedPositiveScale.Z > 0.0 && Chunks.Num() > 0 && BakedTriangleCount > 0;
    int32 CountedTriangles = 0;
    TSet<int32> ChunkIds;
    for (const FTVFChunkRenderData& Chunk : Chunks)
    {
        bValid &= Chunk.ChunkId >= 0 && !ChunkIds.Contains(Chunk.ChunkId) && FMath::IsFinite(Chunk.RadiusCm) &&
            Chunk.RadiusCm > 0.0 && FMath::IsFinite(Chunk.VolumeCm3) && Chunk.VolumeCm3 > 0.0 &&
            !Chunk.Centroid.ContainsNaN() && SectionIsValid(Chunk.Exterior) && SectionIsValid(Chunk.Interior);
        ChunkIds.Add(Chunk.ChunkId);
        CountedTriangles += (Chunk.Exterior.Triangles.Num() + Chunk.Interior.Triangles.Num()) / 3;
    }
    for (const FTVFBondData& Bond : Bonds)
    {
        bValid &= Chunks.IsValidIndex(Bond.ChunkA) && Chunks.IsValidIndex(Bond.ChunkB) && Bond.ChunkA != Bond.ChunkB &&
            FMath::IsFinite(Bond.AreaCm2) && Bond.AreaCm2 > 0.0 && !Bond.Centroid.ContainsNaN() &&
            !Bond.NormalAToB.ContainsNaN();
    }
    bValid &= CountedTriangles == BakedTriangleCount;
    if (BakeVersion >= 3 && !bLegacyRebakeRequired)
    {
        bValid &= CoordinateContract.Version >= 2 && FMath::IsFinite(CoordinateContract.ScaleCm) &&
            CoordinateContract.ScaleCm > 0.0 && !CoordinateContract.OriginCm.ContainsNaN() &&
            SeedPositions.Num() == SeedWeights.Num() && CanonicalVertices.Num() > 0;
        for (const FTVFDiagnostic& Diagnostic : Diagnostics)
        {
            bValid &= Diagnostic.Severity != ETVFDiagnosticSeverity::FatalGeometry;
        }
        TSet<int64> CanonicalIds;
        for (const FTVFCanonicalVertexData& Vertex : CanonicalVertices)
        {
            bValid &= Vertex.VertexId >= 0 && !CanonicalIds.Contains(Vertex.VertexId) &&
                !Vertex.Position.ContainsNaN() && !Vertex.SupportIds.IsEmpty();
            CanonicalIds.Add(Vertex.VertexId);
        }
        for (const FTVFInterfacePatchData& Patch : InterfacePatches)
        {
            bValid &= Patch.PatchId >= 0 && Chunks.IsValidIndex(Patch.ChunkA) && Chunks.IsValidIndex(Patch.ChunkB) &&
                Patch.ChunkA != Patch.ChunkB && Patch.Triangles.Num() % 3 == 0 &&
                FMath::IsFinite(Patch.AreaCm2) && Patch.AreaCm2 > 0.0;
            for (const int64 VertexId : Patch.CanonicalVertexIds) bValid &= CanonicalIds.Contains(VertexId);
            for (const int32 Index : Patch.Triangles) bValid &= Patch.CanonicalVertexIds.IsValidIndex(Index);
        }
    }
    if (!bValid && OutReason)
    {
        *OutReason = TEXT("Baked asset has invalid source data, section arrays, Chunk metrics, Bond endpoints or triangle statistics.");
    }
    return bValid;
}
