#pragma once

#include <array>

namespace TVFMeshAdapter
{
// MeshDescription, DynamicMesh and ProceduralMesh use Unreal's left-handed
// front-face convention.  TAVisualFractureCore uses right-handed algebra for
// signed volume, winding and canonical topology.  Every UE/Core boundary must
// therefore exchange the second and third triangle corners exactly once.
template <typename IndexType>
std::array<IndexType, 3> ReverseTriangle(const IndexType A, const IndexType B, const IndexType C)
{
    return {A, C, B};
}
}
