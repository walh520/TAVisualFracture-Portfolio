# Attribution and authorship

## Implementation boundary

TAVisualFracture is an independent Unreal Engine implementation and integration of established physical methods. The repository does not claim ownership of the underlying mathematical methods. The implementation work presented here is the plugin architecture, data contracts, editor workflow, temporary rendering path, collision adapter, diagnostics, and validation pipeline.

TAVisualFracture 是基于成熟物理方法完成的 Unreal Engine 独立实现与工程集成。本仓库不宣称拥有底层数学方法；公开的实现工作包括插件架构、数据合同、编辑器工作流、临时渲染路径、碰撞适配、诊断系统和验证流程。

## Reference material

- `TAVisualFracture` technical and interface guidance supplied with the project. It defines the intended geometry, Bond, support, impulse, fixed-step, Landscape, and preview semantics.
- Established Voronoi/Power-partition, signed-distance, rigid-motion, analytic-drag, contact, and graph-connectivity formulations. These are treated as established methods; this repository documents their implementation choices and limits rather than claiming invention.
- Unreal Engine 5.7.4 engine APIs and module contracts. Engine source is not redistributed in this repository.

## Third-party code

No third-party implementation files are intentionally copied into this release. Unreal Engine modules and ProceduralMeshComponent remain external dependencies and retain their own licenses. GeometryCore is consumed through the engine module boundary and is not redistributed here.

## File ownership

Files under `Source/`, `Shaders/`, and `Tests/` are released under `LICENSE-PORTFOLIO.txt` unless a file states otherwise. The license does not grant production reuse or redistribution rights.
