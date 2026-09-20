# TAVisualFracture

TAVisualFracture is an Unreal Engine plugin for editor-authored visual fracture previews. It converts a closed static mesh into reusable chunk and bond data, exposes a deterministic impact workflow, and simulates detached chunks with a CPU reference motion system. The release contains source code, shaders, tests, and design documentation; project assets and generated build products are intentionally excluded.

TAVisualFracture 是一个面向 Unreal Engine 编辑器预览的视觉破碎插件。它把闭合 Static Mesh 烘焙为可复用的碎块与 Bond 数据，提供确定性的冲击操作流程，并使用 CPU 参考运动系统驱动脱离碎块。本发布只包含源码、Shader、测试和设计文档，不包含项目资产或生成的构建产物。

## Scope

- LOD0 closed Static Mesh authoring and deterministic chunk/bond baking.
- Editor-only bake, Landscape capture, impact presets, preview, reset, and cleanup.
- CPU fixed-step chunk motion with gravity, drag, angular motion, support connectivity, Bond damage, and sleep semantics.
- Lightweight landed XY avoidance and optional registered static-Box response for visual preview.
- Procedural Mesh rendering for temporary preview chunks; collision is disabled on preview render components.
- Runtime-facing data structures remain separated from the editor bake and preview services.

This release does not provide Chaos integration, GPU simulation, network replication, PIE/gameplay hit detection, dynamic rigid bodies, scene-wide collision, Nanite fracture rendering, or production material fidelity.

## Architecture

```text
StaticMeshActor + UTAVisualFractureComponent
        |
        v
TAVisualFractureEditor
  bake / Landscape capture / preview lifecycle / temporary PMC
        |
        v
TAVisualFractureRuntime assets and reflected settings
        |
        v
TAVisualFractureCore
  normalized geometry / SDF / seeds / Power partition / bonds / motion
        |
        +--> TVFChunkSimulate.usf and shared motion data contracts
```

The core module is UObject-independent. The Editor module owns UObject access, MeshDescription capture, Landscape sampling, asset writes, and preview lifetime. The Runtime module owns reflected component and asset types. ProceduralMeshComponent is used only for temporary visual output.

## Editor workflow

1. Add `UTAVisualFractureComponent` to a `StaticMeshActor`.
2. Bake the target mesh into a `UTAVisualFractureAsset`.
3. Capture the intended Landscape height snapshot.
4. Choose an impact preset and press Fracture in the editor preview.
5. Use Reset to restore the source mesh and destroy temporary preview components.

The lightweight collision path is explicitly landed-only. It uses circular XY chunk footprints and registered Box rectangles for visual separation. Moving registered Boxes can transfer horizontal velocity and apply a small one-shot lift when a new contact closes quickly. This is an intentional approximation, not a replacement for a rigid-body solver.

## Repository layout

- `Source/TAVisualFractureCore`: pure fracture, geometry, bond, collision, and motion contracts.
- `Source/TAVisualFractureRuntime`: reflected components, assets, and runtime-facing data.
- `Source/TAVisualFractureEditor`: bake, Landscape, details panel, preview, and editor collision adapters.
- `Shaders/Private`: chunk motion and interior-detail shader sources.
- `Tests`: standalone C++ regression runner, editor test fixtures, and verification notes.
- `Docs`: bilingual architecture, data flow, API, dependency, attribution, boundary, and verification notes.

## Dependencies

The plugin targets Unreal Engine 5.7.4 and the engine-provided ProceduralMeshComponent, GeometryCore, Landscape, MeshDescription, StaticMeshDescription, PropertyEditor, LevelEditor, AssetTools, and UnrealEd modules. See [DEPENDENCIES.md](DEPENDENCIES.md) for the exact module boundary.

## Verification status

The standalone collision runner has been executed for the current source snapshot and reports `PASS 4724 checks`. This is evidence for the independent C++ core only. UBT/UHT, ShaderCompileWorker, Editor/PIE, GPU capture, real asset preview, and in-editor performance validation are marked pending in [Docs/VerificationStatus_CN_EN.md](Docs/VerificationStatus_CN_EN.md).

## Source and attribution boundary

This repository is source-available under the accompanying review license, not an open-source license. The implementation is an independent Unreal Engine integration of established physical methods. It does not claim ownership of the underlying mathematical methods; the architecture, data contracts, editor workflow, collision adapter, diagnostics, and validation pipeline are documented as implementation work. See [ATTRIBUTION.md](ATTRIBUTION.md) and [LICENSE-PORTFOLIO.txt](LICENSE-PORTFOLIO.txt).

## Current limitations

- Input geometry is expected to be LOD0, finite, closed, consistently oriented, and suitable for the selected bake mode.
- The visual collision path does not support airborne chunk-to-chunk or external collision.
- Box collision is a landed XY rectangle approximation without top support or stacking guarantees.
- High-speed thin-wall interaction, large stacks, extreme mass ratios, and precise concave collision are outside this release boundary.
- No generated assets, maps, materials, screenshots, videos, binaries, or engine source are included.
