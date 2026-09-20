# Architecture / 架构

## Module ownership

`TAVisualFractureCore` contains the deterministic mathematical and motion contracts without UObject access. `TAVisualFractureRuntime` contains reflected assets, components, and serialized settings. `TAVisualFractureEditor` is the only module that reads editor assets, MeshDescription, Landscape data, Details properties, and temporary Procedural Mesh components.

`TAVisualFractureCore` 不依赖 UObject，承载确定性的几何、切割、Bond、碰撞和运动合同。`TAVisualFractureRuntime` 承载反射资产、组件和序列化设置。`TAVisualFractureEditor` 是唯一读取编辑器资产、MeshDescription、Landscape、Details 属性并创建临时 Procedural Mesh 组件的模块。

## Data ownership

- Bake input is captured from LOD0 and copied into normalized double-precision working coordinates.
- Bake output is serialized through Unreal-friendly arrays and reflected structs; STL containers remain private to the core.
- The preview manager owns temporary render components and restores the source mesh on reset, undo, map change, save, PIE entry, and module shutdown.
- Collision proxies and external Box samples are session caches. They do not mutate the baked chunk topology or Bond graph.

## Failure policy

Fatal geometry and topology diagnostics block asset replacement. Quality diagnostics remain visible with measured values. Preview collision preparation may fail independently without invalidating a valid Bake asset.
