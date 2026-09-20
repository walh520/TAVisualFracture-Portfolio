# Data flow / 数据流

```text
StaticMeshDescription
  -> normalized solid-domain capture
  -> SDF / valid cells / seeds
  -> Power partition and canonical interfaces
  -> chunks + bonds + diagnostics
  -> UTAVisualFractureAsset
  -> temporary PMC sections and MotionSystem states
  -> Landscape contact + landed XY avoidance
```

Editor actions are explicit: Bake writes the asset, Capture Landscape writes the local snapshot, Fracture submits one impact event, and Reset destroys session state. The Tick path does not submit repeated impact events.

编辑器操作是显式的：Bake 写入资产，Capture Landscape 写入局部高度快照，Fracture 提交一次冲击事件，Reset 销毁会话状态。Tick 不会重复提交冲击事件。

Moving external Boxes are sampled separately from the fracture asset. A stable registration/geometry key matches samples; the first sample establishes a baseline. A later sample estimates boundary velocity and may apply one horizontal response and one lift per Box/Chunk contact start.
