# TAVisualFracture

CPU 固定步长视觉破碎预览

> 从闭合网格烘焙到碎块运动，为 Unreal 编辑器提供可控的视觉破碎工作流。

[GitHub 仓库](https://github.com/walh520/TAVisualFracture-Portfolio)

## 项目简介

面向 Unreal 编辑器创作和预览的视觉破碎插件。闭合 Static Mesh 被烘焙为可复用的碎块与 Bond 数据，脱离碎块由 CPU 参考运动系统驱动，再通过临时 Procedural Mesh 展示。

仓库包含源码、Shader、独立核心测试和设计文档；场景资产和构建产物由使用工程提供。

## 演示

[破碎与溶解作品演示](https://www.bilibili.com/video/BV1Tteb6jEEr/) · [作品总集](https://www.bilibili.com/video/BV1MVak6jEPv/)

视频展示完整作品中的破碎与溶解效果。本仓库对应其中的视觉破碎编辑器预览部分。

## 实现与贡献

项目工作包括插件架构、数据契约、碎块烘焙、编辑器操作流程、临时渲染、碰撞适配和诊断工具。算法基于 Voronoi/Power partition、SDF、刚体运动、解析阻尼、接触与图连通性等已有方法，来源说明见 [ATTRIBUTION.md](ATTRIBUTION.md)。

## 核心功能

- LOD0 闭合网格的确定性碎块与 Bond 烘焙。
- Editor 中 Bake、Landscape 捕获、冲击预设、Fracture、Reset 与清理。
- CPU 固定步长推进重力、阻尼、角运动、支撑连通性、Bond 损伤与休眠。
- 落地后的 XY 碎块避让与可选已注册 Box 响应。
- 临时 Procedural Mesh 展示与源网格恢复。

## 方案与取舍

| 选择 | 作用 |
| --- | --- |
| UObject 无关 Core + Runtime 数据 + Editor 服务 | 分离纯数学和引擎资产访问，支持独立核心回归。 |
| CPU 固定步长参考运动 | 明确时间推进和状态更新顺序，便于重置与复现。 |
| 落地 XY 圆形/矩形近似接触 | 以轻量接触模型控制碎块的视觉分离。 |
| 临时 Procedural Mesh 预览 | 便于编辑器操作、恢复原网格和清理临时资源。 |

执行链为 Editor PreviewManager → CPU MotionSystem → 固定步长累加器/StepFixed → 更新预览网格。预览组件关闭引擎碰撞，接触由核心运动系统处理。

## 代码阅读入口

1. [架构](Docs/Architecture_CN_EN.md) 与 [数据流](Docs/DataFlow_CN_EN.md)。
2. [TVFFracture.cpp](Source/TAVisualFractureCore/Private/TVFFracture.cpp)、[TVFMotion.cpp](Source/TAVisualFractureCore/Private/TVFMotion.cpp)、[TVFCollision.cpp](Source/TAVisualFractureCore/Private/TVFCollision.cpp)：烘焙、时间推进和碰撞。
3. [Runtime Component](Source/TAVisualFractureRuntime/Public/TAVisualFractureComponent.h) 与 [PreviewManager](Source/TAVisualFractureEditor/Private/TAVisualFracturePreviewManager.cpp)：数据与编辑器预览。
4. [CollisionStandalone.cpp](Tests/CollisionStandalone.cpp)、[验证状态](Docs/VerificationStatus_CN_EN.md)、[功能范围](Docs/ReleaseBoundary_CN_EN.md)。

## 验证状态

仓库记录了独立 C++ 核心 runner 的 `PASS 4724 checks`。该结果对应核心层测试；UE 构建、Shader 编译、Editor/PIE、GPU Capture、真实资产预览与性能测试尚待完成。

## 依赖与运行方式

目标版本为 UE 5.7.4，依赖 ProceduralMeshComponent、GeometryCore、Landscape、MeshDescription、StaticMeshDescription、PropertyEditor、LevelEditor、AssetTools、UnrealEd 等模块，完整划分见 [DEPENDENCIES.md](DEPENDENCIES.md)。

编辑器工作流：向 StaticMeshActor 添加 UTAVisualFractureComponent → Bake → 捕获 Landscape 高度快照 → 选择冲击预设并 Fracture → Reset 恢复源网格、清理临时组件。集成后需在目标 UE 工程中完成构建与预览测试。

独立核心 runner 使用 MSVC C++20：

```powershell
.\Tests\Run-CollisionStandalone.ps1
```

## 适用范围

输入采用有限、闭合、朝向一致且适合所选 Bake 模式的 LOD0 网格。当前功能聚焦 CPU 编辑器预览，未接入 Chaos、GPU 刚体模拟、网络复制或 PIE/gameplay 命中检测。

接触模型覆盖落地后的 XY 避让及已注册 Box 的平面近似。空中碎块碰撞、Box 顶部支撑、稳定堆叠、连续碰撞与精确凹碰撞暂未支持。

## 来源与许可

采用 [LICENSE-PORTFOLIO.txt](LICENSE-PORTFOLIO.txt) 的源码审阅许可，生产复用或再分发需另行授权。算法与实现归属见 [ATTRIBUTION.md](ATTRIBUTION.md)。
