# TAVisualFracture

CPU 固定步长视觉破碎预览

> CPU 固定步长编辑器预览；独立核心测试有记录，不是 GPU / Chaos 完整物理系统。

[GitHub 仓库](https://github.com/walh520/TAVisualFracture-Portfolio)

## 简介与公开范围

面向 Unreal 编辑器创作和预览的视觉破碎插件。闭合 Static Mesh 被烘焙为可复用的碎块与 Bond 数据，脱离碎块由 CPU 参考运动系统驱动。本仓库包含源码、Shader、测试和设计文档，不含项目资产与生成构建产物。

## 本地工程与公开快照

与本地插件对应的 55 个公开路径中，34 个逐字节相同、14 个仅换行不同；另外 7 个是公开包增加的说明/验收文档，本地插件目录没有对应文件。已对应的代码实现没有内容差异，不能把这 7 个文档缺项描述成实现缺失。

编辑器 PreviewManager 持有 CPU `tvf::MotionSystem`，以固定步长推进碎片；预览用 ProceduralMeshComponent 且关闭组件碰撞。Shader 文件存在不代表当前预览切到 GPU，Runtime 模块存在也不代表 Chaos 或完整 PIE 物理已经接通。

## 演示

[破碎与溶解作品演示](https://www.bilibili.com/video/BV1Tteb6jEEr/) · [作品总集](https://www.bilibili.com/video/BV1MVak6jEPv/)

该视频是作品演示入口，可能包含本仓库以外的溶解效果和完整项目资产。本仓库公开范围只按视觉破碎编辑器预览说明，不把视频中的其他系统归入当前代码。

## 实现与贡献

[ATTRIBUTION.md](ATTRIBUTION.md) 将插件架构、数据契约、编辑器流程、临时渲染路径、碰撞适配、诊断与验证流程列为项目实现工作。Voronoi/Power partition、SDF、刚体运动、解析阻尼、接触与图连通性为已有方法；本页不扩展作者或原创算法归属。

## 核心功能

- LOD0 闭合网格的确定性碎块与 Bond 烘焙。
- Editor 中 Bake、Landscape 捕获、冲击预设、Fracture、Reset 与清理。
- CPU 固定步长：重力、阻尼、角运动、支撑连通性、Bond 损伤及休眠。
- 落地后的 XY 碎块避让与可选已注册 Box 响应。
- 临时 Procedural Mesh 展示；预览渲染组件禁用碰撞。

## 方案与取舍

| 选择 | 目的与边界 |
| --- | --- |
| UObject 无关 Core + Runtime 数据 + Editor 服务 | 将纯数学与引擎资产访问分开，支持独立核心回归。 |
| CPU 固定步长参考运动 | 使时间推进与状态约定可审阅；不宣称 GPU 模拟。 |
| 落地 XY 圆形/矩形近似接触 | 服务视觉分离与编辑器操作，不能替代完整刚体求解器。 |
| 临时 PMC 作为预览输出 | 便于恢复原网格和清理；不等同于 Nanite 破碎或生产材质保真。 |

实际执行链是 Editor PreviewManager → CPU MotionSystem → 固定步长累加器/StepFixed → 更新预览网格。几何/接触近似服务于可控编辑器预览；不把视觉碎裂片段当成 GPU 刚体、Chaos、任意场景连续碰撞或稳定堆叠的验证。

## 代码阅读入口

1. [架构](Docs/Architecture_CN_EN.md) 与 [数据流](Docs/DataFlow_CN_EN.md)。
2. [TVFFracture.cpp](Source/TAVisualFractureCore/Private/TVFFracture.cpp)、[TVFMotion.cpp](Source/TAVisualFractureCore/Private/TVFMotion.cpp)、[TVFCollision.cpp](Source/TAVisualFractureCore/Private/TVFCollision.cpp)：烘焙、时间推进和碰撞。
3. [Runtime Component](Source/TAVisualFractureRuntime/Public/TAVisualFractureComponent.h) 与 [PreviewManager](Source/TAVisualFractureEditor/Private/TAVisualFracturePreviewManager.cpp)：数据与编辑器预览。
4. [CollisionStandalone.cpp](Tests/CollisionStandalone.cpp)、[验证状态](Docs/VerificationStatus_CN_EN.md)、[发布边界](Docs/ReleaseBoundary_CN_EN.md)。

## 验证与性能

原仓库记录独立 C++ 核心 runner 的结果为 `PASS 4724 checks`。这是发布快照的原记录，本次整理未重跑；只支持独立核心层的结论。

UBT/UHT、ShaderCompileWorker、Editor/PIE、GPU Capture、真实资产预览和编辑器性能仍待验证，不能从该计数推导通过。高碎块数量、真实拖动响应与帧耗时没有在本次整理中测量。

## 依赖与运行方式

目标 UE 5.7.4，依赖引擎提供的 ProceduralMeshComponent、GeometryCore、Landscape、MeshDescription、StaticMeshDescription、PropertyEditor、LevelEditor、AssetTools、UnrealEd 等模块，完整划分见 [DEPENDENCIES.md](DEPENDENCIES.md)。

在兼容工程中集成并构建后，按原工作流：向 StaticMeshActor 添加 UTAVisualFractureComponent → Bake → 捕获 Landscape 高度快照 → 选择冲击预设并 Fracture → Reset 恢复源网格、清理临时组件。UE 路径待实际验证。

独立核心 runner 需要 MSVC C++ 工具；原脚本使用 C++20：

```powershell
.\Tests\Run-CollisionStandalone.ps1
```

## 限制与来源许可

不提供 Chaos 集成、GPU 模拟、网络复制、PIE/gameplay 命中检测、动态刚体或全场景碰撞。Shader 文件的存在不改变当前 CPU 预览定位。

输入须为有限、闭合、朝向一致且适合所选 Bake 模式的 LOD0 网格；不支持空中碎块相互/外部碰撞。Box 是落地 XY 矩形近似，不保证顶部支撑或堆叠；高速薄墙、极端质量比、精确凹碰撞与大型堆叠不在本轮边界。

使用 [LICENSE-PORTFOLIO.txt](LICENSE-PORTFOLIO.txt) 的源码审阅许可；原说明不授予生产复用或再分发权利。保留 [ATTRIBUTION.md](ATTRIBUTION.md)，不称为开源许可。
