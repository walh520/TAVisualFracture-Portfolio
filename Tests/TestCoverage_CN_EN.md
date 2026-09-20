# Test coverage / 测试覆盖

The standalone fixture covers geometry helpers, strict collision regression, landed XY avoidance, footprint scaling, sleep/wake, external Box filtering, correction speed, moving-Box sample baselines, one-shot impact transfer and lift, removal/re-registration, friction decay, terrain resampling, visual rolling, and 30/60/120 FPS fixture trajectories.

The Editor automation fixture covers reflected settings, visual-mode preparation, collision-signature isolation, external refresh preservation, and collision proxy contracts. It is included but not executed in this source-only release.

The test suite does not claim coverage for UE reflection generation, real Static Mesh assets, Landscape editor data, render-thread lifetime, GPU compilation, PIE, or production performance.

独立测试覆盖几何辅助、严格碰撞回归、落地 XY 避让、占位缩放、休眠/唤醒、外部 Box 过滤、避让修正速度、移动 Box 基线、单次冲量与弹起、移除/重新注册、摩擦衰减、地形重采样、视觉滚动以及 30/60/120 FPS 夹具轨迹。

编辑器自动化夹具覆盖反射设置、视觉模式准备、碰撞签名隔离、外部刷新保持和碰撞代理合同，但本次源码发布未执行。UE 反射生成、真实 Static Mesh、Landscape 编辑器数据、渲染线程生命周期、GPU 编译、PIE 和生产性能不在已执行证据内。
