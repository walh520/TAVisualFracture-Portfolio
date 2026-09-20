# Shader review / Shader 审查

The shader layer is intentionally small in this release. `TVFChunkSimulate.usf` consumes the chunk motion contract, while `TVFChunkMotion.ush` and `TVFChunkIntegrate.ush` define shared layout and integration helpers. `TVFInteriorDetail.usf` and `.ush` provide the interior-detail rendering path.

The CPU Editor preview remains the authoritative validation path for this release. The GPU files are included for interface review, but no ShaderCompileWorker result is claimed here. Parameter lane layout, buffer ownership, pass order, units, and pending evidence are recorded in the source comments and [VerificationStatus_CN_EN.md](VerificationStatus_CN_EN.md).

本版本 Shader 数量保持较小：`TVFChunkSimulate.usf` 使用碎块运动合同，`TVFChunkMotion.ush` 与 `TVFChunkIntegrate.ush` 定义共享布局和积分辅助函数，`TVFInteriorDetail.usf/.ush` 提供断面细节路径。

当前以 CPU 编辑器预览作为权威验证路径。Shader 源码用于接口审查，但本发布不宣称已完成 ShaderCompileWorker 编译；参数 lane、Buffer 所有权、Pass 顺序、单位和待验证证据见源码注释及验证状态文档。
