#pragma once

#include "CoreMinimal.h"
#include "UObject/SoftObjectPath.h"
#include "TAVisualFractureTypes.generated.h"

UENUM()
enum class ETVFSupportMode : uint8
{
    LandscapeAnchored,
    FreeObject
};

UENUM()
enum class ETVFImpactLocationPreset : uint8
{
    Top,
    SideMiddle,
    CenterInterior,
    Custom
};

UENUM()
enum class ETVFSideAxis : uint8
{
    PositiveX,
    NegativeX,
    PositiveY,
    NegativeY
};

UENUM()
enum class ETVFBakeInputMode : uint8
{
    StrictClosed = 0 UMETA(DisplayName="Closed Volume (Strict)"),
    VisualSolidify = 1 UMETA(DisplayName="Visual Union Repair"),
    OpenShellThickness = 2 UMETA(DisplayName="Open Shell Thickness")
};

UENUM()
enum class ETVFDiagnosticSeverity : uint8
{
    FatalGeometry,
    Recovered,
    Quality
};

UENUM()
enum class ETVFDiagnosticStage : uint8
{
    Input,
    SolidDomain,
    Seed,
    PowerPartition,
    CanonicalTopology,
    Validation,
    Performance
};

USTRUCT()
struct TAVISUALFRACTURERUNTIME_API FTVFDiagnostic
{
    GENERATED_BODY()

    UPROPERTY(VisibleAnywhere, Category="Diagnostic")
    ETVFDiagnosticSeverity Severity = ETVFDiagnosticSeverity::Quality;

    UPROPERTY(VisibleAnywhere, Category="Diagnostic")
    ETVFDiagnosticStage Stage = ETVFDiagnosticStage::Input;

    UPROPERTY(VisibleAnywhere, Category="Diagnostic")
    FName Code = NAME_None;

    UPROPERTY(VisibleAnywhere, Category="Diagnostic")
    FString Message;

    UPROPERTY(VisibleAnywhere, Category="Diagnostic")
    int64 SeedId = INDEX_NONE;

    UPROPERTY(VisibleAnywhere, Category="Diagnostic")
    int64 TetId = INDEX_NONE;

    UPROPERTY(VisibleAnywhere, Category="Diagnostic")
    int64 TriangleId = INDEX_NONE;

    UPROPERTY(VisibleAnywhere, Category="Diagnostic")
    int64 PatchId = INDEX_NONE;

    UPROPERTY(VisibleAnywhere, Category="Diagnostic")
    double MeasuredValue = 0.0;

    UPROPERTY(VisibleAnywhere, Category="Diagnostic")
    double Limit = 0.0;

    UPROPERTY(VisibleAnywhere, Category="Diagnostic")
    int64 BeforeCount = 0;

    UPROPERTY(VisibleAnywhere, Category="Diagnostic")
    int64 AfterCount = 0;
};

USTRUCT()
struct TAVISUALFRACTURERUNTIME_API FTVFCoordinateContract
{
    GENERATED_BODY()

    UPROPERTY(VisibleAnywhere, Category="Contract")
    int32 Version = 2;

    UPROPERTY(VisibleAnywhere, Category="Contract")
    FVector OriginCm = FVector::ZeroVector;

    UPROPERTY(VisibleAnywhere, Category="Contract")
    double ScaleCm = 1.0;

    UPROPERTY(VisibleAnywhere, Category="Contract")
    FVector MetricAxis = FVector::UpVector;

    UPROPERTY(VisibleAnywhere, Category="Contract")
    double MetricAxisScale = 1.0;
};

USTRUCT()
struct TAVISUALFRACTURERUNTIME_API FTVFBakePerformanceStats
{
    GENERATED_BODY()

    UPROPERTY(VisibleAnywhere, Category="Statistics")
    int64 ActiveTets = 0;
    UPROPERTY(VisibleAnywhere, Category="Statistics")
    int64 CandidateSitesP50 = 0;
    UPROPERTY(VisibleAnywhere, Category="Statistics")
    int64 CandidateSitesP95 = 0;
    UPROPERTY(VisibleAnywhere, Category="Statistics")
    int64 CandidateSitesMax = 0;
    UPROPERTY(VisibleAnywhere, Category="Statistics")
    int64 ActualCellTetPairs = 0;
    UPROPERTY(VisibleAnywhere, Category="Statistics")
    int64 PlaneTests = 0;
    UPROPERTY(VisibleAnywhere, Category="Statistics")
    int64 CanonicalVertices = 0;
    UPROPERTY(VisibleAnywhere, Category="Statistics")
    int64 SharedFacets = 0;
    UPROPERTY(VisibleAnywhere, Category="Statistics")
    int64 InterfacePatches = 0;
    UPROPERTY(VisibleAnywhere, Category="Statistics")
    int64 PeakTemporaryBytes = 0;
    UPROPERTY(VisibleAnywhere, Category="Statistics")
    int64 OutputTriangles = 0;
    UPROPERTY(VisibleAnywhere, Category="Statistics")
    int32 TargetSeeds = 0;
    UPROPERTY(VisibleAnywhere, Category="Statistics")
    int32 ValidSeeds = 0;
    UPROPERTY(VisibleAnywhere, Category="Statistics")
    int32 FinalChunks = 0;
    UPROPERTY(VisibleAnywhere, Category="Statistics")
    int32 VisualDebris = 0;
};

USTRUCT()
struct TAVISUALFRACTURERUNTIME_API FTVFPhysicsPerformanceBudget
{
    GENERATED_BODY()

    UPROPERTY(EditAnywhere, Category="Physics Budget", meta=(ClampMin="1", ToolTip="编辑器预览可创建的最大运动状态数；不会使烘焙资产失败。"))
    int32 MaxActiveChunkStates = 1024;

    UPROPERTY(EditAnywhere, Category="Physics Budget", meta=(ClampMin="1", ToolTip="每个固定子步允许的 Landscape 接触查询数；不会限制烘焙拓扑。"))
    int32 MaxGroundQueriesPerStep = 4096;
};

USTRUCT()
struct TAVISUALFRACTURERUNTIME_API FTVFRenderPerformanceBudget
{
    GENERATED_BODY()

    UPROPERTY(EditAnywhere, Category="Render Budget", meta=(ClampMin="1", ToolTip="低数量 PMC 预览可创建的最大组件数；超过时只阻止预览，不阻止烘焙。"))
    int32 MaxProceduralMeshComponents = 512;

    UPROPERTY(EditAnywhere, Category="Render Budget", meta=(ClampMin="1000", ToolTip="低数量 PMC 预览允许的总三角形数；超过时只阻止预览。"))
    int64 MaxPreviewTriangles = 1000000;
};

USTRUCT()
struct TAVISUALFRACTURERUNTIME_API FTVFBakeSettings
{
    GENERATED_BODY()

    UPROPERTY(EditAnywhere, Category="Geometry", meta=(ToolTip="Closed Volume 严格验证闭合实体；Visual Union Repair 重建视觉并集；Open Shell Thickness 将开放表面解释为有厚度双侧壳。"))
    ETVFBakeInputMode InputMode = ETVFBakeInputMode::StrictClosed;

    UPROPERTY(EditAnywhere, Category="Geometry", meta=(ClampMin="2", UIMax="512", ToolTip="目标碎块数量。不再按固定数量拒绝，实际可用规模由下方性能预算决定。"))
    int32 TargetChunkCount = 24;

    UPROPERTY(EditAnywhere, Category="Geometry", meta=(ClampMin="8", ClampMax="256", ToolTip="SDF 网格最长轴分辨率。更高会让断面更细，但烘焙更慢并消耗更多内存。"))
    int32 LongestAxisCells = 24;

    UPROPERTY(EditAnywhere, Category="Visual Solidify", meta=(ClampMin="16", ClampMax="256", EditCondition="InputMode == ETVFBakeInputMode::VisualSolidify", ToolTip="Visual Solidify 最长轴体素分辨率。数值越高越能保留轮廓、孔洞和薄结构，但耗时与内存显著增加。"))
    int32 SolidifyVoxelResolution = 96;

    UPROPERTY(EditAnywhere, Category="Visual Solidify", meta=(ClampMin="0.01", ClampMax="0.99", EditCondition="InputMode == ETVFBakeInputMode::VisualSolidify", ToolTip="快速绕数的内外判定阈值。默认 0.5；降低可封住更大的缺口，但也更容易合并邻近表面。"))
    double SolidifyWindingThreshold = 0.5;

    UPROPERTY(EditAnywhere, Category="Open Shell", meta=(ClampMin="0.001", EditCondition="InputMode == ETVFBakeInputMode::OpenShellThickness", ToolTip="开放表面的物理总厚度，单位 cm；实体域为 unsignedDistance <= thickness/2。"))
    double OpenShellThicknessCm = 5.0;

    UPROPERTY(EditAnywhere, Category="Geometry", meta=(ToolTip="控制烘焙随机分块的种子；相同输入和种子会得到相同拓扑。"))
    int32 RandomSeed = 1337;

    UPROPERTY(EditAnywhere, Category="Geometry", meta=(ClampMin="0.0", ClampMax="1.0", ToolTip="种子选点的随机比例：0 使用候选中的密度加权最远点，1 使用随机点。控制分布而非断面锯齿；修改后须重新烘焙。"))
    double Irregularity = 0.60;

    UPROPERTY(EditAnywhere, Category="Geometry", meta=(ClampMin="0.0", ClampMax="1.0", ToolTip="Power 权重扰动幅度：0 不扰动，1 使用文档允许的最大扰动。不改变种子位置，0 不保证所有碎块等大；修改后须重新烘焙。"))
    double SizeVariation = 0.75;

    UPROPERTY(EditAnywhere, Category="Geometry", meta=(ToolTip="控制分块度量的本地方向。"))
    FVector MetricAxis = FVector::UpVector;

    UPROPERTY(EditAnywhere, Category="Geometry", meta=(ClampMin="0.125", ClampMax="8.0", ToolTip="Metric Axis 的尺度权重，用于拉伸或压缩该方向上的分块分布。"))
    double MetricAxisScale = 1.0;

    // This is a bake-time seed-density focus, not a runtime impact location.
    UPROPERTY(EditAnywhere, Category="Chunk Density Focus", meta=(ToolTip="烘焙时提高碎块密度的本地焦点，不是运行时冲击位置。"))
    FVector DensityFocusPointLocal = FVector::ZeroVector;

    UPROPERTY(EditAnywhere, Category="Chunk Density Focus", meta=(ClampMin="0.0", ToolTip="碎块密度焦点的影响半径，单位 cm。"))
    double DensityFocusRadiusCm = 30.0;

    UPROPERTY(EditAnywhere, Category="Chunk Density Focus", meta=(ClampMin="0.0", ClampMax="1.0", ToolTip="焦点处额外分块密度。0 关闭焦点效果。"))
    double DensityFocusConcentration = 0.0;

    UPROPERTY(EditAnywhere, Category="Performance Budget", meta=(ClampMin="100", UIMax="20000", ToolTip="每个主要烘焙阶段允许的实际/估算几何工作量，单位为百万次基础测试。默认预算已比原固定数量限制宽松；超过预算会安全取消而不是生成半成品。"))
    int64 MaxBakeWorkMillions = 5000;

    UPROPERTY(EditAnywhere, Category="Performance Budget", meta=(ClampMin="512", UIMax="16384", ToolTip="烘焙工作集预算，单位 MB。它限制候选点、临时 Cell/Piece 和网格内存，不直接限制碎块数量。"))
    int32 WorkingMemoryBudgetMB = 8192;

    UPROPERTY(EditAnywhere, Category="Performance Budget", meta=(ClampMin="10000", UIMax="50000000", ToolTip="烘焙允许生成的三角形总数。只限制 Bake 输出工作量，不按目标碎块数量拒绝。"))
    int64 MaxOutputTriangles = 5000000;

    UPROPERTY(EditAnywhere, Category="Geometry", meta=(ClampMin="0.0", ClampMax="0.1", ToolTip="最终 Chunk 小于总体积此比例时，按最大共享界面并入邻块。0 关闭美术小块合并，仅保留数值清理。"))
    double MinFinalChunkVolumeFraction = 0.0;
};

USTRUCT()
struct TAVISUALFRACTURERUNTIME_API FTVFObjectResponseSettings
{
    GENERATED_BODY()

    UPROPERTY(EditAnywhere, Category="Mass and Bonds", meta=(ClampMin="0.000000001", ToolTip="碎块质量密度，单位质量/cm³。"))
    double MassPerCm3 = 0.0001;

    UPROPERTY(EditAnywhere, Category="Mass and Bonds", meta=(ClampMin="0.000000001", ToolTip="Bond 单位面积阻力，单位 1/cm²；实际阻力为面积乘以此值。"))
    double BondResistancePerCm2 = 0.01;

    UPROPERTY(EditAnywhere, Category="Motion", meta=(ClampMin="0.0", ToolTip="线速度解析阻力，单位 1/s。"))
    double LinearDragPerSecond = 0.35;

    UPROPERTY(EditAnywhere, Category="Motion", meta=(ClampMin="0.0", ToolTip="角速度解析阻力，单位 1/s。"))
    double AngularDragPerSecond = 0.20;

    UPROPERTY(EditAnywhere, Category="Motion", meta=(ToolTip="施加给每个活动碎块的世界重力加速度，单位 cm/s²。"))
    FVector GravityWorld = FVector(0, 0, -980.0);

    UPROPERTY(EditAnywhere, Category="Contact", meta=(ClampMin="0.0", ClampMax="1.0", ToolTip="碎块与 Landscape 接触时的恢复系数。"))
    double Restitution = 0.15;

    UPROPERTY(EditAnywhere, Category="Contact", meta=(ClampMin="0.0", ClampMax="1.0", ToolTip="碎块与 Landscape 接触时的切向摩擦。"))
    double Friction = 0.4;

    UPROPERTY(EditAnywhere, Category="Contact", meta=(ClampMin="0.0", ToolTip="接触后附加的角速度阻尼，单位 1/s。"))
    double ContactAngularDampingPerSecond = 4.0;

    UPROPERTY(EditAnywhere, Category="Contact", meta=(ClampMin="0.0", ToolTip="低于此法向速度时不再产生反弹，单位 cm/s。"))
    double RestitutionVelocityThresholdCmPerSecond = 20.0;

    UPROPERTY(EditAnywhere, Category="Sleep", meta=(ClampMin="0.0", ToolTip="进入休眠所需的最大线速度，单位 cm/s。"))
    double SleepLinearSpeedCmPerSecond = 0.5;

    UPROPERTY(EditAnywhere, Category="Sleep", meta=(ClampMin="0.0", ToolTip="进入休眠所需的最大角速度，单位 rad/s。"))
    double SleepAngularSpeedRadPerSecond = 0.02;

    UPROPERTY(EditAnywhere, Category="Sleep", meta=(ClampMin="0.0", ToolTip="速度满足休眠阈值后保持多久才休眠，单位 s。"))
    double SleepDelaySeconds = 0.35;

    UPROPERTY(EditAnywhere, Category="Motion", meta=(ClampMin="0.001", ClampMax="0.1", ToolTip="固定步进时长，单位 s；更小更稳定但更耗时。"))
    double FixedTimeStepSeconds = 1.0 / 60.0;
};

USTRUCT()
struct TAVISUALFRACTURERUNTIME_API FTVFImpactSettings
{
    GENERATED_BODY()

    UPROPERTY(VisibleAnywhere, Category="Impact", meta=(ToolTip="当前冲击位置预设。手动修改位置或方向后会变为 Custom。"))
    ETVFImpactLocationPreset Preset = ETVFImpactLocationPreset::Custom;

    UPROPERTY(EditAnywhere, Category="Impact", meta=(ToolTip="侧面中部预设使用的本地侧轴。"))
    ETVFSideAxis SideAxis = ETVFSideAxis::PositiveX;

    UPROPERTY(EditAnywhere, Category="Impact", meta=(ToolTip="冲击点的本地坐标。可手动编辑；编辑后预设变为 Custom。"))
    FVector ImpactPointLocal = FVector::ZeroVector;

    UPROPERTY(EditAnywhere, Category="Impact", meta=(ToolTip="冲击的本地方向。可手动编辑；编辑后预设变为 Custom。"))
    FVector DirectionLocal = FVector(0, 0, -1);

    UPROPERTY(EditAnywhere, Category="Impact", meta=(ClampMin="0.0", ToolTip="获得冲量和 Bond 损伤的作用半径，单位 cm。0 会在首次预览时自动初始化。"))
    double ImpactRadiusCm = 0.0;

    UPROPERTY(EditAnywhere, Category="Impact", meta=(ClampMin="0.0", ToolTip="参与本次 Bond 损伤的作用半径，单位 cm。0 会在首次预览时自动初始化。"))
    double BondRadiusCm = 0.0;

    UPROPERTY(EditAnywhere, Category="Impact", meta=(ClampMin="0.0", ToolTip="本次事件施加给碎块的冲量大小。"))
    double Impulse = 650.0;

    UPROPERTY(EditAnywhere, Category="Impact", meta=(ClampMin="0.0", ToolTip="本次事件对范围内 Bond 增加的不可恢复损伤。"))
    double BondDamage = 15.0;

    UPROPERTY(EditAnywhere, Category="Impact", meta=(ClampMin="0.0", ToolTip="冲量随距离衰减的指数；数值越大，中心越集中。"))
    double FalloffExponent = 1.5;

    UPROPERTY(EditAnywhere, Category="Impact", meta=(ClampMin="0.000001", ToolTip="冲量尺度的参考半径，单位 cm。0 会在首次预览时自动初始化。"))
    double ReferenceRadiusCm = 0.0;

    UPROPERTY(EditAnywhere, Category="Impact", meta=(ToolTip="按碎块尺寸调整初速度的偏置。"))
    double SizeSpeedBias = 0.0;

    UPROPERTY(EditAnywhere, Category="Direction Weights", meta=(ToolTip="从冲击点向外扩散的冲量权重。"))
    double RadialWeight = 0.35;

    UPROPERTY(EditAnywhere, Category="Direction Weights", meta=(ToolTip="沿冲击方向施加的冲量权重。"))
    double DirectionalWeight = 1.0;

    UPROPERTY(EditAnywhere, Category="Direction Weights", meta=(ToolTip="沿世界上方向施加的冲量权重。"))
    double UpwardWeight = 0.0;

    UPROPERTY(EditAnywhere, Category="Direction Weights", meta=(ToolTip="随机方向冲量权重；由 Random Seed 保持可复现。"))
    double RandomWeight = 0.10;

    UPROPERTY(EditAnywhere, Category="Impact", meta=(ToolTip="本次冲击随机方向的种子。"))
    int32 RandomSeed = 1337;
};

USTRUCT()
struct TAVISUALFRACTURERUNTIME_API FTVFLandscapeSnapshot
{
    GENERATED_BODY()

    UPROPERTY(VisibleAnywhere, Category="Landscape")
    FSoftObjectPath LandscapePath;

    UPROPERTY(VisibleAnywhere, Category="Landscape")
    FGuid LandscapeGuid;

    UPROPERTY(VisibleAnywhere, Category="Landscape")
    FTransform CapturedTargetTransform = FTransform::Identity;

    UPROPERTY(VisibleAnywhere, Category="Landscape")
    FVector2D OriginWorldXY = FVector2D::ZeroVector;

    UPROPERTY(VisibleAnywhere, Category="Landscape")
    double GridStepCm = 0.0;

    UPROPERTY(VisibleAnywhere, Category="Landscape")
    int32 Resolution = 0;

    UPROPERTY(VisibleAnywhere, Category="Landscape")
    double CapturedHalfExtentCm = 0.0;

    UPROPERTY()
    TArray<double> WorldHeightsCm;

    UPROPERTY()
    TArray<uint8> ValidMask;

    bool IsValid(FString* OutReason = nullptr) const;
    bool SampleHeightAndNormal(const FVector2D& WorldXY, double& OutHeightCm, FVector& OutNormal) const;
};

USTRUCT()
struct TAVISUALFRACTURERUNTIME_API FTVFChunkMeshSection
{
    GENERATED_BODY()

    UPROPERTY()
    TArray<FVector> Vertices;

    UPROPERTY()
    TArray<int32> Triangles;

    UPROPERTY()
    TArray<FVector> Normals;

    UPROPERTY()
    TArray<FVector2D> UV0;

    // XYZ is the rest-space position and W is 0 for exterior / 1 for interior.
    UPROPERTY()
    TArray<FVector4> RestSpaceData;

    UPROPERTY()
    TArray<FLinearColor> VertexColors;

    UPROPERTY()
    TArray<FVector> Tangents;
};

USTRUCT()
struct TAVISUALFRACTURERUNTIME_API FTVFChunkRenderData
{
    GENERATED_BODY()

    UPROPERTY(VisibleAnywhere, Category="Chunk")
    int32 ChunkId = INDEX_NONE;

    UPROPERTY(VisibleAnywhere, Category="Chunk")
    FVector Centroid = FVector::ZeroVector;

    UPROPERTY(VisibleAnywhere, Category="Chunk")
    double RadiusCm = 0.0;

    UPROPERTY(VisibleAnywhere, Category="Chunk")
    double VolumeCm3 = 0.0;

    UPROPERTY()
    FTVFChunkMeshSection Exterior;

    UPROPERTY()
    FTVFChunkMeshSection Interior;
};

USTRUCT()
struct TAVISUALFRACTURERUNTIME_API FTVFBondData
{
    GENERATED_BODY()

    UPROPERTY()
    int32 ChunkA = INDEX_NONE;

    UPROPERTY()
    int32 ChunkB = INDEX_NONE;

    UPROPERTY()
    double AreaCm2 = 0.0;

    UPROPERTY()
    FVector Centroid = FVector::ZeroVector;

    UPROPERTY()
    FVector NormalAToB = FVector::ForwardVector;
};

USTRUCT()
struct TAVISUALFRACTURERUNTIME_API FTVFCanonicalVertexData
{
    GENERATED_BODY()

    UPROPERTY()
    int64 VertexId = INDEX_NONE;
    UPROPERTY()
    FVector Position = FVector::ZeroVector;
    UPROPERTY()
    TArray<int64> SupportIds;
};

USTRUCT()
struct TAVISUALFRACTURERUNTIME_API FTVFInterfacePatchData
{
    GENERATED_BODY()

    UPROPERTY()
    int64 PatchId = INDEX_NONE;
    UPROPERTY()
    int32 ChunkA = INDEX_NONE;
    UPROPERTY()
    int32 ChunkB = INDEX_NONE;
    UPROPERTY()
    TArray<int64> CanonicalVertexIds;
    UPROPERTY()
    TArray<int32> Triangles;
    UPROPERTY()
    double AreaCm2 = 0.0;
};
