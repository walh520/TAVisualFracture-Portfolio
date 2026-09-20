#pragma once

#include "TVFFracture.h"
#include <cstddef>
#include <cstdint>
#include <functional>
#include <unordered_set>
#include <type_traits>
#include <vector>

namespace tvf {
class CollisionScene;

// Quaternions are (x,y,z,w), map an object/rest vector to world with q*v*q^-1.
struct Quat {
    double x = 0, y = 0, z = 0, w = 1;
    constexpr Quat() = default;
    constexpr Quat(double X, double Y, double Z, double W) : x(X), y(Y), z(Z), w(W) {}
};

TAVISUALFRACTURECORE_API Quat Normalize(Quat q);
TAVISUALFRACTURECORE_API Quat Conjugate(Quat q);
TAVISUALFRACTURECORE_API Quat Multiply(Quat a, Quat b);
TAVISUALFRACTURECORE_API Quat QuatFromRotationVector(Vec3 radians);
TAVISUALFRACTURECORE_API Vec3 Rotate(Quat q, Vec3 v);

struct ChunkMotionDesc {
    uint32_t id = 0;
    Vec3 restCentroid;                 // Object rest-space, cm.
    double radius = 1;                 // Conservatively contains this chunk, cm.
    double mass = 1;                   // Arbitrary but consistent mass unit; must be > 0.
    bool supported = false;            // A chunk connected to a support stays kinematic.
};

struct BondMotionDesc {
    uint32_t chunkA = 0;               // Indices into Initialize's chunk array.
    uint32_t chunkB = 0;
    Vec3 centroid;                     // Rest-space, cm.
    double resistance = 1;             // Damage needed to break; must be > 0.
    double initialDamage = 0;          // [0, resistance].
};

struct MotionSettings {
    double fixedTimeStep = 1.0 / 60.0; // seconds; Step accumulates variable frame time.
    double linearDrag = 0.35;          // s^-1, drag relative to the supplied world wind.
    double angularDrag = 0.20;         // s^-1.
    double sleepLinearSpeed = 0.5;     // cm/s.
    double sleepAngularSpeed = 0.02;   // radians/s.
    double sleepDelay = 0.35;          // seconds below both thresholds after contact.
    double wakeImpulse = 0.01;         // mass*cm/s, comparison is per-chunk impulse magnitude.
    Vec3 initialWorldOffset;           // Put rest centroids in this world position basis.
    Quat initialWorldRotation;
};

struct PlaneCollider {
    Vec3 normal = {0, 0, 1};           // Must be nonzero; normal and offset are normalized together.
    double offset = 0;                 // Plane is dot(normal, x) = offset (cm).
    double restitution = 0.15;         // Clamped to [0, 1].
    double friction = 0.4;             // Clamped to [0, 1], tangential speed multiplier is 1-friction.
    double angularDamping = 4.0;       // s^-1 while this bounding sphere contacts the plane.
    double restitutionVelocityThreshold = 20.0; // cm/s; below this inward speed restitution is forced to zero.
};

// Optional data indexed by chunk array index. Wind is added to MotionStepInput::windWorld.
struct ChunkFieldInput {
    Vec3 windWorld;
    Vec3 accelerationWorld;
    double dragScale = 1.0;            // Multiplies MotionSettings::linearDrag, clamped >= 0.
};

struct MotionStepInput {
    Vec3 gravityWorld = {0, 0, -980.0}; // cm/s^2.
    Vec3 windWorld;
    std::vector<PlaneCollider> planes;
    std::vector<ChunkFieldInput> fields; // Empty: zero field for every chunk.
};

// eventId != 0 is deduplicated for this MotionSystem. Use a distinct id for a deliberate second hit.
struct ImpactEvent {
    uint64_t eventId = 0;
    uint32_t randomSeed = 1337;
    Vec3 pointWorld;
    Vec3 directionWorld = {1, 0, 0};
    Vec3 upWorld = {0, 0, 1};
    double radius = 30;                // cm; a nonpositive radius only affects a coincident center.
    double impulse = 100;              // Total scale in mass*cm/s before distance and size factors.
    double radialWeight = 1;
    double directionalWeight = 0;
    double upwardWeight = 0;
    double randomWeight = 0.15;
    double falloffExponent = 1.5;
    double referenceRadius = 20;       // cm for sizeSpeedBias.
    double sizeSpeedBias = 0;          // Impulse factor pow(referenceRadius / radius, bias).
    double bondDamage = 0;             // Dimensionless damage scale; zero leaves bonds intact.
    double bondRadius = 0;             // cm; <=0 reuses radius.
};

struct ChunkMotionState {
    Vec3 positionWorld;
    Quat rotationWorld;
    Vec3 previousPositionWorld;
    Quat previousRotationWorld;
    Vec3 velocityWorld;
    Vec3 angularVelocityWorld;         // World-space radians/s.
    double sleepTime = 0;
    bool sleeping = true;
    bool detached = false;
};

// Optional CPU contact source queried once per moving chunk after analytic integration in every
// fixed substep. Return false when the chunk center is outside the captured ground domain.
// The returned plane is resolved by the same sphere contact, restitution, friction, angular
// damping and sleep path used by MotionStepInput::planes.
using GroundPlaneQuery = std::function<bool(uint32_t chunkIndex, const Vec3& centerWorld,
                                            PlaneCollider& outPlane)>;

struct BondMotionState {
    double damage = 0;
    bool broken = false;
};

// StructuredBuffer-compatible data. All fields are 16-byte records; CPU doubles are explicitly narrowed.
struct alignas(16) GpuChunkMotion {
    float currentPositionRadius[4];
    float currentRotation[4];
    float previousPositionRadius[4];
    float previousRotation[4];
    float restCentroidInvMass[4];
    float velocityMass[4];
    float angularVelocitySleeping[4];
    uint32_t chunkIdFlags[4];          // x=id, y bit0 sleeping / bit1 detached.
};
static_assert(alignof(GpuChunkMotion) == 16, "GpuChunkMotion must have HLSL 16-byte alignment");
static_assert(sizeof(GpuChunkMotion) == 128, "Update TVFChunkMotion.ush when this layout changes");
static_assert(offsetof(GpuChunkMotion, currentRotation) == 16, "HLSL layout mismatch");
static_assert(offsetof(GpuChunkMotion, previousPositionRadius) == 32, "HLSL layout mismatch");
static_assert(offsetof(GpuChunkMotion, restCentroidInvMass) == 64, "HLSL layout mismatch");
static_assert(offsetof(GpuChunkMotion, chunkIdFlags) == 112, "HLSL layout mismatch");
static_assert(std::is_standard_layout<GpuChunkMotion>::value, "GPU records must be standard layout");

// GPU-authoritative compute input records. These exactly match TVFChunkIntegrate.ush.
struct alignas(16) GpuChunkField {
    float windDragScale[4];
    float accelerationUnused[4];
};
struct alignas(16) GpuChunkPatch {
    float velocityDeltaUnused[4];
    float angularVelocityDeltaUnused[4];
    uint32_t flags[4]; // x: TVF_PATCH_* bitset; host clears/replaces after one dispatch.
};
struct alignas(16) GpuPlaneCollider {
    float normalOffset[4];
    float response[4];
};
// cbuffer record sequence: all entries are explicit float4/uint4, avoiding float3 packing ambiguity.
struct alignas(16) GpuChunkSimulateParams {
    float timeDragSleep[4]; // fixedStep, linearDrag, angularDrag, sleepLinearSpeed
    float sleepParams[4];   // sleepAngularSpeed, sleepDelaySeconds, reserved, reserved
    float gravityWorld[4];
    float globalWindWorld[4];
    uint32_t counts[4];     // planeCount, numChunks, substepCount, flags(bit0 fields, bit1 patches)
};
static_assert(sizeof(GpuChunkField) == 32 && alignof(GpuChunkField) == 16, "TVFChunkFieldGPU ABI mismatch");
static_assert(sizeof(GpuChunkPatch) == 48 && alignof(GpuChunkPatch) == 16, "TVFChunkPatchGPU ABI mismatch");
static_assert(sizeof(GpuPlaneCollider) == 32 && alignof(GpuPlaneCollider) == 16, "TVFPlaneColliderGPU ABI mismatch");
static_assert(sizeof(GpuChunkSimulateParams) == 80 && alignof(GpuChunkSimulateParams) == 16, "TVFChunkSimulate cbuffer ABI mismatch");
static_assert(offsetof(GpuChunkSimulateParams, counts) == 64, "TVFChunkSimulate cbuffer offset mismatch");

class TAVISUALFRACTURECORE_API MotionSystem {
public:
    // Starts chunks at their rest centroids under initialWorldRotation/Offset. Existing state is discarded.
    bool Initialize(const MotionSettings& settings, const std::vector<ChunkMotionDesc>& chunks,
                    const std::vector<BondMotionDesc>& bonds);

    // Applies an instantaneous one-shot impulse and optional bond damage. False means duplicate eventId or non-finite event.
    bool ApplyImpact(const ImpactEvent& impact);

    // Snapshots previous poses at this render-step entry, then adds frameDelta to the fixed accumulator.
    // The snapshot occurs even when no fixed substep executes. Returns the executed substep count.
    uint32_t Step(double frameDeltaSeconds, const MotionStepInput& input);

    // Compatibility extension for a per-chunk, per-substep ground plane such as a sampled Landscape.
    // Existing global planes remain active. An empty query is equivalent to the original overload.
    uint32_t Step(double frameDeltaSeconds, const MotionStepInput& input, const GroundPlaneQuery& groundQuery);

    // Object-rest point to current world point. Invalid index returns restPoint unchanged.
    Vec3 TransformPoint(uint32_t chunkIndex, Vec3 restPoint) const;
    Vec3 TransformPreviousPoint(uint32_t chunkIndex, Vec3 restPoint) const;

    // Raises CPU-only bond damage toward [0,resistance]; bonds do not heal. Recomputes support connectivity.
    bool SetBondDamage(uint32_t bondIndex, double damage);
    // Recomputes the CPU-only intact-bond support graph after a sequence of API-driven edits.
    void UpdateSupportConnectivity();
    void PackGpu(std::vector<GpuChunkMotion>& out) const;

    const std::vector<ChunkMotionState>& States() const { return states_; }
    const std::vector<BondMotionState>& BondStates() const { return bondStates_; }
    const std::vector<ChunkMotionDesc>& Chunks() const { return chunks_; }
    double AccumulatorSeconds() const { return accumulator_; }
    // Non-owning, CPU-only. Caller keeps scene alive through preview/reset.
    void SetCollisionScene(CollisionScene* scene) { collisionScene_ = scene; }

private:
    void StepFixed(const MotionStepInput& input, const GroundPlaneQuery* groundQuery);
    void Wake(uint32_t chunkIndex);
    bool HasSupport() const;

    MotionSettings settings_;
    std::vector<ChunkMotionDesc> chunks_;
    std::vector<BondMotionDesc> bonds_;
    std::vector<ChunkMotionState> states_;
    std::vector<BondMotionState> bondStates_;
    std::unordered_set<uint64_t> appliedEvents_;
    double accumulator_ = 0;
    CollisionScene* collisionScene_ = nullptr;
};

} // namespace tvf
