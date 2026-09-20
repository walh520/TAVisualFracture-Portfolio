#include "TVFMotion.h"
#include "TVFCollision.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>

namespace tvf {
namespace {
constexpr double Epsilon = 1e-12;

inline double Clamp(double v, double lo, double hi) { return std::max(lo, std::min(v, hi)); }
inline bool FiniteScalar(double v) { return std::isfinite(v); }
inline bool FiniteQuat(Quat q) { return FiniteScalar(q.x) && FiniteScalar(q.y) && FiniteScalar(q.z) && FiniteScalar(q.w); }
inline bool FinitePlane(const PlaneCollider& p) {
    return Finite(p.normal) && FiniteScalar(p.offset) && FiniteScalar(p.restitution) &&
           FiniteScalar(p.friction) && FiniteScalar(p.angularDamping) && FiniteScalar(p.restitutionVelocityThreshold);
}
inline bool FiniteImpact(const ImpactEvent& e) {
    return Finite(e.pointWorld) && Finite(e.directionWorld) && Finite(e.upWorld) &&
           FiniteScalar(e.radius) && FiniteScalar(e.impulse) && FiniteScalar(e.radialWeight) &&
           FiniteScalar(e.directionalWeight) && FiniteScalar(e.upwardWeight) && FiniteScalar(e.randomWeight) &&
           FiniteScalar(e.falloffExponent) && FiniteScalar(e.referenceRadius) &&
           FiniteScalar(e.sizeSpeedBias) && FiniteScalar(e.bondDamage) && FiniteScalar(e.bondRadius);
}

bool ResolvePlaneContact(const PlaneCollider& plane, const ChunkMotionDesc& chunk,
                         ChunkMotionState& state, double& contactAngularDamping, bool applyFriction=true) {
    if (!FinitePlane(plane)) return false;
    const double normalLength = Length(plane.normal);
    if (normalLength <= Epsilon) return false;
    const Vec3 n = plane.normal / normalLength;
    const double normalizedOffset = plane.offset / normalLength;
    const double signedCenterDistance = Dot(n, state.positionWorld) - normalizedOffset;
    const double penetration = chunk.radius - signedCenterDistance;
    if (penetration <= 0) return false;

    state.positionWorld += n * penetration;
    const double normalVelocity = Dot(state.velocityWorld, n);
    const Vec3 tangentialVelocity = state.velocityWorld - n * normalVelocity;
    const double restitution = (-normalVelocity < std::max(0.0, plane.restitutionVelocityThreshold)) ?
        0.0 : Clamp(plane.restitution, 0, 1);
    const double outgoingNormal = normalVelocity < 0 ? -restitution * normalVelocity : normalVelocity;
    const double tangentScale = applyFriction ? 1.0 - Clamp(plane.friction, 0, 1) : 1.0;
    state.velocityWorld = n * outgoingNormal + tangentialVelocity * tangentScale;
    contactAngularDamping = std::max(contactAngularDamping, std::max(0.0, plane.angularDamping));
    return true;
}
inline float ToFloat(double v) {
    const double cap = static_cast<double>(std::numeric_limits<float>::max());
    return static_cast<float>(Clamp(v, -cap, cap));
}

Vec3 StableUnitVector(uint32_t id, uint32_t seed) {
    Random random(Hash(id ^ Hash(seed)));
    const double z = random.Unit() * 2.0 - 1.0;
    const double a = random.Unit() * 2.0 * Pi;
    const double r = std::sqrt(std::max(0.0, 1.0 - z * z));
    return {r * std::cos(a), r * std::sin(a), z};
}

double RadiusFalloff(Vec3 point, Vec3 center, double radius, double exponent) {
    const double distance = Length(center - point);
    if (radius <= Epsilon) return distance <= Epsilon ? 1.0 : 0.0;
    // The edge is outside even when exponent=0: avoid pow(0,0)==1 leaking an impulse to the boundary.
    if (distance >= radius) return 0.0;
    return std::pow(1.0 - distance / radius, std::max(0.0, exponent));
}
} // namespace

Quat Normalize(Quat q) {
    const double lengthSquared = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
    if (!(lengthSquared > Epsilon) || !std::isfinite(lengthSquared)) return {};
    const double invLength = 1.0 / std::sqrt(lengthSquared);
    return {q.x*invLength, q.y*invLength, q.z*invLength, q.w*invLength};
}

Quat Conjugate(Quat q) { return {-q.x, -q.y, -q.z, q.w}; }

Quat Multiply(Quat a, Quat b) {
    return {
        a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
        a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
        a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w,
        a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z
    };
}

Quat QuatFromRotationVector(Vec3 radians) {
    const double angle = Length(radians);
    if (angle < 1e-8) {
        // sin(a/2)/a = 1/2 - a^2/48 + O(a^4), avoiding a small-angle divide.
        const double scale = 0.5 - angle * angle / 48.0;
        return Normalize({radians.x*scale, radians.y*scale, radians.z*scale, 1.0 - angle*angle/8.0});
    }
    const double scale = std::sin(angle * 0.5) / angle;
    return {radians.x*scale, radians.y*scale, radians.z*scale, std::cos(angle*0.5)};
}

Vec3 Rotate(Quat q, Vec3 v) {
    q = Normalize(q);
    const Vec3 u = {q.x, q.y, q.z};
    return v + 2.0 * Cross(u, Cross(u, v) + q.w * v);
}

bool MotionSystem::Initialize(const MotionSettings& settings, const std::vector<ChunkMotionDesc>& chunks,
                              const std::vector<BondMotionDesc>& bonds) {
    if (!(settings.fixedTimeStep > 0) || !FiniteScalar(settings.fixedTimeStep) ||
        !FiniteScalar(settings.linearDrag) || !FiniteScalar(settings.angularDrag) ||
        !FiniteScalar(settings.sleepLinearSpeed) || !FiniteScalar(settings.sleepAngularSpeed) ||
        !FiniteScalar(settings.sleepDelay) || !FiniteScalar(settings.wakeImpulse) ||
        !Finite(settings.initialWorldOffset) || !FiniteQuat(settings.initialWorldRotation) ||
        settings.linearDrag < 0 || settings.angularDrag < 0 || settings.sleepLinearSpeed < 0 ||
        settings.sleepAngularSpeed < 0 || settings.sleepDelay < 0 || settings.wakeImpulse < 0) return false;
    for (const ChunkMotionDesc& chunk : chunks) {
        if (!(chunk.radius > 0) || !(chunk.mass > 0) || !FiniteScalar(chunk.radius) ||
            !FiniteScalar(chunk.mass) || !Finite(chunk.restCentroid)) return false;
    }
    for (const BondMotionDesc& bond : bonds) {
        if (bond.chunkA >= chunks.size() || bond.chunkB >= chunks.size() || bond.chunkA == bond.chunkB ||
            !(bond.resistance > 0) || !FiniteScalar(bond.resistance) || !FiniteScalar(bond.initialDamage) ||
            bond.initialDamage < 0 || !Finite(bond.centroid)) return false;
    }

    settings_ = settings;
    settings_.initialWorldRotation = Normalize(settings_.initialWorldRotation);
    chunks_ = chunks;
    bonds_ = bonds;
    states_.assign(chunks.size(), {});
    bondStates_.assign(bonds.size(), {});
    appliedEvents_.clear();
    accumulator_ = 0;
    for (uint32_t i = 0; i < chunks_.size(); ++i) {
        const ChunkMotionDesc& chunk = chunks_[i];
        ChunkMotionState& state = states_[i];
        state.positionWorld = settings_.initialWorldOffset + Rotate(settings_.initialWorldRotation, chunk.restCentroid);
        state.rotationWorld = settings_.initialWorldRotation;
        state.previousPositionWorld = state.positionWorld;
        state.previousRotationWorld = state.rotationWorld;
        state.sleeping = true;
        state.detached = false;
    }
    for (uint32_t i = 0; i < bonds_.size(); ++i) {
        bondStates_[i].damage = Clamp(bonds_[i].initialDamage, 0.0, bonds_[i].resistance);
        bondStates_[i].broken = bondStates_[i].damage >= bonds_[i].resistance;
    }
    UpdateSupportConnectivity();
    return true;
}

bool MotionSystem::HasSupport() const {
    return std::any_of(chunks_.begin(), chunks_.end(), [](const ChunkMotionDesc& c) { return c.supported; });
}

bool MotionSystem::SetBondDamage(uint32_t bondIndex, double damage) {
    if (bondIndex >= bonds_.size() || !FiniteScalar(damage) || damage < 0) return false;
    bondStates_[bondIndex].damage = std::max(bondStates_[bondIndex].damage, Clamp(damage, 0.0, bonds_[bondIndex].resistance));
    bondStates_[bondIndex].broken = bondStates_[bondIndex].damage >= bonds_[bondIndex].resistance;
    UpdateSupportConnectivity();
    return true;
}

void MotionSystem::UpdateSupportConnectivity() {
    if (chunks_.empty() || !HasSupport()) return;
    std::vector<std::vector<uint32_t>> adjacency(chunks_.size());
    for (uint32_t i = 0; i < bonds_.size(); ++i) {
        if (bondStates_[i].broken) continue;
        const BondMotionDesc& bond = bonds_[i];
        adjacency[bond.chunkA].push_back(bond.chunkB);
        adjacency[bond.chunkB].push_back(bond.chunkA);
    }
    std::vector<bool> reachable(chunks_.size(), false);
    std::queue<uint32_t> pending;
    for (uint32_t i = 0; i < chunks_.size(); ++i) {
        if (chunks_[i].supported) { reachable[i] = true; pending.push(i); }
    }
    while (!pending.empty()) {
        const uint32_t a = pending.front(); pending.pop();
        for (uint32_t b : adjacency[a]) if (!reachable[b]) { reachable[b] = true; pending.push(b); }
    }
    // There is no internal rigid-cluster constraint solver. Once an unsupported island leaves its support,
    // sever its remaining bonds so the public bond state never pretends it is still constrained.
    for (uint32_t i = 0; i < bonds_.size(); ++i) {
        const BondMotionDesc& bond = bonds_[i];
        if (!reachable[bond.chunkA] && !reachable[bond.chunkB]) {
            bondStates_[i].broken = true;
            bondStates_[i].damage = bonds_[i].resistance;
        }
    }
    for (uint32_t i = 0; i < chunks_.size(); ++i) {
        if (!reachable[i] && !states_[i].detached) {
            states_[i].detached = true;
            states_[i].sleeping = false;
            states_[i].sleepTime = 0;
        }
        if (reachable[i]) {
            states_[i].detached = false;
            states_[i].sleeping = true;
            states_[i].velocityWorld = {};
            states_[i].angularVelocityWorld = {};
            // Intact supported parts are kinematic at their rest pose.
            states_[i].positionWorld = settings_.initialWorldOffset + Rotate(settings_.initialWorldRotation, chunks_[i].restCentroid);
            states_[i].rotationWorld = settings_.initialWorldRotation;
        }
    }
}

void MotionSystem::Wake(uint32_t chunkIndex) {
    ChunkMotionState& state = states_[chunkIndex];
    state.sleeping = false;
    state.sleepTime = 0;
}

bool MotionSystem::ApplyImpact(const ImpactEvent& impact) {
    if (!FiniteImpact(impact)) return false;
    if (impact.eventId != 0 && !appliedEvents_.insert(impact.eventId).second) return false;
    if (chunks_.empty()) return true;

    const double damageRadius = impact.bondRadius > 0 ? impact.bondRadius : impact.radius;
    if (impact.bondDamage > 0) {
        for (uint32_t i = 0; i < bonds_.size(); ++i) {
            if (bondStates_[i].broken) continue;
            const Vec3 worldCentroid = settings_.initialWorldOffset +
                Rotate(settings_.initialWorldRotation, bonds_[i].centroid);
            const double f = RadiusFalloff(impact.pointWorld, worldCentroid, damageRadius, impact.falloffExponent);
            bondStates_[i].damage += impact.bondDamage * f;
            if (bondStates_[i].damage >= bonds_[i].resistance) {
                bondStates_[i].damage = bonds_[i].resistance;
                bondStates_[i].broken = true;
            }
        }
        UpdateSupportConnectivity();
    }

    const bool supportedModel = HasSupport();
    bool anyAffected = false;
    for (uint32_t i = 0; i < chunks_.size(); ++i)
        anyAffected |= RadiusFalloff(impact.pointWorld, states_[i].positionWorld, impact.radius, impact.falloffExponent) > 0;
    // With no declared support there is also no constraint model. The first effective hit releases all chunks
    // and invalidates all remaining bonds; only in-radius chunks receive this event's impulse.
    if (!supportedModel && anyAffected) {
        for (ChunkMotionState& state : states_) { state.detached = true; state.sleeping = false; state.sleepTime = 0; }
        for (uint32_t i = 0; i < bonds_.size(); ++i) { bondStates_[i].broken = true; bondStates_[i].damage = bonds_[i].resistance; }
    }
    const Vec3 preferredDirection = Normalize(impact.directionWorld, {1, 0, 0});
    const Vec3 up = Normalize(impact.upWorld, {0, 0, 1});
    for (uint32_t i = 0; i < chunks_.size(); ++i) {
        ChunkMotionState& state = states_[i];
        const double f = RadiusFalloff(impact.pointWorld, state.positionWorld, impact.radius, impact.falloffExponent);
        if (f <= 0) continue;
        if (!state.detached) continue;

        const Vec3 radial = Normalize(state.positionWorld - impact.pointWorld, StableUnitVector(chunks_[i].id, impact.randomSeed));
        const Vec3 random = StableUnitVector(chunks_[i].id, impact.randomSeed);
        const Vec3 direction = Normalize(radial * impact.radialWeight + preferredDirection * impact.directionalWeight +
                                         up * impact.upwardWeight + random * impact.randomWeight, radial);
        const double radiusRatio = std::max(1e-6, impact.referenceRadius) / std::max(1e-6, chunks_[i].radius);
        const double sizeFactor = std::pow(radiusRatio, impact.sizeSpeedBias);
        const double magnitude = std::max(0.0, impact.impulse) * f * sizeFactor;
        const Vec3 impulse = direction * magnitude;
        state.velocityWorld += impulse / chunks_[i].mass;
        // A solid sphere is an explicit approximation: I = 2/5 m r^2 about every centroid axis.
        const double inertia = 0.4 * chunks_[i].mass * chunks_[i].radius * chunks_[i].radius;
        const Vec3 arm = impact.pointWorld - state.positionWorld;
        if (collisionScene_ && !collisionScene_->settings.groundOnly && i < collisionScene_->bodies.size()) {
            const Vec3 torqueLocal = Rotate(Conjugate(state.rotationWorld), Cross(arm, impulse));
            state.angularVelocityWorld += Rotate(state.rotationWorld, collisionScene_->bodies[i].inverseInertia.Apply(torqueLocal));
        } else state.angularVelocityWorld += Cross(arm, impulse) / std::max(inertia, Epsilon);
        if (magnitude >= settings_.wakeImpulse) Wake(i);
    }
    return true;
}

uint32_t MotionSystem::Step(double frameDeltaSeconds, const MotionStepInput& input) {
    return Step(frameDeltaSeconds, input, {});
}

uint32_t MotionSystem::Step(double frameDeltaSeconds, const MotionStepInput& input,
                            const GroundPlaneQuery& groundQuery) {
    // Previous poses are render-frame endpoints, not the penultimate fixed substep.
    for (ChunkMotionState& state : states_) { state.previousPositionWorld = state.positionWorld; state.previousRotationWorld = state.rotationWorld; }
    if (!(frameDeltaSeconds > 0) || !FiniteScalar(frameDeltaSeconds) || chunks_.empty()) return 0;
    accumulator_ += std::min(frameDeltaSeconds, 0.25); // prevent a debugger hitch from running an unbounded catch-up loop.
    uint32_t count = 0;
    while (accumulator_ + Epsilon >= settings_.fixedTimeStep) {
        StepFixed(input, groundQuery ? &groundQuery : nullptr);
        accumulator_ -= settings_.fixedTimeStep;
        ++count;
    }
    return count;
}

void MotionSystem::StepFixed(const MotionStepInput& input, const GroundPlaneQuery* groundQuery) {
    if (collisionScene_ && !collisionScene_->settings.groundOnly) {
        if (collisionScene_->Advance(settings_.fixedTimeStep, settings_, input,
            groundQuery ? *groundQuery : GroundPlaneQuery{}, states_)) return;
        // Failed collision steps commit no poses. Visual budget overload falls
        // back for this step only; retry full collisions on the next fixed step.
        if (!collisionScene_->settings.simpleVisual || !collisionScene_->statistics.budgetExceeded) return;
    }
    const double h = settings_.fixedTimeStep;
    const bool groundAvoidance=collisionScene_ && collisionScene_->settings.groundOnly;
    std::vector<uint8_t> grounded(groundAvoidance?states_.size():0,0);
    std::vector<Vec3> contactNormals(groundAvoidance?states_.size():0);
    std::vector<double> contactFriction(groundAvoidance?states_.size():0,0);
    uint64_t groundQueries=0;
    for (uint32_t i = 0; i < chunks_.size(); ++i) {
        ChunkMotionState& state = states_[i];
        if(groundAvoidance && state.detached && state.sleeping)grounded[i]=1;
        if (!state.detached || state.sleeping) continue;

        const ChunkFieldInput rawField = i < input.fields.size() ? input.fields[i] : ChunkFieldInput{};
        const ChunkFieldInput field = {Finite(rawField.windWorld) ? rawField.windWorld : Vec3{},
            Finite(rawField.accelerationWorld) ? rawField.accelerationWorld : Vec3{},
            FiniteScalar(rawField.dragScale) ? rawField.dragScale : 0.0};
        const Vec3 wind = (Finite(input.windWorld) ? input.windWorld : Vec3{}) + field.windWorld;
        const Vec3 acceleration = (Finite(input.gravityWorld) ? input.gravityWorld : Vec3{}) + field.accelerationWorld;
        const double lambda = std::max(0.0, settings_.linearDrag * std::max(0.0, field.dragScale));
        const Vec3 oldVelocity = state.velocityWorld;
        const double x = lambda * h;
        const double A = x < 1e-4 ? h * (1.0 - x*0.5 + x*x/6.0 - x*x*x/24.0) : -std::expm1(-x) / lambda;
        const double B = x < 1e-4 ? h*h * (0.5 - x/6.0 + x*x/24.0 - x*x*x/120.0) : (h - A) / lambda;
        state.positionWorld += oldVelocity * h + (acceleration - lambda * (oldVelocity - wind)) * B;
        state.velocityWorld = wind + (oldVelocity - wind) * std::exp(-x) + acceleration * A;
        const Vec3 oldAngularVelocity = state.angularVelocityWorld;
        const double angularLambda = settings_.angularDrag;
        const double angularTravel = angularLambda * h < 1e-4 ? h * (1.0 - angularLambda*h*0.5 + angularLambda*angularLambda*h*h/6.0) :
            -std::expm1(-angularLambda*h) / angularLambda;
        state.angularVelocityWorld = oldAngularVelocity * std::exp(-angularLambda * h);
        // World-space omega is left-multiplied and integrated over its drag-decayed angular travel.
        state.rotationWorld = Normalize(Multiply(QuatFromRotationVector(oldAngularVelocity * angularTravel), state.rotationWorld));

        bool contact = false;
        double contactAngularDamping = 0;
        for (const PlaneCollider& plane : input.planes) {
            if(ResolvePlaneContact(plane, chunks_[i], state, contactAngularDamping,!groundAvoidance)){
                contact=true;if(groundAvoidance){contactNormals[i]=Normalize(plane.normal);contactFriction[i]=Clamp(plane.friction,0,1);}
            }
        }
        if (groundQuery) {
            ++groundQueries;
            PlaneCollider ground;
            if ((*groundQuery)(i, state.positionWorld, ground)) {
                if(ResolvePlaneContact(ground, chunks_[i], state, contactAngularDamping,!groundAvoidance)){
                    contact=true;if(groundAvoidance){contactNormals[i]=Normalize(ground.normal);contactFriction[i]=Clamp(ground.friction,0,1);}
                }
            }
        }
        if (contactAngularDamping > 0)
            state.angularVelocityWorld = state.angularVelocityWorld * std::exp(-contactAngularDamping * h);

        if(groundAvoidance)grounded[i]=contact?1:0;

        if (contact && Length(state.velocityWorld) <= settings_.sleepLinearSpeed &&
            Length(state.angularVelocityWorld) <= settings_.sleepAngularSpeed) {
            state.sleepTime += h;
            if (state.sleepTime >= settings_.sleepDelay) {
                state.sleeping = true;
                state.velocityWorld = {};
                state.angularVelocityWorld = {};
            }
        } else {
            state.sleepTime = 0;
        }
    }
    if(groundAvoidance){
        std::vector<uint8_t> moved;
        collisionScene_->AvoidOnGround(h,grounded,states_,moved);
        // Sample the NEW XY, never retain an invisible floor outside the capture.
        for(uint32_t i=0;i<moved.size();++i)if(moved[i]){
            double damping=0;contactNormals[i]={};
            auto refreshPlane=[&](const PlaneCollider& plane){
                const bool corrected=ResolvePlaneContact(plane,chunks_[i],states_[i],damping,false);
                const double length=Length(plane.normal);
                if(FinitePlane(plane)&&length>Epsilon&&(corrected||std::abs((Dot(plane.normal,states_[i].positionWorld)-plane.offset)/length-chunks_[i].radius)<=1e-6)){contactNormals[i]=plane.normal/length;contactFriction[i]=Clamp(plane.friction,0,1);}
            };
            for(const auto& plane:input.planes)refreshPlane(plane);
            if(groundQuery){++groundQueries;PlaneCollider plane;if((*groundQuery)(i,states_[i].positionWorld,plane))refreshPlane(plane);}
        }
        const auto& visual=collisionScene_->settings;
        for(size_t i=0;i<states_.size();++i){auto& s=states_[i];const auto n=contactNormals[i];
            if(!s.detached||s.sleeping||LengthSquared(n)<.5||Dot(s.velocityWorld,n)>1e-6)continue;
            const auto normal=n*Dot(s.velocityWorld,n);
            s.velocityWorld=normal+(s.velocityWorld-normal)*std::exp(-visual.groundFrictionDecay*contactFriction[i]*h);
        }
        if(visual.groundVisualRoll&&visual.groundRollStrength>0){
            for(size_t i=0;i<states_.size();++i){auto& s=states_[i];const Vec3 n=contactNormals[i];
                if(!s.detached||s.sleeping||LengthSquared(n)<.5||Dot(s.velocityWorld,n)>1e-6)continue;
                const Vec3 tangent=s.velocityWorld-n*Dot(s.velocityWorld,n);const double speed=Length(tangent);
                if(speed<=5)continue;
                const double t=std::clamp((speed-5)/15.0,0.0,1.0),fade=t*t*(3-2*t);
                // Visual pose only: use physical tangential speed, never the XY
                // depenetration displacement, and retain the original radius.
                const Vec3 angle=Cross(n,tangent)*(visual.groundRollStrength*fade*h/chunks_[i].radius);
                s.rotationWorld=Normalize(Multiply(QuatFromRotationVector(angle),s.rotationWorld));
            }
        }
        collisionScene_->statistics.groundQueries=groundQueries;
    }
}

Vec3 MotionSystem::TransformPoint(uint32_t chunkIndex, Vec3 restPoint) const {
    if (chunkIndex >= states_.size()) return restPoint;
    return states_[chunkIndex].positionWorld + Rotate(states_[chunkIndex].rotationWorld, restPoint - chunks_[chunkIndex].restCentroid);
}

Vec3 MotionSystem::TransformPreviousPoint(uint32_t chunkIndex, Vec3 restPoint) const {
    if (chunkIndex >= states_.size()) return restPoint;
    return states_[chunkIndex].previousPositionWorld + Rotate(states_[chunkIndex].previousRotationWorld, restPoint - chunks_[chunkIndex].restCentroid);
}

void MotionSystem::PackGpu(std::vector<GpuChunkMotion>& out) const {
    out.resize(chunks_.size());
    for (uint32_t i = 0; i < chunks_.size(); ++i) {
        const ChunkMotionDesc& chunk = chunks_[i];
        const ChunkMotionState& state = states_[i];
        GpuChunkMotion& gpu = out[i];
        gpu.currentPositionRadius[0] = ToFloat(state.positionWorld.x); gpu.currentPositionRadius[1] = ToFloat(state.positionWorld.y);
        gpu.currentPositionRadius[2] = ToFloat(state.positionWorld.z); gpu.currentPositionRadius[3] = ToFloat(chunk.radius);
        gpu.currentRotation[0] = ToFloat(state.rotationWorld.x); gpu.currentRotation[1] = ToFloat(state.rotationWorld.y);
        gpu.currentRotation[2] = ToFloat(state.rotationWorld.z); gpu.currentRotation[3] = ToFloat(state.rotationWorld.w);
        gpu.previousPositionRadius[0] = ToFloat(state.previousPositionWorld.x); gpu.previousPositionRadius[1] = ToFloat(state.previousPositionWorld.y);
        gpu.previousPositionRadius[2] = ToFloat(state.previousPositionWorld.z); gpu.previousPositionRadius[3] = ToFloat(chunk.radius);
        gpu.previousRotation[0] = ToFloat(state.previousRotationWorld.x); gpu.previousRotation[1] = ToFloat(state.previousRotationWorld.y);
        gpu.previousRotation[2] = ToFloat(state.previousRotationWorld.z); gpu.previousRotation[3] = ToFloat(state.previousRotationWorld.w);
        gpu.restCentroidInvMass[0] = ToFloat(chunk.restCentroid.x); gpu.restCentroidInvMass[1] = ToFloat(chunk.restCentroid.y);
        gpu.restCentroidInvMass[2] = ToFloat(chunk.restCentroid.z); gpu.restCentroidInvMass[3] = ToFloat(1.0 / chunk.mass);
        gpu.velocityMass[0] = ToFloat(state.velocityWorld.x); gpu.velocityMass[1] = ToFloat(state.velocityWorld.y);
        gpu.velocityMass[2] = ToFloat(state.velocityWorld.z); gpu.velocityMass[3] = ToFloat(chunk.mass);
        gpu.angularVelocitySleeping[0] = ToFloat(state.angularVelocityWorld.x); gpu.angularVelocitySleeping[1] = ToFloat(state.angularVelocityWorld.y);
        gpu.angularVelocitySleeping[2] = ToFloat(state.angularVelocityWorld.z); gpu.angularVelocitySleeping[3] = state.sleeping ? 1.0f : 0.0f;
        gpu.chunkIdFlags[0] = chunk.id; gpu.chunkIdFlags[1] = (state.sleeping ? 1u : 0u) | (state.detached ? 2u : 0u);
        gpu.chunkIdFlags[2] = 0; gpu.chunkIdFlags[3] = 0;
    }
}

} // namespace tvf
