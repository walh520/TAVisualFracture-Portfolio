#pragma once
#include "TVFMotion.h"
#include <string>
#include <set>

namespace tvf {
struct CollisionMatrix {
    double m[3][3]{};
    Vec3 Apply(Vec3 v) const;
};
struct CollisionHull {
    std::vector<Vec3> vertices; // body-local, relative to existing Chunk centroid
    std::vector<std::vector<uint32_t>> faces; // outward, convex polygons
};
struct CollisionBody {
    std::vector<CollisionHull> hulls;
    CollisionMatrix inverseInertia;
    double mass=1, minThickness=1, radius=1;
    double friction=.4,restitution=.15,bounceThreshold=20;
    uint32_t mask=~0u;
    uint64_t geometryVersion=1;
};
struct StaticCollisionBody { CollisionBody body; Vec3 position; Quat rotation; std::string stableId,geometryKey; };
struct CollisionContact {
    uint32_t a=0,b=0; // b >= dynamic body count denotes static environment
    Vec3 localA,localB,normal;
    double gap=0,lambda=0,normalImpulse=0,targetSpeed=0;
    Vec3 tangentImpulse;
    uint64_t feature=0;
    uint64_t geometryA=0,geometryB=0;
    uint32_t age=0;
    double friction=.4,restitution=.15,bounceThreshold=20,angularDamping=0;
};
struct CollisionSettings {
    bool groundOnly=false; // lightweight XY avoidance, called after MotionSystem ground contact
    uint32_t avoidanceIterations=4;
    double avoidanceSpeed=0; // cm/s positional correction; zero = immediate
    double xyFootprintScale=1;
    bool groundVisualRoll=true;
    double groundRollStrength=.35;
    double externalVelocityTransfer=1,externalLiftRatio=.15,externalLiftThreshold=20;
    double groundFrictionDecay=6;
    bool simpleVisual=false; // Editor opts in; existing core callers retain strict behavior.
    bool selfCollision=true,externalCollision=true;
    uint32_t substeps=4,maxSubsteps=8,positionIterations=8,velocityIterations=2;
    uint64_t maxPairs=100000,maxShapePairs=100000,maxContacts=16384;
    uint64_t maxGroundQueries=4096;
    double contactOffset=.2,restOffset=0,allowedPenetration=.2,compliance=0;
};
struct CollisionStatistics {
    uint64_t pairs=0,shapePairs=0,contacts=0,sleeping=0,groundQueries=0;
    double maxPenetration=0;
    uint32_t substeps=0;
    bool paused=false;
    bool budgetExceeded=false; // visual mode requests transactional legacy-ground fallback
    std::string reason;
};
// World-space geometry entrypoints also used by the independent acceptance runner.
TAVISUALFRACTURECORE_API CollisionHull MakeCollisionBox(Vec3 halfExtent);
// Collision proxy utilities only; never used by the fracture Bake pipeline.
TAVISUALFRACTURECORE_API Vec3 CollisionFaceAreaVector(const CollisionHull&,const std::vector<uint32_t>& face);
TAVISUALFRACTURECORE_API std::vector<uint32_t> BuildCollisionFaceBoundary(const std::vector<Vec3>& points,const std::vector<uint32_t>& candidates,Vec3 outwardNormal);
// Collision-only conversion, shared with the Editor and standalone regressions.
TAVISUALFRACTURECORE_API bool BuildCollisionHullFromTriangles(const std::vector<Vec3>& points,const std::vector<std::array<uint32_t,3>>& triangles,CollisionHull& out,std::string& error);
TAVISUALFRACTURECORE_API bool ValidateCollisionHull(const CollisionHull&,std::string& error);
TAVISUALFRACTURECORE_API bool ComputeCollisionInertia(const std::vector<std::array<Vec3,3>>&,double mass,CollisionMatrix& inverse,std::string& error);
TAVISUALFRACTURECORE_API std::vector<CollisionContact> CollideConvex(const CollisionHull& a,const CollisionHull& b,double offset=0);

class TAVISUALFRACTURECORE_API CollisionScene {
public:
    CollisionSettings settings;
    std::vector<CollisionBody> bodies;
    std::vector<StaticCollisionBody> environment;
    CollisionStatistics statistics;
    bool Validate(std::string& error) const;
    bool Advance(double h,const MotionSettings&,const MotionStepInput&,const GroundPlaneQuery&,
                 std::vector<ChunkMotionState>&);
    void AvoidOnGround(double h,const std::vector<uint8_t>& grounded,std::vector<ChunkMotionState>&,std::vector<uint8_t>& moved);
    void ReplaceEnvironment(std::vector<StaticCollisionBody> updated);
    void SampleEnvironment(std::vector<StaticCollisionBody> updated,double sampleTimeSeconds);
    void Reset(){statistics={};preStabilized_=false;avoidanceCursor_=0;planarBoxes_.clear();planarBoxesReady_=false;environmentChanged_=true;environmentSampleTime_=-1;externalContacts_.clear();}
private:
    struct PlanarBox {Vec3 min,max,previousMin,previousMax,velocityMin,velocityMax;uint32_t mask;double restitution=0;std::string key;bool motionValid=false;};
    std::vector<PlanarBox> planarBoxes_;
    bool planarBoxesReady_=false;
    bool environmentChanged_=true;
    double environmentSampleTime_=-1;
    std::set<std::pair<size_t,std::string>> externalContacts_;
    size_t avoidanceCursor_=0;
    bool preStabilized_=false;
};
}
