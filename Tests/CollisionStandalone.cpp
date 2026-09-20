#include "TVFCollision.h"
#include <iostream>
#include <stdexcept>
#include <limits>
using namespace tvf;
int checks=0;
void Check(bool ok,const char* name){++checks;if(!ok)throw std::runtime_error(name);}
CollisionBody Box(Vec3 e,double mass=1){CollisionBody b;b.hulls={MakeCollisionBox(e)};b.mass=mass;b.radius=Length(e);b.minThickness=2*std::min({e.x,e.y,e.z});std::vector<std::array<Vec3,3>> triangles;for(const auto& f:b.hulls[0].faces)for(size_t i=1;i+1<f.size();++i)triangles.push_back({b.hulls[0].vertices[f[0]],b.hulls[0].vertices[f[i]],b.hulls[0].vertices[f[i+1]]});std::string error;Check(ComputeCollisionInertia(triangles,mass,b.inverseInertia,error),"box inertia");return b;}
CollisionHull Shift(CollisionHull h,Vec3 p){for(auto& v:h.vertices)v+=p;return h;}
std::vector<ChunkMotionState> Drop(int count,int fps){CollisionScene scene;std::vector<ChunkMotionState>s;for(int i=0;i<count;++i){scene.bodies.push_back(Box({5,5,5}));ChunkMotionState b;b.positionWorld={double(i%8)*12,double(i/8)*12,8};b.detached=true;b.sleeping=false;s.push_back(b);}MotionSettings m;MotionStepInput input;input.planes.push_back({});double accumulator=0;for(int frame=0;frame<fps*3;++frame){accumulator+=1.0/fps;while(accumulator+1e-12>=m.fixedTimeStep){Check(scene.Advance(m.fixedTimeStep,m,input,{},s),"drop advance");accumulator-=m.fixedTimeStep;}}for(const auto& b:s){Check(Finite(b.positionWorld)&&Finite(b.velocityWorld),"finite drop");Check(b.positionWorld.z>=4.8,"ground penetration");Check(Length(b.velocityWorld)<1,"settled velocity");}return s;}
int main(){try{
{
        auto trajectory=[](int fps){
            CollisionScene c;c.settings.groundOnly=true;c.settings.selfCollision=false;
            CollisionBody b;b.radius=1;c.bodies={b};
            StaticCollisionBody box;box.body=Box({2,2,2});box.position={-4,0,1};box.stableId="trajectory";c.SampleEnvironment({box},0);
            ChunkMotionDesc d;d.mass=1;d.radius=1;d.restCentroid={0,0,1};
            MotionSystem m;MotionSettings settings;settings.linearDrag=settings.angularDrag=0;
            Check(m.Initialize(settings,{d},{}),"moving trajectory initialize");m.SetCollisionScene(&c);
            ImpactEvent release;release.eventId=1;release.pointWorld=d.restCentroid;release.radius=100;release.impulse=0;m.ApplyImpact(release);
            MotionStepInput in;GroundPlaneQuery ground=[](uint32_t,const Vec3&,PlaneCollider& p){p={};p.friction=0;p.restitution=0;return true;};
            for(int frame=1;frame<=fps;++frame){double t=double(frame)/fps;box.position.x=-4+90*std::min(t,.2);c.SampleEnvironment({box},t);m.Step(1.0/fps,in,ground);}
            return m.States()[0];
        };
        auto a=trajectory(30),b=trajectory(60),c=trajectory(120);
        Check(a.velocityWorld.x>90&&b.velocityWorld.x>90&&c.velocityWorld.x>90,"moving trajectory retains inertia after contact");
        Check(Length(a.velocityWorld-b.velocityWorld)<1e-7&&Length(c.velocityWorld-b.velocityWorld)<1e-7,"sampled constant-speed impulse agrees across FPS");
        Check(Length(a.positionWorld-b.positionWorld)<4&&Length(c.positionWorld-b.positionWorld)<4,"sampled contact timing bounded across FPS, not CCD");

        CollisionScene scene;scene.settings.groundOnly=true;scene.settings.selfCollision=false;CollisionBody body;body.radius=1;scene.bodies={body};
        StaticCollisionBody box;box.body=Box({4,1,2});box.stableId="rotation";box.position={0,0,1};
        scene.SampleEnvironment({box},0);box.rotation=QuatFromRotationVector({0,0,1.5707963267948966});scene.SampleEnvironment({box},.1);
        std::vector<ChunkMotionState> states(1);states[0].detached=true;states[0].positionWorld={0,3,1};std::vector<uint8_t> moved;
        scene.AvoidOnGround(1.0/60,{1},states,moved);
        Check(states[0].velocityWorld.y>30&&states[0].velocityWorld.z>0,"rotated boundary expansion supplies approximate surface velocity");
    }
{
        auto strike=[](int iterations,double transfer,double threshold,bool replaceGeometry,bool airborne){
            CollisionScene c;c.settings.groundOnly=true;c.settings.selfCollision=false;c.settings.avoidanceIterations=iterations;c.settings.externalVelocityTransfer=transfer;c.settings.externalLiftThreshold=threshold;
            CollisionBody body;body.radius=1;c.bodies={body};
            StaticCollisionBody box;box.body=Box({2,2,2});box.position={-4,0,1};box.stableId="actor/mesh/0";box.geometryKey="box1";
            c.SampleEnvironment({box},0);
            std::vector<ChunkMotionState> s(1);s[0].positionWorld={0,0,1};s[0].detached=true;s[0].sleeping=!airborne;
            box.position.x=-2;if(replaceGeometry)box.geometryKey="box2";c.SampleEnvironment({box},1.0/60);
            std::vector<uint8_t> moved;c.AvoidOnGround(1.0/60,{uint8_t(!airborne)},s,moved);
            return s[0];
        };
        auto hit=strike(1,1,20,false,false),many=strike(12,1,20,false,false);
        Check(std::abs(hit.velocityWorld.x-138)<1e-8,"moving Box transfers boundary speed with restitution");
        Check(std::abs(hit.velocityWorld.z-18)<1e-8&&!hit.sleeping,"moving Box lifts and wakes");
        Check(Length(hit.velocityWorld-many.velocityWorld)<1e-10,"iterations do not amplify impulse");
        Check(Length(strike(4,0,20,false,false).velocityWorld)==0,"transfer zero leaves correction only");
        Check(strike(4,1,200,false,false).velocityWorld.z==0,"lift threshold suppresses small hit");
        Check(Length(strike(4,1,20,true,false).velocityWorld)==0,"geometry replacement establishes baseline");
        Check(Length(strike(4,1,20,false,true).velocityWorld)==0,"airborne external impact disabled");

        CollisionScene c;c.settings.groundOnly=true;c.settings.selfCollision=false;c.settings.avoidanceSpeed=1;
        CollisionBody body;body.radius=1;c.bodies={body};
        StaticCollisionBody box;box.body=Box({2,2,2});box.position={-4,0,1};box.stableId="stable";
        c.SampleEnvironment({box},0);box.position.x=-2;c.SampleEnvironment({box},1.0/60);
        std::vector<ChunkMotionState> s(1);s[0].detached=true;s[0].positionWorld={0,0,1};
        std::vector<uint8_t> moved;c.AvoidOnGround(1.0/60,{1},s,moved);
        s[0].velocityWorld={};box.position.x=-1;c.SampleEnvironment({box},2.0/60);c.AvoidOnGround(1.0/60,{1},s,moved);
        Check(s[0].velocityWorld.x>0&&s[0].velocityWorld.z==0,"continuous overlap does not repeat lift");
        s[0].velocityWorld={};c.SampleEnvironment({box},3.0/60);c.AvoidOnGround(1.0/60,{1},s,moved);
        Check(Length(s[0].velocityWorld)==0,"stop sampling clears external velocity");
        c.SampleEnvironment({},4.0/60);auto before=s[0].positionWorld;c.AvoidOnGround(1.0/60,{1},s,moved);
        Check(Length(s[0].positionWorld-before)==0&&s[0].detached,"removing Box preserves chunk state");
        c.SampleEnvironment({box},5.0/60);c.AvoidOnGround(1.0/60,{1},s,moved);
        Check(Length(s[0].velocityWorld)==0,"re-registration produces no impulse");
        box.position.x=0;c.SampleEnvironment({box},5.0/60);c.AvoidOnGround(1.0/60,{1},s,moved);
        Check(Length(s[0].velocityWorld)==0,"invalid sample interval produces no impulse");
    }
    {
        auto friction=[](int fps,bool resample){
            CollisionScene c;c.settings.groundOnly=true;c.settings.selfCollision=false;CollisionBody b;b.radius=1;c.bodies={b};
            if(resample){StaticCollisionBody box;box.body=Box({2,100,2});box.position={-2,0,1};c.ReplaceEnvironment({box});}
            ChunkMotionDesc d;d.mass=1;d.radius=1;d.restCentroid={0,0,1};
            MotionSettings settings;settings.linearDrag=settings.angularDrag=0;
            MotionSystem m;Check(m.Initialize(settings,{d},{}),"friction initialize");m.SetCollisionScene(&c);
            ImpactEvent hit;hit.eventId=1;hit.pointWorld=d.restCentroid;hit.radius=100;hit.impulse=100;hit.radialWeight=hit.randomWeight=hit.upwardWeight=0;hit.directionalWeight=1;hit.directionWorld={0,1,0};m.ApplyImpact(hit);
            MotionStepInput in;GroundPlaneQuery ground=[](uint32_t,const Vec3&,PlaneCollider& p){p={};p.friction=.4;p.restitution=0;return true;};
            for(int frame=0;frame<fps;++frame)m.Step(1.0/fps,in,ground);
            return m.States()[0];
        };
        auto a=friction(30,false),b=friction(60,false),c=friction(120,false),r=friction(60,true);
        Check(std::abs(b.velocityWorld.y-100*std::exp(-2.4))<1e-8,"ground friction exponential per second");
        Check(Length(a.velocityWorld-b.velocityWorld)<1e-9&&Length(c.velocityWorld-b.velocityWorld)<1e-9,"friction fixed-step FPS invariant");
        Check(std::abs(r.velocityWorld.y-b.velocityWorld.y)<1e-9,"post-correction resample does not double friction");
        Check(b.positionWorld.y>20,"released chunk continues sliding");
    }
    {
        CollisionScene xy;xy.settings.groundOnly=true;xy.settings.simpleVisual=true;xy.settings.selfCollision=false;
        CollisionBody body;body.radius=1;xy.bodies={body,body};
        std::vector<ChunkMotionState> s(2);for(auto& state:s){state.detached=true;state.sleeping=true;state.positionWorld={0,0,1};}s[1].positionWorld.x=50;
        StaticCollisionBody wall;wall.body=Box({2,2,2});wall.position={20,0,1};xy.ReplaceEnvironment({wall});std::vector<uint8_t> moved;
        xy.AvoidOnGround(1.0/60,{1,1},s,moved);Check(s[0].sleeping,"far moved Box does not wake pile");
        wall.position={0,0,1};xy.ReplaceEnvironment({wall});
        Check(s[0].detached&&s[0].positionWorld.x==0,"external cache update preserves fractured state");
        xy.AvoidOnGround(1.0/60,{1,1},s,moved);
        Check(!s[0].sleeping&&s[0].positionWorld.x<=-3,"moving Box wakes and pushes overlapping sleeping chunk");
        Check(s[1].sleeping&&s[1].positionWorld.x==50,"unaffected sleeping chunk remains unchanged");
        xy.ReplaceEnvironment({});s[0].positionWorld={0,0,1};xy.AvoidOnGround(1.0/60,{1,1},s,moved);
        Check(s[0].positionWorld.x==0,"removed obstacle leaves no stale rectangle");
        xy.settings.avoidanceSpeed=60;xy.ReplaceEnvironment({wall});xy.AvoidOnGround(1.0/60,{1,1},s,moved);
        Check(std::abs(s[0].positionWorld.x+1)<1e-9,"positive speed limits total displacement across iterations");
        Check(Length(s[0].velocityWorld)==0&&!xy.statistics.paused,"limited correction does not inject velocity or pause");
        xy.settings.avoidanceSpeed=0;xy.AvoidOnGround(1.0/60,{1,1},s,moved);
        Check(s[0].positionWorld.x<=-3,"zero speed immediately resolves remaining penetration");
        xy.settings.selfCollision=true;xy.ReplaceEnvironment({});xy.settings.avoidanceSpeed=30;
        s[0].positionWorld=s[1].positionWorld={0,0,1};s[0].sleeping=s[1].sleeping=false;
        xy.AvoidOnGround(1.0/60,{1,1},s,moved);
        Check(std::abs(s[0].positionWorld.x)==.5&&std::abs(s[1].positionWorld.x)==.5,"self collision respects per-body correction speed");
    }
    {
        for(double scale:{1.0,.75,.5}){
            CollisionScene xy;xy.settings.groundOnly=true;xy.settings.xyFootprintScale=scale;xy.settings.avoidanceIterations=8;xy.settings.maxPairs=xy.settings.maxShapePairs=0;xy.settings.avoidanceSpeed=0;
            CollisionBody b;b.radius=50;xy.bodies={b,b};std::vector<ChunkMotionState> s(2);for(auto& v:s){v.detached=true;v.sleeping=false;v.positionWorld={0,0,50};}
            std::vector<uint8_t> moved;xy.AvoidOnGround(1.0/60,{1,1},s,moved);
            Check(std::abs(Length(s[0].positionWorld-s[1].positionWorld)-100*scale)<1e-8,"deep overlap fully separated in one fixed step");
            Check(s[0].positionWorld.z==50&&s[1].positionWorld.z==50,"footprint scaling does not change height");
            Check(Length(s[0].velocityWorld)==0&&Length(s[0].angularVelocityWorld)==0,"separation creates no physical motion");
            Check(!xy.statistics.budgetExceeded&&!xy.statistics.paused,"zero legacy budgets ignored");
            xy.Reset();xy.bodies.resize(1);s.resize(1);s[0].positionWorld={0,0,50};StaticCollisionBody wall;wall.body=Box({100,100,100});xy.environment={wall};
            xy.AvoidOnGround(1.0/60,{1},s,moved);
            Check(std::abs(s[0].positionWorld.x+100+50*scale)<1e-8,"external spacing scales but Box size does not");
        }
        auto roll=[](double speed,double scale,bool enabled,double strength,double height,int frames){
            CollisionScene xy;xy.settings.groundOnly=true;xy.settings.simpleVisual=true;xy.settings.xyFootprintScale=scale;xy.settings.groundVisualRoll=enabled;xy.settings.groundRollStrength=strength;CollisionBody body;body.radius=10;xy.bodies={body};
            ChunkMotionDesc desc;desc.radius=10;desc.mass=1;desc.restCentroid={0,0,height};MotionSettings m;m.linearDrag=m.angularDrag=0;
            MotionSystem motion;Check(motion.Initialize(m,{desc},{}),"roll fixture initialize");motion.SetCollisionScene(&xy);ImpactEvent hit;hit.eventId=1;hit.pointWorld=desc.restCentroid;hit.radius=100;hit.impulse=speed;hit.radialWeight=hit.randomWeight=hit.upwardWeight=0;hit.directionalWeight=1;hit.directionWorld={1,0,0};motion.ApplyImpact(hit);
            MotionStepInput in;uint32_t queries=0;GroundPlaneQuery ground=[&](uint32_t,const Vec3&,PlaneCollider& p){++queries;p={};p.friction=0;return true;};
            for(int i=0;i<frames;++i)motion.Step(1.0/60,in,ground);
            return std::make_pair(motion.States()[0],queries);
        };
        const auto on=roll(30,1,true,.35,10,1),off=roll(30,1,false,.35,10,1),small=roll(30,.5,true,.35,10,1);
        Check(on.first.rotationWorld.y>0,"rolling direction is ground normal cross velocity");
        Check(Length(on.first.angularVelocityWorld)==0,"visual roll does not alter angular velocity");
        Check(Length(on.first.velocityWorld-off.first.velocityWorld)<1e-10&&Length(on.first.positionWorld-off.first.positionWorld)<1e-10,"roll does not affect translation or impact");
        Check(std::abs(on.first.rotationWorld.y-small.first.rotationWorld.y)<1e-12,"footprint scale does not alter roll rate");
        Check(on.second==off.second,"rolling adds no terrain queries");
        Check(roll(4,1,true,.35,10,1).first.rotationWorld.y==0,"low speed has no roll");
        Check(roll(30,1,true,0,10,1).first.rotationWorld.y==0,"zero strength disables roll");
        Check(roll(30,1,true,.35,1000,1).first.rotationWorld.y==0,"airborne has no added roll");
        Check(roll(60,1,true,.35,10,1).first.rotationWorld.y>on.first.rotationWorld.y,"faster movement rolls more");
        const auto stopped=roll(0,1,true,.35,10,120);Check(stopped.first.sleeping&&stopped.first.rotationWorld.y==0,"sleeping remains still");
    }
    {
        auto run=[](int count,int fps){
            CollisionScene xy;xy.settings.simpleVisual=true;xy.settings.groundOnly=true;xy.bodies.resize(count);for(auto& b:xy.bodies)b.radius=1;
            std::vector<ChunkMotionDesc> desc(count);for(int i=0;i<count;++i){desc[i].id=i;desc[i].restCentroid={double(i%16)*1.5,double(i/16)*1.5,8};desc[i].radius=1;desc[i].mass=1;}
            MotionSystem motion;MotionSettings m;Check(motion.Initialize(m,desc,{}),"many ground footprints initialize");motion.SetCollisionScene(&xy);
            ImpactEvent hit;hit.eventId=1;hit.pointWorld={0,0,8};hit.radius=1000;hit.impulse=0;motion.ApplyImpact(hit);
            MotionStepInput in;GroundPlaneQuery ground=[](uint32_t,const Vec3&,PlaneCollider& p){p={};return true;};
            for(int frame=0;frame<fps*2;++frame){motion.Step(1.0/fps,in,ground);Check(!xy.statistics.paused,"ground crowd never pauses");Check(xy.statistics.groundQueries<=uint64_t(count*2),"ground work bounded by chunks not vertices");Check(xy.statistics.shapePairs==0,"no SAT shape checks in ground crowd");}
            return motion.States();
        };
        const auto a=run(24,30),b=run(24,60),c=run(24,120);
        for(size_t i=0;i<a.size();++i){Check(Length(a[i].positionWorld-b[i].positionWorld)<1e-8,"XY 30/60 fixed-step agreement");Check(Length(c[i].positionWorld-b[i].positionWorld)<1e-8,"XY 120/60 fixed-step agreement");}
        const auto crowd=run(183,60);for(const auto& s:crowd)Check(Finite(s.positionWorld)&&s.positionWorld.z>=.99,"183 footprints stay finite above Landscape");
    }
    {
        // New ground-only path: no hulls, no SAT, and no occupancy while airborne.
        CollisionScene xy;xy.settings.simpleVisual=true;xy.settings.groundOnly=true;
        CollisionBody footprint;footprint.radius=1;xy.bodies={footprint,footprint};
        std::string error;Check(xy.Validate(error),"XY mode needs no convex proxies");
        std::vector<ChunkMotionState> states(2);for(auto& s:states){s.detached=true;s.sleeping=false;s.positionWorld={0,0,10};}
        std::vector<uint8_t> moved;xy.AvoidOnGround(1.0/60,{0,0},states,moved);
        Check(states[0].positionWorld.x==0&&states[1].positionWorld.x==0&&xy.statistics.pairs==0,"airborne siblings do not collide");
        states[1].sleeping=true;xy.AvoidOnGround(1.0/60,{1,1},states,moved);
        Check(!states[1].sleeping&&moved[1],"ground neighbour wakes sleeping footprint");
        Check(Length(states[0].velocityWorld)+Length(states[1].velocityWorld)==0,"XY correction injects no speed");
        for(int i=0;i<10;++i)xy.AvoidOnGround(1.0/60,{1,1},states,moved);
        Check(std::abs(states[0].positionWorld.x-states[1].positionWorld.x)>1.999,"ground circles separate");
        xy.settings.maxPairs=0;states[1].positionWorld=states[0].positionWorld;xy.AvoidOnGround(1.0/60,{1,1},states,moved);
        Check(!xy.statistics.budgetExceeded&&!xy.statistics.paused&&xy.statistics.pairs>0,"XY ignores legacy pair budget");
        xy.Reset();xy.bodies.resize(1);states.resize(1);states[0].positionWorld={0,0,1};xy.settings.selfCollision=false;
        StaticCollisionBody wall;wall.body=Box({2,2,2});xy.environment={wall};
        xy.AvoidOnGround(1.0/60,{0},states,moved);Check(states[0].positionWorld.x==0,"external Box ignored in air");
        for(int i=0;i<10;++i)xy.AvoidOnGround(1.0/60,{1},states,moved);
        Check(states[0].positionWorld.x<=-3&&states[0].positionWorld.z==1,"external rectangle pushes XY without lifting onto Box");
        xy.Reset();states[0].positionWorld={0,0,20};xy.AvoidOnGround(1.0/60,{1},states,moved);
        Check(states[0].positionWorld.x==0,"external height interval excludes overhead body");
    }
    {
        CollisionScene xy;xy.settings.simpleVisual=true;xy.settings.groundOnly=true;
        CollisionBody body;body.radius=1;xy.bodies={body,body};
        std::vector<ChunkMotionDesc> chunks(2);for(auto& c:chunks){c.restCentroid={0,0,10};c.radius=1;c.mass=1;}
        MotionSystem motion;MotionSettings m;Check(motion.Initialize(m,chunks,{}),"ground XY initialize");motion.SetCollisionScene(&xy);
        ImpactEvent impact;impact.eventId=1;impact.pointWorld={0,0,10};impact.radius=100;impact.impulse=0;
        Check(motion.ApplyImpact(impact),"ground XY release");
        MotionStepInput in;uint64_t queries=0;GroundPlaneQuery ground=[&](uint32_t,const Vec3&,PlaneCollider& p){++queries;p={};return true;};
        motion.Step(1.0/60,in,ground);Check(motion.States()[0].positionWorld.x==0,"MotionSystem does not avoid before landing");
        for(int i=0;i<180;++i)motion.Step(1.0/60,in,ground);
        Check(std::abs(motion.States()[0].positionWorld.x-motion.States()[1].positionWorld.x)>1.99,"MotionSystem spreads chunks after landing");
        Check(motion.States()[0].sleeping&&motion.States()[1].sleeping,"ground XY settles to sleep");
        Check(motion.States()[0].rotationWorld.w==1&&motion.States()[1].rotationWorld.w==1,"pure depenetration does not drive visual rolling");
        uint64_t before=queries;motion.Step(1.0/60,in,ground);Check(queries==before,"sleeping chunks skip ground queries");
        xy.settings.maxPairs=0;motion.Initialize(m,chunks,{});xy.Reset();impact.eventId=2;motion.ApplyImpact(impact);motion.Step(1.0/60,in,ground);
        Check(motion.States()[0].positionWorld.z<10&&!xy.statistics.paused,"low avoidance budget cannot freeze falling motion");
        xy.settings.maxPairs=100000;xy.Reset();for(auto& d:chunks)d.restCentroid={0,0,1};motion.Initialize(m,chunks,{});impact.eventId=3;impact.pointWorld={0,0,1};motion.ApplyImpact(impact);
        GroundPlaneQuery limited=[](uint32_t,const Vec3& p,PlaneCollider& plane){plane={};return std::abs(p.x)<.5;};
        motion.Step(1.0/60,in,limited);motion.Step(1.0/60,in,limited);
        Check(motion.States()[0].positionWorld.z<1&&motion.States()[1].positionWorld.z<1,"XY movement outside capture resumes free fall");
    }
    {
        for(bool visual:{false,true}){
            CollisionScene c;c.settings.simpleVisual=visual;c.bodies={Box({.01,1,1})};
            std::vector<ChunkMotionState> s(1);s[0].detached=true;s[0].sleeping=false;s[0].velocityWorld={100000,0,0};s[0].angularVelocityWorld={0,0,100};
            MotionSettings m;m.linearDrag=m.angularDrag=0;MotionStepInput in;in.gravityWorld={};
            Check(c.Advance(1.0/60,m,in,{},s),"fast thin body no longer pauses");
            Check(c.statistics.substeps==8&&!c.statistics.paused,"fast body uses bounded substeps");
            Check(std::abs(s[0].positionWorld.x-100000.0/60)<1e-6,"fast motion not clamped");
            Check(std::abs(s[0].velocityWorld.x-100000)<1e-5,"linear speed preserved");
            Check(std::abs(s[0].angularVelocityWorld.z-100)<1e-5,"angular speed preserved");
            c.Reset();s[0].velocityWorld.x=std::numeric_limits<double>::quiet_NaN();
            Check(!c.Advance(1.0/60,m,in,{},s),"nonfinite motion still rejected");
        }
        CollisionScene c;c.settings.simpleVisual=true;std::vector<ChunkMotionState> s(183);
        for(int i=0;i<183;++i){c.bodies.push_back(Box({.1,1,1}));s[i].positionWorld={0,double(i)*10,0};s[i].detached=true;s[i].sleeping=false;s[i].velocityWorld={100000,0,0};}
        MotionSettings m;m.linearDrag=m.angularDrag=0;MotionStepInput in;in.gravityWorld={};
        Check(c.Advance(1.0/60,m,in,{},s),"183 fast bodies advance");
        for(const auto& b:s)Check(b.positionWorld.x>1600,"183 bodies retain fast displacement");
    }
    {
        CollisionScene visual;visual.settings.simpleVisual=true;
        visual.bodies={Box({5,5,5}),Box({5,5,5})};
        std::vector<ChunkMotionState> s(2);s[0].detached=s[1].detached=true;s[0].sleeping=s[1].sleeping=false;
        s[1].positionWorld={7,0,0};MotionSettings motion;MotionStepInput in;in.gravityWorld={};
        for(int i=0;i<120;++i){Check(visual.Advance(1.0/60,motion,in,{},s),"visual overlap does not block preview");Check(Length(s[0].velocityWorld)+Length(s[1].velocityWorld)<.01,"visual overlap correction does not launch siblings");}
        Check(s[1].positionWorld.x-s[0].positionWorld.x>9.7,"visual siblings still separate");
        visual.Reset();s[0].sleeping=s[1].sleeping=false;s[0].velocityWorld={5,0,0};
        Check(visual.Advance(1.0/60,motion,in,{},s),"visual siblings continue colliding after stabilization");
        visual.settings.maxPairs=0;visual.Reset();s[1].positionWorld=s[0].positionWorld;
        Check(!visual.Advance(1.0/60,motion,in,{},s),"visual mode retains budget guard");
    }
    {
        CollisionScene c;c.settings.simpleVisual=true;c.bodies={Box({5,5,5}),Box({5,5,5}),Box({5,5,5})};
        for(auto& b:c.bodies){auto& hull=b.hulls[0];auto faces=hull.faces;hull.faces.clear();for(const auto& f:faces)for(size_t i=1;i+1<f.size();++i)hull.faces.push_back({f[0],f[i],f[i+1]});}
        StaticCollisionBody floor;floor.body=Box({50,50,5});floor.position={0,0,-5};c.environment={floor};
        std::vector<ChunkMotionState> state(3);for(int i=0;i<3;++i){state[i].positionWorld={0,0,5.0+10*i};state[i].detached=true;state[i].sleeping=false;}
        MotionSettings m;MotionStepInput in;
        for(int frame=0;frame<300;++frame)Check(c.Advance(1.0/60,m,in,{},state),"visual triangulated three layer stack advances");
        for(auto& s:state){Check(Finite(s.positionWorld)&&Length(s.velocityWorld)<10,"visual stack stays bounded");Check(s.positionWorld.z>4.0,"visual stack does not fall through floor");}
    }
    {
        // Exercise the SAME conversion used by GeometryCore output in Editor:
        // a nearly collinear tessellation triangle has an unreliable normal.
        for(double scale:{.1,1.0,100.0}){
            auto box=MakeCollisionBox({scale,scale,scale});
            std::vector<std::array<uint32_t,3>> triangles;
            for(const auto& f:box.faces)for(size_t i=1;i+1<f.size();++i)triangles.push_back({f[0],f[i],f[i+1]});
            auto t=triangles[0];const uint32_t mid=uint32_t(box.vertices.size());
            box.vertices.push_back((box.vertices[t[0]]+box.vertices[t[1]])*.5+Vec3{scale*1e-15,0,scale*1e-15});
            if(scale==1.0){
                const Vec3 raw=Cross(box.vertices[t[1]]-box.vertices[t[0]],box.vertices[mid]-box.vertices[t[0]]);
                Check(Length(raw)*.5<1e-12,"fixture reproduces old whole-boundary area rejection");
                Check(std::abs(Dot(raw/Length(raw),Vec3{0,0,-1}))<1-1e-10,"fixture reproduces old normal-group mismatch");
            }
            triangles[0]={t[0],t[1],mid};triangles.push_back({t[1],t[2],mid});triangles.push_back({t[2],t[0],mid});
            CollisionHull result;std::string error;
            Check(BuildCollisionHullFromTriangles(box.vertices,triangles,result,error),"sliver hull conversion");
            Check(result.faces.size()==6&&result.vertices.size()==8,"sliver absorbed into supporting face not deleted as hole");
            Check(ValidateCollisionHull(result,error),"converted sliver hull remains closed");
            std::reverse(triangles.begin(),triangles.end());CollisionHull repeat;
            Check(BuildCollisionHullFromTriangles(box.vertices,triangles,repeat,error),"reversed triangle order conversion");
            Check(repeat.vertices.size()==8&&repeat.faces.size()==6,"triangle ordering cannot seed tiny planes");
        }
        auto box=MakeCollisionBox({1,1,1});std::vector<std::array<uint32_t,3>> open;
        for(size_t f=1;f<box.faces.size();++f)for(size_t i=1;i+1<box.faces[f].size();++i)open.push_back({box.faces[f][0],box.faces[f][i],box.faces[f][i+1]});
        CollisionHull out;std::string error;
        Check(!BuildCollisionHullFromTriangles(box.vertices,open,out,error),"converter does not hide missing face");
        Check(out.vertices.empty(),"conversion failure is transactional");
        box.faces[0]={0,0,0};Check(!ValidateCollisionHull(box,error),"actual zero area still rejected");
        Check(error.find("areaCm2=")!=std::string::npos&&error.find("face=0")!=std::string::npos,"area failure reports measured geometry");
    }
    {
        // A valid box face may begin with three collinear boundary vertices.
        auto h=MakeCollisionBox({1,1,1});h.vertices.push_back({-1,0,-1});
        h.faces[0]={0,8,2,3,1};h.faces[4]={0,4,6,2,8};
        std::string error;Check(ValidateCollisionHull(h,error),"first three collinear is not zero area");
        Check(std::abs(Length(CollisionFaceAreaVector(h,h.faces[0]))-4)<1e-12,"whole polygon area");
        auto contacts=CollideConvex(h,Shift(MakeCollisionBox({1,1,1}),{0,0,-1.5}));
        Check(contacts.size()==4,"collinear face still has four contacts");
        std::vector<Vec3> points={{-1,-1,0},{0,-1,0},{1,-1,0},{1,1,0},{-1,1,0},{0,0,0},{-1,-1,0}};
        auto boundary=BuildCollisionFaceBoundary(points,{0,1,2,3,4,5,6},{0,0,1});
        Check(boundary.size()==4,"boundary removes duplicates interior and collinear points");
        auto reverse=BuildCollisionFaceBoundary(points,{6,5,4,3,2,1,0},{0,0,1});
        Check(boundary==reverse,"boundary independent of candidate order");
        CollisionHull face;face.vertices=points;
        Check(CollisionFaceAreaVector(face,boundary).z>0,"boundary follows outward normal");
        Check(BuildCollisionFaceBoundary({{0,0,0},{1,0,0},{2,0,0}},{0,1,2},{0,0,1}).empty(),"truly zero area rejected");
    }
    {
        CollisionScene stack;stack.bodies={Box({5,5,5}),Box({5,5,5}),Box({5,5,5})};
        StaticCollisionBody floor;floor.body=Box({50,50,5});floor.position={0,0,-5};stack.environment={floor};
        std::vector<ChunkMotionState> state(3);for(int i=0;i<3;++i){state[i].positionWorld={0,0,5.0+10*i};state[i].detached=true;state[i].sleeping=false;}
        MotionSettings m;MotionStepInput in;
        for(int frame=0;frame<300;++frame){bool ok=stack.Advance(1.0/60,m,in,{},state);if(!ok){std::cerr<<"stack frame "<<frame<<" "<<stack.statistics.reason<<"\n";for(const auto& s:state)std::cerr<<s.positionWorld.x<<","<<s.positionWorld.y<<","<<s.positionWorld.z<<" v="<<Length(s.velocityWorld)<<" w="<<Length(s.angularVelocityWorld)<<"\n";}Check(ok,"three layer stack advance");}
        for(int i=0;i<3;++i){Check(std::abs(state[i].positionWorld.z-(5+10*i))<.2,"three layer stack residual");Check(Length(state[i].velocityWorld)<1,"three layer stack settled");}
        Check(stack.statistics.maxPenetration<=.2,"stack penetration budget");
    }
    {
        CollisionScene c;c.bodies={Box({5,5,5})};std::vector<ChunkMotionState>s(1);s[0].detached=true;s[0].sleeping=false;s[0].positionWorld={0,0,4.9};
        MotionSettings m;MotionStepInput in;in.gravityWorld={};in.planes.push_back({});Check(c.Advance(1.0/60,m,in,{},s),"prestabilize advance");Check(Length(s[0].velocityWorld)<.01,"prestabilize no launch");
        in.gravityWorld={0,0,-980};s[0].velocityWorld={10,0,0};for(int i=0;i<120;++i)Check(c.Advance(1.0/60,m,in,{},s),"friction advance");Check(std::abs(s[0].velocityWorld.x)<.1,"friction stops sliding");Check(s[0].sleeping,"contact sleeps");
    }
    auto a=MakeCollisionBox({1,1,1});std::string error;Check(ValidateCollisionHull(a,error),"closed box");Check(CollideConvex(a,Shift(a,{3,0,0})).empty(),"SAT separated");auto face=CollideConvex(a,Shift(a,{0,0,1.5}));Check(face.size()==4,"four face contacts");Check(std::abs(face[0].gap+.5)<1e-8,"face depth");auto contained=CollideConvex(a,MakeCollisionBox({5,5,5}));Check(!contained.empty()&&std::abs(contained[0].gap+6)<1e-8,"SAT containment depth");auto b=Box({1,2,3},2);Check(std::abs(b.inverseInertia.m[0][0]-3.0/26)<1e-10,"integrated inertia analytic");
    auto d30=Drop(24,30),d60=Drop(24,60),d120=Drop(24,120);for(size_t i=0;i<d30.size();++i){Check(Length(d30[i].positionWorld-d60[i].positionWorld)<1e-9,"30/60 deterministic");Check(Length(d120[i].positionWorld-d60[i].positionWorld)<1e-9,"120/60 deterministic");}Drop(64,60);
    CollisionScene scene;scene.bodies={Box({1,1,1}),Box({1,1,1},2)};std::vector<ChunkMotionState>s(2);s[0].detached=s[1].detached=true;s[0].sleeping=s[1].sleeping=false;s[0].positionWorld={0,0,0};s[1].positionWorld={2,0,0};MotionSettings m;MotionStepInput input;input.gravityWorld={};Check(scene.Advance(1.0/60,m,input,{},s),"adjacent blocks");Check(Length(s[0].velocityWorld)+Length(s[1].velocityWorld)<1e-8,"rest adjacency no explosion");
    scene.settings.maxPairs=0;s[1].positionWorld={1.9,0,0};auto before=s;Check(!scene.Advance(1.0/60,m,input,{},s),"budget pauses");Check(Length(s[0].positionWorld-before[0].positionWorld)==0,"budget transactional");
    {
        CollisionScene c;c.bodies={Box({5,5,5},1),Box({5,5,5},2)};
        for(auto& body:c.bodies){body.friction=0;body.restitution=0;}
        MotionSettings m;m.linearDrag=m.angularDrag=0;MotionStepInput in;in.gravityWorld={};
        std::vector<ChunkMotionState>s(2);s[0].detached=s[1].detached=true;s[0].sleeping=false;s[1].sleeping=true;
        s[0].positionWorld={0,0,0};s[1].positionWorld={10,0,0};s[0].velocityWorld={10,0,0};
        Check(c.Advance(1.0/60,m,in,{},s),"unequal mass impact");
        Check(!s[1].sleeping&&s[1].velocityWorld.x>0,"sleeping sibling wakes");
        Check(std::abs(s[0].velocityWorld.x+2*s[1].velocityWorld.x-10)<1e-6,"mass weighted momentum");
        Check(.5*LengthSquared(s[0].velocityWorld)+LengthSquared(s[1].velocityWorld)<=50.1,"inelastic impact no energy gain");
        c.Reset();s={ChunkMotionState{},ChunkMotionState{}};s[0].positionWorld={0,0,0};s[1].positionWorld={10,0,0};s[0].detached=true;s[0].sleeping=false;s[0].velocityWorld={10,0,0};
        Check(c.Advance(1.0/60,m,in,{},s),"support impact");Check(Length(s[1].positionWorld-Vec3{10,0,0})==0,"undetched support stays static");
        c.Reset();c.settings.selfCollision=false;s[0].positionWorld={0,0,0};s[0].velocityWorld={10,0,0};
        Check(c.Advance(1.0/60,m,in,{},s),"self collision opt out");Check(s[0].positionWorld.x>.1,"disabled self bypasses siblings");
    }
    {
        CollisionScene c;c.bodies={Box({5,5,5})};MotionSettings m;MotionStepInput in;in.gravityWorld={};
        std::vector<ChunkMotionState>s(1);s[0].detached=true;s[0].sleeping=false;s[0].positionWorld={0,0,7};s[0].rotationWorld=QuatFromRotationVector({0,.4,0});s[0].velocityWorld={0,0,-10};in.planes.push_back({});
        for(int i=0;i<20;++i)Check(c.Advance(1.0/60,m,in,{},s),"rotated corner advance");Check(Length(s[0].angularVelocityWorld)>.001,"corner contact creates rotation");
        auto h=MakeCollisionBox({4,1,1});for(auto& p:h.vertices)p=Rotate(QuatFromRotationVector({.4,.6,.2}),p)+Vec3{0,0,1};auto other=MakeCollisionBox({1,4,1});auto ab=CollideConvex(h,other),ba=CollideConvex(other,h);Check(!ab.empty()&&!ba.empty(),"rotated edge contacts");Check(Dot(ab[0].normal,ba[0].normal)<-.99,"edge A/B normals reverse");
    }
    {
        CollisionScene c;c.bodies={Box({5,5,5})};std::vector<ChunkMotionState>s(1);s[0].detached=true;s[0].sleeping=false;s[0].positionWorld={0,0,5};s[0].velocityWorld={0,0,-10};MotionSettings m;MotionStepInput in;in.gravityWorld={};PlaneCollider floor;floor.restitution=1;floor.restitutionVelocityThreshold=20;in.planes={floor};
        Check(c.Advance(1.0/60,m,in,{},s),"low speed restitution step");Check(s[0].velocityWorld.z<.1,"low speed restitution suppressed");
        c.Reset();s[0].velocityWorld={0,0,-100};s[0].positionWorld={0,0,5};Check(c.Advance(1.0/60,m,in,{},s),"high speed restitution step");Check(s[0].velocityWorld.z>50,"one fixed bounce target");
        c.Reset();c.settings.maxGroundQueries=0;GroundPlaneQuery query=[](uint32_t,const Vec3&,PlaneCollider& p){p={};return true;};Check(!c.Advance(1.0/60,m,in,query,s),"ground query budget pauses");
    }
    std::cout<<"PASS "<<checks<<" checks\n";return 0;
}catch(const std::exception& e){std::cerr<<"FAIL after "<<checks<<": "<<e.what()<<"\n";return 1;}}
