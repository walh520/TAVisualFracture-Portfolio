#include "TVFCollision.h"
#include <map>
#include <set>
#include <limits>
#include <numeric>
#include <sstream>
#include <iomanip>

namespace tvf {
Vec3 CollisionFaceAreaVector(const CollisionHull& h,const std::vector<uint32_t>& f){
    if(f.size()<3)return {};for(auto i:f)if(i>=h.vertices.size())return {};
    Vec3 area{},origin=h.vertices[f[0]];
    for(size_t i=1;i+1<f.size();++i)area+=Cross(h.vertices[f[i]]-origin,h.vertices[f[i+1]]-origin);
    return area*.5;
}
std::vector<uint32_t> BuildCollisionFaceBoundary(const std::vector<Vec3>& points,const std::vector<uint32_t>& candidates,Vec3 normal){
    if(candidates.size()<3||!Finite(normal)||Length(normal)<=0)return {};
    for(auto i:candidates)if(i>=points.size()||!Finite(points[i]))return {};
    normal=Normalize(normal);const Vec3 origin=points[candidates[0]];
    const Vec3 u=Normalize(Cross(normal,std::abs(normal.x)<.7?Vec3{1,0,0}:Vec3{0,1,0})),v=Cross(normal,u);
    struct Point{double x,y;uint32_t id;};std::vector<Point> projected;
    for(auto i:candidates){Vec3 p=points[i]-origin;projected.push_back({Dot(p,u),Dot(p,v),i});}
    std::sort(projected.begin(),projected.end(),[](const Point&a,const Point&b){return a.x!=b.x?a.x<b.x:(a.y!=b.y?a.y<b.y:a.id<b.id);});
    projected.erase(std::unique(projected.begin(),projected.end(),[](const Point&a,const Point&b){return a.x==b.x&&a.y==b.y;}),projected.end());
    if(projected.size()<3)return {};
    auto turn=[](const Point&a,const Point&b,const Point&c){return (static_cast<long double>(b.x)-a.x)*(static_cast<long double>(c.y)-a.y)-(static_cast<long double>(b.y)-a.y)*(static_cast<long double>(c.x)-a.x);};
    // Monotone-chain convex boundary, not angular sorting of all face points:
    // interior tessellation vertices and collinear edge samples do not belong
    // to the simplified polygon boundary.
    std::vector<Point> boundary;
    for(const auto& p:projected){while(boundary.size()>=2&&turn(boundary[boundary.size()-2],boundary.back(),p)<=0)boundary.pop_back();boundary.push_back(p);}
    const size_t lower=boundary.size();
    for(size_t i=projected.size()-1;i-->0;){const auto& p=projected[i];while(boundary.size()>lower&&turn(boundary[boundary.size()-2],boundary.back(),p)<=0)boundary.pop_back();boundary.push_back(p);}
    boundary.pop_back();if(boundary.size()<3)return {};
    std::vector<uint32_t> result;for(const auto& p:boundary)result.push_back(p.id);return result;
}
bool BuildCollisionHullFromTriangles(const std::vector<Vec3>& points,const std::vector<std::array<uint32_t,3>>& triangles,CollisionHull& out,std::string& error){
    out={};error.clear();
    std::set<uint32_t> used;
    for(const auto& t:triangles)for(auto i:t){if(i>=points.size()||!Finite(points[i])){error="Hull conversion: invalid source vertex";return false;}used.insert(i);}
    if(used.size()<4){error="Hull conversion: fewer than four used vertices";return false;}
    Vec3 lo=points[*used.begin()],hi=lo;
    for(auto i:used){auto p=points[i];lo={std::min(lo.x,p.x),std::min(lo.y,p.y),std::min(lo.z,p.z)};hi={std::max(hi.x,p.x),std::max(hi.y,p.y),std::max(hi.z,p.z)};}
    const Vec3 origin=lo+(hi-lo)*.5;const double scale=Length(hi-lo);
    if(!(scale>0)||!std::isfinite(scale)){error="Hull conversion: invalid extent";return false;}
    CollisionHull normalized;std::map<uint32_t,uint32_t> remap;Vec3 center{};
    for(auto i:used){remap[i]=uint32_t(normalized.vertices.size());normalized.vertices.push_back((points[i]-origin)/scale);center+=normalized.vertices.back();}
    center=center/double(used.size());
    struct Triangle{std::array<uint32_t,3> ids;Vec3 n;double area;};std::vector<Triangle> sorted;
    for(const auto& t:triangles){std::array<uint32_t,3> ids={remap[t[0]],remap[t[1]],remap[t[2]]};
        auto a=normalized.vertices[ids[0]];auto raw=Cross(normalized.vertices[ids[1]]-a,normalized.vertices[ids[2]]-a);double area=Length(raw);
        if(!std::isfinite(area)){error="Hull conversion: nonfinite triangle area";return false;}
        if(Dot(raw,center-a)>0)raw=-raw;
        sorted.push_back({ids,area>0?raw/area:Vec3{},area});
    }
    // Establish reliable supporting planes from large triangles FIRST. A near-
    // collinear sliver normal is ill-conditioned and must not create its own face.
    std::stable_sort(sorted.begin(),sorted.end(),[](const Triangle&a,const Triangle&b){return a.area>b.area;});
    struct Plane{Vec3 n,point;std::set<uint32_t> ids;};std::vector<Plane> planes;
    constexpr double planeTolerance=1e-10; // normalized domain; numerical cleanup only
    for(const auto& t:sorted){size_t match=planes.size();double best=planeTolerance;
        for(size_t j=0;j<planes.size();++j){double distance=0;for(auto i:t.ids)distance=std::max(distance,std::abs(Dot(planes[j].n,normalized.vertices[i]-planes[j].point)));
            if(distance<=best){best=distance;match=j;}}
        if(match==planes.size()){
            if(t.area<=1e-14){error="Hull conversion: unresolved sliver; rebuild convex hull";return false;}
            planes.push_back({t.n,normalized.vertices[t.ids[0]],{}});
        }
        planes[match].ids.insert(t.ids.begin(),t.ids.end());
    }
    for(const auto& p:planes){auto face=BuildCollisionFaceBoundary(normalized.vertices,{p.ids.begin(),p.ids.end()},p.n);if(face.size()<3){error="Hull conversion: collapsed plane boundary";return false;}normalized.faces.push_back(std::move(face));}
    // Global compaction preserves identical vertex IDs on adjacent faces.
    std::set<uint32_t> boundary;for(const auto& f:normalized.faces)boundary.insert(f.begin(),f.end());
    std::map<uint32_t,uint32_t> compact;CollisionHull candidate;
    std::vector<uint32_t> sourceIds(used.begin(),used.end());
    for(auto i:boundary){compact[i]=uint32_t(candidate.vertices.size());candidate.vertices.push_back(points[sourceIds[i]]);}
    for(auto f:normalized.faces){for(auto& i:f)i=compact[i];candidate.faces.push_back(std::move(f));}
    if(!ValidateCollisionHull(candidate,error)){error="Hull conversion: "+error;return false;}
    out=std::move(candidate);return true;
}
Vec3 CollisionMatrix::Apply(Vec3 v)const{return {m[0][0]*v.x+m[0][1]*v.y+m[0][2]*v.z,m[1][0]*v.x+m[1][1]*v.y+m[1][2]*v.z,m[2][0]*v.x+m[2][1]*v.y+m[2][2]*v.z};}
namespace {
Vec3 Normal(const CollisionHull& h,size_t f){Vec3 area=CollisionFaceAreaVector(h,h.faces[f]);double length=Length(area);return length>0?area/length:Vec3{};}
using Edge=std::pair<uint32_t,uint32_t>;
std::vector<Edge> Edges(const CollisionHull& h){std::set<Edge> out;for(const auto& f:h.faces)for(size_t i=0;i<f.size();++i)out.insert(std::minmax(f[i],f[(i+1)%f.size()]));return {out.begin(),out.end()};}
void Project(const CollisionHull& h,Vec3 n,double& lo,double& hi){lo=1e300;hi=-lo;for(Vec3 v:h.vertices){double d=Dot(v,n);lo=std::min(lo,d);hi=std::max(hi,d);}}
CollisionHull WorldHull(const CollisionHull& h,Vec3 x,Quat q){CollisionHull out=h;for(Vec3& p:out.vertices)p=x+Rotate(q,p);return out;}
std::vector<Vec3> ClipPolygon(std::vector<Vec3> p,Vec3 n,double d){std::vector<Vec3> out;if(p.empty())return out;Vec3 a=p.back();double fa=Dot(a,n)-d;for(Vec3 b:p){double fb=Dot(b,n)-d;if((fa<0&&fb>0)||(fa>0&&fb<0))out.push_back(a+(b-a)*(fa/(fa-fb)));if(fb<=0)out.push_back(b);a=b;fa=fb;}return out;}
void SegmentClosest(Vec3 a,Vec3 b,Vec3 c,Vec3 d,Vec3& pa,Vec3& pb){Vec3 u=b-a,v=d-c,r=a-c;double aa=Dot(u,u),bb=Dot(u,v),cc=Dot(v,v),dd=Dot(u,r),ee=Dot(v,r);double denom=aa*cc-bb*bb;double s=denom>1e-20?std::clamp((bb*ee-cc*dd)/denom,0.0,1.0):0;double t=cc>1e-20?(bb*s+ee)/cc:0;if(t<0){t=0;s=aa>0?std::clamp(-dd/aa,0.0,1.0):0;}if(t>1){t=1;s=aa>0?std::clamp((bb-dd)/aa,0.0,1.0):0;}pa=a+u*s;pb=c+v*t;}
void Reduce(std::vector<CollisionContact>& c,size_t limit=4){
    std::vector<CollisionContact> unique;for(const auto& x:c){bool duplicate=false;for(const auto& y:unique)if(LengthSquared(x.localA-y.localA)<1e-12){duplicate=true;break;}if(!duplicate)unique.push_back(x);}c=std::move(unique);
    if(c.size()<=limit)return;std::vector<CollisionContact> out;
    auto first=std::min_element(c.begin(),c.end(),[](const auto&a,const auto&b){return a.gap<b.gap;});out.push_back(*first);c.erase(first);
    while(out.size()<limit){size_t best=0;double distance=-1;for(size_t i=0;i<c.size();++i){double nearest=1e300;for(const auto& p:out)nearest=std::min(nearest,LengthSquared(c[i].localA-p.localA));if(nearest>distance){distance=nearest;best=i;}}out.push_back(c[best]);c.erase(c.begin()+best);}c=std::move(out);
}
}
CollisionHull MakeCollisionBox(Vec3 e){CollisionHull h;for(int z=-1;z<=1;z+=2)for(int y=-1;y<=1;y+=2)for(int x=-1;x<=1;x+=2)h.vertices.push_back({x*e.x,y*e.y,z*e.z});h.faces={{0,2,3,1},{4,5,7,6},{0,1,5,4},{2,6,7,3},{0,4,6,2},{1,3,7,5}};return h;}
bool ValidateCollisionHull(const CollisionHull& h,std::string& error){
    if(h.vertices.size()<4||h.faces.size()<4){error="Empty/degenerate convex proxy";return false;}
    std::map<Edge,int> edges;for(Vec3 v:h.vertices)if(!Finite(v)){error="Nonfinite proxy vertex";return false;}
    for(size_t fi=0;fi<h.faces.size();++fi){const auto& f=h.faces[fi];if(f.size()<3){error="Degenerate proxy face";return false;}for(uint32_t i:f)if(i>=h.vertices.size()){error="Invalid proxy index";return false;}
        Vec3 raw=CollisionFaceAreaVector(h,f);double area=Length(raw);if(!std::isfinite(area)||area<1e-12){std::ostringstream detail;detail<<"Zero-area proxy polygon: face="<<fi<<", vertices="<<f.size()<<", areaCm2="<<std::scientific<<std::setprecision(8)<<area;error=detail.str();return false;}Vec3 n=raw/area;for(Vec3 v:h.vertices)if(Dot(n,v-h.vertices[f[0]])>1e-6){error="Proxy is not outward convex";return false;}for(size_t j=0;j<f.size();++j)++edges[{f[j],f[(j+1)%f.size()]}];}
    for(const auto& e:edges)if(e.second!=1||edges[{e.first.second,e.first.first}]!=1){error="Proxy has open/nonmanifold edge";return false;}return true;
}
bool ComputeCollisionInertia(const std::vector<std::array<Vec3,3>>& triangles,double mass,CollisionMatrix& inverse,std::string& error){
    double volume=0;CollisionMatrix second;Vec3 first{};
    for(const auto& t:triangles){double v=Dot(t[0],Cross(t[1],t[2]))/6;volume+=v;Vec3 sum=t[0]+t[1]+t[2];first+=sum*(v/4);double s[3]={sum.x,sum.y,sum.z};for(int i=0;i<3;++i)for(int j=0;j<3;++j){double diag=0;for(Vec3 p:t){double a[3]={p.x,p.y,p.z};diag+=a[i]*a[j];}second.m[i][j]+=v*(s[i]*s[j]+diag)/20;}}
    if(!(mass>0&&volume>1e-12)){error="Invalid oriented volume for inertia";return false;}
    // Integrate about the existing centroid origin, including all products of inertia.
    CollisionMatrix inertia;double trace=second.m[0][0]+second.m[1][1]+second.m[2][2];for(int i=0;i<3;++i)for(int j=0;j<3;++j)inertia.m[i][j]=mass/volume*((i==j?trace:0)-second.m[i][j]);
    Vec3 a{inertia.m[0][0],inertia.m[0][1],inertia.m[0][2]},b{inertia.m[1][0],inertia.m[1][1],inertia.m[1][2]},c{inertia.m[2][0],inertia.m[2][1],inertia.m[2][2]};double det=Dot(a,Cross(b,c));
    if(!(inertia.m[0][0]>0&&inertia.m[0][0]*inertia.m[1][1]-inertia.m[0][1]*inertia.m[1][0]>0&&det>0&&std::isfinite(det))){error="Inertia is not positive definite";return false;}
    Vec3 columns[3]={Cross(b,c)/det,Cross(c,a)/det,Cross(a,b)/det};for(int i=0;i<3;++i){inverse.m[0][i]=columns[i].x;inverse.m[1][i]=columns[i].y;inverse.m[2][i]=columns[i].z;}return true;
}
std::vector<CollisionContact> CollideConvex(const CollisionHull& a,const CollisionHull& b,double offset){
    if(a.vertices.empty()||b.vertices.empty())return {};
    Vec3 nBest;double best=1e300;int kind=0;size_t ia=0,ib=0;auto ea=Edges(a),eb=Edges(b);
    auto axis=[&](Vec3 axis,int k,size_t i,size_t j){double len=Length(axis);if(len<1e-10)return true;Vec3 n=axis/len;double a0,a1,b0,b1;Project(a,n,a0,a1);Project(b,n,b0,b1);double plus=b1-a0,minus=a1-b0;if(plus < -offset||minus < -offset)return false;double depth=std::min(plus,minus);if(depth<best-1e-9){best=depth;nBest=plus<=minus?n:-n;kind=k;ia=i;ib=j;}return true;};
    for(size_t i=0;i<a.faces.size();++i)if(!axis(Normal(a,i),0,i,0))return {};
    for(size_t i=0;i<b.faces.size();++i)if(!axis(Normal(b,i),1,i,0))return {};
    for(size_t i=0;i<ea.size();++i)for(size_t j=0;j<eb.size();++j)if(!axis(Cross(a.vertices[ea[i].second]-a.vertices[ea[i].first],b.vertices[eb[j].second]-b.vertices[eb[j].first]),2,i,j))return {};
    std::vector<CollisionContact> out;
    if(kind==2){
        // A SAT edge direction is not itself the supporting edge. Select the
        // parallel edges on A's minimum / B's maximum support planes first.
        Vec3 da=Normalize(a.vertices[ea[ia].second]-a.vertices[ea[ia].first]);
        Vec3 db=Normalize(b.vertices[eb[ib].second]-b.vertices[eb[ib].first]);
        double sa=1e300,sb=-1e300;
        for(size_t i=0;i<ea.size();++i){
            Vec3 p=a.vertices[ea[i].first],q=a.vertices[ea[i].second];
            if(std::abs(Dot(Normalize(q-p),da))<1-1e-8)continue;
            double d=Dot((p+q)*.5,nBest);if(d<sa){sa=d;ia=i;}
        }
        for(size_t i=0;i<eb.size();++i){
            Vec3 p=b.vertices[eb[i].first],q=b.vertices[eb[i].second];
            if(std::abs(Dot(Normalize(q-p),db))<1-1e-8)continue;
            double d=Dot((p+q)*.5,nBest);if(d>sb){sb=d;ib=i;}
        }
        Vec3 pa,pb;SegmentClosest(a.vertices[ea[ia].first],a.vertices[ea[ia].second],b.vertices[eb[ib].first],b.vertices[eb[ib].second],pa,pb);CollisionContact c;c.localA=pa;c.localB=pb;c.normal=nBest;c.gap=Dot(pa-pb,nBest);c.feature=(uint64_t(ia)<<32)|ib;out.push_back(c);return out;}
    // Re-select the actual support face; containment can reverse the chosen axis.
    bool referenceA=kind==0;const auto& ref=referenceA?a:b;const auto& inc=referenceA?b:a;Vec3 rn=referenceA?-nBest:nBest;size_t rf=0,inf=0;double rdot=-2,idot=2;
    for(size_t f=0;f<ref.faces.size();++f){double d=Dot(Normal(ref,f),rn);if(d>rdot){rdot=d;rf=f;}}
    rn=Normal(ref,rf);for(size_t f=0;f<inc.faces.size();++f){double d=Dot(Normal(inc,f),rn);if(d<idot){idot=d;inf=f;}}
    std::vector<Vec3> polygon;for(uint32_t i:inc.faces[inf])polygon.push_back(inc.vertices[i]);const auto& face=ref.faces[rf];
    for(size_t i=0;i<face.size();++i){Vec3 p=ref.vertices[face[i]],q=ref.vertices[face[(i+1)%face.size()]],side=Normalize(Cross(q-p,rn));polygon=ClipPolygon(std::move(polygon),side,Dot(side,p));}
    double d=Dot(rn,ref.vertices[face[0]]);for(Vec3 p:polygon){double gap=Dot(rn,p)-d;if(gap>offset)continue;Vec3 projected=p-rn*gap;CollisionContact c;c.normal=nBest;c.localA=referenceA?projected:p;c.localB=referenceA?p:projected;c.gap=Dot(c.localA-c.localB,nBest);c.feature=(uint64_t(rf)<<32)|inf;out.push_back(c);}Reduce(out);return out;
}

namespace {
struct WorkBody {const CollisionBody* body=nullptr;ChunkMotionState state;Vec3 oldX;Quat oldQ;bool dynamic=false;};
Vec3 InvI(const WorkBody& b,Vec3 v){return b.dynamic?Rotate(b.state.rotationWorld,b.body->inverseInertia.Apply(Rotate(Conjugate(b.state.rotationWorld),v))):Vec3{};}
double InvMass(const WorkBody& b){return b.dynamic?1/b.body->mass:0;}
Vec3 Point(const WorkBody& b,Vec3 local){return b.state.positionWorld+Rotate(b.state.rotationWorld,local);}
Vec3 Speed(const WorkBody& b,Vec3 r){return b.state.velocityWorld+Cross(b.state.angularVelocityWorld,r);}
double Effective(const WorkBody&a,const WorkBody&b,Vec3 ra,Vec3 rb,Vec3 u,Vec3 v){return (InvMass(a)+InvMass(b))*Dot(u,v)+Dot(Cross(ra,u),InvI(a,Cross(ra,v)))+Dot(Cross(rb,u),InvI(b,Cross(rb,v)));}
void Impulse(WorkBody& a,WorkBody& b,Vec3 ra,Vec3 rb,Vec3 j){if(a.dynamic){a.state.velocityWorld+=j*InvMass(a);a.state.angularVelocityWorld+=InvI(a,Cross(ra,j));}if(b.dynamic){b.state.velocityWorld-=j*InvMass(b);b.state.angularVelocityWorld-=InvI(b,Cross(rb,j));}}
void PositionImpulse(WorkBody&a,WorkBody&b,Vec3 ra,Vec3 rb,Vec3 n,double dl){if(a.dynamic){a.state.positionWorld+=n*(InvMass(a)*dl);a.state.rotationWorld=Normalize(Multiply(QuatFromRotationVector(InvI(a,Cross(ra,n))*dl),a.state.rotationWorld));}if(b.dynamic){b.state.positionWorld-=n*(InvMass(b)*dl);b.state.rotationWorld=Normalize(Multiply(QuatFromRotationVector(InvI(b,Cross(rb,n))*(-dl)),b.state.rotationWorld));}}
Vec3 AngularDelta(Quat q,Quat old,double h){Quat d=Normalize(Multiply(q,Conjugate(old)));if(d.w<0)d={-d.x,-d.y,-d.z,-d.w};Vec3 v{d.x,d.y,d.z};double l=Length(v);return l>1e-12?v*(2*std::atan2(l,d.w)/(l*h)):v*(2/h);}
}
void CollisionScene::ReplaceEnvironment(std::vector<StaticCollisionBody> updated){SampleEnvironment(std::move(updated),-1);}
void CollisionScene::SampleEnvironment(std::vector<StaticCollisionBody> updated,double time){
    std::map<std::string,PlanarBox> previous;for(const auto& box:planarBoxes_)previous.emplace(box.key,box);
    const double dt=time-environmentSampleTime_;
    std::vector<PlanarBox> next;
    for(size_t i=0;i<updated.size();++i){const auto& b=updated[i];PlanarBox box;
        box.min={1e300,1e300,1e300};box.max={-1e300,-1e300,-1e300};box.mask=b.body.mask;box.restitution=b.body.restitution;
        box.key=(b.stableId.empty()?std::to_string(i):b.stableId)+"|"+b.geometryKey+"|"+std::to_string(b.body.geometryVersion);
        bool any=false;for(const auto& hull:b.body.hulls)for(auto p:hull.vertices){p=b.position+Rotate(b.rotation,p);box.min=Min(box.min,p);box.max=Max(box.max,p);any=true;}
        if(!any)continue;box.previousMin=box.min;box.previousMax=box.max;
        auto it=previous.find(box.key);
        if(it!=previous.end()&&time>=0&&environmentSampleTime_>=0&&std::isfinite(dt)&&dt>1e-6){
            box.previousMin=it->second.min;box.previousMax=it->second.max;
            box.velocityMin=(box.min-box.previousMin)/dt;box.velocityMax=(box.max-box.previousMax)/dt;
            box.motionValid=Finite(box.velocityMin)&&Finite(box.velocityMax);
            if(!box.motionValid){box.velocityMin={};box.velocityMax={};}
        }
        if(it==previous.end()||LengthSquared(box.min-it->second.min)+LengthSquared(box.max-it->second.max)>1e-16)environmentChanged_=true;
        next.push_back(box);
    }
    environment=std::move(updated);planarBoxes_=std::move(next);planarBoxesReady_=true;environmentSampleTime_=std::isfinite(time)?time:-1;
    for(auto it=externalContacts_.begin();it!=externalContacts_.end();){
        bool exists=false;for(const auto& b:planarBoxes_)if(b.key==it->second){exists=true;break;}
        if(!exists)it=externalContacts_.erase(it);else ++it;
    }
}
void CollisionScene::AvoidOnGround(double h,const std::vector<uint8_t>& grounded,std::vector<ChunkMotionState>& states,std::vector<uint8_t>& moved){
    statistics={};statistics.substeps=1;moved.assign(states.size(),0);
    if(states.size()!=bodies.size()||grounded.size()!=states.size()||!(h>0)||!std::isfinite(h)||!(settings.xyFootprintScale>0)||!std::isfinite(settings.xyFootprintScale)){
        statistics.paused=true;statistics.reason="Invalid ground avoidance input";return;
    }
    if(states.empty())return;
    double maxRadius=0;for(size_t i=0;i<states.size();++i){
        if(!Finite(states[i].positionWorld)||!Finite(states[i].velocityWorld)||!Finite(states[i].angularVelocityWorld)||!(bodies[i].radius>0)||!std::isfinite(bodies[i].radius)){
            statistics.paused=true;statistics.reason="Nonfinite ground avoidance state";return;}
        if(!std::isfinite(bodies[i].radius*settings.xyFootprintScale)){statistics.paused=true;statistics.reason="XY footprint overflow";return;}
        maxRadius=std::max(maxRadius,bodies[i].radius*settings.xyFootprintScale);
    }
    if(!planarBoxesReady_)ReplaceEnvironment(environment);
    std::set<std::pair<size_t,std::string>> responded;
    for(auto it=externalContacts_.begin();it!=externalContacts_.end();){
        bool touching=false;if(it->first<states.size())for(const auto& box:planarBoxes_)if(box.key==it->second){
            const auto p=states[it->first].positionWorld;const double r=bodies[it->first].radius*settings.xyFootprintScale+1e-4;
            touching=p.x>=box.min.x-r&&p.x<=box.max.x+r&&p.y>=box.min.y-r&&p.y<=box.max.y+r&&p.z+bodies[it->first].radius>=box.min.z&&p.z-bodies[it->first].radius<=box.max.z;
        }
        if(!touching)it=externalContacts_.erase(it);else ++it;
    }
    // Moving a registered Box must wake only overlapping, landed chunks,
    // including when the entire pile was asleep before the editor transform.
    if(environmentChanged_&&settings.externalCollision){
        for(size_t i=0;i<states.size();++i)if(grounded[i]&&states[i].detached&&states[i].sleeping){
            const auto p=states[i].positionWorld;const double r=bodies[i].radius*settings.xyFootprintScale;
            for(const auto& box:planarBoxes_)if((bodies[i].mask&box.mask)&&p.z+bodies[i].radius>=box.min.z&&p.z-bodies[i].radius<=box.max.z&&p.x>box.min.x-r&&p.x<box.max.x+r&&p.y>box.min.y-r&&p.y<box.max.y+r){states[i].sleeping=false;states[i].sleepTime=0;break;}
        }
    }
    environmentChanged_=false;
    bool activeGround=false;for(size_t i=0;i<states.size();++i){activeGround|=grounded[i]&&states[i].detached&&!states[i].sleeping;statistics.sleeping+=states[i].sleeping;}
    if(!activeGround)return;
    statistics.sleeping=0;
    const double cellSize=std::max(2*maxRadius,1e-6);
    using Cell=std::pair<int64_t,int64_t>;
    auto cell=[&](Vec3 p){return Cell{int64_t(std::floor(p.x/cellSize)),int64_t(std::floor(p.y/cellSize))};};
    for(const auto& s:states)if(std::abs(s.positionWorld.x/cellSize)>9e18||std::abs(s.positionWorld.y/cellSize)>9e18){statistics.paused=true;statistics.reason="Ground grid coordinate overflow";return;}
    std::vector<double> remaining(states.size(),settings.avoidanceSpeed>0?settings.avoidanceSpeed*h:std::numeric_limits<double>::infinity());
    auto move=[&](size_t i,Vec3 delta){double d=Length(delta);if(d<=1e-9||remaining[i]<=1e-9)return;
        const double travel=std::min(d,remaining[i]);delta=delta*(travel/d);remaining[i]-=travel;
        states[i].positionWorld+=delta;
        states[i].sleeping=false;states[i].sleepTime=0;moved[i]=1;};

    const size_t start=avoidanceCursor_%states.size();
    for(uint32_t round=0;round<settings.avoidanceIterations;++round){
        bool corrected=false;
        std::map<Cell,std::vector<size_t>> grid;std::vector<bool> active(states.size(),false);
        for(size_t i=0;i<states.size();++i)if(grounded[i]&&states[i].detached){grid[cell(states[i].positionWorld)].push_back(i);active[i]=!states[i].sleeping;}
        for(size_t step=0;step<states.size();++step){const size_t a=(start+step)%states.size();if(!active[a])continue;
            if(settings.externalCollision)for(const auto& box:planarBoxes_){
                if(!(bodies[a].mask&box.mask))continue;
                ++statistics.shapePairs;
                const auto p=states[a].positionWorld;const double r=bodies[a].radius*settings.xyFootprintScale;
                if(p.z+bodies[a].radius<box.min.z||p.z-bodies[a].radius>box.max.z)continue;
                double x0=box.min.x-r,x1=box.max.x+r,y0=box.min.y-r,y1=box.max.y+r;
                if(p.x<x0-1e-4||p.x>x1+1e-4||p.y<y0-1e-4||p.y>y1+1e-4)continue;
                const double distances[4]={p.x-x0,x1-p.x,p.y-y0,y1-p.y};int side=0;for(int k=1;k<4;++k)if(distances[k]<distances[side])side=k;
                if(box.motionValid){
                    const bool entering[4]={p.x<=box.previousMin.x-r,p.x>=box.previousMax.x+r,p.y<=box.previousMin.y-r,p.y>=box.previousMax.y+r};
                    for(int k=0;k<4;++k)if(entering[k]){side=k;break;}
                }
                Vec3 n=side==0?Vec3{-1,0,0}:side==1?Vec3{1,0,0}:side==2?Vec3{0,-1,0}:Vec3{0,1,0};
                const auto contactKey=std::make_pair(a,box.key);
                const bool beginning=externalContacts_.insert(contactKey).second;
                if(responded.insert(contactKey).second){
                    const double surfaceSpeed=side==0?-box.velocityMin.x:side==1?box.velocityMax.x:side==2?-box.velocityMin.y:box.velocityMax.y;
                    const double closing=surfaceSpeed-Dot(states[a].velocityWorld,n);
                    if(box.motionValid&&closing>0&&settings.externalVelocityTransfer>0){
                        states[a].velocityWorld+=n*(settings.externalVelocityTransfer*(1+std::clamp(std::min(bodies[a].restitution,box.restitution),0.0,1.0))*closing);
                        if(beginning&&closing>settings.externalLiftThreshold)states[a].velocityWorld.z+=settings.externalLiftRatio*closing*settings.externalVelocityTransfer;
                        states[a].sleeping=false;states[a].sleepTime=0;
                    }else{const double vn=Dot(states[a].velocityWorld,n);if(vn<0)states[a].velocityWorld-=n*vn;}
                }
                if(distances[side]<=1e-4)continue;corrected=true;move(a,n*distances[side]);
                ++statistics.contacts;statistics.maxPenetration=std::max(statistics.maxPenetration,distances[side]);
            }
            if(!settings.selfCollision)continue;
            const auto key=cell(states[a].positionWorld);
            for(int dx=-1;dx<=1;++dx)for(int dy=-1;dy<=1;++dy){auto it=grid.find({key.first+dx,key.second+dy});if(it==grid.end())continue;
                for(size_t b:it->second){if(a==b||(active[b]&&b<a)||!(bodies[a].mask&bodies[b].mask))continue;
                    ++statistics.pairs;
                    Vec3 delta=states[a].positionWorld-states[b].positionWorld;delta.z=0;double d=Length(delta),depth=(bodies[a].radius+bodies[b].radius)*settings.xyFootprintScale-d;
                    if(depth<=1e-4)continue;
                    Vec3 n=d>1e-9?delta/d:Vec3{a<b?-1.0:1.0,0,0};
                    corrected=true;move(a,n*(depth*.5));move(b,-n*(depth*.5));
                    // Inelastic XY blocking; no torque or position-derived velocity.
                    double approaching=Dot(states[a].velocityWorld-states[b].velocityWorld,n);
                    if(approaching<0){states[a].velocityWorld-=n*(approaching*.5);states[b].velocityWorld+=n*(approaching*.5);}
                    ++statistics.contacts;statistics.maxPenetration=std::max(statistics.maxPenetration,depth);
                }
            }
        }
        if(!corrected)break;
    }
    avoidanceCursor_=(start+1)%states.size();
    for(const auto& s:states)statistics.sleeping+=s.sleeping;
}

bool CollisionScene::Validate(std::string& error)const{
    if(settings.groundOnly){
        if(!std::isfinite(settings.avoidanceSpeed)||settings.avoidanceSpeed<0||settings.avoidanceIterations<1||!(settings.xyFootprintScale>0)||!std::isfinite(settings.xyFootprintScale)||!std::isfinite(settings.groundRollStrength)||settings.groundRollStrength<0){error="Invalid XY avoidance settings";return false;}
        for(double value:{settings.externalVelocityTransfer,settings.externalLiftRatio,settings.externalLiftThreshold,settings.groundFrictionDecay})if(!std::isfinite(value)||value<0){error="Invalid external impact settings";return false;}
        for(const auto& b:bodies)if(!(b.radius>0&&b.mass>0)||!std::isfinite(b.radius)||!std::isfinite(b.mass)){error="Invalid XY footprint";return false;}
        for(const auto& b:environment)for(const auto& hull:b.body.hulls)for(auto p:hull.vertices)if(!Finite(p)){error="Nonfinite external Box";return false;}
        return true;
    }
    if(!std::isfinite(settings.contactOffset)||!std::isfinite(settings.restOffset)||!std::isfinite(settings.allowedPenetration)||!std::isfinite(settings.compliance)||settings.maxSubsteps>8){error="Nonfinite/out-of-range collision settings";return false;}
    if(settings.substeps==0||settings.maxSubsteps<settings.substeps||!settings.positionIterations||!settings.velocityIterations||settings.allowedPenetration<0||settings.contactOffset<settings.restOffset||settings.compliance<0){error="Invalid collision settings";return false;}
    auto validateBody=[&](const CollisionBody& b){if(!(b.mass>0&&b.minThickness>0&&b.radius>0)||!std::isfinite(b.mass)||b.hulls.empty()){error="Invalid collision body";return false;}for(const auto& h:b.hulls)if(!ValidateCollisionHull(h,error))return false;return true;};
    for(const auto& b:bodies)if(!validateBody(b))return false;for(const auto& b:environment)if(!validateBody(b.body))return false;return true;
}
bool CollisionScene::Advance(double h,const MotionSettings& motion,const MotionStepInput& input,const GroundPlaneQuery& ground,std::vector<ChunkMotionState>& states){
    if(statistics.paused)return false;statistics={};auto fail=[&](const char* message){statistics.paused=true;statistics.reason=message;return false;};
    if(states.size()!=bodies.size())return fail("Collision body/state count mismatch");
    if(!(h>0)||!std::isfinite(h)||!Finite(input.gravityWorld)||!Finite(input.windWorld))return fail("Invalid collision time/field input");
    for(const auto& field:input.fields)if(!Finite(field.windWorld)||!Finite(field.accelerationWorld)||!std::isfinite(field.dragScale)||field.dragScale<0)return fail("Invalid per-chunk collision field");
    uint32_t steps=settings.substeps;
    double minStatic=1e300;if(settings.externalCollision)for(const auto& b:environment)minStatic=std::min(minStatic,b.body.minThickness);
    if(settings.selfCollision)for(const auto& b:bodies)minStatic=std::min(minStatic,b.minThickness);
    for(size_t i=0;i<states.size();++i)if(states[i].detached&&!states[i].sleeping){
        const auto field=i<input.fields.size()?input.fields[i]:ChunkFieldInput{};
        const double drag=std::max(0.0,motion.linearDrag*field.dragScale);
        const Vec3 acceleration=input.gravityWorld+field.accelerationWorld+(input.windWorld+field.windWorld-states[i].velocityWorld)*drag;
        double travel=(Length(states[i].velocityWorld)+Length(states[i].angularVelocityWorld)*bodies[i].radius)*h+Length(acceleration)*h*h;
        // Bound both partners' travel when dynamic siblings can approach.
        if(settings.selfCollision&&bodies.size()>1)travel*=2;
        double required=std::ceil(travel/(.25*std::min(minStatic,bodies[i].minThickness)));
        if(!std::isfinite(required))return fail("Nonfinite collision sweep estimate");
        // Substeps are a cost cap, not a speed limit. Preserve linear/angular
        // velocity even when discrete collision cannot resolve the whole sweep.
        // Clamp before integer conversion; very large finite sweeps are legal.
        steps=std::max(steps,uint32_t(std::clamp(required,0.0,double(settings.maxSubsteps))));
    }
    statistics.substeps=steps;const double dt=h/steps;std::vector<WorkBody> w;
    for(size_t i=0;i<states.size();++i)w.push_back({&bodies[i],states[i],{}, {},states[i].detached});
    for(const auto& e:environment){ChunkMotionState s;s.positionWorld=e.position;s.rotationWorld=e.rotation;w.push_back({&e.body,s,{}, {},false});}
    // A static identity body supplies world-space plane anchors.
    CollisionBody planeBody;WorkBody plane;plane.body=&planeBody;w.push_back(plane);uint32_t planeIndex=uint32_t(w.size()-1);
    std::vector<int> groundCached(bodies.size(),-1);std::vector<PlaneCollider> cachedPlanes(bodies.size());
    std::string budgetReason;
    auto budgetFail=[&](){statistics.budgetExceeded=true;statistics.paused=!settings.simpleVisual;statistics.reason=budgetReason+(settings.simpleVisual?"; visual fallback: legacy ground only for this step":"; collision paused");return false;};
    auto generate=[&](std::vector<CollisionContact>& contacts)->bool{
        // Capacity is per contact pass, not multiplied by position iterations.
        statistics.pairs=statistics.shapePairs=statistics.contacts=0;
        struct Bound{uint32_t id;Bounds box;};std::vector<Bound> bounds;
        for(uint32_t i=0;i<planeIndex;++i){Bounds b{{1e300,1e300,1e300},{-1e300,-1e300,-1e300}};for(const auto& hull:w[i].body->hulls)for(Vec3 v:hull.vertices){Vec3 p=Point(w[i],v),old=w[i].oldX+Rotate(w[i].oldQ,v);b.min=Min(b.min,Min(p,old));b.max=Max(b.max,Max(p,old));}Vec3 margin{settings.contactOffset,settings.contactOffset,settings.contactOffset};b.min-=margin;b.max+=margin;bounds.push_back({i,b});}
        std::stable_sort(bounds.begin(),bounds.end(),[](const Bound&a,const Bound&b){return a.box.min.x<b.box.min.x;});
        for(size_t ai=0;ai<bounds.size();++ai)for(size_t bi=ai+1;bi<bounds.size()&&bounds[bi].box.min.x<=bounds[ai].box.max.x;++bi){auto ba=bounds[ai],bb=bounds[bi];uint32_t a=std::min(ba.id,bb.id),b=std::max(ba.id,bb.id);if(!w[a].dynamic&&!w[b].dynamic)continue;if(!(w[a].body->mask&w[b].body->mask))continue;if(b<bodies.size()&&!settings.selfCollision)continue;if(b>=bodies.size()&&!settings.externalCollision)continue;if(ba.box.max.y<bb.box.min.y||bb.box.max.y<ba.box.min.y||ba.box.max.z<bb.box.min.z||bb.box.max.z<ba.box.min.z)continue;
            if(++statistics.pairs>settings.maxPairs){budgetReason="Candidate pairs "+std::to_string(statistics.pairs)+" > "+std::to_string(settings.maxPairs);return false;}std::vector<CollisionContact> pair;uint32_t ha=0;
            for(const auto& ah:w[a].body->hulls){uint32_t hb=0;for(const auto& bh:w[b].body->hulls){if(++statistics.shapePairs>settings.maxShapePairs){budgetReason="Shape pairs "+std::to_string(statistics.shapePairs)+" > "+std::to_string(settings.maxShapePairs);return false;}auto found=CollideConvex(WorldHull(ah,w[a].state.positionWorld,w[a].state.rotationWorld),WorldHull(bh,w[b].state.positionWorld,w[b].state.rotationWorld),settings.contactOffset);for(auto c:found){c.a=a;c.b=b;c.geometryA=w[a].body->geometryVersion;c.geometryB=w[b].body->geometryVersion;c.friction=std::min(w[a].body->friction,w[b].body->friction);c.restitution=std::min(w[a].body->restitution,w[b].body->restitution);c.bounceThreshold=std::max(w[a].body->bounceThreshold,w[b].body->bounceThreshold);c.feature^=(uint64_t(ha)<<48)^(uint64_t(hb)<<40);pair.push_back(c);}++hb;}++ha;}
            Reduce(pair,settings.simpleVisual?3:4);for(auto c:pair){c.localA=Rotate(Conjugate(w[a].state.rotationWorld),c.localA-w[a].state.positionWorld);c.localB=Rotate(Conjugate(w[b].state.rotationWorld),c.localB-w[b].state.positionWorld);contacts.push_back(c);}
        }
        for(uint32_t a=0;a<bodies.size();++a)if(w[a].dynamic){std::vector<CollisionContact> pair;uint64_t vertex=0;for(const auto& hull:bodies[a].hulls)for(Vec3 v:hull.vertices){Vec3 p=Point(w[a],v);std::vector<PlaneCollider> planes=input.planes;PlaneCollider terrain;if(ground){if(settings.simpleVisual){if(groundCached[a]<0){if(++statistics.groundQueries>settings.maxGroundQueries){budgetReason="Ground query budget exceeded: "+std::to_string(statistics.groundQueries)+"/"+std::to_string(settings.maxGroundQueries);return false;}groundCached[a]=ground(a,w[a].state.positionWorld,cachedPlanes[a])?1:0;}if(groundCached[a]>0)planes.push_back(cachedPlanes[a]);}else{if(++statistics.groundQueries>settings.maxGroundQueries){budgetReason="Ground query budget exceeded: "+std::to_string(statistics.groundQueries)+"/"+std::to_string(settings.maxGroundQueries);return false;}if(ground(a,p,terrain))planes.push_back(terrain);}}for(const auto& pl:planes){double len=Length(pl.normal);if(len<1e-12)continue;Vec3 n=pl.normal/len;double gap=Dot(p,n)-pl.offset/len;if(gap>settings.contactOffset)continue;CollisionContact c;c.a=a;c.b=planeIndex;c.geometryA=bodies[a].geometryVersion;c.localA=p;c.localB=p-n*gap;c.normal=n;c.gap=gap;c.feature=vertex;c.friction=pl.friction;c.restitution=pl.restitution;c.bounceThreshold=pl.restitutionVelocityThreshold;c.angularDamping=pl.angularDamping;pair.push_back(c);}++vertex;}Reduce(pair,settings.simpleVisual?3:4);for(auto c:pair){c.localA=Rotate(Conjugate(w[a].state.rotationWorld),c.localA-w[a].state.positionWorld);contacts.push_back(c);}}
        statistics.contacts=contacts.size();if(contacts.size()>settings.maxContacts){budgetReason="Contact points "+std::to_string(contacts.size())+" > "+std::to_string(settings.maxContacts);return false;}return true;
    };
    if(!preStabilized_){
        for(auto& b:w){b.oldX=b.state.positionWorld;b.oldQ=b.state.rotationWorld;}
        for(uint32_t iteration=0;iteration<settings.positionIterations;++iteration){
            std::vector<CollisionContact> initial;if(!generate(initial))return budgetFail();
            if(iteration==0&&!settings.simpleVisual)for(const auto& c:initial)if(-c.gap>settings.allowedPenetration+1e-8)
                return fail("Initial penetration exceeds allowed tolerance; inspect frozen proxies/registered Boxes");
            for(const auto& c:initial){if(c.gap>=0)continue;auto&a=w[c.a];auto&b=w[c.b];Vec3 ra=Point(a,c.localA)-a.state.positionWorld,rb=Point(b,c.localB)-b.state.positionWorld;double eff=Effective(a,b,ra,rb,c.normal,c.normal);if(eff>1e-15)PositionImpulse(a,b,ra,rb,c.normal,(settings.simpleVisual?std::min(-c.gap,0.5/double(settings.positionIterations)):-c.gap)/eff);}
        }
        // Preserve incoming velocities: pre-stabilization moves both pose endpoints.
        for(auto& b:w){b.oldX=b.state.positionWorld;b.oldQ=b.state.rotationWorld;}
    }
    for(uint32_t sub=0;sub<steps;++sub){
        statistics.pairs=statistics.shapePairs=statistics.groundQueries=0;
        std::fill(groundCached.begin(),groundCached.end(),-1);
        for(size_t i=0;i<w.size();++i){auto& b=w[i];b.oldX=b.state.positionWorld;b.oldQ=b.state.rotationWorld;if(!b.dynamic||b.state.sleeping)continue;ChunkFieldInput field=i<input.fields.size()?input.fields[i]:ChunkFieldInput{};Vec3 wind=input.windWorld+field.windWorld,acc=input.gravityWorld+field.accelerationWorld;double drag=std::max(0.0,motion.linearDrag*field.dragScale),x=drag*dt,A=x<1e-5?dt*(1-x/2+x*x/6):-std::expm1(-x)/drag,B=x<1e-5?dt*dt*(.5-x/6+x*x/24):(dt-A)/drag;Vec3 v=b.state.velocityWorld;b.state.positionWorld+=v*dt+(acc-(v-wind)*drag)*B;b.state.velocityWorld=wind+(v-wind)*std::exp(-x)+acc*A;double ad=motion.angularDrag,travel=ad*dt<1e-5?dt*(1-ad*dt/2):-std::expm1(-ad*dt)/ad;b.state.rotationWorld=Normalize(Multiply(QuatFromRotationVector(b.state.angularVelocityWorld*travel),b.state.rotationWorld));b.state.angularVelocityWorld=b.state.angularVelocityWorld*std::exp(-ad*dt);}
        std::vector<CollisionContact> contacts;if(!generate(contacts))return budgetFail();
        for(auto& c:contacts){auto&a=w[c.a];auto&b=w[c.b];if(c.gap>settings.restOffset)continue;bool wake=(a.dynamic&&!a.state.sleeping)||(b.dynamic&&!b.state.sleeping);if(wake){if(a.dynamic)a.state.sleeping=false;if(b.dynamic)b.state.sleeping=false;}Vec3 ra=Point(a,c.localA)-a.state.positionWorld,rb=Point(b,c.localB)-b.state.positionWorld;double speed=Dot(Speed(a,ra)-Speed(b,rb),c.normal),threshold=c.bounceThreshold;c.targetSpeed=speed < -threshold?-c.restitution*speed:0;}
        for(uint32_t iteration=0;iteration<settings.positionIterations;++iteration){
            if(iteration>0){
                std::vector<CollisionContact> refreshed;
                if(!generate(refreshed))return budgetFail();
                // Feature/geometry-scoped history lives only within this substep.
                // Rebuilding the scene on GeometryVersion changes invalidates it.
                std::set<size_t> used;
                for(auto& next:refreshed){
                    double bestDistance=1e300;size_t best=contacts.size();
                    for(size_t i=0;i<contacts.size();++i){const auto& old=contacts[i];
                        if(used.count(i)||next.a!=old.a||next.b!=old.b||next.feature!=old.feature||next.geometryA!=old.geometryA||next.geometryB!=old.geometryB||Dot(next.normal,old.normal)<.995)continue;
                        double d=LengthSquared(next.localA-old.localA)+LengthSquared(next.localB-old.localB);
                        if(d<bestDistance){bestDistance=d;best=i;}
                    }
                    if(best<contacts.size()&&bestDistance<4*std::max(1e-6,settings.contactOffset*settings.contactOffset)){
                        next.age=contacts[best].age+1;next.lambda=contacts[best].lambda;next.targetSpeed=contacts[best].targetSpeed;used.insert(best);
                    }else{
                        const auto&a=w[next.a];const auto&b=w[next.b];
                        Vec3 ra=Point(a,next.localA)-a.state.positionWorld,rb=Point(b,next.localB)-b.state.positionWorld;
                        double vn=Dot(Speed(a,ra)-Speed(b,rb),next.normal);
                        next.targetSpeed=vn < -next.bounceThreshold?-next.restitution*vn:0;
                    }
                }
                contacts=std::move(refreshed);
            }
            for(auto& c:contacts){auto&a=w[c.a];auto&b=w[c.b];if((!a.dynamic||a.state.sleeping)&&(!b.dynamic||b.state.sleeping))continue;Vec3 pa=Point(a,c.localA),pb=Point(b,c.localB),ra=pa-a.state.positionWorld,rb=pb-b.state.positionWorld;double gap=Dot(pa-pb,c.normal),eff=Effective(a,b,ra,rb,c.normal,c.normal),alpha=settings.compliance/(dt*dt);if(eff<=1e-15)continue;double next=std::max(0.0,c.lambda+(-(gap-settings.restOffset)-alpha*c.lambda)/(eff+alpha));double dl=next-c.lambda;if(settings.simpleVisual){double maxGap=.5/double(settings.positionIterations);dl=std::clamp(dl,-maxGap/eff,maxGap/eff);}double angular=std::max(Length(InvI(a,Cross(ra,c.normal))*dl),Length(InvI(b,Cross(rb,c.normal))*dl));if(angular>.2)dl*=.2/angular;c.lambda+=dl;Vec3 ax=a.state.positionWorld,bx=b.state.positionWorld;Quat aq=a.state.rotationWorld,bq=b.state.rotationWorld;PositionImpulse(a,b,ra,rb,c.normal,dl);if(settings.simpleVisual){a.oldX+=a.state.positionWorld-ax;b.oldX+=b.state.positionWorld-bx;a.oldQ=Normalize(Multiply(Multiply(a.state.rotationWorld,Conjugate(aq)),a.oldQ));b.oldQ=Normalize(Multiply(Multiply(b.state.rotationWorld,Conjugate(bq)),b.oldQ));}}}
        for(auto& b:w)if(b.dynamic&&!b.state.sleeping){b.state.velocityWorld=(b.state.positionWorld-b.oldX)/dt;b.state.angularVelocityWorld=AngularDelta(b.state.rotationWorld,b.oldQ,dt);}
        for(auto& c:contacts)c.normalImpulse=settings.simpleVisual?0:c.lambda/dt;
        for(uint32_t iteration=0;iteration<settings.velocityIterations;++iteration)for(auto& c:contacts){auto&a=w[c.a];auto&b=w[c.b];if(c.lambda==0&&c.gap>settings.restOffset)continue;Vec3 pa=Point(a,c.localA),pb=Point(b,c.localB),p=(pa+pb)*.5,ra=p-a.state.positionWorld,rb=p-b.state.positionWorld;double eff=Effective(a,b,ra,rb,c.normal,c.normal);if(eff<=1e-15)continue;double vn=Dot(Speed(a,ra)-Speed(b,rb),c.normal),jn=std::max(0.0,c.normalImpulse+(c.targetSpeed-vn)/eff);Impulse(a,b,ra,rb,c.normal*(jn-c.normalImpulse));c.normalImpulse=jn;Vec3 t0=Normalize(Cross(c.normal,std::abs(c.normal.x)<.7?Vec3{1,0,0}:Vec3{0,1,0})),t1=Cross(c.normal,t0),vel=Speed(a,ra)-Speed(b,rb);double k00=Effective(a,b,ra,rb,t0,t0),k01=Effective(a,b,ra,rb,t0,t1),k11=Effective(a,b,ra,rb,t1,t1),det=k00*k11-k01*k01;if(det<=1e-20)continue;double v0=Dot(vel,t0),v1=Dot(vel,t1);Vec3 jt=c.tangentImpulse+t0*((-k11*v0+k01*v1)/det)+t1*((k01*v0-k00*v1)/det);double limit=c.friction*jn,len=Length(jt);if(len>limit)jt=jt*(limit/len);Impulse(a,b,ra,rb,jt-c.tangentImpulse);c.tangentImpulse=jt;}
        std::vector<bool> touching(w.size(),false);for(const auto& c:contacts){double depth=std::max(0.0,settings.restOffset-Dot(Point(w[c.a],c.localA)-Point(w[c.b],c.localB),c.normal));statistics.maxPenetration=std::max(statistics.maxPenetration,depth);if(c.lambda>0||c.gap<=settings.allowedPenetration)touching[c.a]=touching[c.b]=true;}
        for(size_t i=0;i<bodies.size();++i){auto& s=w[i].state;double damping=0;for(const auto& c:contacts)if((c.a==i||c.b==i)&&c.lambda>0)damping=std::max(damping,c.angularDamping);s.angularVelocityWorld=s.angularVelocityWorld*std::exp(-damping*dt);if(!Finite(s.positionWorld)||!Finite(s.velocityWorld)||!Finite(s.angularVelocityWorld))return fail("Nonfinite collision state");if(!w[i].dynamic)continue;if(touching[i]&&Length(s.velocityWorld)<=motion.sleepLinearSpeed&&Length(s.angularVelocityWorld)<=motion.sleepAngularSpeed){s.sleepTime+=dt;if(s.sleepTime>=motion.sleepDelay){s.sleeping=true;s.velocityWorld={};s.angularVelocityWorld={};}}else s.sleepTime=0;}
    }
    statistics.sleeping=0;for(size_t i=0;i<states.size();++i){states[i]=w[i].state;statistics.sleeping+=states[i].sleeping;}preStabilized_=true;return true;
}
}
