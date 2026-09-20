#include "TVFFracture.h"
#include "CompGeom/ExactPredicates.h"

#include <algorithm>
#include <array>
#include <cfloat>
#include <cmath>
#include <limits>
#include <map>
#include <numeric>
#include <set>
#include <tuple>
#include <unordered_map>
#include <utility>

namespace tvf {
namespace {
constexpr double kTiny=1e-13;
constexpr uint64_t kKindMask=3ull<<62;
constexpr uint64_t kGridPlane=1ull<<62;
constexpr uint64_t kSolidPlane=2ull<<62;
constexpr uint64_t kPowerPlane=3ull<<62;

struct Polygon {std::vector<Vec3> p;std::vector<std::vector<uint64_t>> supports;int32_t neighbor=-1;uint64_t sourcePlaneId=0;};
struct Polyhedron {std::vector<Polygon> faces;};
struct Plane {Vec3 n;double d=0;int32_t neighbor=-1;uint64_t sourcePlaneId=0;};
struct Piece {uint32_t seed=0;uint64_t tetra=0;Polyhedron poly;double volume=0;Vec3 centroid;Bounds bounds;};
struct FaceRef {uint32_t piece=0,face=0;};
struct FacePair {FaceRef a,b;bool interface=false,verifyMirror=false;Polygon canonical;};

struct DSU {
    std::vector<uint32_t> p;
    explicit DSU(size_t n):p(n){std::iota(p.begin(),p.end(),0);}
    uint32_t Find(uint32_t a){while(p[a]!=a){p[a]=p[p[a]];a=p[a];}return a;}
    void Join(uint32_t a,uint32_t b){a=Find(a);b=Find(b);if(a!=b)p[b]=a;}
};

uint64_t Mix64(uint64_t x){x^=x>>30;x*=0xbf58476d1ce4e5b9ull;x^=x>>27;x*=0x94d049bb133111ebull;x^=x>>31;return x;}
uint64_t HashIds(uint64_t a,uint64_t b,uint64_t c=0){return Mix64(a+0x9e3779b97f4a7c15ull+Mix64(b)+Mix64(c))&~kKindMask;}
uint64_t GridFaceId(std::array<uint64_t,3> ids){std::sort(ids.begin(),ids.end());return kGridPlane|HashIds(ids[0],ids[1],ids[2]);}
uint64_t SolidPlaneId(uint64_t tetra){return kSolidPlane|(Mix64(tetra)&~kKindMask);}
uint64_t PowerPlaneId(uint32_t a,uint32_t b){if(a>b)std::swap(a,b);return kPowerPlane|HashIds(a,b);}
uint64_t PlaneKind(uint64_t id){return id&kKindMask;}
double ExactOrient(Vec3 a,Vec3 b,Vec3 c,Vec3 d){double pa[3]={a.x,a.y,a.z},pb[3]={b.x,b.y,b.z},pc[3]={c.x,c.y,c.z},pd[3]={d.x,d.y,d.z};return UE::Geometry::ExactPredicates::Orient3D(pa,pb,pc,pd);}

void Report(std::vector<Diagnostic>& out,DiagnosticSeverity severity,DiagnosticStage stage,DiagnosticCode code,
            const std::string& message,uint64_t seed=Diagnostic::InvalidId,uint64_t tet=Diagnostic::InvalidId,
            uint64_t patch=Diagnostic::InvalidId,double measured=0,double limit=0){
    Diagnostic d;d.severity=severity;d.stage=stage;d.code=code;d.message=message;d.seedId=seed;d.tetId=tet;d.patchId=patch;d.measuredValue=measured;d.limit=limit;out.push_back(std::move(d));
}
void Report(BakeResult& result,DiagnosticSeverity severity,DiagnosticStage stage,DiagnosticCode code,
            const std::string& message,uint64_t seed=Diagnostic::InvalidId,uint64_t tet=Diagnostic::InvalidId,
            uint64_t patch=Diagnostic::InvalidId,double measured=0,double limit=0){
    Report(result.diagnostics,severity,stage,code,message,seed,tet,patch,measured,limit);result.warnings.push_back(message);
}

inline double PlaneValue(const Plane& plane,Vec3 p){return std::fma(plane.n.x,p.x,std::fma(plane.n.y,p.y,std::fma(plane.n.z,p.z,-plane.d)));}
Bounds PolyBounds(const Polyhedron& p){Bounds b{{DBL_MAX,DBL_MAX,DBL_MAX},{-DBL_MAX,-DBL_MAX,-DBL_MAX}};for(const Polygon& f:p.faces)for(Vec3 v:f.p){b.min=Min(b.min,v);b.max=Max(b.max,v);}return b;}
void NormalizeSupports(std::vector<uint64_t>& ids){std::sort(ids.begin(),ids.end());ids.erase(std::unique(ids.begin(),ids.end()),ids.end());}
std::vector<uint64_t> IntersectionSupport(const std::vector<uint64_t>& a,const std::vector<uint64_t>& b,uint64_t plane){std::vector<uint64_t> out;for(uint64_t id:a)if(std::find(b.begin(),b.end(),id)!=b.end())out.push_back(id);out.push_back(plane);NormalizeSupports(out);return out;}
void UniquePolygonPoints(Polygon& face,double eps){std::vector<Vec3> points;std::vector<std::vector<uint64_t>> supports;double e2=eps*eps;for(size_t i=0;i<face.p.size();++i){bool duplicate=false;for(size_t j=0;j<points.size();++j)if(LengthSquared(face.p[i]-points[j])<=e2){if(i<face.supports.size()){supports[j].insert(supports[j].end(),face.supports[i].begin(),face.supports[i].end());NormalizeSupports(supports[j]);}duplicate=true;break;}if(!duplicate){points.push_back(face.p[i]);supports.push_back(i<face.supports.size()?face.supports[i]:std::vector<uint64_t>{face.sourcePlaneId});}}face.p.swap(points);face.supports.swap(supports);}
double PolygonArea(const Polygon& f,Vec3* normal=nullptr,Vec3* center=nullptr){
    if(f.p.size()<3){if(normal)*normal={};if(center)*center={};return 0;}Vec3 r=f.p[0],areaNormal{},weighted{};double area=0;
    for(size_t i=1;i+1<f.p.size();++i){Vec3 cross=Cross(f.p[i]-r,f.p[i+1]-r);double a=Length(cross)*0.5;areaNormal+=cross;weighted+=(r+f.p[i]+f.p[i+1])*(a/3.0);area+=a;}
    const double normalLength=Length(areaNormal);
    if(normal)*normal=normalLength>0?areaNormal/normalLength:Vec3{};if(center)*center=area>0?weighted/area:r;return area;
}
void SortCap(Polygon& face,Vec3 normal,double eps){
    UniquePolygonPoints(face,eps);if(face.p.size()<3)return;Vec3 c{};for(Vec3 p:face.p)c+=p;c=c/double(face.p.size());Vec3 n=Normalize(normal);
    Vec3 u=std::abs(n.x)<0.7?Normalize(Cross(n,{1,0,0})):Normalize(Cross(n,{0,1,0}));Vec3 v=Cross(n,u);
    std::vector<size_t> order(face.p.size());std::iota(order.begin(),order.end(),0);std::sort(order.begin(),order.end(),[&](size_t ia,size_t ib){Vec3 a=face.p[ia]-c,b=face.p[ib]-c;return std::atan2(Dot(a,v),Dot(a,u))<std::atan2(Dot(b,v),Dot(b,u));});std::vector<Vec3> points;std::vector<std::vector<uint64_t>> supports;for(size_t i:order){points.push_back(face.p[i]);supports.push_back(face.supports[i]);}face.p.swap(points);face.supports.swap(supports);
    Vec3 observed;PolygonArea(face,&observed);if(Dot(observed,n)<0){std::reverse(face.p.begin(),face.p.end());std::reverse(face.supports.begin(),face.supports.end());}
}
std::vector<uint64_t> CommonSupports(const std::vector<uint64_t>& a,const std::vector<uint64_t>& b){std::vector<uint64_t> out;for(uint64_t id:a)if(std::find(b.begin(),b.end(),id)!=b.end())out.push_back(id);NormalizeSupports(out);return out;}
Polygon IntersectCoplanarPolygons(const Polygon& subject,const Polygon& clip,double eps){
    Polygon result=subject;if(result.p.size()<3||clip.p.size()<3||result.supports.size()!=result.p.size()||clip.supports.size()!=clip.p.size())return {};
    Vec3 clipNormal;if(PolygonArea(clip,&clipNormal)<=eps*eps)return {};
    for(size_t edgeIndex=0;edgeIndex<clip.p.size()&&result.p.size()>=3;++edgeIndex){
        const size_t nextEdge=(edgeIndex+1)%clip.p.size();const Vec3 edgeStart=clip.p[edgeIndex],edge=clip.p[nextEdge]-edgeStart;
        std::vector<uint64_t> edgeSupports=CommonSupports(clip.supports[edgeIndex],clip.supports[nextEdge]);edgeSupports.push_back(clip.sourcePlaneId);NormalizeSupports(edgeSupports);
        Polygon output;output.sourcePlaneId=subject.sourcePlaneId;output.neighbor=subject.neighbor;
        Vec3 previous=result.p.back();std::vector<uint64_t> previousSupports=result.supports.back();double previousValue=Dot(Cross(edge,previous-edgeStart),clipNormal);
        for(size_t pointIndex=0;pointIndex<result.p.size();++pointIndex){
            const Vec3 current=result.p[pointIndex];const std::vector<uint64_t>& currentSupports=result.supports[pointIndex];double currentValue=Dot(Cross(edge,current-edgeStart),clipNormal);
            if((previousValue<0&&currentValue>0)||(previousValue>0&&currentValue<0)){
                const bool forward=std::tie(previous.x,previous.y,previous.z)<std::tie(current.x,current.y,current.z);
                const Vec3 first=forward?previous:current,last=forward?current:previous;
                const double firstValue=forward?previousValue:currentValue,lastValue=forward?currentValue:previousValue;
                output.p.push_back(first+(last-first)*(firstValue/(firstValue-lastValue)));
                std::vector<uint64_t> supports=CommonSupports(previousSupports,currentSupports);supports.insert(supports.end(),edgeSupports.begin(),edgeSupports.end());NormalizeSupports(supports);output.supports.push_back(std::move(supports));
            }
            if(currentValue>=0){output.p.push_back(current);auto supports=currentSupports;if(currentValue==0){supports.insert(supports.end(),edgeSupports.begin(),edgeSupports.end());NormalizeSupports(supports);}output.supports.push_back(std::move(supports));}previous=current;previousSupports=currentSupports;previousValue=currentValue;
        }
        UniquePolygonPoints(output,eps);result=std::move(output);
    }
    if(result.p.size()<3||PolygonArea(result)<=eps*eps)return {};return result;
}
bool Clip(Polyhedron& solid,const Plane& plane,double eps){
    if(solid.faces.empty())return false;
    // Classification and interpolation use the SAME zero plane. Expanding both
    // halfspaces by eps creates overlapping slivers and unilateral Power caps.
    bool hasInside=false,hasOutside=false;
    for(const Polygon& face:solid.faces)for(Vec3 p:face.p){double value=PlaneValue(plane,p);hasInside|=value<0;hasOutside|=value>0;}
    if(!hasOutside)return true;
    if(!hasInside){solid.faces.clear();return false;} // zero-volume contact only
    std::vector<Polygon> faces;Polygon cap;cap.sourcePlaneId=plane.sourcePlaneId;cap.neighbor=plane.neighbor;
    for(const Polygon& input:solid.faces){
        if(input.p.size()<3)continue;
        Polygon output;output.neighbor=input.neighbor;output.sourcePlaneId=input.sourcePlaneId;
        for(size_t index=0;index<input.p.size();++index){
            const size_t previous=(index+input.p.size()-1)%input.p.size();
            const Vec3 a=input.p[previous],b=input.p[index];
            const auto sa=input.supports.empty()?std::vector<uint64_t>{input.sourcePlaneId}:input.supports[previous];
            const auto sb=input.supports.empty()?std::vector<uint64_t>{input.sourcePlaneId}:input.supports[index];
            const double fa=PlaneValue(plane,a),fb=PlaneValue(plane,b);
            if((fa<0&&fb>0)||(fa>0&&fb<0)){
                // Canonical endpoint order gives identical construction on an
                // edge visited with reversed winding by another face.
                const bool forward=std::tie(a.x,a.y,a.z)<std::tie(b.x,b.y,b.z);
                const Vec3 first=forward?a:b,last=forward?b:a;
                const double fFirst=forward?fa:fb,fLast=forward?fb:fa;
                const double t=fFirst/(fFirst-fLast);
                const Vec3 point=first+(last-first)*t;
                auto support=IntersectionSupport(sa,sb,plane.sourcePlaneId);
                output.p.push_back(point);output.supports.push_back(support);
                cap.p.push_back(point);cap.supports.push_back(std::move(support));
            }
            if(fb<=0){auto support=sb;if(fb==0){support.push_back(plane.sourcePlaneId);NormalizeSupports(support);cap.p.push_back(b);cap.supports.push_back(support);}output.p.push_back(b);output.supports.push_back(std::move(support));}
        }
        UniquePolygonPoints(output,eps);if(output.p.size()>=3&&PolygonArea(output)>eps*eps)faces.push_back(std::move(output));
    }
    SortCap(cap,plane.n,eps);if(cap.p.size()>=3&&PolygonArea(cap)>eps*eps)faces.push_back(std::move(cap));
    solid.faces.swap(faces);return !solid.faces.empty();
}
Polyhedron MakeTetra(const std::array<Vec3,4>& p,const std::array<uint64_t,4>& vertexIds){
    Polyhedron out;static constexpr uint32_t face[4][3]={{0,2,1},{0,1,3},{0,3,2},{1,2,3}};static constexpr uint32_t opposite[4]={3,2,1,0};
    std::array<uint64_t,4> planeIds{};for(int f=0;f<4;++f){std::array<uint64_t,3> ids{vertexIds[face[f][0]],vertexIds[face[f][1]],vertexIds[face[f][2]]};planeIds[f]=GridFaceId(ids);}std::array<std::vector<uint64_t>,4> vertexSupports;for(int vertex=0;vertex<4;++vertex)for(int f=0;f<4;++f)for(int corner=0;corner<3;++corner)if(face[f][corner]==vertex){vertexSupports[vertex].push_back(planeIds[f]);break;}
    // Orient3D(a,b,c,d) has the opposite sign of dot(cross(b-a,c-a),d-a).
    // An outward face places the opposite tetra vertex behind it: Orient3D > 0.
    for(int f=0;f<4;++f){Polygon poly;poly.sourcePlaneId=planeIds[f];for(int corner=0;corner<3;++corner){uint32_t vertex=face[f][corner];poly.p.push_back(p[vertex]);poly.supports.push_back(vertexSupports[vertex]);}if(ExactOrient(poly.p[0],poly.p[1],poly.p[2],p[opposite[f]])<0){std::swap(poly.p[1],poly.p[2]);std::swap(poly.supports[1],poly.supports[2]);}out.faces.push_back(std::move(poly));}return out;
}
bool VolumeCentroid(const Polyhedron& p,double& volume,Vec3& centroid){
    std::vector<Vec3> all;for(const Polygon& f:p.faces)for(Vec3 v:f.p)all.push_back(v);if(all.empty())return false;Vec3 r{};for(Vec3 v:all)r+=v;r=r/double(all.size());double total=0;Vec3 weighted{};
    for(const Polygon& f:p.faces)for(size_t i=1;i+1<f.p.size();++i){double v=Dot(f.p[0]-r,Cross(f.p[i]-r,f.p[i+1]-r))/6.0;total+=v;weighted+=(r+f.p[0]+f.p[i]+f.p[i+1])*(v*0.25);}
    if(!std::isfinite(total)||total<=0)return false;volume=total;centroid=weighted/total;return Finite(centroid);
}
std::vector<SolidSimplex> Decompose(const Polyhedron& poly,Vec3 center,double eps){
    std::vector<SolidSimplex> out;for(const Polygon& f:poly.faces)for(size_t i=1;i+1<f.p.size();++i){SolidSimplex s{{center,f.p[0],f.p[i],f.p[i+1]},0};s.volume=std::abs(Dot(s.vertices[1]-s.vertices[0],Cross(s.vertices[2]-s.vertices[0],s.vertices[3]-s.vertices[0])))/6.0;if(s.volume>eps*eps*eps)out.push_back(s);}return out;
}
Polyhedron ToPoly(const SolidTetPiece& piece){Polyhedron p;p.faces.reserve(piece.faces.size());for(const ConvexFace& f:piece.faces)p.faces.push_back({f.vertices,f.vertexSupportIds,-1,f.sourcePlaneId});return p;}
SolidTetPiece ToSolidPiece(uint64_t tet,const Polyhedron& poly,double volume,Vec3 center,double eps){SolidTetPiece p;p.tetraId=tet;p.volume=volume;p.centroid=center;p.bounds=PolyBounds(poly);for(const Polygon& f:poly.faces)p.faces.push_back({f.p,f.supports,f.sourcePlaneId});p.samplingTetrahedra=Decompose(poly,center,eps);return p;}

Bounds WhitenBounds(const Bounds& b,const Metric& metric){Bounds out{{DBL_MAX,DBL_MAX,DBL_MAX},{-DBL_MAX,-DBL_MAX,-DBL_MAX}};for(int z=0;z<2;++z)for(int y=0;y<2;++y)for(int x=0;x<2;++x){Vec3 p{x?b.max.x:b.min.x,y?b.max.y:b.min.y,z?b.max.z:b.min.z};p=metric.Whiten(p);out.min=Min(out.min,p);out.max=Max(out.max,p);}return out;}
double BoundsDistanceSquared(const Bounds& a,const Bounds& b){double d=0;for(int axis=0;axis<3;++axis){double amin=axis==0?a.min.x:(axis==1?a.min.y:a.min.z),amax=axis==0?a.max.x:(axis==1?a.max.y:a.max.z),bmin=axis==0?b.min.x:(axis==1?b.min.y:b.min.z),bmax=axis==0?b.max.x:(axis==1?b.max.y:b.max.z);double q=amax<bmin?bmin-amax:(bmax<amin?amin-bmax:0);d+=q*q;}return d;}
double PointBoundsDistanceSquared(Vec3 p,const Bounds& b){double d=0;double v[3]={p.x,p.y,p.z},lo[3]={b.min.x,b.min.y,b.min.z},hi[3]={b.max.x,b.max.y,b.max.z};for(int i=0;i<3;++i){double q=v[i]<lo[i]?lo[i]-v[i]:(v[i]>hi[i]?v[i]-hi[i]:0);d+=q*q;}return d;}
double MaxDistanceSquared(Vec3 p,const Bounds& b){double x=std::max(std::abs(p.x-b.min.x),std::abs(p.x-b.max.x)),y=std::max(std::abs(p.y-b.min.y),std::abs(p.y-b.max.y)),z=std::max(std::abs(p.z-b.min.z),std::abs(p.z-b.max.z));return x*x+y*y+z*z;}

class SeedTree {
    struct Node{Bounds bounds;double maxWeight=-DBL_MAX;uint32_t begin=0,end=0;int left=-1,right=-1;};
    const std::vector<Seed>& seeds;Metric metric;std::vector<Vec3> points;std::vector<uint32_t> order;std::vector<Node> nodes;
    int Build(uint32_t begin,uint32_t end){Node n;n.begin=begin;n.end=end;n.bounds={{DBL_MAX,DBL_MAX,DBL_MAX},{-DBL_MAX,-DBL_MAX,-DBL_MAX}};for(uint32_t i=begin;i<end;++i){Vec3 p=points[order[i]];n.bounds.min=Min(n.bounds.min,p);n.bounds.max=Max(n.bounds.max,p);n.maxWeight=std::max(n.maxWeight,seeds[order[i]].weight);}int id=int(nodes.size());nodes.push_back(n);if(end-begin>8){Vec3 size=n.bounds.Size();int axis=size.x>size.y?(size.x>size.z?0:2):(size.y>size.z?1:2);uint32_t mid=begin+(end-begin)/2;std::nth_element(order.begin()+begin,order.begin()+mid,order.begin()+end,[&](uint32_t a,uint32_t b){return axis==0?points[a].x<points[b].x:(axis==1?points[a].y<points[b].y:points[a].z<points[b].z);});nodes[id].left=Build(begin,mid);nodes[id].right=Build(mid,end);}return id;}
    double PointLower(Vec3 p,const Node& n)const{return PointBoundsDistanceSquared(p,n.bounds)-n.maxWeight;}
    void Nearest(int nodeId,Vec3 p,uint32_t& best,double& value)const{const Node& n=nodes[nodeId];if(PointLower(p,n)>value)return;if(n.left<0){for(uint32_t i=n.begin;i<n.end;++i){uint32_t id=order[i];double q=LengthSquared(p-points[id])-seeds[id].weight;if(q<value||(q==value&&id<best)){value=q;best=id;}}return;}double a=PointLower(p,nodes[n.left]),b=PointLower(p,nodes[n.right]);if(a<b){Nearest(n.left,p,best,value);Nearest(n.right,p,best,value);}else{Nearest(n.right,p,best,value);Nearest(n.left,p,best,value);}}
    void Collect(int nodeId,const Bounds& piece,double upper,double error,std::vector<uint32_t>& out)const{const Node& n=nodes[nodeId];if(BoundsDistanceSquared(piece,n.bounds)-n.maxWeight>upper+error)return;if(n.left<0){for(uint32_t i=n.begin;i<n.end;++i)out.push_back(order[i]);return;}Collect(n.left,piece,upper,error,out);Collect(n.right,piece,upper,error,out);}
public:
    SeedTree(const std::vector<Seed>& in,const Metric& m):seeds(in),metric(m){points.reserve(seeds.size());order.resize(seeds.size());std::iota(order.begin(),order.end(),0);for(const Seed& s:seeds)points.push_back(metric.Whiten(s.position));if(!order.empty())Build(0,uint32_t(order.size()));}
    std::vector<uint32_t> Candidates(const SolidTetPiece& piece,double error)const{Bounds b=WhitenBounds(piece.bounds,metric);Vec3 c=metric.Whiten(piece.centroid);uint32_t reference=0;double best=DBL_MAX;Nearest(0,c,reference,best);double upper=MaxDistanceSquared(points[reference],b)-seeds[reference].weight;std::vector<uint32_t> out;Collect(0,b,upper,error,out);if(std::find(out.begin(),out.end(),reference)==out.end())out.push_back(reference);std::sort(out.begin(),out.end());out.erase(std::unique(out.begin(),out.end()),out.end());return out;}
    uint64_t CapacityBytes()const{return uint64_t(points.capacity())*sizeof(Vec3)+uint64_t(order.capacity())*sizeof(uint32_t)+uint64_t(nodes.capacity())*sizeof(Node);}
};

void UpdateStatistics(BakeResult& result,const SolidDomain& domain,const SeedTree& tree,
                      const std::vector<Piece>& pieces,const std::vector<bool>& seedVolume,
                      const std::vector<uint64_t>& candidateCounts,size_t targetSeedCount){
    result.statistics.activeTets=domain.activeTets;
    result.statistics.actualCellTetPairs=pieces.size();
    result.statistics.planeTests=result.planeTestCount;
    result.statistics.canonicalVertices=result.canonicalVertices.size();
    result.statistics.interfacePatches=result.interfacePatches.size();
    result.statistics.peakTemporaryBytes=domain.peakTemporaryBytes+tree.CapacityBytes()+
        pieces.capacity()*sizeof(Piece)+candidateCounts.capacity()*sizeof(uint64_t);
    result.statistics.targetSeeds=uint32_t(targetSeedCount);
    result.statistics.validSeeds=uint32_t(std::count(seedVolume.begin(),seedVolume.end(),true));
    result.statistics.finalChunks=uint32_t(result.chunks.size());
    result.statistics.outputTriangles=0;
    for(const Chunk& chunk:result.chunks)result.statistics.outputTriangles+=chunk.surface.size();
    if(!candidateCounts.empty()){
        std::vector<uint64_t> sorted=candidateCounts;
        std::sort(sorted.begin(),sorted.end());
        result.statistics.candidateSitesP50=sorted[sorted.size()/2];
        result.statistics.candidateSitesP95=sorted[std::min(sorted.size()-1,size_t(std::floor(sorted.size()*0.95)))];
        result.statistics.candidateSitesMax=sorted.back();
    }
}

bool GeometryMatches(const Polygon& a,const Polygon& b,double eps){Vec3 ca,cb,na,nb;double aa=PolygonArea(a,&na,&ca),ab=PolygonArea(b,&nb,&cb);return std::abs(aa-ab)<=std::max(eps*eps,aa*1e-8)&&Length(ca-cb)<=eps*16&&std::abs(Dot(na,nb))>=1-1e-8;}
void CanonicalOrder(Polygon& face){if(face.p.empty())return;size_t first=0;for(size_t i=1;i<face.p.size();++i)if(std::tie(face.p[i].x,face.p[i].y,face.p[i].z)<std::tie(face.p[first].x,face.p[first].y,face.p[first].z))first=i;std::rotate(face.p.begin(),face.p.begin()+first,face.p.end());if(face.supports.size()==face.p.size())std::rotate(face.supports.begin(),face.supports.begin()+first,face.supports.end());}

struct CanonicalEmitter {
    BakeResult& result;uint64_t nextVertex=0,nextPatch=0;
    std::vector<uint64_t> EmitVertices(const Polygon& face){std::vector<uint64_t> ids;ids.reserve(face.p.size());for(size_t i=0;i<face.p.size();++i){CanonicalVertex v;v.id=nextVertex++;v.position=face.p[i];v.supportIds=i<face.supports.size()?face.supports[i]:std::vector<uint64_t>{face.sourcePlaneId};NormalizeSupports(v.supportIds);ids.push_back(v.id);result.canonicalVertices.push_back(std::move(v));}return ids;}
    uint64_t EmitInterface(Chunk& a,Chunk& b,const Polygon& source,uint32_t seedA,uint32_t seedB,uint32_t chunkA,uint32_t chunkB){Polygon canonical=source;CanonicalOrder(canonical);const std::vector<Vec3>& p=canonical.p;std::vector<uint64_t> ids=EmitVertices(canonical);InterfacePatch patch;patch.id=nextPatch++;patch.chunkA=std::min(chunkA,chunkB);patch.chunkB=std::max(chunkA,chunkB);patch.canonicalVertexIds=ids;patch.area=PolygonArea(canonical);for(size_t i=1;i+1<p.size();++i){patch.triangles.push_back(0);patch.triangles.push_back(uint32_t(i));patch.triangles.push_back(uint32_t(i+1));SurfaceTriangle ta{{p[0],p[i],p[i+1]},{ids[0],ids[i],ids[i+1]},FaceKind::Interior,int32_t(seedB),patch.id};SurfaceTriangle tb{{p[0],p[i+1],p[i]},{ids[0],ids[i+1],ids[i]},FaceKind::Interior,int32_t(seedA),patch.id};a.surface.push_back(ta);b.surface.push_back(tb);}result.interfacePatches.push_back(std::move(patch));return nextPatch-1;}
    void EmitExterior(Chunk& chunk,const Polygon& source){Polygon canonical=source;CanonicalOrder(canonical);const std::vector<Vec3>& p=canonical.p;std::vector<uint64_t> ids=EmitVertices(canonical);for(size_t i=1;i+1<p.size();++i)chunk.surface.push_back({{p[0],p[i],p[i+1]},{ids[0],ids[i],ids[i+1]},FaceKind::Exterior,-1,Diagnostic::InvalidId});}
};

struct BondAccum{double area=0;Vec3 weightedCenter{},weightedNormal{};};

SolidDomain NormalizeDomain(const SolidDomain& source,Vec3 origin,double scale){SolidDomain out=source;out.grid.origin=(source.grid.origin-origin)/scale;out.grid.step/=scale;for(double& phi:out.grid.phi)phi/=scale;out.grid.maxSamplingError/=scale;out.grid.minResolvedNegativeThickness/=scale;out.volume=source.volume/(scale*scale*scale);for(SolidTetPiece& p:out.pieces){p.volume/=scale*scale*scale;p.centroid=(p.centroid-origin)/scale;p.bounds.min=(p.bounds.min-origin)/scale;p.bounds.max=(p.bounds.max-origin)/scale;for(ConvexFace& f:p.faces)for(Vec3& v:f.vertices)v=(v-origin)/scale;for(SolidSimplex& s:p.samplingTetrahedra){s.volume/=scale*scale*scale;for(Vec3& v:s.vertices)v=(v-origin)/scale;}}return out;}
} // namespace

SolidDomain BuildSolidDomain(const ScalarGrid& grid,const SolidDomainSettings& settings){
    SolidDomain result;result.grid.origin=grid.origin;result.grid.step=grid.step;result.grid.nx=grid.nx;result.grid.ny=grid.ny;result.grid.nz=grid.nz;result.grid.maxSamplingError=grid.maxSamplingError;result.grid.minResolvedNegativeThickness=grid.minResolvedNegativeThickness;result.grid.negativeComponentCount=grid.negativeComponentCount;
    if(grid.nx<2||grid.ny<2||grid.nz<2||grid.phi.size()!=size_t(grid.nx)*grid.ny*grid.nz||!(grid.step>0)||!std::isfinite(grid.step)||!Finite(grid.origin)||!std::isfinite(settings.relativeTolerance)||settings.relativeTolerance<=0){Report(result.diagnostics,DiagnosticSeverity::FatalGeometry,DiagnosticStage::SolidDomain,DiagnosticCode::InvalidInput,"Solid domain received an invalid scalar grid or tolerance.");return result;}
    for(double phi:grid.phi)if(!std::isfinite(phi)){Report(result.diagnostics,DiagnosticSeverity::FatalGeometry,DiagnosticStage::SolidDomain,DiagnosticCode::NonFiniteValue,"Solid domain contains NaN/Inf.");return result;}
    double scale=std::max({grid.GetBounds().Size().x,grid.GetBounds().Size().y,grid.GetBounds().Size().z,grid.step});double eps=scale*64*DBL_EPSILON;uint64_t tetId=0;
    static constexpr uint32_t permutations[6][3]={{0,1,2},{0,2,1},{1,0,2},{1,2,0},{2,0,1},{2,1,0}};
    for(uint32_t z=0;z+1<grid.nz;++z)for(uint32_t y=0;y+1<grid.ny;++y)for(uint32_t x=0;x+1<grid.nx;++x){
        if(settings.shouldCancel&&settings.shouldCancel()){Report(result.diagnostics,DiagnosticSeverity::FatalGeometry,DiagnosticStage::Performance,DiagnosticCode::BudgetExceeded,"Solid-domain construction was cancelled.");return result;}
        Vec3 base=grid.Position(x,y,z);
        for(const auto& perm:permutations){uint64_t current=tetId++;std::array<Vec3,4> v{base,base,base,base};std::array<double,4> phi{};std::array<uint64_t,4> ids{};std::array<std::array<uint32_t,3>,4> c{};
            for(uint32_t k=1;k<4;++k){c[k]=c[k-1];++c[k][perm[k-1]];v[k]=base+Vec3(c[k][0],c[k][1],c[k][2])*grid.step;}
            for(uint32_t k=0;k<4;++k){uint32_t gx=x+c[k][0],gy=y+c[k][1],gz=z+c[k][2];phi[k]=grid.phi[grid.Index(gx,gy,gz)];ids[k]=grid.Index(gx,gy,gz);}
            bool negative=false,positive=false;for(double q:phi){negative|=q<0;positive|=q>0;}if(!negative)continue;Polyhedron solid=MakeTetra(v,ids);
            if(positive){Vec3 a=v[1]-v[0],b=v[2]-v[0],c3=v[3]-v[0];double det=Dot(a,Cross(b,c3));if(std::abs(det)<=scale*scale*scale*1e-14){Report(result.diagnostics,DiagnosticSeverity::FatalGeometry,DiagnosticStage::SolidDomain,DiagnosticCode::DegenerateTetra,"Degenerate Freudenthal tetrahedron.",Diagnostic::InvalidId,current);return result;}Vec3 grad=(Cross(b,c3)*(phi[1]-phi[0])+Cross(c3,a)*(phi[2]-phi[0])+Cross(a,b)*(phi[3]-phi[0]))/det;double length=Length(grad);if(!(length>kTiny)||!std::isfinite(length)){Report(result.diagnostics,DiagnosticSeverity::FatalGeometry,DiagnosticStage::SolidDomain,DiagnosticCode::DegenerateFieldPlane,"Degenerate scalar-field plane.",Diagnostic::InvalidId,current);return result;}if(!Clip(solid,{grad/length,(Dot(grad,v[0])-phi[0])/length,-1,SolidPlaneId(current)},eps))continue;}
            double volume=0;Vec3 center;if(!VolumeCentroid(solid,volume,center))continue;if(settings.maxPieces>0&&result.pieces.size()>=settings.maxPieces){Report(result.diagnostics,DiagnosticSeverity::FatalGeometry,DiagnosticStage::Performance,DiagnosticCode::BudgetExceeded,"SolidTetPiece budget exceeded.",Diagnostic::InvalidId,current,Diagnostic::InvalidId,double(result.pieces.size()),double(settings.maxPieces));return result;}result.pieces.push_back(ToSolidPiece(current,solid,volume,center,eps));result.volume+=volume;}
    }
    result.activeTets=result.pieces.size();result.peakTemporaryBytes=result.pieces.capacity()*sizeof(SolidTetPiece)+result.grid.phi.capacity()*sizeof(double);for(const SolidTetPiece& p:result.pieces){result.peakTemporaryBytes+=p.faces.capacity()*sizeof(ConvexFace)+p.samplingTetrahedra.capacity()*sizeof(SolidSimplex);for(const ConvexFace& f:p.faces){result.peakTemporaryBytes+=f.vertices.capacity()*sizeof(Vec3)+f.vertexSupportIds.capacity()*sizeof(std::vector<uint64_t>);for(const auto& ids:f.vertexSupportIds)result.peakTemporaryBytes+=ids.capacity()*sizeof(uint64_t);}}
    if(settings.maxTemporaryBytes>0&&result.peakTemporaryBytes>settings.maxTemporaryBytes){Report(result.diagnostics,DiagnosticSeverity::FatalGeometry,DiagnosticStage::Performance,DiagnosticCode::BudgetExceeded,"Solid-domain working set exceeded its byte budget.",Diagnostic::InvalidId,Diagnostic::InvalidId,Diagnostic::InvalidId,double(result.peakTemporaryBytes),double(settings.maxTemporaryBytes));return result;}
    if(result.pieces.empty()){Report(result.diagnostics,DiagnosticSeverity::FatalGeometry,DiagnosticStage::SolidDomain,DiagnosticCode::UnexpectedEmptyCell,"Scalar field produced no non-empty SolidTetPiece.");return result;}
    DSU components(result.pieces.size());std::unordered_map<uint64_t,std::vector<uint32_t>> faceOwners;
    for(uint32_t i=0;i<result.pieces.size();++i)for(const ConvexFace& f:result.pieces[i].faces)if(PlaneKind(f.sourcePlaneId)==kGridPlane)faceOwners[f.sourcePlaneId].push_back(i);
    for(const auto& entry:faceOwners){const auto& owners=entry.second;if(owners.size()==2)components.Join(owners[0],owners[1]);else if(owners.size()>2){Report(result.diagnostics,DiagnosticSeverity::FatalGeometry,DiagnosticStage::CanonicalTopology,DiagnosticCode::NonManifoldOutput,"A grid face belongs to more than two SolidTetPieces.",Diagnostic::InvalidId,Diagnostic::InvalidId,Diagnostic::InvalidId,double(owners.size()),2);return result;}}
    std::map<uint32_t,uint32_t> ids;for(uint32_t i=0;i<result.pieces.size();++i){uint32_t root=components.Find(i);auto [it,inserted]=ids.emplace(root,uint32_t(ids.size()));result.pieces[i].componentId=it->second;}
    return result;
}

static BakeResult BakeDomainNormalized(const SolidDomain& domain,const std::vector<Seed>& seeds,const BakeSettings& settings){
    BakeResult result;result.inputVolume=domain.volume;result.diagnostics=domain.diagnostics;for(const Diagnostic& d:domain.diagnostics)result.warnings.push_back(d.message);if(domain.HasFatal())return result;
    if(seeds.empty()){Report(result,DiagnosticSeverity::FatalGeometry,DiagnosticStage::Seed,DiagnosticCode::InvalidInput,"Bake requires at least one Seed.");return result;}for(const Seed& s:seeds)if(!Finite(s.position)||!std::isfinite(s.weight)){Report(result,DiagnosticSeverity::FatalGeometry,DiagnosticStage::Seed,DiagnosticCode::NonFiniteValue,"Bake received a non-finite Seed.");return result;}
    const double scale=std::max({domain.grid.GetBounds().Size().x,domain.grid.GetBounds().Size().y,domain.grid.GetBounds().Size().z,domain.grid.step});const double eps=std::max(settings.relativeTolerance*scale,scale*1e-12);SeedTree tree(seeds,settings.metric);std::vector<Piece> pieces;std::vector<bool> seedVolume(seeds.size(),false);std::vector<uint64_t> candidateCounts;candidateCounts.reserve(domain.pieces.size());
    for(const SolidTetPiece& source:domain.pieces){if(settings.shouldCancel&&settings.shouldCancel()){Report(result,DiagnosticSeverity::FatalGeometry,DiagnosticStage::Performance,DiagnosticCode::BudgetExceeded,"Power partition was cancelled.");UpdateStatistics(result,domain,tree,pieces,seedVolume,candidateCounts,seeds.size());return result;}std::vector<uint32_t> candidates=tree.Candidates(source,eps*eps*64);candidateCounts.push_back(candidates.size());
        for(uint32_t owner:candidates){
            if(settings.shouldCancel&&settings.shouldCancel()){Report(result,DiagnosticSeverity::FatalGeometry,DiagnosticStage::Performance,DiagnosticCode::BudgetExceeded,"Cell construction cancelled.",owner,source.tetraId);UpdateStatistics(result,domain,tree,pieces,seedVolume,candidateCounts,seeds.size());return result;}
            Polyhedron cell=ToPoly(source);bool rejected=false;
            for(uint32_t other:candidates)if(other!=owner){
                ++result.planeTestCount;
                if(settings.maxPlaneTests>0&&result.planeTestCount>settings.maxPlaneTests){Report(result,DiagnosticSeverity::FatalGeometry,DiagnosticStage::Performance,DiagnosticCode::BudgetExceeded,"Certified candidate plane-test budget exceeded.",owner,source.tetraId,Diagnostic::InvalidId,double(result.planeTestCount),double(settings.maxPlaneTests));UpdateStatistics(result,domain,tree,pieces,seedVolume,candidateCounts,seeds.size());return result;}
                // Evaluate each unordered Seed pair in the same canonical direction;
                // the other owner uses its exact negation. Review document 5.1.
                const uint32_t lo=std::min(owner,other),hi=std::max(owner,other);
                Vec3 n=settings.metric.Apply(seeds[hi].position-seeds[lo].position);
                double d=Dot(n,(seeds[lo].position+seeds[hi].position)*0.5)+(seeds[lo].weight-seeds[hi].weight)*0.5;
                const double length=Length(n);
                if(!(length>0)){Report(result,DiagnosticSeverity::FatalGeometry,DiagnosticStage::Seed,DiagnosticCode::InvalidInput,"Coincident Seeds have no unique Power plane.",owner,source.tetraId);return result;}
                n=n/length;d/=length;if(owner!=lo){n=-n;d=-d;}
                if(!Clip(cell,{n,d,int32_t(other),PowerPlaneId(lo,hi)},scale*64*DBL_EPSILON)){rejected=true;break;}
            }
            if(rejected)continue;
            double volume=0;Vec3 center;
            if(!VolumeCentroid(cell,volume,center)){Report(result,DiagnosticSeverity::FatalGeometry,DiagnosticStage::Validation,DiagnosticCode::VolumeResidual,"Nonempty Cell has nonpositive or nonfinite oriented volume.",owner,source.tetraId);return result;}
            seedVolume[owner]=true;
            // Keep positive-volume local pieces. Dropping one side here leaves
            // its neighbour's interface orphaned; merge only final Chunks.
            if(settings.maxPieces>0&&pieces.size()>=settings.maxPieces){Report(result,DiagnosticSeverity::FatalGeometry,DiagnosticStage::Performance,DiagnosticCode::BudgetExceeded,"Cell-Tet Piece budget exceeded.",owner,source.tetraId,Diagnostic::InvalidId,double(pieces.size()),double(settings.maxPieces));UpdateStatistics(result,domain,tree,pieces,seedVolume,candidateCounts,seeds.size());return result;}
            pieces.push_back({owner,source.tetraId,std::move(cell),volume,center,{}});pieces.back().bounds=PolyBounds(pieces.back().poly);
        }
    }
    for(bool has:seedVolume)if(!has)++result.emptySeedCount;if(result.emptySeedCount){Report(result,DiagnosticSeverity::FatalGeometry,DiagnosticStage::PowerPartition,DiagnosticCode::UnexpectedEmptyCell,"A generated Seed owns no solid volume.",Diagnostic::InvalidId,Diagnostic::InvalidId,Diagnostic::InvalidId,double(result.emptySeedCount),0);UpdateStatistics(result,domain,tree,pieces,seedVolume,candidateCounts,seeds.size());return result;}if(pieces.empty()){Report(result,DiagnosticSeverity::FatalGeometry,DiagnosticStage::PowerPartition,DiagnosticCode::UnexpectedEmptyCell,"Power partition produced no Cell-Tet Pieces.");UpdateStatistics(result,domain,tree,pieces,seedVolume,candidateCounts,seeds.size());return result;}result.pieceCount=pieces.size();
    using PowerKey=std::pair<uint64_t,uint64_t>;
    std::map<PowerKey,std::vector<FaceRef>> powerFaces;
    std::map<uint64_t,std::vector<FaceRef>> gridFaces;
    // Determine solid occupancy BEFORE Power clipping. A zero-field plane can
    // coincide with a grid face: one solid side is Exterior (document 5.4).
    std::map<uint64_t,std::set<uint64_t>> solidGridSides;
    for(const SolidTetPiece& source:domain.pieces)for(const ConvexFace& face:source.faces)
        if(PlaneKind(face.sourcePlaneId)==kGridPlane)solidGridSides[face.sourcePlaneId].insert(source.tetraId);
    std::vector<FaceRef> exterior;
    for(uint32_t pieceIndex=0;pieceIndex<pieces.size();++pieceIndex){
        for(uint32_t faceIndex=0;faceIndex<pieces[pieceIndex].poly.faces.size();++faceIndex){
            const Polygon& face=pieces[pieceIndex].poly.faces[faceIndex];if(PolygonArea(face)<=0)continue;
            const uint64_t kind=PlaneKind(face.sourcePlaneId);const FaceRef ref{pieceIndex,faceIndex};
            if(kind==kPowerPlane)powerFaces[{face.sourcePlaneId,pieces[pieceIndex].tetra}].push_back(ref);
            else if(kind==kGridPlane)gridFaces[face.sourcePlaneId].push_back(ref);
            else exterior.push_back(ref);
        }
    }
    DSU dsu(pieces.size());std::vector<FacePair> pairs;uint64_t refinedGridFaces=0,mixedOwnerPatches=0;
    for(auto& entry:powerFaces){auto& refs=entry.second;
        if(refs.size()!=2||pieces[refs[0].piece].seed==pieces[refs[1].piece].seed){Report(result,DiagnosticSeverity::FatalGeometry,DiagnosticStage::CanonicalTopology,DiagnosticCode::UnmatchedInterface,"Power interface does not have exactly two different owners.",Diagnostic::InvalidId,entry.first.second,Diagnostic::InvalidId,double(refs.size()),2);continue;}
        const Polygon& a=pieces[refs[0].piece].poly.faces[refs[0].face];const Polygon& b=pieces[refs[1].piece].poly.faces[refs[1].face];
        if(!GeometryMatches(a,b,eps)){Report(result,DiagnosticSeverity::FatalGeometry,DiagnosticStage::CanonicalTopology,DiagnosticCode::UnmatchedInterface,"Power-interface copies disagree geometrically.",Diagnostic::InvalidId,entry.first.second);continue;}
        pairs.push_back({refs[0],refs[1],true,true,a});
    }
    for(auto& entry:gridFaces){
        std::map<uint64_t,std::vector<FaceRef>> tetraSides;for(FaceRef ref:entry.second)tetraSides[pieces[ref.piece].tetra].push_back(ref);
        const auto expected=solidGridSides.find(entry.first);
        if(expected!=solidGridSides.end()&&expected->second.size()==1&&tetraSides.size()==1&&expected->second.contains(tetraSides.begin()->first)){
            exterior.insert(exterior.end(),entry.second.begin(),entry.second.end());continue;
        }
        if(tetraSides.size()!=2){Report(result,DiagnosticSeverity::FatalGeometry,DiagnosticStage::CanonicalTopology,DiagnosticCode::CoverageGap,"A solid grid face does not have exactly two adjacent tetrahedron partitions.",Diagnostic::InvalidId,Diagnostic::InvalidId,Diagnostic::InvalidId,double(tetraSides.size()),2);continue;}
        auto sideIterator=tetraSides.begin();const auto& sideA=sideIterator->second;const uint64_t tetA=sideIterator->first;++sideIterator;const auto& sideB=sideIterator->second;const uint64_t tetB=sideIterator->first;
        double areaA=0,areaB=0,coveredArea=0;for(FaceRef ref:sideA)areaA+=PolygonArea(pieces[ref.piece].poly.faces[ref.face]);for(FaceRef ref:sideB)areaB+=PolygonArea(pieces[ref.piece].poly.faces[ref.face]);
        uint64_t localPatchCount=0;bool hasMixedOwner=false;
        for(FaceRef a:sideA)for(FaceRef b:sideB){Polygon common=IntersectCoplanarPolygons(pieces[a.piece].poly.faces[a.face],pieces[b.piece].poly.faces[b.face],scale*64*DBL_EPSILON);double commonArea=PolygonArea(common);if(commonArea<=0)continue;coveredArea+=commonArea;++localPatchCount;bool interface=pieces[a.piece].seed!=pieces[b.piece].seed;if(interface){hasMixedOwner=true;++mixedOwnerPatches;}else dsu.Join(a.piece,b.piece);pairs.push_back({a,b,interface,false,std::move(common)});}
        const double referenceArea=std::max(areaA,areaB);const double areaTolerance=std::max(eps*eps*64,referenceArea*settings.relativeTolerance*128);const double missingArea=std::max({0.0,areaA-coveredArea,areaB-coveredArea});const double overlapArea=std::max({0.0,coveredArea-areaA,coveredArea-areaB});
        if(missingArea>areaTolerance)Report(result,DiagnosticSeverity::FatalGeometry,DiagnosticStage::CanonicalTopology,DiagnosticCode::CoverageGap,"Canonical grid-face common refinement has uncovered area.",Diagnostic::InvalidId,std::min(tetA,tetB),Diagnostic::InvalidId,missingArea,areaTolerance);
        if(overlapArea>areaTolerance)Report(result,DiagnosticSeverity::FatalGeometry,DiagnosticStage::CanonicalTopology,DiagnosticCode::CoverageOverlap,"Canonical grid-face common refinement has overlapping area.",Diagnostic::InvalidId,std::min(tetA,tetB),Diagnostic::InvalidId,overlapArea,areaTolerance);
        if(localPatchCount>2||hasMixedOwner||missingArea>eps*eps||overlapArea>eps*eps)++refinedGridFaces;
    }
    if(refinedGridFaces>0){Diagnostic recovered;recovered.severity=DiagnosticSeverity::Recovered;recovered.stage=DiagnosticStage::CanonicalTopology;recovered.code=DiagnosticCode::InternalInterface;recovered.message="Shared tetrahedron faces were rebuilt through deterministic polygon common refinement.";recovered.beforeCount=refinedGridFaces;recovered.afterCount=mixedOwnerPatches;result.diagnostics.push_back(recovered);result.warnings.push_back(recovered.message);}
    if(result.HasFatal()){UpdateStatistics(result,domain,tree,pieces,seedVolume,candidateCounts,seeds.size());return result;}
    uint64_t estimatedTriangles=0;for(FaceRef ref:exterior){const size_t count=pieces[ref.piece].poly.faces[ref.face].p.size();if(count>=3)estimatedTriangles+=count-2;}for(const FacePair& pair:pairs)if(pair.interface&&pair.canonical.p.size()>=3)estimatedTriangles+=2*(pair.canonical.p.size()-2);if(settings.maxOutputTriangles>0&&estimatedTriangles>settings.maxOutputTriangles){Report(result,DiagnosticSeverity::FatalGeometry,DiagnosticStage::Performance,DiagnosticCode::BudgetExceeded,"Predicted canonical output exceeds the triangle budget.",Diagnostic::InvalidId,Diagnostic::InvalidId,Diagnostic::InvalidId,double(estimatedTriangles),double(settings.maxOutputTriangles));UpdateStatistics(result,domain,tree,pieces,seedVolume,candidateCounts,seeds.size());return result;}
    std::map<uint32_t,double> rootVolume;for(uint32_t i=0;i<pieces.size();++i)rootVolume[dsu.Find(i)]+=pieces[i].volume;
    if(settings.minFinalChunkVolumeFraction>0){double threshold=domain.volume*settings.minFinalChunkVolumeFraction;std::vector<std::tuple<double,uint32_t,uint32_t>> edges;for(const FacePair& pair:pairs)if(pair.interface){uint32_t a=dsu.Find(pair.a.piece),b=dsu.Find(pair.b.piece);if(a==b)continue;double area=PolygonArea(pair.canonical);edges.push_back({area,a,b});}std::sort(edges.begin(),edges.end(),std::greater<>());for(auto [area,a,b]:edges){a=dsu.Find(a);b=dsu.Find(b);if(a==b)continue;if(rootVolume[a]<threshold||rootVolume[b]<threshold){double combined=rootVolume[a]+rootVolume[b];dsu.Join(a,b);uint32_t root=dsu.Find(a);rootVolume[root]=combined;}}}
    std::map<uint32_t,uint32_t> rootToChunk;for(uint32_t i=0;i<pieces.size();++i){uint32_t root=dsu.Find(i);if(!rootToChunk.contains(root)){uint32_t id=uint32_t(result.chunks.size());rootToChunk[root]=id;Chunk chunk;chunk.id=id;chunk.seedId=pieces[i].seed;chunk.bounds={{DBL_MAX,DBL_MAX,DBL_MAX},{-DBL_MAX,-DBL_MAX,-DBL_MAX}};result.chunks.push_back(std::move(chunk));}}
    std::map<uint32_t,uint32_t> components;std::vector<uint32_t> pieceChunk(pieces.size());for(uint32_t i=0;i<pieces.size();++i){Chunk& c=result.chunks[rootToChunk[dsu.Find(i)]];pieceChunk[i]=c.id;c.volume+=pieces[i].volume;c.centroid+=pieces[i].centroid*pieces[i].volume;c.bounds.min=Min(c.bounds.min,pieces[i].bounds.min);c.bounds.max=Max(c.bounds.max,pieces[i].bounds.max);}for(Chunk& c:result.chunks){c.centroid=c.centroid/c.volume;c.componentId=components[c.seedId]++;result.outputVolume+=c.volume;}
    CanonicalEmitter emitter{result};for(FaceRef ref:exterior)emitter.EmitExterior(result.chunks[pieceChunk[ref.piece]],pieces[ref.piece].poly.faces[ref.face]);std::map<std::pair<uint32_t,uint32_t>,BondAccum> bonds;
    for(const FacePair& pair:pairs){uint32_t a=pieceChunk[pair.a.piece],b=pieceChunk[pair.b.piece];if(a==b)continue;if(!pair.interface)continue;if(pair.verifyMirror&&!GeometryMatches(pieces[pair.a.piece].poly.faces[pair.a.face],pieces[pair.b.piece].poly.faces[pair.b.face],eps)){Report(result,DiagnosticSeverity::FatalGeometry,DiagnosticStage::CanonicalTopology,DiagnosticCode::UnmatchedInterface,"Canonical interface copies disagree geometrically.",Diagnostic::InvalidId,pieces[pair.a.piece].tetra);continue;}Vec3 n,c;double area=PolygonArea(pair.canonical,&n,&c);if(area<=0)continue;uint32_t lo=std::min(a,b),hi=std::max(a,b);if(a!=lo)n=-n;emitter.EmitInterface(result.chunks[a],result.chunks[b],pair.canonical,pieces[pair.a.piece].seed,pieces[pair.b.piece].seed,a,b);BondAccum& acc=bonds[{lo,hi}];acc.area+=area;acc.weightedCenter+=c*area;acc.weightedNormal+=n*area;}
    for(const auto& entry:bonds){Bond b;b.chunkA=entry.first.first;b.chunkB=entry.first.second;b.area=entry.second.area;b.centroid=entry.second.weightedCenter/b.area;b.normalAToB=Normalize(entry.second.weightedNormal);result.bonds.push_back(b);}
    for(Chunk& c:result.chunks){double radius2=0;for(const SurfaceTriangle& t:c.surface)for(Vec3 p:t.p)radius2=std::max(radius2,LengthSquared(p-c.centroid));c.radius=std::sqrt(radius2);}
    double imbalance=std::abs(result.inputVolume-result.outputVolume-result.discardedVolume);double limit=std::max(eps*eps*eps,result.inputVolume*settings.relativeTolerance*64);if(imbalance>limit)Report(result,DiagnosticSeverity::FatalGeometry,DiagnosticStage::Validation,DiagnosticCode::VolumeResidual,"Canonical reconstruction violates volume conservation.",Diagnostic::InvalidId,Diagnostic::InvalidId,Diagnostic::InvalidId,imbalance,limit);
    UpdateStatistics(result,domain,tree,pieces,seedVolume,candidateCounts,seeds.size());result.statistics.sharedFacets=pairs.size();
    if(settings.maxTemporaryBytes>0&&result.statistics.peakTemporaryBytes>settings.maxTemporaryBytes)Report(result,DiagnosticSeverity::FatalGeometry,DiagnosticStage::Performance,DiagnosticCode::BudgetExceeded,"Partition working set exceeded its byte budget.",Diagnostic::InvalidId,Diagnostic::InvalidId,Diagnostic::InvalidId,double(result.statistics.peakTemporaryBytes),double(settings.maxTemporaryBytes));if(settings.maxOutputTriangles>0&&result.statistics.outputTriangles>settings.maxOutputTriangles)Report(result,DiagnosticSeverity::FatalGeometry,DiagnosticStage::Performance,DiagnosticCode::BudgetExceeded,"Bake output triangle budget exceeded.",Diagnostic::InvalidId,Diagnostic::InvalidId,Diagnostic::InvalidId,double(result.statistics.outputTriangles),double(settings.maxOutputTriangles));return result;
}

BakeResult Bake(const SolidDomain& domain,const std::vector<Seed>& seeds,const BakeSettings& settings){
    BakeResult invalid;if(domain.HasFatal()){invalid.diagnostics=domain.diagnostics;for(const Diagnostic& d:domain.diagnostics)invalid.warnings.push_back(d.message);return invalid;}Vec3 size=domain.grid.GetBounds().Size();double scale=std::max({size.x,size.y,size.z});if(!(scale>0)||!std::isfinite(scale)){Report(invalid,DiagnosticSeverity::FatalGeometry,DiagnosticStage::Input,DiagnosticCode::InvalidInput,"Cannot create normalized coordinate contract.");return invalid;}Vec3 origin=domain.grid.GetBounds().Center();SolidDomain normalized=NormalizeDomain(domain,origin,scale);std::vector<Seed> normalizedSeeds=seeds;for(Seed& s:normalizedSeeds){s.position=(s.position-origin)/scale;s.weight/=scale*scale;}BakeSettings normalizedSettings=settings;normalizedSettings.minVolume/=scale*scale*scale;BakeResult result=BakeDomainNormalized(normalized,normalizedSeeds,normalizedSettings);result.coordinateContract={2,origin,scale,settings.metric};double areaScale=scale*scale,volumeScale=areaScale*scale;result.inputVolume*=volumeScale;result.outputVolume*=volumeScale;result.discardedVolume*=volumeScale;for(Chunk& c:result.chunks){c.centroid=origin+c.centroid*scale;c.radius*=scale;c.volume*=volumeScale;c.bounds.min=origin+c.bounds.min*scale;c.bounds.max=origin+c.bounds.max*scale;for(SurfaceTriangle& t:c.surface)for(Vec3& p:t.p)p=origin+p*scale;}for(Bond& b:result.bonds){b.area*=areaScale;b.centroid=origin+b.centroid*scale;}for(CanonicalVertex& v:result.canonicalVertices)v.position=origin+v.position*scale;for(InterfacePatch& p:result.interfacePatches)p.area*=areaScale;return result;
}

BakeResult Bake(const ScalarGrid& grid,const std::vector<Seed>& seeds,const BakeSettings& settings){SolidDomainSettings domainSettings;domainSettings.relativeTolerance=settings.relativeTolerance;domainSettings.maxPieces=settings.maxPieces;domainSettings.maxTemporaryBytes=settings.maxTemporaryBytes;domainSettings.shouldCancel=settings.shouldCancel;SolidDomain domain=BuildSolidDomain(grid,domainSettings);return Bake(domain,seeds,settings);}
} // namespace tvf
