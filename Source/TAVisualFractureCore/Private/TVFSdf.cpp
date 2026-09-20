#include "TVFFracture.h"
#include <limits>
#include <map>
#include <numeric>
#include <queue>
#include <sstream>
#include <stdexcept>
#include <tuple>
#include <unordered_map>
#include <unordered_set>

namespace tvf {
namespace {
double SegmentDistanceSquared(Vec3 p,Vec3 a,Vec3 b) {
    Vec3 ab=b-a;double d=LengthSquared(ab);
    double t=d>0?std::clamp(Dot(p-a,ab)/d,0.0,1.0):0;
    return LengthSquared(p-(a+ab*t));
}
double TriangleDistanceSquared(Vec3 p,Vec3 a,Vec3 b,Vec3 c) {
    Vec3 ab=b-a,ac=c-a,n=Cross(ab,ac);double nn=LengthSquared(n);
    if(nn>0) {
        double h=Dot(p-a,n);Vec3 q=p-n*(h/nn);
        double s0=Dot(Cross(b-a,q-a),n),s1=Dot(Cross(c-b,q-b),n),s2=Dot(Cross(a-c,q-c),n);
        if(s0>=0&&s1>=0&&s2>=0)return h*h/nn;
    }
    return std::min({SegmentDistanceSquared(p,a,b),SegmentDistanceSquared(p,b,c),SegmentDistanceSquared(p,c,a)});
}
void CheckGrid(const ScalarGrid& g) {
    if(g.nx<2||g.ny<2||g.nz<2||!std::isfinite(g.step)||g.step<=0||!Finite(g.origin)||
       g.phi.size()!=size_t(g.nx)*g.ny*g.nz)throw std::invalid_argument("Invalid ScalarGrid");
}

struct Vec2 {double x=0,y=0;};
double Orient2(Vec2 a,Vec2 b,Vec2 c){return (b.x-a.x)*(c.y-a.y)-(b.y-a.y)*(c.x-a.x);}
Vec2 Project(Vec3 p,int drop){return drop==0?Vec2{p.y,p.z}:(drop==1?Vec2{p.x,p.z}:Vec2{p.x,p.y});}
bool OnSegment(Vec2 a,Vec2 b,Vec2 p,double eps){return std::abs(Orient2(a,b,p))<=eps&&p.x>=std::min(a.x,b.x)-eps&&p.x<=std::max(a.x,b.x)+eps&&p.y>=std::min(a.y,b.y)-eps&&p.y<=std::max(a.y,b.y)+eps;}
bool SegmentsIntersect(Vec2 a,Vec2 b,Vec2 c,Vec2 d,double eps){
    double abC=Orient2(a,b,c),abD=Orient2(a,b,d),cdA=Orient2(c,d,a),cdB=Orient2(c,d,b);
    if(((abC>eps&&abD<-eps)||(abC<-eps&&abD>eps))&&((cdA>eps&&cdB<-eps)||(cdA<-eps&&cdB>eps)))return true;
    return OnSegment(a,b,c,eps)||OnSegment(a,b,d,eps)||OnSegment(c,d,a,eps)||OnSegment(c,d,b,eps);
}
bool PointInTriangle2(Vec2 p,Vec2 a,Vec2 b,Vec2 c,double eps){
    double x=Orient2(a,b,p),y=Orient2(b,c,p),z=Orient2(c,a,p);
    return !((x>eps||y>eps||z>eps)&&(x<-eps||y<-eps||z<-eps));
}
bool SegmentTriangle(Vec3 p0,Vec3 p1,Vec3 a,Vec3 b,Vec3 c,double eps){
    Vec3 dir=p1-p0,e1=b-a,e2=c-a,h=Cross(dir,e2);double det=Dot(e1,h);
    if(std::abs(det)<=eps)return false;
    double inv=1.0/det,u=Dot(p0-a,h)*inv;if(u<-eps||u>1+eps)return false;
    Vec3 q=Cross(p0-a,e1);double v=Dot(dir,q)*inv;if(v<-eps||u+v>1+eps)return false;
    double t=Dot(e2,q)*inv;return t>=-eps&&t<=1+eps;
}
bool TrianglesIntersect(Vec3 a0,Vec3 a1,Vec3 a2,Vec3 b0,Vec3 b1,Vec3 b2,double eps,bool& coplanar){
    Vec3 na=Cross(a1-a0,a2-a0),nb=Cross(b1-b0,b2-b0);double la=Length(na),lb=Length(nb);
    coplanar=false;if(la<=eps||lb<=eps)return false;
    Vec3 nua=na/la,nub=nb/lb;
    double maxPlane=std::max({std::abs(Dot(nua,b0-a0)),std::abs(Dot(nua,b1-a0)),std::abs(Dot(nua,b2-a0)),
                              std::abs(Dot(nub,a0-b0)),std::abs(Dot(nub,a1-b0)),std::abs(Dot(nub,a2-b0))});
    if(Length(Cross(nua,nub))<=eps&&maxPlane<=eps){
        coplanar=true;Vec3 an{std::abs(nua.x),std::abs(nua.y),std::abs(nua.z)};int drop=an.x>an.y?(an.x>an.z?0:2):(an.y>an.z?1:2);
        Vec2 a[3]={Project(a0,drop),Project(a1,drop),Project(a2,drop)},b[3]={Project(b0,drop),Project(b1,drop),Project(b2,drop)};
        for(int i=0;i<3;++i)for(int j=0;j<3;++j)if(SegmentsIntersect(a[i],a[(i+1)%3],b[j],b[(j+1)%3],eps))return true;
        return PointInTriangle2(a[0],b[0],b[1],b[2],eps)||PointInTriangle2(b[0],a[0],a[1],a[2],eps);
    }
    Vec3 ae[3][2]={{a0,a1},{a1,a2},{a2,a0}},be[3][2]={{b0,b1},{b1,b2},{b2,b0}};
    for(auto& e:ae)if(SegmentTriangle(e[0],e[1],b0,b1,b2,eps))return true;
    for(auto& e:be)if(SegmentTriangle(e[0],e[1],a0,a1,a2,eps))return true;
    return false;
}

void AddMeshDiagnostic(MeshDiagnostics& d,DiagnosticCode code,const char* message,uint64_t count){
    if(count==0)return;Diagnostic q;q.severity=DiagnosticSeverity::FatalGeometry;q.stage=DiagnosticStage::Input;q.code=code;q.message=message;q.measuredValue=double(count);d.diagnostics.push_back(std::move(q));
}

void FinalizeGridMetrics(ScalarGrid& g){
    g.maxSamplingError=std::sqrt(3.0)*g.step*0.5;
    std::vector<uint8_t> visited(g.phi.size(),0);double minimumComponentThickness=std::numeric_limits<double>::infinity();
    static constexpr int delta[6][3]={{1,0,0},{-1,0,0},{0,1,0},{0,-1,0},{0,0,1},{0,0,-1}};
    for(uint32_t z=0;z<g.nz;++z)for(uint32_t y=0;y<g.ny;++y)for(uint32_t x=0;x<g.nx;++x){size_t start=g.Index(x,y,z);if(visited[start]||g.phi[start]>=0)continue;++g.negativeComponentCount;double deepest=0;std::queue<std::array<uint32_t,3>> q;q.push({x,y,z});visited[start]=1;while(!q.empty()){auto p=q.front();q.pop();deepest=std::max(deepest,-g.phi[g.Index(p[0],p[1],p[2])]);for(const auto& d:delta){int64_t nx=int64_t(p[0])+d[0],ny=int64_t(p[1])+d[1],nz=int64_t(p[2])+d[2];if(nx<0||ny<0||nz<0||nx>=g.nx||ny>=g.ny||nz>=g.nz)continue;size_t i=g.Index(uint32_t(nx),uint32_t(ny),uint32_t(nz));if(!visited[i]&&g.phi[i]<0){visited[i]=1;q.push({uint32_t(nx),uint32_t(ny),uint32_t(nz)});}}}minimumComponentThickness=std::min(minimumComponentThickness,deepest*2.0);}
    g.minResolvedNegativeThickness=std::isfinite(minimumComponentThickness)?minimumComponentThickness:0;
}
}
Bounds MeshBounds(const TriangleMesh& mesh) {
    if(mesh.positions.empty())throw std::invalid_argument("Mesh has no positions");
    Bounds b{mesh.positions[0],mesh.positions[0]};
    for(Vec3 p:mesh.positions){if(!Finite(p))throw std::invalid_argument("Non-finite mesh position");b.min=Min(b.min,p);b.max=Max(b.max,p);}
    return b;
}
MeshDiagnostics ValidateClosedMesh(const TriangleMesh& mesh,double relativeWeldTolerance,const std::function<bool()>& shouldCancel) {
    MeshDiagnostics d;
    if(mesh.positions.empty()||mesh.triangles.empty()){d.message="Mesh is empty";return d;}
    if(!std::isfinite(relativeWeldTolerance)||relativeWeldTolerance<=0||relativeWeldTolerance>1e-3){d.message="Invalid weld tolerance";return d;}
    Bounds b;
    try{b=MeshBounds(mesh);}catch(const std::exception& e){d.message=e.what();return d;}
    Vec3 size=b.Size();double extent=std::max({size.x,size.y,size.z});
    if(extent<=0){d.message="Zero mesh extent";return d;}
    double eps=extent*relativeWeldTolerance;
    using Key=std::tuple<int64_t,int64_t,int64_t>;
    std::map<Key,std::vector<uint32_t>> buckets;
    std::vector<Vec3> welded;std::vector<uint32_t> ids;
    for(size_t sourceVertex=0;sourceVertex<mesh.positions.size();++sourceVertex) {
        if((sourceVertex&1023)==0&&shouldCancel&&shouldCancel()){d.message="Mesh validation cancelled";AddMeshDiagnostic(d,DiagnosticCode::BudgetExceeded,"Mesh validation cancelled",1);return d;}
        Vec3 p=mesh.positions[sourceVertex];
        Vec3 f=(p-b.min)/eps;int64_t x=int64_t(std::floor(f.x)),y=int64_t(std::floor(f.y)),z=int64_t(std::floor(f.z));
        uint32_t found=uint32_t(-1);
        for(int iz=-1;iz<=1;++iz)for(int iy=-1;iy<=1;++iy)for(int ix=-1;ix<=1;++ix) {
            auto it=buckets.find({x+ix,y+iy,z+iz});if(it==buckets.end())continue;
            for(uint32_t id:it->second)if(LengthSquared(welded[id]-p)<=eps*eps)found=std::min(found,id);
        }
        if(found==uint32_t(-1)){found=uint32_t(welded.size());welded.push_back(p);buckets[{x,y,z}].push_back(found);}
        ids.push_back(found);
    }
    struct Edge {uint32_t count=0;int orientation=0;std::vector<uint32_t> triangles;};
    std::map<std::pair<uint32_t,uint32_t>,Edge> edges;
    std::vector<std::array<uint32_t,3>> weldedTriangles;weldedTriangles.reserve(mesh.triangles.size());
    std::vector<std::vector<uint32_t>> incidentTriangles(welded.size());
    for(uint32_t triangleIndex=0;triangleIndex<mesh.triangles.size();++triangleIndex) {
        if((triangleIndex&1023)==0&&shouldCancel&&shouldCancel()){d.message="Mesh validation cancelled";AddMeshDiagnostic(d,DiagnosticCode::BudgetExceeded,"Mesh validation cancelled",1);return d;}
        auto tri=mesh.triangles[triangleIndex];
        for(uint32_t i:tri)if(i>=ids.size()){d.message="Triangle index out of bounds";return d;}
        uint32_t a=ids[tri[0]],bb=ids[tri[1]],c=ids[tri[2]];
        weldedTriangles.push_back({a,bb,c});
        incidentTriangles[a].push_back(triangleIndex);incidentTriangles[bb].push_back(triangleIndex);incidentTriangles[c].push_back(triangleIndex);
        Vec3 ab=welded[bb]-welded[a],ac=welded[c]-welded[a];
        if(a==bb||bb==c||c==a||LengthSquared(Cross(ab,ac))<=eps*eps*std::max(LengthSquared(ab),LengthSquared(ac)))++d.degenerateTriangles;
        std::array<uint32_t,3> t{a,bb,c};
        for(int i=0;i<3;++i){uint32_t u=t[i],v=t[(i+1)%3];auto& e=edges[{std::min(u,v),std::max(u,v)}];++e.count;e.orientation+=(u<v?1:-1);e.triangles.push_back(triangleIndex);}
    }
    for(auto kv:edges){if(kv.second.count==1)++d.boundaryEdges;else if(kv.second.count!=2)++d.nonManifoldEdges;else if(kv.second.orientation!=0)++d.inconsistentEdges;}

    // A manifold vertex has one connected fan in its link. Multiple fans are a bow-tie even when every edge has incidence two.
    for(uint32_t vertex=0;vertex<incidentTriangles.size();++vertex){
        const auto& incident=incidentTriangles[vertex];if(incident.size()<2)continue;
        std::unordered_map<uint32_t,std::vector<uint32_t>> adjacency;
        for(uint32_t ti:incident){
            const auto& t=weldedTriangles[ti];
            for(uint32_t v:t)if(v!=vertex){auto it=edges.find({std::min(vertex,v),std::max(vertex,v)});if(it!=edges.end())for(uint32_t other:it->second.triangles)if(other!=ti)adjacency[ti].push_back(other);}
        }
        std::unordered_set<uint32_t> visited;std::queue<uint32_t> pending;pending.push(incident.front());visited.insert(incident.front());
        while(!pending.empty()){uint32_t a=pending.front();pending.pop();for(uint32_t other:adjacency[a])if(visited.insert(other).second)pending.push(other);}
        if(visited.size()!=incident.size()){++d.disconnectedVertexLinks;++d.bowTieVertices;}
    }

    // Broad phase sorted on X; the narrow phase explicitly includes coplanar overlap.
    struct TriBounds{uint32_t id;Bounds b;};std::vector<TriBounds> triBounds;triBounds.reserve(weldedTriangles.size());
    for(uint32_t i=0;i<weldedTriangles.size();++i){auto t=weldedTriangles[i];Vec3 a=welded[t[0]],bb=welded[t[1]],c=welded[t[2]];triBounds.push_back({i,{Min(a,Min(bb,c)),Max(a,Max(bb,c))}});}
    std::sort(triBounds.begin(),triBounds.end(),[](const TriBounds& a,const TriBounds& b){return a.b.min.x<b.b.min.x;});
    const double pairEps=std::max(eps,extent*1e-12);
    for(size_t ii=0;ii<triBounds.size();++ii)for(size_t jj=ii+1;jj<triBounds.size()&&triBounds[jj].b.min.x<=triBounds[ii].b.max.x+pairEps;++jj){
        if(((jj-ii)&4095)==0&&shouldCancel&&shouldCancel()){d.message="Mesh intersection validation cancelled";AddMeshDiagnostic(d,DiagnosticCode::BudgetExceeded,"Mesh intersection validation cancelled",1);return d;}
        const auto& a=triBounds[ii];const auto& bb=triBounds[jj];
        if(a.b.max.y<bb.b.min.y-pairEps||bb.b.max.y<a.b.min.y-pairEps||a.b.max.z<bb.b.min.z-pairEps||bb.b.max.z<a.b.min.z-pairEps)continue;
        auto ta=weldedTriangles[a.id],tb=weldedTriangles[bb.id];bool adjacent=false;for(uint32_t va:ta)for(uint32_t vb:tb)adjacent|=va==vb;if(adjacent)continue;
        bool coplanar=false;if(TrianglesIntersect(welded[ta[0]],welded[ta[1]],welded[ta[2]],welded[tb[0]],welded[tb[1]],welded[tb[2]],pairEps,coplanar)){++d.selfIntersections;if(coplanar)++d.coplanarOverlaps;}
    }

    // Triangle components define shells. Strict mode permits multiple disjoint shells; negative shells are reported as cavities by orientation.
    std::vector<uint32_t> parent(weldedTriangles.size());std::iota(parent.begin(),parent.end(),0);
    auto find=[&](uint32_t x){while(parent[x]!=x){parent[x]=parent[parent[x]];x=parent[x];}return x;};
    auto join=[&](uint32_t a,uint32_t b){a=find(a);b=find(b);if(a!=b)parent[b]=a;};
    for(const auto& kv:edges)if(kv.second.triangles.size()==2)join(kv.second.triangles[0],kv.second.triangles[1]);
    std::unordered_map<uint32_t,double> shellVolume;std::unordered_map<uint32_t,std::vector<uint32_t>> shellTriangles;
    for(uint32_t i=0;i<weldedTriangles.size();++i){auto t=weldedTriangles[i];uint32_t root=find(i);shellVolume[root]+=Dot(welded[t[0]],Cross(welded[t[1]],welded[t[2]]))/6.0;shellTriangles[root].push_back(i);}
    d.connectedShells=shellVolume.size();for(const auto& kv:shellVolume)if(kv.second<0)++d.nestedCavities;
    size_t ambiguousCavities=0;
    for(const auto& cavity:shellVolume)if(cavity.second<0){
        const auto& first=weldedTriangles[shellTriangles[cavity.first].front()];Vec3 p=welded[first[0]];bool contained=false;
        for(const auto& outer:shellVolume)if(outer.second>0){
            double sum=0,correction=0;for(uint32_t ti:shellTriangles[outer.first]){auto t=weldedTriangles[ti];Vec3 a=welded[t[0]]-p,bb=welded[t[1]]-p,c=welded[t[2]]-p;double la=Length(a),lb=Length(bb),lc=Length(c);if(la<=pairEps||lb<=pairEps||lc<=pairEps){contained=true;break;}a=a/la;bb=bb/lb;c=c/lc;double angle=2*std::atan2(Dot(a,Cross(bb,c)),1+Dot(a,bb)+Dot(bb,c)+Dot(c,a));double y=angle-correction,q=sum+y;correction=(q-sum)-y;sum=q;}if(contained||std::abs(sum)>2*Pi){contained=true;break;}
        }
        if(!contained)++ambiguousCavities;
    }

    AddMeshDiagnostic(d,DiagnosticCode::BoundaryEdge,"Closed Volume has boundary edges",d.boundaryEdges);
    AddMeshDiagnostic(d,DiagnosticCode::NonManifoldEdge,"Closed Volume has non-manifold edges",d.nonManifoldEdges);
    AddMeshDiagnostic(d,DiagnosticCode::InconsistentWinding,"Closed Volume has inconsistent edge winding",d.inconsistentEdges);
    AddMeshDiagnostic(d,DiagnosticCode::DegenerateTriangle,"Closed Volume has degenerate triangles",d.degenerateTriangles);
    AddMeshDiagnostic(d,DiagnosticCode::DisconnectedVertexLink,"Closed Volume has disconnected vertex links",d.disconnectedVertexLinks);
    AddMeshDiagnostic(d,DiagnosticCode::SelfIntersection,"Closed Volume has non-adjacent triangle intersections",d.selfIntersections-d.coplanarOverlaps);
    AddMeshDiagnostic(d,DiagnosticCode::CoplanarOverlap,"Closed Volume has coplanar triangle overlap",d.coplanarOverlaps);
    AddMeshDiagnostic(d,DiagnosticCode::AmbiguousShellNesting,"A negative shell is not contained by a positive outer shell",ambiguousCavities);
    d.valid=d.diagnostics.empty();
    std::ostringstream ss;ss<<(d.valid?"Closed Volume passed edge, vertex-link and triangle-intersection validation":"Invalid Closed Volume")
       <<"; boundary="<<d.boundaryEdges<<", nonManifold="<<d.nonManifoldEdges<<", orientation="<<d.inconsistentEdges<<", degenerate="<<d.degenerateTriangles
       <<", vertexLink="<<d.disconnectedVertexLinks<<", selfIntersection="<<d.selfIntersections<<", coplanar="<<d.coplanarOverlaps<<", shells="<<d.connectedShells;
    d.message=ss.str();return d;
}
double SignedDistanceToMesh(const TriangleMesh& mesh,Vec3 p) {
    if(!Finite(p)||mesh.triangles.empty())throw std::invalid_argument("Invalid distance query");
    double nearest=std::numeric_limits<double>::infinity(),sum=0,correction=0;
    for(auto tri:mesh.triangles) {
        if(tri[0]>=mesh.positions.size()||tri[1]>=mesh.positions.size()||tri[2]>=mesh.positions.size())throw std::invalid_argument("Invalid triangle index");
        Vec3 a=mesh.positions[tri[0]],b=mesh.positions[tri[1]],c=mesh.positions[tri[2]];
        nearest=std::min(nearest,TriangleDistanceSquared(p,a,b,c));
        if(nearest==0)return 0;
        a=a-p;b=b-p;c=c-p;
        double la=Length(a),lb=Length(b),lc=Length(c);
        // Unit vectors keep solid-angle arithmetic stable under a change of scale.
        if(la==0||lb==0||lc==0)return 0;
        a=a/la;b=b/lb;c=c/lc;
        double angle=2*std::atan2(Dot(a,Cross(b,c)),1+Dot(a,b)+Dot(b,c)+Dot(c,a));
        double y=angle-correction,t=sum+y;correction=(t-sum)-y;sum=t;
    }
    double distance=std::sqrt(nearest);
    return std::abs(sum)>2*Pi?-distance:distance;
}
ScalarGrid SampleField(Bounds domain,uint32_t longestAxisCells,const std::function<double(Vec3)>& field) {
    Vec3 size=domain.Size();double extent=std::max({size.x,size.y,size.z});
    if(!Finite(domain.min)||!Finite(domain.max)||size.x<=0||size.y<=0||size.z<=0||longestAxisCells<2||longestAxisCells>256||!field)
        throw std::invalid_argument("SampleField needs finite positive bounds, field and 2..256 cells");
    ScalarGrid g;g.origin=domain.min;g.step=extent/longestAxisCells;
    g.nx=uint32_t(std::ceil(size.x/g.step))+1;g.ny=uint32_t(std::ceil(size.y/g.step))+1;g.nz=uint32_t(std::ceil(size.z/g.step))+1;
    size_t count=size_t(g.nx)*g.ny*g.nz;if(count>18000000)throw std::invalid_argument("Grid vertex budget exceeded");
    g.phi.resize(count);
    for(uint32_t z=0;z<g.nz;++z)for(uint32_t y=0;y<g.ny;++y)for(uint32_t x=0;x<g.nx;++x){double v=field(g.Position(x,y,z));if(!std::isfinite(v))throw std::invalid_argument("Field returned NaN/Inf");g.phi[g.Index(x,y,z)]=v;}
    FinalizeGridMetrics(g);
    return g;
}
void AnalyzeScalarGrid(ScalarGrid& grid){CheckGrid(grid);FinalizeGridMetrics(grid);}
ScalarGrid MeshToGrid(const TriangleMesh& mesh,uint32_t cells) {
    auto validation=ValidateClosedMesh(mesh);if(!validation.valid)throw std::invalid_argument(validation.message);
    if(cells<4||cells>256)throw std::invalid_argument("MeshToGrid cells must be 4..256");
    Bounds b=MeshBounds(mesh);Vec3 s=b.Size();double pad=std::max({s.x,s.y,s.z})*2.0/cells;
    b.min-=Vec3(pad,pad,pad);b.max+=Vec3(pad,pad,pad);
    double step=std::max({s.x,s.y,s.z})*(1+4.0/cells)/cells;
    if(std::min({s.x,s.y,s.z})<2*step)throw std::runtime_error("Mesh bounding thickness is below two grid cells; increase resolution or give thickness");
    auto g=SampleField(b,cells,[&](Vec3 p){return SignedDistanceToMesh(mesh,p);});
    if(std::none_of(g.phi.begin(),g.phi.end(),[](double p){return p<0;}))throw std::runtime_error("No solid grid samples; mesh thickness is unresolved");
    return g;
}
double SampleGrid(const ScalarGrid& g,Vec3 p) {
    CheckGrid(g);if(!Finite(p))throw std::invalid_argument("Non-finite sample position");
    Vec3 q=(p-g.origin)/g.step;
    if(q.x<0||q.y<0||q.z<0||q.x>g.nx-1||q.y>g.ny-1||q.z>g.nz-1) {
        Vec3 clamped{std::clamp(q.x,0.0,double(g.nx-1)),std::clamp(q.y,0.0,double(g.ny-1)),std::clamp(q.z,0.0,double(g.nz-1))};
        return Length(q-clamped)*g.step+g.step*1e-9;
    }
    uint32_t x=std::min(uint32_t(q.x),g.nx-2),y=std::min(uint32_t(q.y),g.ny-2),z=std::min(uint32_t(q.z),g.nz-2);
    const double f[3]={q.x-x,q.y-y,q.z-z};
    std::array<int,3> order{0,1,2};
    std::stable_sort(order.begin(),order.end(),[&](int a,int b){return f[a]>f[b];});
    const double w[4]={1-f[order[0]],f[order[0]]-f[order[1]],f[order[1]]-f[order[2]],f[order[2]]};
    uint32_t xyz[3]={x,y,z};double v=w[0]*g.phi[g.Index(x,y,z)];
    for(int k=0;k<3;++k){++xyz[order[k]];v+=w[k+1]*g.phi[g.Index(xyz[0],xyz[1],xyz[2])];}
    return v;
}
}
