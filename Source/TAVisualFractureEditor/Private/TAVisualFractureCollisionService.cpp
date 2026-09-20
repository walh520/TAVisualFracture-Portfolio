#include "TAVisualFractureCollisionService.h"
#include "TAVisualFractureComponent.h"
#include "TAVisualFractureColliderComponent.h"
#include "TAVisualFractureBakeService.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "GameFramework/Actor.h"
#include "PhysicsEngine/BodySetup.h"
#include "UObject/UnrealType.h"
#include "UObject/UObjectIterator.h"
#include "CompGeom/ConvexDecomposition3.h"
#include "CompGeom/ConvexHull3.h"
#include "DynamicMesh/Operations/MergeCoincidentMeshEdges.h"
#include "DynamicMesh/DynamicMeshAABBTree3.h"
#include "DrawDebugHelpers.h"
#include "Misc/ScopedSlowTask.h"
#include "Misc/SecureHash.h"
#include <map>
#include <set>
#include <tuple>
using namespace UE::Geometry;
namespace {
tvf::Vec3 V(FVector p){return {p.X,p.Y,p.Z};}
FVector U(tvf::Vec3 p){return {p.x,p.y,p.z};}
FString SettingsText(const UTAVisualFractureComponent* c){FString text;FTVFVisualCollisionSettings::StaticStruct()->ExportText(text,&c->VisualCollision,nullptr,nullptr,PPF_None,nullptr);return text;}
FString ReflectedSettingsText(UObject* object){
    FString text;
    for(TFieldIterator<FProperty> it(object->GetClass(),EFieldIteratorFlags::ExcludeSuper);it;++it){
        const FProperty* property=*it;
        if(property->HasAnyPropertyFlags(CPF_Transient|CPF_DuplicateTransient))continue;
        FString value;
        property->ExportText_InContainer(0,value,object,object,object,PPF_None);
        text+=property->GetName()+TEXT("=")+value+TEXT(";");
    }
    return text;
}
void HullStats(tvf::CollisionBody& b){b.minThickness=1e300;b.radius=0;for(const auto& h:b.hulls){for(tvf::Vec3 p:h.vertices)b.radius=std::max(b.radius,tvf::Length(p));for(const auto& f:h.faces){tvf::Vec3 n=tvf::Normalize(tvf::CollisionFaceAreaVector(h,f));double lo=1e300,hi=-lo;for(tvf::Vec3 p:h.vertices){double d=tvf::Dot(p,n);lo=std::min(lo,d);hi=std::max(hi,d);}b.minThickness=std::min(b.minThickness,hi-lo);}}}
bool External(UTAVisualFractureComponent* c,std::vector<tvf::StaticCollisionBody>* out,FString& key,FString& error){
    TArray<UTAVisualFractureColliderComponent*> registered;
    for(TObjectIterator<UTAVisualFractureColliderComponent> it;it;++it)if(IsValid(*it)&&!it->IsTemplate()&&it->IsRegistered()&&it->GetOwner()&&it->GetWorld()==c->GetWorld())registered.Add(*it);
    registered.Sort([](const auto& a,const auto& b){return a.GetPathName()<b.GetPathName();});
    for(auto* reg:registered){if(reg->GetOwner()==c->GetOwner())continue;
        const FString properties=ReflectedSettingsText(reg); // stable reflected component configuration
        key+=reg->GetPathName()+properties;
        if(!c->VisualCollision.bExternalBoxCollision||!reg->bEnabled)continue;
        UStaticMeshComponent* target=reg->TargetMeshComponent?reg->TargetMeshComponent.Get():reg->GetOwner()->FindComponentByClass<UStaticMeshComponent>();
        if(!target||!target->GetStaticMesh()){error=reg->GetPathName()+TEXT(": no target Static Mesh");return false;}
        const FTransform transform=target->GetComponentTransform();FVector scale=transform.GetScale3D();if(transform.ContainsNaN()||scale.GetMin()<=0){error=TEXT("Registered collider requires finite positive scale");return false;}
        key+=target->GetPathName()+transform.ToString()+target->GetStaticMesh()->GetPathName();
        TArray<FTVFManualCollisionBox> boxes=reg->ManualBoxes;
        if(!reg->bUseManualBoxes){boxes.Reset();UBodySetup* setup=target->GetStaticMesh()->GetBodySetup();if(setup){key+=setup->BodySetupGuid.ToString();for(const FKBoxElem& box:setup->AggGeom.BoxElems){FTVFManualCollisionBox& b=boxes.AddDefaulted_GetRef();b.Center=box.Center;b.Rotation=box.Rotation;b.HalfExtent=FVector(box.X,box.Y,box.Z)*.5;}}}
        if(boxes.IsEmpty()){error=reg->GetPathName()+TEXT(": no BoxElems; add Box simple collision or choose manual boxes");return false;}
        for(int32 boxIndex=0;boxIndex<boxes.Num();++boxIndex){const auto& box=boxes[boxIndex];key+=box.Center.ToString()+box.Rotation.ToString()+box.HalfExtent.ToString();if(box.HalfExtent.ContainsNaN()||box.HalfExtent.GetMin()<=0){error=TEXT("Invalid registered Box extents");return false;}if(!out)continue;
            tvf::StaticCollisionBody body;
            body.stableId=TCHAR_TO_UTF8(*(reg->GetPathName()+TEXT("|")+target->GetPathName()+FString::Printf(TEXT("|%d"),boxIndex)));
            FString geometry=target->GetStaticMesh()->GetPathName()+box.Center.ToString()+box.Rotation.ToString()+box.HalfExtent.ToString()+transform.GetScale3D().ToString();
            if(auto* setup=target->GetStaticMesh()->GetBodySetup())geometry+=setup->BodySetupGuid.ToString();
            body.geometryKey=TCHAR_TO_UTF8(*geometry);
            auto hull=tvf::MakeCollisionBox(V(box.HalfExtent));body.position=V(transform.TransformPosition(box.Center));for(auto& p:hull.vertices)p=V(transform.TransformPosition(box.Center+box.Rotation.RotateVector(U(p))))-body.position;
            body.body.hulls.push_back(std::move(hull));body.body.friction=reg->Friction;body.body.restitution=reg->Restitution;body.body.mask=uint32(reg->CollisionMask);HullStats(body.body);out->push_back(std::move(body));
        }
    }return true;
}
// Merge coplanar hull triangles into polygon faces for stable face-face manifolds.
bool ConvertHull(const FDynamicMesh3& mesh,tvf::CollisionHull& out,FString& error){
    std::map<int,uint32_t> ids;std::vector<tvf::Vec3> points;
    std::set<int> used;for(int id:mesh.TriangleIndicesItr()){auto t=mesh.GetTriangle(id);used.insert({t.A,t.B,t.C});}
    for(int id:used){ids[id]=uint32_t(points.size());auto p=mesh.GetVertex(id);points.push_back({p.X,p.Y,p.Z});}
    std::vector<std::array<uint32_t,3>> triangles;for(int id:mesh.TriangleIndicesItr()){auto t=mesh.GetTriangle(id);triangles.push_back({ids[t.A],ids[t.B],ids[t.C]});}
    std::string reason;if(!tvf::BuildCollisionHullFromTriangles(points,triangles,out,reason)){error=UTF8_TO_TCHAR(reason.c_str());return false;}return true;
}
// Recompute the closed hull, never remove individual bad faces (which opens it).
// Cleanup is bounded to 1% of the user's surface tolerance, and every result
// still has to pass the original surface, volume and frozen-overlap checks.
bool BuildVisualHull(const FDynamicMesh3& mesh,double cleanupCm,tvf::CollisionHull& out,FString& error,int visualVertexBudget=0){
    const auto bounds=mesh.GetBounds();const double scale=bounds.DiagonalLength();
    if(!FMath::IsFinite(scale)||scale<=0){error=TEXT("Convex construction: invalid extent");return false;}
    const FVector3d origin=bounds.Center();TArray<FVector3d> points;
    std::set<int> used;for(int id:mesh.TriangleIndicesItr()){auto t=mesh.GetTriangle(id);used.insert({t.A,t.B,t.C});}
    for(int id:used)points.Add(mesh.GetVertex(id));
    TConvexHull3<double> solver;
    if(visualVertexBudget>0)solver.SimplificationSettings.MaxHullVertices=visualVertexBudget;
    if(cleanupCm>0){solver.SimplificationSettings.DegenerateEdgeTolerance=cleanupCm/scale;solver.SimplificationSettings.SkipAtHullDistanceAbsolute=cleanupCm/scale;}
    if(!solver.Solve(points.Num(),[&](int32 id){return (points[id]-origin)/scale;})||!solver.IsSolutionAvailable()){error=TEXT("Convex construction: points do not span a solid");return false;}
    FDynamicMesh3 closed;for(auto p:points)closed.AppendVertex(p);
    for(auto t:solver.GetTriangles())if(closed.AppendTriangle(t)<0){error=TEXT("Convex construction: invalid hull triangle connectivity");return false;}
    if(visualVertexBudget<=0)return ConvertHull(closed,out,error);
    // Keep the shared indexed triangulation; no independent face-boundary merging.
    out={};std::map<int,uint32_t> remap;tvf::Vec3 center{};
    for(auto t:solver.GetTriangles())for(int id:{t.A,t.B,t.C})if(!remap.count(id)){remap[id]=uint32_t(out.vertices.size());out.vertices.push_back(V(points[id]));center+=out.vertices.back();}
    center=center/double(out.vertices.size());
    for(auto t:solver.GetTriangles()){std::vector<uint32_t> face={remap[t.A],remap[t.B],remap[t.C]};if(tvf::Dot(tvf::CollisionFaceAreaVector(out,face),center-out.vertices[face[0]])>0)std::swap(face[1],face[2]);out.faces.push_back(std::move(face));}
    std::string reason;if(!tvf::ValidateCollisionHull(out,reason)){error=UTF8_TO_TCHAR(reason.c_str());return false;}return true;
}
double HullVolume(const tvf::CollisionHull& h){double volume=0;for(const auto& f:h.faces)for(size_t i=1;i+1<f.size();++i)volume+=tvf::Dot(h.vertices[f[0]],tvf::Cross(h.vertices[f[i]],h.vertices[f[i+1]]))/6;return volume;}
bool Quality(const FDynamicMesh3& source,const std::vector<tvf::CollisionHull>& hulls,double volume,double tolerance,double relative,FString& error){
    FDynamicMeshAABBTree3 sourceTree(&source,true);FDynamicMesh3 shell;double proxyVolume=0;std::string coreError;
    for(const auto& h:hulls){if(!tvf::ValidateCollisionHull(h,coreError)){error=UTF8_TO_TCHAR(coreError.c_str());return false;}proxyVolume+=HullVolume(h);int base=shell.MaxVertexID();for(auto p:h.vertices)shell.AppendVertex(U(p));for(const auto& f:h.faces)for(size_t i=1;i+1<f.size();++i)shell.AppendTriangle(base+f[0],base+f[i+1],base+f[i]);}
    if(std::abs(proxyVolume-volume)>volume*relative){error=TEXT("Proxy volume deviation exceeds tolerance");return false;}
    FDynamicMeshAABBTree3 proxyTree(&shell,true);
    auto sample=[&](const FDynamicMesh3& from,const FDynamicMeshAABBTree3& to){for(int id:from.VertexIndicesItr()){double distance;int near=to.FindNearestTriangle(from.GetVertex(id),distance);if(near<0||distance>tolerance*tolerance)return false;}for(int id:from.TriangleIndicesItr()){FVector3d a,b,c;from.GetTriVertices(id,a,b,c);double distance;if(to.FindNearestTriangle((a+b+c)/3,distance)<0||distance>tolerance*tolerance)return false;}return true;};
    if(!sample(source,proxyTree)){error=TEXT("Source-to-proxy surface sample deviation exceeds tolerance");return false;}
    // Internal compound faces are not part of the union boundary. Sampling
    // them against the source would reject every useful concave decomposition.
    auto inside=[](const tvf::CollisionHull& h,tvf::Vec3 point){for(const auto& f:h.faces){auto n=tvf::Normalize(tvf::CollisionFaceAreaVector(h,f));if(tvf::Dot(n,point-h.vertices[f[0]])>1e-8)return false;}return true;};
    for(size_t hi=0;hi<hulls.size();++hi){const auto& h=hulls[hi];for(const auto& f:h.faces){auto n=tvf::Normalize(tvf::CollisionFaceAreaVector(h,f));std::vector<tvf::Vec3> samples;tvf::Vec3 center{};for(auto index:f){samples.push_back(h.vertices[index]);center+=h.vertices[index];}samples.push_back(center/double(f.size()));for(auto p:samples){bool internal=false;for(size_t other=0;other<hulls.size();++other)if(other!=hi&&inside(hulls[other],p+n*1e-6)){internal=true;break;}if(internal)continue;double distance;if(sourceTree.FindNearestTriangle(U(p),distance)<0||distance>tolerance*tolerance){error=TEXT("Proxy union-to-source surface sample deviation exceeds tolerance");return false;}}}}
    return true;
}
}
bool TVFCollisionEditor::Signature(UTAVisualFractureComponent* c,FString& out,FString& error){
    if(!c||!c->BakedAsset){error=TEXT("Bake asset required");return false;}
    FString key=TEXT("GroundXY9|")+SettingsText(c)+c->BakedAsset->GetPathName()+c->BakedAsset->BakeSignature+
        c->BakedAsset->SourceGeometryHash+c->BakedAsset->TopologySettingsHash+c->BakedAsset->AppliedPositiveScale.ToString()+
        FString::Printf(TEXT("|Physics=%d,%d"),c->PhysicsBudget.MaxActiveChunkStates,c->PhysicsBudget.MaxGroundQueriesPerStep);
    if(!c->VisualCollision.bSimpleVisual&&!External(c,nullptr,key,error))return false;out=FMD5::HashAnsiString(*key);return true;
}
bool TVFCollisionEditor::RefreshExternal(UTAVisualFractureComponent* c,FTVFCollisionPreparation& prepared,FString& error){
    error.Reset();if(!prepared.Scene.settings.groundOnly)return true;
    FString key;
    if(!External(c,nullptr,key,error)){prepared.Scene.ReplaceEnvironment({});prepared.ExternalSignature.Reset();return false;}
    const FString signature=FMD5::HashAnsiString(*key);
    if(signature==prepared.ExternalSignature){prepared.Scene.SampleEnvironment(prepared.Scene.environment,FPlatformTime::Seconds());return true;}
    std::vector<tvf::StaticCollisionBody> updated;key.Reset();
    if(!External(c,&updated,key,error)){prepared.Scene.ReplaceEnvironment({});prepared.ExternalSignature.Reset();return false;}
    // Only replace obstacle geometry. Never reset Motion, bonds, events or PMC poses.
    prepared.Scene.SampleEnvironment(std::move(updated),FPlatformTime::Seconds());
    prepared.ExternalSignature=FMD5::HashAnsiString(*key);return true;
}
bool TVFCollisionEditor::Prepare(UTAVisualFractureComponent* c,FTVFCollisionPreparation& out,FString& error){
    const double start=FPlatformTime::Seconds();out={};if(!Signature(c,out.Signature,error))return false;
    const auto& s=c->VisualCollision;auto& scene=out.Scene;scene.settings.selfCollision=s.bSelfCollision;scene.settings.externalCollision=s.bExternalBoxCollision;scene.settings.substeps=s.Substeps;scene.settings.positionIterations=s.PositionIterations;scene.settings.velocityIterations=s.VelocityIterations;scene.settings.contactOffset=s.ContactOffsetCm;scene.settings.restOffset=s.RestOffsetCm;scene.settings.allowedPenetration=s.AllowedPenetrationCm;scene.settings.compliance=s.Compliance;scene.settings.maxPairs=s.MaxCandidatePairs;scene.settings.maxShapePairs=s.MaxShapePairs;scene.settings.maxContacts=s.MaxContactPoints;
    if(!s.bSimpleVisual&&(s.MaxHullsPerChunk<1||s.MaxVerticesPerHull<4||s.Substeps<1||s.Substeps>8||s.PositionIterations<1||s.VelocityIterations<1||s.MaxCandidatePairs<1||s.MaxShapePairs<1||s.MaxContactPoints<1||c->PhysicsBudget.MaxGroundQueriesPerStep<1||!FMath::IsFinite(s.SurfaceToleranceCm)||!FMath::IsFinite(s.RelativeSurfaceTolerance)||!FMath::IsFinite(s.RelativeVolumeTolerance)||s.SurfaceToleranceCm<0||s.RelativeSurfaceTolerance<0||s.RelativeVolumeTolerance<0)){error=TEXT("Invalid proxy/solver settings");return false;}
    scene.settings.maxGroundQueries=c->PhysicsBudget.MaxGroundQueriesPerStep;
    scene.settings.simpleVisual=s.bSimpleVisual;
    scene.settings.groundOnly=s.bSimpleVisual;
    if(s.bSimpleVisual){
        if(!FMath::IsFinite(s.GroundAvoidanceSpeedCmPerSecond)||s.GroundAvoidanceSpeedCmPerSecond<0||s.GroundAvoidanceIterations<1||!(s.XYFootprintScale>0)||!FMath::IsFinite(s.XYFootprintScale)||!FMath::IsFinite(s.GroundRollStrength)||s.GroundRollStrength<0){error=TEXT("Invalid ground avoidance settings");return false;}
        scene.settings.avoidanceSpeed=s.GroundAvoidanceSpeedCmPerSecond;scene.settings.avoidanceIterations=s.GroundAvoidanceIterations;scene.settings.xyFootprintScale=s.XYFootprintScale;scene.settings.groundVisualRoll=s.bGroundVisualRoll;scene.settings.groundRollStrength=s.GroundRollStrength;
        scene.settings.externalVelocityTransfer=s.ExternalVelocityTransfer;scene.settings.externalLiftRatio=s.ExternalLiftRatio;scene.settings.externalLiftThreshold=s.ExternalLiftThreshold;scene.settings.groundFrictionDecay=s.GroundFrictionDecay;
        out.VisualWarnings=TEXT(" GroundXY: airborne collisions off; configurable XY correction; moving Boxes; no pair budget; visual rolling; no stacking.");
    }
    FScopedSlowTask progress(c->BakedAsset->Chunks.Num(),NSLOCTEXT("TVF","PrepareCollision","准备独立碰撞代理（不修改烘焙资产）"));progress.MakeDialog(true);
    std::vector<double> tolerances;
    for(const auto& chunk:c->BakedAsset->Chunks){progress.EnterProgressFrame(1);if(progress.ShouldCancel()){error=TEXT("Collision preparation cancelled");return false;}
        if(s.bSimpleVisual){
            // Occupancy only: read bounds/radius, never build a mesh or convex hull.
            tvf::CollisionBody body;body.mass=chunk.VolumeCm3*c->ObjectResponse.MassPerCm3;body.restitution=c->ObjectResponse.Restitution;
            body.radius=chunk.RadiusCm;body.mask=uint32(s.CollisionMask);bool any=false;
            for(const auto* section:{&chunk.Exterior,&chunk.Interior}){
                if(section->Triangles.Num()%3){error=TEXT("Invalid Chunk triangle indices");return false;}
                for(int index:section->Triangles){if(!section->Vertices.IsValidIndex(index)||section->Vertices[index].ContainsNaN()){error=TEXT("Invalid/nonfinite Chunk vertex");return false;}any=true;body.radius=std::max(body.radius,tvf::Length(V(section->Vertices[index])));}
            }
            if(!any||!(body.radius>0)||!std::isfinite(body.radius)){error=TEXT("Invalid Chunk XY footprint");return false;}
            body.minThickness=2*body.radius;
            scene.bodies.push_back(std::move(body));++out.HullCount;continue;
        }
        FDynamicMesh3 mesh;std::map<std::tuple<double,double,double>,int> vertices;std::vector<std::array<tvf::Vec3,3>> triangles;
        for(const auto* section:{&chunk.Exterior,&chunk.Interior}){if(section->Triangles.Num()%3){error=TEXT("Invalid Chunk triangle indices");return false;}for(int i=0;i<section->Triangles.Num();i+=3){int ids[3];tvf::Vec3 p[3];for(int j=0;j<3;++j){int index=section->Triangles[i+j];if(!section->Vertices.IsValidIndex(index)){error=TEXT("Invalid Chunk vertex index");return false;}auto v=section->Vertices[index];if(v.ContainsNaN()){error=TEXT("Nonfinite Chunk surface");return false;}p[j]=V(v);auto key=std::make_tuple(v.X,v.Y,v.Z);auto it=vertices.find(key);if(it==vertices.end())it=vertices.emplace(key,mesh.AppendVertex(v)).first;ids[j]=it->second;}if(mesh.AppendTriangle(ids[0],ids[1],ids[2])<0){error=TEXT("Chunk surface is not a closed manifold for proxy preparation");return false;}triangles.push_back({p[0],p[2],p[1]});}}
        const double tolerance=std::max(s.SurfaceToleranceCm,mesh.GetBounds().DiagonalLength()*s.RelativeSurfaceTolerance);tolerances.push_back(tolerance);
        // Render sections are triangle soup, not a collision-solid topology contract.
        // First construct a closed hull independently of shared edge connectivity.
        const bool sourceWasClosed=mesh.IsClosed();
        std::vector<tvf::CollisionHull> hulls;bool accepted=false;
        const double diagonal=mesh.GetBounds().DiagonalLength();
        auto build=[&](const FDynamicMesh3& source,double cleanup,tvf::CollisionHull& hull){
            if(!BuildVisualHull(source,cleanup,hull,error))return false;
            if(hull.vertices.size()>size_t(s.MaxVerticesPerHull)){error=FString::Printf(TEXT("Vertex budget: %llu > %d"),static_cast<unsigned long long>(hull.vertices.size()),s.MaxVerticesPerHull);return false;}
            return true;
        };
        for(double fraction:{0.0,1e-10,1e-8,1e-6}){
            if(progress.ShouldCancel()){error=TEXT("Collision preparation cancelled");return false;}
            tvf::CollisionHull hull;error.Reset();
            const double cleanup=std::min(tolerance*.01,diagonal*fraction);
            if(build(mesh,cleanup,hull)){hulls={std::move(hull)};accepted=Quality(mesh,hulls,chunk.VolumeCm3,tolerance,s.RelativeVolumeTolerance,error);}
            if(accepted)break;
            error=FString::Printf(TEXT("single hull cleanup=%.9g cm: %s"),cleanup,*error);
        }
        const FString singleError=error;
        if(!accepted){
            // Only the decomposition path needs a connected solid surface. Weld
            // unambiguous coincident boundary edges at numerical, not art, scale.
            FMergeCoincidentMeshEdges weld(&mesh);weld.OnlyUniquePairs=true;
            weld.MergeVertexTolerance=std::min(tolerance*.01,std::max(1e-7,mesh.GetBounds().DiagonalLength()*1e-8));
            weld.Apply();
            if(!mesh.IsClosed()){
                error=FString::Printf(TEXT("Chunk %d: single visual hull failed quality/budget (%s); concave decomposition needs repaired connectivity. Bake unchanged."),chunk.ChunkId,*error);
                return false;
            }
            FConvexDecomposition3 decomp;decomp.InitializeFromMesh(mesh,true);
            for(int target=2;target<=s.MaxHullsPerChunk;++target){
                if(progress.ShouldCancel()){error=TEXT("Collision preparation cancelled");return false;}
                decomp.Compute(target,0,0,0,s.MaxHullsPerChunk);
                if(decomp.NumHulls()<=0||decomp.NumHulls()>s.MaxHullsPerChunk){error=TEXT("Decomposition hull count outside budget");continue;}
                for(double fraction:{0.0,1e-10,1e-8,1e-6}){
                    hulls.clear();error.Reset();bool valid=true;
                    for(int i=0;i<decomp.NumHulls();++i){
                        if(progress.ShouldCancel()){error=TEXT("Collision preparation cancelled");return false;}
                        tvf::CollisionHull hull;
                        if(!build(decomp.GetHullMesh(i),std::min(tolerance*.01,diagonal*fraction),hull)){error=FString::Printf(TEXT("Decomposition target=%d hull=%d: %s"),target,i,*error);valid=false;break;}
                        hulls.push_back(std::move(hull));
                    }
                    if(valid&&Quality(mesh,hulls,chunk.VolumeCm3,tolerance,s.RelativeVolumeTolerance,error)){accepted=true;break;}
                }
                if(accepted)break;
            }
        }
        if(!accepted){error=FString::Printf(TEXT("Chunk %d proxy rejected: %s. First path: %s. Bake unchanged."),chunk.ChunkId,*error,*singleError);return false;}
        if(!mesh.IsClosed()){
            // An open render soup must not be integrated as a solid. Use the
            // quality-validated closed single proxy, preserving baked mass/origin.
            triangles.clear();for(const auto& h:hulls)for(const auto& f:h.faces)for(size_t i=1;i+1<f.size();++i)
                triangles.push_back({h.vertices[f[0]],h.vertices[f[i]],h.vertices[f[i+1]]});
            ++out.RenderSeamProxyCount;
        }
        else if(!sourceWasClosed){
            // Edge welding may have moved/merged vertices. Integrate the repaired
            // closed copy rather than the original disconnected render soup.
            triangles.clear();for(int id:mesh.TriangleIndicesItr()){
                FVector3d a,b,d;mesh.GetTriVertices(id,a,b,d);triangles.push_back({V(a),V(d),V(b)});
            }
        }
        tvf::CollisionBody body;body.hulls=std::move(hulls);body.mass=chunk.VolumeCm3*c->ObjectResponse.MassPerCm3;body.friction=c->ObjectResponse.Friction;body.restitution=c->ObjectResponse.Restitution;body.bounceThreshold=c->ObjectResponse.RestitutionVelocityThresholdCmPerSecond;body.mask=uint32(s.CollisionMask);std::string coreError;if(!tvf::ComputeCollisionInertia(triangles,body.mass,body.inverseInertia,coreError)){error=UTF8_TO_TCHAR(coreError.c_str());return false;}HullStats(body);out.HullCount+=int(body.hulls.size());scene.bodies.push_back(std::move(body));
    }
    // Strict audit only: visual proxies may overlap and use bounded split correction.
    if(!s.bSimpleVisual){
    uint64 tests=0;for(size_t a=0;a<scene.bodies.size();++a)for(size_t b=a+1;b<scene.bodies.size();++b){auto ca=V(c->BakedAsset->Chunks[a].Centroid),cb=V(c->BakedAsset->Chunks[b].Centroid);if(tvf::Length(ca-cb)>scene.bodies[a].radius+scene.bodies[b].radius)continue;for(auto ha:scene.bodies[a].hulls)for(auto hb:scene.bodies[b].hulls){if(++tests>scene.settings.maxShapePairs){error=TEXT("Frozen proxy audit exceeded shape-pair budget");return false;}for(auto& v:ha.vertices)v+=ca;for(auto& v:hb.vertices)v+=cb;for(const auto& contact:tvf::CollideConvex(ha,hb))if(-contact.gap>std::max(tolerances[a],tolerances[b])){error=FString::Printf(TEXT("Frozen proxies overlap: Chunk %d / %d, %.4f cm"),int(a),int(b),-contact.gap);return false;}}}
    }
    FString key;if(!External(c,&scene.environment,key,error))return false;out.ExternalSignature=FMD5::HashAnsiString(*key);scene.SampleEnvironment(scene.environment,FPlatformTime::Seconds());std::string coreError;if(!scene.Validate(coreError)){error=UTF8_TO_TCHAR(coreError.c_str());return false;}out.PreparationSeconds=FPlatformTime::Seconds()-start;error.Reset();return true;
}
void TVFCollisionEditor::Draw(UTAVisualFractureComponent* c,const FTVFCollisionPreparation& prepared,const std::vector<tvf::ChunkMotionState>& states){
    if(prepared.Scene.settings.groundOnly){
        for(size_t i=0;i<states.size();++i){const auto p=states[i].positionWorld;const double r=prepared.Scene.bodies[i].radius*prepared.Scene.settings.xyFootprintScale;
            for(int k=0;k<24;++k){double a=2*PI*k/24,b=2*PI*(k+1)/24;DrawDebugLine(c->GetWorld(),U(p+tvf::Vec3{r*std::cos(a),r*std::sin(a),0}),U(p+tvf::Vec3{r*std::cos(b),r*std::sin(b),0}),FColor::Cyan,false,0,1,1);}
        }
        for(const auto& b:prepared.Scene.environment){FBox box(ForceInit);for(const auto& h:b.body.hulls)for(auto p:h.vertices)box+=U(b.position+tvf::Rotate(b.rotation,p));DrawDebugBox(c->GetWorld(),box.GetCenter(),box.GetExtent(),FColor::Yellow,false,0,1,1);}
        return;
    }
    auto draw=[&](const tvf::CollisionHull& h,tvf::Vec3 x,tvf::Quat q,FColor color){for(const auto& f:h.faces)for(size_t i=0;i<f.size();++i)DrawDebugLine(c->GetWorld(),U(x+tvf::Rotate(q,h.vertices[f[i]])),U(x+tvf::Rotate(q,h.vertices[f[(i+1)%f.size()]])),color,false,0,1,1);};
    for(size_t i=0;i<states.size();++i)for(const auto& h:prepared.Scene.bodies[i].hulls)draw(h,states[i].positionWorld,states[i].rotationWorld,FColor::Cyan);
    for(const auto& b:prepared.Scene.environment)for(const auto& h:b.body.hulls)draw(h,b.position,b.rotation,FColor::Yellow);
}
