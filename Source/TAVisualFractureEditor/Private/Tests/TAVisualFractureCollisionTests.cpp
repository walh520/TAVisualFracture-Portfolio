#include "Misc/AutomationTest.h"
#include "TAVisualFractureComponent.h"
#include "TAVisualFractureColliderComponent.h"
#include "TAVisualFractureCollisionService.h"
#include "TAVisualFractureAsset.h"
#include "TVFCollision.h"

#if WITH_DEV_AUTOMATION_TESTS
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTVFCollisionProxyFixture,
    "TA.VisualFracture.Collision.EditorProxyContracts",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FTVFCollisionProxyFixture::RunTest(const FString& Parameters)
{
    UTAVisualFractureComponent* Component=NewObject<UTAVisualFractureComponent>();
    TestTrue(TEXT("Visual-first mode is the default"),Component->VisualCollision.bSimpleVisual);
    Component->VisualCollision.bSimpleVisual=false; // retain strict regression coverage
    TestFalse(TEXT("Old authoring defaults leave self collision off"),Component->VisualCollision.bSelfCollision);
    TestFalse(TEXT("Old authoring defaults leave external collision off"),Component->VisualCollision.bExternalBoxCollision);
    UTAVisualFractureColliderComponent* Collider=NewObject<UTAVisualFractureColliderComponent>();
    TestFalse(TEXT("No automatic whole-mesh bounding box fallback"),Collider->bUseManualBoxes);
    Component->VisualCollision.bSelfCollision=true;
    Component->BakedAsset=NewObject<UTAVisualFractureAsset>();
    Component->BakedAsset->BakeSignature=TEXT("CollisionFixture-NoBake");
    auto& Chunk=Component->BakedAsset->Chunks.AddDefaulted_GetRef();
    Chunk.ChunkId=0;Chunk.VolumeCm3=1000000;Chunk.RadiusCm=FMath::Sqrt(7500.0);
    const auto Hull=tvf::MakeCollisionBox({50,50,50});
    for(const auto P:Hull.vertices)Chunk.Exterior.Vertices.Add(FVector(P.x,P.y,P.z));
    for(const auto& Face:Hull.faces)for(size_t I=1;I+1<Face.size();++I){
        Chunk.Exterior.Triangles.Add(Face[0]);Chunk.Exterior.Triangles.Add(Face[I+1]);Chunk.Exterior.Triangles.Add(Face[I]);
    }
    FTVFCollisionPreparation Prepared;FString Error;
    TestTrue(TEXT("Closed cube proxy prepares without running Bake"),TVFCollisionEditor::Prepare(Component,Prepared,Error));
    if(!Error.IsEmpty())AddError(Error);
    TestEqual(TEXT("Cube needs a single hull"),Prepared.HullCount,1);
    TestEqual(TEXT("Preparation leaves Bake signature unchanged"),Component->BakedAsset->BakeSignature,FString(TEXT("CollisionFixture-NoBake")));
    const auto OriginalSection=Chunk.Exterior;
    // Almost-collinear hull input: reproduces the numerical sliver that whole-
    // polygon area alone cannot fix. Exercises GeometryCore + shared converter.
    for(double Noise:{1e-12,1e-9,1e-6}){
        Chunk.Exterior=OriginalSection;
        const int32 SA=Chunk.Exterior.Triangles[0],SB=Chunk.Exterior.Triangles[1],SC=Chunk.Exterior.Triangles[2];
        FVector P=(Chunk.Exterior.Vertices[SA]+Chunk.Exterior.Vertices[SB])*.5;
        P.Z-=Noise;
        const int32 SM=Chunk.Exterior.Vertices.Add(P);
        Chunk.Exterior.Triangles[2]=SM;
        Chunk.Exterior.Triangles.Append({SB,SC,SM,SC,SA,SM});
        const bool Success=TVFCollisionEditor::Prepare(Component,Prepared,Error);
        TestTrue(*FString::Printf(TEXT("Near-coplanar sliver %.3g prepares: %s"),Noise,*Error),Success);
        if(Success)TestEqual(TEXT("Sliver does not require concave decomposition"),Prepared.HullCount,1);
    }
    Chunk.Exterior=OriginalSection;
    // Split one triangle edge without splitting its neighbour: spatially closed,
    // but indexed topology has a T-junction, as render-patch assembly may have.
    const int32 A=Chunk.Exterior.Triangles[0],B=Chunk.Exterior.Triangles[1],C=Chunk.Exterior.Triangles[2];
    const int32 Mid=Chunk.Exterior.Vertices.Add((Chunk.Exterior.Vertices[A]+Chunk.Exterior.Vertices[B])*.5);
    Chunk.Exterior.Triangles[1]=Mid;
    Chunk.Exterior.Triangles.Append({Mid,B,C});
    TestTrue(TEXT("Render T-junction does not prevent a validated convex proxy"),TVFCollisionEditor::Prepare(Component,Prepared,Error));
    TestEqual(TEXT("T-junction uses closed proxy inertia"),Prepared.RenderSeamProxyCount,1);
    Chunk.Exterior=OriginalSection;
    // Triangle-local vertices with tiny seam discrepancies.
    Chunk.Exterior.Vertices.Reset();Chunk.Exterior.Triangles.Reset();
    for(int32 I=0;I<OriginalSection.Triangles.Num();++I){
        FVector P=OriginalSection.Vertices[OriginalSection.Triangles[I]];
        P.X+=(I%3)*1e-8;
        Chunk.Exterior.Triangles.Add(Chunk.Exterior.Vertices.Add(P));
    }
    TestTrue(TEXT("Tiny render seams allow a quality-checked proxy"),TVFCollisionEditor::Prepare(Component,Prepared,Error));
    Chunk.Exterior=OriginalSection;
    Chunk.Exterior.Triangles.RemoveAt(0,6);
    TestFalse(TEXT("An actually missing cube face fails proxy surface coverage"),TVFCollisionEditor::Prepare(Component,Prepared,Error));
    Chunk.Exterior=OriginalSection;
    Component->VisualCollision.MaxVerticesPerHull=4;
    TestFalse(TEXT("Proxy vertex budget fails explicitly, not by truncation"),TVFCollisionEditor::Prepare(Component,Prepared,Error));
    TestTrue(TEXT("Budget failure reports current cause"),Error.Contains(TEXT("budget")));
    Component->VisualCollision.bSimpleVisual=true;
    TestTrue(TEXT("Visual mode simplifies within low hull budget"),TVFCollisionEditor::Prepare(Component,Prepared,Error));
    TestTrue(TEXT("Visual mode selects ground-only avoidance"),Prepared.Scene.settings.groundOnly);
    TestTrue(TEXT("Visual mode prepares no convex geometry"),Prepared.Scene.bodies[0].hulls.empty());
    const FString OriginalCollisionSignature=Prepared.Signature;
    Component->VisualCollision.XYFootprintScale=.5;
    Component->VisualCollision.GroundRollStrength=.7;
    Component->VisualCollision.GroundAvoidanceSpeedCmPerSecond=60;
    Component->VisualCollision.GroundAvoidanceIterations=8;
    Component->VisualCollision.ExternalVelocityTransfer=.8;
    Component->VisualCollision.ExternalLiftRatio=.2;
    Component->VisualCollision.ExternalLiftThreshold=25;
    Component->VisualCollision.GroundFrictionDecay=3;
    Component->VisualCollision.MaxCandidatePairs=Component->VisualCollision.MaxShapePairs=0;
    TestTrue(TEXT("Visual mode ignores legacy zero budgets and accepts eight iterations"),TVFCollisionEditor::Prepare(Component,Prepared,Error));
    TestEqual(TEXT("Footprint scale reaches core"),Prepared.Scene.settings.xyFootprintScale,.5);
    TestEqual(TEXT("Ground roll strength reaches core"),Prepared.Scene.settings.groundRollStrength,.7);
    TestEqual(TEXT("Correction speed reaches core"),Prepared.Scene.settings.avoidanceSpeed,60.0);
    TestEqual(TEXT("External transfer reaches core"),Prepared.Scene.settings.externalVelocityTransfer,.8);
    TestEqual(TEXT("External lift reaches core"),Prepared.Scene.settings.externalLiftRatio,.2);
    TestEqual(TEXT("External threshold reaches core"),Prepared.Scene.settings.externalLiftThreshold,25.0);
    TestEqual(TEXT("Ground friction reaches core"),Prepared.Scene.settings.groundFrictionDecay,3.0);
    const FString BeforeRefreshSignature=Prepared.Signature;
    const size_t BeforeRefreshBodies=Prepared.Scene.bodies.size();
    Prepared.ExternalSignature=TEXT("ForceExternalRefresh");
    TestTrue(TEXT("External-only refresh succeeds"),TVFCollisionEditor::RefreshExternal(Component,Prepared,Error));
    TestEqual(TEXT("External refresh preserves fracture signature"),Prepared.Signature,BeforeRefreshSignature);
    TestTrue(TEXT("External refresh preserves chunk bodies"),Prepared.Scene.bodies.size()==BeforeRefreshBodies);
    TestTrue(TEXT("Visual settings invalidate collision cache only"),Prepared.Signature!=OriginalCollisionSignature);
    Component->VisualCollision.XYFootprintScale=0;
    TestFalse(TEXT("Nonpositive footprint rejected"),TVFCollisionEditor::Prepare(Component,Prepared,Error));
    Component->VisualCollision.XYFootprintScale=1;
    Component->VisualCollision.MaxVerticesPerHull=32;
    Chunk.Exterior.Triangles.RemoveAt(0,6);
    TestTrue(TEXT("Visual mode tolerates missing render face"),TVFCollisionEditor::Prepare(Component,Prepared,Error));
    TestTrue(TEXT("Shape deviation is reported, not fatal"),!Prepared.VisualWarnings.IsEmpty());
    Chunk.Exterior=OriginalSection;
    // Render connectivity is not required for the visual proxy point cloud.
    Chunk.Exterior.Triangles.Reset();for(int32 I=0;I<8;++I)Chunk.Exterior.Triangles.Append({I,I,I});
    TestTrue(TEXT("Visual mode accepts disconnected point soup"),TVFCollisionEditor::Prepare(Component,Prepared,Error));
    Chunk.Exterior.Triangles[0]=999999;
    TestFalse(TEXT("Visual mode still rejects invalid indices"),TVFCollisionEditor::Prepare(Component,Prepared,Error));
    TestEqual(TEXT("Visual preparation did not rebake"),Component->BakedAsset->BakeSignature,FString(TEXT("CollisionFixture-NoBake")));
    return true;
}
#endif
