#pragma once
#include "CoreMinimal.h"
#include "TVFCollision.h"
class UTAVisualFractureComponent;
struct FTVFCollisionPreparation
{
    tvf::CollisionScene Scene;
    FString Signature;
    FString ExternalSignature;
    double PreparationSeconds=0;
    int32 HullCount=0;
    FString VisualWarnings;
    int32 RenderSeamProxyCount=0; // validated single hull replaces render-soup inertia
};
namespace TVFCollisionEditor
{
bool Prepare(UTAVisualFractureComponent*,FTVFCollisionPreparation&,FString& Error);
bool Signature(UTAVisualFractureComponent*,FString& Out,FString& Error);
bool RefreshExternal(UTAVisualFractureComponent*,FTVFCollisionPreparation&,FString& Error);
void Draw(UTAVisualFractureComponent*,const FTVFCollisionPreparation&,const std::vector<tvf::ChunkMotionState>&);
}
