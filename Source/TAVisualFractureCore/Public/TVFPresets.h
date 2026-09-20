#pragma once
#include "TVFMotion.h"
#include <stdexcept>

namespace tvf {
enum class ArtStyle {BlockStone,ImpactDebris,WoodSplinter,LayeredSlate,StylizedFloat};
struct ArtPreset {
    SeedSettings seeds;
    MotionSettings motion;
    ImpactEvent impact;
    Vec3 gravity={0,0,-980};
    double massPerCm3=0.0001; // Stylized mass, NOT material density. 10,000 cm^3 = 1 mass unit.
    double bondResistancePerCm2=0.01;
};
inline ArtPreset MakeArtPreset(ArtStyle style) {
    ArtPreset p;
    p.impact.radius=160;p.impact.impulse=650;p.impact.referenceRadius=20;
    p.impact.randomWeight=0.18;p.impact.upwardWeight=0.15;p.impact.bondDamage=15;
    switch(style) {
    case ArtStyle::BlockStone:
        p.seeds.targetCount=24;p.seeds.irregularity=.60;p.seeds.sizeVariation=.75;break;
    case ArtStyle::ImpactDebris:
        p.seeds.targetCount=48;p.seeds.irregularity=.70;p.seeds.sizeVariation=.85;
        p.seeds.impactConcentration=.85;p.seeds.impactRadius=18;
        p.impact.impulse=900;p.impact.directionalWeight=.8;p.impact.radialWeight=.7;
        p.impact.sizeSpeedBias=.2;break;
    case ArtStyle::WoodSplinter:
        p.seeds.targetCount=32;p.seeds.metric.axisScale=4;p.seeds.metric.axis={0,0,1};
        p.seeds.irregularity=.4;p.seeds.sizeVariation=.5;
        p.motion.linearDrag=.8;p.impact.impulse=420;break;
    case ArtStyle::LayeredSlate:
        p.seeds.targetCount=20;p.seeds.metric.axisScale=.25;p.seeds.metric.axis={0,0,1};
        p.seeds.irregularity=.4;p.seeds.sizeVariation=.45;
        p.impact.impulse=400;break;
    case ArtStyle::StylizedFloat:
        p.seeds.targetCount=16;p.seeds.irregularity=.3;p.seeds.sizeVariation=.2;
        p.gravity={0,0,0};p.motion.linearDrag=1.2;p.motion.angularDrag=.7;
        p.impact.impulse=180;p.impact.upwardWeight=.6;break;
    }
    return p;
}

// Optional engine-independent bridge from bake data to motion data.
// Support selection is by Chunk.id. Density and resistance units must be chosen consistently by caller.
inline std::vector<ChunkMotionDesc> MakeMotionChunks(const BakeResult& bake,double massPerCm3,
                                                   const std::vector<uint32_t>& supportChunkIds={}) {
    if(!std::isfinite(massPerCm3)||massPerCm3<=0)throw std::invalid_argument("massPerCm3 must be finite and positive");
    std::vector<ChunkMotionDesc> out;out.reserve(bake.chunks.size());
    for(const auto& c:bake.chunks) {
        ChunkMotionDesc d;d.id=c.id;d.restCentroid=c.centroid;d.radius=std::max(c.radius,1e-6);
        d.mass=std::max(c.volume*massPerCm3,1e-9);
        d.supported=std::find(supportChunkIds.begin(),supportChunkIds.end(),c.id)!=supportChunkIds.end();
        out.push_back(d);
    }
    return out;
}
inline std::vector<BondMotionDesc> MakeMotionBonds(const BakeResult& bake,double resistancePerCm2) {
    if(!std::isfinite(resistancePerCm2)||resistancePerCm2<=0)throw std::invalid_argument("resistancePerCm2 must be finite and positive");
    std::vector<BondMotionDesc> out;out.reserve(bake.bonds.size());
    for(const auto& b:bake.bonds){BondMotionDesc d;d.chunkA=b.chunkA;d.chunkB=b.chunkB;d.centroid=b.centroid;d.resistance=std::max(b.area*resistancePerCm2,1e-9);out.push_back(d);}
    return out;
}
}
