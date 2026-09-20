#include "TVFFracture.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <numeric>
#include <stdexcept>

namespace tvf {
namespace {
void Validate(const SeedSettings& s){
    if(s.targetCount<1||!std::isfinite(s.irregularity)||s.irregularity<0||s.irregularity>1||!std::isfinite(s.sizeVariation)||s.sizeVariation<0||s.sizeVariation>1||!std::isfinite(s.metric.axisScale)||s.metric.axisScale<0.125||s.metric.axisScale>8||!Finite(s.metric.axis)||LengthSquared(s.metric.axis)<1e-20||!Finite(s.impactPoint)||!std::isfinite(s.impactRadius)||s.impactRadius<0||!std::isfinite(s.impactConcentration)||s.impactConcentration<0||s.impactConcentration>1)throw std::invalid_argument("Invalid SeedSettings");
    if(s.impactConcentration>0&&!(s.impactRadius>0))throw std::invalid_argument("Density focus radius must be positive when concentration is enabled");
}
double Density(const SeedSettings& s,Vec3 p){if(s.impactConcentration<=0)return 1.0;double r2=LengthSquared(p-s.impactPoint)/(s.impactRadius*s.impactRadius);return 1.0+24.0*s.impactConcentration*std::exp(-0.5*r2);}
uint32_t StableSeed(uint32_t bake,uint32_t component,uint32_t candidate){return Hash(bake^Hash(component+0x9e3779b9u)^Hash(candidate+0x85ebca6bu));}
Vec3 SampleSimplex(const SolidSimplex& simplex,Random& random){double e[4],sum=0;for(double& q:e){q=-std::log(std::max(random.Unit(),1.0/16777216.0));sum+=q;}Vec3 p{};for(int i=0;i<4;++i)p+=simplex.vertices[i]*(e[i]/sum);return p;}
void AddRecovered(std::vector<Diagnostic>* diagnostics,DiagnosticCode code,const char* message,uint64_t before,uint64_t after){if(!diagnostics)return;Diagnostic d;d.severity=DiagnosticSeverity::Recovered;d.stage=DiagnosticStage::Seed;d.code=code;d.message=message;d.beforeCount=before;d.afterCount=after;diagnostics->push_back(std::move(d));}
}

std::vector<Seed> GenerateSeeds(const SolidDomain& domain,const SeedSettings& settings,std::vector<Diagnostic>* diagnostics){
    Validate(settings);if(domain.HasFatal()||domain.pieces.empty()||!(domain.volume>0))throw std::invalid_argument("Cannot generate Seeds from an invalid or empty SolidDomain");
    struct SimplexRef{const SolidSimplex* simplex=nullptr;double weightedVolume=0;};struct Component{uint32_t id=0;double weight=0;std::vector<SimplexRef> simplices;uint32_t quota=1;double remainder=0;};
    std::map<uint32_t,Component> components;
    for(size_t pieceIndex=0;pieceIndex<domain.pieces.size();++pieceIndex){if((pieceIndex&1023)==0&&settings.shouldCancel&&settings.shouldCancel())throw std::runtime_error("Seed volume-table construction cancelled");const SolidTetPiece& piece=domain.pieces[pieceIndex];Component& component=components[piece.componentId];component.id=piece.componentId;for(const SolidSimplex& simplex:piece.samplingTetrahedra){double weighted=simplex.volume*Density(settings,(simplex.vertices[0]+simplex.vertices[1]+simplex.vertices[2]+simplex.vertices[3])*0.25);if(weighted>0&&std::isfinite(weighted)){component.weight+=weighted;component.simplices.push_back({&simplex,weighted});}}}
    for(auto it=components.begin();it!=components.end();)if(it->second.simplices.empty()||!(it->second.weight>0))it=components.erase(it);else++it;
    if(components.empty())throw std::runtime_error("SolidDomain has no sampleable volume");if(settings.targetCount<components.size())throw std::runtime_error("Target Seed count is lower than the number of protected solid components");
    double totalWeight=0;for(const auto& kv:components)totalWeight+=kv.second.weight;uint32_t remaining=settings.targetCount-uint32_t(components.size()),assigned=0;
    for(auto& kv:components){double exact=remaining*kv.second.weight/totalWeight;uint32_t whole=uint32_t(std::floor(exact));kv.second.quota+=whole;kv.second.remainder=exact-whole;assigned+=whole;}
    std::vector<Component*> order;for(auto& kv:components)order.push_back(&kv.second);std::sort(order.begin(),order.end(),[](const Component* a,const Component* b){return a->remainder==b->remainder?a->id<b->id:a->remainder>b->remainder;});for(uint32_t i=assigned;i<remaining;++i)++order[(i-assigned)%order.size()]->quota;
    std::vector<Seed> out;out.reserve(settings.targetCount);
    uint64_t selectionTests=0,resampleCount=0;
    // Spacing is a duplicate/near-coincidence guard, not the art control.
    const double spacing=std::cbrt(domain.volume/double(settings.targetCount))*0.06;
    const double spacing2=spacing*spacing;
    for(Component* component:order){
        std::vector<double> cumulative;double total=0;
        for(const SimplexRef& ref:component->simplices){total+=ref.weightedVolume;cumulative.push_back(total);}
        for(uint32_t local=0;local<component->quota;++local){
            Random choice(StableSeed(settings.randomSeed^0x62a9d9edu,component->id,local));
            const bool randomChoice=choice.Unit()<settings.irregularity;
            bool accepted=false;Vec3 chosen{};double localSpacing2=spacing2;
            for(uint32_t attempt=0;attempt<96&&!accepted;++attempt){
                if(settings.shouldCancel&&settings.shouldCancel())throw std::runtime_error("Seed sampling cancelled");
                double bestScore=-1;
                // Best-candidate sampling: 32 volume samples per selection batch.
                // This is a sampling-quality choice, never a Power-neighbour cap.
                for(uint32_t candidate=0;candidate<32;++candidate){
                    const uint32_t candidateId=(local*96u+attempt)*32u+candidate;
                    Random random(StableSeed(settings.randomSeed,component->id,candidateId));
                    const double target=random.Unit()*total;
                    size_t index=std::lower_bound(cumulative.begin(),cumulative.end(),target)-cumulative.begin();
                    index=std::min(index,component->simplices.size()-1);
                    const Vec3 p=SampleSimplex(*component->simplices[index].simplex,random);
                    double nearest=std::numeric_limits<double>::infinity();
                    for(const Seed& existing:out){
                        ++selectionTests;
                        if((selectionTests&4095)==0&&settings.shouldCancel&&settings.shouldCancel())throw std::runtime_error("Seed selection cancelled");
                        if(settings.maxSelectionTests>0&&selectionTests>settings.maxSelectionTests)throw std::runtime_error("Seed selection work budget exceeded");
                        nearest=std::min(nearest,settings.metric.DistanceSquared(p-existing.position));
                    }
                    if(nearest<localSpacing2){++resampleCount;continue;}
                    // Review contract: squared-distance score scales by rho^(2/3).
                    const double score=nearest*std::pow(Density(settings,p),2.0/3.0);
                    if(!accepted||score>bestScore){chosen=p;bestScore=score;accepted=true;}
                    if(randomChoice||out.empty())break;
                }
                if(!accepted&&attempt==63)localSpacing2*=0.49;
            }
            if(!accepted)throw std::runtime_error("Deterministic volume sampling could not place all Seeds without a duplicate");
            out.push_back({chosen,0});
        }
    }
    if(out.size()!=settings.targetCount)throw std::runtime_error("Volume sampler did not produce the requested Seed count");if(resampleCount)AddRecovered(diagnostics,DiagnosticCode::DuplicateSeedResampled,"Minimum-spacing conflicts were deterministically resampled",resampleCount,out.size());
    Random weightRandom(Hash(settings.randomSeed^0xa511e9b3u));for(size_t i=0;i<out.size();++i){double nearest=std::numeric_limits<double>::infinity();for(size_t j=0;j<out.size();++j)if(i!=j){++selectionTests;if(settings.maxSelectionTests>0&&selectionTests>settings.maxSelectionTests)throw std::runtime_error("Seed weight work budget exceeded");nearest=std::min(nearest,settings.metric.DistanceSquared(out[i].position-out[j].position));}out[i].weight=out.size()>1?0.2*settings.sizeVariation*nearest*(2*weightRandom.Unit()-1):0;}
    return out;
}

std::vector<Seed> GenerateSeeds(const ScalarGrid& grid,const SeedSettings& settings){SolidDomainSettings domainSettings;domainSettings.maxPieces=settings.maxCandidateCount;SolidDomain domain=BuildSolidDomain(grid,domainSettings);return GenerateSeeds(domain,settings,nullptr);}
} // namespace tvf
