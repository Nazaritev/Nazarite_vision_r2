#include "engine_impl.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <openssl/evp.h>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <algorithm>
#include <mutex>
#include <unordered_map>
#include <Eigen/Eigenvalues>

namespace rc_bbs {
namespace {
std::string digest(const std::string& path, Config config) {
  std::ifstream file(path,std::ios::binary);
  if (!file) throw std::runtime_error("Cannot read PCD: "+path);
  std::unique_ptr<EVP_MD_CTX,decltype(&EVP_MD_CTX_free)> ctx(EVP_MD_CTX_new(),EVP_MD_CTX_free);
  EVP_DigestInit_ex(ctx.get(),EVP_sha256(),nullptr);
  char buffer[65536];
  while(file) {file.read(buffer,sizeof buffer); EVP_DigestUpdate(ctx.get(),buffer,file.gcount());}
  std::ostringstream params;
  config.visit([&](const char* name,const auto& v) {
    params<<name<<'=';
    using T=std::decay_t<decltype(v)>;
    if constexpr(std::is_same_v<T,std::vector<double>>) {for(double x:v) params<<std::setprecision(17)<<x<<',';}
    else params<<std::setprecision(17)<<v;
    params<<';';
  });
  auto text=params.str(); EVP_DigestUpdate(ctx.get(),text.data(),text.size());
  unsigned char hash[EVP_MAX_MD_SIZE]; unsigned len=0;
  EVP_DigestFinal_ex(ctx.get(),hash,&len);
  std::ostringstream out;
  for(unsigned i=0;i<len;++i) out<<std::hex<<std::setw(2)<<std::setfill('0')<<int(hash[i]);
  return out.str();
}
}
Engine::Engine(Config c,const std::string& file) : impl_(std::make_unique<Impl>()) {
  auto start=Clock::now(); c.validate(); impl_->config=c;
  impl_->map_version=digest(file,c);
  auto raw=std::make_shared<Cloud>();
  if(pcl::io::loadPCDFile<pcl::PointXYZ>(file,*raw)<0) throw std::runtime_error("Invalid PCD "+file);
  // Radius filtering on millions of repeated map returns dominated cold startup.
  // Prove dense-voxel support first; source quality denominators remain untouched.
  impl_->target=c.map_filter_voxel_support ? clean_cloud(raw,c,false,c.map_leaf) :
    voxel(clean_cloud(raw,c,false),c.map_leaf);
  if(impl_->target->size()<static_cast<size_t>(c.min_points)) throw std::runtime_error("Map has insufficient valid geometry");
  impl_->tree.setInputCloud(impl_->target);
  impl_->map_xy=std::make_shared<Cloud>();
  for(const auto& p:*impl_->target)impl_->map_xy->emplace_back(p.x,p.y,0.f);
  impl_->map_xy=voxel(impl_->map_xy,c.map_leaf);
  impl_->xy_tree.setInputCloud(impl_->map_xy);
  impl_->normals=estimate_normals(impl_->target,c);
  // Analyze map and source on the same physical scale. Different KNN
  // footprints otherwise give contradictory ground/edge labels after accumulation.
  auto structure_map=voxel(impl_->target,std::min(c.scan_leaf,c.bbs_scan_leaf));
  auto bbs_map=c.bbs_filter_map_by_normals ? nonground(structure_map,c) : structure_map;
  if(bbs_map->size()<static_cast<size_t>(c.bbs_min_structure_points))
    throw std::runtime_error("Map lacks stable non-ground structure for global localization");
  impl_->backend=make_backend(c,bbs_map);
  impl_->gicp=make_gicp(c,impl_->target);
  if(digest(file,c)!=impl_->map_version) throw std::runtime_error("Map changed while building cache; retry reload");
  impl_->construction_ms=std::chrono::duration<double,std::milli>(Clock::now()-start).count();
}
Engine::~Engine()=default;
std::shared_ptr<Engine> load_cached_engine(const Config& config,const std::string& path) {
  static std::mutex mutex;
  static std::unordered_map<std::string,std::weak_ptr<Engine>> cache;
  const auto key=digest(path,config);
  std::lock_guard<std::mutex> lock(mutex);
  auto found=cache.find(key);
  if(found!=cache.end())if(auto engine=found->second.lock()) {
    if(digest(path,config)!=key)throw std::runtime_error("Map changed while reusing cache");
    return engine;
  }
  auto engine=std::make_shared<Engine>(config,path);
  if(engine->version()!=key)throw std::runtime_error("Map changed while loading cache");
  cache[key]=engine;
  for(auto it=cache.begin();it!=cache.end();) {
    if(it->second.expired())it=cache.erase(it);else ++it;
  }
  return engine;
}
Cloud::ConstPtr Engine::map()const{return impl_->target;}
std::string Engine::version()const{return impl_->map_version;}
double Engine::build_ms()const{return impl_->construction_ms;}
bool Engine::Impl::inside_roi(const Eigen::Vector3d& world) const {
  std::vector<int> ids(1);std::vector<float> ds(1);
  return xy_tree.nearestKSearch(pcl::PointXYZ(world.x(),world.y(),0.f),1,ids,ds)==1 &&
    ds[0]<=config.map_roi_margin_m*config.map_roi_margin_m;
}
Cloud::Ptr Engine::registration_source(const Cloud::ConstPtr& source,const Matrix& guess) const {
  if(!rigid(guess))return std::make_shared<Cloud>();
  auto small=voxel(source,impl_->config.scan_leaf);
  auto fit=small;
  if(impl_->config.unmapped_obstacle_filter) {
    fit=std::make_shared<Cloud>();
    const auto normals=estimate_normals(small,impl_->config);
    const double limit2=std::pow(impl_->config.static_support_distance_m,2);
    const double cosine=std::cos(impl_->config.normal_consistency_rad);
    std::vector<int> ids(1);std::vector<float> ds(1);
    for(size_t i=0;i<small->size();++i) {
      const auto& p=(*small)[i];
      const Eigen::Vector3d world=guess.block<3,3>(0,0)*p.getVector3fMap().cast<double>()+guess.block<3,1>(0,3);
      if(!impl_->inside_roi(world) || impl_->tree.nearestKSearch(pcl::PointXYZ(world.x(),world.y(),world.z()),1,ids,ds)!=1 || ds[0]>limit2)continue;
      const auto sn=guess.block<3,3>(0,0)*normals[i];const auto& tn=impl_->normals[ids[0]];
      if(sn.squaredNorm()>.5 && tn.squaredNorm()>.5 && std::abs(sn.dot(tn))<cosine)continue;
      fit->push_back(p);
    }
  }
  return fit;
}
Candidate Engine::refine(const Cloud::ConstPtr& source,const Matrix& guess) {
  Candidate out;
  if(source->size()<static_cast<size_t>(impl_->config.min_points) || !rigid(guess)) {
    out.quality.reason="insufficient_source_or_invalid_pose";return out;
  }
  const auto fit=registration_source(source,guess);
  if(fit->size()<static_cast<size_t>(impl_->config.unmapped_obstacle_filter?impl_->config.min_points:20)) {
    out.T_map_sensor=guess;out.quality=evaluate(source,guess,false);
    out.quality.accepted=false;out.quality.reason="insufficient_static_gicp_support";return out;
  }
  const auto result=impl_->gicp->align(fit,guess);
  out.T_map_sensor=result.first;
  out.quality=evaluate(source,result.first,result.second);
  return out;
}
SearchResult Engine::search(const Cloud::ConstPtr& source,const Matrix& gravity) {
  SearchResult out;const auto start=Clock::now();const auto& c=impl_->config;
  const auto deadline=start+std::chrono::milliseconds(c.search_budget_ms);
  if(source->size()<static_cast<size_t>(c.min_points) || !rigid(gravity) || gravity.block<3,1>(0,3).norm()>1e-6) {
    out.reason="insufficient_source_or_invalid_gravity";return out;
  }
  // IMU provides canonical +Z-up. The PCD has no gravity metadata, so
  // configured map +Z/-Z hypotheses are searched explicitly in ONE shared budget.
  struct Seed { Matrix pose; double score,sign; };
  struct Aligned {Matrix rotation;Cloud::Ptr cloud;double sign;};
  std::vector<Aligned> inputs;
  std::vector<Seed> seeds;
  out.complete=true;out.reason="no_bbs_candidate";
  for(double sign:c.map_gravity_z_signs) {
    Matrix alignment=gravity;
    if(sign<0.) {
      Matrix flip=Matrix::Identity();
      flip.block<3,3>(0,0)=Eigen::AngleAxisd(M_PI,Eigen::Vector3d::UnitY()).toRotationMatrix();
      alignment=flip*gravity;
    }
    auto aligned=std::make_shared<Cloud>();pcl::transformPointCloud(*source,*aligned,alignment);
    aligned=voxel(aligned,std::min(c.scan_leaf,c.bbs_scan_leaf));
    if(c.bbs_use_nonground)aligned=nonground(aligned,c);
    aligned=voxel(aligned,c.bbs_scan_leaf);
    if(aligned->size()<static_cast<size_t>(c.bbs_min_structure_points)) {
      out.complete=false;out.reason="insufficient_bbs_geometry_"+std::to_string(aligned->size());break;
    }
    inputs.push_back({alignment,aligned,sign});
  }
  // Strict proposals must survive local quality before they suppress the
  // explicit partial-overlap retry. A coarse false positive is not recovery.
  // All phases, hypotheses and refinement share the same deadline.
  for(int phase=0;out.complete && phase<(c.unmapped_obstacle_filter?2:1);++phase) {
    seeds.clear();
    for(const auto& input:inputs) {
      auto coarse=impl_->backend->search(input.cloud,deadline,phase==1);
      for(const auto& item:coarse.poses)seeds.push_back({item.first*input.rotation,item.second,input.sign});
      if(!coarse.complete) {out.complete=false;out.reason=coarse.reason;break;}
      if(seeds.size()>static_cast<size_t>(c.max_candidates)) {
        out.complete=false;out.reason="candidate_cap_across_gravity_hypotheses";break;
      }
    }
    if(!out.complete) {
      for(const auto& item:seeds) {
        Candidate only;only.T_map_sensor=item.pose;only.bbs_hit_fraction=item.score;only.map_gravity_z_sign=item.sign;
        only.quality.reason="coarse_only_search_incomplete";out.candidates.push_back(only);
      }
      break;
    }
    bool qualified=false;
    for(const auto& item:seeds) {
      if(Clock::now()>=deadline){out.complete=false;out.reason="refinement_budget_exceeded";break;}
      const Matrix& guess=item.pose;
      auto candidate=refine(source,guess);candidate.bbs_hit_fraction=item.score;candidate.map_gravity_z_sign=item.sign;
      if((candidate.T_map_sensor.block<3,1>(0,3)-guess.block<3,1>(0,3)).norm()>c.max_refine_translation_m ||
         rotation_distance(candidate.T_map_sensor,guess)>c.max_refine_rotation_rad) {
        candidate.quality.accepted=false;candidate.quality.reason="excessive_coarse_refine_correction";
      }
      qualified=qualified || candidate.quality.accepted;
      out.candidates.push_back(candidate);
    }
    if(!out.complete || qualified)break;
    if(phase==1 && out.candidates.empty())out.reason="no_bbs_candidate_after_partial_overlap_search";
  }
  if(Clock::now()>=deadline) {out.complete=false;out.reason="total_search_budget_exceeded";}
  std::stable_sort(out.candidates.begin(),out.candidates.end(),[](const Candidate& a,const Candidate& b){
    if(a.quality.accepted!=b.quality.accepted) return a.quality.accepted;
    return a.quality.capped_all_mse_m2<b.quality.capped_all_mse_m2;
  });
  if(out.complete && !out.candidates.empty() && out.candidates[0].quality.accepted) {
    const auto& a=out.candidates[0];
    for(size_t i=1;i<out.candidates.size();++i) {
      const auto& b=out.candidates[i];
      const bool different=(a.T_map_sensor.block<3,1>(0,3)-b.T_map_sensor.block<3,1>(0,3)).norm()>c.candidate_separation_m ||
        rotation_distance(a.T_map_sensor,b.T_map_sensor)>c.candidate_separation_rad;
      const double a_coverage=c.unmapped_obstacle_filter?a.quality.roi_coverage:a.quality.coverage;
      const double b_coverage=c.unmapped_obstacle_filter?b.quality.roi_coverage:b.quality.coverage;
      if(different && b.quality.accepted && std::abs(a_coverage-b_coverage)<=c.ambiguity_coverage_margin &&
         std::abs(a.quality.rmse_m-b.quality.rmse_m)<=c.ambiguity_rmse_margin_m) out.ambiguous=true;
    }
    out.accepted=!out.ambiguous;
    out.reason=out.ambiguous?"ambiguous_distinct_poses":"candidate_requires_fresh_multiframe_validation";
  } else if(out.complete && !out.candidates.empty()) out.reason=out.candidates.front().quality.reason;
  out.elapsed_ms=std::chrono::duration<double,std::milli>(Clock::now()-start).count();
  return out;
}
}  // namespace rc_bbs
