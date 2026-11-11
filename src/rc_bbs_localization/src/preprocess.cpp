#include "rc_bbs_localization/core.hpp"
#include <pcl/common/point_tests.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <stdexcept>
#include <type_traits>
#include <array>
#include <unordered_map>

namespace rc_bbs {
void Config::validate() const {
  Config copy = *this;
  copy.visit([](const char* name, const auto& value) {
    using T = std::decay_t<decltype(value)>;
    if constexpr(std::is_arithmetic_v<T>) {
      if (!std::isfinite(static_cast<double>(value))) throw std::invalid_argument(std::string(name)+" must be finite");
    }
  });
  if (backend != "official_cpu") throw std::invalid_argument("Only official_cpu is built; requested backend unavailable, no fallback");
  if(map_gravity_z_signs.empty() || map_gravity_z_signs.size()>2)
    throw std::invalid_argument("Configure one or both map gravity Z signs explicitly");
  for(size_t i=0;i<map_gravity_z_signs.size();++i) {
    if(map_gravity_z_signs[i]!=1. && map_gravity_z_signs[i]!=-1.)
      throw std::invalid_argument("map_gravity_z_signs entries must be +1 or -1");
    if(i && map_gravity_z_signs[0]==map_gravity_z_signs[i])
      throw std::invalid_argument("Duplicate map gravity hypothesis");
  }
  if (search_min.size()!=3 || search_max.size()!=3) throw std::invalid_argument("search_min/max must contain xyz meters");
  for (int i=0;i<3;++i) if (!std::isfinite(search_min[i]) || !std::isfinite(search_max[i]) ||
      search_min[i]>=search_max[i] || std::abs(search_min[i])>map_abs_limit || std::abs(search_max[i])>map_abs_limit)
    throw std::invalid_argument("Invalid global search bounds");
  if (map_leaf<=0 || scan_leaf<=0 || bbs_scan_leaf<=0 || bbs_resolution<0.1 ||
      bbs_levels<1 || bbs_levels>10 || bbs_threads<1 || bbs_threads>32 || search_budget_ms<1 ||
      max_candidates<2 || max_candidates>16 || max_queue_size<100 || max_queue_size>10000000 ||
      gravity_uncertainty_rad<0 || roll_pitch_window_rad<gravity_uncertainty_rad || roll_pitch_window_rad>0.5 ||
      yaw_min_rad>=yaw_max_rad || yaw_min_rad < -M_PI || yaw_max_rad > M_PI ||
      min_range<0 || max_range<=min_range || max_range>200 || map_abs_limit<max_range ||
      outlier_radius<=0 || outlier_min_neighbors<1 || min_points<20 ||
      normal_consistency_rad<=0 || normal_consistency_rad>=M_PI/2 || min_consistent_structure_correspondences<20 ||
      min_correspondences<20 || min_nonground_correspondences<20 || gicp_iterations<1 || gicp_optimizer_iterations<1 || normal_min_neighbors<5 || normal_radius_m<=0 || bbs_min_structure_points<10 || bbs_normal_z_max<=0 || bbs_normal_z_max>1 ||
      gicp_correspondence_m<=0 || match_distance_m<=0 || max_rmse_m<=0 || max_p95_m<=0 ||
      candidate_separation_m<=0 || candidate_separation_rad<=0 || max_refine_translation_m<=0 || max_refine_rotation_rad<=0)
    throw std::invalid_argument("Invalid localization configuration; check positive sizes/budgets and gravity uncertainty");
  if(map_roi_margin_m<=0 || min_roi_source_fraction<=0 || min_roi_source_fraction>1 ||
     max_unmapped_fraction<0 || max_unmapped_fraction>=1 ||
     bbs_max_unmapped_fraction<0 || bbs_max_unmapped_fraction>=1 ||
     static_support_distance_m<match_distance_m || static_support_distance_m>gicp_correspondence_m ||
     bbs_min_matched_points<10 || min_unique_structure_voxels<6 || structure_voxel_m<=0 || min_structure_extent_m<=0)
    throw std::invalid_argument("Invalid partial-map obstacle filter configuration");
  for (double v : {bbs_min_hit_fraction, min_coverage, min_nonground_fraction, min_information_ratio, min_consistent_structure_fraction})
    if (v<=0 || v>1) throw std::invalid_argument("Fractions must be in (0,1]");
  if (ambiguity_coverage_margin<0 || ambiguity_rmse_margin_m<0) throw std::invalid_argument("Invalid ambiguity margins");
  // Bound initial coarse allocation before entering the official int-based implementation.
  double top = bbs_resolution*std::pow(2., bbs_levels), cells=1.;
  for (int i=0;i<3;++i) cells *= std::ceil(search_max[i]/top)-std::floor(search_min[i]/top)+1.;
  cells *= 1000.; // conservative angular allocation guard
  if (cells>1e9) throw std::invalid_argument("Search bounds/levels would overflow coarse search allocation");
}
Cloud::Ptr voxel(const Cloud::ConstPtr& source, double leaf) {
  auto out=std::make_shared<Cloud>();
  pcl::VoxelGrid<pcl::PointXYZ> filter;
  filter.setInputCloud(source); filter.setLeafSize(leaf,leaf,leaf); filter.filter(*out);
  return out;
}
Cloud::Ptr clean_cloud(const Cloud::ConstPtr& source, const Config& c, bool scan, double pre_filter_leaf) {
  auto clean=std::make_shared<Cloud>();
  clean->reserve(source->size());
  for (const auto& p : *source) {
    if (!pcl::isFinite(p) || p.getVector3fMap().cwiseAbs().maxCoeff()>c.map_abs_limit) continue;
    const double r=p.getVector3fMap().norm();
    if (scan && (r<c.min_range || r>c.max_range)) continue;
    clean->push_back(p);
  }
  if (clean->empty()) return clean;
  if (pre_filter_leaf>0.) {
    // A voxel with enough raw returns proves radius support for every return
    // when its diagonal fits within the radius. Only sparse voxels need a tree
    // query. Preserve raw point order and filter BEFORE the final voxel average.
    using Key=std::array<double,3>;
    struct Hash {
      size_t operator()(const Key& k) const {
        size_t h=0;for(double x:k)h^=std::hash<double>{}(x)+0x9e3779b9+(h<<6)+(h>>2);return h;
      }
    };
    const auto key=[&](const auto& p)->Key {
      return {std::floor(p.x/pre_filter_leaf),std::floor(p.y/pre_filter_leaf),std::floor(p.z/pre_filter_leaf)};
    };
    std::unordered_map<Key,int,Hash> counts;
    for(const auto& p:*clean)++counts[key(p)];
    pcl::KdTreeFLANN<pcl::PointXYZ> tree;bool have_tree=false;
    auto out=std::make_shared<Cloud>();
    std::vector<int> ids;std::vector<float> distances;
    const int required=c.outlier_min_neighbors+1;
    const bool fits=std::sqrt(3.)*pre_filter_leaf<c.outlier_radius;
    for(const auto& p:*clean) {
      if(fits && counts[key(p)]>=required) {out->push_back(p);continue;}
      if(!have_tree){tree.setInputCloud(clean);have_tree=true;}
      if(tree.radiusSearch(p,c.outlier_radius,ids,distances,required)>=required)out->push_back(p);
    }
    return voxel(out,pre_filter_leaf);
  }
  pcl::RadiusOutlierRemoval<pcl::PointXYZ> filter;
  filter.setInputCloud(clean); filter.setRadiusSearch(c.outlier_radius);
  filter.setMinNeighborsInRadius(c.outlier_min_neighbors);
  auto out=std::make_shared<Cloud>(); filter.filter(*out);
  return out;
}
bool rigid(const Matrix& t) {
  if (!t.allFinite() || (t.row(3)-Eigen::RowVector4d(0,0,0,1)).norm()>1e-5) return false;
  Eigen::Matrix3d r=t.block<3,3>(0,0);
  return (r.transpose()*r-Eigen::Matrix3d::Identity()).norm()<1e-3 && std::abs(r.determinant()-1.)<1e-3;
}
double rotation_distance(const Matrix& a, const Matrix& b) {
  return Eigen::AngleAxisd(a.block<3,3>(0,0).transpose()*b.block<3,3>(0,0)).angle();
}
Matrix propagate(const Matrix& pose, const Matrix& old_odom, const Matrix& new_odom) {
  return pose*old_odom.inverse()*new_odom;
}
}  // namespace rc_bbs
