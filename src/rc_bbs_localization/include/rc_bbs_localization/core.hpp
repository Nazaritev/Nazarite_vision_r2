#pragma once
#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <chrono>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace rc_bbs {
using Cloud = pcl::PointCloud<pcl::PointXYZ>;
using Matrix = Eigen::Matrix4d;
using Clock = std::chrono::steady_clock;
struct Config {
  std::string backend = "official_cpu";
  double map_leaf = 0.15, scan_leaf = 0.20, bbs_scan_leaf = 0.50;
  double bbs_resolution = 0.50;
  int bbs_levels = 5, bbs_threads = 4, search_budget_ms = 1400;
  int max_candidates = 3, max_queue_size = 1000000;
  double bbs_min_hit_fraction = 0.55;
  double candidate_separation_m = 1.5, candidate_separation_rad = 0.5;
  std::vector<double> search_min = {-30., -30., -3.};
  std::vector<double> search_max = {30., 30., 3.};
  std::vector<double> map_gravity_z_signs = {1.0};
  double gravity_uncertainty_rad = 0.06, roll_pitch_window_rad = 0.08;
  double yaw_min_rad = -3.141592653589793, yaw_max_rad = 3.141592653589793;
  double min_range = 0.5, max_range = 30., outlier_radius = 0.5;
  int outlier_min_neighbors = 3, min_points = 150;
  double map_abs_limit = 1000.;
  bool bbs_use_nonground = true;
  bool bbs_filter_map_by_normals = true;
  bool map_filter_voxel_support = true;
  // Partial maps: exclude returns outside the mapped XY footprint, then
  // tolerate a bounded fraction of unmodelled objects inside that footprint.
  bool unmapped_obstacle_filter = false;
  double map_roi_margin_m = 0.35, min_roi_source_fraction = 0.25;
  double max_unmapped_fraction = 0.65, static_support_distance_m = 0.75;
  double bbs_max_unmapped_fraction = 0.65;
  int bbs_min_matched_points = 20, min_unique_structure_voxels = 12;
  double structure_voxel_m = 0.30, min_structure_extent_m = 1.0;
  double bbs_normal_z_max = 0.85;
  double normal_radius_m = 0.45;
  double normal_consistency_rad = 0.5235987755982988;
  int min_consistent_structure_correspondences = 120;
  double min_consistent_structure_fraction = 0.05;
  int bbs_min_structure_points = 40;
  int gicp_iterations = 35, gicp_optimizer_iterations = 12, normal_min_neighbors = 6;
  double gicp_correspondence_m = 1., match_distance_m = 0.4;
  int min_correspondences = 120, min_nonground_correspondences = 120;
  double min_coverage = 0.65, max_rmse_m = 0.22, max_p95_m = 0.35;
  double min_nonground_fraction = 0.10, min_information_ratio = 0.0001;
  double ambiguity_coverage_margin = 0.04, ambiguity_rmse_margin_m = 0.04;
  double max_refine_translation_m = 1.2, max_refine_rotation_rad = 0.35;
  template<class F> void visit(F f) {
#define P(x) f(#x, x)
    P(backend); P(map_leaf); P(scan_leaf); P(bbs_scan_leaf); P(bbs_resolution);
    P(bbs_levels); P(bbs_threads); P(search_budget_ms); P(max_candidates);
    P(max_queue_size); P(bbs_min_hit_fraction); P(candidate_separation_m);
    P(candidate_separation_rad); P(search_min); P(search_max);
    P(map_gravity_z_signs); P(gravity_uncertainty_rad); P(roll_pitch_window_rad); P(yaw_min_rad); P(yaw_max_rad);
    P(min_range); P(max_range); P(outlier_radius); P(outlier_min_neighbors); P(min_points);
    P(map_abs_limit); P(bbs_use_nonground); P(bbs_filter_map_by_normals); P(map_filter_voxel_support); P(bbs_normal_z_max); P(normal_radius_m); P(bbs_min_structure_points); P(gicp_iterations); P(gicp_optimizer_iterations); P(normal_min_neighbors);
    P(unmapped_obstacle_filter); P(map_roi_margin_m); P(min_roi_source_fraction);
    P(max_unmapped_fraction); P(static_support_distance_m); P(bbs_max_unmapped_fraction);
    P(bbs_min_matched_points); P(min_unique_structure_voxels); P(structure_voxel_m); P(min_structure_extent_m);
    P(normal_consistency_rad); P(min_consistent_structure_correspondences); P(min_consistent_structure_fraction);
    P(gicp_correspondence_m); P(match_distance_m); P(min_correspondences); P(min_nonground_correspondences); P(min_coverage);
    P(max_rmse_m); P(max_p95_m); P(min_nonground_fraction); P(min_information_ratio);
    P(ambiguity_coverage_margin); P(ambiguity_rmse_margin_m); P(max_refine_translation_m);
    P(max_refine_rotation_rad);
#undef P
  }
  void validate() const;
};
struct Quality {
  size_t source_points = 0, correspondences = 0, nonground_correspondences = 0;
  size_t consistent_structure_correspondences = 0;
  size_t roi_source_points = 0, outside_map_points = 0, unmodeled_points = 0;
  size_t unique_structure_voxels = 0;
  double roi_coverage = 0., roi_source_fraction = 0., structure_extent_m = 0.;
  double consistent_structure_fraction = 0.;
  double coverage = 0., mse_m2 = INFINITY, rmse_m = INFINITY, p95_m = INFINITY;
  // Every unmatched source contributes match_distance_m^2 here, but not to inlier MSE.
  double capped_all_mse_m2 = INFINITY, nonground_fraction = 0., information_ratio = 0.;
  bool evaluated = false, converged = false, accepted = false;
  std::string reason = "not_evaluated";
};
struct Candidate {
  Matrix T_map_sensor = Matrix::Identity();
  Quality quality;
  double bbs_hit_fraction = NAN;
  double map_gravity_z_sign = NAN;
};
struct SearchResult {
  bool accepted = false, complete = false, ambiguous = false;
  std::string reason;
  std::vector<Candidate> candidates;
  double elapsed_ms = 0.;
};
struct CoarseResult {
  bool complete = false;
  std::string reason;
  std::vector<std::pair<Matrix, double>> poses;
};
// Backends own immutable per-map voxel structures and mutable per-job search state.
// One Engine has exactly one worker; no backend is concurrently entered.
class GlobalBackend {
public:
  virtual ~GlobalBackend() = default;
  virtual std::string name() const = 0;
  virtual CoarseResult search(const Cloud::ConstPtr& gravity_aligned, Clock::time_point deadline,
                              bool partial_overlap = false) = 0;
};
std::unique_ptr<GlobalBackend> make_backend(const Config&, const Cloud::ConstPtr& map);
Cloud::Ptr voxel(const Cloud::ConstPtr&, double leaf);
// Optional map acceleration proves dense-voxel radius support, then reduces the filtered map.
// Scans keep the default so quality still counts every valid source return.
Cloud::Ptr clean_cloud(const Cloud::ConstPtr&, const Config&, bool scan, double pre_filter_leaf = 0.);
bool rigid(const Matrix&);
double rotation_distance(const Matrix&, const Matrix&);
Matrix propagate(const Matrix& T_map_sensor_at_search, const Matrix& T_odom_sensor_at_search,
                 const Matrix& T_odom_sensor_now);
class Engine {
public:
  Engine(Config config, const std::string& map_file);
  ~Engine();
  Engine(const Engine&) = delete;
  Candidate refine(const Cloud::ConstPtr& all_valid_source, const Matrix& guess);
  Cloud::Ptr registration_source(const Cloud::ConstPtr& all_valid_source, const Matrix& guess) const;
  Quality evaluate(const Cloud::ConstPtr& all_valid_source, const Matrix& pose, bool converged);
  SearchResult search(const Cloud::ConstPtr& all_valid_source, const Matrix& R_gravity_sensor);
  Cloud::ConstPtr map() const;
  std::string version() const;
  double build_ms() const;
private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
std::shared_ptr<Engine> load_cached_engine(const Config&, const std::string&);
}  // namespace rc_bbs
