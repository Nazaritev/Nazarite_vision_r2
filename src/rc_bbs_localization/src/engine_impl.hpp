#pragma once
#include "rc_bbs_localization/core.hpp"
#include <pcl/kdtree/kdtree_flann.h>
namespace rc_bbs {
Cloud::Ptr nonground(const Cloud::ConstPtr& cloud, const Config& config);
std::vector<Eigen::Vector3d> estimate_normals(const Cloud::ConstPtr&, const Config&);
struct LocalRegistration {
  virtual ~LocalRegistration() = default;
  virtual std::pair<Matrix,bool> align(const Cloud::ConstPtr&, const Matrix&) = 0;
};
std::unique_ptr<LocalRegistration> make_gicp(const Config&, const Cloud::ConstPtr&);
struct Engine::Impl {
  Config config;
  Cloud::Ptr target;
  pcl::KdTreeFLANN<pcl::PointXYZ> tree;
  Cloud::Ptr map_xy;
  pcl::KdTreeFLANN<pcl::PointXYZ> xy_tree;
  bool inside_roi(const Eigen::Vector3d& world) const;
  std::vector<Eigen::Vector3d> normals;
  std::unique_ptr<GlobalBackend> backend;
  std::unique_ptr<LocalRegistration> gicp;
  std::string map_version;
  double construction_ms = 0.;
};
}
