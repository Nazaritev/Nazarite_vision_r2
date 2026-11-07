#pragma once

#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <limits>
#include <string>

namespace rc_cloud_localization
{
using Cloud = pcl::PointCloud<pcl::PointXYZ>;
struct RegistrationOptions
{
  std::string backend = "icp";
  double correspondence_distance = 0.8;
  double ndt_resolution = 1.0;
  double ndt_step_size = 0.1;
  int iterations = 40;
};
struct RegistrationResult
{
  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
  bool converged = false;
  double inlier_ratio = 0.0;
  double rmse = std::numeric_limits<double>::infinity();
};
Cloud::Ptr downsample(const Cloud::ConstPtr & cloud, double leaf);
RegistrationResult register_cloud(
  const Cloud::ConstPtr & source, const Cloud::ConstPtr & target,
  const Eigen::Matrix4f & guess, const RegistrationOptions & options);
bool acceptable(const RegistrationResult & result, double min_inlier_ratio, double max_rmse);
// T_map_base = T_map_odom * T_odom_base, for clouds already expressed in odom.
Eigen::Matrix4f map_to_odom_seed(
  const Eigen::Matrix4f & map_to_base, const Eigen::Matrix4f & odom_to_base);
}  // namespace rc_cloud_localization
