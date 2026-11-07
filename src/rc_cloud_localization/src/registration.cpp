#include "rc_cloud_localization/registration.hpp"

#include <pcl/common/transforms.h>
#include <pcl/common/point_tests.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <cmath>
#include <stdexcept>

namespace rc_cloud_localization
{
Cloud::Ptr downsample(const Cloud::ConstPtr & cloud, double leaf)
{
  auto output = std::make_shared<Cloud>();
  pcl::VoxelGrid<pcl::PointXYZ> filter;
  filter.setLeafSize(leaf, leaf, leaf);
  filter.setInputCloud(cloud);
  filter.filter(*output);
  return output;
}

RegistrationResult register_cloud(
  const Cloud::ConstPtr & source, const Cloud::ConstPtr & target,
  const Eigen::Matrix4f & guess, const RegistrationOptions & options)
{
  RegistrationResult result;
  if (source->empty() || target->empty() || !guess.allFinite()) {
    return result;
  }
  Cloud aligned;
  if (options.backend == "icp") {
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> registration;
    registration.setInputSource(source);
    registration.setInputTarget(target);
    registration.setMaximumIterations(options.iterations);
    registration.setMaxCorrespondenceDistance(options.correspondence_distance);
    registration.setTransformationEpsilon(1e-6);
    registration.setEuclideanFitnessEpsilon(1e-6);
    registration.align(aligned, guess);
    result.converged = registration.hasConverged();
    result.transform = registration.getFinalTransformation();
  } else if (options.backend == "ndt") {
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> registration;
    registration.setInputSource(source);
    registration.setInputTarget(target);
    registration.setMaximumIterations(options.iterations);
    registration.setResolution(options.ndt_resolution);
    registration.setStepSize(options.ndt_step_size);
    registration.setTransformationEpsilon(1e-4);
    registration.align(aligned, guess);
    result.converged = registration.hasConverged();
    result.transform = registration.getFinalTransformation();
  } else {
    throw std::invalid_argument("relocalization_backend must be icp or ndt");
  }
  if (!result.transform.allFinite()) {
    result.converged = false;
    return result;
  }
  // Both backends use the same, explicitly bounded nearest-neighbour quality metric.
  // PCL's hasConverged alone is not evidence of a usable map match.
  pcl::transformPointCloud(*source, aligned, result.transform);
  pcl::KdTreeFLANN<pcl::PointXYZ> tree;
  tree.setInputCloud(target);
  std::vector<int> indices(1);
  std::vector<float> distances(1);
  size_t inliers = 0;
  double squared_error = 0.0;
  const double limit = options.correspondence_distance * options.correspondence_distance;
  for (const auto & point : aligned) {
    if (pcl::isFinite(point) && tree.nearestKSearch(point, 1, indices, distances) == 1 &&
      distances[0] <= limit)
    {
      ++inliers;
      squared_error += distances[0];
    }
  }
  result.inlier_ratio = static_cast<double>(inliers) / source->size();
  if (inliers > 0) {
    result.rmse = std::sqrt(squared_error / inliers);
  }
  return result;
}

bool acceptable(const RegistrationResult & result, double min_inlier_ratio, double max_rmse)
{
  return result.converged && result.transform.allFinite() && std::isfinite(result.rmse) &&
         result.inlier_ratio >= min_inlier_ratio && result.rmse <= max_rmse;
}

Eigen::Matrix4f map_to_odom_seed(
  const Eigen::Matrix4f & map_to_base, const Eigen::Matrix4f & odom_to_base)
{
  return map_to_base * odom_to_base.inverse();
}
}  // namespace rc_cloud_localization
