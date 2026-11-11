#include "engine_impl.hpp"
#include <algorithm>
#include <Eigen/Eigenvalues>
#include <set>
#include <array>
namespace rc_bbs {
std::vector<Eigen::Vector3d> estimate_normals(const Cloud::ConstPtr& cloud,const Config& c) {
  std::vector<Eigen::Vector3d> normals(cloud->size(),Eigen::Vector3d::Zero());
  if(cloud->empty())return normals;
  pcl::KdTreeFLANN<pcl::PointXYZ> tree;tree.setInputCloud(cloud);
  std::vector<int> ids(c.normal_min_neighbors);std::vector<float> ds(c.normal_min_neighbors);
  for(size_t i=0;i<cloud->size();++i) {
    const auto& point=(*cloud)[i];
    int n=tree.radiusSearch(point,c.normal_radius_m,ids,ds);if(n<c.normal_min_neighbors)continue;
    Eigen::Vector3d mean=Eigen::Vector3d::Zero();
    for(int j=0;j<n;++j)mean+=(*cloud)[ids[j]].getVector3fMap().cast<double>();
    mean/=n;
    Eigen::Matrix3d cov=Eigen::Matrix3d::Zero();
    for(int j=0;j<n;++j) {Eigen::Vector3d d=(*cloud)[ids[j]].getVector3fMap().cast<double>()-mean;cov+=d*d.transpose();}
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(cov);
    if(eig.info()==Eigen::Success && eig.eigenvalues()(1)>1e-8)normals[i]=eig.eigenvectors().col(0);
  }
  return normals;
}
Cloud::Ptr nonground(const Cloud::ConstPtr& cloud,const Config& c) {
  const auto normals=estimate_normals(cloud,c);auto out=std::make_shared<Cloud>();
  for(size_t i=0;i<cloud->size();++i)
    if(normals[i].squaredNorm()>.5 && std::abs(normals[i].z())<c.bbs_normal_z_max)out->push_back((*cloud)[i]);
  return out;
}

Quality Engine::evaluate(const Cloud::ConstPtr& source,const Matrix& pose,bool converged) {
  Quality q;q.source_points=source->size();q.converged=converged;
  if(!rigid(pose)) {q.reason="nonfinite_or_nonrigid_pose";return q;}
  q.evaluated=true;
  const auto& c=impl_->config;
  auto source_small=voxel(source,c.scan_leaf);
  const auto source_normals=estimate_normals(source_small,c);
  pcl::KdTreeFLANN<pcl::PointXYZ> source_tree;
  if(!source_small->empty())source_tree.setInputCloud(source_small);
  std::vector<int> source_ids(1);std::vector<float> source_distances(1);
  const double cosine=std::cos(c.normal_consistency_rad);
  const double limit2=c.match_distance_m*c.match_distance_m;
  double error=0.,all_error=0.;size_t nonground=0, normals=0;
  std::vector<double> residuals;residuals.reserve(source->size());
  Eigen::Matrix<double,6,6> information=Eigen::Matrix<double,6,6>::Zero();
  std::vector<int> ids(1);std::vector<float> ds(1);
  std::set<std::array<long long,3>> structure_cells;
  Eigen::Vector3d structure_lo=Eigen::Vector3d::Constant(INFINITY),structure_hi=-structure_lo;
  double scale=0.,matched_radius2=0.;
  // Normalize after collecting genuine correspondences. Excluded obstacles
  // must not change the rotational lever-arm scale of retained map geometry.
  if(!c.unmapped_obstacle_filter) {
    for(const auto& p:*source)scale+=p.getVector3fMap().squaredNorm();
    scale=std::max(1.,std::sqrt(scale/std::max<size_t>(1,source->size())));
  } else scale=1.;
  for(const auto& p:*source) {
    Eigen::Vector3d local=p.getVector3fMap().cast<double>();
    Eigen::Vector3d rotated=pose.block<3,3>(0,0)*local;
    Eigen::Vector3d world=rotated+pose.block<3,1>(0,3);
    if(c.unmapped_obstacle_filter && !impl_->inside_roi(world)) {++q.outside_map_points;continue;}
    ++q.roi_source_points;
    pcl::PointXYZ point(world.x(),world.y(),world.z());
    if(impl_->tree.nearestKSearch(point,1,ids,ds)!=1 || ds[0]>limit2) {all_error+=limit2;continue;}
    Eigen::Vector3d source_normal=Eigen::Vector3d::Zero();
    if(!source_small->empty() && source_tree.nearestKSearch(p,1,source_ids,source_distances)==1)
      source_normal=pose.block<3,3>(0,0)*source_normals[source_ids[0]];
    const auto& normal=impl_->normals[ids[0]];
    if(c.unmapped_obstacle_filter && source_normal.squaredNorm()>.5 && normal.squaredNorm()>.5 &&
       std::abs(source_normal.dot(normal))<cosine) {all_error+=limit2;continue;}
    ++q.correspondences;error+=ds[0];all_error+=ds[0];residuals.push_back(std::sqrt(ds[0]));
    matched_radius2+=local.squaredNorm();
    if(normal.squaredNorm()<0.5) continue;
    ++normals;if(std::abs(normal.z())<0.8) ++nonground;
    // A source ground return near a map wall must not borrow the wall's
    // normal as evidence of 3D structure. Require support on BOTH surfaces.
    if(std::abs(normal.z())<0.8) {
      if(source_normal.squaredNorm()>.5 && std::abs(source_normal.z())<0.8 &&
         std::abs(source_normal.dot(normal))>=cosine) {
        ++q.consistent_structure_correspondences;
        const Eigen::Vector3d target=(*impl_->target)[ids[0]].getVector3fMap().cast<double>();
        std::array<long long,3> cell;
        for(int k=0;k<3;++k)cell[k]=std::llround(std::floor(target[k]/c.structure_voxel_m));
        structure_cells.insert(cell);structure_lo=structure_lo.cwiseMin(target);structure_hi=structure_hi.cwiseMax(target);
      }
    }
    Eigen::Matrix<double,6,1> j;
    j.head<3>()=normal;j.tail<3>()=rotated.cross(normal)/scale;
    information+=j*j.transpose();
  }
  if(!source->empty()) {
    q.coverage=double(q.correspondences)/source->size();
    q.roi_source_fraction=double(q.roi_source_points)/source->size();
  }
  q.unmodeled_points=q.roi_source_points-q.correspondences;
  if(q.roi_source_points) {
    q.roi_coverage=double(q.correspondences)/q.roi_source_points;
    q.capped_all_mse_m2=all_error/q.roi_source_points;
  }
  q.unique_structure_voxels=structure_cells.size();
  if(!structure_cells.empty())q.structure_extent_m=(structure_hi-structure_lo).norm();
  if(c.unmapped_obstacle_filter && q.correspondences) {
    const double static_scale=std::max(1.,std::sqrt(matched_radius2/q.correspondences));
    information.block<3,3>(0,3)/=static_scale;information.block<3,3>(3,0)/=static_scale;
    information.block<3,3>(3,3)/=(static_scale*static_scale);
  }
  if(q.correspondences>0) {
    q.mse_m2=error/q.correspondences;q.rmse_m=std::sqrt(q.mse_m2);
    size_t k=static_cast<size_t>(std::ceil(0.95*residuals.size()))-1;
    std::nth_element(residuals.begin(),residuals.begin()+k,residuals.end());q.p95_m=residuals[k];
    q.nonground_fraction=double(nonground)/q.correspondences;
    q.consistent_structure_fraction=double(q.consistent_structure_correspondences)/q.correspondences;
  }
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double,6,6>> eig(information);
  if(eig.info()==Eigen::Success && eig.eigenvalues()(5)>0 && normals>=20)
    q.information_ratio=std::max(0.,eig.eigenvalues()(0)/eig.eigenvalues()(5));
  q.nonground_correspondences=nonground;
  if(!converged) q.reason="gicp_not_converged_before_iteration_limit";
  else if(q.correspondences<static_cast<size_t>(c.min_correspondences)) q.reason="too_few_correspondences";
  else if(c.unmapped_obstacle_filter && q.roi_source_fraction<c.min_roi_source_fraction) q.reason="insufficient_scan_inside_map_roi";
  else if(c.unmapped_obstacle_filter && q.roi_coverage<1.-c.max_unmapped_fraction) q.reason="too_many_unmodeled_returns_inside_map";
  else if(!c.unmapped_obstacle_filter && q.coverage<c.min_coverage) q.reason="insufficient_full_scan_coverage";
  else if(q.rmse_m>c.max_rmse_m || q.p95_m>c.max_p95_m) q.reason="large_residuals";
  else if(q.nonground_correspondences<static_cast<size_t>(c.min_nonground_correspondences)) q.reason="insufficient_nonground_correspondences";
  else if(q.nonground_fraction<c.min_nonground_fraction) q.reason="ground_dominated";
  else if(q.consistent_structure_correspondences<static_cast<size_t>(c.min_consistent_structure_correspondences))
    q.reason="inconsistent_source_target_structure";
  else if(q.consistent_structure_fraction<c.min_consistent_structure_fraction)
    q.reason="insufficient_consistent_structure_fraction";
  else if(c.unmapped_obstacle_filter && q.unique_structure_voxels<static_cast<size_t>(c.min_unique_structure_voxels))
    q.reason="insufficient_independent_static_structure";
  else if(c.unmapped_obstacle_filter && q.structure_extent_m<c.min_structure_extent_m)
    q.reason="static_structure_support_too_concentrated";
  else if(q.information_ratio<c.min_information_ratio) q.reason="geometric_degeneracy";
  else {q.accepted=true;q.reason="quality_pass";}
  return q;
}
}
