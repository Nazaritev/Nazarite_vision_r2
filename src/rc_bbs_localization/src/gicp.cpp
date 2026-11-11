#include "engine_impl.hpp"
#include <pcl/registration/gicp.h>
namespace rc_bbs {
namespace {
class CountedGicp : public pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ,pcl::PointXYZ> {
public:
  int iterations() const {return this->nr_iterations_;}
};
class PclGicp : public LocalRegistration {
  CountedGicp gicp_;
public:
  PclGicp(const Config& c,const Cloud::ConstPtr& target) {
    gicp_.setInputTarget(target);
    gicp_.setMaximumIterations(c.gicp_iterations);
    gicp_.setMaximumOptimizerIterations(c.gicp_optimizer_iterations);
    gicp_.setMaxCorrespondenceDistance(c.gicp_correspondence_m);
    gicp_.setTransformationEpsilon(1e-6);
    gicp_.setRotationEpsilon(1e-6);
    gicp_.setEuclideanFitnessEpsilon(1e-6);
  }
  std::pair<Matrix,bool> align(const Cloud::ConstPtr& source,const Matrix& guess) override {
    gicp_.setInputSource(source);
    Cloud output;
    gicp_.align(output,guess.cast<float>());
    // PCL's iteration-limit termination also sets hasConverged. Require it to finish before cap.
    return {gicp_.getFinalTransformation().cast<double>(),
      gicp_.hasConverged() && gicp_.iterations()<gicp_.getMaximumIterations()};
  }
};
}
std::unique_ptr<LocalRegistration> make_gicp(const Config& c,const Cloud::ConstPtr& target) {
  return std::make_unique<PclGicp>(c,target);
}
}
