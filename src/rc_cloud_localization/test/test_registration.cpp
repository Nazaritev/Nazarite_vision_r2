#include "rc_cloud_localization/registration.hpp"
#include "rc_cloud_localization/output_gate.hpp"
#include <gtest/gtest.h>
#include <pcl/common/transforms.h>
#include <random>

using namespace rc_cloud_localization;  // NOLINT

static Cloud::Ptr scene()
{
  // Asymmetric 3D geometry, independent of any real robot or user map.
  auto cloud = std::make_shared<Cloud>();
  std::mt19937 random(42);
  std::uniform_real_distribution<float> x(-4.0f, 5.0f), y(-3.0f, 4.0f), z(-1.0f, 2.0f);
  for (int i = 0; i < 4000; ++i) {
    cloud->push_back(pcl::PointXYZ(x(random), y(random), z(random)));
  }
  return cloud;
}

TEST(Registration, RecoversSixDofCorrectionForOdomFrameCloud)
{
  auto source = scene();
  auto target = std::make_shared<Cloud>();
  Eigen::Matrix4f expected = Eigen::Matrix4f::Identity();
  expected.block<3, 3>(0, 0) = (
    Eigen::AngleAxisf(0.12f, Eigen::Vector3f::UnitZ()) *
    Eigen::AngleAxisf(-0.04f, Eigen::Vector3f::UnitY()) *
    Eigen::AngleAxisf(0.03f, Eigen::Vector3f::UnitX())).toRotationMatrix();
  expected.block<3, 1>(0, 3) = Eigen::Vector3f(0.25, -0.12, 0.08);
  pcl::transformPointCloud(*source, *target, expected);
  RegistrationOptions options;
  options.iterations = 80;
  const auto result = register_cloud(source, target, Eigen::Matrix4f::Identity(), options);
  ASSERT_TRUE(acceptable(result, 0.9, 0.05));
  EXPECT_TRUE(result.transform.isApprox(expected, 1e-3));
}

TEST(Registration, RejectsNoOverlapEvenWhenBackendTerminates)
{
  auto source = scene();
  auto target = std::make_shared<Cloud>(*source);
  for (auto & point : *target) {point.x += 100.0f;}
  const auto result = register_cloud(source, target, Eigen::Matrix4f::Identity(), {});
  EXPECT_FALSE(acceptable(result, 0.65, 0.25));
  EXPECT_EQ(result.inlier_ratio, 0.0);
}

TEST(Registration, NdtProducesFiniteQualityForSeededSearch)
{
  auto source = scene();
  RegistrationOptions options;
  options.backend = "ndt";
  const auto result = register_cloud(source, source, Eigen::Matrix4f::Identity(), options);
  EXPECT_TRUE(result.transform.allFinite());
  EXPECT_TRUE(acceptable(result, 0.9, 0.1));
}

TEST(Registration, ConvergenceAloneCannotPassQualityGate)
{
  RegistrationResult result;
  result.converged = true;
  result.rmse = 0.02;
  result.inlier_ratio = 0.2;
  EXPECT_FALSE(acceptable(result, 0.65, 0.25));
  result.inlier_ratio = 0.9;
  result.rmse = 0.5;
  EXPECT_FALSE(acceptable(result, 0.65, 0.25));
  result.rmse = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(acceptable(result, 0.65, 0.25));
}

TEST(Frames, BasePoseSeedDoesNotDoubleApplyOdomToRegisteredCloud)
{
  Eigen::Matrix4f odom_base = Eigen::Matrix4f::Identity();
  odom_base.block<3, 3>(0, 0) = Eigen::AngleAxisf(0.6, Eigen::Vector3f::UnitZ()).toRotationMatrix();
  odom_base.block<3, 1>(0, 3) = Eigen::Vector3f(3, 2, 0.4);
  Eigen::Matrix4f map_base = Eigen::Matrix4f::Identity();
  map_base.block<3, 3>(0, 0) = Eigen::AngleAxisf(-0.4, Eigen::Vector3f::UnitZ()).toRotationMatrix();
  map_base.block<3, 1>(0, 3) = Eigen::Vector3f(-2, 4, 0.6);
  const auto correction = map_to_odom_seed(map_base, odom_base);
  EXPECT_TRUE((correction * odom_base).isApprox(map_base, 1e-5));
  EXPECT_FALSE(correction.isApprox(map_base));
}

TEST(OutputSafety, InitialPoseAndTfExistenceCannotAuthorizePublication)
{
  OutputGate gate;
  EXPECT_FALSE(gate.permits(0, 3, false, true, 0.1, 0.6));
  EXPECT_FALSE(gate.permits(2, 3, false, true, 0.1, 0.6));
  EXPECT_FALSE(gate.permits(3, 3, false, false, 0.1, 0.6));
  EXPECT_TRUE(gate.permits(3, 3, false, true, 0.1, 0.6));
}

TEST(OutputSafety, WithdrawsOutputOnLossTimeoutAndInvalidClock)
{
  OutputGate gate;
  EXPECT_FALSE(gate.permits(8, 3, true, true, 0.1, 0.6));
  EXPECT_FALSE(gate.permits(8, 3, false, true, 0.601, 0.6));
  EXPECT_FALSE(gate.permits(8, 3, false, true, -1.0, 0.6));
  EXPECT_FALSE(gate.permits(8, 3, false, true, NAN, 0.6));
}

TEST(OutputSafety, ConflictStaysLatchedDespiteFreshSuccessfulRegistrationAndReseed)
{
  OutputGate gate;
  ASSERT_TRUE(gate.permits(3, 3, false, true, 0.1, 0.6));
  EXPECT_TRUE(gate.latch_conflict());
  EXPECT_FALSE(gate.latch_conflict());
  EXPECT_TRUE(gate.conflict());
  EXPECT_FALSE(gate.permits(0, 3, false, true, 0.1, 0.6));
  EXPECT_FALSE(gate.permits(30, 3, false, true, 0.1, 0.6));
}
