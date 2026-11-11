#include "rc_bbs_localization/core.hpp"
#include <gtest/gtest.h>
#include <pcl/io/pcd_io.h>
#include <filesystem>
#include <random>
using namespace rc_bbs;

TEST(Transforms, PropagatesFullMotionAndMounting) {
  Matrix global=Matrix::Identity(),old=Matrix::Identity(),next=Matrix::Identity();
  global.block<3,3>(0,0)=Eigen::AngleAxisd(1.2,Eigen::Vector3d::UnitZ()).toRotationMatrix();global(0,3)=12.;
  old(0,3)=2.;next(0,3)=3.;next(1,3)=0.4;
  next.block<3,3>(0,0)=Eigen::AngleAxisd(0.3,Eigen::Vector3d::UnitX()).toRotationMatrix();
  EXPECT_TRUE(propagate(global,old,next).isApprox(global*old.inverse()*next));
  Matrix anchor=global*old.inverse();EXPECT_TRUE((anchor*next).isApprox(propagate(global,old,next)));
}
TEST(Transforms, RejectsInvalidAndNonRigid) {
  Matrix t=Matrix::Identity();EXPECT_TRUE(rigid(t));t(0,0)=2.;EXPECT_FALSE(rigid(t));
  t=Matrix::Identity();t(0,3)=NAN;EXPECT_FALSE(rigid(t));
}
TEST(Preprocessing, MapReductionRejectsIsolatedPointsAndPreservesScanCounts) {
  Config c;c.map_leaf=.1;c.outlier_radius=.35;c.outlier_min_neighbors=3;
  auto raw=std::make_shared<Cloud>();
  for(int i=-10;i<=10;++i)for(int j=-10;j<=10;++j)
    for(int k=0;k<8;++k)raw->emplace_back(i*.1f,j*.1f,1.f+k*.001f);
  raw->emplace_back(20.f,20.f,20.f);raw->emplace_back(NAN,0.f,0.f);
  auto map=clean_cloud(raw,c,false,c.map_leaf);
  auto scan=clean_cloud(raw,c,true);
  EXPECT_GT(map->size(),400u);EXPECT_LT(map->size(),500u);
  EXPECT_EQ(scan->size(),441u*8u);
  for(const auto& p:*map){EXPECT_TRUE(std::isfinite(p.x));EXPECT_LT(std::abs(p.x),2.f);}
}
TEST(Preprocessing, MapAccelerationUsesRawSupportForThinStructures) {
  Config c;c.outlier_radius=.35;c.outlier_min_neighbors=3;
  auto raw=std::make_shared<Cloud>();
  for(int k=0;k<8;++k)raw->emplace_back(4.f+k*.001f,0.f,1.f);
  // One occupied voxel has eight real returns: reduced-cloud density alone
  // would discard it, whereas the original radius filter keeps the cluster.
  auto map=clean_cloud(raw,c,false,.1);
  ASSERT_EQ(map->size(),1u);EXPECT_NEAR(map->front().x,4.0035f,1e-5f);
}
TEST(Preprocessing, AcceleratedMapFilterMatchesRawRadiusFilter) {
  Config c;c.outlier_radius=.35;c.outlier_min_neighbors=3;
  auto raw=std::make_shared<Cloud>();std::mt19937 gen(47);
  std::uniform_real_distribution<float> d(-2.f,2.f);
  for(int i=0;i<3000;++i)raw->emplace_back(d(gen),d(gen),d(gen));
  raw->emplace_back(20.f,20.f,20.f);
  for(double leaf:{.1,.3}) {
    auto expected=voxel(clean_cloud(raw,c,false),leaf);
    auto actual=clean_cloud(raw,c,false,leaf);
    ASSERT_EQ(expected->size(),actual->size());
    for(size_t i=0;i<expected->size();++i)
      EXPECT_TRUE((*expected)[i].getVector3fMap().isApprox((*actual)[i].getVector3fMap(),1e-6));
  }
}
TEST(Configuration, RefusesBackendFallbackAndInsufficientGravityWindow) {
  Config c;c.backend="official_gpu";EXPECT_THROW(c.validate(),std::invalid_argument);
  c=Config{};c.roll_pitch_window_rad=0.01;EXPECT_THROW(c.validate(),std::invalid_argument);
  c=Config{};c.search_min[0]=NAN;EXPECT_THROW(c.validate(),std::invalid_argument);
  c=Config{};c.max_unmapped_fraction=1.;EXPECT_THROW(c.validate(),std::invalid_argument);
  c=Config{};c.min_roi_source_fraction=0.;EXPECT_THROW(c.validate(),std::invalid_argument);
}
class MapFixture : public ::testing::Test {
protected:
  std::string path;
  Cloud::Ptr cloud=std::make_shared<Cloud>();
  Config config;
  void SetUp() override {
    path=(std::filesystem::temp_directory_path()/"rc_bbs_test_map.pcd").string();
    for(int i=-15;i<=15;++i)for(int j=-15;j<=15;++j) {
      cloud->emplace_back(i*.1f,j*.1f,0.f);
      cloud->emplace_back(-1.5f,i*.1f,(j+15)*.1f);
      cloud->emplace_back(i*.1f,1.5f,(j+15)*.1f);
    }
    pcl::io::savePCDFileBinary(path,*cloud);
    config.map_leaf=.08;config.outlier_radius=.3;config.outlier_min_neighbors=2;
  }
  void TearDown()override{std::filesystem::remove(path);}
};
TEST_F(MapFixture, CoverageIncludesUnmatchedPointsAndCapsAllPointMse) {
  Engine engine(config,path);auto source=std::make_shared<Cloud>(*cloud);
  for(size_t i=0;i<cloud->size()*3;++i)source->emplace_back(50.+i*.01,50.,50.);
  auto q=engine.evaluate(source,Matrix::Identity(),true);
  EXPECT_EQ(q.source_points,source->size());EXPECT_NEAR(q.coverage,0.25,0.001);
  EXPECT_LT(q.rmse_m,.06);EXPECT_GT(q.capped_all_mse_m2,.11);EXPECT_FALSE(q.accepted);
}
TEST_F(MapFixture, PartialMapIgnoresOutsideClutterButReportsRawCoverage) {
  config.unmapped_obstacle_filter=true;
  Engine engine(config,path);auto source=std::make_shared<Cloud>(*cloud);
  const auto clean_quality=engine.evaluate(cloud,Matrix::Identity(),true);
  for(size_t i=0;i<cloud->size()*2;++i)source->emplace_back(50.+i*.01,50.,50.);
  auto q=engine.evaluate(source,Matrix::Identity(),true);
  EXPECT_TRUE(q.accepted)<<q.reason;
  EXPECT_NEAR(q.coverage,1./3.,.001);EXPECT_GT(q.roi_coverage,.99);
  EXPECT_EQ(q.outside_map_points,cloud->size()*2);
  EXPECT_GE(q.unique_structure_voxels,12u);EXPECT_GT(q.structure_extent_m,1.);
  EXPECT_NEAR(q.information_ratio,clean_quality.information_ratio,1e-8);
}
TEST_F(MapFixture, PartialMapToleratesInteriorUnmodeledObjectsWithoutCountingThemAsMatches) {
  config.unmapped_obstacle_filter=true;
  Engine engine(config,path);auto source=std::make_shared<Cloud>(*cloud);
  for(int repeat=0;repeat<3;++repeat)
    for(int x=-15;x<=15;++x)for(int y=-15;y<=15;++y)source->emplace_back(x*.1f,y*.1f,6.f);
  auto q=engine.evaluate(source,Matrix::Identity(),true);
  EXPECT_TRUE(q.accepted)<<q.reason;EXPECT_GT(q.unmodeled_points,2500u);
  EXPECT_NEAR(q.roi_coverage,.5,.005);EXPECT_LT(q.rmse_m,.06);
  EXPECT_GE(q.unique_structure_voxels,12u);
}
TEST_F(MapFixture, PartialMapCannotAcceptTinySurvivingSubset) {
  config.unmapped_obstacle_filter=true;
  Engine engine(config,path);auto source=std::make_shared<Cloud>(*cloud);
  for(size_t i=0;i<cloud->size()*5;++i)source->emplace_back(50.+i*.01,50.,50.);
  auto q=engine.evaluate(source,Matrix::Identity(),true);
  EXPECT_FALSE(q.accepted);EXPECT_GT(q.roi_coverage,.99);
  EXPECT_EQ(q.reason,"insufficient_scan_inside_map_roi");
}
TEST_F(MapFixture, PartialMapDoesNotHideArbitraryInteriorMismatches) {
  config.unmapped_obstacle_filter=true;
  Engine engine(config,path);auto source=std::make_shared<Cloud>(*cloud);
  for(int repeat=0;repeat<8;++repeat)
    for(int x=-15;x<=15;++x)for(int y=-15;y<=15;++y)source->emplace_back(x*.1f,y*.1f,6.f);
  auto q=engine.evaluate(source,Matrix::Identity(),true);
  EXPECT_FALSE(q.accepted);EXPECT_EQ(q.reason,"too_many_unmodeled_returns_inside_map");
}
TEST_F(MapFixture, PartialMapGicpRejectsUnmodeledClutterBeforeFitting) {
  config.unmapped_obstacle_filter=true;config.gicp_iterations=60;
  Engine engine(config,path);auto source=std::make_shared<Cloud>(*cloud);
  for(int x=-15;x<=15;++x)for(int y=-15;y<=15;++y)source->emplace_back(x*.1f,y*.1f,6.f);
  Matrix initial=Matrix::Identity();initial(0,3)=.08;initial(1,3)=-.08;
  auto clean_fit=engine.refine(cloud,initial);
  auto fit=engine.refine(source,initial);
  EXPECT_LT((fit.T_map_sensor.block<3,1>(0,3)-clean_fit.T_map_sensor.block<3,1>(0,3)).norm(),.01);
  EXPECT_LT(rotation_distance(fit.T_map_sensor,clean_fit.T_map_sensor),.005);
  EXPECT_GE(fit.quality.unique_structure_voxels,12u);
  EXPECT_LT((fit.T_map_sensor.block<3,1>(0,3).norm()),.05);
}
TEST_F(MapFixture, RepeatedCoincidencesCannotReplaceIndependentMapStructure) {
  config.unmapped_obstacle_filter=true;config.scan_leaf=.04;
  auto source=std::make_shared<Cloud>();
  // Hundreds of repeated valid wall returns cover only a 24 cm square.
  // They pass a raw correspondence-count check but cannot certify a pose.
  for(int repeat=0;repeat<12;++repeat)
    for(int y=0;y<=6;++y)for(int z=0;z<=6;++z)
      source->emplace_back(-1.5f,-.95f+y*.04f,1.f+z*.04f);
  Engine engine(config,path);auto q=engine.evaluate(source,Matrix::Identity(),true);
  EXPECT_GE(q.consistent_structure_correspondences,120u);
  EXPECT_LT(q.unique_structure_voxels,12u);
  EXPECT_FALSE(q.accepted);EXPECT_EQ(q.reason,"insufficient_independent_static_structure");
}
TEST_F(MapFixture, RegistrationSourceKeepsArenaAndDropsOutsideAndUnsupportedInteriorObjects) {
  config.unmapped_obstacle_filter=true;
  Engine engine(config,path);auto source=std::make_shared<Cloud>(*cloud);
  // Same XY as the arena but a height absent from the static map.
  for(int x=-15;x<=15;++x)for(int y=-15;y<=15;++y)
    source->emplace_back(x*.1f,y*.1f,6.f);
  // An external obstacle with the same height as mapped arena structures.
  for(int y=-15;y<=15;++y)for(int z=0;z<=30;++z)
    source->emplace_back(10.f,y*.1f,z*.1f);
  const auto fit=engine.registration_source(source,Matrix::Identity());
  const auto clean=engine.registration_source(cloud,Matrix::Identity());
  ASSERT_GE(fit->size(),150u);
  EXPECT_GE(fit->size(),clean->size()*4/5);
  for(const auto& p:*fit) {
    EXPECT_LE(std::abs(p.x),1.501f);EXPECT_LE(std::abs(p.y),1.501f);
    EXPECT_GE(p.z,-.001f);EXPECT_LE(p.z,3.001f);
  }
  Matrix invalid=Matrix::Identity();invalid(0,3)=NAN;
  EXPECT_TRUE(engine.registration_source(source,invalid)->empty());
}
TEST_F(MapFixture, GroundPlaneRejectedEvenAtPerfectFitness) {
  auto source=std::make_shared<Cloud>();
  for(int i=-10;i<=10;++i)for(int j=-10;j<=10;++j)source->emplace_back(i*.1f,j*.1f,0.f);
  Engine engine(config,path);auto q=engine.evaluate(source,Matrix::Identity(),true);
  EXPECT_GT(q.coverage,.99);EXPECT_LT(q.rmse_m,.05);EXPECT_FALSE(q.accepted);
  EXPECT_LT(q.nonground_fraction,.12);
}
TEST_F(MapFixture, ConvergenceIsRequiredIndependentlyOfCoverage) {
  Engine engine(config,path);auto q=engine.evaluate(cloud,Matrix::Identity(),false);
  EXPECT_GT(q.coverage,.99);EXPECT_FALSE(q.accepted);EXPECT_EQ(q.reason,"gicp_not_converged_before_iteration_limit");
}
TEST_F(MapFixture, GroundScanCannotBorrowWallNormalsAsStructureEvidence) {
  auto wall=std::make_shared<Cloud>();
  for(int y=-15;y<=15;++y)for(int z=0;z<=30;++z)wall->emplace_back(0.f,y*.1f,z*.1f);
  pcl::io::savePCDFileBinary(path,*wall);
  auto source=std::make_shared<Cloud>();
  for(int x=-15;x<=15;++x)for(int y=-20;y<=20;++y)source->emplace_back(x*.01f,y*.05f,1.f);
  Engine engine(config,path);auto q=engine.evaluate(source,Matrix::Identity(),true);
  EXPECT_GT(q.coverage,.99);EXPECT_LT(q.rmse_m,.15);
  EXPECT_GT(q.nonground_correspondences,120u);
  EXPECT_EQ(q.consistent_structure_correspondences,0u);
  EXPECT_FALSE(q.accepted);EXPECT_EQ(q.reason,"inconsistent_source_target_structure");
}
TEST_F(MapFixture, PartialMapGroundCannotBorrowWallNormalsAfterObstacleFiltering) {
  config.unmapped_obstacle_filter=true;
  auto wall=std::make_shared<Cloud>();
  for(int y=-15;y<=15;++y)for(int z=0;z<=30;++z)wall->emplace_back(0.f,y*.1f,z*.1f);
  pcl::io::savePCDFileBinary(path,*wall);
  auto source=std::make_shared<Cloud>();
  for(int x=-15;x<=15;++x)for(int y=-20;y<=20;++y)source->emplace_back(x*.01f,y*.05f,1.f);
  Engine engine(config,path);auto q=engine.evaluate(source,Matrix::Identity(),true);
  EXPECT_GT(q.roi_source_fraction,.99);
  EXPECT_EQ(q.consistent_structure_correspondences,0u);
  EXPECT_FALSE(q.accepted);
}
TEST_F(MapFixture, MatchingThreeDimensionalStructureRetainsNormalSupport) {
  Engine engine(config,path);auto q=engine.evaluate(cloud,Matrix::Identity(),true);
  EXPECT_TRUE(q.accepted)<<q.reason;EXPECT_GT(q.consistent_structure_correspondences,120u);
}
TEST_F(MapFixture, DenseRepeatedGroundCannotDiluteStructureRequirement) {
  auto wall=std::make_shared<Cloud>();
  for(int y=-15;y<=15;++y)for(int z=0;z<=30;++z)wall->emplace_back(0.f,y*.1f,z*.1f);
  pcl::io::savePCDFileBinary(path,*wall);
  auto source=std::make_shared<Cloud>();
  for(int repeat=0;repeat<20;++repeat)
    for(int x=-15;x<=15;++x)for(int y=-20;y<=20;++y)source->emplace_back(x*.01f,y*.05f,1.f);
  for(int y=-10;y<=10;++y)for(int z=20;z<=30;++z)source->emplace_back(0.f,y*.05f,z*.1f);
  Engine engine(config,path);auto q=engine.evaluate(source,Matrix::Identity(),true);
  EXPECT_GE(q.consistent_structure_correspondences,120u);
  EXPECT_LT(q.consistent_structure_fraction,.02);EXPECT_FALSE(q.accepted);
  EXPECT_EQ(q.reason,"insufficient_consistent_structure_fraction");
}
TEST_F(MapFixture, SearchTimeoutNeverAccepted) {
  config.search_budget_ms=1;Engine engine(config,path);
  auto result=engine.search(cloud,Matrix::Identity());EXPECT_FALSE(result.accepted);
}
TEST(OfficialExtension, ReturnsDistinctRepeatedModesAndCompletesExclusionSearch) {
  Config c;c.bbs_resolution=.1;c.bbs_levels=4;c.max_candidates=2;c.bbs_min_hit_fraction=.95;
  c.roll_pitch_window_rad=0.;c.gravity_uncertainty_rad=0.;c.yaw_min_rad=-.01;c.yaw_max_rad=.01;
  c.search_min={-.1,-.1,-.1};c.search_max={8.1,.1,.1};
  c.candidate_separation_m=1.;c.candidate_separation_rad=.5;
  auto scan=std::make_shared<Cloud>(),map=std::make_shared<Cloud>();std::mt19937 gen(91);
  std::uniform_real_distribution<float> d(-1.5f,1.5f);
  for(int i=0;i<80;++i) {
    pcl::PointXYZ p(d(gen),d(gen),d(gen));scan->push_back(p);map->push_back(p);p.x+=8.;map->push_back(p);
  }
  auto backend=make_backend(c,map);
  auto result=backend->search(scan,Clock::now()+std::chrono::seconds(4));
  ASSERT_TRUE(result.complete)<<result.reason;ASSERT_EQ(result.poses.size(),2u);
  EXPECT_GT((result.poses[0].first.block<3,1>(0,3)-result.poses[1].first.block<3,1>(0,3)).norm(),7.);
}
TEST(OfficialExtension, PartialOverlapFindsStaticSupportWithoutCountingExternalObstacles) {
  Config c;c.bbs_resolution=.1;c.bbs_levels=4;c.max_candidates=2;c.bbs_min_hit_fraction=.95;
  c.roll_pitch_window_rad=0.;c.gravity_uncertainty_rad=0.;c.yaw_min_rad=-.001;c.yaw_max_rad=.001;
  c.search_min={-.01,-.01,-.01};c.search_max={.01,.01,.01};
  auto scan=std::make_shared<Cloud>(),map=std::make_shared<Cloud>();std::mt19937 gen(93);
  std::uniform_real_distribution<float> d(-1.5f,1.5f);
  for(int i=0;i<80;++i){pcl::PointXYZ p(d(gen),d(gen),d(gen));scan->push_back(p);map->push_back(p);}
  for(int i=0;i<120;++i)scan->emplace_back(20.f+d(gen),20.f+d(gen),20.f+d(gen));
  auto strict=make_backend(c,map)->search(scan,Clock::now()+std::chrono::seconds(2));
  ASSERT_TRUE(strict.complete);EXPECT_TRUE(strict.poses.empty());
  c.unmapped_obstacle_filter=true;
  auto partial=make_backend(c,map)->search(scan,Clock::now()+std::chrono::seconds(2),true);
  ASSERT_TRUE(partial.complete)<<partial.reason;ASSERT_EQ(partial.poses.size(),1u);
  EXPECT_NEAR(partial.poses.front().second,.4,.01);
}
TEST(OfficialExtension, NarrowSignedTranslationBoundsRetainLeafAcrossLevels) {
  auto scan=std::make_shared<Cloud>();std::mt19937 gen(96);
  std::uniform_real_distribution<float> d(-1.5f,1.5f);
  for(int i=0;i<80;++i)scan->emplace_back(d(gen),d(gen),d(gen));
  for(const Eigen::Vector3d truth:{Eigen::Vector3d(-.8,-.4,.2),Eigen::Vector3d(.8,.4,-.2)}) {
    auto map=std::make_shared<Cloud>();
    for(const auto& p:*scan)map->emplace_back(p.x+truth.x(),p.y+truth.y(),p.z+truth.z());
    for(int levels:{1,3,5}) {
      SCOPED_TRACE(::testing::Message()<<"levels="<<levels<<" translation="<<truth.transpose());
      Config c;c.bbs_resolution=.2;c.bbs_levels=levels;c.max_candidates=2;c.bbs_min_hit_fraction=.95;
      c.roll_pitch_window_rad=0.;c.gravity_uncertainty_rad=0.;c.yaw_min_rad=-.001;c.yaw_max_rad=.001;
      for(int k=0;k<3;++k){c.search_min[k]=truth[k]-.01;c.search_max[k]=truth[k]+.01;}
      auto result=make_backend(c,map)->search(scan,Clock::now()+std::chrono::seconds(2));
      ASSERT_TRUE(result.complete)<<result.reason;ASSERT_EQ(result.poses.size(),1u);
      EXPECT_LT((result.poses.front().first.block<3,1>(0,3)-truth).norm(),1e-6);
    }
  }
}

TEST_F(MapFixture, MapCacheVersionAndLifetime) {
  auto first=load_cached_engine(config,path),second=load_cached_engine(config,path);
  EXPECT_EQ(first.get(),second.get());const auto old_version=first->version();
  for(auto& p:*cloud)p.x+=.123f;
  pcl::io::savePCDFileBinary(path,*cloud);
  auto changed=load_cached_engine(config,path);
  EXPECT_NE(changed.get(),first.get());EXPECT_NE(changed->version(),old_version);
  EXPECT_FALSE(first->map()->empty()); // readers of the old immutable snapshot stay valid
}
TEST_F(MapFixture, AmbiguousRepeatedRoomsNeverBecomeGlobalSuccess) {
  auto repeated=std::make_shared<Cloud>(*cloud);
  for(auto p:*cloud){p.x+=8.f;repeated->push_back(p);}
  pcl::io::savePCDFileBinary(path,*repeated);
  config.search_min={-1.,-1.,-1.};config.search_max={9.,1.,1.};
  config.bbs_resolution=.2;config.bbs_scan_leaf=.2;config.bbs_min_hit_fraction=.9;
  config.roll_pitch_window_rad=0.;config.gravity_uncertainty_rad=0.;
  config.yaw_min_rad=-.01;config.yaw_max_rad=.01;config.search_budget_ms=2000;
  Engine engine(config,path);auto result=engine.search(cloud,Matrix::Identity());
  EXPECT_FALSE(result.accepted);
  // Either all plausible modes are refined and rejected as ambiguous, or an
  // explicit finite budget prevents a complete global decision. Both fail closed.
  EXPECT_TRUE(result.ambiguous || !result.complete);
}
TEST_F(MapFixture, ExplicitNegativeMapGravityProducesMapSensorTransform) {
  config.map_gravity_z_signs={-1.};config.roll_pitch_window_rad=0.;config.gravity_uncertainty_rad=0.;
  config.yaw_min_rad=-.01;config.yaw_max_rad=.01;config.search_min={-.2,-.2,-.2};config.search_max={.2,.2,.2};
  config.bbs_resolution=.2;config.bbs_scan_leaf=.2;config.bbs_min_hit_fraction=.9;config.search_budget_ms=2000;
  Matrix truth=Matrix::Identity();truth.block<3,3>(0,0)=Eigen::AngleAxisd(M_PI,Eigen::Vector3d::UnitY()).toRotationMatrix();
  auto source=std::make_shared<Cloud>();
  for(const auto& p:*cloud){Eigen::Vector3d v=truth.block<3,3>(0,0).transpose()*p.getVector3fMap().cast<double>();source->emplace_back(v.x(),v.y(),v.z());}
  Engine engine(config,path);auto result=engine.search(source,Matrix::Identity());
  ASSERT_TRUE(result.accepted)<<result.reason;ASSERT_FALSE(result.candidates.empty());
  EXPECT_LT(rotation_distance(result.candidates.front().T_map_sensor,truth),.05);
  EXPECT_LT((result.candidates.front().T_map_sensor.block<3,1>(0,3)).norm(),.05);
}
