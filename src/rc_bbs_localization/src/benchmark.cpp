#include "engine_impl.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <fstream>
#include <random>
#include <iostream>
#include <iomanip>
#include <type_traits>
#include <algorithm>
namespace fs=std::filesystem;
using namespace rc_bbs;
struct Case {std::string id;Cloud::Ptr scan;Matrix truth=Matrix::Identity(),gravity=Matrix::Identity();bool has_truth=false;};
void load(Config& c,const YAML::Node& values) {
  c.visit([&](const char* k,auto& v){using T=std::decay_t<decltype(v)>;if(values[k])v=values[k].as<T>();});c.validate();
}
int main(int argc,char** argv) {
  if(argc!=2){std::cerr<<"Validation wrapper supplies a YAML path\n";return 2;}
  try {
    auto y=YAML::LoadFile(argv[1]);Config config;
    load(config,YAML::LoadFile(y["localization_config"].as<std::string>())["bbs_localizer"]["ros__parameters"]);
    const std::string map_file=y["map_file"].as<std::string>();
    fs::path output=y["output_dir"].as<std::string>();fs::create_directories(output/"fixtures");
    auto raw=std::make_shared<Cloud>();if(pcl::io::loadPCDFile(map_file,*raw)<0)return 2;
    auto map=clean_cloud(raw,config,false);pcl::PointXYZ lo,hi;pcl::getMinMax3D(*map,lo,hi);
    std::ofstream stats(output/"map_stats.yaml");
    stats<<"raw_points: "<<raw->size()<<"\nvalid_points: "<<map->size()<<"\nmin_xyz: ["<<lo.x<<", "<<lo.y<<", "<<lo.z<<"]\nmax_xyz: ["<<hi.x<<", "<<hi.y<<", "<<hi.z<<"]\n";
    std::cout<<"Map extent: "<<lo<<" -> "<<hi<<std::endl;
    std::vector<Case> cases;
    std::mt19937 rng(y["random_seed"].as<int>());
    const double gravity_error=y["synthetic_gravity_error_rad"] ? y["synthetic_gravity_error_rad"].as<double>() : 0.;
    std::uniform_real_distribution<double> unit(0.,1.);
    std::normal_distribution<double> noise(0.,y["synthetic_noise_std_m"].as<double>());
    std::vector<float> zs;for(const auto& p:*map)zs.push_back(p.z);std::sort(zs.begin(),zs.end());
    const double floor=zs[zs.size()/20];
    std::ofstream manifest(output/"fixtures"/"manifest.yaml");manifest<<"cases:\n";
    for(int i=0;i<y["synthetic_count"].as<int>();++i) {
      Case c;c.id="synthetic_"+std::to_string(i);c.has_truth=true;
      const auto& chosen=(*map)[rng()%map->size()];
      c.truth.block<3,1>(0,3)=Eigen::Vector3d(chosen.x,chosen.y,floor+y["synthetic_sensor_height_m"].as<double>());
      double yaw=(unit(rng)*2-1)*M_PI,roll=(unit(rng)*2-1)*0.20,pitch=(unit(rng)*2-1)*0.20;
      c.truth.block<3,3>(0,0)=(Eigen::AngleAxisd(yaw,Eigen::Vector3d::UnitZ())*
        Eigen::AngleAxisd(pitch,Eigen::Vector3d::UnitY())*Eigen::AngleAxisd(roll,Eigen::Vector3d::UnitX())).toRotationMatrix();
      c.gravity.block<3,3>(0,0)=(Eigen::AngleAxisd(pitch,Eigen::Vector3d::UnitY())*Eigen::AngleAxisd(roll,Eigen::Vector3d::UnitX())).toRotationMatrix();
      Matrix prior_error=Matrix::Identity();
      prior_error.block<3,3>(0,0)=(Eigen::AngleAxisd((i%2?1.:-1.)*gravity_error,Eigen::Vector3d::UnitX())*
        Eigen::AngleAxisd((i%3?1.:-1.)*gravity_error,Eigen::Vector3d::UnitY())).toRotationMatrix();
      c.gravity=prior_error*c.gravity;
      // Match the actual inverted MID360 mounting. Truth remains test-only.
      Matrix mounting=Matrix::Identity();
      mounting.block<3,3>(0,0)=Eigen::AngleAxisd(M_PI,Eigen::Vector3d::UnitY()).toRotationMatrix();
      c.truth=c.truth*mounting;c.gravity=c.gravity*mounting;
      auto visible=std::make_shared<Cloud>();double radius=y["synthetic_radius_m"].as<double>();
      for(const auto& p:*map)if((p.getVector3fMap().cast<double>()-c.truth.block<3,1>(0,3)).norm()<radius)visible->push_back(p);
      visible=voxel(visible,0.18);c.scan=std::make_shared<Cloud>();Matrix inverse=c.truth.inverse();pcl::transformPointCloud(*visible,*c.scan,inverse);
      for(auto& p:*c.scan){p.x+=noise(rng);p.y+=noise(rng);p.z+=noise(rng);}
      size_t extra=c.scan->size()*y["synthetic_outlier_fraction"].as<double>();
      for(size_t j=0;j<extra;++j)c.scan->emplace_back((unit(rng)*2-1)*radius,(unit(rng)*2-1)*radius,(unit(rng)*2-1)*radius);
      pcl::io::savePCDFileBinary((output/"fixtures"/(c.id+".pcd")).string(),*c.scan);
      Eigen::Quaterniond q(c.truth.block<3,3>(0,0));Eigen::Vector3d acc=c.gravity.block<3,3>(0,0).transpose()*Eigen::Vector3d(0,0,9.81);
      manifest<<"  - id: "<<c.id<<"\n    pcd: "<<(output/"fixtures"/(c.id+".pcd")).string()<<"\n    xyz: ["<<c.truth(0,3)<<", "<<c.truth(1,3)<<", "<<c.truth(2,3)<<"]\n    quaternion_xyzw: ["<<q.x()<<", "<<q.y()<<", "<<q.z()<<", "<<q.w()<<"]\n    acceleration: ["<<acc.x()<<", "<<acc.y()<<", "<<acc.z()<<"]\n";
      cases.push_back(c);
    }
    if(fs::exists(output/"bag_scans"/"manifest.yaml")) {
      auto list=YAML::LoadFile((output/"bag_scans"/"manifest.yaml").string());
      for(const auto& v:list["cases"]) {
        Case c;c.id=v["id"].as<std::string>();auto cloud=std::make_shared<Cloud>();
        if(pcl::io::loadPCDFile(v["pcd"].as<std::string>(),*cloud)<0)continue;
        c.scan=cloud;
        Eigen::Vector3d acc;for(int k=0;k<3;++k)acc[k]=v["acceleration_sensor"][k].as<double>();
        if(std::abs(acc.norm()-9.81)>config.gravity_uncertainty_rad*9.81)continue;
        c.gravity.block<3,3>(0,0)=Eigen::Quaterniond::FromTwoVectors(acc,Eigen::Vector3d::UnitZ()).toRotationMatrix();
        cases.push_back(c);
      }
    }
    std::ofstream csv(output/"benchmark.csv");
    csv<<"profile,case,has_truth,valid_source,cold_map_ms,search_ms,complete,accepted,reason,candidate_count,bbs_hit_fraction,coverage,correspondences,inlier_mse_m2,inlier_rmse_m,inlier_p95_m,capped_all_mse_m2,nonground_fraction,information_ratio,translation_error_m,rotation_error_rad,consistent_structure_correspondences,consistent_structure_fraction,roi_source_points,outside_map_points,unmodeled_points,roi_coverage,roi_source_fraction,unique_structure_voxels,structure_extent_m\n";
    std::ofstream diagnostics(output/"diagnostic_candidates.csv");
    diagnostics<<"profile,case,coarse_hits,coverage,correspondences,rmse_m,p95_m,nonground_fraction,information_ratio,converged,quality_reason,global_result_accepted,consistent_structure_correspondences,consistent_structure_fraction,roi_source_points,outside_map_points,unmodeled_points,roi_coverage,roi_source_fraction,unique_structure_voxels,structure_extent_m\n";
    YAML::Emitter poses;poses<<YAML::BeginMap<<YAML::Key<<"candidates"<<YAML::Value<<YAML::BeginSeq;
    for(const auto& profile:y["profiles"]) {
      Config c=config;load(c,profile);auto engine=std::make_shared<Engine>(c,map_file);
      auto mapq=engine->evaluate(engine->map(),Matrix::Identity(),true);
      std::cout<<"Map geometry: nonground="<<mapq.nonground_fraction<<" info="<<mapq.information_ratio
        <<" target_points="<<engine->map()->size()<<std::endl;
      fs::path diagnostic_dir=output/"diagnostic_clouds"/profile["name"].as<std::string>();
      if(profile["diagnostic_only"] && profile["diagnostic_only"].as<bool>()) {
        fs::create_directories(diagnostic_dir);
        pcl::io::savePCDFileBinary((diagnostic_dir/"map.pcd").string(),*engine->map());
        auto structure=voxel(engine->map(),std::min(c.scan_leaf,c.bbs_scan_leaf));
        if(c.bbs_filter_map_by_normals)structure=nonground(structure,c);
        pcl::io::savePCDFileBinary((diagnostic_dir/"bbs_map.pcd").string(),*structure);
      }
      for(const auto& test:cases) {
        if(profile["real_only"] && profile["real_only"].as<bool>() && test.has_truth)continue;
        if(y["case_ids"]) {
          const auto selected=y["case_ids"].as<std::vector<std::string>>();
          if(std::find(selected.begin(),selected.end(),test.id)==selected.end())continue;
        }
        // Preprocessing belongs to the tested profile. Applying only the base
        // config here made range/outlier tuning silently use the wrong scan.
        auto source=clean_cloud(test.scan,c,true);
        if(profile["diagnostic_only"] && profile["diagnostic_only"].as<bool>()) {
          pcl::io::savePCDFileBinary((diagnostic_dir/(test.id+"_sensor.pcd")).string(),*source);
          auto structure=std::make_shared<Cloud>();pcl::transformPointCloud(*source,*structure,test.gravity);
          structure=voxel(structure,std::min(c.scan_leaf,c.bbs_scan_leaf));
          if(c.bbs_use_nonground)structure=nonground(structure,c);
          structure=voxel(structure,c.bbs_scan_leaf);
          if(!structure->empty())pcl::io::savePCDFileBinary((diagnostic_dir/(test.id+"_bbs_up.pcd")).string(),*structure);
        }
        if(test.has_truth) {
          auto oracle=engine->evaluate(source,test.truth,true);
          std::cout<<"Oracle "<<test.id<<" coverage="<<oracle.coverage<<" nonground="<<oracle.nonground_fraction
            <<" info="<<oracle.information_ratio<<" quality="<<oracle.reason<<std::endl;
        }
        auto result=engine->search(source,test.gravity);Candidate best;
        if(!result.candidates.empty())best=result.candidates.front();
        if(profile["diagnostic_only"] && profile["diagnostic_only"].as<bool>()) {
          for(const auto& hint:result.candidates) {
            auto d=engine->refine(source,hint.T_map_sensor);const auto& q=d.quality;
            diagnostics<<profile["name"].as<std::string>()<<','<<test.id<<','<<hint.bbs_hit_fraction<<','
              <<q.coverage<<','<<q.correspondences<<','<<q.rmse_m<<','<<q.p95_m<<','<<q.nonground_fraction<<','
              <<q.information_ratio<<','<<q.converged<<','<<q.reason<<','<<result.accepted<<','<<q.consistent_structure_correspondences<<','<<q.consistent_structure_fraction<<','<<q.roi_source_points<<','<<q.outside_map_points<<','<<q.unmodeled_points<<','<<q.roi_coverage<<','<<q.roi_source_fraction<<','<<q.unique_structure_voxels<<','<<q.structure_extent_m<<'\n';diagnostics.flush();
            poses<<YAML::BeginMap<<YAML::Key<<"profile"<<YAML::Value<<profile["name"].as<std::string>()
              <<YAML::Key<<"case"<<YAML::Value<<test.id
              <<YAML::Key<<"global_accepted"<<YAML::Value<<result.accepted
              <<YAML::Key<<"map_gravity_z_sign"<<YAML::Value<<hint.map_gravity_z_sign
              <<YAML::Key<<"quality_reason"<<YAML::Value<<q.reason
              <<YAML::Key<<"consistent_structure_correspondences"<<YAML::Value<<q.consistent_structure_correspondences
              <<YAML::Key<<"consistent_structure_fraction"<<YAML::Value<<q.consistent_structure_fraction
              <<YAML::Key<<"T_map_sensor_row_major"<<YAML::Value<<YAML::Flow<<YAML::BeginSeq;
            for(int row=0;row<4;++row)for(int col=0;col<4;++col)poses<<d.T_map_sensor(row,col);
            poses<<YAML::EndSeq<<YAML::EndMap;
          }
        }
        double te=NAN,re=NAN;
        if(test.has_truth && !result.candidates.empty()) {
          te=(test.truth.block<3,1>(0,3)-best.T_map_sensor.block<3,1>(0,3)).norm();re=rotation_distance(test.truth,best.T_map_sensor);
        }
        const auto& q=best.quality;
        csv<<profile["name"].as<std::string>()<<','<<test.id<<','<<test.has_truth<<','<<source->size()<<','<<engine->build_ms()<<','
          <<result.elapsed_ms<<','<<result.complete<<','<<result.accepted<<','<<result.reason<<','<<result.candidates.size()<<','
          <<best.bbs_hit_fraction<<','<<q.coverage<<','<<q.correspondences<<','<<q.mse_m2<<','<<q.rmse_m<<','<<q.p95_m<<','
          <<q.capped_all_mse_m2<<','<<q.nonground_fraction<<','<<q.information_ratio<<','<<te<<','<<re<<','<<q.consistent_structure_correspondences<<','<<q.consistent_structure_fraction<<','<<q.roi_source_points<<','<<q.outside_map_points<<','<<q.unmodeled_points<<','<<q.roi_coverage<<','<<q.roi_source_fraction<<','<<q.unique_structure_voxels<<','<<q.structure_extent_m<<'\n';csv.flush();
        std::cout<<profile["name"]<<" "<<test.id<<" "<<result.elapsed_ms<<"ms "<<result.reason<<" error="<<te<<std::endl;
      }
    }
    poses<<YAML::EndSeq<<YAML::EndMap;
    std::ofstream(output/"diagnostic_poses.yaml")<<poses.c_str()<<'\n';
  }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
