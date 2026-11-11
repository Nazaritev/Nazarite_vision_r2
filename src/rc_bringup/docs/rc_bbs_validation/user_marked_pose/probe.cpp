// Offline diagnostic only: no ROS publisher, no global-search acceptance.
#include "rc_bbs_localization/core.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <algorithm>
#include <type_traits>
using namespace rc_bbs;
Matrix matrix(const YAML::Node& v){Matrix m;for(int r=0;r<4;++r)for(int c=0;c<4;++c)m(r,c)=v[4*r+c].as<double>();return m;}
void emit(YAML::Emitter& out,const Matrix& m){out<<YAML::Flow<<YAML::BeginSeq;for(int r=0;r<4;++r)for(int c=0;c<4;++c)out<<m(r,c);out<<YAML::EndSeq;}
int main(int argc,char** argv){try{
 if(argc!=2)return 2;auto y=YAML::LoadFile(argv[1]);Config base;
 auto cfg=YAML::LoadFile(y["localization_config"].as<std::string>())["bbs_localizer"]["ros__parameters"];
 base.visit([&](const char* k,auto& v){using T=std::decay_t<decltype(v)>;if(cfg[k])v=cfg[k].as<T>();});
 const auto output=std::filesystem::path(y["output_dir"].as<std::string>());
 const Matrix sensor_foot=matrix(y["T_base_sensor_row_major"]).inverse()*matrix(y["T_base_footprint_row_major"]);
 std::ofstream csv(output/"probe.csv");csv<<"case,max_range_m,gravity_sign,stage,initial_sensor_z_m,x,y,z,footprint_x,footprint_y,footprint_heading_rad,coverage,rmse_m,p95_m,nonground_fraction,consistent_structure_correspondences,consistent_structure_fraction,information_ratio,quality_reason,local_quality_pass,translation_correction_m,rotation_correction_rad,roi_source_points,outside_map_points,unmodeled_points,roi_coverage,roi_source_fraction,unique_structure_voxels,structure_extent_m\n";
 YAML::Emitter poses;poses<<YAML::BeginMap<<YAML::Key<<"diagnostic_only"<<YAML::Value<<true<<YAML::Key<<"poses"<<YAML::Value<<YAML::BeginSeq;
 size_t pose_index=0;
 for(double range:y["max_ranges_m"].as<std::vector<double>>()){
  Config c=base;c.max_range=range;c.validate();Engine engine(c,y["map_file"].as<std::string>());
  pcl::io::savePCDFileBinary((output/"map.pcd").string(),*engine.map());
  for(const auto& manifest:y["scan_manifests"]){auto scans=YAML::LoadFile(manifest.as<std::string>());
   for(const auto& scan:scans["cases"]){std::string id=scan["id"].as<std::string>();auto selected=y["case_ids"].as<std::vector<std::string>>();if(std::find(selected.begin(),selected.end(),id)==selected.end())continue;
    auto raw=std::make_shared<Cloud>();if(pcl::io::loadPCDFile(scan["pcd"].as<std::string>(),*raw)<0)return 2;auto source=clean_cloud(raw,c,true);
    pcl::io::savePCDFileBinary((output/(id+"_range"+std::to_string(int(range))+"_sensor.pcd")).string(),*source);
    Eigen::Vector3d acc;for(int k=0;k<3;++k)acc[k]=scan["acceleration_sensor"][k].as<double>();
    for(double sign:y["map_gravity_z_signs"].as<std::vector<double>>()){
     const Eigen::Vector3d up(0.,0.,sign);Eigen::Matrix3d gravity=Eigen::Quaterniond::FromTwoVectors(acc,up).toRotationMatrix();
     const Eigen::Vector3d forward=gravity*sensor_foot.block<3,1>(0,0);
     const double delta=y["approximate_footprint_heading_rad"].as<double>()-std::atan2(forward.y(),forward.x());
     const Eigen::Matrix3d rotation=Eigen::AngleAxisd(delta,Eigen::Vector3d::UnitZ()).toRotationMatrix()*gravity;
     std::vector<std::pair<Matrix,Quality>> sweep;
     auto record=[&](const char* stage,const Matrix& pose,const Quality& q,const Matrix& initial){
      Matrix foot=pose*sensor_foot;
      csv<<id<<','<<range<<','<<sign<<','<<stage<<','<<initial(2,3)<<','<<pose(0,3)<<','<<pose(1,3)<<','<<pose(2,3)<<','<<foot(0,3)<<','<<foot(1,3)<<','<<std::atan2(foot(1,0),foot(0,0))<<','<<q.coverage<<','<<q.rmse_m<<','<<q.p95_m<<','<<q.nonground_fraction<<','<<q.consistent_structure_correspondences<<','<<q.consistent_structure_fraction<<','<<q.information_ratio<<','<<q.reason<<','<<q.accepted<<','<<(pose.block<3,1>(0,3)-initial.block<3,1>(0,3)).norm()<<','<<rotation_distance(pose,initial)<<','<<q.roi_source_points<<','<<q.outside_map_points<<','<<q.unmodeled_points<<','<<q.roi_coverage<<','<<q.roi_source_fraction<<','<<q.unique_structure_voxels<<','<<q.structure_extent_m<<'\n';csv.flush();
      if(std::string(stage)=="refined"){
       // Export precisely the points selected for this GICP call at its
       // initial pose, transformed by the resulting pose for visual review.
       const auto used=engine.registration_source(source,initial);
       Cloud used_map;pcl::transformPointCloud(*used,used_map,pose);
       const auto used_path=output/("refined_"+std::to_string(pose_index++)+"_registration_map.pcd");
       pcl::io::savePCDFileBinary(used_path.string(),used_map);
       poses<<YAML::BeginMap<<YAML::Key<<"case"<<YAML::Value<<id<<YAML::Key<<"max_range_m"<<YAML::Value<<range<<YAML::Key<<"gravity_sign"<<YAML::Value<<sign<<YAML::Key<<"local_quality_reason"<<YAML::Value<<q.reason<<YAML::Key<<"global_localization_accepted"<<YAML::Value<<false<<YAML::Key<<"registration_source_points"<<YAML::Value<<used->size()<<YAML::Key<<"registration_source_map_pcd"<<YAML::Value<<used_path.string()<<YAML::Key<<"T_map_sensor_row_major"<<YAML::Value;emit(poses,pose);poses<<YAML::Key<<"initial_T_map_sensor_row_major"<<YAML::Value;emit(poses,initial);poses<<YAML::EndMap;
       std::cout<<id<<" range="<<range<<" sign="<<sign<<" "<<q.reason<<" coverage="<<q.coverage<<" structure="<<q.consistent_structure_fraction<<" xy="<<foot(0,3)<<","<<foot(1,3)<<std::endl;
      }
     };
     for(double z=y["sensor_z_min_m"].as<double>();z<=y["sensor_z_max_m"].as<double>()+1e-9;z+=y["sensor_z_step_m"].as<double>()){
      Matrix pose=Matrix::Identity();pose.block<3,3>(0,0)=rotation;
      const Eigen::Vector3d offset=rotation*sensor_foot.block<3,1>(0,3);
      pose.block<3,1>(0,3)=Eigen::Vector3d(y["approximate_footprint_xy_m"][0].as<double>()-offset.x(),y["approximate_footprint_xy_m"][1].as<double>()-offset.y(),z);
      auto q=engine.evaluate(source,pose,true);record("height_probe",pose,q,pose);sweep.push_back({pose,q});
     }
     std::stable_sort(sweep.begin(),sweep.end(),[](const auto& a,const auto& b){return a.second.capped_all_mse_m2<b.second.capped_all_mse_m2;});
     for(int k=0;k<std::min<int>(y["refine_per_gravity_sign"].as<int>(),sweep.size());++k){auto d=engine.refine(source,sweep[k].first);record("refined",d.T_map_sensor,d.quality,sweep[k].first);}
    }
   }
  }
 }
 poses<<YAML::EndSeq<<YAML::EndMap;std::ofstream(output/"diagnostic_poses.yaml")<<poses.c_str()<<'\n';
 }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
