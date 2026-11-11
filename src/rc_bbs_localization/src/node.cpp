#include "rc_bbs_localization/core.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_msgs/msg/tf_message.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <deque>
#include <future>
#include <optional>
#include <filesystem>
#include <fstream>
#include <cstring>
#include <algorithm>

namespace rc_bbs {
namespace {
Matrix from_pose(const geometry_msgs::msg::Pose& p) {
  Eigen::Quaterniond q(p.orientation.w,p.orientation.x,p.orientation.y,p.orientation.z);
  if(!q.coeffs().allFinite() || std::abs(q.norm()-1.)>0.01) throw std::invalid_argument("Invalid quaternion");
  Matrix out=Matrix::Identity();out.block<3,3>(0,0)=q.normalized().toRotationMatrix();
  out.block<3,1>(0,3)=Eigen::Vector3d(p.position.x,p.position.y,p.position.z);
  if(!rigid(out)) throw std::invalid_argument("Nonfinite pose");return out;
}
Matrix from_tf(const geometry_msgs::msg::Transform& t) {
  geometry_msgs::msg::Pose p;p.position.x=t.translation.x;p.position.y=t.translation.y;p.position.z=t.translation.z;
  p.orientation=t.rotation;return from_pose(p);
}
geometry_msgs::msg::Pose to_pose(const Matrix& t) {
  geometry_msgs::msg::Pose p;p.position.x=t(0,3);p.position.y=t(1,3);p.position.z=t(2,3);
  Eigen::Quaterniond q(t.block<3,3>(0,0));p.orientation.x=q.x();p.orientation.y=q.y();p.orientation.z=q.z();p.orientation.w=q.w();return p;
}
double seconds(Clock::time_point t) {return std::chrono::duration<double>(Clock::now()-t).count();}
}
class Node : public rclcpp::Node {
  enum class State {UNINITIALIZED,SEARCHING,VERIFYING,TRACKING,DEGRADED,LOST};
  static std::string name(State s) {
    switch(s) {
      case State::UNINITIALIZED:return "UNINITIALIZED";case State::SEARCHING:return "SEARCHING";
      case State::VERIFYING:return "VERIFYING";case State::TRACKING:return "TRACKING";
      case State::DEGRADED:return "DEGRADED";default:return "LOST";
    }
  }
  struct Odom {int64_t ns;Matrix T_odom_base;};
  struct Imu {int64_t ns;Eigen::Vector3d acc;double gyro;std::string frame;};
  struct Frame {
    sensor_msgs::msg::PointCloud2::ConstSharedPtr message;
    Matrix T_odom_sensor=Matrix::Identity(),T_base_sensor=Matrix::Identity(),gravity=Matrix::Identity();
    bool gravity_valid=false;
    std::vector<std::pair<sensor_msgs::msg::PointCloud2::ConstSharedPtr,Matrix>> scans;
  };
  enum class Kind {SEARCH,REFINE,MAP};
  struct Result {
    SearchResult search;Candidate candidate;Cloud::Ptr source;std::shared_ptr<Engine> engine;
    std::string error;
    std::optional<std::filesystem::file_time_type> map_mtime;
    double preprocess_ms=0.;
  };
  struct Job {
    Kind kind;Frame frame;uint64_t epoch;std::future<Result> future;Clock::time_point launched;
    Matrix guess=Matrix::Identity();
  };
public:
  Node():rclcpp::Node("bbs_localizer"),tf_buffer_(get_clock()),tf_listener_(tf_buffer_) {
    config_.visit([&](const char* key,auto& value){value=param(key,value);});config_.validate();
    map_file_=param<std::string>("map_file","");
    map_frame_=param<std::string>("map_frame","map");odom_frame_=param<std::string>("odom_frame","odom");
    base_frame_=param<std::string>("base_frame","base_link");sensor_frame_=param<std::string>("sensor_frame","imu_link");
    cloud_topic_=param<std::string>("cloud_topic","/cloud_registered_body");
    odom_topic_=param<std::string>("odom_topic","/Odometry");imu_topic_=param<std::string>("imu_topic","/livox/imu");
    if(map_frame_==odom_frame_ || sensor_frame_==map_frame_ || sensor_frame_==odom_frame_ || cloud_topic_=="/cloud_registered")
      throw std::invalid_argument("Source must be a sensor-local scan; world clouds are forbidden");
    if(!param<bool>("input_is_deskewed",true)) throw std::invalid_argument("Raw moving scans unsupported: enable FAST-LIO deskewed body output");
    if(param<std::string>("gravity_source","stationary_imu")!="stationary_imu")
      throw std::invalid_argument("Only stationary_imu gravity prior is implemented");
    odom_jump_m_=positive("odom_jump_translation_m",2.0);
    odom_jump_rad_=positive("odom_jump_rotation_rad",0.8);
    accumulation_sec_=positive("accumulation_sec",0.15);
    accumulation_max_frames_=param<int>("accumulation_max_frames",8);
    if(accumulation_max_frames_<1 || accumulation_max_frames_>40 || accumulation_sec_>0.5)
      throw std::invalid_argument("Accumulation must be <=0.5 sec and <=40 frames");
    max_age_=positive("max_data_age_sec",0.5);input_timeout_=positive("input_timeout_sec",0.8);
    sync_=positive("odom_sync_tolerance_sec",0.03);retry_=positive("retry_interval_sec",0.5);
    result_max_age_=positive("result_max_age_sec",2.5);verification_timeout_=positive("verification_timeout_sec",1.);
    verification_interval_=positive("verification_min_interval_sec",0.15);
    if(verification_interval_<accumulation_sec_)throw std::invalid_argument("Verification interval must cover accumulation window");
    required_=param<int>("verification_frames",3);fail_limit_=param<int>("tracking_failure_limit",3);
    consistency_m_=positive("consistency_translation_m",0.3);consistency_rad_=positive("consistency_rotation_rad",0.15);
    gravity_window_=positive("gravity_window_sec",0.20);gravity_min_samples_=param<int>("gravity_min_samples",20);
    gravity_acc_tolerance_=positive("gravity_accel_tolerance_mps2",0.6);
    gravity_std_=positive("gravity_max_std_mps2",0.25);gravity_gyro_=positive("gravity_max_gyro_radps",0.15);
    stationary_m_=positive("gravity_stationary_displacement_m",0.025);
    if(required_<2 || fail_limit_<1 || gravity_min_samples_<5) throw std::invalid_argument("Invalid verification/failure/IMU sample counts");
    const auto log_path=param<std::string>("metrics_file","/tmp/rc_bbs_localization_metrics.csv");
    metrics_.open(log_path,std::ios::out);
    if(!metrics_) throw std::runtime_error("Cannot write metrics_file: "+log_path);
    metrics_<<"stamp_ns,epoch,state,reason,job_ms,search_ms,preprocess_ms,source_points,correspondences,coverage,inlier_mse_m2,inlier_rmse_m,inlier_p95_m,capped_all_mse_m2,nonground_fraction,information_ratio,bbs_hit_fraction,verification_count,time_to_tracking_ms,quality_evaluated,selected_map_gravity_z_sign,nonground_correspondences,consistent_structure_correspondences,consistent_structure_fraction,roi_source_points,outside_map_points,unmodeled_points,roi_coverage,roi_source_fraction,unique_structure_voxels,structure_extent_m\n";
    auto latched=rclcpp::QoS(1).reliable().transient_local();
    map_pub_=create_publisher<sensor_msgs::msg::PointCloud2>("/bbs_localization/map",latched);
    aligned_pub_=create_publisher<sensor_msgs::msg::PointCloud2>("/bbs_localization/aligned_scan",1);
    pose_pub_=create_publisher<geometry_msgs::msg::PoseStamped>("/bbs_localization/pose",5);
    sensor_pose_pub_=create_publisher<geometry_msgs::msg::PoseStamped>("/bbs_localization/sensor_pose",5);
    status_pub_=create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/bbs_localization/status",5);
    marker_pub_=create_publisher<visualization_msgs::msg::Marker>("/bbs_localization/status_text",1);
    tf_pub_=create_publisher<tf2_msgs::msg::TFMessage>("/tf",100);
    auto observe=[this](tf2_msgs::msg::TFMessage::ConstSharedPtr message,const rclcpp::MessageInfo& info) {
      const auto& own=tf_pub_->get_gid();const auto& other=info.get_rmw_message_info().publisher_gid;
      if(std::memcmp(own.data,other.data,RMW_GID_STORAGE_SIZE)==0) return;
      for(const auto& t:message->transforms) if(t.header.frame_id==map_frame_ && t.child_frame_id==odom_frame_) {
        conflict_=true;invalidate("competing_map_odom_publisher");break;
      }
    };
    tf_sub_=create_subscription<tf2_msgs::msg::TFMessage>("/tf",rclcpp::QoS(100).best_effort(),observe);
    static_tf_sub_=create_subscription<tf2_msgs::msg::TFMessage>("/tf_static",rclcpp::QoS(100).transient_local(),observe);
    odom_sub_=create_subscription<nav_msgs::msg::Odometry>(odom_topic_,rclcpp::SensorDataQoS().keep_last(300),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr m){
        if(m->header.frame_id!=odom_frame_ || m->child_frame_id!=base_frame_) {reason_="odom_frame_mismatch";return;}
        try {
          auto ns=rclcpp::Time(m->header.stamp).nanoseconds();
          if(!odom_.empty() && ns<odom_.back().ns) {invalidate("odom_time_reset");odom_.clear();}
          if(!odom_.empty() && ns==odom_.back().ns) return;
          auto pose=from_pose(m->pose.pose);
          if(!odom_.empty() && ((pose.block<3,1>(0,3)-odom_.back().T_odom_base.block<3,1>(0,3)).norm()>odom_jump_m_ ||
              rotation_distance(pose,odom_.back().T_odom_base)>odom_jump_rad_))
            invalidate("odom_discontinuity");
          odom_.push_back({ns,pose});while(odom_.size()>1500) odom_.pop_front();
        }catch(const std::exception& e){reason_=e.what();}
      });
    imu_sub_=create_subscription<sensor_msgs::msg::Imu>(imu_topic_,rclcpp::SensorDataQoS().keep_last(500),
      [this](sensor_msgs::msg::Imu::ConstSharedPtr m){
        Eigen::Vector3d acc(m->linear_acceleration.x,m->linear_acceleration.y,m->linear_acceleration.z);
        Eigen::Vector3d gyro(m->angular_velocity.x,m->angular_velocity.y,m->angular_velocity.z);
        if(!acc.allFinite() || !gyro.allFinite())return;
        auto ns=rclcpp::Time(m->header.stamp).nanoseconds();
        if(!imu_.empty() && ns<imu_.back().ns) imu_.clear();
        imu_.push_back({ns,acc,gyro.norm(),m->header.frame_id});while(imu_.size()>2000) imu_.pop_front();
      });
    cloud_sub_=create_subscription<sensor_msgs::msg::PointCloud2>(cloud_topic_,rclcpp::SensorDataQoS().keep_last(1),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr m){
        if(m->header.frame_id!=sensor_frame_) {invalidate("scan_not_in_configured_sensor_frame");return;}
        for(const std::string field:{"x","y","z"}) {
          if(std::none_of(m->fields.begin(),m->fields.end(),[&](const auto& f){return f.name==field && f.datatype==7 && f.count==1;})) {
            invalidate("scan_missing_float32_xyz_fields");return;
          }
        }
        latest_=m;last_input_=Clock::now();
        scan_history_.push_back(m);
        while(scan_history_.size()>40)scan_history_.pop_front();
      });
    initial_sub_=create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose",10,
      [this](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr m){
        try {
          if(m->header.frame_id!=map_frame_) throw std::invalid_argument("initialpose must be in map frame");
          auto ns=rclcpp::Time(m->header.stamp).nanoseconds();
          double age=(now().nanoseconds()-ns)*1e-9;
          if(age< -0.05 || age>max_age_) throw std::invalid_argument("initialpose timestamp stale/future; click again");
          auto pose=from_pose(m->pose.pose);
          invalidate("rviz_initialpose_received");pending_seed_=pose;seed_stamp_=ns;
        }catch(const std::exception& e){RCLCPP_ERROR(get_logger(),"Reject initialpose: %s",e.what());}
      });
    reload_srv_=create_service<std_srvs::srv::Trigger>("/bbs_localization/reload_map",
      [this](const std_srvs::srv::Trigger::Request::SharedPtr,std_srvs::srv::Trigger::Response::SharedPtr r){
        invalidate("map_reload_requested");reload_=true;r->success=true;r->message="Map reload queued; current pose invalidated";
      });
    search_srv_=create_service<std_srvs::srv::Trigger>("/bbs_localization/relocalize",
      [this](const std_srvs::srv::Trigger::Request::SharedPtr,std_srvs::srv::Trigger::Response::SharedPtr r){
        invalidate("global_search_requested");r->success=!conflict_;r->message=conflict_?"TF conflict: restart after removing competing publisher":"Unseeded global search queued";
      });
    // All configuration is read-only: YAML edits take effect on restart. TF changes are observed live.
    timer_=create_wall_timer(std::chrono::milliseconds(10),[this]{tick();});
    status_timer_=create_wall_timer(std::chrono::milliseconds(200),[this]{status();});
    map_timer_=create_wall_timer(std::chrono::milliseconds(250),[this]{check_map();});
    start_map();
    std::string signs;
    for(double sign:config_.map_gravity_z_signs)signs+=(sign>0 ? "+Z " : "-Z ");
    RCLCPP_INFO(get_logger(),"Map gravity hypotheses: %s (all share the search deadline)",signs.c_str());
    RCLCPP_INFO(get_logger(),"Backend=official_cpu+distinct_leaf_extension; local=PCL_GICP; p_map=T_map_sensor*p_sensor; gravity=stationary IMU, combined IMU/map uncertainty %.3f rad",config_.gravity_uncertainty_rad);
  }
  ~Node() override {if(job_)job_->future.wait();}
private:
  template<class T>T param(const std::string& key,T value) {
    rcl_interfaces::msg::ParameterDescriptor d;d.read_only=true;
    return declare_parameter<T>(key,value,d);
  }
  double positive(const std::string& key,double value) {
    double v=param(key,value);if(!std::isfinite(v)||v<=0)throw std::invalid_argument(key+" must be positive");return v;
  }
  void invalidate(const std::string& why) {
    ++epoch_;state_=State::LOST;reason_=why;good_=0;failures_=0;pending_seed_.reset();
    have_anchor_=false;selected_map_gravity_z_sign_=NAN;seed_refining_=false;scan_history_.clear();last_attempt_=Clock::time_point::min();acquiring_=Clock::now();time_to_tracking_ms_=NAN;
  }
  std::optional<Matrix> odom_at(int64_t ns) const {
    if(odom_.empty())return {};
    auto it=std::lower_bound(odom_.begin(),odom_.end(),ns,[](const Odom& a,int64_t t){return a.ns<t;});
    if(it!=odom_.end() && it->ns==ns)return it->T_odom_base;
    if(it==odom_.begin() || it==odom_.end())return {};
    const auto& a=*(it-1);const auto& b=*it;
    if((ns-a.ns)*1e-9>sync_ || (b.ns-ns)*1e-9>sync_)return {};
    double alpha=double(ns-a.ns)/(b.ns-a.ns);Matrix out=Matrix::Identity();
    out.block<3,1>(0,3)=(1-alpha)*a.T_odom_base.block<3,1>(0,3)+alpha*b.T_odom_base.block<3,1>(0,3);
    Eigen::Quaterniond qa(a.T_odom_base.block<3,3>(0,0)),qb(b.T_odom_base.block<3,3>(0,0));
    out.block<3,3>(0,0)=qa.slerp(alpha,qb).toRotationMatrix();return out;
  }
  bool gravity_at(int64_t ns,Matrix& rotation) {
    Eigen::Vector3d mean=Eigen::Vector3d::Zero(),square=Eigen::Vector3d::Zero();int count=0;
    int64_t first=0,last=0;std::string frame;
    for(const auto& s:imu_) if(s.ns<=ns && (ns-s.ns)*1e-9<=gravity_window_) {
      if(s.gyro>gravity_gyro_)return false;
      if(count==0){first=s.ns;frame=s.frame;}if(s.frame!=frame)return false;
      last=s.ns;mean+=s.acc;square+=s.acc.cwiseProduct(s.acc);++count;
    }
    if(count<gravity_min_samples_ || (last-first)*1e-9<gravity_window_*0.8 || (ns-last)*1e-9>sync_)return false;
    mean/=count;
    if(std::abs(mean.norm()-9.81)>gravity_acc_tolerance_ ||
       (square/count-mean.cwiseProduct(mean)).cwiseMax(0.).cwiseSqrt().maxCoeff()>gravity_std_)return false;
    auto a=odom_at(first),b=odom_at(ns);
    if(!a||!b || (a->block<3,1>(0,3)-b->block<3,1>(0,3)).norm()>stationary_m_)return false;
    try {
      auto t=tf_buffer_.lookupTransform(sensor_frame_,frame,tf2::TimePointZero);
      mean=from_tf(t.transform).block<3,3>(0,0)*mean;
    }catch(const std::exception&){return false;}
    rotation=Matrix::Identity();
    rotation.block<3,3>(0,0)=Eigen::Quaterniond::FromTwoVectors(mean,Eigen::Vector3d::UnitZ()).toRotationMatrix();
    return rigid(rotation);
  }
  std::optional<Frame> frame() {
    if(!latest_)return {};
    int64_t ns=rclcpp::Time(latest_->header.stamp).nanoseconds();
    double age=(now().nanoseconds()-ns)*1e-9;
    if(ns<=0 || age< -0.05 || age>max_age_){reason_="scan_timestamp_stale_or_future";return {};}
    auto odom=odom_at(ns);if(!odom){reason_="waiting_for_time_bracketed_odometry";return {};}
    try {
      const auto t=tf_buffer_.lookupTransform(base_frame_,sensor_frame_,tf2::TimePointZero);
      const Matrix extrinsic=from_tf(t.transform);
      if(have_extrinsic_ && (extrinsic-last_extrinsic_).norm()>1e-7)invalidate("extrinsic_changed");
      have_extrinsic_=true;last_extrinsic_=extrinsic;
      Frame f;f.message=latest_;f.T_base_sensor=extrinsic;f.T_odom_sensor=*odom*extrinsic;
      f.gravity_valid=gravity_at(ns,f.gravity);
      for(auto it=scan_history_.rbegin();it!=scan_history_.rend();++it) {
        int64_t old_ns=rclcpp::Time((*it)->header.stamp).nanoseconds();
        if(old_ns>ns || (ns-old_ns)*1e-9>accumulation_sec_)continue;
        auto old_odom=odom_at(old_ns);if(!old_odom)continue;
        f.scans.emplace_back(*it,(*old_odom)*extrinsic);
        if(f.scans.size()>=static_cast<size_t>(accumulation_max_frames_))break;
      }
      if(f.scans.empty())f.scans.emplace_back(latest_,f.T_odom_sensor);
      return f;
    }catch(const std::exception& e){reason_=std::string("missing_sensor_extrinsic: ")+e.what();return {};}
  }
  void check_map() {
    try {
      auto version=std::filesystem::last_write_time(map_file_);
      if(map_mtime_ && version!=*map_mtime_) {invalidate("map_file_changed");reload_=true;}
      map_mtime_=version;
    }catch(const std::exception& e){invalidate(std::string("map_unavailable: ")+e.what());reload_=true;}
    // Look up TF even during a job; changed mounting invalidates that snapshot immediately.
    if(have_extrinsic_)try {
      auto t=tf_buffer_.lookupTransform(base_frame_,sensor_frame_,tf2::TimePointZero);
      if((from_tf(t.transform)-last_extrinsic_).norm()>1e-7) {
        invalidate("extrinsic_changed");have_extrinsic_=false;
      }
    }catch(const std::exception&){invalidate("sensor_extrinsic_unavailable");}
  }
  void start_map() {
    if(job_)return;
    Job job;job.kind=Kind::MAP;job.epoch=epoch_;job.launched=Clock::now();
    auto c=config_;auto file=map_file_;auto current=engine_;
    job.future=std::async(std::launch::async,[c,file,current]{
      Result r;
      try {
        auto before=std::filesystem::last_write_time(file);
        r.engine=load_cached_engine(c,file);
        r.map_mtime=std::filesystem::last_write_time(file);
        if(before!=*r.map_mtime)throw std::runtime_error("Map changed during cache load");
      }catch(const std::exception& e){r.error=e.what();}
      return r;
    });
    reload_=false;reason_="building_map_cache";job_=std::move(job);
  }
  void launch_job(Kind kind,const Frame& f,const Matrix& guess) {
    Job job;job.kind=kind;job.frame=f;job.epoch=epoch_;job.launched=Clock::now();job.guess=guess;
    auto engine=engine_;auto c=config_;
    job.future=std::async(std::launch::async,[engine,c,kind,f,guess]{
      Result r;const auto start=Clock::now();
      try {
        auto raw=std::make_shared<Cloud>();
        for(const auto& scan:f.scans) {
          Cloud input,transformed;pcl::fromROSMsg(*scan.first,input);
          // Each cloud was already deskewed by FAST-LIO. Accumulated scans are
          // motion compensated to the NEWEST sensor frame, never to map/odom.
          Matrix relative=f.T_odom_sensor.inverse()*scan.second;
          pcl::transformPointCloud(input,transformed,relative);
          *raw+=transformed;
        }
        r.source=clean_cloud(raw,c,true);
        r.preprocess_ms=std::chrono::duration<double,std::milli>(Clock::now()-start).count();
        if(kind==Kind::SEARCH)r.search=engine->search(r.source,f.gravity);
        else r.candidate=engine->refine(r.source,guess);
      }catch(const std::exception& e){r.error=e.what();}
      return r;
    });
    processed_ns_=rclcpp::Time(f.message->header.stamp).nanoseconds();
    if(kind==Kind::SEARCH){state_=State::SEARCHING;reason_="global_search_running";last_attempt_=Clock::now();}
    job_=std::move(job);
    // A fast search can finish between periodic diagnostics ticks. Publish this
    // transition immediately so reset/epoch behavior is observable in ROS/RViz.
    if(kind==Kind::SEARCH)status();
  }
  void finish() {
    if(!job_ || job_->future.wait_for(std::chrono::seconds(0))!=std::future_status::ready)return;
    auto job=std::move(*job_);job_.reset();Result result=job.future.get();
    if(job.epoch!=epoch_ || conflict_) {
      // Keep a stale map alive only as a cache owner, then validate/reload it in
      // a new epoch. A reset during cold map construction must not strand the node.
      if(job.kind==Kind::MAP && !conflict_) {
        if(result.engine)engine_=result.engine;
        reload_=true;
      }
      return;
    }
    job_ms_=seconds(job.launched)*1000.;preprocess_ms_=result.preprocess_ms;
    if(!result.error.empty()){if(job.kind==Kind::MAP)engine_.reset();state_=State::LOST;reason_=result.error;last_attempt_=Clock::now();return;}
    if(job.kind==Kind::MAP) {
      std::error_code map_error;
      const auto current_mtime=std::filesystem::last_write_time(map_file_,map_error);
      if(map_error || !result.map_mtime || current_mtime!=*result.map_mtime) {
        engine_=result.engine;invalidate("map_changed_before_cache_commit");reload_=true;return;
      }
      engine_=result.engine;
      publish_cloud(engine_->map(),map_pub_,now());
      try{map_mtime_=std::filesystem::last_write_time(map_file_);}catch(const std::exception&){}
      state_=State::UNINITIALIZED;reason_="map_ready_waiting_for_sensor";
      RCLCPP_INFO(get_logger(),"Map cache %s: %zu points; cold construction %.1f ms",engine_->version().c_str(),engine_->map()->size(),engine_->build_ms());return;
    }
    // Poll again at commit, closing the interval between timer observations.
    // TF's listener has its own thread; a new extrinsic can arrive during GICP.
    check_map();
    if(job.epoch!=epoch_ || reload_ || conflict_)return;
    const int64_t ns=rclcpp::Time(job.frame.message->header.stamp).nanoseconds();
    const double result_age=(now().nanoseconds()-ns)*1e-9;
    const double permitted_age=job.kind==Kind::REFINE ? max_age_ : result_max_age_;
    if(result_age< -0.05 || result_age>permitted_age || seconds(job.launched)>permitted_age) {
      invalidate("search_result_expired");return;
    }
    if(job.kind==Kind::SEARCH) {
      search_ms_=result.search.elapsed_ms;
      search_complete_=result.search.complete;search_ambiguous_=result.search.ambiguous;
      last_candidate_=Candidate();
      if(result.source)last_candidate_.quality.source_points=result.source->size();
      reason_=result.search.reason;
      if(!result.search.candidates.empty())last_candidate_=result.search.candidates[0];
      if(!result.search.accepted || !result.search.complete || job_ms_>config_.search_budget_ms) {
        state_=State::LOST;good_=0;last_attempt_=Clock::now();
        if(job_ms_>config_.search_budget_ms)reason_="preprocess_and_search_budget_exceeded";
        record(ns);return;
      }
      last_candidate_=result.search.candidates.front();
      selected_map_gravity_z_sign_=last_candidate_.map_gravity_z_sign;
      anchor_=last_candidate_.T_map_sensor*job.frame.T_odom_sensor.inverse();
      have_anchor_=true;state_=State::VERIFYING;good_=0;verification_start_=Clock::now();
      verification_after_ns_=now().nanoseconds()+static_cast<int64_t>(verification_interval_*1e9);
      record(ns);return;
    }
    last_candidate_=result.candidate;
    const bool consistent=(result.candidate.T_map_sensor.block<3,1>(0,3)-job.guess.block<3,1>(0,3)).norm()<=consistency_m_ &&
      rotation_distance(result.candidate.T_map_sensor,job.guess)<=consistency_rad_;
    const bool initializing=state_!=State::TRACKING && state_!=State::DEGRADED;
    // First user seed refinement may make a larger correction; subsequent verification is bounded.
    const bool seed_first=seed_refining_;
    seed_refining_=false;
    if(!result.candidate.quality.accepted || (!consistent && !seed_first)) {
      good_=0;reason_=result.candidate.quality.accepted?"motion_compensated_inconsistency":result.candidate.quality.reason;
      if(initializing || ++failures_>=fail_limit_) {invalidate(reason_);last_attempt_=Clock::time_point::min();}
      else state_=State::DEGRADED;
      record(ns);return;
    }
    anchor_=result.candidate.T_map_sensor*job.frame.T_odom_sensor.inverse();have_anchor_=true;failures_=0;
    if(initializing) {
      if(seed_first){good_=0;state_=State::VERIFYING;verification_start_=Clock::now();verification_after_ns_=now().nanoseconds()+static_cast<int64_t>(verification_interval_*1e9);}
      else {++good_;verification_after_ns_=ns+static_cast<int64_t>(verification_interval_*1e9);}
      if(good_>=required_) {
        state_=State::TRACKING;++epoch_; // invalidates any older generation and resets prediction anchor.
        time_to_tracking_ms_=seconds(acquiring_)*1000.;reason_="multiframe_verified";
      }else reason_="awaiting_fresh_consistent_frames";
    }else {state_=State::TRACKING;reason_="tracking_quality_pass";}
    if(state_==State::TRACKING) {
      last_verified_=Clock::now();last_verified_ns_=ns;publish_pose(result.candidate.T_map_sensor,job.frame);
      auto aligned=std::make_shared<Cloud>();pcl::transformPointCloud(*result.source,*aligned,result.candidate.T_map_sensor);
      publish_cloud(aligned,aligned_pub_,rclcpp::Time(job.frame.message->header.stamp));
    }
    record(ns);
  }
  void tick() {
    finish();
    if(conflict_)return;
    if(reload_ && !job_){start_map();return;}
    if((state_==State::TRACKING || state_==State::DEGRADED || state_==State::VERIFYING) &&
       (seconds(last_input_)>input_timeout_ || (latest_ && (now()-rclcpp::Time(latest_->header.stamp)).seconds()>input_timeout_)))
      invalidate("input_timeout");
    if(state_==State::VERIFYING && seconds(verification_start_)>verification_timeout_)invalidate("verification_timeout");
    // Verify new TF/motion snapshots even while background work runs.
    auto f=frame();
    if(f && acquiring_==Clock::time_point{})acquiring_=Clock::now();
    if(job_ || !engine_ || !f)return;
    int64_t ns=rclcpp::Time(f->message->header.stamp).nanoseconds();
    if(ns<=processed_ns_)return;
    if(acquiring_==Clock::time_point{})acquiring_=Clock::now();
    if(pending_seed_) {
      auto old=odom_at(seed_stamp_);
      if(!old){reason_="waiting_for_initialpose_odometry_bracket";return;}
      Matrix guess=(*pending_seed_)*old->inverse()*f->T_odom_sensor;
      pending_seed_.reset();state_=State::VERIFYING;seed_refining_=true;verification_start_=Clock::now();
      launch_job(Kind::REFINE,*f,guess);return;
    }
    if(have_anchor_ && (state_==State::TRACKING || state_==State::DEGRADED || state_==State::VERIFYING)) {
      if(state_==State::VERIFYING && ns<=verification_after_ns_)return;
      launch_job(Kind::REFINE,*f,anchor_*f->T_odom_sensor);return;
    }
    if(last_attempt_!=Clock::time_point::min() && seconds(last_attempt_)<retry_)return;
    if(!f->gravity_valid){reason_="gravity_prior_untrusted_need_stationary_imu_and_odometry";return;}
    launch_job(Kind::SEARCH,*f,Matrix::Identity());
  }
  void publish_cloud(const Cloud::ConstPtr& cloud,const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& pub,rclcpp::Time stamp) {
    sensor_msgs::msg::PointCloud2 m;pcl::toROSMsg(*cloud,m);m.header.frame_id=map_frame_;m.header.stamp=stamp;pub->publish(m);
  }
  void publish_pose(const Matrix& T_map_sensor,const Frame& f) {
    geometry_msgs::msg::PoseStamped sensor;sensor.header.frame_id=map_frame_;sensor.header.stamp=f.message->header.stamp;
    sensor.pose=to_pose(T_map_sensor);sensor_pose_pub_->publish(sensor);
    auto base=sensor;base.pose=to_pose(T_map_sensor*f.T_base_sensor.inverse());pose_pub_->publish(base);
    geometry_msgs::msg::TransformStamped t;t.header=sensor.header;t.child_frame_id=odom_frame_;
    const auto p=to_pose(anchor_);t.transform.translation.x=p.position.x;t.transform.translation.y=p.position.y;t.transform.translation.z=p.position.z;t.transform.rotation=p.orientation;
    tf2_msgs::msg::TFMessage out;out.transforms.push_back(t);tf_pub_->publish(out);
  }
  void record(int64_t ns) {
    const auto& q=last_candidate_.quality;
    metrics_<<ns<<','<<epoch_<<','<<name(state_)<<','<<reason_<<','<<job_ms_<<','<<search_ms_<<','<<preprocess_ms_<<','
      <<q.source_points<<','<<q.correspondences<<','<<q.coverage<<','<<q.mse_m2<<','<<q.rmse_m<<','<<q.p95_m<<','
      <<q.capped_all_mse_m2<<','<<q.nonground_fraction<<','<<q.information_ratio<<','<<last_candidate_.bbs_hit_fraction<<','<<good_<<','<<time_to_tracking_ms_<<','<<q.evaluated<<','<<selected_map_gravity_z_sign_<<','<<q.nonground_correspondences<<','<<q.consistent_structure_correspondences<<','<<q.consistent_structure_fraction<<','<<q.roi_source_points<<','<<q.outside_map_points<<','<<q.unmodeled_points<<','<<q.roi_coverage<<','<<q.roi_source_fraction<<','<<q.unique_structure_voxels<<','<<q.structure_extent_m<<'\n';metrics_.flush();
  }
  void status() {
    diagnostic_msgs::msg::DiagnosticArray array;array.header.stamp=now();
    diagnostic_msgs::msg::DiagnosticStatus s;s.name="bbs_localization";s.hardware_id="official_cpu";
    double verified_age=last_verified_ns_>0 ? (now().nanoseconds()-last_verified_ns_)*1e-9 : INFINITY;
    bool valid=state_==State::TRACKING && verified_age>=-0.05 && verified_age<=max_age_ && seconds(last_verified_)<=max_age_ && !conflict_;
    s.level=valid?0:(state_==State::LOST || conflict_?2:1);s.message=name(state_)+": "+reason_;
    auto add=[&](const std::string& key,const auto& v){diagnostic_msgs::msg::KeyValue kv;kv.key=key;std::ostringstream ss;ss<<v;kv.value=ss.str();s.values.push_back(kv);};
    std::string gravity_signs;for(double sign:config_.map_gravity_z_signs)gravity_signs+=(sign>0?"+Z ":"-Z ");
    add("selected_map_gravity_z_sign",selected_map_gravity_z_sign_);add("map_gravity_hypotheses",gravity_signs);add("state",name(state_));add("pose_valid",valid);add("reason",reason_);add("backend","official_cpu+distinct_leaf_extension");
    add("map_version",engine_?engine_->version():"loading");add("epoch",epoch_);add("active_workers",job_?1:0);
    add("verification_count",good_);add("source_points",last_candidate_.quality.source_points);
    add("correspondences",last_candidate_.quality.correspondences);add("coverage",last_candidate_.quality.coverage);
    add("inlier_mse_m2",last_candidate_.quality.mse_m2);add("inlier_rmse_m",last_candidate_.quality.rmse_m);
    add("inlier_p95_m",last_candidate_.quality.p95_m);add("capped_all_mse_m2",last_candidate_.quality.capped_all_mse_m2);
    add("search_complete",search_complete_);add("candidate_ambiguity",search_ambiguous_);add("quality_evaluated",last_candidate_.quality.evaluated);add("gicp_converged",last_candidate_.quality.converged);add("nonground_correspondences",last_candidate_.quality.nonground_correspondences);add("nonground_fraction",last_candidate_.quality.nonground_fraction);
    add("consistent_structure_correspondences",last_candidate_.quality.consistent_structure_correspondences);
    add("consistent_structure_fraction",last_candidate_.quality.consistent_structure_fraction);
    add("unmapped_obstacle_filter",config_.unmapped_obstacle_filter);
    add("roi_source_points",last_candidate_.quality.roi_source_points);add("outside_map_points",last_candidate_.quality.outside_map_points);
    add("unmodeled_points",last_candidate_.quality.unmodeled_points);add("roi_coverage",last_candidate_.quality.roi_coverage);
    add("roi_source_fraction",last_candidate_.quality.roi_source_fraction);add("unique_structure_voxels",last_candidate_.quality.unique_structure_voxels);
    add("structure_extent_m",last_candidate_.quality.structure_extent_m);
    add("information_ratio",last_candidate_.quality.information_ratio);add("bbs_hit_fraction",last_candidate_.bbs_hit_fraction);
    add("accumulation_sec",accumulation_sec_);add("job_ms",job_ms_);add("search_ms",search_ms_);add("time_to_tracking_ms",time_to_tracking_ms_);
    add("last_verified_age_sec",verified_age);add("last_verified_receipt_age_sec",seconds(last_verified_));array.status.push_back(s);status_pub_->publish(array);
    visualization_msgs::msg::Marker marker;marker.header=array.header;marker.header.frame_id=map_frame_;
    marker.ns="bbs_state";marker.id=0;marker.type=visualization_msgs::msg::Marker::TEXT_VIEW_FACING;marker.action=0;
    marker.pose.orientation.w=1.;marker.pose.position.z=2.;marker.scale.z=0.4;marker.color.a=1.;
    marker.color.g=valid?1.:0.2;marker.color.r=valid?0.2:1.;marker.text=s.message;marker_pub_->publish(marker);
  }
  Config config_;State state_=State::UNINITIALIZED;std::string reason_="starting";
  std::string map_file_,map_frame_,odom_frame_,base_frame_,sensor_frame_,cloud_topic_,odom_topic_,imu_topic_;
  double odom_jump_m_,odom_jump_rad_,verification_interval_;
  double accumulation_sec_;int accumulation_max_frames_;
  double max_age_,input_timeout_,sync_,retry_,result_max_age_,verification_timeout_,consistency_m_,consistency_rad_;
  double gravity_window_,gravity_acc_tolerance_,gravity_std_,gravity_gyro_,stationary_m_;
  int required_,fail_limit_,gravity_min_samples_,good_=0,failures_=0;uint64_t epoch_=0;
  bool search_complete_=false,search_ambiguous_=false;
  bool conflict_=false,reload_=false,have_anchor_=false,have_extrinsic_=false,seed_refining_=false;
  Matrix anchor_=Matrix::Identity(),last_extrinsic_=Matrix::Identity();
  std::optional<Matrix> pending_seed_;int64_t seed_stamp_=0,processed_ns_=0,verification_after_ns_=0,last_verified_ns_=0;
  std::deque<Odom> odom_;std::deque<Imu> imu_;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr latest_;
  std::deque<sensor_msgs::msg::PointCloud2::ConstSharedPtr> scan_history_;
  std::shared_ptr<Engine> engine_;std::optional<Job> job_;
  std::optional<std::filesystem::file_time_type> map_mtime_;
  Clock::time_point last_attempt_=Clock::time_point::min(),last_input_=Clock::now(),verification_start_=Clock::now();
  Clock::time_point acquiring_{},last_verified_=Clock::now();
  Candidate last_candidate_;double selected_map_gravity_z_sign_=NAN;double job_ms_=0.,search_ms_=0.,preprocess_ms_=0.,time_to_tracking_ms_=NAN;
  std::ofstream metrics_;
  tf2_ros::Buffer tf_buffer_;tf2_ros::TransformListener tf_listener_;
  rclcpp::TimerBase::SharedPtr timer_,status_timer_,map_timer_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_,aligned_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_,sensor_pose_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr status_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_sub_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_,static_tf_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reload_srv_,search_srv_;
};
}
int main(int argc,char** argv) {
  rclcpp::init(argc,argv);
  try {rclcpp::spin(std::make_shared<rc_bbs::Node>());}
  catch(const std::exception& e){std::cerr<<"BBS localization fatal: "<<e.what()<<'\n';rclcpp::shutdown();return 1;}
  rclcpp::shutdown();return 0;
}
