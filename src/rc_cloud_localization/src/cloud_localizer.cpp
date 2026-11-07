#include "rc_cloud_localization/registration.hpp"
#include "rc_cloud_localization/output_gate.hpp"

#include <rclcpp/rclcpp.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <pcl/common/point_tests.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <deque>
#include <future>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <stdexcept>

namespace rc_cloud_localization
{
using Steady = std::chrono::steady_clock;
using Pose = geometry_msgs::msg::Pose;

static Eigen::Matrix4f pose_matrix(const Pose & pose)
{
  Eigen::Quaternionf q(pose.orientation.w, pose.orientation.x,
    pose.orientation.y, pose.orientation.z);
  const Eigen::Vector3f t(pose.position.x, pose.position.y, pose.position.z);
  if (!q.coeffs().allFinite() || !t.allFinite() || std::abs(q.norm() - 1.0f) > 0.01f) {
    throw std::invalid_argument("pose must have finite xyz and a unit quaternion");
  }
  Eigen::Matrix4f matrix = Eigen::Matrix4f::Identity();
  matrix.block<3, 3>(0, 0) = q.normalized().toRotationMatrix();
  matrix.block<3, 1>(0, 3) = t;
  return matrix;
}

static Pose matrix_pose(const Eigen::Matrix4f & matrix)
{
  Pose pose;
  pose.position.x = matrix(0, 3);
  pose.position.y = matrix(1, 3);
  pose.position.z = matrix(2, 3);
  const Eigen::Quaternionf q(matrix.block<3, 3>(0, 0));
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
  pose.orientation.w = q.w();
  return pose;
}

class CloudLocalizer : public rclcpp::Node
{
public:
  CloudLocalizer() : Node("cloud_localizer"), boot_(Steady::now())
  {
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    if (map_frame_.empty() || odom_frame_.empty() || base_frame_.empty() ||
      map_frame_ == odom_frame_ || map_frame_ == base_frame_ || odom_frame_ == base_frame_)
    {
      throw std::invalid_argument("map/odom/base frames must be nonempty and distinct");
    }
    options_.backend = declare_parameter<std::string>("relocalization_backend", "icp");
    if (options_.backend != "icp" && options_.backend != "ndt") {
      throw std::invalid_argument("relocalization_backend must be icp or ndt (seeded local search)");
    }
    leaf_ = positive("voxel_leaf_size", 0.15);
    options_.correspondence_distance = positive("correspondence_distance", 0.8);
    options_.ndt_resolution = positive("ndt_resolution", 1.0);
    options_.ndt_step_size = positive("ndt_step_size", 0.1);
    options_.iterations = declare_parameter<int>("max_iterations", 40);
    min_points_ = declare_parameter<int>("min_points", 100);
    required_good_ = declare_parameter<int>("required_consecutive_good", 3);
    if (options_.iterations < 1 || min_points_ < 10 || required_good_ < 1) {
      throw std::invalid_argument("iterations, min_points or required_consecutive_good invalid");
    }
    max_rmse_ = positive("max_rmse", 0.25);
    min_inliers_ = positive("min_inlier_ratio", 0.65);
    if (min_inliers_ > 1.0) {
      throw std::invalid_argument("min_inlier_ratio must be <= 1");
    }
    max_age_ = positive("max_data_age_sec", 0.6);
    timeout_ = positive("input_timeout_sec", 1.0);
    sync_tolerance_ = positive("odom_sync_tolerance_sec", 0.03);
    guard_sec_ = positive("tf_conflict_check_sec", 2.0);
    max_translation_ = positive("max_correction_distance", 1.5);
    max_rotation_ = positive("max_correction_angle", 0.8);
    scan_min_range_ = positive("scan_min_range", 0.5);
    scan_max_range_ = positive("scan_max_range", 30.0);
    if (scan_max_range_ <= scan_min_range_) {
      throw std::invalid_argument("scan_max_range must exceed scan_min_range");
    }
    const double x = finite_parameter("initial_pose_x");
    const double y = finite_parameter("initial_pose_y");
    const double z = finite_parameter("initial_pose_z");
    const double roll = finite_parameter("initial_pose_roll");
    const double pitch = finite_parameter("initial_pose_pitch");
    const double yaw = finite_parameter("initial_pose_yaw");
    seed_base_.block<3, 3>(0, 0) = (
      Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()) *
      Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY()) *
      Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX())).toRotationMatrix();
    seed_base_.block<3, 1>(0, 3) = Eigen::Vector3f(x, y, z);

    const auto latched = rclcpp::QoS(1).reliable().transient_local();
    map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_localization/map", latched);
    scan_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_localization/aligned_scan", 1);
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/cloud_localization/pose", 5);
    status_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/cloud_localization/status", 5);
    marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("/cloud_localization/status_text", 1);
    // Own /tf publisher exposes its DDS GID so our TF observer never mistakes our
    // own map->odom for a competing localizer, even if node names are duplicated.
    tf_pub_ = create_publisher<tf2_msgs::msg::TFMessage>("/tf", rclcpp::QoS(100));
    tf_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions tf_sub_options;
    tf_sub_options.callback_group = tf_group_;
    auto observe_tf = [this](const tf2_msgs::msg::TFMessage::ConstSharedPtr message,
        const rclcpp::MessageInfo & info) {check_tf(*message, info);};
    tf_sub_ = create_subscription<tf2_msgs::msg::TFMessage>(
      "/tf", rclcpp::QoS(100).best_effort(), observe_tf, tf_sub_options);
    static_tf_sub_ = create_subscription<tf2_msgs::msg::TFMessage>(
      "/tf_static", rclcpp::QoS(100).reliable().transient_local(), observe_tf, tf_sub_options);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/Odometry", rclcpp::SensorDataQoS().keep_last(200),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {on_odom(msg);});
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/cloud_registered", rclcpp::SensorDataQoS().keep_last(1),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
        cloud_ = msg;
        cloud_receipt_ = Steady::now();
      });
    initial_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/initialpose", 10,
      [this](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg) {
        if (msg->header.frame_id != map_frame_) {
          RCLCPP_ERROR(get_logger(), "Reject /initialpose: frame must be %s", map_frame_.c_str());
          return;
        }
        try {
          const auto seed = pose_matrix(msg->pose.pose);
          request_seed(seed);
          RCLCPP_INFO(get_logger(), "Accepted /initialpose as base_link pose; awaiting verified registration");
        } catch (const std::exception & e) {
          RCLCPP_ERROR(get_logger(), "Reject /initialpose: %s", e.what());
        }
      });
    relocalize_srv_ = create_service<std_srvs::srv::Trigger>(
      "/cloud_localization/relocalize",
      [this](const std_srvs::srv::Trigger::Request::SharedPtr,
        std_srvs::srv::Trigger::Response::SharedPtr response) {
        response->success = !output_gate_.conflict();
        response->message = response->success ?
          "Seeded search queued using last supplied base_link initial pose; check status for result" :
          "TF_CONFLICT is latched: stop conflicting publisher yourself, then restart this node";
        if (response->success) {request_seed(seed_base_);}
      });

    const std::string map_file = declare_parameter<std::string>("map_file", "");
    pcl::PCLPointCloud2 map_blob;
    auto raw_map = std::make_shared<Cloud>();
    if (map_file.empty() || pcl::io::loadPCDFile(map_file, map_blob) < 0) {
      throw std::invalid_argument("map_file must name a readable 3D PCD containing x/y/z fields");
    }
    for (const std::string field : {"x", "y", "z"}) {
      if (std::none_of(map_blob.fields.begin(), map_blob.fields.end(),
        [&](const auto & f) {return f.name == field && f.datatype == 7 && f.count == 1;}))
      {
        throw std::invalid_argument("PCD must contain float32 x/y/z fields");
      }
    }
    pcl::fromPCLPointCloud2(map_blob, *raw_map);
    auto clean_map = std::make_shared<Cloud>();
    for (const auto & point : *raw_map) {
      if (pcl::isFinite(point)) {clean_map->push_back(point);}
    }
    target_ = downsample(clean_map, leaf_);
    if (target_->size() < static_cast<size_t>(min_points_)) {
      throw std::invalid_argument("PCD has too few finite points after voxel filtering");
    }
    publish_cloud(target_, map_pub_, now());
    // Map loading can be slow; the TF observation window starts only after it.
    boot_ = Steady::now();
    timer_ = create_wall_timer(std::chrono::milliseconds(100), [this]() {tick();});
    RCLCPP_WARN(get_logger(),
      "Stop AMCL/other map->odom publishers yourself before testing. "
      "No motion commands are published. Seeded %s search; initial pose is NOT localization success.",
      options_.backend.c_str());
  }

private:
  struct OdomSample
  {
    int64_t ns;
    Eigen::Matrix4f pose;
  };
  struct JobResult
  {
    RegistrationResult registration;
    double compute_ms;
  };
  struct Job
  {
    std::future<JobResult> future;
    Cloud::Ptr source;
    Eigen::Matrix4f odom_base;
    Eigen::Matrix4f guess;
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    Steady::time_point receipt;
    uint64_t generation;
  };

  double positive(const std::string & name, double fallback)
  {
    const double value = declare_parameter<double>(name, fallback);
    if (!std::isfinite(value) || value <= 0.0) {
      throw std::invalid_argument(name + " must be finite and positive");
    }
    return value;
  }
  double finite_parameter(const std::string & name)
  {
    const double value = declare_parameter<double>(name, 0.0);
    if (!std::isfinite(value)) {throw std::invalid_argument(name + " must be finite");}
    return value;
  }
  static double elapsed(Steady::time_point then)
  {
    return std::chrono::duration<double>(Steady::now() - then).count();
  }
  double age(const builtin_interfaces::msg::Time & stamp)
  {
    return (now() - rclcpp::Time(stamp)).seconds();
  }
  void fail(const std::string & reason)
  {
    good_ = 0;
    lost_ = true;
    state_ = reason;
  }
  void request_seed(const Eigen::Matrix4f & pose)
  {
    if (output_gate_.conflict()) {return;}
    seed_base_ = pose;
    need_seed_ = true;
    lost_ = false;
    good_ = 0;
    processed_ns_ = 0;
    ++generation_;  // Discard any registration that was started with an older seed.
    state_ = "ACQUIRING";
  }
  void check_tf(const tf2_msgs::msg::TFMessage & message, const rclcpp::MessageInfo & info)
  {
    const auto own_gid = tf_pub_->get_gid();
    const auto & incoming_gid = info.get_rmw_message_info().publisher_gid;
    if (std::memcmp(own_gid.data, incoming_gid.data, RMW_GID_STORAGE_SIZE) == 0) {return;}
    for (const auto & transform : message.transforms) {
      // odom already having any parent conflicts with this node's ownership.
      // A parent for map can also create a cycle with odom->base_link.
      if (transform.child_frame_id == odom_frame_ || transform.child_frame_id == map_frame_) {
        std::lock_guard<std::mutex> lock(tf_mutex_);
        if (output_gate_.latch_conflict()) {
          RCLCPP_ERROR(get_logger(),
            "TF_CONFLICT: another DDS publisher sent %s -> %s. TF output disabled until restart; "
            "no user process will be stopped.",
            transform.header.frame_id.c_str(), transform.child_frame_id.c_str());
        }
      }
    }
  }
  void on_odom(const nav_msgs::msg::Odometry::ConstSharedPtr & msg)
  {
    if (msg->header.frame_id != odom_frame_ || msg->child_frame_id != base_frame_ ||
      rclcpp::Time(msg->header.stamp).nanoseconds() <= 0)
    {
      fail("INVALID_ODOMETRY");
      return;
    }
    try {
      const auto matrix = pose_matrix(msg->pose.pose);
      const auto ns = rclcpp::Time(msg->header.stamp).nanoseconds();
      if (!odometry_.empty() && ns < odometry_.back().ns) {
        odometry_.clear();
        fail("ODOMETRY_TIME_RESET");
        ++generation_;
      }
      if (odometry_.empty() || ns > odometry_.back().ns) {
        odometry_.push_back({ns, matrix});
      }
      while (odometry_.size() > 2000 ||
        (!odometry_.empty() && ns - odometry_.front().ns > 10000000000LL))
      {
        odometry_.pop_front();
      }
      latest_odom_ = msg;
      odom_receipt_ = Steady::now();
    } catch (const std::exception &) {
      fail("INVALID_ODOMETRY");
    }
  }
  bool synchronized_pose(int64_t ns, Eigen::Matrix4f & pose)
  {
    auto upper = std::lower_bound(odometry_.begin(), odometry_.end(), ns,
      [](const OdomSample & sample, int64_t stamp) {return sample.ns < stamp;});
    if (upper != odometry_.end() && upper->ns == ns) {
      pose = upper->pose;
      return true;
    }
    // Never substitute latest odometry for an arbitrary old scan.
    if (upper == odometry_.begin() || upper == odometry_.end()) {return false;}
    const auto lower = std::prev(upper);
    if ((ns - lower->ns) * 1e-9 > sync_tolerance_ ||
      (upper->ns - ns) * 1e-9 > sync_tolerance_) {return false;}
    const float alpha = static_cast<double>(ns - lower->ns) / (upper->ns - lower->ns);
    const Eigen::Quaternionf q0(lower->pose.block<3, 3>(0, 0));
    const Eigen::Quaternionf q1(upper->pose.block<3, 3>(0, 0));
    pose = Eigen::Matrix4f::Identity();
    pose.block<3, 3>(0, 0) = q0.slerp(alpha, q1).toRotationMatrix();
    pose.block<3, 1>(0, 3) = (1.0f - alpha) * lower->pose.block<3, 1>(0, 3) +
      alpha * upper->pose.block<3, 1>(0, 3);
    return true;
  }
  bool fresh_inputs()
  {
    if (!cloud_ || !latest_odom_) {
      state_ = "WAITING_INPUT";
      return false;
    }
    const double cloud_age = age(cloud_->header.stamp);
    const double odom_age = age(latest_odom_->header.stamp);
    if (cloud_age < -0.05 || odom_age < -0.05) {
      fail("CLOCK_ERROR");
      return false;
    }
    if (elapsed(cloud_receipt_) > timeout_ || elapsed(odom_receipt_) > timeout_ ||
      cloud_age > max_age_ || odom_age > max_age_)
    {
      fail("INPUT_TIMEOUT");
      return false;
    }
    return true;
  }
  void tick()
  {
    if (output_gate_.conflict()) {
      fail("TF_CONFLICT");
    } else if (elapsed(boot_) < guard_sec_) {
      state_ = "CHECKING_TF";
    } else if (count_publishers(cloud_sub_->get_topic_name()) > 1 ||
      count_publishers(odom_sub_->get_topic_name()) > 1)
    {
      fail("INPUT_PUBLISHER_CONFLICT");
    } else if (fresh_inputs() && !lost_) {
      if (job_ && job_->future.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
        finish_job();
      }
      if (!job_ && !lost_ && !output_gate_.conflict()) {start_job();}
    }
    // Reap cancelled jobs even during a timeout/conflict; never publish their result.
    if (job_ && (lost_ || output_gate_.conflict()) &&
      job_->future.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
    {
      try {job_->future.get();} catch (const std::exception &) {}
      job_.reset();
    }
    if (output_gate_.conflict()) {state_ = "TF_CONFLICT";}
    publish_status();
  }
  void start_job()
  {
    const auto stamp = rclcpp::Time(cloud_->header.stamp);
    if (cloud_->header.frame_id != odom_frame_ || stamp.nanoseconds() <= 0) {
      fail("INVALID_CLOUD_FRAME_OR_STAMP");
      return;
    }
    if (stamp.nanoseconds() <= processed_ns_) {return;}
    Eigen::Matrix4f odom_base;
    if (!synchronized_pose(stamp.nanoseconds(), odom_base)) {
      // Missing odometry for a fresh scan must also withdraw TRACKING immediately.
      good_ = 0;
      state_ = "WAITING_SYNC";
      return;
    }
    processed_ns_ = stamp.nanoseconds();
    const auto has_xyz = [&]() {
        for (const std::string field : {"x", "y", "z"}) {
          const auto found = std::find_if(cloud_->fields.begin(), cloud_->fields.end(),
            [&](const auto & f) {return f.name == field && f.datatype == 7 && f.count == 1 &&
                     f.offset + 4 <= cloud_->point_step;});
          if (found == cloud_->fields.end()) {return false;}
        }
        return cloud_->point_step > 0 && !cloud_->is_bigendian &&
               cloud_->row_step >= static_cast<uint64_t>(cloud_->width) * cloud_->point_step &&
               cloud_->data.size() >= static_cast<uint64_t>(cloud_->row_step) * cloud_->height;
      };
    if (!has_xyz()) {fail("INVALID_CLOUD_LAYOUT"); return;}
    auto raw = std::make_shared<Cloud>();
    pcl::fromROSMsg(*cloud_, *raw);
    auto clean = std::make_shared<Cloud>();
    const Eigen::Vector3f origin = odom_base.block<3, 1>(0, 3);
    for (const auto & point : *raw) {
      const double range = (point.getVector3fMap() - origin).norm();
      if (pcl::isFinite(point) && range >= scan_min_range_ && range <= scan_max_range_) {
        clean->push_back(point);
      }
    }
    auto source = downsample(clean, leaf_);
    if (source->size() < static_cast<size_t>(min_points_)) {
      fail("LOST_TOO_FEW_POINTS");
      return;
    }
    if (need_seed_) {
      map_odom_ = map_to_odom_seed(seed_base_, odom_base);
      need_seed_ = false;
    }
    const auto target = target_;
    const auto guess = map_odom_;
    const auto options = options_;
    job_ = std::make_unique<Job>();
    job_->source = source;
    job_->guess = guess;
    job_->stamp = stamp;
    job_->odom_base = odom_base;
    job_->receipt = cloud_receipt_;
    job_->generation = generation_;
    if (good_ < required_good_) {state_ = "ACQUIRING";}
    job_->future = std::async(std::launch::async, [source, target, guess, options]() {
        const auto begin = Steady::now();
        const auto result = register_cloud(source, target, guess, options);
        return JobResult{result,
          std::chrono::duration<double, std::milli>(Steady::now() - begin).count()};
      });
  }
  void finish_job()
  {
    auto job = std::move(job_);
    if (job->generation != generation_) {
      try {job->future.get();} catch (const std::exception &) {}
      return;
    }
    JobResult result;
    try {
      result = job->future.get();
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "Registration failed: %s", e.what());
      fail("REGISTRATION_ERROR");
      return;
    }
    compute_ms_ = result.compute_ms;
    quality_ = result.registration;
    if ((now() - job->stamp).seconds() > max_age_ || elapsed(job->receipt) > timeout_) {
      fail("RESULT_TOO_OLD");
      return;
    }
    const Eigen::Matrix4f delta = job->guess.inverse() * quality_.transform;
    const double correction = delta.block<3, 1>(0, 3).norm();
    const double angle = Eigen::AngleAxisf(delta.block<3, 3>(0, 0)).angle();
    if (!acceptable(quality_, min_inliers_, max_rmse_) ||
      correction > max_translation_ || angle > max_rotation_)
    {
      fail("LOST_REGISTRATION_REJECTED");
      publish_aligned(*job);
      return;
    }
    map_odom_ = quality_.transform;
    ++good_;
    state_ = good_ >= required_good_ ? "TRACKING" : "ACQUIRING";
    publish_aligned(*job);
    if (good_ < required_good_) {return;}
    std::lock_guard<std::mutex> lock(tf_mutex_);
    if (!output_gate_.permits(good_, required_good_, lost_, elapsed(boot_) >= guard_sec_,
        (now() - job->stamp).seconds(), max_age_) || !fresh_inputs()) {return;}
    const auto pose = matrix_pose(map_odom_ * job->odom_base);
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.frame_id = map_frame_;
    pose_msg.header.stamp = job->stamp;
    pose_msg.pose = pose;
    pose_pub_->publish(pose_msg);
    last_pose_ = pose;
    result_stamp_ = job->stamp;
    geometry_msgs::msg::TransformStamped transform;
    transform.header = pose_msg.header;
    transform.child_frame_id = odom_frame_;
    const auto correction_pose = matrix_pose(map_odom_);
    transform.transform.translation.x = correction_pose.position.x;
    transform.transform.translation.y = correction_pose.position.y;
    transform.transform.translation.z = correction_pose.position.z;
    transform.transform.rotation = correction_pose.orientation;
    tf2_msgs::msg::TFMessage tf;
    tf.transforms.push_back(transform);
    tf_pub_->publish(tf);  // Dynamic, measurement-stamped, and only after quality gates.
  }
  void publish_aligned(const Job & job)
  {
    if (!quality_.transform.allFinite() || output_gate_.conflict()) {return;}
    auto aligned = std::make_shared<Cloud>();
    pcl::transformPointCloud(*job.source, *aligned, quality_.transform);
    publish_cloud(aligned, scan_pub_, job.stamp);
  }
  void publish_cloud(const Cloud::ConstPtr & cloud,
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & publisher,
    const rclcpp::Time & stamp)
  {
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(*cloud, msg);
    msg.header.frame_id = map_frame_;
    msg.header.stamp = stamp;
    publisher->publish(msg);
  }
  void publish_status()
  {
    // A previously accepted TF may remain in buffers. Only this state says valid.
    if (state_ == "TRACKING" &&
      (result_stamp_.nanoseconds() == 0 || (now() - result_stamp_).seconds() > max_age_))
    {
      fail("RESULT_TOO_OLD");
    }
    diagnostic_msgs::msg::DiagnosticArray message;
    message.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "cloud_localizer/localization";
    status.hardware_id = "FAST-LIO + PCD";
    status.message = state_;
    status.level = state_ == "TRACKING" ? 0 : (lost_ || output_gate_.conflict() ? 2 : 1);
    const auto add = [&](const std::string & key, const std::string & value) {
        diagnostic_msgs::msg::KeyValue pair;
        pair.key = key;
        pair.value = value;
        status.values.push_back(pair);
      };
    add("state", state_);
    add("localized", state_ == "TRACKING" ? "true" : "false");
    add("backend", options_.backend);
    add("tf_conflict", output_gate_.conflict() ? "true" : "false");
    add("inlier_ratio", std::to_string(quality_.inlier_ratio));
    add("rmse_m", std::to_string(quality_.rmse));
    add("compute_ms", std::to_string(compute_ms_));
    add("scan_age_ms", cloud_ ? std::to_string(1000.0 * age(cloud_->header.stamp)) : "nan");
    add("odom_age_ms", latest_odom_ ? std::to_string(1000.0 * age(latest_odom_->header.stamp)) : "nan");
    add("result_age_ms", result_stamp_.nanoseconds() ?
      std::to_string(1000.0 * (now() - result_stamp_).seconds()) : "nan");
    add("cloud_receipt_age_ms", cloud_ ? std::to_string(1000.0 * elapsed(cloud_receipt_)) : "nan");
    add("odom_receipt_age_ms", latest_odom_ ? std::to_string(1000.0 * elapsed(odom_receipt_)) : "nan");
    add("consecutive_good", std::to_string(good_));
    message.status.push_back(status);
    status_pub_->publish(message);
    if (state_ != logged_state_) {
      RCLCPP_INFO(get_logger(), "Localization state: %s", state_.c_str());
      logged_state_ = state_;
    }
    visualization_msgs::msg::Marker marker;
    marker.header = message.header;
    marker.header.frame_id = map_frame_;
    marker.ns = "localization_status";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose = last_pose_;
    marker.pose.orientation.w = 1.0;
    marker.pose.orientation.x = marker.pose.orientation.y = marker.pose.orientation.z = 0.0;
    marker.pose.position.z += 1.5;
    marker.scale.z = 0.25;
    marker.color.a = 1.0;
    marker.color.r = state_ == "TRACKING" ? 0.1 : 1.0;
    marker.color.g = state_ == "TRACKING" ? 1.0 : 0.2;
    marker.lifetime = rclcpp::Duration::from_seconds(0.6);
    std::ostringstream label;
    label << state_ << std::fixed << std::setprecision(2)
          << "\ninlier=" << quality_.inlier_ratio << " rmse=" << quality_.rmse
          << "m\ncompute=" << compute_ms_ << "ms age="
          << (cloud_ ? 1000.0 * age(cloud_->header.stamp) : -1.0) << "ms";
    marker.text = label.str();
    marker_pub_->publish(marker);
  }

  std::string map_frame_, odom_frame_, base_frame_, state_ = "WAITING_INPUT", logged_state_;
  RegistrationOptions options_;
  RegistrationResult quality_;
  double leaf_, max_rmse_, min_inliers_, max_age_, timeout_, sync_tolerance_, guard_sec_;
  double max_translation_, max_rotation_, scan_min_range_, scan_max_range_, compute_ms_ = NAN;
  int min_points_, required_good_, good_ = 0;
  bool need_seed_ = true, lost_ = false;
  OutputGate output_gate_;
  std::mutex tf_mutex_;
  Eigen::Matrix4f seed_base_ = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f map_odom_ = Eigen::Matrix4f::Identity();
  Cloud::Ptr target_;
  Pose last_pose_;
  int64_t processed_ns_ = 0;
  uint64_t generation_ = 0;
  rclcpp::Time result_stamp_{0, 0, RCL_ROS_TIME};
  Steady::time_point boot_, cloud_receipt_, odom_receipt_;
  std::deque<OdomSample> odometry_;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_;
  nav_msgs::msg::Odometry::ConstSharedPtr latest_odom_;
  std::unique_ptr<Job> job_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_, scan_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr status_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_pub_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_, static_tf_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr relocalize_srv_;
  rclcpp::CallbackGroup::SharedPtr tf_group_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace rc_cloud_localization

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int exit_code = 0;
  try {
    auto node = std::make_shared<rc_cloud_localization::CloudLocalizer>();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
    executor.add_node(node);
    executor.spin();
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("cloud_localizer"), "%s", e.what());
    exit_code = 1;
  }
  rclcpp::shutdown();
  return exit_code;
}
