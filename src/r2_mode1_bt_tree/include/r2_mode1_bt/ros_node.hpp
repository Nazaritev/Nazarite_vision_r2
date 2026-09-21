#ifndef R2_MODE1_BT_ROS_NODE_HPP_
#define R2_MODE1_BT_ROS_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <serial/serial.h>

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "r2_mode1_bt/point3d.hpp"

namespace r2_mode1_bt {

class R2Mode1ROSNode : public rclcpp::Node {
public:
    struct SerialCommandConfig {
        uint8_t region;
        uint8_t mode;
        uint8_t need_return;
        int16_t data;
    };

    enum class NavTarget {
        None = 0,
        GrabPoint,
        ShiftPoint,
        ShiftPointReturn,
        FinalPoint,
    };

    enum class ChassisMode : int {
        Navigation = 0,
        SpinForward = 1,
        Hold = 2,
        SpinBackward = 3,
    };

    R2Mode1ROSNode();
    ~R2Mode1ROSNode();

    bool sendSerial(const std::string& cmd, bool repeat = false);
    void setActiveSerialCommand(const std::string& cmd);
    bool consumeSerialCommandCompleted(const std::string& cmd);
    void clearSerialCommandCompleted(const std::string& cmd);
    void stopSerial(const std::string& cmd = "");
    std::string getActiveSerialCommand() const;
    bool isSerialOpen() const;

    void publishGoal(const Point3d& goal);
    void publishMode(int mode);
    void publishExternalMode(int mode);
    void selectGoalCheckerForTarget(const std::string& checker_id, const std::string& target_name);
    void setChassisMode(ChassisMode mode);

    bool getNavArrived() const;
    bool consumeNavArrived();
    void resetNavArrived();
    bool getObjectPresent() const;
    float getCurrentDepth() const;
    std::string getBarcodeData() const;
    void clearBarcodeData();
    std::string getLastBarcodeBuffer() const;
    void clearLastBarcodeBuffer();
    void setBarcodeWaitActive(bool active);
    bool getBarcodeWaitActive() const;
    bool getSpecialBarcodeTriggered() const;
    void setSpecialBarcodeTriggered(bool value);
    bool navInterfacesReady() const;

    bool lookupTransform(const std::string& target_frame,
                         const std::string& source_frame,
                         geometry_msgs::msg::TransformStamped& transform);

    const std::vector<Point3d>& getGrabPoints() const { return grab_points_; }
    const std::vector<Point3d>& getShiftPoints() const { return shift_points_; }
    const Point3d& getFinalPoint() const { return final_point_; }

    NavTarget getActiveNavTarget() const;
    int getActiveNavIndex() const;
    void setActiveNavTarget(NavTarget target, int index);
    void clearActiveNavTarget();

    bool hasPendingFinalExit() const;
    void requestFinalExit();
    void clearPendingFinalExit();

    Point3d buildShiftReturnGoal(int index);
    int resolveStartGrabIndex() const;
    int getMaxGrabCount() const;
    bool getFinishAfterFirstSuccess() const;
    bool hasDepthSample() const;
    std::uint64_t getObjectStatusRxCount() const;

    double getAlignTargetY() const { return align_target_y_; }
    double getAlignTolerance() const { return align_tolerance_; }
    double getStabilizationTime() const { return stabilization_time_; }
    double getGrabCheckDelay() const { return grab_check_delay_; }
    double getGrabDepthTolerance() const { return grab_depth_tolerance_; }
    double getPostBarcodeShiftDelay() const { return post_barcode_shift_delay_; }
    double getDynamicShiftForwardDistance() const { return dynamic_shift_forward_distance_; }
    std::string getTargetTagFrame() const { return target_tag_frame_; }
    std::string getReferenceFrame() const { return reference_frame_; }
    std::string getGrabGoalCheckerId() const { return grab_goal_checker_id_; }
    std::string getShiftGoalCheckerId() const { return shift_goal_checker_id_; }
    std::string getFinalGoalCheckerId() const { return final_goal_checker_id_; }

private:
    void navCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void objectStatusCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void objectDepthCallback(const std_msgs::msg::Float32::SharedPtr msg);
    void barcodeCallback(const std_msgs::msg::String::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

    void listenSTM32Data();
    void repeatSerialCommand();
    void processSerialFrames();
    void handleSerialFrame(const std::vector<uint8_t>& frame);
    bool writeSerialFrame(const std::string& cmd, bool repeat);

    static double quaternionToYaw(double x, double y, double z, double w);

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr mode_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr external_mode_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr goal_checker_selector_pub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr nav_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr object_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr depth_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr barcode_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

    std::unique_ptr<serial::Serial> serial_;
    std::thread serial_thread_;
    std::thread serial_repeat_thread_;
    std::atomic<bool> node_running_{true};

    mutable std::mutex mutex_;
    bool nav_arrived_{false};
    bool object_present_{false};
    float current_depth_{0.0f};
    bool depth_sample_received_{false};
    std::uint64_t object_status_rx_count_{0};
    std::string barcode_data_;
    std::string last_barcode_buffer_;
    bool barcode_wait_active_{false};
    bool special_barcode_triggered_{false};
    NavTarget active_nav_target_{NavTarget::None};
    int active_nav_index_{-1};
    ChassisMode current_chassis_mode_{ChassisMode::Navigation};
    bool pending_final_exit_{false};
    Point3d current_odom_pose_;
    bool current_odom_pose_valid_{false};
    std::string active_serial_cmd_;
    std::string current_goal_checker_id_;
    std::unordered_map<std::string, bool> serial_command_completed_;

    std::vector<uint8_t> serial_rx_buffer_;

    std::vector<Point3d> grab_points_;
    std::vector<Point3d> shift_points_;
    Point3d final_point_;

    std::string target_tag_frame_;
    std::string reference_frame_;
    std::string goal_checker_selector_topic_;
    std::string grab_goal_checker_id_;
    std::string shift_goal_checker_id_;
    std::string final_goal_checker_id_;
    double align_target_y_{0.0};
    double align_tolerance_{0.01};
    double stabilization_time_{0.5};
    double grab_check_delay_{1.25};
    double grab_depth_tolerance_{0.05};
    double post_barcode_shift_delay_{15.0};
    double dynamic_shift_forward_distance_{0.3};
    int preferred_start_grab_index_{0};
    bool finish_after_first_success_{true};
    bool serial_repeat_enabled_{false};
    double serial_repeat_interval_{1.0};
};

}  // namespace r2_mode1_bt

#endif  // R2_MODE1_BT_ROS_NODE_HPP_
