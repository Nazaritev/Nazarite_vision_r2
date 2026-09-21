#include "r2_mode1_bt/ros_node.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <map>
#include <thread>

namespace r2_mode1_bt {

using namespace std::chrono_literals;

namespace {

constexpr uint8_t kFrameHeader = 0xAA;
constexpr uint8_t kFrameTail = 0x55;
constexpr std::size_t kPayloadLength = 7;
constexpr std::size_t kFrameLength = 11;

std::array<uint8_t, 2> ModbusCrc16(const std::vector<uint8_t>& data) {
    uint16_t crc = 0xFFFF;
    for (uint8_t byte : data) {
        crc ^= byte;
        for (int i = 0; i < 8; ++i) {
            if (crc & 0x0001U) {
                crc = static_cast<uint16_t>((crc >> 1U) ^ 0xA001U);
            } else {
                crc >>= 1U;
            }
        }
    }
    return {static_cast<uint8_t>(crc & 0xFFU), static_cast<uint8_t>((crc >> 8U) & 0xFFU)};
}

std::vector<uint8_t> BuildVisionFrame(const R2Mode1ROSNode::SerialCommandConfig& cfg) {
    std::vector<uint8_t> payload;
    payload.reserve(kPayloadLength);
    payload.push_back(cfg.region);
    payload.push_back(cfg.mode);
    payload.push_back(cfg.need_return);
    payload.push_back(static_cast<uint8_t>(cfg.data & 0xFF));
    payload.push_back(static_cast<uint8_t>((cfg.data >> 8) & 0xFF));
    payload.push_back(0x00);
    payload.push_back(0x00);

    const auto crc = ModbusCrc16(payload);

    std::vector<uint8_t> frame;
    frame.reserve(kFrameLength);
    frame.push_back(kFrameHeader);
    frame.insert(frame.end(), payload.begin(), payload.end());
    frame.push_back(crc[0]);
    frame.push_back(crc[1]);
    frame.push_back(kFrameTail);
    return frame;
}

const std::map<std::string, R2Mode1ROSNode::SerialCommandConfig> kSerialCommands = {
    {"1", {0x01, 0x01, 0x00, 0}},
    {"2", {0x01, 0x02, 0x00, 0}},
    {"3", {0x01, 0x03, 0x01, 0}},
    {"4", {0x01, 0x04, 0x01, 0}},
    {"5", {0x01, 0x05, 0x00, 0}},
    {"6", {0x01, 0x06, 0x01, 0}},
    {"7", {0x01, 0x07, 0x01, 0}},
    {"8", {0x01, 0x08, 0x01, 0}},
    {"9", {0x01, 0x09, 0x01, 0}},
};

std::string CommandFromMode(uint8_t mode) {
    for (const auto& [cmd, cfg] : kSerialCommands) {
        if (cfg.mode == mode) {
            return cmd;
        }
    }
    return {};
}

std::string TrimCopy(std::string value) {
    value.erase(value.begin(),
                std::find_if(value.begin(), value.end(), [](unsigned char ch) {
                    return !std::isspace(ch);
                }));
    value.erase(std::find_if(value.rbegin(), value.rend(), [](unsigned char ch) {
                    return !std::isspace(ch);
                }).base(),
                value.end());
    return value;
}

}  // namespace

R2Mode1ROSNode::R2Mode1ROSNode() : Node("r2_mode1_bt") {
    this->declare_parameter("target_tag_frame", "base");
    this->declare_parameter("reference_frame", "camera_link");
    this->declare_parameter("goal_checker_selector_topic", "/goal_checker_selector");
    this->declare_parameter("align_target_y", 0.0);
    this->declare_parameter("align_tolerance", 0.01);
    this->declare_parameter("stabilization_time", 0.5);
    this->declare_parameter("grab_check_delay", 1.25);
    this->declare_parameter("grab_depth_tolerance", 0.05);
    this->declare_parameter("post_barcode_shift_delay", 15.0);
    this->declare_parameter("dynamic_shift_forward_distance", 0.3);
    this->declare_parameter("preferred_start_grab_index", 0);
    this->declare_parameter("finish_after_first_success", true);
    this->declare_parameter("grab_goal_checker_id", "grab_goal_checker");
    this->declare_parameter("shift_goal_checker_id", "shift_goal_checker");
    this->declare_parameter("final_goal_checker_id", "shift_goal_checker");
    this->declare_parameter("serial_port", "/dev/ttyUSB1");
    this->declare_parameter("baudrate", 115200);
    this->declare_parameter("timeout", 1.0);
    this->declare_parameter("serial_repeat_enabled", false);
    this->declare_parameter("serial_repeat_interval", 1.0);

    target_tag_frame_ = this->get_parameter("target_tag_frame").as_string();
    reference_frame_ = this->get_parameter("reference_frame").as_string();
    goal_checker_selector_topic_ =
        this->get_parameter("goal_checker_selector_topic").as_string();
    align_target_y_ = this->get_parameter("align_target_y").as_double();
    align_tolerance_ = this->get_parameter("align_tolerance").as_double();
    stabilization_time_ = this->get_parameter("stabilization_time").as_double();
    grab_check_delay_ = this->get_parameter("grab_check_delay").as_double();
    grab_depth_tolerance_ = this->get_parameter("grab_depth_tolerance").as_double();
    post_barcode_shift_delay_ = this->get_parameter("post_barcode_shift_delay").as_double();
    dynamic_shift_forward_distance_ =
        this->get_parameter("dynamic_shift_forward_distance").as_double();
    preferred_start_grab_index_ =
        static_cast<int>(this->get_parameter("preferred_start_grab_index").as_int());
    finish_after_first_success_ =
        this->get_parameter("finish_after_first_success").as_bool();
    grab_goal_checker_id_ = this->get_parameter("grab_goal_checker_id").as_string();
    shift_goal_checker_id_ = this->get_parameter("shift_goal_checker_id").as_string();
    final_goal_checker_id_ = this->get_parameter("final_goal_checker_id").as_string();
    serial_repeat_enabled_ = this->get_parameter("serial_repeat_enabled").as_bool();
    serial_repeat_interval_ = this->get_parameter("serial_repeat_interval").as_double();

    const auto serial_port = this->get_parameter("serial_port").as_string();
    const auto baudrate = static_cast<uint32_t>(this->get_parameter("baudrate").as_int());
    const auto timeout_ms = static_cast<uint32_t>(this->get_parameter("timeout").as_double() * 1000.0);

    grab_points_ = {
        Point3d(-0.85, 0.75, 0.0, 0.0),
        Point3d(-0.85, 0.95, 0.0, 0.0),
        Point3d(-0.85, 1.15, 0.0, 0.0),
        Point3d(-0.85, 1.35, 0.0, 0.0),
        Point3d(-0.85, 1.55, 0.0, 0.0),
        Point3d(-0.85, 1.75, 0.0, 0.0),
    };
    shift_points_ = {
        Point3d(-0.60, 0.75, 0.0, 0.0),
        Point3d(-0.60, 0.95, 0.0, 0.0),
        Point3d(-0.60, 1.15, 0.0, 0.0),
        Point3d(-0.60, 1.35, 0.0, 0.0),
        Point3d(-0.60, 1.55, 0.0, 0.0),
        Point3d(-0.60, 1.75, 0.0, 0.0),
    };
    final_point_ = Point3d(1.85, 1.4, 0.0, 0.0);

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    auto selector_qos = rclcpp::QoS(rclcpp::KeepLast(1))
                            .reliable()
                            .transient_local();

    goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("goal_pose", qos);
    mode_pub_ = this->create_publisher<std_msgs::msg::Int32>("/chassis_mode", qos);
    external_mode_pub_ = this->create_publisher<std_msgs::msg::Int32>("/mode", qos);
    goal_checker_selector_pub_ =
        this->create_publisher<std_msgs::msg::String>(goal_checker_selector_topic_, selector_qos);

    nav_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/agent/arrival_status", qos, std::bind(&R2Mode1ROSNode::navCallback, this, std::placeholders::_1));
    object_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/object_detected", rclcpp::SensorDataQoS(),
        std::bind(&R2Mode1ROSNode::objectStatusCallback, this, std::placeholders::_1));
    depth_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        "/object_depth", rclcpp::SensorDataQoS(),
        std::bind(&R2Mode1ROSNode::objectDepthCallback, this, std::placeholders::_1));
    barcode_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/barcode", 10, std::bind(&R2Mode1ROSNode::barcodeCallback, this, std::placeholders::_1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/Odometry", qos, std::bind(&R2Mode1ROSNode::odomCallback, this, std::placeholders::_1));

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_, this);

    try {
        serial_ = std::make_unique<serial::Serial>();
        serial_->setPort(serial_port);
        serial_->setBaudrate(baudrate);
        auto timeout = serial::Timeout::simpleTimeout(timeout_ms);
        serial_->setTimeout(timeout);
        serial_->open();
        RCLCPP_INFO(this->get_logger(), "Serial connected: %s @ %u", serial_port.c_str(), baudrate);
    } catch (const std::exception& e) {
        serial_.reset();
        RCLCPP_ERROR(this->get_logger(), "Serial open failed: %s", e.what());
    }

    serial_thread_ = std::thread(&R2Mode1ROSNode::listenSTM32Data, this);
    if (serial_repeat_enabled_) {
        serial_repeat_thread_ = std::thread(&R2Mode1ROSNode::repeatSerialCommand, this);
    }

    RCLCPP_INFO(this->get_logger(), "R2Mode1ROSNode initialized with updated state-machine flow");
    RCLCPP_INFO(this->get_logger(), "Serial command send mode: %s",
                serial_repeat_enabled_ ? "repeat" : "single-shot");
}

R2Mode1ROSNode::~R2Mode1ROSNode() {
    node_running_ = false;
    if (serial_thread_.joinable()) {
        serial_thread_.join();
    }
    if (serial_repeat_thread_.joinable()) {
        serial_repeat_thread_.join();
    }
    if (serial_ && serial_->isOpen()) {
        serial_->close();
    }
}

bool R2Mode1ROSNode::sendSerial(const std::string& cmd, bool repeat) {
    if (!repeat) {
        clearSerialCommandCompleted(cmd);
    }
    return writeSerialFrame(cmd, repeat);
}

void R2Mode1ROSNode::setActiveSerialCommand(const std::string& cmd) {
    std::lock_guard<std::mutex> lock(mutex_);
    active_serial_cmd_ = cmd;
}

bool R2Mode1ROSNode::consumeSerialCommandCompleted(const std::string& cmd) {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = serial_command_completed_.find(cmd);
    if (it == serial_command_completed_.end() || !it->second) {
        return false;
    }
    it->second = false;
    return true;
}

void R2Mode1ROSNode::clearSerialCommandCompleted(const std::string& cmd) {
    std::lock_guard<std::mutex> lock(mutex_);
    serial_command_completed_[cmd] = false;
}

void R2Mode1ROSNode::stopSerial(const std::string& cmd) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (active_serial_cmd_.empty()) {
        return;
    }
    if (!cmd.empty() && active_serial_cmd_ != cmd) {
        return;
    }
    RCLCPP_INFO(this->get_logger(), "Clear active serial command %s", active_serial_cmd_.c_str());
    active_serial_cmd_.clear();
}

std::string R2Mode1ROSNode::getActiveSerialCommand() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return active_serial_cmd_;
}

bool R2Mode1ROSNode::isSerialOpen() const {
    return serial_ && serial_->isOpen();
}

void R2Mode1ROSNode::publishGoal(const Point3d& goal) {
    geometry_msgs::msg::PoseStamped msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "odom";
    msg.pose.position.x = goal.x;
    msg.pose.position.y = goal.y;
    msg.pose.position.z = goal.z;

    const double cy = std::cos(goal.yaw * 0.5);
    const double sy = std::sin(goal.yaw * 0.5);
    msg.pose.orientation.z = sy;
    msg.pose.orientation.w = cy;

    goal_pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Goal published: (%.2f, %.2f, %.2f, %.2f)",
                goal.x, goal.y, goal.z, goal.yaw);
}

void R2Mode1ROSNode::publishMode(int mode) {
    std_msgs::msg::Int32 msg;
    msg.data = mode;
    mode_pub_->publish(msg);
}

void R2Mode1ROSNode::publishExternalMode(int mode) {
    std_msgs::msg::Int32 msg;
    msg.data = mode;
    external_mode_pub_->publish(msg);
}

void R2Mode1ROSNode::selectGoalCheckerForTarget(const std::string& checker_id,
                                                const std::string& target_name) {
    const std::string normalized_checker_id = TrimCopy(checker_id);

    if (normalized_checker_id.empty()) {
        RCLCPP_WARN(this->get_logger(), "Target=%s has no goal checker configured, keeping current selection",
                    target_name.c_str());
        return;
    }

    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (current_goal_checker_id_ == normalized_checker_id) {
            return;
        }
        current_goal_checker_id_ = normalized_checker_id;
    }

    std_msgs::msg::String msg;
    msg.data = normalized_checker_id;
    goal_checker_selector_pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Selected goal checker -> %s | target=%s",
                normalized_checker_id.c_str(), target_name.c_str());
}

void R2Mode1ROSNode::setChassisMode(ChassisMode mode) {
    std::lock_guard<std::mutex> lock(mutex_);
    publishMode(static_cast<int>(mode));
    if (current_chassis_mode_ != mode) {
        current_chassis_mode_ = mode;
        RCLCPP_INFO(this->get_logger(), "Chassis mode switched to %d", static_cast<int>(mode));
    }
}

bool R2Mode1ROSNode::getNavArrived() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return nav_arrived_;
}

bool R2Mode1ROSNode::consumeNavArrived() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!nav_arrived_) {
        return false;
    }
    nav_arrived_ = false;
    return true;
}

void R2Mode1ROSNode::resetNavArrived() {
    std::lock_guard<std::mutex> lock(mutex_);
    nav_arrived_ = false;
}

bool R2Mode1ROSNode::getObjectPresent() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return object_present_;
}

float R2Mode1ROSNode::getCurrentDepth() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return current_depth_;
}

bool R2Mode1ROSNode::hasDepthSample() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return depth_sample_received_;
}

std::uint64_t R2Mode1ROSNode::getObjectStatusRxCount() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return object_status_rx_count_;
}

std::string R2Mode1ROSNode::getBarcodeData() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return barcode_data_;
}

void R2Mode1ROSNode::clearBarcodeData() {
    std::lock_guard<std::mutex> lock(mutex_);
    barcode_data_.clear();
}

std::string R2Mode1ROSNode::getLastBarcodeBuffer() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return last_barcode_buffer_;
}

void R2Mode1ROSNode::clearLastBarcodeBuffer() {
    std::lock_guard<std::mutex> lock(mutex_);
    last_barcode_buffer_.clear();
}

void R2Mode1ROSNode::setBarcodeWaitActive(bool active) {
    std::lock_guard<std::mutex> lock(mutex_);
    barcode_wait_active_ = active;
}

bool R2Mode1ROSNode::getBarcodeWaitActive() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return barcode_wait_active_;
}

bool R2Mode1ROSNode::getSpecialBarcodeTriggered() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return special_barcode_triggered_;
}

void R2Mode1ROSNode::setSpecialBarcodeTriggered(bool value) {
    std::lock_guard<std::mutex> lock(mutex_);
    special_barcode_triggered_ = value;
}

bool R2Mode1ROSNode::navInterfacesReady() const {
    return goal_pub_->get_subscription_count() >= 1U && this->count_publishers("/agent/arrival_status") >= 1U;
}

bool R2Mode1ROSNode::lookupTransform(const std::string& target_frame,
                                     const std::string& source_frame,
                                     geometry_msgs::msg::TransformStamped& transform) {
    try {
        transform = tf_buffer_->lookupTransform(target_frame, source_frame, tf2::TimePointZero,
                                                tf2::durationFromSec(0.2));
        return true;
    } catch (const tf2::TransformException&) {
        return false;
    }
}

R2Mode1ROSNode::NavTarget R2Mode1ROSNode::getActiveNavTarget() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return active_nav_target_;
}

int R2Mode1ROSNode::getActiveNavIndex() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return active_nav_index_;
}

void R2Mode1ROSNode::setActiveNavTarget(NavTarget target, int index) {
    std::lock_guard<std::mutex> lock(mutex_);
    active_nav_target_ = target;
    active_nav_index_ = index;
}

void R2Mode1ROSNode::clearActiveNavTarget() {
    std::lock_guard<std::mutex> lock(mutex_);
    active_nav_target_ = NavTarget::None;
    active_nav_index_ = -1;
}

bool R2Mode1ROSNode::hasPendingFinalExit() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return pending_final_exit_;
}

void R2Mode1ROSNode::requestFinalExit() {
    std::lock_guard<std::mutex> lock(mutex_);
    pending_final_exit_ = true;
}

void R2Mode1ROSNode::clearPendingFinalExit() {
    std::lock_guard<std::mutex> lock(mutex_);
    pending_final_exit_ = false;
}

Point3d R2Mode1ROSNode::buildShiftReturnGoal(int index) {
    Point3d current_pose;
    bool current_pose_valid = false;
    double forward_distance = 0.0;

    {
        std::lock_guard<std::mutex> lock(mutex_);
        current_pose = current_odom_pose_;
        current_pose_valid = current_odom_pose_valid_;
        forward_distance = dynamic_shift_forward_distance_;
    }

    if (current_pose_valid) {
        RCLCPP_INFO(this->get_logger(),
                    "Build shift-return goal from odom: (%.3f, %.3f) -> (%.3f, %.3f)",
                    current_pose.x, current_pose.y, current_pose.x + forward_distance, current_pose.y);
        return Point3d(
            current_pose.x + forward_distance,
            current_pose.y,
            current_pose.z,
            current_pose.yaw);
    }

    if (index >= 0 && index < static_cast<int>(shift_points_.size())) {
        RCLCPP_WARN(this->get_logger(),
                    "No /Odometry yet, fallback shift-return goal to shift point %d", index);
        return shift_points_[index];
    }

    RCLCPP_WARN(this->get_logger(),
                "No /Odometry and invalid shift point index %d, fallback to origin", index);
    return Point3d();
}

int R2Mode1ROSNode::resolveStartGrabIndex() const {
    if (grab_points_.empty()) {
        return 0;
    }
    if (preferred_start_grab_index_ < 0) {
        RCLCPP_WARN(this->get_logger(),
                    "preferred_start_grab_index=%d invalid, fallback to 0",
                    preferred_start_grab_index_);
        return 0;
    }
    if (preferred_start_grab_index_ >= static_cast<int>(grab_points_.size())) {
        const int fallback_index = static_cast<int>(grab_points_.size()) - 1;
        RCLCPP_WARN(this->get_logger(),
                    "preferred_start_grab_index out of range, fallback to %d",
                    fallback_index);
        return fallback_index;
    }
    return preferred_start_grab_index_;
}

int R2Mode1ROSNode::getMaxGrabCount() const {
    return static_cast<int>(grab_points_.size());
}

bool R2Mode1ROSNode::getFinishAfterFirstSuccess() const {
    return finish_after_first_success_;
}

void R2Mode1ROSNode::navCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    if (!msg->data) {
        return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    nav_arrived_ = true;
}

void R2Mode1ROSNode::objectStatusCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    object_present_ = msg->data;
    ++object_status_rx_count_;
}

void R2Mode1ROSNode::objectDepthCallback(const std_msgs::msg::Float32::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    current_depth_ = msg->data;
    depth_sample_received_ = true;
}

void R2Mode1ROSNode::barcodeCallback(const std_msgs::msg::String::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!barcode_wait_active_) {
        return;
    }

    if (msg->data == "200" && special_barcode_triggered_) {
        return;
    }

    barcode_data_ = msg->data;
    last_barcode_buffer_ = msg->data;
    if (barcode_data_ == "200" && !special_barcode_triggered_) {
        special_barcode_triggered_ = true;
        RCLCPP_INFO(this->get_logger(), "Special barcode 200 received");
    }
}

void R2Mode1ROSNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    const auto& position = msg->pose.pose.position;
    const auto& q = msg->pose.pose.orientation;
    const double yaw = quaternionToYaw(q.x, q.y, q.z, q.w);

    std::lock_guard<std::mutex> lock(mutex_);
    current_odom_pose_ = Point3d(position.x, position.y, position.z, yaw);
    current_odom_pose_valid_ = true;
}

void R2Mode1ROSNode::listenSTM32Data() {
    while (node_running_ && rclcpp::ok()) {
        if (!serial_ || !serial_->isOpen()) {
            std::this_thread::sleep_for(1s);
            continue;
        }

        try {
            const auto available = serial_->available();
            if (available == 0U) {
                std::this_thread::sleep_for(10ms);
                continue;
            }

            const auto data = serial_->read(available);
            if (data.empty()) {
                std::this_thread::sleep_for(10ms);
                continue;
            }

            {
                std::lock_guard<std::mutex> lock(mutex_);
                serial_rx_buffer_.insert(serial_rx_buffer_.end(), data.begin(), data.end());
            }
            processSerialFrames();
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Serial read failed: %s", e.what());
            std::this_thread::sleep_for(10ms);
        }
    }
}

void R2Mode1ROSNode::repeatSerialCommand() {
    while (node_running_ && rclcpp::ok()) {
        std::string cmd;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            cmd = active_serial_cmd_;
        }

        if (cmd.empty()) {
            std::this_thread::sleep_for(50ms);
            continue;
        }

        std::this_thread::sleep_for(std::chrono::duration<double>(serial_repeat_interval_));
        if (!node_running_ || !rclcpp::ok()) {
            return;
        }

        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (cmd != active_serial_cmd_) {
                continue;
            }
        }

        writeSerialFrame(cmd, true);
    }
}

void R2Mode1ROSNode::processSerialFrames() {
    while (true) {
        std::vector<uint8_t> frame;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            while (!serial_rx_buffer_.empty() && serial_rx_buffer_.front() != kFrameHeader) {
                serial_rx_buffer_.erase(serial_rx_buffer_.begin());
            }

            if (serial_rx_buffer_.size() < kFrameLength) {
                return;
            }

            if (serial_rx_buffer_[kFrameLength - 1] != kFrameTail) {
                serial_rx_buffer_.erase(serial_rx_buffer_.begin());
                continue;
            }

            frame.assign(serial_rx_buffer_.begin(), serial_rx_buffer_.begin() + kFrameLength);
            serial_rx_buffer_.erase(serial_rx_buffer_.begin(), serial_rx_buffer_.begin() + kFrameLength);
        }

        handleSerialFrame(frame);
    }
}

void R2Mode1ROSNode::handleSerialFrame(const std::vector<uint8_t>& frame) {
    if (frame.size() != kFrameLength) {
        return;
    }

    const std::vector<uint8_t> payload(frame.begin() + 1, frame.begin() + 1 + static_cast<long>(kPayloadLength));
    const auto crc = ModbusCrc16(payload);
    if (frame[8] != crc[0] || frame[9] != crc[1]) {
        RCLCPP_WARN(this->get_logger(), "Discarding serial frame with invalid CRC");
        return;
    }

    const uint8_t mode = payload[1];
    const uint8_t task_complete = payload[2];

    if (task_complete == 1U) {
        const std::string completed_cmd = CommandFromMode(mode);
        if (!completed_cmd.empty()) {
            std::lock_guard<std::mutex> lock(mutex_);
            serial_command_completed_[completed_cmd] = true;
        }
    }

    std::string active_cmd;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        active_cmd = active_serial_cmd_;
    }

    if (active_cmd.empty()) {
        return;
    }

    const auto it = kSerialCommands.find(active_cmd);
    if (it == kSerialCommands.end()) {
        return;
    }

    if (it->second.mode == mode && task_complete == 1U) {
        stopSerial(active_cmd);
    }
}

bool R2Mode1ROSNode::writeSerialFrame(const std::string& cmd, bool repeat) {
    const auto it = kSerialCommands.find(cmd);
    if (it == kSerialCommands.end()) {
        if (!repeat) {
            RCLCPP_WARN(this->get_logger(), "Unknown serial command: %s", cmd.c_str());
        }
        return false;
    }

    if (!repeat) {
        setActiveSerialCommand(cmd);
    }

    if (!serial_ || !serial_->isOpen()) {
        if (!repeat) {
            RCLCPP_WARN(this->get_logger(), "Serial not available, skipping: %s", cmd.c_str());
        }
        return false;
    }

    try {
        const auto frame = BuildVisionFrame(it->second);
        serial_->flushOutput();
        serial_->write(frame);
        if (!repeat) {
            RCLCPP_INFO(this->get_logger(), "Serial command sent: %s", cmd.c_str());
        }
        return true;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Serial write failed for %s: %s", cmd.c_str(), e.what());
        return false;
    }
}

double R2Mode1ROSNode::quaternionToYaw(double x, double y, double z, double w) {
    const double siny_cosp = 2.0 * (w * z + x * y);
    const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    return std::atan2(siny_cosp, cosy_cosp);
}

}  // namespace r2_mode1_bt
