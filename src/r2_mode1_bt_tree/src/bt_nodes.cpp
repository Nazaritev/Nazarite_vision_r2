#include "r2_mode1_bt/bt_nodes.hpp"

#include "r2_mode1_bt/blackboard_keys.hpp"

#include <algorithm>
#include <cmath>
#include <string>

namespace r2_mode1_bt {

namespace {

std::shared_ptr<R2Mode1ROSNode> g_ros_node;

std::shared_ptr<R2Mode1ROSNode> GetROSNode(const BT::NodeConfig& config) {
    (void)config;
    return g_ros_node;
}

}  // namespace

WaitForNavReadyNode::WaitForNavReadyNode(const std::string& name, const BT::NodeConfig& config)
    : BT::StatefulActionNode(name, config), ros_node_(GetROSNode(config)) {}

BT::PortsList WaitForNavReadyNode::providedPorts() {
    return {};
}

BT::NodeStatus WaitForNavReadyNode::onStart() {
    return ros_node_->navInterfacesReady() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
}

BT::NodeStatus WaitForNavReadyNode::onRunning() {
    return ros_node_->navInterfacesReady() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
}

void WaitForNavReadyNode::onHalted() {}

PublishGrabGoalNode::PublishGrabGoalNode(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config), ros_node_(GetROSNode(config)) {}

BT::PortsList PublishGrabGoalNode::providedPorts() {
    return {};
}

BT::NodeStatus PublishGrabGoalNode::tick() {
    int point_index = 0;
    const bool got_point_index = config().blackboard->get(bb_keys::kPointIndex, point_index);
    (void)got_point_index;
    const auto& points = ros_node_->getGrabPoints();
    if (point_index < 0 || point_index >= static_cast<int>(points.size())) {
        RCLCPP_ERROR(ros_node_->get_logger(), "Invalid grab point index: %d", point_index);
        return BT::NodeStatus::FAILURE;
    }

    ros_node_->selectGoalCheckerForTarget(
        ros_node_->getGrabGoalCheckerId(),
        "grab point " + std::to_string(point_index));
    ros_node_->setActiveNavTarget(R2Mode1ROSNode::NavTarget::GrabPoint, point_index);
    ros_node_->setChassisMode(R2Mode1ROSNode::ChassisMode::Navigation);
    ros_node_->publishGoal(points[point_index]);
    RCLCPP_INFO(ros_node_->get_logger(), "Navigating to grab point %d", point_index);
    return BT::NodeStatus::SUCCESS;
}

PublishShiftGoalNode::PublishShiftGoalNode(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config), ros_node_(GetROSNode(config)) {}

BT::PortsList PublishShiftGoalNode::providedPorts() {
    return {};
}

BT::NodeStatus PublishShiftGoalNode::tick() {
    int point_index = 0;
    const bool got_point_index = config().blackboard->get(bb_keys::kPointIndex, point_index);
    (void)got_point_index;
    const auto& points = ros_node_->getShiftPoints();
    if (point_index < 0 || point_index >= static_cast<int>(points.size())) {
        RCLCPP_ERROR(ros_node_->get_logger(), "Invalid shift point index: %d", point_index);
        return BT::NodeStatus::FAILURE;
    }

    ros_node_->selectGoalCheckerForTarget(
        ros_node_->getShiftGoalCheckerId(),
        "shift point " + std::to_string(point_index));
    ros_node_->setActiveNavTarget(R2Mode1ROSNode::NavTarget::ShiftPoint, point_index);
    ros_node_->setChassisMode(R2Mode1ROSNode::ChassisMode::Navigation);
    ros_node_->publishGoal(points[point_index]);
    RCLCPP_INFO(ros_node_->get_logger(), "Navigating to shift point %d", point_index);
    return BT::NodeStatus::SUCCESS;
}

PublishShiftReturnGoalNode::PublishShiftReturnGoalNode(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config), ros_node_(GetROSNode(config)) {}

BT::PortsList PublishShiftReturnGoalNode::providedPorts() {
    return {};
}

BT::NodeStatus PublishShiftReturnGoalNode::tick() {
    int point_index = 0;
    const bool got_point_index = config().blackboard->get(bb_keys::kPointIndex, point_index);
    (void)got_point_index;
    const auto& points = ros_node_->getShiftPoints();
    if (point_index < 0 || point_index >= static_cast<int>(points.size())) {
        RCLCPP_ERROR(ros_node_->get_logger(), "Invalid shift return index: %d", point_index);
        return BT::NodeStatus::FAILURE;
    }

    const auto goal = ros_node_->buildShiftReturnGoal(point_index);
    ros_node_->selectGoalCheckerForTarget(
        ros_node_->getShiftGoalCheckerId(),
        "shift return point " + std::to_string(point_index));
    ros_node_->setActiveNavTarget(R2Mode1ROSNode::NavTarget::ShiftPointReturn, point_index);
    ros_node_->setChassisMode(R2Mode1ROSNode::ChassisMode::Navigation);
    ros_node_->publishGoal(goal);
    RCLCPP_INFO(ros_node_->get_logger(), "Navigating to shift return point %d", point_index);
    return BT::NodeStatus::SUCCESS;
}

PublishFinalGoalNode::PublishFinalGoalNode(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config), ros_node_(GetROSNode(config)) {}

BT::PortsList PublishFinalGoalNode::providedPorts() {
    return {};
}

BT::NodeStatus PublishFinalGoalNode::tick() {
    ros_node_->selectGoalCheckerForTarget(
        ros_node_->getFinalGoalCheckerId(),
        "final point");
    ros_node_->setActiveNavTarget(R2Mode1ROSNode::NavTarget::FinalPoint, -1);
    ros_node_->setChassisMode(R2Mode1ROSNode::ChassisMode::Navigation);
    ros_node_->publishGoal(ros_node_->getFinalPoint());
    RCLCPP_INFO(ros_node_->get_logger(), "Navigating to final point");
    return BT::NodeStatus::SUCCESS;
}

WaitForNavArrivalNode::WaitForNavArrivalNode(const std::string& name, const BT::NodeConfig& config)
    : BT::StatefulActionNode(name, config), ros_node_(GetROSNode(config)) {}

BT::PortsList WaitForNavArrivalNode::providedPorts() {
    return {};
}

BT::NodeStatus WaitForNavArrivalNode::onStart() {
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus WaitForNavArrivalNode::onRunning() {
    if (!ros_node_->consumeNavArrived()) {
        return BT::NodeStatus::RUNNING;
    }

    const auto nav_target = ros_node_->getActiveNavTarget();
    const int nav_index = ros_node_->getActiveNavIndex();
    ros_node_->clearActiveNavTarget();

    if ((nav_target == R2Mode1ROSNode::NavTarget::GrabPoint ||
         nav_target == R2Mode1ROSNode::NavTarget::ShiftPointReturn ||
         nav_target == R2Mode1ROSNode::NavTarget::FinalPoint) &&
        ros_node_->getActiveSerialCommand() == "3") {
        ros_node_->stopSerial("3");
    }

    if (nav_target == R2Mode1ROSNode::NavTarget::GrabPoint) {
        config().blackboard->set(bb_keys::kCurrentNavTarget, std::string("grab"));
        config().blackboard->set(bb_keys::kLastArrivedIndex, nav_index);
        RCLCPP_INFO(ros_node_->get_logger(), "Arrived at grab point %d", nav_index);
    } else if (nav_target == R2Mode1ROSNode::NavTarget::ShiftPoint) {
        config().blackboard->set(bb_keys::kCurrentNavTarget, std::string("shift"));
        config().blackboard->set(bb_keys::kLastArrivedIndex, nav_index);
        RCLCPP_INFO(ros_node_->get_logger(), "Arrived at shift point %d", nav_index);
    } else if (nav_target == R2Mode1ROSNode::NavTarget::ShiftPointReturn) {
        config().blackboard->set(bb_keys::kCurrentNavTarget, std::string("shift_return"));
        config().blackboard->set(bb_keys::kLastArrivedIndex, nav_index);
        RCLCPP_INFO(ros_node_->get_logger(), "Arrived at shift return point %d", nav_index);
    } else if (nav_target == R2Mode1ROSNode::NavTarget::FinalPoint) {
        config().blackboard->set(bb_keys::kCurrentNavTarget, std::string("final"));
        config().blackboard->set(bb_keys::kLastArrivedIndex, -1);
        RCLCPP_INFO(ros_node_->get_logger(), "Arrived at final point");
    } else {
        config().blackboard->set(bb_keys::kCurrentNavTarget, std::string("unknown"));
        config().blackboard->set(bb_keys::kLastArrivedIndex, nav_index);
    }

    return BT::NodeStatus::SUCCESS;
}

void WaitForNavArrivalNode::onHalted() {}

StabilizationWaitNode::StabilizationWaitNode(const std::string& name, const BT::NodeConfig& config)
    : BT::StatefulActionNode(name, config), ros_node_(GetROSNode(config)) {}

BT::PortsList StabilizationWaitNode::providedPorts() {
    return {};
}

BT::NodeStatus StabilizationWaitNode::onStart() {
    start_time_ = ros_node_->now();
    duration_ = ros_node_->getStabilizationTime();
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus StabilizationWaitNode::onRunning() {
    return ((ros_node_->now() - start_time_).seconds() >= duration_) ? BT::NodeStatus::SUCCESS
                                                                      : BT::NodeStatus::RUNNING;
}

void StabilizationWaitNode::onHalted() {}

WaitForDetectionReadyNode::WaitForDetectionReadyNode(const std::string& name, const BT::NodeConfig& config)
    : BT::StatefulActionNode(name, config), ros_node_(GetROSNode(config)) {}

BT::PortsList WaitForDetectionReadyNode::providedPorts() {
    return {};
}

BT::NodeStatus WaitForDetectionReadyNode::onStart() {
    start_object_status_rx_count_ = ros_node_->getObjectStatusRxCount();
    if (ros_node_->getObjectPresent() && ros_node_->hasDepthSample()) {
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus WaitForDetectionReadyNode::onRunning() {
    if (ros_node_->getObjectPresent()) {
        return ros_node_->hasDepthSample() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
    }
    return ros_node_->getObjectStatusRxCount() > start_object_status_rx_count_
               ? BT::NodeStatus::FAILURE
               : BT::NodeStatus::RUNNING;
}

void WaitForDetectionReadyNode::onHalted() {}

CheckObjectPresentNode::CheckObjectPresentNode(const std::string& name, const BT::NodeConfig& config)
    : BT::ConditionNode(name, config), ros_node_(GetROSNode(config)) {}

BT::PortsList CheckObjectPresentNode::providedPorts() {
    return {};
}

BT::NodeStatus CheckObjectPresentNode::tick() {
    return ros_node_->getObjectPresent() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

CheckGrabCountNode::CheckGrabCountNode(const std::string& name, const BT::NodeConfig& config)
    : BT::ConditionNode(name, config), ros_node_(GetROSNode(config)) {}

BT::PortsList CheckGrabCountNode::providedPorts() {
    return {};
}

BT::NodeStatus CheckGrabCountNode::tick() {
    int grab_count = 0;
    const bool got_grab_count = config().blackboard->get(bb_keys::kGrabCount, grab_count);
    (void)got_grab_count;
    return (grab_count < ros_node_->getMaxGrabCount()) ? BT::NodeStatus::SUCCESS
                                                        : BT::NodeStatus::FAILURE;
}

RecordPreGrabDepthNode::RecordPreGrabDepthNode(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config), ros_node_(GetROSNode(config)) {}

BT::PortsList RecordPreGrabDepthNode::providedPorts() {
    return {};
}

BT::NodeStatus RecordPreGrabDepthNode::tick() {
    config().blackboard->set(bb_keys::kPreGrabDepth, ros_node_->getCurrentDepth());
    return BT::NodeStatus::SUCCESS;
}

SendSerialCommandNode::SendSerialCommandNode(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config), ros_node_(GetROSNode(config)) {}

BT::PortsList SendSerialCommandNode::providedPorts() {
    return {BT::InputPort<std::string>("command")};
}

BT::NodeStatus SendSerialCommandNode::tick() {
    std::string cmd;
    if (!getInput("command", cmd)) {
        RCLCPP_ERROR(ros_node_->get_logger(), "Missing serial command input");
        return BT::NodeStatus::FAILURE;
    }
    ros_node_->sendSerial(cmd, false);
    return BT::NodeStatus::SUCCESS;
}

StopSerialCommandNode::StopSerialCommandNode(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config), ros_node_(GetROSNode(config)) {}

BT::PortsList StopSerialCommandNode::providedPorts() {
    return {BT::InputPort<std::string>("command")};
}

BT::NodeStatus StopSerialCommandNode::tick() {
    std::string cmd;
    getInput("command", cmd);
    ros_node_->stopSerial(cmd);
    return BT::NodeStatus::SUCCESS;
}

IncrementGrabCountNode::IncrementGrabCountNode(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config), ros_node_(GetROSNode(config)) {}

BT::PortsList IncrementGrabCountNode::providedPorts() {
    return {};
}

BT::NodeStatus IncrementGrabCountNode::tick() {
    int grab_count = 0;
    const bool got_grab_count = config().blackboard->get(bb_keys::kGrabCount, grab_count);
    (void)got_grab_count;
    config().blackboard->set(bb_keys::kGrabCount, grab_count + 1);
    return BT::NodeStatus::SUCCESS;
}

EvaluateGrabFeedbackNode::EvaluateGrabFeedbackNode(const std::string& name, const BT::NodeConfig& config)
    : BT::StatefulActionNode(name, config), ros_node_(GetROSNode(config)) {}

BT::PortsList EvaluateGrabFeedbackNode::providedPorts() {
    return {};
}

BT::NodeStatus EvaluateGrabFeedbackNode::onStart() {
    start_time_ = ros_node_->now();
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus EvaluateGrabFeedbackNode::onRunning() {
    if ((ros_node_->now() - start_time_).seconds() < ros_node_->getGrabCheckDelay()) {
        return BT::NodeStatus::RUNNING;
    }

    float pre_depth = 0.0f;
    const bool got_pre_depth = config().blackboard->get(bb_keys::kPreGrabDepth, pre_depth);
    (void)got_pre_depth;
    const float current_depth = ros_node_->getCurrentDepth();
    const bool object_present = ros_node_->getObjectPresent();
    const bool depth_similar =
        std::abs(current_depth - pre_depth) < static_cast<float>(ros_node_->getGrabDepthTolerance());
    const bool success = !(object_present && depth_similar);

    if (!success) {
        if (ros_node_->getActiveSerialCommand() == "1") {
            ros_node_->stopSerial("1");
        }
        ros_node_->sendSerial("5", false);
        int grab_count = 0;
        const bool got_grab_count = config().blackboard->get(bb_keys::kGrabCount, grab_count);
        (void)got_grab_count;
        config().blackboard->set(bb_keys::kGrabCount, std::max(0, grab_count - 1));
        RCLCPP_WARN(ros_node_->get_logger(), "Grab feedback indicates failure, skip current grab point");
    }

    config().blackboard->set(bb_keys::kGrabFeedbackSuccess, success);
    return BT::NodeStatus::SUCCESS;
}

void EvaluateGrabFeedbackNode::onHalted() {}

CheckGrabFeedbackSuccessNode::CheckGrabFeedbackSuccessNode(const std::string& name, const BT::NodeConfig& config)
    : BT::ConditionNode(name, config), ros_node_(GetROSNode(config)) {}

BT::PortsList CheckGrabFeedbackSuccessNode::providedPorts() {
    return {};
}

BT::NodeStatus CheckGrabFeedbackSuccessNode::tick() {
    bool success = false;
    const bool got_feedback = config().blackboard->get(bb_keys::kGrabFeedbackSuccess, success);
    (void)got_feedback;
    return success ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

SetChassisModeNode::SetChassisModeNode(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config), ros_node_(GetROSNode(config)) {}

BT::PortsList SetChassisModeNode::providedPorts() {
    return {BT::InputPort<int>("mode")};
}

BT::NodeStatus SetChassisModeNode::tick() {
    int mode = 0;
    if (!getInput("mode", mode)) {
        return BT::NodeStatus::FAILURE;
    }
    ros_node_->setChassisMode(static_cast<R2Mode1ROSNode::ChassisMode>(mode));
    return BT::NodeStatus::SUCCESS;
}

PublishExternalModeNode::PublishExternalModeNode(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config), ros_node_(GetROSNode(config)) {}

BT::PortsList PublishExternalModeNode::providedPorts() {
    return {BT::InputPort<int>("mode")};
}

BT::NodeStatus PublishExternalModeNode::tick() {
    int mode = 0;
    if (!getInput("mode", mode)) {
        return BT::NodeStatus::FAILURE;
    }
    ros_node_->publishExternalMode(mode);
    return BT::NodeStatus::SUCCESS;
}

WaitForSerialCommandCompleteNode::WaitForSerialCommandCompleteNode(const std::string& name,
                                                                   const BT::NodeConfig& config)
    : BT::StatefulActionNode(name, config), ros_node_(GetROSNode(config)) {}

BT::PortsList WaitForSerialCommandCompleteNode::providedPorts() {
    return {BT::InputPort<std::string>("command")};
}

BT::NodeStatus WaitForSerialCommandCompleteNode::onStart() {
    if (!getInput("command", command_)) {
        RCLCPP_ERROR(ros_node_->get_logger(), "Missing serial command input");
        return BT::NodeStatus::FAILURE;
    }
    return ros_node_->consumeSerialCommandCompleted(command_) ? BT::NodeStatus::SUCCESS
                                                              : BT::NodeStatus::RUNNING;
}

BT::NodeStatus WaitForSerialCommandCompleteNode::onRunning() {
    return ros_node_->consumeSerialCommandCompleted(command_) ? BT::NodeStatus::SUCCESS
                                                              : BT::NodeStatus::RUNNING;
}

void WaitForSerialCommandCompleteNode::onHalted() {}

CheckTFAlignedNode::CheckTFAlignedNode(const std::string& name, const BT::NodeConfig& config)
    : BT::ConditionNode(name, config), ros_node_(GetROSNode(config)) {}

BT::PortsList CheckTFAlignedNode::providedPorts() {
    return {};
}

BT::NodeStatus CheckTFAlignedNode::tick() {
    geometry_msgs::msg::TransformStamped transform;
    if (!ros_node_->lookupTransform(ros_node_->getReferenceFrame(), ros_node_->getTargetTagFrame(), transform)) {
        return BT::NodeStatus::FAILURE;
    }
    const double transform_age =
        (ros_node_->now() - rclcpp::Time(transform.header.stamp)).seconds();
    if (transform_age > 0.6) {
        return BT::NodeStatus::FAILURE;
    }
    const double tag_y = transform.transform.translation.y;
    const double align_error = tag_y - ros_node_->getAlignTargetY();
    config().blackboard->set(bb_keys::kTagY, tag_y);
    return (std::abs(align_error) < ros_node_->getAlignTolerance()) ? BT::NodeStatus::SUCCESS
                                                                    : BT::NodeStatus::FAILURE;
}

StartBarcodeMonitorNode::StartBarcodeMonitorNode(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config), ros_node_(GetROSNode(config)) {}

BT::PortsList StartBarcodeMonitorNode::providedPorts() {
    return {};
}

BT::NodeStatus StartBarcodeMonitorNode::tick() {
    ros_node_->clearLastBarcodeBuffer();
    ros_node_->setBarcodeWaitActive(true);
    return BT::NodeStatus::SUCCESS;
}

WaitForBarcodeWithTimeoutNode::WaitForBarcodeWithTimeoutNode(const std::string& name,
                                                             const BT::NodeConfig& config)
    : BT::StatefulActionNode(name, config), ros_node_(GetROSNode(config)) {}

BT::PortsList WaitForBarcodeWithTimeoutNode::providedPorts() {
    return {};
}

BT::NodeStatus WaitForBarcodeWithTimeoutNode::onStart() {
    start_time_ = ros_node_->now();
    ros_node_->setBarcodeWaitActive(true);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus WaitForBarcodeWithTimeoutNode::onRunning() {
    const auto barcode = ros_node_->getLastBarcodeBuffer();
    if (barcode == "100" || barcode == "200") {
        ros_node_->setBarcodeWaitActive(false);
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
}

void WaitForBarcodeWithTimeoutNode::onHalted() {
    ros_node_->setBarcodeWaitActive(false);
}

PostBarcodeDelayNode::PostBarcodeDelayNode(const std::string& name, const BT::NodeConfig& config)
    : BT::StatefulActionNode(name, config), ros_node_(GetROSNode(config)) {}

BT::PortsList PostBarcodeDelayNode::providedPorts() {
    return {};
}

BT::NodeStatus PostBarcodeDelayNode::onStart() {
    start_time_ = ros_node_->now();
    duration_ = ros_node_->getPostBarcodeShiftDelay();
    return duration_ <= 0.0 ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
}

BT::NodeStatus PostBarcodeDelayNode::onRunning() {
    return ((ros_node_->now() - start_time_).seconds() >= duration_) ? BT::NodeStatus::SUCCESS
                                                                      : BT::NodeStatus::RUNNING;
}

void PostBarcodeDelayNode::onHalted() {}

CheckBarcodeInBufferNode::CheckBarcodeInBufferNode(const std::string& name, const BT::NodeConfig& config)
    : BT::ConditionNode(name, config), ros_node_(GetROSNode(config)) {}

BT::PortsList CheckBarcodeInBufferNode::providedPorts() {
    return {BT::InputPort<std::string>("expected")};
}

BT::NodeStatus CheckBarcodeInBufferNode::tick() {
    std::string expected;
    if (!getInput("expected", expected)) {
        return BT::NodeStatus::FAILURE;
    }
    return ros_node_->getLastBarcodeBuffer() == expected ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

AdvanceToNextTargetNode::AdvanceToNextTargetNode(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config), ros_node_(GetROSNode(config)) {}

BT::PortsList AdvanceToNextTargetNode::providedPorts() {
    return {BT::InputPort<bool>("force_final"), BT::InputPort<bool>("success_completed")};
}

BT::NodeStatus AdvanceToNextTargetNode::tick() {
    bool force_final = false;
    bool success_completed = false;
    getInput("force_final", force_final);
    getInput("success_completed", success_completed);

    int point_index = 0;
    int grab_count = 0;
    const bool got_point_index = config().blackboard->get(bb_keys::kPointIndex, point_index);
    const bool got_grab_count = config().blackboard->get(bb_keys::kGrabCount, grab_count);
    (void)got_point_index;
    (void)got_grab_count;

    const int last_index = static_cast<int>(ros_node_->getGrabPoints().size()) - 1;
    const bool should_finish_after_success =
        success_completed && ros_node_->getFinishAfterFirstSuccess();
    const bool should_go_final =
        force_final ||
        should_finish_after_success ||
        point_index >= last_index ||
        grab_count >= ros_node_->getMaxGrabCount();

    if (should_go_final) {
        config().blackboard->set(bb_keys::kGoFinal, true);
    } else {
        config().blackboard->set(bb_keys::kPointIndex, point_index + 1);
        config().blackboard->set(bb_keys::kGoFinal, false);
    }
    config().blackboard->set(bb_keys::kForceFinal, false);
    return BT::NodeStatus::SUCCESS;
}

CheckFinalRequestedNode::CheckFinalRequestedNode(const std::string& name, const BT::NodeConfig& config)
    : BT::ConditionNode(name, config), ros_node_(GetROSNode(config)) {}

BT::PortsList CheckFinalRequestedNode::providedPorts() {
    return {};
}

BT::NodeStatus CheckFinalRequestedNode::tick() {
    bool go_final = false;
    const bool got_go_final = config().blackboard->get(bb_keys::kGoFinal, go_final);
    (void)got_go_final;
    return go_final ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

FinalExitNode::FinalExitNode(const std::string& name, const BT::NodeConfig& config)
    : BT::StatefulActionNode(name, config), ros_node_(GetROSNode(config)) {}

BT::PortsList FinalExitNode::providedPorts() {
    return {};
}

BT::NodeStatus FinalExitNode::onStart() {
    ros_node_->requestFinalExit();
    start_time_ = ros_node_->now();
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus FinalExitNode::onRunning() {
    if ((ros_node_->now() - start_time_).seconds() < 2.0) {
        return BT::NodeStatus::RUNNING;
    }
    ros_node_->clearPendingFinalExit();
    config().blackboard->set(bb_keys::kNodeRunning, false);
    return BT::NodeStatus::SUCCESS;
}

void FinalExitNode::onHalted() {}

LogMessageNode::LogMessageNode(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config), ros_node_(GetROSNode(config)) {}

BT::PortsList LogMessageNode::providedPorts() {
    return {BT::InputPort<std::string>("message"), BT::InputPort<std::string>("level")};
}

BT::NodeStatus LogMessageNode::tick() {
    std::string message;
    std::string level = "info";
    getInput("message", message);
    getInput("level", level);

    if (level == "warn") {
        RCLCPP_WARN(ros_node_->get_logger(), "%s", message.c_str());
    } else if (level == "error") {
        RCLCPP_ERROR(ros_node_->get_logger(), "%s", message.c_str());
    } else {
        RCLCPP_INFO(ros_node_->get_logger(), "%s", message.c_str());
    }
    return BT::NodeStatus::SUCCESS;
}

void RegisterBehaviorTreeNodes(BT::BehaviorTreeFactory& factory,
                               std::shared_ptr<R2Mode1ROSNode> ros_node) {
    g_ros_node = std::move(ros_node);

    factory.registerNodeType<WaitForNavReadyNode>("WaitForNavReady");
    factory.registerNodeType<PublishGrabGoalNode>("PublishGrabGoal");
    factory.registerNodeType<PublishShiftGoalNode>("PublishShiftGoal");
    factory.registerNodeType<PublishShiftReturnGoalNode>("PublishShiftReturnGoal");
    factory.registerNodeType<PublishFinalGoalNode>("PublishFinalGoal");
    factory.registerNodeType<WaitForNavArrivalNode>("WaitForNavArrival");
    factory.registerNodeType<StabilizationWaitNode>("StabilizationWait");
    factory.registerNodeType<WaitForDetectionReadyNode>("WaitForDetectionReady");
    factory.registerNodeType<CheckObjectPresentNode>("CheckObjectPresent");
    factory.registerNodeType<CheckGrabCountNode>("CheckGrabCount");
    factory.registerNodeType<RecordPreGrabDepthNode>("RecordPreGrabDepth");
    factory.registerNodeType<SendSerialCommandNode>("SendSerialCommand");
    factory.registerNodeType<StopSerialCommandNode>("StopSerialCommand");
    factory.registerNodeType<IncrementGrabCountNode>("IncrementGrabCount");
    factory.registerNodeType<EvaluateGrabFeedbackNode>("EvaluateGrabFeedback");
    factory.registerNodeType<CheckGrabFeedbackSuccessNode>("CheckGrabFeedbackSuccess");
    factory.registerNodeType<SetChassisModeNode>("SetChassisMode");
    factory.registerNodeType<PublishExternalModeNode>("PublishExternalMode");
    factory.registerNodeType<WaitForSerialCommandCompleteNode>("WaitForSerialCommandComplete");
    factory.registerNodeType<CheckTFAlignedNode>("CheckTFAligned");
    factory.registerNodeType<StartBarcodeMonitorNode>("StartBarcodeMonitor");
    factory.registerNodeType<WaitForBarcodeWithTimeoutNode>("WaitForBarcodeWithTimeout");
    factory.registerNodeType<PostBarcodeDelayNode>("PostBarcodeDelay");
    factory.registerNodeType<CheckBarcodeInBufferNode>("CheckBarcodeInBuffer");
    factory.registerNodeType<AdvanceToNextTargetNode>("AdvanceToNextTarget");
    factory.registerNodeType<CheckFinalRequestedNode>("CheckFinalRequested");
    factory.registerNodeType<FinalExitNode>("FinalExit");
    factory.registerNodeType<LogMessageNode>("LogMessage");
}

}  // namespace r2_mode1_bt
