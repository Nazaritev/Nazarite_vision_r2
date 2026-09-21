#ifndef R2_MODE1_BT_BT_NODES_HPP_
#define R2_MODE1_BT_BT_NODES_HPP_

#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>

#include <cstdint>
#include <memory>

#include "r2_mode1_bt/ros_node.hpp"

namespace r2_mode1_bt {

class WaitForNavReadyNode : public BT::StatefulActionNode {
public:
    WaitForNavReadyNode(const std::string& name, const BT::NodeConfig& config);
    static BT::PortsList providedPorts();
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    std::shared_ptr<R2Mode1ROSNode> ros_node_;
};

class PublishGrabGoalNode : public BT::SyncActionNode {
public:
    PublishGrabGoalNode(const std::string& name, const BT::NodeConfig& config);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;

private:
    std::shared_ptr<R2Mode1ROSNode> ros_node_;
};

class PublishShiftGoalNode : public BT::SyncActionNode {
public:
    PublishShiftGoalNode(const std::string& name, const BT::NodeConfig& config);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;

private:
    std::shared_ptr<R2Mode1ROSNode> ros_node_;
};

class PublishShiftReturnGoalNode : public BT::SyncActionNode {
public:
    PublishShiftReturnGoalNode(const std::string& name, const BT::NodeConfig& config);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;

private:
    std::shared_ptr<R2Mode1ROSNode> ros_node_;
};

class PublishFinalGoalNode : public BT::SyncActionNode {
public:
    PublishFinalGoalNode(const std::string& name, const BT::NodeConfig& config);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;

private:
    std::shared_ptr<R2Mode1ROSNode> ros_node_;
};

class WaitForNavArrivalNode : public BT::StatefulActionNode {
public:
    WaitForNavArrivalNode(const std::string& name, const BT::NodeConfig& config);
    static BT::PortsList providedPorts();
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    std::shared_ptr<R2Mode1ROSNode> ros_node_;
};

class StabilizationWaitNode : public BT::StatefulActionNode {
public:
    StabilizationWaitNode(const std::string& name, const BT::NodeConfig& config);
    static BT::PortsList providedPorts();
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    std::shared_ptr<R2Mode1ROSNode> ros_node_;
    rclcpp::Time start_time_;
    double duration_{0.0};
};

class WaitForDetectionReadyNode : public BT::StatefulActionNode {
public:
    WaitForDetectionReadyNode(const std::string& name, const BT::NodeConfig& config);
    static BT::PortsList providedPorts();
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    std::shared_ptr<R2Mode1ROSNode> ros_node_;
    std::uint64_t start_object_status_rx_count_{0};
};

class CheckObjectPresentNode : public BT::ConditionNode {
public:
    CheckObjectPresentNode(const std::string& name, const BT::NodeConfig& config);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;

private:
    std::shared_ptr<R2Mode1ROSNode> ros_node_;
};

class CheckGrabCountNode : public BT::ConditionNode {
public:
    CheckGrabCountNode(const std::string& name, const BT::NodeConfig& config);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;

private:
    std::shared_ptr<R2Mode1ROSNode> ros_node_;
};

class RecordPreGrabDepthNode : public BT::SyncActionNode {
public:
    RecordPreGrabDepthNode(const std::string& name, const BT::NodeConfig& config);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;

private:
    std::shared_ptr<R2Mode1ROSNode> ros_node_;
};

class SendSerialCommandNode : public BT::SyncActionNode {
public:
    SendSerialCommandNode(const std::string& name, const BT::NodeConfig& config);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;

private:
    std::shared_ptr<R2Mode1ROSNode> ros_node_;
};

class StopSerialCommandNode : public BT::SyncActionNode {
public:
    StopSerialCommandNode(const std::string& name, const BT::NodeConfig& config);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;

private:
    std::shared_ptr<R2Mode1ROSNode> ros_node_;
};

class IncrementGrabCountNode : public BT::SyncActionNode {
public:
    IncrementGrabCountNode(const std::string& name, const BT::NodeConfig& config);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;

private:
    std::shared_ptr<R2Mode1ROSNode> ros_node_;
};

class EvaluateGrabFeedbackNode : public BT::StatefulActionNode {
public:
    EvaluateGrabFeedbackNode(const std::string& name, const BT::NodeConfig& config);
    static BT::PortsList providedPorts();
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    std::shared_ptr<R2Mode1ROSNode> ros_node_;
    rclcpp::Time start_time_;
};

class CheckGrabFeedbackSuccessNode : public BT::ConditionNode {
public:
    CheckGrabFeedbackSuccessNode(const std::string& name, const BT::NodeConfig& config);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;

private:
    std::shared_ptr<R2Mode1ROSNode> ros_node_;
};

class SetChassisModeNode : public BT::SyncActionNode {
public:
    SetChassisModeNode(const std::string& name, const BT::NodeConfig& config);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;

private:
    std::shared_ptr<R2Mode1ROSNode> ros_node_;
};

class PublishExternalModeNode : public BT::SyncActionNode {
public:
    PublishExternalModeNode(const std::string& name, const BT::NodeConfig& config);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;

private:
    std::shared_ptr<R2Mode1ROSNode> ros_node_;
};

class WaitForSerialCommandCompleteNode : public BT::StatefulActionNode {
public:
    WaitForSerialCommandCompleteNode(const std::string& name, const BT::NodeConfig& config);
    static BT::PortsList providedPorts();
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    std::shared_ptr<R2Mode1ROSNode> ros_node_;
    std::string command_;
};

class CheckTFAlignedNode : public BT::ConditionNode {
public:
    CheckTFAlignedNode(const std::string& name, const BT::NodeConfig& config);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;

private:
    std::shared_ptr<R2Mode1ROSNode> ros_node_;
};

class StartBarcodeMonitorNode : public BT::SyncActionNode {
public:
    StartBarcodeMonitorNode(const std::string& name, const BT::NodeConfig& config);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;

private:
    std::shared_ptr<R2Mode1ROSNode> ros_node_;
};

class WaitForBarcodeWithTimeoutNode : public BT::StatefulActionNode {
public:
    WaitForBarcodeWithTimeoutNode(const std::string& name, const BT::NodeConfig& config);
    static BT::PortsList providedPorts();
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    std::shared_ptr<R2Mode1ROSNode> ros_node_;
    rclcpp::Time start_time_;
};

class PostBarcodeDelayNode : public BT::StatefulActionNode {
public:
    PostBarcodeDelayNode(const std::string& name, const BT::NodeConfig& config);
    static BT::PortsList providedPorts();
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    std::shared_ptr<R2Mode1ROSNode> ros_node_;
    rclcpp::Time start_time_;
    double duration_{0.0};
};

class CheckBarcodeInBufferNode : public BT::ConditionNode {
public:
    CheckBarcodeInBufferNode(const std::string& name, const BT::NodeConfig& config);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;

private:
    std::shared_ptr<R2Mode1ROSNode> ros_node_;
};

class AdvanceToNextTargetNode : public BT::SyncActionNode {
public:
    AdvanceToNextTargetNode(const std::string& name, const BT::NodeConfig& config);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;

private:
    std::shared_ptr<R2Mode1ROSNode> ros_node_;
};

class CheckFinalRequestedNode : public BT::ConditionNode {
public:
    CheckFinalRequestedNode(const std::string& name, const BT::NodeConfig& config);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;

private:
    std::shared_ptr<R2Mode1ROSNode> ros_node_;
};

class FinalExitNode : public BT::StatefulActionNode {
public:
    FinalExitNode(const std::string& name, const BT::NodeConfig& config);
    static BT::PortsList providedPorts();
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    std::shared_ptr<R2Mode1ROSNode> ros_node_;
    rclcpp::Time start_time_;
};

class LogMessageNode : public BT::SyncActionNode {
public:
    LogMessageNode(const std::string& name, const BT::NodeConfig& config);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;

private:
    std::shared_ptr<R2Mode1ROSNode> ros_node_;
};

void RegisterBehaviorTreeNodes(BT::BehaviorTreeFactory& factory,
                               std::shared_ptr<R2Mode1ROSNode> ros_node);

}  // namespace r2_mode1_bt

#endif  // R2_MODE1_BT_BT_NODES_HPP_
