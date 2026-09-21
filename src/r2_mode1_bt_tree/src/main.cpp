#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/xml_parsing.h>

#include "r2_mode1_bt/ros_node.hpp"
#include "r2_mode1_bt/bt_nodes.hpp"
#include "r2_mode1_bt/blackboard_keys.hpp"

#include <chrono>
#include <filesystem>
#include <thread>

using namespace std::chrono_literals;

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    // Create ROS node
    auto ros_node = std::make_shared<r2_mode1_bt::R2Mode1ROSNode>();

    // Create Behavior Tree factory
    BT::BehaviorTreeFactory factory;

    // Register custom nodes
    r2_mode1_bt::RegisterBehaviorTreeNodes(factory, ros_node);

    // Load behavior tree from XML
    std::string xml_path;
    ros_node->declare_parameter("bt_xml_path", "config/r2_mode1_bt.xml");
    ros_node->get_parameter("bt_xml_path", xml_path);

    std::filesystem::path resolved_xml_path(xml_path);
    if (!resolved_xml_path.is_absolute() && !std::filesystem::exists(resolved_xml_path)) {
        resolved_xml_path =
            std::filesystem::path(ament_index_cpp::get_package_share_directory("r2_mode1_bt")) / xml_path;
    }

    std::string tree_name;
    ros_node->declare_parameter("bt_tree_name", "MainTree");
    ros_node->get_parameter("bt_tree_name", tree_name);

    try {
        factory.registerBehaviorTreeFromFile(resolved_xml_path.string());
        RCLCPP_INFO(ros_node->get_logger(), "Behavior tree loaded from: %s", resolved_xml_path.c_str());
    } catch (const std::exception& e) {
        RCLCPP_ERROR(ros_node->get_logger(), "Failed to load behavior tree: %s", e.what());
        ros_node.reset();
        rclcpp::shutdown();
        return 1;
    }

    // Create blackboard and initialize values
    auto blackboard = BT::Blackboard::create();
    blackboard->set<std::shared_ptr<r2_mode1_bt::R2Mode1ROSNode>>("ros_node", ros_node);
    const int start_point_index = ros_node->resolveStartGrabIndex();
    blackboard->set<int>(r2_mode1_bt::bb_keys::kPointIndex, start_point_index);
    blackboard->set<int>(r2_mode1_bt::bb_keys::kGrabCount, 0);
    blackboard->set<int>(r2_mode1_bt::bb_keys::kLastArrivedIndex, -1);
    blackboard->set<bool>(r2_mode1_bt::bb_keys::kObjectPresent, false);
    blackboard->set<float>(r2_mode1_bt::bb_keys::kCurrentDepth, 0.0f);
    blackboard->set<float>(r2_mode1_bt::bb_keys::kPreGrabDepth, 0.0f);
    blackboard->set<double>(r2_mode1_bt::bb_keys::kTagY, 0.0);
    blackboard->set<bool>(r2_mode1_bt::bb_keys::kAligned, false);
    blackboard->set<std::string>(r2_mode1_bt::bb_keys::kBarcodeData, "");
    blackboard->set<std::string>(r2_mode1_bt::bb_keys::kCurrentNavTarget, "");
    blackboard->set<bool>(r2_mode1_bt::bb_keys::kSpecialBarcodeTriggered, false);
    blackboard->set<bool>(r2_mode1_bt::bb_keys::kCheckingGrabFeedback, false);
    blackboard->set<double>(r2_mode1_bt::bb_keys::kGrabStartTime, 0.0);
    blackboard->set<bool>(r2_mode1_bt::bb_keys::kGrabSuccess, false);
    blackboard->set<bool>(r2_mode1_bt::bb_keys::kGrabFeedbackSuccess, false);
    blackboard->set<bool>(r2_mode1_bt::bb_keys::kGoFinal, false);
    blackboard->set<bool>(r2_mode1_bt::bb_keys::kForceFinal, false);
    blackboard->set<bool>(r2_mode1_bt::bb_keys::kNodeRunning, true);
    blackboard->set<std::string>(r2_mode1_bt::bb_keys::kCurrentState, "INIT");

    // Create behavior tree
    auto tree = factory.createTree(tree_name, blackboard);

    RCLCPP_INFO(ros_node->get_logger(), "Starting Behavior Tree execution: %s", tree_name.c_str());
    RCLCPP_INFO(ros_node->get_logger(), "Initial grab point index: %d", start_point_index);

    // Main execution loop
    BT::NodeStatus status = BT::NodeStatus::RUNNING;
    bool node_running = true;

    // Create multithreaded executor for ROS
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(ros_node);

    // Run ROS spinner in separate thread
    std::thread ros_thread([&executor]() {
        executor.spin();
    });

    // Behavior tree execution loop
    while (rclcpp::ok() && status == BT::NodeStatus::RUNNING && node_running) {
        // Check if we should stop
        const bool got_node_running =
            blackboard->get(r2_mode1_bt::bb_keys::kNodeRunning, node_running);
        (void)got_node_running;
        
        // Tick the tree
        status = tree.tickOnce();

        // Sleep to avoid busy waiting
        std::this_thread::sleep_for(50ms);
    }

    if (!node_running && status == BT::NodeStatus::RUNNING) {
        status = BT::NodeStatus::SUCCESS;
    }

    // Handle final status
    if (status == BT::NodeStatus::SUCCESS) {
        RCLCPP_INFO(ros_node->get_logger(), "Behavior Tree completed successfully");
    } else if (status == BT::NodeStatus::FAILURE) {
        RCLCPP_ERROR(ros_node->get_logger(), "Behavior Tree failed");
    } else {
        RCLCPP_WARN(ros_node->get_logger(), "Behavior Tree interrupted");
    }

    // Cleanup
    executor.cancel();
    if (ros_thread.joinable()) {
        ros_thread.join();
    }

    rclcpp::shutdown();

    return (status == BT::NodeStatus::SUCCESS) ? 0 : 1;
}
