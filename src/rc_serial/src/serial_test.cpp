#include "rclcpp/rclcpp.hpp"
#include "rc_serial/rc_usb.h"
#include <string>
#include <iostream>

DOGUART dog_usb;

int main(int argc, char** argv){
    // ROS 2 初始化
    rclcpp::init(argc, argv);
    // 创建节点对象
    auto node = std::make_shared<rclcpp::Node>("serial_test_node");
    // 设置循环频率 (10Hz)
    rclcpp::Rate rate(10);

    // 等待串口连接
    while (!dog_usb.serial_connected && rclcpp::ok())
    {
        dog_usb.scan_usb(); 
    }

    while (rclcpp::ok()){
        const std::string id1 = "pci-0000:00:14.0-usb-0:3.1:1.0";
        const std::string id2 = "pci-0000:00:14.0-usb-0:3.2:1.0";
        std::string data = "hello world";
        float data_[3] = {1.0, 2.0, 3.0};

        if (dog_usb.send_usb_string(id1, data)) {
            std::cout << "发送字符串: " << data << std::endl;
        } else {
            // 使用 ROS 2 的日志系统
            RCLCPP_ERROR(node->get_logger( ), "USB send failed!");
        }

        if (dog_usb.send_usb(id2, data_, 3)) {
            std::cout << "发送数组: " << data_[0] << ", " << data_[1] << ", " << data_[2] << std::endl;
        } else {
            RCLCPP_ERROR(node->get_logger(), "USB send failed!");
        }

        std::cout << "serial_connected: " << dog_usb.serial_connected << std::endl;
        
        // 处理 ROS 2 的回调，替代 ros::spinOnce()
        rclcpp::spin_some(node);
        rate.sleep();
    }

    // 退出前关闭 ROS 2
    rclcpp::shutdown();
    return 0;
}