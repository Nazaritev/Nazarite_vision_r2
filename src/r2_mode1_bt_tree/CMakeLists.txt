cmake_minimum_required(VERSION 3.8)
project(r2_mode1_bt)

# C++17 required for behaviortree_cpp
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(serial REQUIRED)
find_package(behaviortree_cpp REQUIRED)

# Include directories
include_directories(
    include
)

# Create library for ROS node
add_library(ros_node_lib
    src/ros_node.cpp
)

target_link_libraries(ros_node_lib
    ${serial_LIBRARIES}
)

ament_target_dependencies(ros_node_lib
    nav_msgs
    rclcpp
    std_msgs
    geometry_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
)

# Create library for BT nodes
add_library(bt_nodes_lib
    src/bt_nodes.cpp
)

target_link_libraries(bt_nodes_lib
    ros_node_lib
    ${behaviortree_cpp_LIBRARIES}
)

ament_target_dependencies(bt_nodes_lib
    rclcpp
    behaviortree_cpp
)

# Create executable
add_executable(r2_mode1_bt_node
    src/main.cpp
)

add_executable(serial_region1_mode0_test
    src/serial_region1_mode0_test.cpp
)

target_link_libraries(r2_mode1_bt_node
    ros_node_lib
    bt_nodes_lib
    ${behaviortree_cpp_LIBRARIES}
)

target_link_libraries(serial_region1_mode0_test
    ${serial_LIBRARIES}
)

ament_target_dependencies(r2_mode1_bt_node
    ament_index_cpp
    rclcpp
    behaviortree_cpp
)

# Install targets
install(TARGETS
    ros_node_lib
    bt_nodes_lib
    r2_mode1_bt_node
    serial_region1_mode0_test
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY include/
    DESTINATION include
)

# Install config files
install(DIRECTORY config/
    DESTINATION share/${PROJECT_NAME}/config
)

# Install launch files (if any)
install(DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch
    OPTIONAL
)

ament_package()
