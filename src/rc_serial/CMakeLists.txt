cmake_minimum_required(VERSION 3.8)
project(rc_serial)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找 ROS 2 依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# 查找 Boost 和 udev 依赖
find_package(Boost REQUIRED COMPONENTS system filesystem)
find_library(UDEV_LIBRARIES udev REQUIRED)

# 包含目录
include_directories(
  include
  ${Boost_INCLUDE_DIRS}
)

# 编译 serial_test 节点
add_executable(serial_test src/serial_test.cpp src/UsbConnect.cpp)
target_include_directories(serial_test PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
# 链接系统库和 ROS 2 依赖
target_link_libraries(serial_test ${Boost_LIBRARIES} ${UDEV_LIBRARIES} pthread)
ament_target_dependencies(serial_test rclcpp std_msgs)

# （可选）预留原有的 odom_to_serial_node 编译规则
# add_executable(odom_to_serial_node src/odom_to_serial_node.cpp src/UsbConnect.cpp)
# target_link_libraries(odom_to_serial_node ${Boost_LIBRARIES} ${UDEV_LIBRARIES} pthread)
# ament_target_dependencies(odom_to_serial_node rclcpp std_msgs nav_msgs tf2_msgs)
# install(TARGETS odom_to_serial_node DESTINATION lib/${PROJECT_NAME})

# --------------------------------------------------------
# ROS 2 安装规则 (Install)
# --------------------------------------------------------
install(TARGETS
  serial_test
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()