cmake_minimum_required(VERSION 3.8)
project(arm-new23)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找 ROS 2 依赖包
find_package(ament_cmake REQUIRED)

# 安装目录 (将网格、模型、配置文件和启动文件安装到 install 目录)
install(
  DIRECTORY config launch meshes urdf rviz
  DESTINATION share/${PROJECT_NAME}
)

# 声明为 ament 包
ament_package()
