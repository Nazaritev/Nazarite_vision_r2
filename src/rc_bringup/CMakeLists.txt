cmake_minimum_required(VERSION 3.10)
project(rc_bringup)

if(CMAKE_EXPORT_COMPILE_COMMANDS)
  set(CMAKE_EXPORT_COMPILE_COMMANDS ${CMAKE_EXPORT_COMPILE_COMMANDS})
endif()

find_package(ament_cmake_auto REQUIRED)

install(PROGRAMS
  scripts/amcl_pose_counter.py
  scripts/goal_distance_stop.py
  scripts/wait_for_localization.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  behavior_trees
  config
  launch
  map
  urdf
  meshes
  rviz
  world
)
