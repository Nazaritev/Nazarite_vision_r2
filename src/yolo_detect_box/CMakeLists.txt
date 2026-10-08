cmake_minimum_required(VERSION 3.8)
project(yolo_detect_box)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 生成同一功能包内的服务接口，供 Python 节点和客户端导入。
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/CameraUse.srv"
)
ament_export_dependencies(rosidl_default_runtime)

install(PROGRAMS scripts/yolo_detect_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME yolo_detect_node
)
# 模型随 ROS 包安装，换电脑时无需修改绝对路径。
install(DIRECTORY launch model
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)
install(FILES README.md requirements.txt
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
