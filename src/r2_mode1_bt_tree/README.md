# R2 Mode 1 Behavior Tree (C++)

基于 `behaviortree_cpp` 的 R2 机器人模式1 C++ 行为树实现，替代原有的 Python 状态机代码。

## 项目结构

```
r2_mode1_bt/
├── include/r2_mode1_bt/
│   ├── blackboard_keys.hpp    # 行为树黑板键名定义
│   ├── point3d.hpp            # 3D点数据结构
│   ├── ros_node.hpp           # ROS2 节点封装
│   └── bt_nodes.hpp           # 行为树节点声明
├── src/
│   ├── ros_node.cpp           # ROS2 节点实现
│   ├── bt_nodes.cpp           # 行为树节点实现
│   └── main.cpp               # 主程序入口
├── config/
│   └── r2_mode1_bt.xml        # 行为树 XML 配置
├── CMakeLists.txt
└── package.xml
```

## 依赖安装

```bash
# ROS2 Humble/Iron/Jazzy
sudo apt update
sudo apt install ros-$ROS_DISTRO-behaviortree-cpp-v3
sudo apt install ros-$ROS_DISTRO-serial
```

## 编译

```bash
cd ~/ros2_ws/src
git clone <repository_url>
cd ~/ros2_ws
colcon build --packages-select r2_mode1_bt
```

## 运行

```bash
# Source workspace
source ~/ros2_ws/install/setup.bash

# Run with default tree
ros2 run r2_mode1_bt r2_mode1_bt_node

# Run with custom parameters
ros2 run r2_mode1_bt r2_mode1_bt_node --ros-args \
    -p bt_xml_path:=/path/to/your/tree.xml \
    -p bt_tree_name:=MainTree \
    -p serial_port:=/dev/ttyUSB0 \
    -p baudrate:=115200
```

## 行为树结构

### MainTree

```
Sequence
├── LogMessage (初始化)
├── Repeat (4次循环)
│   └── Sequence
│       ├── CheckSpecialBarcode (检查特殊二维码)
│       ├── HandleSpecialBarcode (处理特殊二维码)
│       ├── PublishGoal (发布目标点)
│       ├── WaitForNavArrival (等待到达)
│       └── Selector
│           ├── Sequence (点3: 最终退出)
│           │   ├── CheckPointIndex (index=3)
│           │   └── FinalExit
│           └── Sequence (点0-2: 正常流程)
│               ├── StabilizationWait (稳定等待)
│               └── Selector
│                   ├── Sequence (检测到物体)
│                   │   ├── CheckObjectPresent
│                   │   ├── CheckGrabCount
│                   │   ├── RecordPreGrabDepth
│                   │   ├── SendSerialCommand (1)
│                   │   ├── IncrementGrabCount
│                   │   ├── StartGrabFeedbackCheck
│                   │   ├── Retry (TF对位)
│                   │   ├── SendSerialCommand (2)
│                   │   ├── Retry (等待二维码)
│                   │   ├── SendSerialCommand (3)
│                   │   ├── CheckGrabFeedback
│                   │   └── IncrementPointIndex
│                   └── Sequence (未检测到)
│                       ├── LogMessage
│                       └── IncrementPointIndex
└── LogMessage (完成)
```

## 自定义节点说明

| 节点名 | 类型 | 功能 |
|--------|------|------|
| PublishGoal | Action | 发布导航目标点 |
| WaitForNavArrival | Condition | 检查导航是否到达 |
| StabilizationWait | Action | 稳定等待 |
| CheckObjectPresent | Condition | 检查是否检测到物体 |
| CheckGrabCount | Condition | 检查抓取次数 |
| RecordPreGrabDepth | Action | 记录抓取前深度 |
| SendSerialCommand | Action | 发送串口命令 |
| IncrementGrabCount | Action | 增加抓取计数 |
| CheckTFAligned | Condition | 检查TF对位 |
| CheckBarcode | Condition | 检查二维码 |
| CheckSpecialBarcode | Condition | 检查特殊二维码(200) |
| HandleSpecialBarcode | Action | 处理特殊二维码 |
| StartGrabFeedbackCheck | Action | 开始抓取反馈检测 |
| CheckGrabFeedback | Action | 检查抓取反馈结果 |
| IncrementPointIndex | Action | 增加点索引 |
| SetPointIndex | Action | 设置点索引 |
| CheckPointIndex | Condition | 检查当前点索引 |
| FinalExit | Action | 最终退出流程 |
| LogMessage | Action | 日志输出 |

## 与原 Python 代码对比

| 特性 | Python 状态机 | C++ 行为树 |
|------|--------------|-----------|
| 可读性 | 状态跳转分散 | 树状结构直观 |
| 调试 | 日志追踪 | Groot 可视化 |
| 扩展性 | 需修改多处 | 添加节点即可 |
| 中断处理 | 手动实现 | Selector 自动处理 |
| 性能 | Python 解释器 | C++ 原生性能 |

## 配置参数

```yaml
r2_mode1_bt_node:
  ros__parameters:
    # Behavior Tree
    bt_xml_path: "config/r2_mode1_bt.xml"
    bt_tree_name: "MainTree"
    
    # Serial
    serial_port: "/dev/ttyV0"
    baudrate: 115200
    timeout: 1.0
    
    # TF
    target_tag_frame: "base"
    reference_frame: "camera_link"
    align_tolerance: 0.01
    
    # Timing
    stabilization_time: 0.5
    grab_check_delay: 3.0
    grab_depth_tolerance: 0.05
```

## 可视化调试 (Groot)

```bash
# 安装 Groot
sudo apt install ros-$ROS_DISTRO-groot

# 启动 Groot 监控
ros2 run groot Groot --mode=monitor
```

## 许可证

MIT
