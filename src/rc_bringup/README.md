# rc_bringup：BR 夹爪点云自过滤

## 已核对的工程链路

当前环境为 ROS 2 Humble、Nav2 1.1.20。检查了入口 launch、FAST-LIO 发布源码、
投影源码、参数文件、运行时话题及 TF。检查时 `/cloud_registered` 的类型为
`sensor_msgs/msg/PointCloud2`，frame 为 `odom`，带非零时间戳；过滤器仍逐帧读取
真实 header，不把这个运行观测硬编码为坐标系假设。

```text
livox_ros_driver2 (xfer_format=1) → /livox/lidar → fastlio_mapping
  → /cloud_registered
       ├─ 原投影 → /scan → AMCL
       ├─ block_stacking_detection
       ├─ pointcloud_to_grid_node
       └─ gripper_pointcloud_filter → /cloud_registered_costmap
            → pointcloud_to_laserscan_costmap → /scan_costmap
```

关闭过滤时，本地/全局 costmap 仍使用原 `/scan` marking + clearing。
开启过滤时，`/scan_costmap` 仅 marking，原 `/scan` 仅 clearing 且不接受无限远回波。
原始点云、原 `/scan`、定位、建图及堆叠检测的输入保持原样。修改前运行图中，
原点云的订阅者是原投影、堆叠检测、点云栅格；`/scan` 由 AMCL 和两个 costmap 订阅。

Nav2 参数按 `nav2_params_forest_small_teb_mppi.yaml`、`mapping_nav_copy.yaml`
顺序加载；后一个文件只有 AMCL 参数，不覆盖 costmap。launch 在前一个文件的
costmap 节点路径下重写 observation sources 和 marking/clearing 开关。
该文件本次仅改 costmap 部分，不修改 TEB 配置。

launch 虽然向 `pointcloud_to_grid` 传递 `ego_filter_*`，当前节点源码没有读取并
执行这些参数，不能作为现成过滤器复用。现有用户在该子模块和其他文件中的修改保留。

## 配置与 TF

编辑 `config/reality/gripper_filter.yaml`。入口开关 `enable_gripper_filter` 默认
`false`，`box_count` 默认 0；未实测前不启用。若开启但没有配置盒子，节点报错并
发布空云，不将原云伪装成已过滤数据。修改参数后重新启动本 launch；不提供运行中
自动重配 costmap 数据源的功能。

- `box_count`：夹爪及固定车体部件的盒子总数。
- `box_frames`：每个盒子的 TF frame，顺序对应盒子编号。
- `box_min`、`box_max`：按盒子顺序展开的 xyz 三元组，单位米，边界包含在盒内。
- `tf_timeout`：按输入点云时间戳等待 TF 的超时秒数；不回退到 latest TF。
- `block_box_*`：未来被夹持方块的盒子；当前数量为 0、过滤关闭。
- 投影节点部分：目标 frame、高度、角度、量程和时间等投影参数。

盒子数组长度、非有限尺寸、空 frame 或负数量等配置错误会阻止节点启动；不会
静默忽略某个配置错误的夹爪盒子。点云缺少 frame、零时间戳、布局异常、TF 缺失或
非法 TF 时输出空云，并在日志及 `/gripper_filter/status` 报错。
使用两个执行器工作线程，以便在点云回调等待 TF 时继续接收 TF。
过滤结果保留点云原始 frame、stamp、字段、字节序及点记录填充；只删除匹配记录。

入口提供 `filter_input_topic`、`filtered_cloud_topic`、`costmap_scan_topic`
三个 launch 参数，以同时接线过滤器、独立投影及 Nav2 marking 源；默认话题见图。
单独运行节点时也可通过 YAML 的 `input_topic`、`output_topic` 配置点云话题；
入口启动时这两项由 launch 参数覆盖。

当前 TF 是 FAST-LIO 的 `odom → base_link` 及 URDF 的固定
`base_link → footprint_link`。现有 `model.xacro` 没有可用夹爪关节、夹爪 link
或关节反馈。现在只能在实测确认的底盘 frame（例如 `footprint_link`）下配置
对应固定姿态的夹爪盒子；以后发布可靠的夹爪 TF 后可直接改用夹爪 frame。
配置中所有占位尺寸明确标注“待实测”，不能代表真实车体或夹爪尺寸、朝向。

测量时先在 RViz 确认选定 frame 的原点和三根坐标轴，再将夹爪固定到导航姿态，
分别测量各部件在该 frame 下的 x/y/z 最小及最大值。使用多个紧贴部件的盒子，
按安装与 TF 误差留有限余量，并用盒子旁的真实障碍点验证余量没有吞掉环境。
如果夹爪会运动且没有反馈 TF，固定盒子只适用于测量时的姿态。

## 遮挡与 footprint 限制

现有投影在 `use_inf=true` 时把没有点的角度置为 +Inf；Nav2 在
`inf_is_valid=true` 时可把它转换为最大量程附近的清除观测。因此新的过滤 scan
永远不用于 clearing；clearing 只读取原 scan，设置 `inf_is_valid=false`。
夹爪有限回波只能清到该回波，空扇区不会新增一条最大量程清除射线。

这解决“删除夹爪点产生无限远清除射线”的问题，但不重建夹爪后面的环境，
也没有建立精确的三维角度遮挡体。原投影的高度/近距筛选、不同高度的有限回波、
雷达与底盘原点差异、点云累积及车辆运动带来的遮挡仍需实机验收；不能声称完整解决
所有遮挡。保守策略也会使一些旧障碍保留更久。空输出和诊断不等同于车辆自动停车。

两个 costmap 的机器人坐标系均为 `footprint_link`；local 在 `odom`，global 在
`map`。当前静态轮廓未改，单位米：

| costmap | footprint 顶点 |
| --- | --- |
| local | (0.144, 0.724), (0.144, -0.358), (-0.534, -0.128), (-0.534, 0.494) |
| global | (0.339, 0.31), (0.339, -0.54), (-0.339, -0.54), (-0.339, 0.31) |

两者形状不同，没有夹爪实测几何，当前不能确认任一个轮廓覆盖夹爪。需要测量
导航姿态的夹爪平面外缘、相对 `footprint_link` 的位置、运动扫掠范围，以及后续
持块状态的方块外缘和安全余量。第一版不实现动态 footprint；未来状态接入后仍需
独立设计 local/global footprint 更新，过滤盒子不是碰撞轮廓。

## 未来串口状态接入

`stm32_driver` 已有底盘串口和其他任务脚本的收发处理；没有确认可复用的夹稳状态
话题、夹爪 TF 或本需求的自定义状态协议。本次不读写、不连接串口，不借用任务完成
回包推定夹稳，也不定义新协议、串口路径或波特率。

状态入口在 `scripts/gripper_pointcloud_filter.py` 的
`update_state_from_serial(validated_state)`。未来串口模块应先解析并验证用户确定的
协议、校验、时效和明确抓取确认，再通过同进程调用或受控状态发布桥接该方法：

```python
# 示例：仅说明调用位置，协议和抓取确认条件以后确定。
# validated_state 必须来自已经校验且新鲜的确认反馈，而非发送夹取命令。
filter_node.update_state_from_serial(validated_state)  # "EMPTY" / "HOLDING"
```

默认始终从 `EMPTY` 开始，表示没有确认夹住方块。仅在显式开启
`block_filter_enabled` 且状态为 `HOLDING` 时才选用额外方块盒子。
`state_input_enabled=false`，不订阅调试 String 状态话题；人工输入只供受控调试，
不得在正常运行中产生未经确认的 HOLDING。真正接入时还需确定反馈断线/超时的
状态策略和方块实测尺寸。动态 footprint 尚未实现。

## 启动顺序

原 `/scan` 投影、map_server、AMCL 和定位 lifecycle manager 都在
`wait_for_localization.py` 之前启动。FAST-LIO 与 robot_state_publisher 提供
底盘 TF，AMCL 在接收初始位姿后可发布 `map → odom`。因此实际工程中 AMCL
不在等待自身 TF 的门后，没有依据断言存在循环等待，本次不改启动顺序。
导航、过滤分支、栅格和堆叠检测仍在 `map → footprint_link` 就绪后启动。

## 构建、测试及实机验收

在工作空间根目录构建后加载环境；`source` 只加载环境：

```bash
cd /home/dgut/code/Nazarite
colcon build --packages-select rc_bringup
source install/setup.bash
colcon test --packages-select rc_bringup
colcon test-result --test-result-base build/rc_bringup --verbose
```

构建成功，12 个测试用例通过；构建仅有现有 symlink-install 的 CMake CMP0009
开发警告。已检查 launch 参数及安装后的两个新节点独立启动/退出。
已运行节点单元测试、参数重写测试及隔离 ROS 图集成测试。集成测试不启动雷达、
底盘串口或完整导航：一项在点云回调开始等待后才发送 TF；另一项使用已安装的
Nav2 obstacle layer，先建立后方障碍，再输入近距夹爪回波及无限远空扇区，验证
后方障碍保留，并用越过障碍的有限回波验证真实 clearing 仍然工作。

配置实测盒子后启动（已有完整工程运行时应先按现场流程停止，避免重复启动）：

```bash
ros2 launch rc_bringup mapping_nav2.launch.py enable_gripper_filter:=true
```

1. 先静止检查真实 header、对应时间 TF 和状态话题，确认不持续报错。
2. RViz 同时显示原始/过滤后点云、原 `/scan`、`/scan_costmap`、local/global
   costmap 和各自 published footprint；确认原始链的数据未改变。
3. 夹爪空载且固定姿态时，夹爪/车体盒内点应消失；紧邻盒外的真实障碍应保留。
   避免把环境目标方块放进过大的自车盒子，确认默认 EMPTY 不过滤方块。
4. 先让后方障碍被 costmap 观测到，再让夹爪遮挡它；删除夹爪点产生的空扇区不得
   清掉该障碍。移开遮挡并获得真实有限回波后，检查正常标记及清除。
5. 以低速平移、转动重复上述测试，记录 TF 超时、点云延迟、CPU 和漏检现象；
   本次没有真实夹爪、场地运动及性能验收结果。
6. 将实测机器人轮廓与两个 footprint 叠加核对；若夹爪伸出，需要测量后另行更新。
7. 以 `enable_gripper_filter:=false` 重启后，确认 costmap 恢复仅使用原 `/scan`。

本次未重启正在运行的完整工程；新增启动链还需上述实机验收。未修改 FAST-LIO
blind 来代替几何过滤，也未改动用户已有的 FAST-LIO、栅格子模块和 RViz 修改。
