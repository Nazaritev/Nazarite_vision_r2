# 真实机器人三维点云定位独立测试

工作空间：`/home/dgut/code/Nazarite_vision_r2`。
源码入口：`src/rc_bringup/launch/cloud_localization_test.launch.py`。
定位包/可执行文件：`rc_cloud_localization/cloud_localizer`。
当前机器此工作空间路径是原仓库 `/home/dgut/code/Nazarite` 的符号链接；未搬动原仓库。
后续命令、构建、配置均从统一路径使用。GitHub 链接未修改。

本次只完成代码及离线验证，未执行测试 launch，未启动真实雷达、FAST-LIO、
定位节点、RViz 或运动程序，也未执行下面的运行、切换命令。

## 构建与安装

已实际完成以下构建，10 个包成功；不需要 Open3D 或仓库外的定位包：

```bash
cd /home/dgut/code/Nazarite_vision_r2
source /opt/ros/humble/setup.bash
colcon build --packages-up-to rc_bringup rc_cloud_localization --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

保持所有旧文件原样。`rc_bringup` 已有的
`ament_auto_package(INSTALL_TO_SHARE ... launch config rviz ...)`
会实际安装本入口、参数文件和 RViz 配置；本文档交付于源码目录，不额外修改安装规则。
未向旧 `package.xml` 增加依赖，因此构建时须明确列出 `rc_cloud_localization`，
不能只使用 `--packages-up-to rc_bringup` 后假设新定位包也会被构建。
隔离安装布局中的入口为
`install/rc_bringup/share/rc_bringup/launch/cloud_localization_test.launch.py`；
运行时通过包索引解析 share，不在算法中硬编码用户目录。
可离线检查文件存在（不启动 launch）：

```bash
test -f "$(ros2 pkg prefix rc_bringup)/share/rc_bringup/launch/cloud_localization_test.launch.py"
test -f "$(ros2 pkg prefix rc_bringup)/share/rc_bringup/rviz/cloud_localization_test.rviz"
```

## 输入链路与外参

```text
Livox MID360 -> /livox/lidar, /livox/imu -> FAST-LIO
  -> /cloud_registered (odom) + /Odometry (odom -> base_link)
  -> cloud_localizer + 用户提供的三维 PCD
  -> 通过质量门控后动态发布 map -> odom
```

源码已核对 FAST-LIO 的点云在 `odom`，不是雷达坐标系，不能再次乘以
`odom -> base_link` 来变换该点云。定位直接求 `T_map_odom`，机器人位姿使用
`T_map_base = T_map_odom * T_odom_base`。初始位姿是 **base_link 在 map 下的六自由度位姿**，
种子计算为 `T_map_base_seed * inverse(T_odom_base)`。
按扫描时间戳查找相同时间的里程计，或使用前后两帧平移插值和四元数 slerp；
两侧各须在 0.03 秒内，不拿任意 latest odom 代替。

输入模式独立开关：`start_lidar=false`、`start_lio=false` 默认复用现有输入，
也允许只启动缺失的一段。由本入口启动时：

- 驱动使用 `rc_bringup/config/reality/MID360_driver.json`，CustomMsg 模式
  `xfer_format=1`。保留真实机器人已有网络配置，现场网络需与该文件一致。
- FAST-LIO 使用 `config/reality/fast_lio.yaml` 中实机 IMU/LiDAR 外参
  `extrinsic_T=[-0.011,-0.02329,0.04412]`、单位旋转；关闭 PCD 累积保存，强制打开扫描发布。
- 显式 `start_robot_state_publisher=true` 才启动真实机器人
  `urdf/reality/model.xacro`；默认 false，避免与已有发布者重复。
  该 URDF 的 `base_link -> imu_link` pitch 为 3.1415926，
  `imu_link -> livox_frame` 与上述外参一致，包含 `base_link -> footprint_link`。
  当前模型都是固定关节，不需要 joint_state_publisher。FAST-LIO 本身需要这些安装 TF。

入口不包含 Nav2、自动任务、键盘、串口控制、点云二维投影或任何速度发布节点，
新代码不发布 `/cmd_vel`。已有其他 launch 中的运动节点不受本入口管理。
**旧 AMCL、其他三维定位器或静态 TF 发布者必须先停止发布 `map -> odom`。**
本节点不会结束或自动暂停用户进程。

为避免重复启动，入口首先只运行 `/cloud_localization_preflight`，观察 ROS 图和
TF 2 秒，检查已有同名定位器/RViz、map/odom 父变换、多个输入发布者，以及
显式要求启动的驱动、FAST-LIO、URDF 是否已经存在。检查不通过时打印
`STARTUP_CONFLICT`，退出整个新入口，**不会启动后续任何测试节点**。
默认复用已有唯一输入链路会通过这项检查；若某一段已有发布者，应保持对应 start 参数为 false。
检查节点只订阅 TF、读取 ROS 图，不发送 TF 或速度，也不会停掉已有进程。

## 已有雷达和 FAST-LIO 时

确认现有驱动、FAST-LIO、唯一的 robot_state_publisher 已运行，并由用户停止旧定位器
的 `map -> odom` 输出；保持现有自动任务和导航不参与本次定位测试。
不要用会启动完整导航/任务的旧 launch 作为“仅启动输入”的替代品。

```bash
cd /home/dgut/code/Nazarite_vision_r2
source install/setup.bash
ros2 launch rc_bringup cloud_localization_test.launch.py \
  map_file:=/absolute/path/to/localization_map.pcd \
  start_lidar:=false \
  start_lio:=false \
  use_rviz:=true
```

这里的 `/absolute/path/to/localization_map.pcd` 是用户必须替换的文件路径示例，
不是声明已经存在的地图。`map_file` 是必填参数，没有虚构默认地图。
PCD 必须具有 float32 的 x/y/z 字段，坐标定义为 `map`；有限点经体素滤波后不少于 100。
地图和实时扫描必须使用一致的轴向、尺度及建图外参。

## 从零启动输入链路时

在没有现有雷达、FAST-LIO、robot_state_publisher 的情况下显式开启三项：

```bash
cd /home/dgut/code/Nazarite_vision_r2
source install/setup.bash
ros2 launch rc_bringup cloud_localization_test.launch.py \
  map_file:=/absolute/path/to/localization_map.pcd \
  start_lidar:=true \
  start_lio:=true \
  start_robot_state_publisher:=true \
  use_sim_time:=false \
  use_rviz:=true \
  relocalization_backend:=icp \
  initial_pose_x:=0.0 initial_pose_y:=0.0 initial_pose_z:=0.0 \
  initial_pose_roll:=0.0 initial_pose_pitch:=0.0 initial_pose_yaw:=0.0
```

初始 xyz 单位为米，roll/pitch/yaw 为弧度，按实际地图坐标填写。
零位姿只是默认搜索种子，绝不表示已定位。
`icp`（默认）和 `ndt` 都是依赖近似初值的局部配准后端；不提供未知初值下的
全地图特征搜索，不保证从错误种子或重复结构中恢复。服务响应成功只表示已接受请求。

## Launch 参数与传递

| 参数 | 默认 | 实际用途 |
| --- | --- | --- |
| `use_sim_time` | false | 传给定位器、被启用的驱动、FAST-LIO、robot_state_publisher 和 RViz |
| `use_rviz` | true | 启动配套 RViz 配置 |
| `start_lidar` | false | 启动 MID360 驱动 |
| `start_lio` | false | 启动 FAST-LIO |
| `start_robot_state_publisher` | false | 启动实机 URDF 的唯一发布者 |
| `map_file` | 必填 | 传给定位器 `map_file`；仅接受可读 PCD |
| `localization_params_file` | 包 share 下 `config/reality/cloud_localization.yaml` | 作为定位器参数 YAML 加载 |
| `relocalization_backend` | icp | 传给定位器；可选 icp / ndt |
| `initial_pose_x/y/z` | 各 0.0 | 各自转换成 double 传给定位器 |
| `initial_pose_roll/pitch/yaw` | 各 0.0 | 各自转换成 double 传给定位器 |

例如传入 `localization_params_file:=/absolute/path/to/parameters.yaml` 覆盖质量阈值。
YAML 的节点键须为 `cloud_localizer`（或 `/**`）。显式 launch 参数覆盖 YAML 同名值。
节点参数在启动时加载；修改后重启，不提供运行中参数重载。
真实机器人默认 false；如显式使用仿真时间，所有复用输入也应使用同一个 `/clock`。

## 准确节点、话题和服务

| 节点名 | 启动条件 |
| --- | --- |
| `/cloud_localization_preflight` | 先单独运行，只做冲突预检；成功退出后才启动下列节点 |
| `/cloud_localizer` | 总是；可执行文件 `rc_cloud_localization/cloud_localizer` |
| `/cloud_test_livox` | start_lidar=true |
| `/cloud_test_fastlio` | start_lio=true |
| `/cloud_test_robot_state_publisher` | start_robot_state_publisher=true |
| `/cloud_localization_rviz` | use_rviz=true |

| 接口 | 消息/服务类型 | 方向与语义 |
| --- | --- | --- |
| `/cloud_registered` | `sensor_msgs/msg/PointCloud2` | 订阅，frame 必须为 odom，非零时间戳，SensorDataQoS |
| `/Odometry` | `nav_msgs/msg/Odometry` | 订阅，odom -> base_link，SensorDataQoS |
| `/initialpose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | 订阅，map 下 base_link 的初始化种子；协方差不参与配准 |
| `/cloud_localization/relocalize` | `std_srvs/srv/Trigger` | 服务，使用最后一次提供的初始种子重新配准 |
| `/cloud_localization/map` | `sensor_msgs/msg/PointCloud2` | 发布，map 下滤波后的目标 PCD，Reliable + Transient Local |
| `/cloud_localization/aligned_scan` | `sensor_msgs/msg/PointCloud2` | 发布，map 下当前配准候选，包含被质量门控拒绝的候选，须结合状态判断 |
| `/cloud_localization/pose` | `geometry_msgs/msg/PoseStamped` | 发布，map 下 base_link，仅 TRACKING 时发布，扫描测量时间戳 |
| `/cloud_localization/status` | `diagnostic_msgs/msg/DiagnosticArray` | 发布，约 10Hz 状态与质量；DiagnosticStatus.name=`cloud_localizer/localization` |
| `/cloud_localization/status_text` | `visualization_msgs/msg/Marker` | 发布，RViz 文本状态、质量、计算耗时和数据年龄 |
| `/tf` | `tf2_msgs/msg/TFMessage` | 订阅以查冲突；只在通过门控后动态发布 map -> odom |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | 只订阅，Reliable + Transient Local，以检查已有静态冲突 |

没有速度输出接口，也没有静态 `map -> odom`。
未显式设置 namespace；上述是默认入口的准确绝对接口名。

## 初始化与显式重定位

RViz 已提供 `2D Pose Estimate` 工具，发到 `/initialpose`；平面初值会把 z/roll/pitch 设为零。
需要六自由度种子时使用 launch 参数，或发送包含正确四元数的完整消息。
下面是 map 下 base_link 位于原点、单位四元数的语法示例，数值需按实际地图修改：

```bash
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  '{header: {frame_id: map}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}'
ros2 service call /cloud_localization/relocalize std_srvs/srv/Trigger '{}'
```

两种接口都会清除连续通过计数、作废旧的异步配准结果，等待新的质量验证。
服务复用最后的用户初始位姿，**不会把上次跟踪结果当作新的已知当前位置**；
机器人已移动时应重新发 `/initialpose`，或修改启动初值后重启。
失锁/超时后必须显式重新初始化或请求重定位，不会因输入恢复或 TF 仍存在而自动判定成功。

## 状态、时延与地图对齐检查

仅 `state=TRACKING` 且 `localized=true`、诊断时间戳新鲜，才表示当前定位通过门控。
默认连续 3 帧通过：后端收敛、对应点比例 >=0.65、对应点 RMSE <=0.25m，
相对配准种子的单次修正距离 <=1.5m、角度 <=0.8rad。
对应点使用 0.8m 最近邻半径；比例的分母是滤波后的全帧扫描点数。
这些是待实机场地验证的保守初始参数，不构成精度或可靠性验收结果。

| 状态 | 含义/处理 |
| --- | --- |
| CHECKING_TF | 启动后至少观察 TF 2 秒，尚不发布定位 TF |
| WAITING_INPUT | 尚未同时收到点云及合法里程计 |
| WAITING_SYNC | 缺少扫描同一时间的里程计，不发布新定位 TF，连续通过计数归零 |
| ACQUIRING | 正在验证种子，未满足连续通过帧数 |
| TRACKING | 最近质量和时间门控通过 |
| LOST_REGISTRATION_REJECTED / LOST_TOO_FEW_POINTS | 配准质量不够/有效点太少，停止位姿和 TF，需显式重新初始化 |
| INPUT_TIMEOUT / RESULT_TOO_OLD | 输入或配准结果过期，停止位姿和 TF，需显式重新初始化 |
| TF_CONFLICT | 检测到其他发布者拥有 odom 或 map 的父变换；输出锁定直到重启 |
| INPUT_PUBLISHER_CONFLICT | 运行中发现点云或里程计有多个发布者，停止有效输出 |
| CLOCK_ERROR / ODOMETRY_TIME_RESET | 未来时间戳或里程计时间回退，处理时钟/输入后重新初始化 |
| INVALID_ODOMETRY / INVALID_CLOUD_FRAME_OR_STAMP / INVALID_CLOUD_LAYOUT / REGISTRATION_ERROR | 输入、数据布局或配准异常，报错并停止有效输出 |

诊断 level：TRACKING=OK(0)，等待/验证=WARN(1)，失锁、超时、冲突=ERROR(2)。
节点退出后状态话题也会停止，消费者还须检查诊断流本身的时效，不能把最后一个 OK 永久保存。

```bash
ros2 topic echo /cloud_localization/status
ros2 topic echo /cloud_localization/pose
ros2 topic hz /cloud_registered
ros2 topic hz /Odometry
ros2 topic info /tf --verbose
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_link
```

诊断值 `compute_ms` 是最近一次配准和最近邻质量评估的耗时，不包括队列等待/点云预处理；
`scan_age_ms`、`odom_age_ms` 是当前 ROS 时间减最新输入测量时间，
`result_age_ms` 是当前时间减最后一次发布的有效位姿测量时间。
`cloud_receipt_age_ms`、`odom_receipt_age_ms` 是单调时钟下距离上次接收的间隔，
因此 ROS 时钟暂停时仍能发现断流。未获得指标时为 nan，失锁时可能保留历史质量，
必须同时看状态和年龄。默认测量年龄上限 0.6 秒、接收间隔上限 1.0 秒。
异步配准只处理最新扫描、不积压整条输入队列，过期计算结果会被拒绝。

RViz 固定 frame 为 map，已配置：灰色目标 PCD、青色配准候选、验证后的位姿轴、
状态/质量/时延文字、TF 和真实 URDF 订阅。先检查静止状态下墙面、立柱、地面等
不同方向结构是否同时重合，不能只看一片地面对齐。
青色点云在配准拒绝时也可能可见，历史位姿轴/TF 也可能留在 RViz 或 TF 缓冲中；
**TF 存在、点云可见或 Pose 显示有箭头都不单独代表定位成功。**
状态文本有效期 0.6 秒。原 `/cloud_registered` 显示默认关闭，可按需启用辅助观察。

二维地图完全可选：配置包含默认关闭的 `/map` 显示
（`nav_msgs/msg/OccupancyGrid`）；已有二维地图发布者时可启用。
本入口不启动 map_server，也不读取任何二维 YAML，三维定位不依赖二维地图。

## TF 冲突定位

节点观察 `/tf` 和带历史的 `/tf_static`，通过 DDS publisher GID 排除自身输出，
对其他发布者发送的 `* -> odom` 或 `* -> map` 锁定 TF_CONFLICT 并打印具体边。
这也会拒绝旧静态 `map -> odom`，不会用该静态变换证明新定位有效。
其他发布者即使与新节点同名，也不会被当成自身。
发生冲突后只关闭本节点的有效输出，不终止其他进程；由用户停掉冲突来源后重启本入口。
单次 /initialpose 或服务调用不能解除冲突锁定。
运行中也持续检查 `/cloud_registered` 和 `/Odometry` 的发布者数量，出现重复输入
会报 INPUT_PUBLISHER_CONFLICT 并撤销有效定位，不能把两套 FAST-LIO 混合输入当作成功。

`ros2 topic info /tf --verbose` 用于列出发布者，再结合日志与 `/tf` 内容确认具体变换：

```bash
ros2 topic echo /tf tf2_msgs/msg/TFMessage
ros2 topic echo /tf_static tf2_msgs/msg/TFMessage --qos-durability transient_local
```

多个 `/tf` 发布者通常正常（FAST-LIO 和 URDF 各有职责）；冲突判断关注重复父边。
无法预先证明一个尚未发送任何 TF 的静默发布者将来不会发布冲突变换：启动观察后
仍持续检查，收到冲突立即锁定。ROS 中也无法撤销其他节点缓存的历史 TF。
预检通过也不能阻止用户在之后另外启动同一套节点；不要同时从其他终端重复启动输入链路。
若 TF 发布者跨 ROS_DOMAIN_ID 或网络隔离不可见，它们不属于当前可检查的 ROS 图。

## 退出并恢复原 AMCL

在测试入口终端按 Ctrl+C，停止本入口创建的定位器、RViz，以及显式开启的输入节点。
默认复用的驱动、FAST-LIO、robot_state_publisher 不归此 launch 所有，不会被停止。

如果旧 `/amcl` 仍运行、测试前仅由用户停用它的 TF 输出，可采用生命周期切换，
先让原导航/自动任务停止参与，再手工执行（本次没有执行）：

```bash
# 测试前：仅当旧 AMCL 是 active 时，用户手动停用
ros2 lifecycle get /amcl
ros2 lifecycle set /amcl deactivate
# 退出三维测试后：仅当旧 AMCL 是 inactive 时，用户手动恢复
ros2 lifecycle set /amcl activate
```

恢复后对 AMCL 重新发送 `/initialpose`，检查 `/amcl_pose`
（`geometry_msgs/msg/PoseWithCovarianceStamped`）及其动态 `map -> odom`。
若 lifecycle manager 自动恢复了旧 AMCL，测试器会报冲突；应由用户管理旧启动流程，
不能靠测试器抢占 TF。生命周期状态不符合上述条件时，不盲目重复转换。

若旧完整入口已退出，先彻底退出本三维测试，再退出仍单独运行的重复输入节点，
由用户按原现场流程恢复原 AMCL 入口。仓库中已核对的原入口示例为：

```bash
cd /home/dgut/code/Nazarite_vision_r2
source install/setup.bash
ros2 launch rc_bringup mapping_nav2.launch.py
```

**该原入口会启动底盘串口、Nav2，并在 3 秒后启动 `run_keyboard_test/nav_to_pose`
自动任务；它是恢复原完整系统的命令，不是本任务的静止定位测试命令。**
另一个原 `nav2.launch.py` 也包含底盘/导航。旧 `localization.launch.py` 引用外部
`lidar_localization_ros2`，不是本次新建定位器的入口。恢复时不能让新旧两个定位器同时发布 TF。

## 离线验证范围

允许执行、不创建 ROS 节点的检查：

```bash
cd /home/dgut/code/Nazarite_vision_r2
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon test --packages-select rc_cloud_localization --event-handlers console_direct+
colcon test-result --test-result-base build/rc_cloud_localization --verbose
python3 -m pytest src/rc_bringup/test/test_cloud_localization_launch.py -q
```

纯 C++ 测试使用程序生成的不对称点云，验证六自由度 ICP、无重叠拒绝、NDT、
质量阈值、已在 odom 下点云的种子组合，以及初始化不足、失锁/年龄/时钟异常、
TF 冲突锁定时的输出门控。Python 检查只构造 launch 动作、解析实际参数和 xacro、
读取 RViz/YAML、检查安装资源；不调用 LaunchService、不执行 Node 动作。
本次 8 项 C++ 用例与 27 项 Python 检查通过，其中 Python 包含 14 项冲突预检策略
和 13 项 launch/资源检查；失败的预检不会放行后续节点，也由离线测试验证。
构建只有现有包的 CMake/头文件
安装提示及当前环境的 underlay 重复包提示，新定位包无编译错误。
尚未验证真实传感器质量、实际 TF 竞争、现场精度或实时性能，这些运行命令留待用户另行授权。
