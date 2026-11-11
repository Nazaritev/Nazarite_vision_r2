# 独立 3D-BBS 定位入口与验证说明

本次只新增文件，没有修改已有定位、建图、导航、串口、URDF、地图或旧 launch；原有两个子模块的用户改动也保留。新入口使用独立 `rc_bbs_localization` 包，复用现有 MID360 驱动、FAST-LIO 去畸变和机器人 TF。本文中的随机位姿仅存在于测试数据和测试发布器中，生产节点没有固定初始位置。

**目前实机“未知位置后 1 秒内得到准确初始点”的目标没有达成。** 当前硬件实际收到扫描，但没有通过全局定位验收，也没有输出定位成功。合成回归测试的通过率、耗时和误差不能当作实机准确率；实机没有独立位姿真值。

**最新配置与验证见 [纯赛场地图障碍物处理](OBSTACLE_FILTER.md)。** 用户确认周围障碍未包含在地图后，已加入地图覆盖范围过滤、未建图回波容限和独立结构检查。下文原先的全扫描 65% 验收与仅严格 BBS 参数结论保留为历史说明，当前启用过滤时以新报告为准；合成回归通过，实机全局搜索仍超预算。可直接查看 [过滤后实际 GICP 输入点](user_marked_pose/obstacle_filter/filtered_arena_xy_second_scan.png)。

## 启动和调参

已完成该工作区中的编译。新终端加载工作区后执行，无需输入地图或定位参数：

```bash
cd /home/dgut/code/Nazarite
source install/setup.bash
ros2 launch rc_bringup rc_localization.launch.py
```

- 启动开关、地图路径和 DDS：`../../config/reality/rc_localization_launch.yaml`。
- 全部定位参数：`../../config/reality/rc_localization.yaml`。参数只读，编辑后重启生效。
- 测试数据、随机种子、参数组和测试时长：`../../config/reality/rc_localization_validation.yaml`。
- 新 launch：`../../launch/rc_localization.launch.py`，按输入、TF、定位、显示分段编排。
- 地图默认 `/home/dgut/code/Nazarite/src/rc_bringup/map/1.pcd`。

`input_mode: live` 启动驱动、FAST-LIO、机器人 TF 和新定位节点。`external` 复用已有输入；此时已有 FAST-LIO 必须发布 `/cloud_registered_body`，并保证它是已经去畸变的 IMU 局部扫描。不要同时启动旧定位节点/AMCL 的 `map -> odom` 发布。新节点发现竞争的同一 TF 边后会锁定失败，停止使用位姿，移除冲突发布者后重启。

本机原环境中的 `/home/dgut/cyclonedds.xml` 仅绑定 `eno1`，禁用多播，发现列表只有 `10.0.0.1/2`；最初实测本机各进程没有发现彼此。新入口默认 `dds_profile: local`，仅对自己启动的进程设置 `rc_localization_dds.xml`，通过回环接口通信，没有改动系统配置。复用外部进程时必须让双方 DDS 配置兼容，可改为 `system`；本机以外的 RViz/导航程序也需另行配置可互通的网络接口。

## RViz 初值与输出

RViz 默认打开 `map` 固定坐标系、静态 PCD、已验证扫描、位姿、状态文字和 TF。选择 **2D Pose Estimate** 在地图点击并拖动方向，会向 `/initialpose` 发送 `map` 中的 **base_link** 位姿。它只是待验证初值。严重错误时局部 GICP 会失败，随后自动改用全局搜索，不围绕错误初值限制 BBS 窗口。

ROS 测试同时覆盖完整姿态初值和与 RViz 相同的二维初值（只给 x/y/yaw，z=0，roll/pitch=0）。后者依靠 GICP 补全三维姿态，失败时仍须全局重定位。合成测试使用与实际机器人一致的 Y 轴 π 翻转雷达安装。

| 话题/服务 | 含义 |
|---|---|
| `/bbs_localization/map` | 地图，transient-local |
| `/bbs_localization/sensor_pose` | 已验证 `T_map_sensor`，sensor=`imu_link` |
| `/bbs_localization/pose` | 已验证 base_link 位姿 |
| `/bbs_localization/aligned_scan` | 已验证的配准扫描 |
| `/bbs_localization/status` | 状态、失败原因、`pose_valid`、后端、地图版本、质量和耗时 |
| `/bbs_localization/status_text` | RViz 状态文字 |
| `/bbs_localization/relocalize` | Trigger：重新发起未知初值的全局搜索 |
| `/bbs_localization/reload_map` | Trigger：使当前位姿失效并按内容版本重新加载地图 |

失败时不继续发布新的定位位姿或 `map -> odom`。RViz 可能仍显示历史消息，必须检查状态中的 `pose_valid` 和时间。位姿使用 `PoseStamped`，没有把 BBS 分数或 GICP fitness 冒充协方差。原 FAST-LIO 仍独立发布其局部里程计和 TF。

具体配置引用、加速度单位和 TF 树的逐项核对见 [TF_AUDIT.md](TF_AUDIT.md)。本轮冷地图优化和对照数据也记录在该文件中。

## 现有接口分析与坐标链

旧 `rc_cloud_localization` 只实现有初值的 ICP/NDT，并使用已在 `odom` 中的 `/cloud_registered`；不能直接拿它作为 BBS 的传感器局部输入。新节点使用 FAST-LIO `publish_frame_body()` 的 `/cloud_registered_body`，它将去畸变点从 LiDAR 变换到 `imu_link`，时间戳为扫描结束时刻。因此不再重复应用 LiDAR→IMU 外参 `[-0.011,-0.02329,0.04412]`。

现有 URDF 的 `T_base_sensor` 为绕 Y 轴约 π、零平移，节点从 TF 读取并监视变化。FAST-LIO 当前实现还会把启动时的完整 base_link 姿态重设为 odom 原点；因此 odom 的绝对 roll/pitch 不能直接充当重力参考。新实现使用 0.2 s 原始 IMU 加速度窗口，并检查加速度单位/模长、稳定性、角速度和同期里程计位移。IMU 消息标称 frame 经 TF 转入扫描 frame；当前安装中 LiDAR/IMU 的旋转外参为单位阵。

单位是米、弧度、秒、m/s²。拒绝非有限点、非单位四元数、世界坐标扫描、过旧/未来数据、不同 odom/base/sensor frame、缺失外参以及未声明去畸变的输入。里程计按扫描时间插值，禁止外推；跨越过大时间间隙、平移或旋转突变时不能沿用原预测。IMU 不可信时明确等待/拒绝 BBS；目前没有实现在持续剧烈运动中可靠恢复绝对重力的替代后端。

```text
p_map = T_map_sensor * p_sensor

H_map_up = I (+Z) or Ry(pi) (-Z), explicitly configured hypotheses
BBS source = H_map_up * R_gravity_sensor * p_sensor
T_map_sensor(search) = T_map_gravityaligned(BBS) * H_map_up * R_gravity_sensor

T_odom_sensor(t) = T_odom_base(t) * T_base_sensor
T_map_odom = T_map_sensor(search) * inverse(T_odom_sensor(search))
T_guess_sensor(now) = T_map_odom * T_odom_sensor(now)
T_map_base(now) = T_map_sensor(now) * inverse(T_base_sensor)
```

BBS 搜索配置的整个 xyz 范围和完整 yaw；默认范围根据地图边界设置为 `[-2,-11.5,-1.5]` 到 `[7,2,2]` m，和任何初值无关；更换地图后应在 YAML 核对搜索范围是否覆盖新地图。地图没有可核实的重力元数据。默认 `map_gravity_z_signs: [1.0,-1.0]` 明确搜索 +Z/-Z 两种地图向上约定，两种搜索共用同一预算，必须都完成，存在相互冲突的合理结果就拒绝。不会根据较快返回的方向静默选择先验。地图主平面相对 Z 约倾斜 2°，这不能独自证明重力方向的正负。默认组合 roll/pitch 不确定性按 IMU 0.06 rad 加地图轴 0.04 rad 设置为 0.10 rad，预对齐后搜索 ±0.12 rad。这个范围是显式配置的假设，不是任意无先验 6DoF 全局恢复保证。只有取得可信地图重力元数据后，才应采用单方向、更窄的快速配置。

## 搜索、缓存与多帧验证

CPU 使用官方 [KOKIAOKI/3d_bbs](https://github.com/KOKIAOKI/3d_bbs) 的固定提交 `41529a34a2fb9618b5ff560fb3c2363f1615666d`。原文件、README 和 MIT 许可证完整保留；本地扩展仅应用于构建目录副本，详见 `rc_bbs_localization/third_party/README.md` 和 `cpu_safety.patch`。

官方 CPU 接口只返回一个最优结果。扩展在完整配置范围内重复搜索，排除已返回的叶子位姿邻域；不按非叶节点的代表位姿粗暴排除整支。对明显不同的候选分别进行完整 6DoF GICP。必须完成最后一次“剩余无合格候选”的搜索才能进入验证；超时、内存队列上限、存在第七个候选等均拒绝成功。默认候选分离尺度为 0.75 m / 0.35 rad，最多 6 个候选。

歧义判断只覆盖指定体素/角度网格、命中门限以上以及分离尺度之外的模式，不能证明小于该尺度的连续位姿唯一性。重复场景中多个不同位姿的覆盖率与 RMSE 相近时拒绝。本次保留了重复房间的模式枚举与拒绝测试。GPU 源码保留，但本机没有 CUDA/NVIDIA 可用环境，**只编译 CPU**；指定 `official_gpu` 会报错，没有静默切换，也没有未经验证的 GPU 扩展。

地图预处理一次，构建法线、KD-tree 和多层 BBS 稀疏体素。按 PCD 内容和配置的 SHA-256 版本缓存，搜索期间不重建。除了定时检查，还在结果提交时重新检查地图文件时间和 TF 外参，关闭监视定时器两次运行之间的旧结果提交窗口。相同内容的重载复用缓存；旧 worker 持有自己的 shared_ptr，主线程切换版本不会释放仍在使用的旧地图。缓存目前是进程内的，不是跨重启的磁盘缓存，首次构图耗时需计入冷启动。

扫描先在 FAST-LIO 内去畸变，再将最多 0.15 s / 8 帧的局部扫描通过同期 odom 增量补偿到最新 sensor frame，随后过滤无效点、量程、孤立离群点，并分别进行 GICP / BBS 降采样。BBS 默认选择非地面结构，降低大面积地面的影响；GICP 和质量评价仍保留全部有效结构和地面。

仅一个后台 worker，构图、BBS、GICP 和重型滤波都在后台运行。回调持续收取扫描、IMU 和 odom。搜索快照记录扫描时间、当时的 `T_odom_sensor`、外参和 epoch。返回后候选传播到新扫描时刻验证。地图更新、外参变化、odom 重设、用户初值或重新定位请求都会改变 epoch，旧结果不能提交；正常跟踪的重新建立也更新 epoch 和预测锚点。

状态是 `UNINITIALIZED / SEARCHING / VERIFYING / TRACKING / DEGRADED / LOST`。默认连续 3 个新扫描验证通过才进入 TRACKING；验证从搜索完成后开始，间隔至少一个完整累积窗口，避免大量复用同一批点。候选比较补偿帧间运动。连续 3 次跟踪失败触发全局恢复；失去输入、局部结果过旧或几何退化时不发布可信位姿。重试间隔 0.5 s，搜索任务总成功预算 1400 ms，超预算的结果即使残差很好也被拒绝。

## 指标定义和门限

| 指标 | 定义/单位 |
|---|---|
| BBS hit fraction | 命中地图占用体素的 BBS 结构采样点 / 全部 BBS 结构采样点；不是全扫描覆盖率 |
| correspondences | 最终变换后，距地图最近点 ≤0.4 m 的有效源点数量，至少 120 |
| coverage | 对应数 / **降采样之前的全部有效源点数**，至少 0.65；未匹配点留在分母 |
| inlier MSE | 已匹配点最近邻平方距离均值，m² |
| inlier RMSE | sqrt(inlier MSE)，m，至多 0.22 m |
| inlier P95 | 已匹配点距离的 95 分位，m，至多 0.35 m |
| capped all MSE | 全部有效源点的误差均值；未匹配点贡献 0.4² m²，防止少数成功对应掩盖未匹配点 |
| nonground fraction | 有效对应中，目标法线 abs(n_z)<0.8 的比例，至少 0.10 |
| consistent structure correspondences | 对应点双方都有有效非地面法线，锐角差 ≤30°，至少 120 个且占全部有效对应至少 5%；地面点不能借地图墙面法线成为立体结构证据 |
| information ratio | 以源扫描 RMS 距离归一化旋转项的点到面 6×6 信息矩阵最小/最大特征值，至少 1e-4；是退化指标，不是协方差 |

未精配准时 `quality_evaluated=false`，此时对应数/残差不是一次已完成的质量评价；未执行 BBS 的局部跟踪帧用 NaN 表示没有本帧 BBS 分数。所有距离门限在与 PCL 平方距离比较前先平方。还检查 GICP 收敛、未耗尽迭代、刚体变换有限性、粗到细修正幅度和运动补偿后的连续帧一致性。单平面/地面主导和信息矩阵近奇异均拒绝。

## 实测数据与调参结论

最终完整数据见 [SUMMARY.md](SUMMARY.md)、[benchmark.csv](benchmark.csv)、[summary.json](summary.json)、[ros_validation.json](ros_validation.json)、[live_validation.json](live_validation.json) 和 [timing.png](timing.png)。失败样本也保留，耗时分位数包含失败尝试；`round*.csv` 保留了前几轮实验数据。

测试 CPU 为 AMD Ryzen 7 8845HS（8 核/16 线程），PCL 1.12，Eigen 3.4，ROS 2 Humble，Release 构建。地图原始 1,125,614 点，清理后 1,125,607 点，0.15 m 降采样后 5415 点；边界 `[-1.103,-10.341,-0.838]` 到 `[6.177,0.902,1.135]` m。当前半径法线估计下，地图自身有效点的非地面对应占比约 10.6%，说明可区分的立体结构相对有限。

主要调参（最终默认优先保持未知重力情况下的可靠性）：

| 项目 | 初始尝试 | 最终默认 | 原因 |
|---|---:|---:|---|
| 地图邻域过滤加速 | 原始点逐点查询 | `map_filter_voxel_support: true` | 用体素原始密度证明邻域支持，只查询稀疏点；同版本冷构图 1671.5→430.7 ms，候选均为 6/6 |
| BBS 地图搜索体素 | 0.50 m | 0.30 m | 粗体素产生过多相似命中 |
| BBS 扫描体素 | 0.50 m | 0.35 m | 保留窄小立体结构，与地图体素独立配置 |
| GICP 扫描体素 | 0.20 m | 0.20 m | 局部精度要求保持 |
| BBS 最低结构命中比例 | 0.55 | 0.90 | 低分模式过多时拒绝不可靠匹配；这也会降低真实低重叠扫描的可用率 |
| 搜索预算 | 1400 ms；也测试 700/900 ms | 1400 ms（双方向/较宽姿态稳健配置） | 700 ms 对未知地图重力的尝试会超时，不能伪造成功；已知方向快速组单独保留 |
| 并行方式 | 每个小分支都唤醒线程 | 小于 64 个子节点串行，大批次 4 线程 | 减少 CPU OpenMP 调度开销 |
| 候选分离/上限 | 1.5 m / 0.5 rad / 3 | 0.75 m / 0.35 rad / 6 | 检查更接近的歧义模式 |
| GICP 最大迭代 | 35 | 60 | 有先验扰动时避免提前耗尽迭代，没有放宽质量门限 |
| 扫描累积 | 单帧 | 0.15 s，最多 8 帧 | 缓解 40 Hz 单帧立体结构稀疏 |
| 多帧验证 | 3 个相邻扫描 | 3 个新窗口，间隔至少 0.15 s | 避免复用累积点造成假独立证据 |

合成测试的 6 个位置由固定随机种子自动从地图采样，不输入生产节点。扫描加入 2 cm 点噪声、8% 候选离群点，重力预对齐另加每轴 ±0.03 rad 扰动，雷达采用实际倒装外参。ROS 发布器还逐帧加入额外测量噪声和随机丢点，进行真实节点的多帧、运动补偿和恢复测试。它仍是地图派生数据，不含实际遮挡建模，也不是实机真值实验。

实际新 launch 已连接 MID360 并收到大量体坐标扫描。最终实机仍无验证位姿，状态主要是 `no_bbs_candidate`。还测试了更低命中门限、更细源采样、保留地面的地图体素以及 1400 ms 预算；出现无候选、未完成搜索或过多候选，仍没有可信的全局决定。历史 bag 中，甚至存在局部覆盖率约 0.92、RMSE 约 0.13–0.15 m 的候选，但全局搜索/歧义判断未完成，不能据此发布成功。它说明只看局部 fitness 会有误判风险。

用户已确认机器人位于这张地图的覆盖范围内。我们额外测试了两种地图重力方向、更宽的 roll/pitch 窗口和较宽松的诊断参数；没有证实单纯的上下颠倒能解决当前实机匹配，因而没有擅自固定某个重力方向。现有证据支持：当前扫描与地图的稳定立体结构重叠不足、地面占比高以及重复结构导致的全局判别困难；保守门限、地图轴约定和 CPU 时间预算也限制了可用率。**没有独立真值和建图重力元数据，不能断言唯一原因是算法或地图；“在地图内”本身不能证明当前扫描与静态地图中的立体结构仍有足够一致的重叠。** 若继续实机优化，应先核对米制、地图重力元数据、结构变化、遮挡/动态物体和地图几何质量，再补录包含墙角、柱体、门框等稳定结构的扫描与参考位姿。不能用固定初始点或删除失败门限来制造 1 s 成功。

本轮完整结论见 [STRUCTURE_DIAGNOSIS.md](STRUCTURE_DIAGNOSIS.md)，包括长窗口实机试验与最终回归。

## 本轮结构质量检查修复

旧质量统计的 `nonground_correspondences` 只检查地图法线，可能把落在墙面附近的源地面点算作非地面结构。本轮增加 `consistent_structure_correspondences`：源扫描按 GICP 分辨率估计法线，将法线旋转到地图后，要求源和目标都是非地面法线，且法线锐角差不超过 `normal_consistency_rad: 0.5235987755982988`（30°），至少 `min_consistent_structure_correspondences: 120` 个，且占全部有效对应的比例不少于 `min_consistent_structure_fraction: 0.05`。比例检查防止累积重复点仅靠绝对数量通过门限。原有地图单侧统计保留，门限没有放松。新指标同时写入状态话题、运行 CSV 和离线诊断 CSV。

在完整地图、BBS 门限 0.80、量程 7 m 的试验中，旧质量检查曾接受两份保存实机扫描的待验证候选。进一步诊断显示，部分候选虽有约 83%–86% 的全扫描覆盖率、约 2800–3600 个地图单侧非地面对应，却只有 0–31 个双侧一致结构对应。新质量检查拒绝这些证据不足的候选；这些扫描没有独立位姿真值，不能把旧候选当成真实正确位置。参数组没有替换生产默认。

[本轮扫描重叠报告](overlap_with_normal_check/SUMMARY.md) 和 [结构证据图](overlap_with_normal_check/structure_evidence.png) 保存了逐组数据。`overlap_before_normal_check/` 保留旧质量检查的诊断；`overlap_with_normal_check/diagnostic_clouds/` 保留清理后的源扫描、地图与 BBS 结构点；候选姿态见 `diagnostic_poses.yaml`。另外用 8 秒预算分开测试 ±Z、完整/筛选地图，共检查 25 个候选，仍没有质量通过，最多 71 个双侧结构对应；数据保存在 `extended_overlap_probe/`。这不是任意姿态或任意门限下不存在正确位姿的证明。

离线调参工具也已修复：每个参数组现在单独执行源扫描预处理，`max_range` 等参数不再沿用基础配置的预处理结果。可在 CSV 的 `valid_source` 列核对：同一份实机扫描 30/7/3 m 量程的有效点数会不同。本轮修复前暂存的量程对比已重跑；旧量程行没有作为最终量程证据；主报告中其他相同预处理设置的参数组不受此问题影响。

新增回归覆盖累积重复地面点时的结构占比、以及“地面扫描靠近地图墙面，覆盖率和单侧非地面数都很好但双侧结构不成立”，以及真实三维结构匹配仍能通过。其他恢复场景继续使用真实 ROS 节点进行合成输入回归。

重跑专门的重叠诊断，无需在命令行设置参数：

```bash
ros2 run rc_bbs_localization validate_overlap.py
```

配置文件是 `config/reality/rc_localization_overlap_validation.yaml`。诊断只使用已保存扫描，不启动驱动或发布定位位姿。

## 复现验证

```bash
source /home/dgut/code/Nazarite/install/setup.bash
ros2 run rc_bbs_localization validate_localization.py
```

该工具按 YAML 重跑已保存的 PCD 样本、所有参数组、合成 ROS 测试并生成报告；不启动真实雷达。重新提取 bag 用 `extract_validation_scans.py`，重新观察当前硬件用 `validate_live_localization.py`，所有开关/时长仍来自 YAML。最新硬件原始 bag 暂存 `/tmp/rc_bbs_live_validation3/bag`（约 104 MB），已将选取的真实扫描保留到 `bag_scans/`；`/tmp` 清理后仍可用保留样本复现离线测试。

编译/核心回归：

```bash
colcon build --packages-select rc_bringup rc_bbs_localization
colcon test --packages-select rc_bbs_localization
colcon test-result --test-result-base build/rc_bbs_localization --verbose
```

核心测试另覆盖地图滤波加速与原始滤波的逐点一致性、薄结构原始密度支持及扫描点数保持。核心测试覆盖完整变换传播、非法位姿、后端拒绝、重力窗口、全扫描覆盖率分母、未匹配点惩罚、单平面拒绝、收敛要求、超时拒绝、重复模式枚举/拒绝和地图内容版本缓存。ROS 测试覆盖未知初值、错误初值、完整初值、二维 RViz 初值、运动跟踪、输入中断恢复、重力先验拒绝、后台搜索中重设初值、同版本地图重载以及后台搜索中的地图内容版本更新（仅修改测试副本，原地图不写入）。
