# 用户标注附近的障碍物过滤诊断

这里的扫描来自当前实机采集，叠放起点来自用户在地图上标注的近似车位 `(3.7, 0.0)` 米、车头朝 `−Y`。图中显示的是扫描高度后执行 GICP 的局部诊断，**不是未知初值 BBS 独立识别的位置，也没有通过连续新帧验证**。用户标注不是精确位姿真值。

采集窗口收到 567 帧体坐标扫描，生产定位输出验证位姿 0 条。提取的两份扫描 `bag_0_226`、`bag_0_425` 各累积 6 帧，并按里程计补偿运动；约相隔 4.98 秒。该窗口及原始默认配置的观察结果保存在相邻 `current_capture/`。离线过滤后重新检查的是同两份扫描，没有重新采集或向机器人发送运动/初值命令。

## 看图

- [第二份扫描：过滤后赛场内保留点大图](filtered_arena_xy_second_scan.png)：绿色是核心实际选给 GICP 的点，灰色是静态赛场地图。
- [第二份扫描：过滤前后、俯视和侧视对照](obstacle_filter_comparison_second_scan.png)：橙色保留全部有效扫描，用来看到没有参与精配准的外部回波。
- [第一份扫描：过滤后大图](filtered_arena_xy.png) 和 [第一份扫描对照](obstacle_filter_comparison.png)：保留较弱样本，避免只展示较好的局部结果。

两个图各自从 `−Z` 地图重力假设、7 米量程的两次局部结果中，按“质量通过优先、双侧结构占比最高、RMSE 最小”选择一项作代表。该展示选择不改变任何全局接受标志。

## 结果

| 当前扫描 | 过滤前全扫描覆盖率 | 过滤后全扫描覆盖率 | 过滤后地图范围内覆盖率 | 过滤前双侧结构 | 过滤后双侧结构 | 独立结构体素 | 局部质量原因 |
|---|---:|---:|---:|---:|---:|---:|---|
| bag_0_226 | 63.2% | 56.3% | 79.0% | 0.46% | 0.00% | 0 | insufficient_nonground_correspondences |
| bag_0_425 | 59.4% | 55.7% | 81.9% | 3.15% | 4.02% | 13 | ground_dominated |

过滤前后全扫描覆盖率都按原始有效扫描点计数；过滤后额外要求法线相容，所以全扫描覆盖率下降并不表示被屏蔽物体仍参与匹配。地图范围内覆盖率以地图 XY 支持范围内的源点为分母，仍计入范围内没有匹配的点，并没有把所有未匹配点删除后宣称 100%。

第二份代表结果保留了 348 个降采样点用于 GICP。最终质量统计在全部有效扫描上计算：约 68.0% 的源点落在地图 XY 范围内；3662 个源点在范围外，1405 个范围内源点没有满足匹配距离/法线要求。双侧一致结构对应 256 个，落在 13 个独立的 0.30 米地图体素中，跨度约 3.48 米。该结果说明过滤确实减少了外部回波参与配准，并得到位于用户标注附近的局部重叠；但结构证据的占比和帧间稳定性仍不足。第二份代表的地图非地面对应占比约 6.39%，低于当前 8%；双侧结构占比 4.02%，也低于当前 5%。第一份扫描结构对应为 0，不能只凭第二份接近的图接受连续定位。

2 份扫描 × 2 种地图重力方向 × 2 种量程 × 每种 2 个高度起点，共 16 次局部精配准，质量接受数为 0。这不证明不存在正确位姿，也不能推算实机定位误差。

## 实际过滤含义与局限

GICP 使用 `Engine::registration_source(source, initial)` 的真实输出：先降采样，按候选姿态移入地图坐标，筛选地图 XY 支持范围和邻近静态地图的点，并排除已知法线不相容对应。导出的 `refined_*_registration_map.pcd` 恰好是该次 GICP 输入点，使用该次输出位姿投到地图坐标。图中绿色不是绘图脚本另造的过滤近似。

该方法依据地图支持关系抑制未建图回波，不能仅凭点云自动证明每个点的物体身份。一个未建图障碍物恰好靠近地图表面且法线相容时，仍可能留下；因此还需要独立结构数量、结构分布、退化、歧义和多帧检查。没有使用用户标注缩小生产全局搜索区域。

前后对照使用相同原始扫描和地图，但局部精配准姿态会因输入点变化而不同。旧基线的目标非地面比例门限为 10%，新配置为 8%；图中对应数和法线结构比例是实际测量值，不依赖这一接受门限。其他生效参数快照保存在 `localization_snapshot.yaml`。`parameters.yaml` 仍指向生产配置，重跑前应确认它是否已发生后续变化。

## 复现

在 workspace 已构建核心后运行（脚本仅构建离线诊断可执行文件，不启动 ROS，不发布位姿）：

```bash
/usr/bin/python3 src/rc_bringup/docs/rc_bbs_validation/user_marked_pose/run_probe.py src/rc_bringup/docs/rc_bbs_validation/user_marked_pose/obstacle_filter/parameters.yaml
MPLCONFIGDIR=/tmp/rc_bbs_matplotlib /usr/bin/python3 src/rc_bringup/docs/rc_bbs_validation/user_marked_pose/render_filter_comparison.py
MPLCONFIGDIR=/tmp/rc_bbs_matplotlib /usr/bin/python3 src/rc_bringup/docs/rc_bbs_validation/user_marked_pose/render_filter_comparison.py src/rc_bringup/docs/rc_bbs_validation/user_marked_pose/filter_comparison_second_scan.yaml
```

完整高度扫描与局部统计在 `probe.csv`，姿态和真实 GICP 输入路径在 `diagnostic_poses.yaml`。`selected_local_results.json` 保留两份扫描的 `−Z`、7 米量程全部局部结果，而不只保留图中代表。未知初值全局 BBS 结果由主进程另行记录，不能用本目录的用户辅助局部结果替代。
