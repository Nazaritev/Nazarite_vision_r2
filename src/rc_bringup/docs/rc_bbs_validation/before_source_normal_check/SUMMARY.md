# 自动汇总（真实尝试数，失败也计入耗时分位数）

合成扫描来自地图裁剪，含噪声和离群点；其真值只供测试评分。真实扫描没有位姿真值。
search_ms 是已预处理点云上的 BBS + GICP 耗时；不含冷构图、扫描累积和后续多帧验证。
这里的候选通过不等于定位成功；还必须通过 ROS 节点的连续新帧验证。

| 参数组 | 合成候选通过/尝试 | 全部合成尝试 P95 / ms | 接受结果最大位置误差 / m | 真实扫描候选通过/尝试 |
|---|---:|---:|---:|---:|
| raw_map_filter_baseline | 6/6 | 340.8 | 0.0484 | 0/5 |
| strict_1400ms | 6/6 | 331.7 | 0.0484 | 0/5 |
| strict_900ms | 5/6 | 808.7 | 0.0293 | 0/5 |
| single_thread_700ms | 2/6 | 699.9 | 0.0086 | 0/5 |
| real_diagnostic_1400ms | 0/6 | 1399.8 | — | 0/5 |
| fine_modes_700ms | 6/6 | 328.4 | 0.0484 | 0/5 |
| low_score_diagnostic_only | 0/0 | — | — | 0/5 |
| full_map_strict700ms | 0/6 | 699.8 | — | 0/5 |
| full_map_real_1400ms | 0/0 | — | — | 0/5 |
| explicit_z_down_real1400ms | 0/0 | — | — | 0/5 |
| known_z_up_narrow700ms | 6/6 | 110.5 | 0.0505 | 0/5 |

## 同版本地图预处理对照

两组使用相同扫描和质量门限，仅切换 map_filter_voxel_support。冷地图时间为每组一次构建测量，非重复测量分位数。

| 地图预处理 | 冷构图 / ms | 合成候选通过/尝试 |
|---|---:|---:|
| 原始点逐点滤波 | 1671.5 | 6/6 |
| 体素密度证明与稀疏点查询 | 430.7 | 6/6 |

## 实际 ROS 节点的合成输入测试

unknown 包含进程启动和首次地图构建；其他阶段从触发动作到首个通过多帧验证的位姿计时。

| 场景 | 首个验证位姿 / s |
|---|---:|
| unknown | 1.377 |
| motion | 0.100 |
| bad_initialpose | 0.837 |
| trusted_initialpose | 0.640 |
| rviz_2d_initialpose | 0.610 |
| input_resume | 1.000 |
| seed_during_search | 0.840 |
| map_reload | 0.910 |
| map_update_during_search | 1.530 |

最大位置误差：0.0341 m；最大旋转误差：0.0065 rad。

实际机器人录制得到扫描，但没有获得可信全局位姿；详见 live_validation.json 和 README.md。

## 已知地图重力方向的快速参数组（仅合成输入已知此先验）

仅缩小经过合成数据生成器确定的重力先验，不提供任何全局位置初值；生产默认仍保留未知地图方向的稳健搜索。

| 场景 | 首个验证位姿 / s |
|---|---:|
| unknown | 1.282 |
| motion | 0.090 |
| bad_initialpose | 0.671 |
| trusted_initialpose | 0.651 |
| rviz_2d_initialpose | 0.629 |
| input_resume | 0.821 |
| seed_during_search | 0.680 |
| map_reload | 0.691 |
| map_update_during_search | 1.200 |
