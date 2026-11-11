# 自动汇总（真实尝试数，失败也计入耗时分位数）

合成扫描来自地图裁剪，含噪声和离群点；其真值只供测试评分。真实扫描没有位姿真值。
search_ms 是已预处理点云上的 BBS + GICP 耗时；不含冷构图、扫描累积和后续多帧验证。
这里的候选通过不等于定位成功；还必须通过 ROS 节点的连续新帧验证。

| 参数组 | 合成候选通过/尝试 | 全部合成尝试 P95 / ms | 接受结果最大位置误差 / m | 真实单扫描候选通过/尝试 |
|---|---:|---:|---:|---:|
| strict_1400ms | 5/6 | 328.9 | 0.0335 | 0/5 |
| strict_900ms | 5/6 | 823.4 | 0.0298 | 0/5 |
| single_thread_700ms | 2/6 | 700.0 | 0.0074 | 0/5 |
| real_diagnostic_1400ms | 0/6 | 1399.9 | — | 0/5 |
| fine_modes_700ms | 5/6 | 331.0 | 0.0335 | 0/5 |
| low_score_diagnostic_only | 0/0 | — | — | 0/5 |
| full_map_strict700ms | 0/6 | 700.0 | — | 0/5 |
| full_map_real_1400ms | 0/0 | — | — | 0/5 |
| explicit_z_down_real1400ms | 0/0 | — | — | 0/5 |
| known_z_up_narrow700ms | 6/6 | 122.4 | 0.0765 | 0/5 |

## 实际 ROS 节点的合成输入测试

unknown 包含进程启动和首次地图构建；其他阶段从触发动作到首个通过多帧验证的位姿计时。

| 场景 | 首个验证位姿 / s |
|---|---:|
| unknown | 1.169 |
| motion | 0.110 |
| bad_initialpose | 0.825 |
| trusted_initialpose | 0.659 |
| rviz_2d_initialpose | 0.620 |
| input_resume | 0.950 |
| seed_during_search | 0.871 |
| map_reload | 0.959 |
| map_update_during_search | 1.371 |

最大位置误差：0.0318 m；最大旋转误差：0.0065 rad。

实际机器人录制得到扫描，但没有获得可信全局位姿；详见 live_validation.json 和 README.md。

## 已知地图重力方向的快速参数组（仅合成输入已知此先验）

仅缩小经过合成数据生成器确定的重力先验，不提供任何全局位置初值；生产默认仍保留未知地图方向的稳健搜索。

| 场景 | 首个验证位姿 / s |
|---|---:|
| unknown | 1.029 |
| motion | 0.090 |
| bad_initialpose | 0.638 |
| trusted_initialpose | 0.650 |
| rviz_2d_initialpose | 0.640 |
| input_resume | 0.830 |
| seed_during_search | 0.720 |
| map_reload | 0.680 |
| map_update_during_search | 0.959 |
