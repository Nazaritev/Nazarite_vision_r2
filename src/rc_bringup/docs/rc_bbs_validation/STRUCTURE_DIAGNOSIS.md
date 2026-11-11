# 结构对应质量修复与本轮实机诊断

实机准确全局定位和 1 秒目标仍未达成。当前没有独立实机位姿真值；下述合成误差不能当作实机误差。

## 已修复的问题

旧逻辑只用地图法线统计非地面对应，地面扫描点靠近地图墙面时也能借用墙面法线作为立体结构证据。现在对源扫描和地图使用相同物理半径估计法线，要求双方都是非地面、法线锐角差不超过 30°；至少 120 个双侧一致对应，且占全部有效对应至少 5%。新比例门限避免累积重复点仅靠数量通过。原有质量、退化、完整搜索和连续新帧验证门限保留。

新增参数位于 rc_localization.yaml：`normal_consistency_rad`、`min_consistent_structure_correspondences`、`min_consistent_structure_fraction`。新指标同时发布到状态话题与 CSV。

离线基准还修复了预处理未随参数组更新的问题；量程扫描现已逐组重新清理，并导出候选姿态、地图、扫描和 BBS 结构点。

## 验证结果

核心回归共 17 项，包括地面靠近墙面时的误接受、真实三维结构通过、重复地面点稀释结构占比，以及原来的模式歧义、超时和缓存检查。ROS 未知初值、错误初值、RViz 初值、跟踪、输入中断、重力拒绝、重设初值与地图更新等场景均通过。

| 合成 ROS 场景 | 首个验证位姿 / s |
|---|---:|
| unknown | 1.446 |
| bad_initialpose | 0.867 |
| rviz_2d_initialpose | 0.650 |
| input_resume | 0.980 |

该轮合成 ROS 最大位置误差 0.0313 m，最大旋转误差 0.0063 rad。unknown 包含启动和地图构建，其余从触发动作计时；均为单轮测量。

生产默认的地图派生候选回归仍为 6/6。实机保存扫描中，降低 BBS 门限和保留完整地图曾得到覆盖率约 83%–86% 的候选，但其双侧一致结构只有 0–31 个，已由新门限拒绝。详见 [重叠诊断](overlap_with_normal_check/SUMMARY.md) 和 [结构证据图](overlap_with_normal_check/structure_evidence.png)。

8 秒预算分别测试两种地图重力方向、筛选/完整地图，检查的 25 个候选均未通过质量门限，最多 71 个双侧结构对应。这一有限实验不能证明所有位姿和门限下均不存在正确匹配。

最终生产默认硬件复测收到 650 帧扫描，验证位姿数为 0；状态话题已确认包含新的双侧结构数量与比例指标。

## 较长累积窗口实机试验

两组都采用 0.4 秒、最多 20 帧累积，验证帧间隔也调整为 0.4 秒；不是 1 秒性能结果。生产参数已恢复为 0.15 秒累积与原 BBS 设置。

| 实机试验 | 扫描消息数 | 最大有效源点数 | 验证位姿数 |
|---|---:|---:|---:|
| accumulated_full_map | 887 | 32252 | 0 |
| accumulated_structure_map | 888 | 33804 | 0 |

筛选地图组主要无 BBS 候选；完整地图组主要超预算或粗到精修正过大。已保存四份各累积 16–17 帧并补偿里程计运动的扫描，便于在原始 bag 被清理后继续复现。

下面是这些长窗口扫描在完整地图、0.80 BBS 门限、7 m 量程、3 秒诊断预算下的额外 GICP 检查。全局结果均未接受；额外精配准仅用于诊断。

| 扫描 | 双侧一致结构对应 | 占有效对应比例 | 诊断质量原因 |
|---|---:|---:|---|
| bag_0_310 | 374 | 1.37% | insufficient_consistent_structure_fraction |
| bag_0_621 | 39 | 0.15% | inconsistent_source_target_structure |
| bag_1_310 | 646 | 2.53% | gicp_not_converged_before_iteration_limit |
| bag_1_620 | 703 | 2.81% | insufficient_consistent_structure_fraction |

这些样本说明，增加累积点数并不自动增加足够比例的可靠立体结构。当前发现了算法质量检查的明确漏洞并已修复；剩余现象仍涉及扫描/地图的结构一致性、可见范围与搜索配置，不能仅凭这些无真值样本断言唯一原因是地图不好。

## 数据与复现

- [主回归汇总](SUMMARY.md)：全部参数组、合成恢复耗时与误差。
- [重叠诊断](overlap_with_normal_check/SUMMARY.md)：保存扫描的筛选/量程试验。
- [较长窗口样本](long_window_real_fixtures/bag_scans/manifest.yaml)：四份新录制并运动补偿的 PCD。
- `extended_overlap_probe/parameters.yaml`：8 秒诊断的完整配置。
- `ros2 run rc_bbs_localization validate_overlap.py`：按独立 YAML 重跑保存样本。
- `ros2 run rc_bbs_localization validate_live_profiles.py`：按 YAML 重跑硬件窗口试验，结束后恢复生产配置。

本轮只修改新增的 BBS 包与独立入口配置、测试、文档，未修改旧定位/建图/导航代码、原 launch、URDF 或地图。
