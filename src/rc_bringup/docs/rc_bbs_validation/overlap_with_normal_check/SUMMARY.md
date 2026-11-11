# 实机扫描重叠与结构证据诊断

真实扫描没有独立位姿真值。候选通过不等于定位成功；本工具不发布位姿，也不替代连续新帧验证。

质量门限新增双侧结构检查：扫描和地图法线都必须为非地面，且法线锐角差不超过 30°，至少 120 个有效对应，并占全部有效对应的至少 5%。
原有地图非地面占比、覆盖率、残差、收敛、歧义、搜索完整性和预算检查仍然执行。

| 参数组 | 合成候选通过/尝试 | 真实扫描候选通过/尝试 | 真实扫描拒绝原因 |
|---|---:|---:|---|
| current_default | 6/6 | 0/5 | no_bbs_candidate: 3, insufficient_bbs_geometry_16: 1, insufficient_bbs_geometry_32: 1 |
| full_map_080_range30 | 0/6 | 0/5 | candidate_cap_ambiguity_unresolved: 1, insufficient_bbs_geometry_16: 1, insufficient_bbs_geometry_32: 1, inconsistent_source_target_structure: 1, search_timeout_or_queue_limit: 1 |
| full_map_080_range7 | 0/6 | 0/5 | candidate_cap_ambiguity_unresolved: 1, insufficient_bbs_geometry_16: 2, excessive_coarse_refine_correction: 2 |
| full_map_075_range7 | 0/6 | 0/5 | candidate_cap_ambiguity_unresolved: 1, insufficient_bbs_geometry_16: 2, search_timeout_or_queue_limit: 2 |
| full_map_080_range3 | 0/6 | 0/5 | insufficient_bbs_geometry_22: 1, insufficient_bbs_geometry_12: 1, insufficient_bbs_geometry_6: 1, insufficient_bbs_geometry_26: 1, insufficient_bbs_geometry_33: 1 |

## 曾通过旧质量检查的候选

以下候选在保留完整地图、BBS 门限 0.80、量程 7 m 的诊断组中，覆盖率和地图单侧非地面占比看起来可用。双侧结构证据却不足。

| 扫描 | 覆盖率 | 地图单侧非地面对应（约） | 双侧一致结构对应 | 判定 |
|---|---:|---:|---:|---|
| bag_2_297 | 0.859 | 2923 | 31 | inconsistent_source_target_structure |
| bag_2_594 | 0.859 | 2819 | 6 | inconsistent_source_target_structure |
| bag_2_594 | 0.832 | 3631 | 0 | inconsistent_source_target_structure |

范围调参现在对每组重新做扫描清理；可从 benchmark.csv 的 valid_source 核对点数变化。
源扫描坐标、地图、BBS 结构点保存在 diagnostic_clouds/；诊断精配准位姿保存在 diagnostic_poses.yaml。
诊断中对不完整搜索的候选另做 GICP，仅用于检查失败原因，不改变全局接受标志。

完整地图和较低门限在合成场景中也会增加模式数或超时；这些试验参数没有替换生产默认。
目前证据表明部分看似良好的配准缺少源扫描与地图共同的立体结构，不能据此给出准确全局位置。
