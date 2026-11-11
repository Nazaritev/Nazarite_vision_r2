# 无先验全局搜索交付诊断

仅使用当前采集的 bag_0_425，全部全局 XYZ/Yaw 搜索范围与生产配置相同；没有使用用户位置做初值或缩域。每组预算 2.2–2.5 秒，14 组总搜索耗时约 17.1 秒。地图构建和额外诊断 GICP 耗时另计。

|组|搜索 ms|完整|候选接受|原因|诊断位姿距用户标注 1m 内|
|---|---:|---|---|---|---:|
|normal_range3|1926|False|False|candidate_cap_ambiguity_unresolved|0|
|full_range3|2116|False|False|candidate_cap_ambiguity_unresolved|0|
|normal_range5|2200|False|False|search_timeout_or_queue_limit|0|
|full_range5|1911|False|False|candidate_cap_ambiguity_unresolved|0|
|normal_range7|2199|False|False|search_timeout_or_queue_limit|0|
|full_range7|1900|False|False|search_timeout_or_queue_limit|0|
|coarse_range5_sign+1|572|False|False|candidate_cap_ambiguity_unresolved|0|
|coarse_range5_sign-1|882|False|False|candidate_cap_ambiguity_unresolved|0|
|coarse_range7_sign+1|876|False|False|candidate_cap_ambiguity_unresolved|0|
|coarse_range7_sign-1|1031|False|False|candidate_cap_ambiguity_unresolved|0|
|coarse_range7_sign+1_threshold0.6|849|True|True|candidate_requires_fresh_multiframe_validation|0|
|coarse_range7_sign+1_threshold0.7|116|True|False|insufficient_static_gicp_support|0|
|coarse_range7_sign-1_threshold0.6|470|True|False|ground_dominated|0|
|coarse_range7_sign-1_threshold0.7|49|True|False|no_bbs_candidate_after_partial_overlap_search|0|

原始分辨率 .30m 下，量程 3/5/7m、筛选/完整地图均没有得到用户位置附近的合格全局候选；短量程尤其容易出现多个高命中率退化模式。
.45m 粗化后枚举加快，但仍有多个不同模式。额外 .60/.70 命中阈值试验中，+Z/.60 得到完整搜索且单帧候选通过；精配准 footprint 约 (5.35,-3.65)m，距用户约 4m，与其标注不符。这一反例未通过新鲜多帧验证，也不能解释为实机准确定位成功。

用户标注为近似人工参考，不是测量真值。这些有限参数试验不能证明所有配置无可用解。没有修改生产默认或核心代码。细节见 parameters.yaml、benchmark.csv、diagnostic_poses.yaml 以及两个子目录。
