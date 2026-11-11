#!/usr/bin/python3
"""Summarize every attempted scan, preserving failed/expired attempts in timing statistics."""
from pathlib import Path
import csv
import json
from collections import defaultdict
import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory


def main():
    cfg=yaml.safe_load((Path(get_package_share_directory('rc_bringup'))/'config/reality/rc_localization_validation.yaml').read_text())
    out=Path(cfg['output_dir'])
    rows=list(csv.DictReader((out/'benchmark.csv').open()))
    grouped=defaultdict(list)
    for row in rows:grouped[row['profile']].append(row)
    summary=[]
    for profile,values in grouped.items():
        synthetic=[r for r in values if r['has_truth']=='1']
        accepted=[r for r in synthetic if r['accepted']=='1']
        real=[r for r in values if r['has_truth']=='0']
        times=[float(r['search_ms']) for r in synthetic]
        summary.append({
            'profile':profile,'synthetic_success':len(accepted),'synthetic_attempts':len(synthetic),
            'all_attempts_median_ms':float(np.median(times)) if times else None,
            'all_attempts_p95_ms':float(np.percentile(times,95)) if times else None,
            'all_attempts_max_ms':max(times,default=None),
            'accepted_max_translation_error_m':max((float(r['translation_error_m']) for r in accepted),default=None),
            'real_quality_accepted':sum(r['accepted']=='1' for r in real),'real_attempts':len(real),
            'cold_map_ms':float(values[0]['cold_map_ms']),
        })
    (out/'summary.json').write_text(json.dumps(summary,indent=2))
    lines=['# 自动汇总（真实尝试数，失败也计入耗时分位数）','',
           '合成扫描来自地图裁剪，含噪声和离群点；其真值只供测试评分。真实扫描没有位姿真值。',
           'search_ms 是已预处理点云上的 BBS + GICP 耗时；不含冷构图、扫描累积和后续多帧验证。',
           '这里的候选通过不等于定位成功；还必须通过 ROS 节点的连续新帧验证。','',
           '| 参数组 | 合成候选通过/尝试 | 全部合成尝试 P95 / ms | 接受结果最大位置误差 / m | 真实扫描候选通过/尝试 |',
           '|---|---:|---:|---:|---:|']
    for s in summary:
        p95='—' if s['all_attempts_p95_ms'] is None else f"{s['all_attempts_p95_ms']:.1f}"
        error='—' if s['accepted_max_translation_error_m'] is None else f"{s['accepted_max_translation_error_m']:.4f}"
        lines.append(f"| {s['profile']} | {s['synthetic_success']}/{s['synthetic_attempts']} | {p95} | {error} | {s['real_quality_accepted']}/{s['real_attempts']} |")
    baseline=next((s for s in summary if s['profile']=='raw_map_filter_baseline'),None)
    optimized=next((s for s in summary if s['profile']=='strict_1400ms'),None)
    if baseline and optimized:
        lines+=['','## 同版本地图预处理对照','',
                '两组使用相同扫描和质量门限，仅切换 map_filter_voxel_support。冷地图时间为每组一次构建测量，非重复测量分位数。','',
                '| 地图预处理 | 冷构图 / ms | 合成候选通过/尝试 |',
                '|---|---:|---:|',
                f"| 原始点逐点滤波 | {baseline['cold_map_ms']:.1f} | {baseline['synthetic_success']}/{baseline['synthetic_attempts']} |",
                f"| 体素密度证明与稀疏点查询 | {optimized['cold_map_ms']:.1f} | {optimized['synthetic_success']}/{optimized['synthetic_attempts']} |"]
    ros=json.loads((out/'ros_validation.json').read_text())
    lines+=['','## 实际 ROS 节点的合成输入测试','',
            'unknown 包含进程启动和首次地图构建；其他阶段从触发动作到首个通过多帧验证的位姿计时。','',
            '| 场景 | 首个验证位姿 / s |','|---|---:|']
    for k,v in ros['first_pose_seconds'].items():lines.append(f'| {k} | {v:.3f} |')
    lines+=['',f"最大位置误差：{ros['max_translation_error_m']:.4f} m；最大旋转误差：{ros['max_rotation_error_rad']:.4f} rad。",'',
            '实际机器人录制得到扫描，但没有获得可信全局位姿；详见 live_validation.json 和 README.md。',
            '双侧法线结构检查、量程诊断与较长累积窗口实机试验见 [STRUCTURE_DIAGNOSIS.md](STRUCTURE_DIAGNOSIS.md)。']
    fast_path=out/'ros_profiles/known_gravity_fast/ros_validation.json'
    if fast_path.is_file():
        fast=json.loads(fast_path.read_text())
        lines+=['','## 已知地图重力方向的快速参数组（仅合成输入已知此先验）','',
                '仅缩小经过合成数据生成器确定的重力先验，不提供任何全局位置初值；生产默认仍保留未知地图方向的稳健搜索。','',
                '| 场景 | 首个验证位姿 / s |','|---|---:|']
        for k,v in fast['first_pose_seconds'].items():lines.append(f'| {k} | {v:.3f} |')
    (out/'SUMMARY.md').write_text('\n'.join(lines)+'\n')
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig,axes=plt.subplots(1,2,figsize=(13,5),layout='constrained')
        for i,(profile,values) in enumerate(grouped.items()):
            for r in values:
                if r['has_truth']!='1':continue
                axes[0].scatter(float(r['search_ms']),i,marker='o' if r['accepted']=='1' else 'x',s=25,
                                color='#2075ad' if r['accepted']=='1' else '#c74932')
        axes[0].set_yticks(range(len(grouped)),list(grouped))
        axes[0].axvline(700,ls='--',color='gray',lw=1)
        axes[0].set(xlabel='BBS + GICP time (ms)',title='Synthetic scans: circle=accepted, x=rejected')
        phases=ros['first_pose_seconds']
        axes[1].barh(list(phases),list(phases.values()),color='#2075ad')
        axes[1].axvline(1.,ls='--',color='#c74932',lw=1)
        axes[1].set(xlabel='Time to verified ROS pose (s)',title='Full ROS pipeline on synthetic input')
        fig.savefig(out/'timing.png',dpi=150)
        plt.close(fig)
    except ImportError:
        pass
    print((out/'SUMMARY.md').read_text())


if __name__=='__main__':main()
