#!/usr/bin/python3
"""Reproduce saved-scan overlap diagnostics; all settings come from YAML."""
import csv
import json
import os
from pathlib import Path
import subprocess
import tempfile
from collections import Counter

import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory


def summarize(config_path):
    cfg = yaml.safe_load(Path(config_path).read_text())
    out = Path(cfg['output_dir'])
    rows = list(csv.DictReader((out / 'benchmark.csv').open()))
    candidates = list(csv.DictReader((out / 'diagnostic_candidates.csv').open()))
    lines = [
        '# 实机扫描重叠与结构证据诊断', '',
        '真实扫描没有独立位姿真值。候选通过不等于定位成功；本工具不发布位姿，也不替代连续新帧验证。', '',
        '质量门限新增双侧结构检查：扫描和地图法线都必须为非地面，且法线锐角差不超过 30°，至少 120 个有效对应，并占全部有效对应的至少 5%。',
        '原有地图非地面占比、覆盖率、残差、收敛、歧义、搜索完整性和预算检查仍然执行。', '',
        '| 参数组 | 合成候选通过/尝试 | 真实扫描候选通过/尝试 | 真实扫描拒绝原因 |',
        '|---|---:|---:|---|',
    ]
    summary = []
    for profile in cfg['profiles']:
        name = profile['name']
        selected = [r for r in rows if r['profile'] == name]
        synthetic = [r for r in selected if r['has_truth'] == '1']
        real = [r for r in selected if r['has_truth'] == '0']
        reasons = dict(Counter(r['reason'] for r in real))
        ns = sum(r['accepted'] == '1' for r in synthetic)
        nr = sum(r['accepted'] == '1' for r in real)
        summary.append(dict(profile=name, synthetic_accepted=ns, synthetic_attempts=len(synthetic),
                            real_accepted=nr, real_attempts=len(real), real_reasons=reasons))
        detail = ', '.join(f'{k}: {v}' for k, v in reasons.items())
        lines.append(f'| {name} | {ns}/{len(synthetic)} | {nr}/{len(real)} | {detail} |')
    lines += ['', '## 曾通过旧质量检查的候选', '',
              '以下候选在保留完整地图、BBS 门限 0.80、量程 7 m 的诊断组中，覆盖率和地图单侧非地面占比看起来可用。双侧结构证据却不足。', '',
              '| 扫描 | 覆盖率 | 地图单侧非地面对应（约） | 双侧一致结构对应 | 判定 |',
              '|---|---:|---:|---:|---|']
    examples = [r for r in candidates if r['profile'] == 'full_map_080_range7'
                and r['case'].startswith('bag_2') and r['quality_reason'] == 'inconsistent_source_target_structure']
    for r in examples:
        single = round(float(r['nonground_fraction']) * int(r['correspondences']))
        lines.append(f"| {r['case']} | {float(r['coverage']):.3f} | {single} | "
                     f"{r['consistent_structure_correspondences']} | {r['quality_reason']} |")
    lines += ['', '范围调参现在对每组重新做扫描清理；可从 benchmark.csv 的 valid_source 核对点数变化。',
              '源扫描坐标、地图、BBS 结构点保存在 diagnostic_clouds/；诊断精配准位姿保存在 diagnostic_poses.yaml。',
              '诊断中对不完整搜索的候选另做 GICP，仅用于检查失败原因，不改变全局接受标志。', '',
              '完整地图和较低门限在合成场景中也会增加模式数或超时；这些试验参数没有替换生产默认。',
              '目前证据表明部分看似良好的配准缺少源扫描与地图共同的立体结构，不能据此给出准确全局位置。']
    (out / 'SUMMARY.md').write_text('\n'.join(lines) + '\n')
    (out / 'summary.json').write_text(json.dumps(summary, indent=2))
    # A portable image makes the discrepancy visible without a running ROS graph.
    if examples:
        os.environ.setdefault('MPLCONFIGDIR', str(Path(tempfile.gettempdir()) / 'rc_bbs_matplotlib'))
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(9, 4))
        xs = range(len(examples))
        ax.bar([x - .18 for x in xs], [round(float(r['nonground_fraction']) * int(r['correspondences'])) for r in examples],
               width=.36, label='Map-only non-ground evidence', color='#94a3b8')
        ax.bar([x + .18 for x in xs], [int(r['consistent_structure_correspondences']) for r in examples],
               width=.36, label='Source + map consistent structure', color='#0f766e')
        ax.axhline(120, color='#b91c1c', linestyle='--', label='Required consistent correspondences')
        ax.set_xticks(list(xs), [f"{r['case']} / {i+1}" for i, r in enumerate(examples)])
        ax.set_ylabel('Correspondence count')
        ax.set_title('High map-only counts can hide missing source structure')
        ax.legend(fontsize=8)
        fig.tight_layout()
        fig.savefig(out / 'structure_evidence.png', dpi=160)
        plt.close(fig)
    print((out / 'SUMMARY.md').read_text())


def main():
    share = Path(get_package_share_directory('rc_bringup'))
    config = share / 'config/reality/rc_localization_overlap_validation.yaml'
    binary = Path(get_package_prefix('rc_bbs_localization')) / 'lib/rc_bbs_localization/bbs_benchmark'
    subprocess.run([str(binary), str(config)], check=True, env=dict(os.environ, OMP_WAIT_POLICY='PASSIVE'))
    summarize(config)


if __name__ == '__main__':
    main()
