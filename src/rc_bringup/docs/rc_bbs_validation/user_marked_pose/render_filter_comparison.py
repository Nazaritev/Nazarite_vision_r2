#!/usr/bin/python3
"""Render recorded baseline/filter diagnostics; settings are in a YAML file."""
from pathlib import Path
import csv
import json
import sys
import numpy as np
import yaml
import matplotlib
matplotlib.use('Agg')
from matplotlib import pyplot as plt, font_manager


def read_xyz(path):
    with Path(path).open('rb') as stream:
        fields = None
        while True:
            raw = stream.readline()
            if not raw:
                raise ValueError(f'Incomplete PCD header: {path}')
            line = raw.decode().strip()
            if line.startswith('FIELDS '):
                fields = line.split()[1:]
            if line.startswith('DATA '):
                if line != 'DATA binary' or fields != ['x', 'y', 'z']:
                    raise ValueError(f'Expected binary float32 XYZ PCD: {path}')
                break
        xyz = np.fromfile(stream, '<f4').reshape(-1, 3)
    return xyz[np.isfinite(xyz).all(axis=1)]


def display_voxel(xyz, leaf):
    _, ids = np.unique(np.floor(xyz / leaf), axis=0, return_index=True)
    return xyz[ids]


def chosen(directory, settings):
    poses = yaml.safe_load((directory / 'diagnostic_poses.yaml').read_text())['poses']
    rows = [r for r in csv.DictReader((directory / 'probe.csv').open()) if r['stage'] == 'refined']
    if len(rows) != len(poses):
        raise ValueError('Pose/CSV count mismatch')
    candidates = [(r, p) for r, p in zip(rows, poses)
                  if p['case'] == settings['case_id']
                  and p['max_range_m'] == settings['max_range_m']
                  and p['gravity_sign'] == settings['gravity_sign']]
    if not candidates:
        raise ValueError(f'No selected diagnostic candidate in {directory}')
    # This only selects a representative local diagnostic for the figure.
    # It neither changes global acceptance nor declares independent pose truth.
    row, pose = max(candidates, key=lambda v: (int(v[0]['local_quality_pass']),
                    float(v[0]['consistent_structure_fraction']), -float(v[0]['rmse_m'])))
    sensor = read_xyz(directory / f"{settings['case_id']}_range{int(settings['max_range_m'])}_sensor.pcd")
    transform = np.array(pose['T_map_sensor_row_major']).reshape(4, 4)
    all_points = display_voxel(sensor, settings['display_voxel_m'])
    all_points = all_points @ transform[:3, :3].T + transform[:3, 3]
    if 'registration_source_map_pcd' in pose:
        used = read_xyz(pose['registration_source_map_pcd'])
    else:
        # The original baseline used all downsampled points, before the filter existed.
        # Representative display voxels may differ slightly from PCL voxel centroids.
        used = all_points
    return row, pose, all_points, used


def main():
    base = Path(__file__).resolve().parent
    config = Path(sys.argv[1]) if len(sys.argv) > 1 else base / 'filter_comparison.yaml'
    settings = yaml.safe_load(config.read_text())
    output = Path(settings['output_dir'])
    output.mkdir(parents=True, exist_ok=True)
    font_manager.fontManager.addfont('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
    plt.rcParams.update({'font.family': 'Noto Sans CJK JP', 'axes.unicode_minus': False, 'font.size': 11})
    fig, axes = plt.subplots(2, 2, figsize=(14, 12))
    fig.subplots_adjust(top=.84, bottom=.09, hspace=.3, wspace=.23)
    summary = {'diagnostic_only': True, 'global_localization_accepted': False,
               'candidate_selection': 'local_quality_pass, then largest bilateral structure fraction, then smallest RMSE',
               'user_reference_xy_m': settings['approximate_footprint_xy_m'],
               'user_reference_heading_rad': settings['approximate_footprint_heading_rad'],
               'case': settings['case_id'], 'gravity_sign': settings['gravity_sign'],
               'max_range_m': settings['max_range_m'], 'comparisons': []}
    for column, (label, key) in enumerate([('过滤前', 'baseline_dir'), ('过滤后', 'filtered_dir')]):
        directory = Path(settings[key])
        row, pose, all_points, used = chosen(directory, settings)
        mapcloud = read_xyz(directory / 'map.pcd')
        for line, (x, y) in enumerate([(0, 1), (1, 2)]):
            ax = axes[line, column]
            ax.scatter(mapcloud[:, x], mapcloud[:, y], c='#4b5563', s=3, alpha=.48, label='静态赛场地图')
            ax.scatter(all_points[:, x], all_points[:, y], c='#f97316', s=4, alpha=.35,
                       label='当前扫描（全部有效点）')
            if column == 1:
                ax.scatter(used[:, x], used[:, y], c='#0f766e', s=8, alpha=.85,
                           label='实际用于 GICP 的地图支持点')
            ax.set_xlabel('XYZ'[x] + ' / 米')
            ax.set_ylabel('XYZ'[y] + ' / 米')
            ax.grid(alpha=.25)
            if line == 0:
                ax.set_xlim(*settings['xy_xlim_m'])
                ax.set_ylim(*settings['xy_ylim_m'])
                ax.set_aspect('equal')
                px, py = settings['approximate_footprint_xy_m']
                heading = settings['approximate_footprint_heading_rad']
                ax.scatter([px], [py], marker='*', s=120, c='#2563eb', label='用户标注参考')
                ax.annotate('', xy=(px + 1.2*np.cos(heading), py + 1.2*np.sin(heading)),
                            xytext=(px, py), arrowprops={'arrowstyle': '-|>', 'color': '#2563eb', 'lw': 2})
                text = f"{label}：俯视 XY\n全扫描覆盖率 {float(row['coverage']):.1%}；双侧结构 {float(row['consistent_structure_fraction']):.2%}"
                if column == 1:
                    text += f"\n地图范围内覆盖率 {float(row['roi_coverage']):.1%}；独立结构体素 {row['unique_structure_voxels']}"
                ax.set_title(text, fontsize=12)
                ax.legend(fontsize=8, loc='lower left')
            else:
                ax.set_xlim(*settings['yz_xlim_m'])
                ax.set_ylim(*settings['yz_ylim_m'])
                ax.set_title(f'{label}：侧视 YZ')
        comparison = dict(row)
        comparison.update(label=label, diagnostic_pose=pose,
                          display_source_points=len(all_points), registration_source_points=len(used))
        summary['comparisons'].append(comparison)
    fig.suptitle('未建图障碍物过滤：同一份当前扫描的局部诊断对照', fontsize=19, fontweight='bold')
    fig.text(.5, .925,
             f"从用户标注位置/朝向开始，分别扫描高度并局部精配准；显示地图重力 {settings['gravity_sign']:+.0f}Z 假设。",
             ha='center', fontsize=11)
    fig.text(.5, .035, '橙色保留全部扫描用于对照；绿色是核心实际选择给 GICP 的点。\n用户参考不是位姿真值；此图不表示未知初值 BBS 成功或多帧验证通过。', ha='center', fontsize=11)
    suffix = settings.get('output_suffix', '')
    fig.savefig(output / f'obstacle_filter_comparison{suffix}.png', dpi=170)
    plt.close(fig)
    (output / f'comparison_summary{suffix}.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2))
    # A large single view shows which observed arena surfaces survive the
    # actual registration filter, without hiding raw clutter in the comparison.
    directory = Path(settings['filtered_dir'])
    row, pose, all_points, used = chosen(directory, settings)
    mapcloud = read_xyz(directory / 'map.pcd')
    fig, ax = plt.subplots(figsize=(9, 12))
    fig.subplots_adjust(left=.12, right=.96, bottom=.12, top=.86)
    ax.scatter(mapcloud[:, 0], mapcloud[:, 1], c='#64748b', s=7, alpha=.5, label='静态赛场地图')
    ax.scatter(used[:, 0], used[:, 1], c='#059669', s=13, alpha=.85, label='实际用于 GICP 的保留扫描')
    px, py = settings['approximate_footprint_xy_m']
    heading = settings['approximate_footprint_heading_rad']
    ax.scatter([px], [py], marker='*', s=180, c='#2563eb', label='用户标注的近似位置')
    ax.annotate('', xy=(px + 1.3*np.cos(heading), py + 1.3*np.sin(heading)),
                xytext=(px, py), arrowprops={'arrowstyle': '-|>', 'color': '#2563eb', 'lw': 2.5})
    ax.set_aspect('equal'); ax.set_xlim(-1.5, 6.8); ax.set_ylim(-10.8, 1.5)
    ax.set_xlabel('地图 X / 米'); ax.set_ylabel('地图 Y / 米'); ax.grid(alpha=.3)
    ax.legend(loc='lower left', fontsize=11)
    fig.suptitle('用户位置辅助局部配准：过滤后点云', fontsize=19, fontweight='bold', y=.97)
    fig.text(.5, .91,
             f"真实扫描 {settings['case_id']} · 地图 {settings['gravity_sign']:+.0f}Z 假设 · {settings['max_range_m']:g} 米量程\n地图范围内覆盖率 {float(row['roi_coverage']):.1%}；双侧一致结构 {float(row['consistent_structure_fraction']):.2%}",
             ha='center', fontsize=12)
    fig.text(.5, .045, '绿色为核心实际选给 GICP 的点，灰色为静态赛场地图。\n起点来自用户标注，未接受为未知初值全局定位，也未通过多帧验证。',
             ha='center', fontsize=11)
    fig.savefig(output / f'filtered_arena_xy{suffix}.png', dpi=190); plt.close(fig)
    print(output / f'obstacle_filter_comparison{suffix}.png')


if __name__ == '__main__':
    main()
