#!/usr/bin/python3
from pathlib import Path
import csv,json
import numpy as np,yaml
import matplotlib
matplotlib.use('Agg')
from matplotlib import pyplot as plt,font_manager
from matplotlib.ticker import MultipleLocator
from scipy.spatial import cKDTree
base=Path(__file__).resolve().parent
out=base/'current_capture/probe'
font_manager.fontManager.addfont('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
plt.rcParams.update({'font.family':'Noto Sans CJK JP','axes.unicode_minus':False,'font.size':10})
def read(p):
 with p.open('rb') as f:
  fields=None
  while True:
   line=f.readline().decode().strip()
   if line.startswith('FIELDS'):fields=line.split()[1:]
   if line.startswith('DATA'):
    assert line=='DATA binary' and fields==['x','y','z'];break
  a=np.fromfile(f,'<f4').reshape(-1,3)
 return a[np.isfinite(a).all(axis=1)]
def small(a,leaf=.15):
 _,ids=np.unique(np.floor(a/leaf),axis=0,return_index=True);return a[ids]
def structure(a):
 tree=cKDTree(a);mask=np.zeros(len(a),bool)
 for i,ids in enumerate(tree.query_ball_point(a,.45)):
  if len(ids)<6:continue
  d=a[ids]-a[ids].mean(axis=0);v,n=np.linalg.eigh(d.T@d)
  mask[i]=v[1]>1e-8 and abs(n[2,0])<.8
 return mask
mapcloud=read(out/'map.pcd');mapmask=structure(mapcloud)
poses=yaml.safe_load((out/'diagnostic_poses.yaml').read_text())['poses']
rows=list(csv.DictReader((out/'probe.csv').open()))
case=poses[0]['case']
fig,axes=plt.subplots(2,2,figsize=(14,12));fig.subplots_adjust(top=.89,bottom=.09,hspace=.30,wspace=.22)
summaries=[]
for j,sign in enumerate([1.,-1.]):
 options=[(r,p) for r,p in zip([r for r in rows if r['stage']=='refined'],poses) if p['case']==case and p['max_range_m']==7 and p['gravity_sign']==sign]
 row,pose=min(options,key=lambda v:float(v[0]['rmse_m']))
 sensor=small(read(out/(case+'_range7_sensor.pcd')),.20);T=np.array(pose['T_map_sensor_row_major']).reshape(4,4);points=sensor@T[:3,:3].T+T[:3,3];mask=structure(points)
 for i,(a,b) in enumerate([(0,1),(1,2)]):
  ax=axes[i,j];ax.scatter(mapcloud[:,a],mapcloud[:,b],s=2,c='#a9b2bd',alpha=.40,label='地图点云')
  ax.scatter(points[:,a],points[:,b],s=3,c='#f97316',alpha=.40,label='当前扫描（诊断精配准）')
  ax.scatter(mapcloud[mapmask,a],mapcloud[mapmask,b],s=7,c='#334155',alpha=.8,label='地图立体结构')
  ax.scatter(points[mask,a],points[mask,b],s=7,c='#dc2626',alpha=.85,label='扫描立体结构')
  ax.grid(alpha=.3);ax.set_xlabel('XYZ'[a]+' / 米');ax.set_ylabel('XYZ'[b]+' / 米')
  if i==0:
   ax.set_xlim(-2,8);ax.set_ylim(-11,4);ax.set_aspect('equal');ax.plot(3.7,0,'*',color='#2563eb',markersize=13,label='用户标注约 (3.7, 0)');ax.annotate('',xy=(3.7,-1.4),xytext=(3.7,0),arrowprops=dict(arrowstyle='-|>',color='#2563eb',lw=2))
   ax.set_title(f'地图重力方向 {sign:+.0f}Z：俯视 XY\n覆盖率 {float(row["coverage"]):.1%}，双侧一致结构 {float(row["consistent_structure_fraction"]):.2%}')
  else:
   ax.set_xlim(-11,4);ax.set_ylim(-2,3);ax.set_title('侧视 YZ：观察高度与结构差异')
 axes[0,j].legend(fontsize=8,loc='lower left')
 summaries.append({'case':case,'gravity_sign':sign,'range_m':7,'coverage':float(row['coverage']),'consistent_structure_fraction':float(row['consistent_structure_fraction']),'rmse_m':float(row['rmse_m']),'quality_reason':row['quality_reason'],'source_xy_outside_map_bounds_fraction':float(np.mean(np.any((points[:,:2]<mapcloud[:,:2].min(axis=0))|(points[:,:2]>mapcloud[:,:2].max(axis=0)),axis=1)))})
fig.suptitle('用户标注位置附近：当前扫描与地图对照',fontsize=20,fontweight='bold')
fig.text(.5,.925,'两种重力方向均从用户标注 XY 和车头方向开始扫描高度，再作局部精配准；仅用于诊断。',ha='center',fontsize=11)
fig.text(.5,.035,'红/深灰立体结构为显示用法线分类；数值指标由实际定位核心计算。\n用户标注是近似参考；上述结果均未接受为全局定位，也未写入生产初值。',ha='center',fontsize=11)
fig.savefig(out/'current_scan_overlay.png',dpi=160);plt.close(fig)
(out/'overlay_summary.json').write_text(json.dumps(summaries,ensure_ascii=False,indent=2))
print(json.dumps(summaries,ensure_ascii=False,indent=2))
