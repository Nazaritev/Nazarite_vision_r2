from pathlib import Path
import numpy as np,yaml,json
from scipy.spatial import cKDTree
base=Path(__file__).resolve().parent
sourcebase=base.parent/'user_marked_pose/obstacle_filter'
def load(p):
 with Path(p).open('rb') as f:
  while not f.readline().startswith(b'DATA'):pass
  p=np.fromfile(f,'<f4').reshape(-1,3)
 return p[np.isfinite(p).all(axis=1)]
def voxel(p,leaf):
 _,ids=np.unique(np.floor(p/leaf),axis=0,return_index=True);return p[ids]
def normals(p,r):
 n=np.zeros_like(p);ev=np.zeros_like(p);tree=cKDTree(p)
 for k,ids in enumerate(tree.query_ball_point(p,r)):
  if len(ids)<6:continue
  d=p[ids]-p[ids].mean(axis=0);v,u=np.linalg.eigh(d.T@d)
  if v[1]>1e-8:n[k]=u[:,0];ev[k]=v
 return n,ev
map=load(sourcebase/'map.pcd');tree=cKDTree(map);xytree=cKDTree(map[:,:2]);tn,tev=normals(map,.45)
poses=yaml.safe_load((sourcebase/'diagnostic_poses.yaml').read_text())['poses'];results=[]
for case in ['bag_0_226','bag_0_425']:
 raw=load(sourcebase/(case+'_range7_sensor.pcd'));sm=voxel(raw,.2)
 for p in [p for p in poses if p['case']==case and p['max_range_m']==7 and p['gravity_sign']==-1]:
  T=np.array(p['T_map_sensor_row_major']).reshape(4,4);world=sm@T[:3,:3].T+T[:3,3];d,ids=tree.query(world);xy,_=xytree.query(world[:,:2]);roi=xy<=.35;close=d<=.75;match=d<=.4
  row={'case':case,'initial_sensor_z_m':np.array(p['initial_T_map_sensor_row_major']).reshape(4,4)[2,3].item(),'voxel_source_count':len(sm),'roi_count':int(roi.sum()),'static_distance_count':int((roi&close).sum()),'actual_gicp_points':p['registration_source_points'],'normal_radii':[]}
  for radius in [.2,.3,.45,.6]:
   sn,sev=normals(sm,radius);sn=sn@T[:3,:3].T
   valid=np.linalg.norm(sn,axis=1)>.5;sv=valid&(np.abs(sn[:,2])<.8);tv=(np.linalg.norm(tn[ids],axis=1)>.5)&(np.abs(tn[ids,2])<.8);dot=np.abs((sn*tn[ids]).sum(axis=1));compatible=dot>=np.cos(np.pi/6)
   row['normal_radii'].append({'radius_m':radius,'valid_normals':int(valid.sum()),'source_nonground':int(sv.sum()),'source_nonground_inside_roi':int((sv&roi).sum()),'source_nonground_static_distance':int((sv&roi&close).sum()),'bilateral_matched_nonground_voxels':int((sv&tv&compatible&roi&match).sum()),'normal_rejected_static_near':int((roi&close&valid&(np.linalg.norm(tn[ids],axis=1)>.5)&~compatible).sum()),'kept_gicp_like':int((roi&close&(~valid|(np.linalg.norm(tn[ids],axis=1)<.5)|compatible)).sum()),'median_planarity_nonground':float(np.median((sev[:,1]/np.maximum(sev[:,2],1e-12))[sv])) if sv.any() else None})
  results.append(row)
# Plane fits in user-visible platform areas, no semantics assumed.
planes=[]
for name,x0,x1,y0,y1 in [('upper_platform',0,5,-1.5,.6),('middle_platform',1.7,3,-7,-2),('lower_left',-.8,1,-9,-4)]:
 pts=map[(map[:,0]>x0)&(map[:,0]<x1)&(map[:,1]>y0)&(map[:,1]<y1)]
 A=np.c_[pts[:,:2],np.ones(len(pts))];fit=np.linalg.lstsq(A,pts[:,2],rcond=None)[0];res=pts[:,2]-A@fit
 planes.append({'region':name,'points':len(pts),'z_ax_by_c':fit.tolist(),'median_abs_residual_m':float(np.median(np.abs(res))),'tilt_rad':float(np.arctan(np.linalg.norm(fit[:2]))),'z_quantile_10_50_90':np.quantile(pts[:,2],[.1,.5,.9]).tolist()})
(base/'geometry.json').write_text(json.dumps({'diagnostic_only':True,'normal_analysis':results,'map_planes':planes},indent=2));print(json.dumps({'normal_analysis':results,'map_planes':planes},indent=2))
