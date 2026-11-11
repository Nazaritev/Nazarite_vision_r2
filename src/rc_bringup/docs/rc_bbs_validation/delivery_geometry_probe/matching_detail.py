from analyze import load,normals,base,sourcebase
from pathlib import Path
import numpy as np,yaml,json
from scipy.spatial import cKDTree
m=load(sourcebase/'map.pcd');tn,ev=normals(m,.45);tree=cKDTree(m);xy=cKDTree(m[:,:2])
poses=yaml.safe_load((sourcebase/'diagnostic_poses.yaml').read_text())['poses'];rows=[]
def centroid(a,leaf):
 _,ids,count=np.unique(np.floor(a/leaf),axis=0,return_inverse=True,return_counts=True);out=np.zeros((len(count),3));np.add.at(out,ids,a);return out/count[:,None]
for p in [p for p in poses if p['max_range_m']==7 and p['gravity_sign']==-1]:
 raw=load(sourcebase/(p['case']+'_range7_sensor.pcd'));T=np.array(p['T_map_sensor_row_major']).reshape(4,4)
 for leaf in [.1,.15,.2]:
  src=centroid(raw,leaf);sn,sev=normals(src,.45);sn=sn@T[:3,:3].T;world=src@T[:3,:3].T+T[:3,3];dist,ids=tree.query(world);xyd,_=xy.query(world[:,:2]);roi=xyd<=.35;valid=np.linalg.norm(sn,axis=1)>.5;sv=valid&(abs(sn[:,2])<.8);tv=(np.linalg.norm(tn[ids],axis=1)>.5)&(abs(tn[ids,2])<.8);compatible=abs((sn*tn[ids]).sum(axis=1))>=np.cos(np.pi/6);matched=roi&(dist<=.4)
  masks={'matched':matched,'source_nonground_matched':matched&sv,'target_nonground_matched':matched&tv,'bilateral_matched':matched&sv&tv&compatible,'gicp_near':roi&(dist<=.75),'source_nonground_inside':roi&sv}
  r={'case':p['case'],'initial_z':np.array(p['initial_T_map_sensor_row_major']).reshape(4,4)[2,3].item(),'leaf':leaf,'counts':{k:int(v.sum()) for k,v in masks.items()},'matched_source_nz_quantiles':np.quantile(abs(sn[matched,2]),[.1,.5,.9]).tolist()}
  if leaf==.2:
   # Boundary region has the clear two-level platform step on the map.
   for name,points in [('map',m),('source_world',world)]:
    region=points[(points[:,0]>1.5)&(points[:,0]<5.3)&(points[:,1]>-2.2)&(points[:,1]<-1.2)]
    r[name+'_step_region']={'count':len(region),'z_quantile':np.quantile(region[:,2],[0,.1,.5,.9,1]).tolist() if len(region) else []}
   roi_ids=np.where(roi)[0];rn,rev=normals(src[roi_ids],.45);rn=rn@T[:3,:3].T;changed=abs((rn*sn[roi_ids]).sum(axis=1))<np.cos(np.pi/6);r['roi_only_normal_change_over_30deg']=int(changed.sum())
  rows.append(r)
(base/'matching_detail.json').write_text(json.dumps(rows,indent=2));print(json.dumps(rows,indent=2))
