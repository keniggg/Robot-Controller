import json,hashlib
from pathlib import Path
import numpy as np,cv2
from scipy.spatial import cKDTree
from alicia_grasp_modes.tabletop import Config,_project
from alicia_grasp_modes.measured_mask_seeds import local_surface_normals
B=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_restart_20261010T002347Z/gen24_analysis');assert Path('/media/zhuyupei/SanStand').is_mount()
d=dict(np.load(B/'fragment_ownership/masks.npz'));m=json.loads((B/'replay_inputs/manifest.json').read_text());r=next(x for x in m['files']if x['role']=='first_failure');f=dict(np.load(r['path']));k=f['K'];k=[k[0,0],k[1,1],k[0,2],k[1,2]]if k.shape==(3,3)else k
p=_project(d['depth'],k);valid=np.isfinite(d['depth'])&(d['depth']>=Config().depth_min_m)&(d['depth']<=Config().depth_max_m);normals,reliable=local_surface_normals(p,d['depth'],valid)
rep=json.loads((B/'fragment_ownership/first_failure_replay.json').read_text());plane=np.asarray(rep['independent'][0]['rgb_metrics']['support_plane_base']);pc=f['T_base_camera'].T@plane;cos=np.abs(normals@pc[:3]);lost=d['cores'][1]&~d['instances'][0]
subsets={'native0':d['native'][0],'native1':d['native'][1],'core0':d['cores'][0],'core1':d['cores'][1],'unowned':lost,'core1_owned':d['cores'][1]&d['instances'][0]}
res={}
for name,mask in subsets.items():
 xyz=p[mask];reliablemask=mask&reliable;near=mask&(d['heights']>.017)
 res[name]=dict(count=int(mask.sum()),reliable_count=int(reliablemask.sum()),angle_to_support_deg_q=np.percentile(np.degrees(np.arccos(np.clip(cos[reliablemask],0,1))),[0,10,50,90,100]).tolist()if reliablemask.any()else[],camera_xyz_bounds=[xyz.min(0).tolist(),xyz.max(0).tolist()],above17mm=int(near.sum()))
A=p[d['native'][0]];C=p[d['native'][1]];dist,idx=cKDTree(A).query(C)
res['native_pair_nearest_mm']=np.percentile(dist*1000,[0,1,5,10,50,100]).tolist()
res['native1_points_within_existing_edge']={str(x):int(np.count_nonzero(dist<=x))for x in [.002,.003,.004,.006,.008]}
for side in [0,1]:
 mask=d['native'][side];other=d['native'][1-side];dd=cv2.distanceTransform((~other).astype('uint8'),cv2.DIST_L2,cv2.DIST_MASK_PRECISE)
 res['native%d_pixel_gap'%side]=np.percentile(dd[mask],[0,1,5,50]).tolist()
# Direct measured-neighbor contacts only, no filling invalid gaps.
links=[]
for dy,dx in [(0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]:
 yy,xx=np.nonzero(d['native'][0]);y=yy+dy;x=xx+dx;good=(y>=0)&(y<480)&(x>=0)&(x<640);yy,xx,y,x=yy[good],xx[good],y[good],x[good];keep=d['native'][1][y,x]
 v=np.linalg.norm(p[yy[keep],xx[keep]]-p[y[keep],x[keep]],axis=1)
 links.append(dict(dy=dy,dx=dx,count=len(v),distance_mm_quantiles=np.percentile(v*1000,[0,50,100]).tolist()if len(v)else[]))
res['direct_measured_links']=links
img=d['rgb'][305:462,245:415].copy();img=cv2.resize(img,None,fx=3,fy=3,interpolation=cv2.INTER_NEAREST);cv2.imwrite(str(B/'fragment_ownership/current_box_crop.png'),img)
res['provenance']=dict(mask_npz_sha256=hashlib.sha256((B/'fragment_ownership/masks.npz').read_bytes()).hexdigest(),script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),source_stamp_ns=r['source_stamp_ns'])
(B/'fragment_ownership/three_dimensional_diagnostic.json').write_text(json.dumps(res,indent=2)+'\n');print(json.dumps(res))
