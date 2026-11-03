from pathlib import Path
import sys,json,numpy as np
W=Path(__file__).resolve().parents[1];sys.path.insert(0,str(W/'src/alicia_flexible_grasp_supervisor/src'))
C=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen29_restart_20261016T000728Z');I=C/'verification/gen30_attempt1_closed_evidence_r1';D=C/'verification/gen31_hough_top_candidates_r1';rows=json.loads((D/'result.json').read_text())['rows'];z=np.load(C/'verification/gen31_mask_axis_proposal_r1/1792116749240888833_original_basis.npz');x=np.load(I/'1792116749240888833_raw.npz');T=z['T'];K=z['K'];origin=z['origin'];tn=z['tn'];A=z['axes'];scale=9.999999747378752e-05
camera=T[:3,3]
def plane(uv):
 rays=np.column_stack(((uv[:,0]-K[2])/K[0],(uv[:,1]-K[3])/K[1],np.ones(len(uv))))@T[:3,:3].T;return camera+rays*(float((origin-camera)@tn)/(rays@tn))[:,None]
result=[]
for edge in range(4):
 lines={tuple(np.array(row['proposal_lines'][edge]).ravel()) for row in rows}
 for line in sorted(lines):
  uv=np.array(line).reshape(2,2);v=uv[1]-uv[0];metric=plane(uv)@A[:,:2];j=1 if edge%2==0 else 0;sign=-1 if edge in(0,3) else 1;out=np.array([v[1],-v[0]])/np.linalg.norm(v)
  mid=uv.mean(0);p0=plane(mid[None]);p1=plane((mid+out)[None]);
  if float((p1-p0)@A[:,j])*sign<0:out=-out
  checks=[]
  for side in [-1,1]:
   px=np.rint(uv[0]+np.outer(np.linspace(.12,.88,21),v)+side*3*out).astype(int);zz=x['depth_raw'][px[:,1],px[:,0]]*scale;valid=zz>0;pts=np.column_stack(((px[:,0]-K[2])*zz/K[0],(px[:,1]-K[3])*zz/K[1],zz))@T[:3,:3].T+camera;dist=(pts-origin)@tn;checks.append(dict(valid_fraction=float(valid.mean()),same_top_fraction=float(np.count_nonzero(valid&(abs(dist)<.0015)))/max(1,np.count_nonzero(valid)),plane_median_m=float(np.median(dist[valid])) if np.any(valid) else None))
  result.append(dict(edge=edge,line=list(line),inside=checks[0],outside=checks[1]))
(D/'line_depth_bands.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
