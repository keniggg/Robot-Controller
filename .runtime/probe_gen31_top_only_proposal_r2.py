"""Two-frame finite diagnostic of silhouette proposal direction; no source edit."""
from pathlib import Path
import sys,json,inspect
import numpy as np,cv2
W=Path(__file__).resolve().parents[1];sys.path.insert(0,str(W/'src/alicia_flexible_grasp_supervisor/src'))
from alicia_flexible_grasp.vision import regular_box_inference as box,outer_rgb_rectangle as outer
from alicia_flexible_grasp.vision.object_geometry import GeometryEstimate
from alicia_flexible_grasp.vision.target_observation import TargetTrackIdentity
C=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen29_restart_20261016T000728Z');I=C/'verification/gen30_attempt1_closed_evidence_r1';R=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/grasp_records/20261016T021226.502516Z_0494c8bc84a34a74933e8eae122dd1f4');O=C/'verification/gen31_top_only_proposal_r2';O.mkdir(exist_ok=False)
text=inspect.getsource(outer.infer_outer_rgb_rectangle)
needle='    lo, hi = silhouette.min(0), silhouette.max(0)'
text=text.replace(needle,'''    zz = depth[yy,xx].astype(float)*scale
    known = np.isfinite(zz)&(zz>0.)
    measured = np.column_stack(((xx-K[2])*zz/K[0],(yy-K[3])*zz/K[1],zz))@T[:3,:3].T+camera
    possible_top = ~known | (np.abs((measured-origin)@tn)<=.0015)
    original_silhouette = silhouette.copy()
    silhouette = silhouette[possible_top]
'''+needle)
text=text.replace('if np.max(nearest) > math.sqrt(8.):','if np.mean(nearest <= math.sqrt(8.)) < .70:')
space=dict(vars(outer));exec(compile(text,'<isolated_mask_axes>','exec'),space)
rows=[]
for stamp,ap in [(1792116749240888833,C/'verification/gen31_initial_source_geometry_r1/result.json'),(1792116802511774778,R/'keyframes_audits/07a8ac807d6a7f0702ef09c8866949f4ce96f8379dc1421dc6ae203af6086135.json')]:
 a=json.loads(ap.read_text());g=a['replay_geometry'];geom=GeometryEstimate(True,'','',g['obb_center_base_m'],g['R_base_obb'],g['obb_size_xyz_m'],g['support_normal_base'],g['support_offset_m'],g['support_inlier_ratio'],g['object_points_base_m'],g['source_mode']);x=np.load(I/f'{stamp}_raw.npz');K=np.array([395.7682800292969,395.6027526855469,324.9961242675781,250.833984375]);T=np.array(a['snapshot_transform']['T_base_optical']);states=[]
 def trace(f,e,v):
  if e=='return' and f.f_code.co_name=='_qualify_regular_box_top':states.append(f.f_locals.copy())
  return trace
 sys.settrace(trace)
 try:r=box.qualify_regular_box_top(x['depth_raw'],x['object_mask'],K,9.999999747378752e-05,T,geom,TargetTrackIdentity(4,'g1-t4'),stamp,color_bgr=x['color_bgr'])
 finally:sys.settrace(None)
 t=states[0];inputs=[x['color_bgr'],x['object_mask']>0,x['depth_raw'],9.999999747378752e-05,T,K,t['origin'],t['tn'],t['axes'],t['pixel_size'],t['rms']]
 result=space['infer_outer_rgb_rectangle'](*inputs)
 rows.append(dict(source_stamp_ns=stamp,result=result))
 np.savez_compressed(O/f'{stamp}_original_basis.npz',origin=t['origin'],tn=t['tn'],axes=t['axes'],pixel_size=t['pixel_size'],rms=t['rms'],K=K,T=T)
(O/'result.json').write_text(json.dumps(dict(rows=rows,source_modified=False),default=lambda v:v.tolist(),indent=2)+'\n');print(json.dumps(rows,default=lambda v:v.tolist()))
