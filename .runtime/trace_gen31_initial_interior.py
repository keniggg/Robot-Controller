from pathlib import Path
import sys,json,hashlib,cv2,numpy as np
W=Path(__file__).resolve().parents[1];sys.path.insert(0,str(W/'src/alicia_flexible_grasp_supervisor/src'))
from alicia_flexible_grasp.vision import regular_box_inference as b
from alicia_flexible_grasp.vision.object_geometry import GeometryEstimate
from alicia_flexible_grasp.vision.target_observation import TargetTrackIdentity
C=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen29_restart_20261016T000728Z');I=C/'verification/gen30_attempt1_closed_evidence_r1';O=C/'verification/gen31_initial_interior_r1';O.mkdir(exist_ok=False)
a=json.loads((C/'verification/gen31_initial_source_geometry_r1/result.json').read_text());g=a['replay_geometry'];T=np.array(a['snapshot_transform']['T_base_optical']);cam=a['camera_parameters'];K=np.array([cam[k] for k in ('fx','fy','cx','cy')]);x=np.load(I/'1792116749240888833_raw.npz');geom=GeometryEstimate(True,'','',g['obb_center_base_m'],g['R_base_obb'],g['obb_size_xyz_m'],g['support_normal_base'],g['support_offset_m'],g['support_inlier_ratio'],g['object_points_base_m'],g['source_mode']);state={}
def tr(f,e,v):
 if f.f_code.co_name=='_qualify_regular_box_top' and e=='return':state.clear();state.update(f.f_locals)
 return tr
sys.settrace(tr)
try:r=b.qualify_regular_box_top(x['depth_raw'],x['object_mask'],K,cam['depth_scale'],T,geom,TargetTrackIdentity(4,'g1-t4'),1792116749240888833,color_bgr=x['color_bgr'])
finally:sys.settrace(None)
s=state;rows=[];highlight=np.zeros(x['depth_raw'].shape,bool)
for idx in range(1,s['count']):
 comp=s['labels']==idx
 if np.any(comp&s['dilated_boundary']):continue
 yy,xx=np.where(comp);zz=x['depth_raw'][yy,xx].astype(float)*cam['depth_scale'];valid=zz>0;p=np.column_stack(((xx-K[2])*zz/K[0],(yy-K[3])*zz/K[1],zz))@T[:3,:3].T+T[:3,3];res=(p-s['origin'])@s['tn'];rows.append(dict(index=idx,pixels=len(xx),valid=int(valid.sum()),raw_zero=int((~valid).sum()),mask=int((x['object_mask'][yy,xx]>0).sum()),bbox=[int(xx.min()),int(yy.min()),int(xx.max()),int(yy.max())],residual_quantiles_m=np.quantile(res[valid],[0,.1,.5,.9,1]).tolist() if valid.any() else None));
 if len(xx)==s['largest_hole']:highlight=comp
color=x['color_bgr'].copy();color[highlight]=np.rint(.45*color[highlight]+.55*np.array([255,0,255])).astype(np.uint8);yy,xx=np.where(x['object_mask']>0);roi=color[yy.min()-15:yy.max()+16,xx.min()-15:xx.max()+16];cv2.imwrite(str(O/'largest_interior_component.png'),cv2.resize(roi,None,fx=5,fy=5,interpolation=cv2.INTER_NEAREST));np.savez_compressed(O/'arrays.npz',interior=s['interior'],top_mask=s['top_mask'],labels=s['labels'],largest_component=highlight)
report=dict(code=r.code,source_stamp_ns=1792116749240888833,occupancy=s['occupancy'],hole_fraction=s['hole_fraction'],components=rows,source_sha256=hashlib.sha256(Path(b.__file__).read_bytes()).hexdigest());(O/'result.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report))
