"""Bounded offline capture correction and three raw-frame qualifier comparison."""
import hashlib,json,os,sys
from pathlib import Path
W=Path(__file__).resolve().parents[1]
N=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen22_restart_20261015T003700Z')
I=N/'gen28_repositioned_initial_readonly_inputs'
O=N/'verification/gen28_repositioned_initial_replay_r1'
assert Path('/media/zhuyupei/SanStand').is_mount() and O.is_dir()
os.environ.update(ROS_MASTER_URI='http://127.0.0.1:1',ROS_LOG_DIR=str(O/'ros'),TMPDIR=str(O/'tmp'),PYTHONDONTWRITEBYTECODE='1',OPENBLAS_NUM_THREADS='1')
sys.path.insert(0,str(W/'src/alicia_flexible_grasp_supervisor/src'))
import numpy as np
import cv2
from alicia_flexible_grasp.vision.object_geometry import GeometryEstimate
from alicia_flexible_grasp.vision.target_observation import TargetTrackIdentity
from alicia_flexible_grasp.vision.regular_box_inference import qualify_regular_box_top
from alicia_flexible_grasp.robot.observation_view_gateway import _camera_configuration_contract
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def digest(x):return hashlib.sha256(json.dumps(x,ensure_ascii=False,sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()
def save(p,r):
 assert not p.exists();p.write_text(json.dumps(r,ensure_ascii=False,indent=2,default=lambda v:v.tolist() if isinstance(v,np.ndarray) else str(v))+'\n');return dict(path=str(p),sha256=sha(p))
g=json.loads((O/'measured_geometry.json').read_text())
geometry=GeometryEstimate(True,'','',g['center_base'],g['axes_base'],g['size_xyz_m'],g['support_normal_base'],g['support_offset_m'],g['support_inlier_ratio'],g['object_points_base'],g['source_mode'])
identity=TargetTrackIdentity(0,'diagnostic-gen28-repositioned-initial-only')
capture=json.loads((I/'result.json').read_text());rows=[]
for frame in capture['frames']:
 p=Path(frame['npz']);assert sha(p)==frame['sha256'];d=np.load(p,allow_pickle=False);extra={};locals_captured={}
 def trace(f,event,arg):
  if f.f_code is qualify_regular_box_top.__code__ and event=='return':
   v=f.f_locals;visual=v.get('visual')
   extra.update(visual_rectangle_found=visual is not None,occupancy_required_min=.65 if visual is not None else .88,internal_hole_fraction_max=.02)
   for k in ('hole_fraction','occupancy','pixel_size'):
    if k in v:extra[k]=float(v[k])
   for k in ('depth','mask','valid','top_mask','interior','polygon','image_corners','labels','stats','count','dilated_boundary','origin','tn','n','d','K','T','scale','top_height_seed'):
    if k in v:locals_captured[k]=v[k]
  return trace
 sys.settrace(trace)
 try:r=qualify_regular_box_top(d['depth_raw'],d['object_mask'],d['intrinsics'],float(d['depth_scale_m']),d['T_base_optical'],geometry,identity,frame['source_stamp_ns'],color_bgr=d['color'])
 finally:sys.settrace(None)
 components=[];overlay_receipt=None;v=locals_captured
 if 'labels' in v:
  for index in range(1,int(v['count'])):
   component=v['labels']==index
   if np.any(component & v['dilated_boundary']):continue
   ys,xs=np.nonzero(component);raw=v['depth'][ys,xs].astype(float);valid=np.isfinite(raw)&(raw>0.)
   zz=raw[valid]*v['scale'];K=v['K'];T=v['T']
   pc=np.column_stack(((xs[valid]-K[2])*zz/K[0],(ys[valid]-K[3])*zz/K[1],zz))
   points=pc@T[:3,:3].T+T[:3,3];residual=(points-v['origin'])@v['tn'];heights=points@v['n']+v['d']
   finite_nonpositive=int(np.count_nonzero(np.isfinite(raw)&(raw<=0.)))
   area=int(len(xs));components.append(dict(component_index=index,area_pixels=area,area_fraction_of_top_interior=area/int(np.count_nonzero(v['interior'])),
    bbox_xywh=[int(xs.min()),int(ys.min()),int(np.ptp(xs)+1),int(np.ptp(ys)+1)],centroid_uv=[float(np.mean(xs)),float(np.mean(ys))],
    zero_depth_pixels=int(np.count_nonzero(raw==0)),nonfinite_depth_pixels=int(np.count_nonzero(~np.isfinite(raw))),finite_nonpositive_depth_pixels=finite_nonpositive,
    valid_positive_depth_pixels=int(np.count_nonzero(valid)),invalid_depth_fraction=float(np.mean(~valid)),inside_object_mask_pixels=int(np.count_nonzero(v['mask'][ys,xs])),
    valid_signed_top_plane_residual_quantiles_m=np.quantile(residual,[0,.05,.5,.95,1]).tolist() if len(residual) else None,
    valid_above_top_over_1p5mm=int(np.count_nonzero(residual>.0015)),valid_below_top_over_1p5mm=int(np.count_nonzero(residual<-.0015)),
    valid_within_top_band=int(np.count_nonzero(np.abs(residual)<=.0015)),valid_below_half_top_height=int(np.count_nonzero(heights<.5*v['top_height_seed'])),
    depth_quantiles_m=np.quantile(zz,[0,.05,.5,.95,1]).tolist() if len(zz) else None))
  components.sort(key=lambda x:-x['area_pixels'])
  overlay=d['color'].copy();poly=np.rint(v['image_corners']).astype(np.int32);cv2.polylines(overlay,[poly],True,(0,255,0),1)
  all_missing=v['interior']&(v['top_mask']==0);overlay[all_missing]=(.4*overlay[all_missing]+.6*np.array([0,255,255])).astype(np.uint8)
  if components:
   index=components[0]['component_index'];component=v['labels']==index
   invalid=component & ~v['valid'];good=component & v['valid']
   overlay[invalid]=[255,0,255];overlay[good]=[0,0,255]
   x,y,w,h=components[0]['bbox_xywh'];cv2.rectangle(overlay,(x-1,y-1),(x+w,y+h),(255,255,255),1)
  name='source_%d_top_hole_overlay.png'%frame['source_stamp_ns'];op=O/name;assert not op.exists();assert cv2.imwrite(str(op),overlay)
  ys,xs=np.nonzero(v['mask']);x0=max(0,int(xs.min())-18);y0=max(0,int(ys.min())-18);x1=min(overlay.shape[1],int(xs.max())+19);y1=min(overlay.shape[0],int(ys.max())+19)
  crop=np.hstack((d['color'][y0:y1,x0:x1],overlay[y0:y1,x0:x1]));crop=cv2.resize(crop,None,fx=3,fy=3,interpolation=cv2.INTER_NEAREST)
  cp=O/('source_%d_top_hole_crop.png'%frame['source_stamp_ns']);assert not cp.exists();assert cv2.imwrite(str(cp),crop)
  overlay_receipt=dict(overlay=str(op),overlay_sha256=sha(op),crop=str(cp),crop_sha256=sha(cp),crop_xyxy=[x0,y0,x1,y1],
   legend='left crop is original RGB; right: green=projected top rectangle, yellow=all missing top inliers, magenta=largest internal component with invalid depth, red=largest component with valid positive depth, white=component bbox')
 rows.append(dict(source_stamp_ns=frame['source_stamp_ns'],raw_source_path=str(p),raw_source_sha256=sha(p),ok=r.ok,code=r.code,audit=dict(r.audit),branch=extra,
  internal_hole_components=components,overlay=overlay_receipt,model=r.model.to_evidence_dict() if r.model else None))
report=dict(scope='same_current_fused_measured_geometry_per_raw_pixel_triad',geometry_path=str(O/'measured_geometry.json'),geometry_sha256=sha(O/'measured_geometry.json'),
 source_sha256={str(Path(qualify_regular_box_top.__code__.co_filename).relative_to(W)):sha(qualify_regular_box_top.__code__.co_filename),'.runtime/summarize_gen28_repositioned_initial_inputs.py':sha(__file__)},
 rows=rows,production_qualifier_called=True,production_source_modified=False,diagnostic_identity_only=True,no_ros_calls=True,no_hardware_access=True,
 limitations=['Each raw RGBD/mask triad uses the same fused measured geometry and its own timestamp. This is a per-source qualification comparison, not three separate production requests.',
  'Measured joint stability and three-frame consistency do not imply reduced top-plane occupancy, boundary localization, or model uncertainty.',
  'No qualified model means no final inferred-width uncertainty contract has been established. Candidate and path feasibility are separate.'])
print(json.dumps(save(O/'raw_frame_qualification.json',report),ensure_ascii=False),flush=True)
for row in rows:print(json.dumps(dict(stamp=row['source_stamp_ns'],ok=row['ok'],code=row['code'],branch=row['branch']),ensure_ascii=False),flush=True)
