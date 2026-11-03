"""Exact-source test of new module; no ROS or external interfaces."""
import pathlib,json,time,hashlib
import cv2,numpy as np
from alicia_flexible_grasp.vision.cuboid_rgb_interval import infer_cuboid_rgb_interval,validate_cuboid_rgb_interval
D=pathlib.Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen36_restart_20261017T012037Z')
O=D/'verification/gen38_cuboid_silhouette_probe_r1';I=D/'verification/gen37_actual_attempt1_source_images_r1';R=D/'verification/gen37_box_source_replay_r1'
G=pathlib.Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/grasp_records/20261017T032113.476278Z_a84cc89fc53e4c2b82680749d78b9b9f')
assert pathlib.Path('/media/zhuyupei/SanStand').is_mount()
a=json.loads((G/'keyframes_audits/0e5ebbd3c562cfab2617b338045f554ffc0920a23626fbc2a49cc474cc9d798d.json').read_text());f=np.load(R/'exact_current_top_fit.npz')
c=json.loads((D/'gen37_runtime/gen37_grasp_scene_r2/camera_configuration.json').read_text());K=np.array([c[k] for k in ('fx','fy','cx','cy')]);q=a['remote_diagnostics']['tabletop_geometry']['regular_box_inference'];s=str(a['snapshot_stamp_ns'])
color=cv2.imread(str(I/(s+'_color.png')));mask=cv2.imread(str(I/(s+'_mask.png')),0);depth=np.load(I/(s+'_depth.npy'))
for role,array in [('color',color),('mask',mask),('depth',depth)]:
 assert hashlib.sha256(array.tobytes(order='C')).hexdigest()==q['retained_pixel_sha256'][role+'_sha256']
pixel=np.median(depth[f['uv'][f['keep'],1],f['uv'][f['keep'],0]])*c['depth_scale']/min(K[:2])/q['camera_top_cosine']
start=time.monotonic();r=infer_cuboid_rgb_interval(color,mask,depth,c['depth_scale'],np.array(a['snapshot_transform']['T_base_optical']),K,f['origin'],f['tn'],f['axes'],pixel,q['top_plane_rms_m'],a['replay_geometry']['support_offset_m'],q['top_height_m'])
r['diagnostic_wall_seconds']=time.monotonic()-start
if r['ok']:
 size=np.r_[r['upper']-r['lower'],r['nominal_height_m']]
 try:
  validate_cuboid_rgb_interval(r['audit'],size,r['edge_uncertainty'],r['yaw_uncertainty'],r['height_uncertainty']);r['validator_passed']=True
 except Exception as exc:r.update(validator_passed=False,validator_error=str(exc))
 target=O/'new_interval_latest.json'
 target.write_text(json.dumps(r,indent=2,default=lambda x:x.tolist() if hasattr(x,'tolist') else x.item())+'\n')
else:
 (O/'new_interval_latest.json').write_text(json.dumps(r,indent=2)+'\n')
print(json.dumps(dict(ok=r['ok'],reason=r.get('reason'),seconds=r['diagnostic_wall_seconds'],validator=r.get('validator_passed'),validator_error=r.get('validator_error'),edge=r.get('edge_uncertainty'),yaw=r.get('yaw_uncertainty'),height_u=r.get('height_uncertainty'),nominal_h=r.get('nominal_height_m')),ensure_ascii=False))
