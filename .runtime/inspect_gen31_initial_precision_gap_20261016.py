"""Closed original frame: exact-source precision routing and one offline profile."""
from pathlib import Path
from types import SimpleNamespace
import cProfile, hashlib, io, json, pstats, sys, time
import numpy as np
W=Path(__file__).resolve().parents[1]
for d in ['src/alicia_flexible_grasp_supervisor/src','src/alicia_grasp_modes/src']:
    sys.path.insert(0,str(W/d))
from alicia_flexible_grasp.vision import regular_box_inference as box
from alicia_flexible_grasp.vision.object_geometry import GeometryEstimate
from alicia_flexible_grasp.vision.target_observation import TargetTrackIdentity
from alicia_flexible_grasp.vision.rgbd_snapshot import RgbdSample, fuse_stable_samples
from alicia_flexible_grasp.vision.multiview_surface import SurfaceView, fused_surface_from_view
from alicia_flexible_grasp.grasp.tabletop_geometry_candidates import TabletopGeometryConfig
from alicia_flexible_grasp.grasp.gripper_geometry import GripperGeometry
from alicia_grasp_modes.observe_return_visual import unqualified_box_precision_view_gaps
C=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen29_restart_20261016T000728Z')
O=C/'verification/gen31_initial_precision_route_actual_r1';O.mkdir(exist_ok=False)
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
inputs={}
def read(path):
    inputs[str(path)]=sha(path);return json.loads(path.read_text())
r=read(C/'verification/gen31_actual_box_sources_r6/result.json')['rows'][0]
a=read(C/'verification/gen31_initial_source_geometry_r1/result.json')
npz=C/'verification/gen30_attempt1_closed_evidence_r1/1792116749240888833_raw.npz';inputs[str(npz)]=sha(npz);raw=np.load(npz)
g=a['replay_geometry'];cam=a['camera_parameters'];T=np.array(a['snapshot_transform']['T_base_optical']);stamp=r['source_stamp_ns'];identity=TargetTrackIdentity(4,'g1-t4')
geometry=GeometryEstimate(True,'','',g['obb_center_base_m'],g['R_base_obb'],g['obb_size_xyz_m'],g['support_normal_base'],g['support_offset_m'],g['support_inlier_ratio'],g['object_points_base_m'],g['source_mode'])
color,depth,mask=[raw[k] for k in ('color_bgr','depth_raw','object_mask')]
y,x=np.where(mask);bbox=(int(x.min()),int(y.min()),int(np.ptp(x))+1,int(np.ptp(y))+1);edge=min(bbox[0],bbox[1],mask.shape[1]-bbox[0]-bbox[2],mask.shape[0]-bbox[1]-bbox[3])
K=SimpleNamespace(width=depth.shape[1],height=depth.shape[0],**{k:cam[k] for k in ('fx','fy','cx','cy')})
sample=RgbdSample(color,depth,mask,bbox,None,stamp*1e-9,'camera_optical',np.zeros(6),stamp,identity.epoch,identity)
snapshot=fuse_stable_samples([sample],require_mask=True,min_mask_iou=.85,max_centroid_shift_px=5.,max_joint_delta_rad=.01,erosion_px=0,depth_scale=cam['depth_scale'],depth_min_m=.03,depth_max_m=2.,mad_scale=3.,mad_absolute_floor_m=.002,internal_hole_max_area_px=0)
assert snapshot.ok,snapshot.failure_reason
candidate=box.validate_box_precision_candidate_source(r['audit']['precision_view_candidate'],snapshot,K,cam['depth_scale'],T,geometry,identity,stamp)
surface=fused_surface_from_view(SurfaceView(identity,stamp,geometry.object_points_base,geometry.support_normal_base,geometry.support_offset_m,edge))
config=TabletopGeometryConfig();gripper=GripperGeometry(.05,.002,np.array([.0434,.0286,.06]),np.array([.1175,.155,.0774]),.003)
gaps=unqualified_box_precision_view_gaps(candidate,surface,T[:3,3],config,gripper)
assert len(gaps)==1 and gaps[0]['required_open_width_m']<.05<gaps[0]['required_open_upper_m'] and not gaps[0]['contact_or_motion_authorized']
sources=[Path(box.__file__),Path(box.__file__).with_name('outer_rgb_rectangle.py'),W/'src/alicia_grasp_modes/src/alicia_grasp_modes/observe_return_visual.py',Path(__file__)]
before={str(p):sha(p) for p in sources}
profile=cProfile.Profile();profile.enable()
result=box.qualify_regular_box_top(depth,mask,K,cam['depth_scale'],T,geometry,identity,stamp,color_bgr=color)
profile.disable();assert result.code==r['code'] and dict(result.audit)==r['audit'] and result.model is None
text=io.StringIO();pstats.Stats(profile,stream=text).strip_dirs().sort_stats('cumulative').print_stats(35);(O/'profile.txt').write_text(text.getvalue())
report=dict(mode='offline_exact_initial_source_candidate_to_precision_gap',inputs_sha256=inputs,source_before=before,source_after={str(p):sha(p) for p in sources},source_binding_passed=True,actual_result_model=None,gaps=gaps,profile_qualifier_equals_sealed_r6=True,profile_log_sha256=sha(O/'profile.txt'),motion_planned=False,motion_authorized=False,live_ros_contacted=False,physical_success_claimed=False,limitation='single original initial frame with reconstructed measured geometry, not missing historical temporal request')
(O/'result.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
print(json.dumps(dict(path=str(O/'result.json'),sha256=sha(O/'result.json'),nominal_with_fit=gaps[0]['required_open_width_m'],upper=gaps[0]['required_open_upper_m'])))
print(text.getvalue())
