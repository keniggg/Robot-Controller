"""Offline actual fuser/GEN28 box entry/generator for passive current captures."""
import hashlib,json,os,sys,time
from pathlib import Path
from dataclasses import asdict
from types import SimpleNamespace as NS

W=Path(__file__).resolve().parents[1]
N=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen22_restart_20261015T003700Z')
I=N/'gen28_repositioned_initial_readonly_inputs'
O=N/'verification/gen28_repositioned_initial_replay_r1'
assert Path('/media/zhuyupei/SanStand').is_mount() and not O.exists()
O.mkdir();(O/'tmp').mkdir();(O/'ros').mkdir()
os.environ.update(ROS_MASTER_URI='http://127.0.0.1:1',PYTHONDONTWRITEBYTECODE='1',
 ROS_LOG_DIR=str(O/'ros'),TMPDIR=str(O/'tmp'),OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1')
for p in ('src/alicia_flexible_grasp_supervisor/tests','src/alicia_grasp_modes/tests',
 'src/alicia_flexible_grasp_supervisor/src','src/alicia_grasp_modes/src','devel/lib/python3/dist-packages'):
 sys.path.insert(0,str(W/p))
import numpy as np
from _pytest.monkeypatch import MonkeyPatch
from test_gen26_inferred_box_remote_pipeline import pipeline,remote
from alicia_flexible_grasp.vision.rgbd_snapshot import RgbdSample,fuse_stable_samples
from alicia_flexible_grasp.vision.object_geometry import estimate_object_geometry
from alicia_flexible_grasp.vision.target_observation import TargetObservation,TargetTrackIdentity
from alicia_flexible_grasp.vision.multiview_surface import SurfaceView,fused_surface_from_view
from alicia_flexible_grasp.vision.regular_box_inference import qualify_regular_box_top
from alicia_flexible_grasp.robot.observation_view_gateway import _camera_configuration_contract

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def arrsha(a):return hashlib.sha256(np.ascontiguousarray(a).tobytes()).hexdigest()
def serial(x):
 if isinstance(x,np.ndarray):return x.tolist()
 if isinstance(x,np.generic):return x.item()
 return str(x)
def save(name,v):
 p=O/name;assert not p.exists();p.write_text(json.dumps(v,ensure_ascii=False,indent=2,default=serial)+'\n');return dict(path=str(p),sha256=sha(p))
sources=[
 'src/alicia_flexible_grasp_supervisor/scripts/remote_grasp6d_node.py',
 'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/regular_box_inference.py',
 'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/rgbd_snapshot.py',
 'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/object_geometry.py',
 'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/grasp/tabletop_geometry_candidates.py',
 'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/grasp/gripper_geometry.py',
 'src/alicia_grasp_modes/scripts/mode_aware_remote_grasp6d.py',
 'src/alicia_flexible_grasp_supervisor/tests/test_gen26_inferred_box_remote_pipeline.py',
 'src/alicia_flexible_grasp_supervisor/tests/test_remote_grasp6d_streaming.py',
 '.runtime/replay_gen28_repositioned_initial_readonly.py']
source_sha={p:sha(W/p) for p in sources}
capture=json.loads((I/'result.json').read_text());assert capture['usable_detected_target_triples'] is True
params=json.loads((I/'parameters.json').read_text());values=params['before']['parameters']
assert params['before']['stable_configuration_sha256']==params['after']['stable_configuration_sha256']
cfg=values['/grasp_6d/remote'];cam=values['/camera'];gg=cfg['gripper_geometry']
inputs={str(I/name):sha(I/name) for name in ('result.json','parameters.json','accepted_joint_states.json')}
identity=TargetTrackIdentity(0,'diagnostic-gen28-repositioned-initial-only')
samples=[];raws=[]
for frame in capture['frames']:
 p=Path(frame['npz']);assert sha(p)==frame['sha256'];inputs[str(p)]=sha(p)
 a=np.load(p,allow_pickle=False);raws.append(a)
 assert all(arrsha(a[k])==h for k,h in frame['array_sha256'].items())
 ns=frame['source_stamp_ns'];obj=NS(**frame['object_pose_message'])
 assert obj.detected and np.count_nonzero(a['object_mask'])>0
 qb=frame['accepted_joint_bracket'];np.testing.assert_array_equal(qb['before']['positions_rad'],qb['after']['positions_rad'])
 samples.append(RgbdSample(a['color'],a['depth_raw'],a['object_mask'],tuple(int(v) for v in a['bbox_xywh']),
  obj,ns/1e9,frame['topics']['depth']['header']['frame_id'],np.asarray(qb['before']['positions_rad']),ns,0,identity))
for a in raws[1:]:
 np.testing.assert_array_equal(a['intrinsics'],raws[0]['intrinsics'])
 np.testing.assert_allclose(a['T_base_optical'],raws[0]['T_base_optical'],rtol=0,atol=1e-12)
K=NS(fx=cam['fx'],fy=cam['fy'],cx=cam['cx'],cy=cam['cy'],width=cam['width'],height=cam['height'],depth_scale=cam['depth_scale'])
T=raws[-1]['T_base_optical']
sk=dict(require_mask=True,min_mask_iou=cfg['planning_mask_min_iou'],max_centroid_shift_px=cfg['planning_mask_max_centroid_shift_px'],
 max_joint_delta_rad=cfg['planning_max_joint_delta_rad'],erosion_px=cfg['mask_erosion_px'],depth_scale=K.depth_scale,
 depth_min_m=values['/perception/depth_min_m'],depth_max_m=values['/perception/depth_max_m'],mad_scale=cfg['depth_mad_scale'],
 mad_absolute_floor_m=cfg['depth_mad_absolute_floor_m'],internal_hole_max_area_px=0)
snapshot=fuse_stable_samples(samples,**sk);assert snapshot.ok,(snapshot.failure_code,snapshot.failure_reason)
assert len(samples)==cfg['near_field_planning_snapshot_frames']==3
assert snapshot.bbox==tuple(int(v) for v in raws[-1]['bbox_xywh'])
gk=dict(support_bbox_expand_ratio=max(0.,float(cfg.get('support_bbox_expand_ratio',.30))),
 support_distance_threshold_m=max(.0001,float(cfg.get('support_distance_threshold_m',cfg.get('target_cloud_support_plane_inlier_distance_m',.004)))),
 voxel_size_m=max(.0001,float(cfg.get('target_cloud_voxel_size_m',.0025))),min_support_points=max(3,int(cfg.get('geometry_min_support_points',200))),
 min_object_points=max(3,int(cfg.get('geometry_min_object_points',cfg.get('target_cloud_min_points',120)))),
 min_size_m=max(.0001,float(cfg.get('geometry_min_size_m',.005))),max_size_m=float(cfg.get('geometry_max_size_m',.600)),
 max_height_m=float(cfg.get('geometry_max_height_m',.500)),previous_axes_base=None,
 outlier_neighbors=max(1,int(cfg.get('target_cloud_outlier_neighbors',16))),outlier_std_ratio=max(0.,float(cfg.get('target_cloud_outlier_std_ratio',2.0))),
 support_minimum_span_m=max(gg['finger_box_xyz_m'][0],gg['max_inner_gap_m']+2*gg['finger_box_xyz_m'][1])+2*gg.get('support_clearance_m',.003))
g=estimate_object_geometry(snapshot.depth_raw,snapshot.target_depth_raw,snapshot.object_mask,snapshot.bbox,K,K.depth_scale,T,snapshot.source_mode,**gk)
assert g.ok,(g.failure_code,g.failure_reason)
geometry_receipt=save('measured_geometry.json',dict(center_base=g.center_base,axes_base=g.axes_base,size_xyz_m=g.size_xyz_m,
 support_normal_base=g.support_normal_base,support_offset_m=g.support_offset_m,support_inlier_ratio=g.support_inlier_ratio,
 object_points_base=g.object_points_base,source_mode=g.source_mode))
np.savez_compressed(str(O/'snapshot.npz'),color=snapshot.color_bgr,depth_raw=snapshot.depth_raw,target_depth_raw=snapshot.target_depth_raw,
 object_mask=snapshot.object_mask,T_base_optical=T,intrinsics=raws[-1]['intrinsics'],depth_scale_m=np.array(K.depth_scale),bbox_xywh=np.array(snapshot.bbox),source_stamp_ns=np.array(snapshot.stamp_ns))
before_points=arrsha(g.object_points_base)
edge=min(snapshot.bbox[0],snapshot.bbox[1],K.width-snapshot.bbox[0]-snapshot.bbox[2],K.height-snapshot.bbox[1]-snapshot.bbox[3])
observation=TargetObservation(identity,snapshot.stamp_ns,'base_link',g.object_points_base,g.support_normal_base,g.support_offset_m,snapshot.bbox,snapshot.depth_raw.shape,edge,'instance_mask','unknown_small')
view=SurfaceView(identity,snapshot.stamp_ns,observation.points_base,observation.support_normal_base,observation.support_offset_m,edge)
surface=fused_surface_from_view(view)
baseline=qualify_regular_box_top(snapshot.depth_raw,snapshot.object_mask,K,K.depth_scale,T,g,identity,snapshot.stamp_ns,color_bgr=snapshot.color_bgr)
mp=MonkeyPatch();started=time.monotonic()
try:
 node,_,_,fixture_params,_=pipeline(mp)
 def get_parameter(name,default=None):
  if name in values:return values[name]
  for prefix in ('/grasp_6d/remote','/grasp','/gripper','/camera','/handeye'):
   if name.startswith(prefix+'/'):
    item=values[prefix]
    for k in name[len(prefix)+1:].split('/'):
     if not isinstance(item,dict) or k not in item:return default
     item=item[k]
    return item
  return default
 mp.setattr(remote.rospy,'get_param',get_parameter);mp.setattr(remote.rospy,'get_param_cached',get_parameter)
 node.gripper_geometry=remote.GripperGeometry(max_inner_gap_m=gg['max_inner_gap_m'],jaw_clearance_each_side_m=gg['width_safety_margin_per_side_m'],
  finger_size_xyz_m=np.array(gg['finger_box_xyz_m']),palm_size_xyz_m=np.array(gg['palm_box_xyz_m']),support_clearance_m=gg.get('support_clearance_m',.003))
 node.opening_fit_clearance_each_side_m=remote.load_opening_fit_clearance_config(gg,node.gripper_geometry)
 node.tabletop_geometry_enabled,node.tabletop_geometry_config,node.hybrid_merge_config=remote.load_tabletop_geometry_config(cfg,node.gripper_geometry,node.opening_fit_clearance_each_side_m)
 node.adaptive_stage_limits=remote.load_adaptive_stage_config(cfg)
 node.gripper_physical_open_width_m=values['/gripper']['open_position_m']
 node.gripper_tool_jaw_axis=gg['tool_jaw_axis'];node.gripper_tool_finger_length_axis=gg['tool_finger_length_axis']
 node.grasp_config=values['/grasp']
 for key in ('candidate_min_downward_approach_cos','candidate_max_final_approach_lateral_m','target_absolute_sanity_distance_m'):
  setattr(node,key,cfg[key])
 node._current_stream_target_identity=lambda:identity
 node._latest_target_observation=observation;node._near_field_reference_view=view;node._active_multiview_surface=lambda:(surface,view)
 node._camera_intrinsics=lambda:K
 planning=node._prepare_regular_box_snapshot(snapshot,g,T)
 captured=node._regular_box_snapshot;result=captured['result'];model=node._current_inferred_box()
 assert arrsha(g.object_points_base)==before_points==arrsha(planning.object_points_base)
 candidates,diagnostics=node._generate_tabletop_candidates(planning,snapshot)
 assert source_sha=={p:sha(W/p) for p in sources}
 output=dict(capture=str(I/'result.json'),capture_sha256=inputs[str(I/'result.json')],actual_production_fuser_called=True,actual_production_box_entry_called=True,
  actual_production_generator_called=True,diagnostic_identity=asdict(identity),production_track_identity_claimed=False,execution_authority=False,
  candidate_count=len(candidates),candidate_generation_failure_code=diagnostics.get('failure_code'),source_stamp_ns=snapshot.stamp_ns,sample_stamp_ns=list(snapshot.sample_stamp_ns),
  snapshot_arguments=sk,geometry_arguments=gk,effective_tabletop_geometry_config=asdict(node.tabletop_geometry_config),
  effective_gripper_geometry=asdict(node.gripper_geometry),effective_adaptive_stage_limits=asdict(node.adaptive_stage_limits),
  raw_qualification=dict(ok=result.ok,code=result.code,audit=dict(result.audit)),model=model.to_evidence_dict() if model else None,
  fused_pixel_baseline=dict(ok=baseline.ok,code=baseline.code,audit=dict(baseline.audit)),tabletop_diagnostics=diagnostics,
  measured_geometry=geometry_receipt,measured_geometry_size_xyz_m=g.size_xyz_m,fused_geometry_preserved=True,
  source_sha256=source_sha,input_sha256=inputs,source_unchanged=True,elapsed_sec=time.monotonic()-started,
  no_ros_calls=True,no_hardware_access=True,no_temperature_reads=True,physical_grasp_verified=False,
  limitations=['Diagnostic identity is local offline only, not a production track or motion authorization.',
   'Live parameter capture supplies fuser/geometry/gripper/tabletop inputs; unknown mode overrides mask_internal_hole_max_area_px to zero as production mode reset does.',
   'Three frames match near-field contact snapshot configuration; no asynchronous runtime admission/ticket is reproduced, no prior object axes retained.',
   'Actual remote box entry and generator run in a base Remote node fixture, not the complete mode-aware node; fixture identity handeye, visibility disabled, default soft scores and synthetic lifecycle remain. Any positive proposal is provisional geometry evidence only.',
   'No complete collision trajectory, MoveIt/IK planning, controller or robot execution has been evaluated. Real planning requires a separately bound current production request.'])
 print(json.dumps(save('result.json',output),ensure_ascii=False),flush=True)
 print(json.dumps(dict(raw_ok=result.ok,raw_code=result.code,geometry_size=g.size_xyz_m,candidates=len(candidates),failure=diagnostics.get('failure_code')),default=serial),flush=True)
finally:mp.undo()
