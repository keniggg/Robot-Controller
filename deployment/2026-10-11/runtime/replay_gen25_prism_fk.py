"""Sealed request19, current measured support, original bounded FK optimizer. No ROS."""
from pathlib import Path
import json,hashlib,itertools,time,argparse
import numpy as np
from alicia_grasp_modes import reachable_observation_views as views
from alicia_grasp_modes.observation_arrival_projection import capture_arrival_projection
from alicia_grasp_modes import observation_arrival_projection as projection
from alicia_flexible_grasp.grasp.gripper_geometry import GripperGeometry
from alicia_flexible_grasp.robot.observation_path_guard import SerialUrdfFk

ROOT=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_ros_20261009T003407Z/gen25_handoff_analysis')
parser=argparse.ArgumentParser();parser.add_argument('--output',required=True);cli=parser.parse_args()
OUT=Path(cli.output);OUT.mkdir(exist_ok=True)
paths=[Path(views.__file__),Path(projection.__file__)]
source_before={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
meta=json.loads((ROOT/'request19_source/manifest.json').read_text())
images=json.loads((ROOT/'source_images/manifest.json').read_text())
audit=json.loads((Path(meta['record'])/'keyframes_audits/e319974f04dabbffbf5bbaaa9556193b6c2978a352679318fb1f2dd41620ac24.json').read_text())
source_stamp=meta['source_stamp_ns'];camera=meta['camera'];g=audit['replay_geometry']
raw=np.load(ROOT/'request19_source/source.npz');depth=raw['depth_raw'];mask=raw['raw_mask'];T=raw['T_base_camera']
ys,xs=np.nonzero((depth>0)&(mask==0));z=depth[ys,xs].astype(float)*camera['depth_scale']
optical_points=np.column_stack(((xs-camera['cx'])*z/camera['fx'],(ys-camera['cy'])*z/camera['fy'],z))
C=np.eye(4);C[:3,:3]=[[0,0,1],[-1,0,0],[0,-1,0]];optical=T@C
points=optical_points@optical[:3,:3].T+optical[:3,3]
normal=np.array(g['support_normal_base']);offset=g['support_offset_m']
support=points[np.abs(points@normal+offset)<=.002]
assert len(support)==meta['source_audit']['support_point_count']==100664
center=np.array(g['obb_center_base_m']);R=np.array(g['R_base_obb']);size=np.array(g['obb_size_xyz_m'])
corners=center+(np.array(list(itertools.product((-.5,.5),repeat=3)))*size)@R.T
assert hashlib.sha256(np.ascontiguousarray(corners).tobytes()).hexdigest()==meta['source_audit']['conservative_target_corners_sha256']
statuses=json.loads((ROOT/'detector_status.json').read_text());status=next(row['message'] for row in statuses if row['message'].get('source_stamp_ns')==source_stamp and row['message'].get('state')=='ready')
# The old record predates only this new metadata field. Reconstruct the exact
# acquired median height from its original raw frame, never from the later view.
from alicia_grasp_modes import tabletop
arow=next(row for row in images['files'] if row['roles']==['anchor']);a=np.load(ROOT/'source_images'/arow['npz'])
cfg=tabletop.Config();K=[camera[key] for key in ('fx','fy','cx','cy')]
seg=tabletop.segment_with_support_fallback(a['depth_raw'].astype(np.float32)*camera['depth_scale'],K,a['T_base_camera'],cfg)
tracker=tabletop.TargetTracker(cfg);anchor,reason=tracker.choose(seg,a['depth_raw'].shape);assert anchor is not None,reason
assert np.allclose(anchor.position_base,status['anchor_position_base'],atol=1e-10,rtol=0)
assert np.allclose(anchor.support_footprint_base,status['reference_support_footprint_base'],atol=1e-10,rtol=0)
status=dict(status,reference_object_height_m=anchor.metrics['median_height_m'])
selection=dict(mode='unknown',strategy='observe_return',generation=status['generation'],stamp_ns=status['selection_stamp_ns'])
contract=capture_arrival_projection(status,selection,source_stamp,camera)
source=dict(binding=audit['selected']['observation_source_binding'],camera=meta['source_audit']['camera_parameters'],
    points_base=support,corners_base=corners,center_base=center,tool_from_camera=raw['tool_from_camera'],
    geometry=dict(support_normal_base=normal,support_offset_m=offset,R_base_obb=R,obb_size_xyz_m=size),
    view_limits=meta['source_audit']['view_limits'],arrival_projection=contract)
search=audit['selected']['observation_view']['proposal_search'];names=['Joint%d'%i for i in range(1,7)]
model=Path('.runtime/candidate_models/expanded_metric_20260929/alicia_expanded_metric_candidate.urdf').read_text()
fk=SerialUrdfFk(model,names);assert fk.model_sha256==search['model_sha256']
args=dict(source=source,current_joint_positions=search['current_joint_positions'],fk=fk,
    gripper=GripperGeometry(.05,.002,np.array([.0434,.0286,.06]),np.array([.1175,.155,.0774]),.003),
    opening_width_m=.05,gaps=audit['observe_return']['measured_missing_sides'],depth_uncertainty_m=.003,
    side_depth_uncertainty_m=search['optical_side_uncertainty_m'],joint_error_bounds_rad=search['joint_error_bounds_rad'],
    max_directions=2,max_iterations=96,max_seconds=2.,proposal_image_reserve_px=4.)
started=time.perf_counter();rows,report=views.generate_reachable_observation_views(**args)
result=dict(scope='OFFLINE_CURRENT_MEASURED_SOURCE_ENDPOINT_PROPOSALS_NO_AUTHORITY',request_id=19,
    source_stamp_ns=source_stamp,raw_depth_hash_matches=meta['raw_depth_hash_matches'],raw_mask_hash_matches=meta['raw_mask_hash_matches'],
    support_point_count=len(support),support_points_sha256=hashlib.sha256(np.ascontiguousarray(support).tobytes()).hexdigest(),
    original_support_points_sha256=meta['source_audit']['support_points_sha256'],
    candidates=rows,audit=report,wall_sec=time.perf_counter()-started,
    limitations=['New height metadata reconstructed from original acquired source, not present in old runtime receipt.',
      'Saved exact camera TF may have floating reconstruction differences; no new authorization is inferred.',
      'Only original endpoint gates plus new arrival projection; actual MoveIt full collision/path and execution still required.'],
    source_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths})
assert result['source_sha256']==source_before
(OUT/'source_inputs_manifest.json').write_text(json.dumps(dict(
    source_before=source_before,source_after=result['source_sha256'],inputs_manifest=str(ROOT/'request19_source/manifest.json'),
    script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()),indent=2)+'\n')
assert not (OUT/'result.json').exists(), 'Do not overwrite a prior result'
(OUT/'result.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(dict(candidate_count=len(rows),status=report['status'],wall_sec=result['wall_sec'],directions=report['directions'])),flush=True)
