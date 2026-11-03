"""Original contact factory over reconstructed sealed measurements; no ROS I/O."""
from pathlib import Path
from types import SimpleNamespace as NS
from dataclasses import asdict,is_dataclass
import json,hashlib,math
import numpy as np
import yaml,rospy
from tf.transformations import quaternion_from_matrix
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState
from alicia_flexible_grasp.vision.multiview_surface import FusedTargetSurface,SurfaceView
from alicia_flexible_grasp.vision.target_observation import TargetTrackIdentity
from alicia_flexible_grasp.robot.stationary_following import stationary_following_error,ARM_NAMES
from alicia_flexible_grasp.robot.observation_path_guard import SerialUrdfFk
from alicia_flexible_grasp.grasp.gripper_geometry import GripperGeometry,_stage_boxes,_box_corners,_obb_overlap,evaluate_explicit_candidate
from alicia_flexible_grasp.grasp.contact_height_placement import place_contact_height,contact_height_window_proposals
from alicia_flexible_grasp.grasp.contact_height_margin import evaluate_contact_height_margin
import test_remote_grasp6d_streaming as fixtures

remote=fixtures.remote_node
ROOT=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_ros_20261009T003407Z/gen26_contact_analysis')
STAMP=1791546473600880622
meta=json.loads((ROOT/'source_inputs/manifest.json').read_text())
reconstruction=json.loads((ROOT/'fused_reconstruction_final/result.json').read_text())
raw=np.load(ROOT/'fused_reconstruction_final'/('%d_enclosed_geometry.npz'%STAMP))
surface_data=np.load(ROOT/'fused_reconstruction_final'/('%d_surface.npz'%STAMP))
cfg=yaml.safe_load(Path('src/alicia_flexible_grasp_supervisor/config/grasp_params.yaml').read_text())
rcfg=cfg['grasp_6d']['remote'];gg=rcfg['gripper_geometry']
node=remote.RemoteGrasp6DNode.__new__(remote.RemoteGrasp6DNode)
node.gripper_geometry=GripperGeometry(gg['max_inner_gap_m'],gg['width_safety_margin_per_side_m'],np.array(gg['finger_box_xyz_m']),np.array(gg['palm_box_xyz_m']),gg.get('support_clearance_m',.003))
node.gripper_tool_jaw_axis=gg.get('tool_jaw_axis','y');node.gripper_tool_finger_length_axis=gg.get('tool_finger_length_axis','z')
node.opening_fit_clearance_each_side_m=remote.load_opening_fit_clearance_config(gg,node.gripper_geometry)
node.tabletop_geometry_enabled,node.tabletop_geometry_config,node.hybrid_merge_config=remote.load_tabletop_geometry_config(rcfg,node.gripper_geometry,node.opening_fit_clearance_each_side_m)
node.adaptive_stage_limits=remote.load_adaptive_stage_config(rcfg)
node.candidate_min_downward_approach_cos=rcfg.get('candidate_min_downward_approach_cos',.65)
node.candidate_max_final_approach_lateral_m=rcfg.get('candidate_max_final_approach_lateral_m',.02)
node.grasp_config=cfg['grasp'];node.gripper_physical_open_width_m=.05
node._runtime_execution_position_error_m=lambda: meta['execution_error_snapshots'][0]['message']['position_error_m']
identity=TargetTrackIdentity(meta['target_epoch'],meta['target_track_id'])
surface=FusedTargetSurface(identity,surface_data['current_points'],surface_data['view_indices'],tuple(meta['surface_source_stamps_ns']), view_sources=tuple(tuple(x) for x in reconstruction['online_fused_geometry_checks'][-1]['view_sources']))
REG=surface_data['last_transform'];inverse=np.linalg.inv(REG)
root_points=np.load(ROOT/'fused_reconstruction_final'/('%d_geometry.npz'%meta['surface_source_stamps_ns'][0]))['object_points_base']
reference=SurfaceView(identity,meta['surface_source_stamps_ns'][0],root_points@inverse[:3,:3].T+inverse[:3,3],
    inverse[:3,:3]@surface_data['support_normal'],float(surface_data['support_offset']+surface_data['support_normal']@REG[:3,3]),reconstruction['views'][0]['edge_clearance_px'])
node._snapshot_multiview_surface=lambda:(surface,reference)
geometry=NS(**{k:raw[k] for k in ['object_points_base','center_base','axes_base','size_xyz_m','support_normal_base','support_offset_m']})
# Exact temporal quality is supplied by original three-frame fusion, not inferred
# from spatial variation or the model-space execution bias.
quality=json.loads((ROOT/'fused_reconstruction_final/enclosed_geometry_manifest.json').read_text())['snapshot_quality']
if not quality:raise ValueError('original snapshot quality missing')
snapshot=NS(quality=NS(**quality))
candidates,diagnostics=node._generate_tabletop_candidates(geometry,snapshot=snapshot,contact_execution_phase=True)
def convert(v):
    if isinstance(v,np.ndarray):return v.tolist()
    if isinstance(v,np.generic):return v.item()
    if is_dataclass(v):return {k:convert(x) for k,x in asdict(v).items()}
    if isinstance(v,dict):return {k:convert(x) for k,x in v.items()}
    if isinstance(v,(tuple,list)):return [convert(x) for x in v]
    return v
rows=[]
for c in candidates:
    T=np.array(c.T_base_tool0);center=np.array(c.contact_center_base)
    _,evidence,bounds,axis=node._current_bilateral_surface_measurement(center,T[:3,:3])
    geo=node._contact_height_geometry(T,geometry,center,bounds,axis,.002)
    placed,placement=place_contact_height(translation_evidence_limit_m=node._contact_height_evidence_limit(),**geo)
    rows.append(dict(source_index=c.source_index,variant_index=c.variant_index,audit=dict(c.audit),
        required_open_width_m=c.required_open_width_m,contact_center_base=center,T_base_tool0=T,
        evidence=asdict(evidence),bounds=bounds,placement=placement,placed_T=placed))
out=dict(scope='OFFLINE_ORIGINAL_FACTORY_AND_MEASURED_HEIGHT_NO_AUTHORITY',snapshot_stamp_ns=STAMP,
    candidate_count=len(candidates),diagnostics=diagnostics,rows=rows,
    source_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in [Path(__file__),Path(remote.__file__)]},
    limitations=['Original canceled request28 candidate IDs/order and failed MoveIt path were not published.',
        'Reconstructed raw and registered surfaces are compared to available original audits and ObjectGeometry, not assumed bitexact.'])
(ROOT/'factory_replay.json').write_text(json.dumps(convert(out),indent=2)+'\n')
print(json.dumps(dict(candidates=len(rows),height_pass=sum(x['placed_T'] is not None for x in rows),result=str(ROOT/'factory_replay.json'))))
