"""Replay the sealed supplementary-view candidate factory, without ROS I/O."""
from pathlib import Path
from types import SimpleNamespace as NS
from dataclasses import asdict, is_dataclass
import hashlib, gzip, importlib.util, json, os, threading, time

W = Path(__file__).resolve().parents[1]
E = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
R = E/'ros_log/full_latest_gen41_restart_20261018T011222Z/gen58_resume_r1/observation_pose_fix/held_lattice_fix_r1/seed_wait_fix_r1/contact_feedback_fix_r1/actual_gen63_baseline_validation_20261023T041149Z/high_accuracy_grasp_r1/arrival_margin_repair_r1'
OUT = R/'supplemental_view_r3/collision_diagnosis'
AUDIT = E/'grasp_records/20261023T080014.824313Z_3e10cdc5160c4c0f879b83412aa4f27c/keyframes_audits/029f2e570973dae7b025c1262c8b8d5f8bfab98c961338594995ae87b04e48a3.json'
assert Path('/media/zhuyupei/SanStand').is_mount()
os.environ['OFFLINE_BOUNDARY_LOG'] = str(OUT/'boundary.jsonl')
exec((R/'guard.py').read_text())
import numpy as np
import rospy
from alicia_flexible_grasp.vision.multiview_surface import FusedTargetSurface, SurfaceView
from alicia_flexible_grasp.vision.target_observation import TargetTrackIdentity
from alicia_flexible_grasp.grasp.gripper_geometry import GripperGeometry, _stage_boxes, _obb_overlap
from alicia_flexible_grasp.robot.finger_cad_separation import load_finger_cad_model

def sha(data): return hashlib.sha256(data).hexdigest()
def plain(x):
    if isinstance(x, np.ndarray): return x.tolist()
    if isinstance(x, np.generic): return x.item()
    if is_dataclass(x): return plain(asdict(x))
    if isinstance(x, dict): return {str(k):plain(v) for k,v in x.items()}
    if isinstance(x, (tuple,list)): return [plain(v) for v in x]
    return x

d = json.loads(AUDIT.read_text())
assert sha(AUDIT.read_bytes()) == AUDIT.stem
params = json.loads((OUT/'readonly_parameters.json').read_text())['values']
def get_param(name, default=None):
    for root, values in params.items():
        if name == root: return values
        if name.startswith(root+'/'):
            try:
                for part in name[len(root)+1:].split('/'): values = values[part]
                return values
            except (KeyError,TypeError): return default
    return default
rospy.get_param = get_param
spec = importlib.util.spec_from_file_location('sealed_mode_remote', W/'src/alicia_grasp_modes/scripts/mode_aware_remote_grasp6d.py')
mode = importlib.util.module_from_spec(spec); spec.loader.exec_module(mode)
remote = mode.original
node = mode.ModeAwareRemoteGrasp6DNode.__new__(mode.ModeAwareRemoteGrasp6DNode)
node._mode_selection = d['grasp_mode_selection']
node._geometry_state_lock = threading.RLock()
node._observe_return_phase = 'observing_contact'
node._measurement_remote_request = {'state':'qualified'}
node.near_field_planning_active = True
rcfg = params['/grasp_6d']['remote']; gg = rcfg['gripper_geometry']
node.gripper_geometry = GripperGeometry(gg['max_inner_gap_m'],gg['width_safety_margin_per_side_m'],
    np.array(gg['finger_box_xyz_m']),np.array(gg['palm_box_xyz_m']),gg.get('support_clearance_m',.003))
node.gripper_tool_jaw_axis = gg.get('tool_jaw_axis','y')
node.gripper_tool_finger_length_axis = gg.get('tool_finger_length_axis','z')
node.opening_fit_clearance_each_side_m = remote.load_opening_fit_clearance_config(gg,node.gripper_geometry)
node.tabletop_geometry_enabled,node.tabletop_geometry_config,node.hybrid_merge_config = remote.load_tabletop_geometry_config(rcfg,node.gripper_geometry,node.opening_fit_clearance_each_side_m)
node.adaptive_stage_limits = remote.load_adaptive_stage_config(rcfg)
node.candidate_min_downward_approach_cos = rcfg['candidate_min_downward_approach_cos']
node.candidate_max_final_approach_lateral_m = rcfg['candidate_max_final_approach_lateral_m']
node.grasp_config = params['/grasp']
node.gripper_physical_open_width_m = .05
original = d['remote_diagnostics']['tabletop_geometry']
node._runtime_execution_position_error_m = lambda: original['adaptive_stage_profiles'][0]['execution_position_error_m']
ref = d['multiview_surface']['fused_scene_geometry']['surface_replay']['references']['snapshot']
raw = (Path(ref['store_root'])/ref['relative_path']).read_bytes()
assert sha(raw)==ref['gzip_sha256']
raw = gzip.decompress(raw); assert sha(raw)==ref['object_sha256']
s = json.loads(raw); identity = TargetTrackIdentity(**s['identity'])
surface = FusedTargetSurface(identity,np.array(s['points_base']),np.array(s['view_indices']),
    tuple(s['view_stamps_ns']),view_sources=tuple(tuple(v) for v in s['view_sources']))
v=s['reference']; reference=SurfaceView(identity,v['stamp_ns'],np.array(v['points_base']),
    np.array(v['support_normal_base']),v['support_offset_m'],v['edge_clearance_px'])
node._snapshot_multiview_surface = lambda: (surface,reference)
g=d['replay_geometry']
geometry=NS(object_points_base=np.array(g['object_points_base_m']),center_base=np.array(g['obb_center_base_m']),
    axes_base=np.array(g['R_base_obb']),size_xyz_m=np.array(g['obb_size_xyz_m']),
    support_normal_base=np.array(g['support_normal_base']),support_offset_m=g['support_offset_m'])
snapshot=NS(quality=NS(depth_mad_m=original['depth_spatial_mad_m'],depth_repeatability_m=original['depth_repeatability_m']))
started=time.monotonic()
candidates,diagnostics=node._generate_tabletop_candidates(geometry,snapshot)
assert len(candidates)==original['materialized_count'], (len(candidates),original['materialized_count'])
np.testing.assert_allclose([p['tilt_deg'] for p in diagnostics['adaptive_stage_profiles']],
                          [p['tilt_deg'] for p in original['adaptive_stage_profiles']],rtol=0,atol=1e-10)
prepared=NS(geometry=geometry,snapshot=snapshot,near_field=True,
    stamp=rospy.Time(d['snapshot_stamp_ns']//10**9,d['snapshot_stamp_ns']%10**9))
gates=[]; node._record_geometry_gate_result=lambda gate: gates.append(gate)
xml=(W/'deployment/2026-10-08/runtime/candidate_models/expanded_metric_20260929/alicia_expanded_metric_candidate.urdf').read_text()
model=load_finger_cad_model(xml); assert model is not None
rows=[]
for c in candidates:
    gates.clear()
    try:
        node._normalize_tabletop_candidate(prepared,c)
        failure=None
    except remote.CandidateContractError as exc:
        failure=dict(code=exc.code,reason=str(exc))
    T=np.array(c.T_base_tool0); proofs=[]
    for body,center,size in _stage_boxes(T,node.gripper_geometry,.05,'y','z'):
        old_overlap=_obb_overlap(center,T[:3,:3],size,geometry.center_base,geometry.axes_base,geometry.size_xyz_m)
        proof=(model.separation(body,.05,T,geometry.center_base,geometry.axes_base,geometry.size_xyz_m)
               if body in ('left_finger','right_finger') else None)
        proofs.append(dict(body=body,old_box_overlap=old_overlap,cad_separation=proof))
    rows.append(dict(source_index=c.source_index,variant_index=c.variant_index,audit=dict(c.audit),
        T_base_tool0=T,contact_center_base=c.contact_center_base,required_open_width_m=c.required_open_width_m,
        failure=failure,gate=gates[-1] if gates else None,terminal_cad_proofs=proofs))
recorded=json.loads((R/'supplemental_view_r3/local_candidate_rejections.json').read_text())['samples']
assert [(x['source_index'],x['variant_index'],x['failure']['reason']) for x in rows[:len(recorded)]] == [
    (x['source_index'],x['variant_index'],x['reason']) for x in recorded]
out=dict(scope='sealed surface and original candidate factory; terminal CAD diagnosis grants no path or motion authority',
    audit_path=str(AUDIT),audit_sha256=AUDIT.stem,surface_reference=ref,
    original_materialized_count=original['materialized_count'],replayed_count=len(rows),
    first_recorded_rejections_match=True,diagnostics=diagnostics,rows=rows,
    wall_sec=time.monotonic()-started,motion_commands_sent=False,
    source_sha256={str(Path(p).resolve().relative_to(W)):sha(Path(p).read_bytes()) for p in (__file__,mode.__file__,remote.__file__)})
(OUT/'factory_replay.json').write_text(json.dumps(plain(out),ensure_ascii=False,indent=2)+'\n')
print(json.dumps(dict(candidates=len(rows),all_rejected=all(r['failure'] for r in rows),
    terminal_cad_separated=sum(all(not p['old_box_overlap'] or p['cad_separation'] and p['cad_separation']['separated'] for p in row['terminal_cad_proofs']) for row in rows),
    wall_sec=out['wall_sec'],output=str(OUT/'factory_replay.json'))))
