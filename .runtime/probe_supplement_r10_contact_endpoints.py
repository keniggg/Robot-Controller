"""Frozen endpoint ordering diagnosis; no ROS I/O or motion authority."""
from pathlib import Path
from types import SimpleNamespace as NS
import json, os, time, hashlib

W = Path(__file__).resolve().parents[1]
E = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
R = E/'ros_log/full_latest_gen41_restart_20261018T011222Z/gen58_resume_r1/observation_pose_fix/held_lattice_fix_r1/seed_wait_fix_r1/contact_feedback_fix_r1/actual_gen63_baseline_validation_20261023T041149Z/high_accuracy_grasp_r1/arrival_margin_repair_r1'
D = R/'supplemental_view_r10/contact_diagnosis'
assert Path('/media/zhuyupei/SanStand').is_mount()
os.environ['OFFLINE_BOUNDARY_LOG'] = str(D/'endpoint_probe_boundary.jsonl')
exec((R/'guard.py').read_bytes())
import numpy as np
from alicia_flexible_grasp.robot.observation_path_guard import SerialUrdfFk, gripper_target_overlap
from alicia_flexible_grasp.robot.stationary_following import ARM_NAMES
from alicia_flexible_grasp.robot.contact_sequence_precheck import contact_precheck_gripper
from alicia_flexible_grasp.grasp.gripper_geometry import _stage_boxes, _box_corners

index = json.loads((D/'failed_prechecks.json').read_text())
source = Path(index['audit_path'])
assert hashlib.sha256(source.read_bytes()).hexdigest() == index['audit_sha256']
audit = json.loads(source.read_text())
receipt = json.loads((D/'first_curve_replay.json').read_text())
fk = SerialUrdfFk(receipt['model']['xml'], ARM_NAMES)
gripper = contact_precheck_gripper(receipt['config']['gripper_geometry'])
bias = np.asarray(receipt['following_sample']['sdk_positions_rad']) - receipt['following_sample']['accepted_positions_rad']
opening = receipt['opening_m']
g = audit['replay_geometry']
scene = NS(center=np.array(g['obb_center_base_m']), rotation=np.array(g['R_base_obb']),
           size=np.array(g['obb_size_xyz_m']), normal=np.array(g['support_normal_base']),
           offset=g['support_offset_m'])
def check(q):
    try:
        t = fk(q)
        rows = []
        for body, center, size in _stage_boxes(t, gripper, opening, 'y', 'z'):
            overlap, proof = gripper_target_overlap(fk, gripper, opening, body, t, center, size, scene)
            clearance = float(np.min(_box_corners(center, t[:3,:3], size)@scene.normal + scene.offset))
            rows.append(dict(body=body, target_overlap=bool(overlap), support_clearance_m=clearance,
                             cad_gap_m=proof.get('gap_m') if proof else None))
        return dict(checked=True, clear=all(not r['target_overlap'] and r['support_clearance_m'] >= gripper.support_clearance_m for r in rows), bodies=rows)
    except ValueError as exc:
        return dict(checked=False, reason=str(exc))
started = time.monotonic()
rows = []
for row in audit['stable_evaluations']:
    hints = (row.get('endpoint_ik_search_probe') or {}).get('stages', [])
    grasp = next((h for h in hints if h['stage']=='grasp' and h.get('joint_solution')), None)
    if grasp is None:
        continue
    s = grasp['joint_solution']; values = dict(zip(s['name'],s['position_rad']))
    q = np.array([values[n] for n in ARM_NAMES])
    r = dict(source_index=row['source_index'], variant_index=row['variant_index'],
             tilt_deg=(row.get('adaptive_stage_profile') or {}).get('tilt_deg'),
             nominal=check(q), predicted_command=check(q+bias),
             exact_strict_check_was_attempted=bool(row.get('contact_sequence_precheck')),
             q_hint=q.tolist(), sdk_minus_encoder_rad=bias.tolist())
    rows.append(r)
    print(json.dumps(dict(source_index=r['source_index'],variant=r['variant_index'],tilt=r['tilt_deg'],
        nominal=r['nominal'].get('clear'),command=r['predicted_command'].get('clear'),
        rejected_bodies=[x['body'] for x in r['predicted_command'].get('bodies',[]) if x['target_overlap']],
        support_min_mm=min([x['support_clearance_m']*1000 for x in r['predicted_command'].get('bodies',[])] or [None]),
        attempted=r['exact_strict_check_was_attempted'])),flush=True)
result=dict(scope='different pre-registration endpoint IK branch advisory only',
            path_verified=False, execution_authorized=False, candidates_removed=0,
            source_path=str(source), source_sha256=index['audit_sha256'],
            receipt_sha256=hashlib.sha256((D/'first_curve_replay.json').read_bytes()).hexdigest(),
            model_sha256=fk.model_sha256, rows=rows, elapsed_sec=time.monotonic()-started,
            script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
(D/'endpoint_order_probe.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
print('elapsed_sec',result['elapsed_sec'])
