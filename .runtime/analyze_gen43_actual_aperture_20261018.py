#!/usr/bin/env python3
"""Read a sealed record and reproduce its pure aperture geometry; no ROS node."""
import base64
import hashlib
import io
import json
from pathlib import Path
import socket
import sys
import time

W = Path(__file__).resolve().parent.parent
E = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
N = E / 'ros_log/full_latest_gen41_restart_20261018T011222Z'
G = E / 'grasp_records/20261018T052442.803648Z_891dd95b290a47f8a5ee478e99e37329'
OUT = N / 'verification/gen43_actual_aperture_failure_r1'
PLAN = '5c6b13b03534529f4ea083be'
sys.path.insert(0, str(W / 'src/alicia_flexible_grasp_supervisor/src'))
attempts = []
def blocked(*args, **kwargs):
    attempts.append('network_socket')
    raise RuntimeError('offline geometry probe forbids network')
class BlockedSocket(socket.socket):
    def __new__(cls, *args, **kwargs):
        return blocked(*args, **kwargs)
socket.socket = BlockedSocket

import numpy as np
import rosbag
from tf.transformations import quaternion_matrix
from alicia_flexible_grasp.grasp.contact_execution import plan_aperture_evaluator
from alicia_flexible_grasp.grasp.contact_evidence import decode_contact_evidence
from alicia_flexible_grasp.grasp.rich_plan_integrity import pose_values, plan_id_matches_content

def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        for block in iter(lambda: f.read(4*1024*1024), b''):
            h.update(block)
    return h.hexdigest()

def write(name, value):
    path = OUT / name
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False)+'\n')
    return dict(path=str(path), sha256=sha(path))

def plain(v):
    if hasattr(v, 'to_nsec'):
        return v.to_nsec()
    if hasattr(v, '__slots__'):
        return {k: plain(getattr(v, k)) for k in v.__slots__}
    if isinstance(v, (list, tuple)):
        return [plain(x) for x in v]
    return v

def transform(pose):
    x = pose_values(pose)
    value = quaternion_matrix(x[3:])
    value[:3, 3] = x[:3]
    return value

def main():
    assert E.is_dir() and G.is_dir()
    manifest = json.loads((G/'manifest.json').read_text())
    assert manifest['recorder_closed'] is True and manifest['state'] == 'complete'
    assert manifest['result'] == 'failure'
    b = G/'keyframes.bag'
    row = next(x for x in manifest['files'] if x['path'] == b.name)
    assert b.stat().st_size == row['size_bytes']
    assert sha(b) == row['sha256']
    OUT.mkdir(exist_ok=False)
    log = N/'gen43_runtime/compute_gateway_deployment_r1/consumers.log'
    lines = log.read_text().splitlines()
    aperture_line = next(x for x in lines if 'Measured contact aperture: ' in x and '"plan_id": "'+PLAN+'"' in x)
    actual_report = json.loads(aperture_line.split('Measured contact aperture: ', 1)[1])
    compensation_line = next(x for x in lines if 'Measured fixed endpoint compensation completed: ' in x and '"plan_id": "'+PLAN+'"' in x)
    compensation = json.loads(compensation_line.split('Measured fixed endpoint compensation completed: ', 1)[1])
    plans, feedback = [], []
    topics = ['/grasp_6d/plan_enriched', '/grasp_6d/preview_plan_enriched', '/alicia_d/accepted_joint_states']
    with rosbag.Bag(str(b), 'r') as bag:
        for topic, msg, received in bag.read_messages(topics=topics):
            if topic.endswith('joint_states'):
                if msg.header.frame_id == 'sdk_measured' and 1792301294000000000 <= msg.header.stamp.to_nsec() <= 1792301303500000000:
                    feedback.append(dict(topic=topic, received_ns=received.to_nsec(), message=plain(msg)))
            elif getattr(msg, 'plan_id', None) == PLAN:
                plans.append((topic, msg, received.to_nsec()))
    assert plans, 'No exact frozen plan found'
    topic, plan, received = plans[-1]
    assert plan_id_matches_content(plan)
    buffer = io.BytesIO()
    plan.serialize(buffer)
    wire = buffer.getvalue()
    source = write('frozen_plan.json', dict(topic=topic, received_ns=received,
        message_type=plan._type, md5sum=plan._md5sum, message=plain(plan),
        wire_sha256=hashlib.sha256(wire).hexdigest(), wire_base64=base64.b64encode(wire).decode()))
    actual = np.asarray(actual_report['actual_T_base_tool0'], dtype=float)
    stage = transform(plan.poses[0])
    contact = transform(plan.poses[2])
    translation_only, rotation_only = stage.copy(), stage.copy()
    translation_only[:3, 3] = actual[:3, 3]
    rotation_only[:3, :3] = actual[:3, :3]
    # Physical dimensions are the existing contract defaults. Exact equality
    # against the runtime report below checks every aperture-relevant result;
    # these computations create no runtime configuration or motion authority.
    evaluate = plan_aperture_evaluator(plan, {}, actual_report['measured_opening_width_m'], 0)
    computed = {name: evaluate(T) for name, T in (
        ('nominal_frozen_stage', stage), ('actual_measured_stage', actual),
        ('translation_only_diagnostic', translation_only), ('rotation_only_diagnostic', rotation_only))}
    match_fields = ['union_jaw_interval_m', 'negative_reach_m', 'positive_reach_m',
        'union_required_open_width_m', 'aperture_margin_m', 'projected_contact_center_base',
        'inferred_required_open_width_m', 'opening_fit_clearance_each_side_m']
    for key in match_fields:
        assert np.allclose(computed['actual_measured_stage'][key], actual_report[key], rtol=0., atol=1e-12), key
    assert computed['actual_measured_stage']['code'] == actual_report['code']
    R = actual[:3, :3] @ stage[:3, :3].T
    angle = np.arccos(np.clip((np.trace(R)-1.)/2., -1., 1.))
    evidence = decode_contact_evidence(plan, required=True)
    outputs = {
        'runtime_aperture_report': write('runtime_aperture_report.json', actual_report),
        'runtime_compensation_report': write('runtime_compensation_report.json', compensation),
        'accepted_feedback_window': write('accepted_feedback_window.json', feedback),
        'pure_geometry_comparison': write('pure_geometry_comparison.json', computed)}
    report = dict(kind='gen43_sealed_actual_aperture_failure_analysis', completed_ns=time.time_ns(),
        record=str(G), record_manifest_sha256=sha(G/'manifest.json'), bag_sha256=row['sha256'],
        bag_manifest_verified=True, record_result='failure', plan_id=PLAN, exact_frozen_plan=source,
        matching_plan_messages=len(plans), runtime_failure_pose_index=0,
        first_failure_call='grasp_task_node.py:3845 _validate_measured_aperture(plan,gcfg,0,reached contact pregrasp)',
        rejection='measured_contact.py:164-169 unchanged frozen union symmetric jaw aperture',
        contact_geometry_source=evidence['measurement_source'], contact_evidence_schema=evidence['schema_version'],
        planned_stage=stage.tolist(), planned_contact=contact.tolist(),
        frozen_contact_center_base=evidence['contact_center_base'], frozen_object_geometry=plain(plan.object_geometry),
        stage_translation_residual_base_m=(actual[:3,3]-stage[:3,3]).tolist(),
        stage_position_residual_m=float(np.linalg.norm(actual[:3,3]-stage[:3,3])),
        stage_orientation_residual_deg=float(np.degrees(angle)),
        projected_contact_center_delta_base_m=(np.asarray(actual_report['projected_contact_center_base'])-np.asarray(evidence['contact_center_base'])).tolist(),
        runtime_reproduction_matches_fields=match_fields, numeric_match_tolerance=1e-12,
        calculation_config='existing physical contract defaults; actual aperture numerics strictly reproduced',
        counterfactuals_are_diagnostic_only=True, additive_causal_decomposition_claimed=False,
        inferred_model_present=False, uncertainty_not_changed=True, actual_opening_not_changed=True,
        output=outputs, external_boundary_attempts=attempts, ros_init_called=False,
        temperature_topics_or_values_read=False, motion_authorized=False, source_modified=False,
        source_sha256={str(W/'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/grasp'/f):sha(W/'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/grasp'/f) for f in ['contact_execution.py','measured_contact.py']},
        helper_sha256=sha(Path(__file__)))
    assert not attempts
    result=write('result.json',report)
    write('seal.json',dict(result=result, outputs=outputs, frozen_plan=source))
    print(json.dumps(dict(result=result, computed={k:{x:v.get(x) for x in ['ok','code','union_jaw_interval_m','required_open_width_m','aperture_margin_m','projected_contact_center_base']}for k,v in computed.items()}, residual_m=report['stage_position_residual_m'],residual_deg=report['stage_orientation_residual_deg'])))

if __name__ == '__main__':
    main()
