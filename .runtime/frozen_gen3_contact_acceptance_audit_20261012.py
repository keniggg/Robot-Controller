"""Read immutable GEN3 archives; save exact acceptance evidence, never ROS calls."""
from pathlib import Path
import hashlib
import io
import json
import rosbag

BASE = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
NEW = BASE / 'ros_log/full_latest_frozen_no_skin_20261012T021050Z'
RECORD = BASE / 'grasp_records/20261012T032110.570728Z_a1a1c737f44d453aa25a5585504da760'
REQUEST = BASE / 'ros_log/candidate_simulation_full_start_20261012T021050Z_jc7jctxv/service/requests/1791775355523069798_1c1edc921a8041d1a0b4d72461d27072'
PLAN = '9e4f0ab73da260be18b162a8'
OUT = NEW / 'gen3_analysis'
OUT.mkdir(exist_ok=True)

def digest(data):
    return hashlib.sha256(data).hexdigest()

sources = []
def archive(source, destination):
    raw = source.read_bytes()
    (OUT / destination).write_bytes(raw)
    sources.append(dict(source=str(source), archived=destination, sha256=digest(raw)))
    return json.loads(raw)

audit = archive(RECORD / 'keyframes_audits/bdab82e6c9bd304b4aa8c242c03d6399bcf783f8c178e0d1ce0189b34e4aa025.json', 'accepted_preview_gate_audit.json')
request = archive(REQUEST / 'request.raw.json', 'mujoco_request.raw.json')
response = archive(REQUEST / 'response.json', 'mujoco_response.json')
latest = BASE / 'ros_log/planning_audits/grasp6d_gate_audit_latest.json'
if json.loads(latest.read_text()).get('plan_id') == PLAN:
    archive(latest, 'grasp6d_gate_audit_latest_at_review.json')
assert audit['plan_id'] == request['plan_id'] == response['plan_id'] == PLAN
selected = audit['selected']
precheck = selected['contact_sequence_precheck']
assert precheck['success'] is True and precheck['plan_id'] == PLAN
assert selected['final_strict_plan_id'] == PLAN
assert all(selected['moveit'][k] is True for k in ['collision_free','ik_valid','planning_success','reachable','within_joint_limits'])

plan_messages = []
preview = None
with rosbag.Bag(str(RECORD / 'keyframes.bag')) as bag:
    for topic, message, stamp in bag.read_messages(topics=['/grasp_6d/plan_enriched','/grasp_6d/preview_plan_enriched']):
        plan_messages.append(dict(topic=topic, receipt_stamp_ns=stamp.to_nsec(), plan_id=message.plan_id, valid=message.valid, diagnostic=message.diagnostic))
        if message.plan_id == PLAN:
            wire = io.BytesIO()
            message.serialize(wire)
            raw = wire.getvalue()
            (OUT / 'accepted_preview.rosmsg').write_bytes(raw)
            preview = dict(topic=topic, receipt_stamp_ns=stamp.to_nsec(), serialized_sha256=digest(raw), required_open_width_m=message.required_open_width_m,
                contact_geometry_evidence=json.loads(message.contact_geometry_evidence))
assert preview is not None
evidence = preview['contact_geometry_evidence']
assert evidence['schema_version'] == 2
expected_close = preview['required_open_width_m'] - 2 * evidence['opening_fit_clearance_each_side_m'] - .002
assert abs(expected_close-request['gripper']['close_target_inner_gap_m']) < 1e-14
assert not any(row['plan_id']==PLAN and row['topic']=='/grasp_6d/plan_enriched' for row in plan_messages)

curves = []
def curves_walk(value, path):
    if isinstance(value, dict):
        if 'trajectory_sha256' in value and 'certified_minimum_clearance_m' in value:
            assert value['plan_id'] == PLAN
            curves.append(dict(path=path, **value))
        for key, item in value.items():
            curves_walk(item, path+'/'+key)
    elif isinstance(value, list):
        for i, item in enumerate(value):
            curves_walk(item, path+'/'+str(i))
curves_walk(precheck, 'contact_sequence_precheck')
proposal = selected['contact_height_window_search']['proposals'][0]
assert proposal['accepted'] is True and proposal['plan_id'] == PLAN
report = dict(
    scope='GEN3 archived live planning, simulation and ROS-message review; no ROS calls, no execution authorization',
    plan_id=PLAN, record_id=RECORD.name, sources=sources,
    preview=preview, recorded_plan_messages=plan_messages,
    publication=dict(preview_recorded=True, execution_plan_recorded=False, audit_outcome=audit['outcome'], conclusion='Accepted contact preview was not promoted to published execution plan in this recording.'),
    planning=dict(moveit=selected['moveit'], stage_policy=selected['moveit_execution_stage_policy'], precheck_success=precheck['success'], precheck_scope=precheck['scope'],
        checked_stage_names=precheck['checked_stage_names'], current_bias_rad=precheck['contact_prediction']['sdk_minus_encoder_rad'], curves=curves,
        future_following_verified=False, hardware_response_verified=False,
        limitations='Recorded full candidate sequence and constant-current-bias commanded-curve prediction passed. Actual execution must revalidate fresh measured state. Continuous analytical geometry scope is Link6/7/8; complete MoveIt result separately passed. Lift path was not authorized.'),
    height=dict(original_plan_id=selected['contact_height_window_search']['original_plan_id'], original_placement=selected['contact_height_placement'],
        retry_normal_translation_m=proposal['placement']['normal_translation_m'], retry_placement=proposal['placement'], proposal_count=len(selected['contact_height_window_search']['proposals']), maximum_proposals=selected['contact_height_window_search']['maximum_proposals'],
        final_margin=selected['final_contact_height_margin']),
    closure=dict(actual_fit_clearance_each_side_m=evidence['opening_fit_clearance_each_side_m'], measured_contact_width_m=evidence['measured_contact_width_m'], plan_opening_wire_m=preview['required_open_width_m'],
        expected_close_from_schema2_m=expected_close, request_close_m=request['gripper']['close_target_inner_gap_m'], settled_state=response['lift_evidence']['settled_gripper_state'],
        close_hold_gate= {k:v for k,v in selected['mujoco_selection'].items() if k!='response'},
        raw_simulation_ok=response['simulation_ok'], raw_contact_success=response['contact_success'], raw_lift_success=response['lift_success'], raw_failure_code=response['failure_code'], raw_failure_reason=response['failure_reason'],
        conclusion='Loaded opposed closure passed in simulation. Subsequent simulated lift failed. Current close/hold-only client gate explicitly accepted with MUJOCO_LIFT_DIAGNOSTIC_ONLY. No physical closure happened in this trial.'),
    root_blocker='CONTACT_PREGRASP_PUBLISHED_PLAN_REQUIRED: preview existed but contact execution publication absent; no new geometry/closure failure at this boundary.')
destination=OUT/'accepted_contact_plan_audit.json'
destination.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
print(json.dumps(dict(path=str(destination),sha256=digest(destination.read_bytes()),curve_count=len(curves),schema=evidence['schema_version'],close_m=expected_close),ensure_ascii=False))
