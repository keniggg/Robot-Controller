#!/usr/bin/env python3
"""Read sealed GEN14 logs/bag only; never initialize a ROS node."""
from pathlib import Path
import collections
import hashlib
import json
import re
import rosbag

NEW = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_restart_20261010T002347Z')
RECORD = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/grasp_records/20261010T060144.544180Z_94750fef3cae477792d13cdcb9d4ade0')
OUT = NEW / 'gen14_analysis'
OUT.mkdir(exist_ok=True)
result = json.loads((RECORD / 'result_summary.json').read_text())
manifest = json.loads((RECORD / 'manifest.json').read_text())
assert result['result'] == 'failure' and manifest['recorder_closed']
topics = ['/perception/unknown/detector_status', '/grasp/near_field_phase', '/grasp/state',
          '/grasp_6d/plan_enriched', '/grasp_6d/preview_plan_enriched',
          '/grasp_mode/observe_return_measurement_request', '/grasp_mode/observe_return_measurement_ack',
          '/alicia_d/accepted_joint_states']
counts = collections.Counter()
support, phases, states, plans, requests, acks, joints = [], [], [], [], [], [], []
first_loss = None
policies = collections.Counter()
with rosbag.Bag(str(RECORD / 'keyframes.bag')) as bag:
    for topic, msg, stamp in bag.read_messages(topics=topics):
        counts[topic] += 1
        ns = stamp.to_nsec()
        if topic == '/perception/unknown/detector_status':
            d = json.loads(msg.data)
            if d.get('generation') != 14:
                continue
            policy = d.get('support_plane_consistency_policy')
            if policy:
                policies[json.dumps(policy, sort_keys=True)] += 1
            metrics = d.get('geometry_metrics') or {}
            if 'support_reference_max_distance_m' in metrics:
                support.append(dict(bag_stamp_ns=ns, source_stamp_ns=d.get('source_stamp_ns'),
                    state=d.get('state'), detail=d.get('detail'), target_lost=d.get('target_lost'),
                    **{k: v for k, v in metrics.items() if k in (
                        'support_reference_max_distance_m', 'support_reference_anchor_distance_m',
                        'support_reference_distance_warning', 'support_reference_warning_distance_m',
                        'support_reference_hard_distance_m', 'support_reference_angle_deg',
                        'support_current_audit_inlier_ratio', 'support_current_audit_residual_m')}))
            if d.get('target_lost') and first_loss is None:
                first_loss = dict(bag_stamp_ns=ns, source_stamp_ns=d.get('source_stamp_ns'),
                    detail=d.get('detail'), first_loss=d.get('first_loss'), frame_timing=d.get('frame_timing'))
        elif topic == '/grasp/near_field_phase':
            phases.append(dict(bag_stamp_ns=ns, stamp_ns=msg.header.stamp.to_nsec(),
                active=msg.active, phase_id=msg.phase_id, planning_stage=msg.planning_stage,
                deadline_ns=msg.deadline.to_nsec()))
        elif topic == '/grasp/state':
            states.append(dict(bag_stamp_ns=ns, stamp_ns=msg.header.stamp.to_nsec(),
                state=msg.state, active=msg.active, success=msg.success, message=msg.message))
        elif topic in ('/grasp_6d/plan_enriched', '/grasp_6d/preview_plan_enriched'):
            plans.append(dict(topic=topic, bag_stamp_ns=ns, source_stamp_ns=msg.header.stamp.to_nsec(),
                plan_id=msg.plan_id, valid=msg.valid, diagnostic=msg.diagnostic,
                candidate_source=msg.candidate_source, pose_count=len(msg.poses)))
        elif topic == '/grasp_mode/observe_return_measurement_request':
            d = json.loads(msg.data)
            requests.append(dict(bag_stamp_ns=ns, state=d.get('state'), request_id=d.get('request_id'),
                arrival_stamp_ns=d.get('arrival_stamp_ns'), arrival_q=(d.get('settle') or {}).get('accepted_positions_rad'),
                view_plan_id=d.get('view_plan_id')))
        elif topic == '/grasp_mode/observe_return_measurement_ack':
            d = json.loads(msg.data)
            acks.append(dict(bag_stamp_ns=ns, **{k:d.get(k) for k in ('request_id','state','participant',
                'accepted','qualified','reason','source_stamps_ns')}))
        elif topic == '/alicia_d/accepted_joint_states':
            if 1791612194000000000 <= msg.header.stamp.to_nsec() <= 1791612197000000000:
                joints.append(dict(bag_stamp_ns=ns, source_stamp_ns=msg.header.stamp.to_nsec(),
                    frame_id=msg.header.frame_id, q=dict(zip(msg.name, msg.position))))

audit_files = []
contact = None
for path in sorted((RECORD / 'keyframes_audits').glob('*.json')):
    data = json.loads(path.read_text())
    audit_files.append(dict(path=str(path), sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                           request_id=data.get('request_id'), plan_id=data.get('plan_id')))
    if data.get('plan_id') == '212cd5c13c08337a1a24a79b':
        contact = data
assert contact is not None
selected = contact['selected']
surface = contact['multiview_surface']
registration = surface['latest_registration']
qualification = next(item for item in requests if item['state'] == 'qualified')
arrival_q = qualification['arrival_q']
if arrival_q is None:
    # Request wire uses a longer explicit field name; read that sealed request.
    with rosbag.Bag(str(RECORD / 'keyframes.bag')) as bag:
        for _, msg, _ in bag.read_messages(topics=['/grasp_mode/observe_return_measurement_request']):
            d = json.loads(msg.data)
            if d.get('state') == 'qualified':
                arrival_q = d.get('arrival_joint_positions_rad')
assert arrival_q is not None
for row in joints:
    row['maximum_delta_from_qualified_arrival_rad'] = max(abs(row['q']['Joint%d' % (i+1)]-arrival_q[i]) for i in range(6))
crossings = [row for row in joints if row['maximum_delta_from_qualified_arrival_rad'] > .004]
warnings = [row for row in support if row.get('support_reference_distance_warning') is True]
post_arrival = [row for row in support if row['bag_stamp_ns'] >= qualification['bag_stamp_ns']]
consumer = (NEW / 'support_offset_policy_deployment/consumers.log').read_text(errors='replace')
events = []
for line in consumer.splitlines():
    if '[Grasp]' in line or 'Unknown target first loss' in line or 'SUPPORT_OFFSET_DIAGNOSTIC_REQUIRED' in line:
        timestamp = re.search(r'\[(\d+\.\d+)\]', line)
        if timestamp and float(timestamp.group(1)) >= 1791612106:
            events.append(dict(stamp_sec=float(timestamp.group(1)), text=line))
contact_start = next(e['stamp_sec'] for e in events if 'moving 6D near-field pregrasp' in e['text'])
loss = next(e['stamp_sec'] for e in events if 'Unknown target first loss' in e['text'])
terminal = next(e['stamp_sec'] for e in events if 'CONTACT_ENDPOINT_COMPENSATION_FAILED' in e['text'])
report = dict(
    record=str(RECORD), record_closed=True, result='failure',
    analysis_scope='Read-only sealed logs and selected non-temperature bag topics; no ROS node, service, parameter or motion command.',
    topic_counts=dict(counts), policy_status_counts=[dict(policy=json.loads(k), count=v) for k,v in policies.items()],
    support_summary=dict(rows=len(support), maximum_local_separation_m=max(row['support_reference_max_distance_m'] for row in support),
        post_arrival_rows=len(post_arrival), post_arrival_maximum_local_separation_m=max(row['support_reference_max_distance_m'] for row in post_arrival),
        warning_band_rows=len(warnings), warning_band_live_handoff_proven=bool(warnings)),
    support_rows=support, phases=phases, task_states=states, plans=plans,
    measurement_requests=requests, measurement_acks=acks,
    contact_plan=dict(plan_id=contact['plan_id'], request_id=contact['request_id'], outcome=contact['outcome'],
        fusion=surface, contact_sequence_precheck=selected.get('contact_sequence_precheck'),
        final_geometry_gate=selected.get('final_geometry_gate'), final_contact_height_margin=selected.get('final_contact_height_margin'),
        final_contact_rejection=selected.get('final_contact_rejection'),
        final_execution_sequence_checked=selected.get('final_execution_sequence_checked'),
        observation_side_evidence=selected.get('observation_side_evidence'),
        fusion_vs_latest_observation=selected.get('fusion_vs_latest_observation')),
    first_perception_loss=first_loss,
    movement_window=dict(qualified_arrival_q=arrival_q, feedback=joints,
        first_recorded_above_station_threshold=crossings[0] if crossings else None),
    event_timeline=events,
    quantitative_timeline=dict(contact_motion_start_sec=contact_start, first_perception_loss_sec=loss,
        terminal_failure_sec=terminal, motion_to_perception_loss_sec=loss-contact_start,
        motion_to_terminal_failure_sec=terminal-contact_start),
    final_failure_reason=result['failure_reason'],
    conclusions=[
        'Live detector reports the requested 7 mm hard / 4 mm warning policy; accepted registration audit also reports hard_limit_m=0.007.',
        'One supplementary camera motion produced a CONTACT_EXECUTION_PLAN with three measured source frames; source-frame count is not a claim of three independent viewpoints.',
        'This run has no observed 4–7 mm support-warning sample, so it does not demonstrate live warning-band handoff. Offline recorded-plane replay and boundary regressions remain that policy evidence.',
        'Perception station qualification loss occurred after planned contact pregrasp motion began; that loss is separate from the final task failure.',
        'Final failure was CONTACT_ENDPOINT_COMPENSATION_FAILED / PREGRASP_HELD_AXES_GUIDE_DOES_NOT_REACH_FIXED_GOAL. The grasp did not reach closure or verified physical success.',
    ],
    source_evidence=audit_files,
)
path = OUT / 'recorded_policy_fusion_timeline.json'
path.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False)+'\n')
print(json.dumps(dict(path=str(path), sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
    support_summary=report['support_summary'], registration=registration['support_offset_diagnostic'],
    timeline=report['quantitative_timeline'], first_motion_threshold=crossings[0] if crossings else None), ensure_ascii=False))
