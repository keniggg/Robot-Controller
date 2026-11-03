#!/usr/bin/env python3
"""Read sealed gen34 bag, never initialize ROS or invoke robot interfaces."""
from pathlib import Path
import hashlib
import json
import numpy as np
import rosbag
from alicia_grasp_modes.station_compute_reacquisition import (
    validate_registration, validate_contract, validate_recovery_parent)
from alicia_grasp_modes.unknown_pending_identity import reference_binding
from alicia_grasp_modes.rgb_observation_source import bracket_feedback

ROOT = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
BAG = ROOT / 'grasp_records/20261009T145258.586806Z_51a72aa111a74c4d95d2d58e75a1dc07/keyframes.bag'
OUT = ROOT / 'ros_log/full_latest_restart_20261010T002347Z/verification/gen34_remote_join'
PENDING_NS = 1791557651187591314
TASK_FAILED_NS = 1791557651258349657
rows = {}
feedback = []
topics = ['/perception/object', '/perception/object_mask',
          '/perception/unknown/detector_status', '/alicia_d/accepted_joint_states']
with rosbag.Bag(str(BAG)) as bag:
    for topic, msg, t in bag.read_messages(topics=topics):
        if topic == topics[-1]:
            if msg.header.frame_id == 'sdk_measured':
                values = dict(zip([s.lower() for s in msg.name], msg.position))
                feedback.append((msg.header.stamp.to_nsec(),
                    np.array([values['joint%d' % k] for k in range(1, 7)]), t.to_nsec()))
            continue
        if topic == topics[2]:
            status = json.loads(msg.data)
            ns = status.get('source_stamp_ns')
            if not status.get('station_compute_wait') or status['state'] != 'ready':
                continue
            rows.setdefault(ns, {})['status'] = status
            rows[ns]['status_receipt_ns'] = t.to_nsec()
        else:
            ns = msg.header.stamp.to_nsec()
            key = 'object' if topic == topics[0] else 'mask'
            rows.setdefault(ns, {})[key] = msg
            rows[ns][key + '_receipt_ns'] = t.to_nsec()
rows = {ns: row for ns, row in rows.items() if 'status' in row}
summary = []
for ns, row in sorted(rows.items()):
    status, obj, mask = row['status'], row['object'], row['mask']
    contract = status['station_compute_wait']
    now_ns = max(row[k + '_receipt_ns'] for k in ('object', 'mask', 'status'))
    pixels = np.frombuffer(mask.data, np.uint8).reshape(mask.height, mask.step)[:, :mask.width]
    digest = dict(sha256=hashlib.sha256(np.ascontiguousarray(pixels != 0, np.uint8).tobytes()).hexdigest(),
        shape=[mask.height, mask.width], frame_id=mask.header.frame_id)
    assert mask.encoding == 'mono8' and digest == status['station_compute_mask']
    assert obj.header.stamp == mask.header.stamp and obj.header.stamp.to_nsec() == ns
    assert obj.detected and obj.label == 'unknown_small' and obj.header.frame_id == 'base_link'
    assert reference_binding(status, contract['binding']['selection']) == contract['binding']['reference_binding']
    validate_registration(contract, binding=contract['binding'], now_ns=now_ns)
    current_feedback = [(stamp, q) for stamp, q, receipt in feedback if receipt <= now_ns]
    interpolated, before, after = bracket_feedback(current_feedback, ns, contract['binding']['control_epoch_ns'])
    max_q_error = max(float(np.max(np.abs(np.asarray(q)-contract['binding']['station_q_rad'])))
        for q in (interpolated, before[1], after[1]))
    assert max_q_error <= .004
    summary.append(dict(source_ns=ns, receipt_ns=now_ns,
        receipt_by_topic_ns={k:row[k+'_receipt_ns'] for k in ('object','mask','status')},
        source_age_at_recorded_complete_join_ms=(now_ns-ns)/1e6,
        geometry_deadline_ns=contract['geometry_deadline_ns'],
        mask_digest_verified=True, frozen_reference_verified=True,
        actual_object_detected=bool(obj.detected), confidence=float(obj.confidence),
        accepted_feedback_bracket_verified=True, max_bracket_station_error_rad=max_q_error,
        recovery_parent_source_ns=(status.get('station_compute_recovery') or {}).get('source_ns'),
        recovery_chain_sources_ns=[c['source_ns'] for c in status.get('station_compute_recovery_chain', [])]))

last_ns = max(rows)
last = rows[last_ns]['status']
last_receipt_ns = max(rows[last_ns][k + '_receipt_ns'] for k in ('object', 'mask', 'status'))
recovery_checks = []
for ns, row in sorted(rows.items()):
    if ns >= last_ns:
        continue
    old = row['status']['station_compute_wait']
    if old['geometry_deadline_ns'] > PENDING_NS:
        continue
    validate_recovery_parent(old, parent=last['station_compute_recovery'],
        chain=last['station_compute_recovery_chain'], binding=old['binding'],
        now_ns=last_receipt_ns, source_ns=last_ns)
    recovery_checks.append(dict(possible_old_pin_source_ns=ns,
        original_identity_deadline_ns=old['identity_deadline_ns'],
        validate_recovery_parent_passed=True,
        next_source_remaining_geometry_ms=(last['station_compute_wait']['geometry_deadline_ns']-last_receipt_ns)/1e6))

result = dict(schema_version=1, record=str(BAG.parent), read_only_offline=True,
    real_grasp_success=False, actual_remote_internal_timestamps_available=False,
    recorded_current_ready_triples=summary, pending_geometry_stamp_ns=PENDING_NS,
    task_failure_stamp_ns=TASK_FAILED_NS,
    actual_next_ready_source_ns=last_ns, actual_next_recorded_complete_join_ns=last_receipt_ns,
    next_join_before_task_failure_ms=(TASK_FAILED_NS-last_receipt_ns)/1e6,
    next_frame_recovery_contract_checks=recovery_checks,
    findings=[
        'All recorded READY triples are source-paired, digest-matched, original-reference matched, and fresh under the unchanged 0.8 s gate at bag receipt.',
        'The last pre-pending frame source 1791557650545534133 was complete in the recorder at 1791557650851876258 and valid until 1791557651345534133; its timely remote consumption is NOT recorded.',
        'At pending publication, an expired remote pin must be source <= 1791557650387591314. Request 35 used latest snapshot source 1791557650218772172, but this does NOT prove the exact remote registered pin.',
        'The next actual READY triple arrives in the recorder before task failure and its original recovery chain validates against every earlier recorded pin that could have expired. It grants no motion or contact authority.',
        'No exact remote callback-start/lock-acquisition/registration timestamps exist in this sealed recording; scheduling/locking/message drop is a hypothesis for the consumer lag, not a demonstrated exact mutex cause.',
        'Actual recovery was preempted by task owner_changed retirement; passing original contract checks is not proof of full pipeline or physical grasp recovery.'
    ])
OUT.mkdir(parents=True, exist_ok=True)
output = OUT/'result.json'
output.write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
print(json.dumps(dict(output=str(output), sha256=hashlib.sha256(output.read_bytes()).hexdigest(),
    rows=len(summary), possible_old_pin_checks=len(recovery_checks),
    source_age_ms_range=[min(r['source_age_at_recorded_complete_join_ms'] for r in summary),
        max(r['source_age_at_recorded_complete_join_ms'] for r in summary)],
    next_join_before_failure_ms=result['next_join_before_task_failure_ms'])))
