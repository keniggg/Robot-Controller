"""Read a closed bag's preview, audit, station and task timeline; no ROS calls."""
import hashlib
import json
from pathlib import Path

import rosbag

E = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
N = E / 'ros_log/full_latest_gen22_restart_20261015T003700Z'
R = E / 'grasp_records/20261015T092659.242714Z_911b220cf1d34113b7c80a7f97dad165'
O = N / 'verification/gen29_recorded_handoff_diagnosis'
PLAN = '7e5fb7d5f533709420ca757a'
AUDIT = R / 'keyframes_audits/7c655137d87de8d6a7dbd172695a6fb3f617dd9035276ff9eb191f73e8fe558e.json'


def read(path):
    return json.loads(path.read_text())


def sha(path):
    h = hashlib.sha256()
    with path.open('rb') as f:
        for block in iter(lambda: f.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def main():
    assert Path('/media/zhuyupei/SanStand').is_mount()
    assert not O.exists()
    outcome = read(R / 'recorder_outcome.json')
    result = read(R / 'result_summary.json')
    assert outcome['bag_closed'] is True and outcome['interrupted'] is False
    assert result['state'] == 'complete' and result['result'] == 'failure'
    audit = read(AUDIT)
    assert audit['plan_id'] == PLAN and audit['request_id'] == 23
    assert audit['observe_return']['phase'] == 'supplemental_observation'
    assert audit['lineage'][0]['moveit']['reachable'] is True
    assert audit['lineage'][0]['moveit']['evidence_code'] == 'STRICT_SERVICE_SUCCESS'
    evidence = {str(p): sha(p) for p in (
        R / 'manifest.json', R / 'recorder_outcome.json', R / 'result_summary.json',
        R / 'command_result.json', R / 'runner.log', R / 'keyframes.bag', AUDIT,
        Path(__file__).resolve())}
    assert evidence[str(R / 'keyframes.bag')] == 'bb5932dd295865da4c625202138fcf512ba7872748bb7d4affaa202b58bf0236'
    previews, audits, stations, states = [], [], [], []
    topics = ['/grasp_6d/preview_plan_enriched', '/grasp_6d/gate_audit',
              '/perception/unknown/detector_status', '/grasp/state']
    with rosbag.Bag(str(R / 'keyframes.bag')) as bag:
        for topic, msg, stamp in bag.read_messages(topics=topics):
            ns = stamp.to_nsec()
            if topic == topics[0]:
                previews.append(dict(received_ns=ns, source_ns=msg.header.stamp.to_nsec(),
                    plan_id=msg.plan_id, valid=msg.valid, diagnostic=msg.diagnostic,
                    candidate_source=msg.candidate_source))
            elif topic == topics[1]:
                audits.append(dict(received_ns=ns, reference=json.loads(msg.data)))
            elif topic == topics[2]:
                value = json.loads(msg.data)
                stations.append(dict(received_ns=ns, **{k: value.get(k) for k in (
                    'source_stamp_ns', 'state', 'target_lost', 'detail',
                    'station_compute_wait', 'station_compute_recovery_chain')}))
            else:
                states.append(dict(received_ns=ns, source_ns=msg.header.stamp.to_nsec(),
                    state=msg.state, active=msg.active, success=msg.success, message=msg.message))
    published = [row for row in previews if row['plan_id'] == PLAN and row['valid']]
    assert len(published) == 1
    preview = published[0]
    pending = next(row for row in previews if row['received_ns'] > preview['received_ns']
                   and row['valid'] is False and row['diagnostic'].startswith('TARGET_PROCESSING_PENDING'))
    references = [row for row in audits if AUDIT.stem in json.dumps(row['reference'])]
    assert len(references) == 1
    reference = references[0]
    ready = [row for row in stations if row['state'] == 'ready'
             and isinstance(row['station_compute_wait'], dict)]
    previous = max((row for row in ready if row['received_ns'] < preview['received_ns']),
                   key=lambda row: row['received_ns'])
    next_ready = next(row for row in ready if row['received_ns'] > reference['received_ns'])
    assert preview['received_ns'] < reference['received_ns'] < next_ready['received_ns'] < pending['received_ns']
    deadline = previous['station_compute_wait']['geometry_deadline_ns']
    assert deadline < reference['received_ns']
    assert pending['received_ns'] - preview['received_ns'] < 1_000_000_000
    assert not any(row['valid'] for row in previews if row['received_ns'] > pending['received_ns'])
    timeline = dict(valid_preview=preview, matching_audit=reference, preceding_ready=previous,
        next_ready=next_ready, overwriting_pending=pending,
        preview_public_window_ns=pending['received_ns'] - preview['received_ns'],
        audit_after_preceding_geometry_expiry_ns=reference['received_ns'] - deadline,
        station_rows=[row for row in stations
            if preview['received_ns'] - 1_000_000_000 <= row['received_ns'] <= pending['received_ns'] + 1_000_000_000],
        task_states=states, later_valid_previews=0,
        observation_audit_promotion=audit['promotion'],
        timestamp_kind='recorded_bag_receipt_and_original_message_source_not_task_internal_callback_time')
    O.mkdir()
    (O / 'timeline.json').write_text(json.dumps(timeline, ensure_ascii=False, indent=2) + '\n')
    evidence[str(O / 'timeline.json')] = sha(O / 'timeline.json')
    findings = dict(record=str(R), record_closed_verified=True,
        recorded_supplemental_preview_published=True, recorded_pending_overwrote_preview=True,
        evidence_sha256=evidence, timeline=str(O / 'timeline.json'),
        observation_plan_passed_strict_path=True, motion_commands_sent=False,
        live_ros_contacted=False, physical_grasp_verified=False, temperature_queried=False,
        conclusion='A published supplemental preview and its bound strict-path audit were followed by a clock pending preview. The current task kept only the latest public preview; original-source provenance and fresh-station admission must survive this asynchronous delivery interval.',
        limits='Bag receipts do not expose the exact task callback/thread schedule. Deterministic original-code regression supplies causal evidence; no claim of physical grasp success.')
    (O / 'result.json').write_text(json.dumps(findings, ensure_ascii=False, indent=2) + '\n')
    print(json.dumps(dict(result=str(O / 'result.json'), sha256=sha(O / 'result.json'),
        preview_public_window_ns=timeline['preview_public_window_ns'],
        audit_after_preceding_geometry_expiry_ns=timeline['audit_after_preceding_geometry_expiry_ns'])))


if __name__ == '__main__':
    main()
