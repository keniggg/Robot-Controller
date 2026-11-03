"""Recorded-only GEN19 planning interruption timeline; no ROS node or calls."""
from collections import Counter
from decimal import Decimal
import hashlib
import json
from pathlib import Path
import re
import time

import rosbag
import rospy

WORK = Path(__file__).resolve().parents[1]
NEW = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_restart_20261010T002347Z')
RECORD = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/grasp_records/20261010T082926.174495Z_ed2e482a1dbe4e5ab46962660f016697')
OUT = NEW/'gen19_analysis'
START = 1791621014415789842
END = 1791621104429426908


def digest(data):
    return hashlib.sha256(data).hexdigest()


def main():
    assert Path('/media/zhuyupei/SanStand').is_mount()
    station_path = next((NEW/'gen19_watch').glob('station_compute*.jsonl'))
    motion_path = next((NEW/'gen19_watch').glob('motion_boundary*.jsonl'))
    consumed = []
    def rows(path):
        raw = path.read_bytes()
        consumed.append(dict(path=str(path), read_ns=time.time_ns(), bytes=len(raw), captured_prefix_sha256=digest(raw)))
        return [json.loads(line) for line in raw.splitlines() if line.strip()]
    station, motion = rows(station_path), rows(motion_path)
    task = [r for r in station if r.get('kind')=='task']
    first_failed = next(r for r in task if r['state']=='FAILED')
    detector = [r for r in station if r.get('kind')=='detector' and START<=r['received_ros_ns']<=END]
    geom = [r for r in motion if r.get('kind')=='geometry' and START<=r['received_ros_ns']<=END]
    compact_geom = []
    for row in geom:
        code, _, payload = row['reason'].partition(': ')
        contract = json.loads(payload).get('station_compute_wait', {}) if payload.startswith('{') else {}
        compact_geom.append(dict(received_ns=row['received_ros_ns'], header_ns=row['source_ns'],
            valid=row['valid'], code=code, registered_source_ns=contract.get('source_ns'),
            geometry_deadline_ns=contract.get('geometry_deadline_ns'), identity_deadline_ns=contract.get('identity_deadline_ns')))
    log_path = NEW/'gen18_arrival_feedback_gap_compute_deployment/consumers.log'
    metrics = []
    for line in log_path.read_text().splitlines():
        if 'remote 6D pipeline metrics: ' not in line:
            continue
        payload = json.loads(line.split('remote 6D pipeline metrics: ', 1)[1])
        seconds = re.search(r'\[([0-9]+\.[0-9]+)\]', line).group(1)
        stamp = int(Decimal(seconds)*10**9)
        if START<=stamp<=END:
            metrics.append(dict(log_ns=stamp, **payload))
    request14 = next(r for r in metrics if r['request_id']==14)
    cancelled = request14['log_ns']
    after = [r for r in detector if r['received_ros_ns']>cancelled]
    ages = sorted(r['frame_timing']['status_source_age_sec'] for r in after if r.get('frame_timing'))
    ready_after = [r for r in after if r['state']=='ready']
    loss = next(r['first_loss'] for r in station if r.get('first_loss'))
    bag_sources = {}
    phase_rows = []
    topics = ['/perception/object', '/perception/object_mask', '/perception/unknown/detector_status', '/grasp/near_field_phase']
    with rosbag.Bag(str(RECORD/'keyframes.bag')) as bag:
        for topic, message, received in bag.read_messages(topics=topics,
                start_time=rospy.Time(START//10**9, START%10**9), end_time=rospy.Time(END//10**9, END%10**9)):
            if topic=='/grasp/near_field_phase':
                phase_rows.append(dict(recorded_ns=received.to_nsec(), header_ns=message.header.stamp.to_nsec(),
                    active=message.active, phase_id=message.phase_id, planning_stage=message.planning_stage,
                    deadline_ns=message.deadline.to_nsec()))
                continue
            if topic.endswith('detector_status'):
                value=json.loads(message.data);source=int(value['source_stamp_ns'])
                valid=value.get('state')=='ready' and value.get('target_lost') is False
            else:
                source=message.header.stamp.to_nsec()
                valid=bool(message.detected) if topic.endswith('/object') else bool(message.data and any(message.data))
            bag_sources.setdefault(source,{})[topic]=dict(received_ns=received.to_nsec(),valid=valid)
    complete = {ns: parts for ns,parts in bag_sources.items() if ns>cancelled
        and len(parts)==3 and all(item['valid'] for item in parts.values())}
    q_rows=[r for r in motion if r.get('kind')=='accepted_q' and START<=r['received_ros_ns']<=END]
    q0=q_rows[0]['q']
    changed = max(max(abs(a-b) for a,b in zip(row['q'],q0)) for row in q_rows)
    result=dict(record=str(RECORD), generation=19, selection_stamp_ns=1791620967862544298,
        watch_captured_prefixes=consumed, observation_phase_start_ns=START,
        task_first_failure=first_failed, first_producer_loss=loss,
        producer_loss_after_task_failure_ms=(loss['observed_stamp_ns']-first_failed['source_stamp_ns'])/1e6,
        request14=request14, subsequent_request_metrics_before_failure=[r['request_id'] for r in metrics if r['log_ns']>cancelled],
        geometry_notices=compact_geom,
        perception_after_cancellation=dict(count=len(after), states=dict(Counter(r['state'] for r in after)),
            ready_count=len(ready_after), first_ready_source_ns=ready_after[0]['source_stamp_ns'],
            last_ready_source_ns=ready_after[-1]['source_stamp_ns'],
            gap_count=sum(r['state']=='processing_deadline_pending' for r in after),
            first_loss_count=sum(bool(r.get('first_loss')) for r in after),
            status_source_age_sec=dict(min=ages[0],p95=ages[int(.95*(len(ages)-1))],max=ages[-1])),
        raw_bag_complete_valid_triplets_after_cancellation=dict(count=len(complete),
            first_source_ns=min(complete),last_source_ns=max(complete),
            first_receipts=complete[min(complete)],last_receipts=complete[max(complete)]),
        phases=phase_rows, accepted_station=dict(count=len(q_rows),max_delta_from_first_rad=changed),
        code_supported_candidate=dict(
            blocking_poll='mode_aware_remote_grasp6d.py::_poll_direct_snapshot returns while same-generation direct submission latch is occupied.',
            cancellation_classification='A StreamResultCancelled from _prepare_and_predict falls through the generic preparation error path into PREDICT_FAILED.',
            worker_release='_release_expired_direct_snapshot_locked accepts only RESULT_EXPIRED, not TARGET_PROCESSING_PENDING/PREDICT_FAILED.',
            recovery_release='Station recovery clears the direct submission latch only for hard recovery. This run has no producer pending/gap before timeout.',
            inference='These code paths and missing later requests support an unreleased direct submission slot; private live fields were not sampled, so deterministic worker reproduction is still required.'),
        limits=['Raw recorder arrivals prove publication, not exact remote subscriber callback ordering or private cache state.',
            'No stack signal, debugger attachment, ROS call, source mutation, hardware action or temperature access was performed.',
            'The producer first loss is after the task timeout and does not explain the initial request cancellation.'])
    OUT.mkdir(parents=True,exist_ok=True)
    p=OUT/'direct_slot_stall_recorded_evidence.json'
    p.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps({'path':str(p),'sha256':digest(p.read_bytes()),'ready_after_cancel':len(ready_after),
        'valid_triplets':len(complete),'qdelta':changed,'failure_ns':first_failed['source_stamp_ns']},ensure_ascii=False))


if __name__=='__main__':
    main()
