#!/usr/bin/env python3
"""Offline raw-wire extraction of sealed attempt 5, never ROS master access."""
import hashlib
import io
import json
from collections import Counter
from pathlib import Path
import rosbag

W = Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
EXT = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
N = EXT / 'ros_log/full_latest_contact_admission_20261013T011103Z'
R = EXT / 'grasp_records/20261013T112006.120233Z_d2b04edd648643b4b57d8e9febc4d88c'
O = N / 'verification/attempt5_station_timeline_r1'
TOPICS = [
    '/grasp/state', '/grasp/near_field_phase', '/grasp/near_field',
    '/grasp_mode/selection', '/alicia_d/control_reference_epoch',
    '/grasp_mode/observe_return_measurement_request',
    '/grasp_mode/observe_return_measurement_ack',
    '/grasp_mode/observe_return_session', '/grasp_mode/observe_return_ack',
    '/perception/unknown/detector_status', '/perception/object',
    '/grasp_6d/plan_enriched', '/grasp_6d/preview_plan_enriched',
    '/grasp_6d/object_geometry', '/grasp_6d/gate_audit', '/grasp_6d/status',
    '/grasp/remote_pipeline_metrics', '/experiment/runtime_execution_error_snapshot',
    '/experiment/observation_rgbd/metadata', '/grasp_record/camera_runtime_metadata',
    '/alicia_controller/follow_joint_trajectory/goal',
    '/alicia_controller/follow_joint_trajectory/result',
    '/hand_controller/follow_joint_trajectory/goal',
    '/hand_controller/follow_joint_trajectory/result', '/rosout_agg']

def sha(p):
    h = hashlib.sha256()
    with Path(p).open('rb') as f:
        for data in iter(lambda: f.read(8*2**20), b''):
            h.update(data)
    return h.hexdigest()

def plain(m):
    if hasattr(m, 'to_nsec'):
        return m.to_nsec()
    if hasattr(m, '__slots__'):
        return {key: plain(getattr(m, key)) for key in m.__slots__}
    if isinstance(m, (tuple, list)):
        return [plain(v) for v in m]
    return m

def main():
    mp = R/'manifest.json'
    manifest = json.loads(mp.read_text())
    assert manifest['state'] == 'complete' and manifest['recorder_closed']
    assert manifest['result'] == 'failure'
    assert manifest['failure_reason'] == 'STATION_CONTACT_PROVENANCE_INVALID: station_compute_wait_registration_too_late'
    p = R/'keyframes.bag'
    before = p.stat()
    expected = next(r['sha256'] for r in manifest['files'] if r['path'] == 'keyframes.bag')
    assert not p.with_suffix('.bag.active').exists()  # Parent requested no large-bag hash; retain manifest claim separately.
    O.mkdir(exist_ok=False)
    rawdir = O/'wire_raw'; rawdir.mkdir()
    counts, scanned, rows = Counter(), Counter(), []
    with rosbag.Bag(str(p), 'r') as b:
        all_topics = {name: dict(type=t.msg_type, count=t.message_count)
                      for name, t in b.get_type_and_topic_info().topics.items()}
        for topic, raw, received in b.read_messages(topics=TOPICS, raw=True):
            datatype, data, md5sum, position, cls = raw
            message = cls().deserialize(data)
            scanned[topic] += 1
            if topic == '/rosout_agg':
                if not any(term in message.msg for term in (
                    'OBSERVE_RETURN', 'Observe-return', 'REMOTE_PIPELINE',
                    'Unknown target', 'Unknown tabletop', 'Grasp state',
                    '6D planning phase', 'qualified', 'measurement', 'station_', 'STATION_', 'remote 6D pipeline metrics')):
                    continue
                if 'temperature' in message.msg.lower():
                    continue
            buf = io.BytesIO(); message.serialize(buf)
            assert buf.getvalue() == data
            decoded = plain(message)
            if datatype == 'std_msgs/String':
                try: decoded = json.loads(message.data)
                except (ValueError, TypeError): pass
            path = rawdir/('%05d.rosmsg' % len(rows)); path.write_bytes(data)
            rows.append(dict(index=len(rows), topic=topic, received_ns=received.to_nsec(),
                message=decoded, file=str(path), sha256=hashlib.sha256(data).hexdigest(),
                datatype=datatype, md5sum=md5sum, source_bag_position=plain(position)))
            counts[topic] += 1
        assert dict(scanned) == {t: v['count'] for t, v in all_topics.items() if t in TOPICS}
    after = p.stat()
    assert (before.st_size, before.st_mtime_ns, before.st_ino) == (after.st_size, after.st_mtime_ns, after.st_ino)
    wire = O/'wire.json'; wire.write_text(json.dumps(rows, ensure_ascii=False, indent=2)+'\n')
    sources = [W/'src/alicia_grasp_modes/src/alicia_grasp_modes/station_compute_reacquisition.py',
        W/'src/alicia_grasp_modes/src/alicia_grasp_modes/station_compute_wait_task.py',
        W/'src/alicia_grasp_modes/src/alicia_grasp_modes/station_compute_wait_remote.py',
        W/'src/alicia_grasp_modes/src/alicia_grasp_modes/station_contact_admission.py',
        W/'src/alicia_grasp_modes/src/alicia_grasp_modes/observe_return_task.py',
        W/'src/alicia_grasp_modes/src/alicia_grasp_modes/observe_return_remote.py',
        W/'src/alicia_flexible_grasp_supervisor/scripts/grasp_task_node.py']
    report = dict(extraction_complete=True, original_record=str(R), original_failure=manifest['failure_reason'],
        original_manifest_sha256=sha(mp), source_bag=str(p), source_bag_sha256_claim_from_manifest=expected, source_bag_hash_recomputed=False,
        source_bag_stat=dict(size=before.st_size,mtime_ns=before.st_mtime_ns,inode=before.st_ino),
        source_bag_unchanged=True, all_bag_topics=all_topics,
        scanned_counts=dict(scanned), extracted_counts=dict(counts),
        wire=str(wire), wire_sha256=sha(wire), script_sha256=sha(__file__),
        production_sha256={str(s): sha(s) for s in sources},
        no_ros_master=True, motion_commands_sent=False, production_modified=False,
        temperature_queried=False, full_sidecar_accessed=False,
        limits=['Bag receipt ordering is recorder callback receipt order, not proof of producer callback entry order.',
                'rosout raw messages are retained with their own header and receipt; filtered log content remains diagnostic rather than control wire.',
                'No live task or producer memory is available.'])
    result = O/'result.json'; result.write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps(dict(result=str(result),sha256=sha(result),counts=dict(counts)),ensure_ascii=False))

if __name__ == '__main__':
    main()
