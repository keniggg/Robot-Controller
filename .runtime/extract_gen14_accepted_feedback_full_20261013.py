#!/usr/bin/env python3
"""Preserve original accepted SDK messages missing from the numeric replay fixture.

Offline bag extraction only. Both historical inputs and their earliest-receive
deduplication reproduce the existing sealed fixture; no headers are invented.
"""
import hashlib
import io
import json
from pathlib import Path

import rosbag
import rospy
from sensor_msgs.msg import JointState

W = Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
N = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_contact_admission_20261013T011103Z')
F = N / 'verification/gen14_arrival_independent'
R = N.parents[1] / 'grasp_records/20261013T025604.344486Z_af3ecb9373264cc9b19a98143f7b5df5'
OUTPUT = F / 'accepted_feedback_full_r1'
TOPIC = '/alicia_d/accepted_joint_states'


def sha(path):
    result = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(8 * 1024 * 1024), b''):
            result.update(block)
    return result.hexdigest()


def signature(path):
    stat = path.stat()
    return dict(size_bytes=stat.st_size, mtime_ns=stat.st_mtime_ns, inode=stat.st_ino)


def plain(message):
    if hasattr(message, 'to_nsec'):
        return message.to_nsec()
    if hasattr(message, '__slots__'):
        return {key: plain(getattr(message, key)) for key in message.__slots__}
    if isinstance(message, (tuple, list)):
        return [plain(value) for value in message]
    return message


def main():
    fixture_path = F / 'full_arrival_frames/fixture.json'
    fixture_before = sha(fixture_path)
    fixture = json.loads(fixture_path.read_text())
    sidecar_seal_path = N / 'attempt_2_full_frames/result.json'
    sidecar_seal = json.loads(sidecar_seal_path.read_text())
    record_seal_path = R / 'manifest.json'
    record_seal = json.loads(record_seal_path.read_text())
    assert sidecar_seal['recording_closed'] and record_seal['recorder_closed']
    assert record_seal['result'] == 'failure'
    keyframes = R / 'keyframes.bag'
    sidecar = N / 'attempt_2_full_frames/full_frames.bag'
    expected_keyframes = next(row['sha256'] for row in record_seal['files']
                              if row['path'] == 'keyframes.bag')
    assert expected_keyframes == fixture['keyframe_bag_sha256']
    assert sidecar_seal['sha256'] == fixture['sidecar_bag_sha256']
    originals = [(keyframes, expected_keyframes), (sidecar, sidecar_seal['sha256'])]
    source_audit = []
    for path, expected in originals:
        assert path.is_file() and not path.with_suffix('.bag.active').exists()
        before = signature(path)
        actual = sha(path)
        assert actual == expected, path
        source_audit.append(dict(path=str(path), sha256=actual,
                                 stat_before=before, original_topic=TOPIC))
    expected_by_stamp = {row['stamp_ns']: row for row in fixture['sdk']}
    assert len(expected_by_stamp) == len(fixture['sdk']) == 531
    selected = {}
    duplicate_count = 0
    counts = {}
    for index, (path, _) in enumerate(originals):
        options = {}
        if index == 1:
            options = dict(start_time=rospy.Time(fixture['arrival_ns'] // 10**9 - 1, 0),
                           end_time=rospy.Time(fixture['capture_deadline_ns'] // 10**9 + 2, 0))
        count = 0
        with rosbag.Bag(str(path), 'r') as bag:
            for topic, raw, received in bag.read_messages(topics=[TOPIC], raw=True, **options):
                datatype, data, md5sum, position, _ = raw
                assert datatype == JointState._type and md5sum == JointState._md5sum
                message = JointState().deserialize(data)
                # Ensure byte-exact preservation, including seq/frame_id and
                # velocity/effort, rather than reconstructing a permissive DTO.
                buffer = io.BytesIO(); message.serialize(buffer)
                assert buffer.getvalue() == data
                assert message.header.frame_id == 'sdk_measured'
                numeric = dict(received_ns=received.to_nsec(),
                    stamp_ns=message.header.stamp.to_nsec(),
                    names=list(message.name), positions=list(message.position))
                assert numeric['stamp_ns'] in expected_by_stamp
                candidate = dict(numeric=numeric, message=plain(message), raw=data,
                    source_bag=str(path), source_bag_sha256=source_audit[index]['sha256'],
                    source_bag_position=list(position) if isinstance(position, tuple) else position)
                previous = selected.get(numeric['stamp_ns'])
                if previous is not None:
                    duplicate_count += 1
                    assert previous['raw'] == data, 'Duplicate SDK source changed full serialized bytes'
                if previous is None or numeric['received_ns'] < previous['numeric']['received_ns']:
                    selected[numeric['stamp_ns']] = candidate
                count += 1
        counts[str(path)] = count
        assert signature(path) == source_audit[index]['stat_before']
        source_audit[index]['stat_after'] = signature(path)
    ordered = sorted(selected.values(), key=lambda value: value['numeric']['received_ns'])
    assert [value['numeric'] for value in ordered] == fixture['sdk']
    assert sha(fixture_path) == fixture_before
    OUTPUT.mkdir(exist_ok=False)
    rows = []
    for index, value in enumerate(ordered):
        raw = value.pop('raw')
        path = OUTPUT / f'{index:04d}_{value["numeric"]["stamp_ns"]}.rosmsg'
        path.write_bytes(raw)
        rows.append(dict(index=index, topic=TOPIC,
            received_ns=value['numeric']['received_ns'], source_ns=value['numeric']['stamp_ns'],
            file=str(path), sha256=hashlib.sha256(raw).hexdigest(), size_bytes=len(raw),
            message_type=JointState._type, md5sum=JointState._md5sum,
            **{key: item for key, item in value.items() if key != 'numeric'}))
    manifest = dict(passed=True,
        scope='Exact original accepted JointState serialized bytes, complete fields and earliest recorded receive times for every one of the existing 531 numeric SDK observations. No header/value/time is synthesized.',
        rows=rows, message_count=len(rows), original_bags=source_audit,
        scanned_message_counts=counts, duplicate_transport_deliveries=duplicate_count,
        deduplication='Same source stamp, identical complete serialized message; retain earliest actual recorded receive across the same two original bag windows.',
        parent_fixture=str(fixture_path), parent_fixture_sha256=fixture_before,
        numeric_fixture_all_531_rows_exact=True, original_fixture_unchanged=True,
        seal_files={str(path): sha(path) for path in (sidecar_seal_path, record_seal_path)},
        extraction_script=str(Path(__file__).resolve()), extraction_script_sha256=sha(__file__),
        no_ros_master=True, motion_commands_sent=False, temperature_queried=False,
        consumer_paths=[
            dict(topic=TOPIC, consumer='unknown', callback='_rgb_interest_joint_cb',
                 file='src/alicia_grasp_modes/scripts/unknown_tabletop_perception.py'),
            dict(topic=TOPIC, consumer='remote', callback='_station_compute_feedback_ingress_cb',
                 file='src/alicia_grasp_modes/src/alicia_grasp_modes/station_compute_wait_remote.py',
                 deferred_validation='_station_compute_accepted_cb(None), original 50 ms timer'),
            dict(topic='/joint_states', consumer='remote', callback='joint_cb',
                 file='src/alicia_flexible_grasp_supervisor/scripts/remote_grasp6d_node.py',
                 note='Separate topic and path; updates latest_joint_state/RGBD joint cache only. Does not feed accepted station feedback.')])
    path = OUTPUT / 'manifest.json'
    path.write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + '\n')
    print(json.dumps(dict(file=str(path), sha256=sha(path), count=len(rows),
        all_frame_ids_original_sdk_measured=True, parent_fixture_unchanged=True), ensure_ascii=False))


if __name__ == '__main__':
    main()
