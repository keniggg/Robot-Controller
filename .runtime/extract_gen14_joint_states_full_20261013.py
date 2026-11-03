#!/usr/bin/env python3
"""Extract real /joint_states independently from accepted station feedback."""
import hashlib
import importlib.util
import io
import json
from pathlib import Path

import rosbag
import rospy
from sensor_msgs.msg import JointState

W = Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
helper_path = W / '.runtime/extract_gen14_accepted_feedback_full_20261013.py'
spec = importlib.util.spec_from_file_location('accepted_extractor', helper_path)
helper = importlib.util.module_from_spec(spec)
spec.loader.exec_module(helper)
F = helper.F
OUTPUT = F / 'joint_states_full_r1'
TOPIC = '/joint_states'


def main():
    accepted_path = F / 'accepted_feedback_full_r1/manifest.json'
    accepted_sha = helper.sha(accepted_path)
    assert accepted_sha == 'c76bb01de47351639af5642d3d81e35811d2c2dae30a9cb403f68f579cef66de'
    accepted = json.loads(accepted_path.read_text())
    fixture_path = F / 'full_arrival_frames/fixture.json'
    assert helper.sha(fixture_path) == accepted['parent_fixture_sha256']
    fixture = json.loads(fixture_path.read_text())
    selected = {}
    source_audit = []
    counts = {}
    duplicate_count = 0
    for index, original in enumerate(accepted['original_bags']):
        path = Path(original['path'])
        assert helper.signature(path) == original['stat_after']
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
                buffer = io.BytesIO(); message.serialize(buffer)
                assert buffer.getvalue() == data
                message_sha = hashlib.sha256(data).hexdigest()
                source_ns = message.header.stamp.to_nsec()
                row = dict(topic=TOPIC, received_ns=received.to_nsec(), source_ns=source_ns,
                    message=helper.plain(message), raw=data, sha256=message_sha,
                    source_bag=str(path), source_bag_sha256=original['sha256'],
                    source_bag_position=list(position) if isinstance(position, tuple) else position)
                key = (source_ns, message_sha)
                previous = selected.get(key)
                if previous is not None:
                    duplicate_count += 1
                    assert previous['raw'] == data
                if previous is None or row['received_ns'] < previous['received_ns']:
                    selected[key] = row
                count += 1
        counts[str(path)] = count
        assert helper.signature(path) == original['stat_after']
        source_audit.append(dict(path=str(path), sha256=original['sha256'],
            stat_before=original['stat_after'], stat_after=helper.signature(path),
            full_sha256_verified_by=str(accepted_path),
            full_sha256_verified_by_manifest_sha256=accepted_sha))
    assert selected
    rows = sorted(selected.values(), key=lambda row: (row['received_ns'], row['source_ns'], row['sha256']))
    OUTPUT.mkdir(exist_ok=False)
    for index, row in enumerate(rows):
        data = row.pop('raw')
        file = OUTPUT / f'{index:04d}_{row["source_ns"]}.rosmsg'
        file.write_bytes(data)
        row.update(index=index, file=str(file), size_bytes=len(data),
                   message_type=JointState._type, md5sum=JointState._md5sum)
    manifest = dict(passed=True,
        scope='Real /joint_states serialized messages for the same historical bag windows. Separate from /alicia_d/accepted_joint_states; original headers, fields and earliest observed cross-recorder receive retained.',
        rows=rows, message_count=len(rows), original_bags=source_audit,
        scanned_message_counts=counts, duplicate_transport_deliveries=duplicate_count,
        deduplication='Identical serialized bytes and source stamp retain earliest actual receive; different complete messages with the same source stamp remain separate.',
        accepted_fixture=str(accepted_path), accepted_fixture_sha256=accepted_sha,
        numeric_fixture=str(fixture_path), numeric_fixture_sha256=helper.sha(fixture_path),
        extraction_script=str(Path(__file__).resolve()), extraction_script_sha256=helper.sha(__file__),
        extractor_dependency=str(helper_path), extractor_dependency_sha256=helper.sha(helper_path),
        no_ros_master=True, motion_commands_sent=False, temperature_queried=False,
        destination_callback='remote.joint_cb',
        station_feedback_callback='Only the separate exact accepted fixture feeds remote._station_compute_feedback_ingress_cb',
        full_bag_sha_scope='The immediately preceding accepted extraction computed and checked full bag SHA256. This pass verifies that the same size/mtime_ns/inode remain unchanged before and after indexed topic reading; it does not claim a second full hash.')
    output = OUTPUT / 'manifest.json'
    output.write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + '\n')
    print(json.dumps(dict(file=str(output), sha256=helper.sha(output), count=len(rows),
        original_frame_ids=sorted({row['message']['header']['frame_id'] for row in rows}),
        scanned_message_counts=counts), ensure_ascii=False))


if __name__ == '__main__':
    main()
