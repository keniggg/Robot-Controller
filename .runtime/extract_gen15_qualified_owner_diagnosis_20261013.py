#!/usr/bin/env python3
"""Read sealed R3 task wire and audit phase-owner start-time representation."""
import hashlib
import io
import json
from pathlib import Path

import rosbag
from alicia_flexible_grasp.grasp.rich_plan_integrity import stamp_seconds

W = Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
N = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_contact_admission_20261013T011103Z')
R = N.parents[1] / 'grasp_records/20261013T083836.150489Z_f4adfdd7ede44edeb725b4bfd7ac4f32'
O = N / 'verification/gen15_qualified_owner_diagnosis'
TOPICS = [
    '/grasp/state', '/grasp/near_field_phase', '/grasp/near_field',
    '/grasp_mode/selection', '/alicia_d/control_reference_epoch',
    '/grasp_mode/observe_return_measurement_request',
    '/grasp_mode/observe_return_measurement_ack',
    '/grasp_mode/observe_return_session', '/perception/unknown/detector_status',
    '/perception/object', '/perception/unknown/object',
    '/grasp_6d/plan_enriched', '/grasp_6d/preview_plan_enriched',
    '/alicia_controller/follow_joint_trajectory/goal',
    '/alicia_controller/follow_joint_trajectory/result',
    '/hand_controller/follow_joint_trajectory/goal',
    '/hand_controller/follow_joint_trajectory/result']


def sha(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for data in iter(lambda: stream.read(8 * 2**20), b''):
            digest.update(data)
    return digest.hexdigest()


def plain(message):
    if hasattr(message, 'to_nsec'):
        return message.to_nsec()
    if hasattr(message, '__slots__'):
        return {key: plain(getattr(message, key)) for key in message.__slots__}
    if isinstance(message, (tuple, list)):
        return [plain(value) for value in message]
    return message


def main():
    manifest_path = R / 'manifest.json'
    manifest = json.loads(manifest_path.read_text())
    assert manifest['state'] == 'complete' and manifest['recorder_closed']
    assert manifest['result'] == 'failure'
    assert manifest['failure_reason'] == 'QUALIFIED_STATION_PHASE_OWNER_CHANGED_OR_EXPIRED'
    path = R / 'keyframes.bag'
    before = path.stat()
    expected = next(row['sha256'] for row in manifest['files'] if row['path'] == 'keyframes.bag')
    assert not path.with_suffix('.bag.active').exists() and sha(path) == expected
    O.mkdir(exist_ok=False)
    raw_dir = O / 'wire_raw'; raw_dir.mkdir()
    rows, phases, counts = [], [], {}
    with rosbag.Bag(str(path), 'r') as bag:
        topic_counts = {name: value.message_count for name, value in bag.get_type_and_topic_info().topics.items()
                        if name in TOPICS}
        for topic, raw, received in bag.read_messages(topics=TOPICS, raw=True):
            datatype, data, md5sum, position, cls = raw
            message = cls().deserialize(data)
            buffer = io.BytesIO(); message.serialize(buffer)
            assert buffer.getvalue() == data
            decoded = plain(message)
            if datatype == 'std_msgs/String':
                try: decoded = json.loads(message.data)
                except (ValueError, TypeError): pass
            file = raw_dir / ('%05d.rosmsg' % len(rows)); file.write_bytes(data)
            row = dict(index=len(rows), topic=topic, received_ns=received.to_nsec(),
                message=decoded, file=str(file), sha256=hashlib.sha256(data).hexdigest(),
                datatype=datatype, md5sum=md5sum, source_bag_position=plain(position))
            rows.append(row); counts[topic] = counts.get(topic, 0) + 1
            if topic == '/grasp/near_field_phase' and message.active:
                internal_start = stamp_seconds(message.header.stamp)
                wire_start = message.header.stamp.to_sec()
                phases.append(dict(wire_index=row['index'], received_ns=received.to_nsec(),
                    phase_id=message.phase_id, source_ns=message.header.stamp.to_nsec(),
                    phase_deadline_ns=message.deadline.to_nsec(),
                    source_secs=message.header.stamp.secs, source_nsecs=message.header.stamp.nsecs,
                    production_setter_internal_started_sec=internal_start,
                    production_reuse_comparison_started_sec=wire_start,
                    internal_float_hex=internal_start.hex(), comparison_float_hex=wire_start.hex(),
                    strict_equality=internal_start == wire_start,
                    computed_difference_nanoseconds=(internal_start-wire_start)*1e9,
                    full_original_wire_file=str(file), full_original_wire_sha256=row['sha256']))
    assert counts == topic_counts
    after = path.stat()
    assert (before.st_size, before.st_mtime_ns, before.st_ino) == (after.st_size, after.st_mtime_ns, after.st_ino)
    sources = [
        W/'src/alicia_flexible_grasp_supervisor/scripts/grasp_task_node.py',
        W/'src/alicia_grasp_modes/src/alicia_grasp_modes/observe_return_task.py',
        W/'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/grasp/rich_plan_integrity.py']
    wire_path = O / 'wire.json'
    wire_path.write_text(json.dumps(rows, ensure_ascii=False, indent=2) + '\n')
    failed = [row for row in rows if row['topic'] == '/grasp/state'
        and row['message'].get('state') == 'FAILED'
        and 'QUALIFIED_STATION_PHASE_OWNER_CHANGED_OR_EXPIRED' in row['message'].get('message', '')]
    report = dict(passed=True, meaning='Evidence extraction and deterministic original-code comparison succeeded; grasp remains failed.',
        original_record=str(R), original_manifest_sha256=sha(manifest_path),
        source_bag=str(path), source_bag_sha256=expected, source_bag_unchanged=True,
        wire=str(wire_path), wire_sha256=sha(wire_path), topic_counts=counts,
        active_phase_float_comparisons=phases,
        failed_task_wire_indices=[row['index'] for row in failed],
        deterministic_owner_comparison_failure=any(not row['strict_equality'] for row in phases),
        causal_condition='observe_return_task.py:355 compares _near_field_phase_started_sec to phase.header.stamp.to_sec(); setter grasp_task_node.py:2830/2866 stores rich_plan_integrity.stamp_seconds(now)=float(integer_ns)*1e-9.',
        source_sha256={str(source): sha(source) for source in sources},
        script_sha256=sha(__file__), no_ros_master=True, motion_commands_sent=False,
        production_modified=False, temperature_queried=False,
        limits=['No live task memory dump exists. The start-time predicate is reconstructed deterministically from the original emitted phase and exact production setter/helper; other internal predicates are not asserted from missing memory.',
                'This report does not reinterpret successful measurement ACKs as physical grasp success.'])
    report_path = O / 'result.json'
    report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
    print(json.dumps(dict(result=str(report_path), sha256=sha(report_path),
        phases=phases, failure_rows=[{key: row[key] for key in ('index','received_ns','message')} for row in failed]), ensure_ascii=False))


if __name__ == '__main__':
    main()
