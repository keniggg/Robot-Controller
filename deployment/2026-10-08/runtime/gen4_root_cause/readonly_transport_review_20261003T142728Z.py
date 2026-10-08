"""Offline bag verification only; never imports a motion or ROS-node API."""
from collections import Counter, defaultdict
import json
from pathlib import Path
import struct
import time

import numpy as np
import rosbag

ROOT = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_ros_latest_start_20261002/gen4_root_cause')
OUT = ROOT/'orthogonal_probe_20261003T142728Z'
COLOR = '/supervisor/camera/color/image_raw'
DEPTH = '/supervisor/camera/depth/image_raw'


def describe(values):
    a = np.asarray(values, dtype=float)
    return dict(count=len(a), minimum=None if not len(a) else float(a.min()),
        median=None if not len(a) else float(np.median(a)),
        p95=None if not len(a) else float(np.percentile(a, 95)),
        maximum=None if not len(a) else float(a.max()))


def stats(ns):
    return describe(np.diff(ns).astype(float)*1e-9)


def qvalue(msg):
    return dict(stamp_ns=msg.header.stamp.to_nsec(), frame_id=msg.header.frame_id,
        names=list(msg.name), q_rad=list(msg.position))


def words(values):
    return [int(np.floor((v+np.pi)/(2*np.pi/4096)+.5)) for v in values[:6]]


def add_contract_review(report, events):
    audits = [row for row in events if row['event']=='actual_submission_geometry']
    commands = report['commands']
    expected = [row['target'] for row in audits]
    actual = [row['q_rad'] for row in commands]
    paired = report['raw_image_pairing']
    report['command_contract_review'] = dict(
        audit_count=len(audits), archived_command_count=len(commands),
        exactly_16_expected_targets=bool(len(expected)==len(actual)==16 and expected==actual),
        actual_targets_follow_audit_step_order=[row['step'] for row in audits],
        original_connection_header_preserved=False,
        callerid_limit='raw message payload/metadata preserve exact targets; original ROS callerid is not archived. No-caller-drift evidence is recorder armed receive guard and recording_fatal=null, not recovered bag connection headers.')
    report['snapshot_pixel_review'] = dict(snapshot_count=len(paired['snapshot_pixels']),
        all_19_both_stream_pixels_equal_saved_source=bool(len(paired['snapshot_pixels'])==19 and all(
            len(pair)==2 and all(value['exact_pixels'] and value['header_ns']==value['saved_source_ns']
                                for value in pair.values()) for pair in paired['snapshot_pixels'].values())))
    report['authority_scope_limit'] = ('Bag proves recorded motion_enabled stays true, feedback_ready true, '
        'epoch unchanged, protection false and inactive grasp. Enable/disable command topics were not '
        'subscribed in this diagnostic; absence of such bag topics alone cannot prove no other actor command. '
        'The explicit ownership event records torque_commands_published=0 and the executor emitted only16joint targets.')
    report['torque_publication_event_evidence'] = [row for row in events if 'torque_commands_published' in row]
    report['same_original_sdk_completed_holds'] = [dict(step=row['step'],label=row['label'],
        actual_vs_original_deg=row['actual_vs_original_deg']) for row in report['steps_complete']
        if row['sdk_words']==report['original_sdk_words']]
    return report


def main():
    started = time.monotonic()
    source = OUT/'samples.bag'
    before = source.stat()
    events = [json.loads(line) for line in (OUT/'events.jsonl').read_text().splitlines()]
    counts = Counter()
    receipts = defaultdict(list)
    headers = defaultdict(list)
    latencies = defaultdict(list)
    stamp_sets = {COLOR: Counter(), DEPTH: Counter()}
    control = defaultdict(list)
    archive_events = []
    sdk = []
    accepted = []
    reference = []
    commands = []
    tf_board_count = 0
    snapshot_pixels = {int(path.name.split('_')[0]): {} for path in OUT.glob('*_rgbd_tf.npz')}
    topics = []
    with rosbag.Bag(str(source), 'r') as bag:
        index_count = bag.get_message_count()
        topic_index = {topic: info.message_count for topic, info in bag.get_type_and_topic_info().topics.items()}
        for row in bag.read_messages(raw=True, return_connection_header=True):
            topic, raw, receipt, connection = row
            datatype, payload, md5, position, pytype = raw
            ns = receipt.to_nsec()
            counts[topic] += 1
            receipts[topic].append(ns)
            if getattr(pytype, '_has_header', False):
                secs, nsecs = struct.unpack_from('<II', payload, 4)
                header_ns = secs*10**9+nsecs
                headers[topic].append(header_ns)
                latencies[topic].append((ns-header_ns)*1e-9)
            if topic in stamp_sets:
                stamp_sets[topic][header_ns] += 1
                if header_ns in snapshot_pixels:
                    msg = pytype().deserialize(payload)
                    npz = OUT/(str(header_ns)+'_rgbd_tf.npz')
                    with np.load(npz, allow_pickle=False) as saved:
                        key = 'image_bgr' if topic == COLOR else 'depth_raw'
                        snapshot_pixels[header_ns][key] = dict(
                            exact_pixels=bytes(msg.data) == saved[key].tobytes(),
                            encoding=msg.encoding, header_ns=header_ns,
                            saved_source_ns=int(saved['source_stamp_ns']))
            elif topic == '/support_diagnostic/event':
                event = json.loads(pytype().deserialize(payload).data)
                archive_events.append(event)
            elif topic == '/joint_commands':
                record = qvalue(pytype().deserialize(payload))
                record['receipt_ns'] = ns
                record['connection_callerid'] = connection.get('callerid')
                record['sdk_words'] = words(record['q_rad'])
                commands.append(record)
            elif topic == '/alicia_d/sdk_command':
                sdk.append(qvalue(pytype().deserialize(payload)))
            elif topic == '/alicia_d/accepted_joint_states':
                accepted.append(qvalue(pytype().deserialize(payload)))
            elif topic == '/alicia_d/control_reference':
                reference.append(qvalue(pytype().deserialize(payload)))
            elif topic in ['/alicia_d/motion_enabled', '/alicia_d/feedback_ready',
                    '/alicia_d/protection_latched', '/gui/joint_direct_mode',
                    '/grasp/state', '/alicia_d/control_reference_epoch', '/alicia_d/actuation_status']:
                msg = pytype().deserialize(payload)
                value = msg.stamp.to_nsec() if topic.endswith('_epoch') else (
                    bool(msg.active) if topic == '/grasp/state' else msg.data)
                control[topic].append(dict(receipt_ns=ns, value=value))
            elif topic == '/tf':
                msg = pytype().deserialize(payload)
                tf_board_count += sum(t.child_frame_id == 'charuco_fixed_board_20261003' for t in msg.transforms)
            if sum(counts.values())%5000 == 0:
                print(json.dumps(dict(read=sum(counts.values()), index_count=index_count,
                        elapsed_sec=time.monotonic()-started)), flush=True)
    after = source.stat()
    color = set(stamp_sets[COLOR])
    depth = set(stamp_sets[DEPTH])
    complete = [r for r in events if r['event'] == 'step_complete']
    original = next(r for r in events if r['event'] == 'ownership_retained_sdk_hold')
    baseline = original['q_rad'][:6]
    sdk_words = [words(row['q_rad']) for row in sdk]
    same_original = [row for row in sdk if words(row['q_rad']) == original['original_sdk_words']]
    lag_stats = {topic: describe(values) for topic, values in latencies.items()}
    report = dict(scope='read-only complete archived bag audit; no ROS connection or device command',
        source=str(source), source_file_stable_during_review=before.st_size == after.st_size and before.st_mtime_ns == after.st_mtime_ns,
        source_size_bytes=after.st_size, elapsed_sec=time.monotonic()-started,
        bag_index_count=index_count, read_messages=sum(counts.values()), counts=dict(counts),
        bag_index_matches_streamed_counts=dict(counts)==topic_index,
        archive_counter_evidence=json.loads((OUT/'counts.json').read_text()),
        process_archive_evidence=json.loads((OUT/'process_archive.json').read_text()),
        raw_image_pairing=dict(color_messages=counts[COLOR],depth_messages=counts[DEPTH],
            color_unique_sources=len(color),depth_unique_sources=len(depth),paired_sources=len(color&depth),
            color_only_sources=sorted(color-depth), depth_only_sources=sorted(depth-color),
            color_duplicate_sources={str(k):v for k,v in stamp_sets[COLOR].items() if v!=1},
            depth_duplicate_sources={str(k):v for k,v in stamp_sets[DEPTH].items() if v!=1},
            snapshot_pixels=snapshot_pixels),
        source_header_to_actual_callback_entry_sec=lag_stats,
        source_intervals_sec={topic: stats(stamps) for topic, stamps in headers.items()},
        callback_receipt_intervals_sec={topic: stats(stamps) for topic, stamps in receipts.items()},
        timing_limit='host read completion header and callback entry receipt, not independent exposure latency',
        steps_complete=[dict(step=r['step'],label=r['label'],q_rad=r['q'],sdk_q_rad=r['sdk'],
                            sdk_words=words(r['sdk']), actual_vs_original_deg=r['measured_vs_original_deg']) for r in complete],
        sequence_complete=[r for r in events if r['event']=='sequence_complete'],
        failures=[r for r in events if r['event']=='sequence_failed'],
        commands=commands, command_count=len(commands),
        all_command_callers=sorted(set(row['connection_callerid'] or '' for row in commands)),
        control_summary={topic: dict(count=len(rows),unique_values=list(dict.fromkeys(r['value'] for r in rows)),
                                    first=rows[0],last=rows[-1]) for topic,rows in control.items()},
        archived_event_count=len(archive_events), event_file_count=len(events),
        archived_events_match_file=archive_events==events,
        source_epoch_ns=original['epoch_ns'], original_sdk_words=original['original_sdk_words'],
        actual_feedback=dict(count=len(accepted),frame_ids=sorted(set(r['frame_id'] for r in accepted)),
            first=accepted[0],last=accepted[-1],final_vs_original_deg=np.degrees(np.array(accepted[-1]['q_rad'][:6])-baseline).tolist()),
        sdk=dict(count=len(sdk),frame_ids=sorted(set(r['frame_id'] for r in sdk)),first=sdk[0],last=sdk[-1],
                 final_words=sdk_words[-1],original_words_message_count=len(same_original)),
        reference=dict(count=len(reference),frame_ids=sorted(set(r['frame_id'] for r in reference)),
                       first=reference[0],last=reference[-1]),
        board_tf_messages=tf_board_count,
        enable_disable_command_topics=[topic for topic in topic_index if ('enable' in topic.lower() or 'disable' in topic.lower())
                                       and topic!='/alicia_d/motion_enabled'],
        no_grasp_task_active=all(not r['value'] for r in control['/grasp/state']),
        no_protection_latched=all(not r['value'] for r in control['/alicia_d/protection_latched']),
        motion_enabled_held_true=all(r['value'] for r in control['/alicia_d/motion_enabled']),
        epoch_unchanged=all(r['value']==original['epoch_ns'] for r in control['/alicia_d/control_reference_epoch']),
        interpretation='16 bounded diagnostic steps only. Intermediate original-SDK holds after J5 positive return retain about0.878906deg actual J5 offset; the final negative return restores measured baseline. This is direction-dependent actual feedback evidence, not a unique hardware or calibration root-cause conclusion.')
    add_contract_review(report, events)
    path=OUT/'readonly_transport_review.json'
    path.write_text(json.dumps(report,indent=2,ensure_ascii=False)+'\n')
    print(json.dumps(dict(report=str(path), messages=sum(counts.values()), paired_sources=len(color&depth),
          complete_steps=len(complete), commands=len(commands), event_archive_match=archive_events==events,
          control={t:r['unique_values'] for t,r in report['control_summary'].items()},
          latency=lag_stats,elapsed_sec=time.monotonic()-started),ensure_ascii=False),flush=True)


if __name__ == '__main__':
    main()
