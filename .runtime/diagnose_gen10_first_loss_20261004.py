"""Offline exact gen10 source/feedback evidence; no ROS master or robot I/O."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
import rosbag
import rospy
import tf2_ros
from cv_bridge import CvBridge
from tf.transformations import quaternion_matrix

from alicia_grasp_modes.rgb_observation_source import bracket_feedback

ANCHOR = 1791100580167863845
LAST_GOOD = 1791100581567524433
LOSS = 1791100581768626689
START = 1791100579000000000
END = 1791100583200000000
WORK = Path(__file__).resolve().parents[1]
BAG = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/grasp_records/'
           '20261004T075537.154018Z_cc3596cd9ae4469e904d6fd7a2650f16/keyframes.bag')


def stamp(ns):
    return rospy.Time(ns // 10**9, ns % 10**9)


def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(8 * 1024 * 1024), b''):
            h.update(chunk)
    return h.hexdigest()


def plain(value):
    if hasattr(value, '__slots__'):
        return {key: plain(getattr(value, key)) for key in value.__slots__}
    if isinstance(value, (list, tuple)):
        return [plain(item) for item in value]
    if isinstance(value, bytes):
        return value.hex()
    return value


def transform_matrix(value):
    t, q = value.transform.translation, value.transform.rotation
    result = quaternion_matrix([q.x, q.y, q.z, q.w])
    result[:3, 3] = [t.x, t.y, t.z]
    return result


def run(output):
    # Only external evidence writes; original bag and manifest remain immutable.
    output.mkdir(parents=True, exist_ok=False)
    manifest = json.loads((BAG.parent / 'manifest.json').read_text())
    expected = next(row['sha256'] for row in manifest['files'] if row['path'] == BAG.name)
    actual = sha(BAG)
    assert actual == expected, (actual, expected)
    bridge = CvBridge()
    tf = tf2_ros.Buffer(cache_time=rospy.Duration(300), debug=False)
    image_topics = {'/supervisor/camera/color/image_raw': 'color_bgr',
                    '/supervisor/camera/depth/image_raw': 'depth_raw',
                    '/perception/object_mask': 'published_mask'}
    statuses, accepted, diagnostics, objects, images, ancillary = [], [], [], [], {}, []
    metadata, epochs, tf_records = [], [], []
    with rosbag.Bag(str(BAG)) as bag:
        topic_info = {name: {'type': info.msg_type, 'count': info.message_count}
                      for name, info in bag.get_type_and_topic_info().topics.items()}
        for topic, msg, arrival in bag.read_messages(topics=[
                '/grasp_record/camera_runtime_metadata', '/alicia_d/control_reference_epoch',
                '/tf_static', '/tf']):
            ns = arrival.to_nsec()
            if topic == '/grasp_record/camera_runtime_metadata':
                metadata.append({'bag_arrival_ns': ns, 'message': json.loads(msg.data)})
            elif topic == '/alicia_d/control_reference_epoch':
                epochs.append({'bag_arrival_ns': ns, 'message': plain(msg)})
            else:
                static = topic == '/tf_static'
                for transform in msg.transforms:
                    if static:
                        tf.set_transform_static(transform, 'archived_bag')
                    elif START - 10**9 <= transform.header.stamp.to_nsec() <= END + 10**9:
                        tf.set_transform(transform, 'archived_bag')
                    if static or START <= ns <= END:
                        tf_records.append({'topic': topic, 'bag_arrival_ns': ns,
                                           'message': plain(transform)})
        topics = list(image_topics) + ['/perception/unknown/detector_status',
                    '/alicia_d/accepted_joint_states', '/alicia_d/sdk_diagnostics',
                    '/perception/object', '/alicia_d/actuation_status',
                    '/alicia_d/control_reference', '/alicia_d/sdk_command']
        for topic, msg, arrival in bag.read_messages(topics=topics,
                start_time=stamp(START), end_time=stamp(END)):
            ns = arrival.to_nsec()
            if topic in image_topics:
                source = msg.header.stamp.to_nsec()
                key = image_topics[topic]
                slot = images.setdefault(source, {})
                assert key not in slot
                raw = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough').copy()
                slot[key] = raw
                slot[key + '_metadata'] = dict(topic=topic, bag_arrival_ns=ns,
                    source_stamp_ns=source, source_seq=msg.header.seq,
                    frame_id=msg.header.frame_id, encoding=msg.encoding,
                    shape=list(raw.shape), dtype=str(raw.dtype),
                    bytes_sha256=hashlib.sha256(bytes(msg.data)).hexdigest(),
                    array_sha256=hashlib.sha256(raw.tobytes()).hexdigest())
            elif topic.endswith('detector_status'):
                statuses.append({'bag_arrival_ns': ns, 'message': json.loads(msg.data)})
            elif topic.endswith('accepted_joint_states'):
                accepted.append(dict(bag_arrival_ns=ns, source_stamp_ns=msg.header.stamp.to_nsec(),
                    seq=msg.header.seq, frame_id=msg.header.frame_id, name=list(msg.name),
                    position=list(msg.position), velocity=list(msg.velocity), effort=list(msg.effort)))
            elif topic.endswith('sdk_diagnostics'):
                diagnostics.append(dict(bag_arrival_ns=ns, source_stamp_ns=msg.header.stamp.to_nsec(),
                                        message=plain(msg)))
            elif topic == '/perception/object':
                objects.append(dict(bag_arrival_ns=ns, source_stamp_ns=msg.header.stamp.to_nsec(),
                                    message=plain(msg)))
            else:
                ancillary.append(dict(topic=topic, bag_arrival_ns=ns, message=plain(msg)))

    first = next(row for row in statuses if row['message'].get('source_stamp_ns') == LOSS)
    assert first['message']['target_lost']
    assert first['message']['rgb_identity_hold']['reason'] == 'accepted_feedback_does_not_bracket_source'
    observed = first['message']['first_loss']['observed_stamp_ns']
    # Bag arrival is independent subscriber receipt, not producer callback time.
    available = [row for row in accepted if row['bag_arrival_ns'] <= observed]
    lo = max((r for r in available if r['source_stamp_ns'] <= LOSS), key=lambda r: r['source_stamp_ns'])
    hi = min((r for r in available if r['source_stamp_ns'] >= LOSS), key=lambda r: r['source_stamp_ns'])
    error = None
    try:
        bracket_feedback([(r['source_stamp_ns'], r['position'][:6]) for r in available],
                         LOSS, first['message']['selection_stamp_ns'])
    except ValueError as exc:
        error = str(exc)
    assert error == 'accepted_feedback_does_not_bracket_source'
    span = hi['source_stamp_ns'] - lo['source_stamp_ns']
    assert span == 200043250
    raw_frames = []
    for source, slot in sorted(images.items()):
        if not all(key in slot for key in image_topics.values()):
            continue
        matrix = transform_matrix(tf.lookup_transform('base_link', 'camera_link', stamp(source)))
        tool_matrix = transform_matrix(tf.lookup_transform('tool0', 'camera_link', stamp(source)))
        path = output / ('source_%d.npz' % source)
        np.savez_compressed(path, color_bgr=slot['color_bgr'], depth_raw=slot['depth_raw'],
                            published_mask=slot['published_mask'], T_base_camera=matrix,
                            T_tool_camera=tool_matrix)
        raw_frames.append(dict(source_stamp_ns=source, file=str(path), sha256=sha(path),
            images={key: slot[key + '_metadata'] for key in image_topics.values()},
            T_base_camera=matrix.tolist(), T_tool_camera=tool_matrix.tolist()))
    assert {ANCHOR, LAST_GOOD, LOSS}.issubset({row['source_stamp_ns'] for row in raw_frames})
    latest_metadata = max((row for row in metadata if row['bag_arrival_ns'] <= LOSS),
                          key=lambda row: row['bag_arrival_ns'])
    summary = dict(first_loss_source_ns=LOSS, last_good_source_ns=LAST_GOOD,
        anchor_source_ns=ANCHOR, first_loss_hold=first['message']['rgb_identity_hold'],
        accepted_lo=lo, accepted_hi=hi, bracket_span_ns=span,
        bracket_limit_ns=200000000, bracket_excess_ns=span - 200000000,
        production_bracket_error=error,
        endpoints_six_joints_exact_equal=lo['position'][:6] == hi['position'][:6],
        raw_pair_sources=[r['source_stamp_ns'] for r in raw_frames],
        first_loss_bag_arrival_ns=first['bag_arrival_ns'], first_loss_observed_ns=observed)
    report = dict(schema=1, evidence_scope='Offline exact original gen10 first loss; no resampling or fabricated feedback',
        timing_scope='Source header nanoseconds and bag receipt are distinct; camera metadata is a parameter snapshot, not image-bound calibration',
        provenance=dict(bag=str(BAG), bag_sha256=actual, manifest_bag_sha256=expected,
            script=str(Path(__file__).resolve()), script_sha256=sha(__file__),
            producer_sha256=sha(WORK / 'src/alicia_grasp_modes/scripts/unknown_tabletop_perception.py')),
        summary=summary, topic_info=topic_info, statuses=statuses, accepted_feedback=accepted,
        sdk_diagnostics=diagnostics, object_messages=objects, ancillary=ancillary,
        raw_frames=raw_frames, latest_camera_metadata_before_loss=latest_metadata,
        camera_metadata=metadata, control_reference_epochs=epochs, tf_records=tf_records)
    result = output / 'report.json'
    result.write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
    print(json.dumps(dict(report=str(result), report_sha256=sha(result), summary=summary), indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True, type=Path)
    run(parser.parse_args().output)
