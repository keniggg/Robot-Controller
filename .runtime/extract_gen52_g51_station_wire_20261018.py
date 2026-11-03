"""Exact nonimage station-expiry messages from the closed GEN51 attempt."""
import base64, hashlib, io, json, math, os, platform, struct, sys, time
from pathlib import Path
import extract_gen50_retry_r30_rgb_20261018 as boundary

W = Path(__file__).resolve().parents[1]
E, N = boundary.E, boundary.N
V = N / 'verification'
G = E / 'grasp_records/20261018T132526.747381Z_787872d637b749b29ea988feb844e2e8'
OUT = V / 'gen52_station_expiry_evidence_r1'
TOPICS = (
    '/gui/joint_direct_mode', '/grasp/state', '/grasp/task_event', '/grasp/near_field_phase',
    '/grasp_mode/selection', '/grasp_mode/status',
    '/grasp_mode/observe_return_session', '/grasp_mode/observe_return_ack',
    '/grasp_mode/observe_return_measurement_request', '/grasp_mode/observe_return_measurement_ack',
    '/grasp_mode/contact_station_retirement_request', '/grasp_mode/contact_station_retirement_ack',
    '/grasp_6d/status', '/perception/unknown/detector_status',
    '/grasp_6d/plan_enriched', '/grasp_6d/preview_plan_enriched', '/grasp_6d/frozen_contact_plan',
    '/grasp_6d/object_geometry', '/perception/object',
)

def plain(value):
    if hasattr(value, '__slots__'):
        return {k: plain(getattr(value, k)) for k in value.__slots__}
    if isinstance(value, (tuple, list)):
        return [plain(x) for x in value]
    if isinstance(value, bytes):
        return {'bytes_base64': base64.b64encode(value).decode()}
    if isinstance(value, float) and not math.isfinite(value):
        return {'original_nonfinite_repr': repr(value)}
    return value

def put(path, value):
    with path.open('x') as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write('\n')
    assert json.loads(path.read_text()) == value

def main():
    assert E.parent.parent.is_mount() and not OUT.exists()
    OUT.mkdir(); (OUT / 'wire').mkdir()
    os.environ.update(PYTHONDONTWRITEBYTECODE='1', ROS_MASTER_URI='http://127.0.0.1:1',
                      ROS_LOG_DIR=str(OUT), ROS_HOME=str(OUT), TMPDIR=str(OUT))
    sys.dont_write_bytecode = True
    sys.path[:0] = [str(W / 'devel/lib/python3/dist-packages'), '/opt/ros/noetic/lib/python3/dist-packages']
    native = os.uname()
    platform.uname = lambda: platform.uname_result(*(tuple(native) + (native.machine,)))
    platform.architecture = lambda *a, **k: (str(8 * struct.calcsize('P')) + 'bit', 'ELF')
    boundary.OUT = OUT
    sys.addaudithook(boundary.audit)
    import rosbag
    manifest = json.loads((G / 'manifest.json').read_text())
    assert manifest['state'] == 'complete' and manifest['result'] == 'failure' and manifest['recorder_closed'] is True
    bagrow = next(x for x in manifest['files'] if x['path'] == 'keyframes.bag')
    bagpath = G / 'keyframes.bag'
    assert bagpath.stat().st_size == bagrow['size_bytes'] and boundary.sha(bagpath) == bagrow['sha256']
    rows = {t: [] for t in TOPICS}; ordered = []
    with rosbag.Bag(str(bagpath), 'r') as bag:
        available = bag.get_type_and_topic_info().topics
        selected = {t: dict(type=available[t].msg_type, message_count=available[t].message_count)
                    for t in TOPICS if t in available}
        assert all('Image' not in v['type'] and 'temperature' not in t.lower() for t, v in selected.items())
        for ordinal, (topic, msg, received) in enumerate(bag.read_messages(topics=list(selected))):
            assert msg._type not in ('sensor_msgs/Image', 'sensor_msgs/CompressedImage')
            stream = io.BytesIO(); msg.serialize(stream); raw = stream.getvalue()
            parsed = None
            if msg._type == 'std_msgs/String':
                try:
                    parsed = json.loads(msg.data)
                except (ValueError, TypeError):
                    pass
            row = dict(selected_topic_order=ordinal, received_ns=received.to_nsec(),
                       message_type=msg._type, md5sum=msg._md5sum,
                       wire_sha256=hashlib.sha256(raw).hexdigest(),
                       wire_base64=base64.b64encode(raw).decode(), payload=plain(msg), parsed_json=parsed)
            ordered.append(dict(order=ordinal, topic=topic, index=len(rows[topic]),
                                received_ns=row['received_ns'], wire_sha256=row['wire_sha256']))
            rows[topic].append(row)
    files = {}
    for i, topic in enumerate(TOPICS):
        p = OUT / 'wire' / ('%02d_%s.json' % (i, topic.strip('/').replace('/', '__')))
        put(p, dict(topic=topic, messages=rows[topic]))
        files[topic] = dict(path=str(p), sha256=boundary.sha(p), messages=len(rows[topic]))
    put(OUT / 'recorder_selected_message_order.json', ordered)
    assert not boundary.BOUNDARY and boundary.sha(bagpath) == bagrow['sha256']
    result = dict(kind='GEN52_exact_G51_closed_nonimage_station_wire_extraction', passed=True,
                  record=str(G), manifest_sha256=boundary.sha(G / 'manifest.json'), bag_path=str(bagpath),
                  bag_sha256=bagrow['sha256'], bag_size_bytes=bagrow['size_bytes'], bag_before_after_same=True,
                  explicit_topic_whitelist=list(TOPICS), selected_topic_metadata=selected,
                  files=files, selected_messages=len(ordered),
                  selected_message_order_sha256=boundary.sha(OUT / 'recorder_selected_message_order.json'),
                  helper_sha256=boundary.sha(Path(__file__)), reused_boundary_helper_sha256=boundary.sha(Path(boundary.__file__)),
                  external_boundary_attempts=boundary.BOUNDARY, image_or_temperature_topics_read=False,
                  ROS_initialized=False, hardware_contacted=False, source_record_or_tests_modified=False,
                  recorder_receive_order_is_not_subscriber_callback_order=True, created_ns=time.time_ns())
    put(OUT / 'extraction.json', result)
    print(json.dumps(dict(result=str(OUT / 'extraction.json'), sha256=boundary.sha(OUT / 'extraction.json'),
                         count=len(ordered), topic_counts={t: len(v) for t, v in rows.items()}), ensure_ascii=False))

if __name__ == '__main__':
    main()
