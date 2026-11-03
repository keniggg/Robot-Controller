"""Read only a closed G49 bag: exact arrival RGB and non-temperature status."""
import base64
import hashlib
import io
import json
import os
from pathlib import Path
import platform
import socket
import struct
import sys
import time

W = Path(__file__).resolve().parents[1]
E = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
G = E / 'grasp_records/20261018T110845.987064Z_143620cd21094235a5ac9166e012ca87'
O = E / 'ros_log/full_latest_gen41_restart_20261018T011222Z/verification/gen49_arrival_visibility_r1'
START, END = 1792321794500000000, 1792321804653000000
EXACT = (1792321728948197364, 1792321796018874883, 1792321796208590507, 1792321796341869592)
COLOR = '/supervisor/camera/color/image_raw'
MASK = '/perception/object_mask'
TOPICS = (COLOR, MASK, '/perception/object', '/perception/unknown/detector_status',
          '/grasp/state', '/grasp/task_event', '/grasp/near_field_phase',
          '/grasp_mode/observe_return_measurement_request',
          '/grasp_mode/observe_return_measurement_ack',
          '/supervisor/camera/color/camera_info', '/grasp_record/camera_runtime_metadata')
ATTEMPTS = []
sys.dont_write_bytecode = True
os.environ['OPENBLAS_NUM_THREADS'] = '1'
os.environ['OMP_NUM_THREADS'] = '1'
native_uname = os.uname()
platform.uname = lambda: platform.uname_result(*(tuple(native_uname) + (native_uname.machine,)))
platform.architecture = lambda *a, **k: (str(8 * struct.calcsize('P')) + 'bit', 'ELF')
def boundary(event, args):
    if event.startswith('socket.') or event in ('subprocess.Popen', 'os.system', 'os.exec', 'os.posix_spawn', 'os.fork', 'os.forkpty'):
        ATTEMPTS.append(event)
        raise RuntimeError('offline external boundary: ' + event)
    if event == 'open' and isinstance(args[0], (str, bytes, os.PathLike)):
        name = os.fsdecode(args[0]); flags = args[2] if len(args) > 2 and isinstance(args[2], int) else 0
        mode = args[1] if len(args) > 1 and isinstance(args[1], str) else ''
        if (name.startswith('/dev/') and name not in ('/dev/null', '/dev/urandom', '/dev/random')) or '/hwmon/' in name or name.startswith('/sys/class/thermal'):
            ATTEMPTS.append('device:' + name); raise RuntimeError('offline device boundary')
        if name != '/dev/null' and (flags & (os.O_WRONLY | os.O_RDWR | os.O_CREAT | os.O_TRUNC | os.O_APPEND) or any(k in mode for k in 'wax+')):
            try: Path(name).resolve().relative_to(O.resolve())
            except ValueError:
                ATTEMPTS.append('write:' + name); raise RuntimeError('offline write boundary')
sys.addaudithook(boundary)
sys.path[:0] = [str(W / 'devel/lib/python3/dist-packages'), '/opt/ros/noetic/lib/python3/dist-packages']
import rosbag
import rospy
import cv2
import numpy as np
cv2.setNumThreads(1)
def blocked(*a, **k):
    ATTEMPTS.append('ROS_API'); raise RuntimeError('offline ROS API boundary')
for api in ('init_node','Publisher','Subscriber','Service','ServiceProxy','wait_for_service','get_param','set_param','delete_param'):
    setattr(rospy, api, blocked)
def sha(p):
    h = hashlib.sha256()
    with Path(p).open('rb') as f:
        for block in iter(lambda: f.read(4 * 1024 * 1024), b''): h.update(block)
    return h.hexdigest()
def payload(x):
    if hasattr(x, '__slots__'): return {k: payload(getattr(x, k)) for k in x.__slots__}
    if isinstance(x, (list, tuple)): return [payload(v) for v in x]
    if isinstance(x, bytes): return {'bytes_base64': base64.b64encode(x).decode()}
    return x
def save(p, x):
    p.write_text(json.dumps(x, ensure_ascii=False, indent=2, allow_nan=False) + '\n')
    assert json.loads(p.read_text()) == x
def header_ns(m):
    return m.header.stamp.to_nsec() if hasattr(m, 'header') else None
def image_meta(m, received):
    return dict(received_ns=received, source_stamp_ns=header_ns(m), frame_id=m.header.frame_id,
                width=m.width, height=m.height, encoding=m.encoding, step=m.step,
                data_sha256=hashlib.sha256(m.data).hexdigest(), data_bytes=len(m.data))
def main():
    assert Path('/media/zhuyupei/SanStand').is_mount() and not O.exists()
    manifest = json.loads((G/'manifest.json').read_text())
    assert sha(G/'manifest.json') == '747a7e1f2240a6e612763e4320a34a428975fb4fe022b278c479f695b8ad7ed0'
    assert manifest['state'] == 'complete' and manifest['recorder_closed'] is True
    bref = next(v for v in manifest['files'] if v['path'] == 'keyframes.bag')
    assert sha(G/'keyframes.bag') == bref['sha256']
    O.mkdir(); (O/'tmp').mkdir(); os.environ['TMPDIR'] = str(O/'tmp')
    import tempfile
    tempfile.tempdir = str(O/'tmp')
    colors, masks, states, selected, latest = [], [], [], {}, None
    with rosbag.Bag(str(G/'keyframes.bag'), 'r') as bag:
        info = bag.get_type_and_topic_info().topics
        metadata = {t: dict(type=info[t].msg_type, count=info[t].message_count) for t in TOPICS if t in info}
        assert all('temperature' not in t.lower() for t in metadata)
        for topic, m, received in bag.read_messages(topics=list(TOPICS)):
            ns = header_ns(m); recv = received.to_nsec()
            inside = START <= (ns if ns else recv) <= END
            if topic == COLOR:
                if inside: colors.append(image_meta(m, recv))
                if ns in EXACT: selected[ns] = (m, recv, 'exact_requested_source')
                if ns and START <= ns <= END and (latest is None or ns > latest[0]): latest = (ns, m, recv)
            elif topic == MASK:
                if inside or ns in EXACT:
                    row = image_meta(m, recv)
                    row['nonzero_count'] = int(np.count_nonzero(np.frombuffer(m.data, dtype=np.uint8).reshape(m.height, m.step)[:, :m.width]))
                    masks.append(row)
            elif inside or topic in ('/grasp/state','/grasp_mode/observe_return_measurement_request','/grasp_mode/observe_return_measurement_ack'):
                stream = io.BytesIO(); m.serialize(stream); raw = stream.getvalue()
                parsed = None
                if m._type == 'std_msgs/String':
                    try: parsed = json.loads(m.data)
                    except (ValueError, TypeError): pass
                states.append(dict(topic=topic, received_ns=recv, source_stamp_ns=ns, message_type=m._type,
                                   wire_sha256=hashlib.sha256(raw).hexdigest(), wire_base64=base64.b64encode(raw).decode(),
                                   payload=payload(m), parsed_json=parsed))
    if latest: selected.setdefault(latest[0], (latest[1], latest[2], 'latest_recorded_RGB_before_Task_failure'))
    pictures = []
    for ns, (m, recv, why) in sorted(selected.items()):
        assert m.encoding in ('bgr8', 'rgb8')
        arr = np.frombuffer(m.data, dtype=np.uint8).reshape(m.height, m.step)[:, :m.width * 3].reshape(m.height, m.width, 3)
        bgr = arr if m.encoding == 'bgr8' else arr[:, :, ::-1]
        png = O / ('original_color_%s.png' % ns)
        assert cv2.imwrite(str(png), bgr) and np.array_equal(cv2.imread(str(png)), bgr)
        row = image_meta(m, recv); row.update(reason=why, path=str(png), png_sha256=sha(png),
            decoded_BGR_sha256=hashlib.sha256(np.ascontiguousarray(bgr).tobytes()).hexdigest(), resampled=False, overlay_added=False)
        pictures.append(row)
    save(O/'arrival_messages.json', states)
    save(O/'image_headers.json', dict(color=colors, mask=masks, saved_RGB=pictures))
    assert sha(G/'keyframes.bag') == bref['sha256'] and not ATTEMPTS
    result = dict(kind='sealed_G49_arrival_RGB_and_state_extraction', passed=True, record=str(G),
        manifest_sha256=sha(G/'manifest.json'), bag_sha256=bref['sha256'], bag_before_after_same=True,
        requested_RGB_sources=list(EXACT), exact_requested_RGB_found=[ns for ns in EXACT if ns in selected],
        window_ns=[START,END], selected_topics=metadata, window_color_count=len(colors), window_mask_count=len(masks),
        saved_RGB=pictures, messages_path=str(O/'arrival_messages.json'), messages_sha256=sha(O/'arrival_messages.json'),
        headers_sha256=sha(O/'image_headers.json'), helper_sha256=sha(__file__), external_boundary_attempts=ATTEMPTS,
        ROS_initialized=False, hardware_contacted=False, temperature_read=False, original_record_modified=False,
        limit='Recorder arrival ordering does not prove subscriber callback ordering; extraction alone does not classify visual presence.')
    save(O/'result.json', result); print(json.dumps(dict(result=str(O/'result.json'), sha256=sha(O/'result.json'), pictures=pictures), ensure_ascii=False))
if __name__ == '__main__': main()
