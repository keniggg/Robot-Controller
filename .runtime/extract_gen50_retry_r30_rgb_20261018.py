"""Read one exact RGB message from the closed retry bag; no ROS connections."""
import hashlib, io, json, os, platform, struct, sys
from pathlib import Path

E = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
N = E / 'ros_log/full_latest_gen41_restart_20261018T011222Z'
G = E / 'grasp_records/20261018T123624.045793Z_1db3d3bb4c554d7590c8c8003c414fb5'
OUT = N / 'verification/gen50_retry_actual_handoff_r1/exact_r30_rgb'
STAMP = 1792327029323935508
AUDIT_SHA = '3fcf580665a8226efad87b55d1a693dbfd029a7a4641ad3208ee4679d8a7b39b'
BOUNDARY = []

def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        for b in iter(lambda: f.read(4 * 1024 * 1024), b''):
            h.update(b)
    return h.hexdigest()

def deny(kind):
    BOUNDARY.append(kind)
    raise RuntimeError('offline boundary: ' + kind)

def audit(event, args):
    if event.startswith('socket.') or event in ('subprocess.Popen', 'os.system', 'os.exec', 'os.posix_spawn', 'os.fork', 'os.forkpty'):
        deny(event)
    if event == 'open' and isinstance(args[0], (str, bytes, os.PathLike)):
        p = os.fsdecode(args[0])
        if p.startswith('/dev/') and p not in ('/dev/null', '/dev/urandom', '/dev/random'):
            deny('device_open')
        if p.startswith('/sys/class/thermal') or '/hwmon/' in p:
            deny('temperature_file')
        flags = args[2] if len(args) > 2 and isinstance(args[2], int) else 0
        mode = args[1] if len(args) > 1 and isinstance(args[1], str) else ''
        if p != '/dev/null' and (flags & (os.O_WRONLY | os.O_RDWR | os.O_CREAT | os.O_TRUNC | os.O_APPEND) or any(c in mode for c in 'wax+')):
            try:
                Path(p).resolve().relative_to(OUT.resolve())
            except ValueError:
                deny('write_outside_output')

def main():
    assert E.parent.parent.is_mount() and not OUT.exists()
    OUT.mkdir()
    os.environ.update(PYTHONDONTWRITEBYTECODE='1', ROS_MASTER_URI='http://127.0.0.1:1',
                      ROS_LOG_DIR=str(OUT), ROS_HOME=str(OUT), TMPDIR=str(OUT),
                      OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1')
    sys.dont_write_bytecode = True
    native = os.uname()
    platform.uname = lambda: platform.uname_result(*(tuple(native) + (native.machine,)))
    platform.architecture = lambda *a, **k: (str(8 * struct.calcsize('P')) + 'bit', 'ELF')
    sys.addaudithook(audit)
    import numpy as np
    import cv2
    import rosbag
    import genpy
    manifest = json.loads((G / 'manifest.json').read_text())
    assert manifest['state'] == 'complete' and manifest['result'] == 'failure' and manifest['recorder_closed'] is True
    bagref = next(x for x in manifest['files'] if x['path'] == 'keyframes.bag')
    bagpath = G / 'keyframes.bag'
    assert bagpath.stat().st_size == bagref['size_bytes'] and sha(bagpath) == bagref['sha256']
    auditpath = G / 'keyframes_audits' / (AUDIT_SHA + '.json')
    assert sha(auditpath) == AUDIT_SHA
    expected = json.loads(auditpath.read_text())['remote_diagnostics']['tabletop_geometry']['regular_box_inference']['retained_pixel_sha256']['color_sha256']
    rows = []
    with rosbag.Bag(str(bagpath), 'r') as bag:
        alltopics = bag.get_type_and_topic_info().topics
        topics = [t for t in ('/supervisor/camera/color/image_raw', '/experiment/observation_rgbd/color') if t in alltopics]
        assert topics
        lo = genpy.Time(STAMP // 10**9 - 1, 0)
        hi = genpy.Time(STAMP // 10**9 + 4, 0)
        for topic, msg, received in bag.read_messages(topics=topics, start_time=lo, end_time=hi):
            if msg.header.stamp.to_nsec() != STAMP:
                continue
            assert msg.encoding.lower() in ('rgb8', 'bgr8')
            raw = bytes(msg.data)
            assert len(raw) == msg.step * msg.height
            a = np.frombuffer(raw, dtype=np.uint8).reshape(msg.height, msg.step)[:, :msg.width * 3].reshape(msg.height, msg.width, 3).copy()
            bgr = a if msg.encoding.lower() == 'bgr8' else a[:, :, ::-1].copy()
            array_sha = hashlib.sha256(bgr.tobytes()).hexdigest()
            assert array_sha == expected, (topic, array_sha, expected)
            wire = io.BytesIO(); msg.serialize(wire)
            rows.append(dict(topic=topic, received_ns=received.to_nsec(), source_stamp_ns=STAMP,
                             frame_id=msg.header.frame_id, encoding=msg.encoding, height=msg.height,
                             width=msg.width, step=msg.step, raw_data_sha256=hashlib.sha256(raw).hexdigest(),
                             serialized_message_sha256=hashlib.sha256(wire.getvalue()).hexdigest(),
                             retained_bgr_array_sha256=array_sha))
            if len(rows) == 1:
                png = OUT / ('source_' + str(STAMP) + '.png')
                assert cv2.imwrite(str(png), bgr)
                decoded = cv2.imread(str(png), cv2.IMREAD_COLOR)
                assert np.array_equal(decoded, bgr)
    assert rows, 'exact source absent in both selected color topics within indexed window'
    result = dict(kind='exact_original_rgb_no_pixel_edits', source_stamp_ns=STAMP, exact=True,
                  bag_path=str(bagpath), bag_sha256=bagref['sha256'], manifest_sha256=sha(G / 'manifest.json'),
                  retained_audit_path=str(auditpath), retained_audit_sha256=AUDIT_SHA,
                  expected_retained_bgr_array_sha256=expected, messages=rows,
                  png_path=str(png), png_sha256=sha(png), png_decode_equals_original_pixels=True,
                  network_ROS_hardware_access=False, temperature_read=False, boundary_attempts=BOUNDARY,
                  helper_sha256=sha(Path(__file__)))
    p = OUT / 'result.json'
    p.write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n')
    assert json.loads(p.read_text()) == result
    print(json.dumps(dict(result=str(p), sha256=sha(p), png=str(png), exact=True, messages=len(rows)), ensure_ascii=False))

if __name__ == '__main__':
    main()
