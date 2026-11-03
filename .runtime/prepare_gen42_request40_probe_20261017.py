"""Prepare archived request 40 inputs only; no geometry/candidate timing run.

Uses a closed bag and an in-process TF BufferCore. Every output is on SanStand.
The boundary is installed before importing any ROS message/bag libraries.
"""
import argparse
import gzip
import hashlib
import json
import os
from pathlib import Path
import socket
import struct
import sys
import time

W = Path(__file__).resolve().parents[1]
MOUNT = Path('/media/zhuyupei/SanStand')
E = MOUNT/'实验记录/Robot-Controller'
D = E/'ros_log/full_latest_gen36_restart_20261017T012037Z'
G = E/'grasp_records/20261017T114614.348253Z_b91fa535543247a4808d10062586d34d'
STAMPS = (1792237678248981237, 1792237678533539533, 1792237679451562404)
CAS = {
    'snapshot': 'b22627b8e2b7585bdb433b07b21d35bcbe9b588ab26710a00ebf777a9f7f108b',
    'reference': '35cf47a020cd7d5e963d93f8744a95a7e42c1f842c25c3cd743a6ecac2f4c74d',
    'before': '0190863e9a0658bf8972064ae7f9ae13880517a3fb01dd28c8867965cae2fcee',
}


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read(path):
    return json.loads(Path(path).read_bytes())


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', required=True, type=Path)
    a = p.parse_args()
    output = a.output.resolve()
    assert MOUNT.is_mount() and E/'ros_log' in output.parents
    assert output.parent.exists() and output.parent.stat().st_dev == MOUNT.stat().st_dev
    output.mkdir(exist_ok=False)
    os.environ['ROS_LOG_DIR'] = str(output/'ros')
    os.environ['TMPDIR'] = str(output/'tmp')
    (output/'tmp').mkdir()
    sys.dont_write_bytecode = True
    # Avoid OS crash-reporting imports and PyCryptodome's optional `file`
    # subprocess during rosbag import. Architecture comes from this process.
    sys.excepthook = sys.__excepthook__
    import platform
    platform.architecture = lambda *args, **kwargs: (str(struct.calcsize('P')*8)+'bit', 'ELF')
    platform._syscmd_uname = lambda option, default='': default
    socket.has_ipv6 = False
    violations = []

    def save(name, value):
        assert MOUNT.is_mount() and output.stat().st_dev == MOUNT.stat().st_dev
        with (output/name).open('x') as f:
            json.dump(value, f, ensure_ascii=False, indent=2, allow_nan=False)
            f.write('\n')

    class Boundary(BaseException):
        pass

    def forbidden(*args, **kwargs):
        violations.append(repr(args)[:500])
        save('boundary_violation_'+str(len(violations))+'.json', dict(args=repr(args)[:500]))
        raise Boundary('offline request40 input preparation boundary')

    for name in ('connect', 'connect_ex', 'sendto', 'bind', 'listen'):
        setattr(socket.socket, name, forbidden)
    socket.create_connection = forbidden
    socket.getaddrinfo = forbidden

    def guard(event, args):
        if event in ('subprocess.Popen', 'os.system', 'os.posix_spawn', 'os.exec'):
            forbidden(event, args)
        if event == 'open' and args and isinstance(args[0], (str, bytes, os.PathLike)):
            path = os.path.realpath(os.fsdecode(args[0]))
            if path.startswith('/dev/') and path not in (
                    '/dev/null', '/dev/zero', '/dev/random', '/dev/urandom') and not path.startswith(
                    ('/dev/shm/', '/dev/fd/', '/dev/pts/')):
                forbidden('physical_device_open', path)
    sys.addaudithook(guard)
    import numpy as np
    import rosbag
    import rospy
    from genpy import Time, Duration
    from tf2_py import BufferCore
    for name in ('init_node', 'Publisher', 'Subscriber', 'Service', 'ServiceProxy',
                 'wait_for_service', 'set_param', 'delete_param', 'get_param', 'get_master'):
        setattr(rospy, name, forbidden)

    release_path = D/'verification/gen41_release_manifest.json'
    release = read(release_path)
    assert sha(release_path) == 'b98c25da01b77582bff4567a791afed8b8c908fcfe391b3b84601276c9ed706d'
    source = release['source_sha256']
    assert len(source) == 1032 and all(sha(W/n) == h for n, h in source.items())
    manifest = read(G/'manifest.json')
    assert manifest['state'] == 'complete' and manifest['result'] == 'failure' and manifest['recorder_closed']
    bag_row = next(row for row in manifest['files'] if row['path'] == 'keyframes.bag')
    assert sha(G/'keyframes.bag') == bag_row['sha256']
    input_hashes = {str(p): sha(p) for p in (G/'manifest.json', G/'keyframes.bag',
        G/'perception_execution_configuration.json', D/'gen41_runtime/grasp_scene_r2/camera_configuration.json')}
    cas_root = D/'gen41_runtime/compute_deployment_r1/consumers_ros/contact_replay_receipts/sha256'
    cas_rows = {}
    for role, digest in CAS.items():
        path = cas_root/digest[:2]/(digest+'.json.gz')
        data = gzip.decompress(path.read_bytes())
        assert hashlib.sha256(data).hexdigest() == digest
        row = json.loads(data)
        assert row['snapshot_stamp_ns'] == STAMPS[-1]
        assert row['request_binding']['source_stamp_ns'] == STAMPS[-1]
        assert row['request_binding']['object_points_sha256'] == 'eb9ff1c222b80245d63b1fc1ad33be7b5ac1a11d3921b456f67328a3f8164485'
        save('cas_'+role+'.json', row)
        input_hashes[str(path)] = sha(path)
        cas_rows[role] = dict(source_path=str(path), content_sha256=digest,
            point_count=len(row['points_base']), view_stamps_ns=row['view_stamps_ns'])
    save('camera_configuration.json', read(D/'gen41_runtime/grasp_scene_r2/camera_configuration.json'))
    save('perception_execution_configuration.json', read(G/'perception_execution_configuration.json'))
    topics = {'/supervisor/camera/color/image_raw': 'color',
              '/supervisor/camera/depth/image_raw': 'depth', '/perception/object_mask': 'mask'}
    counts = {str(s): {role:0 for role in topics.values()} for s in STAMPS}
    arrays, objects, samples = {}, {}, []
    tf = BufferCore(Duration(300))
    transforms = 0
    with rosbag.Bag(str(G/'keyframes.bag'), 'r') as bag:
        for topic, msg, received in bag.read_messages(topics=list(topics)+['/tf', '/tf_static', '/perception/object']):
            if topic in ('/tf', '/tf_static'):
                for transform in msg.transforms:
                    (tf.set_transform_static if topic == '/tf_static' else tf.set_transform)(transform, 'closed_archive_only')
                    transforms += 1
                continue
            stamp = msg.header.stamp.to_nsec()
            if stamp not in STAMPS:
                continue
            if topic == '/perception/object':
                objects[str(stamp)] = dict(detected=msg.detected, label=msg.label,
                    confidence=msg.confidence, frame_id=msg.header.frame_id,
                    bbox=[msg.bbox_x,msg.bbox_y,msg.bbox_width,msg.bbox_height],
                    bag_receive_ns=received.to_nsec())
                continue
            role = topics[topic]
            counts[str(stamp)][role] += 1
            assert counts[str(stamp)][role] == 1, ('duplicate source image', stamp, role)
            raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
            if role == 'depth':
                assert msg.encoding in ('16UC1', 'mono16')
                array = raw[:,:msg.width*2].copy().view('>u2' if msg.is_bigendian else '<u2').reshape(msg.height,msg.width)
                array = array.astype(np.uint16)
            else:
                assert msg.encoding == ('bgr8' if role == 'color' else 'mono8')
                channels = 3 if role == 'color' else 1
                shape = (msg.height,msg.width,3) if channels == 3 else (msg.height,msg.width)
                array = raw[:,:msg.width*channels].copy().reshape(shape)
            path = output/(str(stamp)+'_'+role+'.npy')
            np.save(path, array, allow_pickle=False)
            arrays[path.name] = dict(sha256=sha(path), shape=list(array.shape), dtype=str(array.dtype),
                raw_array_sha256=hashlib.sha256(array.tobytes()).hexdigest(), frame_id=msg.header.frame_id,
                bag_receive_ns=received.to_nsec())
    assert all(all(n == 1 for n in row.values()) for row in counts.values()), counts
    assert set(objects) == set(counts) and all(r['detected'] for r in objects.values())
    tf_rows = {}
    for stamp in STAMPS:
        t = tf.lookup_transform_core('base_link', 'camera_link', Time(*divmod(stamp,10**9)))
        v, q = t.transform.translation, t.transform.rotation
        # tf2_py's Python Time conversion can round the returned label by one
        # nanosecond (also present in the sealed GEN40 exact-TF receipt). The
        # requested lookup uses the original integer secs/nsecs unchanged.
        delta = t.header.stamp.to_nsec()-stamp
        assert abs(delta) <= 1, ('tf2 Python return-label conversion', stamp, delta)
        tf_rows[str(stamp)] = dict(translation=[v.x,v.y,v.z], quaternion_xyzw=[q.x,q.y,q.z,q.w],
            requested_lookup_stamp_ns=stamp, returned_stamp_ns=t.header.stamp.to_nsec(),
            python_tf_return_stamp_delta_ns=delta, exact_stamp_interpolation=True)
    save('object_messages.json', objects)
    save('exact_tf.json', tf_rows)
    assert not violations and all(sha(W/n) == h for n,h in source.items())
    assert all(sha(path) == digest for path,digest in input_hashes.items())
    save('data_readiness.json', dict(passed=True, prepared_ns=time.time_ns(), request_id=40,
        requested_stamps_ns=STAMPS, snapshot_stamp_ns=STAMPS[-1], counts=counts, arrays=arrays,
        input_sha256=input_hashes, cas=cas_rows, tf_transforms_read=transforms,
        source_count=len(source), source_unchanged=True, release_sha256=sha(release_path),
        helper_sha256=sha(__file__), boundary_attempts=violations,
        geometry_or_candidate_probe_run=False, ROS_initialized=False, hardware_contacted=False,
        limitation='request40 has no final gate audit and no historical callback/lock trace; future timing is offline observation only'))
    print(json.dumps(dict(data_ready=True, output=str(output), array_count=len(arrays), tf_stamps=len(tf_rows))))


if __name__ == '__main__':
    main()
