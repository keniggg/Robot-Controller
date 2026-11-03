"""Export exact sealed initial RGBD evidence only; no ROS node/device calls."""
import hashlib
import json
from pathlib import Path
import cv2
import numpy as np
import rosbag
import rospy
import tf2_py
from cv_bridge import CvBridge
from tf.transformations import quaternion_matrix

E = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
R = E / 'grasp_records/20261015T030730.930818Z_2ff7836aaadc4172980c58ba2741d935'
OUT = E / 'ros_log/full_latest_gen22_restart_20261015T003700Z/verification/gen26_box_inference_design'
A = R / 'keyframes_audits/f531458d35b1b73e8ab32a29209f08d7cfee055f597733a8de99e3fe3f535d27.json'
assert Path('/media/zhuyupei/SanStand').is_mount()
assert json.loads((R / 'manifest.json').read_text())['recorder_closed']
audit = json.loads(A.read_text())
source = audit['selected']['observation_source_audit']
ns = source['source_stamp_ns']
targets = {ns, 1792033654136828184}
def tm(x):
    return rospy.Time(x // 10**9, x % 10**9)
topics = ['/supervisor/camera/color/image_raw', '/supervisor/camera/depth/image_raw', '/perception/object_mask']
frames = {key: {} for key in targets}
buf = tf2_py.BufferCore(rospy.Duration(600.))
cams = []
with rosbag.Bag(str(R / 'keyframes.bag')) as bag:
    for topic, msg, received in bag.read_messages(topics=topics, start_time=tm(min(targets)-100000000), end_time=tm(max(targets)+3000000000)):
        stamp = msg.header.stamp.to_nsec()
        if stamp in targets:
            frames[stamp][topic] = (msg, received.to_nsec())
    for topic, msg, received in bag.read_messages(topics=['/tf_static', '/grasp_record/camera_runtime_metadata']):
        if topic == '/tf_static':
            for tr in msg.transforms:
                buf.set_transform_static(tr, 'sealed_gen26_box_design')
        else:
            data = json.loads(msg.data)
            if data.get('parameters'):
                cams.append((received.to_nsec(), data['parameters']))
    for _, msg, _ in bag.read_messages(topics=['/tf'], start_time=tm(min(targets)-1000000000), end_time=tm(max(targets)+1000000000)):
        for tr in msg.transforms:
            buf.set_transform(tr, 'sealed_gen26_box_design')
OUT.mkdir(parents=True, exist_ok=True)
bridge = CvBridge()
rows = []
for stamp, pairs in sorted(frames.items()):
    row = dict(source_stamp_ns=stamp, available_topics=sorted(pairs))
    if set(pairs) != set(topics):
        rows.append(row)
        continue
    color = bridge.imgmsg_to_cv2(pairs[topics[0]][0], 'bgr8')
    depth = bridge.imgmsg_to_cv2(pairs[topics[1]][0], 'passthrough')
    mask = bridge.imgmsg_to_cv2(pairs[topics[2]][0], 'mono8')
    camera = max((v for v in cams if v[0] <= pairs[topics[0]][1]), default=cams[0], key=lambda v: v[0])[1]
    tr = buf.lookup_transform_core('base_link', pairs[topics[1]][0].header.frame_id, tm(stamp)).transform
    T = quaternion_matrix([tr.rotation.x, tr.rotation.y, tr.rotation.z, tr.rotation.w])
    T[:3, 3] = [tr.translation.x, tr.translation.y, tr.translation.z]
    arrays = dict(color=color, depth_raw=depth, object_mask=mask, T_base_camera=T,
                  intrinsics=np.array([camera[k] for k in ('fx', 'fy', 'cx', 'cy')]),
                  depth_scale_m=np.array(camera['depth_scale']), source_stamp_ns=np.array(stamp))
    path = OUT / ('source_%d.npz' % stamp)
    assert not path.exists(), path
    np.savez_compressed(str(path), **arrays)
    cv2.imwrite(str(OUT / ('source_%d_color.png' % stamp)), color)
    cv2.imwrite(str(OUT / ('source_%d_mask.png' % stamp)), mask)
    row.update(npz=str(path), sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
               array_sha256={k: hashlib.sha256(np.ascontiguousarray(v).tobytes()).hexdigest() for k, v in arrays.items()},
               camera=camera, received_ns={k: v[1] for k, v in pairs.items()})
    if stamp == ns:
        row['original_depth_hash_matches'] = row['array_sha256']['depth_raw'] == source['raw_depth_sha256']
        row['original_mask_hash_matches'] = row['array_sha256']['object_mask'] == source['raw_mask_sha256']
        assert row['original_depth_hash_matches'] and row['original_mask_hash_matches']
    rows.append(row)
report = dict(record=str(R), audit=str(A), audit_sha256=hashlib.sha256(A.read_bytes()).hexdigest(),
              rows=rows, no_ros_calls=True, no_hardware_access=True, no_temperature_reads=True,
              source_geometry=audit['replay_geometry'], source_audit=source)
path = OUT / 'source_manifest.json'
assert not path.exists(), path
path.write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
print(json.dumps(dict(manifest=str(path), sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                     frames=[{k: v for k, v in row.items() if k != 'camera'} for row in rows]), ensure_ascii=False))
