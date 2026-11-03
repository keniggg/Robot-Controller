"""Export exact historical initial-surface evidence, offline and without new authority."""
import io, json, hashlib
from pathlib import Path
import numpy as np
import rosbag, rospy, tf2_py
from cv_bridge import CvBridge
from tf.transformations import quaternion_matrix

N = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_contact_admission_20261013T011103Z')
F = N/'verification/gen14_arrival_independent'
R = N.parents[1]/'grasp_records/20261013T025604.344486Z_af3ecb9373264cc9b19a98143f7b5df5'
O = F/'initial_surface_fixture'
NS = 1791860167684837341


def sha(p):
    h = hashlib.sha256()
    with Path(p).open('rb') as stream:
        for block in iter(lambda: stream.read(8*1024*1024), b''):
            h.update(block)
    return h.hexdigest()


def plain(x):
    if hasattr(x, 'to_nsec'): return x.to_nsec()
    if hasattr(x, '__slots__'): return {key: plain(getattr(x, key)) for key in x.__slots__}
    if isinstance(x, (tuple, list)): return [plain(v) for v in x]
    return x


def main():
    assert Path('/media/zhuyupei/SanStand').is_mount()
    O.mkdir(exist_ok=False)
    f = json.loads((F/'fixture.json').read_text())
    manifest = json.loads((R/'manifest.json').read_text())
    assert manifest['recorder_closed'] and manifest['result'] == 'failure'
    expected = next(row['sha256'] for row in manifest['files'] if row['path'] == 'keyframes.bag')
    assert sha(R/'keyframes.bag') == expected == f['record_bag_sha256']
    bridge = CvBridge(); selected = {}; static = []; dynamic = []; sdk = []; geometry = []
    wanted = {'/supervisor/camera/color/image_raw': 'rgb', '/supervisor/camera/depth/image_raw': 'depth',
              '/perception/object_mask': 'mask', '/perception/object': 'object'}
    topics = list(wanted) + ['/perception/unknown/detector_status', '/tf', '/tf_static',
        '/alicia_d/accepted_joint_states', '/grasp_6d/object_geometry', '/experiment/observation_rgbd/metadata']
    with rosbag.Bag(str(R/'keyframes.bag')) as bag:
        for topic, msg, received in bag.read_messages(topics=topics):
            row = (topic, msg, received.to_nsec())
            if topic in wanted and msg.header.stamp.to_nsec() == NS:
                selected.setdefault(wanted[topic], []).append(row)
            elif topic == '/perception/unknown/detector_status':
                payload = json.loads(msg.data)
                if payload.get('source_stamp_ns') == NS:
                    selected.setdefault('status', []).append(row)
            elif topic == '/tf_static': static.append(row)
            elif topic == '/tf': dynamic.append(row)
            elif topic == '/alicia_d/accepted_joint_states': sdk.append(row)
            elif topic == '/grasp_6d/object_geometry':
                geometry.append(dict(received_ns=received.to_nsec(), header=plain(msg.header)))
                if msg.header.stamp.to_nsec() == NS:
                    selected.setdefault('object_geometry', []).append(row)
            elif topic == '/experiment/observation_rgbd/metadata':
                payload = json.loads(msg.data)
                if any(str(NS) == str(v) for v in payload.values()):
                    selected.setdefault('observation_metadata', []).append(row)
    assert all(name in selected for name in ('rgb', 'depth', 'mask', 'object', 'status')), list(selected)
    cutoff = max(row[2] for rows in selected.values() for row in rows)
    # Preserve the actual availability cutoff and exact bracketing evidence.
    usable_sdk = [row for row in sdk if row[2] <= cutoff]
    before = max((row for row in usable_sdk if row[1].header.stamp.to_nsec() <= NS), key=lambda row: row[1].header.stamp.to_nsec())
    after = min((row for row in usable_sdk if row[1].header.stamp.to_nsec() >= NS), key=lambda row: row[1].header.stamp.to_nsec())
    selected['sdk_before'] = [before]; selected['sdk_after'] = [after]
    dynamic_edges = {}
    for row in dynamic:
        if row[2] <= cutoff:
            for tr in row[1].transforms:
                dynamic_edges.setdefault((tr.header.frame_id, tr.child_frame_id), []).append((tr.header.stamp.to_nsec(), row))
    bracket_audit = []
    for edge, transforms in dynamic_edges.items():
        left = [item for item in transforms if item[0] <= NS]
        right = [item for item in transforms if item[0] >= NS]
        if left and right:
            lo, hi = max(left, key=lambda item: item[0]), min(right, key=lambda item: item[0])
            selected.setdefault('tf_bracket', []).extend([lo[1], hi[1]])
            bracket_audit.append(dict(parent=edge[0], child=edge[1], before_stamp_ns=lo[0], after_stamp_ns=hi[0],
                                      before_received_ns=lo[1][2], after_received_ns=hi[1][2]))
    selected['tf_static'] = [row for row in static if row[2] <= cutoff]
    assert selected['tf_static'] and bracket_audit
    buffer = tf2_py.BufferCore(rospy.Duration(600))
    for row in selected['tf_static']:
        for tr in row[1].transforms: buffer.set_transform_static(tr, 'sealed_original')
    # Feed each original TF message once, preserving receive ordering.
    unique = {row[2]: row for row in selected['tf_bracket']}
    selected['tf_bracket'] = sorted(unique.values(), key=lambda row: row[2])
    for row in selected['tf_bracket']:
        for tr in row[1].transforms: buffer.set_transform(tr, 'sealed_original')
    transform = buffer.lookup_transform_core('base_link', 'camera_link', rospy.Time(NS//10**9, NS%10**9))
    q = transform.transform.rotation; t = transform.transform.translation
    T = quaternion_matrix([q.x, q.y, q.z, q.w]); T[:3, 3] = [t.x, t.y, t.z]
    rgb = bridge.imgmsg_to_cv2(selected['rgb'][0][1], 'bgr8').copy()
    raw = bridge.imgmsg_to_cv2(selected['depth'][0][1], 'passthrough').copy()
    mask = bridge.imgmsg_to_cv2(selected['mask'][0][1], 'passthrough').copy()
    encoding = selected['depth'][0][1].encoding
    assert encoding in ('16UC1', 'mono16', '32FC1')
    depth = raw.astype(np.float32) * f['camera']['depth_scale'] if encoding != '32FC1' else raw.astype(np.float32)
    assert rgb.shape[:2] == raw.shape == mask.shape
    npz = O/f'{NS}_source.npz'
    np.savez_compressed(npz, rgb=rgb, raw_depth=raw, depth=depth, recorded_mask=mask, T_base_camera=T)
    rows = []
    for kind, messages in selected.items():
        for index, (topic, msg, received) in enumerate(messages):
            stream = io.BytesIO(); msg.serialize(stream); data = stream.getvalue()
            path = O/f'{kind}_{index}_{received}.rosmsg'; path.write_bytes(data)
            rows.append(dict(kind=kind, topic=topic, received_ns=received, file=str(path),
                message_type=msg._type, md5sum=msg._md5sum, sha256=hashlib.sha256(data).hexdigest()))
    status = json.loads(selected['status'][0][1].data)
    (O/'status.json').write_text(json.dumps(status, indent=2)+'\n')
    (O/'object.json').write_text(json.dumps(plain(selected['object'][0][1]), indent=2)+'\n')
    report = dict(scope='Exact historical initial surface RGBD, published mask/object/status and available source TF/SDK brackets only; no arrival qualification seeded',
        source_ns=NS, file=str(npz), sha256=sha(npz), rows=rows, camera=f['camera'],
        depth_encoding=encoding, depth_scale=f['camera']['depth_scale'], source_bag=str(R/'keyframes.bag'),
        source_bag_sha256=expected, original_evidence_receive_cutoff_ns=cutoff, tf_brackets=bracket_audit,
        sdk_bracket=[dict(stamp_ns=r[1].header.stamp.to_nsec(), received_ns=r[2], names=list(r[1].name), positions=list(r[1].position)) for r in (before, after)],
        exact_source_tf=plain(transform), T_base_camera=T.tolist(), original_geometry_headers=geometry,
        exact_same_header_object_geometry_available='object_geometry' in selected,
        no_original_pointcloud_sha_claimed=True, no_ros_master=True, motion_commands_sent=False,
        source_arrays_sha256={key: hashlib.sha256(np.ascontiguousarray(a).tobytes()).hexdigest() for key, a in dict(rgb=rgb, raw_depth=raw, recorded_mask=mask, T_base_camera=T).items()},
        script_sha256=sha(__file__))
    (O/'manifest.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(dict(file=str(O/'manifest.json'), sha256=sha(O/'manifest.json'), source_ns=NS,
                         rows=len(rows), kind_counts={k:len(v)for k,v in selected.items()}, camera_origin=T[:3,3].tolist())))


if __name__ == '__main__': main()
