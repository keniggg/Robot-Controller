"""Bounded, offline sealed-bag image export. No ROS node/service/device use."""
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

W = Path(__file__).resolve().parents[1]
E = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
R = E / 'grasp_records/20261015T111226.886093Z_0a4de2be16394344b0fa84a6ad2f8318'
OUT = E / 'ros_log/full_latest_gen22_restart_20261015T003700Z/verification/gen29_attempt1_shadow_frames'
INITIAL = [1792062749691942453, 1792062749893290758, 1792062750092055082]
SHADOW = [
    (1792062835164456367, 'second_station_qualified_source'),
    (1792062835497817993, 'first_tracking_after_second_station'),
    (1792062849966778516, 'before_camera_gap'),
    (1792062850569185018, 'last_good_source'),
    (1792062851088963747, 'first_identity_recovery_failure'),
    (1792062851287487983, 'first_frame_after_permanent_loss'),
]
TOPICS = ['/supervisor/camera/color/image_raw', '/supervisor/camera/depth/image_raw',
          '/perception/object_mask']


def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b''):
            h.update(chunk)
    return h.hexdigest()


def tm(ns):
    return rospy.Time(ns // 10**9, ns % 10**9)


def write_json(path, data):
    assert not path.exists(), path
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False) + '\n')


def array_hash(a):
    return hashlib.sha256(np.ascontiguousarray(a).tobytes()).hexdigest()


def depth_stats(depth, region, scale):
    n = int(np.count_nonzero(region))
    valid = np.isfinite(depth) & (depth > 0)
    values = depth[region & valid].astype(float) * scale
    return dict(region_pixels=n, valid_depth_pixels=int(np.count_nonzero(region & valid)),
        zero_depth_pixels=int(np.count_nonzero(region & (depth == 0))),
        valid_depth_fraction=(float(np.count_nonzero(region & valid)) / n if n else None),
        depth_m_quantiles=(np.quantile(values, [0, .05, .5, .95, 1]).tolist() if len(values) else None),
        valid_definition='finite raw depth > 0; no mask-exterior or GUI padding included')


assert Path('/media/zhuyupei/SanStand').is_mount()
assert not OUT.exists(), OUT
manifest = json.loads((R / 'manifest.json').read_text())
closed = json.loads((R / 'recorder_outcome.json').read_text())
assert manifest['state'] == 'complete' and manifest['recorder_closed'] is True
assert closed['bag_closed'] is True and closed['interrupted'] is False
bag_path = R / 'keyframes.bag'
entry = next(v for v in manifest['files'] if v['path'] == 'keyframes.bag')
before = bag_path.stat()
assert before.st_size == entry['size_bytes']
bag_hash = sha(bag_path)
assert bag_hash == entry['sha256']

targets = set(INITIAL + [x[0] for x in SHADOW])
frames = {stamp: {} for stamp in targets}
statuses, objects, cameras, accepted = {}, {}, [], []
status_transitions = []
buf = tf2_py.BufferCore(rospy.Duration(600.))
with rosbag.Bag(str(bag_path), 'r') as bag:
    for topic, msg, received in bag.read_messages(topics=['/tf_static', '/grasp_record/camera_runtime_metadata']):
        if topic == '/tf_static':
            for tr in msg.transforms:
                buf.set_transform_static(tr, 'sealed_gen29_image_export')
        else:
            payload = json.loads(msg.data)
            if payload.get('parameters'):
                cameras.append((received.to_nsec(), payload['parameters']))
    windows = [(min(INITIAL)-10**9, max(INITIAL)+2*10**9),
               (SHADOW[0][0]-10**9, SHADOW[-1][0]+2*10**9)]
    seen = set()
    for begin, end in windows:
        topics = TOPICS + ['/perception/object', '/perception/unknown/detector_status',
                           '/alicia_d/accepted_joint_states', '/tf']
        for topic, msg, received in bag.read_messages(topics=topics, start_time=tm(begin), end_time=tm(end)):
            if topic == '/tf':
                for tr in msg.transforms:
                    buf.set_transform(tr, 'sealed_gen29_image_export')
                continue
            if topic == '/perception/unknown/detector_status':
                d = json.loads(msg.data)
                stamp = int(d.get('source_stamp_ns') or 0)
                if stamp in targets:
                    statuses[stamp] = dict(payload=d, bag_received_ns=received.to_nsec())
                key = (d.get('state'), d.get('detail'))
                if begin > 1792062800000000000 and (not status_transitions or key != tuple(status_transitions[-1]['key'])):
                    status_transitions.append(dict(key=key, bag_received_ns=received.to_nsec(),
                        source_stamp_ns=stamp, first_loss=d.get('first_loss')))
                continue
            stamp = msg.header.stamp.to_nsec()
            if topic == '/alicia_d/accepted_joint_states':
                if stamp >= SHADOW[0][0]:
                    row = dict(source_stamp_ns=stamp, names=list(msg.name), positions=list(msg.position))
                    if row not in accepted:
                        accepted.append(row)
                continue
            if stamp not in targets:
                continue
            key = (topic, stamp)
            if key in seen:
                raise ValueError('duplicate exact selected source: %r' % (key,))
            seen.add(key)
            if topic == '/perception/object':
                objects[stamp] = dict(detected=bool(msg.detected), frame_id=msg.header.frame_id,
                    bbox_xywh=[int(getattr(msg, k)) for k in ('bbox_x','bbox_y','bbox_width','bbox_height')],
                    bag_received_ns=received.to_nsec())
            else:
                frames[stamp][topic] = (msg, received.to_nsec())

assert cameras
bridge = CvBridge()
decoded = {}
for stamp, channels in frames.items():
    if set(channels) != set(TOPICS):
        continue
    decoded[stamp] = (
        np.array(bridge.imgmsg_to_cv2(channels[TOPICS[0]][0], 'bgr8'), copy=True),
        np.array(bridge.imgmsg_to_cv2(channels[TOPICS[1]][0], 'passthrough'), copy=True),
        np.array(bridge.imgmsg_to_cv2(channels[TOPICS[2]][0], 'mono8'), copy=True))
assert all(stamp in decoded for stamp, _ in SHADOW), sorted(set(targets)-set(decoded))
reference_stamp = 1792062850569185018
reference_mask = decoded[reference_stamp][2] > 0
assert np.any(reference_mask)
union_mask = np.logical_or.reduce([decoded[stamp][2] > 0 for stamp, _ in SHADOW])
ys, xs = np.nonzero(union_mask)
h, w = union_mask.shape
x0, x1 = max(0, int(xs.min())-24), min(w, int(xs.max())+25)
y0, y1 = max(0, int(ys.min())-24), min(h, int(ys.max())+25)
OUT.mkdir(parents=True)
rows, panels = [], []
for stamp in sorted(targets):
    role = next((label for ns, label in SHADOW if ns == stamp), 'initial_request1_source_export_only')
    row = dict(source_stamp_ns=stamp, role=role, available_topics=sorted(frames[stamp]))
    if stamp not in decoded:
        row['exported'] = False
        rows.append(row)
        continue
    color, depth, mask = decoded[stamp]
    assert color.shape[:2] == depth.shape == mask.shape and depth.dtype == np.uint16
    assert set(np.unique(mask)).issubset({0, 255})
    channels = frames[stamp]
    camera = max((v for v in cameras if v[0] <= channels[TOPICS[0]][1]),
                 default=cameras[0], key=lambda v: v[0])[1]
    scale = float(camera['depth_scale'])
    tr = buf.lookup_transform_core('base_link', channels[TOPICS[1]][0].header.frame_id, tm(stamp)).transform
    T = quaternion_matrix([tr.rotation.x, tr.rotation.y, tr.rotation.z, tr.rotation.w])
    T[:3, 3] = [tr.translation.x, tr.translation.y, tr.translation.z]
    folder = OUT / ('source_%d' % stamp)
    folder.mkdir()
    arrays = dict(color_bgr=color, depth_raw=depth, object_mask=mask, T_base_camera_link=T,
        source_stamp_ns=np.array(stamp, dtype=np.int64), depth_scale_m=np.array(scale),
        intrinsics=np.array([camera[k] for k in ('fx', 'fy', 'cx', 'cy')]))
    np.savez_compressed(str(folder/'raw_arrays.npz'), **arrays)
    assert cv2.imwrite(str(folder/'color.png'), color)
    assert cv2.imwrite(str(folder/'mask.png'), mask)
    assert cv2.imwrite(str(folder/'depth_raw_uint16.png'), depth)
    valid = depth > 0
    depth_m = depth.astype(float) * scale
    # Data visualization only, fixed range across all selected sources.
    mapped = (255.*np.clip((depth_m-.10)/.40, 0., 1.)).astype(np.uint8)
    depth_display = cv2.applyColorMap(mapped, cv2.COLORMAP_TURBO)
    depth_display[~valid] = 0
    assert cv2.imwrite(str(folder/'depth_display_0p10_to_0p50m.png'), depth_display)
    region = mask > 0
    n = int(np.count_nonzero(region))
    yy, xx = np.nonzero(region)
    bbox_area = (int(xx.max()-xx.min()+1)*int(yy.max()-yy.min()+1)) if n else 0
    row.update(exported=True, image_shape_hw=list(depth.shape), object=objects.get(stamp),
        mask_pixels=n, mask_fraction_of_sensor_image=n/mask.size,
        mask_fraction_of_own_bbox=(n/bbox_area if bbox_area else None),
        target_mask_depth=depth_stats(depth, region, scale),
        camera_parameters=camera,
        topic_headers={t: dict(frame_id=m.header.frame_id, stamp_ns=m.header.stamp.to_nsec(),
                              encoding=m.encoding, bag_received_ns=received)
                       for t, (m, received) in channels.items()},
        array_sha256={k: array_hash(v) for k,v in arrays.items()},
        detector_status_file=None, folder=str(folder))
    if stamp in statuses:
        write_json(folder/'detector_status.json', statuses[stamp])
        row['detector_status_file'] = str(folder/'detector_status.json')
        d = statuses[stamp]['payload']
        row['detector'] = {k:d.get(k) for k in ('state','detail','source_stamp_ns')}
    if role != 'initial_request1_source_export_only':
        row['fixed_last_good_pixel_region_diagnostic_only'] = dict(
            reference_source_stamp_ns=reference_stamp, reference_mask_sha256=array_hash(decoded[reference_stamp][2]),
            current_target_geometry_authorized=False, statistics=depth_stats(depth, reference_mask, scale),
            assumption='same sensor pixels only; not a recovered current target mask')
        crops = [color[y0:y1,x0:x1], cv2.cvtColor(mask[y0:y1,x0:x1], cv2.COLOR_GRAY2BGR),
                 depth_display[y0:y1,x0:x1]]
        strip = np.hstack([cv2.resize(c, None, fx=2, fy=2, interpolation=cv2.INTER_NEAREST) for c in crops])
        top = np.full((52,strip.shape[1],3), 248, dtype=np.uint8)
        cv2.putText(top, '%d  %s' % (stamp,role), (5,19), cv2.FONT_HERSHEY_SIMPLEX,.45,(0,0,0),1)
        frac=row['target_mask_depth']['valid_depth_fraction']
        caption='RGB | published mask | raw depth display (black = zero); mask px=%d valid=%s' % (n, 'N/A (empty mask)' if frac is None else '%.2f%%'%(100*frac))
        cv2.putText(top, caption, (5,40), cv2.FONT_HERSHEY_SIMPLEX,.40,(0,0,0),1)
        panel=np.vstack([top,strip]);assert cv2.imwrite(str(folder/'review_crop.png'),panel);panels.append(panel)
    row['files_sha256']={str(p):sha(p) for p in folder.iterdir() if p.is_file()}
    write_json(folder/'metadata.json', row)
    rows.append(row)
assert len(panels)==6
assert cv2.imwrite(str(OUT/'shadow_comparison.png'),np.vstack(panels))
after = bag_path.stat()
assert (before.st_size,before.st_mtime_ns,before.st_ino)==(after.st_size,after.st_mtime_ns,after.st_ino)
names=accepted[0]['names'] if accepted else []
same_names=all(v['names']==names for v in accepted)
qspan=(np.ptp(np.asarray([v['positions'] for v in accepted]),axis=0).tolist() if accepted and same_names else None)
report=dict(record=str(R), source_bag=str(bag_path), source_bag_sha256=bag_hash,
    source_bag_size_bytes=before.st_size, source_bag_stat_unchanged=True,
    record_closed_verified=True, manifest_sha256=sha(R/'manifest.json'),
    recorder_outcome_sha256=sha(R/'recorder_outcome.json'),
    exporter=str(Path(__file__).resolve()), exporter_sha256=sha(__file__),
    live_ros_contacted=False, motion_commands_sent=False, production_modified=False,
    qualification_replayed=False, physical_grasp_verified=False,
    exact_stamp_matching=True, shadow_frames=6, initial_requested=INITIAL,
    initial_missing=[s for s in INITIAL if s not in decoded],
    depth_display=dict(range_m=[.10,.50], black='raw depth exactly zero',
        sensor_image_dimensions='no GUI letterbox or rescaling used for statistics'),
    crop_xyxy=[x0,y0,x1,y1], status_transitions=status_transitions,
    second_station_accepted_joints=dict(sample_count=len(accepted), names=names,
        first_stamp_ns=accepted[0]['source_stamp_ns'] if accepted else None,
        last_stamp_ns=accepted[-1]['source_stamp_ns'] if accepted else None,
        peak_to_peak_rad=qspan), rows=rows,
    comparison_image=str(OUT/'shadow_comparison.png'),
    comparison_sha256=sha(OUT/'shadow_comparison.png'))
write_json(OUT/'result.json', report)
print(json.dumps(dict(result=str(OUT/'result.json'),sha256=sha(OUT/'result.json'),
    initial_missing=report['initial_missing'], rows=[dict(source_stamp_ns=x['source_stamp_ns'],
      role=x['role'], exported=x['exported'], mask_pixels=x.get('mask_pixels'),
      valid_fraction=x.get('target_mask_depth',{}).get('valid_depth_fraction'),
      fixed_region_fraction=x.get('fixed_last_good_pixel_region_diagnostic_only',{}).get('statistics',{}).get('valid_depth_fraction')) for x in rows]),ensure_ascii=False))
