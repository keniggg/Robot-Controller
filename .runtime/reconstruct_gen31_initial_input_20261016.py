"""Offline reconstruction from the sealed initial RGB-D frame and recorded TF.

This is a new single-frame geometry calculation, not a claim to reproduce the
unretained request-1 temporal snapshot or its complete historical box audit.
"""
import hashlib
import json
import os
from pathlib import Path
from types import SimpleNamespace as NS
import sys

W = Path(__file__).resolve().parents[1]
M = Path('/media/zhuyupei/SanStand')
E = M/'实验记录/Robot-Controller'
C = E/'ros_log/full_latest_gen29_restart_20261016T000728Z'
R = E/'grasp_records/20261016T021226.502516Z_0494c8bc84a34a74933e8eae122dd1f4'
I = C/'verification/gen30_attempt1_closed_evidence_r1'
O = C/'verification/gen31_initial_source_geometry_r1'
STAMP = 1792116749240888833
assert M.is_mount() and not os.statvfs(M).f_flag & os.ST_RDONLY
assert not O.exists()
os.environ.update(ROS_MASTER_URI='http://127.0.0.1:1', PYTHONDONTWRITEBYTECODE='1')
sys.path[:0] = [str(W/'src/alicia_flexible_grasp_supervisor/src'),
               str(W/'devel/lib/python3/dist-packages')]

import numpy as np
import yaml
import rospy
import rosbag
import tf2_py
from tf.transformations import quaternion_matrix
from alicia_flexible_grasp.vision.rgbd_snapshot import RgbdSample, fuse_stable_samples
from alicia_flexible_grasp.vision.object_geometry import estimate_object_geometry
from alicia_flexible_grasp.vision.target_observation import TargetTrackIdentity


def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(8*1024*1024), b''):
            h.update(block)
    return h.hexdigest()


def read(path):
    return json.loads(Path(path).read_text())


def serial(v):
    if isinstance(v, np.ndarray): return v.tolist()
    if isinstance(v, np.generic): return v.item()
    raise TypeError(type(v).__name__)


manifest = read(R/'manifest.json')
assert manifest['state'] == 'complete' and manifest['recorder_closed'] is True
assert read(R/'recorder_outcome.json')['bag_closed'] is True
bag = R/'keyframes.bag'
entry = next(row for row in manifest['files'] if row['path'] == bag.name)
assert bag.stat().st_size == entry['size_bytes'] and sha(bag) == entry['sha256']
before_stat = bag.stat()
audit_path = R/'keyframes_audits/3dbb02254dd2076aeb040b41d269b98f83f22d2ac1d495ec9ccec2008d738667.json'
trigger = read(audit_path)['observe_return']['trigger_evidence']
assert trigger['request_id'] == 1 and trigger['snapshot_stamp_ns'] == STAMP
identity = TargetTrackIdentity(trigger['target_epoch'], trigger['target_track_id'])
buffer = tf2_py.BufferCore(rospy.Duration(1000.))
rows = []
obj = None
tf_counts = dict(dynamic=0, static=0)
with rosbag.Bag(str(bag)) as stream:
    for topic, msg, received in stream.read_messages(topics=[
            '/tf', '/tf_static', '/alicia_d/accepted_joint_states', '/perception/object']):
        if topic in ('/tf', '/tf_static'):
            for transform in msg.transforms:
                if topic == '/tf_static':
                    buffer.set_transform_static(transform, 'sealed_record')
                    tf_counts['static'] += 1
                else:
                    buffer.set_transform(transform, 'sealed_record')
                    tf_counts['dynamic'] += 1
        elif topic == '/alicia_d/accepted_joint_states' and msg.header.frame_id == 'sdk_measured':
            ns = msg.header.stamp.to_nsec()
            if abs(ns-STAMP) < 2_000_000_000:
                q = dict(zip(msg.name, msg.position))
                rows.append(dict(stamp_ns=ns, received_ns=received.to_nsec(),
                    positions_rad=[q['Joint%d' % i] for i in range(1, 7)]))
        elif topic == '/perception/object' and msg.header.stamp.to_nsec() == STAMP:
            assert obj is None
            obj = msg
assert obj is not None and obj.detected is True
before = max((r for r in rows if r['stamp_ns'] <= STAMP), key=lambda r:r['stamp_ns'])
after = min((r for r in rows if r['stamp_ns'] >= STAMP), key=lambda r:r['stamp_ns'])
assert STAMP-before['stamp_ns'] < 200_000_000 and after['stamp_ns']-STAMP < 200_000_000
np.testing.assert_array_equal(before['positions_rad'], after['positions_rad'])
tf = buffer.lookup_transform_core('base_link', 'camera_link', rospy.Time(*divmod(STAMP, 10**9)))
t, q = tf.transform.translation, tf.transform.rotation
T_link = quaternion_matrix([q.x, q.y, q.z, q.w]); T_link[:3, 3] = [t.x, t.y, t.z]
optical = np.eye(4); optical[:3,:3] = [[0.,0.,1.],[-1.,0.,0.],[0.,-1.,0.]]
T = T_link @ optical
raw_path = I/f'{STAMP}_raw.npz'
with np.load(raw_path, allow_pickle=False) as z:
    color, depth, mask = (z[k].copy() for k in ('color_bgr','depth_raw','object_mask'))
cameras = [v['metadata']['parameters'] for v in read(I/'status_and_camera.json')['camera_metadata']
           if 'parameters' in v['metadata']]
keys = ('fx','fy','cx','cy','width','height','depth_scale')
assert len({tuple(v[k] for k in keys) for v in cameras}) == 1
cam = cameras[0]
K = NS(**{k:cam[k] for k in keys if k != 'depth_scale'}); scale = cam['depth_scale']
bbox = (obj.bbox_x,obj.bbox_y,obj.bbox_width,obj.bbox_height)
sample = RgbdSample(color,depth,mask,bbox,obj,STAMP/1e9,'camera_link',
    np.asarray(before['positions_rad']),STAMP,identity.epoch,identity)
cfg_path = W/'src/alicia_flexible_grasp_supervisor/config/grasp_params.yaml'
camera_path = W/'src/alicia_flexible_grasp_supervisor/config/camera.yaml'
cfg = yaml.safe_load(cfg_path.read_text())['grasp_6d']['remote']
pcfg = yaml.safe_load(camera_path.read_text())['perception']
snapshot = fuse_stable_samples([sample], require_mask=True,
    min_mask_iou=cfg['planning_mask_min_iou'], max_centroid_shift_px=cfg['planning_mask_max_centroid_shift_px'],
    max_joint_delta_rad=cfg['planning_max_joint_delta_rad'], erosion_px=cfg['mask_erosion_px'],
    depth_scale=scale, depth_min_m=pcfg['depth_min_m'], depth_max_m=pcfg['depth_max_m'],
    mad_scale=cfg['depth_mad_scale'], mad_absolute_floor_m=cfg['depth_mad_absolute_floor_m'],
    internal_hole_max_area_px=0)
assert snapshot.ok, (snapshot.failure_code,snapshot.failure_reason)
gg = cfg['gripper_geometry']
kwargs = dict(support_bbox_expand_ratio=cfg.get('support_bbox_expand_ratio',.30),
    support_distance_threshold_m=cfg.get('support_distance_threshold_m',cfg.get('target_cloud_support_plane_inlier_distance_m',.004)),
    voxel_size_m=cfg['target_cloud_voxel_size_m'], min_support_points=cfg.get('geometry_min_support_points',200),
    min_object_points=cfg.get('geometry_min_object_points',cfg.get('target_cloud_min_points',120)),
    min_size_m=cfg.get('geometry_min_size_m',.005), max_size_m=cfg.get('geometry_max_size_m',.600),
    max_height_m=cfg.get('geometry_max_height_m',.500), previous_axes_base=None,
    outlier_neighbors=cfg['target_cloud_outlier_neighbors'],outlier_std_ratio=cfg['target_cloud_outlier_std_ratio'],
    support_minimum_span_m=max(gg['finger_box_xyz_m'][0],gg['max_inner_gap_m']+2*gg['finger_box_xyz_m'][1])+2*gg.get('support_clearance_m',.003))
geometry = estimate_object_geometry(snapshot.depth_raw,snapshot.target_depth_raw,snapshot.object_mask,
    snapshot.bbox,K,scale,T,snapshot.source_mode,**kwargs)
assert geometry.ok,(geometry.failure_code,geometry.failure_reason)
g = dict(obb_center_base_m=geometry.center_base,R_base_obb=geometry.axes_base,
    obb_size_xyz_m=geometry.size_xyz_m,support_normal_base=geometry.support_normal_base,
    support_offset_m=geometry.support_offset_m,support_inlier_ratio=geometry.support_inlier_ratio,
    object_points_base_m=geometry.object_points_base,source_mode=geometry.source_mode)
inputs = {str(p):sha(p) for p in (R/'manifest.json',audit_path,raw_path,I/'status_and_camera.json',cfg_path,camera_path,Path(__file__))}
inputs[str(bag)] = entry['sha256']
assert bag.stat().st_size == before_stat.st_size and bag.stat().st_mtime_ns == before_stat.st_mtime_ns
assert M.is_mount()
O.mkdir()
result = dict(passed=True,kind='offline_new_single_frame_geometry_from_original_initial_pixels',
    source_stamp_ns=STAMP,source_identity=dict(epoch=identity.epoch,track_id=identity.track_id),
    original_request_id=1,historical_full_request_reproduced=False,temporal_snapshot_reconstructed=False,
    cross_view_fusion_used=False,calibration_metadata_image_synchronized=False,
    limitations=['Original complete request-1 temporal snapshot/box audit was not retained.',
                 'Recorded camera parameters are stable across metadata samples, not image-synchronized.'],
    inputs_sha256=inputs,recorded_tf_lookup_stamp_ns=STAMP,tf_counts=tf_counts,
    accepted_joint_bracket=dict(before=before,after=after),bbox_xywh=bbox,
    camera_parameters={k:cam[k] for k in keys},geometry_kwargs=kwargs,
    snapshot_transform=dict(T_base_camera_link=T_link,T_base_optical=T),replay_geometry=g,
    live_ros_contacted=False,motion_commands_sent=False,temperature_queried=False)
with (O/'result.json').open('x') as f:json.dump(result,f,ensure_ascii=False,indent=2,default=serial);f.write('\n')
print(json.dumps(dict(result=str(O/'result.json'),sha256=sha(O/'result.json'),
    size_xyz_m=geometry.size_xyz_m.tolist(),point_count=len(geometry.object_points_base)),ensure_ascii=False))
