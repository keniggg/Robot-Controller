#!/usr/bin/env python3
"""Rebuild exact reached-view samples and registration from frozen ROS bags.

Offline only: no ROS node, publishers, services, device access, or live TF.
Requires the ROS Python message environment for deserializing the recordings.
"""

import argparse
import dataclasses
import json
from pathlib import Path
import sys

import numpy as np
import rosbag
import rospy
import tf2_py
from cv_bridge import CvBridge
from tf.transformations import quaternion_matrix

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/alicia_flexible_grasp_supervisor/src'))
from alicia_flexible_grasp.vision.grasp6d_adapter import CameraIntrinsics
from alicia_flexible_grasp.vision.object_geometry import estimate_object_geometry
from alicia_flexible_grasp.vision.rgbd_snapshot import RgbdSample, fuse_stable_samples
from alicia_flexible_grasp.vision.target_observation import (
    TargetTrackIdentity, observation_from_snapshot,
)
from alicia_flexible_grasp.vision import multiview_surface as surface


COLOR = '/supervisor/camera/color/image_raw'
DEPTH = '/supervisor/camera/depth/image_raw'
MASK = '/perception/object_mask'
OBJECT = '/perception/object'


def plain(value):
    if dataclasses.is_dataclass(value):
        return {field.name: plain(getattr(value, field.name))
                for field in dataclasses.fields(value)}
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, dict):
        return {k: plain(v) for k, v in value.items()}
    if isinstance(value, (tuple, list)):
        return [plain(v) for v in value]
    return value


def read_sources(bag_directory, wanted):
    frames = {stamp: {} for stamp in wanted}
    joints = {stamp: None for stamp in wanted}
    buffer = tf2_py.BufferCore(rospy.Duration(600.0))
    source = Path(bag_directory)
    def volume_key(path):
        suffix = path.stem.rsplit('_', 1)[-1]
        return (int(suffix) if suffix.isdigit() else -1, path.name)
    files = ([source] if source.is_file() else
             sorted(source.glob('*.bag'), key=volume_key))
    if not files:
        raise ValueError('no recorded bag files in %s' % source)
    topics = [COLOR, DEPTH, MASK, OBJECT, '/joint_states', '/tf', '/tf_static']
    for path in files:
        with rosbag.Bag(str(path)) as bag:
            for topic, message, _received in bag.read_messages(topics=topics):
                if topic in ('/tf', '/tf_static'):
                    setter = (buffer.set_transform_static if topic == '/tf_static'
                              else buffer.set_transform)
                    for transform in message.transforms:
                        setter(transform, 'frozen_rosbag')
                    continue
                stamp = message.header.stamp.to_nsec()
                if topic == '/joint_states':
                    for requested in wanted:
                        previous = joints[requested]
                        if stamp <= requested and (previous is None or
                                stamp > previous.header.stamp.to_nsec()):
                            joints[requested] = message
                elif stamp in frames:
                    frames[stamp][topic] = message
        print('READ', path.name, flush=True)
    for stamp, messages in frames.items():
        missing = set((COLOR, DEPTH, MASK, OBJECT)) - set(messages)
        if missing or joints[stamp] is None:
            raise ValueError('exact source %d unavailable: %r' % (stamp, missing))
    return frames, joints, buffer, files


def rebuild_view(stamp, messages, joint, buffer, params, identity):
    return rebuild_fused_view([stamp], {stamp: messages}, {stamp: joint},
                              buffer, params, identity)


def rebuild_fused_view(stamps, frames, joints, buffer, params, identity):
    """Reconstruct the exact ordered source window, not just its last frame."""
    if not stamps or list(stamps) != sorted(set(stamps)):
        raise ValueError('source stamps must be nonempty, unique and increasing')
    bridge = CvBridge()
    remote, camera = params['remote'], params['camera']
    samples = []
    for stamp in stamps:
        messages, joint = frames[stamp], joints[stamp]
        obj = messages[OBJECT]
        by_name = dict(zip(joint.name, joint.position))
        samples.append(RgbdSample(
            color_bgr=bridge.imgmsg_to_cv2(messages[COLOR], 'bgr8'),
            depth_raw=bridge.imgmsg_to_cv2(messages[DEPTH], 'passthrough'),
            object_mask=bridge.imgmsg_to_cv2(messages[MASK], 'mono8'),
            bbox=(obj.bbox_x, obj.bbox_y, obj.bbox_width, obj.bbox_height),
            object_msg=obj, stamp_sec=stamp * 1e-9, stamp_ns=stamp,
            frame_id=messages[COLOR].header.frame_id,
            joint_positions=np.asarray([by_name['Joint%d' % j] for j in range(1, 7)]),
            target_epoch=identity.epoch, target_identity=identity,
        ))
    stamp = stamps[-1]
    pcfg = params.get('perception', {})
    snapshot = fuse_stable_samples(
        samples, require_mask=True,
        min_mask_iou=remote.get('planning_mask_min_iou', .85),
        max_centroid_shift_px=remote.get('planning_mask_max_centroid_shift_px', 5.),
        max_joint_delta_rad=remote.get('planning_max_joint_delta_rad', .01),
        erosion_px=remote.get('mask_erosion_px', 2),
        depth_scale=camera['depth_scale'],
        depth_min_m=pcfg.get('depth_min_m', .03),
        depth_max_m=pcfg.get('depth_max_m', 2.),
        mad_scale=remote.get('depth_mad_scale', 3.5),
        mad_absolute_floor_m=remote.get('depth_mad_absolute_floor_m', .002),
        internal_hole_max_area_px=remote.get('mask_internal_hole_max_area_px', 25),
    )
    if not snapshot.ok:
        raise ValueError(snapshot.failure_code + ': ' + snapshot.failure_reason)
    source = buffer.lookup_transform_core(
        'base_link', snapshot.frame_id, rospy.Time(stamp // 10**9, stamp % 10**9))
    q, t = source.transform.rotation, source.transform.translation
    transform = quaternion_matrix([q.x, q.y, q.z, q.w])
    transform[:3, 3] = [t.x, t.y, t.z]
    if 'optical' not in snapshot.frame_id.lower():
        convention = np.eye(4)
        convention[:3, :3] = [[0, 0, 1], [-1, 0, 0], [0, -1, 0]]
        transform = transform.dot(convention)
    intrinsics = CameraIntrinsics(**{k: camera[k] for k in (
        'width', 'height', 'fx', 'fy', 'cx', 'cy', 'depth_scale')})
    estimate = estimate_object_geometry(
        depth_raw=snapshot.depth_raw, target_depth_raw=snapshot.target_depth_raw,
        object_mask=snapshot.object_mask, bbox=snapshot.bbox,
        intrinsics=intrinsics, depth_scale=camera['depth_scale'],
        T_base_camera=transform, source_mode=snapshot.source_mode,
        support_bbox_expand_ratio=remote.get('support_bbox_expand_ratio', .30),
        support_distance_threshold_m=remote.get('support_distance_threshold_m',
            remote.get('target_cloud_support_plane_inlier_distance_m', .004)),
        voxel_size_m=remote.get('target_cloud_voxel_size_m', .0025),
        min_support_points=remote.get('geometry_min_support_points', 200),
        min_object_points=remote.get('geometry_min_object_points',
            remote.get('target_cloud_min_points', 120)),
        min_size_m=remote.get('geometry_min_size_m', .005),
        max_size_m=remote.get('geometry_max_size_m', .600),
        max_height_m=remote.get('geometry_max_height_m', .500),
        previous_axes_base=None,
        outlier_neighbors=remote.get('target_cloud_outlier_neighbors', 16),
        outlier_std_ratio=remote.get('target_cloud_outlier_std_ratio', 2.),
    )
    if not estimate.ok:
        raise ValueError(estimate.failure_code + ': ' + estimate.failure_reason)
    view = surface.surface_view_from_observation(
        observation_from_snapshot(snapshot, estimate, transform))
    return view, {
        'quality': plain(snapshot.quality),
        'T_base_optical': transform.tolist(),
        'joint_stamp_ns': joint.header.stamp.to_nsec(),
        'joint_positions': list(joint.position),
        'bbox': snapshot.bbox,
        'support_inlier_ratio': estimate.support_inlier_ratio,
        'source_stamps_ns': list(stamps),
        'previous_obb_axes_reconstructed': False,
    }


def seed_diagnostics(reference, moving, config):
    basis = surface._support_basis(reference.support_normal_base)
    r = surface._sorted_points(reference.points_base)
    m = surface._sorted_points(moving.points_base)
    coarse = surface._initial_support_transform(r, m, basis)
    out = {}
    for name, transform in [('identity', np.eye(4)), ('coarse', coarse)]:
        _, _, distances, inliers = surface._registration_correspondences(
            r, m, transform, config)
        out[name] = {
            'yaw_deg': surface._yaw_degrees(transform[:3, :3], basis),
            'inliers': int(np.count_nonzero(inliers)),
            'cost_m2': float(np.mean(np.where(
                inliers, distances**2, config.correspondence_max_m**2))),
        }
    return out


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bags', required=True)
    parser.add_argument('--parameters', required=True)
    parser.add_argument('--reference-stamp-ns', type=int, required=True)
    parser.add_argument('--moving-stamp-ns', type=int, required=True)
    parser.add_argument('--track-id', required=True)
    parser.add_argument('--epoch', type=int, required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    stamps = [args.reference_stamp_ns, args.moving_stamp_ns]
    params = json.loads(Path(args.parameters).read_text())
    frames, joints, buffer, files = read_sources(args.bags, stamps)
    identity = TargetTrackIdentity(args.epoch, args.track_id)
    rebuilt = [rebuild_view(s, frames[s], joints[s], buffer, params, identity)
               for s in stamps]
    reference, moving = [item[0] for item in rebuilt]
    config = surface.RegistrationConfig(**params['remote'].get('multiview', {}))
    result = surface.register_surface_view(reference, moving, config)
    out = {
        'mode': 'offline_exact_source_rgbd_and_recorded_tf',
        'bag_files': [str(p) for p in files],
        'reference': plain(reference), 'moving': plain(moving),
        'source_details': [item[1] for item in rebuilt],
        'config': plain(config), 'result': plain(result),
        'seed_diagnostics': seed_diagnostics(reference, moving, config),
    }
    with Path(args.output).open('x') as handle:
        json.dump(out, handle, indent=2)
    print(json.dumps({k: v for k, v in out.items()
                      if k in ('result', 'seed_diagnostics')}, indent=2))


if __name__ == '__main__':
    main()
