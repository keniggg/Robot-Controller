#!/usr/bin/env python3
"""Replay closed single-axis bags into a fixed-board depth-surface comparison.

This fits depth inside the board, not the motor/table contact region. It is a
diagnostic of two reconstruction chains, never a grasp correction or approval.
"""
import argparse
import bisect
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import sys

import numpy as np
import rosbag
from cv_bridge import CvBridge
from tf.transformations import quaternion_matrix


ROOT = Path(__file__).resolve().parents[1]
MOUNT = Path('/media/zhuyupei/SanStand')
sys.path.insert(0, str(ROOT/'src/alicia_flexible_grasp_supervisor/src'))
from alicia_flexible_grasp.robot.observation_path_guard import SerialUrdfFk


def load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


AUDIT = load_module('board_audit', Path(__file__).with_name('board_conditioned_geometry_audit.py'))
CAMERA = load_module('camera_model', ROOT/'src/real-arm/alicia_d_calibration/scripts/charuco_tracker.py')
OPTICAL_TO_LINK = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0.]])
GEOMETRIES = ('baseline_geometry.json', 'return_original_sdk_geometry.json',
              'return_post_settle_geometry.json')


def matrix(message):
    q, p = message.transform.rotation, message.transform.translation
    result = quaternion_matrix([q.x, q.y, q.z, q.w])
    result[:3, 3] = [p.x, p.y, p.z]
    return result


def fit_board_depth(geometry, depth):
    model = CAMERA.RuntimeColorModel(geometry['quality']['camera_model'])
    params = geometry['camera_parameters']
    runtime = params['runtime_profile']
    if runtime['geometry_intrinsics'] != runtime['color_intrinsics']:
        raise ValueError('depth rays are not in the same color geometry')
    if depth.shape != (model.intrinsics.height, model.intrinsics.width):
        raise ValueError('depth shape differs from current color model')
    y, x = np.mgrid[12:depth.shape[0]-12:4, 12:depth.shape[1]-12:4]
    uv = np.c_[x.ravel(), y.ravel()]
    z = depth[y.ravel(), x.ravel()]*params['depth_scale']
    rays = np.c_[model.normalized_points(uv).reshape(-1, 2), np.ones(len(uv))]
    camera_points = (rays*z[:, None]) @ OPTICAL_TO_LINK.T
    board = np.asarray(geometry['T_camera_board'])
    board_points = (camera_points-board[:3, 3]) @ board[:3, :3]
    mask = ((z > .03) & (z < 1.) & (board_points[:, 0] > .02)
        & (board_points[:, 0] < .16) & (board_points[:, 1] > .02)
        & (board_points[:, 1] < .115) & (abs(board_points[:, 2]) < .04))
    points = camera_points[mask]
    if len(points) < 100:
        raise ValueError('insufficient interior board depth')
    inliers = np.ones(len(points), dtype=bool)
    for _ in range(5):
        if np.count_nonzero(inliers) < 100:
            raise ValueError('insufficient board plane inliers')
        center = points[inliers].mean(axis=0)
        _, _, axes = np.linalg.svd(points[inliers]-center, full_matrices=False)
        normal = axes[-1]
        if normal @ board[:3, 2] < 0:
            normal *= -1
        offset = -normal @ center
        inliers = abs(points @ normal+offset) <= .002
    residual = points @ normal+offset
    rms = float(np.sqrt(np.mean(residual[inliers]**2)))
    if inliers.mean() < .7 or rms > .002:
        raise ValueError('board plane quality insufficient')
    return np.r_[normal, offset], AUDIT.transform_point(board, [.09, .0675, 0]), dict(
        sampled_points=len(points), inliers=int(inliers.sum()), inlier_ratio=float(inliers.mean()),
        inlier_rms_m=rms, fitted_region='board interior, not table or object')


def stationary_at(stamp, accepted):
    stamps = [row[0] for row in accepted]
    begin = max(0, bisect.bisect_right(stamps, stamp-400_000_000)-1)
    end = bisect.bisect_left(stamps, stamp+400_000_000)
    if end >= len(stamps):
        raise ValueError('stationary window has no trailing feedback')
    selected = accepted[begin:end+1]
    times = np.array([row[0] for row in selected], dtype=np.int64)
    positions = np.array([row[1] for row in selected])
    if (len(selected) < 6 or times[-1]-times[0] < 800_000_000
            or np.max(np.diff(times)) > 250_000_000
            or np.max(np.ptp(positions, axis=0)) > np.radians(.15)):
        raise ValueError('independent stationary window invalid')
    return dict(start_stamp_ns=int(times[0]), end_stamp_ns=int(times[-1]),
        sample_count=len(selected), peak_to_peak_rad=np.ptp(positions, axis=0).tolist())


def replay(directory):
    plan = json.loads((directory/'plan.json').read_text())
    fk = SerialUrdfFk((directory/'robot_description.urdf').read_text(),
                      ['Joint%d'%i for i in range(1, 7)])
    geometries = {name: json.loads((directory/name).read_text()) for name in GEOMETRIES
                  if (directory/name).exists()}
    wanted = {g['source_stamp_ns'] for g in geometries.values()}
    messages, accepted = {}, []
    topics = ['/bounded_response/snapshot_rgb', '/supervisor/camera/depth/image_raw',
        '/charuco/quality', '/tf', '/tf_static', '/alicia_d/accepted_joint_states',
        '/bounded_response/base_tool', '/bounded_response/tool_camera']
    with rosbag.Bag(str(directory/'evidence.bag')) as bag:
        for topic, message, received in bag.read_messages(topics=topics):
            if topic == '/alicia_d/accepted_joint_states':
                q = dict(zip(message.name, message.position))
                accepted.append((message.header.stamp.to_nsec(), [q['Joint%d'%i] for i in range(1, 7)]))
                continue
            if topic == '/charuco/quality':
                value = json.loads(message.data)
                stamp = value.get('source_stamp_ns')
                if stamp in wanted:
                    messages[(stamp, 'quality')] = (value, received.to_nsec())
            elif topic in ('/tf', '/tf_static'):
                for value in message.transforms:
                    stamp = value.header.stamp.to_nsec()
                    if (topic == '/tf_static' and value.header.frame_id == 'tool0'
                            and value.child_frame_id == 'camera_link'):
                        messages[('static', 'tool_camera')] = (value, received.to_nsec())
                    if (stamp in wanted and value.header.frame_id == 'camera_link'
                            and value.child_frame_id == 'charuco_board'):
                        messages[(stamp, 'board')] = (value, received.to_nsec())
            else:
                stamp = message.header.stamp.to_nsec()
                if stamp in wanted:
                    key = {'/bounded_response/snapshot_rgb': 'rgb',
                        '/supervisor/camera/depth/image_raw': 'depth',
                        '/bounded_response/base_tool': 'base_tool',
                        '/bounded_response/tool_camera': 'tool_camera'}[topic]
                    messages[(stamp, key)] = (message, received.to_nsec())
    accepted.sort(key=lambda row: row[0])
    observations, details = {}, {}
    bridge = CvBridge()
    for name, geometry in geometries.items():
        stamp = geometry['source_stamp_ns']
        try:
            rows = {key: messages[(stamp, key)] for key in
                    ('rgb', 'depth', 'quality', 'board')}
            rows['tool_camera'] = messages[('static', 'tool_camera')]
            quality = rows['quality'][0]
            if (quality != geometry['quality'] or quality.get('pose_ok') is not True
                    or quality['charuco_count'] < 35 or quality['reprojection_rms_px'] > 1.5
                    or quality['edge_clearance_px'] < 20):
                raise ValueError('source quality differs or is insufficient')
            for key, matrix_name in (('board', 'T_camera_board'),
                    ('tool_camera', 'T_tool_camera')):
                if not np.allclose(matrix(rows[key][0]), geometry[matrix_name], atol=1e-10, rtol=0):
                    raise ValueError('saved geometry differs from source '+key)
            # Geometry association can precede arrival of a still-closer sample.
            associated = [row for row in accepted if row[0] == geometry['accepted_stamp_ns']]
            if len(associated) != 1 or not np.allclose(associated[0][1],
                    geometry['accepted_positions_rad'][:6], atol=1e-10, rtol=0):
                raise ValueError('associated accepted feedback not verified')
            window = stationary_at(stamp, accepted)
            base_tool_fk = fk(associated[0][1])
            fk_translation_difference = float(np.linalg.norm(
                base_tool_fk[:3, 3]-np.asarray(geometry['T_base_tool'])[:3, 3]))
            if fk_translation_difference > .002:
                raise ValueError('source TF and associated accepted FK differ by over 2 mm')
            depth = bridge.imgmsg_to_cv2(rows['depth'][0], 'passthrough')
            coefficients, anchor, fit = fit_board_depth(geometry, depth)
            handeye = np.asarray(geometry['T_tool_camera'], dtype='<f8')
            sample = dict(board_reference_id=directory.name, target_track_id='fixed-board-depth-surface',
                plane_label='board_depth_surface', camera_model_fingerprint=quality['camera_model_fingerprint'],
                handeye_sha256=hashlib.sha256(handeye.tobytes()).hexdigest(),
                driver_epoch_ns=plan['frozen_epoch_ns'], source_stamp_ns=stamp,
                received_stamp_ns=max(rows[key][1] for key in ('quality', 'board', 'depth')),
                accepted_stamp_ns=geometry['accepted_stamp_ns'],
                independent_stationary_window_verified=True, camera_frame_convention='ros_camera_link',
                T_camera_board=geometry['T_camera_board'],
                T_base_camera_fk=(base_tool_fk @ handeye).tolist(),
                support_plane_camera=coefficients.tolist(), anchor_camera=anchor.tolist())
            sample.update({key: stamp for key in AUDIT.STAMP_FIELDS})
            AUDIT.validate_observation(sample)
            observations[name] = sample
            details[name] = dict(status='paired_and_stationary', source_stamp_ns=stamp,
                reception_age_ms=(sample['received_stamp_ns']-stamp)/1e6,
                tf_vs_accepted_fk_translation_m=fk_translation_difference,
                stationary_window=window, plane_fit=fit,
                source_geometry_sha256=hashlib.sha256((directory/name).read_bytes()).hexdigest())
        except (KeyError, ValueError) as error:
            details[name] = dict(status='rejected', source_stamp_ns=stamp, reason=str(error))
    pairs = {}
    if GEOMETRIES[0] in observations:
        for name in GEOMETRIES[1:]:
            if name in observations:
                pairs[name] = AUDIT.audit_pair(observations[GEOMETRIES[0]], observations[name])
    return dict(session=str(directory), samples=details, observations=observations, comparisons=pairs,
        scope='Closed historical bag; exact paired depth plane inside the fixed board, not the motor or support table. '
        'No motion, calibration, target replacement or correction is authorized.')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('sessions', nargs='+', type=Path)
    args = parser.parse_args()
    if (not os.path.ismount(str(MOUNT)) or MOUNT not in args.output.resolve().parents
            or not args.output.parent.is_dir()):
        raise RuntimeError('output must be in an existing mounted SanStand directory; no fallback')
    result = dict(kind='historical_board_conditioned_depth_replay', motion_authorized=False,
                  sessions=[replay(path) for path in args.sessions])
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False)+'\n')
    for session in result['sessions']:
        print(json.dumps(dict(session=session['session'], samples=session['samples'],
            comparison_summary={name: {key: value['absolute_local_plane_separation_m']*1000
                for key, value in comparison['plane_comparisons'].items()}
                for name, comparison in session['comparisons'].items()}), ensure_ascii=False))


if __name__ == '__main__':
    main()
