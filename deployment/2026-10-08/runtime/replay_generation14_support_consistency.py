#!/usr/bin/env python3
"""Read-only gen14 bag replay; never connects to a ROS master or controls an arm.

Run after sourcing /opt/ros/noetic/setup.bash and this workspace's devel/setup.bash:
  OPENBLAS_NUM_THREADS=1 PYTHONNOUSERSITE=1 /usr/bin/python3 \
    .runtime/replay_generation14_support_consistency.py --output EXTERNAL_PATH

The sample window is exactly the bounded window already reviewed, not a new
experiment. The only output written is the requested JSON diagnostic report.
"""
import argparse
from dataclasses import asdict
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import sys

import numpy as np
import rosbag
import rospy
import tf2_py
from tf.transformations import quaternion_matrix

WORKSPACE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(WORKSPACE / 'src/alicia_grasp_modes/src'))
from alicia_grasp_modes import tabletop  # noqa: E402

STORE = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
RECORD = STORE / 'grasp_records/20260930T053058.932518Z_dc1e8707ea6d4477b8f580d8e9a5b63b'
EVIDENCE = STORE / 'ros_log/resumed_observe_return_grasp_20260930T040740Z'
WINDOW_NS = (1790746312000000000, 1790746325800000000)
DEPTH_TOPIC = '/supervisor/camera/depth/image_raw'


def sha256(path):
    digest = hashlib.sha256()
    with path.open('rb') as source:
        for chunk in iter(lambda: source.read(4 * 1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def source(path):
    return {'path': str(path), 'sha256': sha256(path)}


def matrix(transform):
    q, p = transform.transform.rotation, transform.transform.translation
    result = quaternion_matrix([q.x, q.y, q.z, q.w])
    result[:3, 3] = [p.x, p.y, p.z]
    return result


def agreement(reference, current, anchor, footprint):
    if reference[:3] @ current[:3] < 0:
        current = -current
    delta = current - reference
    anchor_shift = float(delta[:3] @ anchor + delta[3])
    footprint_shift = footprint @ delta[:3] + delta[3]
    maximum = max(abs(anchor_shift), float(np.max(np.abs(footprint_shift))))
    return {
        'normal_angle_deg': float(np.degrees(np.arccos(np.clip(
            current[:3] @ reference[:3], -1., 1.)))),
        'signed_anchor_distance_m': anchor_shift,
        'signed_footprint_distances_m': footprint_shift.tolist(),
        'max_footprint_distance_m': float(np.max(np.abs(footprint_shift))),
        'max_local_distance_m': maximum,
    }


def statistics(rows):
    valid = [row for row in rows if row.get('current_fit_evidence_qualified')]
    result = {'sample_count': len(rows), 'qualified_fit_count': len(valid),
              'over_original_4mm_gate_count': sum(row['over_original_4mm_gate'] for row in valid)}
    if valid:
        result['first_source_stamp_ns'] = valid[0]['source_stamp_ns']
        result['last_source_stamp_ns'] = valid[-1]['source_stamp_ns']
        for key in ('max_local_distance_m', 'normal_angle_deg', 'support_rms_m', 'support_inlier_ratio'):
            values = np.asarray([row[key] for row in valid])
            result[key] = dict(min=float(values.min()), median=float(np.median(values)),
                               max=float(values.max()), std=float(values.std()))
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--record-dir', type=Path, default=RECORD)
    parser.add_argument('--evidence-dir', type=Path, default=EVIDENCE)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if not args.output.parent.is_dir():
        raise SystemExit('Output directory unavailable; no fallback directory will be used.')
    bag_path = args.record_dir / 'keyframes.bag'
    old_path = args.evidence_dir / 'generation14_post_observation_failure.json'
    new_path = args.evidence_dir / 'generation15_stationary_detector.json'
    old = json.loads(old_path.read_text())['detector']
    new = json.loads(new_path.read_text())
    first_loss = old['first_loss']
    first_ns = first_loss['source_stamp_ns']
    reference = np.asarray(old['reference_support_plane_base'], dtype=float)
    anchor = np.asarray(old['anchor_position_base'], dtype=float)
    footprint = np.asarray(old['reference_support_footprint_base'], dtype=float)
    config = tabletop.Config()
    buffer = tf2_py.BufferCore(rospy.Duration(200))
    feedback, goal, result, task_failure = [], None, None, None
    with rosbag.Bag(str(bag_path)) as bag:
        metadata = json.loads(next(bag.read_messages(
            topics=['/grasp_record/camera_runtime_metadata']))[1].data)
        parameters = metadata['parameters']
        intrinsics = [parameters[key] for key in ('fx', 'fy', 'cx', 'cy')]
        for topic, message, received in bag.read_messages(topics=[
                '/tf', '/tf_static', '/alicia_d/accepted_joint_states',
                '/alicia_controller/follow_joint_trajectory/goal',
                '/alicia_controller/follow_joint_trajectory/result', '/grasp/state']):
            if topic in ('/tf', '/tf_static'):
                setter = buffer.set_transform_static if topic == '/tf_static' else buffer.set_transform
                for transform in message.transforms:
                    setter(transform, 'offline_gen14_replay')
            elif topic.endswith('/accepted_joint_states'):
                feedback.append({'source_stamp_ns': message.header.stamp.to_nsec(),
                                 'position_deg': np.rad2deg(message.position).tolist()})
            elif topic.endswith('/goal'):
                goal = {'goal_id': message.goal_id.id, 'goal_stamp_ns': message.goal_id.stamp.to_nsec(),
                        'recorded_at_ns': received.to_nsec(),
                        'planned_duration_sec': message.goal.trajectory.points[-1].time_from_start.to_sec()}
            elif topic.endswith('/result'):
                result = {'goal_id': message.status.goal_id.id, 'source_stamp_ns': message.header.stamp.to_nsec(),
                          'status': int(message.status.status), 'error_code': int(message.result.error_code)}
            elif message.state == 'FAILED':
                task_failure = {'source_stamp_ns': message.header.stamp.to_nsec(), 'message': message.message}
        if not goal or not result or goal['goal_id'] != result['goal_id']:
            raise ValueError('Expected one matched observation trajectory goal/result.')
        rows = []
        for _, image, received in bag.read_messages(topics=[DEPTH_TOPIC]):
            stamp = image.header.stamp.to_nsec()
            if not WINDOW_NS[0] <= stamp <= WINDOW_NS[1]:
                continue
            row = {'source_stamp_ns': stamp, 'recorded_at_ns': received.to_nsec(),
                   'phase': 'trajectory_in_progress' if stamp < result['source_stamp_ns'] else 'after_trajectory_success'}
            rows.append(row)
            try:
                transform = matrix(buffer.lookup_transform_core('base_link', 'camera_link', image.header.stamp))
                if image.encoding != '16UC1' or image.is_bigendian or image.step != image.width * 2:
                    raise ValueError('Unexpected recorded depth layout.')
                depth = np.frombuffer(image.data, dtype='<u2').reshape(image.height, image.width).astype(np.float32)
                depth *= parameters['depth_scale']
                center = tabletop.project_base_to_pixel(anchor, transform, intrinsics)
                v, u = np.indices(depth.shape)
                scale = min(image.width / 640., image.height / 480.)
                radius = np.hypot(u - center[0], v - center[1])
                valid = np.isfinite(depth) & (depth >= config.depth_min_m) & (depth <= config.depth_max_m)
                ring = valid & (radius >= 70 * scale) & (radius <= 170 * scale)
                points = tabletop._project_pixels(depth[ring], u[ring], v[ring], intrinsics)
                plane, ratio, rms, count = tabletop._fit_plane(points, config)
                supporting = np.abs(np.einsum('ij,j->i', points, plane[:3]) + plane[3]) <= config.support_threshold_m
                quadrants = tabletop._support_quadrants(supporting, u[ring], v[ring], center)
                current = tabletop._normalized_plane(np.linalg.solve(transform.T, plane))
                row.update(agreement(reference, current, anchor, footprint))
                row.update(support_plane_camera=plane.tolist(), support_plane_base=current.tolist(),
                           support_inlier_ratio=float(ratio), support_rms_m=float(rms),
                           support_sample_count=int(count), support_quadrants=int(quadrants),
                           current_fit_evidence_qualified=bool(np.isfinite(rms) and rms <= config.support_threshold_m and quadrants >= 3),
                           over_original_4mm_gate=bool(row['max_local_distance_m'] > config.support_max_local_shift_m + 1e-12))
            except Exception as exc:
                row['error'] = type(exc).__name__ + ': ' + str(exc)
        first_index = next(index for index, row in enumerate(rows) if row['source_stamp_ns'] == first_ns)
        before = max((item for item in feedback if item['source_stamp_ns'] <= first_ns), key=lambda item: item['source_stamp_ns'])
        after = min((item for item in feedback if item['source_stamp_ns'] >= first_ns), key=lambda item: item['source_stamp_ns'])
        new_agreement = agreement(reference, np.asarray(new['reference_support_plane_base']), anchor, footprint)
        report = {
            'schema_version': 1, 'generated_at_utc': datetime.now(timezone.utc).isoformat(),
            'diagnostic_only': True, 'ros_master_contacted': False, 'thresholds_changed': False,
            'sources': {'bag': source(bag_path), 'first_loss_capture': source(old_path),
                        'stationary_new_generation': source(new_path), 'script': source(Path(__file__).resolve()),
                        'tabletop_source': source(Path(tabletop.__file__).resolve())},
            'configuration': {'tabletop': asdict(config),
                              'tabletop_provenance': 'Current production Config defaults; original full detector parameter snapshot absent. Exact first-loss reproduction checked below.',
                              'intrinsics': intrinsics, 'depth_scale_m': parameters['depth_scale'],
                              'camera_metadata_time_semantics': metadata['time_semantics']},
            'scope': {'source_window_ns': list(WINDOW_NS), 'sampling': 'All already-reviewed recorded depth frames inside this bounded window.',
                      'tf': 'Recorded TF interpolated at each exact depth source timestamp using offline BufferCore.',
                      'after_success_definition': 'Source timestamp at/after recorded trajectory success; not a claim that all mechanical settling had ended.'},
            'first_loss': first_loss, 'first_loss_neighborhood': rows[max(0, first_index - 1):first_index + 2],
            'first_loss_reproduces_logged_rounded_distance': round(rows[first_index]['max_local_distance_m'], 6) == 0.004236,
            'motion': {'goal': goal, 'result': result, 'task_failure': task_failure,
                       'first_loss_after_goal_sec': (first_ns - goal['goal_stamp_ns']) / 1e9,
                       'first_loss_before_result_sec': (result['source_stamp_ns'] - first_ns) / 1e9,
                       'first_loss_processing_age_sec': (first_loss['observed_stamp_ns'] - first_ns) / 1e9,
                       'bracketing_sdk_feedback': [before, after],
                       'bracketing_sdk_interval_sec': (after['source_stamp_ns'] - before['source_stamp_ns']) / 1e9},
            'statistics': {phase: statistics([row for row in rows if row['phase'] == phase])
                           for phase in ('trajectory_in_progress', 'after_trajectory_success')},
            'stationary_gen15_against_gen14_frozen_reference': dict(new_agreement,
                new_anchor_source_stamp_ns=new['anchor_source_stamp_ns'],
                old_reference_plane_base=reference.tolist(), new_reference_plane_base=new['reference_support_plane_base']),
            'samples': rows,
            'conclusions': [
                'First rejection occurred during real trajectory motion with fresh source-time-bracketing feedback.',
                'A qualified measured support fit crossed the unchanged footprint distance gate; the target-center distance alone did not cross it.',
                'Later same-generation geometry below threshold does not clear the latched target loss or restore execution authority.',
                'Motion-period fluctuations are superimposed on a persistent cross-view support discrepancy; this bag cannot uniquely identify hand-eye, arm model, physical support, or depth-noise contributions.',
                'The separate stationary generation is a new measurement, not validation of the revoked plan.']}
    if not report['first_loss_reproduces_logged_rounded_distance']:
        raise ValueError('Replay no longer reproduces recorded first-loss distance; report not saved.')
    with args.output.open('x') as destination:
        json.dump(report, destination, ensure_ascii=False, indent=2, allow_nan=False)
        destination.write('\n')
    print(json.dumps({'output': str(args.output), 'report_sha256': sha256(args.output),
                      'script_sha256': report['sources']['script']['sha256'],
                      'first_loss_reproduced': True, 'statistics': report['statistics']}, ensure_ascii=False))


if __name__ == '__main__':
    main()
