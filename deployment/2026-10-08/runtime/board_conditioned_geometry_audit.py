"""Pure, offline fixed-board geometry comparison; never authorizes movement.

Inputs must be extracted from exact source-paired RGB-D/board observations.
This module verifies supplied provenance, not authenticity of a source bag.
All camera coordinates use ROS camera_link (x forward, y left, z up).
No ROS imports, file writes, fitting, thresholds overrides, or target updates.
"""
import numpy as np


PAIR_FIELDS = ('board_reference_id', 'target_track_id', 'plane_label',
               'camera_model_fingerprint', 'handeye_sha256', 'driver_epoch_ns')
STAMP_FIELDS = ('color_stamp_ns', 'depth_stamp_ns', 'board_stamp_ns',
                'quality_stamp_ns')


def rigid(value):
    matrix = np.asarray(value, dtype=float)
    if (matrix.shape != (4, 4) or not np.isfinite(matrix).all()
            or not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-9, rtol=0)
            or not np.allclose(matrix[:3, :3].T @ matrix[:3, :3],
                               np.eye(3), atol=1e-8, rtol=0)
            or not np.isclose(np.linalg.det(matrix[:3, :3]), 1., atol=1e-8, rtol=0)):
        raise ValueError('transform must be a finite proper rigid 4x4 matrix')
    return matrix


def point(value):
    vector = np.asarray(value, dtype=float)
    if vector.shape != (3,) or not np.isfinite(vector).all():
        raise ValueError('point must be a finite three-vector')
    return vector


def plane(value):
    coefficients = np.asarray(value, dtype=float)
    if (coefficients.shape != (4,) or not np.isfinite(coefficients).all()
            or abs(np.linalg.norm(coefficients[:3])-1.) > 1e-6):
        raise ValueError('plane must have a finite unit normal and signed offset')
    return coefficients


def transform_point(transform, value):
    transform = rigid(transform)
    return transform[:3, :3] @ point(value) + transform[:3, 3]


def transform_plane(transform, value):
    """Transform n*x+d=0 without rescaling or changing its signed convention."""
    transform, value = rigid(transform), plane(value)
    normal = transform[:3, :3] @ value[:3]
    return np.r_[normal, value[3]-normal @ transform[:3, 3]]


def validate_observation(sample):
    if not isinstance(sample, dict):
        raise ValueError('observation must be a mapping')
    for name in PAIR_FIELDS[:-1]:
        if not isinstance(sample.get(name), str) or not sample[name].strip():
            raise ValueError('missing provenance '+name)
    epoch, source = sample.get('driver_epoch_ns'), sample.get('source_stamp_ns')
    if (type(epoch) is not int or type(source) is not int or not 0 < epoch < source):
        raise ValueError('observation must belong to a positive, earlier driver epoch')
    if sample.get('camera_frame_convention') != 'ros_camera_link':
        raise ValueError('explicit ROS camera_link coordinates are required')
    if any(type(sample.get(name)) is not int or sample[name] != source
           for name in STAMP_FIELDS):
        raise ValueError('RGB, depth, board and quality stamps must match exactly')
    received = sample.get('received_stamp_ns')
    if type(received) is not int or not 0 <= received-source <= 350_000_000:
        raise ValueError('source reception age must be in [0, 350] ms')
    accepted = sample.get('accepted_stamp_ns')
    if type(accepted) is not int or abs(accepted-source) > 100_000_000:
        raise ValueError('associated accepted feedback must be within 100 ms')
    # Upstream must calculate this from its independent accepted window; a
    # supplied true value alone cannot replace inspection of the original bag.
    if sample.get('independent_stationary_window_verified') is not True:
        raise ValueError('independent accepted-feedback stationary window required')
    rigid(sample['T_camera_board'])
    rigid(sample['T_base_camera_fk'])
    plane(sample['support_plane_camera'])
    point(sample['anchor_camera'])


def plane_comparison(reference_plane, current_plane, reference_anchor, current_anchor):
    first, second = plane(reference_plane), plane(current_plane)
    # A reversed normal is semantically inconsistent, not a plane to silently flip.
    if first[:3] @ second[:3] <= 0:
        raise ValueError('plane normals must have a consistent signed orientation')
    anchor = .5*(point(reference_anchor)+point(current_anchor))
    delta = float((second[:3]-first[:3]) @ anchor+second[3]-first[3])
    angle = float(np.degrees(np.arccos(np.clip(first[:3] @ second[:3], -1, 1))))
    return dict(shared_anchor=anchor.tolist(), reference_plane=first.tolist(),
                current_plane=second.tolist(), normal_angle_deg=angle,
                current_minus_reference_signed_distance_m=delta,
                absolute_local_plane_separation_m=abs(delta))


def audit_pair(reference, current):
    """Describe a same-board, same-target, same-epoch pair in two frames.

    No 'passed', 'corrected' or motion-authorization result is returned. A board
    is not external truth: PnP and depth share camera/board model assumptions.
    """
    for sample in (reference, current):
        validate_observation(sample)
    if any(reference[name] != current[name] for name in PAIR_FIELDS):
        raise ValueError('pair identity, model, hand-eye or driver epoch changed')
    if current['source_stamp_ns'] <= reference['source_stamp_ns']:
        raise ValueError('pair source stamps must increase')
    comparisons = {}
    for name in ('board', 'base_fk'):
        transformed = []
        for sample in (reference, current):
            transform = (np.linalg.inv(rigid(sample['T_camera_board']))
                         if name == 'board' else rigid(sample['T_base_camera_fk']))
            transformed.append((transform_plane(transform, sample['support_plane_camera']),
                                transform_point(transform, sample['anchor_camera'])))
        comparisons[name] = plane_comparison(transformed[0][0], transformed[1][0],
                                               transformed[0][1], transformed[1][1])
    board_poses = [rigid(s['T_base_camera_fk']) @ rigid(s['T_camera_board'])
                   for s in (reference, current)]
    rotation = board_poses[0][:3, :3].T @ board_poses[1][:3, :3]
    delta = board_poses[1][:3, 3]-board_poses[0][:3, 3]
    return dict(kind='offline_board_conditioned_geometry_comparison',
                motion_authorized=False, **{k: reference[k] for k in PAIR_FIELDS},
                source_stamps_ns=[reference['source_stamp_ns'], current['source_stamp_ns']],
                plane_comparisons=comparisons,
                apparent_base_board_translation_m=delta.tolist(),
                apparent_base_board_displacement_m=float(np.linalg.norm(delta)),
                apparent_base_board_rotation_deg=float(np.degrees(np.arccos(
                    np.clip((np.trace(rotation)-1.)/2., -1., 1.)))),
                limitations=[
                    'Provided provenance must be verified against the original source bag.',
                    'PnP and depth share camera geometry assumptions; board scale is not external truth.',
                    'A board-local invariant alone does not validate camera-to-fingertip geometry.',
                    'No FK correction, hand-eye replacement or contact motion is authorized.'])
