"""Repeat target visibility screening on the final unknown-object observation path.

This does not certify depth availability or permit contact. Arrival still needs
fresh measured support and identity. The calibrated projection is frozen and
rechecked before submission, separately from the existing collision proof.
"""
from alicia_flexible_grasp.robot.observation_view_limits import read_observation_view_parameters
from copy import deepcopy
from itertools import product
import math

import numpy as np
import rospy
import tf2_ros
from tf.transformations import quaternion_matrix

from .observation_path_guard import ObservationPathError
from .observation_view_guard import validate_observation_target_trajectory
from .observation_view_limits import effective_observation_view_limits


def _camera_configuration_contract(camera):
    """Keep configuration and unknown fields; remove known per-read telemetry.

    The complete parameter snapshot remains in the audit. Regional ownership
    and visual policy are checked against their declared fixed schema before
    their optional diagnostic rows are removed. Raw-history intrinsics must
    agree with the active SDK profile, which remains strictly bound below.
    """
    result = deepcopy(camera)
    profile = result.get('runtime_profile')
    if not isinstance(profile, dict) or not isinstance(profile.get('stationary_temporal'), dict):
        return result
    temporal = profile['stationary_temporal']
    for key in ('active', 'history_warming_up', 'regional_output_active', 'fully_active',
                'filtered_pixel_count', 'filtered_fraction', 'alignment_proven', 'history_reset'):
        temporal.pop(key, None)

    def extras(value, known):
        return {key: deepcopy(item) for key, item in value.items() if key not in known}

    def invalid():
        raise ObservationPathError('OBSERVATION_VIEW_CAMERA_CONFIGURATION_INVALID: stationary diagnostic configuration changed')

    regions = temporal.get('regions')
    if (isinstance(regions, list)
            and temporal.get('owner_schema') == 'fixed_center_02_08_01_09_then_four_quadrants_v1'):
        intrinsic = profile.get('color_intrinsics', {})
        width, height = intrinsic.get('width'), intrinsic.get('height')
        if type(width) is not int or type(height) is not int or min(width, height) <= 0:
            invalid()
        center = [int(.2*width), int(.1*height), int(.8*width), int(.9*height)]
        boxes = dict(center=center, top_left=[0, 0, width//2, height//2],
            top_right=[width//2, 0, width, height//2],
            bottom_left=[0, height//2, width//2, height],
            bottom_right=[width//2, height//2, width, height])
        retained = []
        for row in regions:
            if not isinstance(row, dict) or row.get('name') not in boxes:
                invalid()
            name = row['name']; box = boxes[name]
            pixels = (box[2]-box[0])*(box[3]-box[1])
            if name != 'center':
                pixels -= max(0, min(box[2], center[2])-max(box[0], center[0])) * max(
                    0, min(box[3], center[3])-max(box[1], center[1]))
            if row.get('bounds_xyxy') != box or row.get('owned_pixels') != pixels:
                invalid()
            remaining = extras(row, ('name', 'bounds_xyxy', 'owned_pixels',
                'qualified', 'warming', 'history_reset', 'filter_applied', 'visual'))
            visual = row.get('visual')
            if visual is not None:
                if not isinstance(visual, dict):
                    invalid()
                policy = dict(window_sec=temporal.get('visual_window_sec'),
                    max_motion_px=temporal.get('visual_max_motion_full_resolution_px'),
                    max_points=64, coordinate_units='full_resolution_pixels')
                if any(visual.get(key) != value for key, value in policy.items()):
                    invalid()
                extra_visual = extras(visual, (*policy, 'qualified', 'reason', 'tracked_points',
                    'frame_number', 'device_timestamp_ms', 'timestamp_domain', 'stream_fps',
                    'continuous_frames', 'covered_quadrants', 'host_quiet_span_sec',
                    'quiet_span_sec', 'motion_median_px', 'motion_p90_px'))
                if extra_visual:
                    remaining['visual'] = extra_visual
            if remaining:
                retained.append(dict(name=name, **remaining))
        temporal.pop('regions')
        if retained:
            temporal['regions'] = retained

    history = temporal.get('pair_history')
    if isinstance(history, dict):
        if (history.get('diagnostic_only') is not True
                or history.get('authorizes_pair_or_motion') is not False):
            invalid()
        retained = extras(history, ('diagnostic_only', 'authorizes_pair_or_motion',
            'state', 'reason', 'history_epoch', 'first_rejection_host_monotonic',
            'last_complete_filter_host_monotonic', 'rejected_packet_count', 'failed_stage',
            'last_complete_raw_frames', 'rejected_raw_frames', 'latest_rejected_raw_frames',
            'next_accepted_raw_frames', 'existing_gate_evaluation', 'parameter_encoding'))
        for field in ('last_complete_raw_frames', 'rejected_raw_frames',
                      'latest_rejected_raw_frames', 'next_accepted_raw_frames'):
            raw = history.get(field, {})
            if not isinstance(raw, dict):
                invalid()
            remainder = extras(raw, ('color', 'depth'))
            for stream in ('color', 'depth'):
                frame = raw.get(stream, {})
                if not isinstance(frame, dict):
                    invalid()
                signature = frame.get('profile_signature')
                if signature is not None:
                    intrinsic = profile.get('color_intrinsics' if stream == 'color' else 'native_depth_intrinsics', {})
                    if not isinstance(signature, dict) or any(
                            signature.get(key) != intrinsic.get(alias)
                            for key, alias in (('width', 'width'), ('height', 'height'),
                                ('fx', 'fx'), ('fy', 'fy'), ('ppx', 'cx'), ('ppy', 'cy'), ('coeffs', 'coeffs'))):
                        invalid()
                    if (str(signature.get('model')).rsplit('.', 1)[-1] != intrinsic.get('model')
                            or signature.get('fps') != camera.get('fps')
                            or str(signature.get('format')).rsplit('.', 1)[-1] != (
                                'bgr8' if stream == 'color' else 'z16')
                            or str(signature.get('stream_type')).rsplit('.', 1)[-1] != stream
                            or signature.get('stream_index') != 0):
                        invalid()
                extra_frame = extras(frame, ('present', 'frame_number', 'device_timestamp_ms',
                    'stream_fps', 'timestamp_domain', 'metadata_raw', 'unavailable', 'profile_signature'))
                if isinstance(signature, dict):
                    extra_signature = extras(signature, ('width', 'height', 'fx', 'fy', 'ppx', 'ppy',
                        'coeffs', 'model', 'fps', 'format', 'stream_index', 'stream_type', 'unique_id'))
                    if extra_signature:
                        extra_frame['profile_signature'] = extra_signature
                for metadata_key in ('metadata_raw', 'unavailable'):
                    metadata = frame.get(metadata_key, {})
                    if not isinstance(metadata, dict):
                        invalid()
                    extra_metadata = extras(metadata, ('actual_fps', 'backend_timestamp', 'frame_timestamp',
                        'time_of_arrival', 'frame_counter', 'sensor_timestamp'))
                    if extra_metadata:
                        extra_frame[metadata_key] = extra_metadata
                if extra_frame:
                    remainder[stream] = extra_frame
            if remainder:
                retained[field] = remainder
        evaluation = history.get('existing_gate_evaluation', {})
        if not isinstance(evaluation, dict):
            invalid()
        extra_evaluation = extras(evaluation, ('encoder_active', 'alignment_proven',
            'filtered_pixel_count', 'regional_global_reset_reason', 'regions'))
        evaluation_regions = evaluation.get('regions', ())
        if not isinstance(evaluation_regions, (list, tuple)):
            invalid()
        for row in evaluation_regions:
            if not isinstance(row, dict):
                invalid()
            extra_row = extras(row, ('name', 'filter_applied', 'qualified', 'warming', 'history_reset'))
            if extra_row:
                extra_evaluation.setdefault('regions', []).append(dict(name=row.get('name'), **extra_row))
        if extra_evaluation:
            retained['existing_gate_evaluation'] = extra_evaluation
        encoding = history.get('parameter_encoding', {})
        if not isinstance(encoding, dict):
            invalid()
        extra_encoding = extras(encoding, ('large_integers', 'typed_raw_source',
            'omitted_null_fields', 'empty_string_list_nulls'))
        if extra_encoding:
            retained['parameter_encoding'] = extra_encoding
        temporal.pop('pair_history')
        if retained:
            temporal['pair_history'] = retained
    return result


class ObservationViewGateway:
    def _initialize_observation_view_tf(self):
        self._observation_view_tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.))
        self._observation_view_tf_listener = tf2_ros.TransformListener(self._observation_view_tf_buffer)

    def _unknown_observation_view_configuration(self, scene):
        camera = deepcopy(rospy.get_param('/camera', {}))
        handeye = deepcopy(rospy.get_param('/handeye', {}))
        frame = camera.get('frame_id', 'camera_link')
        if (handeye.get('mode') != 'eye_on_hand'
                or handeye.get('parent_frame') != 'tool0'
                or frame != 'camera_link' or handeye.get('camera_frame') != frame
                or handeye.get('child_frame', frame) != frame):
            raise ObservationPathError('OBSERVATION_VIEW_CALIBRATION_FRAME_INVALID')
        keys = ('fx', 'fy', 'cx', 'cy', 'width', 'height')
        values = {key: camera[key] for key in keys}
        if (any(isinstance(values[k], bool) or not isinstance(values[k], (int, float))
                or not math.isfinite(values[k]) for k in keys)
                or min(values['fx'], values['fy']) <= 0
                or any(type(values[k]) is not int or values[k] <= 0 for k in ('width', 'height'))
                or camera.get('align_depth_to_color') is not True):
            raise ObservationPathError('OBSERVATION_VIEW_CAMERA_CONFIGURATION_INVALID')
        values.update(frame_id=frame, parent_frame='tool0', align_depth_to_color=True)
        requested_limits = read_observation_view_parameters(rospy.get_param)
        try:
            limits = effective_observation_view_limits(requested_limits)
        except ValueError as exc:
            raise ObservationPathError('OBSERVATION_VIEW_LIMITS_INVALID: ' + str(exc)) from exc
        stamp = rospy.Time(scene.source_ns // 1000000000, scene.source_ns % 1000000000)
        transform = self._observation_view_tf_buffer.lookup_transform(
            'tool0', frame, stamp, rospy.Duration(.2)).transform
        q = np.asarray([getattr(transform.rotation, k) for k in 'xyzw'], dtype=float)
        t = np.asarray([getattr(transform.translation, k) for k in 'xyz'], dtype=float)
        if (not np.all(np.isfinite(q)) or not np.all(np.isfinite(t))
                or abs(float(np.linalg.norm(q)) - 1.) > 1e-6):
            raise ObservationPathError('OBSERVATION_VIEW_HAND_EYE_TF_INVALID')
        matrix = quaternion_matrix(q)
        matrix[:3, 3] = t
        # A latest lookup is used ONLY to verify that the rigid installation
        # has not changed; the projection above always uses the source stamp.
        latest = self._observation_view_tf_buffer.lookup_transform(
            'tool0', frame, rospy.Time(0), rospy.Duration(.2)).transform
        latest_q = np.asarray([getattr(latest.rotation, k) for k in 'xyzw'], dtype=float)
        latest_t = np.asarray([getattr(latest.translation, k) for k in 'xyz'], dtype=float)
        if (not np.all(np.isfinite(latest_q)) or not np.all(np.isfinite(latest_t))
                or abs(float(np.linalg.norm(latest_q)) - 1.) > 1e-6):
            raise ObservationPathError('OBSERVATION_VIEW_CURRENT_MOUNT_INVALID')
        latest_matrix = quaternion_matrix(latest_q)
        latest_matrix[:3, 3] = latest_t
        if not np.allclose(latest_matrix, matrix, atol=1e-9, rtol=0.):
            raise ObservationPathError('OBSERVATION_VIEW_MOUNT_CHANGED_SINCE_SOURCE')
        return dict(camera=values, limits=limits, tool_from_camera=matrix.tolist(),
                    requested_visibility_limits_snapshot=requested_limits,
                    camera_configuration_snapshot=camera,
                    camera_configuration_contract=_camera_configuration_contract(camera),
                    handeye_configuration_snapshot=handeye,
                    source_stamp_ns=scene.source_ns)

    def _unknown_observation_view_audit(self, scene, trajectory, model, desired):
        if not getattr(scene, 'require_target_visibility', False):
            return None
        configuration = self._unknown_observation_view_configuration(scene)
        camera = configuration['camera']
        corners = scene.center + (np.asarray(tuple(product((-1., 1.), repeat=3)))
                                  * scene.size * .5) @ scene.rotation.T
        report = validate_observation_target_trajectory(trajectory, model,
            np.asarray(configuration['tool_from_camera']), corners,
            [camera[k] for k in ('fx', 'fy', 'cx', 'cy')],
            camera['width'], camera['height'], desired, **configuration['limits'])
        return dict(configuration=configuration, target_visibility=report,
            endpoint_support='requires_fresh_measured_arrival_not_certified_by_this_screen',
            plan_id=scene.plan_id, plan_digest=scene.digest)

    def _revalidate_unknown_observation_view(self, scene, audit):
        if not getattr(scene, 'require_target_visibility', False):
            if audit is not None:
                raise ObservationPathError('OBSERVATION_VIEW_UNEXPECTED_AUDIT')
            return
        previous = audit.get('configuration') if isinstance(audit, dict) else None
        if (not isinstance(audit, dict) or audit.get('plan_id') != scene.plan_id
                or audit.get('plan_digest') != scene.digest or not isinstance(previous, dict)
                or not isinstance(previous.get('camera_configuration_snapshot'), dict)):
            raise ObservationPathError('OBSERVATION_VIEW_CONFIGURATION_CHANGED')
        current = self._unknown_observation_view_configuration(scene)
        # Keep both raw snapshots auditable while comparing the same stable
        # contract. Unknown parameter/configuration changes still invalidate.
        comparable = lambda value: {key: item for key, item in value.items()
                                    if key != 'camera_configuration_snapshot'}
        if (previous.get('camera_configuration_contract') != _camera_configuration_contract(
                    previous['camera_configuration_snapshot'])
                or comparable(previous) != comparable(current)):
            raise ObservationPathError('OBSERVATION_VIEW_CONFIGURATION_CHANGED')
