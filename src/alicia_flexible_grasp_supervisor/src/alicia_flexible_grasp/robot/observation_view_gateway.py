"""Repeat target visibility screening on the final unknown-object observation path.

This does not certify depth availability or permit contact. Arrival still needs
fresh measured support and identity. The calibrated projection is frozen and
rechecked before submission, separately from the existing collision proof.
"""
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
        requested_limits = {
            key: rospy.get_param('/grasp_6d/remote/' + key, default)
            for key, default in (
                ('camera_visibility_margin_px', 36.),
                ('camera_visibility_min_depth_m', .070),
                ('camera_visibility_max_depth_m', .500))}
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
        if (not isinstance(audit, dict) or audit.get('plan_id') != scene.plan_id
                or audit.get('plan_digest') != scene.digest
                or audit.get('configuration') != self._unknown_observation_view_configuration(scene)):
            raise ObservationPathError('OBSERVATION_VIEW_CONFIGURATION_CHANGED')
