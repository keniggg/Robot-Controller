"""One observation-specific use of the existing bounded following correction.

Only the command is compensated. The original observation pose, frozen OBB,
camera and view limits remain immutable. FK projection never certifies RGB
identity or supplies an arrival measurement.
"""
from copy import deepcopy
import math

import numpy as np
from trajectory_msgs.msg import JointTrajectoryPoint

from alicia_flexible_grasp.grasp.observation_working_volume import (
    observation_obb_corners, validate_planned_observation_working_volume,
    ObservationWorkingVolumeError)
from alicia_flexible_grasp.grasp.rich_plan_integrity import pose_values
from .observation_path_guard import ObservationPathError, segment_coefficients, _polynomial_bounds
from .pregrasp_compensation import pose_matrix, PregraspCompensation
from .pregrasp_feedforward import PregraspFeedforward
from .stationary_following import ARM_NAMES, SDK_QUANTUM_RAD as Q


MARGIN_FAILURE = 'target OBB leaves required camera image margin'


def observation_endpoint_view(scene, configuration, base_from_tool, rgb_valid):
    camera = configuration['camera']
    corners = observation_obb_corners(scene.center, scene.rotation, scene.size)
    transform = np.asarray(base_from_tool) @ np.asarray(configuration['tool_from_camera'])
    audit = validate_planned_observation_working_volume(
        corners, transform, [camera[k] for k in ('fx', 'fy', 'cx', 'cy')],
        camera['width'], camera['height'], snapshot_stamp_ns=scene.source_ns,
        **configuration['limits'])
    valid = np.asarray(rgb_valid)
    if valid.dtype != np.bool_ or valid.shape != (camera['height'], camera['width']):
        raise ObservationPathError('OBSERVATION_CORRECTION_RGB_DOMAIN_INVALID')
    # This is the frozen OBB's observed background, not a newly invented mask.
    margin = max(8, int(round(16 * min(camera['width']/640., camera['height']/480.))))
    low = np.floor(audit['target_uv_min_px']).astype(int)-margin
    high = np.ceil(audit['target_uv_max_px']).astype(int)+1+margin
    x0,y0 = low; x1,y1 = high
    if (x0 < 0 or y0 < 0 or x1 > camera['width'] or y1 > camera['height']
            or np.any(~valid[y0:y1, x0:x1])):
        raise ObservationPathError('OBSERVATION_CORRECTION_RGB_REGION_UNOBSERVED')
    audit.update(scope='frozen_obb_fk_endpoint_projection_only',
        actual_arrival_measurement_required=True, source_freshness_authority=False,
        rgb_identity_verified=False, required_observed_box_xyxy=[int(x0),int(y0),int(x1),int(y1)])
    return audit


class ObservationEndpointCompensation:
    def __init__(self, fk, sample, goal, scene, configuration, rgb_valid):
        self.fk, self.scene = fk, scene
        self.configuration = deepcopy(configuration)
        if np.asarray(rgb_valid).dtype != np.bool_:
            raise ObservationPathError('OBSERVATION_CORRECTION_RGB_DOMAIN_INVALID')
        self.rgb_valid = np.array(rgb_valid, copy=True)
        self.rgb_valid.setflags(write=False)
        self.goal = deepcopy(goal)
        # Preserve the existing local capture and response contracts, while
        # requiring the independent observation view gate for convergence.
        self.recovery = PregraspFeedforward(fk, sample, goal,
            position_tolerance_m=.010, orientation_tolerance_rad=math.radians(5.))
        self.original_view = self.view(pose_matrix(pose_values(goal)))

    def view(self, tool):
        return observation_endpoint_view(self.scene, self.configuration, tool, self.rgb_valid)

    def current_view(self, sample):
        self.recovery.contract._validate(sample)
        try:
            return self.view(self.fk(sample['accepted_positions_rad']))
        except ObservationWorkingVolumeError as exc:
            if str(exc) != MARGIN_FAILURE:
                raise
            # Only this exact margin failure may propose one correction.
            return None

    def propose(self, nominal):
        step, evidence = self.recovery.propose(nominal)
        bias = (np.asarray(self.recovery.initial['sdk_positions_rad'])-
                np.asarray(self.recovery.initial['accepted_positions_rad']))
        predicted = (np.asarray(step.target_counts)-2048)*Q-bias
        evidence['predicted_view'] = self.view(self.fk(predicted))
        evidence.update(scope='observation_fixed_goal_single_feedforward',
                        actual_arrival_measurement_required=True)
        return step, evidence

    def committed(self, step, completed_sec):
        self.recovery.committed(step, completed_sec)

    def evaluate(self, sample):
        passed, evidence = self.recovery.evaluate(sample)
        if not passed:
            raise ObservationPathError(evidence['code'])
        evidence['view_audit'] = self.view(self.fk(sample['accepted_positions_rad']))
        return evidence


def validate_observation_correction_path_bounds(path, baseline, target):
    """The same 32-count box bounds every supplied timed command spline."""
    tr = path.joint_trajectory
    names, points = tuple(tr.joint_names), list(tr.points)
    if names != ARM_NAMES or len(points) < 2:
        raise ObservationPathError('OBSERVATION_CORRECTION_PATH_INVALID')
    start, end = np.asarray(baseline, dtype=float), np.asarray(target, dtype=float)
    if (start.shape != (6,) or end.shape != (6,) or not np.all(np.isfinite(start))
            or not np.all(np.isfinite(end))
            or max(abs(np.asarray(points[0].positions)-start)) > 1e-9
            or max(abs(np.asarray(points[-1].positions)-end)) > 1e-12):
        raise ObservationPathError('OBSERVATION_CORRECTION_PATH_ENDPOINT_CHANGED')
    positive = next((i for i,p in enumerate(points) if p.time_from_start.to_sec() > 0), None)
    if positive is None:
        raise ObservationPathError('OBSERVATION_CORRECTION_PATH_TIMING_INVALID')
    hold = JointTrajectoryPoint(positions=start.tolist(), velocities=[0.]*6, accelerations=[0.]*6)
    pairs = [(hold, points[positive], points[positive].time_from_start.to_sec())]
    pairs += [(a,b,(b.time_from_start-a.time_from_start).to_sec())
              for a,b in zip(points[positive:],points[positive+1:])]
    cap = PregraspCompensation.MAX_TOTAL_COUNTS*Q
    for a,b,duration in pairs:
        if not math.isfinite(duration) or duration <= 0:
            raise ObservationPathError('OBSERVATION_CORRECTION_PATH_TIMING_INVALID')
        low,high = _polynomial_bounds(segment_coefficients(a,b,duration,6),0.,1.)
        if np.any(low < start-cap-1e-9) or np.any(high > start+cap+1e-9):
            raise ObservationPathError('OBSERVATION_CORRECTION_PATH_COUNT_BUDGET')
    return dict(maximum_sdk_delta_counts=32, complete_timed_curve_checked=True)
