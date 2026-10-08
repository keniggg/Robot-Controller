"""One bounded pregrasp correction from measured IK and current following bias.

This replaces an axis-by-axis episode; it does not extend a spent episode.
The caller must certify the entire command path and obtain new feedback.
"""
from copy import deepcopy
import math

import numpy as np

from .contact_linear_compensation import compensate_cartesian_path
from .endpoint_correction import CorrectionStep, sdk_counts
from .pregrasp_compensation import PregraspCompensation
from .stationary_following import SDK_QUANTUM_RAD as Q


class PregraspFeedforward:
    def __init__(self, fk, initial, goal, **tolerances):
        from alicia_flexible_grasp.grasp.rich_plan_integrity import pose_values
        self.contract = PregraspCompensation(fk, initial, pose_values(goal), **tolerances)
        self.initial, self.goal = deepcopy(initial), deepcopy(goal)
        self.step, self.completed, self.finished = None, None, False
        self.evidence = dict(fixed_goal_pose=list(self.contract.goal_values),
            initial_sdk_counts=list(self.contract.initial_counts),
            method='measured_ik_following_feedforward', real_grasp_success=False)

    def propose(self, nominal):
        if self.step is not None or self.completed is not None or self.finished:
            raise ValueError('PREGRASP_FEEDFORWARD_ALREADY_PROPOSED')
        c = self.contract
        position, angle = c.residual(self.initial['accepted_positions_rad'])
        if position > .025 or angle > c.tolerances[1]:
            raise ValueError('PREGRASP_OUTSIDE_LOCAL_CAPTURE_RANGE')
        shifted, evidence = compensate_cartesian_path(nominal, c.fk, self.initial, self.goal)
        target = sdk_counts(shifted.joint_trajectory.points[-1].positions)
        delta = target - c.initial_counts
        if not np.any(delta) or max(abs(delta)) > c.MAX_TOTAL_COUNTS:
            raise ValueError('PREGRASP_FEEDFORWARD_TOTAL_COUNT_BUDGET')
        actual = np.asarray(self.initial['accepted_positions_rad'])
        offset = np.asarray(self.initial['sdk_positions_rad']) - actual
        # The wire target is quantized. Check its conditional endpoint too.
        predicted = (target - 2048)*Q - offset
        c.fk((target - 2048)*Q)
        predicted_position, predicted_angle = c.residual(predicted)
        if (predicted_position > min(.0015, c.tolerances[0])
                or predicted_angle > min(.005, c.tolerances[1])
                or predicted_position >= position - 1e-6):
            raise ValueError('PREGRASP_FEEDFORWARD_QUANTIZED_GOAL_INVALID')
        self.step = CorrectionStep(c.initial_counts, tuple(target.tolist()),
            tuple(sdk_counts(actual).tolist()), self.initial['sdk_stamp_ns'],
            self.initial['accepted_stamp_ns'])
        self.evidence.update(evidence, proposed_delta_counts=delta.tolist(),
            predicted_position_error_m=predicted_position,
            predicted_orientation_error_rad=predicted_angle,
            position_error_m=position, orientation_error_rad=angle,
            hardware_response_verified=False)
        return self.step, deepcopy(self.evidence)

    def propose_held_axes(self):
        """Use the existing finite guide without commanding weak Joint2/Joint5.

        This is one conditional complete correction, not a measured arrival or
        a path authorization. A failed guide never falls back to those joints.
        """
        if self.step is not None or self.completed is not None or self.finished:
            raise ValueError('PREGRASP_FEEDFORWARD_ALREADY_PROPOSED')
        c = self.contract
        actual = np.asarray(self.initial['accepted_positions_rad'], dtype=float)
        position, angle = c.residual(actual)
        if position > .025 or angle > c.tolerances[1]:
            raise ValueError('PREGRASP_OUTSIDE_LOCAL_CAPTURE_RANGE')
        guide, _ = c._remaining_budget_guide(actual)
        target = np.asarray(c.initial_counts) + guide
        if (not np.any(guide) or max(abs(guide)) > c.MAX_TOTAL_COUNTS
                or any(guide[axis] != 0 for axis in c.HELD_AXES)
                or np.any(target < 0) or np.any(target > 4095)):
            raise ValueError('PREGRASP_HELD_AXES_GUIDE_INVALID')
        offset = np.asarray(self.initial['sdk_positions_rad']) - actual
        command = (target - 2048)*Q
        predicted = command - offset
        c.fk(command)
        predicted_position, predicted_angle = c.residual(predicted)
        if (max(abs(command-predicted)) > .035
                or predicted_position > c.tolerances[0]
                or predicted_angle > c.tolerances[1]
                or predicted_position >= position - 1e-6):
            raise ValueError('PREGRASP_HELD_AXES_GUIDE_DOES_NOT_REACH_FIXED_GOAL')
        self.step = CorrectionStep(c.initial_counts, tuple(target.tolist()),
            tuple(sdk_counts(actual).tolist()), self.initial['sdk_stamp_ns'],
            self.initial['accepted_stamp_ns'])
        self.evidence.update(method='held_axes_following_feedforward',
            proposal_policy='held_axes_full_budget_guide',
            held_joints=['Joint%d' % (axis+1) for axis in c.HELD_AXES],
            sdk_minus_encoder_rad=offset.tolist(),
            measured_start_positions_rad=actual.tolist(),
            command_start_positions_rad=list(self.initial['sdk_positions_rad']),
            proposed_delta_counts=guide.tolist(),
            predicted_position_error_m=predicted_position,
            predicted_orientation_error_rad=predicted_angle,
            position_error_m=position, orientation_error_rad=angle,
            conditional_budget_guide=dict(delta_counts=guide.tolist(),
                predicted_position_error_m=predicted_position,
                predicted_orientation_error_rad=predicted_angle,
                physical_convergence_proven=False, path_authorized=False,
                globally_optimal=False),
            hardware_response_verified=False)
        return self.step, deepcopy(self.evidence)

    def committed(self, step, completed_sec):
        c = self.contract
        if (self.finished or self.completed is not None or step is None or step != self.step
                or not math.isfinite(completed_sec)
                or not c.started <= completed_sec <= c.started + c.MAX_SECONDS):
            raise ValueError('PREGRASP_FEEDFORWARD_COMMIT_INVALID')
        self.completed = int(completed_sec*1e9)
        c.expected = step.target_counts

    def evaluate(self, sample):
        if self.finished or self.completed is None:
            raise ValueError('PREGRASP_FEEDFORWARD_RESPONSE_NOT_PENDING')
        self.finished = True  # No retry or additional integration after this path.
        c = self.contract
        actual = c._validate(sample)
        if (sample['stationary_window_start_ns'] <= self.completed
                or sample['sdk_stamp_ns'] <= self.step.sdk_stamp_ns
                or sample['accepted_stamp_ns'] <= self.step.accepted_stamp_ns):
            raise ValueError('PREGRASP_RESPONSE_NOT_POST_COMMAND')
        position, angle = c.residual(sample['accepted_positions_rad'])
        delta = np.asarray(self.step.target_counts) - self.step.baseline_counts
        response = actual - self.step.measured_counts
        changed = delta != 0
        directional = response*np.sign(delta)
        reversed_axes = changed & (directional < 0)
        overshoot_axes = changed & (abs(response) > abs(delta)+2)
        uncommanded_axes = ~changed & (abs(response) > 2)
        bounded = not (np.any(reversed_axes) or np.any(overshoot_axes)
                       or np.any(uncommanded_axes))
        responded = bounded and np.any(directional[changed] >= 2)
        self.evidence.update(sdk_counts=list(c.expected), measured_counts=actual.tolist(),
            sdk_stamp_ns=sample['sdk_stamp_ns'], accepted_stamp_ns=sample['accepted_stamp_ns'],
            position_error_m=position, orientation_error_rad=angle, steps_completed=1,
            hardware_response_verified=bool(responded),
            last_step_response=dict(command_delta_counts=delta.tolist(),
                measured_delta_counts=response.tolist(), bounded_directional_response=bool(responded),
                reversed_response_joints=['Joint%d' % (i+1) for i in np.flatnonzero(reversed_axes)],
                overshoot_joints=['Joint%d' % (i+1) for i in np.flatnonzero(overshoot_axes)],
                uncommanded_motion_joints=['Joint%d' % (i+1) for i in np.flatnonzero(uncommanded_axes)],
                uncommanded_motion_limit_counts=2))
        if not responded:
            code = 'PREGRASP_NO_BOUNDED_DIRECTIONAL_RESPONSE'
        elif position >= c.residual(self.initial['accepted_positions_rad'])[0] - 1e-6:
            code = 'PREGRASP_NO_CARTESIAN_PROGRESS'
        elif position > c.tolerances[0] or angle > c.tolerances[1]:
            code = 'PREGRASP_FEEDFORWARD_MEASURED_ARRIVAL_FAILED'
        else:
            code = 'PREGRASP_MEASURED_CONVERGED'
        self.evidence['code'] = code
        return code == 'PREGRASP_MEASURED_CONVERGED', deepcopy(self.evidence)
