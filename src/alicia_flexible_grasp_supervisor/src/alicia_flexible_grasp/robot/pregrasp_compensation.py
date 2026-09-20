"""Quantized Cartesian pregrasp correction; no ROS, planning or motion authority.

The frozen visual pose is the goal. SDK targets are separate command variables.
Joint2 is held at its initial command: this strategy does not integrate into the
axis which failed to respond in the recorded pregrasp. Every other commanded
axis must respond before another step. FK convergence is not absolute accuracy.
"""
from copy import deepcopy
import math

import numpy as np
from tf.transformations import quaternion_matrix

from .endpoint_correction import CorrectionStep, sdk_counts
from .stationary_following import ARM_NAMES, SDK_QUANTUM_RAD as Q, SerialUrdfFk


def pose_matrix(values):
    values = np.asarray(values, dtype=float)
    if (values.shape != (7,) or not np.all(np.isfinite(values))
            or abs(np.linalg.norm(values[3:]) - 1.) > 1e-3):
        raise ValueError('PREGRASP_INVALID_FIXED_POSE')
    result = quaternion_matrix(values[3:])
    result[:3, 3] = values[:3]
    return result


class PregraspCompensation:
    MAX_STEPS = 12
    MAX_SECONDS = 60.
    MAX_STEP_COUNTS = 4
    MAX_TOTAL_COUNTS = 32
    HELD_AXES = (1,)

    def __init__(self, fk, initial, goal_values, *, position_tolerance_m=.006,
                 orientation_tolerance_rad=math.radians(5)):
        if not isinstance(fk, SerialUrdfFk) or tuple(fk.names) != ARM_NAMES:
            raise ValueError('PREGRASP_CANONICAL_FK_REQUIRED')
        if not (0 < position_tolerance_m <= .006
                and 0 < orientation_tolerance_rad <= math.radians(5)):
            raise ValueError('PREGRASP_TOLERANCE_CANNOT_BE_RELAXED')
        self.fk, self.goal = fk, pose_matrix(goal_values)
        self.goal.setflags(write=False)
        self.goal_values = tuple(float(v) for v in goal_values)
        self.tolerances = float(position_tolerance_m), float(orientation_tolerance_rad)
        self.epoch = initial['epoch_ns']
        self.started = float(initial['stamp_sec'])
        self.initial_counts = tuple(int(v) for v in sdk_counts(initial['sdk_positions_rad']))
        self.expected = self.initial_counts
        self.steps, self.pending, self.proposed = 0, None, None
        self.error, self.last_evidence = '', None
        self._validate(initial)

    def _fail(self, code):
        self.error = code
        raise ValueError(code)

    def _validate(self, sample):
        if self.error:
            raise ValueError(self.error)
        if (sample.get('measurement_kind') != 'stationary_sdk_following_not_executed_endpoint'
                or sample.get('model_sha256') != self.fk.model_sha256
                or sample.get('epoch_ns') != self.epoch
                or tuple(sample.get('joint_names', ())) != ARM_NAMES):
            self._fail('PREGRASP_CONTEXT_CHANGED')
        now = float(sample['stamp_sec'])
        stamps = (sample['sdk_stamp_ns'], sample['accepted_stamp_ns'])
        if (not math.isfinite(now) or not self.started <= now <= self.started + self.MAX_SECONDS
                or type(self.epoch) is not int or self.epoch <= 0
                or any(type(s) is not int or s < self.epoch or not 0 <= now-s*1e-9 <= .5 for s in stamps)
                or sample['stationary_window_end_ns']-sample['stationary_window_start_ns'] < 300_000_000):
            self._fail('PREGRASP_STALE_OR_EXPIRED')
        actual = sdk_counts(sample['accepted_positions_rad'])
        if tuple(sdk_counts(sample['sdk_positions_rad'])) != self.expected:
            self._fail('PREGRASP_SDK_TARGET_CHANGED')
        if max(abs(actual-np.asarray(self.expected))) * Q > .035:
            self._fail('PREGRASP_FOLLOWING_OUTSIDE_CONTRACT')
        return actual

    def residual(self, measured):
        transform = self.fk(measured)
        position = float(np.linalg.norm(self.goal[:3, 3]-transform[:3, 3]))
        orientation = math.acos(float(np.clip(
            (np.trace(self.goal[:3, :3].T @ transform[:3, :3])-1)/2, -1., 1.)))
        return position, orientation

    def _cost(self, measured):
        position, angle = self.residual(measured)
        return (position/self.tolerances[0])**2 + (angle/self.tolerances[1])**2

    def evaluate(self, sample):
        actual = self._validate(sample)
        measured = np.asarray(sample['accepted_positions_rad'], dtype=float)
        position, angle = self.residual(measured)
        evidence = dict(fixed_goal_pose=list(self.goal_values), initial_sdk_counts=list(self.initial_counts),
            sdk_counts=list(self.expected), measured_counts=actual.tolist(),
            position_error_m=position, orientation_error_rad=angle,
            sdk_stamp_ns=sample['sdk_stamp_ns'], accepted_stamp_ns=sample['accepted_stamp_ns'],
            steps_completed=self.steps, held_joints=['Joint2'], real_grasp_success=False)
        self.last_evidence = deepcopy(evidence)
        if self.pending is not None:
            step, completed_ns, previous_cost = self.pending
            if (sample['stationary_window_start_ns'] <= completed_ns
                    or sample['sdk_stamp_ns'] <= step.sdk_stamp_ns
                    or sample['accepted_stamp_ns'] <= step.accepted_stamp_ns):
                self._fail('PREGRASP_RESPONSE_NOT_POST_COMMAND')
            delta = np.asarray(step.target_counts)-step.baseline_counts
            response = actual-step.measured_counts
            changed = delta != 0
            ok = not (np.any(response[changed]*np.sign(delta[changed]) < 2)
                      or np.any(abs(response[changed]) > abs(delta[changed])+2)
                      or np.any(abs(response[~changed]) > 2))
            evidence['last_step_response'] = dict(command_delta_counts=delta.tolist(),
                measured_delta_counts=response.tolist(), bounded_directional_response=bool(ok))
            self.last_evidence = deepcopy(evidence)
            if not ok:
                self._fail('PREGRASP_NO_BOUNDED_DIRECTIONAL_RESPONSE')
            if self._cost(measured) >= previous_cost-1e-4:
                self._fail('PREGRASP_NO_CARTESIAN_PROGRESS')
            self.pending = None
        self.proposed = None
        if position <= self.tolerances[0] and angle <= self.tolerances[1]:
            return 'PREGRASP_MEASURED_CONVERGED', None, evidence
        if position > .025 or angle > self.tolerances[1]:
            self._fail('PREGRASP_OUTSIDE_LOCAL_CAPTURE_RANGE')
        if self.steps >= self.MAX_STEPS:
            self._fail('PREGRASP_STEP_BUDGET')
        # Finite coordinate search directly on the SDK grid. The selected
        # increment is applied to the SDK baseline, while predictions use the
        # measured pose. Never replace the SDK baseline with encoder feedback.
        delta = np.zeros(6, dtype=int)
        best = self._cost(measured)
        initial_cost = best
        for _ in range(3):
            changed = False
            for axis in range(6):
                if axis in self.HELD_AXES:
                    continue
                chosen = delta[axis]
                for value in (-4, -2, 0, 2, 4):
                    trial = delta.copy()
                    trial[axis] = value
                    target = np.asarray(self.expected)+trial
                    if (np.any(target < 0) or np.any(target > 4095)
                            or max(abs(target-np.asarray(self.initial_counts))) > self.MAX_TOTAL_COUNTS):
                        continue
                    try:
                        self.fk((target-2048)*Q)  # command as well as measured limits
                        predicted = measured+trial*Q
                        p, a = self.residual(predicted)
                        score = self._cost(predicted)
                    except ValueError:
                        continue
                    if a <= self.tolerances[1] and score < best-1e-9:
                        chosen, best = value, score
                if chosen != delta[axis]:
                    delta[axis], changed = chosen, True
            if not changed:
                break
        if not np.any(delta) or best >= initial_cost-1e-4:
            self._fail('PREGRASP_NO_BOUNDED_IMPROVING_STEP')
        self.proposed = CorrectionStep(self.expected, tuple((np.asarray(self.expected)+delta).tolist()),
            tuple(actual.tolist()), sample['sdk_stamp_ns'], sample['accepted_stamp_ns'])
        evidence['proposed_delta_counts'] = delta.tolist()
        evidence['predicted_position_error_m'], evidence['predicted_orientation_error_rad'] = (
            self.residual(measured+delta*Q))
        self.last_evidence = deepcopy(evidence)
        self.proposed_cost = initial_cost
        return 'PREGRASP_STEP_PROPOSED', self.proposed, evidence

    def committed(self, step, completed_sec):
        if (self.error or self.pending is not None or step is None or step != self.proposed
                or not math.isfinite(completed_sec)
                or not self.started <= completed_sec <= self.started+self.MAX_SECONDS):
            self._fail('PREGRASP_COMMIT_INVALID')
        self.expected = step.target_counts
        self.pending = step, int(completed_sec*1e9), self.proposed_cost
        self.proposed = None
        self.steps += 1
