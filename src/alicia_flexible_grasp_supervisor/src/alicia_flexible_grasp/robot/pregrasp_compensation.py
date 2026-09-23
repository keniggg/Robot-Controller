"""Quantized Cartesian pregrasp correction; no ROS, planning or motion authority.

The frozen visual pose is the goal. SDK targets are separate command variables.
Joint2 and Joint5 retain their initial SDK commands: the recorded pregrasps
showed absent Joint2 response and Joint5 oscillation after a small step. This
does not stabilize an already oscillating actuator. Weakly responding axes
are held for the rest of the episode; only bounded directional responses and
measured Cartesian progress can authorize another step. FK convergence is not
absolute accuracy.
"""
from copy import deepcopy
from itertools import product
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
    HELD_AXES = (1, 4)

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
        self.held_axes = set(self.HELD_AXES)
        self.response_gains = np.ones(6)
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
        position, _ = self.residual(measured)
        # Orientation is a hard admissibility constraint. Do not sacrifice
        # required position accuracy to improve an already admissible angle.
        return (position/self.tolerances[0])**2

    def _remaining_budget_guide(self, measured):
        """Conditional endpoint heuristic, never a path or response proof.

        Looking only four counts ahead can spend an axis in the wrong
        direction and strand the later steps at another axis's total bound.
        Search the remaining ORIGINAL box, then execute only a small improving
        step toward that guide. Recompute after every actual response.
        """
        sdk = np.asarray(self.expected)
        origin = np.asarray(self.initial_counts)
        reach = (self.MAX_STEPS-self.steps)*self.MAX_STEP_COUNTS
        low = np.maximum(origin-self.MAX_TOTAL_COUNTS-sdk, -reach)
        high = np.minimum(origin+self.MAX_TOTAL_COUNTS-sdk, reach)
        delta = np.zeros(6, dtype=int)
        best = self.residual(measured)[0]
        # Fixed work bound; coordinate search is not a global reachability test.
        for _ in range(8):
            changed = False
            for axis in range(6):
                if axis in self.held_axes:
                    continue
                chosen = delta[axis]
                for value in range(int(low[axis]), int(high[axis])+1, 2):
                    trial = delta.copy()
                    trial[axis] = value
                    target = sdk+trial
                    if np.any(target < 0) or np.any(target > 4095):
                        continue
                    try:
                        command = (target-2048)*Q
                        self.fk(command)
                        predicted = measured+trial*self.response_gains*Q
                        if np.any(abs(command-predicted) > .035):
                            continue
                        position, angle = self.residual(predicted)
                    except ValueError:
                        continue
                    if angle <= self.tolerances[1] and position < best-1e-9:
                        chosen, best = value, position
                if chosen != delta[axis]:
                    delta[axis], changed = chosen, True
            if not changed:
                break
        return delta, best

    def _guided_step(self, measured, actual, guide, position):
        choices = []
        for axis, distance in enumerate(guide):
            if axis in self.held_axes or distance == 0:
                choices.append((0,))
            else:
                sign = int(np.sign(distance))
                choices.append(tuple(sign*v for v in (0, 2, 4) if v <= abs(distance)))
        best_key, selected = None, np.zeros(6, dtype=int)
        initial_cost = (position/self.tolerances[0])**2
        for values in product(*choices):
            trial = np.asarray(values, dtype=int)
            if not np.any(trial):
                continue
            target = np.asarray(self.expected)+trial
            active = trial != 0
            if (np.any(target < 0) or np.any(target > 4095)
                    or max(abs(target-np.asarray(self.initial_counts))) > self.MAX_TOTAL_COUNTS
                    or np.any(abs(target-actual)[active] < 2)):
                continue
            minimum_response = actual+2*np.sign(trial)
            if np.any(abs(target-minimum_response)*Q > .035):
                continue
            try:
                self.fk((target-2048)*Q)
                p, a = self.residual(measured+trial*self.response_gains*Q)
            except ValueError:
                continue
            if (a > self.tolerances[1] or p >= position-1e-6
                    or (p/self.tolerances[0])**2 >= initial_cost-1e-4):
                continue
            # Prefer progress toward the whole-budget solution; immediate FK
            # error only breaks ties. Every step must still improve measured FK.
            key = (int(np.sum((guide-trial)**2)), p)
            if best_key is None or key < best_key:
                best_key, selected = key, trial
        return selected

    def evaluate(self, sample):
        actual = self._validate(sample)
        measured = np.asarray(sample['accepted_positions_rad'], dtype=float)
        position, angle = self.residual(measured)
        evidence = dict(fixed_goal_pose=list(self.goal_values), initial_sdk_counts=list(self.initial_counts),
            sdk_counts=list(self.expected), measured_counts=actual.tolist(),
            position_error_m=position, orientation_error_rad=angle,
            sdk_stamp_ns=sample['sdk_stamp_ns'], accepted_stamp_ns=sample['accepted_stamp_ns'],
            steps_completed=self.steps, held_joints=[ARM_NAMES[i] for i in sorted(self.held_axes)],
            response_gains=self.response_gains.tolist(), real_grasp_success=False)
        self.last_evidence = deepcopy(evidence)
        if self.pending is not None:
            step, completed_ns, previous_cost, previous_position = self.pending
            if (sample['stationary_window_start_ns'] <= completed_ns
                    or sample['sdk_stamp_ns'] <= step.sdk_stamp_ns
                    or sample['accepted_stamp_ns'] <= step.accepted_stamp_ns):
                self._fail('PREGRASP_RESPONSE_NOT_POST_COMMAND')
            delta = np.asarray(step.target_counts)-step.baseline_counts
            response = actual-step.measured_counts
            changed = delta != 0
            directional = response*np.sign(delta)
            bounded = not (np.any(directional[changed] < 0)
                      or np.any(abs(response[changed]) > abs(delta[changed])+2)
                      or np.any(abs(response[~changed]) > 2))
            strong = changed & (directional >= 2)
            weak = changed & (directional < 2)
            evidence['last_step_response'] = dict(command_delta_counts=delta.tolist(),
                measured_delta_counts=response.tolist(),
                bounded_directional_response=bool(bounded and not np.any(weak)),
                partial_bounded_response=bool(bounded and np.any(strong) and np.any(weak)),
                newly_held_joints=[ARM_NAMES[i] for i in np.flatnonzero(weak)])
            self.last_evidence = deepcopy(evidence)
            if not bounded or not np.any(strong):
                self._fail('PREGRASP_NO_BOUNDED_DIRECTIONAL_RESPONSE')
            if self._cost(measured) >= previous_cost-1e-4 or position >= previous_position-1e-6:
                self._fail('PREGRASP_NO_CARTESIAN_PROGRESS')
            # Do not integrate another command into an axis with <2 counts
            # of directional response. Keep its last SDK word, not its measured
            # angle, and monitor it as an uncommanded axis on later steps.
            self.held_axes.update(int(i) for i in np.flatnonzero(weak))
            self.response_gains[strong] = np.minimum(1., directional[strong]/abs(delta[strong]))
            evidence['held_joints'] = [ARM_NAMES[i] for i in sorted(self.held_axes)]
            evidence['response_gains'] = self.response_gains.tolist()
            self.last_evidence = deepcopy(evidence)
            self.pending = None
        self.proposed = None
        if position <= self.tolerances[0] and angle <= self.tolerances[1]:
            return 'PREGRASP_MEASURED_CONVERGED', None, evidence
        if position > .025 or angle > self.tolerances[1]:
            self._fail('PREGRASP_OUTSIDE_LOCAL_CAPTURE_RANGE')
        if self.steps >= self.MAX_STEPS:
            self._fail('PREGRASP_STEP_BUDGET')
        guide, guide_error = self._remaining_budget_guide(measured)
        delta = self._guided_step(measured, actual, guide, position)
        initial_cost = self._cost(measured)
        predicted_position, predicted_angle = self.residual(measured+delta*self.response_gains*Q)
        if (not np.any(delta) or (predicted_position/self.tolerances[0])**2 >= initial_cost-1e-4
                or predicted_position >= position-1e-6):
            self._fail('PREGRASP_NO_BOUNDED_IMPROVING_STEP')
        self.proposed = CorrectionStep(self.expected, tuple((np.asarray(self.expected)+delta).tolist()),
            tuple(actual.tolist()), sample['sdk_stamp_ns'], sample['accepted_stamp_ns'])
        evidence['proposed_delta_counts'] = delta.tolist()
        evidence['predicted_position_error_m'] = predicted_position
        evidence['predicted_orientation_error_rad'] = predicted_angle
        evidence['conditional_budget_guide'] = dict(delta_counts=guide.tolist(),
            predicted_position_error_m=guide_error, physical_convergence_proven=False,
            path_authorized=False, globally_optimal=False)
        self.last_evidence = deepcopy(evidence)
        self.proposed_cost = initial_cost
        self.proposed_position = position
        return 'PREGRASP_STEP_PROPOSED', self.proposed, evidence

    def committed(self, step, completed_sec):
        if (self.error or self.pending is not None or step is None or step != self.proposed
                or not math.isfinite(completed_sec)
                or not self.started <= completed_sec <= self.started+self.MAX_SECONDS):
            self._fail('PREGRASP_COMMIT_INVALID')
        self.expected = step.target_counts
        self.pending = step, int(completed_sec*1e9), self.proposed_cost, self.proposed_position
        self.proposed = None
        self.steps += 1
