"""Bounded fixed-goal feedback correction, with no ROS or hardware authority.

The original physical goal never follows the compensated SDK command. A caller
must independently authorize and collision-check EVERY proposed step. A failed
response ends the episode; neither retries nor rollback are implicit commands.
"""
from dataclasses import dataclass
from copy import deepcopy
import math

import numpy as np

from .stationary_following import ARM_NAMES, SDK_QUANTUM_RAD, SerialUrdfFk


def sdk_counts(positions):
    q = np.asarray(positions, dtype=float)
    if q.shape != (6,) or not np.all(np.isfinite(q)) or np.any(np.abs(q) > math.pi):
        raise ValueError('six finite in-domain arm positions required')
    return np.clip(np.floor((q + math.pi) / SDK_QUANTUM_RAD + .5), 0, 4095).astype(int)


@dataclass(frozen=True)
class CorrectionStep:
    baseline_counts: tuple
    target_counts: tuple
    measured_counts: tuple
    sdk_stamp_ns: int
    accepted_stamp_ns: int

    @property
    def positions(self):
        return tuple((value - 2048) * SDK_QUANTUM_RAD for value in self.target_counts)


class EndpointCorrection:
    """One <=30s, <=6-step episode, <=4 counts/step and <=12 total/axis.

    A 2-count floor accommodates encoder quantization. No gain adaptation or
    blind integration across a nonresponding joint. These bounds cannot be
    widened through runtime parameters. Model-space arrival is not calibration
    or grasp success; temperature/collision/ownership gates belong to caller.
    """
    def __init__(self, fk, initial, *, position_tolerance_m=.006,
                 orientation_tolerance_rad=math.radians(5)):
        if not isinstance(fk, SerialUrdfFk) or tuple(fk.names) != ARM_NAMES:
            raise ValueError('canonical six-axis FK required')
        if not (math.isfinite(position_tolerance_m) and 0 < position_tolerance_m <= .006
                and math.isfinite(orientation_tolerance_rad)
                and 0 < orientation_tolerance_rad <= math.radians(5)):
            raise ValueError('cannot relax the measured contact tolerance')
        self.fk = fk
        self.epoch = initial['epoch_ns']
        self.started = float(initial['stamp_sec'])
        self.expected = tuple(int(v) for v in sdk_counts(initial['sdk_positions_rad']))
        self.goal_counts = self.expected
        self.goal = np.array(initial['sdk_positions_rad'], dtype=float, copy=True)
        self.goal.setflags(write=False)
        self.tolerance = position_tolerance_m, orientation_tolerance_rad
        self.steps = 0
        self.pending = None
        self.proposed = None
        self.error = ''
        self.last_evidence = None
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
            self._fail('ENDPOINT_CORRECTION_CONTEXT_CHANGED')
        now = float(sample['stamp_sec'])
        stamps = [sample['sdk_stamp_ns'], sample['accepted_stamp_ns']]
        if (not math.isfinite(now) or not self.started <= now <= self.started + 30.
                or type(self.epoch) is not int or self.epoch <= 0
                or any(type(s) is not int or s < self.epoch or not 0 <= now - s*1e-9 <= .5
                       for s in stamps)
                or sample['stationary_window_end_ns'] - sample['stationary_window_start_ns'] < 300_000_000):
            self._fail('ENDPOINT_CORRECTION_STALE_OR_EXPIRED')
        measured = sdk_counts(sample['accepted_positions_rad'])
        if tuple(sdk_counts(sample['sdk_positions_rad'])) != self.expected:
            self._fail('ENDPOINT_CORRECTION_SDK_TARGET_CHANGED')
        if np.max(np.abs(measured - np.asarray(self.expected))) * SDK_QUANTUM_RAD > .035:
            self._fail('ENDPOINT_CORRECTION_FOLLOWING_OUTSIDE_CONTRACT')
        return measured

    def evaluate(self, sample):
        actual_counts = self._validate(sample)
        # Preserve the latest VALID measured state even when the response gate
        # rejects it. Previously the gateway retained pre-step counts/error on
        # NO_BOUNDED_DIRECTIONAL_RESPONSE, while nesting a newer raw sample.
        # Measurement is diagnostic only; it must not bypass the response gate,
        # even if this state is geometrically inside the arrival tolerance.
        desired = self.fk(self.goal)
        actual = self.fk(sample['accepted_positions_rad'])
        position = float(np.linalg.norm(actual[:3, 3] - desired[:3, 3]))
        orientation = math.acos(float(np.clip(
            (np.trace(desired[:3, :3].T @ actual[:3, :3]) - 1) / 2, -1., 1.)))
        evidence = {
            'fixed_goal_counts': list(self.goal_counts),
            'current_sdk_counts': list(self.expected),
            'measured_counts': actual_counts.tolist(),
            'fixed_goal_position_error_m': position,
            'fixed_goal_orientation_error_rad': orientation,
            'sdk_following_error_m': float(sample['position_error_m']),
            'steps_completed': self.steps,
            'grasp_or_calibration_success': False,
            'measurement_stamp_sec': sample['stamp_sec'],
            'sdk_stamp_ns': sample['sdk_stamp_ns'],
            'accepted_stamp_ns': sample['accepted_stamp_ns'],
        }
        self.last_evidence = deepcopy(evidence)
        if self.pending is not None:
            step, completed = self.pending
            if (sample['stationary_window_start_ns'] <= completed
                    or sample['sdk_stamp_ns'] <= step.sdk_stamp_ns
                    or sample['accepted_stamp_ns'] <= step.accepted_stamp_ns):
                self._fail('ENDPOINT_CORRECTION_RESPONSE_NOT_POST_COMMAND')
            delta = np.asarray(step.target_counts) - step.baseline_counts
            response = actual_counts - step.measured_counts
            changed = delta != 0
            response_ok = not (np.any(response[changed] * np.sign(delta[changed]) < 2)
                    or np.any(np.abs(response[changed]) > np.abs(delta[changed]) + 2)
                    or np.any(np.abs(response[~changed]) > 2))
            evidence['last_step_response'] = {
                'command_delta_counts': delta.tolist(),
                'measured_delta_counts': response.tolist(),
                'bounded_directional_response': bool(response_ok),
            }
            self.last_evidence = deepcopy(evidence)
            if not response_ok:
                self._fail('ENDPOINT_CORRECTION_NO_BOUNDED_DIRECTIONAL_RESPONSE')
            self.pending = None
        self.proposed = None
        if position <= self.tolerance[0] and orientation <= self.tolerance[1]:
            return 'MEASURED_ENDPOINT_CONVERGED', None, evidence
        if self.steps >= 6:
            self._fail('ENDPOINT_CORRECTION_STEP_BUDGET')
        error = np.asarray(self.goal_counts) - actual_counts
        delta = np.where(np.abs(error) > 2, np.clip(error, -4, 4), 0)
        target = np.asarray(self.expected) + delta
        if not np.any(delta):
            self._fail('ENDPOINT_CORRECTION_QUANTIZATION_FLOOR')
        if (np.any(target < 0) or np.any(target > 4095)
                or np.max(np.abs(target - self.goal_counts)) > 12):
            self._fail('ENDPOINT_CORRECTION_TOTAL_BUDGET')
        self.proposed = CorrectionStep(self.expected, tuple(int(v) for v in target),
                                      tuple(int(v) for v in actual_counts),
                                      sample['sdk_stamp_ns'], sample['accepted_stamp_ns'])
        return 'ENDPOINT_CORRECTION_STEP_PROPOSED', self.proposed, evidence

    def committed(self, step, completed_sec):
        """Record exactly the independently executed step, not an SDK ACK."""
        if (self.error or self.pending is not None or step != self.proposed
                or step is None or not math.isfinite(completed_sec)
                or not self.started <= completed_sec <= self.started + 30.):
            self._fail('ENDPOINT_CORRECTION_COMMIT_INVALID')
        self.expected = step.target_counts
        self.pending = (step, int(completed_sec * 1e9))
        self.proposed = None
        self.steps += 1
