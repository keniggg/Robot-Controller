"""Quantized Cartesian pregrasp correction; no ROS, planning or motion authority.

The frozen visual pose is the goal. SDK targets are separate command variables.
Joint2 and Joint5 retain their initial SDK commands: the recorded pregrasps
showed absent Joint2 response and Joint5 oscillation after a small step. This
does not stabilize an already oscillating actuator. Weakly responding axes
are held for the rest of the episode; only bounded directional responses and
measured Cartesian progress can authorize another step. FK convergence is not
absolute accuracy.

An explicitly enabled Joint2 recovery may propose ONE isolated step when the
ordinary search has no improving step. It shares the original episode limits
and must demonstrate the same measured response and progress as every step.
It never enables repeated integration into a stalled joint or probes Joint5.
"""
from copy import deepcopy
from itertools import combinations, product
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


def remaining_budget_guide(fk, residual, measured, expected_counts, initial_counts, *,
                           maximum_steps, steps, maximum_step_counts, maximum_total_counts,
                           held_axes, response_gains, tolerances, cost=None):
    """Conditional endpoint heuristic, never a path or response proof.

    Looking only four counts ahead can spend an axis in the wrong
    direction and strand the later steps at another axis's total bound.
    Search the remaining ORIGINAL box, then execute only a small improving
    step toward that guide. Recompute after every actual response.
    """
    sdk = np.asarray(expected_counts)
    origin = np.asarray(initial_counts)
    reach = (maximum_steps-steps)*maximum_step_counts
    low = np.maximum(origin-maximum_total_counts-sdk, -reach)
    high = np.minimum(origin+maximum_total_counts-sdk, reach)
    delta = np.zeros(6, dtype=int)
    best_position = residual(measured)[0]
    contact_mode = cost is not None
    best = cost(measured) if contact_mode else best_position
    # Fixed work bound; coordinate search is not a global reachability test.
    for _ in range(8):
        changed = False
        for axis in range(6):
            if axis in held_axes:
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
                    fk(command)
                    predicted = measured+trial*response_gains*Q
                    if np.any(abs(command-predicted) > .035):
                        continue
                    position, angle = residual(predicted)
                    merit = cost(predicted) if contact_mode else position
                except ValueError:
                    continue
                if (angle <= tolerances[1] and merit < best-1e-9
                        and (not contact_mode or position <= tolerances[0])):
                    chosen, best, best_position = value, merit, position
            if chosen != delta[axis]:
                delta[axis], changed = chosen, True
        if not changed:
            break
    # A coordinate minimum can miss two jointly improving integer changes.
    # Preserve every already-qualified guide and the separate contact merit
    # policy. This one finite neighbourhood adds at most 96 endpoint checks
    # for the four free axes; it never enlarges the original remaining box.
    if not contact_mode and best_position > tolerances[0]:
        centre = delta.copy()
        selected, selected_key = None, None
        free = [axis for axis in range(6) if axis not in held_axes]
        for left, right in combinations(free, 2):
            for dl, dr in product((-2, -1, 1, 2), repeat=2):
                trial = centre.copy()
                trial[left] += dl
                trial[right] += dr
                if np.any(trial < low) or np.any(trial > high):
                    continue
                target = sdk+trial
                if np.any(target < 0) or np.any(target > 4095):
                    continue
                try:
                    command = (target-2048)*Q
                    fk(command)
                    predicted = measured+trial*response_gains*Q
                    if np.any(abs(command-predicted) > .035):
                        continue
                    position, angle = residual(predicted)
                except ValueError:
                    continue
                if (not math.isfinite(position) or not math.isfinite(angle)
                        or position > tolerances[0] or angle > tolerances[1]
                        or not position < best_position-1e-6):
                    continue
                # Prefer margin in both unchanged hard gates, rather than
                # buying a smaller position norm at the angle boundary.
                ratios = position/tolerances[0], angle/tolerances[1]
                key = max(ratios), sum(ratios), tuple(trial)
                if selected_key is None or key < selected_key:
                    selected, selected_key = (trial, position), key
        if selected is not None:
            delta, best_position = selected
    return delta, best_position


class PregraspCompensation:
    MAX_STEPS = 12
    MAX_SECONDS = 60.
    MAX_STEP_COUNTS = 4
    MAX_TOTAL_COUNTS = 32
    HELD_AXES = (1, 4)

    def __init__(self, fk, initial, goal_values, *, position_tolerance_m=.006,
                 orientation_tolerance_rad=math.radians(5),
                 joint2_response_probe_enabled=False, endpoint_acceptance=None):
        if not isinstance(fk, SerialUrdfFk) or tuple(fk.names) != ARM_NAMES:
            raise ValueError('PREGRASP_CANONICAL_FK_REQUIRED')
        # The operator-selected unknown-object policy may use 10 mm. The
        # default remains 6 mm; response/progress checks are independent.
        if not (0 < position_tolerance_m <= .010
                and 0 < orientation_tolerance_rad <= math.radians(5)):
            raise ValueError('PREGRASP_TOLERANCE_CANNOT_BE_RELAXED')
        if type(joint2_response_probe_enabled) is not bool:
            raise ValueError('PREGRASP_JOINT2_PROBE_CONFIG_INVALID')
        if endpoint_acceptance is not None and not callable(endpoint_acceptance):
            raise ValueError('PREGRASP_ENDPOINT_ACCEPTANCE_INVALID')
        self.endpoint_acceptance = endpoint_acceptance
        self._contact_cache = {}
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
        self.joint2_response_probe_enabled = joint2_response_probe_enabled
        self.joint2_response_probe_used = False
        self._proposed_joint2_probe = False
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
        cost = (position/self.tolerances[0])**2
        if self.endpoint_acceptance is not None:
            contact = self._contact_report(measured)
            deficit = max(0., contact['minimum_contact_overlap_m']
                          - contact['actual_overlap_m'])
            # A fixed tool0 norm alone can decrease while the finite contact
            # patch moves farther above the measured target. Within the SAME
            # position/angle and support bounds, require measured improvement
            # in a merit which also consumes the frozen contact requirement.
            cost += 4.*(deficit/contact['minimum_contact_overlap_m'])**2
            height = contact.get('height_registration_margin')
            if height is not None:
                limit = height['translation_evidence_limit_m']
                height_deficit = max(0., limit-height['maximum_symmetric_normal_translation_m'])
                if not contact['ok'] or not height['ok']:
                    # Within the existing hard position/orientation bounds,
                    # satisfying REQUIRED contact has priority over reducing
                    # an already admissible tool0 norm. A weighted sum can
                    # otherwise settle at a lower norm with a failing margin.
                    # The feasible tier has cost <= 1; no predicted sample is
                    # allowed to declare real convergence.
                    return (2. + deficit/contact['minimum_contact_overlap_m']
                            + height_deficit/limit
                            + 1e-6*(position/self.tolerances[0])**2)
        return cost

    def _contact_report(self, measured):
        key = np.asarray(measured, dtype=float).tobytes()
        if key in self._contact_cache:
            return self._contact_cache[key]
        contact = self.endpoint_acceptance(self.fk(measured))
        if (not isinstance(contact, dict) or type(contact.get('ok')) is not bool
                or contact.get('code') not in ('MEASURED_CONTACT_GEOMETRY_OK',
                                                'MEASURED_CONTACT_PATCH_MISS')):
            raise ValueError('PREGRASP_ENDPOINT_ACCEPTANCE_INVALID_OR_UNSAFE')
        try:
            minimum = float(contact['minimum_contact_overlap_m'])
            overlap = float(contact['actual_overlap_m'])
            clearance = float(contact['minimum_support_clearance_m'])
        except (KeyError, TypeError, ValueError, OverflowError) as exc:
            raise ValueError('PREGRASP_ENDPOINT_ACCEPTANCE_INVALID') from exc
        if (not all(math.isfinite(v) for v in (minimum, overlap, clearance))
                or minimum <= 0. or overlap < 0. or clearance < .003-1e-9
                or contact['ok'] != (overlap+1e-9 >= minimum)
                or contact['ok'] != (contact['code'] == 'MEASURED_CONTACT_GEOMETRY_OK')):
            raise ValueError('PREGRASP_ENDPOINT_ACCEPTANCE_INVALID_OR_UNSAFE')
        height = contact.get('height_registration_margin')
        if 'height_registration_margin' in contact:
            try:
                if not isinstance(height, dict):
                    raise ValueError('height evidence must be a mapping')
                limit = height['translation_evidence_limit_m']
                margin = height['maximum_symmetric_normal_translation_m']
                expected_ok = bool(contact['ok'] and margin+1e-12 >= limit)
                if (height.get('evaluated') is not True
                        or type(height.get('ok')) is not bool
                        or type(limit) not in (int, float) or type(margin) not in (int, float)
                        or not math.isfinite(limit) or not math.isfinite(margin)
                        or limit <= 0. or margin < 0.
                        or height['ok'] != expected_ok
                        or height.get('covers_entire_normal_translation_interval') is not expected_ok
                        or height.get('code') != ('CONTACT_HEIGHT_NORMAL_MARGIN_OK'
                            if expected_ok else 'CONTACT_HEIGHT_NORMAL_MARGIN_INSUFFICIENT')):
                    raise ValueError('inconsistent height margin evidence')
            except (KeyError, TypeError, ValueError) as exc:
                raise ValueError('PREGRASP_CONTACT_HEIGHT_EVIDENCE_INVALID') from exc
        # Bounded to this one measured-response evaluation. A fresh accepted
        # sample/opening always clears it, and it cannot certify real response.
        if len(self._contact_cache) < 8192:
            self._contact_cache[key] = contact
        return contact

    def _improves(self, predicted, position, previous_cost):
        p, angle = self.residual(predicted)
        if self.endpoint_acceptance is None:
            return p < position-1e-6 and self._cost(predicted) < previous_cost-1e-4
        if angle > self.tolerances[1]:
            return False
        return (p <= self.tolerances[0]
                and self._cost(predicted) < previous_cost-1e-4)

    def _remaining_budget_guide(self, measured):
        return remaining_budget_guide(self.fk, self.residual, measured,
            self.expected, self.initial_counts, maximum_steps=self.MAX_STEPS,
            steps=self.steps, maximum_step_counts=self.MAX_STEP_COUNTS,
            maximum_total_counts=self.MAX_TOTAL_COUNTS, held_axes=self.held_axes,
            response_gains=self.response_gains, tolerances=self.tolerances,
            cost=self._cost if self.endpoint_acceptance is not None else None)

    def _guided_step(self, measured, actual, guide, position):
        choices = []
        for axis, distance in enumerate(guide):
            if axis in self.held_axes or distance == 0:
                choices.append((0,))
            else:
                sign = int(np.sign(distance))
                choices.append(tuple(sign*v for v in (0, 2, 4) if v <= abs(distance)))
        best_key, selected = None, np.zeros(6, dtype=int)
        initial_cost = self._cost(measured)
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
                predicted = measured+trial*self.response_gains*Q
                p, a = self.residual(predicted)
                improves = self._improves(predicted, position, initial_cost)
                merit = self._cost(predicted) if self.endpoint_acceptance is not None else p
            except ValueError:
                continue
            if a > self.tolerances[1] or not improves:
                continue
            # Prefer progress toward the whole-budget solution. Immediate
            # position (or contact merit) breaks ties; real feedback must later
            # demonstrate the same improvement before another proposal.
            key = (int(np.sum((guide-trial)**2)), merit)
            if best_key is None or key < best_key:
                best_key, selected = key, trial
        return selected

    def _contact_local_step(self, measured, actual, position):
        """A fixed-size simultaneous search when the coordinate guide stalls.

        The narrow physical contact band and support plane can require two
        axes to move together. Every endpoint still obeys the original single
        step/episode box; no intermediate coordinate trial authorizes motion.
        """
        selected = np.zeros(6, dtype=int)
        if self.endpoint_acceptance is None:
            return selected
        choices = [(0,) if i in self.held_axes else (-4, -2, 0, 2, 4)
                   for i in range(6)]
        best = self._cost(measured)
        for values in product(*choices):
            trial = np.asarray(values, dtype=int)
            active = trial != 0
            if not np.any(active):
                continue
            target = np.asarray(self.expected)+trial
            minimum_response = actual+2*np.sign(trial)
            if (np.any(target < 0) or np.any(target > 4095)
                    or max(abs(target-np.asarray(self.initial_counts))) > self.MAX_TOTAL_COUNTS
                    or np.any(abs(target-actual)[active] < 2)
                    or np.any(abs(target-minimum_response)*Q > .035)):
                continue
            try:
                self.fk((target-2048)*Q)
                predicted = measured+trial*self.response_gains*Q
                if not self._improves(predicted, position, best):
                    continue
                cost = self._cost(predicted)
            except ValueError:
                continue
            selected, best = trial, cost
        return selected

    def _joint2_response_probe(self, measured, actual, position):
        """One conditional recovery, not a claim that this actuator responds."""
        selected = np.zeros(6, dtype=int)
        if not self.joint2_response_probe_enabled or self.joint2_response_probe_used:
            return selected
        best = position
        for amount in (-4, -2, 2, 4):
            trial = np.zeros(6, dtype=int)
            trial[1] = amount
            target = np.asarray(self.expected) + trial
            minimum_response = actual + 2 * np.sign(trial)
            if (np.any(target < 0) or np.any(target > 4095)
                    or max(abs(target - np.asarray(self.initial_counts))) > self.MAX_TOTAL_COUNTS
                    or abs(target[1] - actual[1]) < 2
                    or np.any(abs(target - minimum_response) * Q > .035)):
                continue
            try:
                self.fk((target - 2048) * Q)
                p, angle = self.residual(measured + trial * Q)
            except ValueError:
                continue
            if (angle <= self.tolerances[1] and p < best - 1e-6
                    and (p / self.tolerances[0])**2 < (position / self.tolerances[0])**2 - 1e-4):
                selected, best = trial, p
        return selected

    def evaluate(self, sample):
        self._contact_cache.clear()
        actual = self._validate(sample)
        measured = np.asarray(sample['accepted_positions_rad'], dtype=float)
        position, angle = self.residual(measured)
        evidence = dict(fixed_goal_pose=list(self.goal_values), initial_sdk_counts=list(self.initial_counts),
            sdk_counts=list(self.expected), measured_counts=actual.tolist(),
            position_error_m=position, orientation_error_rad=angle,
            sdk_stamp_ns=sample['sdk_stamp_ns'], accepted_stamp_ns=sample['accepted_stamp_ns'],
            steps_completed=self.steps, held_joints=[ARM_NAMES[i] for i in sorted(self.held_axes)],
            response_gains=self.response_gains.tolist(), real_grasp_success=False,
            joint2_response_probe_enabled=self.joint2_response_probe_enabled,
            joint2_response_probe_used=self.joint2_response_probe_used,
            saturated_joints=[ARM_NAMES[i] for i in range(6)
                              if abs(self.expected[i]-self.initial_counts[i]) >= self.MAX_TOTAL_COUNTS])
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
            try:
                improves = self._improves(measured, previous_position, previous_cost)
            except ValueError as exc:
                self._fail(str(exc))
            if not improves:
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
        self._proposed_joint2_probe = False
        contact_accepted = True
        if self.endpoint_acceptance is not None:
            # A small tool0 residual is not proof that the finite finger patch
            # still overlaps a small object. Use the same frozen contact band
            # at every REAL feedback sample; predicted points never converge.
            try:
                contact = self._contact_report(measured)
            except ValueError as exc:
                self._fail(str(exc))
            evidence['measured_contact'] = deepcopy(contact)
            evidence['contact_merit'] = self._cost(measured)
            self.last_evidence = deepcopy(evidence)
            contact_accepted = contact['ok'] and contact.get(
                'height_registration_margin', {}).get('ok', True)
        if position <= self.tolerances[0] and angle <= self.tolerances[1] and contact_accepted:
            return 'PREGRASP_MEASURED_CONVERGED', None, evidence
        if position > .025 or angle > self.tolerances[1]:
            self._fail('PREGRASP_OUTSIDE_LOCAL_CAPTURE_RANGE')
        if self.steps >= self.MAX_STEPS:
            self._fail('PREGRASP_STEP_BUDGET')
        guide, guide_error = self._remaining_budget_guide(measured)
        delta = self._guided_step(measured, actual, guide, position)
        if not np.any(delta):
            delta = self._contact_local_step(measured, actual, position)
        if not np.any(delta):
            delta = self._joint2_response_probe(measured, actual, position)
            self._proposed_joint2_probe = bool(np.any(delta))
        initial_cost = self._cost(measured)
        predicted = measured+delta*self.response_gains*Q
        predicted_position, predicted_angle = self.residual(predicted)
        try:
            improves = self._improves(predicted, position, initial_cost)
        except ValueError:
            improves = False
        if not np.any(delta) or not improves:
            self._fail('PREGRASP_NO_BOUNDED_IMPROVING_STEP')
        self.proposed = CorrectionStep(self.expected, tuple((np.asarray(self.expected)+delta).tolist()),
            tuple(actual.tolist()), sample['sdk_stamp_ns'], sample['accepted_stamp_ns'])
        evidence['proposed_delta_counts'] = delta.tolist()
        evidence['step_kind'] = ('joint2_response_probe' if self._proposed_joint2_probe
                                 else 'bounded_cartesian_correction')
        evidence['predicted_position_error_m'] = predicted_position
        evidence['predicted_orientation_error_rad'] = predicted_angle
        if self.endpoint_acceptance is not None:
            evidence['predicted_contact_merit'] = self._cost(predicted)
            evidence['predicted_contact'] = deepcopy(self._contact_report(predicted))
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
        if self._proposed_joint2_probe:
            self.joint2_response_probe_used = True
        self.pending = step, int(completed_sec*1e9), self.proposed_cost, self.proposed_position
        self.proposed = None
        self.steps += 1
