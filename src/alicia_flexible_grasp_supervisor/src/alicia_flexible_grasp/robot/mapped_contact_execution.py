"""One physical-reference episode inside an already authenticated contact task.

The model is a frozen feedforward calibration, not replacement feedback.  This
adapter never creates a task lifetime, renews a failed episode, or dispatches a
gripper command.  A completed episode releases to the identical SDK words
before its caller may advance the task.
"""
from copy import deepcopy
import hashlib
import json
import math
import time
import uuid

import numpy as np
import rospy

from .following_model import (
    source_bias_model, source_bias_model_record, validate_source_bias_model,
    following_offset, following_evidence,
)
from .observation_path_guard import ObservationPathError, SerialUrdfFk
from .physical_reference_contract import (
    physical_plan_from_wire, physical_reference_from_wire, bind_dual_reference,
    message_bytes,
)
from .physical_reference_client import PhysicalReferenceLease, MappedTrajectorySubmission
from .stationary_following import ARM_NAMES


ENABLED_KEY = 'unknown_physical_reference_mapping_enabled'
PLANNER_ATTRIBUTES = ('observation_trajectory_schedule',
                      'observation_physical_reference_provider')


def reference_options(context, domain='wire'):
    episode = context.get('physical_reference_episode')
    return ({} if episode is None else
            dict(following_model=episode.model, reference_domain=domain))


def following_options(context):
    episode = context.get('physical_reference_episode')
    return {} if episode is None else dict(following_model=episode.model)


def _digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'),
                                    allow_nan=False).encode()).hexdigest()


def _live_sample(sample, epoch_ns, now_ns):
    try:
        start, end = (sample[k] for k in
                      ('stationary_window_start_ns', 'stationary_window_end_ns'))
        stamps = [sample[k] for k in ('sdk_stamp_ns', 'accepted_stamp_ns')]
        if (sample.get('measurement_kind') != 'stationary_sdk_following_not_executed_endpoint'
                or sample.get('epoch_ns') != epoch_ns
                or any(type(s) is not int or not epoch_ns <= s <= now_ns for s in [start, end]+stamps)
                or end-start < 300000000 or end != min(stamps)
                or any(now_ns-s > 500000000 for s in stamps)
                or any(type(sample.get(k)) is not int or sample[k] < 3
                       for k in ('sdk_sample_count', 'accepted_sample_count'))
                or sample.get('opening_stamp_ns') != sample['accepted_stamp_ns']):
            raise ValueError('invalid stationary source')
    except (KeyError, TypeError, ValueError) as exc:
        raise ObservationPathError('PHYSICAL_FRESH_STATIONARY_SOURCE_REQUIRED') from exc


class MappedContactEpisode:
    """Caller serializes this object with the normal planner operation lock."""
    @classmethod
    def capture(cls, node, context, sample, phase, validate):
        enabled = context['config'].get(ENABLED_KEY, False)
        if type(enabled) is not bool:
            raise ObservationPathError('PHYSICAL_REFERENCE_CONFIG_INVALID')
        if not enabled:
            return None
        if context['plan'].model_choice != 'unknown_tabletop':
            return None
        previous = context.get('physical_reference_episode')
        if previous is not None:
            if type(previous) is not cls or previous.phase != phase:
                raise ObservationPathError('PHYSICAL_EPISODE_CONTEXT_REPLACED')
            previous.check()
            _live_sample(sample, context['epoch'], rospy.Time.now().to_nsec())
            validate_source_bias_model(previous.model, sample)
            return previous
        result = cls(node, context, sample, phase, validate)
        context['physical_reference_episode'] = result
        return result

    def __init__(self, node, context, sample, phase, validate):
        if (not isinstance(phase, str) or not phase or not callable(validate)
                or getattr(node, '_physical_reference_lease', None) is not None):
            raise ObservationPathError('PHYSICAL_EPISODE_INITIAL_STATE_INVALID')
        self.node, self.context, self.phase, self.validate = node, context, phase, validate
        self.lease = None
        self.finished = False
        self.failed = False
        self.planner = None
        self.physical_prove = None
        self.physical_revalidate = None
        self.saved = {}
        self.submissions = []
        validate()
        mono, now = time.monotonic(), rospy.Time.now().to_nsec()
        authority, outer_mono, outer_ros = self._outer_authority()
        # Nominal pregrasp has a separate original computation budget and an
        # original outer execution budget. Local corrections retain their own
        # original 60-second clock, including time already spent planning.
        budget = context.get('contact_deadlines')
        local_mono = (budget['execution_deadline'] if budget is not None
                      else context['deadline'])
        self.outer_deadline = outer_mono
        self.outer_ros_deadline = outer_ros
        self.local_deadline = min(local_mono, outer_mono)
        self.local_ros_deadline = min(outer_ros,
            now + int((self.local_deadline-mono)*1e9))
        if (not math.isfinite(self.local_deadline) or mono >= self.local_deadline
                or type(outer_ros) is not int or now >= self.local_ros_deadline):
            raise ObservationPathError('PHYSICAL_ORIGINAL_DEADLINE_EXPIRED')
        self.authority = authority
        self.identity = dict(authority=authority, epoch_ns=context['epoch'],
            revocation=context['revocation'], selection=deepcopy(context['selection']),
            plan_sha256=context['scene'].digest,
            model_sha256=hashlib.sha256(context['model'].encode()).hexdigest())
        self.owner_token = _digest(self.identity)
        self.episode_id = _digest(dict(owner=self.owner_token, phase=phase))
        chains = getattr(node, '_physical_contact_chains', None)
        if chains is None:
            chains = node._physical_contact_chains = {}
        chain = chains.get(self.owner_token)
        if chain is None:
            chain = chains[self.owner_token] = dict(identity=deepcopy(self.identity),
                outer_deadline=outer_mono, outer_ros_deadline=outer_ros,
                previous_binding_id='', phases={}, failed=False)
        if (chain['identity'] != self.identity or chain['outer_deadline'] != outer_mono
                or chain['outer_ros_deadline'] != outer_ros or chain['failed']
                or self.episode_id in chain['phases']):
            raise ObservationPathError('PHYSICAL_EPISODE_CONSUMED_OR_CHAIN_CHANGED')
        self.chain = chain
        self.previous_binding_id = chain['previous_binding_id']
        # Reserve the phase before any source/cap parsing can fail. A caller
        # cannot repair a failed capture by constructing a fresh local clock.
        chain['phases'][self.episode_id] = 'captured'
        chain['failed'] = True
        _live_sample(sample, context['epoch'], now)
        constraints = deepcopy(rospy.get_param('/alicia_controller/constraints', None))
        try:
            caps = [constraints[name]['trajectory'] for name in ARM_NAMES]
            if any(type(v) not in (int, float) or not math.isfinite(v) or v <= 0. for v in caps):
                raise ValueError('invalid cap')
        except (KeyError, TypeError, ValueError) as exc:
            raise ObservationPathError('PHYSICAL_SOURCE_CONTROLLER_CAP_REQUIRED') from exc
        limit = .09 if self.previous_binding_id else .035
        self.model = source_bias_model(sample, binding_id=uuid.uuid4().hex,
            source_bias_limit_rad=[min(limit, v) for v in caps])
        if (self.model.model_sha256 != self.identity['model_sha256']
                or self.model.epoch_ns != context['epoch']):
            raise ObservationPathError('PHYSICAL_SOURCE_CONTEXT_MISMATCH')
        self.constraints = constraints
        self.evidence = dict(schema=1, phase=phase, owner_token=self.owner_token,
            episode_id=self.episode_id, model=source_bias_model_record(self.model),
            previous_binding_id=self.previous_binding_id,
            outer_monotonic_deadline=outer_mono, outer_ros_deadline_ns=outer_ros,
            episode_monotonic_deadline=self.local_deadline,
            episode_ros_deadline_ns=self.local_ros_deadline,
            execution_authorized=False, release_verified=False, submissions=self.submissions)
        # Opening this provisional capture is the final construction step.
        # Every exception above leaves the original chain closed.
        chain['failed'] = False

    @property
    def kwargs(self):
        return dict(following_model=self.model)

    def _outer_authority(self):
        """Return only an actual captured task/session, never now + a budget."""
        context, node = self.context, self.node
        origin = getattr(node, '_observe_return_origin', None)
        if origin is not None and origin.get('selection') == context['selection']:
            if (origin.get('epoch') != context['epoch']
                    or origin.get('revocation') != context['revocation']
                    or origin.get('model') != context['model']
                    or origin.get('config') != context['config']
                    or origin['scene'].track_id != context['scene'].track_id):
                raise ObservationPathError('PHYSICAL_OUTER_SESSION_CHANGED')
            session = origin.get('view_session')
            if not isinstance(session, dict):
                raise ObservationPathError('PHYSICAL_QUALIFIED_OUTER_SESSION_REQUIRED')
            validated = node._validated_observe_return_view_session(origin, session)
            if validated != session:
                raise ObservationPathError('PHYSICAL_OUTER_SESSION_REPLACED')
            return (dict(kind='observation_session', identity=origin['identity'],
                         session_id=session['session_id']),
                    origin['deadline'], session['deadline_ns'])
        owner = getattr(node, '_direct_contact_authority', None)
        if owner is None or not owner.has_budget:
            raise ObservationPathError('PHYSICAL_AUTHENTICATED_OUTER_TASK_REQUIRED')
        lease = owner.capture(bindings=node._direct_contact_task_bindings(),
            now_ns=rospy.Time.now().to_nsec(), monotonic_now=time.monotonic())
        return (dict(kind='direct_contact_task', task_key=lease['task_key']),
                lease['monotonic_deadline'], lease['ros_deadline_ns'])

    def check(self):
        if self.failed or self.finished:
            raise ObservationPathError('PHYSICAL_EPISODE_CLOSED')
        self.validate()
        authority, mono, ros = self._outer_authority()
        if (authority != self.authority or mono != self.outer_deadline
                or ros != self.outer_ros_deadline
                or self.chain['failed'] or self.chain['identity'] != self.identity
                or self.chain['previous_binding_id'] != self.previous_binding_id
                or rospy.get_param('/alicia_controller/constraints', None) != self.constraints
                or time.monotonic() >= self.local_deadline
                or rospy.Time.now().to_nsec() >= self.local_ros_deadline):
            raise ObservationPathError('PHYSICAL_EPISODE_AUTHORITY_OR_DEADLINE_CHANGED')
        if self.lease is not None:
            self.lease.check_deadline()

    def _sample(self):
        self.check()
        fk = SerialUrdfFk(self.context['model'], ARM_NAMES)
        result = self.node._endpoint_following_snapshot(fk, self.context['epoch'],
            self.context, context_validator=self.check)
        _live_sample(result, self.context['epoch'], rospy.Time.now().to_nsec())
        validate_source_bias_model(self.model, result)
        following_offset(result['sdk_positions_rad'], result['accepted_positions_rad'],
                         following_model=self.model)
        return result

    def install(self, planner, *, physical_prove=None, physical_revalidate=None):
        """Install after the caller sets its original full wire path validator.

        Optional domain-specific callbacks take (physical_plan, reference) and
        (physical_plan, physical_audit), respectively. Their audits still pass
        the immutable DualReferenceContract's strict proof-kind validation.
        """
        self.check()
        if self.planner is not None:
            if self.planner is not planner:
                raise ObservationPathError('PHYSICAL_EPISODE_PLANNER_REPLACED')
            return
        if not callable(getattr(planner, 'observation_path_validator', None)):
            raise ObservationPathError('PHYSICAL_ORIGINAL_PATH_VALIDATOR_REQUIRED')
        self.saved = {name: (hasattr(planner, name), getattr(planner, name, None))
                      for name in PLANNER_ATTRIBUTES}
        if any(existed and value is not None for existed, value in self.saved.values()):
            raise ObservationPathError('PHYSICAL_PLANNER_HOOK_CONFLICT')
        self.planner = planner
        self.physical_prove, self.physical_revalidate = physical_prove, physical_revalidate
        planner.observation_trajectory_schedule = self._schedule
        planner.observation_physical_reference_provider = self._submission

    def _prove(self, physical, reference):
        self.check()
        if self.physical_prove is not None:
            return self.physical_prove(physical, reference)
        options = dict(following_model=self.model, reference_domain='physical')
        if self.context.get('mapped_contact_microstep') is True:
            options['contact_tracking_context'] = self.context
        return self.node._validate_frozen_observation_path(
            (self.context['scene'], self.context['revocation']), physical, self.planner,
            context_validator=self.check, **options)

    def _revalidate(self, physical, audit):
        self.check()
        if self.physical_revalidate is not None:
            self.physical_revalidate(physical, audit)
        elif self.context.get('mapped_contact_microstep') is True:
            self.node._revalidate_pregrasp_tracking_submission(
                self.context, physical, audit, reference_domain='physical')
        else:
            self.node._revalidate_observation_contract_submission(
                (self.context['scene'], self.context['revocation']), physical, audit,
                context_validator=self.check, following_model=self.model,
                reference_domain='physical')

    def _schedule(self, plan):
        self.check()
        # Measure this exact relative curve's real checking cost. Nothing is
        # admitted here. The timed curve is proved again after fixing its
        # absolute start, with the original clocks still running.
        before = message_bytes(plan)
        started = time.monotonic()
        self.planner.observation_path_validator(plan)
        reference = self.node._observation_controller_hold(plan.joint_trajectory.joint_names)
        physical = physical_plan_from_wire(plan, self.model)
        self._prove(physical, physical_reference_from_wire(reference, self.model,
                    plan.joint_trajectory.joint_names))
        elapsed = time.monotonic()-started
        if message_bytes(plan) != before:
            raise ObservationPathError('PHYSICAL_SCHEDULING_PROOF_MUTATED_CURVE')
        self.check()
        # Two measured proof costs leave one repeat's slack, plus two seconds
        # for stationary reconfirmation and the gated same-word transaction.
        # An overrun rejects this consumed episode; it never slides the start.
        lead = 2.*elapsed + 2.
        now = rospy.Time.now().to_nsec()
        plan.joint_trajectory.header.stamp = rospy.Time.from_sec((now+int(lead*1e9))*1e-9)
        duration = plan.joint_trajectory.points[-1].time_from_start.to_sec()
        grace = self.constraints['goal_time']
        stop = rospy.get_param('/alicia_controller/stop_trajectory_duration', None)
        if (type(grace) not in (int, float) or type(stop) not in (int, float)
                or not all(math.isfinite(x) and x > 0. for x in (duration, grace, stop))
                or lead+duration+grace+stop >= self.local_deadline-time.monotonic()
                or plan.joint_trajectory.header.stamp.to_nsec()+int((duration+grace+stop)*1e9)
                    >= self.local_ros_deadline):
            raise ObservationPathError('PHYSICAL_SCHEDULE_EXCEEDS_ORIGINAL_DEADLINE')
        self.evidence['schedule'] = dict(proof_measurement_sec=elapsed,
            start_lead_sec=lead, scheduled_start_ns=plan.joint_trajectory.header.stamp.to_nsec(),
            relative_curve_duration_sec=duration, original_deadlines_retained=True)
        return plan

    def _submission(self, wire_plan, wire_audit, wire_reference):
        self.check()
        physical = physical_plan_from_wire(wire_plan, self.model)
        physical_reference = physical_reference_from_wire(wire_reference, self.model,
            wire_plan.joint_trajectory.joint_names)
        physical_audit = self._prove(physical, physical_reference)
        dual = bind_dual_reference(wire_plan, physical, wire_audit, physical_audit,
            following_model=self.model, wire_reference=wire_reference,
            physical_reference=physical_reference)
        self.check()
        if self.lease is None:
            if (self.chain['phases'].get(self.episode_id) != 'captured'
                    or getattr(self.node, '_physical_reference_lease', None) is not None):
                raise ObservationPathError('PHYSICAL_EPISODE_ALREADY_SUBMITTED')
            def source_snapshot():
                # Called once after transport discovery and full dual-contract
                # checking. _sample observes actual new packets under the same
                # frozen owner/model; no timestamp is synthesized here.
                fresh = self._sample()
                state = self.node._trajectory_controller_state
                return fresh, state.header.stamp.to_nsec()
            try:
                self.lease = PhysicalReferenceLease.connect(dual,
                    owner_token=self.owner_token, goal_token=uuid.uuid4().hex,
                    selection_generation=int(self.context['selection']['generation']),
                    issued_ns=rospy.Time.now().to_nsec(), deadline_ns=self.outer_ros_deadline,
                    maximum_duration_sec=self.outer_deadline-time.monotonic(),
                    fresh_sample=None, controller_stamp_ns=None,
                    source_snapshot=source_snapshot,
                    episode_id=self.episode_id, episode_deadline_ns=self.local_ros_deadline,
                    episode_duration_sec=self.local_deadline-time.monotonic(),
                    previous_binding_id=self.previous_binding_id,
                    outer_monotonic_deadline=self.outer_deadline,
                    episode_monotonic_deadline=self.local_deadline)
            except Exception as exc:
                if hasattr(exc, 'source_admission'):
                    self.evidence['source_admission'] = deepcopy(exc.source_admission)
                raise
            self.evidence['source_admission'] = deepcopy(self.lease.source_admission)
            self.chain['phases'][self.episode_id] = 'consumed_before_prepare'
            self.node._prepare_physical_reference_lease(self.lease)
        else:
            self.lease.bind_next_goal(dual, goal_token=uuid.uuid4().hex)
        self.submissions.append(dict(proof_sha256=dual.proof_sha256,
            physical_trajectory_sha256=dual.physical_trajectory_sha256,
            wire_trajectory_sha256=dual.wire_trajectory_sha256,
            goal_token=self.lease.template.goal_token))
        def revalidate():
            self.check()
            self._revalidate(physical, physical_audit)
        return MappedTrajectorySubmission(dual, self.lease, revalidate=revalidate)

    def finish(self, success):
        if type(success) is not bool:
            raise ObservationPathError('PHYSICAL_EPISODE_RESULT_INVALID')
        if self.finished:
            if success and not self.failed:
                return deepcopy(self.evidence)
            raise ObservationPathError('PHYSICAL_EPISODE_RESULT_REPLACED')
        try:
            if success:
                self.check()
                if self.lease is not None:
                    fresh = self._sample()
                    self.evidence['final_following'] = following_evidence(
                        fresh['sdk_positions_rad'], fresh['accepted_positions_rad'],
                        following_model=self.model)
                    self.node._release_physical_reference_lease(self.lease)
                    if not self.lease.released:
                        raise ObservationPathError('PHYSICAL_EPISODE_RELEASE_NOT_CONFIRMED')
                    self.chain['previous_binding_id'] = self.model.binding_id
                    self.evidence['release_verified'] = True
                    self.chain['phases'][self.episode_id] = 'completed_and_released'
                else:
                    self.evidence['no_motion_submitted'] = True
                    self.chain['phases'][self.episode_id] = 'measured_no_motion_completion'
            else:
                self.failed = True
                self.chain['failed'] = True
                self.chain['phases'][self.episode_id] = 'failed_no_automatic_release'
            self.evidence['episode_success'] = success
            self.finished = True
        except Exception:
            self.failed = True
            self.chain['failed'] = True
            self.chain['phases'][self.episode_id] = 'release_or_result_failed'
            self.evidence['episode_success'] = False
            raise
        finally:
            if self.lease is not None:
                self.evidence['transaction_receipts_available'] = hasattr(
                    self.lease, 'transaction_receipts')
                self.evidence['transaction_receipts'] = deepcopy(
                    getattr(self.lease, 'transaction_receipts', []))
                self.evidence['transaction_receipts_dropped'] = getattr(
                    self.lease, 'transaction_receipts_dropped', 0)
        return deepcopy(self.evidence)

    def refresh_completed_sample(self, expected):
        """Reconfirm an unchanged endpoint after coordinate release.

        This reads new SDK/encoder packets; it never timestamps old values as
        new. Any value change invalidates the old measured endpoint proof.
        """
        try:
            if (not self.finished or self.failed or self.lease is None
                    or not self.lease.released or not self.evidence['release_verified']):
                raise ObservationPathError('PHYSICAL_COMPLETED_RELEASE_REQUIRED')
            validate_source_bias_model(self.model, expected)
            def live():
                self.validate()
                authority, mono, ros = self._outer_authority()
                if (self.failed or self.chain['failed'] or authority != self.authority
                        or mono != self.outer_deadline or ros != self.outer_ros_deadline
                        or self.chain['previous_binding_id'] != self.model.binding_id
                        or getattr(self.node, '_physical_reference_lease', None) is not None
                        or time.monotonic() >= self.local_deadline
                        or rospy.Time.now().to_nsec() >= self.local_ros_deadline):
                    raise ObservationPathError('PHYSICAL_RELEASED_ENDPOINT_AUTHORITY_CHANGED')
            live()
            after = rospy.Time.now().to_nsec()
            fk = SerialUrdfFk(self.context['model'], ARM_NAMES)
            fresh = self.node._endpoint_following_snapshot(fk, self.context['epoch'],
                self.context, after_ns=after, context_validator=live)
            _live_sample(fresh, self.context['epoch'], rospy.Time.now().to_nsec())
            if (fresh['stationary_window_start_ns'] <= after
                    or any(fresh[key] <= max(after, expected[key])
                           for key in ('sdk_stamp_ns', 'accepted_stamp_ns'))):
                raise ObservationPathError('PHYSICAL_RELEASED_ENDPOINT_NEW_SOURCES_REQUIRED')
            validate_source_bias_model(self.model, fresh)
            if (any(not np.array_equal(fresh[key], expected[key]) for key in
                    ('sdk_positions_rad', 'accepted_positions_rad'))
                    or fresh['accepted_gripper_opening_m'] != expected['accepted_gripper_opening_m']):
                raise ObservationPathError('PHYSICAL_RELEASED_MEASURED_ENDPOINT_CHANGED')
            following_offset(fresh['sdk_positions_rad'], fresh['accepted_positions_rad'],
                             following_model=self.model)
            live()
            self.evidence['post_release_endpoint_confirmation'] = dict(
                same_sdk_encoder_opening_values=True,
                original_sdk_stamp_ns=expected['sdk_stamp_ns'],
                original_accepted_stamp_ns=expected['accepted_stamp_ns'],
                sdk_stamp_ns=fresh['sdk_stamp_ns'], accepted_stamp_ns=fresh['accepted_stamp_ns'],
                stationary_window_start_ns=fresh['stationary_window_start_ns'],
                stationary_window_end_ns=fresh['stationary_window_end_ns'])
            return fresh
        except Exception:
            self.failed = True
            self.chain['failed'] = True
            self.chain['phases'][self.episode_id] = 'post_release_endpoint_rejected'
            self.evidence['post_release_endpoint_confirmed'] = False
            self.evidence['episode_success'] = False
            raise

    def restore(self):
        if self.planner is not None:
            for name, (existed, value) in self.saved.items():
                if existed:
                    setattr(self.planner, name, value)
                elif hasattr(self.planner, name):
                    delattr(self.planner, name)
