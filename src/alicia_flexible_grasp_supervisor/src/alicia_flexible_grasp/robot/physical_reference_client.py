"""Typed two-phase mapping admission. Construction never contacts ROS.

The gateway owns pose/scene qualification and publishes the transactional
physical hold while both writers are gated. This adapter cannot turn a pure
model or an endpoint check into execution permission.
"""
from copy import deepcopy
import math
import json
import time

import numpy as np

from .endpoint_correction import sdk_counts
from .following_model import ARM_NAMES, validate_source_bias_model
from .physical_reference_contract import (
    PhysicalReferenceError, message_bytes, message_sha256, validate_dual_reference,
)


class PhysicalReferenceLease:
    QUERY, PREPARE, COMMIT, ARM, QUIESCE, RELEASE_PREPARE, RELEASE_COMMIT, REVOKE = range(8)
    LEGACY, PREPARED, ACTIVE_IDLE, ARMED, EXECUTING, RELEASE_PENDING, FAULT, MANUAL = range(8)

    def __init__(self, contract, *, request_factory, driver_call, hardware_call,
                 owner_token, goal_token, selection_generation, issued_ns, deadline_ns,
                 maximum_duration_sec, fresh_sample, controller_stamp_ns,
                 episode_id, episode_deadline_ns, episode_duration_sec,
                 previous_binding_id='', clock=time.monotonic, ros_now_ns=None,
                 outer_monotonic_deadline=None, episode_monotonic_deadline=None,
                 source_snapshot=None):
        validation_started = time.perf_counter()
        self.contract = validate_dual_reference(contract)
        validation_elapsed = time.perf_counter()-validation_started
        self.model = contract.following_model
        self.clock = clock
        self.ros_now_ns = ros_now_ns
        if (not callable(ros_now_ns) or not callable(clock)
                or not callable(driver_call) or not callable(hardware_call)
                or not callable(request_factory)):
            raise PhysicalReferenceError('PHYSICAL_TYPED_TRANSPORT_REQUIRED')
        if (not isinstance(owner_token, str) or not owner_token
                or not isinstance(goal_token, str) or not goal_token
                or type(selection_generation) is not int or selection_generation <= 0
                or type(issued_ns) is not int or type(deadline_ns) is not int
                or not 0 < issued_ns < deadline_ns
                or isinstance(maximum_duration_sec, bool)
                or not math.isfinite(maximum_duration_sec) or maximum_duration_sec <= 0.):
            raise PhysicalReferenceError('PHYSICAL_LEASE_BINDING_INVALID')
        if (not isinstance(episode_id, str) or not episode_id
                or type(episode_deadline_ns) is not int
                or not issued_ns < episode_deadline_ns <= deadline_ns
                or isinstance(episode_duration_sec, bool)
                or not math.isfinite(episode_duration_sec) or episode_duration_sec <= 0.):
            raise PhysicalReferenceError('PHYSICAL_EPISODE_BINDING_INVALID')
        setup_issued_ns = issued_ns
        sampling_elapsed = 0.
        if source_snapshot is not None:
            # Service discovery and the complete immutable-contract validation
            # precede the final real observation. Refresh only this admission's
            # source packets, never the model, curve or original task clocks.
            if (not callable(source_snapshot) or fresh_sample is not None
                    or controller_stamp_ns is not None
                    or any(type(v) not in (int, float) or not math.isfinite(v)
                           for v in (outer_monotonic_deadline, episode_monotonic_deadline))):
                raise PhysicalReferenceError('PHYSICAL_SOURCE_SNAPSHOT_BINDING_INVALID')
            mono_before_sample = clock()
            if (type(mono_before_sample) not in (int, float)
                    or not math.isfinite(mono_before_sample)
                    or mono_before_sample >= min(outer_monotonic_deadline,
                                                 episode_monotonic_deadline)):
                raise PhysicalReferenceError('PHYSICAL_ORIGINAL_MONOTONIC_DEADLINE_INVALID')
            sampling_started = time.perf_counter()
            fresh_sample, controller_stamp_ns = source_snapshot()
            sampling_elapsed = time.perf_counter()-sampling_started
            if type(fresh_sample) is not dict:
                raise PhysicalReferenceError('PHYSICAL_SOURCE_SNAPSHOT_INVALID')
        if type(controller_stamp_ns) is not int or controller_stamp_ns <= 0:
            raise PhysicalReferenceError('PHYSICAL_LEASE_BINDING_INVALID')
        validate_source_bias_model(self.model, fresh_sample)
        # Reconfirmation may refresh timestamps, never the frozen vectors or
        # bias. Another settled endpoint requires its own explicit new model.
        if (not np.array_equal(fresh_sample['sdk_positions_rad'], self.model.source_sdk_positions_rad)
                or not np.array_equal(fresh_sample['accepted_positions_rad'],
                    self.model.source_accepted_positions_rad)):
            raise PhysicalReferenceError('PHYSICAL_SOURCE_RECONFIRMATION_CHANGED')
        now = ros_now_ns()
        mono = clock()
        if source_snapshot is not None and type(now) is int:
            # This is the actual new request issuance, not a renewed lifetime.
            # The original absolute deadlines below remain authoritative.
            issued_ns = now
        stamps = dict(sdk_stamp_ns=fresh_sample.get('sdk_stamp_ns'),
                      accepted_stamp_ns=fresh_sample.get('accepted_stamp_ns'),
                      controller_stamp_ns=controller_stamp_ns)
        self.source_admission = dict(source_snapshot_used=source_snapshot is not None,
            setup_issued_ns=setup_issued_ns, issued_ns=issued_ns, checked_ns=now,
            maximum_source_age_ns=500000000, source_stamps_ns=stamps,
            source_ages_ns={k: now-v if type(now) is int and type(v) is int else None
                           for k, v in stamps.items()},
            contract_validation_elapsed_sec=validation_elapsed,
            source_snapshot_elapsed_sec=sampling_elapsed,
            original_outer_ros_deadline_ns=deadline_ns,
            original_episode_ros_deadline_ns=episode_deadline_ns,
            original_outer_monotonic_deadline=outer_monotonic_deadline,
            original_episode_monotonic_deadline=episode_monotonic_deadline,
            passed=False)
        if (type(now) is not int or not isinstance(mono, (int, float))
                or isinstance(mono, bool) or not math.isfinite(mono)
                or not setup_issued_ns <= issued_ns <= now < min(deadline_ns, episode_deadline_ns)
                or any(type(fresh_sample.get(k)) is not int
                    or not 0 <= now-fresh_sample[k] <= 500000000
                    for k in ('sdk_stamp_ns', 'accepted_stamp_ns'))
                or not 0 <= now-controller_stamp_ns <= 500000000):
            error = PhysicalReferenceError('PHYSICAL_LEASE_SOURCE_STALE')
            error.source_admission = deepcopy(self.source_admission)
            raise error
        if source_snapshot is not None:
            maximum_duration_sec = min(maximum_duration_sec,
                (deadline_ns-now)*1e-9, outer_monotonic_deadline-mono)
            episode_duration_sec = min(episode_duration_sec,
                (episode_deadline_ns-now)*1e-9, episode_monotonic_deadline-mono,
                maximum_duration_sec)
        self.driver_call, self.hardware_call = driver_call, hardware_call
        self.request_factory = request_factory
        self.outer_deadline = mono+min(maximum_duration_sec, (deadline_ns-now)*1e-9)
        self.local_deadline = min(self.outer_deadline,
            mono+min(episode_duration_sec, (episode_deadline_ns-now)*1e-9))
        for value in (outer_monotonic_deadline, episode_monotonic_deadline):
            if value is not None and (isinstance(value, bool)
                    or not math.isfinite(value) or value <= clock()):
                error = PhysicalReferenceError('PHYSICAL_ORIGINAL_MONOTONIC_DEADLINE_INVALID')
                error.source_admission = deepcopy(self.source_admission)
                raise error
        if outer_monotonic_deadline is not None:
            self.outer_deadline = min(self.outer_deadline, outer_monotonic_deadline)
        self.local_deadline = min(self.local_deadline, self.outer_deadline)
        if episode_monotonic_deadline is not None:
            self.local_deadline = min(self.local_deadline, episode_monotonic_deadline)
        self.source_admission.update(passed=True,
            maximum_duration_sec=maximum_duration_sec,
            episode_duration_sec=episode_duration_sec,
            outer_monotonic_deadline=self.outer_deadline,
            episode_monotonic_deadline=self.local_deadline)
        self.sequence = 0
        self.armed_envelope = None
        self.release_counts = None
        self.released = False
        self.transaction_receipts = []
        self.transaction_receipts_dropped = 0
        self.template = request_factory()
        values = dict(binding_id=self.model.binding_id, previous_binding_id=previous_binding_id,
            owner_token=owner_token, goal_token=goal_token, epoch_ns=self.model.epoch_ns,
            selection_generation=selection_generation, plan_sha256=contract.plan_sha256,
            model_sha256=self.model.model_sha256, proof_sha256=contract.proof_sha256,
            physical_trajectory_sha256=contract.physical_trajectory_sha256,
            wire_trajectory_sha256=contract.wire_trajectory_sha256, bias_rad=list(self.model.bias_rad),
            stop_duration_sec=json.loads(contract.physical_audit_json)[
                'execution_tracking_contract']['stop_trajectory_duration_sec'],
            baseline_sdk_counts=sdk_counts(fresh_sample['sdk_positions_rad']).tolist(),
            source_sdk_stamp_ns=fresh_sample['sdk_stamp_ns'],
            source_encoder_stamp_ns=fresh_sample['accepted_stamp_ns'],
            controller_stamp_ns=controller_stamp_ns, issued_ns=issued_ns,
            deadline_ns=deadline_ns, maximum_duration_sec=float(maximum_duration_sec),
            episode_id=episode_id, episode_deadline_ns=episode_deadline_ns,
            episode_duration_sec=float(episode_duration_sec))
        for name, value in values.items():
            setattr(self.template, name, value)

    @classmethod
    def connect(cls, contract, **kwargs):
        import rospy
        from alicia_d_driver.srv import SetPhysicalReference, SetPhysicalReferenceRequest
        clock = kwargs.get('clock', time.monotonic)
        kwargs.setdefault('clock', clock)
        started = clock()
        paths = ('/alicia_d/physical_reference', '/bessica_d_hw_interface/physical_reference')
        for path in paths:
            rospy.wait_for_service(path, timeout=2.)
        elapsed = clock()-started
        for field in ('maximum_duration_sec', 'episode_duration_sec'):
            kwargs[field] -= elapsed
        try:
            result = cls(contract, request_factory=SetPhysicalReferenceRequest,
                driver_call=rospy.ServiceProxy(paths[0], SetPhysicalReference),
                hardware_call=rospy.ServiceProxy(paths[1], SetPhysicalReference),
                ros_now_ns=lambda: rospy.Time.now().to_nsec(), **kwargs)
        except PhysicalReferenceError as error:
            if hasattr(error, 'source_admission'):
                error.source_admission['service_wait_elapsed_sec'] = elapsed
            raise
        result.source_admission['service_wait_elapsed_sec'] = elapsed
        return result

    def check_deadline(self):
        now = self.ros_now_ns()
        mono = self.clock()
        if (type(now) is not int or not isinstance(mono, (int, float))
                or isinstance(mono, bool) or not math.isfinite(mono)
                or self.released or not self.template.issued_ns <= now < min(
                self.template.deadline_ns, self.template.episode_deadline_ns)
                or mono >= self.local_deadline):
            raise PhysicalReferenceError('PHYSICAL_LEASE_DEADLINE_EXCEEDED')

    def _request(self, operation, action_goal=None):
        self.check_deadline()
        self.sequence += 1
        request = deepcopy(self.template)
        request.operation, request.request_sequence = operation, self.sequence
        # Requests carry remaining wall time so an old client object cannot
        # extend a newly received prepare. Server additionally takes min with
        # the original lease for every rollover.
        request.maximum_duration_sec = min(request.maximum_duration_sec,
            self.outer_deadline-self.clock())
        request.episode_duration_sec = min(request.episode_duration_sec,
            self.local_deadline-self.clock())
        if operation in (self.RELEASE_PREPARE, self.RELEASE_COMMIT) and self.release_counts is not None:
            request.baseline_sdk_counts = list(self.release_counts)
        if action_goal is not None:
            request.action_goal = deepcopy(action_goal)
            request.wire_trajectory = deepcopy(self.contract.wire_plan().joint_trajectory)
        return request

    def _state(self, response, phases, *, armed=False, baseline=False):
        if getattr(response, 'success', False) is not True:
            raise PhysicalReferenceError('PHYSICAL_TYPED_REJECTED: '+str(getattr(response, 'code', 'missing')))
        state = response.state
        if state.phase not in phases or state.evidence_valid is not True:
            raise PhysicalReferenceError('PHYSICAL_TYPED_STATE_INVALID')
        for field in ('binding_id', 'owner_token', 'epoch_ns', 'selection_generation',
                'plan_sha256', 'model_sha256', 'deadline_ns', 'episode_id', 'episode_deadline_ns',
                'previous_binding_id'):
            if getattr(state, field) != getattr(self.template, field):
                raise PhysicalReferenceError('PHYSICAL_TYPED_BINDING_CHANGED: '+field)
        if tuple(state.bias_rad) != self.model.bias_rad:
            raise PhysicalReferenceError('PHYSICAL_TYPED_BIAS_CHANGED')
        if (not math.isfinite(state.monotonic_remaining_sec)
                or state.monotonic_remaining_sec <= 0.
                or state.monotonic_remaining_sec > self.template.episode_duration_sec+1e-6
                or not math.isfinite(state.outer_monotonic_remaining_sec)
                or not 0. < state.outer_monotonic_remaining_sec <= self.template.maximum_duration_sec+1e-6):
            raise PhysicalReferenceError('PHYSICAL_TYPED_DEADLINE_INVALID')
        now = self.ros_now_ns()
        if not 0 <= now-state.header.stamp.to_nsec() <= 500000000:
            raise PhysicalReferenceError('PHYSICAL_TYPED_STATE_STALE')
        if baseline and tuple(state.sdk_counts) != tuple(self.template.baseline_sdk_counts):
            raise PhysicalReferenceError('PHYSICAL_TRANSACTION_WIRE_CHANGED')
        if armed:
            for field in ('goal_token', 'proof_sha256', 'physical_trajectory_sha256',
                    'wire_trajectory_sha256', 'stop_duration_sec'):
                if getattr(state, field) != getattr(self.template, field):
                    raise PhysicalReferenceError('PHYSICAL_TYPED_GOAL_CHANGED: '+field)
            if self.armed_envelope is not None:
                if (state.action_goal_id != self.armed_envelope.goal_id.id
                        or state.action_goal_stamp_ns != self.armed_envelope.goal_id.stamp.to_nsec()):
                    raise PhysicalReferenceError('PHYSICAL_TYPED_ACTION_CHANGED')
        self.check_deadline()
        return state

    def _both(self, operation, phases, *, action_goal=None, baseline=False, armed=False):
        request = self._request(operation, action_goal)
        states = []
        # Driver gates first; no hardware writer can leak a transition through
        # the old legacy admission while its own prepare is still pending.
        for transport in (self.driver_call, self.hardware_call):
            state = self._state(self._call(transport,deepcopy(request)), phases,
                baseline=baseline, armed=armed)
            if operation != self.QUERY and state.request_sequence != request.request_sequence:
                raise PhysicalReferenceError('PHYSICAL_TYPED_ACK_SEQUENCE_CHANGED')
            states.append(state)
        return tuple(states)

    def _call(self, transport, request):
        """Bounded metadata only; successful RPC is never physical arrival."""
        receipt = dict(endpoint='driver' if transport is self.driver_call else 'hardware',
            operation=int(request.operation),request_sequence=int(request.request_sequence),
            binding_id=request.binding_id,goal_token=request.goal_token)
        try:
            response = transport(request)
            if request.operation != self.QUERY:
                state = response.state
                receipt.update(success=bool(response.success),code=str(response.code),
                    state={key:getattr(state,key,None) for key in (
                        'phase','binding_id','owner_token','epoch_ns','request_sequence',
                        'goal_token','action_goal_id','action_goal_stamp_ns','proof_sha256',
                        'physical_trajectory_sha256','wire_trajectory_sha256',
                        'sdk_stamp_ns','encoder_stamp_ns','controller_stamp_ns','output_gated')})
                receipt['state']['sdk_counts'] = list(state.sdk_counts)
                receipt['state']['state_stamp_ns'] = state.header.stamp.to_nsec()
            return response
        except Exception as exc:
            receipt.update(success=False,transport_error_type=type(exc).__name__,
                transport_error=str(exc)[:256])
            raise
        finally:
            if request.operation != self.QUERY:
                if len(self.transaction_receipts) >= 128:
                    self.transaction_receipts.pop(0);self.transaction_receipts_dropped += 1
                self.transaction_receipts.append(receipt)

    def prepare(self):
        request = self._request(self.PREPARE)
        driver = self._state(self._call(self.driver_call,deepcopy(request)),
            (self.PREPARED,),baseline=True)
        if driver.request_sequence != request.request_sequence:
            raise PhysicalReferenceError('PHYSICAL_DRIVER_PREPARE_ACK_CHANGED')
        deadline = min(self.local_deadline,self.clock()+.1)
        while True:
            request.maximum_duration_sec = min(request.maximum_duration_sec,self.outer_deadline-self.clock())
            request.episode_duration_sec = min(request.episode_duration_sec,self.local_deadline-self.clock())
            response = self._call(self.hardware_call,deepcopy(request))
            if response.success is True or response.code != 'REFERENCE_DRIVER_NOT_PREPARED':
                hardware = self._state(response,(self.PREPARED,),baseline=True)
                break
            self._state(self._call(self.driver_call,self._request(self.QUERY)),
                (self.PREPARED,),baseline=True)
            if self.clock() >= deadline:
                raise PhysicalReferenceError('PHYSICAL_PREPARE_PEER_RECEIPT_TIMEOUT')
            time.sleep(.002)
        if hardware.request_sequence != request.request_sequence:
            raise PhysicalReferenceError('PHYSICAL_HARDWARE_PREPARE_ACK_CHANGED')
        states = (driver,hardware)
        if any(s.output_gated is not True or s.pending_complete is not True
                or s.stationary is not True for s in states):
            raise PhysicalReferenceError('PHYSICAL_PREPARE_NOT_GATED')
        return states

    def commit(self):
        states = self._both(self.COMMIT, (self.ACTIVE_IDLE,), baseline=True)
        if any(s.output_gated is not False or s.pending_complete is not True
                or s.stationary is not True for s in states):
            raise PhysicalReferenceError('PHYSICAL_COMMIT_STILL_GATED')
        return states

    def query(self, *, phases=None, armed=None):
        if phases is None:
            phases = (self.ACTIVE_IDLE, self.ARMED, self.EXECUTING)
        if armed is None:
            armed = self.armed_envelope is not None
        return self._both(self.QUERY, phases, armed=armed)

    def validate_action_budget(self, action_goal):
        self.check_deadline()
        start = action_goal.goal.trajectory.header.stamp.to_nsec()
        duration = action_goal.goal.trajectory.points[-1].time_from_start.to_nsec()
        grace = action_goal.goal.goal_time_tolerance.to_nsec()
        stop = json.loads(self.contract.physical_audit_json)['execution_tracking_contract']['stop_trajectory_duration_sec']
        stop_ns = int(math.ceil(float(stop)*1e9))
        now = self.ros_now_ns()
        required = (start-now+duration+grace+stop_ns)*1e-9
        if (not action_goal.goal_id.id or action_goal.goal_id.stamp.to_nsec() <= 0
                or not now < start
                or start+duration+grace+stop_ns > min(self.template.deadline_ns,
                    self.template.episode_deadline_ns)
                or not math.isfinite(required) or required <= 0.
                or required > self.local_deadline-self.clock()):
            raise PhysicalReferenceError('PHYSICAL_ACTION_START_OR_DEADLINE_INVALID')
        return start

    def arm(self, action_goal):
        validate_dual_reference(self.contract)
        if self.armed_envelope is not None:
            raise PhysicalReferenceError('PHYSICAL_GOAL_ALREADY_CONSUMED')
        if message_bytes(action_goal.goal) != message_bytes(self.contract.action_goal()):
            raise PhysicalReferenceError('PHYSICAL_ACTION_GOAL_CHANGED')
        start = self.validate_action_budget(action_goal)
        self.armed_envelope = deepcopy(action_goal)
        request = self._request(self.ARM, action_goal)
        hardware = self._state(self._call(self.hardware_call,request), (self.ARMED,), armed=True)
        if hardware.request_sequence != request.request_sequence:
            raise PhysicalReferenceError('PHYSICAL_HARDWARE_ARM_ACK_CHANGED')
        barrier_deadline = min(self.local_deadline, self.clock()+.1)
        while True:
            driver = self._state(self._call(self.driver_call,self._request(self.QUERY)),
                (self.ACTIVE_IDLE,), armed=False)
            if driver.last_command_sequence >= hardware.last_command_sequence:
                break
            if self.clock() >= barrier_deadline or self.ros_now_ns() >= start:
                raise PhysicalReferenceError('PHYSICAL_IDLE_QUEUE_BARRIER_TIMEOUT')
            time.sleep(.002)
        request = self._request(self.ARM, action_goal)
        driver = self._state(self._call(self.driver_call,request), (self.ARMED,), armed=True)
        if driver.request_sequence != request.request_sequence:
            raise PhysicalReferenceError('PHYSICAL_DRIVER_ARM_ACK_CHANGED')
        states = (driver, hardware)
        if any(s.output_gated is not True for s in states) or self.ros_now_ns() >= start:
            raise PhysicalReferenceError('PHYSICAL_ACTION_ARM_NOT_BEFORE_START')
        return states

    def quiesce(self):
        states = self.query(phases=(self.ACTIVE_IDLE, self.EXECUTING), armed=True)
        if all(s.phase == self.ACTIVE_IDLE and s.goal_terminal and s.pending_complete
                and s.stationary for s in states):
            return states
        if any(s.phase != self.EXECUTING or not s.goal_terminal for s in states):
            raise PhysicalReferenceError('PHYSICAL_QUIESCE_TERMINAL_REQUIRED')
        return self._both(self.QUIESCE, (self.ACTIVE_IDLE,), armed=True)

    def bind_next_goal(self, contract, *, goal_token):
        validate_dual_reference(contract)
        if (not isinstance(goal_token, str) or not goal_token or goal_token == self.template.goal_token
                or contract.following_model != self.model
                or contract.plan_sha256 != self.contract.plan_sha256):
            raise PhysicalReferenceError('PHYSICAL_NEXT_GOAL_BINDING_CHANGED')
        states = self.quiesce()
        if not all(s.goal_terminal and s.goal_status == 3 and s.goal_error_code == 0 for s in states):
            raise PhysicalReferenceError('PHYSICAL_PREVIOUS_GOAL_NOT_SUCCESSFUL')
        self.contract = contract
        self.template.goal_token = goal_token
        self.template.proof_sha256 = contract.proof_sha256
        self.template.physical_trajectory_sha256 = contract.physical_trajectory_sha256
        self.template.wire_trajectory_sha256 = contract.wire_trajectory_sha256
        self.template.stop_duration_sec = json.loads(contract.physical_audit_json)[
            'execution_tracking_contract']['stop_trajectory_duration_sec']
        self.armed_envelope = None
        return self

    def release_prepare(self):
        states = self.quiesce()
        if (not all(s.goal_terminal and s.goal_status == 3 and s.goal_error_code == 0
                and s.pending_complete and s.stationary for s in states)
                or tuple(states[0].sdk_counts) != tuple(states[1].sdk_counts)):
            raise PhysicalReferenceError('PHYSICAL_RELEASE_SUCCESSFUL_STATIONARY_TERMINAL_REQUIRED')
        self.release_counts = tuple(states[0].sdk_counts)
        request = self._request(self.RELEASE_PREPARE)
        hardware = self._state(self._call(self.hardware_call,request), (self.RELEASE_PENDING,), armed=True)
        if not hardware.output_gated or hardware.request_sequence != request.request_sequence:
            raise PhysicalReferenceError('PHYSICAL_HARDWARE_RELEASE_NOT_GATED')
        barrier_deadline = min(self.local_deadline, self.clock()+.1)
        while True:
            driver = self._state(self._call(self.driver_call,self._request(self.QUERY)),
                (self.ACTIVE_IDLE,), armed=True)
            if driver.last_command_sequence >= hardware.last_command_sequence:
                break
            if self.clock() >= barrier_deadline:
                raise PhysicalReferenceError('PHYSICAL_RELEASE_QUEUE_BARRIER_TIMEOUT')
            time.sleep(.002)
        request = self._request(self.RELEASE_PREPARE)
        driver = self._state(self._call(self.driver_call,request), (self.RELEASE_PENDING,), armed=True)
        if not driver.output_gated or driver.request_sequence != request.request_sequence:
            raise PhysicalReferenceError('PHYSICAL_DRIVER_RELEASE_NOT_GATED')
        if any(tuple(s.sdk_counts) != self.release_counts for s in (driver, hardware)):
            raise PhysicalReferenceError('PHYSICAL_RELEASE_BASELINE_CHANGED')
        return driver, hardware

    def release_commit(self):
        # A successful release intentionally clears the old binding. Do not
        # apply active-state equality to the resulting legacy record.
        request = self._request(self.RELEASE_COMMIT)
        states = []
        for transport in (self.driver_call, self.hardware_call):
            response = self._call(transport,deepcopy(request))
            state = self._state(response, (self.LEGACY,), armed=True)
            if (state.output_gated is not False or not state.pending_complete or not state.stationary
                    or state.request_sequence != request.request_sequence
                    or tuple(state.sdk_counts) != self.release_counts):
                raise PhysicalReferenceError('PHYSICAL_RELEASE_NOT_CONTINUOUS')
            states.append(state)
        self.released = True
        return tuple(states)

    def _manual_request(self, operation):
        """Old failed lease cleanup never creates or renews motion authority."""
        if self.released:
            raise PhysicalReferenceError('PHYSICAL_MANUAL_CLEANUP_ALREADY_RELEASED')
        self.sequence += 1
        request = deepcopy(self.template)
        request.operation, request.request_sequence = operation, self.sequence
        if self.release_counts is not None:
            request.baseline_sdk_counts = list(self.release_counts)
        return request

    def _manual_state(self, response, phases, *, query=False, require_stationary=True):
        state = response.state
        if ((not query and response.success is not True) or state.phase not in phases
                or state.evidence_valid is not True or state.manual_reference is not True
                or state.pending_complete is not True
                or (require_stationary and state.stationary is not True)):
            raise PhysicalReferenceError('PHYSICAL_MANUAL_CLEANUP_EVIDENCE_REQUIRED')
        for field in ('binding_id', 'owner_token', 'epoch_ns', 'selection_generation',
                'plan_sha256', 'model_sha256', 'episode_id', 'deadline_ns', 'episode_deadline_ns'):
            if getattr(state, field) != getattr(self.template, field):
                raise PhysicalReferenceError('PHYSICAL_MANUAL_CLEANUP_BINDING_CHANGED')
        now, mono = self.ros_now_ns(), self.clock()
        if (type(now) is not int or not math.isfinite(mono)
                or not 0 <= now-state.header.stamp.to_nsec() <= 500000000
                or not 0 <= now-state.sdk_stamp_ns <= 500000000
                or not 0 <= now-state.encoder_stamp_ns <= 500000000):
            raise PhysicalReferenceError('PHYSICAL_MANUAL_CLEANUP_SOURCE_STALE')
        return state

    def manual_cleanup_query(self, phases=None, *, require_stationary=True):
        if phases is None:
            phases = (self.FAULT, self.MANUAL)
        request = self._manual_request(self.QUERY)
        return tuple(self._manual_state(self._call(call,deepcopy(request)), phases, query=True,
            require_stationary=require_stationary)
            for call in (self.driver_call, self.hardware_call))

    def manual_cleanup_prepare(self):
        states = self.manual_cleanup_query()
        if tuple(states[0].sdk_counts) != tuple(states[1].sdk_counts):
            raise PhysicalReferenceError('PHYSICAL_MANUAL_CLEANUP_WORDS_DISAGREE')
        self.release_counts = tuple(states[0].sdk_counts)
        request = self._manual_request(self.RELEASE_PREPARE)
        # Failed/manual writers are already gated; use the same HW-first
        # preparation order as successful release without opening either gate.
        hardware = self._manual_state(self._call(self.hardware_call,deepcopy(request)), (self.RELEASE_PENDING,))
        driver = self._manual_state(self._call(self.driver_call,deepcopy(request)), (self.RELEASE_PENDING,))
        states = (driver, hardware)
        if not all(s.output_gated and s.request_sequence == request.request_sequence
                and tuple(s.sdk_counts) == self.release_counts for s in states):
            raise PhysicalReferenceError('PHYSICAL_MANUAL_CLEANUP_NOT_GATED')
        return states

    def manual_cleanup_commit(self):
        request = self._manual_request(self.RELEASE_COMMIT)
        states = tuple(self._manual_state(self._call(call,deepcopy(request)), (self.LEGACY,))
            for call in (self.driver_call, self.hardware_call))
        if not all(not s.output_gated and s.request_sequence == request.request_sequence
                and tuple(s.sdk_counts) == self.release_counts for s in states):
            raise PhysicalReferenceError('PHYSICAL_MANUAL_CLEANUP_NOT_CONTINUOUS')
        self.released = True
        return states


class MappedTrajectorySubmission:
    def __init__(self, contract, lease, *, revalidate):
        self.contract = validate_dual_reference(contract)
        if (type(lease) is not PhysicalReferenceLease
                or lease.contract != contract or not callable(revalidate)):
            raise PhysicalReferenceError('PHYSICAL_SUBMISSION_BINDING_INVALID')
        self.lease, self.revalidate = lease, revalidate

    def action_goal(self):
        return self.contract.action_goal()

    def arm(self, action_goal):
        self.revalidate()
        self.lease.check_deadline()
        self.lease.arm(action_goal)
        self.revalidate()
        self.lease.validate_action_budget(action_goal)

    def check_active(self):
        # C++ enforces live tokens/epoch/lease at every write. Python keeps the
        # inherited clocks without adding a blocking service RPC to each tick.
        self.lease.check_deadline()

    def confirm_result(self, state, result):
        if state != 3 or result is None or getattr(result, 'error_code', None) != 0:
            return
        deadline = min(self.lease.local_deadline, self.lease.clock()+.1)
        while True:
            states = self.lease.query()
            if all(s.goal_terminal and s.goal_status == state
                    and s.goal_error_code == result.error_code for s in states):
                return
            if self.lease.clock() >= deadline:
                raise PhysicalReferenceError('PHYSICAL_TERMINAL_PROVENANCE_MISSING')
            time.sleep(.002)
