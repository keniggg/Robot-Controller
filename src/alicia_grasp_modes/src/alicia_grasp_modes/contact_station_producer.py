"""Remove only stationary privileges after a source-bound contact handoff."""
from alicia_grasp_modes.runtime_gate_policy import (SOURCE_MAX_AGE_SEC)
from copy import deepcopy
from dataclasses import asdict
import json
import time

import numpy as np
import rospy
from std_msgs.msg import String

from .contact_station_retirement import REQUEST_TOPIC, ACK_TOPIC, parse_request, ack, wait_for_pregrasp_state
from .observe_return_measurement import measurement_digest
from .rgb_observation_handoff import _candidate_fingerprint
from .rgb_observation_source import digest, require_fresh_source
from .selection import parse_selection
from alicia_flexible_grasp.robot.observation_view_gateway import _camera_configuration_contract
from alicia_flexible_grasp.robot.observation_path_guard import committed_plan_digest
from alicia_flexible_grasp.grasp.grasp_state_machine import GraspStages
from alicia_flexible_grasp.grasp.refinement_continuity import binding_from_phase


class ContactStationProducerMixin:
    def _setup_contact_station_io(self):
        self._contact_station_prepared = None
        self._contact_station_version = 0
        self._contact_station_status = None
        self._contact_spatial_context = None
        self._contact_station_ack_pub = rospy.Publisher(ACK_TOPIC, String, queue_size=4)
        self._contact_station_sub = rospy.Subscriber(REQUEST_TOPIC, String, self._contact_station_request_cb, queue_size=4)

    def _contact_station_context(self, request):
        reference = getattr(self, '_locked_rgb_hold_reference', None)
        marker = (reference or {}).get('observation_arrival_retry')
        completed = getattr(self, '_observation_measurement_status', None) or {}
        measurement = completed.get('request') or {}
        phase = getattr(self, '_station_compute_phase', None)
        state = getattr(self, '_station_compute_task_state', None)
        now = rospy.Time.now().to_nsec()
        if (marker is None or marker.get('invalidated_reason') or reference['pose_invalidated']
                or completed.get('state') != 'completed' or measurement.get('state') != 'qualified'
                or measurement_digest(measurement) != request['measurement_sha256']
                or any(measurement.get(k) != request[k] for k in
                       ('selection', 'session_id', 'target_track_id', 'target_epoch', 'control_epoch_ns'))
                or self.mode != 'unknown' or self.generation != request['selection']['generation']
                or self.selection_ns != request['selection']['stamp_ns']
                or self._rgb_interest_epoch != request['control_epoch_ns']
                or self.first_loss is not None or self.tracker.lost
                or self.last_good_source_stamp_ns != reference['source_stamp_ns']
                or not measurement['source_stamps_ns'][-1] <= request['plan_source_ns'] <= self.last_good_source_stamp_ns
                or getattr(self, '_observation_measurement', None) is not None
                or getattr(self, '_locked_source_pending', None) is not None
                or getattr(self, '_metric_ambiguity_pending', None) is not None
                or phase is None or not phase.active or phase.planning_stage != 'contact_replan'
                or phase.phase_id != request['phase_id'] or phase.header.stamp.to_nsec() != request['phase_started_ns']
                or phase.reference_target_track_id != request['target_track_id']
                or state is None or not state.active or state.state != request['stage']
                or rospy.get_param_cached('/gui/joint_direct_mode', False) is True
                or now >= min(request['deadline_ns'], marker['session_deadline_ns'], phase.deadline.to_nsec())
                or time.monotonic() >= marker['session_deadline_monotonic']
                or digest(asdict(self.config)) != marker['config_sha256']
                or _candidate_fingerprint(self.tracker.anchor) != marker['anchor_fingerprint']
                or _candidate_fingerprint(self.tracker.last) != reference['last_fingerprint']
                or not np.array_equal(self.tracker.plane_base, marker['reference_plane'])
                or digest(_camera_configuration_contract(rospy.get_param_cached('/camera'))) != marker['camera_configuration_sha256']):
            raise ValueError('CONTACT_STATION_PRODUCER_CONTEXT_CHANGED')
        require_fresh_source(reference['source_stamp_ns'], now, min(self.max_age, SOURCE_MAX_AGE_SEC))
        stamp = rospy.Time(reference['source_stamp_ns']//10**9, reference['source_stamp_ns']%10**9)
        if digest(self._rgb_interest_source_context(stamp)) != marker['source_id']:
            raise ValueError('CONTACT_STATION_SOURCE_CHANGED')
        rows = [(ns, q) for ns, q in self._rgb_interest_feedback if ns >= marker['qualified_source_ns']]
        if not rows or any(np.max(np.abs(q-marker['arrival_q'])) > .004 for ns, q in rows):
            raise ValueError('CONTACT_STATION_MOVED_BEFORE_SUBMISSION')
        require_fresh_source(rows[-1][0], now, min(self.max_age, SOURCE_MAX_AGE_SEC))
        return reference, marker

    def _contact_station_request_cb(self, message):
        request = None
        try:
            if len(message.data) > 32768:
                raise ValueError('CONTACT_STATION_REQUEST_TOO_LARGE')
            raw = json.loads(message.data)
            if raw.get('state') == 'prepare':
                request = parse_request(raw, rospy.get_param_cached('/grasp_mode/selection'), rospy.Time.now().to_nsec())
                def state():
                    value = getattr(self, '_station_compute_task_state', None)
                    return getattr(value, 'state', ''), getattr(value, 'active', False)
                wait_for_pregrasp_state(request, state,
                    lambda: self._rgb_interest_epoch == request['control_epoch_ns']
                        and parse_selection(rospy.get_param_cached('/grasp_mode/selection')) == request['selection']
                        and rospy.get_param_cached('/gui/joint_direct_mode', False) is False,
                    lambda: rospy.Time.now().to_nsec())
        except (ValueError, KeyError, TypeError, AttributeError) as exc:
            if request is not None:
                self._contact_station_ack_pub.publish(String(data=json.dumps(ack(request, 'unknown', False, str(exc)))))
            return
        with self.lock, self.input_lock:
            try:
                if len(message.data) > 32768:
                    raise ValueError('CONTACT_STATION_REQUEST_TOO_LARGE')
                raw = json.loads(message.data)
                previous = getattr(self, '_contact_station_prepared', None)
                request = parse_request(raw, rospy.get_param_cached('/grasp_mode/selection'),
                    rospy.Time.now().to_nsec(), previous if raw.get('state') == 'retire' else None)
                if request['state'] == 'prepare' and previous is not None:
                    raise ValueError('CONTACT_STATION_REPLAY_OR_ALREADY_PREPARED')
                reference, marker = self._contact_station_context(request)
                if request['state'] == 'prepare':
                    self._contact_station_prepared = deepcopy(request)
                else:
                    # No geometry or lost-target status changes here. Frames
                    # already computing at the old station cannot commit later.
                    # A frozen base-frame prism may still propose a search
                    # crop under each NEW camera TF. It carries no pixels,
                    # accepted measurement, stationary-time allowance or q.
                    self._contact_spatial_context = (dict(
                        binding=deepcopy(request),
                        spatial_reference=deepcopy(marker['spatial_reference']),
                        source_id=marker['source_id'], config_sha256=marker['config_sha256'],
                        camera_sha256=marker['camera_configuration_sha256'],
                        anchor_fingerprint=marker['anchor_fingerprint'],
                        reference_plane=marker['reference_plane'].copy(),
                        deadline_ns=min(marker['session_deadline_ns'], self._station_compute_phase.deadline.to_nsec()),
                        session_deadline_ns=marker['session_deadline_ns'],
                        deadline_monotonic=marker['session_deadline_monotonic'])
                        if marker.get('spatial_reference') is not None else None)
                    if self._contact_spatial_context is not None:
                        for value in self._contact_spatial_context['spatial_reference'].values():
                            if isinstance(value, np.ndarray):
                                value.setflags(write=False)
                    reference.pop('observation_arrival_retry')
                    self._arrival_frame_completion = None
                    self._station_compute_registration = None
                    self._station_compute_frame = None
                    self._contact_station_version = getattr(self, '_contact_station_version', 0) + 1
                    self._contact_station_status = deepcopy(request)
                    self._contact_station_prepared = None
                self._contact_station_ack_pub.publish(String(data=json.dumps(ack(request, 'unknown'))))
            except (ValueError, KeyError, TypeError, AttributeError) as exc:
                if request is not None:
                    self._contact_station_ack_pub.publish(String(data=json.dumps(ack(request, 'unknown', False, str(exc)))))
                else:
                    rospy.logwarn('Contact station request rejected: %s', exc)

    def _contact_station_frame_retired(self, version):
        # Only a successful exact handoff increments this counter. Arbitrary
        # movement, manual control or failed measurements never gain this exit.
        return version != getattr(self, '_contact_station_version', 0)

    def _station_compute_state_cb(self, message):
        with self.input_lock:
            if not message.active or message.state not in ('MOVE_PREGRASP', 'APPROACH_TARGET'):
                context = getattr(self, '_contact_spatial_context', None)
                previous = getattr(self, '_station_compute_task_state', None)
                normal = (context is not None and previous is not None and previous.active
                    and (previous.state in ('MOVE_PREGRASP', 'APPROACH_TARGET')
                         or context.get('normal_phase_end') is True)
                    and self._contact_spatial_final_state(message, context))
                self._end_contact_spatial_crop(normal)
            super()._station_compute_state_cb(message)

    def _station_compute_phase_cb(self, message):
        with self.input_lock:
            context = getattr(self, '_contact_spatial_context', None)
            if context is not None:
                binding = context['binding']
                if (not message.active or message.planning_stage != 'contact_replan'
                        or message.phase_id != binding['phase_id']
                        or message.header.stamp.to_nsec() != binding['phase_started_ns']
                        or message.reference_target_track_id != binding['target_track_id']):
                    self._end_contact_spatial_crop(self._contact_spatial_final_phase(message, context))
            super()._station_compute_phase_cb(message)

    @staticmethod
    def _contact_spatial_final_state(message, context):
        try:
            return (message.active is True and message.state == 'PLAN_PREGRASP'
                and message.stage == GraspStages.PLAN_PREGRASP
                and message.message == 'waiting for bounded final visual refinement'
                and context['binding']['stamp_ns'] <= message.header.stamp.to_nsec()
                    <= rospy.Time.now().to_nsec())
        except (AttributeError, KeyError, TypeError, ValueError):
            return False

    @staticmethod
    def _contact_spatial_final_phase(message, context):
        try:
            phase = binding_from_phase(message, {})
            binding = context['binding']
            return (phase is not None and phase.stage == 'final_refinement'
                and phase.phase_id > binding['phase_id']
                and binding['stamp_ns'] <= phase.started_ns <= rospy.Time.now().to_nsec()
                and phase.target_track_id == binding['target_track_id']
                and phase.plan_id == binding['plan_id']
                and committed_plan_digest(message.reference_contact_plan) == binding['plan_sha256'])
        except (AttributeError, KeyError, TypeError, ValueError):
            return False

    def _contact_spatial_identity_matches(self, context, camera):
        binding = context['binding']
        completed = getattr(self, '_observation_measurement_status', None) or {}
        return (self.mode == 'unknown' and self.generation == binding['selection']['generation']
            and self.selection_ns == binding['selection']['stamp_ns']
            and parse_selection(rospy.get_param_cached('/grasp_mode/selection')) == binding['selection']
            and self._rgb_interest_epoch == binding['control_epoch_ns']
            and rospy.get_param_cached('/gui/joint_direct_mode', False) is False
            and self.first_loss is None and not self.tracker.lost
            and completed.get('state') == 'completed'
            and measurement_digest(completed.get('request') or {}) == binding['measurement_sha256']
            and getattr(self, '_observation_measurement', None) is None
            and digest(asdict(self.config)) == context['config_sha256']
            and digest(_camera_configuration_contract(camera)) == context['camera_sha256']
            and _candidate_fingerprint(self.tracker.anchor) == context['anchor_fingerprint']
            and np.array_equal(self.tracker.plane_base, context['reference_plane']))

    def _end_contact_spatial_crop(self, normal):
        context = getattr(self, '_contact_spatial_context', None)
        if context is None:
            return
        try:
            if normal and self._contact_spatial_identity_matches(context, rospy.get_param_cached('/camera')):
                # Keep only the existing object until the single Timer's
                # in-flight frame is discarded. This cannot renew a deadline
                # or authorize geometry. A later real revocation still clears
                # the pointer, so restoring its fields cannot revive the frame.
                context['normal_phase_end'] = True
                return
        except (ValueError, KeyError, TypeError, AttributeError):
            pass
        self._contact_spatial_context = None

    def _contact_spatial_reference(self, source, camera, expected=None):
        """Current-frame crop proposal only, bounded by the original phase.

        The source-age request deadline belongs to the completed handshake; this
        spatial template instead expires with its existing contact phase/session.
        A phase-only expiry returns None so an in-flight crop frame is discarded,
        not published as target loss. Owner/session faults still revoke a captured
        context; restoring those inputs cannot revive it.
        """
        context = getattr(self, '_contact_spatial_context', None)
        if context is None:
            if expected is not None:
                raise ValueError('contact_spatial_context_revoked_before_commit')
            return None
        try:
            binding = context['binding']
            phase = getattr(self, '_station_compute_phase', None)
            state = getattr(self, '_station_compute_task_state', None)
            normal_end = context.get('normal_phase_end') is True
            phase_matches = (phase is not None and phase.active and phase.planning_stage == 'contact_replan'
                and phase.phase_id == binding['phase_id']
                and phase.header.stamp.to_nsec() == binding['phase_started_ns']
                and phase.reference_target_track_id == binding['target_track_id'])
            state_matches = (state is not None and state.active
                and state.state in ('MOVE_PREGRASP', 'APPROACH_TARGET'))
            if (expected is not None and context is not expected
                    or not self._contact_spatial_identity_matches(context, camera)
                    or not (phase_matches or normal_end and self._contact_spatial_final_phase(phase, context))
                    or not (state_matches or normal_end and self._contact_spatial_final_state(state, context))
                    or digest(self._rgb_interest_source_context(source[0].header.stamp)) != context['source_id']):
                raise ValueError('contact_spatial_context_changed')
            now = rospy.Time.now().to_nsec()
            if (now >= context['session_deadline_ns']
                    or time.monotonic() >= context['deadline_monotonic']):
                raise ValueError('contact_spatial_context_changed')
            require_fresh_source(source[0].header.stamp.to_nsec(), now, min(self.max_age, SOURCE_MAX_AGE_SEC))
            if normal_end or now >= min(context['deadline_ns'], phase.deadline.to_nsec()):
                self._contact_spatial_context = None
                return None
            return context
        except (ValueError, KeyError, TypeError, AttributeError):
            self._contact_spatial_context = None
            if expected is not None:
                raise ValueError('contact_spatial_context_revoked_before_commit')
            return None
