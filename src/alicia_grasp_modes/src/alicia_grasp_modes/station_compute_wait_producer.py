"""Fresh READY enrollment and identity-only recovery at a qualified station.

No stale candidate survives this protocol. It only retains the frozen identity
reference while a later independent RGB-D measurement runs all original gates.
"""
from copy import deepcopy
from dataclasses import asdict
import time
import hashlib

import cv2
import numpy as np
import rospy
import tf2_ros

from .station_compute_reacquisition import (build_contract, validate_contract, validate_registration,
    build_station_support_gap, validate_station_support_gap)
from .unknown_pending_identity import reference_binding
from .rgb_observation_source import digest, require_fresh_source
from .rgb_observation_handoff import _candidate_fingerprint, _appearance, HandoffPolicy
from .selection import parse_selection
from .tabletop import _matching_candidates, _validate_support_plane_update
from .feedback_bracket_gap import validate_span_gap_evidence
from alicia_flexible_grasp.robot.observation_view_gateway import _camera_configuration_contract


class ObservationArrivalStationChanged(ValueError):
    """A revoked fixed station cannot be handled as transient RGB failure."""


class StationComputeDeadline(ValueError):
    """A discarded frame's clock expired under an independently intact lease."""


class StationFeedbackBracketGap(ValueError):
    """Unusable current bracket under an existing qualified-arrival contract."""
    def __init__(self, registration, evidence):
        super().__init__('station_accepted_feedback_bracket_pending_revalidation')
        self.registration, self.evidence = registration, deepcopy(evidence)


class StationComputeRetired(ValueError):
    """A timely in-flight calculation belongs to a retired planning phase."""
    def __init__(self, registration):
        super().__init__('station_compute_frame_retired')
        self.registration = registration


class StationComputeWaitProducerMixin:
    def _station_requalification_feedback(self, stamp_ns, q):
        """Revoke this station, retaining only its already registered identity."""
        from .observe_return_measurement import station_requalification_witness, station_requalification_key
        if getattr(self, '_station_requalification_hold', None) is not None:
            return
        registration = getattr(self, '_station_compute_registration', None)
        if registration is None:
            return
        try:
            reference, marker = registration['reference'], registration['marker']
            contract = registration['contract']
            if (self._locked_rgb_hold_reference is not reference
                    or reference.get('observation_arrival_retry') is not marker
                    or reference['pose_invalidated'] or marker.get('invalidated_reason')
                    or self.first_loss is not None or self.tracker.lost
                    or self._station_compute_binding(reference) != contract['binding']
                    or getattr(self, '_observation_measurement', None) is not None
                    or rospy.get_param_cached('/gui/joint_direct_mode', False) is not False
                    or self._rgb_interest_epoch != marker['epoch_ns']
                    or time.monotonic() >= registration['monotonic_deadline']):
                return
            key = station_requalification_key(contract)
            if key in getattr(self, '_station_requalification_used', set()):
                return
            now = rospy.Time.now().to_nsec()
            witness = station_requalification_witness(contract, stamp_ns, np.asarray(q).tolist(), now)
            self._station_requalification_hold = dict(witness=witness, registration=registration,
                reference=reference, marker=marker, anchor=self.tracker.anchor, last=self.tracker.last,
                monotonic_deadline=registration['monotonic_deadline'], new_lease=None,
                proven_feedback=(stamp_ns, np.asarray(q).tolist()), finalizing=False)
            # SDK ingress will remove the old marker immediately afterwards.
            # The tombstone is never a replacement live geometric reference.
        except (ValueError, KeyError, TypeError, AttributeError):
            return

    def _station_requalification_check(self, request=None):
        from .observe_return_measurement import STATION_REQUALIFICATION, validate_station_requalification
        from .selection import parse_selection
        hold = getattr(self, '_station_requalification_hold', None)
        if hold is None:
            raise ValueError('STATION_REQUALIFICATION_NOT_LOCALLY_ARMED')
        value = hold['witness'] if request is None else request[STATION_REQUALIFICATION]
        parent = value['parent_contract']
        proven = getattr(self, '_station_requalification_parents', {}).get(parent['source_ns'])
        if proven != parent or parent['binding'] != hold['witness']['parent_contract']['binding']:
            raise ValueError('STATION_REQUALIFICATION_PARENT_UNREGISTERED')
        feedback = tuple(self._rgb_interest_feedback)
        if not feedback:
            raise ValueError('STATION_REQUALIFICATION_FEEDBACK_STALE')
        require_fresh_source(max(ns for ns,_ in feedback), rospy.Time.now().to_nsec(), min(self.max_age,.8))
        # Preserve independently observed SDK evidence after its rolling buffer
        # naturally evicts the original sample; a request cannot create this.
        feedback += (hold['proven_feedback'],)
        validate_station_requalification(value, parent=proven,
            feedback=feedback, now_ns=rospy.Time.now().to_nsec())
        hold['monotonic_deadline'] = min(hold['monotonic_deadline'],
            self._station_requalification_parent_mono[parent['source_ns']])
        b = parent['binding']; marker = hold['marker']
        if (self.first_loss is not None or self.tracker.lost
                or self.tracker.anchor is not hold['anchor'] or self.tracker.last is not hold['last']
                or self._rgb_interest_epoch != b['control_epoch_ns']
                or (self.mode, self.generation, self.selection_ns) !=
                   ('unknown', b['selection']['generation'], b['selection']['stamp_ns'])
                or parse_selection(rospy.get_param_cached('/grasp_mode/selection')) != b['selection']
                or rospy.get_param_cached('/gui/joint_direct_mode', False) is not False
                or digest(asdict(self.config)) != b['config_sha256']
                or digest(_camera_configuration_contract(rospy.get_param_cached('/camera', {})))
                   != b['camera_configuration_sha256']
                or not np.array_equal(self.tracker.plane_base, marker['reference_plane'])
                or self._station_compute_phase.phase_id != b['phase_id']
                or self._station_compute_phase.deadline.to_nsec() != parent['phase_deadline_ns']
                or not self._station_compute_phase.active
                or self._station_compute_task_state.state != 'PLAN_PREGRASP'
                or not self._station_compute_task_state.active
                or time.monotonic() >= hold['monotonic_deadline']
                or (request is not None and hold['new_lease'] not in (None, request['lease_id']))):
            raise ValueError('STATION_REQUALIFICATION_OWNER_CHANGED')
        return hold

    def _station_requalification_queue(self, request):
        """Defer ACK until the same drift is independently received via SDK."""
        from .observe_return_measurement import STATION_REQUALIFICATION, station_requalification_key
        value = request[STATION_REQUALIFICATION]; parent = value['parent_contract']
        proven = getattr(self, '_station_requalification_parents', {}).get(parent['source_ns'])
        registration = getattr(self, '_station_compute_registration', None)
        previous = getattr(self, '_station_requalification_admission', None)
        if (proven != parent or registration is None
                or registration['contract']['binding'] != parent['binding']
                or station_requalification_key(parent) in getattr(self, '_station_requalification_used', set())
                or (previous is not None and previous['request'] != request)):
            raise ValueError('STATION_REQUALIFICATION_PARENT_UNREGISTERED')
        self._station_compute_scope(registration)
        self._station_requalification_admission = dict(request=deepcopy(request),
            registration=registration,
            monotonic_deadline=self._station_requalification_parent_mono[parent['source_ns']])

    def _station_requalification_admission_tick(self, source):
        from .observe_return_measurement import STATION_REQUALIFICATION
        from std_msgs.msg import String
        import json
        pending = getattr(self, '_station_requalification_admission', None)
        if pending is None:
            return False
        request = pending['request']; value = request[STATION_REQUALIFICATION]
        try:
            if (rospy.Time.now().to_nsec() >= value['deadline_ns']
                    or time.monotonic() >= pending['monotonic_deadline']):
                raise ValueError('STATION_REQUALIFICATION_EXPIRED')
            if getattr(self, '_station_requalification_hold', None) is None:
                self._station_compute_scope(pending['registration'])
                return True
            # Clear only this queued envelope. Admission still rechecks both
            # the historical registered parent and actual local SDK witness.
            self._station_requalification_admission = None
            self._observation_measurement_request_cb(String(data=json.dumps(request)))
            return True
        except (ValueError, KeyError, TypeError, AttributeError) as exc:
            self._station_requalification_admission = None
            self._measurement_ack(request, False, str(exc))
            self.unavailable('station_requalification_rejected: '+str(exc),
                None if source is None else source[0].header.stamp, source)
            return True

    def _station_requalification_finalizing_check(self, binding=None):
        hold = getattr(self, '_station_requalification_hold', None)
        if hold is None or not hold.get('finalizing'):
            return
        parent = hold['witness']['parent_contract']; b = parent['binding']
        if (rospy.Time.now().to_nsec() >= hold['witness']['deadline_ns']
                or time.monotonic() >= hold['monotonic_deadline']
                or self.first_loss is not None or self.tracker.lost
                or self.tracker.anchor is not hold['anchor']
                or self._rgb_interest_epoch != b['control_epoch_ns']
                or parse_selection(rospy.get_param_cached('/grasp_mode/selection')) != b['selection']
                or rospy.get_param_cached('/gui/joint_direct_mode', False) is not False
                or not self._station_compute_task_state.active
                or self._station_compute_task_state.state != 'PLAN_PREGRASP'
                or not self._station_compute_phase.active
                or self._station_compute_phase.deadline.to_nsec() != parent['phase_deadline_ns']):
            raise ValueError('STATION_REQUALIFICATION_FINAL_REGISTRATION_EXPIRED_OR_CHANGED')
        if binding is not None and (binding['phase_id'] <= b['phase_id']
                or binding['lease_id'] != hold['new_lease']
                or any(binding[k] != b[k] for k in ('selection','session_id','view_plan_id',
                    'view_index','target_epoch','target_track_id','control_epoch_ns'))):
            raise ValueError('STATION_REQUALIFICATION_NEW_PHASE_REQUIRED')

    def _station_requalification_wait(self, source, error=None):
        from .observe_return_measurement import STATION_REQUALIFICATION
        if getattr(self, '_station_requalification_hold', None) is None:
            return False
        if getattr(self, '_observation_measurement', None) is not None:
            return False
        if self._station_requalification_hold.get('finalizing'):
            return False
        if error is not None and str(error) not in (
                "'observation_arrival_retry'", 'station_compute_wait_station_changed',
                'observation_arrival_station_changed', 'observation_arrival_station_invalidated_before_publish',
                'observation_arrival_station_invalidated_at_commit',
                'observation_arrival_station_changed_during_processing'):
            return False
        try:
            self._station_requalification_check()
        except (ValueError, KeyError, TypeError, AttributeError) as exc:
            self._station_requalification_hold = None
            self.unavailable('station_requalification_rejected: '+str(exc),
                None if source is None else source[0].header.stamp, source)
            return True
        # A state-only notification avoids turning a known discarded station
        # into a new negative object measurement. Geometry is revoked by both
        # consumers from their own SDK ingress before any new plan may submit.
        self.status('pending', STATION_REQUALIFICATION, source)
        return True

    def _require_observation_arrival_marker(self, reference, expected=None,
                                            reason='observation_arrival_station_changed'):
        """Read a live marker without racing callback revocation.

        This is a boundary check, not a lease or a copy of station authority.
        Expensive work must still run its original source/pose checks and the
        publication commit must revalidate under input_lock.
        """
        with self.input_lock:
            marker = (reference or {}).get('observation_arrival_retry')
            completion = getattr(self, '_arrival_frame_completion', None)
            if (expected is None and completion is not None
                    and completion['reference'] is reference
                    and completion['marker_kind'] == 'observation_arrival_retry'):
                expected = completion['marker']
            if (reference is None or self._locked_rgb_hold_reference is not reference
                    or marker is None or (expected is not None and marker is not expected)
                    or marker.get('invalidated_reason') or reference['pose_invalidated']
                    or self._rgb_interest_epoch != marker['epoch_ns']
                    or rospy.get_param_cached('/gui/joint_direct_mode', False) is True):
                raise ObservationArrivalStationChanged(reason)
            return marker

    def _station_compute_retire_registration(self):
        registration = getattr(self, '_station_compute_registration', None)
        frame = getattr(self, '_station_compute_frame', None)
        if registration is not None and frame is not None and frame['registration'] is registration:
            try:
                # Validate the existing owner BEFORE replacing its phase/state.
                # A late frame or a real station fault is never this transition.
                self._station_compute_scope(registration)
                now = rospy.Time.now().to_nsec()
                require_fresh_source(registration['reference']['source_stamp_ns'], now, min(self.max_age, .8))
                require_fresh_source(frame['source_ns'], now, min(self.max_age, .8))
                registration['retirement'] = dict(stamp_ns=now,
                    phase=deepcopy(self._station_compute_phase),
                    state=deepcopy(self._station_compute_task_state), frame=frame)
            except (ValueError, KeyError, TypeError, AttributeError):
                pass
        self._station_compute_registration = None

    def _station_compute_phase_cb(self, message):
        with self.input_lock:
            previous = getattr(self, '_station_compute_phase', None)
            if (not message.active or message.planning_stage != 'contact_replan'
                    or (previous is not None and previous != message)):
                self._station_compute_retire_registration()
            if previous != message:
                self._station_compute_phase_received_ns = rospy.Time.now().to_nsec()
            self._station_compute_phase = deepcopy(message)

    def _station_compute_state_cb(self, message):
        with self.input_lock:
            if message.state != 'PLAN_PREGRASP' or not message.active:
                self._station_compute_retire_registration()
            self._station_compute_task_state = deepcopy(message)

    def _station_compute_binding(self, reference, payload=None, *, phase=None, state=None, marker=None):
        marker = self._require_observation_arrival_marker(reference, marker,
            reason='station_compute_wait_station_changed')
        request = marker['measurement_binding']
        phase = self._station_compute_phase if phase is None else phase
        state = self._station_compute_task_state if state is None else state
        selection = parse_selection(rospy.get_param_cached('/grasp_mode/selection'))
        if (state.state != 'PLAN_PREGRASP' or not state.active or not phase.active
                or phase.planning_stage != 'contact_replan'
                or phase.reference_target_track_id != request['target_track_id']
                or phase.reference_source_stamp.to_nsec() < marker['qualified_source_ns']
                or selection != request['selection']
                or (self.mode, self.generation, self.selection_ns) !=
                   ('unknown', selection['generation'], selection['stamp_ns'])):
            raise ValueError('station_compute_wait_owner_changed')
        if payload is None:
            ref_binding = reference['station_compute_reference_binding']
        else:
            ref_binding = reference_binding(payload, selection)
        self._require_observation_arrival_marker(reference, marker,
            reason='station_compute_wait_station_changed')
        return dict(selection=selection,
            **{key: request[key] for key in ('lease_id', 'session_id', 'view_plan_id',
                'view_index', 'target_epoch', 'target_track_id', 'control_epoch_ns')},
            phase_id=int(phase.phase_id), phase_started_ns=phase.header.stamp.to_nsec(),
            phase_reference_source_ns=phase.reference_source_stamp.to_nsec(),
            reference_binding=ref_binding, source_id=reference['source_id'],
            config_sha256=marker['config_sha256'],
            camera_configuration_sha256=marker['camera_configuration_sha256'],
            station_q_rad=np.asarray(marker['arrival_q']).tolist())

    def _station_compute_scope(self, registration, camera=None, *, permit_retired=False):
        """Recheck positive station facts separately from every frame-age gate."""
        reference, marker = registration['reference'], registration['marker']
        support_gap = registration.get('support_gap')
        if support_gap is not None:
            validate_station_support_gap(support_gap, contract=registration['contract'],
                now_ns=rospy.Time.now().to_nsec())
            if time.monotonic() >= registration['support_gap_monotonic']:
                raise ValueError('station_support_gap_deadline_expired')
        retired = getattr(self, '_station_compute_registration', None) is not registration
        retirement = registration.get('retirement') if retired else None
        if ((retired and retirement is None)
                or self._locked_rgb_hold_reference is not reference
                or reference.get('observation_arrival_retry') is not marker
                or marker.get('invalidated_reason') or reference['pose_invalidated']
                or self.first_loss is not None or self.tracker.lost
                or self.last_good_source_stamp_ns != reference['source_stamp_ns']
                or getattr(self, '_locked_source_pending', None) is not None
                or getattr(self, '_metric_ambiguity_pending', None) is not None
                or getattr(self, '_observation_measurement', None) is not None
                or rospy.get_param_cached('/gui/joint_direct_mode', False) is True
                or self._rgb_interest_epoch != marker['epoch_ns']
                or digest(asdict(self.config)) != marker['config_sha256']
                or _candidate_fingerprint(self.tracker.anchor) != marker['anchor_fingerprint']
                or _candidate_fingerprint(self.tracker.last) != reference['last_fingerprint']
                or not np.array_equal(self.tracker.plane_base, marker['reference_plane'])
                or time.monotonic() >= registration['monotonic_deadline']
                or time.monotonic() >= marker['session_deadline_monotonic']):
            raise ValueError('station_compute_wait_station_changed')
        phase = retirement['phase'] if retired else self._station_compute_phase
        state = retirement['state'] if retired else self._station_compute_task_state
        binding = self._station_compute_binding(reference, phase=phase, state=state, marker=marker)
        if (phase.deadline.to_nsec() != registration['contract']['phase_deadline_ns']
                or marker['session_deadline_ns'] != registration['contract']['session_deadline_ns']):
            raise ValueError('station_compute_wait_deadline_changed')
        validate_contract(registration['contract'], binding=binding, now_ns=rospy.Time.now().to_nsec())
        if camera is None:
            camera = rospy.get_param_cached('/camera', {})
        if digest(_camera_configuration_contract(camera)) != marker['camera_configuration_sha256']:
            raise ValueError('station_compute_wait_camera_changed')
        with self.input_lock:
            feedback = tuple(self._rgb_interest_feedback)
            if not feedback:
                raise ValueError('station_compute_wait_feedback_missing')
            feedback_ns = max(row[0] for row in feedback)
            checked_ns = rospy.Time.now().to_nsec()
            try:
                require_fresh_source(feedback_ns, checked_ns, min(self.max_age, .8))
            except ValueError as exc:
                # Private, typed provenance for the empty-input watchdog.
                # A same-spelled error from another gate cannot request retry.
                if (type(feedback_ns) is int and feedback_ns > 0
                        and checked_ns-feedback_ns > round(min(self.max_age, .8)*1e9)):
                    exc.station_idle_feedback_failure = dict(registration=registration,
                        reference=reference, marker=marker,
                        feedback_stamp_ns=feedback_ns, checked_ns=checked_ns)
                raise
            positions = [q for ns, q in feedback if ns >= marker['qualified_source_ns']]
            if not positions or any(np.max(np.abs(q-marker['arrival_q'])) > .004 for q in positions):
                raise ValueError('station_compute_wait_station_moved')
        self._require_observation_arrival_marker(reference, marker,
            reason='station_compute_wait_station_changed')
        if retired:
            frame = getattr(self, '_station_compute_frame', None)
            now = rospy.Time.now().to_nsec()
            if (frame is not retirement['frame'] or frame['registration'] is not registration
                    or not frame['entry_ns'] <= retirement['stamp_ns'] <= now
                    or not getattr(self, '_station_compute_task_state', None).active
                    or self._station_compute_task_state.state not in ('PLAN_PREGRASP', 'MOVE_PREGRASP')):
                raise ValueError('station_compute_retirement_context_changed')
            # Cancellation cannot carry old identity beyond the original metric
            # lifetime. A next lease must still freeze a fresh measured source.
            require_fresh_source(reference['source_stamp_ns'], now, min(self.max_age, .8))
            require_fresh_source(frame['source_ns'], now, min(self.max_age, .8))
            if not permit_retired:
                raise StationComputeRetired(registration)
        self._require_observation_arrival_marker(reference, marker,
            reason='station_compute_wait_station_changed')
        return reference

    def _station_compute_retired_frame(self, source, error):
        if not isinstance(error, StationComputeRetired):
            return False
        try:
            # The original arrival gate also binds robot/camera/hand-eye
            # configuration; retirement cannot bypass its source-context check.
            reference = error.registration['reference']
            if digest(self._rgb_interest_source_context(source[0].header.stamp)) != reference['source_id']:
                return False
            with self.input_lock:
                frame = getattr(self, '_station_compute_frame', None)
                if frame is None or frame['source'] is not source:
                    return False
                self._station_compute_scope(error.registration, permit_retired=True)
                timing = getattr(self, '_processing_timing', None)
                if timing is not None:
                    timing['station_compute_retired'] = dict(source_ns=frame['source_ns'],
                        retired_ns=error.registration['retirement']['stamp_ns'],
                        metric_geometry_authority=False, motion_authorized=False,
                        reference_refreshed=False)
                rospy.loginfo('Discarded retired station calculation source=%s; reference unchanged', frame['source_ns'])
                return True
        except (ValueError, KeyError, TypeError, AttributeError, tf2_ros.LookupException,
                tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return False

    def _arrival_phase_begin_frame(self, source):
        """Snapshot a pre-contact owner; never enroll geometry or renew identity."""
        self._arrival_phase_frame = None
        reference = getattr(self, '_locked_rgb_hold_reference', None)
        marker = (reference or {}).get('observation_arrival_retry')
        previous = getattr(self, '_station_compute_phase', None)
        if (source is None or marker is None
                or getattr(self, '_station_compute_registration', None) is not None
                or (previous is not None and previous.active)):
            return
        self._arrival_phase_frame = dict(source=source, reference=reference, marker=marker,
            previous_phase=deepcopy(previous), entry_ns=rospy.Time.now().to_nsec())

    def _arrival_phase_checkpoint(self, source, camera):
        """Cancel only timely old-owner work, before another fallible operation.

        This is not exception recovery: known input/identity failures retain
        their original rejection. A later frame still needs its own complete
        measurement and a source strictly after the new phase start.
        """
        with self.input_lock:
            frame = getattr(self, '_arrival_phase_frame', None)
            if frame is None or frame['source'] is not source:
                return False
            phase = getattr(self, '_station_compute_phase', None)
            if phase is None or phase == frame['previous_phase']:
                return False
            state = getattr(self, '_station_compute_task_state', None)
            reference, marker = frame['reference'], frame['marker']
            now = rospy.Time.now().to_nsec()
            received = getattr(self, '_station_compute_phase_received_ns', 0)
            source_ns = source[0].header.stamp.to_nsec()
            request = marker['measurement_binding']
            if (not phase.active or phase.planning_stage != 'contact_replan'
                    or state is None or not state.active or state.state != 'PLAN_PREGRASP'
                    or not frame['entry_ns'] <= received <= now
                    or not reference['source_stamp_ns'] < source_ns < phase.header.stamp.to_nsec() <= received
                    or not now < phase.deadline.to_nsec()
                    or phase.reference_source_stamp.to_nsec() not in
                       (reference['source_stamp_ns'], marker['qualified_source_ns'])
                    or phase.reference_target_track_id != request['target_track_id']
                    or self._locked_rgb_hold_reference is not reference
                    or reference.get('observation_arrival_retry') is not marker
                    or getattr(self, '_locked_source_pending', None) is not None
                    or getattr(self, '_metric_ambiguity_pending', None) is not None
                    or getattr(self, '_observation_measurement', None) is not None):
                return False
            # Cancellation itself stays inside the ORIGINAL .8s clock. It
            # cannot create a pending identity or resurrect an expired frame.
            require_fresh_source(reference['source_stamp_ns'], now, min(self.max_age, .8))
            require_fresh_source(source_ns, now, min(self.max_age, .8))
            if (parse_selection(rospy.get_param_cached('/grasp_mode/selection')) != request['selection']
                    or self._observation_arrival_retry_reference(source, camera, expected=reference) is not reference):
                raise ValueError('arrival_phase_changed_with_invalid_station')
            require_fresh_source(reference['source_stamp_ns'], rospy.Time.now().to_nsec(), min(self.max_age, .8))
            timing = getattr(self, '_processing_timing', None)
            if timing is not None:
                timing['arrival_phase_frame_cancelled'] = dict(source_stamp_ns=source_ns,
                    phase_id=phase.phase_id, phase_started_ns=phase.header.stamp.to_nsec(),
                    phase_received_ns=received, reference_refreshed=False,
                    metric_geometry_authority=False, motion_authorized=False)
            rospy.loginfo('Discarded first-phase predecessor source=%s phase=%s; reference unchanged',
                source_ns, phase.phase_id)
            return True

    def _station_compute_begin_frame(self, source):
        # Registration and frame enrollment share the callback retirement lock.
        with self.input_lock:
            self._station_compute_frame = None
            registration = getattr(self, '_station_compute_registration', None)
            if registration is None:
                return
            self._station_compute_scope(registration)
            ns = source[0].header.stamp.to_nsec()
            validate_contract(registration['contract'], binding=self._station_compute_binding(registration['reference'], marker=registration['marker']),
                now_ns=rospy.Time.now().to_nsec(), source_ns=ns)
            if source[1].header.stamp.to_nsec() != ns:
                raise ValueError('station_compute_wait_source_mismatch')
            self._station_compute_frame = dict(registration=registration, source=source,
                source_ns=ns, entry_ns=rospy.Time.now().to_nsec())

    def _station_compute_reference_lifetime(self, reference, marker, source_ns, now_ns):
        frame = getattr(self, '_station_compute_frame', None)
        if frame is None:
            return False
        registration = frame['registration']
        self._station_compute_scope(registration)
        if (registration['reference'] is not reference or registration['marker'] is not marker
                or frame['source_ns'] != source_ns):
            raise ValueError('station_compute_wait_frame_changed')
        maximum_ns = registration['contract']['geometry_deadline_ns']-registration['contract']['source_ns']
        # The owner/SDK scope audit consumes the same source budget. Evaluate
        # its deadline at audit completion, preserving this typed outcome.
        now_ns = max(now_ns, rospy.Time.now().to_nsec())
        if now_ns-source_ns >= maximum_ns:
            raise StationComputeDeadline('station_compute_current_measurement_expired')
        validate_contract(registration['contract'], binding=self._station_compute_binding(reference, marker=marker),
            now_ns=now_ns, source_ns=source_ns)
        return True

    def _station_compute_recovery_evidence(self, source, result, candidate, camera):
        frame = getattr(self, '_station_compute_frame', None)
        if frame is None:
            return
        registration = frame['registration']
        self._station_compute_scope(registration, camera)
        try:
            self._station_compute_measure_recovery(source, result, candidate, camera)
        except ValueError as exc:
            # Optional proof lets a receiver bridge a missed READY. It cannot
            # add a new rejection to an otherwise timely normal measurement.
            # An actually expired reference, however, requires all this proof.
            self._station_compute_scope(registration, camera)
            frame['recovery_proof_error'] = str(exc)
            if (registration.get('feedback_gap') is not None
                    or registration.get('support_gap') is not None
                    or rospy.Time.now().to_nsec() >= registration['contract']['geometry_deadline_ns']):
                raise

    def _station_compute_measure_recovery(self, source, result, candidate, camera):
        frame = getattr(self, '_station_compute_frame', None)
        if frame is None:
            return
        registration = frame['registration']
        # A receiver may have missed an intermediate READY and be PENDING.
        # Compute optional proof without changing the normal READY verdict.
        reference = self._station_compute_scope(registration, camera)
        marker = registration['marker']
        if len(_matching_candidates(result.candidates, self.tracker.anchor, self.tracker.last, self.config)) != 1:
            raise ValueError('station_compute_recovery_unique_geometry_required')
        if len(_matching_candidates([candidate], self.tracker.anchor, self.tracker.last, self.config)) != 1:
            raise ValueError('station_compute_recovery_final_geometry_required')
        metrics = result.metrics
        if metrics.get('support_current_audit_available') is not True:
            raise ValueError('station_compute_recovery_current_support_required')
        if (metrics.get('support_current_audit_quadrants') not in (3, 4)
                or metrics.get('support_reference_footprint_vertices') != len(reference['reference_footprint'])):
            raise ValueError('station_compute_recovery_support_footprint_changed')
        _validate_support_plane_update(reference['reference_plane'], result.plane_base,
            self.tracker.anchor.position_base, self.config, reference['reference_footprint'])
        for key, low, high in (('support_current_audit_inlier_ratio', self.config.support_min_ratio, 1.),
                ('support_current_audit_residual_m', 0., self.config.support_threshold_m),
                ('support_current_audit_quadrants', 3, 4)):
            value = metrics[key]
            if not np.isfinite(value) or not low <= value <= high:
                raise ValueError('station_compute_recovery_support_evidence_invalid')
        rgb = self.bridge.imgmsg_to_cv2(source[0], desired_encoding='bgr8')
        previous, current = reference['mask'] > 0, candidate.mask > 0
        iou = float(np.count_nonzero(previous & current)/np.count_nonzero(previous | current))
        if not iou >= .85:
            raise ValueError('station_compute_recovery_rgb_identity_required')
        from .observation_appearance import match_observation_appearance
        from alicia_flexible_grasp.vision.rgbd_rectification import rectification_rgb_valid_mask
        def check_appearance_source():
            self._station_compute_scope(registration, camera)
            require_fresh_source(source[0].header.stamp.to_nsec(),rospy.Time.now().to_nsec(),min(self.max_age,.8))
        appearance_histogram, appearance_contour, _ = self._observation_marker_appearance(marker)
        appearance = match_observation_appearance(rgb,candidate.mask,
            appearance_histogram, appearance_contour,
            rgb_valid=rectification_rgb_valid_mask(camera,candidate.mask.shape),
            appearance_context=self._observation_marker_appearance_context(source,marker,check_appearance_source))
        if not appearance['appearance_match']:
            raise ValueError('station_compute_recovery_rgb_identity_required')
        frame['recovery_evidence'] = dict(mask_iou=iou, colour_distance=appearance['colour_distance'],
            contour_distance=appearance['contour_distance'], appearance_identity=appearance)

    def _station_compute_commit_check(self, source, camera, result, candidate):
        frame = getattr(self, '_station_compute_frame', None)
        if frame is None:
            return
        registration = frame['registration']
        reference = self._station_compute_scope(registration, camera)
        self._station_compute_reference_lifetime(reference, registration['marker'],
            source[0].header.stamp.to_nsec(), rospy.Time.now().to_nsec())
        if ((registration.get('feedback_gap') is not None
                    or registration.get('support_gap') is not None
                or rospy.Time.now().to_nsec() >= registration['contract']['geometry_deadline_ns'])
                and 'recovery_evidence' not in frame):
            self._station_compute_recovery_evidence(source, result, candidate, camera)
            self._station_compute_reference_lifetime(reference, registration['marker'],
                source[0].header.stamp.to_nsec(), rospy.Time.now().to_nsec())

    def _station_compute_status(self, payload, source):
        from .observe_return_measurement import STATION_REQUALIFICATION
        if (payload['detail'] == STATION_REQUALIFICATION
                and getattr(self, '_station_requalification_hold', None) is not None):
            payload[STATION_REQUALIFICATION] = deepcopy(self._station_requalification_hold['witness'])
            return
        if payload['state'] != 'ready':
            if payload['detail'] in ('processing_deadline_pending',
                    'station_accepted_feedback_bracket_pending_revalidation'):
                registration = self._station_compute_registration
                payload.update(state='processing_deadline_pending',
                    station_compute_wait=deepcopy(registration['contract']),
                    station_compute_recovery_chain=deepcopy(
                        (registration.get('recovery_chain', [])+[registration['contract']])[-32:]),
                    discarded_source_stamp_ns=registration.get('discarded_source_ns'))
                if registration.get('feedback_gap') is not None:
                    payload['station_feedback_gap'] = deepcopy(registration['feedback_gap'])
                if registration.get('support_gap') is not None:
                    gap = registration['support_gap']
                    validate_station_support_gap(gap, contract=registration['contract'],
                        now_ns=rospy.Time.now().to_nsec())
                    # Empty/stale-input notices can still carry the original
                    # READY source.  They are identity waits, not a new support
                    # audit: attach this receipt only to its measured frame.
                    if payload.get('source_stamp_ns') == gap['source_ns']:
                        payload['station_support_gap'] = deepcopy(gap)
                        payload['detail'] = 'support_measurement_pending_revalidation'
            return
        reference = getattr(self, '_locked_rgb_hold_reference', None)
        if source is None or reference is None or reference.get('observation_arrival_retry') is None:
            return
        try:
            marker = self._require_observation_arrival_marker(reference)
            if (self.first_loss is not None or self.tracker.lost
                    or getattr(self, '_locked_source_pending', None) is not None
                    or getattr(self, '_metric_ambiguity_pending', None) is not None
                    or getattr(self, '_observation_measurement', None) is not None
                    or reference['pose_invalidated'] or marker.get('invalidated_reason')
                    or rospy.get_param_cached('/gui/joint_direct_mode', False) is True
                    or self._rgb_interest_epoch != marker['epoch_ns']):
                raise ValueError('station_compute_ready_not_qualified')
            binding = self._station_compute_binding(reference, payload, marker=marker)
            now = rospy.Time.now().to_nsec()
            contract = build_contract(source_ns=reference['source_stamp_ns'], maximum_frame_age_sec=min(self.max_age, .8),
                session_deadline_ns=marker['session_deadline_ns'],
                phase_deadline_ns=self._station_compute_phase.deadline.to_nsec(), binding=binding)
            validate_registration(contract, binding=binding, now_ns=now)
            payload['station_compute_mask'] = dict(
                sha256=hashlib.sha256(np.ascontiguousarray(reference['mask'] > 0, dtype=np.uint8).tobytes()).hexdigest(),
                shape=list(reference['image_shape']), frame_id=source[0].header.frame_id)
            if source[0].header.stamp.to_nsec() != reference['source_stamp_ns']:
                raise ValueError('station_compute_ready_source_changed')
            previous = getattr(self, '_station_compute_registration', None)
            if previous is not None and previous['contract']['source_ns'] == contract['source_ns']:
                with self.input_lock:
                    self._require_observation_arrival_marker(reference, marker)
                    if previous['contract'] != contract:
                        raise ValueError('station_compute_same_source_changed')
                    payload['station_compute_wait'] = deepcopy(contract)
                    if previous.get('recovery_contract') is not None:
                        payload['station_compute_recovery'] = deepcopy(previous['recovery_contract'])
                        payload['station_compute_recovery_chain'] = deepcopy(previous['recovery_chain'])
                    if previous.get('feedback_gap_recovery') is not None:
                        payload['station_feedback_gap_recovery'] = deepcopy(previous['feedback_gap_recovery'])
                    if previous.get('support_gap_recovery') is not None:
                        payload['station_support_gap_recovery'] = deepcopy(previous['support_gap_recovery'])
                    return  # Repeated status cannot renew the monotonic deadline.
            frame = getattr(self, '_station_compute_frame', None)
            chain = []
            if frame is not None:
                parent = frame['registration']
                chain = parent.get('recovery_chain', []) + [parent['contract']]
                chain = [row for row in chain if row['source_ns'] >= contract['source_ns']-5_000_000_000][-32:]
                if 'recovery_evidence' in frame:
                    payload['station_compute_recovery'] = deepcopy(parent['contract'])
                    payload['station_compute_recovery_chain'] = deepcopy(chain)
                    if parent.get('feedback_gap') is not None:
                        payload['station_feedback_gap_recovery'] = deepcopy(parent['feedback_gap'])
                    if parent.get('support_gap') is not None:
                        validate_station_support_gap(parent['support_gap'], contract=parent['contract'],
                            now_ns=rospy.Time.now().to_nsec(), source_ns=source[0].header.stamp.to_nsec())
                        payload['station_support_gap_recovery'] = deepcopy(parent['support_gap'])
            # Enrollment cannot resurrect a marker revoked during proof construction.
            with self.input_lock:
                self._require_observation_arrival_marker(reference, marker)
                self._station_requalification_finalizing_check(binding)
                if frame is not None and frame['registration'].get('support_gap') is not None:
                    parent = frame['registration']
                    validate_station_support_gap(parent['support_gap'], contract=parent['contract'],
                        now_ns=rospy.Time.now().to_nsec(), source_ns=source[0].header.stamp.to_nsec())
                    if time.monotonic() >= parent['support_gap_monotonic']:
                        raise ValueError('station_support_gap_deadline_expired')
                reference['station_compute_reference_binding'] = binding['reference_binding']
                self._station_compute_registration = dict(contract=contract, reference=reference, marker=marker,
                    ready_source=source,
                    recovery_contract=deepcopy(payload.get('station_compute_recovery')),
                    feedback_gap_recovery=deepcopy(payload.get('station_feedback_gap_recovery')),
                    support_gap_recovery=deepcopy(payload.get('station_support_gap_recovery')),
                    recovery_chain=deepcopy(chain),
                    monotonic_deadline=min(time.monotonic()+(contract['identity_deadline_ns']-now)*1e-9,
                                           marker['session_deadline_monotonic']))
                payload['station_compute_wait'] = deepcopy(contract)
                parents = getattr(self, '_station_requalification_parents', {})
                parents[contract['source_ns']] = deepcopy(contract)
                monos = getattr(self, '_station_requalification_parent_mono', {})
                monos[contract['source_ns']] = self._station_compute_registration['monotonic_deadline']
                while len(parents) > 32:
                    oldest = min(parents)
                    del parents[oldest]; monos.pop(oldest, None)
                self._station_requalification_parents = parents
                self._station_requalification_parent_mono = monos
                hold = getattr(self, '_station_requalification_hold', None)
                if hold is not None and hold.get('finalizing'):
                    self._station_requalification_hold = None
        except (ValueError, KeyError, TypeError, AttributeError):
            # An absent/late phase subscription cannot change ordinary READY.
            self._station_compute_registration = None

    def _station_compute_publish_support_gap(self, source, reference, reason):
        """Bridge an audited absent support measurement to an enrolled station.

        Called only after the original support-gap/positive-conflict audit. A
        reason string alone cannot enter: current frame, parent, frozen source,
        exact RGB-D and measured fixed joints must independently remain valid.
        """
        frame = getattr(self, '_station_compute_frame', None)
        if frame is None:
            return False  # Initial/unregistered stations keep their old policy.
        with self.input_lock:
            registration = frame['registration']
            if (frame['source'] is not source or registration is not self._station_compute_registration
                    or self._station_compute_scope(registration) is not reference):
                raise ValueError('station_support_gap_frame_changed')
            ns = source[0].header.stamp.to_nsec()
            if frame['source_ns'] != ns or source[1].header.stamp.to_nsec() != ns:
                raise ValueError('station_support_gap_source_changed')
            rgb = self.bridge.imgmsg_to_cv2(source[0], desired_encoding='bgr8')
            if rgb.dtype != np.uint8 or rgb.shape != reference['image_shape']+(3,):
                raise ValueError('support_gap_rgb_input_invalid')
            self._qualify_support_gap_source(source, reference)
            previous = registration.get('support_gap')
            receipt = build_station_support_gap(registration['contract'], source_ns=ns,
                reason=reason, first_source_ns=None if previous is None else previous['first_source_ns'])
            now = rospy.Time.now().to_nsec()
            validate_station_support_gap(receipt, contract=registration['contract'], now_ns=now)
            self._station_compute_scope(registration)
            registration['support_gap'] = receipt
            if previous is None:
                registration['support_gap_monotonic'] = min(
                    registration['monotonic_deadline'], time.monotonic()+(receipt['deadline_ns']-now)*1e-9)
            registration['discarded_source_ns'] = ns
            self._locked_source_status = dict(kind='support_measurement_gap',
                state='pending_current_support', reason=reason, source_stamp_ns=ns,
                reference_source_stamp_ns=reference['source_stamp_ns'], source_id=reference['source_id'],
                first_unqualified_source_stamp_ns=receipt['first_source_ns'],
                last_unqualified_source_stamp_ns=ns, source_qualified=True, identity_verified=False,
                support_measurement_available=False, support_audit=None,
                metric_geometry_authority=False, motion_authorized=False, reference_refreshed=False,
                maximum_hold_seconds=(receipt['deadline_ns']-registration['contract']['source_ns'])*1e-9,
                geometry_maximum_age_seconds=min(self.max_age, .8),
                geometry_deadline_ns=registration['contract']['geometry_deadline_ns'],
                identity_deadline_ns=receipt['deadline_ns'],
                elapsed_since_measured_sec=(now-reference['source_stamp_ns'])*1e-9)
            self._arrival_frame_completion = None
            # Existing typed station envelope; status adds the exact support
            # receipt and truthful detail, not a new execution authorization.
            self.unavailable('processing_deadline_pending', source[0].header.stamp, source, lose=False)
            return True

    def _station_compute_validate_feedback_gap(self, source, reference, evidence):
        frame = getattr(self, '_station_compute_frame', None)
        if frame is None or frame['source'] is not source:
            raise ValueError('station_feedback_gap_requires_registered_frame')
        registration = frame['registration']
        if self._station_compute_scope(registration) is not reference:
            raise ValueError('station_feedback_gap_reference_changed')
        ns = source[0].header.stamp.to_nsec()
        if (frame['source_ns'] != ns or source[1].header.stamp.to_nsec() != ns
                or not ns <= frame['entry_ns'] < ns+round(min(self.max_age, .8)*1e9)
                or digest(self._rgb_interest_source_context(source[0].header.stamp)) != reference['source_id']):
            raise ValueError('station_feedback_gap_source_changed')
        # Source-context lookup can wait for TF. Evaluate clocks afterwards.
        now = rospy.Time.now().to_nsec()
        self._station_compute_reference_lifetime(reference, registration['marker'], ns, now)
        validate_span_gap_evidence(evidence, ns, reference['source_stamp_ns'],
            registration['marker']['epoch_ns'], registration['marker']['arrival_q'], now)
        return registration

    def _station_compute_publish_feedback_gap(self, source, error):
        # Keep original identity clocks. Never renew an old metric frame.
        with self.input_lock:
            registration = error.registration
            if self._station_compute_validate_feedback_gap(source,
                    registration['reference'], error.evidence) is not registration:
                raise ValueError('station_feedback_gap_registration_changed')
            ns = source[0].header.stamp.to_nsec()
            previous = registration.get('feedback_gap', {})
            discarded = previous.get('discarded_source_stamps_ns', [])
            if (discarded and ns <= discarded[-1]) or len(discarded) >= 32:
                raise ValueError('station_feedback_gap_order_or_capacity')
            registration['feedback_gap'] = dict(discarded_source_stamp_ns=ns,
                discarded_source_stamps_ns=discarded+[ns], feedback_gap_evidence=deepcopy(error.evidence),
                discarded_source_evidence=deepcopy(previous.get('discarded_source_evidence', []))+[
                    dict(source_stamp_ns=ns, feedback_gap_evidence=deepcopy(error.evidence))],
                source_qualified=False, identity_verified=False, metric_geometry_authority=False,
                motion_authorized=False, reference_refreshed=False)
            registration['discarded_source_ns'] = ns
            self._arrival_frame_completion = None
            self.unavailable(str(error), source[0].header.stamp, source, lose=False)

    def _station_compute_timeout(self, source, error):
        frame = getattr(self, '_station_compute_frame', None)
        registration = (frame['registration'] if frame is not None else
                        getattr(self, '_station_compute_registration', None))
        if registration is None or not (isinstance(error, StationComputeDeadline)
                or str(error) in ('source_frame_stale_or_future', 'source_exceeds_existing_frame_limit')):
            return False
        try:
            reference = self._station_compute_scope(registration)
            ns, now = source[0].header.stamp.to_nsec(), rospy.Time.now().to_nsec()
            if not registration['contract']['source_ns'] < ns < now:
                return False
            if frame is not None and (frame['source'] is not source or frame['source_ns'] != ns
                    or not ns <= frame['entry_ns'] < ns+round(min(self.max_age, .8)*1e9)):
                return False
            if now-ns < round(min(self.max_age, .8)*1e9):
                return False
            if digest(self._rgb_interest_source_context(source[0].header.stamp)) != reference['source_id']:
                return False
            with self.input_lock:
                self._station_compute_scope(registration)
                registration['discarded_source_ns'] = ns
                self._arrival_frame_completion = None
                self.unavailable('processing_deadline_pending', source[0].header.stamp, source, lose=False)
            return True
        except (ValueError, KeyError, TypeError, AttributeError, tf2_ros.LookupException,
                tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return False

    def _station_compute_idle_wait(self):
        registration = getattr(self, '_station_compute_registration', None)
        self._station_compute_idle_feedback_retry = None
        # Diagnostic snapshot only: the caller must still decide stream loss
        # against current ingress. Preserve the actual scope failure instead
        # of making every rejected station appear to be missing RGB-D.
        contract = registration.get('contract', {}) if isinstance(registration, dict) else {}
        if not isinstance(contract, dict):
            contract = {}
        audit = dict(schema_version=1, diagnostic_only=True,
            registration_present=registration is not None,
            audit_started_monotonic_sec=time.monotonic(),
            source_ns=contract.get('source_ns'),
            identity_deadline_ns=contract.get('identity_deadline_ns'),
            geometry_deadline_ns=contract.get('geometry_deadline_ns'),
            session_deadline_ns=contract.get('session_deadline_ns'),
            phase_deadline_ns=contract.get('phase_deadline_ns'),
            identity_deadline_monotonic_sec=(registration.get('monotonic_deadline')
                                           if isinstance(registration, dict) else None),
            outcome='no_registration', scope_failure_reason=None)
        self._station_compute_idle_audit = audit
        if registration is None:
            return False
        try:
            self._station_compute_scope(registration)
            if rospy.Time.now().to_nsec() < registration['contract']['geometry_deadline_ns']:
                audit['outcome'] = 'geometry_still_fresh'
                return False
            with self.input_lock:
                self._station_compute_scope(registration)
                self.status('processing_deadline_pending', 'processing_deadline_pending', registration['ready_source'])
            audit['outcome'] = 'identity_wait'
            return True
        except (ValueError, KeyError, TypeError, AttributeError) as exc:
            audit.update(outcome='scope_rejected', scope_failure_reason=str(exc),
                         scope_failure_type=type(exc).__name__)
            failed = getattr(exc, 'station_idle_feedback_failure', None)
            if failed is not None and failed['registration'] is registration:
                self._station_compute_idle_feedback_retry = failed
                audit.update(failed_feedback_stamp_ns=failed['feedback_stamp_ns'],
                             feedback_checked_ros_ns=failed['checked_ns'])
            return False

    def _station_compute_idle_feedback_updated(self):
        """Defer one empty-input decision after a real SDK refresh, under input_lock.

        No scope requalification, publication or deadline update happens here.
        The next timer tick must run the complete original station scope again.
        """
        failed = getattr(self, '_station_compute_idle_feedback_retry', None)
        self._station_compute_idle_feedback_retry = None  # One decision only.
        if failed is None:
            return False
        try:
            registration, reference, marker = (failed[k] for k in ('registration', 'reference', 'marker'))
            contract = registration['contract']; binding = contract['binding']
            now, mono = rospy.Time.now().to_nsec(), time.monotonic()
            phase = self._station_compute_phase
            state = self._station_compute_task_state
            if (self._station_compute_registration is not registration
                    or self._locked_rgb_hold_reference is not reference
                    or reference.get('observation_arrival_retry') is not marker
                    or marker.get('invalidated_reason') or reference['pose_invalidated']
                    or self.first_loss is not None or self.tracker.lost
                    or self.last_good_source_stamp_ns != contract['source_ns']
                    or getattr(self, '_observation_measurement', None) is not None
                    or getattr(self, '_locked_source_pending', None) is not None
                    or getattr(self, '_metric_ambiguity_pending', None) is not None
                    or (self.mode, self.generation, self.selection_ns) != ('unknown',
                        binding['selection']['generation'], binding['selection']['stamp_ns'])
                    or self._rgb_interest_epoch != binding['control_epoch_ns']
                    or self._rgb_interest_epoch != marker['epoch_ns']
                    or not failed['checked_ns'] <= now < min(contract['identity_deadline_ns'],
                        contract['phase_deadline_ns'], contract['session_deadline_ns'], marker['session_deadline_ns'])
                    or mono >= min(registration['monotonic_deadline'], marker['session_deadline_monotonic'])
                    or state.state != 'PLAN_PREGRASP' or not state.active
                    or not phase.active or phase.planning_stage != 'contact_replan'
                    or phase.phase_id != binding['phase_id']
                    or phase.header.stamp.to_nsec() != binding['phase_started_ns']
                    or phase.reference_source_stamp.to_nsec() != binding['phase_reference_source_ns']
                    or phase.reference_target_track_id != binding['target_track_id']
                    or phase.deadline.to_nsec() != contract['phase_deadline_ns']):
                return False
            support_gap = registration.get('support_gap')
            if support_gap is not None and (now >= support_gap['deadline_ns']
                    or mono >= registration['support_gap_monotonic']):
                return False
            feedback = tuple(self._rgb_interest_feedback)
            newest = max(ns for ns, _ in feedback)
            if newest <= failed['feedback_stamp_ns']:
                return False
            require_fresh_source(newest, now, min(self.max_age, .8))
            positions = [np.asarray(q) for ns, q in feedback if ns >= marker['qualified_source_ns']]
            if not positions or any(q.shape != (6,) or not np.isfinite(q).all()
                    or np.max(np.abs(q-marker['arrival_q'])) > .004 for q in positions):
                return False
            self._station_compute_idle_audit['feedback_updated_before_loss_commit'] = True
            return True
        except (ValueError, KeyError, TypeError, AttributeError):
            return False
