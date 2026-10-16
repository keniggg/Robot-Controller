"""Source-joined, identity-only computation wait at a qualified observation.

The old view remains historical session evidence. Real missing/invalid input
revokes geometry, caches and computation. Pure clock handoff may retain only
audited, unpublished computation; public geometry stays unavailable until
a fresh measured three-topic join and the original result gates succeed.
"""
from copy import deepcopy
import hashlib
import json
from numbers import Integral
import time

import numpy as np
import rospy
from sensor_msgs.msg import JointState

from .station_compute_reacquisition import (
    validate_contract, validate_registration, validate_recovery_parent, validate_station_support_gap)
from .unknown_pending_identity import reference_binding
from .observe_return_measurement import (measurement_digest, STATION_REQUALIFICATION,
    station_requalification_key, station_requalification_witness, validate_station_requalification)
from .rgb_observation_source import bracket_feedback
from .accepted_feedback_ingress import AcceptedFeedbackIngress, FeedbackIngressError
from .feedback_bracket_gap import validate_station_feedback_gap
from .station_contact_admission import KEY as CONTACT_RECEIPT, source_witness, build_receipt, validate_receipt
from .station_supplemental_admission import (
    KEY as SUPPLEMENT_RECEIPT, build_receipt as build_supplement_receipt,
    validate_receipt as validate_supplement_receipt)


CODE = 'TARGET_PROCESSING_PENDING'


class _StationRequalificationPending(ValueError):
    pass


class StationComputeWaitRemoteMixin:
    def _reset_station_compute_wait(self):
        self._contact_station_prepared = None
        self._station_contact_history = None
        self._contact_station_retired_measurement = None
        self._station_compute_reference = None
        self._station_compute_pending = False
        self._station_compute_soft_wait = False
        self._station_compute_work_published = False
        self._station_compute_publication_started = False
        self._station_compute_ready_history = {}
        self._station_compute_work = {}
        self._station_supplement_proposal = None
        self._station_compute_retired = False
        self._station_compute_cutoff_ns = 0
        self._station_compute_soft_cutoff_ns = 0
        self._station_compute_objects = {}
        self._station_compute_masks = {}
        self._station_compute_status = {}
        self._station_compute_phase = None
        self._station_compute_task_state = ''
        self._station_compute_feedback = AcceptedFeedbackIngress(maxlen=128)
        self._station_compute_feedback_diagnostic = None
        self._station_feedback_gap_tombstones = {}
        self._station_feedback_gap_history = {}
        self._station_requalification_hold = None
        self._station_requalification_notice = False
        self._station_requalification_parents = {}
        self._station_requalification_used = set()

    def _station_gap_late_status(self, status):
        ns = status.get('source_stamp_ns')
        history = self._station_feedback_gap_history.get(ns)
        if history is not None and status.get('state') == 'ready':
            if (status.get('station_compute_wait') != history['contract']
                    or status.get('station_compute_mask') != history['mask']):
                self._station_compute_retire('historical station READY changed')
                return True
        tombstone = self._station_feedback_gap_tombstones.get(ns)
        reference = self._station_compute_reference or self._station_contact_history
        if tombstone is None or reference is None:
            return False
        if status.get('target_lost') is True:
            return False  # A real loss is never a canceled pending message.
        try:
            if (status.get('state') != 'processing_deadline_pending'
                    or status.get('detail') != 'station_accepted_feedback_bracket_pending_revalidation'
                    or status.get('target_lost') is not False or status.get('first_loss') is not None
                    or status.get('station_compute_wait') != tombstone['contract']
                    or reference_binding(status, self._mode_selection)
                        != tombstone['contract']['binding']['reference_binding']
                    or reference['contract']['binding'] != tombstone['contract']['binding']):
                raise ValueError('late station gap receipt changed')
            stamps = validate_station_feedback_gap(status.get('station_feedback_gap'),
                contract=tombstone['contract'], now_ns=int(self._ros_source_clock_ns()),
                recovery_source_ns=tombstone['recovery_source_ns'])
            if ns not in stamps or any(self._station_feedback_gap_tombstones.get(v, {}).get('contract')
                                      != tombstone['contract'] for v in stamps):
                raise ValueError('late station gap source unproven')
            return True
        except (ValueError, KeyError, TypeError, AttributeError):
            self._station_compute_retire('late station gap receipt changed')
            return True

    def _setup_station_compute_io(self):
        self._station_compute_feedback_sub = rospy.Subscriber(
            '/alicia_d/accepted_joint_states', JointState,
            self._station_compute_feedback_ingress_cb, queue_size=128)
        self._station_compute_feedback_timer = rospy.Timer(
            rospy.Duration(.05), lambda event: self._station_compute_accepted_cb(None))

    def _station_compute_scope(self):
        request = getattr(self, '_measurement_remote_request', None) or {}
        phase = getattr(self, '_station_compute_phase', None)
        internal_phase = getattr(self, '_observe_return_phase', '')
        planning_station = (internal_phase == 'observing_contact'
            or (internal_phase == 'observation_prefix'
                and getattr(self, '_observe_return_pending_supplement', None) is not None
                and (getattr(self, '_observe_return_session', None) or {}).get('phase') == 'observing'))
        paused = getattr(self, '_observe_return_paused', True)
        reached_retry = getattr(self, '_observe_return_reached_retry', None)
        # A qualified view can await its original RGB-D delivery while paused.
        # Permit only the existing three-topic station identity registration;
        # snapshot policy still rejects observing and cannot authorize work.
        reached_identity_wait = (internal_phase == 'observing' and paused
            and reached_retry is not None
            and self._observe_return_reached_retry_valid(reached_retry))
        return bool(getattr(self, '_mode_ready', False)
            and (self._mode_selection or {}).get('mode') == 'unknown'
            and (self._mode_selection or {}).get('strategy') == 'observe_return'
            and getattr(self, 'streaming_enabled', False)
            and ((planning_station and not paused) or reached_identity_wait)
            and not getattr(self, '_observe_return_diagnostic_observation_only', True)
            and self._station_compute_task_state == 'PLAN_PREGRASP'
            and request.get('state') == 'qualified'
            and getattr(self, '_contact_station_retired_measurement', None) != measurement_digest(request)
            and request.get('motion_kind') == 'observation'
            and phase is not None and phase.active
            and phase.planning_stage == 'contact_replan'
            and self._unknown_identity_pending is None
            and self._unknown_identity_retired_selection != self._mode_selection)

    def _station_compute_context(self, contract, *, check_joints=True):
        if not self._station_compute_scope() or not self._selection_is_current():
            raise ValueError('station_compute_wait_scope_changed')
        if rospy.get_param('/gui/joint_direct_mode', False) is not False:
            raise ValueError('station_compute_wait_manual_takeover')
        binding = contract['binding']
        request = self._measurement_remote_request
        session = self._observe_return_session or {}
        phase = self._station_compute_phase
        identity = self._current_stream_target_identity()
        if (binding['selection'] != self._mode_selection
                or any(binding[key] != request.get(key) for key in (
                    'lease_id', 'session_id', 'view_plan_id', 'view_index',
                    'target_epoch', 'target_track_id', 'control_epoch_ns'))
                or any(binding[key] != session.get(key) for key in (
                    'session_id', 'view_plan_id', 'view_index', 'target_epoch', 'target_track_id'))
                or request['deadline_ns'] != contract['session_deadline_ns']
                or session.get('deadline_ns') != contract['session_deadline_ns']
                or identity.epoch != binding['target_epoch']
                or identity.track_id != binding['target_track_id']
                or self._measurement_remote_epoch_ns != binding['control_epoch_ns']
                or phase.phase_id != binding['phase_id']
                or phase.header.stamp.to_nsec() != binding['phase_started_ns']
                or phase.reference_source_stamp.to_nsec() != binding['phase_reference_source_ns']
                or phase.deadline.to_nsec() != contract['phase_deadline_ns']
                or phase.reference_target_track_id != binding['target_track_id']
                or not phase.reference_center_valid
                or binding['phase_reference_source_ns'] < request['source_stamps_ns'][-1]
                or (self._measurement_remote_reference or {}).get('binding') != binding['reference_binding']):
            raise ValueError('station_compute_wait_owner_changed')
        if self._observe_return_phase == 'observation_prefix':
            supplement = self._observe_return_pending_supplement or {}
            if (any(supplement.get(k) != binding[k] for k in (
                    'session_id', 'target_epoch', 'target_track_id'))
                    or supplement.get('previous_view_index') != binding['view_index']
                    or supplement.get('view_index') != binding['view_index'] + 1
                    or supplement.get('deadline_ns') != contract['session_deadline_ns']
                    or supplement.get('contact_or_motion_authorized') is not False):
                raise ValueError('station_compute_wait_supplement_binding_changed')
        if check_joints:
            self._station_compute_check_joints(binding)

    def _station_compute_check_joints(self, binding, source_ns=None, *, check_freshness=True):
        epoch = binding['control_epoch_ns']
        now = int(self._ros_source_clock_ns())
        q_station = np.asarray(binding['station_q_rad'])
        since = self._measurement_remote_request['source_stamps_ns'][-1]
        owner = ('station', json.dumps(binding, sort_keys=True, separators=(',', ':')))
        rows, diagnostic = self._station_feedback_snapshot(owner, epoch, since, now)
        now = diagnostic['validation_now_ns']
        measured = [q for ns, q in rows if ns >= since]
        reason = (('empty' if not rows else
                  'future' if now < diagnostic['latest_source_stamp_ns'] else
                  'stale' if now-diagnostic['latest_source_stamp_ns'] >= 800_000_000 else
                  'no_post_station_samples' if not measured else None)
                  if check_freshness else None)
        if reason is None and any(np.max(np.abs(q-q_station)) > .004 for q in measured):
            reason = 'moved'
        diagnostic['maximum_station_delta_rad'] = max(
            (float(np.max(np.abs(q-q_station))) for q in measured), default=None)
        if reason == 'moved' and self._station_requalification_arm(rows, now):
            raise _StationRequalificationPending(STATION_REQUALIFICATION)
        if reason:
            self._station_feedback_failure('station_compute_wait_feedback_'+reason, diagnostic,
                                           owner=owner if reason == 'moved' else None)
        if source_ns is not None:
            q, before, after = bracket_feedback(rows, source_ns, epoch)
            if any(np.max(np.abs(np.asarray(v)-q_station)) > .004 for v in (q, before[1], after[1])):
                raise ValueError('station_compute_wait_source_station_changed')

    def _station_requalification_arm(self, rows, now):
        """Cancel the old metric authority from our own registered SDK evidence."""
        reference = self._station_compute_reference
        if (reference is None or self._station_compute_retired
                or self._observe_return_phase != 'observing_contact'):
            return False
        parent = reference['contract']
        key = station_requalification_key(parent)
        if key in self._station_requalification_used:
            return False
        try:
            self._station_compute_context(parent, check_joints=False)
            validate_contract(parent, binding=parent['binding'], now_ns=now)
            if time.monotonic() >= reference['identity_monotonic']:
                return False
            registered = self._station_requalification_parents[parent['source_ns']]
            if registered['contract'] != parent:
                return False
            moved = [(ns, q) for ns, q in rows if ns > parent['source_ns']
                and np.max(np.abs(q-np.asarray(parent['binding']['station_q_rad']))) > .004]
            if not moved:
                return False
            ns, q = moved[0]
            witness = station_requalification_witness(parent, ns, np.asarray(q).tolist(), now)
            self._station_feedback_snapshot(
                ('station_requalification', json.dumps(parent['binding'], sort_keys=True)),
                parent['binding']['control_epoch_ns'], parent['source_ns'], now)
            self._station_compute_quarantine(ns)  # Permanently cancel all old tickets.
            self._station_requalification_hold = dict(
                witness=witness, registered=deepcopy(registered),
                deadline_ns=witness['deadline_ns'],
                deadline_monotonic=registered['identity_monotonic'],
                last_now_ns=now, lease_id=None)
            self._station_requalification_used.add(key)
            self._station_requalification_notice = False
            self._station_compute_reference = None
            self._observe_return_paused = True
            return True
        except (ValueError, KeyError, TypeError, AttributeError):
            return False

    def _station_requalification_validate(self, request=None, *, require_feedback=False, check_freshness=True):
        """Keep the old owner/deadline; a task payload never supplies authority."""
        witness = request.get(STATION_REQUALIFICATION) if request is not None else None
        hold = self._station_requalification_hold
        if witness is None:
            if hold is None:
                raise ValueError('STATION_REQUALIFICATION_NOT_PENDING')
            witness = hold['witness']
        parent = witness['parent_contract']
        registered = self._station_requalification_parents.get(parent['source_ns'])
        if registered is None or registered['contract'] != parent:
            raise ValueError('STATION_REQUALIFICATION_PARENT_UNREGISTERED')
        binding = parent['binding']
        now = int(self._ros_source_clock_ns())
        current = self._station_compute_reference
        session = self._observe_return_session or {}
        phase = self._station_compute_phase
        identity = self._current_stream_target_identity()
        if (self._station_compute_retired or not self._selection_is_current()
                or rospy.get_param('/gui/joint_direct_mode', False) is not False
                or self._unknown_identity_retired_selection == self._mode_selection
                or self._unknown_identity_pending is not None
                or self._station_compute_task_state != 'PLAN_PREGRASP'
                or self._observe_return_phase != 'observing_contact'
                or self._observe_return_diagnostic_observation_only
                or not self.streaming_enabled or session.get('phase') != 'observing'
                or binding['selection'] != self._mode_selection
                or identity.epoch != binding['target_epoch']
                or identity.track_id != binding['target_track_id']
                or self._measurement_remote_epoch_ns != binding['control_epoch_ns']
                or any(session.get(k) != binding[k] for k in (
                    'session_id', 'view_plan_id', 'view_index', 'target_epoch', 'target_track_id'))
                or session.get('deadline_ns') != parent['session_deadline_ns']
                or phase is None or not phase.active or phase.planning_stage != 'contact_replan'
                or phase.deadline.to_nsec() != parent['phase_deadline_ns']
                or phase.reference_target_track_id != binding['target_track_id']
                or not phase.reference_center_valid):
            raise ValueError('STATION_REQUALIFICATION_OWNER_CHANGED')
        if hold is None:
            if (station_requalification_key(parent) in self._station_requalification_used
                    or current is None or current['contract']['binding'] != binding):
                raise ValueError('STATION_REQUALIFICATION_PARENT_RETIRED')
        elif (hold['witness']['parent_contract']['binding'] != binding
                or (hold['lease_id'] is not None and request is not None
                    and hold['lease_id'] != request['lease_id'])):
            raise ValueError('STATION_REQUALIFICATION_LEASE_CHANGED')
        active = self._measurement_remote_request or {}
        if phase.phase_id == binding['phase_id']:
            if (phase.header.stamp.to_nsec() != binding['phase_started_ns']
                    or phase.reference_source_stamp.to_nsec() != binding['phase_reference_source_ns']):
                raise ValueError('STATION_REQUALIFICATION_PHASE_CHANGED')
        elif (phase.phase_id <= binding['phase_id'] or hold is None
                or active.get('state') != 'qualified'
                or active.get(STATION_REQUALIFICATION) != hold['witness']
                or phase.reference_source_stamp.to_nsec() < active['source_stamps_ns'][-1]):
            raise ValueError('STATION_REQUALIFICATION_PHASE_CHANGED')
        deadline = min(witness['deadline_ns'], parent['identity_deadline_ns'],
                       hold['deadline_ns'] if hold is not None else witness['deadline_ns'])
        mono_deadline = min(registered['identity_monotonic'],
                           hold['deadline_monotonic'] if hold is not None else float('inf'))
        if (not witness['feedback_stamp_ns'] <= now < deadline
                or time.monotonic() >= mono_deadline
                or (hold is not None and now < hold['last_now_ns'])):
            raise ValueError('STATION_REQUALIFICATION_EXPIRED')
        if hold is not None:
            hold['last_now_ns'] = now
        rows = []
        if require_feedback or hold is not None:
            rows, diagnostic = self._station_feedback_snapshot(
                ('station_requalification', json.dumps(binding, sort_keys=True)),
                binding['control_epoch_ns'], parent['source_ns'], now)
            latest = diagnostic['latest_source_stamp_ns']
            if check_freshness and (not rows or not latest <= int(self._ros_source_clock_ns()) < latest+800_000_000):
                raise ValueError('STATION_REQUALIFICATION_FEEDBACK_STALE')
        if require_feedback:
            proven = list(rows)
            if hold is not None:
                owned = hold['witness']
                proven.append((owned['feedback_stamp_ns'], owned['feedback_q_rad']))
            validate_station_requalification(witness, parent=registered['contract'],
                feedback=proven, now_ns=int(self._ros_source_clock_ns()))
            if hold is None:
                if not self._station_requalification_arm(rows, now):
                    raise ValueError('STATION_REQUALIFICATION_LOCAL_DRIFT_UNAVAILABLE')
                hold = self._station_requalification_hold
            if request is not None:
                # Historical locally registered parents may differ in source,
                # but can only shorten this already running recovery deadline.
                hold['deadline_ns'] = min(hold['deadline_ns'], witness['deadline_ns'])
                hold['deadline_monotonic'] = min(hold['deadline_monotonic'], mono_deadline)
                hold['witness'] = deepcopy(witness)
                hold['registered'] = deepcopy(registered)
        return registered

    def _station_requalification_admit(self, request):
        registered = self._station_requalification_validate(request, require_feedback=True)
        hold = self._station_requalification_hold
        hold['lease_id'] = request['lease_id']
        self._observe_return_paused = True
        return deepcopy(registered['measurement_reference'])

    def _station_feedback_snapshot(self, owner, epoch, since, now):
        try:
            rows, diagnostic = self._station_compute_feedback.snapshot(
                owner=owner, epoch_ns=epoch, since_ns=since)
        except FeedbackIngressError as exc:
            exc.diagnostic['validation_now_ns'] = int(self._ros_source_clock_ns())
            self._station_feedback_failure('station_compute_wait_'+str(exc), exc.diagnostic)
        # Ingress may advance while planning locks are held. Use the clock
        # after the snapshot, never an older pre-ingress evaluation instant.
        now = int(self._ros_source_clock_ns())
        diagnostic.update(validation_now_ns=now,
            latest_source_age_ns=now-diagnostic['latest_source_stamp_ns'])
        self._station_compute_feedback_diagnostic = diagnostic
        return rows, diagnostic

    def _station_feedback_failure(self, reason, diagnostic, owner=None):
        if owner is not None:
            self._station_compute_feedback.mark_fault(owner, 'feedback_moved')
        self._station_compute_feedback_diagnostic = dict(diagnostic, reason=reason)
        rospy.logwarn('Accepted feedback qualification failed: %s', json.dumps(
            self._station_compute_feedback_diagnostic, sort_keys=True, separators=(',', ':')))
        raise ValueError(reason)

    def _station_compute_feedback_ingress_cb(self, msg):
        # TCPROS dispatch is serial per connection. This entry point must
        # return without acquiring ANY planning/geometry lock, including a
        # post-append validation or notification under those locks.
        buffer = self._station_compute_feedback
        buffer.ingest(msg, int(self._ros_source_clock_ns()), time.monotonic())

    def _station_compute_accepted_cb(self, msg=None):
        # Deferred validator; optional message supports synchronous internal
        # callers. The ROS subscriber itself uses only the fast ingress above.
        if msg is not None:
            self._station_compute_feedback_ingress_cb(msg)
        if not hasattr(self, '_stream_condition'):
            return
        with self._stream_condition, self._geometry_state_guard():
            try:
                if self._station_requalification_hold is not None:
                    self._station_requalification_validate(check_freshness=False)
                elif self._station_compute_scope() and self._station_compute_reference is not None:
                    # Preserve the original callback's malformed/movement
                    # checks. Source freshness belongs to the existing
                    # synchronous planning/admission gates, not a partially
                    # delivered batch of accepted feedback.
                    self._station_compute_check_joints(
                        self._station_compute_reference['contract']['binding'], check_freshness=False)
            except _StationRequalificationPending:
                pass
            except (ValueError, KeyError, TypeError) as exc:
                if self._station_compute_reference is not None or self._station_requalification_hold is not None:
                    self._station_compute_retire(str(exc))

    def _measurement_remote_context(self, request):
        if request.get(STATION_REQUALIFICATION) is not None:
            self._station_requalification_validate(request)
        elif request.get('state') == 'executing' and self._station_compute_pending:
            raise ValueError('station_compute_wait_movement_while_pending')
        return super()._measurement_remote_context(request)

    def _measurement_remote_ack(self, request, accepted=True, reason='OK', qualified=False):
        # Original movement admission has completed all its checks. Disarm
        # before publishing its ACK, not on a later state/feedback callback.
        if accepted and request.get('state') == 'executing':
            self._station_compute_reference = None
        return super()._measurement_remote_ack(request, accepted, reason, qualified)

    def _station_compute_retire(self, reason):
        self._station_supplement_proposal = None
        self._station_contact_history = None
        self._station_compute_reference = None
        self._station_compute_work.clear()
        self._station_compute_ready_history.clear()
        self._station_compute_pending = False
        self._station_compute_retired = True
        self._station_requalification_hold = None
        self._station_requalification_notice = False
        self._unknown_identity_retire('station computation wait: ' + str(reason))

    def _station_compute_quarantine(self, source_ns=None, *, clock_only=False):
        if not clock_only:
            self._station_supplement_proposal = None
        reference = self._station_compute_reference
        if reference is None:
            if not clock_only and any(p.get('contact_proposal') is not None
                                      for p in self._station_compute_work.values()):
                self._station_compute_retire('published contact source invalidated')
            return
        supplement = self._station_supplement_proposal
        retained_supplement = bool(supplement is not None
            and supplement['receipt']['current']['contract']['binding'] == reference['contract']['binding'])
        soft = bool(clock_only and not self._station_compute_work_published
                    and not self._station_compute_publication_started
                    and int(getattr(self, 'near_field_planning_snapshot_frames', 0)) == 3
                    and self._configured_near_field_strategy() == 'single_snapshot_direct')
        if soft:
            self._station_compute_soft_cutoff_ns = max(self._station_compute_soft_cutoff_ns,
                reference['contract']['source_ns'], int(self.last_submitted_stamp_ns))
        if not soft:
            self._station_supplement_proposal = None
            self._station_compute_cutoff_ns = max(self._station_compute_cutoff_ns,
                reference['contract']['source_ns'], int(self.last_submitted_stamp_ns),
                int(source_ns or 0))
        if self._station_compute_pending and (self._station_compute_soft_wait == soft or not self._station_compute_soft_wait):
            return
        self._station_compute_pending = True
        self._station_compute_soft_wait = soft
        if not soft:
            # Actual missing/invalid measurement permanently cancels old work.
            # Timer-only handoff retains private, unpublished computation;
            # it never retains a valid public geometry or executable plan.
            self._station_compute_cutoff_ns = max(self._station_compute_cutoff_ns,
                reference['contract']['source_ns'], int(self.last_submitted_stamp_ns),
                int(source_ns or 0))
            self._station_compute_ready_history.clear()
            self._station_compute_work.clear()
            request_id = self._pending_request_id
            self.inference_coordinator.reset_target_epoch(self.target_instance_epoch)
            self._pending_request_id = None
            if request_id is not None:
                claimed = self._claim_terminal_request_locked(request_id, CODE)
                self._emit_pending_drop_metrics(request_id, CODE, terminal_claimed=claimed)
            self._geometry_invalidation_generation = int(getattr(self, '_geometry_invalidation_generation', 0)) + 1
            self._last_geometry_invalidation_code = CODE
            self._clear_geometry_cache()
            self._near_field_request_plan_cache = {}
            self._candidate_plan_metrics = {}
            self._direct_near_field_submission_generation = None
        original = self._observe_return_original
        geometry = original.geometry_estimate_to_message(
            self._invalid_geometry_estimate(CODE, 'waiting for a fresh complete station measurement', ''),
            stamp=self._safe_time(), target_identity=self._current_stream_target_identity())
        # ObjectGeometry has no separate source-contract field. Carry this
        # non-executable invalidation's exact registered owner in its reason;
        # a wall-clock header alone cannot distinguish late old notifications.
        source_status = self._station_compute_status.get(reference['contract']['source_ns'], {})
        chain = deepcopy(source_status.get('station_compute_recovery_chain'))
        if isinstance(chain, list):
            if not chain or chain[-1] != reference['contract']:
                chain.append(deepcopy(reference['contract']))
            chain = chain[-32:]
        else:
            chain = None
        geometry.failure_reason = CODE + ': ' + json.dumps(dict(
            station_compute_wait=reference['contract'],
            station_compute_recovery_chain=chain, clock_only=bool(clock_only)),
            sort_keys=True, separators=(',', ':'))
        self.latest_object_geometry = deepcopy(geometry)
        self.previous_object_axes_base = None
        self._latest_preview_gate_audit_report = None
        self._publish_invalidation_safely(self.geometry_pub, geometry, 'object geometry')
        preview = original.Grasp6DPlan()
        preview.header = deepcopy(geometry.header)
        preview.valid = False
        preview.candidate_source = 'near_field_pending'
        preview.diagnostic = CODE + ': current geometry expired; identity only'
        if soft and (retained_supplement or any(p.get('contact_proposal') is not None
                        for p in self._station_compute_work.values())):
            preview.diagnostic = geometry.failure_reason
            preview.target_track_id = geometry.target_track_id
        legacy = original.PoseArray(); legacy.header = deepcopy(preview.header)
        # This publishes only the non-executable preview. The old FAR_FIELD
        # message is retained solely as a session/history link, not reissued.
        # Invalidation already holds stream->geometry. Do not acquire the
        # worker's mode-work lock here (the worker takes work->stream).
        original.RemoteGrasp6DNode._publish_preview_plan(self, preview, legacy)
        self._stream_condition.notify_all()

    def _station_supplement_preview_work(self, plan, ticket):
        if ticket is None or not self._station_compute_work_certificate(ticket):
            return False
        proof = self._station_compute_work[int(ticket.request_id)]
        try:
            from alicia_flexible_grasp.grasp.rich_plan_integrity import (
                compute_plan_id, validate_plan_header_binding)
            validate_plan_header_binding(plan)
            return bool(plan.valid is True and plan.diagnostic == 'FAR_FIELD_OBSERVATION_PLAN'
                and plan.model_choice == 'unknown_tabletop' and compute_plan_id(plan) == plan.plan_id
                and plan.target_track_id == proof['binding']['target_track_id']
                and plan.header.stamp.to_nsec() == ticket.payload[0].stamp_ns
                and proof['snapshot_policy']['purpose'] == 'supplementary_observation'
                and proof['snapshot_policy']['phase'] == 'observation_prefix'
                and len(proof['source_witnesses']) == proof['snapshot_policy']['frame_count'])
        except (ValueError, KeyError, TypeError, AttributeError):
            return False

    def _station_supplement_computation_receipt(self, plan, ticket):
        while True:
            self._station_compute_wait_for_current(ticket)
            with self._stream_condition, self._geometry_state_guard():
                self._station_compute_tick_locked()
                self._require_stream_ticket_current_locked(ticket)
                if self._station_compute_pending:
                    continue
                if not self._station_supplement_preview_work(plan, ticket):
                    raise self._observe_return_original.StreamResultCancelled(CODE)
                proof = self._station_compute_work[int(ticket.request_id)]
                reference = self._station_compute_reference
                self._station_compute_context(reference['contract'])
                now = int(self._ros_source_clock_ns()); mono = time.monotonic()
                if now < reference['last_now_ns'] or mono >= reference['identity_monotonic']:
                    raise ValueError('station_compute_wait_clock_or_deadline')
                self._station_compute_support_gap_live(reference, now)
                state = validate_contract(reference['contract'], binding=proof['binding'], now_ns=now)
                reference['last_now_ns'] = now
                if state == 'PENDING' or mono >= reference['geometry_monotonic']:
                    self._station_compute_quarantine(clock_only=True)
                    continue  # Release both input locks before waiting for READY.
                return build_supplement_receipt(plan,
                    ticket=dict(request_id=int(ticket.request_id), generation=int(ticket.generation),
                                target_epoch=int(ticket.target_epoch)),
                    source_witnesses=proof['source_witnesses'],
                    current_witness=source_witness(reference['contract'], reference['object'], reference['mask']),
                    issued_ns=now, snapshot_policy=proof['snapshot_policy'])

    def _publish_preview_plan(self, rich_plan, legacy_plan, ticket=None, **kwargs):
        held = None
        receipt = (self._station_supplement_computation_receipt(rich_plan, ticket)
                   if self._station_supplement_preview_work(rich_plan, ticket) else None)
        with self._stream_condition:
            if getattr(rich_plan, 'valid', False):
                self._station_supplement_proposal = None
                if receipt is not None:
                    self._require_stream_ticket_current_locked(ticket)
                    held = dict(plan=deepcopy(rich_plan), receipt=receipt, published=False)
                    self._station_supplement_proposal = held
            else:
                # Clock-only notices bypass this wrapper. Ordinary failures
                # revoke the private view even if their text resembles pending.
                self._station_supplement_proposal = None
        try:
            result = super()._publish_preview_plan(rich_plan, legacy_plan, ticket=ticket, **kwargs)
        except BaseException:
            with self._stream_condition:
                if self._station_supplement_proposal is held:
                    self._station_supplement_proposal = None
            raise
        if held is not None:
            with self._stream_condition:
                if self._station_supplement_proposal is held:
                    held['published'] = True
        return result

    def _write_gate_audit_report(self, report, ticket=None, **kwargs):
        with self._stream_condition:
            held = self._station_supplement_proposal
            if (held is not None and held['published'] is True and ticket is not None
                    and report.get('plan_id') == held['plan'].plan_id
                    and report.get('planning_origin') == 'observe_return_supplemental_observation'
                    and held['receipt']['ticket'] == dict(request_id=int(ticket.request_id),
                        generation=int(ticket.generation), target_epoch=int(ticket.target_epoch))):
                validate_supplement_receipt(held['receipt'], held['plan'],
                    now_ns=int(self._ros_source_clock_ns()))
                report[SUPPLEMENT_RECEIPT] = deepcopy(held['receipt'])
        return super()._write_gate_audit_report(report, ticket=ticket, **kwargs)

    def _station_compute_support_gap_live(self, record, now):
        hold = record.get('support_gap')
        if hold is not None:
            validate_station_support_gap(hold['receipt'], hold['parent'], now)
            if time.monotonic() >= hold['monotonic_deadline']:
                raise ValueError('station_support_gap_monotonic_expired')

    def _station_compute_support_gap_receipt(self, status, recovery=False):
        record = self._station_compute_reference
        receipt = status.get('station_support_gap_recovery' if recovery else 'station_support_gap')
        hold = None if record is None else record.get('support_gap')
        if receipt is None:
            if (recovery and hold is not None or not recovery
                    and status.get('detail') == 'support_measurement_pending_revalidation'):
                raise ValueError('station_support_gap_receipt_missing')
            return
        if record is None:
            raise ValueError('station_support_gap_requires_registered_parent')
        now = int(self._ros_source_clock_ns())
        parent = status.get('station_compute_recovery' if recovery else 'station_compute_wait')
        ns = status.get('source_stamp_ns')
        if (status.get('target_lost') is not False or status.get('first_loss') is not None
                or (not recovery and (status.get('state') != 'processing_deadline_pending'
                    or status.get('detail') != 'support_measurement_pending_revalidation'
                    or receipt.get('source_ns') != ns))):
            raise ValueError('station_support_gap_status_changed')
        validate_recovery_parent(record['contract'], parent=parent,
            chain=status.get('station_compute_recovery_chain'),
            binding=record['contract']['binding'], now_ns=now,
            source_ns=ns if recovery else None)
        checked = validate_station_support_gap(receipt, parent, now,
            source_ns=ns if recovery else None)
        if hold is not None:
            self._station_compute_support_gap_live(record, now)
            if (parent != hold['parent'] or any(checked[k] != hold['receipt'][k]
                    for k in ('first_source_ns', 'deadline_ns', 'parent_source_ns', 'source_id'))
                    or recovery and checked['source_ns'] < hold['receipt']['source_ns']):
                raise ValueError('station_support_gap_parent_or_deadline_changed')
            # Repeated/out-of-order notifications never reset either clock.
            if checked['source_ns'] > hold['receipt']['source_ns']:
                hold['receipt'] = checked
        else:
            record['support_gap'] = dict(receipt=checked, parent=deepcopy(parent),
                monotonic_deadline=min(record['identity_monotonic'],
                    time.monotonic()+(checked['deadline_ns']-now)/1e9))
        self._station_compute_quarantine(checked['source_ns'])

    def _station_compute_tick_locked(self):
        # Ticket/publication entry points already own stream, but not always
        # geometry. Keep invalidation and deadline checks in the same order.
        with self._geometry_state_guard():
            return self._station_compute_tick_geometry_locked()

    def _station_compute_tick_geometry_locked(self):
        if self._station_requalification_hold is not None:
            try:
                self._station_requalification_validate()
            except (ValueError, KeyError, TypeError, AttributeError) as exc:
                self._station_compute_retire(str(exc))
            return self._station_compute_pending
        reference = getattr(self, '_station_compute_reference', None)
        if reference is None:
            return False
        if not self._station_compute_scope():
            if self._station_compute_pending:
                if not self._station_contact_preserve_history():
                    self._station_compute_retire('pending station context changed')
            else:
                # CONTACT/motion leaves this exception before any action. Its
                # original frozen-contact loss rules continue unchanged.
                self._station_compute_reference = None
            return False
        try:
            self._station_compute_context(reference['contract'])
            now = int(self._ros_source_clock_ns())
            mono = time.monotonic()
            if now < reference['last_now_ns'] or mono >= reference['identity_monotonic']:
                raise ValueError('station_compute_wait_clock_or_deadline')
            reference['last_now_ns'] = now
            self._station_compute_support_gap_live(reference, now)
            state = validate_contract(reference['contract'],
                binding=reference['contract']['binding'], now_ns=now)
            if state == 'PENDING' or mono >= reference['geometry_monotonic']:
                self._station_compute_quarantine(clock_only=True)
        except _StationRequalificationPending:
            pass
        except (ValueError, KeyError, TypeError, AttributeError) as exc:
            self._station_compute_retire(str(exc))
        return self._station_compute_pending

    def _station_compute_poll(self):
        with self._mode_control_lock, self._stream_condition, self._geometry_state_guard():
            self._station_compute_join_locked()
            self._station_compute_tick_locked()
            return self._station_compute_pending or self._station_compute_retired

    def _station_compute_ticket_code(self, ticket):
        with self._geometry_state_guard():
            return self._station_compute_ticket_code_geometry_locked(ticket)

    def _station_compute_ticket_code_geometry_locked(self, ticket):
        self._station_compute_join_locked()
        self._station_compute_tick_locked()
        if self._station_compute_retired:
            return 'TARGET_LOST'
        if self._station_compute_pending:
            return '' if (self._station_compute_soft_wait
                and self._station_compute_work_certificate(ticket)) else CODE
        if self._station_compute_cutoff_ns:
            try:
                snapshot = ticket.payload[0]
                stamps = tuple(getattr(snapshot, 'sample_stamp_ns', ())) or (snapshot.stamp_ns,)
            except (AttributeError, TypeError, IndexError):
                stamps = (round(float(ticket.snapshot_stamp_sec) * 1e9),)
            if any(int(ns) <= self._station_compute_cutoff_ns for ns in stamps):
                return CODE
        if self._station_compute_soft_cutoff_ns and not self._station_compute_work_certificate(ticket):
            try:
                snapshot = ticket.payload[0]
                stamps = tuple(getattr(snapshot, 'sample_stamp_ns', ())) or (snapshot.stamp_ns,)
                if any(int(ns) <= self._station_compute_soft_cutoff_ns for ns in stamps):
                    return CODE
            except (AttributeError, TypeError, IndexError):
                return CODE
        return ''

    def _station_requalification_withdraw_notice(self, status):
        """An exact producer notice may withdraw authority, never prove drift."""
        if (self._station_compute_reference is None
                or status.get('mode') != 'unknown'
                or status.get('state') != 'pending'
                or status.get('detail') != STATION_REQUALIFICATION
                or status.get('target_lost') is not False
                or status.get('first_loss') is not None):
            return False
        witness = status.get(STATION_REQUALIFICATION)
        try:
            parent = witness['parent_contract']
            known = self._station_requalification_parents.get(parent['source_ns'])
            now = int(self._ros_source_clock_ns())
            ns = status.get('source_stamp_ns')
            if known is None or known['contract'] != parent:
                return False
            if ns is None:
                # process(None) emits state only: no RGB-D/TF source exists.
                # Do not synthesize a frame to pass a measurement-source gate.
                if any(key in status for key in ('color_stamp_ns', 'depth_stamp_ns',
                        'tf_lookup_stamp_ns', 'exact_rgbd', 'color_frame_id', 'depth_frame_id')):
                    return False
            elif (type(ns) is not int or not parent['source_ns'] < ns <= now
                    or reference_binding(status, self._mode_selection)
                        != parent['binding']['reference_binding']):
                return False
            # This checks the wire shape only. The supplied sample below is
            # not recorded as local SDK evidence and cannot enroll recovery.
            validate_station_requalification(witness, parent=known['contract'],
                feedback=[(witness['feedback_stamp_ns'], witness['feedback_q_rad'])], now_ns=now)
            self._station_requalification_validate({STATION_REQUALIFICATION: witness})
            self._station_compute_quarantine(ns)
            self._station_requalification_notice = True
            return True
        except (ValueError, KeyError, TypeError, AttributeError, OverflowError):
            return False

    def _station_compute_status_cb(self, status):
        with self._mode_control_lock, self._stream_condition, self._geometry_state_guard():
            if (status.get('generation') != (self._mode_selection or {}).get('generation')
                    or status.get('selection_stamp_ns') != (self._mode_selection or {}).get('stamp_ns')):
                return
            reference = self._station_compute_reference
            if ((reference is not None or self._station_requalification_hold is not None
                    or self._station_contact_history is not None)
                    and status.get('mode') == 'unknown'
                    and (status.get('target_lost') is True or status.get('first_loss') is not None)):
                self._station_compute_retire('producer retired target')
                return
            if self._station_requalification_withdraw_notice(status):
                return
            ns = status.get('source_stamp_ns')
            if type(ns) is not int or ns <= 0:
                return
            if self._station_gap_late_status(status):
                return
            if self._station_contact_source_changed('status', ns, status):
                self._station_compute_quarantine(ns)
                return
            if (reference is not None and ns < reference['contract']['source_ns']
                    and ns <= self._station_compute_cutoff_ns
                    and status.get('state') == 'processing_deadline_pending'
                    and status.get('target_lost') is False and status.get('first_loss') is None):
                return  # Late old pending cannot withdraw a recovered source.
            self._unknown_identity_cache(self._station_compute_status, ns, deepcopy(status))
            if (reference is not None and ns == reference['contract']['source_ns']
                    and status.get('state') == 'ready'
                    and status.get('station_compute_wait') is not None
                    and status['station_compute_wait'] != reference['contract']):
                self._station_compute_retire('same ready source changed its contract')
                return
            if (self._station_compute_reference is not None or self._station_requalification_hold is not None
                    or self._station_contact_history is not None) and (status.get('target_lost') is True or status.get('first_loss') is not None):
                self._station_compute_retire('producer retired target')
                return
            self._station_compute_admit_ready_before_tick(ns)
            self._station_compute_tick_locked()
            reference = self._station_compute_reference
            if reference is not None and status.get('state') == 'processing_deadline_pending':
                try:
                    now = int(self._ros_source_clock_ns())
                    if (status.get('target_lost') is not False
                            or status.get('first_loss') is not None
                            or ns > now
                            or reference_binding(status, self._mode_selection)
                               != reference['contract']['binding']['reference_binding']):
                        raise ValueError('processing pending receipt changed its registered identity')
                    validate_recovery_parent(reference['contract'],
                        parent=status.get('station_compute_wait'),
                        chain=status.get('station_compute_recovery_chain'),
                        binding=reference['contract']['binding'], now_ns=now)
                    self._station_compute_support_gap_receipt(status)
                    self._station_compute_quarantine(ns)
                except (ValueError, KeyError, TypeError) as exc:
                    self._station_compute_retire(str(exc))
                    return
            self._station_compute_join_locked()

    def _station_compute_admit_ready_before_tick(self, ns):
        # All three real messages may have arrived while callbacks waited for
        # their locks. Audit that complete fresh source before expiring its
        # older pin, otherwise a usable replacement first cancels private work.
        # This is the normal full join: no source, proof or deadline is renewed.
        status = self._station_compute_status.get(ns, {})
        obj = self._station_compute_objects.get(ns)
        mask = self._station_compute_masks.get(ns)
        if (status.get('state') == 'ready' and obj is not None
                and obj.detected and mask is not None and mask[0]):
            self._station_compute_join_locked()

    def _station_compute_join_locked(self):
        admission = getattr(self, '_measurement_remote_admission', None) or {}
        if (self._station_requalification_notice
                or admission.get(STATION_REQUALIFICATION) is not None):
            return  # Status/request/SDK fan-in cannot republish the canceled parent.
        if self._station_compute_retired or not self._station_compute_scope():
            return
        reference = self._station_compute_reference
        for ns in sorted(self._station_compute_status):
            # Original object/mask gates can consume part of a frame budget.
            # Each cached candidate must be assessed with the current clock.
            now = int(self._ros_source_clock_ns())
            status = self._station_compute_status[ns]
            if reference is not None and ns <= reference['contract']['source_ns']:
                continue
            obj = self._station_compute_objects.get(ns)
            mask = self._station_compute_masks.get(ns)
            if obj is None:
                continue
            try:
                if not obj.detected:
                    if reference is None:
                        continue
                    if (status.get('state') != 'processing_deadline_pending'
                            or status.get('target_lost') is not False
                            or status.get('first_loss') is not None):
                        raise ValueError('untyped false object cannot preserve station')
                    validate_recovery_parent(reference['contract'],
                        parent=status.get('station_compute_wait'),
                        chain=status.get('station_compute_recovery_chain'),
                        binding=reference['contract']['binding'], now_ns=now)
                    self._station_compute_support_gap_receipt(status)
                    self._station_compute_quarantine(ns)
                    continue
                if ns <= self._station_compute_cutoff_ns:
                    continue
                if status.get('state') != 'ready' or mask is None:
                    continue
                contract = status.get('station_compute_wait')
                if contract is None:
                    continue
                if self._station_requalification_hold is not None:
                    self._station_requalification_validate()
                self._station_compute_context(contract)
                binding = reference_binding(status, self._mode_selection)
                if (binding != contract['binding']['reference_binding']
                        or contract['source_ns'] != ns or not mask[0]
                        or mask[1] != status['color_frame_id']
                        or status.get('station_compute_mask') != mask[3]
                        or obj.header.stamp.to_nsec() != ns
                        or obj.header.frame_id != 'base_link'
                        or obj.label != 'unknown_small'):
                    raise ValueError('station ready does not match actual object/mask/source')
                self._station_compute_support_gap_receipt(status, recovery=True)
                if reference is not None:
                    self._station_compute_support_gap_live(reference, now)
                recovered = bool(reference is not None and (
                    self._station_compute_pending
                    or now >= reference['contract']['geometry_deadline_ns']
                    or time.monotonic() >= reference['geometry_monotonic']))
                hard_recovery = self._station_compute_pending and not self._station_compute_soft_wait
                gap_sources = ()
                gap_receipt = status.get('station_feedback_gap_recovery')
                if gap_receipt is not None:
                    if reference is None:
                        raise ValueError('station gap recovery requires registered parent')
                    parent = status.get('station_compute_recovery')
                    validate_recovery_parent(reference['contract'], parent=parent,
                        chain=status.get('station_compute_recovery_chain'),
                        binding=contract['binding'], now_ns=now, source_ns=ns)
                    gap_sources = validate_station_feedback_gap(gap_receipt,
                        contract=parent, now_ns=now, recovery_source_ns=ns)
                    # A missed false/pending publication must not turn a real
                    # rejected input into the clock-only private-work exception.
                    self._station_compute_quarantine(max(gap_sources))
                    recovered = hard_recovery = True
                if reference is not None:
                    if contract['binding'] != reference['contract']['binding']:
                        raise ValueError('station binding changed')
                    if recovered:
                        if status.get('station_compute_recovery') is None:
                            # A normal READY may not have run the additional
                            # recovery proof while its producer reference was
                            # still fresh. It cannot recover OUR older pin.
                            continue
                # Stale cached READY is unusable, not a positive identity
                # contradiction. It cannot shorten or renew the old wait.
                validate_contract(contract, binding=contract['binding'], now_ns=ns)
                if not ns <= now < min(contract['geometry_deadline_ns'], contract['identity_deadline_ns']):
                    continue
                self._station_compute_check_joints(contract['binding'], ns)
                if reference is None:
                    self._station_compute_check_joints(contract['binding'],
                        self._measurement_remote_request['source_stamps_ns'][-1])
                admitted = validate_registration(contract, binding=contract['binding'], now_ns=now)
                if reference is not None:
                    if recovered:
                        if status.get('station_compute_recovery') is None:
                            continue
                        validate_recovery_parent(reference['contract'],
                            parent=status['station_compute_recovery'],
                            chain=status.get('station_compute_recovery_chain'),
                            binding=contract['binding'], now_ns=now, source_ns=ns)
                        if time.monotonic() >= reference['identity_monotonic']:
                            raise ValueError('recovery local deadline expired')
                # Run the original mask/object gates on the actual new data.
                # The reference remains pending until both calls and all
                # post-call lifecycle/deadline checks have succeeded.
                super().mask_cb(mask[2])
                super().object_cb(obj)
                self._station_compute_context(contract)
                now = int(self._ros_source_clock_ns())
                if (self._current_stream_target_identity().epoch != contract['binding']['target_epoch']
                        or not getattr(self.latest_object, 'detected', False)
                        or self.latest_object.header.stamp.to_nsec() != ns):
                    raise ValueError('original measured object gate rejected recovery')
                if reference is not None:
                    # A candidate aging during its original measurement gates
                    # cannot renew or destroy a still-live registered identity.
                    # Real owner/feedback/clock/deadline failures remain fatal.
                    self._station_compute_context(reference['contract'])
                    self._station_compute_support_gap_live(reference, now)
                    if (now < reference['last_now_ns']
                            or time.monotonic() >= reference['identity_monotonic']):
                        raise ValueError('station_compute_wait_clock_or_deadline')
                    validate_contract(reference['contract'],
                        binding=reference['contract']['binding'], now_ns=now)
                    recovered = bool(recovered
                        or now >= reference['contract']['geometry_deadline_ns']
                        or time.monotonic() >= reference['geometry_monotonic'])
                    if recovered:
                        if status.get('station_compute_recovery') is None:
                            continue
                        validate_recovery_parent(reference['contract'],
                            parent=status['station_compute_recovery'],
                            chain=status.get('station_compute_recovery_chain'),
                            binding=contract['binding'], now_ns=now)
                if now >= admitted['geometry_deadline_ns']:
                    if reference is not None:
                        self._station_compute_quarantine(ns, clock_only=True)
                    continue  # Discard this source; never register its clock.
                validate_registration(admitted, binding=contract['binding'], now_ns=now)
                if recovered:
                    self._station_compute_support_gap_live(reference, now)
                    validate_recovery_parent(reference['contract'],
                        parent=status['station_compute_recovery'],
                        chain=status.get('station_compute_recovery_chain'),
                        binding=contract['binding'], now_ns=now, source_ns=ns)
                    if time.monotonic() >= reference['identity_monotonic']:
                        raise ValueError('recovery expired during commit')
                    if hard_recovery:
                        self._station_compute_cutoff_ns = max(self._station_compute_cutoff_ns, ns - 1)
                        self._direct_near_field_submission_generation = None
                mono = time.monotonic()
                if gap_sources:
                    self._unknown_identity_cache(self._station_feedback_gap_history,
                        reference['contract']['source_ns'], dict(contract=deepcopy(reference['contract']),
                            object=deepcopy(reference['object']), mask=deepcopy(reference['mask'])))
                    for discarded in gap_sources:
                        self._unknown_identity_cache(self._station_feedback_gap_tombstones, discarded,
                            dict(contract=deepcopy(status['station_compute_recovery']), recovery_source_ns=ns))
                if self._station_requalification_hold is not None:
                    self._station_requalification_validate()
                    active = self._measurement_remote_request
                    if (active.get('state') != 'qualified'
                            or active.get(STATION_REQUALIFICATION) != self._station_requalification_hold['witness']
                            or admitted['binding']['lease_id'] != active['lease_id']
                            or admitted['binding']['phase_id'] <= self._station_requalification_hold['witness']['parent_contract']['binding']['phase_id']):
                        raise ValueError('STATION_REQUALIFICATION_NEW_REGISTRATION_REQUIRED')
                self._station_compute_reference = dict(contract=admitted, last_now_ns=now,
                    object=deepcopy(obj), mask=deepcopy(mask[3]),
                    geometry_monotonic=mono + (admitted['geometry_deadline_ns'] - now) / 1e9,
                    identity_monotonic=mono + (admitted['identity_deadline_ns'] - now) / 1e9)
                self._station_requalification_parents[ns] = dict(contract=deepcopy(admitted),
                    identity_monotonic=self._station_compute_reference['identity_monotonic'],
                    measurement_reference=deepcopy(self._measurement_remote_reference))
                while len(self._station_requalification_parents) > 64:
                    del self._station_requalification_parents[min(self._station_requalification_parents)]
                self._station_requalification_hold = None
                self._station_compute_pending = False
                self._station_compute_soft_wait = False
                if reference is None:
                    self._station_compute_work_published = False
                    self._station_compute_publication_started = False
                    self._station_compute_ready_history.clear()
                    self._station_compute_work.clear()
                self._station_compute_ready_history[ns] = dict(
                    contract=deepcopy(admitted),
                    binding=deepcopy(admitted['binding']), object=deepcopy(obj),
                    mask=deepcopy(mask[3]), admitted_ns=now,
                    deadline_ns=admitted['geometry_deadline_ns'])
                while len(self._station_compute_ready_history) > 64:
                    del self._station_compute_ready_history[min(self._station_compute_ready_history)]
                reference = self._station_compute_reference
                self._stream_condition.notify_all()
            except _StationRequalificationPending:
                return
            except (ValueError, KeyError, TypeError, AttributeError, OverflowError) as exc:
                if reference is not None or self._station_requalification_hold is not None:
                    self._station_compute_retire(str(exc))
                    return
                # Missing/late registration cannot create a new exception.
                continue

    def object_cb(self, msg):
        if (not hasattr(self, '_stream_condition')
                or ((getattr(self, '_measurement_remote_request', None) or {}).get('state') != 'qualified'
                    and self._station_compute_reference is None)):
            return super().object_cb(msg)
        with self._stream_condition, self._geometry_state_guard():
            ns = msg.header.stamp.to_nsec()
            if self._station_contact_source_changed('object', ns, msg):
                self._station_compute_quarantine(ns)
                return
            if self._station_requalification_hold is not None and not msg.detected:
                self._station_compute_retire('producer published false object during station requalification')
                return
            history = self._station_feedback_gap_history.get(ns)
            if history is not None and msg.detected is True and msg != history['object']:
                self._station_compute_retire('historical station object changed')
                return
            if ns in self._station_feedback_gap_tombstones:
                if msg.detected is not False:
                    self._station_compute_retire('discarded station source became detected')
                return
            reference = self._station_compute_reference or self._station_contact_history
            if (ns <= self._station_compute_cutoff_ns
                    and (not self._station_compute_pending
                         or (reference is not None and ns < reference['contract']['source_ns']))):
                return
            audited = self._station_compute_ready_history.get(ns)
            if audited is not None and msg != audited['object']:
                self._station_compute_quarantine(ns)
            self._unknown_identity_cache(self._station_compute_objects, ns, deepcopy(msg))
            self._station_compute_admit_ready_before_tick(ns)
            self._station_compute_tick_locked()
            reference = self._station_compute_reference
            if (reference is not None and ns == reference['contract']['source_ns']
                    and msg.detected and msg != reference['object']):
                self._station_compute_retire('same source changed measured object')
                return
            if (self._station_compute_reference is not None or self._station_contact_history is not None) and not msg.detected:
                self._station_compute_quarantine(ns)
            if self._station_compute_pending:
                self._station_compute_join_locked()
                return
            result = super().object_cb(msg)
            self._station_compute_join_locked()
            return result

    def mask_cb(self, msg):
        if (not hasattr(self, '_stream_condition')
                or ((getattr(self, '_measurement_remote_request', None) or {}).get('state') != 'qualified'
                    and self._station_compute_reference is None)):
            return super().mask_cb(msg)
        with self._stream_condition, self._geometry_state_guard():
            ns = msg.header.stamp.to_nsec()
            if ns in self._station_feedback_gap_history or ns in self._station_feedback_gap_tombstones:
                pixels = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
                normalized = np.ascontiguousarray(pixels != 0, dtype=np.uint8)
                current_mask = dict(sha256=hashlib.sha256(normalized.tobytes()).hexdigest(),
                    shape=list(normalized.shape), frame_id=msg.header.frame_id)
                history = self._station_feedback_gap_history.get(ns)
                if history is not None and current_mask != history['mask']:
                    self._station_compute_retire('historical station mask changed')
                    return
                if ns in self._station_feedback_gap_tombstones:
                    if np.any(normalized):
                        self._station_compute_retire('discarded station source became nonempty')
                    return
            reference = self._station_compute_reference or self._station_contact_history
            if (ns <= self._station_compute_cutoff_ns
                    and (not self._station_compute_pending
                         or (reference is not None and ns < reference['contract']['source_ns']))):
                return
            mask = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
            normalized = np.ascontiguousarray(mask != 0, dtype=np.uint8)
            digest = dict(sha256=hashlib.sha256(normalized.tobytes()).hexdigest(),
                shape=list(normalized.shape), frame_id=msg.header.frame_id)
            if self._station_contact_source_changed('mask', msg.header.stamp.to_nsec(), digest):
                self._station_compute_quarantine(msg.header.stamp.to_nsec())
                return
            audited = self._station_compute_ready_history.get(ns)
            if audited is not None and digest != audited['mask']:
                self._station_compute_quarantine(ns)
            self._unknown_identity_cache(self._station_compute_masks, msg.header.stamp.to_nsec(),
                (bool(mask.ndim == 2 and np.any(mask)), msg.header.frame_id, deepcopy(msg), digest))
            self._station_compute_admit_ready_before_tick(ns)
            self._station_compute_tick_locked()
            reference = self._station_compute_reference
            if (reference is not None and msg.header.stamp.to_nsec() == reference['contract']['source_ns']
                    and np.any(mask) and digest != reference['mask']):
                self._station_compute_retire('same source changed measured mask')
                return
            if self._station_contact_history is not None and not (mask.ndim == 2 and np.any(mask)):
                self._station_compute_quarantine(ns)
                return
            if self._station_compute_pending:
                self._station_compute_join_locked()
                return
            result = super().mask_cb(msg)
            self._station_compute_join_locked()
            return result


    def _station_compute_snapshot_policy(self):
        """Match the actual collector purpose without extending its authority.

        Contact uses the configured direct snapshot count. A supplementary
        observation at the same qualified station uses the rolling collector's
        own count. The internal purpose is frozen with each computation so a
        five-frame observation cannot turn into a contact certificate.
        """
        if not self._station_compute_scope():
            return None
        phase = self._observe_return_phase
        if (phase == 'observing_contact' and self.near_field_planning_active
                and self._configured_near_field_strategy() == 'single_snapshot_direct'):
            count = self.near_field_planning_snapshot_frames
            purpose = 'direct_contact'
        elif (phase == 'observation_prefix' and not self.near_field_planning_active
                and self._independent_observation_planning_active()):
            count = self.planning_snapshot_frames
            purpose = 'supplementary_observation'
        else:
            return None
        if isinstance(count, bool) or not isinstance(count, Integral) or count <= 0:
            return None
        return dict(purpose=purpose, phase=phase, frame_count=int(count),
            maximum_span_sec=self.planning_snapshot_max_span_sec)

    def _station_compute_snapshot_work(self, snapshot):
        """Bind a newly submitted computation to individually audited inputs."""
        if (self._station_compute_reference is None or self._station_compute_pending
                or self._station_compute_retired or self._station_compute_work_published
                or self._station_compute_publication_started):
            return None
        try:
            from alicia_flexible_grasp.vision.rgbd_snapshot import retained_exact_source
            contract = self._station_compute_reference['contract']
            self._station_compute_context(contract)
            now = int(self._ros_source_clock_ns())
            validate_registration(contract, binding=contract['binding'], now_ns=now)
            stamps = tuple(snapshot.sample_stamp_ns)
            policy = self._station_compute_snapshot_policy()
            if policy is None:
                return None
            count = policy['frame_count']
            raw = retained_exact_source(snapshot)
            records = [self._station_compute_ready_history[ns] for ns in stamps]
            if (not snapshot.ok or len(stamps) != count or snapshot.quality.fused_frames != count
                    or any(a >= b for a,b in zip(stamps,stamps[1:]))
                    or not stamps or stamps[-1] != snapshot.stamp_ns
                    or not 0 <= now-stamps[-1] < 800_000_000
                    or stamps[-1]-stamps[0] > round(self.planning_snapshot_max_span_sec*1e9)
                    or snapshot.target_identity != self._current_stream_target_identity()
                    or raw is None or raw.object_msg != records[-1]['object']
                    or hashlib.sha256(np.ascontiguousarray(np.asarray(raw.object_mask)>0,
                        dtype=np.uint8).tobytes()).hexdigest() != records[-1]['mask']['sha256']
                    or tuple(snapshot.sample_mask_sha256) != tuple(r['mask']['sha256'] for r in records)
                    or any(r['binding'] != contract['binding']
                           or not ns <= r['admitted_ns'] < r['deadline_ns']
                           or r['deadline_ns']-ns > 800_000_000
                           or self._station_compute_objects.get(ns) != r['object']
                           or self._station_compute_masks.get(ns, (None,)*4)[3] != r['mask']
                           for ns,r in zip(stamps,records))):
                return None
            return dict(snapshot=snapshot, binding=deepcopy(contract['binding']),
                generation=self._stream_generation, target_epoch=self.target_instance_epoch,
                snapshot_policy=policy, source_stamps_ns=stamps,
                source_witnesses=[source_witness(r['contract'], r['object'], r['mask'])
                                  for r in records])
        except (ValueError, KeyError, TypeError, AttributeError, IndexError):
            return None

    def _station_compute_register_work(self, snapshot, proof):
        if proof is None:
            return
        request_id = self._pending_request_id
        if request_id is None:
            ticket = self._stream_worker_ticket
            if ticket is None or ticket.payload[0] is not snapshot:
                return
            request_id = ticket.request_id
        self._station_compute_work[int(request_id)] = proof
        while len(self._station_compute_work) > 64:
            del self._station_compute_work[min(self._station_compute_work)]

    def _station_compute_work_certificate(self, ticket):
        proof = self._station_compute_work.get(int(getattr(ticket, 'request_id', -1)))
        reference = self._station_compute_reference
        return bool(proof is not None and reference is not None
            and proof['generation'] == ticket.generation == self._stream_generation
            and proof['target_epoch'] == ticket.target_epoch == self.target_instance_epoch
            and proof['snapshot'] is ticket.payload[0]
            and proof.get('snapshot_policy') is not None
            and proof['snapshot_policy'] == self._station_compute_snapshot_policy()
            and proof.get('source_stamps_ns') == tuple(ticket.payload[0].sample_stamp_ns)
            and proof['binding'] == reference['contract']['binding'])

    def _station_contact_preview_work(self, plan, ticket):
        """A registered computation proposal, never an execution certificate.

        Only this exact contact path can keep private work after its preview.
        Other previews and all actual authority publication keep their original
        publication barrier. Neither source nor measured geometry is restamped.
        """
        if ticket is None or not self._station_compute_work_certificate(ticket):
            return False
        proof = self._station_compute_work.get(int(ticket.request_id), {})
        try:
            if (plan.valid is not True or plan.diagnostic != 'CONTACT_EXECUTION_PLAN'
                    or plan.model_choice != 'unknown_tabletop'
                    or plan.refinement_status != 'VALID_3D'
                    or plan.target_track_id != proof['binding']['target_track_id']
                    or plan.header.stamp.to_nsec() != ticket.payload[0].stamp_ns
                    or plan.object_geometry.header.stamp != plan.header.stamp
                    or len(proof.get('source_witnesses', ())) != 3
                    or proof['snapshot_policy']['purpose'] != 'direct_contact'
                    or proof['snapshot_policy']['phase'] != 'observing_contact'):
                return False
            self._observe_return_original.decode_contact_evidence(plan, required=True)
            from alicia_flexible_grasp.grasp.rich_plan_integrity import (
                compute_plan_id, validate_plan_header_binding)
            validate_plan_header_binding(plan)
            if compute_plan_id(plan) != plan.plan_id:
                return False
            return True
        except (ValueError, KeyError, TypeError, AttributeError, IndexError):
            return False

    def _station_contact_retained_proposal(self, ticket):
        if not self._station_compute_work_certificate(ticket):
            return None
        plan = self._station_compute_work[int(ticket.request_id)].get('contact_proposal')
        return deepcopy(plan) if self._station_contact_preview_work(plan, ticket) else None

    def _station_contact_commit_audit(self, report, reference, ticket):
        """Called only after the original atomic audit commit succeeds."""
        if ticket is None or CONTACT_RECEIPT not in report:
            return
        proof = self._station_compute_work.get(int(ticket.request_id), {})
        plan = proof.get('contact_proposal')
        receipt = report[CONTACT_RECEIPT]
        if (proof.get('contact_published') is not True or plan is None
                or report.get('plan_id') != plan.plan_id
                or reference.get('atomic_committed') is not True
                or reference != getattr(self, '_latest_gate_audit_reference', None)):
            return
        binding = validate_receipt(receipt, plan, now_ns=int(self._ros_source_clock_ns()))
        if (binding != proof['binding']
                or receipt['ticket'] != dict(request_id=int(ticket.request_id),
                    generation=int(ticket.generation), target_epoch=int(ticket.target_epoch))):
            raise ValueError('station_contact_audit_work_changed')
        proof['contact_audit'] = dict(receipt=deepcopy(receipt), reference=deepcopy(reference))

    def _station_contact_preserve_history(self):
        """Disarm stationary privileges, retaining only a published audit fact.

        MOVE is not evidence that a task froze this plan. No ACK is issued here;
        the exact, parsed retirement request must still match this history and
        pass every original contact owner/phase/deadline check afterwards.
        """
        if (not self._station_compute_pending or not self._station_compute_soft_wait
                or self._station_compute_retired
                or self._station_compute_task_state != 'MOVE_PREGRASP'
                or not getattr(self, 'robot_execution_active', False)):
            return False
        reference = self._station_compute_reference or self._station_contact_history
        if reference is None:
            return False
        from alicia_flexible_grasp.robot.observation_path_guard import committed_plan_digest
        for proof in list(self._station_compute_work.values()):
            plan = proof.get('contact_proposal')
            if plan is None:
                continue
            try:
                # A local lookup key, never a prepare or a claim of task freeze.
                query = dict(proof['binding'], plan_id=plan.plan_id,
                    plan_source_ns=plan.header.stamp.to_nsec(),
                    plan_sha256=committed_plan_digest(plan),
                    measurement_sha256=measurement_digest(self._measurement_remote_request),
                    deadline_ns=min(reference['contract']['phase_deadline_ns'],
                        reference['contract']['session_deadline_ns']))
                self._contact_station_live_context(query)
                if not self._station_contact_audited_proposals(query):
                    continue
                self._station_contact_history = dict(contract=deepcopy(reference['contract']))
                self._station_compute_reference = None
                return True
            except (ValueError, KeyError, TypeError, AttributeError, IndexError):
                continue
        return False

    def _station_contact_pending_retirement_allowed(self, request):
        # Both prepare and retire arrive under the original control/stream/
        # geometry locks. Never turn arbitrary pending into publication or work
        # authority, and never use a latest-preview fallback for this exception.
        return (self._station_contact_preserve_history()
            and bool(self._station_contact_audited_proposals(request)))

    def _station_contact_audited_proposals(self, request):
        """An already published/audited proposal can attest its exact retire.

        MOVE_PREGRASP normally disarms the planning reference. Read the
        immutable committed proof, not a fresh planning-work privilege; every
        original contact-station context/prepare/retire check still follows.
        """
        from alicia_flexible_grasp.robot.observation_path_guard import committed_plan_digest
        plans = []
        now = int(self._ros_source_clock_ns())
        for request_id, proof in list(self._station_compute_work.items()):
            plan = proof.get('contact_proposal')
            audit = proof.get('contact_audit')
            if (proof.get('contact_published') is not True or plan is None or not audit
                    or plan.plan_id != request['plan_id']
                    or plan.header.stamp.to_nsec() != request['plan_source_ns']
                    or committed_plan_digest(plan) != request['plan_sha256']):
                continue
            try:
                binding = validate_receipt(audit['receipt'], plan, now_ns=now)
                report = getattr(self, '_active_gate_audit_report', {}) or {}
                if (audit['reference'] != getattr(self, '_latest_gate_audit_reference', None)
                        or audit['reference'].get('atomic_committed') is not True
                        or report.get(CONTACT_RECEIPT) != audit['receipt']
                        or report.get('plan_id') != plan.plan_id
                        or proof['binding'] != binding
                        or proof['generation'] != self._stream_generation
                        or proof['target_epoch'] != self.target_instance_epoch
                        or audit['receipt']['ticket'] != dict(request_id=request_id,
                            generation=proof['generation'],target_epoch=proof['target_epoch'])
                        or any(binding[k] != self._measurement_remote_request.get(k) for k in
                            ('selection','lease_id','session_id','view_plan_id','view_index',
                             'target_epoch','target_track_id','control_epoch_ns'))
                        or any(binding[k] != request[k] for k in
                            ('selection','session_id','target_track_id','target_epoch',
                             'control_epoch_ns','phase_id','phase_started_ns'))):
                    continue
                # This is only the equivalent of finding the original
                # published immutable plan. MOVE has ended stationary joins;
                # their old .8s sample must not be repurposed as an execution
                # gate. The unchanged task/producer contact gates check live
                # measurement and feedback before either ACK or submission.
                plans.append(deepcopy(plan))
            except (ValueError,KeyError,TypeError,AttributeError,IndexError):
                continue
        return plans

    def _station_contact_source_changed(self, kind, ns, value):
        # Long computation may outlive the 64-source observation cache. Its
        # three immutable input witnesses remain obligations until cancellation;
        # an out-of-order contradiction must still withdraw that computation.
        for proof in self._station_compute_work.values():
            for row in proof.get('source_witnesses', ()):
                if row['contract']['source_ns'] != ns:
                    continue
                if kind == 'mask':
                    return value != row['mask']
                if kind == 'object':
                    return source_witness(row['contract'], value, row['mask']) != row
                if kind == 'status':
                    return (value.get('state') != 'ready'
                        or value.get('target_lost') is not False
                        or value.get('first_loss') is not None
                        or value.get('station_compute_wait') != row['contract']
                        or value.get('station_compute_mask') != row['mask'])
        return False

    def _station_contact_computation_receipt(self, plan, ticket):
        """Bind the immutable original proposal to an actual current join.

        The wait releases stream/geometry locks so all three input callbacks
        can run. The final snapshot is taken under the existing ordered locks.
        Its original source deadline and the phase/session limits are unchanged.
        """
        if not self._station_contact_preview_work(plan, ticket):
            return None
        while True:
            self._station_compute_wait_for_current(ticket)
            with self._stream_condition, self._geometry_state_guard():
                self._station_compute_tick_locked()
                self._require_stream_ticket_current_locked(ticket)
                if self._station_compute_pending:
                    continue
                if not self._station_contact_preview_work(plan, ticket):
                    raise self._observe_return_original.StreamResultCancelled(CODE)
                proof = self._station_compute_work[int(ticket.request_id)]
                reference = self._station_compute_reference
                self._station_compute_context(reference['contract'])
                now = int(self._ros_source_clock_ns())
                mono = time.monotonic()
                if now < reference['last_now_ns'] or mono >= reference['identity_monotonic']:
                    raise ValueError('station_compute_wait_clock_or_deadline')
                self._station_compute_support_gap_live(reference, now)
                state = validate_contract(reference['contract'],
                    binding=proof['binding'], now_ns=now)
                reference['last_now_ns'] = now
                if state == 'PENDING' or mono >= reference['geometry_monotonic']:
                    # Context validation may consume the last fresh-source
                    # margin. Keep only this registered owner's private work;
                    # release both locks before waiting for a real new join.
                    # Neither source nor identity/phase clocks are renewed.
                    self._station_compute_quarantine(clock_only=True)
                    continue
                validate_registration(reference['contract'],
                    binding=proof['binding'], now_ns=now)
                return build_receipt(plan,
                    ticket=dict(request_id=int(ticket.request_id),
                                generation=int(ticket.generation),
                                target_epoch=int(ticket.target_epoch)),
                    source_witnesses=proof['source_witnesses'],
                    current_witness=source_witness(reference['contract'],
                        reference['object'], reference['mask']),
                    issued_ns=now, snapshot_policy=proof['snapshot_policy'])

    def _publish_execution_with_token(self, rich_plan, signature, score,
                                      now_sec, ticket, token):
        # Preview/audit provenance is not an executable publication. Close the
        # private computation exception at the original authority boundary,
        # only after the original no-send conditions pass. The super method
        # repeats every audit/token check and retains rollback on partial send.
        if (self._station_contact_preview_work(rich_plan, ticket)
                or self._station_supplement_preview_work(rich_plan, ticket)):
            self._station_compute_wait_for_current(ticket)
            with self._stream_condition, self._geometry_state_guard():
                self._station_compute_tick_locked()
                if self._station_compute_pending:
                    return self._observe_return_original.PromotionDecision(False, CODE,
                        'waiting for a current measured station')
                if not self._execution_promotion_audit_ready(rich_plan):
                    return self._observe_return_original.PromotionDecision(False, 'PLAN_AUDIT_NOT_READY',
                        'execution audit is missing or no longer content-bound')
                current, code = self._promotion_token_state(ticket, token)
                if not current:
                    return self._observe_return_original.PromotionDecision(False, code,
                        'promotion token is stale')
                # A rejected check ends this attempt. A later positive recheck
                # must not reach super's first send without this barrier.
                self._station_compute_publication_started = True
                self._station_supplement_proposal = None
        result = super()._publish_execution_with_token(
            rich_plan, signature, score, now_sec, ticket, token)
        if getattr(result, 'promote', False):
            with self._stream_condition:
                if self._station_compute_publication_started:
                    self._station_compute_work_published = True
        return result

    def _release_preparation_cancelled_direct_snapshot_locked(self, ticket, error):
        """Reopen collection only after a typed, pre-acceptance cancellation.

        A soft wait may cancel a snapshot lacking private-work proof. Keeping
        its one-shot slot would strand the phase even after new measured READY.
        This retires that computation, not any freshness or identity gate.
        The worker calls this only after it has finished, before any acceptance.
        """
        if (not isinstance(error, self._observe_return_original.StreamResultCancelled)
                or error.code != CODE):
            return False
        self._station_compute_tick_locked()
        if (not self.streaming_enabled or self._stream_worker_busy
                or not self.near_field_planning_active
                or self._configured_near_field_strategy() != 'single_snapshot_direct'
                or not self._station_compute_scope()
                or self._station_compute_reference is None or self._station_compute_retired
                or self._station_compute_work_published or self._station_compute_publication_started
                or bool(getattr(getattr(self, 'latest_preview_rich_plan', None), 'valid', False))
                or ticket.generation != self._stream_generation
                or ticket.target_epoch != self.target_instance_epoch
                or self._direct_near_field_submission_generation != ticket.generation
                or ticket.payload[0].target_identity != self._current_stream_target_identity()
                or not self._near_field_phase_started_sec <= ticket.snapshot_stamp_sec
                or not self._near_field_phase_started_sec <= float(self._stream_source_clock())
                    < self._near_field_phase_deadline_sec):
            return False
        # No generation, source watermark, contract or deadline is renewed.
        # Poll/submission must still rejoin fresh measurement and current scope.
        self._station_compute_work.pop(int(ticket.request_id), None)
        self._direct_near_field_submission_generation = None
        self._stream_condition.notify_all()
        rospy.loginfo('Near-field preparation cancelled pending station measurement; '
            'new-source collection released within original phase: request=%d', ticket.request_id)
        return True

    def _station_compute_wait_for_current(self, ticket):
        """Wait without input locks; full recheck only on changed input/deadline."""
        from .selection import parse_selection, SELECTION_PARAM
        seen = None
        with self._stream_condition:
            while True:
                reference = self._station_compute_reference
                now = int(self._ros_source_clock_ns())
                # Cached owner checks do not make a Master RPC every wait tick.
                try:
                    owner = parse_selection(rospy.get_param_cached(SELECTION_PARAM, {}))
                    manual = rospy.get_param_cached('/gui/joint_direct_mode', False)
                except (ValueError, TypeError):
                    owner, manual = None, True
                if owner != self._mode_selection or manual is not False:
                    self._station_compute_retire('station computation owner changed while waiting')
                rows, _ = self._station_compute_feedback.snapshot()
                now = int(self._ros_source_clock_ns())
                latest_feedback_ns = max((ns for ns, _q in rows), default=0)
                signature = (id(reference), self._station_compute_pending,
                    self._station_compute_soft_wait,
                    self._stream_generation,
                    self._station_compute_scope(),
                    not 0 <= now-latest_feedback_ns < 800_000_000,
                    bool(reference and now < reference['last_now_ns']),
                    self._station_compute_retired, self._measurement_remote_epoch_ns,
                    tuple(max(cache, default=0) for cache in (self._station_compute_status,
                        self._station_compute_objects, self._station_compute_masks)),
                    bool(reference and (now >= reference['contract']['geometry_deadline_ns']
                        or time.monotonic() >= reference['geometry_monotonic'])),
                    bool(reference and (now >= reference['contract']['identity_deadline_ns']
                        or time.monotonic() >= reference['identity_monotonic'])))
                if signature != seen:
                    with self._geometry_state_guard():
                        # ticket_code joins, checks authoritative owner and
                        # original ROS/monotonic deadlines exactly once.
                        self._require_stream_ticket_current_locked(ticket)
                    if not self._station_compute_pending:
                        return
                    if not (self._station_compute_soft_wait
                            and self._station_compute_work_certificate(ticket)):
                        raise self._observe_return_original.StreamResultCancelled(CODE)
                    seen = signature
                # Optional supplemental-search cancellation only. This hook
                # cannot certify READY, publish geometry or authorize a path.
                checkpoint = getattr(self, '_station_observation_geometry_wait_checkpoint', None)
                if checkpoint is not None:
                    checkpoint(ticket)
                self._stream_condition.wait(timeout=.05)

    def _activate_prepared_geometry(self, prepared):
        if self._station_compute_work_certificate(prepared.ticket):
            while True:
                self._station_compute_wait_for_current(prepared.ticket)
                with self._stream_condition:
                    self._station_compute_tick_locked()
                    if self._station_compute_pending:
                        continue
                    return super()._activate_prepared_geometry(prepared)
        return super()._activate_prepared_geometry(prepared)

    def _publish_plan_pair_if_current(self, rich_plan, expected_generation):
        with self._stream_condition:
            self._station_compute_tick_locked()
            if self._station_compute_pending or self._station_compute_retired:
                return False, CODE
            # Mirror only the original no-publication preconditions. A reject
            # before any ROS send does not close another valid computation.
            ready = bool(self._station_compute_reference is not None
                and getattr(rich_plan, 'valid', False)
                and int(getattr(self, '_geometry_invalidation_generation', 0)) == expected_generation
                and getattr(self, 'rich_plan_pub', None) is not None)
            if ready:
                try:
                    self._observe_return_original.decode_contact_evidence(rich_plan)
                except ValueError:
                    ready = False
            if ready:
                self._station_compute_publication_started = True
                self._station_supplement_proposal = None
            result = super()._publish_plan_pair_if_current(rich_plan, expected_generation)
            if ready and result[0]:
                self._station_compute_work_published = True
            return result
