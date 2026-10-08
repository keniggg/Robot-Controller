"""Measured-surface lifecycle for observe, return and fresh contact planning.

No robot commands: the task owns motion and supplies verified phase messages.
The remote side owns cancellation barriers and measured target provenance.
"""
from copy import deepcopy
import json
import math

import rospy
from std_msgs.msg import String

from .observe_return_session import ACK_TOPIC, SESSION_TOPIC, parse_session, structured_view_deficit
from .active_observation_policy import registered_information_gain, evidence_digest, distinct_view, information_gain_valid
from .silhouette_observation import validate_silhouette_ambiguity, silhouette_view_gain
from .observe_return_visual import (bounded_bilateral_view_gaps, measured_bilateral_view_gaps,
                                    missing_side_view_gain)


class ObserveReturnRemoteMixin:
    def _reset_observe_return(self):
        self._observe_return_phase = 'initial_contact'
        self._observe_return_session = None
        self._observe_return_saved = None
        self._observe_return_prefix = None
        self._observe_return_origin_prefix = None
        self._observe_return_pending_supplement = None
        self._observe_return_step_gain = None
        self._observe_return_visited_camera_origins = []
        self._observe_return_trigger = ''
        self._observe_return_trigger_evidence = None
        self._observe_return_view_gaps = ()
        self._observe_return_view_uncertainty_m = None
        self._observe_return_directed_views = ()
        self._observe_return_initial_stamp = 0
        self._observe_return_paused = False
        self._observe_return_reached_phase_id = None
        self._observe_return_diagnostic_observation_only = False
        self._observe_return_diagnostic_attempted = False
        self._observe_return_silhouette_evidence = None
        self._observe_return_diagnostic_collision_geometry = None
        self._observe_return_diagnostic_collision_source = None
        self._silhouette_observation_envelope_audit = None
        self._observe_return_initial_contact_deadline_ns = 0
        self._observe_return_identity_transition = None
        self._observe_return_identity_terminal = None

    def _advance_target_instance_epoch(self, reason, preserve_identity=False):
        condition = getattr(self, '_stream_condition', None)
        if condition is None:
            return super()._advance_target_instance_epoch(reason, preserve_identity=preserve_identity)
        # No new-epoch ticket may start between the base cancellation barrier
        # and removal of the old observe-return phase evidence.
        with condition:
            return self._observe_return_advance_target_instance_epoch(reason, preserve_identity)

    def _observe_return_advance_target_instance_epoch(self, reason, preserve_identity):
        """Retire visual evidence when the base tracker retires its identity.

        Before any published prefix/authority, a new identity needs its own
        current-view contact failure. An acknowledged or published observation
        cannot be silently restarted, relabelled, or granted another budget.
        """
        selected = self._observe_return_selected()
        phase = getattr(self, '_observe_return_phase', 'initial_contact')
        active = bool(getattr(self, 'streaming_enabled', False))
        old_identity = self._current_stream_target_identity()
        deadline = int(getattr(self, '_observe_return_initial_contact_deadline_ns', 0))
        reserved = bool(
            getattr(self, '_observe_return_prefix', None) is not None
            or getattr(self, '_observe_return_origin_prefix', None) is not None
            or getattr(self, '_observe_return_session', None) is not None
            or getattr(self, '_observe_return_pending_supplement', None) is not None
            or getattr(self, '_execution_authority_ticket', None) is not None
            or getattr(getattr(self, 'execution_plan_controller', None), 'has_execution', False)
            or getattr(self, 'robot_execution_active', False)
            or getattr(getattr(self, 'latest_preview_rich_plan', None), 'valid', False)
            or getattr(getattr(self, 'latest_rich_plan', None), 'valid', False))
        changed = super()._advance_target_instance_epoch(reason, preserve_identity=preserve_identity)
        if (not changed or preserve_identity or not selected or not active
                or phase == 'initial_contact'):
            return changed
        with self._stream_condition, self._geometry_state_guard():
            identity = self._current_stream_target_identity()
            if identity == old_identity:
                return changed
            now = int(self._ros_source_clock_ns())
            rebuild = phase == 'observation_prefix' and not reserved and deadline > 0
            # No old measured surface, source, gap, silhouette, plan or gain is
            # transferred. The base transition already cancelled old tickets.
            self._reset_observe_return()
            self._observe_return_initial_contact_deadline_ns = deadline
            self._observe_return_identity_transition = dict(
                reason=str(reason), stamp_ns=now,
                previous_target_epoch=old_identity.epoch,
                previous_target_track_id=old_identity.track_id,
                target_epoch=identity.epoch, target_track_id=identity.track_id,
                original_contact_deadline_ns=deadline,
                action='rebuild_initial_contact' if rebuild else 'terminate_observation',
                old_evidence_reused=False, contact_or_motion_authorized=False)
            self._near_field_phase_id += 1
            self._direct_near_field_submission_generation = None
            self._direct_snapshot_terminal_generation = None
            self._direct_snapshot_last_rejection = None
            self.last_submitted_stamp_ns = max(self.last_submitted_stamp_ns, now)
            self._near_field_request_plan_cache = {}
            self._candidate_plan_metrics = {}
            self.latest_preview_rich_plan = None
            self._latest_preview_proposal = None
            self._latest_preview_gate_audit_report = None
            self._reset_multiview_surface()
            self.near_field_planning_active = rebuild
            self._near_field_phase_started_sec = now / 1e9 if rebuild else 0.
            self._near_field_phase_deadline_sec = deadline / 1e9 if rebuild else 0.
            if not rebuild:
                self._observe_return_phase = 'identity_lost'
                self._observe_return_paused = True
                self._observe_return_identity_terminal = dict(
                    generation=self._stream_generation, target_epoch=identity.epoch,
                    selection=deepcopy(self._mode_selection), published=False)
            self._stream_condition.notify_all()
        return changed

    def _observe_return_publish_identity_terminal(self):
        """Publish an invalid preview outside input callbacks; never move."""
        pending = getattr(self, '_observe_return_identity_terminal', None)
        if pending is None:
            return False
        with self._mode_work_lock:
            with self._stream_condition:
                if (pending is not self._observe_return_identity_terminal
                        or pending['selection'] != self._mode_selection
                        or not self._selection_is_current()
                        or pending['generation'] != self._stream_generation
                        or pending['target_epoch'] != self.target_instance_epoch):
                    return False
                if pending['published']:
                    return True
                pending['published'] = True
            original = self._observe_return_original
            terminal = original.Grasp6DPlan()
            terminal.header.frame_id = 'base_link'
            terminal.header.stamp = self._safe_time()
            terminal.valid = False
            terminal.candidate_source = 'near_field_terminal'
            terminal.diagnostic = 'OBSERVE_RETURN_TARGET_CHANGED: previous visual evidence retired'
            legacy = original.PoseArray()
            legacy.header = deepcopy(terminal.header)
            self._publish_preview_plan(terminal, legacy)
            self.status_pub.publish(String(terminal.diagnostic))
        return True

    def _observe_return_selected(self):
        return (self._mode_selection or {}).get('strategy') == 'observe_return'

    def _setup_observe_return_io(self):
        self._observe_return_ack_pub = rospy.Publisher(ACK_TOPIC, String, queue_size=8, latch=True)
        self._observe_return_session_sub = rospy.Subscriber(
            SESSION_TOPIC, String, self._observe_return_session_cb, queue_size=8)

    def _observe_return_save_surface(self):
        surface, reference = self._active_multiview_surface()
        if surface is not None:
            self._observe_return_saved = (surface, reference)
        saved = self._observe_return_saved
        if saved is None or saved[0].identity != self._current_stream_target_identity():
            raise ValueError('OBSERVE_RETURN_MEASURED_SURFACE_BINDING_LOST')
        return saved

    def _observe_return_advance(self, phase, start_ns, deadline_ns, *, active, paused):
        """Cancel the old generation without changing target identity or points."""
        # Saving and restoring a measured surface is a single identity
        # transaction. Input callbacks retire identities under this same lock.
        with self._stream_condition, self._geometry_state_guard():
            identity = self._current_stream_target_identity()
            saved = self._observe_return_save_surface()
            if (identity != self._current_stream_target_identity()
                    or saved[0].identity != identity):
                raise ValueError('OBSERVE_RETURN_TARGET_CHANGED')
            if (self._observe_return_phase == 'initial_contact'
                    and not self._observe_return_initial_contact_deadline_ns):
                self._observe_return_initial_contact_deadline_ns = int(round(
                    self._near_field_phase_deadline_sec * 1e9))
            self._observe_return_paused = True
            self._advance_target_instance_epoch('OBSERVE_RETURN_PHASE_CHANGED', preserve_identity=True)
            if identity != self._current_stream_target_identity():
                raise ValueError('OBSERVE_RETURN_TARGET_CHANGED')
            self._observe_return_phase = phase
            self.near_field_planning_active = active
            self._near_field_phase_id += 1
            self._near_field_phase_started_sec = start_ns / 1e9 if active else 0.
            self._near_field_phase_deadline_sec = deadline_ns / 1e9 if active else 0.
            self._direct_near_field_submission_generation = None
            self._direct_snapshot_terminal_generation = None
            self._direct_snapshot_last_rejection = None
            self.last_submitted_stamp_ns = max(self.last_submitted_stamp_ns, start_ns)
            self._near_field_request_plan_cache = {}
            self._candidate_plan_metrics = {}
            self.latest_preview_rich_plan = None
            self._latest_preview_proposal = None
            self._latest_preview_gate_audit_report = None
            self._near_field_fused_surface, self._near_field_reference_view = saved
            self._near_field_surface_phase_id = self._near_field_phase_id
            self._near_field_surface_generation = self._stream_generation
            self._latest_registration_evidence = None
            self._observe_return_saved = saved
            self._observe_return_paused = paused
            self._stream_condition.notify_all()

    def _observe_return_precheck_visual_failure(self, prepared, code):
        """Require this request's early failure before any motion comparison.

        Local rejection of random GraspNet proposals cannot disprove the
        tabletop source's measured bilateral deficit. IK or simulation errors
        alone are not view triggers; independent mixed-case evidence has a
        separate request-bound admission below.
        """
        if (not self._observe_return_selected() or self._observe_return_phase != 'initial_contact'
                or code != 'NEAR_FIELD_NO_HARD_SAFE_CANDIDATE'
                or getattr(prepared, 'remote_failure_code', '')):
            return False
        report = getattr(self, '_active_gate_audit_report', None)
        try:
            ticket = prepared.ticket
            if (not isinstance(report, dict)
                    or report['request_id'] != ticket.request_id
                    or report['generation'] != ticket.generation
                    or report['target_epoch'] != ticket.target_epoch
                    or report['snapshot_stamp_ns'] != prepared.snapshot.stamp_ns):
                return False
            funnel = report['pipeline_funnel']
            return bool(report['moveit_selection']['checked_count'] == 0
                and funnel['stage_counts']['locally_valid']['passed'] == 0)
        except (AttributeError, KeyError, TypeError, ValueError):
            return False

    def _observe_return_contact_failure_phase(self):
        session = self._observe_return_session or {}
        return (self._observe_return_phase == 'initial_contact' or
            (self._observe_return_phase == 'observing_contact'
             and session.get('version') == 2 and session.get('phase') == 'observing'))

    def _observe_return_accept_view_plan(self, plan):
        """Keep the original return authority separate from each new view."""
        if (self._observe_return_phase != 'observation_prefix' or not plan.valid
                or plan.diagnostic != 'FAR_FIELD_OBSERVATION_PLAN'):
            return
        self._observe_return_prefix = deepcopy(plan)
        if self._observe_return_session is None:
            self._observe_return_origin_prefix = deepcopy(plan)

    def _observe_return_completed_contact_failure(self, prepared, code):
        """Bind a completed initial search; this proof grants no motion."""
        if (not self._observe_return_selected() or not self._observe_return_contact_failure_phase()
                or code not in ('NEAR_FIELD_NO_REACHABLE_CANDIDATE',
                                'NEAR_FIELD_NO_HARD_SAFE_CANDIDATE')
                or getattr(prepared, 'remote_failure_code', '')):
            return None
        report = getattr(self, '_active_gate_audit_report', None)
        try:
            ticket = prepared.ticket
            if (not isinstance(report, dict)
                    or report['grasp_mode_selection'] != self._mode_selection
                    or report['request_id'] != ticket.request_id
                    or report['generation'] != ticket.generation
                    or report['target_epoch'] != ticket.target_epoch
                    or report['snapshot_stamp_ns'] != prepared.snapshot.stamp_ns
                    or report.get('selected') is not None):
                return None
            search = report['moveit_selection']
            names = ('checked_count', 'shortlist_count', 'reachable_count')
            if (search['terminated_early'] is not False
                    or any(type(search[name]) is not int or search[name] < 0 for name in names)
                    or search['reachable_count'] != 0
                    or search['checked_count'] != search['shortlist_count']):
                return None
            local = report['pipeline_funnel']['stage_counts']['locally_valid']
            if (any(type(local[name]) is not int or local[name] < 0
                    for name in ('entered', 'passed', 'rejected'))
                    or local['entered'] != local['passed'] + local['rejected']):
                return None
            if code == 'NEAR_FIELD_NO_HARD_SAFE_CANDIDATE':
                if search['checked_count'] != 0 or local['passed'] != 0:
                    return None
            elif search['checked_count'] <= 0:
                return None
            return dict(request_id=ticket.request_id, generation=ticket.generation,
                target_epoch=ticket.target_epoch, snapshot_stamp_ns=prepared.snapshot.stamp_ns,
                contact_search_code=code, selected=False, terminated_early=False,
                checked_count=search['checked_count'], shortlist_count=search['shortlist_count'],
                reachable_count=0, locally_valid=deepcopy(local),
                contact_or_motion_authorized=False)
        except (AttributeError, KeyError, TypeError, ValueError):
            return None

    def _observe_return_silhouette_failure(self, prepared, code):
        """An ownership ambiguity permits diagnosis only, never a smaller body."""
        if self._observe_return_diagnostic_attempted or self._observe_return_phase != 'initial_contact':
            return None
        proof = self._observe_return_completed_contact_failure(prepared, code)
        if proof is None:
            return None
        try:
            identity = self._current_stream_target_identity()
            evidence = validate_silhouette_ambiguity(
                self._observe_return_original._thaw_request_evidence(
                    prepared.remote_diagnostics['silhouette_ambiguity']),
                selection=self._mode_selection, request_id=prepared.ticket.request_id,
                generation=prepared.ticket.generation, target_epoch=identity.epoch,
                snapshot_stamp_ns=prepared.snapshot.stamp_ns, target_track_id=identity.track_id)
            if (prepared.ticket.target_epoch != identity.epoch
                    or prepared.snapshot.target_identity != identity):
                return None
            return dict(policy='current_request_silhouette_initialization_ambiguity',
                initial_contact_search=proof, silhouette_ambiguity=evidence,
                diagnostic_observation_only=True, contact_or_motion_authorized=False)
        except (AttributeError, KeyError, TypeError, ValueError, OverflowError):
            return None

    def _observe_return_silhouette_fallback(self, prepared, opportunity):
        """Freeze one verified ambiguity; retain every original occupied point."""
        current = self._observe_return_silhouette_failure(
            prepared, opportunity.get('initial_contact_search', {}).get('contact_search_code'))
        if current is None or current != opportunity:
            return False
        with self._stream_condition, self._geometry_state_guard():
            self._require_stream_ticket_current_locked(prepared.ticket)
            surface, _ = self._observe_return_save_surface()
            if (prepared.snapshot.target_identity != surface.identity
                    or prepared.snapshot.stamp_ns not in surface.view_stamps_ns):
                return False
            evidence = current['silhouette_ambiguity']
            if not isinstance(prepared.geometry, self._observe_return_original.GeometryEstimate) or not prepared.geometry.ok:
                return False
            self._observe_return_diagnostic_collision_geometry = deepcopy(prepared.geometry)
            self._observe_return_diagnostic_collision_source = dict(
                selection=deepcopy(self._mode_selection), request_id=prepared.ticket.request_id,
                generation=prepared.ticket.generation, target_epoch=surface.identity.epoch,
                target_track_id=surface.identity.track_id, source_stamp_ns=prepared.snapshot.stamp_ns,
                perception_status=deepcopy(evidence.get('perception_source_status')))
            self._observe_return_diagnostic_attempted = True
            self._observe_return_diagnostic_observation_only = True
            self._observe_return_silhouette_evidence = deepcopy(evidence)
            self._observe_return_trigger_evidence = deepcopy(current)
            self._observe_return_trigger = 'RGB_DEPTH_SILHOUETTE_AMBIGUOUS'
            self._observe_return_view_gaps = ()
            self._observe_return_view_uncertainty_m = float(
                evidence['view_geometry']['depth_uncertainty_m'])
            self._observe_return_initial_stamp = prepared.snapshot.stamp_ns
            self._observe_return_advance('observation_prefix',
                int(self._ros_source_clock_ns()), 0, active=False, paused=False)
        self.status_pub.publish(String(data='OBSERVE_RETURN_VIEW_REQUIRED '
            'RGB_DEPTH_SILHOUETTE_AMBIGUOUS DIAGNOSTIC_OBSERVATION_ONLY'))
        return True

    def _observe_return_mixed_visual_failure(self, prepared, code):
        """Other rejected paths cannot erase an independently missing side.

        Prefer any successful initial contact. Only a completed current
        request with no selection can expose a separately recorded,
        aperture-fitting missing-view opportunity. No failed path is reused.
        """
        if (not self._observe_return_selected() or not self._observe_return_contact_failure_phase()
                or code not in ('NEAR_FIELD_NO_REACHABLE_CANDIDATE',
                                'NEAR_FIELD_NO_HARD_SAFE_CANDIDATE')
                or getattr(prepared, 'remote_failure_code', '')):
            return None
        report = getattr(self, '_active_gate_audit_report', None)
        try:
            ticket = prepared.ticket
            if (not isinstance(report, dict)
                    or report['request_id'] != ticket.request_id
                    or report['generation'] != ticket.generation
                    or report['target_epoch'] != ticket.target_epoch
                    or report['snapshot_stamp_ns'] != prepared.snapshot.stamp_ns
                    or report.get('selected') is not None):
                return None
            search = report['moveit_selection']
            if (search['terminated_early'] is not False
                    or type(search['reachable_count']) is not int
                    or search['reachable_count'] != 0
                    or type(search['checked_count']) is not int
                    or type(search['shortlist_count']) is not int
                    or search['checked_count'] != search['shortlist_count']):
                return None
            if code == 'NEAR_FIELD_NO_HARD_SAFE_CANDIDATE':
                local = report['pipeline_funnel']['stage_counts']['locally_valid']
                if (search['checked_count'] != 0
                        or type(local['passed']) is not int or local['passed'] != 0
                        or type(local['entered']) is not int or local['entered'] < 0
                        or type(local['rejected']) is not int
                        or local['rejected'] != local['entered']):
                    return None
            elif search['checked_count'] <= 0:
                return None
            gaps = bounded_bilateral_view_gaps(prepared.remote_diagnostics,
                prepared.geometry, self.tabletop_geometry_config)
            if not gaps:
                return None
            return dict(policy='current_request_independent_bilateral_view_gap',
                request_id=ticket.request_id, generation=ticket.generation,
                target_epoch=ticket.target_epoch, snapshot_stamp_ns=prepared.snapshot.stamp_ns,
                contact_search_code=code, checked_count=search['checked_count'],
                missing_surface_direction=deepcopy(gaps[0]),
                contact_or_motion_authorized=False)
        except (AttributeError, KeyError, TypeError, ValueError):
            return None

    def _observe_return_fallback(self, prepared, *, visual_opportunity=None):
        if (not self._observe_return_selected()
                or self._observe_return_phase != 'initial_contact'):
            return False
        trigger = structured_view_deficit(prepared.remote_diagnostics)
        if visual_opportunity is not None:
            current = self._observe_return_mixed_visual_failure(
                prepared, visual_opportunity.get('contact_search_code'))
            if current is None or current != visual_opportunity:
                return False
            trigger = 'BILATERAL_SURFACE_EVIDENCE_MISSING'
        if not trigger:
            return False
        # Freeze the initial measured deficit before advancing/cancelling its
        # ticket. In production both identities and the exact source frame
        # must still match; callers without prepared inputs can only enter an
        # inert prefix with no admissible observation view.
        with self._stream_condition, self._geometry_state_guard():
            ticket = getattr(prepared, 'ticket', None)
            if ticket is not None:
                self._require_stream_ticket_current_locked(ticket)
            surface, _ = self._observe_return_save_surface()
            snapshot = getattr(prepared, 'snapshot', None)
            if snapshot is not None and (
                    getattr(snapshot, 'target_identity', None) != surface.identity
                    or snapshot.stamp_ns not in surface.view_stamps_ns):
                return False
            try:
                gaps = measured_bilateral_view_gaps(prepared.remote_diagnostics,
                    prepared.geometry, self.tabletop_geometry_config, surface,
                    self.gripper_geometry, prepared.pose_estimator.T_base_camera_link[:3, 3])
            except (AttributeError, KeyError, TypeError, ValueError):
                gaps = ()
            self._observe_return_view_gaps = gaps
            if gaps and snapshot is not None:
                # Remember the manually aligned source view too; returning to
                # that ray after a later view is not a new observation.
                self._observe_return_visited_camera_origins.append(
                    deepcopy(gaps[0]['initial_camera_origin_base_m']))
            try:
                profiles = prepared.remote_diagnostics['tabletop_geometry']['adaptive_stage_profiles']
                uncertainties = tuple(float(row['depth_uncertainty_m']) for row in profiles)
                self._observe_return_view_uncertainty_m = (max(uncertainties) if uncertainties
                    and all(math.isfinite(value) and value >= 0. for value in uncertainties) else None)
            except (AttributeError, KeyError, TypeError, ValueError):
                self._observe_return_view_uncertainty_m = None
            self._observe_return_initial_stamp = min(surface.view_stamps_ns)
            self._observe_return_trigger = trigger
            self._observe_return_trigger_evidence = deepcopy(visual_opportunity)
            now = int(self._ros_source_clock_ns())
            self._observe_return_advance('observation_prefix', now, 0, active=False, paused=False)
        self.status_pub.publish(String(data='OBSERVE_RETURN_VIEW_REQUIRED ' + trigger))
        return True

    def _observe_return_prepare_supplement(self, prepared, code):
        """Prepare another view only for a completed search plus measured deficit.

        Failed paths, stale frames, registration errors and controller failures
        alone never call for a new motion. The task still owns all execution.
        """
        session = self._observe_return_session or {}
        if (session.get('version') != 2 or session.get('phase') != 'observing'
                or self._observe_return_phase != 'observing_contact'
                or self._observe_return_diagnostic_observation_only
                or session['view_index'] >= session['max_observations']
                or int(self._ros_source_clock_ns()) >= session['deadline_ns']):
            return False
        proof = self._observe_return_completed_contact_failure(prepared, code)
        opportunity = self._observe_return_mixed_visual_failure(prepared, code)
        gain = self._observe_return_step_gain
        if proof is None or opportunity is None or not information_gain_valid(gain):
            return False
        with self._stream_condition, self._geometry_state_guard():
            self._require_stream_ticket_current_locked(prepared.ticket)
            surface, _ = self._observe_return_save_surface()
            identity = self._current_stream_target_identity()
            evidence = getattr(self, '_latest_registration_evidence', None)
            if (prepared.snapshot.target_identity != identity or surface.identity != identity
                    or prepared.snapshot.stamp_ns not in surface.view_stamps_ns
                    or evidence is None or evidence.result.ok is not True
                    or evidence.identity != identity or evidence.stamp_ns != prepared.snapshot.stamp_ns
                    or gain['target_epoch'] != identity.epoch or gain['target_track_id'] != identity.track_id
                    or gain['source_stamp_ns'] not in surface.view_stamps_ns):
                return False
            gaps = measured_bilateral_view_gaps(prepared.remote_diagnostics,
                prepared.geometry, self.tabletop_geometry_config, surface,
                self.gripper_geometry, prepared.pose_estimator.T_base_camera_link[:3, 3])
            if not gaps:
                return False
            try:
                uncertainties = [float(row['depth_uncertainty_m']) for row in
                    prepared.remote_diagnostics['tabletop_geometry']['adaptive_stage_profiles']]
                if not uncertainties or not all(math.isfinite(v) and v >= 0. for v in uncertainties):
                    return False
                uncertainty = max(uncertainties)
            except (AttributeError, KeyError, TypeError, ValueError, OverflowError):
                return False
            request = dict(session_id=session['session_id'],
                view_index=session['view_index'] + 1, previous_view_index=session['view_index'],
                origin_plan_id=session['origin_plan_id'],
                origin_snapshot_stamp_ns=session['origin_snapshot_stamp_ns'],
                target_epoch=identity.epoch, target_track_id=identity.track_id,
                deadline_ns=session['deadline_ns'], max_observations=session['max_observations'],
                failed_search=deepcopy(proof), visual_opportunity=deepcopy(opportunity),
                previous_gain=deepcopy(gain), previous_gain_sha256=evidence_digest(gain),
                contact_or_motion_authorized=False)
            request['view_request_id'] = evidence_digest(request)
            self._observe_return_pending_supplement = request
            self._observe_return_view_gaps = gaps
            # Keep the registered actual source ray as well as planned views.
            self._observe_return_visited_camera_origins.append(
                deepcopy(gaps[0]['initial_camera_origin_base_m']))
            self._observe_return_trigger = 'BILATERAL_SURFACE_EVIDENCE_MISSING'
            self._observe_return_trigger_evidence = deepcopy(opportunity)
            # The new request is tied to the latest real view, not the first
            # request's noise/geometry; predictions cannot renew that evidence.
            self._observe_return_view_uncertainty_m = uncertainty
            self._observe_return_advance('observation_prefix',
                int(self._ros_source_clock_ns()), 0, active=False, paused=False)
        self.status_pub.publish(String(data='OBSERVE_RETURN_SUPPLEMENTAL_VIEW_REQUIRED '
            + request['view_request_id']))
        return True

    def _observe_return_view_gain(self, sequence):
        """Screen an existing observation only; never generate robot poses."""
        try:
            identity = self._current_stream_target_identity()
            camera = self._observe_return_original.pose_matrix(sequence.pregrasp).dot(
                self._tool_from_camera_matrix())
            uncertainty = sequence.adaptive_stage_profile.depth_uncertainty_m
            if self._observe_return_diagnostic_observation_only:
                evidence = self._observe_return_silhouette_evidence
                if (evidence['target_epoch'] != identity.epoch
                        or evidence['target_track_id'] != identity.track_id):
                    raise ValueError('silhouette evidence target changed')
                return silhouette_view_gain(evidence, camera, uncertainty)
            gaps = tuple(gap for gap in self._observe_return_view_gaps
                if gap['target_epoch'] == identity.epoch and gap['target_track_id'] == identity.track_id)
            result = missing_side_view_gain(gaps, camera[:3, 3], uncertainty)
            if result['ok'] and gaps and not distinct_view(camera[:3, 3],
                    gaps[0]['target_center_base_m'], self._observe_return_visited_camera_origins,
                    float(uncertainty)):
                return dict(result, ok=False, code='OBSERVE_RETURN_VIEW_ALREADY_ATTEMPTED')
            return result
        except (AttributeError, KeyError, TypeError, ValueError):
            return dict(ok=False, code='OBSERVE_RETURN_VIEW_GAIN_INPUT_INVALID',
                contact_or_motion_authorized=False, measured_registration_required=True)

    def _observe_return_ready_contact(self, returning):
        if self._observe_return_diagnostic_observation_only:
            return False
        plan = getattr(self, 'latest_preview_rich_plan', None)
        session = self._observe_return_session
        surface, _ = self._observe_return_save_surface()
        evidence = getattr(self, '_latest_registration_evidence', None)
        return bool(session and plan is not None and plan.valid
            and plan.diagnostic == 'CONTACT_EXECUTION_PLAN'
            and plan.target_track_id == session['target_track_id']
            and int(plan.fused_view_count) >= 2
            and len(surface.view_stamps_ns) >= 2
            and surface.identity.epoch == session['target_epoch']
            and surface.identity.track_id == session['target_track_id']
            and self._observe_return_initial_stamp in surface.view_stamps_ns
            and any(session['origin_snapshot_stamp_ns'] < stamp < returning['stamp_ns']
                    for stamp in surface.view_stamps_ns)
            and evidence is not None and evidence.result.ok
            and evidence.identity == surface.identity
            and evidence.stamp_ns == plan.header.stamp.to_nsec()
            and evidence.stamp_ns in surface.view_stamps_ns)

    def _observe_return_session_cb(self, msg):
        raw = {}
        session = None
        ack = dict(accepted=False, reason='OBSERVE_RETURN_SESSION_INVALID')
        try:
            raw = json.loads(msg.data)
            for key in ('session_id', 'phase', 'stamp_ns'):
                ack[key] = raw.get(key)
            view_camera_origin = None
            with self._mode_reset_lock:
                with self._mode_control_lock, self._stream_condition, self._geometry_state_guard():
                    if not self._observe_return_selected() or not self._selection_is_current():
                        raise ValueError('OBSERVE_RETURN_SELECTION_CHANGED')
                    session = parse_session(raw, self._mode_selection,
                        int(self._ros_source_clock_ns()), self._observe_return_session)
                    identity = self._current_stream_target_identity()
                    if (session['target_epoch'] != identity.epoch
                            or session['target_track_id'] != identity.track_id):
                        raise ValueError('OBSERVE_RETURN_TARGET_CHANGED')
                    prefix = self._observe_return_origin_prefix or self._observe_return_prefix
                    if (prefix is None or prefix.plan_id != session['origin_plan_id']
                            or prefix.header.stamp.to_nsec() != session['origin_snapshot_stamp_ns']):
                        raise ValueError('OBSERVE_RETURN_ORIGIN_PREFIX_MISMATCH')
                    self._observe_return_origin_prefix = deepcopy(prefix)
                    if session['version'] == 2 and session['phase'] == 'observing':
                        view = self._observe_return_prefix
                        if (view is None or view.plan_id != session['view_plan_id']
                                or view.header.stamp.to_nsec() != session['view_source_stamp_ns']
                                or view.target_track_id != session['target_track_id']):
                            raise ValueError('OBSERVE_RETURN_VIEW_PLAN_MISMATCH')
                        if session['view_index'] > 1:
                            pending = self._observe_return_pending_supplement
                            if (not isinstance(pending, dict) or any(pending[key] != session[key]
                                    for key in ('session_id', 'view_index', 'view_request_id',
                                        'previous_gain_sha256', 'deadline_ns', 'max_observations',
                                        'origin_plan_id', 'origin_snapshot_stamp_ns', 'target_epoch', 'target_track_id'))):
                                raise ValueError('OBSERVE_RETURN_SUPPLEMENTAL_REQUEST_MISMATCH')
                        camera = self._observe_return_original.pose_matrix(
                            self._observe_return_original.PoseStamped(
                                header=deepcopy(view.header), pose=deepcopy(view.poses[0]))).dot(
                                    self._tool_from_camera_matrix())
                        view_camera_origin = camera[:3, 3].tolist()
                        if not all(math.isfinite(value) for value in view_camera_origin):
                            raise ValueError('OBSERVE_RETURN_VIEW_CAMERA_INVALID')
                    if self._observe_return_diagnostic_observation_only and session['version'] != 1:
                        raise ValueError('DIAGNOSTIC_OBSERVATION_REQUIRES_SINGLE_VIEW_SESSION')
                    # Mark paused first, then release the input lock before
                    # draining an in-flight worker (which rechecks tickets).
                    self._observe_return_paused = True
                with self._mode_work_lock, self._mode_phase_lock, self._stream_condition, self._geometry_state_guard():
                    if not self._selection_is_current():
                        raise ValueError('OBSERVE_RETURN_SELECTION_CHANGED')
                    identity = self._current_stream_target_identity()
                    if identity.epoch != session['target_epoch'] or identity.track_id != session['target_track_id']:
                        raise ValueError('OBSERVE_RETURN_TARGET_CHANGED')
                    phase = session['phase']
                    if self._observe_return_diagnostic_observation_only and phase != 'observing':
                        raise ValueError('DIAGNOSTIC_OBSERVATION_FORBIDS_CONTACT_AND_RETURN')
                    if phase == 'observing':
                        if self._observe_return_phase != 'observation_prefix':
                            raise ValueError('OBSERVE_RETURN_PREFIX_NOT_ACTIVE')
                        next_phase, active, paused = 'observing', False, True
                    elif phase == 'returning':
                        if self._observe_return_phase != 'observing_contact' or not self._observe_return_ready_contact(session):
                            raise ValueError('OBSERVE_RETURN_CONTACT_EVIDENCE_NOT_READY')
                        next_phase, active, paused = 'returning', False, True
                    else:
                        if self._observe_return_phase != 'returning':
                            raise ValueError('OBSERVE_RETURN_RETURN_NOT_PENDING')
                        budget = float(getattr(self, 'near_field_direct_timeout_sec', 60.))
                        if (not math.isfinite(budget) or budget <= 0.
                                or session['contact_deadline_ns'] - session['stamp_ns'] > budget * 1e9 + 1000):
                            raise ValueError('OBSERVE_RETURN_CONTACT_BUDGET_INVALID')
                        next_phase, active, paused = 'returned_contact', True, False
                    self._observe_return_advance(next_phase,
                        session['stamp_ns'],
                        session['contact_deadline_ns'], active=active, paused=paused)
                    if identity != self._current_stream_target_identity():
                        raise ValueError('OBSERVE_RETURN_TARGET_CHANGED')
                    self._observe_return_session = session
                    if session['phase'] == 'observing' and session['version'] == 2:
                        self._observe_return_step_gain = None
                        self._observe_return_pending_supplement = None
                        self._observe_return_visited_camera_origins.append(view_camera_origin)
                    ack.update(deepcopy(session))
                    ack.update(accepted=True, reason='OK')
        except Exception as exc:
            ack['reason'] = str(exc)
            rospy.logwarn('Observe-return session rejected: %s', exc)
        self._observe_return_publish_session_ack(ack, session)

    def _observe_return_publish_session_ack(self, ack, session):
        # The input callback can retire the identity after the worker lock is
        # released. Never acknowledge the superseded session as actionable.
        with self._stream_condition:
            if ack.get('accepted'):
                identity = self._current_stream_target_identity()
                if not self._selection_is_current() or ack['selection'] != self._mode_selection:
                    ack.update(accepted=False, reason='OBSERVE_RETURN_SELECTION_CHANGED')
                elif (session is not self._observe_return_session
                        or ack['target_epoch'] != identity.epoch
                        or ack['target_track_id'] != identity.track_id):
                    ack.update(accepted=False, reason='OBSERVE_RETURN_TARGET_CHANGED')
            self._observe_return_ack_pub.publish(String(data=json.dumps(ack, sort_keys=True)))

    def _observe_return_near_field_cb(self, msg):
        """Keep the initial surface; reached views append real registration."""
        session = self._observe_return_session
        if self._observe_return_diagnostic_observation_only:
            # The task owns bounded stationary capture and completion. This
            # branch never unlocks contact planning or appends measured views.
            return
        if session is None or not getattr(msg, 'active', False):
            return
        # Returned is already activated by the acknowledged session message;
        # the separately delivered task phase must never erase or reseed it.
        if self._observe_return_phase == 'returned_contact':
            return
        if self._observe_return_phase not in ('observing', 'observing_contact'):
            return
        original = self._observe_return_original
        start = original._stamp_to_nsec(msg.header.stamp)
        deadline = original._stamp_to_nsec(msg.deadline)
        now = int(self._ros_source_clock_ns())
        budget = float(getattr(self, 'near_field_direct_timeout_sec', 60.))
        if (not self._selection_is_current() or getattr(msg.header, 'frame_id', '') != 'base_link'
                or not math.isfinite(budget) or budget <= 0.
                or not session['stamp_ns'] < start <= now < deadline
                or deadline > session['deadline_ns'] or deadline-start > budget * 1e9 + 1000
                or int(getattr(msg, 'phase_id', 0)) <= 0):
            return
        if getattr(self, '_observe_return_reached_phase_id', None) == msg.phase_id:
            return
        observation = self._resolve_reached_view_observation(msg)
        if (observation is None or observation.identity != self._current_stream_target_identity()
                or observation.stamp_ns <= session['origin_snapshot_stamp_ns']):
            return
        # The task reached reference binds a source frame but is not itself
        # registration evidence. The unchanged registration gate still runs.
        self._observe_return_paused = True
        with self._mode_work_lock, self._mode_phase_lock, self._stream_condition, self._geometry_state_guard():
            if (self._observe_return_session is not session or not self._selection_is_current()
                    or observation.identity != self._current_stream_target_identity()):
                return
            previous_surface, _ = self._observe_return_save_surface()
            self._observe_return_advance('observing_contact', start, deadline, active=True, paused=True)
            self._latest_target_observation = observation
            result = original.RemoteGrasp6DNode._register_near_field_surface(self, observation)
            if result is None or not result.ok:
                code = getattr(result, 'code', 'NO_SURFACE')
                rospy.logwarn('Observe-return reached view registration rejected: %s', code)
                terminal = original.Grasp6DPlan()
                terminal.header.stamp = self._safe_time()
                terminal.header.frame_id = 'base_link'
                terminal.valid = False
                terminal.candidate_source = 'near_field_terminal'
                terminal.diagnostic = 'OBSERVE_RETURN_REGISTRATION_FAILED: ' + str(code)
                legacy = original.PoseArray()
                legacy.header = deepcopy(terminal.header)
                self._publish_preview_plan(terminal, legacy)
                return
            self._observe_return_saved = self._active_multiview_surface()
            if session.get('version') == 2:
                try:
                    gain = registered_information_gain(previous_surface,
                        self._observe_return_saved[0], result,
                        voxel_size_m=float(getattr(self, 'geometry_voxel_size_m', .0025)),
                        depth_uncertainty_m=self._observe_return_view_uncertainty_m)
                    self._observe_return_step_gain = gain
                except (TypeError, ValueError, AttributeError) as exc:
                    terminal = original.Grasp6DPlan()
                    terminal.header.stamp = self._safe_time()
                    terminal.header.frame_id = 'base_link'
                    terminal.valid = False
                    terminal.candidate_source = 'near_field_terminal'
                    terminal.diagnostic = 'OBSERVE_RETURN_INFORMATION_GAIN_FAILED: ' + str(exc)
                    legacy = original.PoseArray(); legacy.header = deepcopy(terminal.header)
                    self._publish_preview_plan(terminal, legacy)
                    return
            self._observe_return_reached_phase_id = msg.phase_id
            self._observe_return_paused = False

    def _observe_return_audit(self, report, prepared):
        if not self._observe_return_selected():
            return
        phase = self._observe_return_phase
        snapshot = int(prepared.snapshot.stamp_ns)
        identity = self._current_stream_target_identity()
        metadata = dict(phase='initial_contact', trigger_code='', target_epoch=int(identity.epoch),
            target_track_id=identity.track_id, origin_snapshot_stamp_ns=snapshot,
            initial_surface_stamp_ns=self._observe_return_initial_stamp or snapshot)
        if self._observe_return_identity_transition is not None:
            metadata['identity_transition'] = deepcopy(self._observe_return_identity_transition)
        if self._observe_return_diagnostic_observation_only:
            metadata['diagnostic_observation_only'] = True
            metadata['maximum_diagnostic_observations'] = 1
            metadata['diagnostic_observation_attempted'] = self._observe_return_diagnostic_attempted
            metadata['contact_and_return_authorized'] = False
            envelope = self._observe_return_original._thaw_request_evidence(
                (getattr(prepared, 'remote_diagnostics', {}) or {}).get('silhouette_observation_envelope', {}))
            metadata['collision_envelope'] = envelope
        if self._observe_return_trigger_evidence is not None:
            metadata['trigger_evidence'] = deepcopy(self._observe_return_trigger_evidence)
        if self._observe_return_view_gaps:
            metadata['measured_missing_sides'] = deepcopy(list(self._observe_return_view_gaps))
        origin = 'current_view_contact_snapshot'
        performed = False
        if phase == 'observation_prefix':
            metadata.update(phase='observation_prefix', trigger_code=self._observe_return_trigger)
            origin = 'observe_return_observation_prefix'
            pending = self._observe_return_pending_supplement
            if pending is not None:
                metadata.update(deepcopy(pending))
                metadata.update(phase='supplemental_observation', version=2,
                    view_source_stamp_ns=snapshot, view_plan_id=report.get('plan_id'))
                origin = 'observe_return_supplemental_observation'
                performed = True
        elif phase in ('observing_contact', 'returned_contact'):
            session = self._observe_return_session or {}
            metadata.update({key: session.get(key) for key in ('session_id', 'origin_plan_id',
                'origin_snapshot_stamp_ns', 'return_verified_stamp_ns', 'deadline_ns', 'contact_deadline_ns')})
            metadata.update(phase=('returned_contact' if phase == 'returned_contact' else 'observation_contact'),
                            trigger_code=self._observe_return_trigger)
            surface, _ = self._active_multiview_surface()
            metadata['fused_source_stamps_ns'] = list(surface.view_stamps_ns) if surface else []
            origin = 'observed_return_contact_snapshot' if phase == 'returned_contact' else 'observation_view_contact_preview'
            performed = True
        report.update(execution_strategy='observe_return', planning_origin=origin,
                      observation_motion_performed=performed, observe_return=metadata)
