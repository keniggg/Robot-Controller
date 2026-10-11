"""Measured-surface lifecycle for observe, return and fresh contact planning.

No robot commands: the task owns motion and supplies verified phase messages.
The remote side owns cancellation barriers and measured target provenance.
"""
from copy import deepcopy
from dataclasses import replace
import json
import math
import time
from types import SimpleNamespace

import rospy
from std_msgs.msg import String

from .observe_return_session import ACK_TOPIC, SESSION_TOPIC, parse_session, structured_view_deficit, VIEW_FIELDS
from .observe_return_measurement import measurement_digest, STATION_REQUALIFICATION
from .active_observation_policy import registered_information_gain, evidence_digest, distinct_view, information_gain_valid
from .silhouette_observation import validate_silhouette_ambiguity, silhouette_view_gain
from .observe_return_visual import (bounded_bilateral_view_gaps, measured_bilateral_view_gaps,
                                    missing_side_view_gain)
from .observation_information_uncertainty import observation_information_uncertainty_m
from .contact_search_yield import CODE as CONTACT_VIEW_YIELD, contact_search_yield_proof_valid
from .selection import parse_selection


class ObserveReturnRemoteMixin:
    def _reset_observe_return(self):
        reset_measurement = getattr(self, '_reset_observation_measurement_remote', None)
        if reset_measurement is not None:
            reset_measurement()
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
        self._observe_return_reached_contact_context = None
        self._observe_return_diagnostic_observation_only = False
        self._observe_return_diagnostic_attempted = False
        self._observe_return_silhouette_evidence = None
        self._observe_return_diagnostic_collision_geometry = None
        self._observe_return_diagnostic_collision_source = None
        self._silhouette_observation_envelope_audit = None
        self._observe_return_initial_contact_deadline_ns = 0
        self._observe_return_identity_transition = None
        self._observe_return_identity_terminal = None
        self._contact_yield_search = None
        self._observe_return_registration_ticket = None
        self._observe_return_registration_pending = None

    def _observe_return_reacquire_failed_registration(self, snapshot):
        """Discard one failed current registration, retaining only station identity.

        Reached-view registration has already passed. A later frame outside
        the yaw bound cannot supply current contact geometry, but does not by
        itself prove that all grasp candidates failed. The existing worker
        cancellation/release path collects three entirely newer frames within
        the same phase; neither this method nor release renews a deadline.
        """
        with self._stream_condition, self._geometry_state_guard():
            ticket = getattr(self, '_observe_return_registration_ticket', None)
            if ticket is None or ticket.payload[0] is not snapshot:
                return False
            evidence = getattr(self, '_latest_registration_evidence', None)
            observation = getattr(self, '_latest_target_observation', None)
            identity = self._current_stream_target_identity()
            session = self._observe_return_session or {}
            if (not self._observe_return_selected()
                    or self._observe_return_phase != 'observing_contact'
                    or not self.near_field_planning_active
                    or self._configured_near_field_strategy() != 'single_snapshot_direct'
                    or int(self.near_field_planning_snapshot_frames) != 3
                    or snapshot.target_identity != identity
                    or snapshot.target_epoch != identity.epoch
                    or evidence is None or evidence.identity != identity
                    or evidence.stamp_ns != snapshot.stamp_ns
                    or evidence.result.ok is not False
                    or evidence.result.code != 'YAW_BOUND_EXCEEDED'
                    or observation is None or observation.identity != identity
                    or observation.stamp_ns != snapshot.stamp_ns
                    or session.get('phase') != 'observing'):
                return False
            # These are the existing registered station, owner, measured-joint
            # and absolute-time checks, including hard retirement on takeover.
            self._require_stream_ticket_current_locked(ticket)
            reference = self._station_compute_reference
            now = int(self._ros_source_clock_ns())
            surface, _ = self._active_multiview_surface()
            if (reference is None or self._station_compute_retired
                    or not self._station_compute_scope()
                    or self._station_compute_work_published
                    or self._station_compute_publication_started
                    or bool(getattr(self.latest_preview_rich_plan, 'valid', False))
                    or surface is None or surface.identity != identity
                    or len(surface.view_stamps_ns) < 2
                    or snapshot.stamp_ns in surface.view_stamps_ns
                    or not self._near_field_phase_started_sec <= snapshot.stamp_ns / 1e9
                    or not self._near_field_phase_started_sec <= now / 1e9
                        < self._near_field_phase_deadline_sec
                    or now >= session['deadline_ns']):
                return False
            # Invalid public geometry/preview only. Historical surface and
            # rejected evidence remain intact; neither is execution authority.
            pending = dict(source_stamp_ns=snapshot.stamp_ns,
                request_id=ticket.request_id, generation=ticket.generation,
                phase_id=self._near_field_phase_id,
                phase_deadline_ns=round(self._near_field_phase_deadline_sec * 1e9),
                session_deadline_ns=session['deadline_ns'], code=evidence.result.code,
                rotation_deg=self._registration_rotation_deg(evidence.result.transform_base),
                overlap_fraction=evidence.result.overlap_fraction, rmse_m=evidence.result.rmse_m,
                inlier_count=evidence.result.inlier_count)
            self._station_compute_quarantine(clock_only=True)
            self._observe_return_registration_pending = pending
            rospy.loginfo('STATION_REGISTRATION_REACQUIRE: %s',
                json.dumps(pending, sort_keys=True, separators=(',', ':')))
            raise self._observe_return_original.StreamResultCancelled('TARGET_PROCESSING_PENDING')

    def _contact_yield_measured_opportunity(self, prepared):
        """Current measured deficit only; neither collision nor timeout is a view."""
        try:
            session = self._observe_return_session or {}
            gain = self._observe_return_step_gain
            if (not self._observe_return_selected()
                    or self._observe_return_phase != 'observing_contact'
                    or session.get('version') != 2 or session.get('phase') != 'observing'
                    or self._observe_return_diagnostic_observation_only
                    or session['view_index'] >= session['max_observations']
                    or int(self._ros_source_clock_ns()) >= session['deadline_ns']
                    or not self._direct_near_field_active(prepared)
                    or not self._direct_near_field_deadline_gate(prepared)()
                    or getattr(prepared, 'remote_failure_code', '')
                    or not information_gain_valid(gain)):
                return None
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
                    return None
                current, _ = self._snapshot_multiview_surface()
                if (current is None or current.identity != identity
                        or current.view_stamps_ns != surface.view_stamps_ns):
                    return None
                gaps = measured_bilateral_view_gaps(prepared.remote_diagnostics,
                    prepared.geometry, self.tabletop_geometry_config, current,
                    self.gripper_geometry, prepared.pose_estimator.T_base_camera_link[:3, 3],
                    include_thin=True)
                return gaps or None
        except (AttributeError, KeyError, TypeError, ValueError, OverflowError):
            return None

    @staticmethod
    def _contact_yield_request_key(prepared):
        t = prepared.ticket
        return (t.request_id, t.generation, t.target_epoch, prepared.snapshot.stamp_ns,
                prepared.near_field_phase_id)

    def _contact_yield_family(self, candidate):
        runtime = self._stable_variant_runtime[(candidate.track_id, candidate.variant_index)]
        sequence = self._execution_sequence_audit(runtime.get('sequence'), True)
        if not sequence.get('available'):
            raise ValueError('missing original four-stage family')
        return dict(track_id=int(candidate.track_id), variant_index=int(candidate.variant_index),
                    family_sha256=evidence_digest(sequence))

    def _contact_yield_begin(self, prepared, candidates):
        if not candidates or not self._contact_yield_measured_opportunity(prepared):
            return None
        try:
            families = [self._contact_yield_family(c) for c in candidates]
            if len({f['family_sha256'] for f in families}) < 2:
                return None
            key = self._contact_yield_request_key(prepared)
            previous = getattr(self, '_contact_yield_search', None)
            if previous is not None and previous['key'] == key:
                return previous  # Re-entering a request never renews its budget.
            now = int(self._ros_source_clock_ns())
            phase = int(self._direct_near_field_deadline_sec(prepared) * 1e9)
            deadline = min(now + 30_000_000_000, phase - 2_000_000_000)
            if not prepared.snapshot.stamp_ns <= now < deadline:
                return None
            state = dict(key=key, started_ns=now, original_phase_deadline_ns=phase,
                budget_deadline_ns=deadline, wall_deadline=time.monotonic()+(deadline-now)*1e-9,
                original_families=[], attempted_families=[], collision_receipts=[], incumbent_found=False,
                stopped=False, exhausted_by='', yielded_ns=0, deferred_height_families=[], fallback=False)
            self._contact_yield_search = state
            return state
        except (AttributeError, KeyError, TypeError, ValueError, OverflowError):
            return None

    def _direct_near_field_comparison_search(self, prepared, candidates=None):
        check, continuation = super()._direct_near_field_comparison_search(prepared, candidates)
        state = self._contact_yield_begin(prepared, candidates)
        if state is None or state["fallback"]:
            return check, continuation

        def checked(candidate):
            runtime = self._stable_variant_runtime[(candidate.track_id, candidate.variant_index)]
            runtime.pop('validated_contact_precheck_failure', None)
            runtime.pop('validated_contact_precheck_plan_id', None)
            family = self._contact_yield_family(candidate)
            if family['family_sha256'] not in {f['family_sha256'] for f in state['attempted_families']}:
                state['attempted_families'].append(family)
            result = check(candidate)
            maximum = float(getattr(self, 'candidate_max_joint_delta_rad', 0.) or 0.)
            if (all(getattr(result, k, None) is True for k in
                    ('reachable', 'collision_free', 'within_joint_limits', 'ik_valid', 'planning_success'))
                    and all(math.isfinite(getattr(result, k, float('nan')))
                            and getattr(result, k) >= 0. for k in ('joint_path_cost', 'joint_max_delta_rad'))
                    and (maximum <= 0. or result.joint_max_delta_rad <= maximum)):
                state['incumbent_found'] = True
            runtime = self._stable_variant_runtime[(candidate.track_id, candidate.variant_index)]
            receipt = runtime.get('validated_contact_precheck_failure')
            try:
                receipt_selection_current = (isinstance(receipt, dict)
                    and parse_selection(receipt.get('selection'))
                    == parse_selection(self._mode_selection))
            except (TypeError, ValueError, OverflowError):
                receipt_selection_current = False
            # Dispatch is not completion: a second RPC timeout must not count
            # as a fully rejected original family in the yielded proof.
            if (runtime.get('prepared') is prepared and isinstance(receipt, dict)
                    and receipt.get('plan_id') == runtime.get('validated_contact_precheck_plan_id')
                    and receipt.get('scope') == 'read_only_candidate_constant_bias_prediction'
                    and receipt.get('success') is False
                    and receipt.get('execution_authorized') is False
                    and receipt.get('failure_code') == 'CONTACT_SEQUENCE_PRECHECK_FAILED'
                    and receipt_selection_current
                    and family['family_sha256'] not in {f['family_sha256'] for f in state['original_families']}):
                state['original_families'].append(family)
            if (runtime.get('prepared') is prepared and isinstance(receipt, dict)
                    and receipt.get('plan_id') == runtime.get('validated_contact_precheck_plan_id')
                    and receipt.get('scope') == 'read_only_candidate_constant_bias_prediction'
                    and receipt.get('success') is False and receipt.get('execution_authorized') is False
                    and receipt.get('failed_stage') == 'grasp'
                    and receipt.get('failure_code') == 'CONTACT_SEQUENCE_PRECHECK_FAILED'
                    and str(receipt.get('reason', '')).startswith(
                        'predicted_command continuous geometry: target OBB collision at ')
                    and receipt_selection_current):
                sample = receipt.get('source_sample') or {}
                t = prepared.ticket
                bound = {k: deepcopy(receipt.get(k)) for k in ('plan_id','scope','success',
                    'execution_authorized','failed_stage','failure_code','reason')}
                bound.update(request_id=t.request_id, generation=t.generation,
                    target_epoch=t.target_epoch, snapshot_stamp_ns=prepared.snapshot.stamp_ns,
                    evidence_sha256=evidence_digest(receipt), sdk_stamp_ns=sample.get('sdk_stamp_ns'),
                    accepted_stamp_ns=sample.get('accepted_stamp_ns'))
                state['collision_receipts'].append(bound)
            return result

        def continuing():
            if not continuation():
                return False
            if state['incumbent_found']:
                return True
            now = int(self._ros_source_clock_ns())
            count_limit = len(state['attempted_families']) >= 2
            timed = now >= state['budget_deadline_ns'] or time.monotonic() >= state['wall_deadline']
            if (len(state['original_families']) == 2 and state['collision_receipts']
                    and self._contact_yield_measured_opportunity(prepared)):
                state.update(stopped=True, exhausted_by='original_family_limit', yielded_ns=now)
                return False
            # An elapsed sub-budget alone is not a measured-view certificate.
            return not (timed or count_limit)
        return checked, continuing

    def _direct_near_field_candidate_deadline_sec(self, prepared, runtime):
        deadline = super()._direct_near_field_candidate_deadline_sec(prepared, runtime)
        state = getattr(self, '_contact_yield_search', None)
        if (state is not None and state['key'] == self._contact_yield_request_key(prepared)
                and not state['incumbent_found'] and not state['fallback']):
            deadline = min(deadline, state['budget_deadline_ns']*1e-9,
                self._execution_plan_validity_now_sec()+max(0.,state['wall_deadline']-time.monotonic()))
        return deadline

    def _contact_height_window_retry(self, plan, prepared, runtime, result, deadline_sec):
        state = getattr(self, '_contact_yield_search', None)
        if (state is not None and state['key'] == self._contact_yield_request_key(prepared)
                and not state['incumbent_found'] and not state['fallback']
                and self._contact_yield_measured_opportunity(prepared)):
            # Keep the alternatives explicitly unsearched. Another original
            # orientation gets its first check before this family's two retries.
            runtime['contact_height_window_search'] = dict(
                policy='deferred_for_distinct_original_contact_families',
                original_plan_id=str(plan.plan_id), proposals=[], deferred=True,
                unchecked_is_not_rejected=True, deadline_sec=deadline_sec,
                deadline_extended=False, motion_authorized=False)
            if str(plan.plan_id) not in state['deferred_height_families']:
                state['deferred_height_families'].append(str(plan.plan_id))
            return None
        return super()._contact_height_window_retry(plan, prepared, runtime, result, deadline_sec)

    def _contact_search_yield_selection(self, prepared, selection, *, candidates=None, ranking_key=None, local_funnel=None):
        state = getattr(self, '_contact_yield_search', None)
        if state is None or state['key'] != self._contact_yield_request_key(prepared):
            return selection
        if (state['stopped'] and not state['incumbent_found']
                and selection.selected is None and not selection.reachable
                and selection.terminated_early and 0 < len(selection.checked) < selection.shortlist_count
                and self._contact_yield_measured_opportunity(prepared)):
            # Validate the same wire proof against real selection/local counts
            # before converting the stop reason. Final audit rebuilds it again.
            t = prepared.ticket
            current_report = dict(request_id=t.request_id, generation=t.generation,
                target_epoch=t.target_epoch, snapshot_stamp_ns=prepared.snapshot.stamp_ns,
                grasp_mode_selection=self._mode_selection, selected=None,
                pipeline_funnel=local_funnel,
                moveit_selection=dict(termination_reason=CONTACT_VIEW_YIELD, terminated_early=selection.terminated_early,
                    checked_count=len(selection.checked), shortlist_count=selection.shortlist_count,
                    reachable_count=len(selection.reachable)))
            if self._contact_yield_proof(prepared, current_report) is not None:
                return replace(selection, termination_reason=CONTACT_VIEW_YIELD)
        # A child scheduling budget is never the task's absolute deadline.
        # If it did not yield a valid certificate, retain every original family
        # and its deferred height alternatives under the remaining ORIGINAL
        # phase. This is a single fallback, not a renewed 30-second budget.
        if (not state['incumbent_found'] and not state['fallback']
                and candidates and self._direct_near_field_deadline_gate(prepared)()
                and (state['deferred_height_families'] or selection.terminated_early)):
            state['fallback'] = True
            state['stopped'] = False
            state['first_pass_checked'] = len(selection.checked)
            prior = {(c.track_id,c.variant_index) for c in selection.checked}
            check, gate = super()._direct_near_field_comparison_search(prepared, candidates)
            order = lambda c: ((c.track_id,c.variant_index) in prior, *ranking_key(c))
            selection = self._observe_return_original.bounded_moveit_select(
                candidates, check, top_n=max(1,len(candidates)), exhaustive=True,
                max_joint_delta_rad=float(getattr(self,'candidate_max_joint_delta_rad',0.) or 0.),
                ranking_key=order, continue_checking=gate,
                continuation_stop_reason='NEAR_FIELD_DIRECT_TIMEOUT')
            if selection.reachable:
                selection = replace(selection, selected=min(selection.reachable,
                    key=self._direct_near_field_execution_rank_key))
            if not self._direct_near_field_deadline_gate(prepared)():
                selection = replace(selection,selected=None,terminated_early=True,
                                    termination_reason='NEAR_FIELD_DIRECT_TIMEOUT')
            elif selection.selected is not None and selection.terminated_early:
                selection = replace(selection,termination_reason='NEAR_FIELD_COMPARISON_BUDGET_REACHED')
        return selection

    def _contact_yield_proof(self, prepared, report):
        state = getattr(self, '_contact_yield_search', None)
        try:
            search = report['moveit_selection']
            if (state is None or state['key'] != self._contact_yield_request_key(prepared)
                    or not state['stopped'] or state['incumbent_found']
                    or search['termination_reason'] != CONTACT_VIEW_YIELD
                    or search.get('terminated_early') is not True
                    or report['grasp_mode_selection'] != self._mode_selection
                    or report.get('selected') is not None
                    or any(report[k] != v for k,v in zip(
                        ('request_id','generation','target_epoch','snapshot_stamp_ns'),state['key'][:4]))
                    or not self._contact_yield_measured_opportunity(prepared)):
                return None
            proof = {k: report[k] for k in ('request_id','generation','target_epoch','snapshot_stamp_ns')}
            proof.update(contact_search_code=CONTACT_VIEW_YIELD, selected=False, terminated_early=True,
                checked_count=search['checked_count'], shortlist_count=search['shortlist_count'],
                reachable_count=search['reachable_count'], contact_or_motion_authorized=False,
                locally_valid=deepcopy(report['pipeline_funnel']['stage_counts']['locally_valid']))
            proof['yield_evidence'] = {k: deepcopy(state[k]) for k in ('started_ns',
                'original_phase_deadline_ns','budget_deadline_ns','yielded_ns','exhausted_by',
                'original_families','collision_receipts','incumbent_found')}
            proof['yield_evidence'].update(policy='bounded_contact_search_for_measured_view',
                phase_id=prepared.near_field_phase_id, maximum_duration_sec=30.,maximum_original_families=2)
            return proof if contact_search_yield_proof_valid(proof) else None
        except (AttributeError, KeyError, TypeError, ValueError, OverflowError):
            return None

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
            self._observe_return_reached_contact_context = None
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

    def _observe_return_measured_view_preflight(self, ticket, snapshot, geometry,
            transform, tabletop_candidates, tabletop_diagnostics):
        """A measured missing side may request observation before learned work.

        This is not a completed contact search or a contact-failure claim.
        Every proposed view still needs a new source and complete path proof;
        every later contact is planned from newly measured registered evidence.
        """
        if ((self._mode_selection or {}).get('mode') != 'unknown'
                or not self._observe_return_selected()
                or self._observe_return_phase != 'initial_contact'
                or self._observe_return_paused
                or self._observe_return_diagnostic_observation_only
                or self._observe_return_session is not None
                or not getattr(self, 'near_field_planning_active', False)
                or tabletop_candidates
                or not isinstance(tabletop_diagnostics, dict)
                or type(tabletop_diagnostics.get('materialized_count')) is not int
                or tabletop_diagnostics['materialized_count'] != 0
                or tabletop_diagnostics.get('plan_phase') != 'CONTACT_EXECUTION_PLAN'
                or tabletop_diagnostics.get('contact_execution_gate_deferred') is not False):
            return False
        # Preserve an already available contact, including an outstanding
        # authority. Observation is never an alternative execution of it.
        if (getattr(self, 'robot_execution_active', False)
                or getattr(self, '_execution_authority_ticket', None) is not None
                or any(bool(getattr(getattr(self, name, None), 'valid', False))
                    for name in ('latest_preview_rich_plan', 'latest_rich_plan'))):
            return False
        phase_budget = self._direct_near_field_deadline_gate(SimpleNamespace(near_field=True))
        if not phase_budget():
            return False
        diagnostics = dict(tabletop_geometry=tabletop_diagnostics)
        with self._stream_condition, self._geometry_state_guard():
            self._require_stream_ticket_current_locked(ticket)
            identity = self._current_stream_target_identity()
            surface, _ = self._active_multiview_surface()
            if (surface is None or surface.identity != identity
                    or snapshot.target_identity != identity
                    or ticket.target_epoch != identity.epoch
                    or snapshot.stamp_ns not in surface.view_stamps_ns
                    or not geometry.ok or not bool(snapshot.object_msg.detected)):
                return False
            gaps = measured_bilateral_view_gaps(diagnostics, geometry,
                self.tabletop_geometry_config, surface, self.gripper_geometry,
                transform[:3, 3])
            if not gaps:
                return False
            # A generator can return before constructing contact profiles.
            # Derive the unchanged live uncertainty directly from this source.
            try:
                uncertainty = observation_information_uncertainty_m(
                    self._adaptive_stage_profiles(geometry, snapshot))
            except (AttributeError, TypeError, ValueError, OverflowError):
                return False
            self._require_stream_ticket_current_locked(ticket)
            if not phase_budget():
                return False
            evidence = dict(policy='current_request_measured_view_deficit_preflight',
                request_id=ticket.request_id, generation=ticket.generation,
                target_epoch=identity.epoch, target_track_id=identity.track_id,
                snapshot_stamp_ns=snapshot.stamp_ns,
                selection=deepcopy(self._mode_selection),
                contact_search_completed=False, learned_candidates_requested=False,
                tabletop_geometry_preflight_completed=True,
                selected=False, materialized_count=0,
                missing_surface_direction=deepcopy(gaps[0]),
                measured_missing_sides=deepcopy(list(gaps)),
                source_view_stamps_ns=list(surface.view_stamps_ns),
                contact_or_motion_authorized=False)
            evidence['evidence_sha256'] = evidence_digest(evidence)
            self._observe_return_view_gaps = gaps
            self._observe_return_view_uncertainty_m = uncertainty
            self._observe_return_visited_camera_origins.append(
                deepcopy(gaps[0]['initial_camera_origin_base_m']))
            self._observe_return_initial_stamp = min(surface.view_stamps_ns)
            self._observe_return_trigger = 'BILATERAL_SURFACE_EVIDENCE_MISSING'
            self._observe_return_trigger_evidence = evidence
            self._observe_return_advance('observation_prefix',
                int(self._ros_source_clock_ns()), 0, active=False, paused=False)
        self.status_pub.publish(String(data='OBSERVE_RETURN_VIEW_REQUIRED '
            'BILATERAL_SURFACE_EVIDENCE_MISSING measured_view_deficit_preflight'))
        return True

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
        if code == CONTACT_VIEW_YIELD:
            report = getattr(self, '_active_gate_audit_report', None)
            return self._contact_yield_proof(prepared, report) if isinstance(report, dict) else None
        if (not self._observe_return_selected() or not self._observe_return_contact_failure_phase()
                or code not in ('NEAR_FIELD_NO_REACHABLE_CANDIDATE',
                                'NEAR_FIELD_NO_HARD_SAFE_CANDIDATE',
                                'NEAR_FIELD_SURFACE_VIEW_REQUIRED')
                or getattr(prepared, 'remote_failure_code', '')):
            return None
        report = getattr(self, '_active_gate_audit_report', None)
        try:
            ticket = prepared.ticket
            if (code == 'NEAR_FIELD_SURFACE_VIEW_REQUIRED'
                    and not self._near_field_surface_view_required(prepared)):
                return None
            if (not isinstance(report, dict)
                    or report['grasp_mode_selection'] != self._mode_selection
                    or report['request_id'] != ticket.request_id
                    or report['generation'] != ticket.generation
                    or report['target_epoch'] != ticket.target_epoch
                    or report['snapshot_stamp_ns'] != prepared.snapshot.stamp_ns
                    or report.get('selected') is not None):
                return None
            if (code == 'NEAR_FIELD_SURFACE_VIEW_REQUIRED'
                    and report.get('outcome', {}).get('code') != code):
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
            if code in ('NEAR_FIELD_NO_HARD_SAFE_CANDIDATE', 'NEAR_FIELD_SURFACE_VIEW_REQUIRED'):
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
        if code == CONTACT_VIEW_YIELD:
            proof = self._observe_return_completed_contact_failure(prepared, code)
            if proof is None:
                return None
            gaps = bounded_bilateral_view_gaps(prepared.remote_diagnostics,
                prepared.geometry, self.tabletop_geometry_config, include_thin=True)
            if not gaps:
                return None
            opportunity = {k: proof[k] for k in ('request_id','generation','target_epoch',
                'snapshot_stamp_ns','contact_search_code','checked_count')}
            opportunity.update(policy='current_request_independent_bilateral_view_gap',
                missing_surface_direction=deepcopy(gaps[0]), contact_or_motion_authorized=False,
                yielded_contact_search=deepcopy(proof))
            return opportunity
        if (not self._observe_return_selected() or not self._observe_return_contact_failure_phase()
                or code not in ('NEAR_FIELD_NO_REACHABLE_CANDIDATE',
                                'NEAR_FIELD_NO_HARD_SAFE_CANDIDATE',
                                'NEAR_FIELD_SURFACE_VIEW_REQUIRED')
                or getattr(prepared, 'remote_failure_code', '')):
            return None
        report = getattr(self, '_active_gate_audit_report', None)
        try:
            ticket = prepared.ticket
            surface_proof = None
            if code == 'NEAR_FIELD_SURFACE_VIEW_REQUIRED':
                surface_proof = self._observe_return_completed_contact_failure(prepared, code)
                if surface_proof is None:
                    return None
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
            if code in ('NEAR_FIELD_NO_HARD_SAFE_CANDIDATE', 'NEAR_FIELD_SURFACE_VIEW_REQUIRED'):
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
                prepared.geometry, self.tabletop_geometry_config, include_thin=True)
            if not gaps:
                return None
            opportunity = dict(policy='current_request_independent_bilateral_view_gap',
                request_id=ticket.request_id, generation=ticket.generation,
                target_epoch=ticket.target_epoch, snapshot_stamp_ns=prepared.snapshot.stamp_ns,
                contact_search_code=code, checked_count=search['checked_count'],
                missing_surface_direction=deepcopy(gaps[0]),
                contact_or_motion_authorized=False)
            if surface_proof is not None:
                # Preserve the emitted code and its completed zero-candidate
                # proof; the same literal also has a nonzero-search producer.
                opportunity['completed_contact_search'] = surface_proof
            return opportunity
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
                    self.gripper_geometry, prepared.pose_estimator.T_base_camera_link[:3, 3],
                    include_thin=visual_opportunity is not None)
            except (AttributeError, KeyError, TypeError, ValueError):
                gaps = ()
            trigger_evidence = deepcopy(visual_opportunity)
            if gaps and snapshot is not None and trigger_evidence is None:
                proof = self._observe_return_completed_contact_failure(
                    prepared, 'NEAR_FIELD_NO_HARD_SAFE_CANDIDATE')
                if proof is None:
                    return False
                trigger_evidence = dict(policy='current_request_completed_contact_view_deficit',
                    request_id=ticket.request_id, generation=ticket.generation,
                    target_epoch=surface.identity.epoch, target_track_id=surface.identity.track_id,
                    snapshot_stamp_ns=snapshot.stamp_ns, selection=deepcopy(self._mode_selection),
                    contact_search_completed=True, initial_contact_search=proof,
                    missing_surface_direction=deepcopy(gaps[0]),
                    measured_missing_sides=deepcopy(list(gaps)),
                    source_view_stamps_ns=list(surface.view_stamps_ns),
                    contact_or_motion_authorized=False)
                trigger_evidence['evidence_sha256'] = evidence_digest(trigger_evidence)
            self._observe_return_view_gaps = gaps
            if gaps and snapshot is not None:
                # Remember the manually aligned source view too; returning to
                # that ray after a later view is not a new observation.
                self._observe_return_visited_camera_origins.append(
                    deepcopy(gaps[0]['initial_camera_origin_base_m']))
            try:
                profiles = prepared.remote_diagnostics['tabletop_geometry']['adaptive_stage_profiles']
                self._observe_return_view_uncertainty_m = observation_information_uncertainty_m(profiles)
            except (AttributeError, KeyError, TypeError, ValueError):
                self._observe_return_view_uncertainty_m = None
            self._observe_return_initial_stamp = min(surface.view_stamps_ns)
            self._observe_return_trigger = trigger
            self._observe_return_trigger_evidence = trigger_evidence
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
            # The saved surface/gain stay in the immutable phase reference.
            # Prepared geometry describes this snapshot, so measure its gaps
            # against the existing same-source inverse registration instead.
            snapshot_surface, _ = self._snapshot_multiview_surface()
            if (snapshot_surface is None or snapshot_surface.identity != identity
                    or snapshot_surface.view_stamps_ns != surface.view_stamps_ns):
                return False
            gaps = measured_bilateral_view_gaps(prepared.remote_diagnostics,
                prepared.geometry, self.tabletop_geometry_config, snapshot_surface,
                self.gripper_geometry, prepared.pose_estimator.T_base_camera_link[:3, 3],
                include_thin=True)
            if not gaps:
                return False
            try:
                uncertainty = observation_information_uncertainty_m(
                    prepared.remote_diagnostics['tabletop_geometry']['adaptive_stage_profiles'])
            except (AttributeError, KeyError, TypeError, ValueError, OverflowError):
                return False
            if code == CONTACT_VIEW_YIELD:
                now = int(self._ros_source_clock_ns())
                if (not self._direct_near_field_deadline_gate(prepared)()
                        or now >= proof['yield_evidence']['original_phase_deadline_ns']):
                    return False
                proof['yield_evidence']['transition_ns'] = now
                opportunity['yielded_contact_search'] = deepcopy(proof)
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
            if code == CONTACT_VIEW_YIELD and not self._direct_near_field_deadline_gate(prepared)():
                return False
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
        started = time.monotonic()

        def trace(stage):
            # One bounded record per handoff stage, not a frame-rate log. Raw
            # identifiers correlate receipt only; they grant no authority.
            rospy.loginfo('OBSERVE_RETURN_SESSION_HANDOFF %s', json.dumps(dict(
                stage=stage, elapsed_sec=max(0., time.monotonic()-started),
                session_id=str(raw.get('session_id', ''))[:128],
                phase=str(raw.get('phase', ''))[:32],
                stamp_ns=str(raw.get('stamp_ns', ''))[:32],
                paused=bool(self._observe_return_paused)), sort_keys=True))

        try:
            raw = json.loads(msg.data)
            trace('callback_entry')
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
                trace('paused_written')
                trace('work_drain_begin')
                with self._mode_work_lock, self._mode_phase_lock, self._stream_condition, self._geometry_state_guard():
                    trace('work_drain_end')
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
            hold = getattr(self, '_station_requalification_hold', None)
            measurement = getattr(self, '_measurement_remote_request', None) or {}
            same_view_requalification = bool(hold is not None
                and measurement.get('state') == 'qualified'
                and measurement.get(STATION_REQUALIFICATION) == hold['witness'])
            if hold is not None:
                try:
                    if not same_view_requalification:
                        raise ValueError('STATION_REQUALIFICATION_QUALIFIED_LEASE_REQUIRED')
                    self._station_requalification_validate(measurement)
                except (ValueError, KeyError, TypeError, AttributeError) as exc:
                    self._station_compute_retire(str(exc))
                    return
            prior_gain = deepcopy(self._observe_return_step_gain)
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
                    if same_view_requalification:
                        self._station_requalification_validate(measurement)
                        # Re-measuring this existing view is not another camera
                        # viewpoint. Keep its already registered coverage proof;
                        # the new registration above still qualifies geometry.
                        if (information_gain_valid(prior_gain)
                                and prior_gain['target_epoch'] == observation.identity.epoch
                                and prior_gain['target_track_id'] == observation.identity.track_id
                                and prior_gain['source_stamp_ns'] in previous_surface.view_stamps_ns):
                            gain = prior_gain
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
            self._observe_return_reached_contact_context = dict(
                phase_id=int(msg.phase_id), start_stamp_ns=start, deadline_ns=deadline,
                source_stamp_ns=int(observation.stamp_ns),
                registration_phase_id=int(self._near_field_phase_id))
            self._observe_return_paused = False

    def _observe_return_audit(self, report, prepared):
        if not self._observe_return_selected():
            return
        state = getattr(self, '_contact_yield_search', None)
        if state is not None and state['key'] == self._contact_yield_request_key(prepared):
            report['contact_search_yield_attempt'] = {k:deepcopy(state[k]) for k in
                ('started_ns','budget_deadline_ns','original_phase_deadline_ns',
                 'original_families','attempted_families','collision_receipts','deferred_height_families',
                 'incumbent_found','fallback')}
            proof = self._contact_yield_proof(prepared, report)
            if proof is not None:
                report['moveit_selection']['contact_search_yield'] = proof
        phase = self._observe_return_phase
        snapshot = int(prepared.snapshot.stamp_ns)
        identity = self._current_stream_target_identity()
        metadata = dict(phase='initial_contact', trigger_code='', target_epoch=int(identity.epoch),
            target_track_id=identity.track_id, origin_snapshot_stamp_ns=snapshot,
            initial_surface_stamp_ns=self._observe_return_initial_stamp or snapshot)
        if (getattr(prepared, 'remote_diagnostics', {}) or {}).get('candidate_role') == 'measured_observation':
            metadata['diagnostic_observation_only'] = False
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
            if phase == 'observing_contact':
                metadata.update({key: deepcopy(session.get(key)) for key in ('version', *VIEW_FIELDS)})
                metadata['reached_view'] = deepcopy(self._observe_return_reached_contact_context)
                receipt = getattr(self, '_measurement_remote_request', None)
                if receipt is not None and receipt.get('state') == 'qualified':
                    metadata['measurement_receipt_sha256'] = measurement_digest(receipt)
                    metadata['qualified_source_stamps_ns'] = list(receipt['source_stamps_ns'])
            origin = 'observed_return_contact_snapshot' if phase == 'returned_contact' else 'observation_view_contact_preview'
            performed = True
        report.update(execution_strategy='observe_return', planning_origin=origin,
                      observation_motion_performed=performed, observe_return=metadata)
