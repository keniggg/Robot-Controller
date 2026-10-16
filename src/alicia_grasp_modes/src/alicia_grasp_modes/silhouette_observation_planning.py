"""Independent, non-contact camera-view planning for bound observation prefixes.

This path has no grasp candidate, inferred object width, contact gate, or
historical grasp tracker. It reuses the original observation envelope, strict
path checks and promotion transaction with a separately typed observation seed.
"""
from copy import deepcopy
from dataclasses import asdict, dataclass, replace
from types import SimpleNamespace
import math
import time
import json
import re
import hashlib

import numpy as np

from alicia_flexible_grasp.vision.rgbd_snapshot import retained_exact_source

from .runner_contract import observe_return_audit_error, supplemental_observation_audit_error
from .observe_return_visual import diversified_observation_checks
from .observation_view_utility import information_motion_rank
from .observation_support_schedule import (
    ObservationSupportSchedule, support_schedule_priority)
from .observation_search_progress import SearchProgress, family_key, canonical
from .observation_search_failure import (
    CODE as SEARCH_BUDGET_CODE, SOURCE as SEARCH_BUDGET_SOURCE,
    POLICY as SEARCH_BUDGET_POLICY, REASON as SEARCH_BUDGET_REASON)
from .silhouette_observation_order import (
    POLICY as DIAGNOSTIC_ORDER_POLICY, COVERAGE_POLICY, INFORMATION_COVERAGE_POLICY,
    diagnostic_coverage_indices, ObservationCheckSchedule,
    endpoint_schedule_metrics, transform_fingerprint, rigid_transform)
from .silhouette_observation_view_source import (
    measured_view_source, camera_parameters, array_fingerprint)
from .observation_arrival_projection import (
    capture_arrival_projection, validate_arrival_projection, arrival_projection_camera_signature)
from alicia_flexible_grasp.robot.observation_view_guard import (
    validate_observation_view_trajectory)
from alicia_flexible_grasp.robot.observation_view_limits import (
    DEFAULT_OBSERVATION_VIEW_PARAMETERS, effective_observation_view_limits,
    read_observation_view_parameters)


SOURCE = 'silhouette_observation'
MEASURED_SOURCE = 'measured_observation'
INFORMATION_ORDER_POLICY = 'measured_support_witness_then_bounded_information_utility_and_ik_motion_proxy'


class _StationObservationBudgetReached(Exception):
    """Unwind unfinished advisory work to a bound, non-executable result."""


@dataclass(frozen=True)
class ObservationPlanningSeed:
    request_id: int
    generation: int
    target_epoch: int
    target_track_id: str
    snapshot_stamp_ns: int
    snapshot_stamp_sec: float
    target_label: str
    model_choice: str = 'unknown_tabletop'
    candidate_source: str = SOURCE
    source_lineage: tuple = (SOURCE,)
    track_id: int = 0
    variant_index: int = 0
    source_index: int = 0
    moveit_result: object = None
    final_score: float = 0.


class SilhouetteObservationPlanningMixin:
    def _station_observation_search_context(self, prepared):
        """Optional scheduling scope, admitted by the existing station proof.

        Only constructor-family counts survive fresh frames. The station's
        frozen geometry reference/gaps and exact model/configuration/owner are
        part of the key. Current noisy depth, poses and paths are never cached
        or compared with an invented geometric tolerance.
        """
        if (not self._unknown_observation_planning_active()
                or self._diagnostic_observation_planning_active()
                or not getattr(self, '_observe_return_pending_supplement', None)
                or not self._station_compute_work_certificate(prepared.ticket)):
            return None
        original = self._observe_return_original
        with self._stream_condition:
            self._require_unknown_observation_prepared(prepared)
            reference = self._station_compute_reference
            contract = reference['contract']
            self._station_compute_context(contract)
            source = self._require_diagnostic_observation_view_source(prepared)
            owner = dict(binding=deepcopy(contract['binding']),
                phase_deadline_ns=contract['phase_deadline_ns'],
                session_deadline_ns=contract['session_deadline_ns'],
                model_sha256=hashlib.sha256(str(original.rospy.get_param('/robot_description', '')).encode()).hexdigest(),
                tool_from_camera=np.asarray(source['tool_from_camera']).tolist(),
                camera=original._thaw_request_evidence(source['camera']),
                view_limits=original._thaw_request_evidence(source['view_limits']),
                frozen_geometry_gaps=deepcopy(self._observe_return_view_gaps),
                frozen_geometry_uncertainty=self._observe_return_view_uncertainty_m,
                grasp_configuration=deepcopy(self.grasp_config))
            now = int(self._ros_source_clock_ns())
            clock = getattr(self, '_station_observation_search_clock', time.monotonic)
            progress = getattr(self, '_station_observation_search_progress', None)
            if progress is None:
                progress = SearchProgress()
                self._station_observation_search_progress = progress
            token = progress.bind(owner,
                remaining_sec=(min(contract['phase_deadline_ns'], contract['session_deadline_ns'])-now)/1e9,
                monotonic_now=clock())
            return dict(prepared=prepared, contract=deepcopy(contract), owner=owner,
                        progress=progress, token=token, clock=clock, source=source,
                        dispatched=[], stop_reason='', station_budget_exhausted=False)

    def _station_observation_search_reserve(self, context):
        prepared = context['prepared']
        with self._stream_condition:
            self._require_unknown_observation_prepared(prepared)
            self._station_compute_context(context['contract'])
            if not self._station_compute_work_certificate(prepared.ticket):
                raise self._observe_return_original.StreamResultCancelled('GENERATION_STALE')
            now = int(self._ros_source_clock_ns())
            contract = context['contract']
            phase_remaining = (min(contract['phase_deadline_ns'], contract['session_deadline_ns'])-now)/1e9
            planning = float(self._observe_return_original.rospy.get_param('/robot/observation_pose_planning_time', 1.))
            poll = max(.02, float(self.grasp_config.get('near_field_replan_poll_sec', .05)))
            if not math.isfinite(poll) or poll <= 0.:
                raise ValueError('finite original near-field poll required')
            reserve = context['progress'].reserve(remaining_sec=phase_remaining,
                monotonic_now=context['clock'](), planning_sec=planning,
                publication_sec=float(contract['maximum_frame_age_sec'])+poll)
            if (not reserve['can_start'] and phase_remaining
                    > reserve['estimated_strict_sec'] + reserve['publication_reserve_sec']):
                # A monotonic reserve reached before the original ROS bound
                # is a clock discontinuity, not a station-budget receipt. Do
                # not invent a new deadline or emit contradictory JSON.
                raise self._observe_return_original.StreamResultCancelled(
                    'OBSERVATION_SEARCH_CLOCK_CHANGED')
            reserve['station_can_start'] = reserve['can_start']
            source_remaining = (float(prepared.ticket.snapshot_stamp_sec)
                + float(self._configured_execution_plan_validity_sec())
                - float(getattr(self, 'mujoco_selection_snapshot_reserve_sec', 0.))
                - float(self._stream_source_clock()))
            reserve['source_remaining_sec'] = source_remaining
            reserve['can_start'] = bool(reserve['can_start'] and source_remaining
                > reserve['estimated_strict_sec'] + reserve['publication_reserve_sec'])
            return reserve

    def _station_observation_family(self, variant):
        return family_key(variant, transform_fingerprint(
            self._observe_return_original.pose_matrix(variant['sequence'].pregrasp)))

    def _station_observation_search_checkpoint(self, context, stage):
        reserve = self._station_observation_search_reserve(context)
        if reserve['can_start']:
            return reserve
        context.update(budget=reserve, stopped_stage=stage,
            station_budget_exhausted=not reserve['station_can_start'],
            stop_reason=('OBSERVATION_STATION_BUDGET_RESERVE_REACHED'
                if not reserve['station_can_start'] else 'OBSERVATION_SOURCE_RESERVE_REACHED'))
        if context['station_budget_exhausted']:
            raise _StationObservationBudgetReached(context['stop_reason'])
        # Advisory ordering wraps ValueError as a malformed reference. Source
        # reserve is cancellation of this request, not invalid target geometry.
        raise self._observe_return_original.StreamResultCancelled(
            'OBSERVATION_SOURCE_RESERVE_REACHED')

    def _station_observation_geometry_wait_checkpoint(self, ticket):
        """Failure-only interruption of an existing geometry activation wait."""
        context = getattr(self, '_station_observation_activation_context', None)
        if context is not None and context['prepared'].ticket is ticket:
            self._station_observation_search_checkpoint(context, 'geometry_ready_wait')

    def _station_observation_positive_endpoint(self, variant, context):
        """Advisory evidence must belong to this current prepared source."""
        original = self._observe_return_original
        pose = original.pose_matrix(variant['sequence'].pregrasp)
        audit = variant.get('observation_view', {})
        if self._observation_information_utility(variant) <= 0.:
            return False
        hint = audit.get('check_scheduling_metrics', {}).get('endpoint_ik_hint', {})
        if (hint.get('status') == 'endpoint_found'
                and hint.get('pose_sha256') == transform_fingerprint(pose)
                and hint.get('source_stamp_ns') == context['prepared'].snapshot.stamp_ns):
            return True
        proposal = audit.get('proposal', {})
        return bool(variant.get('search_family') == 'joint_space_fk'
            and proposal.get('execution_authority') is False
            and proposal.get('binding') == dict(context['source']['binding'])
            and proposal.get('model_sha256') == context['owner']['model_sha256']
            and np.asarray(proposal.get('T_base_tool0', [])).shape == (4, 4)
            and np.allclose(pose, proposal['T_base_tool0'], atol=1e-10, rtol=0.))

    def _station_progress_observation_checks(self, variants, context):
        """Preserve every current candidate; reorder only before an incumbent."""
        runtime = self._active_unknown_observation_check_runtime
        runtime['station_observation_search'] = context
        ordering_started = context['clock']()
        ordered = self._order_unknown_observation_checks(variants)
        inspection = ordered.initial_order if isinstance(ordered, ObservationCheckSchedule) else tuple(ordered)
        rows = [(self._station_observation_family(v),
                 self._station_observation_positive_endpoint(v, context), index, v)
                for index, v in enumerate(inspection)]
        with self._stream_condition:
            ranked = context['progress'].order(rows)
        if isinstance(ordered, ObservationCheckSchedule):
            ordered.reorder_before_incumbent(ranked)
        else:
            ordered = ranked
        report = dict(policy='same_station_least_attempted_then_current_positive_endpoint',
            execution_authority=False, candidates_removed=0, candidate_count=len(variants),
            owner_sha256=hashlib.sha256(canonical(context['owner']).encode()).hexdigest(),
            request_id=int(context['prepared'].ticket.request_id),
            current_positive_endpoint_count=sum(bool(row[1]) for row in rows),
            prior_attempt_counts={row[0]:context['progress'].count(row[0]) for row in rows},
            dispatched=context['dispatched'], stop_reason='',
            ordering_wall_sec=max(0.,context['clock']()-ordering_started))
        runtime['unknown_observation_scheduling']['station_progress'] = report
        self._observe_return_original.rospy.loginfo('observation station schedule request=%s summary=%s',
            context['prepared'].ticket.request_id, canonical({key:value for key,value in report.items()
                if key not in ('prior_attempt_counts','dispatched')}))

        def dispatch():
            for variant in ordered:
                reserve = self._station_observation_search_reserve(context)
                if not reserve['can_start']:
                    context.update(station_budget_exhausted=not reserve['station_can_start'],
                        stop_reason=('OBSERVATION_STATION_BUDGET_RESERVE_REACHED'
                            if not reserve['station_can_start'] else 'OBSERVATION_SOURCE_RESERVE_REACHED'),
                        budget=reserve)
                    report.update(stop_reason=context['stop_reason'], budget=deepcopy(reserve))
                    return
                family = self._station_observation_family(variant)
                runtime['_station_search_dispatch'] = dict(family=family, variant=variant)
                try:
                    yield variant
                finally:
                    runtime.pop('_station_search_dispatch', None)
        return dispatch()

    def _publish_station_observation_search_exhausted(self, prepared, runtime):
        """Publish only a bound failure; no geometry READY wait or authority."""
        original = self._observe_return_original
        context = runtime['station_observation_search']
        with self._stream_condition:
            self._require_unknown_observation_prepared(prepared)
            self._station_compute_context(context['contract'])
            if not self._station_compute_work_certificate(prepared.ticket):
                raise original.StreamResultCancelled('GENERATION_STALE')
            now = int(self._ros_source_clock_ns())
            contract = context['contract']; budget = context['budget']
            receipt = dict(version=1, policy=SEARCH_BUDGET_POLICY, execution_authority=False,
                reason=SEARCH_BUDGET_REASON, binding=deepcopy(contract['binding']),
                phase_deadline_ns=contract['phase_deadline_ns'], session_deadline_ns=contract['session_deadline_ns'],
                request=dict(request_id=int(prepared.ticket.request_id), generation=int(prepared.ticket.generation),
                    target_epoch=int(prepared.ticket.target_epoch), snapshot_stamp_ns=int(prepared.snapshot.stamp_ns)),
                emitted_ns=now, remaining_sec=(min(contract['phase_deadline_ns'],contract['session_deadline_ns'])-now)/1e9,
                estimated_strict_sec=budget['estimated_strict_sec'], publication_reserve_sec=budget['publication_reserve_sec'],
                attempted_count=int(budget['attempted_count']), timeout_count=int(budget['timeout_count']),
                unchecked_count=max(0,len(runtime['observation_variants'])-len(context['dispatched'])))
            from .observation_search_failure import validate_search_budget_failure
            validate_search_budget_failure(receipt, contract, now_ns=now, header_ns=now)
            terminal = original.Grasp6DPlan(); terminal.header.frame_id='base_link'
            # This is a failure emission timestamp, explicitly not a new source.
            terminal.header.stamp = original.rospy.Time(now//10**9, now%10**9)
            terminal.valid=False; terminal.candidate_source=SEARCH_BUDGET_SOURCE
            terminal.target_track_id=contract['binding']['target_track_id']
            terminal.diagnostic=SEARCH_BUDGET_CODE+': '+canonical(receipt)
            legacy=original.PoseArray(); legacy.header=deepcopy(terminal.header)
            self._publish_preview_plan(terminal, legacy, ticket=prepared.ticket)
            runtime['observation_search_budget_failure']=receipt
            self._observe_return_paused=True
            original.rospy.loginfo('observation search budget exhausted: %s', canonical(receipt))
            return dict(status=SEARCH_BUDGET_CODE, observation_search_budget_failure=receipt)

    def _unknown_observation_planning_active(self):
        return bool((self._mode_selection or {}).get('mode') == 'unknown'
            and self._observe_return_selected()
            and self._observe_return_phase == 'observation_prefix'
            and not self._observe_return_paused)

    def _observation_contract_code(self, suffix):
        return ('SILHOUETTE_OBSERVATION_' if self._diagnostic_observation_planning_active()
            else 'UNKNOWN_OBSERVATION_') + suffix

    def _diagnostic_observation_planning_active(self):
        return bool((self._mode_selection or {}).get('mode') == 'unknown'
            and self._observe_return_selected() and self._observe_return_diagnostic_observation_only
            and self._observe_return_phase == 'observation_prefix'
            and not self._observe_return_paused)

    def _independent_observation_planning_active(self):
        return bool(self._diagnostic_observation_planning_active()
            or (self._unknown_observation_planning_active()
                and not self._observe_return_diagnostic_observation_only
                and self._observe_return_trigger in ('BILATERAL_SURFACE_EVIDENCE_MISSING',
                                                    'BOX_INFERENCE_PRECISION_VIEW_REQUIRED')))

    def _independent_observation_source(self):
        return SOURCE if self._diagnostic_observation_planning_active() else MEASURED_SOURCE

    def _independent_observation_role(self):
        return ('diagnostic_observation' if self._diagnostic_observation_planning_active()
                else 'measured_observation')

    def _make_observation_sequence(self, grasp_pose, geometry, insertion_axis_base,
            snapshot=None, observation_reference_pose=None, observation_camera_distance_m=None):
        context = getattr(self, '_station_observation_generation_context', None)
        if context is not None:
            self._station_observation_search_checkpoint(context, 'candidate_generation')
        if not self._independent_observation_planning_active():
            return super()._make_observation_sequence(grasp_pose, geometry, insertion_axis_base,
                snapshot=snapshot, observation_reference_pose=observation_reference_pose,
                observation_camera_distance_m=observation_camera_distance_m)
        original = self._observe_return_original
        nominal = float(self.grasp_config.get('observation_camera_target_nominal_distance_m', .200))
        minimum = float(self.grasp_config.get('observation_camera_target_min_distance_m', .180))
        maximum = float(self.grasp_config.get('observation_camera_target_max_distance_m', .220))
        selected = nominal if observation_camera_distance_m is None else float(observation_camera_distance_m)
        pose, audit = original.centered_observation_pose_at_camera_distance(
            observation_reference_pose if observation_reference_pose is not None else grasp_pose,
            geometry.center_base, self._tool_from_camera_matrix(), selected, minimum, maximum)
        # Retain the original measured noise model, but never build hypothetical
        # approach/grasp/lift poses for an observation-only candidate.
        profiles = self._adaptive_stage_profiles(geometry, snapshot)
        if self._diagnostic_observation_planning_active():
            uncertainty = max(float(profile.depth_uncertainty_m) for profile in profiles)
        else:
            from .observation_information_uncertainty import observation_information_uncertainty_m
            uncertainty = observation_information_uncertainty_m(profiles)
        if not math.isfinite(uncertainty) or uncertainty < 0.:
            raise original.CandidateContractError('OBSERVATION_VIEW_GEOMETRY_INVALID',
                'diagnostic observation requires finite live depth uncertainty')
        audit = dict(audit, nominal_camera_target_distance_m=nominal,
            orientation_source='frozen_snapshot_current_tool0',
            candidate_role=self._independent_observation_role(), contact_geometry_checked=False)
        if observation_camera_distance_m is not None:
            audit['selection_rule'] = 'bounded_camera_target_distance_search'
        return SimpleNamespace(pregrasp=pose,
            adaptive_stage_profile=SimpleNamespace(depth_uncertainty_m=uncertainty),
            observation_view_audit=audit)

    @classmethod
    def _merge_pipeline_funnel(cls, local_funnel, moveit_funnel=None, stable_count=0,
            preview_count=0, promotion_count=0, stable_candidates=(), preview_candidate=None,
            promoted_candidate=None):
        report = super()._merge_pipeline_funnel(local_funnel, moveit_funnel,
            stable_count, preview_count, promotion_count, stable_candidates,
            preview_candidate, promoted_candidate)
        if not (local_funnel or {}).get('independent_camera_view_seed'):
            return report
        # No grasp tracker or contact candidate was consulted. Keep the shared
        # publication counters while identifying their independent source.
        source = local_funnel.get('independent_observation_source', SOURCE)
        report['source_counts'] = {source:dict(generated=1,
            locally_valid=int(report['stage_counts']['locally_valid']['passed']),
            stable=int(report['stage_counts']['stable']['passed']),
            preview=int(report['stage_counts']['preview']['passed']),
            promoted=int(report['stage_counts']['promoted']['passed']))}
        report.update(independent_camera_view_seed=True,grasp_tracker_used=False,
            contact_geometry_checked=False)
        return report

    def _activate_far_field_observation_variant(self, runtime, variant, attempted_count):
        super()._activate_far_field_observation_variant(runtime, variant, attempted_count)
        if runtime.get('diagnostic_observation_seed') is not None:
            if not self._diagnostic_observation_planning_active():
                runtime['observation_orientation_search'].update(
                    candidate_role=self._independent_observation_role(),
                    contact_candidate_yaw_used=False)
                return
            directed = (runtime['observation_view'].get('orientation_source')
                == 'measured_silhouette_ambiguity_geodesic')
            runtime['observation_orientation_search'].update(
                policy=('measured_silhouette_ambiguity_and_existing_camera_view_search'
                    if self._diagnostic_observation_planning_active()
                    else 'measured_missing_side_and_existing_camera_view_search'),
                object_yaw_used=directed, measured_target_geometry_direction_used=directed,
                contact_candidate_yaw_used=False, candidate_role=self._independent_observation_role())

    @staticmethod
    def _observation_information_utility(variant):
        evidence = (variant.get('observation_view') or {}).get('information_utility')
        if not isinstance(evidence, dict):
            raise ValueError('ordinary observation information utility unavailable')
        value = evidence.get('utility')
        if (isinstance(value, (bool, np.bool_)) or not isinstance(value, (int, float, np.number))
                or not math.isfinite(float(value)) or not 0. <= float(value) <= 1.):
            raise ValueError('ordinary observation information utility invalid')
        return float(value)

    @staticmethod
    def _observation_information_geometry(metrics, utility, current_distance):
        if not math.isfinite(current_distance) or current_distance <= 0.:
            raise ValueError('frozen current camera target distance invalid')
        motion = metrics['tool_so3_delta_rad'] + metrics['tool_translation_delta_m'] / current_distance
        return dict(predicted_information_utility=utility,
            geometric_motion_proxy=motion,
            current_camera_target_distance_m=current_distance,
            information_geometry_rank=information_motion_rank(utility, motion),
            information_prediction_only=True, motion_proxy_is_duration=False)

    def _observation_support_schedule(self, prepared):
        # Recheck all original source/configuration/lifetime contracts before
        # consulting request-local predictions; they never carry authority.
        source = self._require_diagnostic_observation_view_source(prepared)
        cache = getattr(self, '_unknown_observation_support_schedule', None)
        if (cache is None or cache[0] is not prepared or cache[1].source is not source):
            cache = (prepared, ObservationSupportSchedule(source))
            self._unknown_observation_support_schedule = cache
        return cache[1]

    def _order_unknown_observation_checks(self, variants, *, defer_endpoint_hints=False):
        """Schedule only this request's frozen views; retain all strict checks."""
        original = self._observe_return_original
        try:
            runtime = getattr(self, '_active_unknown_observation_check_runtime', None)
            if runtime is None and self._independent_observation_planning_active():
                runtime = self._stable_variant_runtime[(0, 0)]
            if not isinstance(runtime, dict):
                raise ValueError('candidate-bound observation runtime unavailable')
            prepared = runtime['prepared']
            self._require_unknown_observation_prepared(prepared)
            seed = runtime.get('diagnostic_observation_seed')
            expected = dict(request_id=prepared.ticket.request_id,
                generation=prepared.ticket.generation, target_epoch=prepared.ticket.target_epoch,
                target_track_id=prepared.snapshot.target_identity.track_id,
                snapshot_stamp_ns=prepared.snapshot.stamp_ns)
            if isinstance(seed, ObservationPlanningSeed):
                binding = runtime['diagnostic_observation_reference']
                if (any(getattr(seed, key) != value for key, value in expected.items())
                        or runtime['grasp_pose'].header.frame_id != 'base_link'
                        or runtime['grasp_pose'].header.stamp.to_nsec() != prepared.snapshot.stamp_ns
                        or transform_fingerprint(original.pose_matrix(runtime['grasp_pose']))
                            != binding['transform_sha256']):
                    raise ValueError('frozen diagnostic tool reference changed')
            else:
                source = self._require_diagnostic_observation_view_source(prepared)
                binding = dict(source['binding'], base_from_tool=source['base_from_tool'],
                    transform_sha256=source['audit']['frozen_tool_reference_sha256'])
            if (any(binding.get(key) != value for key, value in expected.items())
                    or transform_fingerprint(binding['base_from_tool']) != binding['transform_sha256']):
                raise ValueError('frozen view reference belongs to another request or changed')
            reference = binding['base_from_tool']
            information_driven = not self._diagnostic_observation_planning_active()
            current_distance = None
            support_schedule = None
            if information_driven:
                source = self._require_diagnostic_observation_view_source(prepared)
                support_schedule = self._observation_support_schedule(prepared)
                camera_origin = np.asarray(reference).dot(np.asarray(source['tool_from_camera']))[:3, 3]
                current_distance = float(np.linalg.norm(np.asarray(source['center_base']) - camera_origin))
            rows = []
            for index, variant in enumerate(variants):
                progress_context = runtime.get('station_observation_search')
                if progress_context is not None:
                    self._station_observation_search_checkpoint(progress_context, 'endpoint_ordering')
                sequence = variant['sequence']; pose = sequence.pregrasp
                envelope = variant['observation_envelope']
                if (pose.header.frame_id != 'base_link'
                        or pose.header.stamp.to_nsec() != prepared.snapshot.stamp_ns
                        or not envelope.ok):
                    raise ValueError('candidate is not a qualified same-source endpoint')
                metrics = endpoint_schedule_metrics(reference, original.pose_matrix(pose),
                    envelope.minimum_support_clearance_m)
                sort_key = metrics.pop('sort_key')
                if information_driven:
                    metrics.update(self._observation_information_geometry(metrics,
                        self._observation_information_utility(variant), current_distance))
                    metrics['measured_support_schedule'] = support_schedule.evaluate(original.pose_matrix(pose))
                    sort_key = (support_schedule_priority(metrics['measured_support_schedule']),
                        metrics['information_geometry_rank'], sort_key)
                rows.append((sort_key, index, variant, metrics))
            ordered = sorted(rows, key=lambda row: (row[0], row[1]))
            if defer_endpoint_hints:
                # Geometry-qualified FK views may enter the original strict
                # loop before advisory IK searches. No hint budget is spent
                # or reset here; a fallback uses the same prepared cache.
                hints, hint_report = {}, dict(status='deferred_until_first_fk_check',
                    execution_authority=False, candidates_removed=0,
                    maximum_start_budget_sec=3., maximum_queries=96)
            else:
                hints, hint_report = self._observation_endpoint_search_hints(
                    tuple(row[2] for row in ordered), prepared)
            def endpoint_priority(row):
                hint = hints.get(transform_fingerprint(
                    original.pose_matrix(row[2]['sequence'].pregrasp)), {})
                row[3]['endpoint_ik_hint'] = deepcopy(hint)
                support_priority = (support_schedule_priority(row[3]['measured_support_schedule'])
                    if information_driven else 0)
                if hint.get('status') == 'endpoint_found':
                    if information_driven:
                        rank = information_motion_rank(row[3]['predicted_information_utility'],
                            hint['joint_max_delta_rad'], hint['joint_max_delta_rad'], hint['joint_l1_delta_rad'])
                        row[3].update(information_motion_rank=rank,
                            motion_cost_proxy=hint['joint_max_delta_rad'],
                            motion_cost_proxy_kind='maximum_joint_displacement_rad',
                            motion_proxy_is_duration=False)
                        return (support_priority, 0, rank, row[0], row[1])
                    return (0, hint['joint_max_delta_rad'], hint['joint_l1_delta_rad'], row[0], row[1])
                if information_driven:
                    row[3].update(information_motion_rank=row[3]['information_geometry_rank'],
                        motion_cost_proxy=row[3]['geometric_motion_proxy'],
                        motion_cost_proxy_kind='tool_so3_rad_plus_translation_over_current_camera_distance',
                        motion_proxy_is_duration=False)
                    return (support_priority, 1, row[3]['information_geometry_rank'], row[0], row[1])
                return (1, 0., 0., row[0], row[1])
            ordered = sorted(ordered, key=endpoint_priority)
            scheduling_policy = INFORMATION_ORDER_POLICY if information_driven else DIAGNOSTIC_ORDER_POLICY
            for rank, (_key, _index, variant, metrics) in enumerate(ordered):
                sequence = variant['sequence']
                if information_driven:
                    for key in ('information_geometry_rank', 'information_motion_rank'):
                        metrics[key] = [value if math.isfinite(float(value)) else None for value in metrics[key]]
                sequence.observation_view_audit = dict(sequence.observation_view_audit,
                    **({'information_utility': deepcopy(variant['observation_view']['information_utility'])}
                       if information_driven else {}),
                    check_scheduling_policy=scheduling_policy,
                    check_scheduling_index=rank, check_scheduling_metrics=metrics,
                    check_scheduling_reference=dict(expected, transform_sha256=binding['transform_sha256']),
                    original_strict_check_budget_unchanged=not information_driven,
                    strict_check_maximum_count_and_source_limits_unchanged=True,
                    comparison_timing_policy=('shared_eight_seconds_after_first_qualified_view'
                        if information_driven else 'legacy_tilt_budget'),
                    endpoint_schedule_proves_joint_motion=False,
                    endpoint_schedule_proves_path_visibility=False,
                    endpoint_schedule_is_global_optimum=False)
                variant['observation_view'] = deepcopy(sequence.observation_view_audit)
            runtime['unknown_observation_scheduling'] = dict(policy=scheduling_policy,
                reference=dict(expected, transform_sha256=binding['transform_sha256']),
                candidate_count=len(rows), pruned_candidate_count=0,
                endpoint_ik_search_hint=deepcopy(hint_report),
                measured_support_schedule=(support_schedule.report() if support_schedule is not None else None),
                final_selection=('predicted_information_utility_and_checked_path_duration' if information_driven
                    else 'minimum_checked_timed_path_duration_then_joint_motion'),
                information_prediction_only=information_driven,
                motion_proxy_is_duration=False,
                original_strict_check_budget_unchanged=not information_driven,
                strict_check_maximum_count_and_source_limits_unchanged=True,
                comparison_timing_policy=('shared_eight_seconds_after_first_qualified_view'
                    if information_driven else 'legacy_tilt_budget'),
                proves_joint_motion_or_path_visibility=False, claims_global_optimum=False)
            original_order = tuple(row[2] for row in ordered)
            if (len(original_order) > 1
                    and (information_driven or not any(
                        row[3]['endpoint_ik_hint'].get('status') == 'endpoint_found'
                        for row in ordered))
                    and all(v.get('camera_target_distance_m') is not None for v in original_order)):
                mount = rigid_transform(self._tool_from_camera_matrix())
                order = diagnostic_coverage_indices(
                    [v['camera_target_distance_m'] for v in original_order],
                    [(original.pose_matrix(v['sequence'].pregrasp) @ mount)[:3, 0]
                     for v in original_order],
                    self.grasp_config.get('observation_camera_target_nominal_distance_m', .26))
                positive_hint_first = None
                if information_driven:
                    # One promising endpoint may lead, but an IK hint never
                    # replaces the full-path incumbent. After a failed first
                    # check, spread the remaining work across view families.
                    best_support_priority = support_schedule_priority(ordered[0][3]['measured_support_schedule'])
                    positive_hint_first = next((index for index, row in enumerate(ordered)
                        if (row[3]['endpoint_ik_hint'].get('status') == 'endpoint_found'
                            and row[3]['predicted_information_utility'] > 0.
                            and support_schedule_priority(row[3]['measured_support_schedule'])
                                == best_support_priority)), None)
                    if positive_hint_first is not None:
                        order = (positive_hint_first, *(index for index in order
                            if index != positive_hint_first))
                    # Retain zero-information candidates without spending the
                    # first-path budget on them before positive-gain views.
                    positive_gain = tuple(index for index in order
                        if ordered[index][3]['predicted_information_utility'] > 0.)
                    zero_gain = tuple(index for index in order
                        if ordered[index][3]['predicted_information_utility'] <= 0.)
                    order = tuple(sorted(positive_gain + zero_gain, key=lambda index: (
                        support_schedule_priority(ordered[index][3]['measured_support_schedule']),
                        ordered[index][3]['predicted_information_utility'] <= 0.)))
                coverage = dict(policy=(INFORMATION_COVERAGE_POLICY if information_driven else COVERAGE_POLICY),
                    execution_authority=False,
                    candidates_removed=0, full_path_incumbent_found=False,
                    distance_direction_order=list(order), dispatched_original_indices=[],
                    positive_hint_first_original_index=positive_hint_first,
                    original_strict_check_budget_unchanged=True,
                    after_incumbent_policy=scheduling_policy)
                runtime['unknown_observation_scheduling']['first_incumbent_coverage'] = coverage

                return ObservationCheckSchedule(original_order, order, coverage,
                    validate=lambda: self._require_unknown_observation_prepared(prepared),
                    qualified=lambda: runtime.get('_observation_qualified_incumbent_ticket')
                        is prepared.ticket)
            return original_order
        except (KeyError, AttributeError, TypeError, ValueError, OverflowError) as exc:
            raise original.CandidateContractError(self._observation_contract_code('REFERENCE_INVALID'), str(exc))

    def _observation_endpoint_search_hints(self, variants, prepared):
        """Bound read-only IK work; hints reorder, never authorize or prune.

        One budget and exact-pose cache belong to this prepared request. A
        failed/timed-out query is unknown, not proof of unreachability. Every
        candidate still enters the unchanged strict path/following/view gates.
        """
        original = self._observe_return_original
        rospy = original.rospy
        self._require_unknown_observation_prepared(prepared)
        key = (prepared.ticket.generation, prepared.ticket.target_epoch,
            prepared.ticket.request_id, prepared.snapshot.stamp_ns,
            prepared.snapshot.target_identity.track_id)
        model = rospy.get_param('/robot_description', '')
        model_sha = hashlib.sha256(str(model).encode()).hexdigest()
        cache = getattr(self, '_unknown_observation_endpoint_hint_cache', None)
        if (not isinstance(cache, dict) or cache.get('prepared_id') != id(prepared)
                or cache.get('key') != key or cache.get('model_sha256') != model_sha):
            cache = dict(prepared_id=id(prepared), key=key, model_sha256=model_sha,
                hints={}, used_sec=0., checked_count=0, unavailable=False)
            self._unknown_observation_endpoint_hint_cache = cache
        report = dict(policy='request_local_collision_aware_endpoint_ik_hint',
            execution_authority=False, candidates_removed=0,
            maximum_start_budget_sec=3., maximum_queries=96, ik_timeout_sec=.02,
            robot_description_sha256=model_sha, checked_count=cache['checked_count'],
            hinted_count=0, status='unavailable', used_sec=cache['used_sec'])
        hints = cache['hints']
        state = deepcopy(getattr(self, 'latest_joint_state', None))
        try:
            names = list(state.name); positions = list(state.position)
            stamp_ns = state.header.stamp.to_nsec()
            age = (rospy.Time.now().to_nsec() - stamp_ns) * 1e-9
            joints = tuple('Joint%d' % i for i in range(1, 7))
            if (not model or len(names) != len(set(names)) or len(names) != len(positions)
                    or not set(joints).issubset(names) or stamp_ns <= 0
                    or not 0. <= age <= .5 or not np.all(np.isfinite(positions))):
                raise ValueError('missing or stale finite joint seed / runtime model')
            seed = dict(zip(names, positions))
        except (AttributeError, TypeError, ValueError, OverflowError) as exc:
            report['reason'] = str(exc)
            return {}, report
        report['joint_seed'] = dict(stamp_ns=stamp_ns, age_sec=age,
            name=names, position=positions)
        if cache['unavailable']:
            report['reason'] = 'IK transport unavailable for this request'
            return hints, report
        clock = getattr(self, '_observation_endpoint_hint_clock', time.monotonic)
        remaining = 3. - cache['used_sec']
        started = clock(); deadline = started + max(0., remaining)
        report['status'] = 'complete'
        try:
            from moveit_msgs.msg import RobotState
            from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest
            if remaining > 0. and cache['checked_count'] < 96:
                try:
                    rospy.wait_for_service('/compute_ik', timeout=min(.15, remaining))
                    service = rospy.ServiceProxy('/compute_ik', GetPositionIK)
                except Exception as exc:
                    cache['unavailable'] = True
                    report.update(status='unavailable', reason=str(exc))
                    return hints, report
                # Probe dispersed existing rolls/distances without changing
                # their poses; actual tool and joint deltas rank found hints.
                probe_order = diversified_observation_checks(variants,
                    self.grasp_config.get('observation_camera_target_nominal_distance_m', .26))
                if not self._diagnostic_observation_planning_active():
                    source = self._require_diagnostic_observation_view_source(prepared)
                    support_schedule = self._observation_support_schedule(prepared)
                    reference = np.asarray(source['base_from_tool'])
                    camera_origin = reference.dot(np.asarray(source['tool_from_camera']))[:3, 3]
                    distance = float(np.linalg.norm(np.asarray(source['center_base']) - camera_origin))
                    probe_priorities = {}
                    def information_probe_priority(variant):
                        metrics = endpoint_schedule_metrics(reference,
                            original.pose_matrix(variant['sequence'].pregrasp),
                            variant['observation_envelope'].minimum_support_clearance_m)
                        priority = (support_schedule_priority(support_schedule.evaluate(
                            original.pose_matrix(variant['sequence'].pregrasp))),
                            self._observation_information_geometry(metrics,
                                self._observation_information_utility(variant), distance)['information_geometry_rank'])
                        probe_priorities[id(variant)] = priority
                        return priority
                    probe_order = tuple(sorted(probe_order, key=information_probe_priority))
                    report.update(probe_order_policy='measured_support_witness_then_information_motion_rank_then_roll_distance_ties',
                        measured_support_schedule=support_schedule.report(),
                        information_prediction_only=True, motion_proxy_is_duration=False)
                    if (len(probe_order) > 1 and all(
                            v.get('camera_target_distance_m') is not None for v in probe_order)):
                        # An endpoint hint is advisory, not a full-path
                        # incumbent. After the best information-ranked probe,
                        # expose other distance/direction families within the
                        # SAME start/count budget instead of exhausting it on
                        # the many rolls of one high-utility tilt direction.
                        mount = rigid_transform(source['tool_from_camera'])
                        coverage = diagnostic_coverage_indices(
                            [v['camera_target_distance_m'] for v in probe_order],
                            [(original.pose_matrix(v['sequence'].pregrasp) @ mount)[:3, 0]
                             for v in probe_order],
                            self.grasp_config.get('observation_camera_target_nominal_distance_m', .26))
                        coverage = (0, *(index for index in coverage if index != 0))
                        # Preserve measured-support strata and keep zero-gain
                        # views behind useful views within the same stratum.
                        coverage = sorted(coverage, key=lambda index: (
                            probe_priorities[id(probe_order[index])][0],
                            self._observation_information_utility(probe_order[index]) <= 0.))
                        probe_order = tuple(probe_order[index] for index in coverage)
                        report.update(
                            probe_order_policy='measured_support_then_best_information_then_distance_direction_coverage',
                            best_information_probe_first=True,
                            coverage_changes_poses=False, coverage_extends_budget=False)
                runtime = getattr(self, '_active_unknown_observation_check_runtime', None)
                progress_context = (runtime.get('station_observation_search')
                    if isinstance(runtime, dict) else None)
                if progress_context is not None:
                    # A fresh request must probe untried families too. These
                    # counts confer no IK knowledge and never remove a query.
                    progress = progress_context['progress']
                    probe_order = tuple(sorted(probe_order,
                        key=lambda v: progress.count(self._station_observation_family(v))))
                    report['station_progress_probe_order'] = 'least_attempted_current_family_first'
                for variant in probe_order:
                    if progress_context is not None:
                        self._station_observation_search_checkpoint(progress_context, 'endpoint_ik_hints')
                    self._require_unknown_observation_prepared(prepared)
                    remaining = deadline - clock()
                    if remaining <= 0. or cache['checked_count'] >= 96:
                        report['status'] = 'budget_exhausted'
                        break
                    pose = variant['sequence'].pregrasp
                    fingerprint = transform_fingerprint(original.pose_matrix(pose))
                    if fingerprint in hints:
                        continue
                    request = GetPositionIKRequest()
                    request.ik_request.group_name = 'alicia'
                    request.ik_request.ik_link_name = 'tool0'
                    request.ik_request.robot_state = RobotState()
                    request.ik_request.robot_state.joint_state = deepcopy(state)
                    request.ik_request.pose_stamped = deepcopy(pose)
                    request.ik_request.avoid_collisions = True
                    request.ik_request.timeout = rospy.Duration(min(.02, remaining))
                    cache['checked_count'] += 1
                    hint = dict(status='unknown', execution_authority=False,
                        pose_sha256=fingerprint, source_stamp_ns=prepared.snapshot.stamp_ns,
                        joint_seed_stamp_ns=stamp_ns, joint_seed_positions=[float(seed[n]) for n in joints])
                    hints[fingerprint] = hint
                    try:
                        response = service(request)
                    except Exception as exc:
                        # A broken service must not burn another timeout per
                        # pose or be mistaken for a collision/unreachable test.
                        cache['unavailable'] = True
                        hint['reason'] = str(exc)
                        report['status'] = 'unavailable'
                        break
                    # Invalidation during a synchronous service call propagates
                    # instead of turning the old response into a current hint.
                    self._require_unknown_observation_prepared(prepared)
                    try:
                        code = int(response.error_code.val)
                    except (AttributeError, TypeError, ValueError, OverflowError):
                        hint['reason'] = 'invalid IK status; remains unknown'
                        continue
                    hint['error_code'] = code
                    if code != 1:
                        continue
                    try:
                        solved = response.solution.joint_state
                        sn, sp = list(solved.name), list(solved.position)
                    except (AttributeError, TypeError):
                        hint['reason'] = 'missing IK joint solution; remains unknown'
                        continue
                    if (len(sn) != len(sp) or len(sn) != len(set(sn))
                            or not set(joints).issubset(sn) or not np.all(np.isfinite(sp))):
                        hint['reason'] = 'invalid IK joint solution; remains unknown'
                        continue
                    mapping = dict(zip(sn, sp))
                    deltas = [abs(float(mapping[name])-float(seed[name])) for name in joints]
                    hint.update(status='endpoint_found', joint_max_delta_rad=max(deltas),
                        joint_l1_delta_rad=sum(deltas), joint_names=list(joints),
                        joint_positions=[float(mapping[name]) for name in joints])
            else:
                report['status'] = 'budget_exhausted'
        finally:
            cache['used_sec'] += max(0., clock()-started)
            report.update(used_sec=cache['used_sec'], checked_count=cache['checked_count'],
                hinted_count=sum(h.get('status') == 'endpoint_found' for h in hints.values()))
        self._require_unknown_observation_prepared(prepared)
        if hashlib.sha256(str(rospy.get_param('/robot_description', '')).encode()).hexdigest() != model_sha:
            self._unknown_observation_endpoint_hint_cache = None
            raise ValueError('runtime model changed during observation endpoint hints')
        return hints, report

    def _check_moveit_stable_candidate(self, candidate):
        runtime = getattr(self, '_stable_variant_runtime', {}).get(
            (candidate.track_id, candidate.variant_index), {})
        prepared = runtime.get('prepared')
        guarded = (self._unknown_observation_planning_active()
            and prepared is not None and not prepared.near_field)
        if not guarded:
            return super()._check_moveit_stable_candidate(candidate)
        self._require_unknown_observation_prepared(prepared)
        if getattr(self, 'observation_following_guard_required', False) is not True:
            raise self._observe_return_original.CandidateContractError(
                self._observation_contract_code('VIEW_GUARD_REQUIRED'),
                'unknown observations require path visibility and following qualification')
        # The worker executes prepare->accept serially. Pending camera requests
        # have no prepared cloud; retain exactly this active candidate runtime.
        previous = getattr(self, '_active_unknown_observation_check_runtime', None)
        self._active_unknown_observation_check_runtime = runtime
        try:
            result = super()._check_moveit_stable_candidate(candidate)
            scheduling = runtime.get('unknown_observation_scheduling')
            if scheduling and 'observation_orientation_search' in runtime:
                runtime['observation_orientation_search'].update(
                    check_order=scheduling['policy'], scheduling=deepcopy(scheduling))
            return result
        finally:
            self._active_unknown_observation_check_runtime = previous

    def _observation_view_limit_configuration(self):
        requested = read_observation_view_parameters(self._observe_return_original.rospy.get_param)
        return dict(requested_visibility_limits_snapshot=deepcopy(requested),
            effective_visibility_limits=effective_observation_view_limits(requested))

    def _capture_diagnostic_observation_view_source(self, ticket):
        original = self._observe_return_original
        try:
            snapshot, _config = ticket.payload
            sample = retained_exact_source(snapshot)
            if sample is None:
                # Compatibility for snapshots produced without retained raw
                # evidence. This fallback still uses original cache gates.
                sample = self.frames.wait_for_exact_sample(snapshot.stamp_ns,
                    snapshot.target_identity, timeout_sec=0., require_mask=True,
                    max_age_sec=self.planning_snapshot_max_age_sec,
                    max_inference_latency_sec=self.planning_snapshot_max_inference_latency_sec)
            if sample is None:
                raise ValueError('exact original RGB-D sample unavailable')
            captured = dict(sample=sample, intrinsics=deepcopy(self._camera_intrinsics()),
                tool_from_camera=np.array(self._tool_from_camera_matrix(), copy=True),
                view_limits=self._observation_view_limit_configuration())
            if not self._diagnostic_observation_planning_active():
                from .unknown_pending_identity import reference_binding
                status = deepcopy(self._unknown_identity_status.get(snapshot.stamp_ns))
                if status is None:
                    raise ValueError('same-source arrival projection receipt unavailable')
                reference = (getattr(self, '_measurement_remote_reference', None)
                             or getattr(self, '_unknown_identity_reference', None))
                if (reference is None or reference.get('binding') != reference_binding(status, self._mode_selection)):
                    raise ValueError('arrival projection frozen target binding changed')
                camera = original.rospy.get_param('/camera', {})
                if any(camera.get(key) != value for key, value in camera_parameters(captured['intrinsics']).items()):
                    raise ValueError('arrival projection camera differs from snapshot intrinsics')
                captured['arrival_projection'] = capture_arrival_projection(
                    status, self._mode_selection, snapshot.stamp_ns, camera)
            return captured
        except (AttributeError, KeyError, TypeError, ValueError, OverflowError) as exc:
            raise original.CandidateContractError(self._observation_contract_code('VIEW_SOURCE_INVALID'), str(exc))

    def _bind_diagnostic_observation_view_source(self, prepared, captured):
        original = self._observe_return_original
        try:
            source = measured_view_source(captured['sample'], prepared,
                captured['intrinsics'], captured['tool_from_camera'])
            limits = captured['view_limits']
            if (set(limits) != {'requested_visibility_limits_snapshot', 'effective_visibility_limits'}
                    or set(limits['requested_visibility_limits_snapshot']) != set(DEFAULT_OBSERVATION_VIEW_PARAMETERS)
                    or limits['effective_visibility_limits'] != effective_observation_view_limits(
                        limits['requested_visibility_limits_snapshot'])):
                raise ValueError('original observation visibility configuration unavailable')
            source['view_limits'] = deepcopy(limits)
            source['audit']['view_limits'] = deepcopy(limits)
            if not self._diagnostic_observation_planning_active():
                source['arrival_projection'] = captured['arrival_projection']
                source['audit']['arrival_projection'] = original._thaw_request_evidence(captured['arrival_projection']['audit'])
            self._diagnostic_observation_view_source = original._freeze_request_evidence(source)
            self._diagnostic_observation_view_prepared = prepared
            self._require_diagnostic_observation_view_source(prepared)
        except (AttributeError, KeyError, TypeError, ValueError, OverflowError) as exc:
            raise original.CandidateContractError(self._observation_contract_code('VIEW_SOURCE_INVALID'), str(exc))

    def _require_diagnostic_observation_view_source(self, prepared):
        self._require_unknown_observation_prepared(prepared)
        source = self._diagnostic_observation_view_source
        binding = source['binding']
        if (self._diagnostic_observation_view_prepared is not prepared
                or binding['request_id'] != prepared.ticket.request_id
                or binding['generation'] != prepared.ticket.generation
                or binding['target_epoch'] != prepared.ticket.target_epoch
                or binding['target_track_id'] != prepared.snapshot.target_identity.track_id
                or binding['snapshot_stamp_ns'] != prepared.snapshot.stamp_ns
                or self._observation_view_limit_configuration() != source['view_limits']
                or camera_parameters(self._camera_intrinsics()) != source['camera']
                or transform_fingerprint(self._tool_from_camera_matrix()) != source['audit']['tool_camera_sha256']
                or transform_fingerprint(prepared.pose_estimator._T_base_optical) != source['audit']['base_optical_sha256']
                or array_fingerprint(source['points_base']) != source['audit']['support_points_sha256']
                or array_fingerprint(source['corners_base']) != source['audit']['conservative_target_corners_sha256']):
            raise ValueError('observation view request, camera, handeye or source points changed')
        if not self._diagnostic_observation_planning_active():
            contract = source['arrival_projection']
            audit = contract['audit']
            if (audit != source['audit']['arrival_projection']
                    or audit['source_stamp_ns'] != prepared.snapshot.stamp_ns
                    or audit['camera_signature'] != arrival_projection_camera_signature(
                        self._observe_return_original.rospy.get_param('/camera', {}))
                    or array_fingerprint(contract['prism_base']) != audit['prism_sha256']
                    or array_fingerprint(contract['rgb_valid']) != audit['rgb_valid_sha256']
                    or array_fingerprint(contract['invalid_integral']) != audit['invalid_integral_sha256']):
                raise ValueError('arrival projection source, geometry or RGB validity changed')
        return source

    def _diagnostic_observation_view_evaluation(self, prepared, result, target):
        original = self._observe_return_original
        source = self._require_diagnostic_observation_view_source(prepared)
        text = str(result.reason)
        def json_field(key):
            matches = re.findall(r'(?:^|\s)' + key + r'=(\[[^\]]+\])(?=\s|$)', text)
            if len(matches) != 1:
                raise ValueError('missing/ambiguous strict path field ' + key)
            return json.loads(matches[0])
        names = json_field('observation_joint_names')
        if len(names) != 6 or set(names) != {'Joint%d' % i for i in range(1, 7)}:
            raise ValueError('strict observation joint names mismatch')
        description = original.rospy.get_param('/robot_description', '')
        fk = original.SerialUrdfFk(description, names)
        signatures = re.findall(r'(?:^|\s)observation_robot_description_sha256=([a-f0-9]{64})(?=\s|$)', text)
        if signatures != [fk.model_sha256]:
            raise ValueError('strict observation FK model mismatch')
        tokens = re.findall(r'(?:^|\s)observation_path_evidence=(\S+)', text)
        if len(tokens) != 1 or target is None:
            raise ValueError('complete candidate path evidence unavailable')
        evidence, trajectory, hold = original.decode_path_evidence(tokens[0], target, fk)
        if not np.array_equal(np.asarray(trajectory.joint_trajectory.points[-1].positions),
                np.asarray(json_field('observation_goal_joint_positions'))):
            raise ValueError('candidate trajectory/strict IK endpoint mismatch')
        camera = source['camera']
        report = validate_observation_view_trajectory(trajectory, fk, source['tool_from_camera'],
            source['corners_base'], source['center_base'], source['points_base'],
            [camera[k] for k in ('fx', 'fy', 'cx', 'cy')],
            camera['width'], camera['height'], hold,
            **source['view_limits']['effective_visibility_limits'])
        if source.get('arrival_projection') is not None:
            report['arrival_projection_endpoint'] = validate_arrival_projection(
                source['arrival_projection'], report['target_trajectory']['endpoint_transform_base_camera'],
                [camera[k] for k in ('fx', 'fy', 'cx', 'cy')], camera['width'], camera['height'],
                **source['view_limits']['effective_visibility_limits'])
        if original.rospy.get_param('/robot_description', '') != description:
            raise ValueError('FK model changed during candidate visibility qualification')
        self._require_diagnostic_observation_view_source(prepared)
        report.update(source=original._thaw_request_evidence(source['audit']),
            reference_mode=evidence['reference_mode'], reference_epoch_ns=evidence['reference_epoch_ns'],
            candidate_path_only_not_execution_authority=True,
            admission_requires_original_following_and_collision_checks=True)
        return report

    def _observation_following_support_evaluation(self, prepared, result, target=None):
        if not self._unknown_observation_planning_active() or prepared.near_field:
            return super()._observation_following_support_evaluation(prepared, result, target)
        self._require_unknown_observation_prepared(prepared)
        try:
            view = self._diagnostic_observation_view_evaluation(prepared, result, target)
        except (AttributeError, KeyError, TypeError, ValueError, OverflowError,
                self._observe_return_original.ObservationPathError) as exc:
            code = self._observation_contract_code('PATH_VIEW_INVALID')
            # Preserve failed-source provenance for diagnosis; this is not a
            # fallback measurement or permission to reuse invalid authority.
            source_audit = {}
            if getattr(self, '_diagnostic_observation_view_prepared', None) is prepared:
                source_audit = self._observe_return_original._thaw_request_evidence(
                    getattr(self, '_diagnostic_observation_view_source', {}).get('audit', {}))
            report = dict(ok=False, reason=str(exc), failure_location='path_view',
                view_qualification=deepcopy(getattr(exc, 'audit', {})),
                measured_source=source_audit,
                certifies_hardware_tracking_stopping_or_calibration=False)
            return replace(result, reachable=False, failure_code=code,
                collision_free=None, within_joint_limits=None, ik_valid=None,
                planning_success=None, evidence_code='', reason=code + ': ' + str(exc)), report
        result, report = super()._observation_following_support_evaluation(prepared, result, target)
        # An invalidation or calibration change during expensive support proof
        # cannot reuse the earlier inexpensive projection result.
        self._require_diagnostic_observation_view_source(prepared)
        return result, dict(report, unknown_observation_view=view)

    def _require_unknown_observation_prepared(self, prepared):
        original = self._observe_return_original
        self._require_stream_ticket_current(prepared.ticket)
        if (self._information_driven_observation_comparison()
                and not self._observation_source_reserve_available(prepared.ticket)):
            raise original.CandidateContractError('OBSERVATION_SOURCE_RESERVE_REACHED',
                'observation source is future-dated or lacks planning/publication reserve')
        identity = self._current_stream_target_identity()
        if (not self._unknown_observation_planning_active() or prepared.near_field
                or prepared.model_choice != 'unknown_tabletop'
                or prepared.snapshot.target_identity != identity
                or prepared.ticket.target_epoch != identity.epoch
                or prepared.remote_failure_code
                or not prepared.geometry.ok
                or not bool(prepared.snapshot.object_msg.detected)):
            raise original.CandidateContractError(self._observation_contract_code('CONTEXT_INVALID'),
                'camera-view source requires a current detected unknown far-field observation request')

    def _require_diagnostic_observation_prepared(self, prepared):
        original = self._observe_return_original
        self._require_unknown_observation_prepared(prepared)
        if not self._independent_observation_planning_active():
            raise original.CandidateContractError('SILHOUETTE_OBSERVATION_CONTEXT_INVALID',
                'independent source requires its bound observation prefix')
        identity = self._current_stream_target_identity()
        if not self._diagnostic_observation_planning_active():
            gaps = self._observe_return_view_gaps
            if (not gaps or any(gap.get('target_epoch') != identity.epoch
                    or gap.get('target_track_id') != identity.track_id
                    or not gap.get('source_view_stamps_ns')
                    or any(type(ns) is not int or not 0 < ns <= prepared.snapshot.stamp_ns
                           for ns in gap['source_view_stamps_ns']) for gap in gaps)
                    or prepared.candidates or prepared.tabletop_candidates
                    or prepared.graspnet_input is not None
                    or prepared.remote_diagnostics.get('candidate_role') != MEASURED_SOURCE
                    or prepared.remote_diagnostics.get('contact_candidates_requested') is not False
                    or prepared.remote_diagnostics.get('contact_geometry_checked') is not False):
                raise original.CandidateContractError('UNKNOWN_OBSERVATION_CONTEXT_INVALID',
                    'independent measured view lacks bound missing-side evidence or contains contact candidates')
            return
        envelope = original._thaw_request_evidence(prepared.remote_diagnostics).get('silhouette_observation_envelope', {})
        if (envelope.get('fresh_request_id') != prepared.ticket.request_id
                or envelope.get('fresh_generation') != prepared.ticket.generation
                or envelope.get('target_epoch') != identity.epoch
                or envelope.get('target_track_id') != identity.track_id
                or envelope.get('source', {}).get('fresh_stamp_ns') != prepared.snapshot.stamp_ns):
            raise original.CandidateContractError('SILHOUETTE_OBSERVATION_CONTEXT_INVALID',
                'independent camera-view source lacks its actual conservative collision envelope')

    def _prepare_diagnostic_observation_prediction(self, ticket):
        """Freeze current geometry/TF without requesting irrelevant WSL grasps."""
        original = self._observe_return_original
        started = time.perf_counter()
        self._require_stream_ticket_current(ticket)
        snapshot, input_config = ticket.payload
        if (not self._independent_observation_planning_active()
                or not isinstance(snapshot, original.SnapshotResult) or not snapshot.ok
                or not bool(snapshot.object_msg.detected)):
            raise original.CandidateContractError('SILHOUETTE_OBSERVATION_CONTEXT_INVALID',
                'current detected diagnostic snapshot required')
        captured_view_source = self._capture_diagnostic_observation_view_source(ticket)
        revision = self._capture_geometry_generation()
        self._refresh_runtime_params()
        mismatch = self._gripper_contract_mismatch_reason()
        if mismatch:
            raise original.CandidateContractError('GRIPPER_MODEL_MISMATCH', mismatch)
        stamp = self._snapshot_ros_stamp(snapshot)
        geometry, _depth, transform = self._prepare_snapshot_geometry(snapshot, stamp)
        self._require_stream_ticket_current(ticket)
        if not geometry.ok:
            raise original.CandidateContractError(geometry.failure_code, geometry.failure_reason)
        estimator = original.FrozenSnapshotCandidatePoseEstimator(transform, stamp, snapshot.frame_id,
            raw_candidate_convention=self.candidate_frame_convention)
        prepared = original.PreparedPrediction(ticket=ticket,snapshot=snapshot,stamp=stamp,
            geometry=geometry,pose_estimator=estimator,graspnet_input=None,candidates=(),
            remote_diagnostics=dict(candidate_role=self._independent_observation_role(),
                contact_candidates_requested=False,contact_geometry_checked=False,
                remote_grasp_inference_required=False),remote_performance={},tabletop_candidates=(),
            request_invalidation_generation=revision,graspnet_input_config=input_config,
            model_choice='unknown_tabletop',ros_prepare_ms=max(0.,(time.perf_counter()-started)*1000.),
            near_field=False,near_field_phase_id=self._near_field_phase_id,
            target_observation=getattr(self,'_latest_target_observation',None))
        prepared = self._add_silhouette_envelope_diagnostic(prepared)
        self._require_diagnostic_observation_prepared(prepared)
        self._bind_diagnostic_observation_view_source(prepared, captured_view_source)
        return prepared

    def _accept_diagnostic_observation_prediction(self, prepared):
        original = self._observe_return_original
        self._require_diagnostic_observation_prepared(prepared)
        source = self._independent_observation_source()
        # Failure diagnostics need no newly activated metric geometry. Check
        # the original station budget before and during its existing READY
        # wait; successful work still passes the original activation unchanged.
        progress_context = self._station_observation_search_context(prepared)
        previous_activation = getattr(self, '_station_observation_activation_context', None)
        self._station_observation_activation_context = progress_context
        try:
            if progress_context is not None:
                self._station_observation_search_checkpoint(progress_context, 'before_geometry_activation')
            if not self._activate_prepared_geometry(prepared):
                raise original.StreamResultCancelled('GEOMETRY_STALE')
        except _StationObservationBudgetReached:
            runtime = dict(station_observation_search=progress_context, observation_variants=())
            return self._publish_station_observation_search_exhausted(prepared, runtime)
        finally:
            self._station_observation_activation_context = previous_activation
        identity = prepared.snapshot.target_identity
        seed = ObservationPlanningSeed(prepared.ticket.request_id,prepared.ticket.generation,
            identity.epoch,identity.track_id,prepared.snapshot.stamp_ns,prepared.snapshot.stamp_sec,
            str(getattr(prepared.snapshot.object_msg,'label','') or ''),
            candidate_source=source, source_lineage=(source,))
        reference = self._frozen_observation_reference_pose(prepared)
        transform = original.pose_matrix(reference)
        previous_context = getattr(self, '_station_observation_generation_context', None)
        self._station_observation_generation_context = progress_context
        generation_started = time.monotonic()
        try:
            if progress_context is not None:
                self._station_observation_search_checkpoint(progress_context, 'before_candidate_generation')
            passing,rejected = self._far_field_observation_variants(reference,prepared.geometry,
                transform[:3,2],prepared.snapshot,reference,prepared)
        except _StationObservationBudgetReached:
            runtime = dict(station_observation_search=progress_context, observation_variants=())
            return self._publish_station_observation_search_exhausted(prepared,runtime)
        finally:
            self._station_observation_generation_context = previous_context
        if progress_context is not None:
            original.rospy.loginfo('observation station generation request=%s wall_sec=%.6f passing=%d rejected=%d',
                prepared.ticket.request_id, max(0.,time.monotonic()-generation_started),len(passing),len(rejected))
        runtime = dict(prepared=prepared,grasp_pose=reference,sequence=None,
            observation_variants=tuple(passing),soft_evidence={},diagnostic_observation_seed=seed,
            observation_only=True,contact_geometry_checked=False,observation_endpoint_rejections=deepcopy(rejected),
            diagnostic_observation_reference=dict(request_id=seed.request_id,generation=seed.generation,
                target_epoch=seed.target_epoch,target_track_id=seed.target_track_id,
                snapshot_stamp_ns=seed.snapshot_stamp_ns,
                base_from_tool=tuple(tuple(float(v) for v in row) for row in transform),
                transform_sha256=transform_fingerprint(transform)))
        if passing:
            self._activate_far_field_observation_variant(runtime,passing[0],0)
        with self._stream_condition:
            self._require_stream_ticket_current_locked(prepared.ticket)
            self._stable_variant_runtime = {(seed.track_id,seed.variant_index):runtime}
        checker = self._check_moveit_stable_candidate
        if source == MEASURED_SOURCE:
            # Retain the normal per-request comparison deadline and source
            # reserve. A preceding request cannot donate an old incumbent or
            # its expired comparison clock to this independent seed.
            checker, _continuing = self._far_field_observation_search()
        try:
            result = (checker(seed) if passing else
                original.MoveItResult(reachable=False,joint_path_cost=0.,joint_max_delta_rad=0.,
                    reason='no original endpoint-CAD and measured-information-gain observation passed',
                    failure_code=self._observation_contract_code('NO_SAFE_VIEW')))
        except _StationObservationBudgetReached:
            return self._publish_station_observation_search_exhausted(prepared,runtime)
        progress_context = runtime.get('station_observation_search')
        if (not result.reachable and progress_context is not None
                and progress_context['station_budget_exhausted']):
            return self._publish_station_observation_search_exhausted(prepared, runtime)
        self._require_diagnostic_observation_prepared(prepared)
        checked = bool(passing)
        selected = None
        if result.reachable:
            self._require_current_diagnostic_observation_solution(prepared)
            score=float(result.joint_path_cost)
            if not math.isfinite(score) or score < 0.:
                raise original.CandidateContractError('SILHOUETTE_OBSERVATION_CONTEXT_INVALID','invalid strict path cost')
            selected=replace(seed,moveit_result=result,final_score=score)
        selection=SimpleNamespace(selected=selected,checked=(selected or seed,) if checked else (),
            reachable=(selected,) if selected else (),configured_top_n=1,shortlist_count=int(bool(passing)),
            terminated_early=False,termination_reason='')
        local=dict(input_count=1,stage_counts=dict(locally_valid=self._funnel_stage(1,int(bool(passing)))),
            source_counts={source:dict(generated=1,locally_valid=int(bool(passing)),stable=0,preview=0,promoted=0)},
            rejection_counts={},snapshot_evidence=dict(source_stamp_ns=list(prepared.snapshot.sample_stamp_ns),
                unique_source_frames=len(prepared.snapshot.sample_stamp_ns),near_field=False),
            independent_camera_view_seed=True, independent_observation_source=source,
            grasp_tracker_used=False,contact_geometry_checked=False)
        moveit=dict(stage_counts=dict(moveit_reachable=self._funnel_stage(int(checked),int(bool(selected)))),
            rejection_counts={} if selected else {result.failure_code or self._observation_contract_code('NO_SAFE_VIEW'):1})
        if selected is None:
            runtime['diagnostic_observation_failure']=asdict(result)
            pending = result.failure_code == 'OBSERVATION_SEARCH_INCOMPLETE'
            status=(self._observation_contract_code('SEARCH_PENDING') if pending else
                    result.failure_code if result.failure_code == 'OBSERVATION_REFERENCE_UNAVAILABLE'
                    else self._observation_contract_code('NO_SAFE_VIEW'))
            funnel=self._merge_pipeline_funnel(local,moveit,stable_count=int(bool(passing)))
            self._finalize_streaming_gate_audit(prepared,selection,funnel,status,
                lifecycle_commit_callback=lambda:self._observe_execution_candidate_invalid(ticket=prepared.ticket))
            terminal=original.Grasp6DPlan();terminal.header=deepcopy(prepared.snapshot.object_msg.header)
            terminal.header.frame_id='base_link';terminal.valid=False
            terminal.candidate_source=('observation_search_pending' if pending else 'near_field_terminal')
            terminal.diagnostic=status+': '+str(result.reason)
            legacy=original.PoseArray();legacy.header=deepcopy(terminal.header)
            self._publish_preview_plan(terminal,legacy,ticket=prepared.ticket)
            # An incomplete search carries no executable authority and makes
            # no negative reachability claim. Keep accepting fresh snapshots;
            # the runner's original wait deadline still bounds this attempt.
            if not pending:
                self._observe_return_paused=True
            return dict(status=status,funnel=funnel)
        plan=original.Grasp6DPlan();plan.header=deepcopy(runtime['observation_sequence'].pregrasp.header)
        plan.header.stamp=prepared.stamp;plan.header.frame_id='base_link';plan.valid=True
        plan.poses=[deepcopy(runtime['observation_sequence'].pregrasp.pose)]
        plan.object_geometry=original.geometry_estimate_to_message(prepared.geometry,
            snapshot=prepared.snapshot,stamp=prepared.stamp,label=seed.target_label)
        plan.target_track_id=identity.track_id;plan.model_choice='unknown_tabletop'
        plan.candidate_source=source;plan.candidate_source_lineage=[source]
        plan.score=selected.final_score;plan.has_candidate_model_width=False
        plan.candidate_width_m=0.;plan.required_open_width_m=0.
        plan.refinement_status='NOT_EVALUATED';plan.fused_view_count=0
        plan.contact_geometry_evidence='';plan.diagnostic='FAR_FIELD_OBSERVATION_PLAN'
        plan.plan_id=original.compute_plan_id(plan)
        legacy=original.PoseArray();legacy.header=deepcopy(plan.header);legacy.poses=deepcopy(plan.poses)
        signature=self._preview_target_signature(seed)
        proposal=dict(rich_plan=plan,signature=signature,score=selected.final_score,ticket=prepared.ticket,
            expected_generation=prepared.request_invalidation_generation)
        metadata={key:proposal[key] for key in ('signature','score','expected_generation')}
        metadata['plan_id']=plan.plan_id
        runtime['diagnostic_observation_plan']=deepcopy(plan)
        self._require_current_diagnostic_observation_solution(prepared)
        self._publish_preview_plan(plan,legacy,ticket=prepared.ticket,proposal_metadata=metadata,
            commit_callback=lambda:setattr(self,'_latest_streaming_audit',dict(
                request_id=prepared.ticket.request_id,plan_id=plan.plan_id,candidate_source=source,
                contact_geometry_checked=False,strict_sequence_stages=['observation'])))
        funnel,_decision=self._finalize_promotion_transaction(prepared,selection,proposal,local,moveit,1,
            'PREVIEW_READY',None,stable_candidates=(seed,))
        return dict(status='PREVIEW_READY',funnel=funnel)

    def _require_current_diagnostic_observation_solution(self, prepared):
        """A completed search cannot renew its source or camera calibration."""
        self._require_diagnostic_observation_prepared(prepared)
        if not self._observation_source_reserve_available(prepared.ticket):
            raise self._observe_return_original.CandidateContractError(
                'OBSERVATION_SOURCE_RESERVE_REACHED',
                'observation solution lacks its original planning/publication reserve')
        try:
            self._require_diagnostic_observation_view_source(prepared)
        except (AttributeError, KeyError, TypeError, ValueError, OverflowError) as exc:
            raise self._observe_return_original.CandidateContractError(
                self._observation_contract_code('VIEW_SOURCE_INVALID'), str(exc)) from exc
        # Configuration reads may block; they do not freeze the source clock.
        if not self._observation_source_reserve_available(prepared.ticket):
            raise self._observe_return_original.CandidateContractError(
                'OBSERVATION_SOURCE_RESERVE_REACHED',
                'observation source reserve expired during final source validation')

    def _capture_promotion_token_locked(self, ticket, expected_generation):
        token = super()._capture_promotion_token_locked(ticket, expected_generation)
        if self._independent_observation_planning_active():
            runtime = getattr(self, '_stable_variant_runtime', {}).get((0, 0), {})
            prepared = runtime.get('prepared')
            if (prepared is None or prepared.ticket is not ticket
                    or runtime.get('observation_only') is not True):
                raise self._observe_return_original.CandidateContractError(
                    self._observation_contract_code('CONTEXT_INVALID'),
                    'observation promotion lacks its exact prepared request')
            self._require_current_diagnostic_observation_solution(prepared)
            # This is the existing in-process transaction token, not a wire
            # authority. Retain the exact request through audit I/O and every
            # original pre/post-publication token check.
            token['independent_observation_prepared'] = prepared
        return token

    def _try_promote_cached_preview_after_replan_serialized(self):
        """Reuse the original request transaction for cached observation plans.

        The generic cache path binds geometry only. An observation also owns
        a frozen RGB-D/camera source and its original publication reserve; a
        replan request cannot renew either by writing a new execution audit.
        """
        preview = getattr(self, 'latest_preview_rich_plan', None)
        if (getattr(preview, 'candidate_source', None) not in (SOURCE, MEASURED_SOURCE)
                and not self._independent_observation_planning_active()):
            return super()._try_promote_cached_preview_after_replan_serialized()
        original = self._observe_return_original
        ticket = None
        proposal = None
        previous_report = deepcopy(getattr(self, '_active_execution_audit_report', None))
        try:
            with self._stream_condition, self._geometry_state_guard():
                if bool(getattr(self, 'robot_execution_active', False)):
                    return original.PromotionDecision(False, 'EXECUTION_FROZEN',
                        'robot execution is active')
                proposal, reason = self._cached_preview_promotion_proposal_locked()
                if proposal is None:
                    return original.PromotionDecision(False, 'NO_CACHED_PREVIEW', reason)
                runtime = getattr(self, '_stable_variant_runtime', {}).get((0, 0), {})
                prepared = runtime.get('prepared')
                bound_plan = runtime.get('diagnostic_observation_plan')
                plan = proposal['rich_plan']
                try:
                    content_matches = (bound_plan is not None
                        and plan.plan_id == bound_plan.plan_id
                        and original.compute_plan_id(plan) == bound_plan.plan_id)
                except (AttributeError, TypeError, ValueError, OverflowError):
                    content_matches = False
                if (runtime.get('observation_only') is not True or prepared is None
                        or not content_matches
                        or plan.header.stamp != prepared.stamp):
                    raise original.CandidateContractError(
                        self._observation_contract_code('CONTEXT_INVALID'),
                        'cached observation lacks its exact original prepared solution')
                ticket = prepared.ticket
                self._require_current_diagnostic_observation_solution(prepared)
                decision, now_sec = self._decide_preview_promotion(
                    plan, proposal['signature'], proposal['score'])
                self._latest_promotion_decision = decision
                if not decision.promote:
                    return decision
                token = self._capture_promotion_token_locked(
                    ticket, proposal['expected_generation'])

            ready, reason = self._write_cached_preview_execution_audit(plan, decision)
            if not ready:
                actual = original.PromotionDecision(False, 'PLAN_AUDIT_NOT_READY',
                    reason or 'cached observation execution audit is not ready')
            else:
                # Includes checks after audit I/O, each ROS send, and before
                # authority commit, with the original source ticket intact.
                actual = self._publish_execution_with_token(plan,
                    proposal['signature'], proposal['score'], now_sec, ticket, token)
        except (original.CandidateContractError, original.StreamResultCancelled) as exc:
            actual = original.PromotionDecision(False, exc.code, str(exc))

        if not actual.promote and ticket is not None:
            report = deepcopy(getattr(self, '_active_execution_audit_report', None))
            reference = dict(getattr(self, '_latest_execution_audit_reference', {}) or {})
            if (isinstance(report, dict) and reference.get('report_sha256')
                    and report.get('plan_id') == proposal['rich_plan'].plan_id
                    and report.get('request_id') == ticket.request_id
                    and report.get('generation') == ticket.generation
                    and dict(report.get('outcome', {}) or {}).get('valid_plan')):
                # CAS compensation cannot overwrite a later audit. The existing
                # observation override refuses to restore revoked source data.
                self._compensate_execution_audit(report, reference, actual, ticket,
                    restore_report=previous_report)
        if actual.promote:
            actual = original.PromotionDecision(True, 'PROMOTED_CACHED_PREVIEW',
                'cached observation Preview promoted with its original source authority')
        if ticket is None or self._stream_ticket_still_current(ticket):
            with self._geometry_state_guard():
                self._latest_promotion_decision = actual
        return actual

    def _promotion_token_state(self, ticket, token):
        current, code = super()._promotion_token_state(ticket, token)
        if not current:
            return current, code
        prepared = token.get('independent_observation_prepared')
        if prepared is not None:
            original = self._observe_return_original
            try:
                if prepared.ticket is not ticket:
                    return False, 'GENERATION_STALE'
                self._require_current_diagnostic_observation_solution(prepared)
            except (original.CandidateContractError, original.StreamResultCancelled) as exc:
                return False, exc.code
        return True, ''

    def _recover_aborted_execution_publish(self, token, reason):
        prepared = token.get('independent_observation_prepared')
        if prepared is not None:
            original = self._observe_return_original
            try:
                self._require_current_diagnostic_observation_solution(prepared)
            except (original.CandidateContractError, original.StreamResultCancelled):
                # Geometry equality alone cannot revive an observation plan
                # whose original source time or camera contract was revoked.
                if self._clear_aborted_observation_authority(prepared.ticket):
                    self._publish_aborted_execution_tombstone(reason)
                return
        return super()._recover_aborted_execution_publish(token, reason)

    def _clear_aborted_observation_authority(self, ticket):
        """Retire only this idle request's authority; keep audit CAS intact."""
        original = self._observe_return_original
        try:
            with self._stream_condition, self._geometry_state_guard():
                self._require_stream_ticket_current_locked(ticket)
                if bool(getattr(self, 'robot_execution_active', False)):
                    return False
                owned = getattr(self, '_execution_authority_ticket', None)
                incoming = self._execution_authority_order_key(
                    self._execution_authority_record(ticket))
                if owned is not None and self._execution_authority_order_key(owned) > incoming:
                    return False
                self._promotion_controller().clear_execution()
                self.latest_rich_plan = None
                self.latest_plan = None
                self._execution_authority_ticket = None
                # The surrounding promotion transaction must still CAS-revoke
                # its provisional audit; clearing its reference here would
                # prevent that durable compensation.
                return True
        except original.StreamResultCancelled:
            return False

    def _compensate_execution_audit(self, report, reference, decision, ticket,
            funnel=None, restore_report=None):
        runtime = getattr(self, '_stable_variant_runtime', {}).get((0, 0), {})
        prepared = runtime.get('prepared')
        source_revoked = False
        if (ticket is not None and prepared is not None and prepared.ticket is ticket
                and runtime.get('observation_only') is True):
            original = self._observe_return_original
            try:
                self._require_current_diagnostic_observation_solution(prepared)
            except (original.CandidateContractError, original.StreamResultCancelled):
                source_revoked = True
                restore_report = None
        result = super()._compensate_execution_audit(
            report, reference, decision, ticket, funnel=funnel, restore_report=restore_report)
        if source_revoked and self._clear_aborted_observation_authority(ticket):
            self._publish_aborted_execution_tombstone(decision.code)
        return result

    def _build_diagnostic_observation_gate_audit(self,prepared,selection,funnel,status,
            base_report=None,promotion_decision=None,execution_authority=False):
        original=self._observe_return_original
        self._require_diagnostic_observation_prepared(prepared)
        source = self._independent_observation_source()
        runtime=self._stable_variant_runtime.get((0,0),{})
        selected=selection.selected
        evaluation=None;plan=runtime.get('diagnostic_observation_plan')
        if selected is not None:
            self._require_current_diagnostic_observation_solution(prepared)
            result=selected.moveit_result
            evaluation=dict(candidate_source=source,source_index=0,variant_index=0,source_lineage=[source],
                selected=True,candidate_role=self._independent_observation_role(),contact_geometry_checked=False,
                required_open_width_m=0.,opening_role='no_contact_width_or_gripper_action_claim',
                tracking=dict(track_id=0,hit_count=1,window_count=1,hit_request_ids=[prepared.ticket.request_id],
                    policy='current_fused_rgbd_snapshot_only_no_grasp_tracker'),
                lineage_binding=dict(source_request_id=prepared.ticket.request_id,
                    source_snapshot_stamp_sec=prepared.ticket.snapshot_stamp_sec,source_raw_candidate_index=0,
                    source_variant_index=0,evaluation_request_id=prepared.ticket.request_id,
                    evaluation_snapshot_stamp_sec=prepared.ticket.snapshot_stamp_sec,evaluation_variant_index=0),
                final_score=selected.final_score,moveit=asdict(result),
                execution_sequence=self._execution_sequence_audit(runtime['observation_sequence'],False),
                observation_view=deepcopy(runtime.get('observation_view',{})),
                observation_envelope=deepcopy(runtime.get('observation_envelope',{})),
                observation_orientation_search=deepcopy(runtime.get('observation_orientation_search',{})),
                moveit_input_joint_state=deepcopy(runtime.get('moveit_input_joint_state',{})))
            if source == MEASURED_SOURCE:
                measured = self._require_diagnostic_observation_view_source(prepared)
                evaluation['observation_source_binding'] = dict(measured['binding'])
                evaluation['observation_source_audit'] = original._thaw_request_evidence(measured['audit'])
        promotes=bool(execution_authority and promotion_decision is not None and promotion_decision.promote)
        promotion=(None if promotion_decision is None else dict(promote=bool(promotion_decision.promote),
            code=promotion_decision.code,reason=promotion_decision.reason))
        report=dict(report_version=3,mode='continuous_execution' if execution_authority else 'continuous_preview',
            request_id=prepared.ticket.request_id,generation=prepared.ticket.generation,
            target_epoch=prepared.ticket.target_epoch,snapshot_stamp_ns=prepared.snapshot.stamp_ns,
            snapshot_stamp_sec=prepared.ticket.snapshot_stamp_sec,grasp_mode_selection=deepcopy(self._mode_selection),
            execution_strategy='observe_return',plan_id=str(plan.plan_id) if plan is not None and selected is not None else '',
            replay_geometry=self._geometry_replay_audit(prepared.geometry),pipeline_funnel=deepcopy(funnel),
            remote_diagnostics=original._thaw_request_evidence(prepared.remote_diagnostics),
            selected=evaluation,stable_evaluations=[] if evaluation is None else [deepcopy(evaluation)],
            lineage=[] if evaluation is None else [deepcopy(evaluation)],rows=[] if evaluation is None else [deepcopy(evaluation)],
            promotion=promotion,outcome=dict(code='PLAN_READY' if promotes else status,
                reason=status,preview_valid=selected is not None,valid_plan=promotes),
            moveit_selection=dict(policy='FIRST_REACHABLE_BY_AUTHORITATIVE_RANK',
                shortlist_count=selection.shortlist_count,checked_count=len(selection.checked),
                reachable_count=len(selection.reachable),terminated_early=False,
                ranking_scope='qualified_checked_observation_views_only',global_time_optimality_claimed=False),
            diagnostic_observation_failure=deepcopy(runtime.get('diagnostic_observation_failure')))
        if selected is None:
            # Failed searches need the same per-view evidence as selected
            # searches. Copy existing request-local diagnostics only: do not
            # repeat source validation, query ROS, or grant plan authority.
            expected = dict(request_id=prepared.ticket.request_id,
                generation=prepared.ticket.generation, target_epoch=prepared.ticket.target_epoch,
                target_track_id=prepared.snapshot.target_identity.track_id,
                snapshot_stamp_ns=prepared.snapshot.stamp_ns)
            captured = getattr(self, '_diagnostic_observation_view_source', None)
            captured_prepared = getattr(self, '_diagnostic_observation_view_prepared', None)
            binding = (original._thaw_request_evidence(captured.get('binding', {}))
                if captured is not None else {})
            same_source = captured_prepared is prepared and binding == expected
            source_evidence = dict(available=captured is not None,
                prepared_identity_matches=captured_prepared is prepared,
                binding_matches_current_request=binding == expected,
                expected_binding=expected, captured_binding=binding,
                captured_audit=(original._thaw_request_evidence(captured.get('audit', {}))
                    if captured is not None else {}),
                source_validation_repeated=False, execution_authority=False)
            cache = getattr(self, '_joint_space_observation_cache', None)
            same_cache = (same_source and isinstance(cache, dict)
                and cache.get('prepared') is prepared and cache.get('source') is captured)
            fk_search = dict(available=bool(same_cache), execution_authority=False)
            if same_cache:
                fk_search.update(audit=deepcopy(cache.get('audit', {})),
                    used_sec=cache.get('used_sec'),
                    cached_inputs_validated=cache.get('key') is not None,
                    candidates=deepcopy(cache.get('candidates', ())))
            elif cache is not None:
                fk_search['reason'] = 'cache does not belong to this prepared source binding'
            candidates = []
            for index, variant in enumerate(runtime.get('observation_variants', ())):
                pose = variant['sequence'].pregrasp
                candidates.append(dict(candidate_index=index,
                    search_family=variant.get('search_family', 'camera_orientation_lattice'),
                    camera_roll_offset_deg=variant.get('camera_roll_offset_deg', 0.),
                    camera_tilt_offset_deg=variant.get('camera_tilt_offset_deg', 0.),
                    camera_target_distance_m=variant.get('camera_target_distance_m'),
                    preference_index=variant.get('preference_index'),
                    frame_id=pose.header.frame_id, source_stamp_ns=pose.header.stamp.to_nsec(),
                    base_from_tool=original.pose_matrix(pose).tolist(),
                    observation_view=deepcopy(variant.get('observation_view', {})),
                    observation_side_evidence=deepcopy(variant.get('observation_side_evidence', {})),
                    observation_envelope=(asdict(variant['observation_envelope'])
                        if variant.get('observation_envelope') is not None else None)))
            report['observation_failure_diagnostics'] = dict(execution_authority=False,
                source=source_evidence, joint_space_fk_search=fk_search,
                observation_candidates=candidates,
                observation_orientation_search=deepcopy(runtime.get('observation_orientation_search', {})),
                observation_endpoint_rejections=deepcopy(runtime.get('observation_endpoint_rejections', ())),
                unknown_observation_scheduling=deepcopy(runtime.get('unknown_observation_scheduling', {})),
                moveit_input_joint_state=deepcopy(runtime.get('moveit_input_joint_state', {})))
        self._observe_return_audit(report,prepared)
        if selected is not None:
            if report.get('observe_return', {}).get('phase') == 'supplemental_observation':
                error = supplemental_observation_audit_error(report,
                    session=self._observe_return_session,
                    pending_request=self._observe_return_pending_supplement,
                    now_ns=int(self._ros_source_clock_ns()))
            else:
                error=observe_return_audit_error(report)
            if error:
                raise original.CandidateContractError(self._observation_contract_code('AUDIT_INVALID'),error)
        summary=dict(status=status,pipeline_funnel=deepcopy(funnel),promotion=promotion)
        return report,summary,evaluation
