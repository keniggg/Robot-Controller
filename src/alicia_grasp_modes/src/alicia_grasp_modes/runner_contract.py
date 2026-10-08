"""Audit-to-plan binding for a single mode-aware grasp attempt (no ROS I/O)."""
import hashlib
import json
import math
import re

from alicia_flexible_grasp.grasp.gripper_geometry import CandidateGateResult
from alicia_flexible_grasp.grasp.rich_plan_integrity import (
    is_silhouette_observation_plan, validate_silhouette_observation_plan)
from alicia_grasp_modes.selection import parse_selection
from alicia_grasp_modes.observation_policy import WORKING_VOLUME_POLICY
from alicia_grasp_modes.silhouette_observation import validate_silhouette_ambiguity


def aligned_unknown_strategy(selection, requested_strategy=''):
    """Keep this operator's recorded unknown run on the primary/auxiliary policy.

    This validates an idle handoff, never changes a selection or grants motion.
    Legacy explicit modes remain available in the UI for separate experiments;
    an environment variable must not silently change this recorded workflow.
    """
    selected = parse_selection(selection)
    strategy = selected['strategy']
    if selected['mode'] == 'unknown' and strategy != 'observe_return':
        raise ValueError('Recorded unknown grasp requires observe_return: initial '
                         'contact first, supplemental observation only for missing evidence')
    if requested_strategy and requested_strategy != strategy:
        raise ValueError('Recorded handoff cannot override the selected grasp strategy')
    return strategy


def selection_audit_error(report, selection, minimum_stamp_ns):
    try:
        if parse_selection(report.get('grasp_mode_selection')) != selection:
            return 'audit belongs to a different mode, strategy or generation'
        stamp = report['snapshot_stamp_ns']
        if type(stamp) is not int or stamp < minimum_stamp_ns or stamp <= selection['stamp_ns']:
            return 'audit source predates this attempt or selection'
    except (AttributeError, KeyError, TypeError, ValueError):
        return 'audit selection/source binding is missing or malformed'
    return ''



def unknown_target_loss_reason(status, selection, now_ns):
    """Recognize an irreversible loss of this selection, never grant motion.

    The unknown tracker cannot recover without a new generation. A latched
    loss from this same selection remains applicable even when its observation
    predates the runner; old generations and malformed records are ignored.
    """
    try:
        selection = parse_selection(selection)
        if (selection['mode'] != 'unknown' or status.get('mode') != 'unknown'
                or status.get('state') != 'unavailable'
                or status.get('target_locked') is not True
                or status.get('target_lost') is not True):
            return ''
        loss = status['first_loss']
        for record in (status, loss):
            if (type(record.get('generation')) is not int
                    or record['generation'] != selection['generation']
                    or type(record.get('selection_stamp_ns')) is not int
                    or record['selection_stamp_ns'] != selection['stamp_ns']):
                return ''
        observed = loss['observed_stamp_ns']
        anchor = loss['anchor_source_stamp_ns']
        if (type(now_ns) is not int or type(observed) is not int or type(anchor) is not int
                or not selection['stamp_ns'] < anchor <= observed <= now_ns
                or status.get('anchor_source_stamp_ns') != anchor
                or loss.get('diagnostic_only') is not True
                or loss.get('target_recovered') is not False):
            return ''
        reason = loss['reason']
        if not isinstance(reason, str) or not reason.strip():
            return ''
        return 'TARGET_LOST: %s (generation=%d observed_stamp_ns=%d)' % (
            reason, selection['generation'], observed)
    except (AttributeError, KeyError, TypeError, ValueError):
        return ''


def direct_audit_error(report):
    """Require the original direct provenance plus complete contact checks."""
    if (report.get('execution_strategy') != 'direct'
            or report.get('planning_origin') != 'current_view_contact_snapshot'
            or report.get('observation_motion_performed') is not False):
        return 'audit is not a current-view direct contact request'
    return _contact_plan_audit_error(report)



def _silhouette_collision_envelope_error(report, selection, metadata, proof):
    envelope = metadata['collision_envelope']
    if (envelope.get('version') != 1
            or envelope.get('code') != 'SILHOUETTE_OBSERVATION_CONSERVATIVE_ENVELOPE'
            or envelope.get('diagnostic_observation_only') is not True
            or envelope.get('executable_contact_geometry') is not False
            or envelope.get('motion_authorized') is not False
            or envelope.get('selection') != selection
            or envelope.get('target_epoch') != metadata['target_epoch']
            or envelope.get('target_track_id') != metadata['target_track_id']
            or envelope.get('original_request_id') != proof['request_id']
            or envelope.get('original_generation') != proof['generation']
            or envelope.get('fresh_request_id') != report['request_id']
            or envelope.get('fresh_generation') != report['generation']
            or envelope['source']['original_stamp_ns'] != proof['snapshot_stamp_ns']
            or envelope['source']['fresh_stamp_ns'] != report['snapshot_stamp_ns']
            or envelope['source']['fresh_stamp_ns'] <= envelope['source']['original_stamp_ns']
            or envelope.get('perception_source_stamp_ns') != report['snapshot_stamp_ns']):
        return 'silhouette collision envelope source binding is invalid'
    for name in ('original', 'fresh', 'envelope'):
        record = dict(envelope[name]); digest = record.pop('geometry_sha256')
        actual = hashlib.sha256(json.dumps(record, sort_keys=True,
            separators=(',', ':'), allow_nan=False).encode()).hexdigest()
        if digest != actual:
            return 'silhouette collision envelope geometry digest is invalid'
    containment = envelope['containment']
    if (containment.get('original_corners_preserved') is not True
            or containment.get('fresh_corners_preserved') is not True
            or containment.get('original_measured_points_preserved') is not True
            or containment.get('fresh_measured_points_preserved') is not True
            or not math.isfinite(float(containment['maximum_measured_point_excess_m']))
            or float(containment['maximum_measured_point_excess_m']) > 1e-12
            or not math.isfinite(float(containment['maximum_corner_excess_m']))
            or float(containment['maximum_corner_excess_m']) > 1e-12):
        return 'silhouette collision envelope did not retain both occupied boxes'
    points = envelope['measured_points']; geometry = envelope['envelope']; fresh = envelope['fresh']
    replay = report['replay_geometry']
    if (points.get('source') != 'fresh_measurement_only'
            or type(points.get('synthetic_points_added')) is not int or points['synthetic_points_added'] != 0
            or type(points.get('count')) is not int or points['count'] <= 0
            or points['count'] != fresh['object_points_count']
            or points['sha256'] != fresh['object_points_sha256']
            or points['count'] != replay['object_points_count']
            or points['sha256'] != replay['object_points_sha256']
            or points['count'] != geometry['object_points_count']
            or points['sha256'] != geometry['object_points_sha256']
            or geometry['center_base_m'] != replay['obb_center_base_m']
            or geometry['axes_base'] != replay['R_base_obb']
            or geometry['size_xyz_m'] != replay['obb_size_xyz_m']
            or geometry['support_plane_base'] != replay['support_normal_base']+[replay['support_offset_m']]):
        return 'selected observation did not use its conservative collision envelope'
    support = envelope['support_consistency']
    if (support.get('perception_fixed_anchor_gate_replaced') is not False
            or support.get('footprint_kind') != 'projected_expanded_collision_envelope_not_measured_support'
            or not 0. < float(support['maximum_local_shift_m']) <= .004
            or not 0. < float(support['maximum_normal_change_deg']) <= 4.
            or not 0. <= float(support['support_reference_max_distance_m']) <= float(support['maximum_local_shift_m'])+1e-12
            or not 0. <= float(support['support_reference_angle_deg']) <= float(support['maximum_normal_change_deg'])+1e-9):
        return 'diagnostic observation changed the support consistency contract'
    source_status = metadata['trigger_evidence']['silhouette_ambiguity']['perception_source_status']
    if any(envelope['perception_reference'][key] != source_status[key] for key in
           ('anchor_source_stamp_ns','anchor_position_base','reference_support_plane_base',
            'reference_support_footprint_base')):
        return 'diagnostic observation changed the perception support reference'
    return ''


def _silhouette_observation_audit_error(report, selection, metadata):
    """Diagnosis is a one-stage observation authority, never contact authority."""
    try:
        if (metadata.get('diagnostic_observation_only') is not True
                or type(metadata.get('maximum_diagnostic_observations')) is not int
                or metadata['maximum_diagnostic_observations'] != 1
                or metadata.get('diagnostic_observation_attempted') is not True
                or metadata.get('contact_and_return_authorized') is not False):
            return 'silhouette observation lacks an explicit one-observation-only contract'
        trigger = metadata['trigger_evidence']
        if (trigger.get('policy') != 'current_request_silhouette_initialization_ambiguity'
                or trigger.get('diagnostic_observation_only') is not True
                or trigger.get('contact_or_motion_authorized') is not False):
            return 'silhouette trigger is not diagnostic evidence'
        proof = trigger['initial_contact_search']
        for name in ('request_id', 'generation', 'target_epoch', 'snapshot_stamp_ns',
                     'checked_count', 'shortlist_count', 'reachable_count'):
            if type(proof[name]) is not int or proof[name] < 0:
                return 'silhouette initial contact proof contains invalid counters'
        if (proof['request_id'] <= 0
                or proof['snapshot_stamp_ns'] != metadata['initial_surface_stamp_ns']
                or proof['target_epoch'] != metadata['target_epoch']
                or proof.get('selected') is not False
                or proof.get('terminated_early') is not False
                or proof.get('contact_or_motion_authorized') is not False
                or proof['reachable_count'] != 0
                or proof['checked_count'] != proof['shortlist_count']):
            return 'silhouette initial contact search is not completed and source-bound'
        local = proof['locally_valid']
        if (any(type(local[name]) is not int or local[name] < 0
                for name in ('entered', 'passed', 'rejected'))
                or local['entered'] != local['passed'] + local['rejected']):
            return 'silhouette local candidate checks are incomplete'
        code = proof['contact_search_code']
        if code == 'NEAR_FIELD_NO_HARD_SAFE_CANDIDATE':
            if proof['checked_count'] != 0 or local['passed'] != 0:
                return 'silhouette local rejection proof disagrees with candidate counts'
        elif code != 'NEAR_FIELD_NO_REACHABLE_CANDIDATE' or proof['checked_count'] <= 0:
            return 'silhouette trigger is not a completed initial contact rejection'
        validate_silhouette_ambiguity(trigger['silhouette_ambiguity'],
            selection=selection, request_id=proof['request_id'], generation=proof['generation'],
            target_epoch=metadata['target_epoch'], snapshot_stamp_ns=proof['snapshot_stamp_ns'],
            target_track_id=metadata['target_track_id'])
        collision_error = _silhouette_collision_envelope_error(report, selection, metadata, proof)
        if collision_error:
            return collision_error
        gain = report['selected']['observation_view']['observe_return_silhouette_gain']
        if (gain.get('ok') is not True
                or gain.get('code') != 'SILHOUETTE_VIEW_POTENTIAL_GAIN'
                or gain.get('source_stamp_ns') != proof['snapshot_stamp_ns']
                or gain.get('target_epoch') != metadata['target_epoch']
                or gain.get('target_track_id') != metadata['target_track_id']
                or gain.get('diagnostic_observation_only') is not True
                or gain.get('prediction_only') is not True
                or gain.get('measured_registration_required') is not True
                or gain.get('physical_contour_certified') is not False
                or gain.get('contact_or_motion_authorized') is not False):
            return 'silhouette observation has no bound predicted information gain'
        values = [gain[name] for name in ('predicted_edge_exposure_m', 'initial_edge_exposure_m',
            'predicted_gain_m', 'depth_uncertainty_m', 'camera_translation_baseline_m')]
        if (any(type(value) not in (int, float) or not math.isfinite(value) or value < 0.
                for value in values)
                or abs(values[0] - values[1] - values[2]) > 1e-9
                or values[2] <= values[3] + 1e-6
                or values[3] < trigger['silhouette_ambiguity']['view_geometry']['depth_uncertainty_m']
                or values[4] < .015):
            return 'silhouette observation predicted gain is inconsistent or insufficient'
        return ''
    except (AttributeError, KeyError, TypeError, ValueError, OverflowError):
        return 'silhouette observation evidence is incomplete or malformed'


def observe_return_audit_error(report, *, allow_returned=False):
    """Admit an explicit observation prefix or a fully checked contact plan.

    Returned contact is continuation evidence only. External single-attempt
    runners use the default, while the task must additionally bind an allowed
    returned report to its own observation/return session.
    """
    try:
        selection = parse_selection(report['grasp_mode_selection'])
        if (selection['mode'] != 'unknown' or selection['strategy'] != 'observe_return'
                or report.get('execution_strategy') != 'observe_return'):
            return 'audit is not an unknown observe_return request'
        metadata = report['observe_return']
        phase = metadata['phase']
        stamp = report['snapshot_stamp_ns']
        origin = metadata['origin_snapshot_stamp_ns']
        initial = metadata['initial_surface_stamp_ns']
        epoch = metadata['target_epoch']
        track = metadata['target_track_id']
        if (type(stamp) is not int or type(origin) is not int or type(initial) is not int
                or not selection['stamp_ns'] < initial <= origin <= stamp
                or type(epoch) is not int or epoch < 0
                or type(report.get('target_epoch')) is not int
                or report['target_epoch'] != epoch
                or not isinstance(track, str) or not track.strip() or track != track.strip()):
            return 'observe_return source or target identity binding is invalid'
        if (not isinstance(report.get('plan_id'), str)
                or not re.fullmatch('[0-9a-f]{24}', report['plan_id'])):
            return 'observe_return plan_id is invalid'
        diagnostic = metadata.get('diagnostic_observation_only', False)
        trigger = metadata.get('trigger_code')
        if (type(diagnostic) is not bool
                or ((diagnostic or trigger == 'RGB_DEPTH_SILHOUETTE_AMBIGUOUS')
                    and phase != 'observation_prefix')):
            return 'diagnostic observation cannot authorize contact or return'
        if phase == 'initial_contact':
            if (report.get('planning_origin') != 'current_view_contact_snapshot'
                    or report.get('observation_motion_performed') is not False
                    or origin != stamp):
                return 'observe_return initial contact provenance is invalid'
            return _contact_plan_audit_error(report)
        if phase == 'observation_prefix':
            if (report.get('planning_origin') != 'observe_return_observation_prefix'
                    or report.get('observation_motion_performed') is not False
                    or origin != stamp
                    or trigger not in ('BILATERAL_SURFACE_EVIDENCE_MISSING',
                                       'RGB_DEPTH_SILHOUETTE_AMBIGUOUS')):
                return 'observe_return observation prefix lacks bound view-insufficiency evidence'
            if trigger == 'RGB_DEPTH_SILHOUETTE_AMBIGUOUS':
                error = _silhouette_observation_audit_error(report, selection, metadata)
                if error:
                    return error
            elif diagnostic:
                return 'diagnostic observation flag does not match its trigger'
            return unknown_observation_audit_error(report)
        if phase != 'returned_contact' or allow_returned is not True:
            return 'observe_return phase is not an initial attempt entry'
        if (report.get('planning_origin') != 'observed_return_contact_snapshot'
                or report.get('observation_motion_performed') is not True):
            return 'observe_return returned contact provenance is invalid'
        returned = metadata['return_verified_stamp_ns']
        deadline = metadata['deadline_ns']
        contact_deadline = metadata['contact_deadline_ns']
        session = metadata['session_id']
        origin_plan = metadata['origin_plan_id']
        sources = metadata['fused_source_stamps_ns']
        if (type(returned) is not int or type(deadline) is not int
                or type(contact_deadline) is not int
                or not origin < returned < stamp < contact_deadline <= deadline
                or not isinstance(session, str) or not session.strip()
                or not isinstance(origin_plan, str)
                or not re.fullmatch('[0-9a-f]{24}', origin_plan)):
            return 'observe_return returned contact lacks verified return/session evidence'
        if (not isinstance(sources, (list, tuple))
                or any(type(value) is not int or not selection['stamp_ns'] < value <= stamp
                       for value in sources)
                or initial not in sources or stamp not in sources
                or not any(origin < value < returned for value in sources)):
            return 'observe_return returned contact lacks registered initial, supplemental and return views'
        return _contact_plan_audit_error(report)
    except (AttributeError, KeyError, TypeError, ValueError, OverflowError):
        return 'observe_return audit is incomplete or malformed'


def _contact_plan_audit_error(report):
    """Physical contact proof shared by explicitly distinct strategy records."""
    try:
        selected = report['selected']
        plan_id = report['plan_id']
        if (report['outcome'].get('preview_valid') is not True
                or selected.get('selected') is not True
                or not isinstance(plan_id, str) or not re.fullmatch('[0-9a-f]{24}', plan_id)):
            return 'audit has no valid uniquely selected contact plan'
        selection = report['moveit_selection']
        policy = selection.get('policy')
        if (policy not in ('BOUNDED_APERTURE_THEN_HARDWARE_DURATION',
                           'CHECKED_HARDWARE_DURATION_THEN_JOINT_MOTION')
                or int(selection.get('checked_count', 0)) < 1
                or int(selection.get('reachable_count', 0)) < 1):
            return 'direct audit lacks checked reachable contact candidates'
        if policy == 'CHECKED_HARDWARE_DURATION_THEN_JOINT_MOTION' and (
                selection.get('ranking_order') != ['execution_duration_lower_bound_sec',
                    'joint_max_delta_rad', 'joint_path_cost', 'aperture_width_m']
                or selection.get('physical_aperture_and_gates_unchanged') is not True
                or selection.get('motion_metric') != 'strict_sequence_hardware_duration_lower_bound'
                or selection.get('optimality_scope') != 'checked_hard_safe_candidates_only'
                or selection.get('global_time_optimality_claimed') is not False):
            return 'contact duration-first audit lacks the checked-motion ranking contract'
        moveit = selected['moveit']
        if (any(moveit.get(name) is not True for name in (
                'reachable', 'collision_free', 'within_joint_limits', 'ik_valid', 'planning_success'))
                or moveit.get('failure_code')):
            return 'contact plan lacks complete final strict MoveIt evidence'
        if (selected.get('final_execution_sequence_checked') is not True
                or selected.get('final_strict_plan_id') != plan_id):
            return 'final strict sequence is not bound to this contact plan_id'
        sequence = selected['execution_sequence']
        stages = sequence['stages']
        if (sequence.get('available') is not True or sequence.get('kind') != 'near_field_contact'
                or [stage.get('stage') for stage in stages] != ['pregrasp', 'approach', 'grasp', 'lift']
                or [stage.get('linear_from_prior_state') for stage in stages] != [False, True, True, True]):
            return 'audit does not contain the checked four-stage contact sequence'
        if selected['final_geometry_gate'].get('ok') is not True:
            return 'final contact geometry is not explicitly valid'
        gate = CandidateGateResult(**selected['final_geometry_gate'])
        if not gate.ok or gate.required_open_width_m <= 0:
            return 'final contact geometry did not pass all original analytical gates'
        replay = report['replay_geometry']
        if (replay.get('available') is not True or int(replay.get('object_points_count', 0)) < 1
                or not re.fullmatch('[0-9a-f]{64}', str(replay.get('object_points_sha256', '')))):
            return 'direct audit has no request-bound measured target point cloud'
    except (AttributeError, KeyError, TypeError, ValueError, OverflowError):
        return 'direct contact audit is incomplete or malformed'
    return ''


def audit_matches_preview(report, preview, direct):
    """Compare every audited direct pose and exact source stamp to the message."""
    try:
        stamp = preview.header.stamp
        stamp_ns = int(stamp.secs) * 1000000000 + int(stamp.nsecs)
        if report['snapshot_stamp_ns'] != stamp_ns or report['plan_id'] != preview.plan_id:
            return False
        expected_phase = 'CONTACT_EXECUTION_PLAN' if direct else 'FAR_FIELD_OBSERVATION_PLAN'
        observation_source = is_silhouette_observation_plan(preview)
        if observation_source:
            validate_silhouette_observation_plan(preview)
            metadata = report.get('observe_return', {})
            selected = report.get('selected', {})
            if (direct or metadata.get('diagnostic_observation_only') is not True
                    or metadata.get('trigger_code') != 'RGB_DEPTH_SILHOUETTE_AMBIGUOUS'
                    or selected.get('candidate_source') != 'silhouette_observation'
                    or selected.get('source_lineage') != ['silhouette_observation']
                    or selected.get('candidate_role') != 'diagnostic_observation'
                    or selected.get('contact_geometry_checked') is not False
                    or selected.get('required_open_width_m') != 0.):
                return False
        expected_poses = 1 if observation_source else 4
        if preview.diagnostic != expected_phase or len(preview.poses) != expected_poses:
            return False
        stages = report['selected']['execution_sequence']['stages']
        if len(stages) != (4 if direct else 1):
            return False
        for stage, pose in zip(stages, preview.poses):
            position = tuple(float(value) for value in stage['position_m'])
            quaternion = tuple(float(value) for value in stage['quaternion_xyzw'])
            if (len(position) != 3 or len(quaternion) != 4
                    or stage['frame_id'] != preview.header.frame_id):
                return False
            observed_position = tuple(float(getattr(pose.position, key)) for key in ('x', 'y', 'z'))
            observed_quaternion = tuple(float(getattr(pose.orientation, key)) for key in ('x', 'y', 'z', 'w'))
            if not all(math.isfinite(value) for value in position + quaternion + observed_position + observed_quaternion):
                return False
            if max(abs(a-b) for a, b in zip(position, observed_position)) > 1e-9:
                return False
            if min(max(abs(a-b) for a, b in zip(quaternion, observed_quaternion)),
                   max(abs(a+b) for a, b in zip(quaternion, observed_quaternion))) > 1e-9:
                return False
        if direct:
            required = float(report['selected']['final_geometry_gate']['required_open_width_m'])
            if not math.isclose(required, float(preview.required_open_width_m), rel_tol=1e-6, abs_tol=1e-9):
                return False
        return True
    except (AttributeError, KeyError, IndexError, TypeError, ValueError, OverflowError):
        return False


def unknown_observation_audit_error(report):
    """Original observation audit gates with the bound unknown view policy."""
    selection = dict(report.get("moveit_selection", {}) or {})
    selected = report.get("selected")
    replay = dict(report.get("replay_geometry", {}) or {})
    outcome = dict(report.get("outcome", {}) or {})
    if outcome.get("preview_valid") is not True:
        return "audit does not authorize a valid preview"
    if not isinstance(selected, dict) or selected.get("selected") is not True:
        return "audit has no unique selected lineage"
    if selection.get("policy") != "FIRST_REACHABLE_BY_AUTHORITATIVE_RANK":
        return "far-field MoveIt selection policy is not authoritative"
    if int(selection.get("checked_count", 0)) < 1:
        return "no strict MoveIt candidate was checked"
    if int(selection.get("reachable_count", 0)) != 1:
        return "far-field audit must bind exactly one reachable optimum"
    if selection.get("terminated_early") is True and (
        selection.get("termination_reason")
        != "FIRST_REACHABLE_BY_AUTHORITATIVE_RANK"
    ):
        return "early convergence reason is not bound"
    if replay.get("available") is not True:
        return "request-bound replay geometry is unavailable"
    if int(replay.get("object_points_count", 0)) < 1:
        return "request-bound target point cloud is empty"
    if len(str(replay.get("object_points_sha256", ""))) != 64:
        return "request-bound target point-cloud hash is invalid"

    moveit = dict(selected.get("moveit", {}) or {})
    if moveit.get("reachable") is not True:
        return "selected observation is not strictly reachable"
    hard_states = tuple(
        moveit.get(name)
        for name in (
            "collision_free",
            "within_joint_limits",
            "ik_valid",
            "planning_success",
        )
    )
    structured_success = (
        all(value is True for value in hard_states)
        and not str(moveit.get("failure_code", "") or "")
        and not str(moveit.get("evidence_code", "") or "")
    )
    strict_service_success = (
        all(value is None for value in hard_states)
        and not str(moveit.get("failure_code", "") or "")
        and moveit.get("evidence_code") == "STRICT_SERVICE_SUCCESS"
    )
    if not (structured_success or strict_service_success):
        return "selected observation lacks strict MoveIt evidence"

    delta = float(moveit.get("joint_max_delta_rad", float("nan")))
    if not math.isfinite(delta) or delta < 0.0:
        return "unknown observation has invalid joint motion evidence"

    sequence = dict(selected.get("execution_sequence", {}) or {})
    stages = sequence.get("stages")
    if (
        sequence.get("available") is not True
        or sequence.get("kind") != "far_field_observation"
        or not isinstance(stages, list)
        or len(stages) != 1
        or dict(stages[0]).get("stage") != "observation"
    ):
        return "audit does not bind exactly one observation execution stage"

    view = dict(selected.get("observation_view", {}) or {})
    working = dict(view.get('working_volume', {}) or {})
    try:
        def numeric(value):
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ValueError('nonfinite or nonnumeric working-volume evidence')
            return float(value)
        near = numeric(working['minimum_camera_depth_m'])
        far = numeric(working['maximum_camera_depth_m'])
        depths = [numeric(value) for value in working['target_corner_depths_m']]
        margin = numeric(working['required_image_margin_px'])
        edge = numeric(working['minimum_image_edge_distance_px'])
        radial = numeric(working['target_center_radial_distance_m'])
        distance = numeric(view['actual_camera_target_distance_m'])
        if (view.get('acceptance_policy') != WORKING_VOLUME_POLICY
                or view.get('distance_is_soft_preference') is not True
                or working.get('ok') is not True
                or working.get('evidence_kind') != 'planned_endpoint_geometry'
                or working.get('actual_arrival_measurement_required') is not True
                or working.get('kind') != 'planned_observation_working_volume'
                or working.get('camera_transform_source') != 'planned_camera_pose'
                or type(working.get('snapshot_stamp_ns')) is not int
                or working['snapshot_stamp_ns'] != report.get('snapshot_stamp_ns')
                or any(name in working for name in (
                    'source_stamp_ns', 'image_source_stamp_ns', 'camera_transform_stamp_ns',
                    'camera_transform_stamp_delta_ns', 'camera_transform_stamp_tolerance_ns',
                    'camera_transform_query_stamp_ns', 'visible_surface_stamp_ns'))
                or len(depths) != 8 or not .070 <= near < far <= .500
                or min(depths) < near or max(depths) > far
                or margin < 36. or edge < margin or distance <= 0.
                or not math.isclose(radial, distance, abs_tol=1e-9, rel_tol=0.)
                or numeric(working['minimum_depth_m']) != min(depths)
                or numeric(working['maximum_depth_m']) != max(depths)):
            return 'observation lacks snapshot-bound planned RGB-D working-volume evidence'
    except (KeyError, TypeError, ValueError, OverflowError):
        return 'RGB-D working-volume evidence is malformed'

    envelope = dict(selected.get("observation_envelope", {}) or {})
    try:
        clearance = float(envelope["minimum_support_clearance_m"])
    except (KeyError, TypeError, ValueError, OverflowError):
        return "observation support-clearance evidence is malformed"
    if (
        envelope.get("ok") is not True
        or not math.isfinite(clearance)
        or clearance < 0.003
    ):
        return "observation envelope does not preserve 3 mm support clearance"
    return ""

