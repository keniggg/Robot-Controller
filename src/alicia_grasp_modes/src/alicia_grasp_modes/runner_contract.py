"""Audit-to-plan binding for a single mode-aware grasp attempt (no ROS I/O)."""
import math
import re

from alicia_flexible_grasp.grasp.gripper_geometry import CandidateGateResult
from alicia_grasp_modes.selection import parse_selection
from alicia_grasp_modes.observation_policy import observation_config


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


def direct_audit_error(report):
    """Require real four-stage contact checks; observation evidence cannot pass."""
    try:
        selected = report['selected']
        plan_id = report['plan_id']
        if (report.get('execution_strategy') != 'direct'
                or report.get('planning_origin') != 'current_view_contact_snapshot'
                or report.get('observation_motion_performed') is not False):
            return 'audit is not a current-view direct contact request'
        if (report['outcome'].get('preview_valid') is not True
                or selected.get('selected') is not True
                or not isinstance(plan_id, str) or not re.fullmatch('[0-9a-f]{24}', plan_id)):
            return 'audit has no valid uniquely selected contact plan'
        selection = report['moveit_selection']
        if (selection.get('policy') != 'BOUNDED_APERTURE_THEN_HARDWARE_DURATION'
                or int(selection.get('checked_count', 0)) < 1
                or int(selection.get('reachable_count', 0)) < 1):
            return 'direct audit lacks checked reachable contact candidates'
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
        if preview.diagnostic != expected_phase or len(preview.poses) != 4:
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
    config = observation_config({}, dict(mode="unknown", strategy="two_stage"))
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
    try:
        distance = float(view["actual_camera_target_distance_m"])
        minimum = float(view["min_camera_target_distance_m"])
        maximum = float(view["max_camera_target_distance_m"])
    except (KeyError, TypeError, ValueError, OverflowError):
        return "camera-target distance evidence is malformed"
    if (
        not all(math.isfinite(value) for value in (distance, minimum, maximum))
        or minimum != config["observation_camera_target_min_distance_m"]
        or maximum != config["observation_camera_target_max_distance_m"]
        # Rigid-transform arithmetic can move a boundary by one ULP.
        # This picometre tolerance is numerical, not a motion margin.
        or distance < minimum - 1.0e-12
        or distance > maximum + 1.0e-12
    ):
        return "camera-target distance differs from the unknown observation policy"

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

