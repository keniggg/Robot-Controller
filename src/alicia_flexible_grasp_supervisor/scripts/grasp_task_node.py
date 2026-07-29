#!/usr/bin/env python3
from copy import deepcopy
from dataclasses import dataclass
import hashlib
import io
import json
import math
import os
import tempfile
import threading
import time
import rospy
from geometry_msgs.msg import PoseArray, PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from alicia_flexible_grasp_supervisor.msg import Grasp6DPlan, ObjectPose, GraspState
from alicia_flexible_grasp_supervisor.srv import (
    CheckPoseSequence,
    SetFloat,
    SetTargetPose,
    StartGrasp,
    StartGraspResponse,
    StopGrasp,
    StopGraspResponse,
    TriggerZero,
    TriggerZeroResponse,
)
from alicia_flexible_grasp.grasp.grasp_state_machine import GraspStages, STATE_NAMES
from alicia_flexible_grasp.grasp.grasp_pose_generator import make_pregrasp_pose, make_lift_pose
from alicia_flexible_grasp.grasp.rich_plan_integrity import (
    compute_plan_id,
    plan_id_matches_content,
    required_open_width_is_valid,
    stamp_nanoseconds as _stamp_nanoseconds,
    stamp_seconds as _stamp_seconds,
    strict_plan_id_equal,
    validate_candidate_model_width,
    validate_finite_pose,
    validate_plan_header_binding,
    validate_rich_geometry,
)
from alicia_flexible_grasp.robot.planning_feedback import (
    is_orientation_fallback_message,
    is_position_only_fallback_message,
    orientation_fallback_rejection_message,
    position_only_rejection_message,
)
from alicia_flexible_grasp.vision.mujoco_digital_twin_client import (
    MujocoDigitalTwinClient,
    build_mujoco_payload,
    validate_mujoco_gate_response,
)
try:
    import tf2_ros
except Exception:
    tf2_ros = None


@dataclass(frozen=True)
class PlanValidationResult:
    ok: bool
    code: str = ''
    reason: str = ''
    age_sec: float = float('inf')


_MUJOCO_AUDIT_SCHEMA_VERSION = 1
_MUJOCO_AUDIT_DEFAULT_PATH = '~/.ros/grasp6d_mujoco_audit_latest.json'
_FAR_FIELD_OBSERVATION_PLAN = 'FAR_FIELD_OBSERVATION_PLAN'
_CONTACT_EXECUTION_PLAN = 'CONTACT_EXECUTION_PLAN'
_MUJOCO_SAFETY_KEYS = (
    'simulation_ok',
    'ik_success',
    'collision_free',
    'contact_success',
    'lift_success',
)
_MUJOCO_AUDIT_TEXT_LIMIT = 2048
_MUJOCO_STATUS_TEXT_LIMIT = 320


class AuditPathConflictError(ValueError):
    pass


def _bounded_text(value, limit=_MUJOCO_AUDIT_TEXT_LIMIT):
    if not isinstance(value, str):
        return None
    maximum = max(0, int(limit))
    return value[:maximum]


def _plan_phase(plan):
    return str(getattr(plan, 'diagnostic', '') or '').strip()


def _strict_json_number(value):
    if isinstance(value, bool):
        return None
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError):
        return None
    return result if math.isfinite(result) else None


def _wall_time_sec():
    value = _strict_json_number(time.time())
    return 0.0 if value is None else value


def _exception_audit_record(exception):
    if exception is None:
        return None
    return {
        'type': _bounded_text(type(exception).__name__, 128),
        'message': _bounded_text(str(exception)),
    }


def _new_mujoco_execution_audit(plan):
    request_plan_id = getattr(plan, 'plan_id', None)
    return {
        'schema_version': _MUJOCO_AUDIT_SCHEMA_VERSION,
        'audit_kind': 'mujoco_rich_plan_execution_gate',
        'request_plan_id': _bounded_text(request_plan_id, 256),
        'attempt': {
            'started_unix_sec': _wall_time_sec(),
            'completed_unix_sec': None,
            'duration_sec': None,
        },
        'payload': {
            'built': False,
            'plan_id': None,
            'sha256': None,
            'summary': None,
            'build_error': None,
        },
        'response': {
            'received': False,
            'json_object': False,
            'strict_json_serializable': False,
            'sha256': None,
            'raw_echo_plan_id': None,
            'raw_echo_plan_id_type': None,
            'candidate_source': None,
            'candidate_source_lineage': None,
            'score': None,
            'failure_code': None,
            'failure_reason': None,
            'failure_reason_length': None,
            'diagnosis': None,
            'diagnosis_count': None,
            'lift_evidence': None,
            'used_joint_state_source': None,
            'network_error': None,
            **{key: None for key in _MUJOCO_SAFETY_KEYS},
        },
        'authority_after_network': {
            'checked': False,
            'ok': None,
            'code': None,
            'reason': None,
        },
        'gate_validation': {
            'checked': False,
            'ok': None,
            'code': None,
            'reason': None,
            'score': None,
        },
        'final_validation': {
            'ok': False,
            'code': None,
            'reason': None,
            'score': None,
            'completed_unix_sec': None,
        },
    }


def _record_mujoco_payload(audit, payload):
    encoded = json.dumps(
        payload,
        allow_nan=False,
        ensure_ascii=True,
        separators=(',', ':'),
        sort_keys=True,
    ).encode('utf-8')
    trajectory = payload.get('trajectory') if isinstance(payload, dict) else None
    joint_names = payload.get('joint_names') if isinstance(payload, dict) else None
    audit['payload'] = {
        'built': True,
        'plan_id': _bounded_text(payload.get('plan_id'), 256),
        'sha256': hashlib.sha256(encoded).hexdigest(),
        'summary': {
            'schema_version': payload.get('schema_version'),
            'snapshot_stamp_sec': _strict_json_number(
                payload.get('snapshot_stamp_sec')
            ),
            'model_choice': _bounded_text(payload.get('model_choice'), 256),
            'candidate_source': _bounded_text(
                payload.get('candidate_source'), 64
            ),
            'candidate_source_lineage': (
                list(payload.get('candidate_source_lineage'))
                if isinstance(payload.get('candidate_source_lineage'), list)
                else None
            ),
            'joint_count': len(joint_names) if isinstance(joint_names, list) else None,
            'trajectory_count': len(trajectory) if isinstance(trajectory, list) else None,
            'candidate_width_m': _strict_json_number(
                payload.get('candidate_width_m')
            ),
            'required_open_width_m': _strict_json_number(
                payload.get('required_open_width_m')
            ),
            'gripper_model_name': _bounded_text(
                (payload.get('gripper') or {}).get('model_name')
                if isinstance(payload.get('gripper'), dict)
                else None,
                256,
            ),
        },
        'build_error': None,
    }


def _record_mujoco_response(audit, response):
    record = audit['response']
    record['received'] = True
    record['json_object'] = isinstance(response, dict)
    try:
        encoded = json.dumps(
            response,
            allow_nan=False,
            ensure_ascii=True,
            separators=(',', ':'),
            sort_keys=True,
        ).encode('utf-8')
    except (TypeError, ValueError, OverflowError):
        encoded = None
    if encoded is not None:
        record['strict_json_serializable'] = True
        record['sha256'] = hashlib.sha256(encoded).hexdigest()
    if not isinstance(response, dict):
        record['raw_echo_plan_id_type'] = type(response).__name__
        return

    echoed_id = response.get('plan_id')
    record['raw_echo_plan_id'] = echoed_id if isinstance(echoed_id, str) else None
    record['raw_echo_plan_id_type'] = type(echoed_id).__name__
    raw_source = response.get('candidate_source')
    record['candidate_source'] = (
        raw_source if isinstance(raw_source, str) else None
    )
    raw_lineage = response.get('candidate_source_lineage')
    record['candidate_source_lineage'] = (
        list(raw_lineage)
        if isinstance(raw_lineage, list)
        and all(isinstance(item, str) for item in raw_lineage)
        else None
    )
    record['score'] = _strict_json_number(response.get('score'))
    raw_code = response.get('failure_code')
    record['failure_code'] = raw_code if isinstance(raw_code, str) else None
    raw_reason = response.get('failure_reason')
    record['failure_reason'] = raw_reason if isinstance(raw_reason, str) else None
    record['failure_reason_length'] = (
        len(raw_reason) if isinstance(raw_reason, str) else None
    )
    raw_diagnosis = response.get('diagnosis')
    if (
        isinstance(raw_diagnosis, list)
        and all(isinstance(item, str) for item in raw_diagnosis)
    ):
        record['diagnosis'] = [
            _bounded_text(item, 512) for item in raw_diagnosis[:20]
        ]
        record['diagnosis_count'] = len(raw_diagnosis)
    raw_joint_source = response.get('used_joint_state_source')
    record['used_joint_state_source'] = (
        _bounded_text(raw_joint_source, 128)
        if isinstance(raw_joint_source, str)
        else None
    )
    raw_lift_evidence = response.get('lift_evidence')
    if isinstance(raw_lift_evidence, dict):
        try:
            json.dumps(raw_lift_evidence, allow_nan=False)
        except (TypeError, ValueError, OverflowError):
            raw_lift_evidence = None
        if raw_lift_evidence is not None:
            record['lift_evidence'] = deepcopy(raw_lift_evidence)
    for key in _MUJOCO_SAFETY_KEYS:
        value = response.get(key)
        record[key] = value if type(value) is bool else None


def _validation_audit_record(result, checked=True):
    return {
        'checked': bool(checked),
        'ok': bool(getattr(result, 'ok', False)) if checked else None,
        'code': _bounded_text(str(getattr(result, 'code', '') or ''), 256)
        if checked else None,
        'reason': _bounded_text(str(getattr(result, 'reason', '') or ''))
        if checked else None,
        'score': _strict_json_number(getattr(result, 'score', None))
        if checked else None,
    }


def normalize_audit_output_path(output_path, label='audit_output_path'):
    if not isinstance(output_path, str) or not output_path.strip():
        raise ValueError('%s must be a non-empty string' % str(label))
    return os.path.normcase(
        os.path.realpath(
            os.path.abspath(os.path.expanduser(output_path.strip()))
        )
    )


def validate_distinct_audit_output_paths(
    mujoco_output_path,
    planning_output_path,
):
    """Return canonical audit paths and reject cross-gate file aliasing."""
    mujoco_path = normalize_audit_output_path(
        mujoco_output_path,
        'MuJoCo audit_output_path',
    )
    if planning_output_path is None or (
        isinstance(planning_output_path, str)
        and not planning_output_path.strip()
    ):
        return mujoco_path, None
    planning_path = normalize_audit_output_path(
        planning_output_path,
        'planning gate_audit_output_path',
    )
    same_path = planning_path == mujoco_path
    if not same_path and os.path.exists(planning_path) and os.path.exists(mujoco_path):
        try:
            same_path = os.path.samefile(planning_path, mujoco_path)
        except OSError:
            same_path = False
    if same_path:
        raise AuditPathConflictError(
            'MuJoCo and planning gate audits must use distinct canonical paths'
        )
    return mujoco_path, planning_path


def write_mujoco_execution_audit(output_path, report):
    """Atomically persist one strict-JSON MuJoCo execution-gate attempt."""
    path = normalize_audit_output_path(
        output_path,
        'MuJoCo audit_output_path',
    )
    directory = os.path.dirname(path) or os.curdir
    payload = json.dumps(
        report,
        allow_nan=False,
        ensure_ascii=True,
        indent=2,
        sort_keys=True,
    ).encode('utf-8')
    os.makedirs(directory, exist_ok=True)
    descriptor, temporary_path = tempfile.mkstemp(
        prefix='.%s.tmp-' % os.path.basename(path),
        dir=directory,
    )
    replaced = False
    try:
        with os.fdopen(descriptor, 'wb') as handle:
            descriptor = -1
            handle.write(payload)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary_path, path)
        replaced = True
        directory_descriptor = os.open(
            directory,
            os.O_RDONLY | getattr(os, 'O_DIRECTORY', 0),
        )
        try:
            os.fsync(directory_descriptor)
        finally:
            os.close(directory_descriptor)
    finally:
        if descriptor >= 0:
            os.close(descriptor)
        if not replaced:
            try:
                os.unlink(temporary_path)
            except FileNotFoundError:
                pass
    return {
        'path': path,
        'sha256': hashlib.sha256(payload).hexdigest(),
        'bytes': len(payload),
    }


def _finalize_mujoco_execution_audit(
    audit,
    output_path,
    ok,
    code,
    reason,
    score=None,
):
    completed = _wall_time_sec()
    started = _strict_json_number(audit['attempt'].get('started_unix_sec'))
    audit['attempt']['completed_unix_sec'] = completed
    audit['attempt']['duration_sec'] = (
        max(0.0, completed - started) if started is not None else None
    )
    audit['final_validation'] = {
        'ok': bool(ok),
        'code': _bounded_text(str(code or ''), 256),
        'reason': _bounded_text(str(reason or '')),
        'score': _strict_json_number(score),
        'completed_unix_sec': completed,
    }
    return write_mujoco_execution_audit(output_path, audit)


def _mujoco_audit_reference_text(reference):
    return 'audit_path=%s audit_sha256=%s' % (
        _bounded_text(str(reference.get('path', '') or ''), 256),
        str(reference.get('sha256', '') or ''),
    )


def _bounded_status_reason(reason):
    return _bounded_text(str(reason or ''), _MUJOCO_STATUS_TEXT_LIMIT) or ''


def _bounded_status_code(code):
    return _bounded_text(str(code or 'MUJOCO_GATE_FAILED'), 128) or 'MUJOCO_GATE_FAILED'


def _finite_pose(pose):
    try:
        validate_finite_pose(pose)
    except (TypeError, ValueError, AttributeError):
        return False
    return True


def _finite_geometry(geometry):
    try:
        validate_rich_geometry(geometry)
    except (TypeError, ValueError, AttributeError):
        return False
    return True


def validate_execution_plan(plan, now_sec, validity_sec, enforce_freshness=True):
    if plan is None:
        return PlanValidationResult(False, 'PLAN_MISSING', 'no rich 6D plan')
    diagnostic = str(getattr(plan, 'diagnostic', '') or '')
    if not bool(getattr(plan, 'valid', False)):
        code = diagnostic.split(':', 1)[0].strip() if diagnostic else 'PLAN_INVALID'
        return PlanValidationResult(False, code, diagnostic or 'rich plan is invalid')
    plan_id = str(getattr(plan, 'plan_id', '') or '').strip()
    if not plan_id:
        return PlanValidationResult(False, 'PLAN_ID_MISSING', 'plan_id is empty')
    if not str(getattr(plan, 'model_choice', '') or '').strip():
        return PlanValidationResult(False, 'MODEL_MISSING', 'model_choice is empty')
    poses = list(getattr(plan, 'poses', ()) or ())
    if len(poses) != 4 or not all(_finite_pose(pose) for pose in poses):
        return PlanValidationResult(
            False,
            'PLAN_MALFORMED',
            'rich plan must contain exactly four finite non-zero-quaternion poses',
        )
    try:
        score = float(plan.score)
        required_width = float(plan.required_open_width_m)
    except Exception:
        return PlanValidationResult(False, 'PLAN_MALFORMED', 'plan scalar fields are invalid')
    if not math.isfinite(score):
        return PlanValidationResult(False, 'PLAN_MALFORMED', 'plan score is non-finite')
    try:
        validate_candidate_model_width(plan)
    except (TypeError, ValueError, AttributeError) as exc:
        return PlanValidationResult(
            False,
            'CANDIDATE_SOURCE_INVALID',
            str(exc),
        )
    if not required_open_width_is_valid(required_width):
        return PlanValidationResult(False, 'GRIPPER_TOO_NARROW', 'required opening is outside (0, 0.050] m')
    if not _finite_geometry(getattr(plan, 'object_geometry', None)):
        return PlanValidationResult(False, 'OBB_INVALID', 'embedded object geometry is invalid')
    try:
        validate_plan_header_binding(plan)
    except (TypeError, ValueError, AttributeError) as exc:
        return PlanValidationResult(
            False,
            'PLAN_SNAPSHOT_MISMATCH',
            str(exc),
        )
    if not plan_id_matches_content(plan):
        return PlanValidationResult(
            False,
            'PLAN_ID_MISMATCH',
            'plan_id does not match canonical rich-plan content',
        )
    stamp_sec = _stamp_seconds(getattr(getattr(plan, 'header', None), 'stamp', None))
    if not math.isfinite(stamp_sec) or stamp_sec <= 0.0:
        return PlanValidationResult(False, 'PLAN_STALE', 'plan source timestamp is zero')
    age = float(now_sec) - stamp_sec
    if not enforce_freshness:
        return PlanValidationResult(True, age_sec=age)
    if age < 0.0:
        return PlanValidationResult(False, 'PLAN_FUTURE', 'plan source timestamp is in the future', age)
    if age > max(0.0, float(validity_sec)):
        return PlanValidationResult(False, 'PLAN_STALE', 'plan source timestamp is stale', age)
    return PlanValidationResult(True, age_sec=age)


def split_rich_plan_poses(plan):
    if len(getattr(plan, 'poses', ()) or ()) != 4:
        raise ValueError('rich 6D plan must contain exactly four poses')
    result = []
    for source in plan.poses:
        stamped = PoseStamped()
        stamped.header = deepcopy(plan.header)
        stamped.pose = deepcopy(source)
        result.append(stamped)
    return tuple(result)


def _vector3_norm(vector):
    return math.sqrt(sum(float(value) * float(value) for value in vector))


def _normalize_vector3(vector, label):
    values = tuple(float(value) for value in vector)
    if len(values) != 3 or not all(math.isfinite(value) for value in values):
        raise ValueError('%s must contain three finite values' % label)
    norm = _vector3_norm(values)
    if norm <= 1e-12:
        raise ValueError('%s has zero length' % label)
    return tuple(value / norm for value in values)


def _quaternion_xyzw(pose):
    orientation = pose.orientation
    values = (
        float(orientation.x),
        float(orientation.y),
        float(orientation.z),
        float(orientation.w),
    )
    norm = math.sqrt(sum(value * value for value in values))
    if not all(math.isfinite(value) for value in values) or norm <= 1e-12:
        raise ValueError('pose quaternion is invalid')
    return tuple(value / norm for value in values)


def _quaternion_multiply(first, second):
    x1, y1, z1, w1 = first
    x2, y2, z2, w2 = second
    return (
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    )


def _axis_angle_quaternion(axis, angle_rad):
    unit = _normalize_vector3(axis, 'rotation axis')
    half = 0.5 * float(angle_rad)
    scale = math.sin(half)
    return (
        unit[0] * scale,
        unit[1] * scale,
        unit[2] * scale,
        math.cos(half),
    )


def _rotate_vector(quaternion, vector):
    q = tuple(float(value) for value in quaternion)
    v = (float(vector[0]), float(vector[1]), float(vector[2]), 0.0)
    conjugate = (-q[0], -q[1], -q[2], q[3])
    rotated = _quaternion_multiply(_quaternion_multiply(q, v), conjugate)
    return rotated[:3]


def _dot3(first, second):
    return sum(float(a) * float(b) for a, b in zip(first, second))


def _cross3(first, second):
    return (
        first[1] * second[2] - first[2] * second[1],
        first[2] * second[0] - first[0] * second[2],
        first[0] * second[1] - first[1] * second[0],
    )


def _project_unit_to_plane(vector, normal, label):
    projected = tuple(
        float(vector[index]) - _dot3(vector, normal) * float(normal[index])
        for index in range(3)
    )
    return _normalize_vector3(projected, label)


def build_bounded_final_visual_refinement(
    current_plan,
    observed_plan,
    max_translation_m,
    max_yaw_rad,
    max_roll_pitch_change_rad,
):
    """Fuse bounded xyz/yaw observations while preserving the prior tilt."""

    def reject(code, reason):
        return PlanValidationResult(False, code, reason), None, {}

    if current_plan is None or observed_plan is None:
        return reject(
            'FINAL_REFINE_PLAN_MISSING',
            'current and observed rich plans are required',
        )
    if str(current_plan.model_choice) != str(observed_plan.model_choice):
        return reject(
            'FINAL_REFINE_CANDIDATE_SWITCH',
            'model choice changed during final refinement',
        )
    if str(current_plan.candidate_source) != str(
        observed_plan.candidate_source
    ) or tuple(current_plan.candidate_source_lineage) != tuple(
        observed_plan.candidate_source_lineage
    ):
        return reject(
            'FINAL_REFINE_CANDIDATE_SWITCH',
            'candidate source lineage changed during final refinement',
        )
    if str(current_plan.object_geometry.label) != str(
        observed_plan.object_geometry.label
    ):
        return reject(
            'FINAL_REFINE_TARGET_CHANGED',
            'target label changed during final refinement',
        )

    current_poses = list(getattr(current_plan, 'poses', ()) or ())
    observed_poses = list(getattr(observed_plan, 'poses', ()) or ())
    if len(current_poses) != 4 or len(observed_poses) != 4:
        return reject(
            'FINAL_REFINE_PLAN_MALFORMED',
            'both rich plans must contain four poses',
        )
    try:
        for index, pose in enumerate(current_poses):
            validate_finite_pose(pose, 'current pose %d' % index)
        for index, pose in enumerate(observed_poses):
            validate_finite_pose(pose, 'observed pose %d' % index)
        normal_msg = current_plan.object_geometry.support_normal_base
        support_normal = _normalize_vector3(
            (normal_msg.x, normal_msg.y, normal_msg.z),
            'support normal',
        )
        current_grasp = current_poses[2]
        observed_grasp = observed_poses[2]
        translation = (
            float(observed_grasp.position.x)
            - float(current_grasp.position.x),
            float(observed_grasp.position.y)
            - float(current_grasp.position.y),
            float(observed_grasp.position.z)
            - float(current_grasp.position.z),
        )
        translation_norm = _vector3_norm(translation)
        translation_limit = max(0.0, float(max_translation_m))
        if translation_norm > translation_limit + 1e-12:
            return reject(
                'FINAL_REFINE_TRANSLATION_LIMIT',
                'observed grasp center correction %.4fm exceeds %.4fm'
                % (translation_norm, translation_limit),
            )

        current_q = _quaternion_xyzw(current_grasp)
        observed_q = _quaternion_xyzw(observed_grasp)
        current_jaw = _project_unit_to_plane(
            _rotate_vector(current_q, (0.0, 1.0, 0.0)),
            support_normal,
            'current jaw projection',
        )
        observed_jaw = _project_unit_to_plane(
            _rotate_vector(observed_q, (0.0, 1.0, 0.0)),
            support_normal,
            'observed jaw projection',
        )
        yaw_rad = math.atan2(
            _dot3(support_normal, _cross3(current_jaw, observed_jaw)),
            max(-1.0, min(1.0, _dot3(current_jaw, observed_jaw))),
        )
        yaw_limit = max(0.0, float(max_yaw_rad))
        if abs(yaw_rad) > yaw_limit + 1e-12:
            return reject(
                'FINAL_REFINE_YAW_LIMIT',
                'observed yaw correction %.2fdeg exceeds %.2fdeg'
                % (math.degrees(abs(yaw_rad)), math.degrees(yaw_limit)),
            )

        inverse_yaw = _axis_angle_quaternion(support_normal, -yaw_rad)
        current_approach = _normalize_vector3(
            _rotate_vector(current_q, (0.0, 0.0, 1.0)),
            'current approach axis',
        )
        observed_approach_without_yaw = _normalize_vector3(
            _rotate_vector(
                inverse_yaw,
                _rotate_vector(observed_q, (0.0, 0.0, 1.0)),
            ),
            'observed approach axis without yaw',
        )
        roll_pitch_change_rad = math.acos(
            max(
                -1.0,
                min(
                    1.0,
                    _dot3(
                        current_approach,
                        observed_approach_without_yaw,
                    ),
                ),
            )
        )
        roll_pitch_limit = max(
            0.0,
            float(max_roll_pitch_change_rad),
        )
        if roll_pitch_change_rad > roll_pitch_limit + 1e-12:
            return reject(
                'FINAL_REFINE_ROLL_PITCH_LIMIT',
                'observed roll/pitch change %.2fdeg exceeds %.2fdeg'
                % (
                    math.degrees(roll_pitch_change_rad),
                    math.degrees(roll_pitch_limit),
                ),
            )

        yaw_quaternion = _axis_angle_quaternion(support_normal, yaw_rad)
        old_grasp_xyz = (
            float(current_grasp.position.x),
            float(current_grasp.position.y),
            float(current_grasp.position.z),
        )
        new_grasp_xyz = (
            float(observed_grasp.position.x),
            float(observed_grasp.position.y),
            float(observed_grasp.position.z),
        )
        corrected_poses = []
        for pose in current_poses:
            corrected = deepcopy(pose)
            relative = (
                float(pose.position.x) - old_grasp_xyz[0],
                float(pose.position.y) - old_grasp_xyz[1],
                float(pose.position.z) - old_grasp_xyz[2],
            )
            rotated_relative = _rotate_vector(yaw_quaternion, relative)
            corrected.position.x = new_grasp_xyz[0] + rotated_relative[0]
            corrected.position.y = new_grasp_xyz[1] + rotated_relative[1]
            corrected.position.z = new_grasp_xyz[2] + rotated_relative[2]
            corrected_q = _quaternion_multiply(
                yaw_quaternion,
                _quaternion_xyzw(pose),
            )
            corrected_q_norm = math.sqrt(
                sum(value * value for value in corrected_q)
            )
            corrected.orientation.x = corrected_q[0] / corrected_q_norm
            corrected.orientation.y = corrected_q[1] / corrected_q_norm
            corrected.orientation.z = corrected_q[2] / corrected_q_norm
            corrected.orientation.w = corrected_q[3] / corrected_q_norm
            corrected_poses.append(corrected)

        refined = deepcopy(current_plan)
        refined.header = deepcopy(observed_plan.header)
        refined.object_geometry = deepcopy(observed_plan.object_geometry)
        refined.poses = corrected_poses
        refined.score = float(observed_plan.score)
        refined.required_open_width_m = max(
            float(current_plan.required_open_width_m),
            float(observed_plan.required_open_width_m),
        )
        refined.diagnostic = (
            'FINAL_VISUAL_REFINE: translation=%.4fm yaw=%.2fdeg '
            'roll_pitch_frozen=1'
            % (translation_norm, math.degrees(yaw_rad))
        )
        refined.valid = True
        refined.plan_id = compute_plan_id(refined)
        if not plan_id_matches_content(refined):
            return reject(
                'FINAL_REFINE_PLAN_ID_FAILED',
                'corrected rich plan failed canonical integrity',
            )
    except Exception as exc:
        return reject('FINAL_REFINE_INVALID', str(exc))

    metrics = {
        'translation_m': translation_norm,
        'yaw_rad': yaw_rad,
        'observed_roll_pitch_change_rad': roll_pitch_change_rad,
    }
    return PlanValidationResult(True), refined, metrics


def _plan_support_surface_xyz(plan):
    geometry = getattr(plan, 'object_geometry', None)
    center = getattr(
        getattr(geometry, 'pose_base', None),
        'position',
        None,
    )
    size = getattr(geometry, 'size_xyz_m', None)
    normal = getattr(geometry, 'support_normal_base', None)
    center_xyz = (
        float(center.x),
        float(center.y),
        float(center.z),
    )
    height_m = float(size.z)
    if not all(math.isfinite(value) for value in center_xyz):
        raise ValueError('plan geometry center is non-finite')
    if not math.isfinite(height_m) or height_m <= 0.0:
        raise ValueError('plan geometry height is invalid')
    support_normal = _normalize_vector3(
        (normal.x, normal.y, normal.z),
        'support normal',
    )
    return tuple(
        center_xyz[index] + support_normal[index] * 0.5 * height_m
        for index in range(3)
    )


def build_bounded_final_center_refinement(
    current_plan,
    observed_surface_center_xyz,
    observed_stamp,
    max_translation_m,
):
    """Translate a frozen candidate from a stable observed surface center."""

    def reject(code, reason):
        return PlanValidationResult(False, code, reason), None, {}

    if current_plan is None:
        return reject(
            'FINAL_REFINE_PLAN_MISSING',
            'a current rich plan is required',
        )
    poses = list(getattr(current_plan, 'poses', ()) or ())
    if len(poses) != 4:
        return reject(
            'FINAL_REFINE_PLAN_MALFORMED',
            'the current rich plan must contain four poses',
        )
    if _stamp_nanoseconds(observed_stamp) <= 0:
        return reject(
            'FINAL_REFINE_CENTER_STAMP_INVALID',
            'the observed surface center needs a positive source timestamp',
        )
    try:
        for index, pose in enumerate(poses):
            validate_finite_pose(pose, 'current pose %d' % index)
        observed_surface = tuple(
            float(value) for value in observed_surface_center_xyz
        )
        if len(observed_surface) != 3 or not all(
            math.isfinite(value) for value in observed_surface
        ):
            raise ValueError('observed surface center must be finite xyz')
        planned_surface = _plan_support_surface_xyz(current_plan)
        translation = tuple(
            observed_surface[index] - planned_surface[index]
            for index in range(3)
        )
        translation_norm = _vector3_norm(translation)
        translation_limit = max(0.0, float(max_translation_m))
        if translation_norm > translation_limit + 1e-12:
            return reject(
                'FINAL_REFINE_TRANSLATION_LIMIT',
                'surface-center correction %.4fm exceeds %.4fm'
                % (translation_norm, translation_limit),
            )

        refined = deepcopy(current_plan)
        refined.header.stamp = deepcopy(observed_stamp)
        refined.object_geometry.header = deepcopy(refined.header)
        refined.poses = []
        for pose in poses:
            corrected = deepcopy(pose)
            corrected.position.x += translation[0]
            corrected.position.y += translation[1]
            corrected.position.z += translation[2]
            refined.poses.append(corrected)
        geometry_center = refined.object_geometry.pose_base.position
        geometry_center.x += translation[0]
        geometry_center.y += translation[1]
        geometry_center.z += translation[2]
        refined.diagnostic = (
            'FINAL_CENTER_REFINE: translation=%.4fm '
            'surface_center=1 orientation_frozen=1 width_frozen=1'
            % translation_norm
        )
        refined.valid = True
        refined.plan_id = compute_plan_id(refined)
        if not plan_id_matches_content(refined):
            return reject(
                'FINAL_REFINE_PLAN_ID_FAILED',
                'center-corrected rich plan failed canonical integrity',
            )
    except Exception as exc:
        return reject('FINAL_REFINE_CENTER_INVALID', str(exc))

    return PlanValidationResult(True), refined, {
        'translation_m': translation_norm,
        'translation_xyz': translation,
        'planned_surface_xyz': planned_surface,
        'observed_surface_xyz': observed_surface,
    }


def _poses_same_enough(first, second, position_tolerance_m=0.001, quaternion_tolerance=0.001):
    try:
        first_pose = getattr(first, 'pose', first)
        second_pose = getattr(second, 'pose', second)
        first_position = first_pose.position
        second_position = second_pose.position
        dx = float(first_position.x) - float(second_position.x)
        dy = float(first_position.y) - float(second_position.y)
        dz = float(first_position.z) - float(second_position.z)
        position_delta = math.sqrt(dx * dx + dy * dy + dz * dz)
        first_orientation = first_pose.orientation
        second_orientation = second_pose.orientation
        dot = (
            float(first_orientation.x) * float(second_orientation.x)
            + float(first_orientation.y) * float(second_orientation.y)
            + float(first_orientation.z) * float(second_orientation.z)
            + float(first_orientation.w) * float(second_orientation.w)
        )
    except Exception:
        return False
    return (
        position_delta <= float(position_tolerance_m)
        and 1.0 - min(1.0, abs(dot)) <= float(quaternion_tolerance)
    )


def make_observation_camera_retreat_pose(
    current_tool_pose,
    camera_position_xyz,
    target_position_xyz,
    nominal_distance_m,
    execution_error_vector_xyz,
):
    """Build one measured radial range correction with endpoint feed-forward.

    The function name is retained for compatibility with existing callers.
    A positive radial correction retreats a too-close camera; a negative one
    continues a too-far observation move toward the same live target ray.
    """

    camera = tuple(float(value) for value in camera_position_xyz)
    target = tuple(float(value) for value in target_position_xyz)
    execution_error = tuple(
        float(value) for value in execution_error_vector_xyz
    )
    nominal = float(nominal_distance_m)
    values = camera + target + execution_error + (nominal,)
    if not all(math.isfinite(value) for value in values):
        raise ValueError('observation retreat evidence must be finite')
    if nominal <= 0.0:
        raise ValueError('nominal observation distance must be positive')
    target_to_camera = tuple(
        camera[index] - target[index] for index in range(3)
    )
    current_distance = math.sqrt(
        sum(value * value for value in target_to_camera)
    )
    if current_distance <= 1e-9:
        raise ValueError('camera and target centres must be distinct')
    radial_distance_delta = nominal - current_distance
    if abs(radial_distance_delta) <= 1e-9:
        raise ValueError('observation range correction is already nominal')
    radial_correction = tuple(
        target_to_camera[index]
        * radial_distance_delta
        / current_distance
        for index in range(3)
    )
    command_delta = tuple(
        radial_correction[index] - execution_error[index]
        for index in range(3)
    )
    corrected = deepcopy(current_tool_pose)
    current_position = corrected.pose.position
    current_xyz = (
        float(current_position.x),
        float(current_position.y),
        float(current_position.z),
    )
    if not all(math.isfinite(value) for value in current_xyz):
        raise ValueError('current tool position must be finite')
    current_position.x = current_xyz[0] + command_delta[0]
    current_position.y = current_xyz[1] + command_delta[1]
    current_position.z = current_xyz[2] + command_delta[2]
    return corrected, {
        'current_camera_target_distance_m': float(current_distance),
        'nominal_camera_target_distance_m': float(nominal),
        'radial_correction_xyz_m': [
            float(value) for value in radial_correction
        ],
        # Retain the legacy field so old audit readers do not fail. Its signed
        # vector now has the same generalized meaning as radial_correction.
        'radial_retreat_xyz_m': [
            float(value) for value in radial_correction
        ],
        'correction_direction': (
            'retreat' if radial_distance_delta > 0.0 else 'approach'
        ),
        'execution_error_feedforward_xyz_m': [
            float(-value) for value in execution_error
        ],
        'command_translation_xyz_m': [float(value) for value in command_delta],
        'command_translation_norm_m': math.sqrt(
            sum(value * value for value in command_delta)
        ),
        'orientation_policy': 'preserve_current_measured_tool_orientation',
        'correction_policy': (
            'single_out_of_range_radial_correction_with_current_endpoint_error_feedforward'
        ),
    }


def is_observation_contract_recoverable_execution_failure(message):
    """Identify a submitted strict path whose controller reported failure."""

    return str(message or '').startswith(
        'execute failed from cached plan (strict pose):'
    )


class GraspTaskNode:
    def __init__(self):
        self.latest_obj = None
        self.latest_obj_time = None
        self.latest_visual_obj = None
        self.latest_visual_obj_time = None
        self.latest_grasp6d_plan = None
        self.latest_grasp6d_preview_plan = None
        self.latest_grasp6d_legacy_plan = None
        self._bound_execution_plan = None
        self._bound_execution_plan_id = ''
        self._bound_execution_plan_digest = ''
        self._execution_authority_revoked = False
        self._bound_target_occlusion_allowed = False
        self._last_execution_plan_event = ''
        self._last_measured_endpoint_sample = None
        self._last_observation_camera_range_evidence = None
        self._grasp6d_watermark_stamp_ns = 0
        self._grasp6d_watermark_plan_id = ''
        self._grasp6d_watermark_tombstoned = False
        self._grasp6d_plan_lock = threading.RLock()
        self._start_lock = threading.RLock()
        self._start_inflight = False
        self.latest_joint_state = None
        self.latest_actuation_status = ''
        self.latest_actuation_status_time = None
        self.latest_raw_detection = False
        self.latest_raw_detection_time = None
        self.active = False
        self._near_field_active = None
        self.stage = GraspStages.IDLE
        self.tf_buffer = None
        self.tf_listener = None
        if tf2_ros is not None and bool(rospy.get_param('/handeye/use_tf', True)):
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.pub = rospy.Publisher(
            '/grasp/state',
            GraspState,
            queue_size=10,
            latch=True,
        )
        self.near_field_pub = rospy.Publisher(
            '/grasp/near_field_active',
            Bool,
            queue_size=1,
            latch=True,
        )
        rospy.Subscriber('/perception/object', ObjectPose, self.obj_cb, queue_size=1)
        rospy.Subscriber(
            rospy.get_param(
                '/grasp/grasp6d_enriched_plan_topic',
                '/grasp_6d/plan_enriched',
            ),
            Grasp6DPlan,
            self.grasp6d_plan_cb,
            queue_size=1,
        )
        rospy.Subscriber(
            rospy.get_param('/grasp/grasp6d_plan_topic', '/grasp_6d/plan'),
            PoseArray,
            self.grasp6d_legacy_plan_cb,
            queue_size=1,
        )
        rospy.Subscriber(
            '/grasp_6d/preview_plan_enriched',
            Grasp6DPlan,
            self.grasp6d_preview_plan_cb,
            queue_size=1,
        )
        rospy.Subscriber('/joint_states', JointState, self.joint_cb, queue_size=1)
        rospy.Subscriber(
            '/alicia_d/actuation_status',
            String,
            self.actuation_status_cb,
            queue_size=1,
        )
        rospy.Subscriber('/perception/raw_object_detected', Bool, self.raw_detection_cb, queue_size=1)
        rospy.Service('/grasp/start', StartGrasp, self.start_cb)
        rospy.Service(
            '/grasp/current_plan',
            TriggerZero,
            self.current_plan_cb,
        )
        rospy.Service('/grasp/stop', StopGrasp, self.stop_cb)
        rospy.loginfo('GraspTaskNode ready')
        self._set_near_field_active(False, force=True)
        self.set_state(GraspStages.IDLE, 'ready')

    def obj_cb(self, msg):
        if not msg.detected:
            self.latest_visual_obj = None
            self.latest_visual_obj_time = None
            with self._grasp6d_plan_guard():
                if self._target_occlusion_allowed_locked():
                    rospy.logwarn_throttle(
                        1.0,
                        'Preserving frozen 6D execution authority through close-range target occlusion',
                    )
                    return
                self.latest_obj = None
                self.latest_obj_time = None
                self._clear_grasp6d_authority()
            return
        gcfg = rospy.get_param('/grasp', {})
        confidence = float(getattr(msg, 'confidence', 1.0) or 0.0)
        now = rospy.Time.now()
        # Preserve low-confidence close-range detections for visual retargeting.
        # Candidate generation still uses latest_obj and its stricter threshold.
        self.latest_visual_obj = msg
        self.latest_visual_obj_time = now
        source_stamp = self._object_source_stamp(msg)
        source_age = (
            float('inf')
            if source_stamp is None
            else _stamp_seconds(now) - _stamp_seconds(source_stamp)
        )
        source_validity = self._configured_target_observation_validity(gcfg)
        if (
            source_stamp is None
            or not math.isfinite(source_age)
            or source_age < 0.0
            or source_age > source_validity
        ):
            rospy.logwarn_throttle(
                1.0,
                'Grasp ignored stale object source age %.3fs outside [0, %.3f]s',
                source_age,
                source_validity,
            )
            with self._grasp6d_plan_guard():
                if self._target_occlusion_allowed_locked():
                    rospy.logwarn_throttle(
                        1.0,
                        'Preserving frozen 6D execution authority through stale close-range target observation',
                    )
                    return
                self.latest_obj = None
                self.latest_obj_time = None
                self._clear_grasp6d_authority()
            return
        min_confidence = self._cfg_float(gcfg, 'min_object_confidence', 0.50)
        if confidence < min_confidence:
            rospy.logwarn_throttle(
                1.0,
                'Grasp ignored low-confidence object %.3f < %.3f',
                confidence,
                min_confidence,
            )
            with self._grasp6d_plan_guard():
                if self._target_occlusion_allowed_locked():
                    rospy.logwarn_throttle(
                        1.0,
                        'Preserving frozen 6D execution authority through low-confidence close-range target observation',
                    )
                    return
                self.latest_obj = None
                self.latest_obj_time = None
                self._clear_grasp6d_authority()
            return

        previous = getattr(self, 'latest_obj', None)
        previous_time = getattr(self, 'latest_obj_time', None)
        max_jump = self._cfg_float(gcfg, 'max_object_jump_m', 0.12)
        jump_window = self._cfg_float(gcfg, 'object_jump_filter_window_sec', 4.0)
        if previous is not None and previous_time is not None and max_jump > 0.0:
            try:
                age = (now - previous_time).to_sec()
            except Exception:
                age = float('inf')
            jump = self._object_distance(previous, msg)
            if age <= jump_window and jump > max_jump:
                rospy.logwarn_throttle(
                    1.0,
                    'Grasp ignored object jump %.3f m > %.3f m within %.1fs',
                    jump,
                    max_jump,
                    jump_window,
                )
                with self._grasp6d_plan_guard():
                    self.latest_obj = None
                    self.latest_obj_time = None
                    self._clear_grasp6d_authority()
                return

        # Live authority updates share the plan RLock with physical commits so
        # a drift-changing observation cannot cross the action boundary.
        with self._grasp6d_plan_guard():
            self.latest_obj = msg
            self.latest_obj_time = source_stamp

    @staticmethod
    def _object_source_stamp(msg):
        primary = getattr(getattr(msg, 'header', None), 'stamp', None)
        if _stamp_nanoseconds(primary) > 0:
            return primary
        fallback = getattr(
            getattr(getattr(msg, 'pose_base', None), 'header', None),
            'stamp',
            None,
        )
        if _stamp_nanoseconds(fallback) > 0:
            return fallback
        return None

    def joint_cb(self, msg):
        self.latest_joint_state = msg

    def actuation_status_cb(self, msg):
        self.latest_actuation_status = str(
            getattr(msg, 'data', '') or ''
        ).strip()
        self.latest_actuation_status_time = rospy.Time.now()

    def _automatic_actuation_gate(self, gcfg, now_sec=None):
        if not self._cfg_bool(
            gcfg,
            'require_actuation_confirmation',
            False,
        ):
            return True, ''
        status = str(
            getattr(self, 'latest_actuation_status', '') or ''
        ).strip()
        received = getattr(
            self,
            'latest_actuation_status_time',
            None,
        )
        if now_sec is None:
            now_sec = _stamp_seconds(rospy.Time.now())
        age = (
            float('inf')
            if received is None
            else float(now_sec) - _stamp_seconds(received)
        )
        freshness = max(
            0.0,
            self._cfg_float(
                gcfg,
                'actuation_confirmation_freshness_sec',
                2.0,
            ),
        )
        if status.split(':', 1)[0] != 'CONFIRMED':
            return (
                False,
                'ACTUATION_UNCONFIRMED: %s'
                % (status or 'status missing'),
            )
        if not math.isfinite(age) or age < 0.0 or age > freshness:
            return (
                False,
                (
                    'ACTUATION_UNCONFIRMED: confirmation age %.3fs '
                    'exceeds %.3fs'
                )
                % (age, freshness),
            )
        return True, ''

    def raw_detection_cb(self, msg):
        self.latest_raw_detection = bool(msg.data)
        self.latest_raw_detection_time = rospy.Time.now()

    def grasp6d_plan_cb(self, msg):
        result = validate_execution_plan(
            msg,
            _stamp_seconds(rospy.Time.now()),
            self._configured_plan_validity({}),
        )
        with self._grasp6d_plan_guard():
            current = getattr(self, 'latest_grasp6d_plan', None)
            self._seed_grasp6d_watermark_locked(current)
            incoming_ns = _stamp_nanoseconds(
                getattr(getattr(msg, 'header', None), 'stamp', None)
            )
            incoming_id = str(getattr(msg, 'plan_id', '') or '')
            execution_frozen = (
                bool(getattr(self, 'active', False))
                and getattr(self, '_bound_execution_plan', None) is not None
            )
            if execution_frozen and bool(getattr(msg, 'valid', False)):
                self._last_execution_plan_event = 'EXECUTION_FROZEN'
                rospy.logwarn(
                    'Ignored rich 6D plan %s: EXECUTION_FROZEN (%s)',
                    incoming_id,
                    result.code if not result.ok else 'VALID_REPLACEMENT',
                )
                return
            if not result.ok:
                if execution_frozen:
                    self._execution_authority_revoked = True
                    self._last_execution_plan_event = (
                        'EXECUTION_AUTHORITY_REVOKED'
                    )
                if incoming_ns > self._grasp6d_watermark_stamp_ns:
                    self._grasp6d_watermark_stamp_ns = incoming_ns
                    self._grasp6d_watermark_plan_id = incoming_id
                if self._grasp6d_watermark_stamp_ns > 0:
                    self._grasp6d_watermark_tombstoned = True
                self.latest_grasp6d_plan = None
                rospy.logwarn_throttle(
                    1.0,
                    'Rejected rich 6D plan %s: %s',
                    result.code,
                    result.reason,
                )
                return
            replayed_source = (
                incoming_ns < self._grasp6d_watermark_stamp_ns
                or (
                    incoming_ns == self._grasp6d_watermark_stamp_ns
                    and (
                        self._grasp6d_watermark_tombstoned
                        or not strict_plan_id_equal(
                            incoming_id, self._grasp6d_watermark_plan_id
                        )
                    )
                )
            )
            if replayed_source:
                if self._grasp6d_watermark_stamp_ns > 0:
                    self._grasp6d_watermark_tombstoned = True
                self.latest_grasp6d_plan = None
                rospy.logwarn(
                    'Rejected rich 6D plan PLAN_REPLAYED: older, conflicting, or tombstoned source timestamp'
                )
                return
            if incoming_ns > self._grasp6d_watermark_stamp_ns:
                self._grasp6d_watermark_stamp_ns = incoming_ns
                self._grasp6d_watermark_plan_id = incoming_id
                self._grasp6d_watermark_tombstoned = False
            self.latest_grasp6d_plan = deepcopy(msg)
            rospy.loginfo(
                'Accepted rich 6D execution plan plan_id=%s source_stamp_ns=%d',
                incoming_id,
                incoming_ns,
            )

    def grasp6d_preview_plan_cb(self, msg):
        """Cache Preview for diagnostics without changing execution authority."""
        self.latest_grasp6d_preview_plan = deepcopy(msg)

    def grasp6d_legacy_plan_cb(self, msg):
        # Compatibility visualization only. Never assign execution authority.
        self.latest_grasp6d_legacy_plan = deepcopy(msg)

    def _grasp6d_plan_guard(self):
        lock = getattr(self, '_grasp6d_plan_lock', None)
        if lock is None:
            lock = threading.RLock()
            self._grasp6d_plan_lock = lock
        return lock

    def _start_guard(self):
        lock = getattr(self, '_start_lock', None)
        if lock is None:
            lock = threading.RLock()
            self._start_lock = lock
        return lock

    def _seed_grasp6d_watermark_locked(self, current=None):
        if not hasattr(self, '_grasp6d_watermark_stamp_ns'):
            self._grasp6d_watermark_stamp_ns = 0
            self._grasp6d_watermark_plan_id = ''
            self._grasp6d_watermark_tombstoned = False
        if current is None:
            current = getattr(self, 'latest_grasp6d_plan', None)
        if current is not None and self._grasp6d_watermark_stamp_ns <= 0:
            self._grasp6d_watermark_stamp_ns = _stamp_nanoseconds(
                getattr(getattr(current, 'header', None), 'stamp', None)
            )
            self._grasp6d_watermark_plan_id = str(
                getattr(current, 'plan_id', '') or ''
            )
            self._grasp6d_watermark_tombstoned = False

    def _clear_grasp6d_authority(self, expected_plan_id=None):
        with self._grasp6d_plan_guard():
            if getattr(self, '_bound_execution_plan', None) is not None:
                self._execution_authority_revoked = True
                self._last_execution_plan_event = 'EXECUTION_AUTHORITY_REVOKED'
            current = getattr(self, 'latest_grasp6d_plan', None)
            self._seed_grasp6d_watermark_locked(current)
            if (
                expected_plan_id is not None
                and current is not None
                and str(getattr(current, 'plan_id', '') or '')
                != str(expected_plan_id)
            ):
                return False
            if self._grasp6d_watermark_stamp_ns > 0:
                self._grasp6d_watermark_tombstoned = True
            self.latest_grasp6d_plan = None
            return True

    @staticmethod
    def _execution_plan_digest(plan):
        wire = io.BytesIO()
        plan.serialize(wire)
        return hashlib.sha256(wire.getvalue()).hexdigest()

    def _freeze_execution_plan(self, plan):
        frozen = deepcopy(plan)
        if not plan_id_matches_content(frozen):
            raise ValueError('cannot freeze an invalid rich execution plan')
        digest = self._execution_plan_digest(frozen)
        with self._grasp6d_plan_guard():
            self._bound_execution_plan = frozen
            self._bound_execution_plan_id = str(frozen.plan_id)
            self._bound_execution_plan_digest = digest
            self._execution_authority_revoked = False
            self._bound_target_occlusion_allowed = False
            self._last_execution_plan_event = 'EXECUTION_FROZEN'
        return deepcopy(frozen)

    def _clear_bound_execution_plan(self):
        with self._grasp6d_plan_guard():
            self._bound_execution_plan = None
            self._bound_execution_plan_id = ''
            self._bound_execution_plan_digest = ''
            self._execution_authority_revoked = False
            self._bound_target_occlusion_allowed = False

    def set_state(self, stage, message='', success=False):
        self.stage = stage
        msg = GraspState()
        msg.header.stamp = rospy.Time.now()
        msg.stage = int(stage)
        msg.state = STATE_NAMES.get(stage, 'UNKNOWN')
        msg.active = self.active
        msg.success = success
        msg.message = message
        self.pub.publish(msg)
        rospy.loginfo('[Grasp] %s %s', msg.state, message)

    def _set_near_field_active(self, active, force=False):
        """Publish the explicit near-field planning phase without moving hardware."""

        active = bool(active)
        previous = getattr(self, '_near_field_active', None)
        self._near_field_active = active
        if previous == active and not force:
            return False
        publisher = getattr(self, 'near_field_pub', None)
        if publisher is not None and callable(
            getattr(publisher, 'publish', None)
        ):
            publisher.publish(Bool(data=active))
        rospy.loginfo(
            '6D planning phase signal: %s',
            'near_field' if active else 'far_field',
        )
        return True

    def current_plan_cb(self, req):
        del req
        with self._grasp6d_plan_guard():
            current = deepcopy(
                getattr(self, 'latest_grasp6d_plan', None)
            )
            tombstoned = bool(
                getattr(self, '_grasp6d_watermark_tombstoned', False)
            )
        if current is None:
            return TriggerZeroResponse(
                False,
                'PLAN_MISSING: no current rich plan',
            )
        plan_id = str(getattr(current, 'plan_id', '') or '')
        source_stamp_ns = _stamp_nanoseconds(
            getattr(getattr(current, 'header', None), 'stamp', None)
        )
        if tombstoned:
            return TriggerZeroResponse(
                False,
                'PLAN_REPLAYED: plan_id=%s source_stamp_ns=%d is tombstoned'
                % (plan_id, source_stamp_ns),
            )
        validation = validate_execution_plan(
            current,
            _stamp_seconds(rospy.Time.now()),
            self._configured_plan_validity({}),
        )
        return TriggerZeroResponse(
            bool(validation.ok),
            (
                'plan_id=%s source_stamp_ns=%d validation=%s%s'
                % (
                    plan_id,
                    source_stamp_ns,
                    validation.code or 'VALID',
                    (
                        ': %s' % validation.reason
                        if validation.reason
                        else ''
                    ),
                )
            ),
        )

    def start_cb(self, req):
        if not bool(getattr(req, 'execute', False)):
            return StartGraspResponse(False, 'execute=false')
        gcfg = rospy.get_param('/grasp', {})
        if bool(gcfg.get('calibration_interlock_active', False)):
            reason = str(
                gcfg.get(
                    'calibration_interlock_reason',
                    'hand-eye/TCP calibration is not verified',
                )
                or 'hand-eye/TCP calibration is not verified'
            ).strip()
            message = 'CALIBRATION_INTERLOCK: %s' % reason
            rospy.logerr_throttle(1.0, 'Rejected grasp start: %s', message)
            return StartGraspResponse(False, message)
        actuation_ok, actuation_reason = self._automatic_actuation_gate(gcfg)
        if not actuation_ok:
            rospy.logerr_throttle(
                1.0,
                'Rejected grasp start: %s',
                actuation_reason,
            )
            return StartGraspResponse(False, actuation_reason)
        bound_plan = None
        result = False
        response_message = 'failed'
        with self._start_guard():
            if getattr(self, '_start_inflight', False) or self.active:
                return StartGraspResponse(False, 'already active')
            with self._grasp6d_plan_guard():
                if bool(gcfg.get('use_grasp6d_plan', False)):
                    validation, bound_plan = self._copy_requested_grasp6d_plan(
                        getattr(req, 'plan_id', ''),
                        gcfg,
                    )
                    if not validation.ok:
                        return StartGraspResponse(
                            False,
                            '%s: %s' % (validation.code, validation.reason),
                        )
                    bound_plan = self._freeze_execution_plan(bound_plan)
                self._start_inflight = True
                self.active = True
        self._set_near_field_active(False)
        try:
            result = self.execute(grasp6d_plan=bound_plan)
            response_message = 'success' if result else 'failed'
            return StartGraspResponse(result, response_message)
        except Exception as exc:
            response_message = str(exc)
            self.set_state(GraspStages.FAILED, str(exc), False)
            return StartGraspResponse(False, str(exc))
        finally:
            with self._start_guard():
                with self._grasp6d_plan_guard():
                    self.active = False
                    self._start_inflight = False
                    self._clear_bound_execution_plan()
            self._set_near_field_active(False)
            self.set_state(
                getattr(self, 'stage', GraspStages.IDLE),
                'execution slot released: %s' % response_message,
                bool(result),
            )

    def stop_cb(self, req):
        with self._start_guard():
            with self._grasp6d_plan_guard():
                # Global lock order is start -> plan. A synchronous physical
                # action holding plan first commits before stop returns; a
                # stop that obtains plan first cancels every later action.
                # Only start_cb's finally block releases the execution slot.
                if getattr(self, '_bound_execution_plan', None) is not None:
                    self._execution_authority_revoked = True
                    self._last_execution_plan_event = (
                        'EXECUTION_AUTHORITY_REVOKED'
                    )
                self.active = False
        self._set_near_field_active(False)
        self.set_state(GraspStages.EMERGENCY_STOP if req.emergency else GraspStages.IDLE, 'stop requested')
        return StopGraspResponse(True, 'stop requested')

    def execute(self, grasp6d_plan=None):
        gcfg = rospy.get_param('/grasp', {})
        use_grasp6d_plan = bool(gcfg.get('use_grasp6d_plan', False))
        strict_execute_pose = None
        if use_grasp6d_plan:
            rospy.wait_for_service('/supervisor/check_pose_strict', timeout=10)
            rospy.wait_for_service(
                '/supervisor/plan_and_execute_pose_strict',
                timeout=10,
            )
            move_pose = rospy.ServiceProxy(
                '/supervisor/check_pose_strict', SetTargetPose
            )
            strict_execute_pose = rospy.ServiceProxy(
                '/supervisor/plan_and_execute_pose_strict',
                SetTargetPose,
            )
        else:
            rospy.wait_for_service('/supervisor/move_to_pose', timeout=10)
            move_pose = rospy.ServiceProxy('/supervisor/move_to_pose', SetTargetPose)
        rospy.wait_for_service('/supervisor/move_to_pose_linear', timeout=10)
        rospy.wait_for_service('/supervisor/set_gripper', timeout=10)
        move_pose_linear = rospy.ServiceProxy('/supervisor/move_to_pose_linear', SetTargetPose)
        set_gripper = rospy.ServiceProxy('/supervisor/set_gripper', SetFloat)
        gripper_cfg = rospy.get_param('/gripper', {})
        close = None
        if bool(gripper_cfg.get('use_compliant_close', True)):
            rospy.wait_for_service('/supervisor/compliant_close', timeout=10)
            close = rospy.ServiceProxy('/supervisor/compliant_close', StartGrasp)
        pregrasp_distance = float(gcfg.get('pregrasp_distance', gcfg.get('pregrasp_distance_m', 0.08)))
        final_offset = max(0.0, float(gcfg.get('final_approach_offset_m', 0.015)))
        pregrasp_mode = str(gcfg.get('pregrasp_offset_mode', 'base_z'))
        lift_height = float(gcfg.get('lift_height_m', 0.05))
        pregrasp_reached_tolerance = float(gcfg.get('pregrasp_reached_tolerance_m', 0.04))
        open_position = float(gripper_cfg.get('open_position_m', 0.0))

        if use_grasp6d_plan:
            return self._execute_grasp6d_plan(
                gcfg,
                gripper_cfg,
                open_position,
                move_pose,
                move_pose_linear,
                set_gripper,
                close,
                grasp6d_plan,
                strict_execute_pose=strict_execute_pose,
            )

        self.set_state(GraspStages.SEARCH_OBJECT, 'waiting for object')
        t0 = rospy.Time.now()
        while self.latest_obj is None and (rospy.Time.now()-t0).to_sec() < 5.0 and self.active:
            rospy.sleep(0.05)
        if self.latest_obj is None:
            self.set_state(GraspStages.FAILED, 'no object')
            return False
        locked_obj = deepcopy(self.latest_obj)
        self._log_object_pose('locked target', locked_obj)

        self.set_state(GraspStages.PLAN_PREGRASP, 'compute pregrasp')
        camera_pose = self._lookup_camera_pose_base()
        pre = make_pregrasp_pose(
            locked_obj.pose_base,
            pregrasp_distance,
            camera_pose=camera_pose,
            mode=pregrasp_mode,
        )
        if self._pose_close_enough(self._current_tool_pose_base(), pre, pregrasp_reached_tolerance):
            self.set_state(GraspStages.MOVE_PREGRASP, 'already at pregrasp')
        else:
            self.set_state(GraspStages.MOVE_PREGRASP, 'planning')
            resp = move_pose(pre, False)
            if not resp.success:
                self.set_state(GraspStages.FAILED, 'pregrasp planning failed: ' + resp.message)
                return False
            self.set_state(GraspStages.MOVE_PREGRASP, 'moving')
            resp = move_pose(pre, True)
            if not resp.success:
                self.set_state(GraspStages.FAILED, resp.message)
                return False
            self._wait_for_motion_settle('pregrasp')

        if not self._command_gripper_position(
            set_gripper,
            open_position,
            'open gripper',
            self._cfg_float(gripper_cfg, 'open_wait_sec', 0.5),
        ):
            return False
        self._wait_for_motion_settle('before approach')

        if not self.active:
            self.set_state(GraspStages.IDLE, 'stopped before target approach')
            return False

        camera_pose = self._lookup_camera_pose_base() or camera_pose
        target_obj = self._target_for_approach(locked_obj, gcfg)
        self._log_object_pose('approach target', target_obj)
        approach = make_pregrasp_pose(
            target_obj.pose_base,
            final_offset,
            camera_pose=camera_pose,
            mode=pregrasp_mode,
        )
        self.set_state(GraspStages.APPROACH_TARGET, 'planning target approach')
        resp = move_pose_linear(approach, False)
        if not resp.success:
            self.set_state(GraspStages.FAILED, 'approach target planning failed: ' + resp.message)
            return False

        self.set_state(GraspStages.APPROACH_TARGET, 'moving to target')
        resp = move_pose_linear(approach, True)
        if not resp.success:
            self.set_state(GraspStages.FAILED, 'approach target failed: ' + resp.message)
            return False
        self._wait_for_motion_settle('approach')

        close_label = 'force-guided close' if bool(gripper_cfg.get('use_compliant_close', True)) else 'fixed gripper close'
        self.set_state(GraspStages.COMPLIANT_CLOSE, close_label)
        ok, message = self._close_gripper(gripper_cfg, set_gripper, close)
        if not ok:
            self.set_state(GraspStages.FAILED, message)
            return False

        self.set_state(GraspStages.LIFT_OBJECT, 'lifting')
        lift = make_lift_pose(approach, lift_height)
        if not self._plan_and_execute_pose(
            GraspStages.LIFT_OBJECT,
            'linear lift',
            lift,
            move_pose_linear,
            'lift',
        ):
            return False
        self.set_state(GraspStages.SUCCESS, 'grasp done', True)
        return True

    def _execute_grasp6d_plan(
        self,
        gcfg,
        gripper_cfg,
        open_position,
        move_pose,
        move_pose_linear,
        set_gripper,
        close,
        plan,
        strict_execute_pose=None,
    ):
        validation = validate_execution_plan(
            plan,
            _stamp_seconds(rospy.Time.now()),
            self._configured_plan_validity(gcfg),
            enforce_freshness=False,
        )
        if not validation.ok:
            if plan is not None:
                self._clear_grasp6d_authority(
                    expected_plan_id=str(getattr(plan, 'plan_id', '') or '')
                )
            self.set_state(
                GraspStages.FAILED,
                '%s: %s' % (validation.code, validation.reason),
            )
            return False
        if not self._execution_checkpoint(plan, gcfg, 'execution entry'):
            return False
        if self._position_only_execute_globally_enabled():
            self.set_state(
                GraspStages.FAILED,
                'POSITION_ONLY_FALLBACK_FORBIDDEN: rich 6D execution requires '
                '/robot/position_only_execute_enabled=false',
            )
            return False
        if not callable(strict_execute_pose):
            self.set_state(
                GraspStages.FAILED,
                'STRICT_CACHED_EXECUTOR_UNAVAILABLE: rich 6D execution requires '
                '/supervisor/execute_pose_strict before simulation or physical action',
            )
            return False

        pregrasp, approach, grasp, lift = split_rich_plan_poses(plan)
        if not self._execution_checkpoint(plan, gcfg, 'MuJoCo gate'):
            return False
        plan_phase = _plan_phase(plan)
        near_field_enabled = self._cfg_bool(
            gcfg,
            'near_field_replan_enabled',
            False,
        )
        near_field_required = self._cfg_bool(
            gcfg,
            'near_field_replan_required',
            True,
        )
        if (
            plan_phase == _FAR_FIELD_OBSERVATION_PLAN
            and not (near_field_enabled and near_field_required)
        ):
            self.set_state(
                GraspStages.FAILED,
                (
                    'NEAR_FIELD_REPLAN_CONFIG_INVALID: far-field observation '
                    'authority cannot execute contact stages'
                ),
            )
            return False
        defer_contact_gate = (
            plan_phase == _FAR_FIELD_OBSERVATION_PLAN
        )
        reused_observation_pose = None
        if defer_contact_gate:
            self.set_state(
                GraspStages.PLAN_PREGRASP,
                'far-field observation plan; contact simulation deferred '
                'until near-field replan',
            )
        elif not self._simulate_grasp6d_plan_if_required(
            gcfg,
            gripper_cfg,
            plan,
        ):
            return False
        if not self._execution_checkpoint(plan, gcfg, 'gripper open'):
            return False

        self.set_state(GraspStages.PLAN_PREGRASP, 'using 6D grasp plan')
        if defer_contact_gate:
            reuse_tolerance = max(
                0.0,
                self._cfg_float(
                    gcfg,
                    'observation_reuse_position_tolerance_m',
                    0.0,
                ),
            )
            current_tool_pose = self._current_tool_pose_base()
            observation_distance = self._pose_distance(
                current_tool_pose,
                pregrasp,
            )
            if (
                reuse_tolerance > 0.0
                and observation_distance <= reuse_tolerance
                and self._current_observation_view_reusable(plan, gcfg)
            ):
                reused_observation_pose = current_tool_pose
                pregrasp = current_tool_pose
                self.set_state(
                    GraspStages.MOVE_PREGRASP,
                    'already inside 6D observation region; preserving '
                    'reached camera view',
                )
                rospy.loginfo(
                    'Reusing reached observation pose: position delta '
                    '%.4fm <= %.4fm; candidate wrist orientation will not '
                    'replace the current stable camera view',
                    observation_distance,
                    reuse_tolerance,
                )
            else:
                self.set_state(
                    GraspStages.MOVE_PREGRASP,
                    'preflight planning 6D observation pregrasp',
                )
                preflight = move_pose(pregrasp, False)
                if not preflight.success:
                    self.set_state(
                        GraspStages.FAILED,
                        '6D observation pregrasp preflight failed: %s'
                        % preflight.message,
                    )
                    return False
                if not self._execution_checkpoint(
                    plan,
                    gcfg,
                    'gripper open after observation preflight',
                ):
                    return False
        if not self._command_gripper_position(
            set_gripper,
            open_position,
            'open gripper before 6D motion',
            self._cfg_float(gripper_cfg, 'open_wait_sec', 0.5),
            execution_plan=plan,
            gcfg=gcfg,
            skip_if_reached=True,
            reached_tolerance=self._cfg_float(
                gripper_cfg,
                'open_skip_tolerance_m',
                0.001,
            ),
        ):
            return False
        if not self._execution_checkpoint(plan, gcfg, 'pregrasp'):
            return False
        if reused_observation_pose is None:
            if not self._plan_and_execute_pose(
                GraspStages.MOVE_PREGRASP,
                '6D pregrasp',
                pregrasp,
                move_pose,
                '6D pregrasp',
                execution_plan_id=plan.plan_id,
                execution_plan=plan,
                gcfg=gcfg,
                execute_pose=strict_execute_pose,
                allow_post_failure_observation_validation=defer_contact_gate,
            ):
                return False
            if (
                defer_contact_gate
                and not self._converge_far_field_observation_endpoint(
                    pregrasp,
                    plan,
                    gcfg,
                    move_pose,
                    strict_execute_pose,
                )
            ):
                return False
        else:
            self._wait_for_motion_settle('reused 6D observation')
            if (
                self._cfg_bool(
                    gcfg,
                    'measured_endpoint_check_enabled',
                    False,
                )
                and not self._record_and_validate_measured_endpoint(
                    pregrasp,
                    plan,
                    gcfg,
                    'reused 6D observation',
                    required=False,
                )
            ):
                return False

        if defer_contact_gate:
            minimum_observation_stamp_ns = _stamp_nanoseconds(
                rospy.Time.now()
            )
            observation_range = (
                self._wait_for_fresh_observation_camera_target_range(
                    plan,
                    gcfg,
                    minimum_observation_stamp_ns,
                )
            )
            if not observation_range.ok:
                correction = self._maybe_execute_observation_camera_retreat(
                    plan,
                    gcfg,
                    observation_range,
                    move_pose,
                    strict_execute_pose,
                )
                if correction is None:
                    return False
                if correction:
                    minimum_observation_stamp_ns = _stamp_nanoseconds(
                        rospy.Time.now()
                    )
                    observation_range = (
                        self._wait_for_fresh_observation_camera_target_range(
                            plan,
                            gcfg,
                            minimum_observation_stamp_ns,
                        )
                    )
            if not observation_range.ok:
                self.set_state(
                    GraspStages.FAILED,
                    '%s: %s'
                    % (observation_range.code, observation_range.reason),
                )
                return False
            self._set_near_field_active(True)

        rebound = self._maybe_rebind_near_field_grasp6d_plan(
            gcfg,
            gripper_cfg,
            plan,
        )
        if rebound is None:
            return False
        if (
            defer_contact_gate
            and _plan_phase(rebound) != _CONTACT_EXECUTION_PLAN
        ):
            self.set_state(
                GraspStages.FAILED,
                'NEAR_FIELD_PLAN_PHASE_INVALID: observation authority cannot '
                'continue without a simulated contact execution plan',
            )
            return False
        if not strict_plan_id_equal(
            getattr(rebound, 'plan_id', ''),
            getattr(plan, 'plan_id', ''),
        ):
            prior_pregrasp = pregrasp
            plan = rebound
            pregrasp, approach, grasp, lift = split_rich_plan_poses(plan)
            if not self._execution_checkpoint(
                plan,
                gcfg,
                'near-field pregrasp',
            ):
                return False
            if _poses_same_enough(prior_pregrasp, pregrasp):
                self.set_state(
                    GraspStages.PLAN_PREGRASP,
                    'near-field plan reuses reached pregrasp',
                )
            else:
                if not self._plan_and_execute_pose(
                    GraspStages.MOVE_PREGRASP,
                    '6D near-field pregrasp',
                    pregrasp,
                    move_pose,
                    '6D near-field pregrasp',
                    execution_plan_id=plan.plan_id,
                    execution_plan=plan,
                    gcfg=gcfg,
                    execute_pose=strict_execute_pose,
                ):
                    return False

        refined = self._maybe_final_refine_grasp6d_plan(
            gcfg,
            gripper_cfg,
            plan,
        )
        if refined is None:
            return False
        final_refinement_applied = not strict_plan_id_equal(
            getattr(refined, 'plan_id', ''),
            getattr(plan, 'plan_id', ''),
        )
        if final_refinement_applied:
            reached_pregrasp = pregrasp
            plan = refined
            pregrasp, approach, grasp, lift = split_rich_plan_poses(plan)
            if not self._execution_checkpoint(
                plan,
                gcfg,
                'final visual correction pregrasp',
            ):
                return False
            if _poses_same_enough(reached_pregrasp, pregrasp):
                self.set_state(
                    GraspStages.PLAN_PREGRASP,
                    'final visual correction keeps reached pregrasp',
                )
            elif not self._plan_and_execute_pose(
                GraspStages.MOVE_PREGRASP,
                'bounded final visual correction',
                pregrasp,
                move_pose,
                'bounded final visual correction',
                execution_plan_id=plan.plan_id,
                execution_plan=plan,
                gcfg=gcfg,
                execute_pose=strict_execute_pose,
            ):
                return False

        if (
            final_refinement_applied
            and not self._confirm_final_center_alignment(plan, gcfg)
        ):
            return False

        if not self._execution_checkpoint(plan, gcfg, 'approach'):
            return False
        with self._grasp6d_plan_guard():
            self._bound_target_occlusion_allowed = True
        if not self._plan_and_execute_pose(
            GraspStages.APPROACH_TARGET,
            'linear 6D approach',
            approach,
            move_pose_linear,
            '6D approach',
            execution_plan_id=plan.plan_id,
            execution_plan=plan,
            gcfg=gcfg,
            execute_pose=move_pose_linear,
        ):
            return False

        if not self._execution_checkpoint(plan, gcfg, 'grasp pose'):
            return False
        if not self._plan_and_execute_pose(
            GraspStages.APPROACH_TARGET,
            'linear 6D grasp pose',
            grasp,
            move_pose_linear,
            '6D grasp pose',
            execution_plan_id=plan.plan_id,
            execution_plan=plan,
            gcfg=gcfg,
            execute_pose=move_pose_linear,
        ):
            return False

        if not self._execution_checkpoint(plan, gcfg, 'gripper close'):
            return False
        close_label = 'force-guided close' if bool(gripper_cfg.get('use_compliant_close', True)) else 'fixed gripper close'
        self.set_state(GraspStages.COMPLIANT_CLOSE, close_label)
        ok, message = self._close_gripper(
            gripper_cfg,
            set_gripper,
            close,
            execution_plan=plan,
            gcfg=gcfg,
        )
        if not ok:
            self.set_state(GraspStages.FAILED, message)
            return False

        if not self._execution_checkpoint(plan, gcfg, 'lift'):
            return False
        if not self._plan_and_execute_pose(
            GraspStages.LIFT_OBJECT,
            'linear 6D lift',
            lift,
            move_pose_linear,
            '6D lift',
            execution_plan_id=plan.plan_id,
            execution_plan=plan,
            gcfg=gcfg,
            execute_pose=move_pose_linear,
        ):
            return False
        if not self._execution_checkpoint(plan, gcfg, 'success acknowledgement'):
            return False
        self.set_state(GraspStages.SUCCESS, '6D grasp done', True)
        return True

    def _maybe_rebind_near_field_grasp6d_plan(
        self,
        gcfg,
        gripper_cfg,
        current_plan,
    ):
        if not self._cfg_bool(gcfg, 'near_field_replan_enabled', False):
            return current_plan
        required = self._cfg_bool(gcfg, 'near_field_replan_required', True)
        timeout = max(
            0.0,
            self._cfg_float(gcfg, 'near_field_replan_timeout_sec', 8.0),
        )
        poll_sec = max(
            0.02,
            self._cfg_float(gcfg, 'near_field_replan_poll_sec', 0.05),
        )
        slack_sec = max(
            0.0,
            self._cfg_float(
                gcfg,
                'near_field_replan_snapshot_slack_sec',
                0.20,
            ),
        )
        now = rospy.Time.now()
        minimum_stamp_ns = max(
            _stamp_nanoseconds(
                getattr(getattr(current_plan, 'header', None), 'stamp', None)
            ) + 1,
            _stamp_nanoseconds(now) - int(slack_sec * 1e9),
        )
        self.set_state(
            GraspStages.PLAN_PREGRASP,
            'waiting for near-field 6D preview',
        )
        if not self._request_near_field_preview_stream(gcfg):
            if required:
                self.set_state(
                    GraspStages.FAILED,
                    'NEAR_FIELD_REPLAN_UNAVAILABLE: cannot request 6D preview stream',
                )
                return None
            rospy.logwarn(
                'Near-field 6D preview stream request failed; keeping existing bound plan'
            )
            return current_plan

        start = time.monotonic()
        last_result = PlanValidationResult(
            False,
            'NEAR_FIELD_PLAN_WAITING',
            'waiting for fresh Preview rich plan',
        )
        while (
            self.active
            and not rospy.is_shutdown()
            and time.monotonic() - start <= timeout
        ):
            last_result, candidate = self._copy_near_field_preview_candidate(
                current_plan,
                minimum_stamp_ns,
                gcfg,
            )
            if last_result.ok:
                frozen = self._freeze_execution_plan(candidate)
                rospy.loginfo(
                    'Rebound 6D execution authority to near-field plan %s '
                    'from Preview stamp age %.3fs',
                    str(getattr(frozen, 'plan_id', '') or ''),
                    float(last_result.age_sec),
                )
                self.set_state(
                    GraspStages.PLAN_PREGRASP,
                    'near-field 6D plan rebound: %s'
                    % str(getattr(frozen, 'plan_id', '') or ''),
                )
                if not self._simulate_grasp6d_plan_if_required(
                    gcfg,
                    gripper_cfg,
                    frozen,
                ):
                    return None
                return frozen
            rospy.sleep(poll_sec)

        message = (
            'NEAR_FIELD_REPLAN_TIMEOUT: no fresh near-field 6D preview after '
            '%.1fs; last=%s: %s'
        ) % (timeout, last_result.code, last_result.reason)
        if required:
            self.set_state(GraspStages.FAILED, message)
            return None
        rospy.logwarn('%s; keeping existing bound plan', message)
        return current_plan

    def _request_near_field_preview_stream(self, gcfg):
        if not self._cfg_bool(gcfg, 'near_field_replan_request_stream', True):
            return True
        timeout = max(
            0.0,
            self._cfg_float(
                gcfg,
                'near_field_replan_service_timeout_sec',
                3.0,
            ),
        )
        try:
            rospy.wait_for_service('/grasp_6d/request_plan', timeout=timeout)
            response = rospy.ServiceProxy(
                '/grasp_6d/request_plan',
                TriggerZero,
            )(True)
        except Exception as exc:
            rospy.logwarn('Near-field 6D stream request failed: %s', exc)
            return False
        if bool(getattr(response, 'success', False)):
            return True
        rospy.logwarn(
            'Near-field 6D stream request rejected: %s',
            str(getattr(response, 'message', '') or ''),
        )
        return False

    def _check_final_refine_sequence(self, plan, gcfg):
        timeout = max(
            0.0,
            self._cfg_float(
                gcfg,
                'final_visual_refine_service_timeout_sec',
                3.0,
            ),
        )
        try:
            pregrasp, approach, grasp, lift = split_rich_plan_poses(plan)
            service_name = '/supervisor/check_pose_sequence_strict'
            rospy.wait_for_service(service_name, timeout=timeout)
            response = rospy.ServiceProxy(
                service_name,
                CheckPoseSequence,
            )(
                [pregrasp, approach, grasp, lift],
                ['pregrasp', 'approach', 'grasp', 'lift'],
                [False, True, True, True],
            )
        except Exception as exc:
            return PlanValidationResult(
                False,
                'FINAL_REFINE_MOVEIT_ERROR',
                str(exc),
            )
        if not bool(getattr(response, 'success', False)):
            failed_stage = str(
                getattr(response, 'failed_stage', '') or ''
            )
            message = str(getattr(response, 'message', '') or '')
            return PlanValidationResult(
                False,
                str(
                    getattr(response, 'failure_code', '')
                    or 'FINAL_REFINE_MOVEIT_UNREACHABLE'
                ),
                (
                    'final refinement %s failed: %s'
                    % (failed_stage or 'sequence', message)
                ),
            )
        return PlanValidationResult(
            True,
            reason=str(getattr(response, 'message', '') or ''),
        )

    def _maybe_final_refine_grasp6d_plan(
        self,
        gcfg,
        gripper_cfg,
        current_plan,
    ):
        if not self._cfg_bool(
            gcfg,
            'final_visual_refine_enabled',
            False,
        ):
            return current_plan
        required = self._cfg_bool(
            gcfg,
            'final_visual_refine_required',
            True,
        )
        timeout = max(
            0.0,
            self._cfg_float(
                gcfg,
                'final_visual_refine_timeout_sec',
                8.0,
            ),
        )
        poll_sec = max(
            0.02,
            self._cfg_float(
                gcfg,
                'final_visual_refine_poll_sec',
                0.05,
            ),
        )
        slack_sec = max(
            0.0,
            self._cfg_float(
                gcfg,
                'final_visual_refine_snapshot_slack_sec',
                0.20,
            ),
        )
        now = rospy.Time.now()
        minimum_stamp_ns = max(
            _stamp_nanoseconds(
                getattr(getattr(current_plan, 'header', None), 'stamp', None)
            ) + 1,
            _stamp_nanoseconds(now) - int(slack_sec * 1e9),
        )
        self.set_state(
            GraspStages.PLAN_PREGRASP,
            'waiting for bounded final visual refinement',
        )
        if not self._request_near_field_preview_stream(gcfg):
            if required:
                self.set_state(
                    GraspStages.FAILED,
                    'FINAL_REFINE_UNAVAILABLE: cannot request 6D preview stream',
                )
                return None
            rospy.logwarn(
                'Final visual refinement stream unavailable; keeping bound plan'
            )
            return current_plan

        translation_limit = max(
            0.0,
            self._cfg_float(
                gcfg,
                'final_visual_refine_max_translation_m',
                0.025,
            ),
        )
        yaw_limit = math.radians(
            max(
                0.0,
                self._cfg_float(
                    gcfg,
                    'final_visual_refine_max_yaw_deg',
                    10.0,
                ),
            )
        )
        roll_pitch_limit = math.radians(
            max(
                0.0,
                self._cfg_float(
                    gcfg,
                    'final_visual_refine_max_roll_pitch_change_deg',
                    4.0,
                ),
            )
        )
        center_fallback_enabled = self._cfg_bool(
            gcfg,
            'final_visual_refine_center_fallback_enabled',
            True,
        )
        center_fallback_delay = max(
            0.0,
            self._cfg_float(
                gcfg,
                'final_visual_refine_center_fallback_delay_sec',
                3.0,
            ),
        )
        center_required_samples = max(
            1,
            int(
                gcfg.get(
                    'final_visual_refine_center_fallback_required_samples',
                    5,
                )
            ),
        )
        center_max_jitter = max(
            0.0,
            self._cfg_float(
                gcfg,
                'final_visual_refine_center_fallback_max_jitter_m',
                0.006,
            ),
        )
        start = time.monotonic()
        center_samples = []
        last_center_token = None
        last_center_attempt_token = None
        last_result = PlanValidationResult(
            False,
            'FINAL_REFINE_WAITING',
            'waiting for a same-candidate close-range Preview plan',
        )
        while (
            self.active
            and not rospy.is_shutdown()
            and time.monotonic() - start <= timeout
        ):
            preview_result, observed = (
                self._copy_near_field_preview_candidate(
                    current_plan,
                    minimum_stamp_ns,
                    gcfg,
                )
            )
            if preview_result.ok:
                refinement, candidate, metrics = (
                    build_bounded_final_visual_refinement(
                        current_plan,
                        observed,
                        translation_limit,
                        yaw_limit,
                        roll_pitch_limit,
                    )
                )
                last_result = refinement
                if refinement.ok:
                    sequence_result = self._check_final_refine_sequence(
                        candidate,
                        gcfg,
                    )
                    last_result = sequence_result
                    if sequence_result.ok:
                        frozen = self._freeze_execution_plan(candidate)
                        rospy.loginfo(
                            (
                                'Bound final visual refinement plan %s: '
                                'translation=%.1fmm yaw=%.2fdeg '
                                'observed_roll_pitch=%.2fdeg; %s'
                            ),
                            str(getattr(frozen, 'plan_id', '') or ''),
                            float(metrics['translation_m']) * 1000.0,
                            math.degrees(float(metrics['yaw_rad'])),
                            math.degrees(
                                float(
                                    metrics[
                                        'observed_roll_pitch_change_rad'
                                    ]
                                )
                            ),
                            sequence_result.reason,
                        )
                        self.set_state(
                            GraspStages.PLAN_PREGRASP,
                            'bounded final visual refinement rebound: %s'
                            % str(getattr(frozen, 'plan_id', '') or ''),
                        )
                        if not self._simulate_grasp6d_plan_if_required(
                            gcfg,
                            gripper_cfg,
                            frozen,
                        ):
                            return None
                        return frozen
            else:
                last_result = preview_result
                if (
                    preview_result.code == 'NEAR_FIELD_PLAN_UNCHANGED'
                    and self._cfg_bool(
                        gcfg,
                        (
                            'final_visual_refine_accept_unchanged_after_'
                            'post_move_confirm'
                        ),
                        False,
                    )
                ):
                    rospy.loginfo(
                        (
                            'Final refinement candidate is unchanged; '
                            'requiring fresh post-move center evidence '
                            'before preserving plan %s'
                        ),
                        str(getattr(current_plan, 'plan_id', '') or ''),
                    )
                    if self._confirm_final_center_alignment(
                        current_plan,
                        gcfg,
                    ):
                        return current_plan
                    return None

            elapsed = time.monotonic() - start
            if center_fallback_enabled and elapsed >= center_fallback_delay:
                center_result, token, xyz, stamp = (
                    self._copy_final_center_sample(
                        current_plan,
                        minimum_stamp_ns,
                        gcfg,
                    )
                )
                if center_result.ok and token != last_center_token:
                    last_center_token = token
                    center_samples.append((xyz, stamp, token))
                    center_samples = center_samples[-center_required_samples:]
                elif not center_result.ok:
                    last_result = center_result

                if len(center_samples) >= center_required_samples:
                    median_xyz = tuple(
                        sorted(
                            float(item[0][axis])
                            for item in center_samples
                        )[len(center_samples) // 2]
                        for axis in range(3)
                    )
                    spread = max(
                        _vector3_norm(
                            tuple(
                                float(item[0][axis]) - median_xyz[axis]
                                for axis in range(3)
                            )
                        )
                        for item in center_samples
                    )
                    representative_stamp = center_samples[
                        len(center_samples) // 2
                    ][1]
                    newest_token = center_samples[-1][2]
                    if spread > center_max_jitter:
                        last_result = PlanValidationResult(
                            False,
                            'FINAL_REFINE_CENTER_UNSTABLE',
                            'surface-center spread %.4fm exceeds %.4fm'
                            % (spread, center_max_jitter),
                        )
                        center_samples = center_samples[
                            -max(1, center_required_samples - 1):
                        ]
                    elif newest_token != last_center_attempt_token:
                        last_center_attempt_token = newest_token
                        refinement, candidate, metrics = (
                            build_bounded_final_center_refinement(
                                current_plan,
                                median_xyz,
                                representative_stamp,
                                translation_limit,
                            )
                        )
                        last_result = refinement
                        if refinement.ok:
                            sequence_result = (
                                self._check_final_refine_sequence(
                                    candidate,
                                    gcfg,
                                )
                            )
                            last_result = sequence_result
                            if sequence_result.ok:
                                frozen = self._freeze_execution_plan(
                                    candidate
                                )
                                delta = metrics['translation_xyz']
                                rospy.loginfo(
                                    (
                                        'Bound final center-only refinement '
                                        'plan %s: translation=(%.1f, %.1f, '
                                        '%.1f)mm norm=%.1fmm samples=%d '
                                        'spread=%.1fmm orientation/width '
                                        'frozen; %s'
                                    ),
                                    str(
                                        getattr(frozen, 'plan_id', '') or ''
                                    ),
                                    float(delta[0]) * 1000.0,
                                    float(delta[1]) * 1000.0,
                                    float(delta[2]) * 1000.0,
                                    float(metrics['translation_m']) * 1000.0,
                                    len(center_samples),
                                    spread * 1000.0,
                                    sequence_result.reason,
                                )
                                self.set_state(
                                    GraspStages.PLAN_PREGRASP,
                                    (
                                        'bounded center-only final '
                                        'refinement rebound: %s'
                                    )
                                    % str(
                                        getattr(frozen, 'plan_id', '') or ''
                                    ),
                                )
                                if not (
                                    self._simulate_grasp6d_plan_if_required(
                                        gcfg,
                                        gripper_cfg,
                                        frozen,
                                    )
                                ):
                                    return None
                                return frozen
            rospy.sleep(poll_sec)

        message = (
            'FINAL_REFINE_TIMEOUT: no bounded same-candidate refinement after '
            '%.1fs; last=%s: %s'
        ) % (
            timeout,
            last_result.code,
            last_result.reason,
        )
        if required:
            self.set_state(GraspStages.FAILED, message)
            return None
        rospy.logwarn('%s; keeping existing bound plan', message)
        return current_plan

    def _copy_near_field_preview_candidate(
        self,
        current_plan,
        minimum_stamp_ns,
        gcfg,
    ):
        with self._grasp6d_plan_guard():
            preview = deepcopy(
                getattr(self, 'latest_grasp6d_preview_plan', None)
            )
        if preview is None:
            return (
                PlanValidationResult(
                    False,
                    'NEAR_FIELD_PLAN_MISSING',
                    'no Preview rich plan has arrived',
                ),
                None,
            )
        preview_id = str(getattr(preview, 'plan_id', '') or '')
        current_id = str(getattr(current_plan, 'plan_id', '') or '')
        current_phase = _plan_phase(current_plan)
        if (
            current_phase
            in (_FAR_FIELD_OBSERVATION_PLAN, _CONTACT_EXECUTION_PLAN)
            and _plan_phase(preview) != _CONTACT_EXECUTION_PLAN
        ):
            return (
                PlanValidationResult(
                    False,
                    'NEAR_FIELD_PLAN_PHASE_INVALID',
                    'Preview is not a contact execution plan',
                ),
                None,
            )
        if strict_plan_id_equal(preview_id, current_id):
            return (
                PlanValidationResult(
                    False,
                    'NEAR_FIELD_PLAN_UNCHANGED',
                    'Preview still matches the current execution plan',
                ),
                None,
            )
        stamp_ns = _stamp_nanoseconds(
            getattr(getattr(preview, 'header', None), 'stamp', None)
        )
        if stamp_ns <= 0 or stamp_ns < int(minimum_stamp_ns):
            return (
                PlanValidationResult(
                    False,
                    'NEAR_FIELD_PLAN_OLD',
                    'Preview source timestamp is older than the near-field request window',
                ),
                None,
            )
        now_sec = _stamp_seconds(rospy.Time.now())
        validation = validate_execution_plan(
            preview,
            now_sec,
            self._configured_plan_validity(gcfg),
        )
        if not validation.ok:
            return validation, None
        drift = self._target_drift_result(
            preview,
            gcfg,
            clear_authority_on_fail=False,
        )
        if not drift.ok:
            return drift, None
        observation_range = self._observation_camera_target_range_result(
            preview,
            gcfg,
        )
        if not observation_range.ok:
            return observation_range, None
        return validation, preview

    def _copy_final_center_sample(
        self,
        current_plan,
        minimum_stamp_ns,
        gcfg,
    ):
        with self._grasp6d_plan_guard():
            obj = deepcopy(getattr(self, 'latest_obj', None))
            stamp = deepcopy(getattr(self, 'latest_obj_time', None))
        if obj is None or not bool(getattr(obj, 'detected', False)):
            return (
                PlanValidationResult(
                    False,
                    'FINAL_REFINE_CENTER_MISSING',
                    'no detected live target is available',
                ),
                None,
                None,
                None,
            )
        stamp_ns = _stamp_nanoseconds(stamp)
        if stamp_ns <= 0 or stamp_ns < int(minimum_stamp_ns):
            return (
                PlanValidationResult(
                    False,
                    'FINAL_REFINE_CENTER_OLD',
                    'live target is older than the final-refinement window',
                ),
                None,
                None,
                None,
            )
        age = _stamp_seconds(rospy.Time.now()) - _stamp_seconds(stamp)
        validity = self._configured_target_observation_validity(gcfg)
        if (
            not math.isfinite(age)
            or age < 0.0
            or age > validity
        ):
            return (
                PlanValidationResult(
                    False,
                    'FINAL_REFINE_CENTER_STALE',
                    'live target age %.3fs is outside [0, %.3f]s'
                    % (age, validity),
                ),
                None,
                None,
                None,
            )
        plan_label = str(
            getattr(
                getattr(current_plan, 'object_geometry', None),
                'label',
                '',
            )
            or ''
        ).strip().lower()
        live_label = str(getattr(obj, 'label', '') or '').strip().lower()
        if not plan_label or live_label != plan_label:
            return (
                PlanValidationResult(
                    False,
                    'FINAL_REFINE_TARGET_CHANGED',
                    'live target label does not match the frozen plan',
                ),
                None,
                None,
                None,
            )
        camera_cfg = rospy.get_param('/camera', {})
        edge_margin_px = max(
            0,
            int(
                gcfg.get(
                    'final_visual_refine_center_fallback_edge_margin_px',
                    4,
                )
            ),
        )
        try:
            edge_clearance_px = self._object_bbox_edge_clearance_px(
                obj,
                camera_cfg.get('width', 640),
                camera_cfg.get('height', 480),
            )
        except Exception as exc:
            return (
                PlanValidationResult(
                    False,
                    'FINAL_REFINE_CENTER_BBOX_INVALID',
                    str(exc),
                ),
                None,
                None,
                None,
            )
        if edge_clearance_px < edge_margin_px:
            return (
                PlanValidationResult(
                    False,
                    'FINAL_REFINE_CENTER_CLIPPED',
                    (
                        'target bbox edge clearance %dpx is below %dpx; '
                        'a clipped mask centroid cannot drive motion'
                    )
                    % (edge_clearance_px, edge_margin_px),
                ),
                None,
                None,
                None,
            )
        xyz = self._object_xyz(obj)
        if xyz is None or not all(math.isfinite(value) for value in xyz):
            return (
                PlanValidationResult(
                    False,
                    'FINAL_REFINE_CENTER_INVALID',
                    'live target surface center is unavailable',
                ),
                None,
                None,
                None,
            )
        return PlanValidationResult(True), stamp_ns, xyz, stamp

    def _confirm_final_center_alignment(self, plan, gcfg):
        if not self._cfg_bool(
            gcfg,
            'final_visual_refine_post_move_confirm_enabled',
            True,
        ):
            return True
        timeout = max(
            0.0,
            self._cfg_float(
                gcfg,
                'final_visual_refine_post_move_confirm_timeout_sec',
                12.0,
            ),
        )
        poll_sec = max(
            0.02,
            self._cfg_float(
                gcfg,
                'final_visual_refine_poll_sec',
                0.05,
            ),
        )
        required_samples = max(
            1,
            int(
                gcfg.get(
                    'final_visual_refine_post_move_confirm_required_samples',
                    5,
                )
            ),
        )
        max_jitter = max(
            0.0,
            self._cfg_float(
                gcfg,
                'final_visual_refine_post_move_confirm_max_jitter_m',
                0.006,
            ),
        )
        max_residual = max(
            0.0,
            self._cfg_float(
                gcfg,
                'final_visual_refine_post_move_confirm_max_residual_m',
                0.006,
            ),
        )
        with self._grasp6d_plan_guard():
            initial_stamp = deepcopy(
                getattr(self, 'latest_obj_time', None)
            )
        minimum_stamp_ns = max(
            _stamp_nanoseconds(
                getattr(getattr(plan, 'header', None), 'stamp', None)
            ) + 1,
            _stamp_nanoseconds(initial_stamp) + 1,
        )
        self.set_state(
            GraspStages.PLAN_PREGRASP,
            'confirming post-move pregrasp center alignment',
        )
        samples = []
        last_token = None
        last_result = PlanValidationResult(
            False,
            'FINAL_REFINE_CONFIRM_WAITING',
            'waiting for post-correction target observations',
        )
        start = time.monotonic()
        while (
            self.active
            and not rospy.is_shutdown()
            and time.monotonic() - start <= timeout
        ):
            result, token, xyz, _stamp = self._copy_final_center_sample(
                plan,
                minimum_stamp_ns,
                gcfg,
            )
            last_result = result
            if result.ok and token != last_token:
                last_token = token
                samples.append(xyz)
                samples = samples[-required_samples:]
            if len(samples) >= required_samples:
                median_xyz = tuple(
                    sorted(float(item[axis]) for item in samples)[
                        len(samples) // 2
                    ]
                    for axis in range(3)
                )
                spread = max(
                    _vector3_norm(
                        tuple(
                            float(item[axis]) - median_xyz[axis]
                            for axis in range(3)
                        )
                    )
                    for item in samples
                )
                if spread > max_jitter:
                    last_result = PlanValidationResult(
                        False,
                        'FINAL_REFINE_CONFIRM_UNSTABLE',
                        'post-correction spread %.4fm exceeds %.4fm'
                        % (spread, max_jitter),
                    )
                    samples = samples[
                        -max(1, required_samples - 1):
                    ]
                else:
                    try:
                        planned_surface = _plan_support_surface_xyz(plan)
                    except Exception as exc:
                        self.set_state(
                            GraspStages.FAILED,
                            'FINAL_REFINE_CONFIRM_INVALID: %s' % exc,
                        )
                        return False
                    residual = tuple(
                        median_xyz[index] - planned_surface[index]
                        for index in range(3)
                    )
                    residual_norm = _vector3_norm(residual)
                    rospy.loginfo(
                        (
                            'Post-correction center confirmation: '
                            'residual=(%.1f, %.1f, %.1f)mm norm=%.1fmm '
                            'samples=%d spread=%.1fmm limit=%.1fmm'
                        ),
                        residual[0] * 1000.0,
                        residual[1] * 1000.0,
                        residual[2] * 1000.0,
                        residual_norm * 1000.0,
                        len(samples),
                        spread * 1000.0,
                        max_residual * 1000.0,
                    )
                    if residual_norm <= max_residual + 1e-12:
                        self.set_state(
                            GraspStages.PLAN_PREGRASP,
                            (
                                'post-move pregrasp center confirmed: '
                                'residual %.1f mm'
                            )
                            % (residual_norm * 1000.0),
                        )
                        return True
                    self.set_state(
                        GraspStages.FAILED,
                        (
                            'FINAL_REFINE_RESIDUAL: post-move pregrasp '
                            'center residual %.1fmm exceeds %.1fmm; '
                            'refusing contact'
                        )
                        % (
                            residual_norm * 1000.0,
                            max_residual * 1000.0,
                        ),
                    )
                    return False
            rospy.sleep(poll_sec)
        self.set_state(
            GraspStages.FAILED,
            (
                'FINAL_REFINE_CONFIRM_TIMEOUT: no stable post-correction '
                'center after %.1fs; last=%s: %s'
            )
            % (timeout, last_result.code, last_result.reason),
        )
        return False

    def _plan_and_execute_pose(
        self,
        stage,
        label,
        pose,
        move_pose,
        settle_reason,
        execution_plan_id=None,
        execution_plan=None,
        gcfg=None,
        execute_pose=None,
        allow_post_failure_observation_validation=False,
    ):
        bound_plan = execution_plan
        if bound_plan is None and execution_plan_id is not None:
            with self._grasp6d_plan_guard():
                frozen = getattr(self, '_bound_execution_plan', None)
                frozen_id = str(
                    getattr(self, '_bound_execution_plan_id', '') or ''
                )
                if frozen is not None and strict_plan_id_equal(
                    frozen_id, execution_plan_id
                ):
                    bound_plan = deepcopy(frozen)
        if bound_plan is not None:
            validation = self._validate_bound_plan(bound_plan, gcfg or {})
            if not validation.ok:
                self.set_state(
                    GraspStages.FAILED,
                    '%s: %s before planning %s'
                    % (validation.code, validation.reason, label),
                )
                return False
        elif execution_plan_id is not None:
            frozen = getattr(self, '_bound_execution_plan', None)
            code = (
                'EXECUTION_PLAN_NOT_FROZEN'
                if frozen is None
                else 'EXECUTION_PLAN_MISMATCH'
            )
            self.set_state(
                GraspStages.FAILED,
                '%s: no matching frozen execution authority before planning %s'
                % (code, label),
            )
            return False
        strict_rich_plan = (
            bound_plan is not None or execution_plan_id is not None
        )
        recoverable_observation_execution = (
            bool(allow_post_failure_observation_validation)
            and bound_plan is not None
            and _plan_phase(bound_plan) == _FAR_FIELD_OBSERVATION_PLAN
        )
        if recoverable_observation_execution:
            # This marker is consumed only if the following live
            # camera-to-target range check asks for another physical motion.
            # A successful new observation execution supersedes any older
            # failure marker for the same task.
            self._last_observation_execution_failure = None
        if (
            strict_rich_plan
            and self._position_only_execute_globally_enabled()
        ):
            self.set_state(
                GraspStages.FAILED,
                'POSITION_ONLY_FALLBACK_FORBIDDEN: rich 6D planning requires '
                '/robot/position_only_execute_enabled=false',
            )
            return False
        self.set_state(stage, 'planning ' + label)
        resp = move_pose(pose, False)
        if not resp.success:
            self.set_state(GraspStages.FAILED, '%s planning failed: %s' % (label, resp.message))
            return False
        if (
            is_position_only_fallback_message(getattr(resp, 'message', ''))
            and not self._position_only_execute_allowed(
                strict_rich_plan=strict_rich_plan
            )
        ):
            self.set_state(
                GraspStages.FAILED,
                position_only_rejection_message(label, getattr(resp, 'message', '')),
            )
            return False
        if (
            is_orientation_fallback_message(getattr(resp, 'message', ''))
            and not self._orientation_fallback_execute_allowed(
                strict_rich_plan=strict_rich_plan
            )
        ):
            self.set_state(
                GraspStages.FAILED,
                orientation_fallback_rejection_message(label, getattr(resp, 'message', '')),
            )
            return False
        self.set_state(stage, 'moving ' + label)
        if strict_rich_plan and not callable(execute_pose):
            self.set_state(
                GraspStages.FAILED,
                'STRICT_CACHED_EXECUTOR_UNAVAILABLE: %s cannot use the planning '
                'service or generic move_to_pose as an execution fallback' % label,
            )
            return False
        execute_pose = execute_pose or move_pose
        if bound_plan is not None:
            validation, resp = self._invoke_plan_bound_action(
                bound_plan,
                gcfg or {},
                label,
                lambda: execute_pose(pose, True),
            )
            if not validation.ok:
                self.set_state(
                    GraspStages.FAILED,
                    '%s: %s before moving %s'
                    % (validation.code, validation.reason, label),
                )
                return False
        else:
            resp = execute_pose(pose, True)
        if not resp.success:
            response_message = str(getattr(resp, 'message', '') or '')
            may_validate_actual_observation = (
                recoverable_observation_execution
                and is_observation_contract_recoverable_execution_failure(
                    response_message
                )
            )
            if not may_validate_actual_observation:
                self.set_state(
                    GraspStages.FAILED,
                    '%s failed: %s' % (label, response_message),
                )
                return False
            rospy.logwarn(
                '%s controller reported failure after strict submission; '
                'waiting for motion settle before the unchanged live '
                'observation-range contract decides acceptance: %s',
                label,
                response_message,
            )
            self._last_observation_execution_failure = {
                'plan_id': str(getattr(bound_plan, 'plan_id', '') or ''),
                'stage_label': str(label),
                'message': response_message,
            }
        settled = self._wait_for_motion_settle(settle_reason)
        if settled is False:
            self.set_state(
                GraspStages.FAILED,
                '%s feedback did not settle; refusing to overlap the next motion' % label,
            )
            return False
        if (
            strict_rich_plan
            and self._cfg_bool(
                gcfg or {},
                'measured_endpoint_check_enabled',
                False,
            )
        ):
            contact_phase = (
                bound_plan is not None
                and _plan_phase(bound_plan) == _CONTACT_EXECUTION_PLAN
            )
            if not self._record_and_validate_measured_endpoint(
                pose,
                bound_plan,
                gcfg or {},
                label,
                required=contact_phase,
            ):
                return False
        return True

    def _visual_retarget_6d_poses(self, reference_obj, poses, gcfg, stage_label, required=False):
        """Legacy entry point retained as a drift guard; never translate rich poses."""
        if not bool(gcfg.get('visual_retarget_enabled', False)):
            return [deepcopy(pose) for pose in poses], reference_obj
        if reference_obj is None or not bool(getattr(reference_obj, 'detected', False)):
            message = 'visual retarget unavailable: no object locked with the 6D plan'
            if required:
                self.set_state(GraspStages.FAILED, message)
                return None
            rospy.logwarn('%s', message)
            return [deepcopy(pose) for pose in poses], reference_obj

        live_obj = self._wait_for_stable_visual_target(reference_obj, gcfg, stage_label)
        if live_obj is None:
            message = 'target not visible/stable after %s; keeping last trusted 6D pose' % stage_label
            if required:
                self.set_state(GraspStages.FAILED, message)
                return None
            rospy.logwarn('%s', message)
            return [deepcopy(pose) for pose in poses], reference_obj

        delta = self._object_delta_xyz(reference_obj, live_obj)
        distance = math.sqrt(sum(float(value) ** 2 for value in delta))
        max_drift = self._configured_target_max_drift(gcfg)
        if distance > max_drift:
            self.set_state(
                GraspStages.FAILED,
                (
                    'TARGET_DRIFT after %s: target shifted %.3fm > %.3fm; '
                    'generate a new rich plan'
                ) % (stage_label, distance, max_drift),
            )
            return None

        rospy.loginfo(
            'Grasp rich-plan drift guard after %s: delta=(%.3f, %.3f, %.3f)m norm=%.3fm retarget=0',
            stage_label,
            float(delta[0]),
            float(delta[1]),
            float(delta[2]),
            distance,
        )
        self.set_state(
            GraspStages.APPROACH_TARGET,
            'rich-plan target stable after %s; drift %.1f mm; retarget disabled'
            % (stage_label, distance * 1000.0),
        )
        return [deepcopy(pose) for pose in poses], reference_obj

    def _wait_for_stable_visual_target(self, reference_obj, gcfg, stage_label):
        timeout = max(0.0, self._cfg_float(gcfg, 'visual_retarget_timeout_sec', 1.2))
        sample_period = max(0.02, self._cfg_float(gcfg, 'visual_retarget_sample_sec', 0.05))
        required = max(1, int(gcfg.get('visual_retarget_required_samples', 3)))
        max_jitter = max(0.0, self._cfg_float(gcfg, 'visual_retarget_max_jitter_m', 0.012))
        raw_max_age = max(0.0, self._cfg_float(gcfg, 'visual_retarget_raw_max_age_sec', 0.30))
        min_confidence = max(
            0.0,
            min(1.0, self._cfg_float(gcfg, 'visual_retarget_min_object_confidence', 0.35)),
        )
        samples = []
        last_token = None
        start = time.monotonic()

        while self.active and time.monotonic() - start < timeout and not rospy.is_shutdown():
            obj = getattr(self, 'latest_visual_obj', None)
            obj_time = getattr(self, 'latest_visual_obj_time', None)
            if obj is None or obj_time is None:
                obj = getattr(self, 'latest_obj', None)
                obj_time = getattr(self, 'latest_obj_time', None)
            token = self._time_token(obj_time)
            if (
                obj is not None
                and bool(getattr(obj, 'detected', False))
                and float(getattr(obj, 'confidence', 1.0) or 0.0) >= min_confidence
                and token is not None
                and token != last_token
                and self._raw_detection_is_fresh(raw_max_age)
                and self._same_target_label(reference_obj, obj)
            ):
                last_token = token
                xyz = self._object_xyz(obj)
                if xyz is not None:
                    samples.append((xyz, deepcopy(obj)))
                    samples = samples[-required:]
                    if len(samples) >= required:
                        center = tuple(
                            sorted(float(item[0][axis]) for item in samples)[len(samples) // 2]
                            for axis in range(3)
                        )
                        spread = max(
                            math.sqrt(sum((float(item[0][axis]) - center[axis]) ** 2 for axis in range(3)))
                            for item in samples
                        )
                        if spread <= max_jitter:
                            result = samples[-1][1]
                            point = result.pose_base.pose.position
                            point.x, point.y, point.z = center
                            rospy.loginfo(
                                (
                                    'Grasp live target after %s: xyz=(%.3f, %.3f, %.3f) '
                                    'samples=%d spread=%.3fm confidence=%.3f'
                                ),
                                stage_label,
                                center[0],
                                center[1],
                                center[2],
                                len(samples),
                                spread,
                                float(getattr(result, 'confidence', 1.0) or 0.0),
                            )
                            return result
                        samples = samples[-max(1, required - 1):]
            rospy.sleep(sample_period)
        return None

    def _raw_detection_is_fresh(self, max_age_sec):
        if not bool(getattr(self, 'latest_raw_detection', False)):
            return False
        stamp = getattr(self, 'latest_raw_detection_time', None)
        if stamp is None:
            return False
        if max_age_sec <= 0.0:
            return True
        try:
            return (rospy.Time.now() - stamp).to_sec() <= max_age_sec
        except Exception:
            return False

    @staticmethod
    def _time_token(stamp):
        if stamp is None:
            return None
        try:
            return float(stamp.to_sec())
        except Exception:
            return id(stamp)

    @staticmethod
    def _same_target_label(first, second):
        first_label = str(getattr(first, 'label', '') or '').strip().lower()
        second_label = str(getattr(second, 'label', '') or '').strip().lower()
        return bool(first_label and second_label and first_label == second_label)

    @staticmethod
    def _object_xyz(obj):
        try:
            p = obj.pose_base.pose.position
            return float(p.x), float(p.y), float(p.z)
        except Exception:
            return None

    @staticmethod
    def _object_bbox_edge_clearance_px(obj, image_width, image_height):
        width = int(image_width)
        height = int(image_height)
        x = int(getattr(obj, 'bbox_x'))
        y = int(getattr(obj, 'bbox_y'))
        bbox_width = int(getattr(obj, 'bbox_width'))
        bbox_height = int(getattr(obj, 'bbox_height'))
        if width <= 0 or height <= 0:
            raise ValueError('camera image dimensions must be positive')
        if bbox_width <= 0 or bbox_height <= 0:
            raise ValueError('target bounding box must be positive')
        right = width - (x + bbox_width)
        bottom = height - (y + bbox_height)
        return min(x, y, right, bottom)

    def _object_delta_xyz(self, first, second):
        a = self._object_xyz(first)
        b = self._object_xyz(second)
        if a is None or b is None:
            return 0.0, 0.0, 0.0
        return tuple(float(b[index]) - float(a[index]) for index in range(3))

    @staticmethod
    def _translate_pose(pose, delta_xyz):
        shifted = deepcopy(pose)
        shifted.pose.position.x += float(delta_xyz[0])
        shifted.pose.position.y += float(delta_xyz[1])
        shifted.pose.position.z += float(delta_xyz[2])
        return shifted

    def _configured_plan_validity(self, gcfg):
        if isinstance(gcfg, dict) and 'plan_validity_sec' in gcfg:
            return max(0.0, self._cfg_float(gcfg, 'plan_validity_sec', 2.0))
        try:
            default = rospy.get_param('/grasp_6d/plan_validity_sec', 2.0)
        except Exception:
            default = 2.0
        return max(0.0, float(default))

    def _configured_target_max_drift(self, gcfg):
        if isinstance(gcfg, dict) and 'target_max_drift_m' in gcfg:
            return max(0.0, self._cfg_float(gcfg, 'target_max_drift_m', 0.02))
        try:
            default = rospy.get_param('/grasp_6d/target_max_drift_m', 0.02)
        except Exception:
            default = 0.02
        return max(0.0, float(default))

    def _configured_target_observation_validity(self, gcfg):
        if isinstance(gcfg, dict) and 'target_observation_validity_sec' in gcfg:
            return max(
                0.0,
                self._cfg_float(
                    gcfg, 'target_observation_validity_sec', 3.0
                ),
            )
        try:
            default = rospy.get_param(
                '/grasp_6d/target_observation_validity_sec', 3.0
            )
        except Exception:
            default = 3.0
        return max(0.0, float(default))

    def _copy_requested_grasp6d_plan(self, plan_id, gcfg=None):
        requested_id = plan_id if isinstance(plan_id, str) else ''
        if not requested_id:
            return (
                PlanValidationResult(
                    False,
                    'PLAN_ID_MISSING',
                    '6D execution requires a non-empty plan_id',
                ),
                None,
            )
        with self._grasp6d_plan_guard():
            current = getattr(self, 'latest_grasp6d_plan', None)
            self._seed_grasp6d_watermark_locked(current)
            if current is None:
                return (
                    PlanValidationResult(
                        False,
                        'PLAN_MISSING',
                        'no current rich plan',
                    ),
                    None,
                )
            if self._grasp6d_watermark_tombstoned:
                return (
                    PlanValidationResult(
                        False,
                        'PLAN_REPLAYED',
                        'current rich plan source timestamp is tombstoned',
                    ),
                    None,
                )
            current_id = getattr(current, 'plan_id', None)
            if not strict_plan_id_equal(current_id, requested_id):
                return (
                    PlanValidationResult(
                        False,
                        'PLAN_ID_MISMATCH',
                        (
                            'requested plan_id %s is not current rich plan %s'
                            % (requested_id, str(current_id or ''))
                        ),
                    ),
                    None,
                )
            now_sec = _stamp_seconds(rospy.Time.now())
            validity_sec = self._configured_plan_validity(gcfg or {})
            if not self._near_field_replan_required_enabled(gcfg or {}):
                superseded = self._preview_supersedes_execution_locked(
                    requested_id,
                    now_sec,
                    validity_sec,
                )
                if superseded is not None:
                    return superseded, None
            copied = deepcopy(current)
            validation = validate_execution_plan(
                copied,
                now_sec,
                validity_sec,
            )
            if not validation.ok:
                self._clear_grasp6d_authority(
                    expected_plan_id=requested_id
                )
                return validation, None
        drift = self._bound_target_drift_result(copied, gcfg or {})
        if not drift.ok:
            return drift, None
        return validation, copied

    def _preview_supersedes_execution_locked(
        self,
        execution_plan_id,
        now_sec,
        validity_sec,
    ):
        preview = getattr(self, 'latest_grasp6d_preview_plan', None)
        if preview is None:
            return None
        preview_validation = validate_execution_plan(
            preview,
            now_sec,
            validity_sec,
        )
        if not preview_validation.ok:
            return None
        preview_id = str(getattr(preview, 'plan_id', '') or '')
        execution_id = str(execution_plan_id or '')
        if (
            not preview_id
            or not execution_id
            or strict_plan_id_equal(preview_id, execution_id)
        ):
            return None
        current = getattr(self, 'latest_grasp6d_plan', None)
        execution_stamp_ns = _stamp_nanoseconds(
            getattr(getattr(current, 'header', None), 'stamp', None)
        )
        preview_stamp_ns = _stamp_nanoseconds(
            getattr(getattr(preview, 'header', None), 'stamp', None)
        )
        if (
            execution_stamp_ns <= 0
            or preview_stamp_ns <= 0
            or preview_stamp_ns < execution_stamp_ns
        ):
            return None
        return PlanValidationResult(
            False,
            'PLAN_SUPERSEDED_BY_PREVIEW',
            (
                'Preview rich plan %s is newer than Execution %s; wait for '
                'execution plan sync before starting'
            )
            % (preview_id, execution_id),
        )

    def _near_field_replan_required_enabled(self, gcfg):
        return (
            self._cfg_bool(gcfg, 'near_field_replan_enabled', False)
            and self._cfg_bool(gcfg, 'near_field_replan_required', True)
        )

    def validate_plan_id_for_execution(self, plan_id, gcfg=None):
        requested_id = plan_id if isinstance(plan_id, str) else ''
        with self._grasp6d_plan_guard():
            current = getattr(self, 'latest_grasp6d_plan', None)
            self._seed_grasp6d_watermark_locked(current)
            if current is None:
                return PlanValidationResult(False, 'PLAN_MISSING', 'no current rich plan')
            if self._grasp6d_watermark_tombstoned:
                return PlanValidationResult(
                    False,
                    'PLAN_REPLAYED',
                    'current rich plan source timestamp is tombstoned',
                )
            current_id = getattr(current, 'plan_id', None)
            if not strict_plan_id_equal(current_id, requested_id):
                return PlanValidationResult(
                    False,
                    'PLAN_REPLACED',
                    'current rich plan id no longer matches execution copy',
            )
            now_sec = _stamp_seconds(rospy.Time.now())
            validity_sec = self._configured_plan_validity(gcfg or {})
            if not self._near_field_replan_required_enabled(gcfg or {}):
                superseded = self._preview_supersedes_execution_locked(
                    requested_id,
                    now_sec,
                    validity_sec,
                )
                if superseded is not None:
                    return superseded
            copied = deepcopy(current)
            result = validate_execution_plan(
                copied,
                now_sec,
                validity_sec,
                enforce_freshness=False,
            )
            if not result.ok:
                self._clear_grasp6d_authority(expected_plan_id=requested_id)
            return result

    def _validate_bound_plan_locked(self, plan, gcfg):
        if not bool(getattr(self, 'active', False)):
            return PlanValidationResult(
                False,
                'EXECUTION_CANCELLED',
                'grasp execution was stopped',
            )
        if bool(getattr(self, '_execution_authority_revoked', False)):
            return PlanValidationResult(
                False,
                'EXECUTION_AUTHORITY_REVOKED',
                'frozen execution authority was revoked by a hard safety event',
            )
        frozen = getattr(self, '_bound_execution_plan', None)
        frozen_id = str(getattr(self, '_bound_execution_plan_id', '') or '')
        frozen_digest = str(
            getattr(self, '_bound_execution_plan_digest', '') or ''
        )
        if frozen is None or not frozen_id or not frozen_digest:
            return PlanValidationResult(
                False,
                'EXECUTION_PLAN_NOT_FROZEN',
                'no immutable execution plan is bound to this execution',
            )
        supplied_id = str(getattr(plan, 'plan_id', '') or '')
        if not strict_plan_id_equal(supplied_id, frozen_id):
            return PlanValidationResult(
                False,
                'EXECUTION_PLAN_MISMATCH',
                'supplied execution plan does not match the frozen plan_id',
            )
        for candidate in (plan, frozen):
            integrity = validate_execution_plan(
                candidate,
                _stamp_seconds(rospy.Time.now()),
                self._configured_plan_validity(gcfg),
                enforce_freshness=False,
            )
            if not integrity.ok:
                self._execution_authority_revoked = True
                return integrity
        try:
            supplied_digest = self._execution_plan_digest(plan)
            current_frozen_digest = self._execution_plan_digest(frozen)
        except Exception as exc:
            self._execution_authority_revoked = True
            return PlanValidationResult(
                False,
                'EXECUTION_PLAN_INTEGRITY_CHANGED',
                str(exc),
            )
        if (
            supplied_digest != frozen_digest
            or current_frozen_digest != frozen_digest
        ):
            self._execution_authority_revoked = True
            return PlanValidationResult(
                False,
                'EXECUTION_PLAN_INTEGRITY_CHANGED',
                'execution plan content no longer matches its frozen digest',
            )
        return self._bound_target_drift_result(frozen, gcfg)

    def _validate_bound_plan(self, plan, gcfg):
        with self._grasp6d_plan_guard():
            return self._validate_bound_plan_locked(plan, gcfg)

    def _invoke_plan_bound_action(self, plan, gcfg, stage_label, action):
        """Validate and commit one synchronous physical action atomically.

        The service call is the physical commit boundary. Holding the same
        RLock as rich-plan callbacks makes the action/replacement order equal
        to their lock-acquisition order. Planning and settling stay outside.
        """
        del stage_label
        with self._grasp6d_plan_guard():
            validation = self._validate_bound_plan_locked(plan, gcfg or {})
            if not validation.ok:
                return validation, None
            return validation, action()

    def _plan_geometry_center_xyz(self, plan):
        try:
            point = plan.object_geometry.pose_base.position
            values = (float(point.x), float(point.y), float(point.z))
        except Exception:
            return None
        return values if all(math.isfinite(value) for value in values) else None

    @staticmethod
    def _bound_object_pose_from_plan(plan):
        geometry = deepcopy(plan.object_geometry)
        reference = ObjectPose()
        reference.header = deepcopy(plan.header)
        reference.detected = bool(geometry.valid)
        reference.label = str(geometry.label or '')
        reference.confidence = 1.0
        reference.pose_base.header = deepcopy(plan.header)
        reference.pose_base.pose = deepcopy(geometry.pose_base)
        return reference

    def _target_drift_result(
        self,
        plan,
        gcfg,
        clear_authority_on_fail=True,
    ):
        def fail(code, reason):
            if clear_authority_on_fail:
                self._clear_grasp6d_authority(
                    expected_plan_id=str(getattr(plan, 'plan_id', '') or '')
                )
            return PlanValidationResult(False, code, reason)

        latest_obj = getattr(self, 'latest_obj', None)
        if latest_obj is None:
            if self._target_occlusion_allowed_locked():
                return PlanValidationResult(True)
            return fail('TARGET_LOST', 'live target observation is missing')
        if not bool(getattr(latest_obj, 'detected', False)):
            if self._target_occlusion_allowed_locked():
                return PlanValidationResult(True)
            return fail('TARGET_LOST', 'live target is not detected')
        observation_time = getattr(self, 'latest_obj_time', None)
        if observation_time is None:
            if self._target_occlusion_allowed_locked():
                return PlanValidationResult(True)
            return fail(
                'TARGET_STALE',
                'live target observation timestamp is missing',
            )
        observation_age = (
            _stamp_seconds(rospy.Time.now())
            - _stamp_seconds(observation_time)
        )
        observation_validity = self._configured_target_observation_validity(
            gcfg
        )
        if (
            not math.isfinite(observation_age)
            or observation_age < 0.0
            or observation_age > observation_validity
        ):
            if self._target_occlusion_allowed_locked():
                return PlanValidationResult(True)
            return fail(
                'TARGET_STALE',
                'live target observation age %.3fs is outside [0, %.3f]s'
                % (observation_age, observation_validity),
            )
        plan_label = str(
            getattr(getattr(plan, 'object_geometry', None), 'label', '') or ''
        ).strip().lower()
        live_label = str(getattr(latest_obj, 'label', '') or '').strip().lower()
        if not plan_label or not live_label:
            return fail(
                'TARGET_LABEL_MISSING',
                'plan and live target labels must both be non-empty',
            )
        if plan_label != live_label:
            return fail(
                'TARGET_LABEL_MISMATCH',
                'live target label %s does not match plan geometry label %s'
                % (live_label, plan_label),
            )
        plan_center = self._plan_geometry_center_xyz(plan)
        live_center = self._object_xyz(latest_obj)
        if plan_center is None or live_center is None:
            return fail(
                'OBB_INVALID',
                'plan or live target center is unavailable',
            )
        drift = math.sqrt(
            sum(
                (float(live_center[index]) - float(plan_center[index])) ** 2
                for index in range(3)
            )
        )
        maximum = self._configured_target_max_drift(gcfg)
        if drift > maximum:
            return fail(
                'TARGET_DRIFT',
                'live target drift %.3fm exceeds %.3fm' % (drift, maximum),
            )
        return PlanValidationResult(True)

    def _bound_target_drift_result(self, plan, gcfg):
        return self._target_drift_result(
            plan,
            gcfg,
            clear_authority_on_fail=True,
        )

    def _target_occlusion_allowed_locked(self):
        if not bool(getattr(self, 'active', False)):
            return False
        if getattr(self, '_bound_execution_plan', None) is None:
            return False
        if not bool(getattr(self, '_bound_target_occlusion_allowed', False)):
            return False
        if bool(getattr(self, '_execution_authority_revoked', False)):
            return False
        latest_obj = getattr(self, 'latest_obj', None)
        return latest_obj is not None and bool(getattr(latest_obj, 'detected', False))

    def _fresh_grasp6d_plan(self, gcfg):
        with self._grasp6d_plan_guard():
            current = getattr(self, 'latest_grasp6d_plan', None)
            plan = deepcopy(current) if current is not None else None
        result = validate_execution_plan(
            plan,
            _stamp_seconds(rospy.Time.now()),
            self._configured_plan_validity(gcfg),
        )
        if not result.ok:
            if plan is not None:
                self._clear_grasp6d_authority(
                    expected_plan_id=str(getattr(plan, 'plan_id', '') or '')
                )
            rospy.logwarn('Rejected rich 6D plan %s: %s', result.code, result.reason)
            return None
        drift = self._bound_target_drift_result(plan, gcfg)
        if not drift.ok:
            self._clear_grasp6d_authority(expected_plan_id=plan.plan_id)
            rospy.logwarn('Rejected rich 6D plan %s: %s', drift.code, drift.reason)
            return None
        return plan

    def _execution_checkpoint(self, plan, gcfg, stage_label):
        validation = self._validate_bound_plan(plan, gcfg)
        if not validation.ok:
            stage = (
                GraspStages.IDLE
                if validation.code == 'EXECUTION_CANCELLED'
                else GraspStages.FAILED
            )
            self.set_state(
                stage,
                '%s: %s before %s'
                % (validation.code, validation.reason, stage_label),
            )
            return False
        return True

    def _simulate_grasp6d_plan_if_required(self, gcfg, gripper_cfg, plan):
        twin_cfg = rospy.get_param('/mujoco_digital_twin', {})
        if not isinstance(twin_cfg, dict):
            self.set_state(
                GraspStages.FAILED,
                'MUJOCO_GATE_CONFIG_INVALID: /mujoco_digital_twin must be a mapping',
            )
            return False
        for key in ('enabled', 'execution_gate_enabled'):
            value = twin_cfg.get(key)
            if type(value) is not bool or value is not True:
                self.set_state(
                    GraspStages.FAILED,
                    (
                        'MUJOCO_GATE_CONFIG_INVALID: '
                        '/mujoco_digital_twin/%s must be the boolean true'
                    )
                    % key,
                )
                return False

        self.set_state(GraspStages.PLAN_PREGRASP, 'MuJoCo digital twin checking 6D plan')
        raw_audit_path = twin_cfg.get(
            'audit_output_path', _MUJOCO_AUDIT_DEFAULT_PATH
        )
        planning_audit_path = rospy.get_param(
            '/grasp_6d/remote/gate_audit_output_path',
            '',
        )
        try:
            audit_path, _planning_audit_path = (
                validate_distinct_audit_output_paths(
                    raw_audit_path,
                    planning_audit_path,
                )
            )
        except AuditPathConflictError as exc:
            self.set_state(
                GraspStages.FAILED,
                'MUJOCO_AUDIT_PATH_CONFLICT: %s' % exc,
            )
            return False
        except (TypeError, ValueError, OSError) as exc:
            self.set_state(
                GraspStages.FAILED,
                'MUJOCO_AUDIT_PATH_INVALID: %s' % exc,
            )
            return False
        audit = _new_mujoco_execution_audit(plan)

        def finish(ok, code, reason, score=None, stage=GraspStages.FAILED):
            try:
                reference = _finalize_mujoco_execution_audit(
                    audit,
                    audit_path,
                    ok,
                    code,
                    reason,
                    score=score,
                )
            except Exception as audit_error:
                message = (
                    '%s: %s; MUJOCO_AUDIT_WRITE_FAILED: %s audit_path=%s'
                    % (
                        _bounded_status_code(code),
                        _bounded_status_reason(reason),
                        _bounded_status_reason(audit_error),
                        _bounded_text(str(audit_path), 256),
                    )
                )
                # A passing simulation cannot become motion authority unless
                # its exact request/response association is durably recorded.
                self.set_state(
                    GraspStages.FAILED if ok else stage,
                    message,
                )
                return False

            reference_text = _mujoco_audit_reference_text(reference)
            if ok:
                self.set_state(
                    GraspStages.PLAN_PREGRASP,
                    'MuJoCo simulation passed score=%.3f; %s'
                    % (float(score), reference_text),
                )
                return True
            self.set_state(
                stage,
                '%s: %s; %s'
                % (_bounded_status_code(code), _bounded_status_reason(reason), reference_text),
            )
            return False

        require_object = bool(twin_cfg.get('require_object_pose', True))
        object_pose = self._bound_object_pose_from_plan(plan)
        if require_object and (object_pose is None or not bool(getattr(object_pose, 'detected', False))):
            return finish(
                False,
                'OBB_INVALID',
                'MuJoCo simulation blocked: rich plan has no valid object geometry',
            )

        if not bool(twin_cfg.get('send_joint_state_in_request', False)):
            return finish(
                False,
                'WSL_UNAVAILABLE',
                'WSL_UNAVAILABLE: strict MuJoCo gate requires current joint state in every request',
            )
        joint_state = deepcopy(getattr(self, 'latest_joint_state', None))
        if joint_state is None:
            return finish(
                False,
                'WSL_UNAVAILABLE',
                'WSL_UNAVAILABLE: no /joint_states for strict MuJoCo request payload',
            )
        joint_names = list(getattr(joint_state, 'name', ()) or ())
        joint_positions = list(getattr(joint_state, 'position', ()) or ())

        try:
            payload = build_mujoco_payload(
                plan,
                joint_names,
                joint_positions,
                twin_cfg,
            )
            _record_mujoco_payload(audit, payload)
        except Exception as exc:
            audit['payload']['build_error'] = _exception_audit_record(exc)
            return finish(
                False,
                'WSL_UNAVAILABLE',
                'invalid MuJoCo request payload: %s' % exc,
            )

        request_plan_id = str(getattr(plan, 'plan_id', '') or '')
        payload_plan_id = payload.get('plan_id') if isinstance(payload, dict) else None
        if not strict_plan_id_equal(payload_plan_id, request_plan_id):
            return finish(
                False,
                'PLAN_ID_MISMATCH',
                'MuJoCo payload plan_id does not exactly match the bound rich plan',
            )

        response = None
        request_error = None
        try:
            client = MujocoDigitalTwinClient(
                twin_cfg.get('server_url', 'http://172.23.132.97:8000'),
                timeout_sec=self._cfg_float(twin_cfg, 'timeout_sec', 20.0),
            )
            response = client.simulate_grasp(payload)
        except Exception as exc:
            request_error = exc
            audit['response']['network_error'] = _exception_audit_record(exc)
        else:
            _record_mujoco_response(audit, response)

        post_network = self._validate_bound_plan(plan, gcfg)
        audit['authority_after_network'] = _validation_audit_record(post_network)
        if not post_network.ok:
            stage = (
                GraspStages.IDLE
                if post_network.code == 'EXECUTION_CANCELLED'
                else GraspStages.FAILED
            )
            return finish(
                False,
                post_network.code,
                '%s after MuJoCo network return' % post_network.reason,
                stage=stage,
            )

        if request_error is not None:
            return finish(
                False,
                'WSL_UNAVAILABLE',
                'MuJoCo simulation request failed: %s' % request_error,
            )

        if audit['response'].get('strict_json_serializable') is not True:
            gate = PlanValidationResult(
                False,
                'WSL_UNAVAILABLE',
                'MuJoCo response must be fully strict-JSON serializable',
            )
            audit['gate_validation'] = _validation_audit_record(gate)
            return finish(
                False,
                gate.code,
                gate.reason,
            )

        try:
            gate = validate_mujoco_gate_response(
                response,
                request_plan_id,
                twin_cfg.get('min_score', 80),
                expected_candidate_source=plan.candidate_source,
                expected_candidate_source_lineage=(
                    plan.candidate_source_lineage
                ),
            )
        except Exception as exc:
            gate = PlanValidationResult(
                False,
                'WSL_UNAVAILABLE',
                'MuJoCo response validation failed: %s' % exc,
            )
        audit['gate_validation'] = _validation_audit_record(gate)
        if not gate.ok:
            return finish(
                False,
                gate.code,
                gate.reason,
                score=getattr(gate, 'score', None),
            )
        return finish(
            True,
            'MUJOCO_GATE_PASSED',
            'MuJoCo response exactly matched the bound rich plan and all gates passed',
            score=gate.score,
        )

    @staticmethod
    def _pose_array_item_as_stamped(plan, index):
        pose = PoseStamped()
        pose.header = plan.header
        pose.pose = deepcopy(plan.poses[index])
        return pose

    def _current_tool_pose_base(self):
        tf_buffer = getattr(self, 'tf_buffer', None)
        if tf_buffer is None:
            return None
        base_frame = str(rospy.get_param('/handeye/base_frame', 'base_link'))
        tool_frame = str(rospy.get_param('/handeye/parent_frame', 'tool0'))
        timeout = float(rospy.get_param('/handeye/tf_timeout_sec', 0.2))
        try:
            transform = tf_buffer.lookup_transform(
                base_frame,
                tool_frame,
                rospy.Time(0),
                rospy.Duration(timeout),
            )
        except Exception as exc:
            rospy.logwarn_throttle(2.0, 'Grasp tool TF lookup failed: %s', exc)
            return None
        pose = PoseStamped()
        pose.header = transform.header
        pose.pose.position.x = transform.transform.translation.x
        pose.pose.position.y = transform.transform.translation.y
        pose.pose.position.z = transform.transform.translation.z
        pose.pose.orientation = transform.transform.rotation
        return pose

    def _current_camera_pose_base(self):
        tf_buffer = getattr(self, 'tf_buffer', None)
        if tf_buffer is None:
            return None
        base_frame = str(rospy.get_param('/handeye/base_frame', 'base_link'))
        camera_frame = str(
            rospy.get_param(
                '/handeye/camera_frame',
                rospy.get_param('/handeye/child_frame', 'camera_link'),
            )
        )
        timeout = float(rospy.get_param('/handeye/tf_timeout_sec', 0.2))
        try:
            transform = tf_buffer.lookup_transform(
                base_frame,
                camera_frame,
                rospy.Time(0),
                rospy.Duration(timeout),
            )
        except Exception as exc:
            rospy.logwarn_throttle(
                2.0,
                'Grasp camera TF lookup failed: %s',
                exc,
            )
            return None
        pose = PoseStamped()
        pose.header = transform.header
        pose.pose.position.x = transform.transform.translation.x
        pose.pose.position.y = transform.transform.translation.y
        pose.pose.position.z = transform.transform.translation.z
        pose.pose.orientation = transform.transform.rotation
        return pose

    def _observation_camera_target_range_result(
        self,
        plan,
        gcfg,
        target_xyz=None,
    ):
        if not self._cfg_bool(
            gcfg,
            'observation_camera_target_range_check_enabled',
            False,
        ):
            return PlanValidationResult(True)
        minimum = self._cfg_float(
            gcfg,
            'observation_camera_target_min_distance_m',
            0.180,
        )
        maximum = self._cfg_float(
            gcfg,
            'observation_camera_target_max_distance_m',
            0.220,
        )
        if (
            not math.isfinite(minimum)
            or not math.isfinite(maximum)
            or minimum <= 0.0
            or maximum < minimum
        ):
            return PlanValidationResult(
                False,
                'OBSERVATION_CAMERA_RANGE_CONFIG_INVALID',
                'camera-to-target observation range must be finite, positive, '
                'and ordered',
            )
        camera_pose = self._current_camera_pose_base()
        target = (
            self._plan_geometry_center_xyz(plan)
            if target_xyz is None
            else tuple(float(value) for value in target_xyz)
        )
        if camera_pose is None or target is None:
            return PlanValidationResult(
                False,
                'OBSERVATION_CAMERA_RANGE_UNAVAILABLE',
                'current base-to-camera TF or fresh target centre is unavailable',
            )
        camera = self._pose_position_xyz(camera_pose)
        distance = math.sqrt(
            sum(
                (float(camera[index]) - float(target[index])) ** 2
                for index in range(3)
            )
        )
        if not math.isfinite(distance):
            return PlanValidationResult(
                False,
                'OBSERVATION_CAMERA_RANGE_UNAVAILABLE',
                'current camera-to-target distance is not finite',
            )
        self._last_observation_camera_range_evidence = {
            'camera_position_m': [float(value) for value in camera],
            'target_position_m': [float(value) for value in target],
            'distance_m': float(distance),
        }
        if distance < minimum or distance > maximum:
            return PlanValidationResult(
                False,
                'OBSERVATION_CAMERA_TARGET_OUT_OF_RANGE',
                (
                    'fresh camera-to-target distance %.4fm is outside the '
                    'inclusive observation interval [%.4f, %.4f]m'
                )
                % (distance, minimum, maximum),
            )
        rospy.loginfo(
            (
                'Fresh camera observation accepted: camera-to-target '
                'distance=%.4fm inside [%.4f, %.4f]m'
            ),
            distance,
            minimum,
            maximum,
        )
        return PlanValidationResult(
            True,
            reason='camera-to-target distance %.4fm is inside observation range'
            % distance,
        )

    def _current_observation_view_reusable(self, plan, gcfg):
        if not self._cfg_bool(
            gcfg,
            'observation_camera_target_range_check_enabled',
            False,
        ):
            return True
        with self._grasp6d_plan_guard():
            obj = deepcopy(getattr(self, 'latest_obj', None))
        if obj is None or not bool(getattr(obj, 'detected', False)):
            return False
        target = self._object_xyz(obj)
        if target is None:
            return False
        result = self._observation_camera_target_range_result(
            plan,
            gcfg,
            target_xyz=target,
        )
        if not result.ok:
            rospy.loginfo(
                'Reached observation pose is not reusable: %s: %s',
                result.code,
                result.reason,
            )
        return bool(result.ok)

    def _wait_for_fresh_observation_camera_target_range(
        self,
        plan,
        gcfg,
        minimum_stamp_ns,
    ):
        """Validate the reached camera view before requesting contact plans."""

        if not self._cfg_bool(
            gcfg,
            'observation_camera_target_range_check_enabled',
            False,
        ):
            return PlanValidationResult(True)
        timeout = max(
            0.0,
            self._configured_target_observation_validity(gcfg),
        )
        poll_sec = max(
            0.02,
            self._cfg_float(gcfg, 'near_field_replan_poll_sec', 0.05),
        )
        plan_label = str(
            getattr(
                getattr(plan, 'object_geometry', None),
                'label',
                '',
            )
            or ''
        ).strip().lower()
        last_result = PlanValidationResult(
            False,
            'OBSERVATION_CAMERA_RANGE_UNAVAILABLE',
            'waiting for a target observation captured after reaching the camera view',
        )
        start = time.monotonic()
        while (
            self.active
            and not rospy.is_shutdown()
            and time.monotonic() - start <= timeout
        ):
            with self._grasp6d_plan_guard():
                obj = deepcopy(getattr(self, 'latest_obj', None))
                stamp = deepcopy(getattr(self, 'latest_obj_time', None))
            if obj is None or not bool(getattr(obj, 'detected', False)):
                last_result = PlanValidationResult(
                    False,
                    'OBSERVATION_CAMERA_RANGE_UNAVAILABLE',
                    'fresh target is not detected at the reached camera view',
                )
                rospy.sleep(poll_sec)
                continue
            stamp_ns = _stamp_nanoseconds(stamp)
            if stamp_ns < int(minimum_stamp_ns):
                last_result = PlanValidationResult(
                    False,
                    'OBSERVATION_CAMERA_RANGE_OLD',
                    'target observation predates the reached camera view',
                )
                rospy.sleep(poll_sec)
                continue
            age = _stamp_seconds(rospy.Time.now()) - _stamp_seconds(stamp)
            if (
                not math.isfinite(age)
                or age < 0.0
                or age > timeout
            ):
                last_result = PlanValidationResult(
                    False,
                    'OBSERVATION_CAMERA_RANGE_STALE',
                    'target observation age %.3fs is outside [0, %.3f]s'
                    % (age, timeout),
                )
                rospy.sleep(poll_sec)
                continue
            live_label = str(getattr(obj, 'label', '') or '').strip().lower()
            if not plan_label or live_label != plan_label:
                return PlanValidationResult(
                    False,
                    'OBSERVATION_CAMERA_TARGET_CHANGED',
                    'fresh target label does not match the frozen observation plan',
                )
            target = self._object_xyz(obj)
            if target is None or not all(
                math.isfinite(value) for value in target
            ):
                return PlanValidationResult(
                    False,
                    'OBSERVATION_CAMERA_RANGE_UNAVAILABLE',
                    'fresh target centre is unavailable at the reached camera view',
                )
            plan_center = self._plan_geometry_center_xyz(plan)
            if plan_center is None:
                return PlanValidationResult(
                    False,
                    'OBB_INVALID',
                    'frozen observation target centre is unavailable',
                )
            drift_m = math.sqrt(
                sum(
                    (float(target[index]) - float(plan_center[index])) ** 2
                    for index in range(3)
                )
            )
            maximum_drift_m = self._configured_target_max_drift(gcfg)
            if drift_m > maximum_drift_m:
                return PlanValidationResult(
                    False,
                    'TARGET_DRIFT',
                    'live target drift %.3fm exceeds %.3fm'
                    % (drift_m, maximum_drift_m),
                )
            result = self._observation_camera_target_range_result(
                plan,
                gcfg,
                target_xyz=target,
            )
            evidence = getattr(
                self,
                '_last_observation_camera_range_evidence',
                None,
            )
            if isinstance(evidence, dict):
                evidence['source_stamp_ns'] = int(stamp_ns)
            return result
        return PlanValidationResult(
            False,
            'OBSERVATION_CAMERA_RANGE_TIMEOUT',
            'no fresh post-arrival target observation within %.3fs; last=%s: %s'
            % (timeout, last_result.code, last_result.reason),
        )

    def _maybe_execute_observation_camera_retreat(
        self,
        plan,
        gcfg,
        range_result,
        move_pose,
        strict_execute_pose,
    ):
        if not self._cfg_bool(
            gcfg,
            'observation_camera_target_retreat_correction_enabled',
            False,
        ):
            return False
        if range_result.code != 'OBSERVATION_CAMERA_TARGET_OUT_OF_RANGE':
            return False
        evidence = getattr(
            self,
            '_last_observation_camera_range_evidence',
            None,
        )
        endpoint = getattr(self, '_last_measured_endpoint_sample', None)
        if not isinstance(evidence, dict) or not isinstance(endpoint, dict):
            self.set_state(
                GraspStages.FAILED,
                'OBSERVATION_CAMERA_CORRECTION_EVIDENCE_INVALID: live range '
                'or measured endpoint evidence is unavailable',
            )
            return None
        try:
            distance = float(evidence['distance_m'])
            minimum = self._cfg_float(
                gcfg,
                'observation_camera_target_min_distance_m',
                0.180,
            )
            maximum = self._cfg_float(
                gcfg,
                'observation_camera_target_max_distance_m',
                0.220,
            )
            nominal = self._cfg_float(
                gcfg,
                'observation_camera_target_nominal_distance_m',
                0.200,
            )
            endpoint_plan_id = str(endpoint.get('plan_id', '') or '')
            endpoint_phase = str(endpoint.get('plan_phase', '') or '')
            error_vector = endpoint['position_error_vector_m']
            camera_position = evidence['camera_position_m']
            target_position = evidence['target_position_m']
        except (KeyError, TypeError, ValueError, OverflowError) as exc:
            self.set_state(
                GraspStages.FAILED,
                'OBSERVATION_CAMERA_CORRECTION_EVIDENCE_INVALID: %s' % exc,
            )
            return None
        execution_failure = getattr(
            self,
            '_last_observation_execution_failure',
            None,
        )
        if (
            isinstance(execution_failure, dict)
            and str(execution_failure.get('plan_id', '') or '')
            == str(getattr(plan, 'plan_id', '') or '')
        ):
            self.set_state(
                GraspStages.FAILED,
                (
                    'OBSERVATION_CORRECTION_AFTER_FAILED_EXECUTION_FORBIDDEN: '
                    '%s controller failed and live camera-target distance '
                    '%.4fm remains outside [%.4f, %.4f]m; refusing another '
                    'physical trajectory'
                )
                % (
                    str(execution_failure.get('stage_label', '') or 'observation'),
                    distance,
                    minimum,
                    maximum,
                ),
            )
            return None
        if minimum <= distance <= maximum:
            return False
        if (
            endpoint_plan_id != str(getattr(plan, 'plan_id', '') or '')
            or endpoint_phase != _FAR_FIELD_OBSERVATION_PLAN
        ):
            self.set_state(
                GraspStages.FAILED,
                'OBSERVATION_CAMERA_CORRECTION_EVIDENCE_INVALID: endpoint '
                'sample is not bound to the active far-field plan',
            )
            return None
        current_tool = self._current_tool_pose_base()
        if current_tool is None:
            self.set_state(
                GraspStages.FAILED,
                'OBSERVATION_CAMERA_CORRECTION_EVIDENCE_INVALID: current '
                'tool pose is unavailable',
            )
            return None
        try:
            correction_pose, audit = make_observation_camera_retreat_pose(
                current_tool,
                camera_position,
                target_position,
                nominal,
                error_vector,
            )
        except (TypeError, ValueError, OverflowError) as exc:
            self.set_state(
                GraspStages.FAILED,
                'OBSERVATION_CAMERA_CORRECTION_GEOMETRY_INVALID: %s' % exc,
            )
            return None
        rospy.loginfo(
            'Measured observation radial-correction audit: %s',
            json.dumps(audit, sort_keys=True, separators=(',', ':')),
        )
        self.set_state(
            GraspStages.MOVE_PREGRASP,
            'planning one measured camera-range radial correction',
        )
        preflight = move_pose(correction_pose, False)
        if not bool(getattr(preflight, 'success', False)):
            self.set_state(
                GraspStages.FAILED,
                'OBSERVATION_CAMERA_CORRECTION_UNREACHABLE: %s'
                % str(getattr(preflight, 'message', '') or ''),
            )
            return None
        if not self._plan_and_execute_pose(
            GraspStages.MOVE_PREGRASP,
            'single measured camera-range radial correction',
            correction_pose,
            move_pose,
            'single measured camera-range radial correction',
            execution_plan_id=plan.plan_id,
            execution_plan=plan,
            gcfg=gcfg,
            execute_pose=strict_execute_pose,
            allow_post_failure_observation_validation=False,
        ):
            return None
        return True

    @staticmethod
    def _pose_distance(first, second):
        if first is None or second is None:
            return float('inf')
        try:
            a = first.pose.position
            b = second.pose.position
            return math.sqrt(
                (float(a.x) - float(b.x)) ** 2
                + (float(a.y) - float(b.y)) ** 2
                + (float(a.z) - float(b.z)) ** 2
            )
        except Exception:
            return float('inf')

    def _pose_close_enough(self, first, second, tolerance_m):
        return self._pose_distance(first, second) <= max(0.0, float(tolerance_m))

    @staticmethod
    def _pose_orientation_error_rad(first, second):
        if first is None or second is None:
            return float('inf')
        try:
            first_q = _quaternion_xyzw(first.pose)
            second_q = _quaternion_xyzw(second.pose)
        except (AttributeError, TypeError, ValueError):
            return float('inf')
        dot = abs(sum(a * b for a, b in zip(first_q, second_q)))
        return 2.0 * math.acos(max(-1.0, min(1.0, dot)))

    def _measured_endpoint_residual(self, requested):
        actual = self._current_tool_pose_base()
        return (
            actual,
            self._pose_distance(actual, requested),
            self._pose_orientation_error_rad(actual, requested),
        )

    @staticmethod
    def _pose_position_xyz(pose_stamped):
        point = pose_stamped.pose.position
        values = (float(point.x), float(point.y), float(point.z))
        if not all(math.isfinite(value) for value in values):
            raise ValueError('pose position is not finite')
        return values

    def _converge_far_field_observation_endpoint(
        self,
        requested,
        plan,
        gcfg,
        move_pose,
        strict_execute_pose,
    ):
        del move_pose, strict_execute_pose
        if not self._cfg_bool(
            gcfg,
            'measured_endpoint_check_enabled',
            False,
        ):
            return True
        # Far-field observation is a camera-view acquisition stage, not a
        # contact endpoint.  Record the measured tool0 error for near-field
        # uncertainty propagation, but do not reuse the 6 mm contact gate and
        # do not issue another same-pose motion.  The following fresh preview
        # is accepted only when the measured camera-to-target distance is in
        # the configured observation interval.
        return self._record_and_validate_measured_endpoint(
            requested,
            plan,
            gcfg,
            '6D far-field observation',
            required=False,
        )

    def _wait_for_measured_endpoint_contract(
        self,
        requested,
        position_tolerance,
        orientation_tolerance_deg,
        reason,
    ):
        """Wait for measured FK, not elapsed time, to satisfy one endpoint.

        The hardware endpoint outer loop can make several bounded corrections
        after a FollowJointTrajectory action reports success.  A generic
        low-delta joint window can occur between those iterations.  Reuse the
        existing motion-settle time/sample budget, but authorize progress only
        after consecutive live tool0 samples satisfy the unchanged Cartesian
        endpoint contract.
        """
        timeout = max(
            0.0,
            float(rospy.get_param('/grasp/motion_settle_timeout_sec', 2.0)),
        )
        sample_period = max(
            0.02,
            float(rospy.get_param('/grasp/motion_settle_sample_sec', 0.05)),
        )
        required = max(
            1,
            int(rospy.get_param('/grasp/motion_settle_required_samples', 3)),
        )
        start = time.monotonic()
        stable_count = 0
        last_position_error = float('inf')
        last_orientation_error_deg = float('inf')

        while self.active and not rospy.is_shutdown():
            _actual, position_error, orientation_error = (
                self._measured_endpoint_residual(requested)
            )
            orientation_error_deg = math.degrees(orientation_error)
            last_position_error = float(position_error)
            last_orientation_error_deg = float(orientation_error_deg)
            sample_is_valid = (
                math.isfinite(position_error)
                and math.isfinite(orientation_error_deg)
            )
            if (
                sample_is_valid
                and position_error <= position_tolerance
                and orientation_error_deg <= orientation_tolerance_deg
            ):
                stable_count += 1
                if stable_count >= required:
                    rospy.loginfo(
                        (
                            'Measured endpoint contract stable after %.2fs for '
                            '%s: residual=%.4fm / %.2fdeg within %.4fm / '
                            '%.2fdeg for %d consecutive sample(s)'
                        ),
                        time.monotonic() - start,
                        reason,
                        position_error,
                        orientation_error_deg,
                        position_tolerance,
                        orientation_tolerance_deg,
                        stable_count,
                    )
                    return (
                        True,
                        float(position_error),
                        float(orientation_error_deg),
                    )
            else:
                stable_count = 0

            elapsed = time.monotonic() - start
            if timeout <= 0.0 or elapsed >= timeout:
                break
            rospy.sleep(min(sample_period, max(0.0, timeout - elapsed)))

        rospy.logwarn(
            (
                'Measured endpoint contract timeout after %.2fs for %s: '
                'last residual=%.4fm / %.2fdeg, required %.4fm / %.2fdeg'
            ),
            time.monotonic() - start,
            reason,
            last_position_error,
            last_orientation_error_deg,
            position_tolerance,
            orientation_tolerance_deg,
        )
        return (
            False,
            last_position_error,
            last_orientation_error_deg,
        )

    def _record_and_validate_measured_endpoint(
        self,
        requested,
        plan,
        gcfg,
        label,
        required,
    ):
        actual, position_error, orientation_error = (
            self._measured_endpoint_residual(requested)
        )
        if (
            not math.isfinite(position_error)
            or not math.isfinite(orientation_error)
        ):
            self.set_state(
                GraspStages.FAILED,
                'MEASURED_ENDPOINT_UNAVAILABLE: %s has no finite tool0 feedback'
                % label,
            )
            return False

        now = rospy.Time.now()
        now_sec = (
            float(now.to_sec())
            if hasattr(now, 'to_sec')
            else float(now.secs)
            + float(getattr(now, 'nsecs', 0)) * 1e-9
        )
        try:
            requested_position = self._pose_position_xyz(requested)
            actual_position = self._pose_position_xyz(actual)
            requested_quaternion = _quaternion_xyzw(requested.pose)
            actual_quaternion = _quaternion_xyzw(actual.pose)
        except (AttributeError, TypeError, ValueError):
            self.set_state(
                GraspStages.FAILED,
                'MEASURED_ENDPOINT_UNAVAILABLE: %s has invalid pose evidence'
                % label,
            )
            return False
        sample = {
            'stamp_sec': now_sec,
            'position_error_m': float(position_error),
            'position_error_vector_m': [
                float(actual_position[index] - requested_position[index])
                for index in range(3)
            ],
            'orientation_error_rad': float(orientation_error),
            'requested_position_m': list(requested_position),
            'actual_position_m': list(actual_position),
            'requested_quaternion_xyzw': list(requested_quaternion),
            'actual_quaternion_xyzw': list(actual_quaternion),
            'stage_label': str(label),
            'plan_id': str(getattr(plan, 'plan_id', '') or ''),
            'plan_phase': _plan_phase(plan),
        }
        self._last_measured_endpoint_sample = deepcopy(sample)
        try:
            rospy.set_param(
                '/grasp_6d/runtime_execution_error',
                sample,
            )
        except Exception as exc:
            self.set_state(
                GraspStages.FAILED,
                'MEASURED_ENDPOINT_PUBLISH_FAILED: %s' % exc,
            )
            return False

        rospy.loginfo(
            (
                'Measured rich-6D endpoint for %s: position_error=%.4fm '
                'orientation_error=%.2fdeg phase=%s'
            ),
            label,
            position_error,
            math.degrees(orientation_error),
            _plan_phase(plan),
        )
        if not bool(required):
            return True

        position_tolerance = max(
            0.0,
            self._cfg_float(
                gcfg,
                'measured_endpoint_position_tolerance_m',
                0.006,
            ),
        )
        orientation_tolerance_deg = max(
            0.0,
            self._cfg_float(
                gcfg,
                'measured_endpoint_orientation_tolerance_deg',
                5.0,
            ),
        )
        if position_error > position_tolerance:
            self.set_state(
                GraspStages.FAILED,
                (
                    'MEASURED_ENDPOINT_POSITION_ERROR: %s tool0 residual '
                    '%.4fm exceeds %.4fm; refusing the next contact motion'
                )
                % (label, position_error, position_tolerance),
            )
            return False
        if math.degrees(orientation_error) > orientation_tolerance_deg:
            self.set_state(
                GraspStages.FAILED,
                (
                    'MEASURED_ENDPOINT_ORIENTATION_ERROR: %s tool0 residual '
                    '%.2fdeg exceeds %.2fdeg; refusing the next contact motion'
                )
                % (
                    label,
                    math.degrees(orientation_error),
                    orientation_tolerance_deg,
                ),
            )
            return False
        return True

    @staticmethod
    def _cfg_float(cfg, key, default):
        try:
            if isinstance(cfg, dict) and key in cfg:
                return float(cfg.get(key))
        except Exception:
            pass
        return float(default)

    @staticmethod
    def _cfg_bool(cfg, key, default):
        value = default
        if isinstance(cfg, dict) and key in cfg:
            value = cfg.get(key)
        if type(value) is bool:
            return value
        if isinstance(value, (int, float)) and not isinstance(value, bool):
            return bool(value)
        text = str(value).strip().lower()
        if text in ('1', 'true', 'yes', 'on'):
            return True
        if text in ('0', 'false', 'no', 'off', ''):
            return False
        return bool(default)

    def _object_distance(self, first, second):
        try:
            return self._pose_distance(first.pose_base, second.pose_base)
        except Exception:
            return float('inf')

    def _target_for_approach(self, locked_obj, gcfg):
        latest = getattr(self, 'latest_obj', None)
        if latest is None:
            return deepcopy(locked_obj)
        max_refine = self._cfg_float(gcfg, 'max_locked_target_refine_m', 0.06)
        distance = self._object_distance(locked_obj, latest)
        if distance <= max(0.0, max_refine):
            if distance > 1e-6:
                rospy.loginfo('Grasp refined locked target by %.3f m before approach', distance)
            return deepcopy(latest)
        rospy.logwarn(
            'Grasp kept locked target; latest detection jumped %.3f m > %.3f m',
            distance,
            max_refine,
        )
        return deepcopy(locked_obj)

    def _log_object_pose(self, label, obj):
        try:
            p = obj.pose_base.pose.position
            rospy.loginfo(
                'Grasp %s: xyz=(%.3f, %.3f, %.3f) confidence=%.3f',
                label,
                float(p.x),
                float(p.y),
                float(p.z),
                float(getattr(obj, 'confidence', 1.0) or 0.0),
            )
        except Exception:
            rospy.loginfo('Grasp %s: pose unavailable', label)

    def _wait_for_motion_settle(self, reason='motion'):
        timeout = max(0.0, float(rospy.get_param('/grasp/motion_settle_timeout_sec', 2.0)))
        min_sec = max(0.0, float(rospy.get_param('/grasp/motion_settle_min_sec', 0.8)))
        sample_period = max(0.02, float(rospy.get_param('/grasp/motion_settle_sample_sec', 0.05)))
        epsilon = max(0.0, float(rospy.get_param('/grasp/motion_settle_position_epsilon_rad', 0.0025)))
        required = max(1, int(rospy.get_param('/grasp/motion_settle_required_samples', 3)))
        if timeout <= 0.0:
            return True

        start = time.monotonic()
        previous = self._joint_positions_tuple()
        stable_count = 0
        if previous is None:
            rospy.logwarn_throttle(2.0, 'Grasp settle fallback sleep: no joint state for %s', reason)
            rospy.sleep(min_sec if min_sec > 0.0 else min(timeout, 0.2))
            return False

        while self.active and time.monotonic() - start < timeout and not rospy.is_shutdown():
            rospy.sleep(sample_period)
            current = self._joint_positions_tuple()
            elapsed = time.monotonic() - start
            if current is None:
                stable_count = 0
                continue
            max_delta = max(abs(float(a) - float(b)) for a, b in zip(previous, current))
            if max_delta <= epsilon and elapsed >= min_sec:
                stable_count += 1
                if stable_count >= required:
                    rospy.loginfo('Grasp motion settled after %.2fs for %s', elapsed, reason)
                    return True
            else:
                stable_count = 0
            previous = current

        rospy.logwarn('Grasp motion settle timeout after %.2fs for %s', time.monotonic() - start, reason)
        return False

    @staticmethod
    def _position_only_execute_globally_enabled():
        return bool(rospy.get_param('/robot/position_only_execute_enabled', False))

    @staticmethod
    def _position_only_execute_allowed(strict_rich_plan=False):
        if bool(strict_rich_plan):
            return False
        return GraspTaskNode._position_only_execute_globally_enabled()

    @staticmethod
    def _orientation_fallback_execute_allowed(strict_rich_plan=False):
        if bool(strict_rich_plan):
            return False
        return bool(rospy.get_param('/grasp/accept_orientation_fallback', False))

    def _command_gripper_position(
        self,
        set_gripper,
        position,
        label,
        wait_sec,
        execution_plan=None,
        gcfg=None,
        skip_if_reached=False,
        reached_tolerance=0.0,
    ):
        already_reached = (
            bool(skip_if_reached)
            and self._gripper_position_reached(
                position,
                reached_tolerance,
            )
        )
        try:
            if execution_plan is not None:
                validation, resp = self._invoke_plan_bound_action(
                    execution_plan,
                    gcfg or {},
                    label,
                    (
                        (lambda: None)
                        if already_reached
                        else lambda: set_gripper(float(position))
                    ),
                )
                if not validation.ok:
                    self.set_state(
                        GraspStages.FAILED,
                        '%s: %s before %s'
                        % (validation.code, validation.reason, label),
                    )
                    return False
            else:
                resp = (
                    None
                    if already_reached
                    else set_gripper(float(position))
                )
        except Exception as exc:
            self.set_state(GraspStages.FAILED, '%s failed: %s' % (label, exc))
            return False
        if already_reached:
            rospy.loginfo(
                '%s skipped: gripper feedback already within %.4fm of %.4fm',
                label,
                max(0.0, float(reached_tolerance)),
                float(position),
            )
            return True
        if not bool(getattr(resp, 'success', True)):
            self.set_state(GraspStages.FAILED, '%s failed: %s' % (label, getattr(resp, 'message', '')))
            return False
        delay = max(0.0, float(wait_sec))
        if delay > 0.0:
            rospy.sleep(delay)
        return True

    def _gripper_position_reached(self, target_position, tolerance):
        msg = getattr(self, 'latest_joint_state', None)
        names = list(getattr(msg, 'name', []) or [])
        positions = list(getattr(msg, 'position', []) or [])
        try:
            index = names.index('right_finger')
            actual = float(positions[index])
            target = float(target_position)
            limit = max(0.0, float(tolerance))
        except (ValueError, IndexError, TypeError):
            return False
        return (
            math.isfinite(actual)
            and math.isfinite(target)
            and abs(actual - target) <= limit
        )

    def _close_gripper(
        self,
        gripper_cfg,
        set_gripper,
        close,
        execution_plan=None,
        gcfg=None,
    ):
        if bool(gripper_cfg.get('use_compliant_close', True)):
            if close is None:
                return False, 'compliant close service is unavailable'
            if execution_plan is not None:
                validation, resp = self._invoke_plan_bound_action(
                    execution_plan,
                    gcfg or {},
                    'compliant gripper close',
                    lambda: close(execute=True),
                )
                if not validation.ok:
                    return (
                        False,
                        '%s: %s' % (validation.code, validation.reason),
                    )
            else:
                resp = close(execute=True)
            return bool(resp.success), getattr(resp, 'message', '')

        close_position = self._cfg_float(
            gripper_cfg,
            'simple_close_position_m',
            self._cfg_float(gripper_cfg, 'close_limit_m', 0.05),
        )
        wait_sec = self._cfg_float(gripper_cfg, 'simple_close_wait_sec', 0.8)
        ok = self._command_gripper_position(
            set_gripper,
            close_position,
            'fixed gripper close',
            wait_sec,
            execution_plan=execution_plan,
            gcfg=gcfg,
        )
        return ok, 'fixed gripper close command published' if ok else 'fixed gripper close failed'

    def _joint_positions_tuple(self):
        msg = getattr(self, 'latest_joint_state', None)
        positions = getattr(msg, 'position', None)
        if not positions:
            return None
        try:
            return tuple(float(v) for v in positions)
        except Exception:
            return None

    def _lookup_camera_pose_base(self):
        tf_buffer = getattr(self, 'tf_buffer', None)
        if tf_buffer is None:
            return None
        base_frame = str(rospy.get_param('/handeye/base_frame', 'base_link'))
        camera_frame = str(rospy.get_param('/handeye/camera_frame', rospy.get_param('/camera/frame_id', 'camera_link')))
        timeout = float(rospy.get_param('/handeye/tf_timeout_sec', 0.2))
        try:
            transform = tf_buffer.lookup_transform(
                base_frame,
                camera_frame,
                rospy.Time(0),
                rospy.Duration(timeout),
            )
        except Exception as exc:
            rospy.logwarn_throttle(2.0, 'Grasp camera TF lookup failed: %s', exc)
            return None
        pose = PoseStamped()
        pose.header = transform.header
        pose.pose.position.x = transform.transform.translation.x
        pose.pose.position.y = transform.transform.translation.y
        pose.pose.position.z = transform.transform.translation.z
        pose.pose.orientation = transform.transform.rotation
        return pose

if __name__ == '__main__':
    rospy.init_node('grasp_task_node')
    GraspTaskNode()
    rospy.spin()
