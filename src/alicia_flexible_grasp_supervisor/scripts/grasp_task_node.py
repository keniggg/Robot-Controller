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
from std_msgs.msg import Bool
from alicia_flexible_grasp_supervisor.msg import Grasp6DPlan, ObjectPose, GraspState
from alicia_flexible_grasp_supervisor.srv import StartGrasp, StartGraspResponse, StopGrasp, StopGraspResponse, SetTargetPose, SetFloat
from alicia_flexible_grasp.grasp.grasp_state_machine import GraspStages, STATE_NAMES
from alicia_flexible_grasp.grasp.grasp_pose_generator import make_pregrasp_pose, make_lift_pose
from alicia_flexible_grasp.grasp.rich_plan_integrity import (
    plan_id_matches_content,
    required_open_width_is_valid,
    stamp_nanoseconds as _stamp_nanoseconds,
    stamp_seconds as _stamp_seconds,
    strict_plan_id_equal,
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
            'score': None,
            'failure_code': None,
            'failure_reason': None,
            'failure_reason_length': None,
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
    record['score'] = _strict_json_number(response.get('score'))
    raw_code = response.get('failure_code')
    record['failure_code'] = raw_code if isinstance(raw_code, str) else None
    raw_reason = response.get('failure_reason')
    record['failure_reason'] = raw_reason if isinstance(raw_reason, str) else None
    record['failure_reason_length'] = (
        len(raw_reason) if isinstance(raw_reason, str) else None
    )
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
        candidate_width = float(plan.candidate_width_m)
        required_width = float(plan.required_open_width_m)
    except Exception:
        return PlanValidationResult(False, 'PLAN_MALFORMED', 'plan scalar fields are invalid')
    if (
        not math.isfinite(score)
        or not math.isfinite(candidate_width)
        or candidate_width < 0.0
    ):
        return PlanValidationResult(False, 'PLAN_MALFORMED', 'plan score or candidate width is non-finite')
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
        self._last_execution_plan_event = ''
        self._grasp6d_watermark_stamp_ns = 0
        self._grasp6d_watermark_plan_id = ''
        self._grasp6d_watermark_tombstoned = False
        self._grasp6d_plan_lock = threading.RLock()
        self._start_lock = threading.RLock()
        self._start_inflight = False
        self.latest_joint_state = None
        self.latest_raw_detection = False
        self.latest_raw_detection_time = None
        self.active = False
        self.stage = GraspStages.IDLE
        self.tf_buffer = None
        self.tf_listener = None
        if tf2_ros is not None and bool(rospy.get_param('/handeye/use_tf', True)):
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.pub = rospy.Publisher('/grasp/state', GraspState, queue_size=10)
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
        rospy.Subscriber('/perception/raw_object_detected', Bool, self.raw_detection_cb, queue_size=1)
        rospy.Service('/grasp/start', StartGrasp, self.start_cb)
        rospy.Service('/grasp/stop', StopGrasp, self.stop_cb)
        rospy.loginfo('GraspTaskNode ready')

    def obj_cb(self, msg):
        if not msg.detected:
            self.latest_visual_obj = None
            self.latest_visual_obj_time = None
            with self._grasp6d_plan_guard():
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
                rospy.logwarn(
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
            self._last_execution_plan_event = 'EXECUTION_FROZEN'
        return frozen

    def _clear_bound_execution_plan(self):
        with self._grasp6d_plan_guard():
            self._bound_execution_plan = None
            self._bound_execution_plan_id = ''
            self._bound_execution_plan_digest = ''
            self._execution_authority_revoked = False

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

    def start_cb(self, req):
        if not bool(getattr(req, 'execute', False)):
            return StartGraspResponse(False, 'execute=false')
        gcfg = rospy.get_param('/grasp', {})
        bound_plan = None
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
        try:
            result = self.execute(grasp6d_plan=bound_plan)
            return StartGraspResponse(result, 'success' if result else 'failed')
        except Exception as exc:
            self.set_state(GraspStages.FAILED, str(exc), False)
            return StartGraspResponse(False, str(exc))
        finally:
            with self._start_guard():
                with self._grasp6d_plan_guard():
                    self.active = False
                    self._start_inflight = False
                    self._clear_bound_execution_plan()

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
        self.set_state(GraspStages.EMERGENCY_STOP if req.emergency else GraspStages.IDLE, 'stop requested')
        return StopGraspResponse(True, 'stop requested')

    def execute(self, grasp6d_plan=None):
        gcfg = rospy.get_param('/grasp', {})
        use_grasp6d_plan = bool(gcfg.get('use_grasp6d_plan', False))
        strict_execute_pose = None
        if use_grasp6d_plan:
            rospy.wait_for_service('/supervisor/check_pose_strict', timeout=10)
            rospy.wait_for_service('/supervisor/execute_pose_strict', timeout=10)
            move_pose = rospy.ServiceProxy(
                '/supervisor/check_pose_strict', SetTargetPose
            )
            strict_execute_pose = rospy.ServiceProxy(
                '/supervisor/execute_pose_strict', SetTargetPose
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
        drift = self._bound_target_drift_result(plan, gcfg)
        if not drift.ok:
            self.set_state(
                GraspStages.FAILED,
                '%s: %s' % (drift.code, drift.reason),
            )
            return False
        with self._grasp6d_plan_guard():
            if getattr(self, '_bound_execution_plan', None) is None:
                try:
                    plan = self._freeze_execution_plan(plan)
                except (TypeError, ValueError, AttributeError) as exc:
                    self.set_state(
                        GraspStages.FAILED,
                        'EXECUTION_PLAN_FREEZE_FAILED: %s' % exc,
                    )
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
        if not self._simulate_grasp6d_plan_if_required(gcfg, gripper_cfg, plan):
            return False
        if not self._execution_checkpoint(plan, gcfg, 'gripper open'):
            return False

        self.set_state(GraspStages.PLAN_PREGRASP, 'using 6D grasp plan')
        if not self._command_gripper_position(
            set_gripper,
            open_position,
            'open gripper before 6D motion',
            self._cfg_float(gripper_cfg, 'open_wait_sec', 0.5),
            execution_plan=plan,
            gcfg=gcfg,
        ):
            return False
        if not self._execution_checkpoint(plan, gcfg, 'pregrasp'):
            return False
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
        ):
            return False

        if not self._execution_checkpoint(plan, gcfg, 'approach'):
            return False
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
    ):
        bound_plan = execution_plan
        if bound_plan is None and execution_plan_id is not None:
            with self._grasp6d_plan_guard():
                current = getattr(self, 'latest_grasp6d_plan', None)
                if current is not None and strict_plan_id_equal(
                    getattr(current, 'plan_id', None), execution_plan_id
                ):
                    bound_plan = deepcopy(current)
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
            validation = self.validate_plan_id_for_execution(
                execution_plan_id, gcfg or {}
            )
            if not validation.ok:
                self.set_state(
                    GraspStages.FAILED,
                    '%s: %s before planning %s'
                    % (validation.code, validation.reason, label),
                )
                return False
        strict_rich_plan = (
            bound_plan is not None or execution_plan_id is not None
        )
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
            self.set_state(GraspStages.FAILED, '%s failed: %s' % (label, resp.message))
            return False
        settled = self._wait_for_motion_settle(settle_reason)
        if settled is False:
            self.set_state(
                GraspStages.FAILED,
                '%s feedback did not settle; refusing to overlap the next motion' % label,
            )
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
                    gcfg, 'target_observation_validity_sec', 1.5
                ),
            )
        try:
            default = rospy.get_param(
                '/grasp_6d/target_observation_validity_sec', 1.5
            )
        except Exception:
            default = 1.5
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
                        'requested plan_id is not the current rich plan',
                    ),
                    None,
                )
            copied = deepcopy(current)
            validation = validate_execution_plan(
                copied,
                _stamp_seconds(rospy.Time.now()),
                self._configured_plan_validity(gcfg or {}),
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
            copied = deepcopy(current)
            result = validate_execution_plan(
                copied,
                _stamp_seconds(rospy.Time.now()),
                self._configured_plan_validity(gcfg or {}),
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

    def _bound_target_drift_result(self, plan, gcfg):
        def fail(code, reason):
            self._clear_grasp6d_authority(
                expected_plan_id=str(getattr(plan, 'plan_id', '') or '')
            )
            return PlanValidationResult(False, code, reason)

        latest_obj = getattr(self, 'latest_obj', None)
        if latest_obj is None:
            return fail('TARGET_LOST', 'live target observation is missing')
        if not bool(getattr(latest_obj, 'detected', False)):
            return fail('TARGET_LOST', 'live target is not detected')
        observation_time = getattr(self, 'latest_obj_time', None)
        if observation_time is None:
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
    def _cfg_float(cfg, key, default):
        try:
            if isinstance(cfg, dict) and key in cfg:
                return float(cfg.get(key))
        except Exception:
            pass
        return float(default)

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
    ):
        try:
            if execution_plan is not None:
                validation, resp = self._invoke_plan_bound_action(
                    execution_plan,
                    gcfg or {},
                    label,
                    lambda: set_gripper(float(position)),
                )
                if not validation.ok:
                    self.set_state(
                        GraspStages.FAILED,
                        '%s: %s before %s'
                        % (validation.code, validation.reason, label),
                    )
                    return False
            else:
                resp = set_gripper(float(position))
        except Exception as exc:
            self.set_state(GraspStages.FAILED, '%s failed: %s' % (label, exc))
            return False
        if not bool(getattr(resp, 'success', True)):
            self.set_state(GraspStages.FAILED, '%s failed: %s' % (label, getattr(resp, 'message', '')))
            return False
        delay = max(0.0, float(wait_sec))
        if delay > 0.0:
            rospy.sleep(delay)
        return True

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
