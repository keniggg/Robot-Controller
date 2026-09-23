from dataclasses import dataclass
import json
import math
from numbers import Real
import urllib.error
import urllib.parse
import urllib.request

from alicia_flexible_grasp.grasp.rich_plan_integrity import (
    validate_candidate_model_width,
    validate_candidate_source,
)


_GRIPPER_MODEL_NAME = 'Alicia_D_v5_6_gripper_50mm'
_MAX_INNER_GAP_M = 0.050
_FINGER_SIZE_XYZ_M = (0.0434, 0.0286, 0.0600)
_PALM_SIZE_XYZ_M = (0.1175, 0.1550, 0.0774)
_DEFAULT_CLOSE_TARGET_INNER_GAP_M = 0.0
_DEFAULT_CLOSE_SETTLE_SEC = 0.8
_DEFAULT_DENSITY_UPPER_BOUND_KG_M3 = 20000.0
_DEFAULT_OBJECT_MASS_FLOOR_KG = 0.02
_DEFAULT_OPERATIONAL_MASS_CEILING_KG = 0.50
_DEFAULT_FRICTION_LOWER_BOUND = (0.10, 0.005, 0.0001)
_SAFETY_KEYS = (
    'simulation_ok',
    'ik_success',
    'collision_free',
    'contact_success',
    'lift_success',
)
_COMPONENT_FAILURES = (
    ('ik_success', 'MUJOCO_IK_FAILED', 'MuJoCo inverse kinematics failed'),
    ('collision_free', 'MUJOCO_COLLISION', 'MuJoCo trajectory collision check failed'),
    ('contact_success', 'MUJOCO_CONTACT_FAILED', 'MuJoCo two-sided contact check failed'),
    ('lift_success', 'MUJOCO_LIFT_FAILED', 'MuJoCo lift check failed'),
)


@dataclass(frozen=True)
class MujocoGateValidationResult:
    ok: bool
    code: str = ''
    reason: str = ''
    score: float = 0.0

    @property
    def failure_code(self):
        return self.code

    @property
    def failure_reason(self):
        return self.reason


def _finite_float(value, field_name):
    if isinstance(value, bool):
        raise TypeError('%s must be a finite number' % field_name)
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise TypeError('%s must be a finite number' % field_name) from exc
    if not math.isfinite(result):
        raise ValueError('%s must be finite' % field_name)
    return result


def _finite_vector3(value, field_name, positive=False):
    try:
        values = [value.x, value.y, value.z]
    except AttributeError:
        try:
            values = list(value)
        except (TypeError, ValueError) as exc:
            raise TypeError('%s must contain three values' % field_name) from exc
    if len(values) != 3:
        raise ValueError('%s must contain exactly three values' % field_name)
    result = [
        _finite_float(item, '%s[%d]' % (field_name, index))
        for index, item in enumerate(values)
    ]
    if positive and any(item <= 0.0 for item in result):
        raise ValueError('%s values must be positive' % field_name)
    return result


def _pose_to_dict(value, field_name):
    pose = getattr(value, 'pose', value)
    try:
        position = pose.position
        orientation = pose.orientation
    except AttributeError as exc:
        raise TypeError('%s must be a pose' % field_name) from exc
    position_m = _finite_vector3(position, field_name + '.position')
    quaternion = [
        _finite_float(getattr(orientation, axis), field_name + '.orientation.' + axis)
        for axis in ('x', 'y', 'z', 'w')
    ]
    if math.sqrt(sum(item * item for item in quaternion)) <= 1e-12:
        raise ValueError('%s quaternion must be non-zero' % field_name)
    return {
        'position_m': position_m,
        'quaternion_xyzw': quaternion,
    }


def _config_mapping(value, field_name):
    if value is None:
        return {}
    if not isinstance(value, dict):
        raise TypeError('%s must be a mapping' % field_name)
    return value


def _configured_vector(config, keys, default, field_name):
    configured = None
    for key in keys:
        if key in config:
            configured = config[key]
            break
    result = list(default) if configured is None else _finite_vector3(
        configured,
        field_name,
        positive=True,
    )
    if any(abs(actual - expected) > 1e-9 for actual, expected in zip(result, default)):
        raise ValueError('%s does not match the fixed 50 mm gripper contract' % field_name)
    return list(default)


def _conservative_object_dynamics(object_size_xyz_m, object_config):
    """Derive a category-independent dynamics envelope from the live OBB."""

    if not isinstance(object_config, dict):
        raise TypeError('object_model must be a mapping')
    legacy = sorted(
        key for key in ('mass_kg', 'friction') if key in object_config
    )
    if legacy:
        raise ValueError(
            'object_model fixed %s is category-specific; configure the '
            'live-OBB dynamics envelope instead'
            % ','.join(legacy)
        )
    size = _finite_vector3(
        object_size_xyz_m,
        'object_model.size_xyz_m',
        positive=True,
    )
    density = _finite_float(
        object_config.get(
            'density_upper_bound_kg_m3',
            _DEFAULT_DENSITY_UPPER_BOUND_KG_M3,
        ),
        'object_model.density_upper_bound_kg_m3',
    )
    mass_floor = _finite_float(
        object_config.get(
            'mass_floor_kg',
            _DEFAULT_OBJECT_MASS_FLOOR_KG,
        ),
        'object_model.mass_floor_kg',
    )
    mass_ceiling = _finite_float(
        object_config.get(
            'operational_mass_ceiling_kg',
            _DEFAULT_OPERATIONAL_MASS_CEILING_KG,
        ),
        'object_model.operational_mass_ceiling_kg',
    )
    if (
        density <= 0.0
        or mass_floor <= 0.0
        or mass_ceiling < mass_floor
    ):
        raise ValueError(
            'object_model dynamics bounds must be positive and ordered'
        )
    volume = float(size[0] * size[1] * size[2])
    unclamped_mass = volume * density
    mass = min(mass_ceiling, max(mass_floor, unclamped_mass))
    friction = _finite_vector3(
        object_config.get(
            'friction_lower_bound',
            _DEFAULT_FRICTION_LOWER_BOUND,
        ),
        'object_model.friction_lower_bound',
        positive=True,
    )
    return mass, friction, {
        'source': 'live_obb_conservative_envelope',
        'obb_volume_m3': volume,
        'density_upper_bound_kg_m3': density,
        'unclamped_mass_kg': unclamped_mass,
        'mass_floor_kg': mass_floor,
        'operational_mass_ceiling_kg': mass_ceiling,
        'friction_lower_bound': list(friction),
    }


def _apply_target_mass_estimate(plan, object_config, evidence, mass, assumptions):
    """Use explicit plan-bound operator evidence, with an auditable 2x margin.

    This is an estimate-based simulation assumption, not a measured upper
    bound. Friction and every simulation acceptance requirement stay unchanged.
    """
    if evidence is None:
        return mass, assumptions
    if (not isinstance(evidence, dict) or plan.model_choice != 'unknown_tabletop'
            or evidence.get('source') != 'operator_estimate'
            or not evidence.get('operator_statement')
            or not evidence.get('evidence_sha256')
            or evidence.get('plan_id') != plan.plan_id
            or not getattr(plan, 'target_track_id', '')
            or evidence.get('target_track_id') != plan.target_track_id
            or str(evidence.get('snapshot_stamp_ns')) != str(plan.header.stamp.to_nsec())):
        raise ValueError('operator mass evidence is not bound to this exact target plan')
    estimate = _finite_float(evidence.get('estimated_mass_kg'), 'operator estimated_mass_kg')
    floor = assumptions['mass_floor_kg']
    ceiling = assumptions['operational_mass_ceiling_kg']
    if estimate <= 0.0 or 2.0*estimate > ceiling:
        raise ValueError('operator mass with uncertainty margin exceeds the operating range')
    simulation_mass = max(floor, 2.0*estimate)
    updated = dict(assumptions, source='plan_bound_operator_mass_estimate',
        default_density_envelope_mass_kg=mass, estimated_mass_kg=estimate,
        estimate_uncertainty_multiplier=2.0, simulation_mass_kg=simulation_mass,
        measured_upper_bound=False, operator_evidence=dict(evidence))
    return simulation_mass, updated


def build_mujoco_payload(plan, joint_names, joint_positions, gripper_config=None):
    """Build the finite, JSON-only schema-v3 request for one immutable plan."""
    if plan is None:
        raise TypeError('plan is required')
    if getattr(plan, 'valid', None) is not True:
        raise ValueError('plan.valid must be true')
    plan_id = getattr(plan, 'plan_id', None)
    if not isinstance(plan_id, str) or not plan_id:
        raise ValueError('plan_id must be a non-empty string')
    model_choice = getattr(plan, 'model_choice', None)
    if not isinstance(model_choice, str) or not model_choice:
        raise ValueError('model_choice must be a non-empty string')
    _finite_float(getattr(plan, 'score', None), 'plan.score')
    candidate_source, candidate_source_lineage = validate_candidate_source(
        getattr(plan, 'candidate_source', None),
        getattr(plan, 'candidate_source_lineage', None),
    )

    header = getattr(plan, 'header', None)
    stamp = getattr(header, 'stamp', None)
    try:
        snapshot_stamp_sec = _finite_float(
            stamp.to_sec(),
            'snapshot_stamp_sec',
        )
    except AttributeError as exc:
        raise TypeError('plan.header.stamp must provide to_sec()') from exc
    if snapshot_stamp_sec <= 0.0:
        raise ValueError('snapshot_stamp_sec must be positive')

    names = list(joint_names) if joint_names is not None else []
    positions = list(joint_positions) if joint_positions is not None else []
    if not names or len(names) != len(positions):
        raise ValueError('joint_names and joint_positions must be non-empty and have equal length')
    for index, name in enumerate(names):
        if not isinstance(name, str) or not name:
            raise ValueError('joint_names[%d] must be a non-empty string' % index)
    positions = [
        _finite_float(value, 'joint_positions[%d]' % index)
        for index, value in enumerate(positions)
    ]

    poses = list(getattr(plan, 'poses', ()) or ())
    if len(poses) != 4:
        raise ValueError('plan.poses must contain exactly four poses')
    trajectory = [
        _pose_to_dict(pose, 'trajectory[%d]' % index)
        for index, pose in enumerate(poses)
    ]

    candidate_width = validate_candidate_model_width(plan)
    required_width = _finite_float(
        getattr(plan, 'required_open_width_m', None),
        'required_open_width_m',
    )
    if required_width <= 0.0 or required_width > _MAX_INNER_GAP_M:
        raise ValueError('required_open_width_m must be in (0, 0.050]')

    root_config = _config_mapping(gripper_config, 'gripper_config')
    model_config = _config_mapping(
        root_config.get('gripper_model', root_config),
        'gripper_model',
    )
    configured_name = model_config.get(
        'name',
        model_config.get('model_name', _GRIPPER_MODEL_NAME),
    )
    if configured_name != _GRIPPER_MODEL_NAME:
        raise ValueError('gripper model does not match the fixed 50 mm contract')
    configured_gap = _finite_float(
        model_config.get('max_inner_gap_m', _MAX_INNER_GAP_M),
        'gripper.max_inner_gap_m',
    )
    if abs(configured_gap - _MAX_INNER_GAP_M) > 1e-9:
        raise ValueError('gripper max_inner_gap_m does not match 0.050 m')
    if 'open_width_m' in root_config:
        open_width = _finite_float(root_config['open_width_m'], 'open_width_m')
        if abs(open_width - _MAX_INNER_GAP_M) > 1e-9:
            raise ValueError('open_width_m does not match the fixed 0.050 m contract')
    finger_size = _configured_vector(
        model_config,
        ('finger_size_xyz_m', 'finger_box_xyz_m'),
        _FINGER_SIZE_XYZ_M,
        'gripper.finger_size_xyz_m',
    )
    palm_size = _configured_vector(
        model_config,
        ('palm_size_xyz_m', 'palm_box_xyz_m'),
        _PALM_SIZE_XYZ_M,
        'gripper.palm_size_xyz_m',
    )
    close_target_inner_gap = _finite_float(
        model_config.get(
            'close_target_inner_gap_m',
            _DEFAULT_CLOSE_TARGET_INNER_GAP_M,
        ),
        'gripper.close_target_inner_gap_m',
    )
    close_settle_sec = _finite_float(
        model_config.get(
            'close_settle_sec',
            _DEFAULT_CLOSE_SETTLE_SEC,
        ),
        'gripper.close_settle_sec',
    )
    if (
        close_target_inner_gap < 0.0
        or close_target_inner_gap > _MAX_INNER_GAP_M
    ):
        raise ValueError(
            'gripper.close_target_inner_gap_m must be within the 50 mm range'
        )
    if close_settle_sec <= 0.0:
        raise ValueError('gripper.close_settle_sec must be positive')

    geometry = getattr(plan, 'object_geometry', None)
    if geometry is None or getattr(geometry, 'valid', None) is not True:
        raise ValueError('plan.object_geometry.valid must be true')
    object_pose = _pose_to_dict(
        getattr(geometry, 'pose_base', None),
        'object_model.pose_base',
    )
    object_size = _finite_vector3(
        getattr(geometry, 'size_xyz_m', None),
        'object_model.size_xyz_m',
        positive=True,
    )
    support_normal = _finite_vector3(
        getattr(geometry, 'support_normal_base', None),
        'support_plane.normal_base',
    )
    if math.sqrt(sum(item * item for item in support_normal)) <= 1e-12:
        raise ValueError('support_plane.normal_base must be non-zero')
    support_offset = _finite_float(
        getattr(geometry, 'support_offset_m', None),
        'support_plane.offset_m',
    )

    object_config = _config_mapping(
        root_config.get('object_model', {}),
        'object_model',
    )
    object_type = object_config.get('type', 'obb_box')
    if object_type != 'obb_box':
        raise ValueError('object_model.type must be obb_box')
    mass, friction, dynamics_assumptions = _conservative_object_dynamics(
        object_size,
        object_config,
    )

    mass, dynamics_assumptions = _apply_target_mass_estimate(
        plan, object_config, root_config.get('target_mass_evidence'), mass, dynamics_assumptions)

    payload = {
        'schema_version': 3,
        'plan_id': plan_id,
        'snapshot_stamp_sec': snapshot_stamp_sec,
        'model_choice': model_choice,
        'candidate_source': candidate_source,
        'candidate_source_lineage': list(candidate_source_lineage),
        'joint_names': names,
        'joint_positions': positions,
        'trajectory': trajectory,
        'candidate_width_m': candidate_width,
        'required_open_width_m': required_width,
        'gripper': {
            'model_name': _GRIPPER_MODEL_NAME,
            'max_inner_gap_m': _MAX_INNER_GAP_M,
            'finger_size_xyz_m': finger_size,
            'palm_size_xyz_m': palm_size,
            'close_target_inner_gap_m': close_target_inner_gap,
            'close_settle_sec': close_settle_sec,
        },
        'object_model': {
            'type': 'obb_box',
            'pose_base': object_pose,
            'size_xyz_m': object_size,
            'mass_kg': mass,
            'friction': friction,
            'dynamics_assumptions': dynamics_assumptions,
        },
        'support_plane': {
            'normal_base': support_normal,
            'offset_m': support_offset,
        },
    }
    json.dumps(payload, allow_nan=False)
    return payload


def _validate_lift_evidence(response, score):
    evidence = response.get('lift_evidence')
    if not isinstance(evidence, dict):
        return MujocoGateValidationResult(
            False,
            'WSL_UNAVAILABLE',
            'MuJoCo response lacks structured strict dynamic-lift evidence',
            score,
        )
    if evidence.get('contract_version') != 1:
        return MujocoGateValidationResult(
            False,
            'WSL_UNAVAILABLE',
            'MuJoCo lift evidence contract_version must be 1',
            score,
        )
    numeric_names = (
        'object_lift_m',
        'commanded_lift_m',
        'minimum_lift_m',
    )
    numeric = {}
    try:
        for name in numeric_names:
            numeric[name] = _finite_float(
                evidence.get(name),
                'lift_evidence.%s' % name,
            )
    except (TypeError, ValueError):
        return MujocoGateValidationResult(
            False,
            'WSL_UNAVAILABLE',
            'MuJoCo lift evidence distances must be finite numbers',
            score,
        )
    count_names = (
        'two_sided_lift_samples',
        'lost_contact_samples',
        'lift_sample_count',
        'max_lost_contact_streak',
        'contact_loss_grace_samples',
    )
    counts = {}
    for name in count_names:
        value = evidence.get(name)
        if isinstance(value, bool) or not isinstance(value, int) or value < 0:
            return MujocoGateValidationResult(
                False,
                'WSL_UNAVAILABLE',
                'MuJoCo lift evidence %s must be a non-negative integer'
                % name,
                score,
            )
        counts[name] = value
    if (
        counts['lift_sample_count'] <= 0
        or (
            counts['two_sided_lift_samples']
            + counts['lost_contact_samples']
            != counts['lift_sample_count']
        )
    ):
        return MujocoGateValidationResult(
            False,
            'WSL_UNAVAILABLE',
            'MuJoCo lift evidence sample accounting is inconsistent',
            score,
        )
    if (
        counts['lost_contact_samples']
        > counts['contact_loss_grace_samples']
        or counts['max_lost_contact_streak']
        > counts['contact_loss_grace_samples']
    ):
        return MujocoGateValidationResult(
            False,
            'MUJOCO_CONTACT_FAILED',
            'MuJoCo lift lost two-sided contact beyond the strict grace bound',
            score,
        )
    if (
        numeric['minimum_lift_m'] <= 0.0
        or numeric['commanded_lift_m'] + 1e-9
        < numeric['minimum_lift_m']
        or numeric['object_lift_m'] + 1e-9
        < numeric['minimum_lift_m']
    ):
        return MujocoGateValidationResult(
            False,
            'MUJOCO_LIFT_FAILED',
            'MuJoCo dynamic object lift is below the strict minimum',
            score,
        )
    return None


def _failure_details(response, default_code, default_reason):
    code = response.get('failure_code')
    reason = response.get('failure_reason')
    if not isinstance(code, str) or not code:
        code = default_code
    if not isinstance(reason, str) or not reason:
        reason = default_reason
    return code, reason


def validate_mujoco_gate_response(
    response,
    expected_plan_id,
    min_score,
    expected_candidate_source=None,
    expected_candidate_source_lineage=None,
):
    """Accept only an exactly correlated, fully explicit MuJoCo safety pass."""
    if not isinstance(response, dict):
        return MujocoGateValidationResult(
            False,
            'WSL_UNAVAILABLE',
            'MuJoCo response must be a JSON object',
        )
    try:
        # Python's JSON decoder accepts NaN/Infinity by default.  Re-check the
        # complete response here as a second trust boundary so an in-process
        # client, test double, or future transport cannot authorize motion
        # with a non-standard JSON value hidden in an otherwise valid object.
        json.dumps(response, allow_nan=False)
    except (TypeError, ValueError, OverflowError):
        return MujocoGateValidationResult(
            False,
            'WSL_UNAVAILABLE',
            'MuJoCo response must be fully strict-JSON serializable',
        )
    echoed_id = response.get('plan_id')
    if (
        not isinstance(expected_plan_id, str)
        or not expected_plan_id
        or not isinstance(echoed_id, str)
        or echoed_id != expected_plan_id
    ):
        return MujocoGateValidationResult(
            False,
            'PLAN_ID_MISMATCH',
            'MuJoCo response plan_id does not exactly match the bound plan',
        )

    if (
        response.get('simulation_ok') is False
        and isinstance(response.get('failure_code'), str)
        and response.get('failure_code')
    ):
        raw_score = response.get('score', 0.0)
        score = None
        if not isinstance(raw_score, bool) and isinstance(raw_score, Real):
            score = float(raw_score)
            if not math.isfinite(score):
                score = None
        code, reason = _failure_details(
            response,
            'WSL_UNAVAILABLE',
            'MuJoCo simulation preflight failed',
        )
        return MujocoGateValidationResult(False, code, reason, score)

    try:
        response_source, response_lineage = validate_candidate_source(
            response.get('candidate_source'),
            response.get('candidate_source_lineage'),
        )
        if (
            expected_candidate_source is not None
            or expected_candidate_source_lineage is not None
        ):
            expected_source, expected_lineage = validate_candidate_source(
                expected_candidate_source,
                expected_candidate_source_lineage,
            )
            if (
                response_source != expected_source
                or response_lineage != expected_lineage
            ):
                raise ValueError('MuJoCo response candidate provenance differs')
    except (TypeError, ValueError, AttributeError):
        return MujocoGateValidationResult(
            False,
            'CANDIDATE_SOURCE_MISMATCH',
            'MuJoCo response candidate provenance is invalid or mismatched',
        )

    raw_score = response.get('score')
    if isinstance(raw_score, bool) or not isinstance(raw_score, Real):
        return MujocoGateValidationResult(
            False,
            'WSL_UNAVAILABLE',
            'MuJoCo response score must be a finite number',
        )
    score = float(raw_score)
    try:
        threshold = _finite_float(min_score, 'min_score')
    except (TypeError, ValueError):
        return MujocoGateValidationResult(
            False,
            'WSL_UNAVAILABLE',
            'configured MuJoCo min_score is invalid',
        )
    if not math.isfinite(score):
        return MujocoGateValidationResult(
            False,
            'WSL_UNAVAILABLE',
            'MuJoCo response score must be finite',
        )

    for key in _SAFETY_KEYS:
        if key not in response or type(response[key]) is not bool:
            return MujocoGateValidationResult(
                False,
                'WSL_UNAVAILABLE',
                'MuJoCo response %s must be an explicit Python bool' % key,
                score,
            )
    if (
        response['simulation_ok'] is not True
        and isinstance(response.get('failure_code'), str)
        and response.get('failure_code')
    ):
        code, reason = _failure_details(
            response,
            'WSL_UNAVAILABLE',
            'MuJoCo simulation preflight failed',
        )
        return MujocoGateValidationResult(False, code, reason, score)
    for key, code, default_reason in _COMPONENT_FAILURES:
        if response[key] is not True:
            _ignored_code, reason = _failure_details(
                response,
                code,
                default_reason,
            )
            return MujocoGateValidationResult(False, code, reason, score)
    if score < threshold:
        code, reason = _failure_details(
            response,
            'WSL_UNAVAILABLE',
            'MuJoCo score %.3f is below required %.3f' % (score, threshold),
        )
        return MujocoGateValidationResult(False, code, reason, score)
    if response['simulation_ok'] is not True:
        code, reason = _failure_details(
            response,
            'WSL_UNAVAILABLE',
            'MuJoCo simulation_ok is false',
        )
        return MujocoGateValidationResult(False, code, reason, score)
    lift_evidence_failure = _validate_lift_evidence(response, score)
    if lift_evidence_failure is not None:
        return lift_evidence_failure
    return MujocoGateValidationResult(True, score=score)


def validate_mujoco_digital_twin_url(server_url):
    normalized = str(server_url or '').strip().rstrip('/')
    if not normalized:
        raise ValueError('mujoco digital twin server_url is empty')
    if '<' in normalized or '>' in normalized:
        raise ValueError(
            'mujoco digital twin server_url contains placeholder angle brackets; '
            'replace it with a real URL such as http://192.168.26.1:8000'
        )
    parsed = urllib.parse.urlparse(normalized)
    if parsed.scheme not in ('http', 'https') or not parsed.netloc:
        raise ValueError('mujoco digital twin server_url must start with http:// or https:// and include host:port')
    return normalized


def _reject_nonstandard_json_constant(value):
    raise ValueError(
        'MuJoCo response contains non-standard JSON constant %s' % value
    )


class MujocoDigitalTwinClient:
    def __init__(self, server_url, timeout_sec=5.0):
        self.server_url = validate_mujoco_digital_twin_url(server_url)
        self.timeout_sec = float(timeout_sec)

    def health(self):
        return self._request_json('/health', None)

    def sync_joint_state(self, joint_state):
        payload = joint_state_payload(joint_state)
        return self._request_json('/sync_joint_state', payload)

    def simulate_grasp(self, payload):
        return self._request_json('/simulate_grasp', payload)

    def _request_json(self, path, payload):
        url = self.server_url + path
        data = None
        method = 'GET'
        headers = {'Accept': 'application/json'}
        if payload is not None:
            data = json.dumps(payload, allow_nan=False).encode('utf-8')
            method = 'POST'
            headers['Content-Type'] = 'application/json'
        request = urllib.request.Request(url, data=data, headers=headers, method=method)
        try:
            with urllib.request.urlopen(request, timeout=self.timeout_sec) as response:
                return json.loads(
                    response.read().decode('utf-8'),
                    parse_constant=_reject_nonstandard_json_constant,
                )
        except urllib.error.HTTPError as exc:
            body = exc.read().decode('utf-8', errors='replace')
            raise RuntimeError('mujoco digital twin HTTP %s: %s' % (exc.code, body)) from exc
        except urllib.error.URLError as exc:
            raise RuntimeError('mujoco digital twin connection failed: %s' % exc) from exc


def build_simulation_payload(
    joint_state=None,
    object_pose=None,
    grasp_plan=None,
    gripper_width_m=0.05,
    object_model=None,
):
    payload = {
        'gripper_width_m': float(gripper_width_m),
        'object_model': dict(object_model or {}),
    }
    if joint_state is not None:
        payload.update(joint_state_payload(joint_state))
    if object_pose is not None:
        payload['object_pose_base'] = object_pose_payload(object_pose, object_model=object_model)
    if grasp_plan is not None:
        payload['grasp_sequence_base'] = grasp_plan_payload(grasp_plan)
    return payload


def joint_state_payload(joint_state):
    return {
        'joint_names': [str(name) for name in getattr(joint_state, 'name', [])],
        'joint_positions': [float(value) for value in getattr(joint_state, 'position', [])],
    }


def object_pose_payload(object_pose, object_model=None):
    pose_stamped = getattr(object_pose, 'pose_base', object_pose)
    payload = pose_payload(pose_stamped)
    model = dict(object_model or {})
    size = model.get('size_xyz_m') or getattr(object_pose, 'size_xyz_m', None)
    payload.update(
        {
            'label': str(getattr(object_pose, 'label', model.get('label', 'object')) or 'object'),
            'confidence': float(getattr(object_pose, 'confidence', 1.0) or 0.0),
            'size_xyz_m': [float(v) for v in (size or [0.10, 0.06, 0.035])],
        }
    )
    return payload


def grasp_plan_payload(grasp_plan):
    names = ['pregrasp', 'approach', 'grasp', 'lift']
    poses = list(getattr(grasp_plan, 'poses', []) or [])
    frame_id = getattr(getattr(grasp_plan, 'header', None), 'frame_id', 'base_link') or 'base_link'
    result = []
    for index, pose in enumerate(poses[:4]):
        stamped = _pose_stamped_from_pose(pose, frame_id)
        item = pose_payload(stamped)
        item['name'] = names[index] if index < len(names) else 'pose_%d' % index
        result.append(item)
    return result


def pose_payload(pose_stamped):
    pose = getattr(pose_stamped, 'pose', pose_stamped)
    position = getattr(pose, 'position')
    orientation = getattr(pose, 'orientation')
    return {
        'frame_id': str(getattr(getattr(pose_stamped, 'header', None), 'frame_id', 'base_link') or 'base_link'),
        'position': [float(position.x), float(position.y), float(position.z)],
        'quaternion_xyzw': [
            float(orientation.x),
            float(orientation.y),
            float(orientation.z),
            float(orientation.w),
        ],
    }


def _pose_stamped_from_pose(pose, frame_id):
    class Header:
        pass

    class PoseStampedLike:
        pass

    stamped = PoseStampedLike()
    stamped.header = Header()
    stamped.header.frame_id = frame_id
    stamped.pose = pose
    return stamped
