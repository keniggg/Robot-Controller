from dataclasses import dataclass, replace
import math

import numpy as np
from tf.transformations import quaternion_from_matrix, quaternion_matrix


ANALYTICAL_GRIPPER_MODEL_NAME = 'Alicia_D_v5_6_gripper_50mm'
ANALYTICAL_MAX_INNER_GAP_M = 0.050
ANALYTICAL_JAW_CLEARANCE_EACH_SIDE_M = 0.002
ANALYTICAL_FINGER_SIZE_XYZ_M = np.asarray(
    [0.0434, 0.0286, 0.0600],
    dtype=float,
)
ANALYTICAL_PALM_SIZE_XYZ_M = np.asarray(
    [0.1175, 0.1550, 0.0774],
    dtype=float,
)
ANALYTICAL_SUPPORT_CLEARANCE_M = 0.003
GRIPPER_CONTRACT_TOLERANCE_M = 0.0005
_CENTER_TOLERANCE_M = 0.003
_GATE_COUNT = 6
_INTERPOLATION_SAMPLES = 11


def _readonly_vector(value, name):
    output = np.asarray(value, dtype=float)
    if output.shape != (3,):
        raise ValueError('%s must have shape (3,)' % name)
    if not np.all(np.isfinite(output)):
        raise ValueError('%s must contain only finite values' % name)
    output = output.copy()
    output.setflags(write=False)
    return output


def _finite_number(value, name):
    number = float(value)
    if not np.isfinite(number):
        raise ValueError('%s must be finite' % name)
    return number


@dataclass(frozen=True)
class GripperGeometry:
    max_inner_gap_m: float
    jaw_clearance_each_side_m: float
    finger_size_xyz_m: np.ndarray
    palm_size_xyz_m: np.ndarray
    support_clearance_m: float

    def __post_init__(self):
        max_gap = _finite_number(self.max_inner_gap_m, 'max_inner_gap_m')
        jaw_clearance = _finite_number(
            self.jaw_clearance_each_side_m,
            'jaw_clearance_each_side_m',
        )
        support_clearance = _finite_number(
            self.support_clearance_m,
            'support_clearance_m',
        )
        finger = _readonly_vector(self.finger_size_xyz_m, 'finger_size_xyz_m')
        palm = _readonly_vector(self.palm_size_xyz_m, 'palm_size_xyz_m')
        if max_gap <= 0.0:
            raise ValueError('max_inner_gap_m must be positive')
        if jaw_clearance < 0.0:
            raise ValueError('jaw_clearance_each_side_m must be non-negative')
        if support_clearance < 0.0:
            raise ValueError('support_clearance_m must be non-negative')
        if np.any(finger <= 0.0):
            raise ValueError('finger_size_xyz_m values must be positive')
        if np.any(palm <= 0.0):
            raise ValueError('palm_size_xyz_m values must be positive')
        object.__setattr__(self, 'max_inner_gap_m', max_gap)
        object.__setattr__(self, 'jaw_clearance_each_side_m', jaw_clearance)
        object.__setattr__(self, 'support_clearance_m', support_clearance)
        object.__setattr__(self, 'finger_size_xyz_m', finger)
        object.__setattr__(self, 'palm_size_xyz_m', palm)


@dataclass(frozen=True)
class CandidateGateResult:
    ok: bool
    failure_code: str
    failure_reason: str
    required_open_width_m: float
    center_distance_m: float
    support_clearance_m: float
    jaw_alignment: float
    motion_cost: float
    geometry_cost: float
    failed_gate: str = ''
    passed_gate_count: int = 0

    def __post_init__(self):
        required = _finite_number(
            self.required_open_width_m,
            'required_open_width_m',
        )
        center_distance = _finite_number(
            self.center_distance_m,
            'center_distance_m',
        )
        support_clearance = _finite_number(
            self.support_clearance_m,
            'support_clearance_m',
        )
        jaw_alignment = _finite_number(self.jaw_alignment, 'jaw_alignment')
        motion_cost = _finite_number(self.motion_cost, 'motion_cost')
        geometry_cost = _finite_number(self.geometry_cost, 'geometry_cost')
        passed = int(self.passed_gate_count)
        if required < 0.0:
            raise ValueError('required_open_width_m must be non-negative')
        if center_distance < 0.0:
            raise ValueError('center_distance_m must be non-negative')
        if not 0.0 <= jaw_alignment <= 1.0 + 1e-9:
            raise ValueError('jaw_alignment must be between 0 and 1')
        if motion_cost < 0.0:
            raise ValueError('motion_cost must be non-negative')
        if geometry_cost < 0.0:
            raise ValueError('geometry_cost must be non-negative')
        if not 0 <= passed <= _GATE_COUNT:
            raise ValueError('passed_gate_count is outside the analytical gate range')
        if bool(self.ok):
            if str(self.failure_code or '') or str(self.failure_reason or ''):
                raise ValueError('successful candidate gate result cannot contain a failure')
            if str(self.failed_gate or ''):
                raise ValueError('successful candidate gate result cannot name a failed gate')
            if passed != _GATE_COUNT:
                raise ValueError('successful candidate must pass every analytical gate')
        else:
            if not str(self.failure_code or ''):
                raise ValueError('failed candidate gate result requires a failure_code')
            if not str(self.failure_reason or ''):
                raise ValueError('failed candidate gate result requires a failure_reason')
            if not str(self.failed_gate or ''):
                raise ValueError('failed candidate gate result requires failed_gate')
        object.__setattr__(self, 'ok', bool(self.ok))
        object.__setattr__(self, 'failure_code', str(self.failure_code or ''))
        object.__setattr__(self, 'failure_reason', str(self.failure_reason or ''))
        object.__setattr__(self, 'required_open_width_m', required)
        object.__setattr__(self, 'center_distance_m', center_distance)
        object.__setattr__(self, 'support_clearance_m', support_clearance)
        object.__setattr__(self, 'jaw_alignment', min(1.0, max(0.0, jaw_alignment)))
        object.__setattr__(self, 'motion_cost', motion_cost)
        object.__setattr__(self, 'geometry_cost', geometry_cost)
        object.__setattr__(self, 'failed_gate', str(self.failed_gate or ''))
        object.__setattr__(self, 'passed_gate_count', passed)


def parse_tool_axis(axis_name):
    name = str(axis_name or '').strip().lower()
    sign = -1.0 if name.startswith('-') else 1.0
    name = name.lstrip('+-')
    if name not in ('x', 'y', 'z'):
        raise ValueError('tool axis must be x, y, z, -x, -y, or -z')
    index = {'x': 0, 'y': 1, 'z': 2}[name]
    axis = np.zeros(3, dtype=float)
    axis[index] = sign
    return axis, index


def _validated_rotation(value, name):
    rotation = np.asarray(value, dtype=float)
    if rotation.shape != (3, 3):
        raise ValueError('%s must have shape (3, 3)' % name)
    if not np.all(np.isfinite(rotation)):
        raise ValueError('%s contains non-finite values' % name)
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6):
        raise ValueError('%s is not orthonormal' % name)
    determinant = float(np.linalg.det(rotation))
    if not np.isfinite(determinant) or abs(determinant - 1.0) > 1e-6:
        raise ValueError('%s is not right-handed' % name)
    return rotation


def _validated_transform(value, name):
    transform = np.asarray(value, dtype=float)
    if transform.shape != (4, 4):
        raise ValueError('%s must have shape (4, 4)' % name)
    if not np.all(np.isfinite(transform)):
        raise ValueError('%s contains non-finite values' % name)
    if not np.allclose(transform[3], [0.0, 0.0, 0.0, 1.0], atol=1e-7):
        raise ValueError('%s is not homogeneous' % name)
    _validated_rotation(transform[:3, :3], '%s rotation' % name)
    return transform


def required_open_width_m(
    obb_size_xyz_m,
    R_base_obb,
    jaw_axis_base,
    clearance_each_side_m,
):
    size = _readonly_vector(obb_size_xyz_m, 'obb_size_xyz_m')
    if np.any(size <= 0.0):
        raise ValueError('obb_size_xyz_m values must be positive')
    rotation = _validated_rotation(R_base_obb, 'R_base_obb')
    jaw_axis = _readonly_vector(jaw_axis_base, 'jaw_axis_base')
    jaw_norm = float(np.linalg.norm(jaw_axis))
    if abs(jaw_norm - 1.0) > 1e-6:
        raise ValueError('jaw_axis_base must be a unit vector')
    clearance = _finite_number(
        clearance_each_side_m,
        'clearance_each_side_m',
    )
    if clearance < 0.0:
        raise ValueError('clearance_each_side_m must be non-negative')
    half = 0.5 * size
    projected_width = 2.0 * float(
        np.dot(np.abs(rotation.T @ jaw_axis), half)
    )
    return projected_width + 2.0 * clearance


def gripper_box_centers(
    center_base,
    R_base_tool,
    required_open_width_m,
    gripper,
    tool_jaw_axis='y',
    tool_finger_length_axis='z',
):
    if not isinstance(gripper, GripperGeometry):
        raise ValueError('gripper must be a GripperGeometry')
    center = _readonly_vector(center_base, 'center_base')
    rotation = _validated_rotation(R_base_tool, 'R_base_tool')
    opening = _finite_number(required_open_width_m, 'required_open_width_m')
    if opening < 0.0 or opening > gripper.max_inner_gap_m + 1e-9:
        raise ValueError('required_open_width_m is outside the gripper range')
    jaw_local, jaw_index = parse_tool_axis(tool_jaw_axis)
    finger_local, finger_index = parse_tool_axis(tool_finger_length_axis)
    if jaw_index == finger_index:
        raise ValueError('jaw and finger length axes must be different')
    jaw_axis = rotation @ jaw_local
    finger_axis = rotation @ finger_local
    finger_length = float(gripper.finger_size_xyz_m[finger_index])
    finger_thickness = float(gripper.finger_size_xyz_m[jaw_index])
    palm_length = float(gripper.palm_size_xyz_m[finger_index])
    finger_back = center - 0.5 * finger_length * finger_axis
    finger_offset = 0.5 * (opening + finger_thickness) * jaw_axis
    palm_center = center - (
        finger_length + 0.5 * palm_length
    ) * finger_axis
    return {
        'grasp_center': center.copy(),
        'left_finger': finger_back + finger_offset,
        'right_finger': finger_back - finger_offset,
        'palm': palm_center,
    }


def _obb_line_interval(center, direction, obb_center, rotation, size):
    local_center = rotation.T @ (center - obb_center)
    local_direction = rotation.T @ direction
    half = 0.5 * size
    lower = -float('inf')
    upper = float('inf')
    for axis in range(3):
        component = float(local_direction[axis])
        coordinate = float(local_center[axis])
        if abs(component) <= 1e-10:
            if coordinate < -half[axis] or coordinate > half[axis]:
                return None
            continue
        first = (-half[axis] - coordinate) / component
        second = (half[axis] - coordinate) / component
        lower = max(lower, min(first, second))
        upper = min(upper, max(first, second))
        if lower > upper:
            return None
    if not np.isfinite(lower) or not np.isfinite(upper):
        return None
    return float(lower), float(upper)


def _box_corners(center, rotation, size):
    half = 0.5 * np.asarray(size, dtype=float)
    local = np.asarray(
        [
            [sx * half[0], sy * half[1], sz * half[2]]
            for sx in (-1.0, 1.0)
            for sy in (-1.0, 1.0)
            for sz in (-1.0, 1.0)
        ],
        dtype=float,
    )
    return local @ rotation.T + np.asarray(center, dtype=float)


def _obb_overlap(center_a, rotation_a, size_a, center_b, rotation_b, size_b):
    half_a = 0.5 * np.asarray(size_a, dtype=float)
    half_b = 0.5 * np.asarray(size_b, dtype=float)
    axes = [rotation_a[:, index] for index in range(3)]
    axes.extend(rotation_b[:, index] for index in range(3))
    for first in range(3):
        for second in range(3):
            cross = np.cross(rotation_a[:, first], rotation_b[:, second])
            norm = float(np.linalg.norm(cross))
            if norm > 1e-9:
                axes.append(cross / norm)
    delta = np.asarray(center_b, dtype=float) - np.asarray(center_a, dtype=float)
    for axis in axes:
        distance = abs(float(np.dot(delta, axis)))
        radius_a = float(np.dot(half_a, np.abs(rotation_a.T @ axis)))
        radius_b = float(np.dot(half_b, np.abs(rotation_b.T @ axis)))
        if distance > radius_a + radius_b + 1e-9:
            return False
    return True


def _matrix_quaternion(rotation):
    matrix = np.eye(4, dtype=float)
    matrix[:3, :3] = rotation
    quaternion = np.asarray(quaternion_from_matrix(matrix), dtype=float)
    norm = float(np.linalg.norm(quaternion))
    if norm <= 1e-12:
        raise ValueError('rotation produced a degenerate quaternion')
    quaternion /= norm
    if quaternion[3] < 0.0:
        quaternion = -quaternion
    return quaternion


def _slerp(first, second, fraction):
    first = np.asarray(first, dtype=float)
    second = np.asarray(second, dtype=float)
    dot = float(np.dot(first, second))
    if dot < 0.0:
        second = -second
        dot = -dot
    dot = min(1.0, max(-1.0, dot))
    if dot > 0.9995:
        output = first + float(fraction) * (second - first)
        output /= np.linalg.norm(output)
        return output
    angle = math.acos(dot)
    sine = math.sin(angle)
    output = (
        math.sin((1.0 - float(fraction)) * angle) / sine * first
        + math.sin(float(fraction) * angle) / sine * second
    )
    output /= np.linalg.norm(output)
    return output


def _interpolate_transforms(first, second, samples=_INTERPOLATION_SAMPLES):
    first = np.asarray(first, dtype=float)
    second = np.asarray(second, dtype=float)
    first_quaternion = _matrix_quaternion(first[:3, :3])
    second_quaternion = _matrix_quaternion(second[:3, :3])
    output = []
    for fraction in np.linspace(0.0, 1.0, int(samples)):
        transform = np.eye(4, dtype=float)
        transform[:3, 3] = (
            (1.0 - fraction) * first[:3, 3]
            + fraction * second[:3, 3]
        )
        transform[:3, :3] = quaternion_matrix(
            _slerp(first_quaternion, second_quaternion, fraction)
        )[:3, :3]
        output.append(transform)
    return output


def _stage_boxes(
    transform,
    gripper,
    opening_width_m,
    tool_jaw_axis,
    tool_finger_length_axis,
):
    centers = gripper_box_centers(
        transform[:3, 3],
        transform[:3, :3],
        opening_width_m,
        gripper,
        tool_jaw_axis,
        tool_finger_length_axis,
    )
    return (
        ('left_finger', centers['left_finger'], gripper.finger_size_xyz_m),
        ('right_finger', centers['right_finger'], gripper.finger_size_xyz_m),
        ('palm', centers['palm'], gripper.palm_size_xyz_m),
    )


def _intended_finger_contact(
    box_name,
    box_center,
    stage_center,
    jaw_axis,
    jaw_index,
    finger_size,
    opening_width_m,
    obb_center,
    obb_rotation,
    obb_size,
):
    side = 1.0 if box_name == 'left_finger' else -1.0
    if side * float(np.dot(box_center - stage_center, jaw_axis)) <= 0.0:
        return False
    object_center = float(np.dot(obb_center - stage_center, jaw_axis))
    object_radius = float(
        np.dot(
            0.5 * obb_size,
            np.abs(obb_rotation.T @ jaw_axis),
        )
    )
    inner_face = side * 0.5 * opening_width_m
    object_face = object_center + side * object_radius
    penetration = side * (object_face - inner_face)
    finger_thickness = float(finger_size[jaw_index])
    return -1e-6 <= penetration <= 0.5 * finger_thickness + 1e-6


def _check_boxes(
    transform,
    gripper,
    opening_width_m,
    tool_jaw_axis,
    tool_finger_length_axis,
    support_normal,
    support_offset,
    obb_center,
    obb_rotation,
    obb_size,
    allow_finger_contact,
):
    rotation = transform[:3, :3]
    stage_center = transform[:3, 3]
    jaw_local, jaw_index = parse_tool_axis(tool_jaw_axis)
    jaw_axis = rotation @ jaw_local
    minimum_clearance = float('inf')
    for name, box_center, box_size in _stage_boxes(
        transform,
        gripper,
        opening_width_m,
        tool_jaw_axis,
        tool_finger_length_axis,
    ):
        corners = _box_corners(box_center, rotation, box_size)
        clearance = corners @ support_normal + float(support_offset)
        minimum_clearance = min(minimum_clearance, float(np.min(clearance)))
        if float(np.min(clearance)) < gripper.support_clearance_m - 1e-9:
            return False, minimum_clearance, '%s enters support clearance' % name
        if not _obb_overlap(
            box_center,
            rotation,
            box_size,
            obb_center,
            obb_rotation,
            obb_size,
        ):
            continue
        if (
            allow_finger_contact
            and name in ('left_finger', 'right_finger')
            and _intended_finger_contact(
                name,
                box_center,
                stage_center,
                jaw_axis,
                jaw_index,
                gripper.finger_size_xyz_m,
                opening_width_m,
                obb_center,
                obb_rotation,
                obb_size,
            )
        ):
            continue
        return False, minimum_clearance, '%s intrudes into carton OBB' % name
    return True, minimum_clearance, ''


def _failed_result(
    gate,
    code,
    reason,
    passed,
    required_width,
    center_distance,
    support_clearance,
    jaw_alignment,
    motion_cost,
    geometry_cost,
):
    return CandidateGateResult(
        ok=False,
        failure_code=code,
        failure_reason=reason,
        required_open_width_m=max(0.0, float(required_width)),
        center_distance_m=max(0.0, float(center_distance)),
        support_clearance_m=float(support_clearance),
        jaw_alignment=min(1.0, max(0.0, float(jaw_alignment))),
        motion_cost=max(0.0, float(motion_cost)),
        geometry_cost=max(0.0, float(geometry_cost)),
        failed_gate=gate,
        passed_gate_count=int(passed),
    )


def evaluate_candidate(
    *,
    gripper,
    candidate_center_base,
    R_base_tool,
    candidate_width_m,
    obb_center_base,
    R_base_obb,
    obb_size_xyz_m,
    support_normal_base,
    support_offset_m,
    pregrasp_T_base_tool,
    approach_T_base_tool,
    grasp_T_base_tool,
    lift_T_base_tool,
    tool_jaw_axis='y',
    tool_finger_length_axis='z',
    motion_cost=0.0,
):
    """Fail-closed analytical prefilter for one base-frame 6D grasp."""
    required_width = 0.0
    center_distance = 0.0
    support_clearance = -1.0e6
    jaw_alignment = 0.0
    geometry_cost = 0.0
    safe_motion_cost = 0.0
    try:
        if not isinstance(gripper, GripperGeometry):
            raise ValueError('gripper must be a GripperGeometry')
        center = _readonly_vector(candidate_center_base, 'candidate_center_base')
        tool_rotation = _validated_rotation(R_base_tool, 'R_base_tool')
        obb_center = _readonly_vector(obb_center_base, 'obb_center_base')
        obb_rotation = _validated_rotation(R_base_obb, 'R_base_obb')
        obb_size = _readonly_vector(obb_size_xyz_m, 'obb_size_xyz_m')
        if np.any(obb_size <= 0.0):
            raise ValueError('obb_size_xyz_m values must be positive')
        support_normal = _readonly_vector(
            support_normal_base,
            'support_normal_base',
        )
        normal_norm = float(np.linalg.norm(support_normal))
        if normal_norm <= 1e-12:
            raise ValueError('support_normal_base must be non-zero')
        support_normal = np.asarray(support_normal / normal_norm, dtype=float)
        support_offset = _finite_number(support_offset_m, 'support_offset_m')
        _finite_number(candidate_width_m, 'candidate_width_m')
        if float(candidate_width_m) < 0.0:
            raise ValueError('candidate_width_m must be non-negative')
        safe_motion_cost = _finite_number(motion_cost, 'motion_cost')
        if safe_motion_cost < 0.0:
            raise ValueError('motion_cost must be non-negative')
        transforms = [
            _validated_transform(pregrasp_T_base_tool, 'pregrasp_T_base_tool'),
            _validated_transform(approach_T_base_tool, 'approach_T_base_tool'),
            _validated_transform(grasp_T_base_tool, 'grasp_T_base_tool'),
            _validated_transform(lift_T_base_tool, 'lift_T_base_tool'),
        ]
        if not np.allclose(transforms[2][:3, 3], center, atol=1e-7):
            raise ValueError('candidate center does not match grasp transform')
        if not np.allclose(transforms[2][:3, :3], tool_rotation, atol=1e-7):
            raise ValueError('candidate rotation does not match grasp transform')
        jaw_local, jaw_index = parse_tool_axis(tool_jaw_axis)
        _finger_local, finger_index = parse_tool_axis(tool_finger_length_axis)
        if jaw_index == finger_index:
            raise ValueError('jaw and finger length axes must be different')
        jaw_axis = tool_rotation @ jaw_local
        center_distance = float(np.linalg.norm(center - obb_center))
        support_clearance = float(np.dot(center, support_normal) + support_offset)
        jaw_alignment = float(
            max(
                abs(np.dot(jaw_axis, obb_rotation[:, 0])),
                abs(np.dot(jaw_axis, obb_rotation[:, 1])),
            )
        )
        geometry_cost = center_distance
    except Exception as exc:
        return _failed_result(
            'transform',
            'GRIPPER_SWEEP_COLLISION',
            'invalid analytical gripper input: %s' % exc,
            0,
            required_width,
            center_distance,
            support_clearance,
            jaw_alignment,
            safe_motion_cost,
            geometry_cost,
        )

    if support_clearance < gripper.support_clearance_m:
        return _failed_result(
            'center',
            'GRIPPER_SWEEP_COLLISION',
            'candidate center support clearance %.6fm is below %.6fm'
            % (support_clearance, gripper.support_clearance_m),
            1,
            required_width,
            center_distance,
            support_clearance,
            jaw_alignment,
            safe_motion_cost,
            geometry_cost,
        )
    local_center = obb_rotation.T @ (center - obb_center)
    if np.any(np.abs(local_center) > 0.5 * obb_size + _CENTER_TOLERANCE_M):
        return _failed_result(
            'center',
            'CENTER_OUTSIDE_OBB',
            'candidate center is outside the carton OBB tolerance',
            1,
            required_width,
            center_distance,
            support_clearance,
            jaw_alignment,
            safe_motion_cost,
            geometry_cost,
        )

    interval = _obb_line_interval(
        center,
        jaw_axis,
        obb_center,
        obb_rotation,
        obb_size,
    )
    if (
        interval is None
        or interval[0] >= -1e-8
        or interval[1] <= 1e-8
    ):
        return _failed_result(
            'jaw_width',
            'GRIPPER_SWEEP_COLLISION',
            'candidate jaw line does not cross both sides of the carton OBB',
            2,
            required_width,
            center_distance,
            support_clearance,
            jaw_alignment,
            safe_motion_cost,
            geometry_cost,
        )
    required_width = required_open_width_m(
        obb_size,
        obb_rotation,
        jaw_axis,
        gripper.jaw_clearance_each_side_m,
    )
    if required_width > gripper.max_inner_gap_m + 1e-9:
        return _failed_result(
            'jaw_width',
            'GRIPPER_TOO_NARROW',
            'required opening %.6fm exceeds physical inner gap %.6fm'
            % (required_width, gripper.max_inner_gap_m),
            2,
            required_width,
            center_distance,
            support_clearance,
            jaw_alignment,
            safe_motion_cost,
            geometry_cost,
        )

    negative_reach = -float(interval[0]) + gripper.jaw_clearance_each_side_m
    positive_reach = float(interval[1]) + gripper.jaw_clearance_each_side_m
    maximum_side_reach = 0.5 * gripper.max_inner_gap_m
    if (
        negative_reach > maximum_side_reach + 1e-9
        or positive_reach > maximum_side_reach + 1e-9
    ):
        return _failed_result(
            'finger_reach',
            'GRIPPER_SWEEP_COLLISION',
            'one-sided finger reach %.6f/%.6fm exceeds %.6fm'
            % (negative_reach, positive_reach, maximum_side_reach),
            3,
            required_width,
            center_distance,
            support_clearance,
            jaw_alignment,
            safe_motion_cost,
            geometry_cost,
        )

    minimum_box_clearance = float('inf')
    endpoint_contact = (False, False, True, True)
    for endpoint_index, (transform, allow_contact) in enumerate(
        zip(transforms, endpoint_contact)
    ):
        endpoint_obb_center = obb_center
        if endpoint_index == 3:
            endpoint_obb_center = (
                obb_center
                + transform[:3, 3]
                - transforms[2][:3, 3]
            )
        safe, clearance, reason = _check_boxes(
            transform,
            gripper,
            gripper.max_inner_gap_m,
            tool_jaw_axis,
            tool_finger_length_axis,
            support_normal,
            support_offset,
            endpoint_obb_center,
            obb_rotation,
            obb_size,
            allow_contact,
        )
        minimum_box_clearance = min(minimum_box_clearance, clearance)
        if not safe:
            return _failed_result(
                'static_envelope',
                'GRIPPER_SWEEP_COLLISION',
                reason,
                4,
                required_width,
                center_distance,
                minimum_box_clearance,
                jaw_alignment,
                safe_motion_cost,
                geometry_cost,
            )

    segment_contact = (False, True, True)
    for segment_index, (first, second) in enumerate(zip(transforms[:-1], transforms[1:])):
        first_center_clearance = float(
            np.dot(first[:3, 3], support_normal) + support_offset
        )
        second_center_clearance = float(
            np.dot(second[:3, 3], support_normal) + support_offset
        )
        if (
            segment_index < 2
            and first_center_clearance <= gripper.support_clearance_m
            and second_center_clearance > first_center_clearance
        ):
            return _failed_result(
                'swept_envelope',
                'GRIPPER_SWEEP_COLLISION',
                'approach enters the carton from the support-plane side',
                5,
                required_width,
                center_distance,
                minimum_box_clearance,
                jaw_alignment,
                safe_motion_cost,
                geometry_cost,
            )
        for transform in _interpolate_transforms(first, second):
            swept_obb_center = obb_center
            if segment_index == 2:
                swept_obb_center = (
                    obb_center
                    + transform[:3, 3]
                    - transforms[2][:3, 3]
                )
            safe, clearance, reason = _check_boxes(
                transform,
                gripper,
                gripper.max_inner_gap_m,
                tool_jaw_axis,
                tool_finger_length_axis,
                support_normal,
                support_offset,
                swept_obb_center,
                obb_rotation,
                obb_size,
                segment_contact[segment_index],
            )
            minimum_box_clearance = min(minimum_box_clearance, clearance)
            if not safe:
                return _failed_result(
                    'swept_envelope',
                    'GRIPPER_SWEEP_COLLISION',
                    'segment %d analytical sweep: %s'
                    % (segment_index, reason),
                    5,
                    required_width,
                    center_distance,
                    minimum_box_clearance,
                    jaw_alignment,
                    safe_motion_cost,
                    geometry_cost,
                )

    return CandidateGateResult(
        ok=True,
        failure_code='',
        failure_reason='',
        required_open_width_m=required_width,
        center_distance_m=center_distance,
        support_clearance_m=minimum_box_clearance,
        jaw_alignment=jaw_alignment,
        motion_cost=safe_motion_cost,
        geometry_cost=geometry_cost,
        failed_gate='',
        passed_gate_count=_GATE_COUNT,
    )


def candidate_with_motion_cost(result, motion_cost):
    if not isinstance(result, CandidateGateResult):
        raise ValueError('result must be a CandidateGateResult')
    cost = _finite_number(motion_cost, 'motion_cost')
    if cost < 0.0:
        raise ValueError('motion_cost must be non-negative')
    return replace(result, motion_cost=cost)


def candidate_rank_key(result, model_score):
    if not isinstance(result, CandidateGateResult) or not result.ok:
        raise ValueError('only successful analytical candidates can be ranked')
    score = _finite_number(model_score, 'model_score')
    return (
        float(result.geometry_cost),
        -float(result.support_clearance_m),
        -float(result.jaw_alignment),
        float(result.motion_cost),
        -score,
    )


def gripper_contract_mismatch_reason(
    gripper,
    remote_max_inner_gap_m,
    physical_open_width_m,
    twin_model_name,
    twin_max_inner_gap_m,
    tolerance_m=GRIPPER_CONTRACT_TOLERANCE_M,
    tool_jaw_axis='y',
    tool_finger_length_axis='z',
):
    """Validate configuration only; Task 11 remains the MJCF endpoint authority."""
    if not isinstance(gripper, GripperGeometry):
        return 'analytical gripper geometry is unavailable'
    tolerance = _finite_number(tolerance_m, 'tolerance_m')
    if tolerance < 0.0:
        return 'gripper contract tolerance is negative'
    checks = (
        ('analytical max inner gap', gripper.max_inner_gap_m),
        ('remote max inner gap', remote_max_inner_gap_m),
        ('physical open width', physical_open_width_m),
        ('digital-twin max inner gap', twin_max_inner_gap_m),
    )
    for label, value in checks:
        try:
            number = _finite_number(value, label)
        except Exception as exc:
            return str(exc)
        if abs(number - ANALYTICAL_MAX_INNER_GAP_M) > tolerance:
            return (
                '%s %.6fm differs from fixed analytical 50 mm contract'
                % (label, number)
            )
    scalar_geometry = (
        (
            'jaw clearance each side',
            gripper.jaw_clearance_each_side_m,
            ANALYTICAL_JAW_CLEARANCE_EACH_SIDE_M,
        ),
        (
            'support clearance',
            gripper.support_clearance_m,
            ANALYTICAL_SUPPORT_CLEARANCE_M,
        ),
    )
    for label, actual, expected in scalar_geometry:
        if abs(float(actual) - float(expected)) > tolerance:
            return (
                '%s %.6fm differs from fixed analytical %.6fm envelope'
                % (label, actual, expected)
            )
    vector_geometry = (
        (
            'finger box',
            gripper.finger_size_xyz_m,
            ANALYTICAL_FINGER_SIZE_XYZ_M,
        ),
        (
            'palm box',
            gripper.palm_size_xyz_m,
            ANALYTICAL_PALM_SIZE_XYZ_M,
        ),
    )
    for label, actual, expected in vector_geometry:
        if not np.allclose(actual, expected, rtol=0.0, atol=tolerance):
            return (
                '%s %s differs from fixed analytical %s envelope'
                % (
                    label,
                    np.asarray(actual, dtype=float).tolist(),
                    np.asarray(expected, dtype=float).tolist(),
                )
            )
    try:
        jaw_axis, jaw_index = parse_tool_axis(tool_jaw_axis)
        finger_axis, finger_index = parse_tool_axis(tool_finger_length_axis)
    except Exception as exc:
        return str(exc)
    if (
        jaw_index != 1
        or jaw_axis[1] <= 0.0
        or finger_index != 2
        or finger_axis[2] <= 0.0
    ):
        return (
            'tool axes %s/%s do not match fixed +Y jaw and +Z finger envelope'
            % (tool_jaw_axis, tool_finger_length_axis)
        )
    if str(twin_model_name or '') != ANALYTICAL_GRIPPER_MODEL_NAME:
        return (
            'digital-twin gripper model %r does not match %s'
            % (twin_model_name, ANALYTICAL_GRIPPER_MODEL_NAME)
        )
    return ''
