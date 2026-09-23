from dataclasses import dataclass, replace
import math
from numbers import Integral

import numpy as np
from tf.transformations import quaternion_from_matrix, quaternion_matrix

from alicia_flexible_grasp.vision.multiview_surface import FusedTargetSurface


ANALYTICAL_GRIPPER_MODEL_NAME = 'Alicia_D_v5_6_gripper_50mm'
ANALYTICAL_MAX_INNER_GAP_M = 0.050
ANALYTICAL_JAW_CLEARANCE_EACH_SIDE_M = 0.002
ANALYTICAL_FINGER_SIZE_XYZ_M = np.asarray(
    [0.0434, 0.0286, 0.0600],
    dtype=float,
)
# The common center of the Link7/Link8 CAD AABBs in the fixed Alicia tool0
# frame.  Individual finger centers add/subtract the commanded opening along
# tool +Y.  The offset is intentionally independent of the GraspNet contact
# center and was measured from the checked-in 50 mm gripper URDF/STL pair.
ANALYTICAL_FINGER_PAIR_CENTER_TOOL_XYZ_M = np.asarray(
    [0.0004, 0.0003, -0.0302],
    dtype=float,
)
# The usable opposing contact patch of Link7/Link8 expressed as tool0 X/Z
# coordinates relative to ANALYTICAL_FINGER_PAIR_CENTER_TOOL_XYZ_M.  This is
# the boundary of the connected, planar inner-face triangles in the checked-in
# Alicia-D v5.6 50 mm collision meshes, not the larger collision AABB:
#   Link7.STL sha256 d546b7ab908c74281a41261412089a2f83f7519c2444739afe8ffff46450d19d
#   Link8.STL sha256 5e5b548734269b731c1d60502c742e3d70d5a97a1e8316db523e2232b4c5043b
# The two meshes map to the same counter-clockwise tool0 X/Z polygon.  Keeping
# this hardware contract distinct from ANALYTICAL_FINGER_BOX_PADDING_XYZ_M is
# essential: collision padding may conservatively reject a near miss, but it
# cannot create a physical gripping surface.
ANALYTICAL_FINGER_CONTACT_PATCH_TOOL_XZ_M = np.asarray(
    [
        [-0.0211033, -0.0275186],
        [-0.0209239, -0.0280557],
        [-0.0206282, -0.0285385],
        [-0.0202314, -0.0289425],
        [-0.0197540, -0.0292468],
        [-0.0192202, -0.0294359],
        [-0.0186577, -0.0295000],
        [-0.0050000, -0.0295000],
        [-0.0050000, -0.0250000],
        [0.0050000, -0.0250000],
        [0.0050000, -0.0295000],
        [0.0186577, -0.0295000],
        [0.0192202, -0.0294359],
        [0.0197540, -0.0292468],
        [0.0202314, -0.0289425],
        [0.0206282, -0.0285385],
        [0.0209239, -0.0280557],
        [0.0211033, -0.0275186],
        [0.0211573, -0.0269550],
        [0.0210830, -0.0263937],
        [0.0075417, 0.0277715],
        [0.0053279, 0.0295000],
        [-0.0053279, 0.0295000],
        [-0.0075417, 0.0277715],
        [-0.0210830, -0.0263937],
        [-0.0211573, -0.0269550],
    ],
    dtype=float,
)
# Total (not per-face) expansion applied to the configured CAD envelope.  It
# leaves at least GRIPPER_CONTRACT_TOLERANCE_M around every Link7/Link8 STL
# face at both the fully open and fully closed joint limits.
ANALYTICAL_FINGER_BOX_PADDING_XYZ_M = np.asarray(
    [0.0012, 0.0012, 0.0012],
    dtype=float,
)
ANALYTICAL_PALM_SIZE_XYZ_M = np.asarray(
    [0.1175, 0.1550, 0.0774],
    dtype=float,
)
# Link6.STL AABB center expressed in the fixed Alicia tool0 frame.  The
# conservative analytical size above surrounds the CAD mesh with at least the
# gripper contract tolerance on every face.
ANALYTICAL_PALM_CENTER_TOOL_XYZ_M = np.asarray(
    [-0.0393, 0.0003, -0.09344],
    dtype=float,
)
ANALYTICAL_SUPPORT_CLEARANCE_M = 0.003
ANALYTICAL_GRASPNET_DEPTH_BINS_M = (0.01, 0.02, 0.03, 0.04)
ANALYTICAL_GRASPNET_DEPTH_TOLERANCE_M = 1.0e-6
GRIPPER_CONTRACT_TOLERANCE_M = 0.0005
# The physical opening is a measured quantity (49.9375 mm on the current
# gripper), not a collision-envelope knob.  Keep its symmetric tolerance
# independent from the directional envelope comparison below.
PHYSICAL_OPEN_GAP_TOLERANCE_M = 0.0005
# Conservative collision envelopes are directional contracts.  A configured
# box or safety clearance may grow by the contract tolerance, but it must not
# shrink by more than this epsilon.  This deliberately does not replace the
# symmetric 0.5 mm tolerance used for the independently measured open gap.
_CONSERVATIVE_ENVELOPE_FLOAT_EPSILON_M = 1.0e-9
_CENTER_TOLERANCE_M = 0.003
_GATE_COUNT = 6
_INTERPOLATION_SAMPLES = 11
_MEASURED_SURFACE_TRIM_FRACTION = 0.05


class AnalyticalGripperContractError(ValueError):
    def __init__(self, code, message):
        self.code = str(code or 'TOOL0_CONTRACT_INVALID')
        super().__init__(str(message))


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
class BilateralSurfaceEvidence:
    ok: bool
    code: str
    negative_jaw_points: int
    positive_jaw_points: int
    negative_view_count: int
    positive_view_count: int
    measured_width_m: float
    contact_height_m: float


def _linear_quantiles(values, fractions):
    """Linear order statistics using operations available in NumPy 1.17.

    Weighted endpoints avoid overflowing the difference of finite extremes.
    No version-specific quantile keyword or warning suppression is needed.
    """

    ordered = np.sort(np.asarray(values, dtype=float).reshape(-1))
    ranks = np.asarray(fractions, dtype=float) * (len(ordered) - 1)
    lower = np.floor(ranks).astype(np.int64)
    upper = np.ceil(ranks).astype(np.int64)
    weight = ranks - lower
    return (1.0 - weight) * ordered[lower] + weight * ordered[upper]


def _bilateral_surface_measurement(
    fused_surface,
    contact_center_base,
    jaw_axis_base,
    insertion_axis_base,
    finger_geometry,
    minimum_points_per_side,
    support_normal_base=None,
):
    if not isinstance(fused_surface, FusedTargetSurface):
        raise ValueError('fused_surface must be a FusedTargetSurface')
    center = _readonly_vector(contact_center_base, 'contact_center_base')
    jaw = _readonly_vector(jaw_axis_base, 'jaw_axis_base')
    insertion = _readonly_vector(
        insertion_axis_base, 'insertion_axis_base'
    )
    if not math.isclose(
        float(np.linalg.norm(jaw)), 1.0, rel_tol=0.0, abs_tol=1e-6
    ):
        raise ValueError('jaw_axis_base must be a unit vector')
    if not math.isclose(
        float(np.linalg.norm(insertion)), 1.0, rel_tol=0.0, abs_tol=1e-6
    ):
        raise ValueError('insertion_axis_base must be a unit vector')
    if abs(float(np.dot(jaw, insertion))) > 1e-6:
        raise ValueError('jaw and insertion axes must be orthogonal')
    if not isinstance(finger_geometry, GripperGeometry):
        raise ValueError('finger_geometry must be a GripperGeometry')
    if (
        isinstance(minimum_points_per_side, (bool, np.bool_))
        or not isinstance(minimum_points_per_side, Integral)
        or int(minimum_points_per_side) <= 0
    ):
        raise ValueError('minimum_points_per_side must be a positive integer')
    minimum = int(minimum_points_per_side)
    # Samples/provenance alone cannot determine the measured support plane.
    # A top face has apparent insertion-height extent when the tool is tilted.
    try:
        normal = _readonly_vector(support_normal_base, 'support_normal_base')
        if not math.isclose(float(np.linalg.norm(normal)), 1.0,
                            rel_tol=0.0, abs_tol=1e-6):
            raise ValueError('support normal must be unit length')
    except (TypeError, ValueError):
        return None, None, None, None

    # The fixed Alicia semantic mapping is tool Y = jaw, tool Z = insertion,
    # and cross(Y, Z) = tool X.  The configured finger X/Z box dimensions are
    # therefore the conservative physical contact footprint available through
    # this interface; the exact mesh polygon additionally needs tool0 offset,
    # which this measured-surface contract intentionally does not accept.
    cross_axis = np.cross(jaw, insertion)
    cross_axis /= float(np.linalg.norm(cross_axis))
    delta = np.asarray(fused_surface.points_base, dtype=float) - center
    jaw_projection = delta.dot(jaw)
    insertion_projection = delta.dot(insertion)
    cross_projection = delta.dot(cross_axis)
    support_projection = delta.dot(normal)
    half_cross = 0.5 * float(finger_geometry.finger_size_xyz_m[0])
    half_insertion = 0.5 * float(finger_geometry.finger_size_xyz_m[2])
    # Keep the measured jaw bands inside the actual opening travel.  Without
    # this bound, a sufficiently dense cluster of arbitrary far-field points
    # could move the robust quantile extremes outward and be misclassified as
    # near-side support.  The contact center is the physical jaw midpoint.
    physical_gap = min(
        float(finger_geometry.max_inner_gap_m),
        ANALYTICAL_MAX_INNER_GAP_M,
    )
    half_physical_gap = 0.5 * physical_gap
    footprint = (
        (np.abs(cross_projection) <= half_cross)
        & (np.abs(insertion_projection) <= half_insertion)
        & (np.abs(jaw_projection) <= half_physical_gap + 1.0e-9)
    )
    footprint_indices = np.flatnonzero(footprint)
    if len(footprint_indices) < 2:
        return None, None, None, footprint

    footprint_jaw = jaw_projection[footprint]
    lower, upper = _linear_quantiles(
        footprint_jaw,
        (_MEASURED_SURFACE_TRIM_FRACTION,
         1.0 - _MEASURED_SURFACE_TRIM_FRACTION),
    )
    lower = float(lower)
    upper = float(upper)
    width = upper - lower
    if not math.isfinite(width) or width <= 1e-9:
        return None, None, None, footprint

    # A measured point can support one side only when it lies within the
    # existing fixed jaw-clearance contract of that robust measured extreme.
    # This threshold is hardware-derived and does not depend on target class.
    side_depth = float(finger_geometry.jaw_clearance_each_side_m)
    midpoint = 0.5 * lower + 0.5 * upper
    negative_mask = (
        footprint & (np.abs(jaw_projection - lower) <= side_depth)
        & (jaw_projection < midpoint)
    )
    positive_mask = (
        footprint & (np.abs(jaw_projection - upper) <= side_depth)
        & (jaw_projection > midpoint)
    )
    negative_count = int(np.count_nonzero(negative_mask))
    positive_count = int(np.count_nonzero(positive_mask))
    negative_views = int(np.unique(
        fused_surface.view_indices[negative_mask]
    ).size)
    positive_views = int(np.unique(
        fused_surface.view_indices[positive_mask]
    ).size)
    counts = (
        negative_count,
        positive_count,
        negative_views,
        positive_views,
        width,
    )
    if negative_count < minimum or positive_count < minimum:
        return counts, None, None, (negative_mask, positive_mask)

    # Require actual common support-normal extent before using insertion
    # coordinates for physical CAD overlap. Top-plane samples cannot acquire
    # a contact height simply by rotating the tool frame.
    quantiles = (_MEASURED_SURFACE_TRIM_FRACTION,
                 1.0 - _MEASURED_SURFACE_TRIM_FRACTION)
    negative_support = _linear_quantiles(support_projection[negative_mask], quantiles)
    positive_support = _linear_quantiles(support_projection[positive_mask], quantiles)
    support_lower = max(float(negative_support[0]), float(positive_support[0]))
    support_upper = min(float(negative_support[1]), float(positive_support[1]))
    if support_upper - support_lower <= 1e-9:
        return counts, None, None, (negative_mask, positive_mask)
    common_support = ((support_projection >= support_lower)
                      & (support_projection <= support_upper))
    negative_mask &= common_support
    positive_mask &= common_support
    counts = (
        int(np.count_nonzero(negative_mask)),
        int(np.count_nonzero(positive_mask)),
        int(np.unique(fused_surface.view_indices[negative_mask]).size),
        int(np.unique(fused_surface.view_indices[positive_mask]).size),
        width,
    )
    if counts[0] < minimum or counts[1] < minimum:
        return counts, None, None, (negative_mask, positive_mask)

    negative_heights = insertion_projection[negative_mask]
    positive_heights = insertion_projection[positive_mask]
    negative_bounds = _linear_quantiles(
        negative_heights,
        (_MEASURED_SURFACE_TRIM_FRACTION,
         1.0 - _MEASURED_SURFACE_TRIM_FRACTION),
    )
    positive_bounds = _linear_quantiles(
        positive_heights,
        (_MEASURED_SURFACE_TRIM_FRACTION,
         1.0 - _MEASURED_SURFACE_TRIM_FRACTION),
    )
    common_lower = max(float(negative_bounds[0]), float(positive_bounds[0]))
    common_upper = min(float(negative_bounds[1]), float(positive_bounds[1]))
    if (
        not math.isfinite(common_lower)
        or not math.isfinite(common_upper)
        or common_upper <= common_lower
    ):
        return counts, None, None, (negative_mask, positive_mask)
    return (
        counts,
        common_lower,
        common_upper,
        (negative_mask, positive_mask),
    )


def evaluate_bilateral_surface_evidence_and_bounds(
    fused_surface,
    contact_center_base,
    jaw_axis_base,
    insertion_axis_base,
    finger_geometry,
    minimum_points_per_side=12,
    *,
    support_normal_base=None,
):
    """Return evidence and tool-Z bounds from one measured surface evaluation.

    Contact height is relative to the contact center along insertion, matching
    the tool-Z CAD overlap contract. Support-normal extent is independently
    required and filters which measured points may contribute to that height.
    A bound unit support normal is required to pass; missing support yields
    unsuccessful evidence and None bounds. Results are local to this call.
    """
    measurement = _bilateral_surface_measurement(
        fused_surface,
        contact_center_base,
        jaw_axis_base,
        insertion_axis_base,
        finger_geometry,
        minimum_points_per_side,
        support_normal_base,
    )
    counts, lower, upper, _masks = measurement
    if counts is None:
        counts = (0, 0, 0, 0, 0.0)
    negative, positive, negative_views, positive_views, width = counts
    ok = lower is not None and upper is not None
    evidence = BilateralSurfaceEvidence(
        ok=bool(ok),
        code=(
            'BILATERAL_SURFACE_EVIDENCE_OK'
            if ok
            else 'BILATERAL_SURFACE_EVIDENCE_MISSING'
        ),
        negative_jaw_points=int(negative),
        positive_jaw_points=int(positive),
        negative_view_count=int(negative_views),
        positive_view_count=int(positive_views),
        measured_width_m=float(width),
        contact_height_m=(
            0.5 * (float(lower) + float(upper)) if ok else 0.0
        ),
    )
    bounds = (float(lower), float(upper)) if ok else None
    return evidence, bounds


def evaluate_bilateral_surface_evidence(
    fused_surface,
    contact_center_base,
    jaw_axis_base,
    insertion_axis_base,
    finger_geometry,
    minimum_points_per_side=12,
    *,
    support_normal_base=None,
):
    """Measure jaw bands; a bound unit support normal is required to pass."""

    evidence, _bounds = evaluate_bilateral_surface_evidence_and_bounds(
        fused_surface,
        contact_center_base,
        jaw_axis_base,
        insertion_axis_base,
        finger_geometry,
        minimum_points_per_side,
        support_normal_base=support_normal_base,
    )
    return evidence


def bilateral_surface_contact_bounds_m(
    fused_surface,
    contact_center_base,
    jaw_axis_base,
    insertion_axis_base,
    finger_geometry,
    minimum_points_per_side=12,
    *,
    support_normal_base=None,
):
    """Return the measured tool-Z interval, or None without support evidence."""

    _counts, lower, upper, _masks = _bilateral_surface_measurement(
        fused_surface,
        contact_center_base,
        jaw_axis_base,
        insertion_axis_base,
        finger_geometry,
        minimum_points_per_side,
        support_normal_base,
    )
    if lower is None or upper is None:
        return None
    return float(lower), float(upper)


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


@dataclass(frozen=True)
class ObservationEnvelopeResult:
    """Open-gripper CAD clearance at one non-contact observation endpoint."""

    ok: bool
    failure_code: str
    failure_reason: str
    minimum_support_clearance_m: float

    def __post_init__(self):
        clearance = _finite_number(
            self.minimum_support_clearance_m,
            'minimum_support_clearance_m',
        )
        code = str(self.failure_code or '')
        reason = str(self.failure_reason or '')
        if bool(self.ok):
            if code or reason:
                raise ValueError(
                    'successful observation envelope cannot contain a failure'
                )
        elif not code or not reason:
            raise ValueError(
                'failed observation envelope requires a code and reason'
            )
        object.__setattr__(self, 'ok', bool(self.ok))
        object.__setattr__(self, 'failure_code', code)
        object.__setattr__(self, 'failure_reason', reason)
        object.__setattr__(
            self,
            'minimum_support_clearance_m',
            clearance,
        )


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
    return _finite_number(
        projected_width + 2.0 * clearance,
        'required open width',
    )


def _opening_fit_clearance_each_side_m(value, gripper):
    if value is None:
        return float(gripper.jaw_clearance_each_side_m)
    clearance = _finite_number(
        value,
        'opening_fit_clearance_each_side_m',
    )
    if clearance < 0.0:
        raise ValueError('opening_fit_clearance_each_side_m must be non-negative')
    return clearance


def projected_cloud_width_m(
    points,
    jaw_axis,
    clearance_each_side_m,
    trim_fraction=0.0,
):
    """Return target-cloud span along a unit jaw axis plus jaw clearances."""
    cloud = np.asarray(points, dtype=float)
    if cloud.ndim != 2 or cloud.shape[1:] != (3,) or cloud.shape[0] == 0:
        raise ValueError('points must be a non-empty Nx3 array')
    if not np.all(np.isfinite(cloud)):
        raise ValueError('points contain non-finite values')
    axis = _readonly_vector(jaw_axis, 'jaw_axis')
    if abs(float(np.linalg.norm(axis)) - 1.0) > 1e-6:
        raise ValueError('jaw_axis must be a unit vector')
    clearance = _finite_number(
        clearance_each_side_m,
        'clearance_each_side_m',
    )
    if clearance < 0.0:
        raise ValueError('clearance_each_side_m must be non-negative')
    trim = _finite_number(trim_fraction, 'trim_fraction')
    if not 0.0 <= trim <= 0.05:
        raise ValueError('trim_fraction must be in [0, 0.05]')
    projection = cloud @ axis
    if trim > 0.0:
        lower, upper = np.quantile(projection, (trim, 1.0 - trim))
    else:
        lower, upper = np.min(projection), np.max(projection)
    return _finite_number(
        float(upper - lower) + 2.0 * clearance,
        'projected cloud width',
    )


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
    physical_gap = min(
        float(gripper.max_inner_gap_m),
        ANALYTICAL_MAX_INNER_GAP_M,
    )
    if opening < 0.0 or opening > physical_gap:
        raise ValueError('required_open_width_m is outside the gripper range')
    jaw_local, jaw_index = parse_tool_axis(tool_jaw_axis)
    _finger_local, finger_index = parse_tool_axis(tool_finger_length_axis)
    if jaw_index == finger_index:
        raise ValueError('jaw and finger length axes must be different')
    jaw_axis = rotation @ jaw_local
    finger_thickness = float(gripper.finger_size_xyz_m[jaw_index])
    finger_pair_center = (
        center + rotation @ ANALYTICAL_FINGER_PAIR_CENTER_TOOL_XYZ_M
    )
    finger_offset = 0.5 * (opening + finger_thickness) * jaw_axis
    palm_center = center + rotation @ ANALYTICAL_PALM_CENTER_TOOL_XYZ_M
    return {
        'grasp_center': center.copy(),
        'left_finger': finger_pair_center + finger_offset,
        'right_finger': finger_pair_center - finger_offset,
        'palm': palm_center,
    }


def _signed_polygon_margin(point, polygon):
    """Return positive interior distance or negative exterior distance."""

    sample = np.asarray(point, dtype=float)
    boundary = np.asarray(polygon, dtype=float)
    if (
        sample.shape != (2,)
        or boundary.ndim != 2
        or boundary.shape[0] < 3
        or boundary.shape[1] != 2
        or not np.all(np.isfinite(sample))
        or not np.all(np.isfinite(boundary))
    ):
        raise ValueError('contact patch point and polygon must be finite 2D data')
    following = np.roll(boundary, -1, axis=0)
    edge = following - boundary
    length_squared = np.sum(edge * edge, axis=1)
    if np.any(length_squared <= 1e-18):
        raise ValueError('contact patch polygon contains a degenerate edge')
    fraction = np.clip(
        np.sum((sample - boundary) * edge, axis=1) / length_squared,
        0.0,
        1.0,
    )
    closest = boundary + fraction[:, None] * edge
    distance = float(np.min(np.linalg.norm(sample - closest, axis=1)))
    if distance <= 1e-12:
        return 0.0

    inside = False
    previous = boundary[-1]
    for current in boundary:
        crosses = (
            (current[1] > sample[1]) != (previous[1] > sample[1])
        )
        if crosses:
            crossing_x = (
                (previous[0] - current[0])
                * (sample[1] - current[1])
                / (previous[1] - current[1])
                + current[0]
            )
            if sample[0] < crossing_x:
                inside = not inside
        previous = current
    return distance if inside else -distance


def finger_contact_patch_margin_m(
    candidate_center_base,
    candidate_tool0_base,
    R_base_tool,
):
    """Measure whether a semantic contact center lands on the real finger pad.

    The result is the shortest tool0-X/Z distance to the Link7/Link8 inner
    contact-patch boundary. Positive values are inside, zero is on the mesh
    boundary, and negative values miss the physical patch.
    """

    center = _readonly_vector(candidate_center_base, 'candidate_center_base')
    tool0 = _readonly_vector(candidate_tool0_base, 'candidate_tool0_base')
    rotation = _validated_rotation(R_base_tool, 'R_base_tool')
    pair_center = (
        tool0 + rotation @ ANALYTICAL_FINGER_PAIR_CENTER_TOOL_XYZ_M
    )
    local_center = rotation.T @ (center - pair_center)
    return _signed_polygon_margin(
        local_center[[0, 2]],
        ANALYTICAL_FINGER_CONTACT_PATCH_TOOL_XZ_M,
    )


def contact_height_axis_base(jaw_axis_base, support_normal_base):
    """Return the support-derived height axis inside the jaw contact plane.

    Opposing finger faces are perpendicular to the jaw axis.  For an
    arbitrarily oriented 6D grasp, the world support normal is generally not
    contained in those faces.  Its normalized projection into the contact
    plane is the unique category-independent direction along which target
    height and finger-pad coverage can be compared in common metric units.
    """

    jaw = _readonly_vector(jaw_axis_base, 'jaw_axis_base')
    support = _readonly_vector(
        support_normal_base,
        'support_normal_base',
    )
    jaw_norm = float(np.linalg.norm(jaw))
    support_norm = float(np.linalg.norm(support))
    if not math.isclose(jaw_norm, 1.0, rel_tol=0.0, abs_tol=1e-6):
        raise ValueError('jaw_axis_base must be a unit vector')
    if not math.isclose(support_norm, 1.0, rel_tol=0.0, abs_tol=1e-6):
        raise ValueError('support_normal_base must be a unit vector')
    projected = support - float(np.dot(support, jaw)) * jaw
    projected_norm = float(np.linalg.norm(projected))
    if projected_norm <= 1e-6:
        raise ValueError(
            'jaw axis parallel to support normal has no support-derived '
            'contact-height direction'
        )
    return projected / projected_norm


def bilateral_contact_height_bounds_m(
    target_points_base,
    candidate_center_base,
    jaw_axis_base,
    support_normal_base,
    contact_band_fraction=0.12,
):
    """Return the common measured height interval on both jaw-side bands.

    Heights are expressed along the projection of the live support normal
    into the candidate's jaw contact plane, relative to the candidate contact
    center.  This is identical to the support normal for tabletop-parallel
    jaws and remains metrically valid for a general 6D jaw orientation.
    Intersecting the negative and positive jaw-side bands prevents a one-sided
    cloud edge from creating fictitious opposing contact support.
    """

    points = np.asarray(target_points_base, dtype=float)
    if (
        points.ndim != 2
        or points.shape[1] != 3
        or points.shape[0] < 2
        or not np.all(np.isfinite(points))
    ):
        raise ValueError('target_points_base must be finite Nx3 data')
    center = _readonly_vector(candidate_center_base, 'candidate_center_base')
    jaw = _readonly_vector(jaw_axis_base, 'jaw_axis_base')
    height_axis = contact_height_axis_base(jaw, support_normal_base)
    fraction = _finite_number(
        contact_band_fraction,
        'contact_band_fraction',
    )
    if not 0.0 < fraction < 0.5:
        raise ValueError('contact_band_fraction must be in (0, 0.5)')

    delta = points - center
    jaw_projection = delta @ jaw
    lower = float(np.min(jaw_projection))
    upper = float(np.max(jaw_projection))
    span = upper - lower
    if not math.isfinite(span) or span <= 1e-9:
        return None
    band_width = span * fraction
    negative = delta[jaw_projection <= lower + band_width]
    positive = delta[jaw_projection >= upper - band_width]
    if negative.shape[0] == 0 or positive.shape[0] == 0:
        return None
    negative_heights = negative @ height_axis
    positive_heights = positive @ height_axis
    lower_height = max(
        float(np.min(negative_heights)),
        float(np.min(positive_heights)),
    )
    upper_height = min(
        float(np.max(negative_heights)),
        float(np.max(positive_heights)),
    )
    if (
        not math.isfinite(lower_height)
        or not math.isfinite(upper_height)
        or upper_height < lower_height
    ):
        return None
    return float(lower_height), float(upper_height)


def _cross_2d(first, second):
    return float(first[0] * second[1] - first[1] * second[0])


def finger_contact_patch_height_intervals_m(
    candidate_center_base,
    candidate_tool0_base,
    R_base_tool,
    support_normal_base,
    height_min_m,
    height_max_m,
):
    """Intersect a live support-normal height line with the CAD contact patch.

    Returned intervals are continuous height ranges relative to the candidate
    center.  The exact polygon is concave, so this uses all polygon-edge
    crossings rather than a convex support-function approximation.
    """

    center = _readonly_vector(candidate_center_base, 'candidate_center_base')
    tool0 = _readonly_vector(candidate_tool0_base, 'candidate_tool0_base')
    rotation = _validated_rotation(R_base_tool, 'R_base_tool')
    support = _readonly_vector(
        support_normal_base,
        'support_normal_base',
    )
    support_norm = float(np.linalg.norm(support))
    if not math.isclose(support_norm, 1.0, rel_tol=0.0, abs_tol=1e-6):
        raise ValueError('support_normal_base must be a unit vector')
    lower = _finite_number(height_min_m, 'height_min_m')
    upper = _finite_number(height_max_m, 'height_max_m')
    if upper < lower:
        raise ValueError('contact height bounds must be ordered')

    pair_center = (
        tool0 + rotation @ ANALYTICAL_FINGER_PAIR_CENTER_TOOL_XYZ_M
    )
    local_center = rotation.T @ (center - pair_center)
    local_direction = rotation.T @ support
    point = local_center[[0, 2]]
    direction = local_direction[[0, 2]]
    polygon = ANALYTICAL_FINGER_CONTACT_PATCH_TOOL_XZ_M
    direction_norm_squared = float(np.dot(direction, direction))
    if direction_norm_squared <= 1e-18:
        if _signed_polygon_margin(point, polygon) >= -1e-12:
            return ((float(lower), float(upper)),)
        return ()

    breakpoints = [float(lower), float(upper)]
    for edge_start, edge_end in zip(polygon, np.roll(polygon, -1, axis=0)):
        edge = edge_end - edge_start
        denominator = _cross_2d(direction, edge)
        offset = edge_start - point
        if abs(denominator) > 1e-14:
            height = _cross_2d(offset, edge) / denominator
            edge_fraction = _cross_2d(offset, direction) / denominator
            if (
                lower - 1e-12 <= height <= upper + 1e-12
                and -1e-12 <= edge_fraction <= 1.0 + 1e-12
            ):
                breakpoints.append(
                    min(float(upper), max(float(lower), float(height)))
                )
            continue
        if abs(_cross_2d(offset, direction)) > 1e-12:
            continue
        first_height = float(
            np.dot(edge_start - point, direction)
            / direction_norm_squared
        )
        second_height = float(
            np.dot(edge_end - point, direction)
            / direction_norm_squared
        )
        for height in (first_height, second_height):
            if lower - 1e-12 <= height <= upper + 1e-12:
                breakpoints.append(
                    min(float(upper), max(float(lower), height))
                )

    ordered = []
    for value in sorted(breakpoints):
        if not ordered or abs(value - ordered[-1]) > 1e-11:
            ordered.append(float(value))
    intervals = []
    for first, second in zip(ordered, ordered[1:]):
        if second - first <= 1e-12:
            continue
        midpoint = 0.5 * (first + second)
        sample = point + midpoint * direction
        if _signed_polygon_margin(sample, polygon) < -1e-10:
            continue
        if intervals and first - intervals[-1][1] <= 1e-10:
            intervals[-1] = (intervals[-1][0], second)
        else:
            intervals.append((first, second))
    return tuple((float(first), float(second)) for first, second in intervals)


def finger_contact_patch_overlap_m(
    candidate_center_base,
    candidate_tool0_base,
    R_base_tool,
    support_normal_base,
    height_min_m,
    height_max_m,
):
    """Return the longest continuous target-height/CAD-patch intersection."""

    intervals = finger_contact_patch_height_intervals_m(
        candidate_center_base,
        candidate_tool0_base,
        R_base_tool,
        support_normal_base,
        height_min_m,
        height_max_m,
    )
    if not intervals:
        return 0.0
    return max(float(upper - lower) for lower, upper in intervals)


def _obb_line_interval(center, direction, obb_center, rotation, size):
    local_center = rotation.T @ (center - obb_center)
    local_direction = rotation.T @ direction
    half = 0.5 * size
    if not (
        np.all(np.isfinite(local_center))
        and np.all(np.isfinite(local_direction))
        and np.all(np.isfinite(half))
    ):
        return None
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


def _carried_obb_pose(
    transform,
    grasp_transform,
    obb_center,
    obb_rotation,
):
    """Transport a grasped target rigidly with the current tool transform."""

    current = np.asarray(transform, dtype=float)
    grasp = np.asarray(grasp_transform, dtype=float)
    center = np.asarray(obb_center, dtype=float)
    rotation = np.asarray(obb_rotation, dtype=float)
    relative_rotation = current[:3, :3] @ grasp[:3, :3].T
    return (
        current[:3, 3]
        + relative_rotation @ (center - grasp[:3, 3]),
        relative_rotation @ rotation,
    )


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
    finger_box_size = (
        gripper.finger_size_xyz_m + ANALYTICAL_FINGER_BOX_PADDING_XYZ_M
    )
    return (
        ('left_finger', centers['left_finger'], finger_box_size),
        ('right_finger', centers['right_finger'], finger_box_size),
        ('palm', centers['palm'], gripper.palm_size_xyz_m),
    )


def _intended_finger_contact(
    box_name,
    box_center,
    stage_center,
    jaw_axis,
    jaw_index,
    finger_size,
    finger_padding_size,
    opening_width_m,
    contact_gap_tolerance_m,
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
    tolerated_gap = max(
        0.0,
        float(contact_gap_tolerance_m),
    ) + 0.5 * float(finger_padding_size[jaw_index])
    return (
        -tolerated_gap - 1e-6
        <= penetration
        <= 0.5 * finger_thickness + 1e-6
    )


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
    contact_gap_tolerance_m=0.0,
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
                ANALYTICAL_FINGER_BOX_PADDING_XYZ_M,
                opening_width_m,
                contact_gap_tolerance_m,
                obb_center,
                obb_rotation,
                obb_size,
            )
        ):
            continue
        return False, minimum_clearance, '%s intrudes into target OBB' % name
    return True, minimum_clearance, ''


def evaluate_open_gripper_observation_envelope(
    *,
    gripper,
    T_base_tool0,
    opening_width_m,
    support_normal_base,
    support_offset_m,
    obb_center_base,
    R_base_obb,
    obb_size_xyz_m,
    tool_jaw_axis='y',
    tool_finger_length_axis='z',
):
    """Check an open, non-contact observation pose against live geometry.

    MoveIt does not necessarily contain the measured support plane or target
    OBB.  This endpoint-only gate therefore checks the same conservative palm
    and finger CAD boxes used by the contact sequence, while allowing no
    target contact at an observation pose.
    """

    minimum_clearance = -1.0e6
    try:
        if not isinstance(gripper, GripperGeometry):
            raise ValueError('gripper must be a GripperGeometry')
        transform = _validated_transform(T_base_tool0, 'T_base_tool0')
        opening = _finite_number(opening_width_m, 'opening_width_m')
        if opening < 0.0 or opening > gripper.max_inner_gap_m + 1e-9:
            raise ValueError(
                'opening_width_m must be inside the physical gripper range'
            )
        support_normal = _readonly_vector(
            support_normal_base,
            'support_normal_base',
        )
        if not math.isclose(
            float(np.linalg.norm(support_normal)),
            1.0,
            rel_tol=0.0,
            abs_tol=1e-6,
        ):
            raise ValueError('support_normal_base must be a unit vector')
        support_offset = _finite_number(
            support_offset_m,
            'support_offset_m',
        )
        obb_center = _readonly_vector(
            obb_center_base,
            'obb_center_base',
        )
        obb_rotation = _validated_rotation(
            R_base_obb,
            'R_base_obb',
        )
        obb_size = _readonly_vector(
            obb_size_xyz_m,
            'obb_size_xyz_m',
        )
        if np.any(obb_size <= 0.0):
            raise ValueError('obb_size_xyz_m values must be positive')
        safe, minimum_clearance, reason = _check_boxes(
            transform,
            gripper,
            opening,
            tool_jaw_axis,
            tool_finger_length_axis,
            support_normal,
            support_offset,
            obb_center,
            obb_rotation,
            obb_size,
            False,
        )
    except Exception as exc:
        return ObservationEnvelopeResult(
            ok=False,
            failure_code='OBSERVATION_ENVELOPE_INVALID',
            failure_reason='invalid observation envelope input: %s' % exc,
            minimum_support_clearance_m=minimum_clearance,
        )
    if not safe:
        code = (
            'OBSERVATION_SUPPORT_COLLISION'
            if 'support clearance' in reason
            else 'OBSERVATION_TARGET_COLLISION'
        )
        return ObservationEnvelopeResult(
            ok=False,
            failure_code=code,
            failure_reason=str(reason),
            minimum_support_clearance_m=minimum_clearance,
        )
    return ObservationEnvelopeResult(
        ok=True,
        failure_code='',
        failure_reason='',
        minimum_support_clearance_m=minimum_clearance,
    )


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
    def finite_or(value, fallback):
        try:
            number = float(value)
        except (TypeError, ValueError, OverflowError):
            return float(fallback)
        return number if np.isfinite(number) else float(fallback)

    return CandidateGateResult(
        ok=False,
        failure_code=code,
        failure_reason=reason,
        required_open_width_m=max(0.0, finite_or(required_width, 0.0)),
        center_distance_m=max(0.0, finite_or(center_distance, 0.0)),
        support_clearance_m=finite_or(support_clearance, -1.0e6),
        jaw_alignment=min(1.0, max(0.0, finite_or(jaw_alignment, 0.0))),
        motion_cost=max(0.0, finite_or(motion_cost, 0.0)),
        geometry_cost=max(0.0, finite_or(geometry_cost, 0.0)),
        failed_gate=gate,
        passed_gate_count=min(_GATE_COUNT, max(0, int(passed))),
    )


def _evaluate_physical_candidate(
    *,
    gripper,
    candidate_center_base,
    candidate_tool0_base,
    R_base_tool,
    required_width_m,
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
    opening_fit_clearance_each_side_m=None,
    contact_height_bounds_m=None,
    contact_height_axis_base_override=None,
    minimum_contact_patch_overlap_m=GRIPPER_CONTRACT_TOLERANCE_M,
):
    """Run source-independent physical gates for an explicit tool0 pose."""
    required_width = 0.0
    center_distance = 0.0
    support_clearance = -1.0e6
    jaw_alignment = 0.0
    geometry_cost = 0.0
    safe_motion_cost = 0.0
    try:
        if not isinstance(gripper, GripperGeometry):
            raise ValueError('gripper must be a GripperGeometry')
        fit_clearance = _opening_fit_clearance_each_side_m(
            opening_fit_clearance_each_side_m,
            gripper,
        )
        center = _readonly_vector(candidate_center_base, 'candidate_center_base')
        tool0 = _readonly_vector(candidate_tool0_base, 'candidate_tool0_base')
        tool_rotation = _validated_rotation(R_base_tool, 'R_base_tool')
        required_width = _finite_number(
            required_width_m,
            'required_width_m',
        )
        if required_width < 0.0:
            raise ValueError('required_width_m must be non-negative')
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
        if not math.isclose(normal_norm, 1.0, rel_tol=0.0, abs_tol=1e-6):
            raise ValueError('support_normal_base must be a unit vector')
        support_alignment = float(np.dot(support_normal, obb_rotation[:, 2]))
        if support_alignment < 1.0 - 1e-5:
            raise ValueError(
                'support_normal_base must align with OBB positive z'
            )
        support_offset = _finite_number(support_offset_m, 'support_offset_m')
        minimum_contact_overlap = _finite_number(
            minimum_contact_patch_overlap_m,
            'minimum_contact_patch_overlap_m',
        )
        if minimum_contact_overlap < 0.0:
            raise ValueError(
                'minimum_contact_patch_overlap_m must be non-negative'
            )
        jaw_for_height = tool_rotation @ parse_tool_axis(tool_jaw_axis)[0]
        if contact_height_axis_base_override is None:
            contact_height_axis = contact_height_axis_base(
                jaw_for_height,
                support_normal,
            )
        else:
            contact_height_axis = _readonly_vector(
                contact_height_axis_base_override,
                'contact_height_axis_base_override',
            )
            if not math.isclose(
                float(np.linalg.norm(contact_height_axis)),
                1.0,
                rel_tol=0.0,
                abs_tol=1e-6,
            ) or abs(float(np.dot(contact_height_axis, jaw_for_height))) > 1e-6:
                raise ValueError(
                    'contact height axis must be unit and orthogonal to jaw'
                )
        obb_height_center = float(
            np.dot(obb_center - center, contact_height_axis)
        )
        obb_height_half_extent = 0.5 * float(
            np.sum(
                np.abs(obb_rotation.T @ contact_height_axis) * obb_size
            )
        )
        obb_height_bounds = (
            obb_height_center - obb_height_half_extent,
            obb_height_center + obb_height_half_extent,
        )
        if contact_height_bounds_m is None:
            contact_height_bounds = obb_height_bounds
        else:
            bounds = np.asarray(contact_height_bounds_m, dtype=float)
            if (
                bounds.shape != (2,)
                or not np.all(np.isfinite(bounds))
                or float(bounds[1]) < float(bounds[0])
            ):
                raise ValueError(
                    'contact_height_bounds_m must be two ordered finite values'
                )
            contact_height_bounds = (
                max(float(bounds[0]), obb_height_bounds[0]),
                min(float(bounds[1]), obb_height_bounds[1]),
            )
        safe_motion_cost = _finite_number(motion_cost, 'motion_cost')
        if safe_motion_cost < 0.0:
            raise ValueError('motion_cost must be non-negative')
        transforms = [
            _validated_transform(pregrasp_T_base_tool, 'pregrasp_T_base_tool'),
            _validated_transform(approach_T_base_tool, 'approach_T_base_tool'),
            _validated_transform(grasp_T_base_tool, 'grasp_T_base_tool'),
            _validated_transform(lift_T_base_tool, 'lift_T_base_tool'),
        ]
        # ``center`` is the source's jaw/contact center.  The transforms are
        # the physical Alicia tool0 trajectory, so analytical boxes remain
        # anchored to the actual robot frame for every candidate source.
        if not np.allclose(transforms[2][:3, 3], tool0, atol=1e-7):
            raise AnalyticalGripperContractError(
                'TOOL0_INCONSISTENT',
                'candidate tool0 does not match grasp transform translation',
            )
        if not all(
            np.allclose(transforms[index][:3, :3], tool_rotation, atol=1e-7)
            for index in (1, 2)
        ):
            raise ValueError(
                'approach/grasp rotation does not match contact candidate'
            )
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
        if not np.all(
            np.isfinite(
                [
                    center_distance,
                    support_clearance,
                    jaw_alignment,
                    geometry_cost,
                ]
            )
        ):
            raise ValueError('derived analytical candidate metrics are non-finite')
    except Exception as exc:
        failure_code = (
            str(exc.code)
            if isinstance(exc, AnalyticalGripperContractError)
            else 'GRIPPER_SWEEP_COLLISION'
        )
        return _failed_result(
            'transform',
            failure_code,
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
            'candidate center is outside the target OBB tolerance',
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
            'candidate jaw line does not cross both sides of the target OBB',
            2,
            required_width,
            center_distance,
            support_clearance,
            jaw_alignment,
            safe_motion_cost,
            geometry_cost,
        )
    physical_gap = min(
        float(gripper.max_inner_gap_m),
        ANALYTICAL_MAX_INNER_GAP_M,
    )
    if required_width > physical_gap:
        return _failed_result(
            'jaw_width',
            'GRIPPER_TOO_NARROW',
            'required opening %.6fm exceeds physical inner gap %.6fm'
            % (required_width, physical_gap),
            2,
            required_width,
            center_distance,
            support_clearance,
            jaw_alignment,
            safe_motion_cost,
            geometry_cost,
        )

    negative_reach = -float(interval[0]) + fit_clearance
    positive_reach = float(interval[1]) + fit_clearance
    maximum_side_reach = 0.5 * physical_gap
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

    if contact_height_bounds[1] < contact_height_bounds[0]:
        contact_patch_overlap = 0.0
    else:
        contact_patch_overlap = finger_contact_patch_overlap_m(
            center,
            tool0,
            tool_rotation,
            contact_height_axis,
            contact_height_bounds[0],
            contact_height_bounds[1],
        )
    if contact_patch_overlap + 1e-9 < minimum_contact_overlap:
        return _failed_result(
            'finger_reach',
            'GRIPPER_CONTACT_PATCH_MISS',
            (
                'continuous bilateral target/CAD contact overlap %.6fm '
                'is below required %.6fm'
                % (contact_patch_overlap, minimum_contact_overlap)
            ),
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
        endpoint_obb_rotation = obb_rotation
        if endpoint_index == 3:
            endpoint_obb_center, endpoint_obb_rotation = _carried_obb_pose(
                transform,
                transforms[2],
                obb_center,
                obb_rotation,
            )
        safe, clearance, reason = _check_boxes(
            transform,
            gripper,
            physical_gap,
            tool_jaw_axis,
            tool_finger_length_axis,
            support_normal,
            support_offset,
            endpoint_obb_center,
            endpoint_obb_rotation,
            obb_size,
            allow_contact,
            fit_clearance,
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
                'approach enters the target from the support-plane side',
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
            swept_obb_rotation = obb_rotation
            if segment_index == 2:
                swept_obb_center, swept_obb_rotation = _carried_obb_pose(
                    transform,
                    transforms[2],
                    obb_center,
                    obb_rotation,
                )
            safe, clearance, reason = _check_boxes(
                transform,
                gripper,
                physical_gap,
                tool_jaw_axis,
                tool_finger_length_axis,
                support_normal,
                support_offset,
                swept_obb_center,
                swept_obb_rotation,
                obb_size,
                segment_contact[segment_index],
                fit_clearance,
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


def _evaluate_candidate_impl(
    *,
    gripper,
    candidate_center_base,
    candidate_tool0_base,
    candidate_depth_m,
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
    opening_fit_clearance_each_side_m=None,
    target_points_base=None,
    contact_band_fraction=0.12,
    bilateral_surface_evidence=None,
    contact_height_bounds_m=None,
    contact_height_axis_base_override=None,
    minimum_contact_patch_overlap_m=GRIPPER_CONTRACT_TOLERANCE_M,
):
    """Validate the strict GraspNet source contract, then run physical gates."""
    required_width = 0.0
    contact_height_bounds = None
    try:
        if not isinstance(gripper, GripperGeometry):
            raise ValueError('gripper must be a GripperGeometry')
        fit_clearance = _opening_fit_clearance_each_side_m(
            opening_fit_clearance_each_side_m,
            gripper,
        )
        center = _readonly_vector(candidate_center_base, 'candidate_center_base')
        tool0 = _readonly_vector(candidate_tool0_base, 'candidate_tool0_base')
        tool_rotation = _validated_rotation(R_base_tool, 'R_base_tool')
        if isinstance(candidate_depth_m, (bool, np.bool_)):
            raise AnalyticalGripperContractError(
                'DEPTH_INVALID',
                'candidate_depth_m must be a numeric GraspNet depth bin',
            )
        try:
            depth = _finite_number(candidate_depth_m, 'candidate_depth_m')
        except Exception as exc:
            raise AnalyticalGripperContractError(
                'DEPTH_INVALID',
                'candidate_depth_m must be finite',
            ) from exc
        nearest_depth = min(
            ANALYTICAL_GRASPNET_DEPTH_BINS_M,
            key=lambda item: abs(depth - item),
        )
        if abs(depth - nearest_depth) > ANALYTICAL_GRASPNET_DEPTH_TOLERANCE_M:
            raise AnalyticalGripperContractError(
                'DEPTH_OUT_OF_RANGE',
                'candidate_depth_m %.9g is not a GraspNet depth bin' % depth,
            )
        expected_tool0 = center + float(nearest_depth) * tool_rotation[:, 2]
        if not np.all(np.isfinite(expected_tool0)):
            raise AnalyticalGripperContractError(
                'TOOL0_INVALID',
                'derived candidate tool0 is non-finite',
            )
        if not np.allclose(tool0, expected_tool0, atol=1e-7):
            raise AnalyticalGripperContractError(
                'TOOL0_INCONSISTENT',
                'candidate tool0 must equal center + depth * tool +Z',
            )
        model_width = _finite_number(candidate_width_m, 'candidate_width_m')
        if model_width < 0.0:
            raise ValueError('candidate_width_m must be non-negative')
        obb_rotation = _validated_rotation(R_base_obb, 'R_base_obb')
        obb_size = _readonly_vector(obb_size_xyz_m, 'obb_size_xyz_m')
        if np.any(obb_size <= 0.0):
            raise ValueError('obb_size_xyz_m values must be positive')
        jaw_local, jaw_index = parse_tool_axis(tool_jaw_axis)
        _finger_local, finger_index = parse_tool_axis(tool_finger_length_axis)
        if jaw_index == finger_index:
            raise ValueError('jaw and finger length axes must be different')
        if bilateral_surface_evidence is not None:
            if (
                not isinstance(
                    bilateral_surface_evidence, BilateralSurfaceEvidence
                )
                or not bilateral_surface_evidence.ok
            ):
                raise ValueError(
                    'bilateral_surface_evidence must be successful measured evidence'
                )
            required_width = (
                float(bilateral_surface_evidence.measured_width_m)
                + 2.0 * fit_clearance
            )
            if contact_height_bounds_m is None:
                raise ValueError(
                    'measured bilateral evidence requires contact height bounds'
                )
            contact_height_bounds = tuple(
                float(value) for value in contact_height_bounds_m
            )
        else:
            required_width = required_open_width_m(
                obb_size,
                obb_rotation,
                tool_rotation @ jaw_local,
                fit_clearance,
            )
        if target_points_base is not None and bilateral_surface_evidence is None:
            contact_height_bounds = bilateral_contact_height_bounds_m(
                target_points_base,
                center,
                tool_rotation @ jaw_local,
                support_normal_base,
                contact_band_fraction,
            )
            if contact_height_bounds is None:
                return _failed_result(
                    'finger_reach',
                    'GRIPPER_CONTACT_PATCH_MISS',
                    'target cloud has no common bilateral contact height',
                    3,
                    required_width,
                    0.0,
                    -1.0e6,
                    0.0,
                    0.0,
                    0.0,
                )
    except Exception as exc:
        failure_code = (
            str(exc.code)
            if isinstance(exc, AnalyticalGripperContractError)
            else 'GRIPPER_SWEEP_COLLISION'
        )
        return _failed_result(
            'transform',
            failure_code,
            'invalid analytical gripper input: %s' % exc,
            0,
            required_width,
            0.0,
            -1.0e6,
            0.0,
            0.0,
            0.0,
        )
    return _evaluate_physical_candidate(
        gripper=gripper,
        candidate_center_base=candidate_center_base,
        candidate_tool0_base=candidate_tool0_base,
        R_base_tool=R_base_tool,
        required_width_m=required_width,
        obb_center_base=obb_center_base,
        R_base_obb=R_base_obb,
        obb_size_xyz_m=obb_size_xyz_m,
        support_normal_base=support_normal_base,
        support_offset_m=support_offset_m,
        pregrasp_T_base_tool=pregrasp_T_base_tool,
        approach_T_base_tool=approach_T_base_tool,
        grasp_T_base_tool=grasp_T_base_tool,
        lift_T_base_tool=lift_T_base_tool,
        tool_jaw_axis=tool_jaw_axis,
        tool_finger_length_axis=tool_finger_length_axis,
        motion_cost=motion_cost,
        opening_fit_clearance_each_side_m=fit_clearance,
        contact_height_bounds_m=contact_height_bounds,
        contact_height_axis_base_override=contact_height_axis_base_override,
        minimum_contact_patch_overlap_m=minimum_contact_patch_overlap_m,
    )


def _evaluate_explicit_candidate_impl(
    *,
    gripper,
    candidate_center_base,
    candidate_tool0_base,
    R_base_tool,
    required_open_width_m,
    target_points_base,
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
    opening_fit_clearance_each_side_m=None,
    width_projection_trim_fraction=0.0,
    contact_band_fraction=0.12,
    contact_height_bounds_m=None,
    bilateral_surface_evidence=None,
    contact_height_axis_base_override=None,
    minimum_contact_patch_overlap_m=GRIPPER_CONTRACT_TOLERANCE_M,
):
    """Validate an explicit target-cloud width, then run physical gates."""
    contact_height_bounds = None
    try:
        if not isinstance(gripper, GripperGeometry):
            raise ValueError('gripper must be a GripperGeometry')
        fit_clearance = _opening_fit_clearance_each_side_m(
            opening_fit_clearance_each_side_m,
            gripper,
        )
        rotation = _validated_rotation(R_base_tool, 'R_base_tool')
        jaw_local, jaw_index = parse_tool_axis(tool_jaw_axis)
        _finger_local, finger_index = parse_tool_axis(tool_finger_length_axis)
        if jaw_index == finger_index:
            raise ValueError('jaw and finger length axes must be different')
        if bilateral_surface_evidence is not None:
            if (
                not isinstance(
                    bilateral_surface_evidence, BilateralSurfaceEvidence
                )
                or not bilateral_surface_evidence.ok
            ):
                raise ValueError(
                    'bilateral_surface_evidence must be successful measured evidence'
                )
            recomputed = (
                float(bilateral_surface_evidence.measured_width_m)
                + 2.0 * fit_clearance
            )
            measured_contact_height_bounds = contact_height_bounds_m
            if measured_contact_height_bounds is None:
                raise ValueError(
                    'measured bilateral evidence requires contact height bounds'
                )
            contact_height_bounds = tuple(
                float(value) for value in measured_contact_height_bounds
            )
        else:
            recomputed = projected_cloud_width_m(
                target_points_base,
                rotation @ jaw_local,
                fit_clearance,
                trim_fraction=width_projection_trim_fraction,
            )
            measured_contact_height_bounds = bilateral_contact_height_bounds_m(
                target_points_base,
                candidate_center_base,
                rotation @ jaw_local,
                support_normal_base,
                contact_band_fraction,
            )
            if measured_contact_height_bounds is None:
                return _failed_result(
                    'finger_reach',
                    'GRIPPER_CONTACT_PATCH_MISS',
                    'target cloud has no common bilateral contact height',
                    3,
                    recomputed,
                    0.0,
                    -1.0e6,
                    0.0,
                    motion_cost,
                    0.0,
                )
            contact_height_bounds = measured_contact_height_bounds
        if (
            contact_height_bounds_m is not None
            and bilateral_surface_evidence is None
        ):
            supplied_bounds = np.asarray(
                contact_height_bounds_m,
                dtype=float,
            ).reshape(-1)
            if (
                supplied_bounds.shape != (2,)
                or not np.all(np.isfinite(supplied_bounds))
                or supplied_bounds[1] < supplied_bounds[0]
            ):
                raise ValueError(
                    'contact_height_bounds_m must be two ordered finite values'
                )
            height_axis = contact_height_axis_base(
                rotation @ jaw_local,
                support_normal_base,
            )
            obb_interval = _obb_line_interval(
                np.asarray(candidate_center_base, dtype=float),
                height_axis,
                np.asarray(obb_center_base, dtype=float),
                np.asarray(R_base_obb, dtype=float),
                np.asarray(obb_size_xyz_m, dtype=float),
            )
            if obb_interval is None:
                raise ValueError(
                    'contact height override has no target OBB intersection'
                )
            tolerance = GRIPPER_CONTRACT_TOLERANCE_M
            if (
                float(supplied_bounds[0]) < float(obb_interval[0]) - tolerance
                or float(supplied_bounds[1]) > float(obb_interval[1]) + tolerance
            ):
                raise ValueError(
                    'contact height override exceeds the target OBB interval'
                )
            measured_overlap = min(
                float(supplied_bounds[1]),
                float(measured_contact_height_bounds[1]),
            ) - max(
                float(supplied_bounds[0]),
                float(measured_contact_height_bounds[0]),
            )
            if measured_overlap < -1e-9:
                raise ValueError(
                    'contact height override is disjoint from measured bilateral support'
                )
            contact_height_bounds = (
                float(supplied_bounds[0]),
                float(supplied_bounds[1]),
            )
        supplied_width = _finite_number(
            required_open_width_m,
            'required_open_width_m',
        )
        if supplied_width < 0.0:
            raise ValueError('required_open_width_m must be non-negative')
        if abs(supplied_width - recomputed) > 5e-4:
            return _failed_result(
                'jaw_width',
                'GRIPPER_WIDTH_INVALID',
                'explicit width does not match target-cloud projection',
                2,
                recomputed,
                0.0,
                -1.0e6,
                0.0,
                motion_cost,
                0.0,
            )
    except Exception as exc:
        failure_code = (
            str(exc.code)
            if isinstance(exc, AnalyticalGripperContractError)
            else 'GRIPPER_SWEEP_COLLISION'
        )
        return _failed_result(
            'transform',
            failure_code,
            'invalid explicit analytical gripper input: %s' % exc,
            0,
            0.0,
            0.0,
            -1.0e6,
            0.0,
            motion_cost,
            0.0,
        )
    return _evaluate_physical_candidate(
        gripper=gripper,
        candidate_center_base=candidate_center_base,
        candidate_tool0_base=candidate_tool0_base,
        R_base_tool=R_base_tool,
        required_width_m=recomputed,
        obb_center_base=obb_center_base,
        R_base_obb=R_base_obb,
        obb_size_xyz_m=obb_size_xyz_m,
        support_normal_base=support_normal_base,
        support_offset_m=support_offset_m,
        pregrasp_T_base_tool=pregrasp_T_base_tool,
        approach_T_base_tool=approach_T_base_tool,
        grasp_T_base_tool=grasp_T_base_tool,
        lift_T_base_tool=lift_T_base_tool,
        tool_jaw_axis=tool_jaw_axis,
        tool_finger_length_axis=tool_finger_length_axis,
        motion_cost=motion_cost,
        opening_fit_clearance_each_side_m=fit_clearance,
        contact_height_bounds_m=contact_height_bounds,
        contact_height_axis_base_override=contact_height_axis_base_override,
        minimum_contact_patch_overlap_m=minimum_contact_patch_overlap_m,
    )


def evaluate_candidate(
    *,
    gripper,
    candidate_center_base,
    candidate_tool0_base,
    candidate_depth_m,
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
    opening_fit_clearance_each_side_m=None,
    target_points_base=None,
    contact_band_fraction=0.12,
    bilateral_surface_evidence=None,
    contact_height_bounds_m=None,
    contact_height_axis_base_override=None,
    minimum_contact_patch_overlap_m=GRIPPER_CONTRACT_TOLERANCE_M,
):
    """Evaluate one candidate and always fail closed on invalid derived state."""
    try:
        return _evaluate_candidate_impl(
            gripper=gripper,
            candidate_center_base=candidate_center_base,
            candidate_tool0_base=candidate_tool0_base,
            candidate_depth_m=candidate_depth_m,
            R_base_tool=R_base_tool,
            candidate_width_m=candidate_width_m,
            obb_center_base=obb_center_base,
            R_base_obb=R_base_obb,
            obb_size_xyz_m=obb_size_xyz_m,
            support_normal_base=support_normal_base,
            support_offset_m=support_offset_m,
            pregrasp_T_base_tool=pregrasp_T_base_tool,
            approach_T_base_tool=approach_T_base_tool,
            grasp_T_base_tool=grasp_T_base_tool,
            lift_T_base_tool=lift_T_base_tool,
            tool_jaw_axis=tool_jaw_axis,
            tool_finger_length_axis=tool_finger_length_axis,
            motion_cost=motion_cost,
            opening_fit_clearance_each_side_m=opening_fit_clearance_each_side_m,
            target_points_base=target_points_base,
            contact_band_fraction=contact_band_fraction,
            bilateral_surface_evidence=bilateral_surface_evidence,
            contact_height_bounds_m=contact_height_bounds_m,
            contact_height_axis_base_override=(
                contact_height_axis_base_override
            ),
            minimum_contact_patch_overlap_m=(
                minimum_contact_patch_overlap_m
            ),
        )
    except Exception as exc:
        return _failed_result(
            'transform',
            (
                str(exc.code)
                if isinstance(exc, AnalyticalGripperContractError)
                else 'GRIPPER_SWEEP_COLLISION'
            ),
            'analytical gripper evaluation failed closed: %s' % exc,
            0,
            0.0,
            0.0,
            -1.0e6,
            0.0,
            0.0,
            0.0,
        )


def evaluate_explicit_candidate(
    *,
    gripper,
    candidate_center_base,
    candidate_tool0_base,
    R_base_tool,
    required_open_width_m,
    target_points_base,
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
    opening_fit_clearance_each_side_m=None,
    width_projection_trim_fraction=0.0,
    contact_band_fraction=0.12,
    contact_height_bounds_m=None,
    bilateral_surface_evidence=None,
    contact_height_axis_base_override=None,
    minimum_contact_patch_overlap_m=GRIPPER_CONTRACT_TOLERANCE_M,
):
    """Evaluate an explicit tool0 candidate without a GraspNet depth field."""
    try:
        return _evaluate_explicit_candidate_impl(
            gripper=gripper,
            candidate_center_base=candidate_center_base,
            candidate_tool0_base=candidate_tool0_base,
            R_base_tool=R_base_tool,
            required_open_width_m=required_open_width_m,
            target_points_base=target_points_base,
            obb_center_base=obb_center_base,
            R_base_obb=R_base_obb,
            obb_size_xyz_m=obb_size_xyz_m,
            support_normal_base=support_normal_base,
            support_offset_m=support_offset_m,
            pregrasp_T_base_tool=pregrasp_T_base_tool,
            approach_T_base_tool=approach_T_base_tool,
            grasp_T_base_tool=grasp_T_base_tool,
            lift_T_base_tool=lift_T_base_tool,
            tool_jaw_axis=tool_jaw_axis,
            tool_finger_length_axis=tool_finger_length_axis,
            motion_cost=motion_cost,
            opening_fit_clearance_each_side_m=opening_fit_clearance_each_side_m,
            width_projection_trim_fraction=width_projection_trim_fraction,
            contact_band_fraction=contact_band_fraction,
            contact_height_bounds_m=contact_height_bounds_m,
            bilateral_surface_evidence=bilateral_surface_evidence,
            contact_height_axis_base_override=(
                contact_height_axis_base_override
            ),
            minimum_contact_patch_overlap_m=(
                minimum_contact_patch_overlap_m
            ),
        )
    except Exception as exc:
        return _failed_result(
            'transform',
            (
                str(exc.code)
                if isinstance(exc, AnalyticalGripperContractError)
                else 'GRIPPER_SWEEP_COLLISION'
            ),
            'explicit analytical gripper evaluation failed closed: %s' % exc,
            0,
            0.0,
            0.0,
            -1.0e6,
            0.0,
            0.0,
            0.0,
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
    """Validate configuration only; Task 11 remains the MJCF endpoint authority.

    The measured physical open gap retains an independent symmetric 0.5 mm
    comparison.  Collision box sizes and jaw/support safety clearances use a
    one-sided conservative contract instead: they may grow by at most
    ``tolerance_m`` (itself capped at the fixed contract tolerance), while
    shrinking is allowed only within floating-point epsilon.
    """
    if not isinstance(gripper, GripperGeometry):
        return 'analytical gripper geometry is unavailable'
    tolerance = _finite_number(tolerance_m, 'tolerance_m')
    if tolerance < 0.0:
        return 'gripper contract tolerance is negative'
    if (
        tolerance
        > GRIPPER_CONTRACT_TOLERANCE_M
        + _CONSERVATIVE_ENVELOPE_FLOAT_EPSILON_M
    ):
        return (
            'gripper contract tolerance %.9fm exceeds fixed %.9fm maximum'
            % (tolerance, GRIPPER_CONTRACT_TOLERANCE_M)
        )
    tolerance = min(tolerance, GRIPPER_CONTRACT_TOLERANCE_M)
    checks = (
        ('analytical max inner gap', gripper.max_inner_gap_m),
        ('remote max inner gap', remote_max_inner_gap_m),
        ('digital-twin max inner gap', twin_max_inner_gap_m),
    )
    for label, value in checks:
        try:
            number = _finite_number(value, label)
        except Exception as exc:
            return str(exc)
        if (
            abs(number - ANALYTICAL_MAX_INNER_GAP_M)
            > tolerance + _CONSERVATIVE_ENVELOPE_FLOAT_EPSILON_M
        ):
            return (
                '%s %.6fm differs from fixed analytical 50 mm contract'
                % (label, number)
            )
    try:
        physical_open_width = _finite_number(
            physical_open_width_m,
            'physical open width',
        )
    except Exception as exc:
        return str(exc)
    if (
        abs(physical_open_width - ANALYTICAL_MAX_INNER_GAP_M)
        > PHYSICAL_OPEN_GAP_TOLERANCE_M
        + _CONSERVATIVE_ENVELOPE_FLOAT_EPSILON_M
    ):
        return (
            'physical open width %.6fm differs from fixed analytical '
            '50 mm contract'
            % physical_open_width
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
        difference = float(actual) - float(expected)
        if difference < -_CONSERVATIVE_ENVELOPE_FLOAT_EPSILON_M:
            return (
                '%s %.9fm is below fixed analytical %.9fm '
                'conservative envelope'
                % (label, actual, expected)
            )
        if difference > tolerance + _CONSERVATIVE_ENVELOPE_FLOAT_EPSILON_M:
            return (
                '%s %.9fm exceeds fixed analytical %.9fm envelope plus '
                'the %.9fm conservative growth tolerance'
                % (label, actual, expected, tolerance)
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
        actual_array = np.asarray(actual, dtype=float)
        expected_array = np.asarray(expected, dtype=float)
        difference = actual_array - expected_array
        if np.any(
            difference < -_CONSERVATIVE_ENVELOPE_FLOAT_EPSILON_M
        ):
            return (
                '%s %s is below fixed analytical %s conservative envelope'
                % (
                    label,
                    actual_array.tolist(),
                    expected_array.tolist(),
                )
            )
        if np.any(
            difference
            > tolerance + _CONSERVATIVE_ENVELOPE_FLOAT_EPSILON_M
        ):
            return (
                '%s %s exceeds fixed analytical %s envelope plus the '
                '%.9fm conservative growth tolerance'
                % (
                    label,
                    actual_array.tolist(),
                    expected_array.tolist(),
                    tolerance,
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
