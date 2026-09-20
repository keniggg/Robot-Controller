"""Frozen-scene, continuous *commanded gripper* path checks, without ROS I/O.

This does not certify whole-arm environmental collision, tracking error or
hand-eye accuracy. MoveIt and measured-endpoint checks remain independent.
"""
from collections import OrderedDict
from copy import deepcopy
from dataclasses import dataclass
import hashlib
import io
import math
from collections.abc import Mapping
import time
import xml.etree.ElementTree as ET

import numpy as np
from tf.transformations import quaternion_matrix

from alicia_flexible_grasp.grasp.gripper_geometry import (
    GripperGeometry, _stage_boxes, _box_corners, _obb_overlap,
)
from alicia_flexible_grasp.grasp.rich_plan_integrity import (
    plan_id_matches_content, pose_values, stamp_nanoseconds,
    validate_plan_header_binding, validate_rich_geometry,
)

# Computational limits only. A six-dimensional uncertainty proof needs more
# subdivisions than the old nominal-curve proof; no geometric or controller
# tolerance changes with this budget. A timeout still fails before submission.
FOLLOWING_PATH_MAX_CHECKS = 32768
FOLLOWING_PATH_MAX_SECONDS = 25.0


class ObservationPathError(ValueError):
    pass


class ObservationVelocityLimitError(ObservationPathError):
    """Only this failure carries a completed, potentially retimable proof."""
    def __init__(self, audit):
        self.audit = deepcopy(audit)
        joint = audit['limiting_joint']
        super().__init__(
            'continuous speed exceeds limit at %s/%s: certified_peak=%.12g '
            'limit=%.12g rad/s required_time_scale=%.12g' % (
                audit['limiting_segment'], joint,
                audit['maximum_certified_speed_bound_rad_s'][joint],
                audit['joint_velocity_limits_rad_s'][joint],
                audit['required_time_scale']))


def wire_digest(message):
    stream = io.BytesIO()
    message.serialize(stream)
    return hashlib.sha256(stream.getvalue()).hexdigest()


def committed_plan_digest(plan):
    """Hash the full committed payload, excluding only its transport sequence.

    ROS Publisher changes the outer Header.seq when an identical plan is
    republished. That counter is not a source-frame, scene or plan revision.
    Keep source nanoseconds, frame, nested geometry headers and every other
    byte strict; do not alter wire_digest used to bind actual trajectories.
    """
    canonical = deepcopy(plan)
    canonical.header.seq = 0
    return wire_digest(canonical)


def _array(values, size, label):
    result = np.array(values, dtype=float, copy=True)
    if result.shape != (size,) or not np.all(np.isfinite(result)):
        raise ObservationPathError('%s must have %d finite values' % (label, size))
    return result


@dataclass(frozen=True)
class FrozenObservationScene:
    plan_id: str
    digest: str
    source_ns: int
    normal: np.ndarray
    offset: float
    center: np.ndarray
    rotation: np.ndarray
    size: np.ndarray
    track_id: str = ''
    geometry_digest: str = ''

    @classmethod
    def from_plan(cls, plan):
        return cls._from_plan_phase(plan, 'FAR_FIELD_OBSERVATION_PLAN')

    @classmethod
    def from_contact_plan(cls, plan):
        """Contact geometry for separately authorized no-contact pregrasp steps."""
        return cls._from_plan_phase(plan, 'CONTACT_EXECUTION_PLAN')

    @classmethod
    def _from_plan_phase(cls, plan, phase):
        if (getattr(plan, 'valid', False) is not True
                or str(plan.diagnostic) != phase
                or len(plan.poses) != 4 or not plan_id_matches_content(plan)):
            raise ObservationPathError('invalid committed %s plan' % phase)
        stamp = validate_plan_header_binding(plan)
        pose, size, support = validate_rich_geometry(plan.object_geometry)
        normal = _array(support[:3], 3, 'support normal')
        # Do not silently replace a measured plane with a different one.
        if abs(float(np.linalg.norm(normal)) - 1.) > 1e-6:
            raise ObservationPathError('support normal is not unit length')
        # Float32 transport can change length by an ULP; scale d with n.
        norm = float(np.linalg.norm(normal))
        normal /= norm
        center = _array(pose[:3], 3, 'OBB center')
        rotation = quaternion_matrix(pose[3:])[:3, :3]
        dimensions = _array(size, 3, 'OBB size')
        for array in (normal, center, rotation, dimensions):
            array.setflags(write=False)
        return cls(str(plan.plan_id), committed_plan_digest(plan), stamp, normal,
                   float(support[3]) / norm, center, rotation, dimensions,
                   str(plan.target_track_id), wire_digest(plan.object_geometry))


class CommittedObservationContexts:
    """Caller synchronizes access. Preview messages must never enter here."""
    def __init__(self, capacity=128):
        self.capacity = int(capacity)
        self.entries = OrderedDict()
        self.revocation = 0
        self.active = False
        self.blocked = False
        self.active_scene = None

    @staticmethod
    def key(target):
        values = pose_values(target.pose)
        if (not all(math.isfinite(v) for v in values)
                or sum(v*v for v in values[3:]) <= 1e-24):
            raise ObservationPathError('non-finite observation request')
        return (stamp_nanoseconds(target.header.stamp),
                str(target.header.frame_id))

    def set_active(self, active):
        active = bool(active)
        if active != self.active:
            self.revocation += 1
            self.active_scene = None
            self.blocked = False
        self.active = active

    def clear(self):
        self.entries.clear()
        self.revocation += 1
        self.active_scene = None
        self.blocked = True

    def ingest(self, plan):
        if getattr(plan, 'valid', False) is not True:
            self.clear()
            return
        if (self.active_scene is not None
                and str(getattr(plan, 'target_track_id', '')) != self.active_scene.track_id):
            self.clear()
            return
        if str(getattr(plan, 'diagnostic', '')) != 'FAR_FIELD_OBSERVATION_PLAN':
            return
        scene = FrozenObservationScene.from_plan(deepcopy(plan))
        # Once a task owns its scene, valid same-track replacements cannot
        # mutate it. The task separately enforces its frozen plan integrity.
        if self.active_scene is not None:
            return
        key = (scene.source_ns, str(plan.header.frame_id))
        if key in self.entries:
            old = self.entries[key]
            if old is None or old.digest != scene.digest:
                self.entries[key] = None  # ambiguous source/scene never chooses latest
            return
        self.entries[key] = scene
        while len(self.entries) > self.capacity:
            self.entries.popitem(last=False)

    def capture(self, target, now_sec, validity_sec):
        if not self.active or self.blocked:
            raise ObservationPathError('no active task owns observation geometry')
        key = self.key(target)
        if not math.isfinite(float(now_sec)) or key[0] <= 0 or float(now_sec) < key[0]*1e-9:
            raise ObservationPathError('observation source timestamp is future or invalid')
        if self.active_scene is not None:
            if key != (self.active_scene.source_ns, 'base_link'):
                raise ObservationPathError('request differs from active frozen source/frame')
            return self.active_scene, self.revocation
        scene = self.entries.get(key)
        if scene is None:
            raise ObservationPathError('no unique committed source/frame/scene binding')
        age = float(now_sec) - scene.source_ns * 1e-9
        if (not math.isfinite(age) or not math.isfinite(float(validity_sec))
                or not 0. <= age <= float(validity_sec)):
            raise ObservationPathError('committed observation source is stale or future at admission')
        self.active_scene = scene
        return scene, self.revocation

    def validate_capture(self, scene, revocation):
        if (not self.active or self.blocked or self.revocation != revocation
                or self.active_scene is not scene):
            raise ObservationPathError('committed observation context revoked')


def _axis_rotation(axis, angle):
    x, y, z = axis
    skew = np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])
    return np.eye(3) + math.sin(angle) * skew + (1. - math.cos(angle)) * skew.dot(skew)


class SerialUrdfFk:
    """Exact origin/axis FK of a fixed/revolute URDF chain, in joint-name order."""
    def __init__(self, xml, joint_names, base='base_link', tool='tool0'):
        self.model_sha256 = hashlib.sha256(str(xml).encode('utf-8')).hexdigest()
        self.names = tuple(str(name) for name in joint_names)
        if not self.names or len(set(self.names)) != len(self.names):
            raise ObservationPathError('FK requires distinct joint names')
        robot = ET.fromstring(xml)
        by_child = {}
        for joint in robot.findall('joint'):
            child = joint.find('child').get('link')
            if child in by_child:
                raise ObservationPathError('URDF has multiple parents')
            by_child[child] = joint
        chain, visited, child = [], set(), tool
        while child != base:
            if child in visited or child not in by_child:
                raise ObservationPathError('URDF base/tool chain is missing or cyclic')
            visited.add(child)
            joint = by_child[child]
            chain.append(joint)
            child = joint.find('parent').get('link')
        chain.reverse()
        self.chain, moving = [], []
        self.limits = np.array([[-np.inf, np.inf]] * len(self.names))
        for joint in chain:
            kind = joint.get('type')
            if kind not in ('fixed', 'revolute'):
                raise ObservationPathError('unsupported FK/wrap joint type: ' + str(kind))
            origin = joint.find('origin')
            xyz = _array([float(x) for x in (origin.get('xyz', '0 0 0') if origin is not None else '0 0 0').split()], 3, 'URDF xyz')
            rpy = _array([float(x) for x in (origin.get('rpy', '0 0 0') if origin is not None else '0 0 0').split()], 3, 'URDF rpy')
            transform = np.eye(4)
            transform[:3, :3] = (_axis_rotation([0., 0., 1.], rpy[2])
                                     @ _axis_rotation([0., 1., 0.], rpy[1])
                                     @ _axis_rotation([1., 0., 0.], rpy[0]))
            transform[:3, 3] = xyz
            index, axis = None, None
            if kind == 'revolute':
                name = joint.get('name')
                if name not in self.names:
                    raise ObservationPathError('URDF chain joint absent from trajectory: ' + name)
                index = self.names.index(name)
                axis_element = joint.find('axis')
                axis = _array([float(v) for v in (axis_element.get('xyz', '1 0 0') if axis_element is not None else '1 0 0').split()], 3, 'URDF axis')
                norm = float(np.linalg.norm(axis))
                if norm <= 1e-12:
                    raise ObservationPathError('zero URDF joint axis')
                axis /= norm
                limit = joint.find('limit')
                if limit is not None:
                    values = _array([float(limit.get('lower')), float(limit.get('upper'))],
                                    2, 'URDF joint limits')
                    if values[0] > values[1]:
                        raise ObservationPathError('reversed URDF joint limits')
                    self.limits[index] = values
                moving.append(name)
            self.chain.append((transform, index, axis))
        if set(moving) != set(self.names) or len(moving) != len(self.names):
            raise ObservationPathError('trajectory names do not exactly match the tool chain')

    def __call__(self, positions):
        q = _array(positions, len(self.names), 'FK joints')
        if np.any(q < self.limits[:, 0]) or np.any(q > self.limits[:, 1]):
            raise ObservationPathError('commanded spline exceeds unchanged URDF joint limits')
        transform = np.eye(4)
        for origin, index, axis in self.chain:
            transform = transform @ origin
            if index is not None:
                transform[:3, :3] = transform[:3, :3] @ _axis_rotation(axis, q[index])
        return transform

    def point_motion_radii(self, tool_radius):
        # For every revolute axis, triangle inequality gives a global bound
        # on distance to every gripper point, valid at every joint posture.
        result = np.zeros(len(self.names))
        downstream = float(tool_radius)
        for origin, index, _axis in reversed(self.chain):
            if index is not None:
                result[index] = downstream
            downstream += float(np.linalg.norm(origin[:3, 3]))
        return result

    def corner_motion_radii(self, local_corner):
        """Same global triangle bound, with the fixed tool suffix exact.

        Fixed transforms need not be treated as independently rotating links.
        Compose them on the actual corner before the last moving joint; from
        that joint upstream, retain the original global triangle inequality.
        """
        point = _array(local_corner, 3, 'local CAD corner')
        result, radius = np.zeros(len(self.names)), None
        for origin, index, _axis in reversed(self.chain):
            if radius is None and index is None:
                point = origin[:3, :3] @ point + origin[:3, 3]
                continue
            if radius is None:
                radius = float(np.linalg.norm(point))
            if index is not None:
                result[index] = radius
            radius += float(np.linalg.norm(origin[:3, 3]))
        return result


def observation_endpoint_support_bound(fk, goal, errors, normal, offset, gripper, opening):
    """Bound support clearance over an endpoint joint-error box, without I/O.

    Conditional on |actual_q-goal_q| <= errors; this is NOT a certificate for
    transient tracking, whole-arm collision, calibration or table estimation.
    For each CAD corner use its exact support-height Jacobian and Taylor's
    remainder. Serial revolute joints obey |d_i d_j p| <= min(R_i,R_j),
    where R_i is the global point-motion radius used by the path validator.
    Thus the second-order remainder is bounded on the entire error box, not
    just sampled corners. The physical 3mm gate itself is never modified.
    """
    if not isinstance(fk, SerialUrdfFk) or not isinstance(gripper, GripperGeometry):
        raise ObservationPathError('endpoint bound requires the exact URDF/CAD model')
    q = _array(goal, len(fk.names), 'endpoint goal')
    epsilon = _array(errors, len(fk.names), 'endpoint error bounds')
    n = _array(normal, 3, 'endpoint support normal')
    d = float(offset)
    if (np.any(epsilon < 0.) or np.any(epsilon > math.pi)
            or not math.isfinite(d) or abs(np.linalg.norm(n)-1.) > 1e-6
            or not math.isfinite(opening) or not 0. <= opening <= gripper.max_inner_gap_m
            or gripper.support_clearance_m < .003):
        raise ObservationPathError('invalid endpoint error box or physical support contract')
    length = float(np.linalg.norm(n))
    n, d = n/length, d/length
    transform = fk(q)  # Includes unchanged URDF joint-limit validation.
    chain_pose = np.eye(4)
    origins, axes = np.zeros((len(q), 3)), np.zeros((len(q), 3))
    for origin, index, axis in fk.chain:
        chain_pose = chain_pose @ origin
        if index is not None:
            origins[index] = chain_pose[:3, 3]
            axes[index] = chain_pose[:3, :3] @ axis
            chain_pose[:3, :3] = chain_pose[:3, :3] @ _axis_rotation(axis, q[index])
    identity = np.eye(4)
    local = np.concatenate([
        _box_corners(center, identity[:3, :3], size)
        for _, center, size in _stage_boxes(identity, gripper, opening, 'y', 'z')
    ])
    points = local @ transform[:3, :3].T + transform[:3, 3]
    gradients = np.cross(axes[None, :, :], points[:, None, :]-origins[None, :, :]) @ n
    radii = fk.point_motion_radii(float(np.max(np.linalg.norm(local, axis=1))))
    remainder = .5 * float(epsilon @ np.minimum.outer(radii, radii) @ epsilon)
    nominal = points @ n + d
    linear = np.abs(gradients) @ epsilon
    lower = nominal-linear-remainder-1e-12
    return {
        'kind': 'conditional_endpoint_joint_error_box_support_bound',
        'robot_description_sha256': fk.model_sha256,
        'joint_names': list(fk.names),
        'goal_joint_positions_rad': q.tolist(),
        'joint_error_bounds_rad': epsilon.tolist(),
        'nominal_minimum_support_clearance_m': float(np.min(nominal)),
        'maximum_first_order_allowance_m': float(np.max(linear)),
        'second_order_allowance_m': remainder,
        'minimum_support_clearance_lower_bound_m': float(np.min(lower)),
        'required_support_clearance_m': float(gripper.support_clearance_m),
        'ok': bool(np.min(lower) >= gripper.support_clearance_m),
        'certifies_transient_tracking_or_calibration': False,
    }


def observation_tracking_error_bounds(constraints, names):
    """Planning allowance, NOT a hardware tracking/stopping guarantee.

    Use the controller's path band, never its much smaller endpoint band.
    Keep the existing 0.12 rad floor and half an SDK encoder count. A missing
    or disabled controller constraint cannot silently become zero uncertainty.
    """
    if not isinstance(constraints, Mapping):
        raise ObservationPathError('observation controller constraints unavailable')
    result = []
    for name in names:
        row = constraints.get(name)
        value = row.get('trajectory') if isinstance(row, Mapping) else None
        if (isinstance(value, bool) or not isinstance(value, (int, float))
                or not math.isfinite(value) or not 0. < value <= .5):
            raise ObservationPathError('invalid observation path band for ' + name)
        result.append(max(.12, float(value)) + math.pi/4096.)
    return np.array(result)


class ObservationSupportBox:
    """Conservative plane-height bounds for all CAD corners in a joint box.

    Subdivision tightens a Taylor enclosure, not a grid-sampling assertion.
    FK at enclosure centres intentionally permits angles beyond the URDF
    limits: this only enlarges the uncertainty set, never authorizes those
    commands. The nominal commanded spline still obeys the original limits.
    Target OBB, whole-arm geometry and calibration are separate checks.
    """
    def __init__(self, fk, gripper, opening, normal, offset):
        if not isinstance(fk, SerialUrdfFk) or not isinstance(gripper, GripperGeometry):
            raise ObservationPathError('support box requires exact FK and CAD')
        self.fk, self.gripper = fk, gripper
        n = _array(normal, 3, 'support box normal')
        length = float(np.linalg.norm(n))
        if (abs(length-1.) > 1e-6 or not math.isfinite(float(offset))
                or not math.isfinite(opening) or not 0. <= opening <= gripper.max_inner_gap_m
                or not math.isfinite(gripper.support_clearance_m)
                or gripper.support_clearance_m < .003):
            raise ObservationPathError('invalid support box physical contract')
        self.normal, self.offset = n/length, float(offset)/length
        identity = np.eye(4)
        self.local = np.concatenate([
            _box_corners(center, identity[:3, :3], size)
            for _, center, size in _stage_boxes(identity, gripper, opening, 'y', 'z')])
        # A palm corner's radius must not inflate every finger corner's
        # Taylor remainder. Each corner has its own global radius bound.
        radii = np.array([fk.corner_motion_radii(p) for p in self.local])
        self.hessian = np.minimum(radii[:, :, None], radii[:, None, :])

    def enclosure(self, center, half_width):
        pose = np.eye(4)
        origins = np.zeros((len(self.fk.names), 3))
        axes = np.zeros_like(origins)
        for origin, index, axis in self.fk.chain:
            pose = pose @ origin
            if index is not None:
                origins[index] = pose[:3, 3]
                axes[index] = pose[:3, :3] @ axis
                pose[:3, :3] = pose[:3, :3] @ _axis_rotation(axis, center[index])
        points = self.local @ pose[:3, :3].T + pose[:3, 3]
        gradients = np.cross(axes[None, :, :], points[:, None, :]-origins[None, :, :]) @ self.normal
        nominal = points @ self.normal + self.offset
        remainder = .5 * np.einsum('i,cij,j->c', half_width, self.hessian, half_width)
        lower = nominal - np.abs(gradients) @ half_width - remainder - 1e-12
        # Split axes contributing most to the current worst corner's bound.
        scores = half_width * (np.abs(gradients[int(np.argmin(lower))])
                               + self.hessian[int(np.argmin(lower))] @ half_width)
        return (float(np.min(lower)), float(np.min(nominal)),
                np.copysign(scores, gradients[int(np.argmin(lower))]))

    def check(self, lower, upper, *, max_checks=8192, max_seconds=2.,
              clock=time.monotonic):
        lo = _array(lower, len(self.fk.names), 'support box lower')
        hi = _array(upper, len(self.fk.names), 'support box upper')
        if (np.any(lo > hi) or np.any(hi-lo > 2.*math.pi)
                or not isinstance(max_checks, int) or max_checks < 1
                or not math.isfinite(max_seconds) or max_seconds <= 0.):
            raise ObservationPathError('invalid support box or computation budget')
        started, checked, minimum = clock(), 0, float('inf')
        stack = [(.5*(lo+hi), .5*(hi-lo))]
        while stack:
            if checked >= max_checks or clock()-started > max_seconds:
                raise ObservationPathError('following support box proof budget exhausted')
            center, width = stack.pop()
            bound, nominal, scores = self.enclosure(center, width)
            checked += 1
            if nominal < self.gripper.support_clearance_m:
                return {'ok': False, 'reason': 'joint error box contains insufficient support clearance',
                        'counterexample_joint_positions_rad': center.tolist(),
                        'counterexample_model_clearance_m': nominal,
                        'box_checks': checked, 'elapsed_sec': clock()-started}
            if bound >= self.gripper.support_clearance_m:
                minimum = min(minimum, bound)
                continue
            if checked == 1:
                # Try one exact point at the box corner suggested by the
                # worst CAD corner's gradient. A real counterexample can
                # reject immediately instead of subdividing along the
                # clearance boundary for seconds. A passing point never
                # certifies the box: retain the original enclosure proof.
                if checked >= max_checks or clock()-started > max_seconds:
                    raise ObservationPathError('following support box proof budget exhausted')
                witness = np.clip(center - np.sign(scores)*width, lo, hi)
                _, witness_clearance, _ = self.enclosure(witness, np.zeros_like(width))
                checked += 1
                if witness_clearance < self.gripper.support_clearance_m:
                    return {'ok': False, 'reason': 'joint error box contains insufficient support clearance',
                            'counterexample_joint_positions_rad': witness.tolist(),
                            'counterexample_model_clearance_m': witness_clearance,
                            'box_checks': checked, 'elapsed_sec': clock()-started}
            axis = int(np.argmax(np.abs(scores)))
            if width[axis] < 1e-8:
                raise ObservationPathError('following support box unresolved at precision limit')
            child_width = width.copy()
            child_width[axis] *= .5
            shift = np.zeros_like(width)
            shift[axis] = child_width[axis]
            # Visit the lower-clearance side first. Otherwise depth-first
            # search can linger on an uncertainty-box boundary instead of
            # finding the adjacent counterexample within its finite budget.
            if scores[axis] < 0.:
                shift = -shift
            stack.extend(((center+shift, child_width), (center-shift, child_width)))
        return {'ok': True, 'minimum_support_clearance_lower_bound_m': minimum,
                'box_checks': checked, 'elapsed_sec': clock()-started}


def observation_following_support_bound(fk, goal, errors, normal, offset, gripper, opening,
                                       **kwargs):
    q = _array(goal, len(fk.names), 'observation goal')
    epsilon = _array(errors, len(fk.names), 'observation following allowance')
    if np.any(epsilon < 0.) or np.any(epsilon > math.pi):
        raise ObservationPathError('invalid observation following allowance')
    fk(q)  # Only the nominal command is an authorized joint-limit target.
    checker = ObservationSupportBox(fk, gripper, opening, normal, offset)
    result = checker.check(q-epsilon, q+epsilon, **kwargs)
    result.update({
        'scope': 'conditional_joint_error_box_Link6_Link7_Link8_support_only',
        'joint_names': list(fk.names), 'joint_error_bounds_rad': epsilon.tolist(),
        'robot_description_sha256': fk.model_sha256,
        'goal_joint_positions_rad': q.tolist(),
        'nominal_minimum_support_clearance_m': checker.enclosure(q, np.zeros_like(q))[1],
        'required_support_clearance_m': gripper.support_clearance_m,
        'certifies_hardware_tracking_stopping_or_calibration': False,
    })
    return result


def segment_coefficients(first, second, duration, dimensions):
    """Noetic QuinticSplineSegment coefficients in normalized time s in [0,1]."""
    if not math.isfinite(duration) or duration <= 0.:
        raise ObservationPathError('nonpositive segment duration')
    p0 = _array(first.positions, dimensions, 'segment start')
    p1 = _array(second.positions, dimensions, 'segment end')
    c = np.zeros((dimensions, 6))
    c[:, 0], c[:, 1] = p0, p1 - p0
    v0, v1 = getattr(first, 'velocities', ()), getattr(second, 'velocities', ())
    a0, a1 = getattr(first, 'accelerations', ()), getattr(second, 'accelerations', ())
    for values in (v0, v1, a0, a1):
        if len(values):
            _array(values, dimensions, 'segment derivatives')
    if not len(v0) or not len(v1):
        return c
    v0, v1 = np.asarray(v0) * duration, np.asarray(v1) * duration
    c[:, 1] = v0
    c[:, 2] = -3*p0 + 3*p1 - 2*v0 - v1
    c[:, 3] = 2*p0 - 2*p1 + v0 + v1
    if not len(a0) or not len(a1):
        return c
    a0, a1 = np.asarray(a0)*duration**2, np.asarray(a1)*duration**2
    c[:, 2] = .5*a0
    c[:, 3] = (-20*p0+20*p1-3*a0+a1-12*v0-8*v1)/2
    c[:, 4] = (30*p0-30*p1+3*a0-2*a1+16*v0+14*v1)/2
    c[:, 5] = (-12*p0+12*p1-a0+a1-6*v0-6*v1)/2
    if not np.all(np.isfinite(c)):
        raise ObservationPathError('nonfinite spline coefficients')
    return c


def _polynomial(c, value):
    return c @ np.power(float(value), np.arange(c.shape[1]))


def _polynomial_bounds(power, lo, hi):
    """Bernstein convex-hull enclosure, avoiding fragile polynomial root tests."""
    degree = power.shape[1] - 1
    shifted = np.zeros_like(power)
    for k in range(degree+1):
        for i in range(k, degree+1):
            shifted[:, k] += power[:, i] * math.comb(i, k) * lo**(i-k) * (hi-lo)**k
    bernstein = np.zeros_like(power)
    for k in range(degree+1):
        for j in range(k+1):
            bernstein[:, k] += shifted[:, j] * math.comb(k, j) / math.comb(degree, j)
    rounding = (128.*np.finfo(float).eps
                * np.maximum(1., np.sum(np.abs(power), axis=1)))
    return np.min(bernstein, axis=1)-rounding, np.max(bernstein, axis=1)+rounding


def _derivative_bound(c, lo, hi):
    lower, upper = _polynomial_bounds(c[:, 1:] * np.arange(1, c.shape[1]), lo, hi)
    return np.maximum(np.abs(lower), np.abs(upper))


def analyze_observation_trajectory_velocity(
        plan, controller_desired, joint_velocity_limits_rad_s, *,
        max_checks=8192, max_depth=20, max_seconds=2., clock=time.monotonic):
    """Bound every commanded spline's speed, including the real Noetic bridge.

    A returned scale is timing advice, never execution permission. With the
    required stationary desired baseline, uniformly scaling all times by S,
    waypoint velocities by 1/S and accelerations by 1/S**2 preserves every
    normalized polynomial, including the bridge. Callers must revalidate the
    final serialized trajectory's speed, CAD and unchanged baseline afterward.
    """
    started = clock()
    if (not math.isfinite(float(max_seconds)) or max_seconds <= 0.
            or not isinstance(max_checks, int) or max_checks < 0
            or not isinstance(max_depth, int) or not 0 <= max_depth <= 30):
        raise ObservationPathError('invalid continuous velocity validation budget')
    trajectory = plan.joint_trajectory
    names = tuple(trajectory.joint_names)
    if not names or len(set(names)) != len(names):
        raise ObservationPathError('velocity proof requires distinct joint names')
    if (not isinstance(joint_velocity_limits_rad_s, dict)
            or set(joint_velocity_limits_rad_s) != set(names)):
        raise ObservationPathError('velocity limits must exactly cover trajectory joint names')
    try:
        if any(isinstance(joint_velocity_limits_rad_s[n], (bool, np.bool_)) for n in names):
            raise ValueError('boolean limit')
        limits = _array([joint_velocity_limits_rad_s[n] for n in names], len(names), 'velocity limits')
    except (TypeError, ValueError, OverflowError) as exc:
        raise ObservationPathError('invalid joint velocity limits: %s' % exc)
    if np.any(limits <= 0.):
        raise ObservationPathError('joint velocity limits must be positive')
    if stamp_nanoseconds(trajectory.header.stamp) != 0:
        raise ObservationPathError('nonzero trajectory start stamp is unsupported')
    if getattr(getattr(plan, 'multi_dof_joint_trajectory', None), 'points', ()):
        raise ObservationPathError('multi-DOF observation path is unsupported')
    points = list(trajectory.points)
    if not 2 <= len(points) <= 4096:
        raise ObservationPathError('observation trajectory must contain 2..4096 waypoints')
    times = [float(p.time_from_start.to_sec()) for p in points]
    if (not all(math.isfinite(t) and t >= 0. for t in times)
            or any(b <= a for a, b in zip(times, times[1:]))):
        raise ObservationPathError('observation trajectory timing is invalid')
    baseline = _array(controller_desired.positions, len(names), 'controller desired')
    for field in ('velocities', 'accelerations'):
        values = _array(getattr(controller_desired, field, ()), len(names), 'controller desired '+field)
        if np.any(values != 0.):
            raise ObservationPathError('controller desired state is not a stationary hold')
    first = next((i for i, t in enumerate(times) if t > 0.), None)
    if first is None:
        raise ObservationPathError('no positive-time controller waypoint')
    segments = [(segment_coefficients(controller_desired, points[first], times[first], len(names)),
                 'bridge', times[first])]
    if first:
        # Validate the dropped point's fields, but do not substitute its
        # nonexecuted polynomial for the controller's desired-state bridge.
        segment_coefficients(points[0], points[first], times[first], len(names))
    for index in range(first, len(points)-1):
        duration = times[index+1]-times[index]
        segments.append((segment_coefficients(points[index], points[index+1], duration, len(names)),
                         'segment %d' % index, duration))
    maximum = np.zeros(len(names))
    sampled = np.zeros(len(names))
    limiting_segments = ['bridge'] * len(names)
    checked, rows = 0, []
    for coefficients, label, duration in segments:
        if not np.all(np.isfinite(coefficients)):
            raise ObservationPathError('nonfinite velocity spline coefficients')
        derivative = coefficients[:, 1:] * np.arange(1, coefficients.shape[1])
        local_sampled = np.max(np.abs(np.array([
            _polynomial(derivative, s) / duration for s in (0., .5, 1.)])), axis=0)
        local_upper = np.zeros(len(names))
        stack = [(0., 1., 0)]
        while stack:
            if checked >= max_checks or clock()-started > max_seconds:
                raise ObservationPathError('continuous velocity validation computation budget exhausted')
            lo, hi, depth = stack.pop()
            mid = .5*(lo+hi)
            upper = _derivative_bound(coefficients, lo, hi) / duration
            local_sampled = np.maximum(local_sampled, np.abs(_polynomial(derivative, mid) / duration))
            if not np.all(np.isfinite(upper)) or not np.all(np.isfinite(local_sampled)):
                raise ObservationPathError('nonfinite continuous velocity bound')
            checked += 1
            # This numerical accuracy only limits retiming overestimation;
            # it never enlarges the speed limit used for final acceptance.
            accuracy = 1e-6 * np.maximum(limits, local_sampled)
            if np.all(upper <= np.maximum(limits, local_sampled + accuracy)):
                local_upper = np.maximum(local_upper, upper)
                continue
            if depth >= max_depth:
                raise ObservationPathError('continuous velocity bound is unresolved at subdivision limit')
            stack.extend(((mid, hi, depth+1), (lo, mid, depth+1)))
        for index in range(len(names)):
            if local_upper[index] > maximum[index]:
                limiting_segments[index] = label
        maximum = np.maximum(maximum, local_upper)
        sampled = np.maximum(sampled, local_sampled)
        rows.append({'segment': label, 'duration_sec': duration,
                     'certified_speed_bound_rad_s': dict(zip(names, local_upper.tolist()))})
    ratios = maximum / limits
    if not np.all(np.isfinite(ratios)):
        raise ObservationPathError('nonfinite required velocity time scale')
    limiting_index = int(np.argmax(ratios))
    return {
        'scope': 'continuous_commanded_joint_velocity_including_noetic_bridge',
        'trajectory_sha256': wire_digest(plan),
        'joint_velocity_limits_rad_s': dict(zip(names, limits.tolist())),
        'maximum_certified_speed_bound_rad_s': dict(zip(names, maximum.tolist())),
        'maximum_sampled_speed_rad_s': dict(zip(names, sampled.tolist())),
        'required_time_scale': max(1., float(ratios[limiting_index])),
        'limiting_joint': names[limiting_index],
        'limiting_segment': limiting_segments[limiting_index],
        'bridge_start_joint_positions_rad': baseline.tolist(),
        'segments': len(segments), 'segment_bounds': rows,
        'interval_checks': checked, 'elapsed_sec': clock()-started,
    }


def validate_observation_trajectory_velocity(
        plan, controller_desired, joint_velocity_limits_rad_s, **kwargs):
    audit = analyze_observation_trajectory_velocity(
        plan, controller_desired, joint_velocity_limits_rad_s, **kwargs)
    if audit['required_time_scale'] > 1.:
        raise ObservationVelocityLimitError(audit)
    return audit


def validate_observation_trajectory(plan, scene, fk, gripper, opening,
                                    controller_desired, max_checks=8192,
                                    max_depth=20, max_seconds=5., clock=time.monotonic,
                                    joint_error_bounds_rad=None):
    """Certify continuous nominal Link6/7/8 clearance, including Noetic bridge.

    A zero-stamped command starts at controller reception time. Its t=0 point
    is skipped and the current desired hold is bridged to the first t>0 point.
    Require a stationary desired state so network latency cannot change it.
    """
    started = clock()
    if (not math.isfinite(float(max_seconds)) or max_seconds <= 0.
            or not isinstance(max_checks, int) or max_checks < 0
            or not isinstance(max_depth, int) or not 0 <= max_depth <= 30):
        raise ObservationPathError('invalid continuous validation budget')
    if not isinstance(gripper, GripperGeometry) or gripper.support_clearance_m < .003:
        raise ObservationPathError('3 mm physical gripper clearance is mandatory')
    if not math.isfinite(opening) or not 0. <= opening <= gripper.max_inner_gap_m:
        raise ObservationPathError('invalid measured opening')
    trajectory = plan.joint_trajectory
    names = tuple(trajectory.joint_names)
    if names != fk.names or len(set(names)) != len(names):
        raise ObservationPathError('trajectory/FK joint order mismatch')
    if stamp_nanoseconds(trajectory.header.stamp) != 0:
        raise ObservationPathError('nonzero trajectory start stamp is unsupported')
    if getattr(getattr(plan, 'multi_dof_joint_trajectory', None), 'points', ()):
        raise ObservationPathError('multi-DOF observation path is unsupported')
    points = list(trajectory.points)
    if not 2 <= len(points) <= 4096:
        raise ObservationPathError('observation trajectory must contain 2..4096 waypoints')
    times = [float(p.time_from_start.to_sec()) for p in points]
    if (not all(math.isfinite(t) and t >= 0. for t in times)
            or any(b <= a for a, b in zip(times, times[1:]))):
        raise ObservationPathError('observation trajectory timing is invalid')
    _array(controller_desired.positions, len(names), 'controller desired')
    for field in ('velocities', 'accelerations'):
        values = _array(getattr(controller_desired, field, ()), len(names), 'controller desired '+field)
        if np.any(values != 0.):
            raise ObservationPathError('controller desired state is not a stationary hold')
    first = next((i for i, t in enumerate(times) if t > 0.), None)
    if first is None:
        raise ObservationPathError('no positive-time controller waypoint')
    segments = [(segment_coefficients(controller_desired, points[first], times[first], len(names)), 'bridge')]
    # Also check the requested t=0 state, even though Noetic drops that point.
    if first:
        segment_coefficients(points[0], points[first], times[first], len(names))
    for index in range(first, len(points)-1):
        segments.append((segment_coefficients(points[index], points[index+1],
                                              times[index+1]-times[index], len(names)), str(index)))
    identity = np.eye(4)
    radius = max(float(np.max(np.linalg.norm(_box_corners(center, identity[:3, :3], size), axis=1)))
                 for _name, center, size in _stage_boxes(identity, gripper, opening, 'y', 'z'))
    radii = fk.point_motion_radii(radius)
    checked, fk_checked, minimum, certified = 0, 0, float('inf'), float('inf')
    following_checks, following_minimum = 0, float('inf')
    epsilon, support_box = None, None
    if joint_error_bounds_rad is not None:
        epsilon = _array(joint_error_bounds_rad, len(names), 'path following allowance')
        if np.any(epsilon <= 0.) or np.any(epsilon > math.pi):
            raise ObservationPathError('invalid path following allowance')
        support_box = ObservationSupportBox(fk, gripper, opening, scene.normal, scene.offset)

    def following_box(lower, upper, endpoint=False, location='interval'):
        nonlocal following_checks, following_minimum
        if support_box is None:
            return True
        report = support_box.check(lower-epsilon, upper+epsilon,
            max_checks=max_checks-following_checks,
            max_seconds=max_seconds-(clock()-started), clock=clock)
        following_checks += report['box_checks']
        if not report['ok']:
            if endpoint:
                report['failure_location'] = location
                report['reference_joint_positions_rad'] = lower.tolist()
                report['joint_error_bounds_rad'] = epsilon.tolist()
                raise ObservationPathError('OBSERVATION_FOLLOWING_SUPPORT_INVALID: '
                                           + str(report))
            return False
        following_minimum = min(following_minimum,
                               report['minimum_support_clearance_lower_bound_m'])
        return True

    def boxes_at(q, displacement, label):
        nonlocal minimum, fk_checked
        if fk_checked >= max_checks or clock()-started > max_seconds:
            raise ObservationPathError('continuous path validation computation budget exhausted')
        fk_checked += 1
        transform = fk(q)
        ok, local_min = True, float('inf')
        for name, center, size in _stage_boxes(transform, gripper, opening, 'y', 'z'):
            clearance = float(np.min(_box_corners(center, transform[:3, :3], size) @ scene.normal + scene.offset))
            minimum, local_min = min(minimum, clearance), min(local_min, clearance)
            if clearance < gripper.support_clearance_m - 1e-9:
                raise ObservationPathError('support clearance %.9fm at %s/%s' % (clearance, label, name))
            if _obb_overlap(center, transform[:3, :3], size, scene.center, scene.rotation, scene.size):
                raise ObservationPathError('target OBB collision at %s/%s' % (label, name))
            if (clearance - displacement < gripper.support_clearance_m
                    or _obb_overlap(center, transform[:3, :3], size + 2.*displacement,
                                    scene.center, scene.rotation, scene.size)):
                ok = False
        return ok, local_min - displacement

    if first:
        dropped = _array(points[0].positions, len(names), 'dropped start')
        boxes_at(dropped, 0., 'dropped-start')
    # Reject an inadequate selected endpoint before spending the proof budget
    # on its preceding path. All intermediate states are covered below by
    # interval proofs; do not repeat expensive uncertainty proofs at each knot.
    terminal = _array(points[-1].positions, len(names), 'terminal')
    following_box(terminal, terminal, endpoint=True, location='terminal')
    initial = _array(controller_desired.positions, len(names), 'controller hold')
    following_box(initial, initial, endpoint=True, location='controller_hold')
    for coefficients, label in segments:
        for t, suffix in ((0., '-start'), (1., '-end')):
            q = _polynomial(coefficients, t)
            boxes_at(q, 0., label+suffix)
        stack = [(0., 1., 0)]
        while stack:
            if checked >= max_checks or clock()-started > max_seconds:
                raise ObservationPathError('continuous path validation computation budget exhausted')
            lo, hi, depth = stack.pop()
            mid = .5*(lo+hi)
            displacement = .5*(hi-lo)*float(radii @ _derivative_bound(coefficients, lo, hi)) + 1e-12
            passed, bound = boxes_at(_polynomial(coefficients, mid), displacement, label)
            lower, upper = _polynomial_bounds(coefficients, lo, hi)
            passed = passed and bool(np.all(lower >= fk.limits[:, 0])
                                      and np.all(upper <= fk.limits[:, 1]))
            if passed:
                passed = following_box(lower, upper)
            checked += 1
            if passed:
                certified = min(certified, bound)
                continue
            if depth >= max_depth:
                raise ObservationPathError('continuous path clearance is unresolved at subdivision limit')
            stack.extend(((mid, hi, depth+1), (lo, mid, depth+1)))
    return {'scope': 'continuous_commanded_Link6_Link7_Link8_only',
            'plan_id': scene.plan_id, 'scene_sha256': scene.digest,
            'model_sha256': fk.model_sha256, 'trajectory_sha256': wire_digest(plan),
            'segments': len(segments), 'interval_checks': checked,
            'fk_checks': fk_checked,
            'minimum_sampled_clearance_m': minimum,
            'certified_minimum_clearance_m': certified,
            'bridge': 'stationary_desired_to_first_positive_time_waypoint',
            'bridge_start_joint_positions_rad': list(controller_desired.positions),
            'opening_width_m': float(opening),
            'following_support': ({
                'scope': 'conditional_continuous_Link6_Link7_Link8_support_only',
                'joint_error_bounds_rad': epsilon.tolist(),
                'minimum_support_clearance_lower_bound_m': following_minimum,
                'box_checks': following_checks,
                'includes_controller_bridge': True,
                'certifies_hardware_tracking_stopping_or_calibration': False,
                'target_obb_tracking_uncertainty_checked': False,
            } if support_box is not None else None),
            'elapsed_sec': clock()-started}
