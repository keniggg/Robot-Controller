"""Offline lift derivation and conditional dual-occupancy path certificates.

No publishers, services, scene mutation, attachment or execution authorization.
The two object poses are mutually exclusive hypotheses, not a grasp result.
A constant measured actuator bias is a counterfactual response model only.
"""
from copy import deepcopy
from dataclasses import dataclass
from itertools import combinations
import math
import time

import numpy as np

from alicia_flexible_grasp.grasp.contact_evidence import decode_contact_evidence
from alicia_flexible_grasp.grasp.gripper_geometry import (
    GripperGeometry, _stage_boxes, _box_corners, _obb_overlap,
    ANALYTICAL_FINGER_BOX_PADDING_XYZ_M, finger_contact_patch_margin_m,
    ANALYTICAL_FINGER_PAIR_CENTER_TOOL_XYZ_M,
    ANALYTICAL_FINGER_CONTACT_PATCH_TOOL_XZ_M, _signed_polygon_margin,
)
from .contact_linear_compensation import compensate_cartesian_path
from .observation_path_guard import (
    FrozenObservationScene, ObservationPathError, _array, _derivative_bound,
    _polynomial, _polynomial_bounds, segment_coefficients, wire_digest,
)
from .stationary_following import ARM_NAMES


class LiftPathError(ObservationPathError):
    pass


def _readonly(value, shape, label):
    a = np.array(value, dtype=float, copy=True)
    if a.shape != shape or not np.isfinite(a).all():
        raise LiftPathError('invalid '+label)
    a.setflags(write=False)
    return a


def _transform(value, label):
    a = _readonly(value, (4, 4), label)
    if (not np.allclose(a[3], [0, 0, 0, 1], atol=1e-10, rtol=0)
            or not np.allclose(a[:3,:3].T@a[:3,:3], np.eye(3), atol=1e-8, rtol=0)
            or np.linalg.det(a[:3,:3]) < 0.):
        raise LiftPathError('invalid rigid '+label)
    return a


@dataclass(frozen=True)
class ToolBodyBox:
    """Caller supplies a measured/CAD body in tool0, with exact provenance.

    A camera extrinsic alone is not a camera collision body. The caller must
    resolve the mesh/body origin to the runtime tool0 frame before admission.
    """
    name: str
    T_tool_box: np.ndarray
    size: np.ndarray
    source_sha256: str

    def __post_init__(self):
        if (self.name not in ('camera', 'mount') or len(self.source_sha256) != 64
                or any(c not in '0123456789abcdef' for c in self.source_sha256)):
            raise LiftPathError('extra tool body needs identity and source SHA256')
        object.__setattr__(self, 'T_tool_box', _transform(self.T_tool_box, 'body pose'))
        size = _readonly(self.size, (3,), 'body size')
        if np.any(size <= 0):
            raise LiftPathError('nonpositive body size')
        object.__setattr__(self, 'size', size)


@dataclass(frozen=True)
class FrozenLiftContext:
    scene: FrozenObservationScene
    model_sha256: str
    measured: np.ndarray
    sdk: np.ndarray
    opening: float
    epoch_ns: int
    accepted_stamp_ns: int
    sdk_stamp_ns: int
    T_tool_object: np.ndarray
    contact_evidence: str

    @classmethod
    def from_plan(cls, plan, fk, sample, opening):
        scene = FrozenObservationScene.from_contact_plan(plan)
        decode_contact_evidence(plan, required=True)
        if (sample.get('model_sha256') != fk.model_sha256
                or tuple(sample.get('joint_names', ())) != ARM_NAMES
                or tuple(fk.names) != ARM_NAMES):
            raise LiftPathError('lift model/joint binding mismatch')
        epoch = sample.get('epoch_ns')
        stamps = [sample.get(k) for k in ('accepted_stamp_ns', 'sdk_stamp_ns')]
        if (type(epoch) is not int or epoch <= 0
                or any(type(s) is not int or s < epoch for s in stamps)):
            raise LiftPathError('lift feedback epoch/stamp missing or mismatched')
        if (not math.isfinite(float(opening)) or not 0 <= opening <= .05
                or sample.get('accepted_opening_m') != opening):
            raise LiftPathError('opening must equal frozen accepted feedback')
        measured = _readonly(sample['accepted_positions_rad'], (6,), 'accepted joints')
        sdk = _readonly(sample['sdk_positions_rad'], (6,), 'SDK joints')
        if np.max(abs(sdk-measured)) > .035:
            raise LiftPathError('lift following bias outside existing 0.035 rad contract')
        obj = np.eye(4); obj[:3,:3] = scene.rotation; obj[:3,3] = scene.center
        local = _transform(np.linalg.inv(fk(measured))@obj, 'hypothetical object pose')
        return cls(scene, fk.model_sha256, measured, sdk, float(opening), epoch,
                   stamps[0], stamps[1], local, str(plan.contact_geometry_evidence))

    def assert_binding(self, plan, fk, sample, opening):
        other = type(self).from_plan(plan, fk, sample, opening)
        if (other.scene.digest != self.scene.digest
                or other.model_sha256 != self.model_sha256
                or other.epoch_ns != self.epoch_ns
                or other.accepted_stamp_ns != self.accepted_stamp_ns
                or other.sdk_stamp_ns != self.sdk_stamp_ns
                or other.opening != self.opening
                or not np.array_equal(other.measured, self.measured)
                or not np.array_equal(other.sdk, self.sdk)):
            raise LiftPathError('frozen lift context changed')


def derive_lift_path(nominal, plan, fk, sample, opening):
    """Preserve SDK-minus-accepted at EVERY waypoint, including derivatives.

    The frozen visual lift goal stays the nominal endpoint. This function
    deliberately grants no motion authority and makes no attachment assertion.
    """
    context = FrozenLiftContext.from_plan(plan, fk, sample, opening)
    shifted, audit = compensate_cartesian_path(nominal, fk, sample, plan.poses[3])
    audit.update(kind='offline_complete_lift_bias_derivation',
        plan_id=context.scene.plan_id, scene_sha256=context.scene.digest,
        model_sha256=context.model_sha256, epoch_ns=context.epoch_ns,
        accepted_stamp_ns=context.accepted_stamp_ns, sdk_stamp_ns=context.sdk_stamp_ns,
        accepted_opening_m=context.opening,
        object_model='frozen_rich_plan_OBB', object_geometry_sha256=context.scene.geometry_digest,
        occupancy_hypotheses=['target_remains_on_table', 'target_rigidly_follows_tool'],
        stationary_world_target_removed=False, attachment_confirmed=False,
        motion_authorized=False)
    return shifted, context, audit


def occupancy_at(context, transform):
    """Both hypotheses always remain present; no planning-scene side effects."""
    current = _transform(transform, 'tool pose')
    carried = current@context.T_tool_object
    return ((context.scene.center, context.scene.rotation, context.scene.size, 'world'),
            (carried[:3,3], carried[:3,:3], context.scene.size, 'hypothetical_carried'))


def _pad_margin(transform, center, rotation, size, name, gripper, opening):
    """Sufficient (conservative) permission for existing finite inner-pad contact.

    The entire target's X/Z projection fits a disk inside the real STL pad,
    and jaw penetration obeys the existing half-finger-thickness policy.
    No permission is granted for a palm, camera, mount or new finger contact.
    An OBB extending past the pad may be physically valid but is unresolved
    here; it must not silently become a whole-finger collision exemption.
    """
    local_center = transform[:3,:3].T@(center-transform[:3,3])
    local_rotation = transform[:3,:3].T@rotation
    local_corners = _box_corners(local_center, local_rotation, size)
    projected_radius = max(np.linalg.norm(p[[0,2]]-local_center[[0,2]]) for p in local_corners)
    patch = finger_contact_patch_margin_m(center, transform[:3,3], transform[:3,:3])-projected_radius
    side = 1. if name == 'left_finger' else -1.
    object_face = np.max(side*local_corners[:,1])
    penetration = object_face-.5*opening
    padding = .5*float(ANALYTICAL_FINGER_BOX_PADDING_XYZ_M[1])
    return min(float(patch), float(penetration+padding),
               .5*float(gripper.finger_size_xyz_m[1])-float(penetration))


def _intersection_vertices(c1, r1, s1, c2, r2, s2):
    """Vertices of the intersection of two OBBs, by all face triples.

    Empty/degenerate/ill-conditioned numerical results never grant contact.
    The padded finger OBB is a conservative CAD envelope, not a mesh claim.
    """
    A = np.concatenate((r1.T, -r1.T, r2.T, -r2.T))
    b = np.concatenate((r1.T@c1+s1/2, -r1.T@c1+s1/2,
                        r2.T@c2+s2/2, -r2.T@c2+s2/2))
    vertices = []
    for indices in combinations(range(12),3):
        rows = A[list(indices)]; rhs = b[list(indices)]
        det = abs(float(np.linalg.det(rows)))
        if det < 1e-12:
            # Only exact opposite/repeated face normals are structurally
            # redundant. An almost-parallel real vertex may be crucial.
            if any(np.array_equal(a,b) or np.array_equal(a,-b)
                   for a,b in combinations(rows,2)):
                continue
            # Exact coordinate-zero rank deficiency is also provable without
            # guessing a numerical rank (e.g. three faces in the XY plane).
            if np.any(np.all(rows == 0.,axis=0)):
                continue
            return None
        if np.linalg.cond(rows) > 1e8:
            return None
        point = np.linalg.solve(rows,rhs)
        inverse = np.linalg.inv(rows)
        gamma = 8*np.finfo(float).eps
        inverse_residual = (np.max(np.sum(np.abs(np.eye(3)-inverse@rows),axis=1))
                            + gamma*np.linalg.norm(np.abs(inverse)@np.abs(rows),ord=np.inf))
        if inverse_residual >= .5:
            return None
        inverse_bound = np.linalg.norm(inverse,ord=np.inf)/(1-inverse_residual)
        residual = np.abs(rows@point-rhs)+gamma*(np.abs(rows)@np.abs(point)+np.abs(rhs))
        error = float(inverse_bound*np.max(residual))
        if not math.isfinite(error) or error > 1e-9:
            return None
        # Retain every possibly feasible vertex; conservative extras only
        # make finite-pad admission harder. A strict 1e-8 pad margin below
        # exceeds the computed vertex enclosure and arithmetic allowance.
        evaluation_error = np.sum(np.abs(A),axis=1)*error+gamma*(np.abs(A)@np.abs(point)+np.abs(b))
        if np.all(A@point <= b+evaluation_error):
            vertices.append(point)
    return np.asarray(vertices) if vertices else None


def _intersection_in_pad(transform, center, rotation, size, name, gripper,
                         opening, relative_displacement=0.):
    """Prove only CAD-envelope∩target lies in the finite inner pad.

    Transform the fixed world object to tool0 and expand that OBB by the
    certified inverse-motion displacement on all its axes. Its intersection
    with the STATIC finger envelope contains every possible contact in the
    entire path interval. Thus no Lipschitz assumption about a changing
    intersection or projected-radius scalar is needed.
    """
    if name not in ('left_finger','right_finger'):
        return False
    d = float(relative_displacement)
    if not math.isfinite(d) or d < 0:
        return False
    R = transform[:3,:3]
    local_center = R.T@(center-transform[:3,3]); local_rotation = R.T@rotation
    row = next(row for row in _stage_boxes(np.eye(4),gripper,opening,'y','z') if row[0]==name)
    vertices = _intersection_vertices(row[1],np.eye(3),row[2],local_center,
                                      local_rotation,np.asarray(size)+2*d)
    if vertices is None:
        return False
    side = 1. if name=='left_finger' else -1.
    penetration = side*(vertices[:,1]-ANALYTICAL_FINGER_PAIR_CENTER_TOOL_XYZ_M[1])-.5*opening
    if (np.min(penetration) < -.5*ANALYTICAL_FINGER_BOX_PADDING_XYZ_M[1]-1e-10
            or np.max(penetration) > .5*gripper.finger_size_xyz_m[1]-1e-8):
        return False
    projected = vertices[:,[0,2]]-ANALYTICAL_FINGER_PAIR_CENTER_TOOL_XYZ_M[[0,2]]
    polygon = ANALYTICAL_FINGER_CONTACT_PATCH_TOOL_XZ_M
    # Strict numerical interior. Checking all chords also handles the STL
    # pad's concave notch; vertex-inside tests alone would miss that notch.
    if any(_signed_polygon_margin(v,polygon) < 1e-8 for v in projected):
        return False
    for a,b in combinations(projected,2):
        delta = b-a
        if np.linalg.norm(delta) < 1e-10:
            continue
        for c,e in zip(polygon,np.roll(polygon,-1,axis=0)):
            edge = e-c
            cross = float(np.cross(delta,edge))
            if abs(cross) <= 1e-14:
                continue
            t = float(np.cross(c-a,edge))/cross
            u = float(np.cross(c-a,delta))/cross
            if -1e-10 <= t <= 1.+1e-10 and -1e-10 <= u <= 1.+1e-10:
                return False
    return True


def validate_lift_trajectory(trajectory, context, fk, gripper, controller_hold,
                             extra_bodies=(), max_checks=8192, max_depth=20,
                             max_seconds=5., clock=time.monotonic):
    """Continuous conditional CAD/OBB certificate for one exact Noetic curve.

    Covers gripper, supplied rigid tool bodies, both object hypotheses, support,
    joint limits, requested t=0, and the supplied hold->first t>0 bridge.
    The public pair wrapper derives and binds that hold to measured/SDK data;
    Missing camera/mount bodies are reported explicitly. This is never a
    substitute for current MoveIt whole-arm scene checks or actual tracking.
    """
    if (not isinstance(context, FrozenLiftContext) or fk.model_sha256 != context.model_sha256
            or not isinstance(gripper, GripperGeometry) or gripper.support_clearance_m < .003
            or not 0 <= context.opening <= gripper.max_inner_gap_m
            or not isinstance(max_checks, int) or max_checks <= 0
            or not isinstance(max_depth, int) or not 0 <= max_depth <= 24
            or not math.isfinite(max_seconds) or max_seconds <= 0):
        raise LiftPathError('invalid lift geometry/model/budget context')
    extra_bodies = tuple(extra_bodies)
    if (any(not isinstance(b, ToolBodyBox) for b in extra_bodies)
            or len({b.name for b in extra_bodies}) != len(extra_bodies)):
        raise LiftPathError('invalid or duplicate extra tool body')
    tr = trajectory.joint_trajectory
    if (tuple(tr.joint_names) != fk.names or tr.header.stamp.to_nsec() != 0
            or getattr(trajectory.multi_dof_joint_trajectory, 'points', ())):
        raise LiftPathError('unsupported lift trajectory names/stamp/MultiDOF')
    points = list(tr.points)
    times = [float(p.time_from_start.to_sec()) for p in points]
    if (not 2 <= len(points) <= 4096 or not all(math.isfinite(t) and t >= 0 for t in times)
            or any(a >= b for a,b in zip(times,times[1:]))):
        raise LiftPathError('invalid lift waypoint timing')
    held = _array(controller_hold.positions, 6, 'stationary SDK hold')
    for field in ('velocities', 'accelerations'):
        if np.any(_array(getattr(controller_hold, field, ()), 6, field) != 0):
            raise LiftPathError('SDK bridge requires stationary derivatives')
    first = next(i for i,t in enumerate(times) if t > 0)
    segments = [(segment_coefficients(controller_hold, points[first], times[first], 6), 'SDK_bridge')]
    for i in range(first,len(points)-1):
        segments.append((segment_coefficients(points[i],points[i+1],times[i+1]-times[i],6), str(i)))
    started, checks, intervals = clock(), 0, 0
    scene = context.scene
    identity = np.eye(4)
    local_bodies = [(name,np.eye(3),center,size) for name,center,size in
                    _stage_boxes(identity,gripper,context.opening,'y','z')]
    for b in extra_bodies:
        local_bodies.append((b.name,b.T_tool_box[:3,:3],b.T_tool_box[:3,3],b.size))
    radius = max(float(np.linalg.norm(p)) for _,r,c,s in local_bodies for p in _box_corners(c,r,s))
    carried_radius = max(float(np.linalg.norm(p)) for p in _box_corners(
        context.T_tool_object[:3,3],context.T_tool_object[:3,:3],scene.size))
    body_radii = fk.point_motion_radii(radius)
    object_radii = fk.point_motion_radii(carried_radius)
    tool_radii = fk.point_motion_radii(0.)
    initial = fk(context.measured)
    allowed = set()
    for name,rot,c,size in local_bodies:
        wc = initial[:3,:3]@c+initial[:3,3]; wr = initial[:3,:3]@rot
        if _obb_overlap(wc,wr,size,scene.center,scene.rotation,scene.size):
            if name not in ('left_finger','right_finger') or not _intersection_in_pad(
                    initial,scene.center,scene.rotation,scene.size,name,gripper,context.opening):
                raise LiftPathError('initial CAD-envelope/target intersection has unresolved finite pad coverage: '+name)
            allowed.add(name)
    # The object's world and carried representations are alternative states of
    # the same object. They are never collision-tested against each other.
    minimum = {'gripper_support_m':float('inf'), 'carried_support_m':float('inf')}
    def pose_check(q, delta, label):
        nonlocal checks
        checks += 1
        if checks > max_checks or clock()-started > max_seconds:
            raise LiftPathError('continuous lift certificate computation budget exhausted')
        if np.any(q < fk.limits[:,0]) or np.any(q > fk.limits[:,1]):
            raise LiftPathError('lift state exceeds URDF joint limits at '+label)
        T = fk(q)
        displacement = float(body_radii@delta)
        object_displacement = float(object_radii@delta)
        tool_shift = float(tool_radii@delta)
        relative_shift = tool_shift+float(np.sum(delta))*(
            float(np.linalg.norm(scene.center-T[:3,3]))+.5*float(np.linalg.norm(scene.size))+tool_shift)
        okay = True
        world, carried = occupancy_at(context,T)
        carried_min = float(np.min(_box_corners(*carried[:3])@scene.normal+scene.offset))
        minimum['carried_support_m'] = min(minimum['carried_support_m'],carried_min)
        if carried_min < -1e-9:
            raise LiftPathError('hypothetical carried object enters support at '+label)
        if carried_min-object_displacement < 0:
            okay = False
        for name,local_rot,local_center,size in local_bodies:
            rot,center = T[:3,:3]@local_rot,T[:3,:3]@local_center+T[:3,3]
            clearance = float(np.min(_box_corners(center,rot,size)@scene.normal+scene.offset))
            minimum['gripper_support_m'] = min(minimum['gripper_support_m'],clearance)
            if clearance < gripper.support_clearance_m-1e-9:
                raise LiftPathError('body support violation at '+label+'/'+name)
            if clearance-displacement < gripper.support_clearance_m:
                okay = False
            for oc,orr,os,kind in (world,carried):
                overlap = _obb_overlap(center,rot,size,oc,orr,os)
                permission = (name in allowed and _intersection_in_pad(T,oc,orr,os,name,gripper,context.opening))
                if overlap and not permission:
                    raise LiftPathError(kind+' object collision or unresolved finite pad coverage at '+label+'/'+name)
                # Rigid carried/body relative geometry is unchanged for any q.
                if kind == 'hypothetical_carried':
                    continue
                if _obb_overlap(center,rot,size+2*displacement,oc,orr,os):
                    if (not permission or not _intersection_in_pad(T,oc,orr,os,name,gripper,context.opening,relative_shift)):
                        okay = False
        return okay
    if first:
        pose_check(_array(points[0].positions,6,'dropped start'),np.zeros(6),'dropped_t0')
    for c,label in segments:
        for s in (0.,1.): pose_check(_polynomial(c,s),np.zeros(6),label+' endpoint')
        stack = [(0.,1.,0)]
        while stack:
            lo,hi,depth = stack.pop(); mid = .5*(lo+hi)
            delta = .5*(hi-lo)*_derivative_bound(c,lo,hi)
            lower,upper = _polynomial_bounds(c,lo,hi)
            if np.any(lower < fk.limits[:,0]) or np.any(upper > fk.limits[:,1]):
                raise LiftPathError('lift spline exceeds URDF joint limits')
            passed = pose_check(_polynomial(c,mid),delta,label)
            intervals += 1
            if not passed:
                if depth >= max_depth:
                    raise LiftPathError('dual-occupancy continuous lift unresolved at subdivision limit')
                stack.extend(((mid,hi,depth+1),(lo,mid,depth+1)))
    missing = sorted({'camera','mount'}-{b.name for b in extra_bodies})
    return dict(kind='conditional_continuous_lift_geometry',
        plan_id=scene.plan_id, scene_sha256=scene.digest, model_sha256=fk.model_sha256,
        trajectory_sha256=wire_digest(trajectory), accepted_opening_m=context.opening,
        object_geometry_sha256=scene.geometry_digest,
        occupancy_hypotheses=['target_remains_on_table','target_rigidly_follows_tool'],
        stationary_world_target_removed=False, attachment_confirmed=False,
        allowed_initial_contact_bodies=sorted(allowed),
        body_sources={b.name:b.source_sha256 for b in extra_bodies},
        gripper_geometry=dict(max_inner_gap_m=gripper.max_inner_gap_m,
            finger_size_xyz_m=gripper.finger_size_xyz_m.tolist(),
            palm_size_xyz_m=gripper.palm_size_xyz_m.tolist(),
            support_clearance_m=gripper.support_clearance_m),
        missing_rigid_tool_body_models=missing, complete_tool_body_coverage=not missing,
        whole_arm_collision_checked=False, hardware_response_verified=False,
        motion_authorized=False, grasp_success=False,
        bridge='stationary_supplied_hold_to_first_positive_time_point', bridge_start_rad=held.tolist(),
        hold_binding='conditional_supplied_hold_use_validate_lift_pair_for_frozen_SDK_binding',
        segments=len(segments), interval_checks=intervals, fk_checks=checks,
        minimum_sampled_clearance=minimum, elapsed_seconds=clock()-started)


def validate_lift_pair(nominal, shifted, plan, context, fk, sample, opening,
                       gripper, extra_bodies=(), **budgets):
    """Rebind and certify both full paths, each including its proper bridge."""
    context.assert_binding(plan,fk,sample,opening)
    derived, expected, _ = derive_lift_path(nominal,plan,fk,sample,opening)
    if wire_digest(derived) != wire_digest(shifted):
        raise LiftPathError('whole lift bias was changed or applied to only part of path')
    # Also derive the hold rather than trusting a caller-provided synthetic t=0.
    measured_hold = deepcopy(nominal.joint_trajectory.points[0])
    measured_hold.positions = context.measured.tolist()
    measured_hold.velocities = [0.]*6; measured_hold.accelerations = [0.]*6
    sdk_hold = deepcopy(measured_hold); sdk_hold.positions = context.sdk.tolist()
    return dict(nominal=validate_lift_trajectory(nominal,context,fk,gripper,measured_hold,
                                               extra_bodies,**budgets),
                shifted_with_sdk_bridge=validate_lift_trajectory(shifted,context,fk,gripper,sdk_hold,
                                                                 extra_bodies,**budgets),
                motion_authorized=False, attachment_confirmed=False)
