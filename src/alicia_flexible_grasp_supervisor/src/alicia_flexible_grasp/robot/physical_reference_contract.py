"""Immutable physical/wire trajectory proofs; no ROS connection or permission.

Both curves retain their own complete CAD, controller bridge and stopping
proof. A source-bias model alone is insufficient to create this contract.
Live typed driver/HW admission is a separate execution requirement.
"""
from copy import deepcopy
from dataclasses import dataclass
import hashlib
import io
import json
import math
import re
from types import SimpleNamespace

import numpy as np

from .following_model import ARM_NAMES, SDK_QUANTUM_RAD, SourceBiasModel, validate_source_bias_model

WIRE_KIND = 'mapped_wire_commanded_reference_curve_v1'
WIRE_POLICY = 'commanded_stop_and_sdk_quantization_support_only_no_raw_tracking_claim'


class PhysicalReferenceError(ValueError):
    pass


def message_bytes(message):
    stream = io.BytesIO()
    message.serialize(stream)
    return stream.getvalue()


def message_sha256(message):
    return hashlib.sha256(message_bytes(message)).hexdigest()


def _json(value):
    return json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False)


def _trajectory(plan):
    try:
        trajectory = plan.joint_trajectory
        names = tuple(trajectory.joint_names)
        points = tuple(trajectory.points)
    except AttributeError as exc:
        raise PhysicalReferenceError('PHYSICAL_TRAJECTORY_INVALID') from exc
    if (len(names) != 6 or set(names) != set(ARM_NAMES) or len(points) < 2
            or getattr(getattr(plan, 'multi_dof_joint_trajectory', None), 'points', ())):
        raise PhysicalReferenceError('PHYSICAL_TRAJECTORY_INVALID')
    previous = -1
    for point in points:
        stamp = point.time_from_start.to_nsec()
        if stamp < 0 or stamp <= previous:
            raise PhysicalReferenceError('PHYSICAL_TRAJECTORY_TIME_INVALID')
        previous = stamp
        for field in ('positions', 'velocities', 'accelerations', 'effort'):
            values = tuple(getattr(point, field, ()))
            if (len(values) not in ((6,) if field == 'positions' else (0, 6))
                    or any(isinstance(v, bool) or not math.isfinite(v) for v in values)):
                raise PhysicalReferenceError('PHYSICAL_TRAJECTORY_VECTOR_INVALID')
    return trajectory


def physical_plan_from_wire(wire_plan, following_model):
    model = validate_source_bias_model(following_model)
    trajectory = _trajectory(wire_plan)
    by_name = dict(zip(ARM_NAMES, model.bias_rad))
    result = deepcopy(wire_plan)
    for point in result.joint_trajectory.points:
        point.positions = [float(value)-by_name[name]
            for name, value in zip(trajectory.joint_names, point.positions)]
    _trajectory(result)
    return result


def physical_reference_from_wire(reference, following_model, names=ARM_NAMES):
    model = validate_source_bias_model(following_model)
    names = tuple(names)
    if len(names) != 6 or set(names) != set(ARM_NAMES):
        raise PhysicalReferenceError('PHYSICAL_REFERENCE_JOINTS_INVALID')
    positions, velocities, accelerations = _reference(reference)
    by_name = dict(zip(ARM_NAMES, model.bias_rad))
    return SimpleNamespace(positions=tuple(v-by_name[n] for n, v in zip(names, positions)),
        velocities=velocities, accelerations=accelerations)


def _reference(value):
    vectors = []
    for field in ('positions', 'velocities', 'accelerations'):
        vector = tuple(getattr(value, field, ()))
        if len(vector) != 6 or any(isinstance(v, bool) or not math.isfinite(v) for v in vector):
            raise PhysicalReferenceError('PHYSICAL_REFERENCE_INVALID')
        vectors.append(tuple(float(v) for v in vector))
    if any(v != 0. for row in vectors[1:] for v in row):
        raise PhysicalReferenceError('PHYSICAL_REFERENCE_NOT_STATIONARY')
    return tuple(vectors)


def _ref_object(rows):
    return SimpleNamespace(**dict(zip(('positions', 'velocities', 'accelerations'), rows)))


def _restore_robot(data):
    from moveit_msgs.msg import RobotTrajectory
    result = RobotTrajectory()
    result.deserialize(data)
    return result


def commanded_reference_bounds(plan, reference, stop_duration):
    """Exact spline/bridge range and original commanded-stop arithmetic.

    SDK clipping is excluded continuously. This does not prove encoder joint
    limits or actual target/whole-arm stopping swept volume.
    """
    from .observation_path_guard import segment_coefficients, _polynomial_bounds
    from .observation_tracking_contract import validate_command_derivatives, commanded_stop_reserve
    tr = _trajectory(plan)
    _reference(reference)
    derivatives = validate_command_derivatives(plan, reference)
    stop = commanded_stop_reserve(stop_duration,
        velocity=max(derivatives['maximum_velocity_rad_s']),
        acceleration=max(derivatives['maximum_acceleration_rad_s2']))
    points = list(tr.points)
    times = [p.time_from_start.to_sec() for p in points]
    first = next((i for i,t in enumerate(times) if t > 0.), None)
    if first is None:
        raise PhysicalReferenceError('PHYSICAL_SDK_CURVE_TIME_INVALID')
    pairs = [(reference, points[first], times[first])]
    pairs.extend((points[i],points[i+1],times[i+1]-times[i]) for i in range(first,len(points)-1))
    rows = np.asarray([reference.positions]+[p.positions for p in points],dtype=float)
    lower, upper = np.min(rows,axis=0), np.max(rows,axis=0)
    for left,right,duration in pairs:
        c = segment_coefficients(left,right,duration,6)
        for lo,hi in zip(np.linspace(0.,1.,17)[:-1],np.linspace(0.,1.,17)[1:]):
            low,high = _polynomial_bounds(c,float(lo),float(hi))
            lower,upper = np.minimum(lower,low),np.maximum(upper,high)
    if (np.any(lower-stop < -math.pi) or np.any(upper+stop > math.pi-SDK_QUANTUM_RAD)):
        raise PhysicalReferenceError('PHYSICAL_SDK_CONTINUOUS_RANGE_OR_STOP_OUTSIDE_DOMAIN')
    return dict(policy=WIRE_POLICY, joint_names=list(tr.joint_names),
        trajectory_sha256=message_sha256(plan),
        controller_hold_positions_rad=list(reference.positions),
        command_derivative_bounds=derivatives, commanded_stop_reserve_rad=stop,
        stop_trajectory_duration_sec=float(stop_duration),
        sdk_quantization_bound_rad=SDK_QUANTUM_RAD/2,
        joint_error_bounds_rad=[stop+SDK_QUANTUM_RAD/2]*6,
        commanded_spline_minimum_rad=lower.tolist(), commanded_spline_maximum_rad=upper.tolist(),
        sdk_representable_domain_rad=[-math.pi,math.pi-SDK_QUANTUM_RAD],
        commanded_stop_sdk_domain_checked=True, raw_tracking_enforced=False,
        target_obb_tracking_uncertainty_checked=False, joint_limit_uncertainty_checked=False,
        certifies_hardware_tracking_stopping_or_calibration=False)


def build_wire_reference_audit(plan, scene, fk, gripper, opening, reference, *,
                               constraints, stop_duration, gripper_geometry_snapshot,
                               scheduled_start_ns=None, **budgets):
    from .observation_path_guard import validate_observation_trajectory
    bounds = commanded_reference_bounds(plan,reference,stop_duration)
    geometry = validate_observation_trajectory(plan,scene,fk,gripper,opening,reference,
        joint_error_bounds_rad=bounds['joint_error_bounds_rad'],
        scheduled_start_ns=scheduled_start_ns,**budgets)
    geometry.update(kind=WIRE_KIND, reference_domain='wire',
        wire_reference_contract=bounds,
        controller_constraints_snapshot=deepcopy(constraints),
        gripper_geometry_snapshot=deepcopy(gripper_geometry_snapshot),
        raw_tracking_enforced=False, execution_authorized=False,
        target_obb_tracking_uncertainty_checked=False, joint_limit_uncertainty_checked=False)
    return geometry


def _common_geometry(audit, plan, reference, model):
    if (not isinstance(audit,dict) or audit.get('trajectory_sha256') != message_sha256(plan)
            or audit.get('model_sha256') != model.model_sha256
            or re.fullmatch(r'[0-9a-f]{64}',str(audit.get('scene_sha256',''))) is None
            or not isinstance(audit.get('plan_id'),str) or not audit['plan_id']
            or not isinstance(audit.get('gripper_geometry_snapshot'),dict)
            or not audit['gripper_geometry_snapshot']
            or isinstance(audit.get('opening_width_m'),bool)
            or not isinstance(audit.get('opening_width_m'),(int,float))
            or not math.isfinite(audit['opening_width_m']) or not 0.<=audit['opening_width_m']<=.05):
        raise PhysicalReferenceError('PHYSICAL_COMMON_GEOMETRY_BINDING_INVALID')
    caps = np.asarray([audit['controller_constraints_snapshot'][n]['trajectory'] for n in ARM_NAMES])
    if np.any(np.asarray(model.source_bias_limit_rad)>caps):
        raise PhysicalReferenceError('PHYSICAL_SOURCE_CAP_CONFIG_MISMATCH')


def validate_wire_reference_audit(plan,audit,reference,model):
    _common_geometry(audit,plan,reference,model)
    bounds = audit.get('wire_reference_contract') or {}
    if bounds != commanded_reference_bounds(plan,reference,bounds.get('stop_trajectory_duration_sec')):
        raise PhysicalReferenceError('PHYSICAL_WIRE_REFERENCE_BOUNDS_CHANGED')
    support = audit.get('following_support') or {}
    target = audit.get('target_cad_projection') or {}
    if (audit.get('kind') != WIRE_KIND or audit.get('reference_domain') != 'wire'
            or 'execution_tracking_contract' in audit
            or audit.get('raw_tracking_enforced') is not False
            or audit.get('target_obb_tracking_uncertainty_checked') is not False
            or audit.get('joint_limit_uncertainty_checked') is not False
            or support.get('joint_error_bounds_rad') != bounds['joint_error_bounds_rad']
            or support.get('includes_controller_bridge') is not True
            or support.get('target_obb_tracking_uncertainty_checked') is not False
            or not math.isfinite(float(support.get('minimum_support_clearance_lower_bound_m',float('nan'))))
            or support['minimum_support_clearance_lower_bound_m'] < .003
            or not math.isfinite(float(audit.get('certified_minimum_clearance_m',float('nan'))))
            or audit['certified_minimum_clearance_m'] < .003
            or audit.get('bridge') != 'stationary_desired_to_first_positive_time_waypoint'
            or tuple(audit.get('bridge_start_joint_positions_rad',())) != _reference(reference)[0]
            or target.get('target_envelope_unchanged') is not True
            or target.get('support_and_palm_checks_unchanged') is not True):
        raise PhysicalReferenceError('PHYSICAL_WIRE_REFERENCE_PROOF_INVALID')
    return bounds


def _check_geometry(audit, plan, reference, model):
    from .contracted_trajectory import bound_action_goal
    if not isinstance(audit, dict):
        raise PhysicalReferenceError('PHYSICAL_COMPLETE_PROOF_REQUIRED')
    target = audit.get('target_cad_projection') or {}
    if (audit.get('model_sha256') != model.model_sha256
            or not isinstance(audit.get('scene_sha256'), str)
            or re.fullmatch(r'[0-9a-f]{64}', audit['scene_sha256']) is None
            or not isinstance(audit.get('plan_id'), str) or not audit['plan_id']
            or not isinstance(audit.get('gripper_geometry_snapshot'), dict)
            or not audit['gripper_geometry_snapshot']
            or isinstance(audit.get('opening_width_m'), bool)
            or not isinstance(audit.get('opening_width_m'), (int, float))
            or not math.isfinite(audit['opening_width_m'])
            or not 0. <= audit['opening_width_m'] <= .05
            or audit.get('bridge') != 'stationary_desired_to_first_positive_time_waypoint'
            or tuple(audit.get('bridge_start_joint_positions_rad', ())) != _reference(reference)[0]
            or target.get('target_envelope_unchanged') is not True
            or target.get('support_and_palm_checks_unchanged') is not True):
        raise PhysicalReferenceError('PHYSICAL_GEOMETRY_BINDING_INVALID')
    constraints = audit.get('controller_constraints_snapshot')
    tracking = audit.get('execution_tracking_contract') or {}
    stop = tracking.get('stop_trajectory_duration_sec')
    # This original builder verifies exact path/endpoint tolerances, full
    # following support and stopping reserve against this particular curve.
    goal = bound_action_goal(plan, audit, constraints, stop, reference=reference)
    caps = np.asarray([constraints[name]['trajectory'] for name in ARM_NAMES], dtype=float)
    if (caps.shape != (6,) or not np.all(np.isfinite(caps))
            or np.any(np.asarray(model.source_bias_limit_rad) > caps)):
        raise PhysicalReferenceError('PHYSICAL_SOURCE_CAP_CONFIG_MISMATCH')
    return goal


def _proof_kind(audit):
    from .contact_linear_reference import KIND as CONTACT_LINEAR_KIND
    from .lift_moveit_adapter import KIND as LIFT_KIND
    if audit.get('kind') == CONTACT_LINEAR_KIND:
        return 'contact_linear'
    if audit.get('kind') == LIFT_KIND:
        return 'closed_lift'
    return 'observation'


def _dispatch_geometry(proof_kind,domain,audit,plan,reference,model):
    _common_geometry(audit,plan,reference,model)
    if proof_kind == 'observation':
        if domain == 'wire':
            return validate_wire_reference_audit(plan,audit,reference,model)
        result = _check_geometry(audit,plan,reference,model)
    elif proof_kind == 'contact_linear':
        from .contact_linear_reference import bound_contact_linear_action_goal
        if audit.get('reference_domain') != domain:
            raise PhysicalReferenceError('PHYSICAL_CONTACT_REFERENCE_DOMAIN_CHANGED')
        result = bound_contact_linear_action_goal(plan,audit,
            audit['controller_constraints_snapshot'],
            audit['execution_tracking_contract']['stop_trajectory_duration_sec'],reference=reference)
    elif proof_kind == 'closed_lift':
        from .lift_moveit_adapter import bound_lift_action_goal
        if audit.get('reference_domain') != domain:
            raise PhysicalReferenceError('PHYSICAL_LIFT_REFERENCE_DOMAIN_CHANGED')
        result = bound_lift_action_goal(plan,audit,audit['controller_constraints_snapshot'],
            audit['execution_tracking_contract']['stop_trajectory_duration_sec'],reference=reference)
    else:
        raise PhysicalReferenceError('PHYSICAL_PROOF_KIND_UNSUPPORTED')
    commanded_reference_bounds(plan,reference,
        audit['execution_tracking_contract']['stop_trajectory_duration_sec'])
    return result


def _stop_duration(audit):
    return (audit.get('wire_reference_contract') or audit.get('execution_tracking_contract') or {})[
        'stop_trajectory_duration_sec']


@dataclass(frozen=True)
class DualReferenceContract:
    following_model: SourceBiasModel
    wire_robot_bytes: bytes
    physical_robot_bytes: bytes
    wire_audit_json: str
    physical_audit_json: str
    wire_reference: tuple
    physical_reference: tuple
    wire_trajectory_sha256: str
    physical_trajectory_sha256: str
    plan_sha256: str
    proof_kind: str = 'observation'

    @property
    def proof_sha256(self):
        value = dict(schema=2, proof_kind=self.proof_kind, source_bias_sha256=self.following_model.digest,
            wire_robot_sha256=hashlib.sha256(self.wire_robot_bytes).hexdigest(),
            physical_robot_sha256=hashlib.sha256(self.physical_robot_bytes).hexdigest(),
            wire_audit_sha256=hashlib.sha256(self.wire_audit_json.encode()).hexdigest(),
            physical_audit_sha256=hashlib.sha256(self.physical_audit_json.encode()).hexdigest(),
            wire_reference=self.wire_reference, physical_reference=self.physical_reference,
            wire_trajectory_sha256=self.wire_trajectory_sha256,
            physical_trajectory_sha256=self.physical_trajectory_sha256,
            plan_sha256=self.plan_sha256)
        return hashlib.sha256(_json(value).encode()).hexdigest()

    def physical_plan(self):
        return _restore_robot(self.physical_robot_bytes)

    def wire_plan(self):
        return _restore_robot(self.wire_robot_bytes)

    def action_goal(self):
        validate_dual_reference(self)
        return _dispatch_geometry(self.proof_kind,'physical',json.loads(self.physical_audit_json),
            self.physical_plan(),_ref_object(self.physical_reference),self.following_model)


def bind_dual_reference(wire_plan, physical_plan, wire_audit, physical_audit, *,
                        following_model, wire_reference, physical_reference, proof_kind=None):
    model = validate_source_bias_model(following_model)
    _trajectory(wire_plan)
    _trajectory(physical_plan)
    if message_bytes(physical_plan_from_wire(wire_plan, model)) != message_bytes(physical_plan):
        raise PhysicalReferenceError('PHYSICAL_WIRE_MAP_MISMATCH')
    expected = physical_reference_from_wire(wire_reference, model,
        wire_plan.joint_trajectory.joint_names)
    if _reference(expected) != _reference(physical_reference):
        raise PhysicalReferenceError('PHYSICAL_BRIDGE_MAP_MISMATCH')
    kind = _proof_kind(physical_audit) if proof_kind is None else proof_kind
    if kind != _proof_kind(physical_audit):
        raise PhysicalReferenceError('PHYSICAL_PROOF_KIND_CHANGED')
    for domain,plan,audit,reference in (('wire',wire_plan,wire_audit,wire_reference),
            ('physical',physical_plan,physical_audit,physical_reference)):
        _dispatch_geometry(kind,domain,audit,plan,reference,model)
    if (wire_audit['scene_sha256'] != physical_audit['scene_sha256']
            or wire_audit.get('plan_id') != physical_audit.get('plan_id')
            or wire_audit.get('opening_width_m') != physical_audit.get('opening_width_m')
            or wire_audit['gripper_geometry_snapshot'] != physical_audit['gripper_geometry_snapshot']
            or wire_audit['controller_constraints_snapshot'] != physical_audit['controller_constraints_snapshot']
            or _stop_duration(wire_audit) != _stop_duration(physical_audit)):
        raise PhysicalReferenceError('PHYSICAL_DUAL_CONTEXT_MISMATCH')
    if kind == 'closed_lift' and wire_audit['body_geometry_snapshot'] != physical_audit['body_geometry_snapshot']:
        raise PhysicalReferenceError('PHYSICAL_LIFT_BODY_CONTEXT_MISMATCH')
    return DualReferenceContract(following_model=model,
        wire_robot_bytes=message_bytes(wire_plan), physical_robot_bytes=message_bytes(physical_plan),
        wire_audit_json=_json(wire_audit), physical_audit_json=_json(physical_audit),
        wire_reference=_reference(wire_reference), physical_reference=_reference(physical_reference),
        wire_trajectory_sha256=message_sha256(wire_plan.joint_trajectory),
        physical_trajectory_sha256=message_sha256(physical_plan.joint_trajectory),
        plan_sha256=wire_audit['scene_sha256'], proof_kind=kind)


def validate_dual_reference(contract, wire_plan=None, physical_plan=None):
    if type(contract) is not DualReferenceContract:
        raise PhysicalReferenceError('PHYSICAL_DUAL_CONTRACT_REQUIRED')
    if (type(contract.wire_robot_bytes) is not bytes
            or type(contract.physical_robot_bytes) is not bytes
            or type(contract.wire_reference) is not tuple
            or type(contract.physical_reference) is not tuple
            or any(type(row) is not tuple for row in contract.wire_reference+contract.physical_reference)):
        raise PhysicalReferenceError('PHYSICAL_DUAL_CONTRACT_MUTABLE')
    wire = contract.wire_plan() if wire_plan is None else wire_plan
    physical = contract.physical_plan() if physical_plan is None else physical_plan
    if (message_bytes(wire) != contract.wire_robot_bytes
            or message_bytes(physical) != contract.physical_robot_bytes):
        raise PhysicalReferenceError('PHYSICAL_PROOF_TRAJECTORY_CHANGED')
    rebuilt = bind_dual_reference(wire, physical, json.loads(contract.wire_audit_json),
        json.loads(contract.physical_audit_json), following_model=contract.following_model,
        wire_reference=_ref_object(contract.wire_reference),
        physical_reference=_ref_object(contract.physical_reference),proof_kind=contract.proof_kind)
    if rebuilt != contract:
        raise PhysicalReferenceError('PHYSICAL_DUAL_CONTRACT_CHANGED')
    return contract
