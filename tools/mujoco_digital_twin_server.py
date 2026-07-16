#!/usr/bin/env python3
"""Combined WSL GraspNet + MuJoCo digital twin server.

This server is intended to run in the Windows/WSL2 GPU environment.  It keeps
the existing GraspNet baseline /predict protocol and adds a MuJoCo
/simulate_grasp pre-execution gate for the Alicia-D grasp sequence.
"""
import argparse
import copy
from dataclasses import dataclass
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import os
from pathlib import Path
import sys
import threading
import time
from xml.etree import ElementTree

import numpy as np


TOOLS_DIR = Path(__file__).resolve().parent
REPO_ROOT = TOOLS_DIR.parent
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

from graspnet_baseline_server import (  # noqa: E402
    GraspNetBaselineBackend,
    MockGraspNetBackend as _BaselineMockGraspNetBackend,
)


DEFAULT_MODEL_XML = (
    REPO_ROOT
    / 'src'
    / 'arm-mujoco'
    / 'synriard'
    / 'mjcf'
    / 'Alicia_D_v5_6'
    / 'Alicia_D_v5_6_gripper_50mm.xml'
)

SCHEMA_VERSION = 2
MAX_SNAPSHOT_AGE_SEC = 2.0
MAX_INNER_GAP_M = 0.050
MAX_CARTON_DIMENSION_M = 0.600
MAX_CARTON_HEIGHT_M = 0.500
MIN_LIFT_SUCCESS_M = 0.015
MAX_SINGLE_FINGER_OBJECT_MOTION_M = 0.002
OBJECT_SUPPORT_PENETRATION_TOLERANCE_M = 0.001
GRIPPER_MODEL_NAME = 'Alicia_D_v5_6_gripper_50mm'
GRIPPER_FINGER_SIZE_XYZ_M = (0.0434, 0.0286, 0.0600)
GRIPPER_PALM_SIZE_XYZ_M = (0.1175, 0.1550, 0.0774)
ARM_JOINT_NAMES = ('Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6')
TRAJECTORY_NAMES = ('pregrasp', 'approach', 'grasp', 'lift')


class ProtocolValidationError(ValueError):
    def __init__(self, code, reason):
        super().__init__(str(reason))
        self.code = str(code)
        self.reason = str(reason)


@dataclass(frozen=True)
class ContactClassification:
    left_contact: bool = False
    right_contact: bool = False
    two_sided: bool = False
    palm_contact: bool = False
    robot_support_collision: bool = False
    object_support_penetration: bool = False
    other_disallowed_collision: bool = False
    disallowed_collision: bool = False


@dataclass(frozen=True)
class CloseContactResult:
    success: bool
    contact_width_m: float = None
    failure_code: str = ''
    failure_reason: str = ''


@dataclass(frozen=True)
class LiftResult:
    collision_free: bool
    contact_retained: bool
    lift_success: bool
    failure_code: str = ''
    failure_reason: str = ''
    diagnosis: tuple = ()


@dataclass(frozen=True)
class BodyContact:
    first_body: str
    second_body: str
    distance_m: float = 0.0


def _echo_plan_id(payload):
    if isinstance(payload, dict) and isinstance(payload.get('plan_id'), str):
        return payload.get('plan_id')
    return ''


def _finite_float(value, code, field_name):
    if isinstance(value, bool):
        raise ProtocolValidationError(code, '%s must be a finite number' % field_name)
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError):
        raise ProtocolValidationError(code, '%s must be a finite number' % field_name)
    if not math.isfinite(result):
        raise ProtocolValidationError(code, '%s must be finite' % field_name)
    return result


def _finite_vector(value, length, code, field_name):
    if isinstance(value, (str, bytes)):
        raise ProtocolValidationError(code, '%s must contain %d values' % (field_name, length))
    try:
        result = list(value)
    except (TypeError, ValueError):
        raise ProtocolValidationError(code, '%s must contain %d values' % (field_name, length))
    if len(result) != length:
        raise ProtocolValidationError(code, '%s must contain exactly %d values' % (field_name, length))
    return [
        _finite_float(item, code, '%s[%d]' % (field_name, index))
        for index, item in enumerate(result)
    ]


def _normalized_xyzw(value, code, field_name):
    quaternion = _finite_vector(value, 4, code, field_name)
    norm = math.sqrt(sum(item * item for item in quaternion))
    if norm <= 1e-12:
        raise ProtocolValidationError(code, '%s must be non-zero' % field_name)
    quaternion = [item / norm for item in quaternion]
    if quaternion[3] < 0.0:
        quaternion = [-item for item in quaternion]
    return quaternion


def _validated_pose(value, code, field_name):
    if not isinstance(value, dict):
        raise ProtocolValidationError(code, '%s must be an object' % field_name)
    return {
        'position_m': _finite_vector(value.get('position_m'), 3, code, field_name + '.position_m'),
        'quaternion_xyzw': _normalized_xyzw(
            value.get('quaternion_xyzw'),
            code,
            field_name + '.quaternion_xyzw',
        ),
    }


def _validate_v2_payload(payload, now_sec=None, max_snapshot_age_sec=MAX_SNAPSHOT_AGE_SEC):
    """Return a normalized schema-v2 request or raise a stable fail-closed error."""
    if not isinstance(payload, dict):
        raise ProtocolValidationError('PLAN_INVALID', 'request must be a JSON object')
    result = copy.deepcopy(payload)
    if result.get('schema_version') != SCHEMA_VERSION:
        raise ProtocolValidationError('PLAN_INVALID', 'schema_version must be 2')
    if not isinstance(result.get('plan_id'), str) or not result.get('plan_id'):
        raise ProtocolValidationError('PLAN_INVALID', 'plan_id must be a non-empty string')
    if not isinstance(result.get('model_choice'), str) or not result.get('model_choice'):
        raise ProtocolValidationError('PLAN_INVALID', 'model_choice must be a non-empty string')

    stamp = _finite_float(result.get('snapshot_stamp_sec'), 'PLAN_INVALID', 'snapshot_stamp_sec')
    trajectory = result.get('trajectory')
    if not isinstance(trajectory, list) or len(trajectory) != 4:
        raise ProtocolValidationError('PLAN_INVALID', 'trajectory must contain exactly four poses')
    result['trajectory'] = [
        _validated_pose(item, 'PLAN_INVALID', 'trajectory[%d]' % index)
        for index, item in enumerate(trajectory)
    ]
    candidate_width = _finite_float(
        result.get('candidate_width_m'),
        'PLAN_INVALID',
        'candidate_width_m',
    )
    if candidate_width < 0.0:
        raise ProtocolValidationError('PLAN_INVALID', 'candidate_width_m must be non-negative')
    result['candidate_width_m'] = candidate_width

    now = time.time() if now_sec is None else _finite_float(now_sec, 'PLAN_INVALID', 'now_sec')
    max_age = _finite_float(max_snapshot_age_sec, 'PLAN_INVALID', 'max_snapshot_age_sec')
    age = now - stamp
    if stamp <= 0.0 or age < 0.0 or age > max_age:
        raise ProtocolValidationError(
            'PLAN_STALE',
            'snapshot_stamp_sec is stale or from the future (age=%.6fs)' % age,
        )
    result['snapshot_stamp_sec'] = stamp

    names = result.get('joint_names')
    positions = result.get('joint_positions')
    if not isinstance(names, list) or not isinstance(positions, list) or not names or len(names) != len(positions):
        raise ProtocolValidationError(
            'JOINT_STATE_INVALID',
            'joint_names and joint_positions must be non-empty and have equal length',
        )
    if any(not isinstance(name, str) or not name for name in names) or len(set(names)) != len(names):
        raise ProtocolValidationError('JOINT_STATE_INVALID', 'joint names must be unique non-empty strings')
    if any(names.count(required) != 1 for required in ARM_JOINT_NAMES):
        raise ProtocolValidationError('JOINT_STATE_INVALID', 'Joint1 through Joint6 must each appear exactly once')
    result['joint_positions'] = [
        _finite_float(value, 'JOINT_STATE_INVALID', 'joint_positions[%d]' % index)
        for index, value in enumerate(positions)
    ]

    required_width = _finite_float(
        result.get('required_open_width_m'),
        'GRIPPER_TOO_NARROW',
        'required_open_width_m',
    )
    if required_width <= 0.0 or required_width > MAX_INNER_GAP_M:
        raise ProtocolValidationError(
            'GRIPPER_TOO_NARROW',
            'required_open_width_m must be in (0, 0.050]',
        )
    result['required_open_width_m'] = required_width

    gripper = result.get('gripper')
    if not isinstance(gripper, dict):
        raise ProtocolValidationError('GRIPPER_MODEL_MISMATCH', 'gripper contract is missing')
    max_gap = _finite_float(
        gripper.get('max_inner_gap_m'),
        'GRIPPER_MODEL_MISMATCH',
        'gripper.max_inner_gap_m',
    )
    finger_size = _finite_vector(
        gripper.get('finger_size_xyz_m'),
        3,
        'GRIPPER_MODEL_MISMATCH',
        'gripper.finger_size_xyz_m',
    )
    palm_size = _finite_vector(
        gripper.get('palm_size_xyz_m'),
        3,
        'GRIPPER_MODEL_MISMATCH',
        'gripper.palm_size_xyz_m',
    )
    if (
        gripper.get('model_name') != GRIPPER_MODEL_NAME
        or abs(max_gap - MAX_INNER_GAP_M) > 1e-9
        or any(abs(a - b) > 1e-9 for a, b in zip(finger_size, GRIPPER_FINGER_SIZE_XYZ_M))
        or any(abs(a - b) > 1e-9 for a, b in zip(palm_size, GRIPPER_PALM_SIZE_XYZ_M))
    ):
        raise ProtocolValidationError(
            'GRIPPER_MODEL_MISMATCH',
            'request does not match the Alicia-D v5.6 50 mm gripper contract',
        )

    object_model = result.get('object_model')
    if not isinstance(object_model, dict) or object_model.get('type') != 'carton_box':
        raise ProtocolValidationError('OBB_INVALID', 'object_model.type must be carton_box')
    object_model['pose_base'] = _validated_pose(
        object_model.get('pose_base'),
        'OBB_INVALID',
        'object_model.pose_base',
    )
    size = _finite_vector(
        object_model.get('size_xyz_m'),
        3,
        'OBB_INVALID',
        'object_model.size_xyz_m',
    )
    if (
        any(value <= 0.0 or value > MAX_CARTON_DIMENSION_M for value in size)
        or size[2] > MAX_CARTON_HEIGHT_M
    ):
        raise ProtocolValidationError(
            'OBB_INVALID',
            'carton dimensions must be positive, at most 0.600 m, and height at most 0.500 m',
        )
    mass = _finite_float(object_model.get('mass_kg'), 'OBB_INVALID', 'object_model.mass_kg')
    friction = _finite_vector(object_model.get('friction'), 3, 'OBB_INVALID', 'object_model.friction')
    if mass <= 0.0 or any(value < 0.0 for value in friction):
        raise ProtocolValidationError('OBB_INVALID', 'carton mass must be positive and friction non-negative')
    object_model['size_xyz_m'] = size
    object_model['mass_kg'] = mass
    object_model['friction'] = friction

    support = result.get('support_plane')
    if not isinstance(support, dict):
        raise ProtocolValidationError('SUPPORT_PLANE_INVALID', 'support_plane must be an object')
    normal = _finite_vector(
        support.get('normal_base'),
        3,
        'SUPPORT_PLANE_INVALID',
        'support_plane.normal_base',
    )
    offset = _finite_float(
        support.get('offset_m'),
        'SUPPORT_PLANE_INVALID',
        'support_plane.offset_m',
    )
    normal_norm = math.sqrt(sum(value * value for value in normal))
    if normal_norm <= 1e-12:
        raise ProtocolValidationError('SUPPORT_PLANE_INVALID', 'support normal must be non-zero')
    support['normal_base'] = [value / normal_norm for value in normal]
    support['offset_m'] = offset / normal_norm
    return result


def _quantized_size_mm(size_xyz_m):
    return tuple(max(1, int(math.floor(float(value) * 1000.0 + 0.5))) for value in size_xyz_m)


def _model_cache_key(payload):
    object_model = payload.get('object_model') if isinstance(payload, dict) else None
    if not isinstance(object_model, dict):
        raise ProtocolValidationError('OBB_INVALID', 'object_model is missing')
    size_mm = _quantized_size_mm(object_model.get('size_xyz_m') or ())
    if len(size_mm) != 3:
        raise ProtocolValidationError('OBB_INVALID', 'object size must contain three values')
    return (
        str(object_model.get('type') or ''),
        size_mm,
        float(object_model.get('mass_kg')),
        tuple(float(value) for value in object_model.get('friction') or ()),
    )


def _canonical_object_model(object_model):
    result = dict(object_model)
    result['size_xyz_m'] = [value / 1000.0 for value in _quantized_size_mm(object_model['size_xyz_m'])]
    return result


def _inject_dynamic_scene(xml, object_model):
    root = ElementTree.fromstring(xml)
    worldbody = root.find('worldbody')
    if worldbody is None:
        raise ProtocolValidationError('PLAN_INVALID', 'MuJoCo XML has no worldbody')
    for geom in root.findall(".//geom[@name='floor']"):
        geom.set('contype', '0')
        geom.set('conaffinity', '0')
    for child in list(worldbody):
        if child.tag == 'body' and child.attrib.get('name') in ('target_object', 'detected_support'):
            worldbody.remove(child)
    canonical = _canonical_object_model(object_model)
    half = [0.5 * value for value in canonical['size_xyz_m']]
    carton_body = ElementTree.SubElement(worldbody, 'body', {'name': 'target_object', 'pos': '0 0 0'})
    ElementTree.SubElement(carton_body, 'freejoint', {'name': 'target_object_joint'})
    ElementTree.SubElement(
        carton_body,
        'geom',
        {
            'name': 'target_carton',
            'type': 'box',
            'size': '%.5f %.5f %.5f' % tuple(half),
            'mass': '%.5f' % float(canonical['mass_kg']),
            'rgba': '0.72 0.52 0.28 1',
            'friction': '%.5f %.5f %.5f' % tuple(float(value) for value in canonical['friction']),
        },
    )
    support_body = ElementTree.SubElement(
        worldbody,
        'body',
        {'name': 'detected_support', 'mocap': 'true', 'pos': '0 0 0'},
    )
    ElementTree.SubElement(
        support_body,
        'geom',
        {
            'name': 'detected_support_plane',
            'type': 'plane',
            'size': '10 10 0.125',
            'contype': '1',
            'conaffinity': '1',
            'friction': '1.2 0.08 0.02',
            'rgba': '0.65 0.65 0.65 0.35',
        },
    )
    return ElementTree.tostring(root, encoding='unicode')


def _support_plane_pose(normal, offset):
    normal = np.asarray(normal, dtype=float).reshape(3)
    offset = float(offset)
    norm = float(np.linalg.norm(normal))
    if not np.isfinite(normal).all() or not math.isfinite(offset) or norm <= 1e-12:
        raise ProtocolValidationError('SUPPORT_PLANE_INVALID', 'support plane is invalid')
    unit = normal / norm
    scaled_offset = offset / norm
    position = -scaled_offset * unit
    local_z = np.asarray([0.0, 0.0, 1.0], dtype=float)
    dot = float(np.clip(np.dot(local_z, unit), -1.0, 1.0))
    if dot <= -1.0 + 1e-10:
        quaternion = np.asarray([0.0, 1.0, 0.0, 0.0], dtype=float)
    else:
        cross = np.cross(local_z, unit)
        quaternion = np.asarray([1.0 + dot, cross[0], cross[1], cross[2]], dtype=float)
        quaternion /= np.linalg.norm(quaternion)
    return position.tolist(), quaternion.tolist()


def _rotate_vector_by_quaternion_wxyz(quaternion, vector):
    w, x, y, z = np.asarray(quaternion, dtype=float).reshape(4)
    rotation = np.asarray(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=float,
    )
    return (rotation @ np.asarray(vector, dtype=float).reshape(3)).tolist()


def _dynamic_scene_state(payload):
    object_pose = payload['object_model']['pose_base']
    quaternion_xyzw = _normalized_xyzw(
        object_pose['quaternion_xyzw'],
        'OBB_INVALID',
        'object_model.pose_base.quaternion_xyzw',
    )
    support_position, support_quaternion = _support_plane_pose(
        payload['support_plane']['normal_base'],
        payload['support_plane']['offset_m'],
    )
    return {
        'object_position_m': [float(value) for value in object_pose['position_m']],
        'object_quaternion_wxyz': [
            quaternion_xyzw[3],
            quaternion_xyzw[0],
            quaternion_xyzw[1],
            quaternion_xyzw[2],
        ],
        'support_position_m': support_position,
        'support_quaternion_wxyz': support_quaternion,
    }


def _apply_dynamic_scene_state(model, data, meta, state):
    object_joint = int(meta['object_joint'])
    qpos_address = int(model.jnt_qposadr[object_joint])
    data.qpos[qpos_address : qpos_address + 3] = state['object_position_m']
    data.qpos[qpos_address + 3 : qpos_address + 7] = state['object_quaternion_wxyz']
    mocap_id = int(meta['support_mocap_id'])
    data.mocap_pos[mocap_id] = state['support_position_m']
    data.mocap_quat[mocap_id] = state['support_quaternion_wxyz']


def _validate_gripper_contract_values(
    left_range,
    right_range,
    facing_surface_gap_m,
    tolerance_m=0.0005,
):
    left = np.asarray(left_range, dtype=float).reshape(2)
    right = np.asarray(right_range, dtype=float).reshape(2)
    gap = float(facing_surface_gap_m)
    if (
        not np.isfinite(left).all()
        or not np.isfinite(right).all()
        or not math.isfinite(gap)
        or np.max(np.abs(left - np.asarray([-0.025, 0.0]))) > tolerance_m
        or np.max(np.abs(right - np.asarray([0.0, 0.025]))) > tolerance_m
        or abs(gap - MAX_INNER_GAP_M) > tolerance_m
    ):
        raise ProtocolValidationError(
            'GRIPPER_MODEL_MISMATCH',
            'runtime finger ranges or facing collision-surface gap do not match the 50 mm contract',
        )


def _finger_qpos_for_inner_gap(inner_gap_m, max_gap_m=MAX_INNER_GAP_M):
    gap = _finite_float(inner_gap_m, 'GRIPPER_TOO_NARROW', 'inner_gap_m')
    maximum = _finite_float(max_gap_m, 'GRIPPER_MODEL_MISMATCH', 'max_gap_m')
    if gap < 0.0 or gap > maximum or maximum <= 0.0:
        raise ProtocolValidationError('GRIPPER_TOO_NARROW', 'inner gap is outside the validated gripper range')
    closing_travel = 0.5 * (maximum - gap)
    return -closing_travel, closing_travel


def _closure_widths(open_width_m, close_width_m, increments=35):
    open_width = _finite_float(open_width_m, 'GRIPPER_TOO_NARROW', 'open_width_m')
    close_width = _finite_float(close_width_m, 'GRIPPER_TOO_NARROW', 'close_width_m')
    if open_width <= close_width or open_width > MAX_INNER_GAP_M or close_width < 0.0:
        raise ProtocolValidationError('GRIPPER_TOO_NARROW', 'closure widths are invalid')
    return np.linspace(open_width, close_width, max(35, int(increments)) + 1).tolist()


def _contact_parts(contact):
    first = getattr(contact, 'first_body', None)
    second = getattr(contact, 'second_body', None)
    distance = getattr(contact, 'distance_m', getattr(contact, 'dist', 0.0))
    if first is None or second is None:
        try:
            first, second = contact[:2]
        except (TypeError, ValueError):
            raise ValueError('contact must provide two body names')
    return str(first), str(second), float(distance)


def _classify_close_contacts(
    contacts,
    left_body,
    right_body,
    object_body='target_object',
    palm_body='Link6',
    support_body='detected_support',
    penetration_tolerance_m=OBJECT_SUPPORT_PENETRATION_TOLERANCE_M,
):
    left = False
    right = False
    palm = False
    robot_support = False
    object_support_penetration = False
    other = False
    robot_bodies = set(ARM_JOINT_NAMES) | {
        'base_link', 'Link1', 'Link2', 'Link3', 'Link4', 'Link5', palm_body, left_body, right_body
    }
    for contact in contacts:
        first, second, distance = _contact_parts(contact)
        pair = {first, second}
        if object_body in pair and left_body in pair:
            left = True
            continue
        if object_body in pair and right_body in pair:
            right = True
            continue
        if object_body in pair and palm_body in pair:
            palm = True
            continue
        if object_body in pair and support_body in pair:
            if distance < -abs(float(penetration_tolerance_m)):
                object_support_penetration = True
            continue
        if support_body in pair and any(body in robot_bodies for body in pair):
            robot_support = True
            continue
        if object_body in pair or support_body in pair:
            other = True
    disallowed = bool(palm or robot_support or object_support_penetration or other)
    return ContactClassification(
        left_contact=bool(left),
        right_contact=bool(right),
        two_sided=bool(left and right),
        palm_contact=bool(palm),
        robot_support_collision=bool(robot_support),
        object_support_penetration=bool(object_support_penetration),
        other_disallowed_collision=bool(other),
        disallowed_collision=disallowed,
    )


def _evaluate_close_contact_samples(
    samples,
    left_body,
    right_body,
    max_single_finger_object_motion_m=MAX_SINGLE_FINGER_OBJECT_MOTION_M,
):
    if not samples:
        return CloseContactResult(False, failure_code='MUJOCO_CONTACT_FAILED', failure_reason='no closure samples')
    initial_position = np.asarray(samples[0].get('object_position_m', [0.0, 0.0, 0.0]), dtype=float)
    for sample in samples:
        classification = _classify_close_contacts(
            sample.get('contacts') or [],
            left_body=left_body,
            right_body=right_body,
        )
        if classification.disallowed_collision:
            return CloseContactResult(
                False,
                failure_code='MUJOCO_COLLISION',
                failure_reason='disallowed collision during gripper closure',
            )
        if classification.left_contact != classification.right_contact:
            position = np.asarray(sample.get('object_position_m', initial_position), dtype=float)
            if float(np.linalg.norm(position - initial_position)) > float(max_single_finger_object_motion_m):
                return CloseContactResult(
                    False,
                    failure_code='MUJOCO_CONTACT_FAILED',
                    failure_reason='single-finger contact made the object unstable',
                )
        if classification.two_sided:
            return CloseContactResult(True, contact_width_m=float(sample['width_m']))
    return CloseContactResult(
        False,
        failure_code='MUJOCO_CONTACT_FAILED',
        failure_reason='closure ended without simultaneous two-sided contact',
    )


def _lift_succeeded(
    object_delta_m,
    commanded_delta_m,
    contact_retained,
    collision_free,
    min_lift_m=MIN_LIFT_SUCCESS_M,
):
    values = (object_delta_m, commanded_delta_m, min_lift_m)
    if not all(isinstance(value, (int, float, np.number)) and math.isfinite(float(value)) for value in values):
        return False
    return bool(
        contact_retained
        and collision_free
        and float(commanded_delta_m) > 0.0
        and float(object_delta_m) >= float(min_lift_m)
    )


def _build_component_response(
    plan_id,
    pass_score,
    score,
    ik_success,
    collision_free,
    contact_success,
    lift_success,
    failure_code='',
    failure_reason='',
    backend='mujoco',
):
    try:
        finite_score = float(score)
    except (TypeError, ValueError, OverflowError):
        finite_score = 0.0
    if not math.isfinite(finite_score):
        finite_score = 0.0
        failure_code = failure_code or 'MUJOCO_INTERNAL_ERROR'
        failure_reason = failure_reason or 'MuJoCo produced a non-finite score'
    threshold = float(pass_score)
    components = (
        bool(ik_success),
        bool(collision_free),
        bool(contact_success),
        bool(lift_success),
    )
    simulation_ok = bool(all(components) and finite_score >= threshold)
    if not simulation_ok and not failure_code:
        if not components[0]:
            failure_code, failure_reason = 'MUJOCO_IK_FAILED', failure_reason or 'inverse kinematics failed'
        elif not components[1]:
            failure_code, failure_reason = 'MUJOCO_COLLISION', failure_reason or 'collision check failed'
        elif not components[2]:
            failure_code, failure_reason = 'MUJOCO_CONTACT_FAILED', failure_reason or 'two-sided contact failed'
        elif not components[3]:
            failure_code, failure_reason = 'MUJOCO_LIFT_FAILED', failure_reason or 'lift check failed'
        else:
            failure_code = 'MUJOCO_SCORE_BELOW_THRESHOLD'
            failure_reason = failure_reason or 'simulation score is below the configured threshold'
    if simulation_ok:
        failure_code = ''
        failure_reason = ''
    return {
        'backend': str(backend),
        'plan_id': plan_id if isinstance(plan_id, str) else '',
        'simulation_ok': simulation_ok,
        'score': finite_score,
        'ik_success': bool(ik_success),
        'collision_free': bool(collision_free),
        'contact_success': bool(contact_success),
        'lift_success': bool(lift_success),
        'failure_code': str(failure_code or ''),
        'failure_reason': str(failure_reason or ''),
    }


def _internal_simulation_failure(plan_id, backend, reason):
    return _build_component_response(
        plan_id=plan_id,
        pass_score=80,
        score=0.0,
        ik_success=False,
        collision_free=False,
        contact_success=False,
        lift_success=False,
        failure_code='MUJOCO_INTERNAL_ERROR',
        failure_reason=str(reason or 'internal simulation protocol error'),
        backend=backend,
    )


def _normalize_http_simulation_result(payload, result, sim_backend):
    plan_id = _echo_plan_id(payload)
    try:
        backend_name = str(getattr(sim_backend, 'name', 'unknown'))
    except Exception:
        backend_name = 'unknown'

    required_bool_keys = (
        'simulation_ok',
        'ik_success',
        'collision_free',
        'contact_success',
        'lift_success',
    )
    if not isinstance(result, dict):
        return _internal_simulation_failure(
            plan_id,
            backend_name,
            'simulation backend returned a non-object result',
        )
    if result.get('plan_id') != plan_id:
        return _internal_simulation_failure(
            plan_id,
            backend_name,
            'simulation backend returned a missing or mismatched plan_id',
        )
    if any(type(result.get(key)) is not bool for key in required_bool_keys):
        return _internal_simulation_failure(
            plan_id,
            backend_name,
            'simulation backend returned missing or non-boolean safety components',
        )
    score = result.get('score')
    if isinstance(score, bool):
        return _internal_simulation_failure(
            plan_id,
            backend_name,
            'simulation backend returned a non-finite score',
        )
    try:
        score = float(score)
    except (TypeError, ValueError, OverflowError):
        score = float('nan')
    if not math.isfinite(score):
        return _internal_simulation_failure(
            plan_id,
            backend_name,
            'simulation backend returned a non-finite score',
        )
    failure_code = result.get('failure_code')
    failure_reason = result.get('failure_reason')
    if not isinstance(failure_code, str) or not isinstance(failure_reason, str):
        return _internal_simulation_failure(
            plan_id,
            backend_name,
            'simulation backend returned incomplete failure details',
        )
    try:
        pass_score = float(getattr(sim_backend, 'pass_score', 80))
    except (TypeError, ValueError, OverflowError):
        pass_score = 80.0
    if not math.isfinite(pass_score):
        pass_score = 80.0
    normalized = _build_component_response(
        plan_id=plan_id,
        pass_score=pass_score,
        score=score,
        ik_success=result['ik_success'],
        collision_free=result['collision_free'],
        contact_success=result['contact_success'],
        lift_success=result['lift_success'],
        failure_code=failure_code,
        failure_reason=failure_reason,
        backend=backend_name,
    )
    if normalized['simulation_ok'] is not result['simulation_ok']:
        return _internal_simulation_failure(
            plan_id,
            backend_name,
            'simulation backend returned inconsistent safety components',
        )
    return normalized


def make_server(host, port, grasp_backend, sim_backend):
    server = ThreadingHTTPServer((host, int(port)), MujocoDigitalTwinHTTPHandler)
    server.grasp_backend = grasp_backend
    server.sim_backend = sim_backend
    return server


class MujocoDigitalTwinHTTPHandler(BaseHTTPRequestHandler):
    server_version = 'AliciaMujocoDigitalTwinHTTP/1.0'

    def do_GET(self):
        if self.path != '/health':
            self._send_json(404, {'ok': False, 'error': 'unknown path'})
            return
        grasp_health = self.server.grasp_backend.health()
        sim_health = self.server.sim_backend.health()
        self._send_json(
            200,
            {
                'ok': bool(grasp_health.get('ok', False)) and bool(sim_health.get('ok', False)),
                'backend': grasp_health.get('backend', 'unknown'),
                'loaded': grasp_health.get('loaded', False),
                'grasp_backend': grasp_health,
                'digital_twin': sim_health,
            },
        )

    def do_POST(self):
        handlers = {
            '/predict': self._handle_predict,
            '/sync_joint_state': self._handle_sync_joint_state,
            '/simulate_grasp': self._handle_simulate_grasp,
        }
        handler = handlers.get(self.path)
        if handler is None:
            self._send_json(404, {'ok': False, 'error': 'unknown path'})
            return
        payload = None
        try:
            payload = self._read_payload()
            self._send_json(200, handler(payload))
        except Exception as exc:
            if self.path == '/simulate_grasp':
                try:
                    backend_name = str(getattr(self.server.sim_backend, 'name', 'unknown'))
                except Exception:
                    backend_name = 'unknown'
                result = _internal_simulation_failure(
                    _echo_plan_id(payload),
                    backend_name,
                    exc,
                )
                result['ok'] = False
                self._send_json(200, result)
                return
            self._send_json(200, {'ok': False, 'error': str(exc)})

    def _handle_predict(self, payload):
        candidates = self.server.grasp_backend.predict(payload)
        return {
            'ok': True,
            'backend': self.server.grasp_backend.name,
            'candidates': candidates,
        }

    def _handle_sync_joint_state(self, payload):
        self.server.sim_backend.update_joint_state(payload)
        return {'ok': True, 'backend': self.server.sim_backend.name, 'message': 'joint state synced'}

    def _handle_simulate_grasp(self, payload):
        try:
            result = self.server.sim_backend.simulate_grasp(payload)
        except Exception as exc:
            result = _internal_simulation_failure(
                _echo_plan_id(payload),
                getattr(self.server.sim_backend, 'name', 'unknown'),
                exc,
            )
        result = _normalize_http_simulation_result(
            payload,
            result,
            self.server.sim_backend,
        )
        result['ok'] = True
        return result

    def _read_payload(self):
        length = int(self.headers.get('Content-Length', '0'))
        return json.loads(self.rfile.read(length).decode('utf-8'))

    def _send_json(self, status, payload):
        data = json.dumps(payload, allow_nan=False).encode('utf-8')
        self.send_response(status)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def log_message(self, fmt, *args):
        sys.stderr.write('[%s] %s\n' % (self.log_date_time_string(), fmt % args))


class MockGraspNetBackend(_BaselineMockGraspNetBackend):
    def predict(self, payload):
        if payload.get('encoding') == 'mock':
            return [
                {
                    'score': 1.0,
                    'width_m': 0.05,
                    'translation_m': [0.30, 0.0, 0.20],
                    'rotation_matrix': [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
                }
            ]
        return super().predict(payload)


class JointStateCache:
    def __init__(self):
        self._lock = threading.Lock()
        self.names = []
        self.positions = []
        self.stamp = 0.0
        self.source = 'none'

    def update(self, names, positions, source='http'):
        with self._lock:
            self.names = [str(name) for name in names]
            self.positions = [float(value) for value in positions]
            self.stamp = time.time()
            self.source = str(source)

    def snapshot(self):
        with self._lock:
            return list(self.names), list(self.positions), float(self.stamp), str(self.source)

    def age_sec(self):
        with self._lock:
            if not self.stamp:
                return None
            return max(0.0, time.time() - float(self.stamp))


class MockDigitalTwinBackend:
    name = 'mock_mujoco'

    def __init__(self):
        self.joint_cache = JointStateCache()

    def health(self):
        return {
            'ok': True,
            'backend': self.name,
            'joint_state_age_sec': self.joint_cache.age_sec(),
        }

    def update_joint_state(self, payload):
        self.joint_cache.update(payload.get('joint_names') or [], payload.get('joint_positions') or [], source='http')

    def simulate_grasp(self, payload):
        plan_id = _echo_plan_id(payload)
        try:
            _validate_v2_payload(payload, now_sec=time.time())
        except ProtocolValidationError as exc:
            response = _build_component_response(
                plan_id=plan_id,
                pass_score=80,
                score=0.0,
                ik_success=False,
                collision_free=False,
                contact_success=False,
                lift_success=False,
                failure_code=exc.code,
                failure_reason=exc.reason,
                backend=self.name,
            )
            response['diagnosis'] = [exc.reason]
            response['used_joint_state_source'] = self.joint_cache.snapshot()[3]
            return response
        response = _build_component_response(
            plan_id=plan_id,
            pass_score=80,
            score=0.0,
            ik_success=False,
            collision_free=False,
            contact_success=False,
            lift_success=False,
            failure_code='MUJOCO_INTERNAL_ERROR',
            failure_reason='mock MuJoCo cannot authorize physical execution',
            backend=self.name,
        )
        response['diagnosis'] = ['mock MuJoCo is protocol-only and always fails closed']
        response['used_joint_state_source'] = self.joint_cache.snapshot()[3]
        return response


class MujocoDigitalTwinBackend:
    name = 'mujoco'

    def __init__(
        self,
        model_xml=DEFAULT_MODEL_XML,
        pass_score=80,
        max_joint_state_age_sec=2.0,
        ros_sync_joint_states=False,
        ros_joint_state_topic='/joint_states',
        ee_orientation_body='Link6',
        left_finger_body='Link7',
        right_finger_body='Link8',
        min_lift_success_m=MIN_LIFT_SUCCESS_M,
    ):
        self.model_xml = Path(model_xml).expanduser()
        self.pass_score = int(pass_score)
        self.max_joint_state_age_sec = float(max_joint_state_age_sec)
        self.ros_sync_joint_states = bool(ros_sync_joint_states)
        self.ros_joint_state_topic = str(ros_joint_state_topic)
        self.ee_orientation_body = str(ee_orientation_body)
        self.left_finger_body = str(left_finger_body)
        self.right_finger_body = str(right_finger_body)
        self.min_lift_success_m = float(min_lift_success_m)
        if not math.isfinite(self.min_lift_success_m) or self.min_lift_success_m <= 0.0:
            raise ValueError('min_lift_success_m must be a positive finite number')
        self.joint_cache = JointStateCache()
        self._lock = threading.Lock()
        self._model_cache = {}
        self._mujoco = None
        if self.ros_sync_joint_states:
            self._start_ros_joint_state_subscriber()

    def health(self):
        missing = []
        if not self.model_xml.exists():
            missing.append('model_xml not found: %s' % self.model_xml)
        try:
            self._import_mujoco()
            mujoco_version = getattr(self._mujoco, '__version__', 'unknown')
        except Exception as exc:
            missing.append('mujoco import failed: %s' % exc)
            mujoco_version = 'missing'
        return {
            'ok': not missing,
            'backend': self.name,
            'model_xml': str(self.model_xml),
            'mujoco': str(mujoco_version),
            'joint_state_age_sec': self.joint_cache.age_sec(),
            'ros_sync_joint_states': self.ros_sync_joint_states,
            'min_lift_success_m': self.min_lift_success_m,
            'missing': missing,
        }

    def update_joint_state(self, payload):
        self.joint_cache.update(payload.get('joint_names') or [], payload.get('joint_positions') or [], source='http')

    def simulate_grasp(self, payload):
        plan_id = _echo_plan_id(payload)
        try:
            payload = _validate_v2_payload(payload, now_sec=time.time())
        except ProtocolValidationError as exc:
            return self._failure_response(plan_id, exc.code, exc.reason)
        with self._lock:
            try:
                return self._simulate_validated_grasp(payload, plan_id)
            except ProtocolValidationError as exc:
                return self._failure_response(plan_id, exc.code, exc.reason)
            except Exception as exc:
                return self._failure_response(plan_id, 'MUJOCO_INTERNAL_ERROR', str(exc))

    def _simulate_validated_grasp(self, payload, plan_id):
        self._import_mujoco()
        model, data, meta = self._model_for_payload(payload)
        self._apply_joint_state(model, data, payload, meta)
        self._apply_gripper_inner_gap(model, data, MAX_INNER_GAP_M, meta)
        _apply_dynamic_scene_state(model, data, meta, _dynamic_scene_state(payload))
        self._mujoco.mj_forward(model, data)
        sequence = _parse_grasp_sequence(payload)

        ik_results = []
        trajectory = []
        current_data = self._copy_data(model, data)
        for target in sequence:
            result = self._solve_ik(
                model,
                current_data,
                target['position'],
                target['rotation_matrix'],
                meta,
            )
            ik_results.append((target['name'], result))
            if not result['success']:
                reason = 'IK failed at %s: position error %.4fm orientation error %.4f' % (
                    target['name'],
                    result['position_error_m'],
                    result['orientation_error'],
                )
                return self._score_response(
                    plan_id,
                    ik_results,
                    collision_free=False,
                    contact_success=False,
                    lift_success=False,
                    failure_code='MUJOCO_IK_FAILED',
                    failure_reason=reason,
                    diagnosis=['IK failed for %s' % target['name']],
                )
            self._set_arm_qpos(
                model,
                current_data,
                meta['arm_joints'],
                result['joint_positions'],
                actuator_ids=meta['arm_actuators'],
            )
            trajectory.append((target['name'], result['joint_positions']))

        collision_free, collision_diag = self._check_trajectory_collisions(model, data, meta, trajectory)
        if not collision_free:
            reason = collision_diag[0] if collision_diag else 'trajectory collision detected'
            return self._score_response(
                plan_id,
                ik_results,
                collision_free=False,
                contact_success=False,
                lift_success=False,
                failure_code='MUJOCO_COLLISION',
                failure_reason=reason,
                diagnosis=collision_diag,
            )

        contact_success, close_collision_free, contact_width, retained_data, contact_diag = (
            self._simulate_close_contact(model, data, meta, trajectory[2][1])
        )
        collision_free = bool(collision_free and close_collision_free)
        if not collision_free or not contact_success:
            code = 'MUJOCO_COLLISION' if not collision_free else 'MUJOCO_CONTACT_FAILED'
            reason = contact_diag[0] if contact_diag else 'gripper did not form stable two-sided contact'
            return self._score_response(
                plan_id,
                ik_results,
                collision_free=collision_free,
                contact_success=False,
                lift_success=False,
                failure_code=code,
                failure_reason=reason,
                diagnosis=collision_diag + contact_diag,
            )

        support_normal = np.asarray(payload['support_plane']['normal_base'], dtype=float)
        lift_vector = np.asarray(sequence[3]['position'], dtype=float) - np.asarray(sequence[2]['position'], dtype=float)
        commanded_lift_m = max(0.0, float(np.dot(lift_vector, support_normal)))
        lift_result = self._simulate_lift(
            model=model,
            retained_data=retained_data,
            meta=meta,
            grasp_q=trajectory[2][1],
            lift_q=trajectory[3][1],
            payload=payload,
            contact_width_m=contact_width,
            commanded_lift_m=commanded_lift_m,
        )
        collision_free = bool(collision_free and lift_result.collision_free)
        contact_success = bool(contact_success and lift_result.contact_retained)
        lift_success = bool(lift_result.lift_success)
        lift_diag = list(lift_result.diagnosis)
        return self._score_response(
            plan_id,
            ik_results,
            collision_free=collision_free,
            contact_success=contact_success,
            lift_success=lift_success,
            failure_code=lift_result.failure_code,
            failure_reason=lift_result.failure_reason,
            diagnosis=collision_diag + contact_diag + lift_diag,
        )

    def _failure_response(self, plan_id, code, reason):
        response = _build_component_response(
            plan_id=plan_id,
            pass_score=self.pass_score,
            score=0.0,
            ik_success=False,
            collision_free=False,
            contact_success=False,
            lift_success=False,
            failure_code=code,
            failure_reason=reason,
            backend=self.name,
        )
        response['diagnosis'] = [str(reason)]
        response['used_joint_state_source'] = self.joint_cache.snapshot()[3]
        return response

    def _score_response(
        self,
        plan_id,
        ik_results,
        collision_free,
        contact_success,
        lift_success,
        failure_code,
        failure_reason,
        diagnosis,
    ):
        ik_success = bool(ik_results) and all(item[1]['success'] for item in ik_results)
        orientation_ok = bool(ik_results) and all(item[1]['orientation_error'] <= 0.18 for item in ik_results)
        score = 0
        score += 20 if ik_success else 0
        score += 20 if collision_free else 0
        score += 15 if orientation_ok else 0
        score += 20 if contact_success else 0
        score += 20 if lift_success else 0
        score += 5 if not failure_reason else 0
        response = _build_component_response(
            plan_id=plan_id,
            pass_score=self.pass_score,
            score=float(score),
            ik_success=ik_success,
            collision_free=collision_free,
            contact_success=contact_success,
            lift_success=lift_success,
            failure_code=failure_code,
            failure_reason=failure_reason,
            backend=self.name,
        )
        response.update(
            {
                'diagnosis': diagnosis or ['simulation score=%d' % score],
                'ik_results': [
                    {
                        'name': name,
                        'success': bool(result['success']),
                        'position_error_m': float(result['position_error_m']),
                        'orientation_error': float(result['orientation_error']),
                        'iterations': int(result['iterations']),
                    }
                    for name, result in ik_results
                ],
                'used_joint_state_source': self.joint_cache.snapshot()[3],
            }
        )
        return response

    def _import_mujoco(self):
        if self._mujoco is None:
            import mujoco

            self._mujoco = mujoco
        return self._mujoco

    def _start_ros_joint_state_subscriber(self):
        try:
            import rospy
            from sensor_msgs.msg import JointState
        except Exception as exc:
            sys.stderr.write('WARNING: ROS joint state sync disabled; rospy import failed: %s\n' % exc)
            self.ros_sync_joint_states = False
            return
        try:
            if not rospy.core.is_initialized():
                rospy.init_node('mujoco_digital_twin_joint_sync', anonymous=True, disable_signals=True)
            rospy.Subscriber(self.ros_joint_state_topic, JointState, self._ros_joint_state_cb, queue_size=1)
        except Exception as exc:
            sys.stderr.write('WARNING: ROS joint state sync disabled; subscriber failed: %s\n' % exc)
            self.ros_sync_joint_states = False

    def _ros_joint_state_cb(self, msg):
        self.joint_cache.update(getattr(msg, 'name', []), getattr(msg, 'position', []), source='ros')

    def _model_for_payload(self, payload):
        object_model = dict(payload['object_model'])
        key = _model_cache_key(payload)
        cached = self._model_cache.get(key)
        if cached is not None:
            model = cached[0]
            return model, self._mujoco.MjData(model), cached[1]
        xml = self.model_xml.read_text(encoding='utf-8')
        xml = _inject_options(xml)
        xml = _inject_actuators(xml)
        xml = _inject_dynamic_scene(xml, object_model)
        cwd = os.getcwd()
        try:
            os.chdir(str(self.model_xml.parent))
            model = self._mujoco.MjModel.from_xml_string(xml)
        finally:
            os.chdir(cwd)
        meta = self._model_meta(model)
        contract_data = self._mujoco.MjData(model)
        self._validate_runtime_gripper_contract(model, contract_data, meta)
        self._model_cache[key] = (model, meta)
        return model, self._mujoco.MjData(model), meta

    def _model_meta(self, model):
        support_body = self._body_id(model, 'detected_support')
        support_mocap_id = int(model.body_mocapid[support_body])
        if support_mocap_id < 0:
            raise RuntimeError('detected_support is not a mocap body')
        return {
            'arm_joints': [self._joint_id(model, name) for name in ARM_JOINT_NAMES],
            'arm_actuators': [self._actuator_id(model, name + '_act') for name in ARM_JOINT_NAMES],
            'left_finger_joint': self._joint_id(model, 'left_finger'),
            'right_finger_joint': self._joint_id(model, 'right_finger'),
            'left_finger_actuator': self._actuator_id(model, 'left_finger_act'),
            'right_finger_actuator': self._actuator_id(model, 'right_finger_act'),
            'left_body': self._body_id(model, self.left_finger_body),
            'right_body': self._body_id(model, self.right_finger_body),
            'orientation_body': self._body_id(model, self.ee_orientation_body),
            'object_body': self._body_id(model, 'target_object'),
            'object_joint': self._joint_id(model, 'target_object_joint'),
            'support_body': support_body,
            'support_mocap_id': support_mocap_id,
        }

    def _joint_id(self, model, name, required=True):
        joint_id = self._mujoco.mj_name2id(model, self._mujoco.mjtObj.mjOBJ_JOINT, str(name))
        if required and joint_id < 0:
            raise RuntimeError('MuJoCo joint not found: %s' % name)
        return joint_id

    def _body_id(self, model, name):
        body_id = self._mujoco.mj_name2id(model, self._mujoco.mjtObj.mjOBJ_BODY, str(name))
        if body_id < 0:
            raise RuntimeError('MuJoCo body not found: %s' % name)
        return body_id

    def _actuator_id(self, model, name):
        actuator_id = self._mujoco.mj_name2id(
            model,
            self._mujoco.mjtObj.mjOBJ_ACTUATOR,
            str(name),
        )
        if actuator_id < 0:
            raise RuntimeError('MuJoCo actuator not found: %s' % name)
        return actuator_id

    def _validate_runtime_gripper_contract(self, model, data, meta):
        left_joint = meta['left_finger_joint']
        right_joint = meta['right_finger_joint']
        data.qpos[model.jnt_qposadr[left_joint]] = 0.0
        data.qpos[model.jnt_qposadr[right_joint]] = 0.0
        data.qvel[model.jnt_dofadr[left_joint]] = 0.0
        data.qvel[model.jnt_dofadr[right_joint]] = 0.0
        data.ctrl[meta['left_finger_actuator']] = 0.0
        data.ctrl[meta['right_finger_actuator']] = 0.0
        self._mujoco.mj_forward(model, data)

        jaw_axis = np.asarray(data.xmat[meta['orientation_body']], dtype=float).reshape(3, 3)[:, 1]
        jaw_norm = float(np.linalg.norm(jaw_axis))
        if jaw_norm <= 1e-12:
            raise ProtocolValidationError('GRIPPER_MODEL_MISMATCH', 'runtime jaw axis is invalid')
        jaw_axis /= jaw_norm

        def projected_vertices(body_id):
            projections = []
            for geom_id in range(int(model.ngeom)):
                if int(model.geom_bodyid[geom_id]) != int(body_id):
                    continue
                if int(model.geom_contype[geom_id]) == 0:
                    continue
                mesh_id = int(model.geom_dataid[geom_id])
                if mesh_id < 0:
                    continue
                start = int(model.mesh_vertadr[mesh_id])
                count = int(model.mesh_vertnum[mesh_id])
                vertices = np.asarray(model.mesh_vert[start : start + count], dtype=float)
                rotation = np.asarray(data.geom_xmat[geom_id], dtype=float).reshape(3, 3)
                origin = np.asarray(data.geom_xpos[geom_id], dtype=float)
                world_vertices = vertices @ rotation.T + origin
                projections.extend((world_vertices @ jaw_axis).tolist())
            if not projections:
                raise ProtocolValidationError(
                    'GRIPPER_MODEL_MISMATCH',
                    'finger body has no enabled collision mesh',
                )
            return projections

        left_surface = max(projected_vertices(meta['left_body']))
        right_surface = min(projected_vertices(meta['right_body']))
        _validate_gripper_contract_values(
            model.jnt_range[left_joint],
            model.jnt_range[right_joint],
            right_surface - left_surface,
        )

    def _apply_joint_state(self, model, data, payload, meta):
        names = payload.get('joint_names')
        positions = payload.get('joint_positions')
        source = 'request'
        if not names or not positions:
            names, positions, stamp, source = self.joint_cache.snapshot()
            if not names or not positions:
                raise RuntimeError('no joint state supplied and WSL ROS/http joint cache is empty')
            if self.max_joint_state_age_sec > 0.0 and time.time() - stamp > self.max_joint_state_age_sec:
                raise RuntimeError('cached joint state is stale %.2fs' % (time.time() - stamp))
        mapping = {str(name): float(value) for name, value in zip(names, positions)}
        values = [mapping[name] for name in ARM_JOINT_NAMES]
        self._set_arm_qpos(
            model,
            data,
            meta['arm_joints'],
            values,
            actuator_ids=meta['arm_actuators'],
        )
        self.joint_cache.update(names, positions, source=source)

    def _apply_gripper_inner_gap(self, model, data, width_m, meta):
        left, right = _finger_qpos_for_inner_gap(width_m, max_gap_m=MAX_INNER_GAP_M)
        joint_ids = [meta['left_finger_joint'], meta['right_finger_joint']]
        actuator_ids = [meta['left_finger_actuator'], meta['right_finger_actuator']]
        for joint_id, actuator_id, value in zip(joint_ids, actuator_ids, (left, right)):
            data.qpos[model.jnt_qposadr[joint_id]] = float(value)
            data.qvel[model.jnt_dofadr[joint_id]] = 0.0
            data.ctrl[actuator_id] = float(value)
        self._mujoco.mj_forward(model, data)

    def _copy_data(self, model, data):
        copied = self._mujoco.MjData(model)
        copy_data = getattr(self._mujoco, 'mj_copyData', None)
        if callable(copy_data):
            try:
                copy_data(copied, model, data)
                return copied
            except (TypeError, AttributeError):
                pass
        for field in (
            'qpos',
            'qvel',
            'act',
            'ctrl',
            'qacc_warmstart',
            'mocap_pos',
            'mocap_quat',
            'userdata',
            'plugin_state',
            'eq_active',
        ):
            source = getattr(data, field, None)
            destination = getattr(copied, field, None)
            if source is not None and destination is not None:
                destination[...] = source
        if hasattr(data, 'time') and hasattr(copied, 'time'):
            copied.time = data.time
        self._mujoco.mj_forward(model, copied)
        return copied

    def _solve_ik(self, model, seed_data, target_pos, target_rot, meta):
        data = self._copy_data(model, seed_data)
        arm_joints = meta['arm_joints']
        dofs = [model.jnt_dofadr[joint] for joint in arm_joints]
        lower = np.asarray([model.jnt_range[joint][0] for joint in arm_joints], dtype=float)
        upper = np.asarray([model.jnt_range[joint][1] for joint in arm_joints], dtype=float)
        target_pos = np.asarray(target_pos, dtype=float).reshape(3)
        target_rot = np.asarray(target_rot, dtype=float).reshape(3, 3)
        jacp_l = np.zeros((3, model.nv))
        jacr_l = np.zeros((3, model.nv))
        jacp_r = np.zeros((3, model.nv))
        jacr_r = np.zeros((3, model.nv))
        jacp_o = np.zeros((3, model.nv))
        jacr_o = np.zeros((3, model.nv))
        lam = 0.025
        final_pos_err = float('inf')
        final_rot_err = float('inf')
        iterations = 0
        for iterations in range(1, 241):
            self._mujoco.mj_forward(model, data)
            center = self._gripper_center(data, meta)
            pos_err = target_pos - center
            rot_err = _orientation_error(data.xmat[meta['orientation_body']].reshape(3, 3), target_rot)
            final_pos_err = float(np.linalg.norm(pos_err))
            final_rot_err = float(np.linalg.norm(rot_err))
            if final_pos_err <= 0.005 and final_rot_err <= 0.16:
                break
            self._mujoco.mj_jacBody(model, data, jacp_l, jacr_l, meta['left_body'])
            self._mujoco.mj_jacBody(model, data, jacp_r, jacr_r, meta['right_body'])
            self._mujoco.mj_jacBody(model, data, jacp_o, jacr_o, meta['orientation_body'])
            jac_pos = ((jacp_l + jacp_r) * 0.5)[:, dofs]
            jac = np.vstack((jac_pos, 0.25 * jacr_o[:, dofs]))
            err = np.concatenate((pos_err, 0.25 * rot_err))
            lhs = jac.T @ jac + lam * np.eye(len(dofs))
            rhs = jac.T @ err
            try:
                dq = np.linalg.solve(lhs, rhs)
            except np.linalg.LinAlgError:
                dq = np.linalg.lstsq(lhs, rhs, rcond=None)[0]
            dq = np.clip(dq, -0.08, 0.08)
            q = np.asarray([data.qpos[model.jnt_qposadr[joint]] for joint in arm_joints], dtype=float)
            q = np.clip(q + 0.45 * dq, lower, upper)
            self._set_arm_qpos(model, data, arm_joints, q)
        success = bool(final_pos_err <= 0.005 and final_rot_err <= 0.16)
        q = np.asarray([data.qpos[model.jnt_qposadr[joint]] for joint in arm_joints], dtype=float)
        return {
            'success': success,
            'joint_positions': q.tolist(),
            'position_error_m': final_pos_err,
            'orientation_error': final_rot_err,
            'iterations': iterations,
        }

    def _set_arm_qpos(self, model, data, joints, values, actuator_ids=None):
        if actuator_ids is None:
            actuator_ids = [None] * len(joints)
        for joint_id, value, actuator_id in zip(joints, values, actuator_ids):
            data.qpos[model.jnt_qposadr[joint_id]] = float(value)
            data.qvel[model.jnt_dofadr[joint_id]] = 0.0
            if actuator_id is not None:
                data.ctrl[int(actuator_id)] = float(value)
        self._mujoco.mj_forward(model, data)

    def _gripper_center(self, data, meta):
        return (data.xpos[meta['left_body']] + data.xpos[meta['right_body']]) * 0.5

    def _check_trajectory_collisions(self, model, seed_data, meta, trajectory):
        data = self._copy_data(model, seed_data)
        previous = np.asarray([data.qpos[model.jnt_qposadr[j]] for j in meta['arm_joints']], dtype=float)
        diagnosis = []
        for name, target in trajectory:
            target = np.asarray(target, dtype=float)
            for alpha in np.linspace(0.0, 1.0, 20):
                q = (1.0 - alpha) * previous + alpha * target
                self._set_arm_qpos(
                    model,
                    data,
                    meta['arm_joints'],
                    q,
                    actuator_ids=meta['arm_actuators'],
                )
                bad = self._bad_contacts(model, data, allow_finger_object=(name in ('grasp', 'lift')))
                if bad:
                    diagnosis.append('collision at %s: %s' % (name, bad[0]))
                    return False, diagnosis
            previous = target
        diagnosis.append('trajectory collision check passed')
        return True, diagnosis

    def _simulate_close_contact(self, model, seed_data, meta, grasp_q):
        data = self._copy_data(model, seed_data)
        self._set_arm_qpos(
            model,
            data,
            meta['arm_joints'],
            grasp_q,
            actuator_ids=meta['arm_actuators'],
        )
        initial_object_position = np.asarray(data.xpos[meta['object_body']], dtype=float).copy()
        for width in _closure_widths(MAX_INNER_GAP_M, 0.0):
            self._set_arm_qpos(
                model,
                data,
                meta['arm_joints'],
                grasp_q,
                actuator_ids=meta['arm_actuators'],
            )
            self._apply_gripper_inner_gap(model, data, width, meta)
            self._mujoco.mj_step(model, data)
            classification = self._classify_current_contacts(model, data, meta)
            if classification.disallowed_collision:
                return (
                    False,
                    False,
                    None,
                    None,
                    ['disallowed collision during gripper closure at %.4fm gap' % width],
                )
            if classification.left_contact != classification.right_contact:
                displacement = float(
                    np.linalg.norm(
                        np.asarray(data.xpos[meta['object_body']], dtype=float)
                        - initial_object_position
                    )
                )
                if displacement > MAX_SINGLE_FINGER_OBJECT_MOTION_M:
                    return (
                        False,
                        True,
                        None,
                        None,
                        ['single-finger contact made the object unstable (%.4fm)' % displacement],
                    )
            if classification.two_sided:
                return (
                    True,
                    True,
                    float(width),
                    self._copy_data(model, data),
                    ['first simultaneous two-sided contact retained at %.4fm gap' % width],
                )
        return (
            False,
            True,
            None,
            None,
            ['closure ended without simultaneous two-sided finger/object contact'],
        )

    def _simulate_lift(
        self,
        model,
        retained_data,
        meta,
        grasp_q,
        lift_q,
        payload,
        contact_width_m,
        commanded_lift_m,
    ):
        data = self._copy_data(model, retained_data)
        start_position = np.asarray(data.xpos[meta['object_body']], dtype=float).copy()
        support = payload.get('support_plane') if isinstance(payload, dict) else None
        if isinstance(support, dict) and support.get('normal_base') is not None:
            support_normal = np.asarray(support['normal_base'], dtype=float).reshape(3)
            support_normal /= np.linalg.norm(support_normal)
        else:
            support_normal = np.asarray([0.0, 0.0, 1.0], dtype=float)
        for alpha in np.linspace(0.0, 1.0, 40):
            q = (1.0 - alpha) * np.asarray(grasp_q) + alpha * np.asarray(lift_q)
            self._set_arm_qpos(
                model,
                data,
                meta['arm_joints'],
                q,
                actuator_ids=meta['arm_actuators'],
            )
            self._apply_gripper_inner_gap(model, data, contact_width_m, meta)
            self._mujoco.mj_step(model, data)
            classification = self._classify_current_contacts(model, data, meta)
            if classification.disallowed_collision:
                reason = 'disallowed collision appeared during lift'
                return LiftResult(
                    collision_free=False,
                    contact_retained=bool(classification.two_sided),
                    lift_success=False,
                    failure_code='MUJOCO_COLLISION',
                    failure_reason=reason,
                    diagnosis=(reason,),
                )
            if not classification.two_sided:
                reason = 'two-sided finger/object contact was lost during lift'
                return LiftResult(
                    collision_free=True,
                    contact_retained=False,
                    lift_success=False,
                    failure_code='MUJOCO_CONTACT_FAILED',
                    failure_reason=reason,
                    diagnosis=(reason,),
                )
        end_position = np.asarray(data.xpos[meta['object_body']], dtype=float)
        object_delta = float(np.dot(end_position - start_position, support_normal))
        if _lift_succeeded(
            object_delta_m=object_delta,
            commanded_delta_m=commanded_lift_m,
            contact_retained=True,
            collision_free=True,
            min_lift_m=self.min_lift_success_m,
        ):
            reason = 'object followed support normal by %.3fm during lift' % object_delta
            return LiftResult(
                collision_free=True,
                contact_retained=True,
                lift_success=True,
                diagnosis=(reason,),
            )
        reason = (
            'object did not lift enough along support normal: %.3fm < %.3fm'
            % (object_delta, self.min_lift_success_m)
        )
        return LiftResult(
            collision_free=True,
            contact_retained=True,
            lift_success=False,
            failure_code='MUJOCO_LIFT_FAILED',
            failure_reason=reason,
            diagnosis=(reason,),
        )

    def _bad_contacts(self, model, data, allow_finger_object=False):
        contacts = self._body_contacts(model, data)
        classification = _classify_close_contacts(
            contacts,
            left_body=self.left_finger_body,
            right_body=self.right_finger_body,
        )
        bad = []
        if classification.disallowed_collision:
            bad.append('disallowed object/support/robot collision')
        if not allow_finger_object and (classification.left_contact or classification.right_contact):
            bad.append('finger/object contact before grasp stage')
        return bad

    def _body_contacts(self, model, data):
        contacts = []
        for index in range(int(data.ncon)):
            contact = data.contact[index]
            contacts.append(
                BodyContact(
                    first_body=self._geom_body_name(model, contact.geom1),
                    second_body=self._geom_body_name(model, contact.geom2),
                    distance_m=float(getattr(contact, 'dist', 0.0)),
                )
            )
        return contacts

    def _classify_current_contacts(self, model, data, meta):
        return _classify_close_contacts(
            self._body_contacts(model, data),
            left_body=self.left_finger_body,
            right_body=self.right_finger_body,
            object_body='target_object',
            palm_body=self.ee_orientation_body,
            support_body='detected_support',
        )

    def _geom_body_name(self, model, geom_id):
        body_id = int(model.geom_bodyid[int(geom_id)])
        return self._mujoco.mj_id2name(model, self._mujoco.mjtObj.mjOBJ_BODY, body_id) or str(body_id)


def _inject_options(xml):
    if '<option' not in xml:
        return xml.replace('<mujoco', '<mujoco', 1).replace('>', '>\n  <option integrator="implicitfast" cone="elliptic"/>', 1)
    return xml


def _inject_actuators(xml):
    if '<actuator>' in xml:
        return xml
    actuator = """
  <actuator>
    <position name="Joint1_act" joint="Joint1" kp="90" kv="12"/>
    <position name="Joint2_act" joint="Joint2" kp="90" kv="12"/>
    <position name="Joint3_act" joint="Joint3" kp="90" kv="12"/>
    <position name="Joint4_act" joint="Joint4" kp="70" kv="10"/>
    <position name="Joint5_act" joint="Joint5" kp="70" kv="10"/>
    <position name="Joint6_act" joint="Joint6" kp="45" kv="7"/>
    <position name="left_finger_act" joint="left_finger" kp="55" kv="7"/>
    <position name="right_finger_act" joint="right_finger" kp="55" kv="7"/>
  </actuator>"""
    return xml.replace('</mujoco>', actuator + '\n</mujoco>')


def _parse_grasp_sequence(payload):
    result = []
    for index, item in enumerate(payload.get('trajectory') or []):
        quat = _normalize_quat(item.get('quaternion_xyzw') or [0.0, 0.0, 0.0, 1.0])
        result.append(
            {
                'name': TRAJECTORY_NAMES[index] if index < len(TRAJECTORY_NAMES) else 'pose_%d' % index,
                'position': _vector3(item.get('position_m'), [0.0, 0.0, 0.0]),
                'rotation_matrix': _quat_xyzw_to_matrix(quat),
            }
        )
    return result


def _orientation_error(current, target):
    return 0.5 * (
        np.cross(current[:, 0], target[:, 0])
        + np.cross(current[:, 1], target[:, 1])
        + np.cross(current[:, 2], target[:, 2])
    )


def _quat_xyzw_to_matrix(quat):
    x, y, z, w = _normalize_quat(quat)
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return np.asarray(
        [
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ],
        dtype=float,
    )


def _normalize_quat(value):
    quat = np.asarray(value, dtype=float).reshape(4)
    norm = float(np.linalg.norm(quat))
    if norm <= 1e-12:
        return np.asarray([0.0, 0.0, 0.0, 1.0], dtype=float)
    quat = quat / norm
    if quat[3] < 0.0:
        quat = -quat
    return quat


def _vector3(value, default):
    if value is None:
        return np.asarray(default, dtype=float)
    vec = np.asarray(value, dtype=float).reshape(3)
    return vec


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description='Alicia WSL GraspNet + MuJoCo digital twin server')
    parser.add_argument('--host', default='0.0.0.0')
    parser.add_argument('--port', type=int, default=8000)
    parser.add_argument('--baseline-root', default=str(Path.home() / 'grasp6d_ws' / 'graspnet-baseline'))
    parser.add_argument('--checkpoint', default=str(Path.home() / 'grasp6d_ws' / 'checkpoints' / 'checkpoint-rs.tar'))
    parser.add_argument('--device', default='cuda:0')
    parser.add_argument('--num-view', type=int, default=300)
    parser.add_argument('--num-points', type=int, default=20000)
    parser.add_argument('--collision-thresh', type=float, default=0.01)
    parser.add_argument('--collision-voxel-size', type=float, default=0.01)
    parser.add_argument('--model-xml', default=str(DEFAULT_MODEL_XML))
    parser.add_argument('--pass-score', type=int, default=80)
    parser.add_argument('--min-lift-success-m', type=float, default=MIN_LIFT_SUCCESS_M)
    parser.add_argument('--max-joint-state-age-sec', type=float, default=2.0)
    parser.add_argument('--ros-sync-joint-states', action='store_true')
    parser.add_argument('--ros-joint-state-topic', default='/joint_states')
    parser.add_argument('--mock', action='store_true', help='Use mock inference and mock simulation backends')
    parser.add_argument('--mock-graspnet', action='store_true', help='Mock only GraspNet /predict')
    parser.add_argument('--mock-mujoco', action='store_true', help='Mock only MuJoCo /simulate_grasp')
    parser.add_argument('--warmup', action='store_true')
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    if args.mock or args.mock_graspnet:
        grasp_backend = MockGraspNetBackend()
    else:
        grasp_backend = GraspNetBaselineBackend(
            baseline_root=args.baseline_root,
            checkpoint=args.checkpoint,
            device=args.device,
            num_view=args.num_view,
            num_points=args.num_points,
            collision_thresh=args.collision_thresh,
            collision_voxel_size=args.collision_voxel_size,
        )
    if args.mock or args.mock_mujoco:
        sim_backend = MockDigitalTwinBackend()
    else:
        sim_backend = MujocoDigitalTwinBackend(
            model_xml=args.model_xml,
            pass_score=args.pass_score,
            max_joint_state_age_sec=args.max_joint_state_age_sec,
            min_lift_success_m=args.min_lift_success_m,
            ros_sync_joint_states=args.ros_sync_joint_states,
            ros_joint_state_topic=args.ros_joint_state_topic,
        )
    if args.warmup and hasattr(grasp_backend, 'load'):
        grasp_backend.load()
    server = make_server(args.host, args.port, grasp_backend, sim_backend)
    print(
        'Alicia MuJoCo digital twin server listening on http://%s:%d (grasp=%s, sim=%s)'
        % (args.host, args.port, grasp_backend.name, sim_backend.name),
        flush=True,
    )
    server.serve_forever()


if __name__ == '__main__':
    main()
