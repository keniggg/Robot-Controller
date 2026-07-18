#!/usr/bin/env python3
import pathlib
import struct
import sys
import xml.etree.ElementTree as ET

import numpy as np
import pytest


ROOT = pathlib.Path(__file__).resolve().parents[1]
for path in (ROOT, ROOT / 'src'):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from alicia_flexible_grasp.grasp.gripper_geometry import (  # noqa: E402
    ANALYTICAL_FINGER_BOX_PADDING_XYZ_M,
    ANALYTICAL_FINGER_PAIR_CENTER_TOOL_XYZ_M,
    ANALYTICAL_PALM_CENTER_TOOL_XYZ_M,
    ANALYTICAL_PALM_SIZE_XYZ_M,
    CandidateGateResult,
    GRIPPER_CONTRACT_TOLERANCE_M,
    GripperGeometry,
    candidate_rank_key,
    evaluate_candidate,
    gripper_box_centers,
    gripper_contract_mismatch_reason,
    required_open_width_m,
)


GRIPPER = GripperGeometry(
    max_inner_gap_m=0.050,
    jaw_clearance_each_side_m=0.002,
    finger_size_xyz_m=np.array([0.0434, 0.0286, 0.0600]),
    palm_size_xyz_m=np.array([0.1175, 0.1550, 0.0774]),
    support_clearance_m=0.003,
)


CONSERVATIVE_CONTRACT_FIELDS = (
    ('jaw_clearance', 'jaw_clearance_each_side_m', None),
    ('support_clearance', 'support_clearance_m', None),
    ('finger_x', 'finger_size_xyz_m', 0),
    ('finger_y', 'finger_size_xyz_m', 1),
    ('finger_z', 'finger_size_xyz_m', 2),
    ('palm_x', 'palm_size_xyz_m', 0),
    ('palm_y', 'palm_size_xyz_m', 1),
    ('palm_z', 'palm_size_xyz_m', 2),
)


def gripper_with_contract_delta(field, index, delta_m):
    values = {
        'max_inner_gap_m': GRIPPER.max_inner_gap_m,
        'jaw_clearance_each_side_m': GRIPPER.jaw_clearance_each_side_m,
        'finger_size_xyz_m': np.array(GRIPPER.finger_size_xyz_m, copy=True),
        'palm_size_xyz_m': np.array(GRIPPER.palm_size_xyz_m, copy=True),
        'support_clearance_m': GRIPPER.support_clearance_m,
    }
    if index is None:
        values[field] = float(values[field]) + float(delta_m)
    else:
        values[field][index] += float(delta_m)
    return GripperGeometry(**values)


def fixed_contract_reason(
    gripper,
    physical_open_width_m=0.0499375,
    tolerance_m=GRIPPER_CONTRACT_TOLERANCE_M,
):
    return gripper_contract_mismatch_reason(
        gripper,
        remote_max_inner_gap_m=0.050,
        physical_open_width_m=physical_open_width_m,
        twin_model_name='Alicia_D_v5_6_gripper_50mm',
        twin_max_inner_gap_m=0.050,
        tolerance_m=tolerance_m,
        tool_jaw_axis='y',
        tool_finger_length_axis='z',
    )


def transform(center, rotation):
    output = np.eye(4, dtype=float)
    output[:3, :3] = np.asarray(rotation, dtype=float)
    output[:3, 3] = np.asarray(center, dtype=float)
    return output


def rotation_about_y(angle):
    cosine = float(np.cos(angle))
    sine = float(np.sin(angle))
    return np.array(
        [[cosine, 0.0, sine], [0.0, 1.0, 0.0], [-sine, 0.0, cosine]],
        dtype=float,
    )


def rotation_from_rpy(values):
    roll, pitch, yaw = np.asarray(values, dtype=float)
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ],
        dtype=float,
    )


def xml_vector(element, attribute):
    return np.asarray(
        [float(value) for value in element.attrib[attribute].split()],
        dtype=float,
    )


def binary_stl_vertices(path):
    data = path.read_bytes()
    assert len(data) >= 84
    triangle_count = struct.unpack_from('<I', data, 80)[0]
    assert len(data) == 84 + triangle_count * 50
    vertices = []
    for index in range(triangle_count):
        values = struct.unpack_from('<12fH', data, 84 + index * 50)
        vertices.extend(np.asarray(values[3:12], dtype=float).reshape(3, 3))
    return np.asarray(vertices, dtype=float)


def finger_cad_vertices_tool(robot, urdf_path, joint_name, joint_position):
    joint = robot.find("./joint[@name='%s']" % joint_name)
    joint_origin = joint.find('origin')
    joint_xyz = xml_vector(joint_origin, 'xyz')
    joint_rotation = rotation_from_rpy(xml_vector(joint_origin, 'rpy'))
    joint_axis = xml_vector(joint.find('axis'), 'xyz')
    link = robot.find(
        "./link[@name='%s']" % joint.find('child').attrib['link']
    )
    collision = link.find('collision')
    collision_origin = collision.find('origin')
    collision_rotation = rotation_from_rpy(
        xml_vector(collision_origin, 'rpy')
    )
    mesh_path = (
        urdf_path.parent
        / collision.find('geometry/mesh').attrib['filename']
    ).resolve()
    vertices_link6 = (
        joint_rotation
        @ (
            collision_rotation @ binary_stl_vertices(mesh_path).T
            + xml_vector(collision_origin, 'xyz').reshape(3, 1)
            + float(joint_position) * joint_axis.reshape(3, 1)
        )
    ).T + joint_xyz

    tool_joint = robot.find("./joint[@name='Grasp2tool']")
    tool_origin = tool_joint.find('origin')
    tool_rotation = rotation_from_rpy(xml_vector(tool_origin, 'rpy'))
    return (
        tool_rotation.T
        @ (
            vertices_link6 - xml_vector(tool_origin, 'xyz')
        ).T
    ).T


def candidate_fixture(**overrides):
    rotation = rotation_about_y(np.pi * 0.5)
    center = np.array([0.0, 0.0, 0.080])
    depth = 0.030
    tool0 = center + depth * rotation[:, 2]
    values = {
        'gripper': GRIPPER,
        'candidate_center_base': center,
        'candidate_tool0_base': tool0,
        'candidate_depth_m': depth,
        'R_base_tool': rotation,
        'candidate_width_m': 0.012,
        'obb_center_base': center,
        'R_base_obb': np.eye(3),
        # A 30 mm GraspNet insertion keeps the physical palm clear of this
        # 40 mm approach-axis extent while the 40 mm jaw cross-section remains
        # the quantity exercised by the width gates below.
        'obb_size_xyz_m': np.array([0.040, 0.040, 0.060]),
        'support_normal_base': np.array([0.0, 0.0, 1.0]),
        'support_offset_m': 0.0,
        'pregrasp_T_base_tool': transform(np.array([-0.050, 0.0, 0.080]), rotation),
        'approach_T_base_tool': transform(np.array([0.010, 0.0, 0.080]), rotation),
        'grasp_T_base_tool': transform(tool0, rotation),
        'lift_T_base_tool': transform(np.array([0.030, 0.0, 0.130]), rotation),
        'tool_jaw_axis': 'y',
        'tool_finger_length_axis': 'z',
        'motion_cost': 0.0,
    }
    aliases = {
        'center_base': 'candidate_center_base',
        'pregrasp_center_base': 'pregrasp_T_base_tool',
        'approach_center_base': 'approach_T_base_tool',
        'grasp_center_base': 'grasp_T_base_tool',
        'lift_center_base': 'lift_T_base_tool',
    }
    for key, value in overrides.items():
        target = aliases.get(key, key)
        if target.endswith('_T_base_tool') and np.asarray(value).shape == (3,):
            values[target] = transform(value, rotation)
        else:
            values[target] = value
    if (
        (
            'candidate_center_base' in overrides
            or 'center_base' in overrides
            or 'candidate_depth_m' in overrides
        )
        and 'candidate_tool0_base' not in overrides
    ):
        values['candidate_tool0_base'] = (
            np.asarray(values['candidate_center_base'], dtype=float)
            + float(values['candidate_depth_m'])
            * np.asarray(values['R_base_tool'], dtype=float)[:, 2]
        )
    if (
        (
            'candidate_center_base' in overrides
            or 'center_base' in overrides
            or 'candidate_tool0_base' in overrides
            or 'candidate_depth_m' in overrides
        )
        and 'grasp_T_base_tool' not in overrides
    ):
        values['grasp_T_base_tool'] = transform(
            values['candidate_tool0_base'],
            values['R_base_tool'],
        )
    return values


def test_gripper_contract_defensively_copies_readonly_arrays():
    finger = np.array([0.0434, 0.0286, 0.0600])
    palm = np.array([0.1175, 0.1550, 0.0774])
    geometry = GripperGeometry(0.050, 0.002, finger, palm, 0.003)
    finger[:] = 1.0
    palm[:] = 1.0

    np.testing.assert_allclose(geometry.finger_size_xyz_m, [0.0434, 0.0286, 0.0600])
    np.testing.assert_allclose(geometry.palm_size_xyz_m, [0.1175, 0.1550, 0.0774])
    assert not geometry.finger_size_xyz_m.flags.writeable
    assert not geometry.palm_size_xyz_m.flags.writeable
    with pytest.raises(ValueError):
        geometry.finger_size_xyz_m[0] = 0.1


@pytest.mark.parametrize(
    'kwargs',
    [
        {'max_inner_gap_m': 0.0},
        {'jaw_clearance_each_side_m': -0.001},
        {'finger_size_xyz_m': [0.04, np.nan, 0.06]},
        {'palm_size_xyz_m': [0.1, 0.1]},
        {'support_clearance_m': -0.001},
    ],
)
def test_gripper_contract_rejects_invalid_physical_values(kwargs):
    values = {
        'max_inner_gap_m': 0.050,
        'jaw_clearance_each_side_m': 0.002,
        'finger_size_xyz_m': [0.0434, 0.0286, 0.0600],
        'palm_size_xyz_m': [0.1175, 0.1550, 0.0774],
        'support_clearance_m': 0.003,
    }
    values.update(kwargs)
    with pytest.raises(ValueError):
        GripperGeometry(**values)


def test_projected_carton_width_includes_both_clearances():
    required = required_open_width_m(
        obb_size_xyz_m=np.array([0.20, 0.040, 0.10]),
        R_base_obb=np.eye(3),
        jaw_axis_base=np.array([0.0, 1.0, 0.0]),
        clearance_each_side_m=0.002,
    )
    assert required == pytest.approx(0.044)


def test_tool_y_is_opposing_jaw_motion_and_finger_length_is_tool_z():
    rotation = rotation_about_y(np.pi * 0.5)
    boxes = gripper_box_centers(
        center_base=np.array([0.0, 0.0, 0.080]),
        R_base_tool=rotation,
        required_open_width_m=0.044,
        gripper=GRIPPER,
        tool_jaw_axis='y',
        tool_finger_length_axis='z',
    )

    jaw_axis = rotation[:, 1]
    finger_axis = rotation[:, 2]
    left_delta = boxes['left_finger'] - boxes['grasp_center']
    right_delta = boxes['right_finger'] - boxes['grasp_center']
    pair_center = 0.5 * (boxes['left_finger'] + boxes['right_finger'])
    assert np.dot(left_delta, jaw_axis) > 0.0
    assert np.dot(right_delta, jaw_axis) < 0.0
    assert np.dot(
        boxes['left_finger'] - pair_center,
        jaw_axis,
    ) == pytest.approx(
        -np.dot(boxes['right_finger'] - pair_center, jaw_axis)
    )
    np.testing.assert_allclose(
        pair_center,
        np.array([0.0, 0.0, 0.080])
        + rotation @ ANALYTICAL_FINGER_PAIR_CENTER_TOOL_XYZ_M,
        atol=1e-9,
    )
    np.testing.assert_allclose(
        boxes['left_finger'],
        [-0.0302, 0.0366, 0.0796],
        atol=1e-9,
    )
    np.testing.assert_allclose(
        boxes['right_finger'],
        [-0.0302, -0.0360, 0.0796],
        atol=1e-9,
    )
    np.testing.assert_allclose(
        boxes['palm'],
        np.array([0.0, 0.0, 0.080])
        + rotation @ ANALYTICAL_PALM_CENTER_TOOL_XYZ_M,
        atol=1e-9,
    )
    np.testing.assert_allclose(finger_axis, [1.0, 0.0, 0.0], atol=1e-9)
    assert GRIPPER.finger_size_xyz_m[2] == pytest.approx(0.060)


def test_real_urdf_and_stl_define_tool_y_jaws_and_tool_z_finger_length():
    urdf_path = (
        ROOT.parents[1]
        / 'src/arm-mujoco/synriard/urdf/Alicia_D_v5_6'
        / 'Alicia_D_v5_6_gripper_50mm.urdf'
    )
    robot = ET.parse(str(urdf_path)).getroot()
    tool_joint = robot.find("./joint[@name='Grasp2tool']")
    assert tool_joint.find('parent').attrib['link'] == 'Link6'
    assert tool_joint.find('child').attrib['link'] == 'tool0'
    np.testing.assert_allclose(
        xml_vector(tool_joint.find('origin'), 'rpy'),
        np.zeros(3),
        atol=1e-12,
    )

    finger_data = []
    for joint_name, link_name, open_limit, closed_limit in (
        ('left_finger', 'Link7', 'upper', 'lower'),
        ('right_finger', 'Link8', 'lower', 'upper'),
    ):
        joint = robot.find("./joint[@name='%s']" % joint_name)
        assert joint.find('parent').attrib['link'] == 'Link6'
        assert joint.find('child').attrib['link'] == link_name
        joint_origin = joint.find('origin')
        origin_xyz = xml_vector(joint_origin, 'xyz')
        joint_rotation = rotation_from_rpy(xml_vector(joint_origin, 'rpy'))
        joint_axis = xml_vector(joint.find('axis'), 'xyz')
        limits = joint.find('limit').attrib
        open_q = float(limits[open_limit])
        closed_q = float(limits[closed_limit])
        assert open_q == pytest.approx(0.0)
        closed_delta = joint_rotation @ joint_axis * (closed_q - open_q)

        link = robot.find("./link[@name='%s']" % link_name)
        collision = link.find('collision')
        collision_origin = collision.find('origin')
        collision_rotation = rotation_from_rpy(
            xml_vector(collision_origin, 'rpy')
        )
        mesh_filename = collision.find('geometry/mesh').attrib['filename']
        mesh_path = (urdf_path.parent / mesh_filename).resolve()
        vertices_link = binary_stl_vertices(mesh_path)
        vertices_link6 = (
            joint_rotation
            @ (
                collision_rotation @ vertices_link.T
                + xml_vector(collision_origin, 'xyz').reshape(3, 1)
            )
        ).T + origin_xyz
        envelope = np.ptp(vertices_link6, axis=0)
        finger_data.append((origin_xyz, closed_delta, envelope))

    left_origin, left_delta, left_envelope = finger_data[0]
    right_origin, right_delta, right_envelope = finger_data[1]
    np.testing.assert_allclose(left_delta, -right_delta, atol=2e-7)
    assert left_delta[1] > 0.0
    assert right_delta[1] < 0.0
    assert np.linalg.norm(
        (right_origin + right_delta) - (left_origin + left_delta)
    ) < np.linalg.norm(
        right_origin - left_origin,
    )
    np.testing.assert_allclose(
        left_envelope,
        [0.0434, 0.0286, 0.0600],
        atol=5e-4,
    )
    np.testing.assert_allclose(right_envelope, left_envelope, atol=2e-5)
    assert int(np.argmax(left_envelope)) == 2
    assert left_envelope[2] == pytest.approx(0.0600, abs=5e-4)


@pytest.mark.parametrize(
    'opening_width_m,joint_positions',
    [
        (0.050, {'left_finger': 0.0, 'right_finger': 0.0}),
        (0.0, {'left_finger': -0.025, 'right_finger': 0.025}),
    ],
    ids=('fully_open', 'fully_closed'),
)
def test_analytical_finger_boxes_cover_both_cad_fingers_on_all_six_faces(
    opening_width_m,
    joint_positions,
):
    urdf_path = (
        ROOT.parents[1]
        / 'src/arm-mujoco/synriard/urdf/Alicia_D_v5_6'
        / 'Alicia_D_v5_6_gripper_50mm.urdf'
    )
    robot = ET.parse(str(urdf_path)).getroot()
    boxes = gripper_box_centers(
        center_base=np.zeros(3, dtype=float),
        R_base_tool=np.eye(3, dtype=float),
        required_open_width_m=opening_width_m,
        gripper=GRIPPER,
        tool_jaw_axis='y',
        tool_finger_length_axis='z',
    )
    analytical_size = (
        GRIPPER.finger_size_xyz_m + ANALYTICAL_FINGER_BOX_PADDING_XYZ_M
    )
    analytical_half = 0.5 * analytical_size

    # The URDF's left finger occupies tool -Y, whereas the analytical names
    # denote the positive/negative jaw sides rather than the URDF link names.
    for joint_name, box_name in (
        ('left_finger', 'right_finger'),
        ('right_finger', 'left_finger'),
    ):
        cad_vertices = finger_cad_vertices_tool(
            robot,
            urdf_path,
            joint_name,
            joint_positions[joint_name],
        )
        cad_min = np.min(cad_vertices, axis=0)
        cad_max = np.max(cad_vertices, axis=0)
        analytical_min = boxes[box_name] - analytical_half
        analytical_max = boxes[box_name] + analytical_half

        assert np.all(
            cad_min - analytical_min >= GRIPPER_CONTRACT_TOLERANCE_M
        )
        assert np.all(
            analytical_max - cad_max >= GRIPPER_CONTRACT_TOLERANCE_M
        )


def test_analytical_palm_box_covers_link6_cad_aabb_with_contract_tolerance():
    urdf_path = (
        ROOT.parents[1]
        / 'src/arm-mujoco/synriard/urdf/Alicia_D_v5_6'
        / 'Alicia_D_v5_6_gripper_50mm.urdf'
    )
    robot = ET.parse(str(urdf_path)).getroot()
    link = robot.find("./link[@name='Link6']")
    collision = link.find('collision')
    collision_origin = collision.find('origin')
    mesh_path = (
        urdf_path.parent
        / collision.find('geometry/mesh').attrib['filename']
    ).resolve()
    vertices_link6 = (
        rotation_from_rpy(xml_vector(collision_origin, 'rpy'))
        @ binary_stl_vertices(mesh_path).T
    ).T + xml_vector(collision_origin, 'xyz')

    tool_joint = robot.find("./joint[@name='Grasp2tool']")
    tool_origin = tool_joint.find('origin')
    R_link6_tool = rotation_from_rpy(xml_vector(tool_origin, 'rpy'))
    vertices_tool = (
        R_link6_tool.T
        @ (
            vertices_link6
            - xml_vector(tool_origin, 'xyz')
        ).T
    ).T
    cad_min = np.min(vertices_tool, axis=0)
    cad_max = np.max(vertices_tool, axis=0)
    analytical_half = 0.5 * ANALYTICAL_PALM_SIZE_XYZ_M
    analytical_min = ANALYTICAL_PALM_CENTER_TOOL_XYZ_M - analytical_half
    analytical_max = ANALYTICAL_PALM_CENTER_TOOL_XYZ_M + analytical_half

    assert np.all(
        cad_min - analytical_min >= GRIPPER_CONTRACT_TOLERANCE_M
    )
    assert np.all(
        analytical_max - cad_max >= GRIPPER_CONTRACT_TOLERANCE_M
    )


def test_palm_tool_x_offset_is_included_in_support_collision_check():
    rotation = rotation_about_y(-np.pi * 0.5)
    center = np.array([0.0, 0.0, 0.100])
    depth = 0.030
    tool0 = center + depth * rotation[:, 2]
    args = candidate_fixture(
        R_base_tool=rotation,
        candidate_center_base=center,
        candidate_tool0_base=tool0,
        candidate_depth_m=depth,
        obb_center_base=center,
        pregrasp_T_base_tool=transform(
            tool0 - 0.080 * rotation[:, 2],
            rotation,
        ),
        approach_T_base_tool=transform(
            tool0 - 0.020 * rotation[:, 2],
            rotation,
        ),
        grasp_T_base_tool=transform(tool0, rotation),
        lift_T_base_tool=transform(
            tool0 + np.array([0.0, 0.0, 0.050]),
            rotation,
        ),
    )

    result = evaluate_candidate(**args)

    assert not result.ok
    assert result.failure_code == 'GRIPPER_SWEEP_COLLISION'
    assert result.failed_gate == 'static_envelope'
    assert 'palm enters support clearance' in result.failure_reason
    assert result.support_clearance_m < GRIPPER.support_clearance_m


def test_valid_40_mm_cross_section_passes():
    result = evaluate_candidate(**candidate_fixture())

    assert result.ok
    assert result.failure_code == ''
    assert result.required_open_width_m == pytest.approx(0.044)
    assert result.support_clearance_m >= GRIPPER.support_clearance_m
    assert result.jaw_alignment == pytest.approx(1.0)


@pytest.mark.parametrize('depth_m', [0.01, 0.02, 0.03, 0.04])
def test_geometry_accepts_each_graspnet_depth_relation_before_physical_gates(depth_m):
    result = evaluate_candidate(
        **candidate_fixture(
            candidate_depth_m=depth_m,
            obb_size_xyz_m=np.array([0.020, 0.040, 0.060]),
        )
    )

    assert result.failed_gate != 'transform'


def test_geometry_fails_closed_when_tool0_or_grasp_transform_drifts_from_depth():
    wrong_tool0 = candidate_fixture()
    wrong_tool0['candidate_tool0_base'] = (
        np.asarray(wrong_tool0['candidate_tool0_base'], dtype=float)
        + np.array([0.30, 0.0, 0.0])
    )
    tool0_result = evaluate_candidate(**wrong_tool0)

    wrong_transform = candidate_fixture()
    wrong_transform['grasp_T_base_tool'] = np.array(
        wrong_transform['grasp_T_base_tool'],
        copy=True,
    )
    wrong_transform['grasp_T_base_tool'][:3, 3] += np.array([0.30, 0.0, 0.0])
    transform_result = evaluate_candidate(**wrong_transform)

    assert tool0_result.failed_gate == 'transform'
    assert tool0_result.failure_code == 'TOOL0_INCONSISTENT'
    assert transform_result.failed_gate == 'transform'
    assert transform_result.failure_code == 'TOOL0_INCONSISTENT'


@pytest.mark.parametrize(
    'depth_m,expected_code',
    [
        (None, 'DEPTH_INVALID'),
        (False, 'DEPTH_INVALID'),
        (float('nan'), 'DEPTH_INVALID'),
        (float('inf'), 'DEPTH_INVALID'),
        (0.001, 'DEPTH_OUT_OF_RANGE'),
        (0.50, 'DEPTH_OUT_OF_RANGE'),
    ],
)
def test_geometry_rejects_invalid_graspnet_depth_domain(depth_m, expected_code):
    args = candidate_fixture()
    args['candidate_depth_m'] = depth_m
    result = evaluate_candidate(**args)

    assert result.failed_gate == 'transform'
    assert result.failure_code == expected_code


def test_51_mm_required_opening_is_rejected_even_when_model_width_is_small():
    args = candidate_fixture(obb_size_xyz_m=np.array([0.080, 0.047, 0.060]))
    result = evaluate_candidate(**args)

    assert not result.ok
    assert result.failure_code == 'GRIPPER_TOO_NARROW'
    assert result.required_open_width_m == pytest.approx(0.051)


def test_exactly_50_mm_required_opening_remains_inside_physical_limit():
    args = candidate_fixture(obb_size_xyz_m=np.array([0.040, 0.046, 0.060]))

    result = evaluate_candidate(**args)

    assert result.ok
    assert result.required_open_width_m == pytest.approx(0.050)


def test_fixed_50_mm_limit_cannot_be_bypassed_by_config_tolerance():
    configured_above_physical_limit = GripperGeometry(
        max_inner_gap_m=0.0504,
        jaw_clearance_each_side_m=0.002,
        finger_size_xyz_m=np.array([0.0434, 0.0286, 0.0600]),
        palm_size_xyz_m=np.array([0.1175, 0.1550, 0.0774]),
        support_clearance_m=0.003,
    )
    args = candidate_fixture(
        gripper=configured_above_physical_limit,
        obb_size_xyz_m=np.array([0.080, 0.0463, 0.060]),
    )

    result = evaluate_candidate(**args)

    assert not result.ok
    assert result.failure_code == 'GRIPPER_TOO_NARROW'
    assert result.required_open_width_m == pytest.approx(0.0503)


def test_model_width_is_diagnostic_not_a_physical_bypass_or_rejection():
    narrow_model = evaluate_candidate(**candidate_fixture(candidate_width_m=0.001))
    wide_model = evaluate_candidate(**candidate_fixture(candidate_width_m=0.090))

    assert narrow_model.ok
    assert wide_model.ok
    assert narrow_model.required_open_width_m == pytest.approx(0.044)
    assert wide_model.required_open_width_m == pytest.approx(0.044)


def test_center_below_support_or_outside_obb_is_rejected_with_stable_codes():
    below = candidate_fixture(center_base=np.array([0.0, 0.0, -0.004]))
    outside = candidate_fixture(center_base=np.array([0.30, 0.0, 0.080]))

    assert evaluate_candidate(**below).failure_code == 'GRIPPER_SWEEP_COLLISION'
    assert evaluate_candidate(**outside).failure_code == 'CENTER_OUTSIDE_OBB'


def test_jaw_line_that_misses_obb_is_rejected():
    args = candidate_fixture(center_base=np.array([0.022, 0.0, 0.080]))
    result = evaluate_candidate(**args)

    assert not result.ok
    assert result.failure_code == 'GRIPPER_SWEEP_COLLISION'
    assert result.failed_gate == 'jaw_width'


def test_one_sided_finger_reach_is_rejected():
    args = candidate_fixture(center_base=np.array([0.0, 0.008, 0.080]))
    result = evaluate_candidate(**args)

    assert not result.ok
    assert result.failure_code == 'GRIPPER_SWEEP_COLLISION'
    assert result.failed_gate == 'finger_reach'


def test_palm_sweep_through_tilted_plane_is_rejected():
    normal = np.array([0.20, 0.0, 0.98], dtype=float)
    normal /= np.linalg.norm(normal)
    obb_rotation = np.column_stack(
        (np.array([normal[2], 0.0, -normal[0]]), [0.0, 1.0, 0.0], normal)
    )
    args = candidate_fixture(
        R_base_obb=obb_rotation,
        support_normal_base=normal,
        support_offset_m=0.0,
        pregrasp_center_base=np.array([-0.080, 0.0, -0.010]),
    )
    result = evaluate_candidate(**args)

    assert not result.ok
    assert result.failure_code == 'GRIPPER_SWEEP_COLLISION'
    assert result.failed_gate in ('static_envelope', 'swept_envelope')


def test_support_plane_requires_unit_normal_aligned_with_obb_positive_z():
    center = np.array([0.0, 0.0, 0.200])
    common = {
        'center_base': center,
        'obb_center_base': center,
        'pregrasp_center_base': np.array([-0.080, 0.0, 0.200]),
        'approach_center_base': np.array([-0.020, 0.0, 0.200]),
        'lift_center_base': np.array([0.0, 0.0, 0.250]),
    }
    unit = evaluate_candidate(
        **candidate_fixture(
            **common,
            support_normal_base=np.array([0.0, 0.0, 1.0]),
            support_offset_m=-0.050,
        )
    )
    scaled_equivalent = evaluate_candidate(
        **candidate_fixture(
            **common,
            support_normal_base=np.array([0.0, 0.0, 0.1]),
            support_offset_m=-0.005,
        )
    )
    reversed_equivalent = evaluate_candidate(
        **candidate_fixture(
            **common,
            support_normal_base=np.array([0.0, 0.0, -1.0]),
            support_offset_m=0.050,
        )
    )

    assert unit.ok
    assert not scaled_equivalent.ok
    assert scaled_equivalent.failed_gate == 'transform'
    assert not reversed_equivalent.ok
    assert reversed_equivalent.failed_gate == 'transform'


def test_palm_intrusion_into_non_grasp_region_is_rejected():
    args = candidate_fixture(
        obb_size_xyz_m=np.array([0.160, 0.040, 0.060]),
    )
    result = evaluate_candidate(**args)

    assert not result.ok
    assert result.failure_code == 'GRIPPER_SWEEP_COLLISION'
    assert result.failed_gate == 'static_envelope'
    assert 'palm intrudes' in result.failure_reason


def test_intermediate_palm_sweep_collision_is_detected_between_safe_endpoints():
    args = candidate_fixture(
        pregrasp_center_base=np.array([-0.100, 0.0, 0.080]),
        approach_center_base=np.array([0.300, 0.0, 0.080]),
    )
    result = evaluate_candidate(**args)

    assert not result.ok
    assert result.failure_code == 'GRIPPER_SWEEP_COLLISION'
    assert result.failed_gate == 'swept_envelope'
    assert 'palm intrudes' in result.failure_reason


def test_nonfinite_or_left_handed_transform_fails_closed():
    nonfinite = candidate_fixture()
    nonfinite['candidate_center_base'] = np.array([0.0, np.nan, 0.080])
    left_handed = candidate_fixture()
    left_handed['R_base_tool'] = np.diag([1.0, 1.0, -1.0])

    assert evaluate_candidate(**nonfinite).failed_gate == 'transform'
    assert evaluate_candidate(**left_handed).failed_gate == 'transform'
    assert evaluate_candidate(**left_handed).failure_code == 'GRIPPER_SWEEP_COLLISION'


def test_extreme_finite_coordinates_return_a_finite_failed_result():
    result = evaluate_candidate(
        **candidate_fixture(center_base=np.array([1.0e308, 0.0, 0.080]))
    )

    assert not result.ok
    assert result.failure_code == 'GRIPPER_SWEEP_COLLISION'
    assert result.failed_gate == 'transform'
    assert np.all(
        np.isfinite(
            [
                result.required_open_width_m,
                result.center_distance_m,
                result.support_clearance_m,
                result.jaw_alignment,
                result.motion_cost,
                result.geometry_cost,
            ]
        )
    )


def test_candidate_result_rejects_nonfinite_costs():
    with pytest.raises(ValueError):
        CandidateGateResult(
            ok=True,
            failure_code='',
            failure_reason='',
            required_open_width_m=0.044,
            center_distance_m=0.0,
            support_clearance_m=0.010,
            jaw_alignment=1.0,
            motion_cost=np.nan,
            geometry_cost=0.0,
            failed_gate='',
            passed_gate_count=6,
        )


def test_geometry_rank_precedes_motion_and_model_score():
    geometry_best = CandidateGateResult(
        True, '', '', 0.044, 0.001, 0.010, 1.0, 10.0, 0.001, '', 6
    )
    motion_best = CandidateGateResult(
        True, '', '', 0.044, 0.002, 0.010, 1.0, 0.0, 0.002, '', 6
    )
    same_geometry_slow = CandidateGateResult(
        True, '', '', 0.044, 0.001, 0.010, 1.0, 2.0, 0.001, '', 6
    )
    same_geometry_fast = CandidateGateResult(
        True, '', '', 0.044, 0.001, 0.010, 1.0, 1.0, 0.001, '', 6
    )

    assert candidate_rank_key(geometry_best, model_score=0.1) < candidate_rank_key(
        motion_best, model_score=0.99
    )
    assert candidate_rank_key(same_geometry_fast, model_score=0.1) < candidate_rank_key(
        same_geometry_slow, model_score=0.99
    )


@pytest.mark.parametrize(
    '_name,field,index',
    CONSERVATIVE_CONTRACT_FIELDS,
    ids=[item[0] for item in CONSERVATIVE_CONTRACT_FIELDS],
)
def test_conservative_envelope_accepts_only_float_epsilon_shrink(
    _name,
    field,
    index,
):
    gripper = gripper_with_contract_delta(field, index, -0.5e-9)

    assert fixed_contract_reason(gripper) == ''


@pytest.mark.parametrize(
    '_name,field,index',
    CONSERVATIVE_CONTRACT_FIELDS,
    ids=[item[0] for item in CONSERVATIVE_CONTRACT_FIELDS],
)
def test_conservative_envelope_rejects_shrink_beyond_float_epsilon(
    _name,
    field,
    index,
):
    gripper = gripper_with_contract_delta(field, index, -2.0e-9)

    assert 'below fixed analytical' in fixed_contract_reason(gripper)


@pytest.mark.parametrize(
    '_name,field,index',
    CONSERVATIVE_CONTRACT_FIELDS,
    ids=[item[0] for item in CONSERVATIVE_CONTRACT_FIELDS],
)
def test_conservative_envelope_accepts_growth_at_contract_tolerance(
    _name,
    field,
    index,
):
    gripper = gripper_with_contract_delta(
        field,
        index,
        GRIPPER_CONTRACT_TOLERANCE_M,
    )

    assert fixed_contract_reason(gripper) == ''


@pytest.mark.parametrize(
    '_name,field,index',
    CONSERVATIVE_CONTRACT_FIELDS,
    ids=[item[0] for item in CONSERVATIVE_CONTRACT_FIELDS],
)
def test_conservative_envelope_rejects_growth_beyond_contract_tolerance(
    _name,
    field,
    index,
):
    gripper = gripper_with_contract_delta(
        field,
        index,
        GRIPPER_CONTRACT_TOLERANCE_M + 2.0e-9,
    )

    assert 'conservative growth tolerance' in fixed_contract_reason(gripper)


@pytest.mark.parametrize(
    'physical_open_width_m',
    [0.0499375, 0.0495, 0.0505],
    ids=('measured_49_9375_mm', 'lower_boundary', 'upper_boundary'),
)
def test_physical_open_gap_retains_independent_symmetric_half_mm_tolerance(
    physical_open_width_m,
):
    assert fixed_contract_reason(
        GRIPPER,
        physical_open_width_m=physical_open_width_m,
        tolerance_m=0.0,
    ) == ''


@pytest.mark.parametrize('physical_open_width_m', [0.049499998, 0.050500002])
def test_physical_open_gap_rejects_values_outside_symmetric_half_mm_tolerance(
    physical_open_width_m,
):
    assert 'physical open width' in fixed_contract_reason(
        GRIPPER,
        physical_open_width_m=physical_open_width_m,
        tolerance_m=0.0,
    )


def test_conservative_growth_tolerance_cannot_exceed_fixed_half_mm_limit():
    reason = fixed_contract_reason(
        GRIPPER,
        tolerance_m=GRIPPER_CONTRACT_TOLERANCE_M + 2.0e-9,
    )

    assert 'contract tolerance' in reason
    assert 'exceeds fixed' in reason


def test_fixed_analytical_contract_rejects_configured_envelope_mismatch():
    matching = gripper_contract_mismatch_reason(
        GRIPPER,
        remote_max_inner_gap_m=0.050,
        physical_open_width_m=0.050,
        twin_model_name='Alicia_D_v5_6_gripper_50mm',
        twin_max_inner_gap_m=0.050,
        tool_jaw_axis='y',
        tool_finger_length_axis='z',
    )
    wrong_finger = GripperGeometry(
        0.050,
        0.002,
        np.array([0.0434, 0.0300, 0.0600]),
        np.array([0.1175, 0.1550, 0.0774]),
        0.003,
    )

    assert matching == ''
    assert 'finger box' in gripper_contract_mismatch_reason(
        wrong_finger,
        0.050,
        0.050,
        'Alicia_D_v5_6_gripper_50mm',
        0.050,
    )
    assert 'fixed +Y jaw and +Z finger' in gripper_contract_mismatch_reason(
        GRIPPER,
        0.050,
        0.050,
        'Alicia_D_v5_6_gripper_50mm',
        0.050,
        tool_jaw_axis='x',
    )
