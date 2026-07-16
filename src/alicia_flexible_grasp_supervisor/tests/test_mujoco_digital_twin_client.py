#!/usr/bin/env python3
import json
import pathlib
import sys
import types
import unittest
import xml.etree.ElementTree as ET

import pytest
from geometry_msgs.msg import PoseArray, PoseStamped
from sensor_msgs.msg import JointState
from alicia_flexible_grasp_supervisor.msg import Grasp6DPlan


ROOT = pathlib.Path(__file__).resolve().parents[1]
for path in (ROOT, ROOT / 'src'):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from alicia_flexible_grasp.vision.mujoco_digital_twin_client import (  # noqa: E402
    build_simulation_payload,
    validate_mujoco_digital_twin_url,
)
from alicia_flexible_grasp.vision import mujoco_digital_twin_client as client_module  # noqa: E402


SAFETY_KEYS = (
    'simulation_ok',
    'ik_success',
    'collision_free',
    'contact_success',
    'lift_success',
)


def test_grasp_system_launch_wires_shared_remote_url_to_both_clients():
    launch_root = ET.parse(ROOT / 'launch' / 'grasp_system.launch').getroot()

    remote_url_args = launch_root.findall(".//arg[@name='remote_grasp6d_url']")
    assert len(remote_url_args) == 1

    for param_name in (
        '/grasp_6d/remote/server_url',
        '/mujoco_digital_twin/server_url',
    ):
        matching_params = [
            param
            for param in launch_root.findall('.//param')
            if param.get('name') == param_name
        ]
        assert len(matching_params) == 1
        assert matching_params[0].get('value') == '$(arg remote_grasp6d_url)'


def _rich_plan():
    plan = Grasp6DPlan()
    plan.header.frame_id = 'base_link'
    plan.header.stamp.secs = 123
    plan.header.stamp.nsecs = 456789000
    plan.valid = True
    plan.score = 0.91
    plan.candidate_width_m = 0.039
    plan.required_open_width_m = 0.044
    plan.model_choice = 'carton_segment'
    plan.plan_id = '0123456789abcdef01234567'
    for index, x in enumerate((0.10, 0.20, 0.30, 0.30)):
        pose = PoseStamped().pose
        pose.position.x = x
        pose.position.y = -0.02 + 0.01 * index
        pose.position.z = 0.25 + 0.01 * index
        pose.orientation.x = 0.1
        pose.orientation.y = 0.2
        pose.orientation.z = 0.3
        pose.orientation.w = 0.9
        plan.poses.append(pose)
    geometry = plan.object_geometry
    geometry.valid = True
    geometry.pose_base.position.x = 0.30
    geometry.pose_base.position.y = -0.04
    geometry.pose_base.position.z = 0.12
    geometry.pose_base.orientation.w = 1.0
    geometry.size_xyz_m.x = 0.24
    geometry.size_xyz_m.y = 0.16
    geometry.size_xyz_m.z = 0.10
    geometry.support_normal_base.z = 1.0
    geometry.support_offset_m = -0.02
    return plan


def _passing_response(plan_id='0123456789abcdef01234567'):
    response = {'plan_id': plan_id, 'score': 92.5}
    response.update({key: True for key in SAFETY_KEYS})
    return response


class MujocoDigitalTwinClientTest(unittest.TestCase):
    def _pose(self, x, y=0.0, z=0.2):
        pose = PoseStamped()
        pose.header.frame_id = 'base_link'
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = float(z)
        pose.pose.orientation.w = 1.0
        return pose

    def test_rejects_placeholder_url(self):
        with self.assertRaises(ValueError):
            validate_mujoco_digital_twin_url('http://<WSL_IP>:8000')

    def test_builds_simulation_payload_from_ros_messages(self):
        joints = JointState()
        joints.name = ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6', 'right_finger']
        joints.position = [0.1, -0.2, 0.3, -0.4, 0.5, -0.6, 0.05]
        plan = PoseArray()
        plan.header.frame_id = 'base_link'
        for x in (0.10, 0.20, 0.30, 0.30):
            plan.poses.append(self._pose(x).pose)
        obj = types.SimpleNamespace(
            detected=True,
            pose_base=self._pose(0.30, -0.04, 0.12),
            label='mouse',
            confidence=0.91,
        )

        payload = build_simulation_payload(
            joint_state=joints,
            object_pose=obj,
            grasp_plan=plan,
            gripper_width_m=0.05,
            object_model={'type': 'mouse_compound', 'size_xyz_m': [0.10, 0.06, 0.035]},
        )

        self.assertEqual(payload['joint_names'][:2], ['Joint1', 'Joint2'])
        self.assertEqual(payload['gripper_width_m'], 0.05)
        self.assertEqual(payload['object_pose_base']['label'], 'mouse')
        self.assertEqual(payload['object_pose_base']['size_xyz_m'], [0.10, 0.06, 0.035])
        self.assertEqual([item['name'] for item in payload['grasp_sequence_base']], ['pregrasp', 'approach', 'grasp', 'lift'])
        self.assertEqual(payload['grasp_sequence_base'][2]['position'], [0.30, 0.0, 0.2])


def test_build_mujoco_payload_serializes_exact_v2_rich_plan_schema():
    plan = _rich_plan()
    payload = client_module.build_mujoco_payload(
        plan,
        ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6'],
        [0.1, -0.2, 0.3, -0.4, 0.5, -0.6],
        {
            'name': 'Alicia_D_v5_6_gripper_50mm',
            'max_inner_gap_m': 0.050,
        },
    )

    assert set(payload) == {
        'schema_version',
        'plan_id',
        'snapshot_stamp_sec',
        'model_choice',
        'joint_names',
        'joint_positions',
        'trajectory',
        'candidate_width_m',
        'required_open_width_m',
        'gripper',
        'object_model',
        'support_plane',
    }
    assert payload['schema_version'] == 2
    assert payload['plan_id'] == plan.plan_id
    assert payload['snapshot_stamp_sec'] == pytest.approx(123.456789)
    assert payload['model_choice'] == plan.model_choice
    assert payload['joint_names'] == [
        'Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6'
    ]
    assert payload['joint_positions'] == pytest.approx(
        [0.1, -0.2, 0.3, -0.4, 0.5, -0.6]
    )
    assert payload['candidate_width_m'] == pytest.approx(plan.candidate_width_m)
    assert payload['required_open_width_m'] == pytest.approx(
        plan.required_open_width_m
    )
    assert len(payload['trajectory']) == 4
    assert payload['trajectory'][0]['position_m'] == pytest.approx(
        [0.10, -0.02, 0.25]
    )
    assert payload['trajectory'][0]['quaternion_xyzw'] == pytest.approx(
        [0.1, 0.2, 0.3, 0.9]
    )
    assert payload['gripper'] == {
        'model_name': 'Alicia_D_v5_6_gripper_50mm',
        'max_inner_gap_m': 0.050,
        'finger_size_xyz_m': [0.0434, 0.0286, 0.0600],
        'palm_size_xyz_m': [0.1175, 0.1550, 0.0774],
    }
    assert payload['object_model']['type'] == 'carton_box'
    assert payload['object_model']['pose_base']['position_m'] == pytest.approx(
        [0.30, -0.04, 0.12]
    )
    assert payload['object_model']['size_xyz_m'] == pytest.approx(
        [0.24, 0.16, 0.10]
    )
    assert payload['object_model']['mass_kg'] == pytest.approx(0.08)
    assert payload['object_model']['friction'] == pytest.approx(
        [1.2, 0.08, 0.02]
    )
    assert payload['support_plane']['normal_base'] == pytest.approx(
        [0.0, 0.0, 1.0]
    )
    assert payload['support_plane']['offset_m'] == pytest.approx(-0.02)
    json.dumps(payload, allow_nan=False)


@pytest.mark.parametrize(
    'mutation',
    [
        lambda plan, names, positions, config: setattr(
            plan.poses[0].position, 'x', float('nan')
        ),
        lambda plan, names, positions, config: setattr(
            plan.object_geometry.size_xyz_m, 'y', float('inf')
        ),
        lambda plan, names, positions, config: setattr(
            plan.object_geometry, 'support_offset_m', float('-inf')
        ),
        lambda plan, names, positions, config: positions.__setitem__(2, float('nan')),
        lambda plan, names, positions, config: setattr(
            plan, 'required_open_width_m', float('nan')
        ),
        lambda plan, names, positions, config: setattr(plan, 'score', float('inf')),
        lambda plan, names, positions, config: names.__setitem__(1, ''),
        lambda plan, names, positions, config: positions.pop(),
    ],
    ids=(
        'pose-nan',
        'obb-inf',
        'support-inf',
        'joint-nan',
        'width-nan',
        'score-inf',
        'empty-joint-name',
        'joint-length-mismatch',
    ),
)
def test_build_mujoco_payload_rejects_nonfinite_or_malformed_fields(mutation):
    plan = _rich_plan()
    names = ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6']
    positions = [0.1, -0.2, 0.3, -0.4, 0.5, -0.6]
    config = {
        'name': 'Alicia_D_v5_6_gripper_50mm',
        'max_inner_gap_m': 0.050,
    }
    mutation(plan, names, positions, config)

    with pytest.raises((TypeError, ValueError)):
        client_module.build_mujoco_payload(plan, names, positions, config)


def test_validate_mujoco_gate_response_accepts_only_complete_correlated_pass():
    result = client_module.validate_mujoco_gate_response(
        _passing_response(),
        '0123456789abcdef01234567',
        80,
    )

    assert result.ok
    assert result.code == ''
    assert result.reason == ''
    assert result.score == pytest.approx(92.5)


@pytest.mark.parametrize('echoed_id', [None, '', 'different-plan'])
def test_validate_mujoco_gate_response_rejects_missing_or_mismatched_plan_id(
    echoed_id,
):
    response = _passing_response()
    if echoed_id is None:
        response.pop('plan_id')
    else:
        response['plan_id'] = echoed_id

    result = client_module.validate_mujoco_gate_response(
        response,
        '0123456789abcdef01234567',
        80,
    )

    assert not result.ok
    assert result.code == 'PLAN_ID_MISMATCH'


@pytest.mark.parametrize('key', SAFETY_KEYS)
def test_validate_mujoco_gate_response_rejects_each_absent_safety_boolean(key):
    response = _passing_response()
    response.pop(key)

    result = client_module.validate_mujoco_gate_response(
        response,
        '0123456789abcdef01234567',
        80,
    )

    assert not result.ok
    assert key in result.reason


@pytest.mark.parametrize(
    ('key', 'expected_code'),
    [
        ('simulation_ok', 'GRIPPER_MODEL_MISMATCH'),
        ('ik_success', 'MUJOCO_IK_FAILED'),
        ('collision_free', 'MUJOCO_COLLISION'),
        ('contact_success', 'MUJOCO_CONTACT_FAILED'),
        ('lift_success', 'MUJOCO_LIFT_FAILED'),
    ],
)
def test_validate_mujoco_gate_response_rejects_each_false_safety_boolean(
    key,
    expected_code,
):
    response = _passing_response()
    response[key] = False
    if key == 'simulation_ok':
        response['failure_code'] = 'GRIPPER_MODEL_MISMATCH'
        response['failure_reason'] = 'WSL gripper contract rejected'

    result = client_module.validate_mujoco_gate_response(
        response,
        '0123456789abcdef01234567',
        80,
    )

    assert not result.ok
    assert result.code == expected_code
    if key == 'simulation_ok':
        assert result.reason == 'WSL gripper contract rejected'


def test_validate_mujoco_gate_response_preserves_wsl_preflight_failure():
    response = _passing_response()
    response.update({key: False for key in SAFETY_KEYS})
    response['failure_code'] = 'GRIPPER_MODEL_MISMATCH'
    response['failure_reason'] = 'WSL rejected the configured gripper contract'

    result = client_module.validate_mujoco_gate_response(
        response,
        '0123456789abcdef01234567',
        80,
    )

    assert not result.ok
    assert result.code == 'GRIPPER_MODEL_MISMATCH'
    assert result.reason == 'WSL rejected the configured gripper contract'


@pytest.mark.parametrize('key', SAFETY_KEYS)
@pytest.mark.parametrize('value', [1, 1.0, 'true'])
def test_validate_mujoco_gate_response_requires_python_bool_true(key, value):
    response = _passing_response()
    response[key] = value

    result = client_module.validate_mujoco_gate_response(
        response,
        '0123456789abcdef01234567',
        80,
    )

    assert not result.ok


@pytest.mark.parametrize(
    'score',
    [None, True, '80', [], {}, float('nan'), float('inf'), -float('inf')],
)
def test_validate_mujoco_gate_response_rejects_malformed_or_nonfinite_score(score):
    response = _passing_response()
    response['score'] = score

    result = client_module.validate_mujoco_gate_response(
        response,
        '0123456789abcdef01234567',
        80,
    )

    assert not result.ok


def test_validate_mujoco_gate_response_rejects_missing_score():
    response = _passing_response()
    response.pop('score')

    result = client_module.validate_mujoco_gate_response(
        response,
        '0123456789abcdef01234567',
        80,
    )

    assert not result.ok


def test_validate_mujoco_gate_response_accepts_score_equal_to_threshold():
    response = _passing_response()
    response['score'] = 80

    result = client_module.validate_mujoco_gate_response(
        response,
        '0123456789abcdef01234567',
        80,
    )

    assert result.ok
    assert result.score == pytest.approx(80.0)


def test_validate_mujoco_gate_response_rejects_score_below_threshold():
    response = _passing_response()
    response['score'] = 79.999
    response['failure_code'] = 'MUJOCO_COLLISION'
    response['failure_reason'] = 'aggregate score below policy threshold'

    result = client_module.validate_mujoco_gate_response(
        response,
        '0123456789abcdef01234567',
        80,
    )

    assert not result.ok
    assert result.code == 'MUJOCO_COLLISION'
    assert result.reason == 'aggregate score below policy threshold'


@pytest.mark.parametrize('response', [None, [], 'not-json-object'])
def test_validate_mujoco_gate_response_rejects_malformed_response(response):
    result = client_module.validate_mujoco_gate_response(
        response,
        '0123456789abcdef01234567',
        80,
    )

    assert not result.ok
    assert result.code == 'WSL_UNAVAILABLE'


if __name__ == '__main__':
    unittest.main()
