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
    plan.candidate_source = 'graspnet'
    plan.candidate_source_lineage = ['graspnet']
    plan.has_candidate_model_width = True
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
    response = {
        'plan_id': plan_id,
        'candidate_source': 'graspnet',
        'candidate_source_lineage': ['graspnet'],
        'score': 92.5,
    }
    response.update({key: True for key in SAFETY_KEYS})
    response['lift_evidence'] = {
        'contract_version': 1,
        'object_lift_m': 0.020,
        'commanded_lift_m': 0.050,
        'minimum_lift_m': 0.015,
        'two_sided_lift_samples': 39,
        'lost_contact_samples': 1,
        'lift_sample_count': 40,
        'max_lost_contact_streak': 1,
        'contact_loss_grace_samples': 3,
    }
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


def test_build_mujoco_payload_serializes_exact_v3_rich_plan_schema():
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
        'candidate_source',
        'candidate_source_lineage',
        'joint_names',
        'joint_positions',
        'trajectory',
        'candidate_width_m',
        'required_open_width_m',
        'gripper',
        'object_model',
        'support_plane',
    }
    assert payload['schema_version'] == 3
    assert payload['plan_id'] == plan.plan_id
    assert payload['snapshot_stamp_sec'] == pytest.approx(123.456789)
    assert payload['model_choice'] == plan.model_choice
    assert payload['candidate_source'] == 'graspnet'
    assert payload['candidate_source_lineage'] == ['graspnet']
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
        'close_target_inner_gap_m': 0.0,
        'close_settle_sec': 0.8,
    }
    assert payload['object_model']['type'] == 'obb_box'
    assert payload['object_model']['pose_base']['position_m'] == pytest.approx(
        [0.30, -0.04, 0.12]
    )
    assert payload['object_model']['size_xyz_m'] == pytest.approx(
        [0.24, 0.16, 0.10]
    )
    assert payload['object_model']['mass_kg'] == pytest.approx(0.50)
    assert payload['object_model']['friction'] == pytest.approx(
        [0.10, 0.005, 0.0001]
    )
    assert payload['object_model']['dynamics_assumptions']['source'] == (
        'live_obb_conservative_envelope'
    )
    assert payload['support_plane']['normal_base'] == pytest.approx(
        [0.0, 0.0, 1.0]
    )
    assert payload['support_plane']['offset_m'] == pytest.approx(-0.02)
    json.dumps(payload, allow_nan=False)


def test_build_mujoco_payload_serializes_geometry_width_as_json_null():
    plan = _rich_plan()
    plan.candidate_source = 'tabletop_geometry'
    plan.candidate_source_lineage = ['tabletop_geometry']
    plan.has_candidate_model_width = False
    plan.candidate_width_m = 0.0

    payload = client_module.build_mujoco_payload(
        plan,
        ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6'],
        [0.0] * 6,
    )

    assert payload['candidate_width_m'] is None
    assert payload['candidate_source'] == 'tabletop_geometry'


@pytest.mark.parametrize(
    'mutation',
    (
        lambda plan: setattr(plan, 'has_candidate_model_width', False),
        lambda plan: setattr(plan, 'candidate_width_m', 0.0),
        lambda plan: setattr(plan, 'candidate_source_lineage', []),
        lambda plan: (
            setattr(plan, 'candidate_source', 'tabletop_geometry'),
            setattr(plan, 'candidate_source_lineage', ['tabletop_geometry']),
        ),
    ),
)
def test_build_mujoco_payload_rejects_source_width_mismatches(mutation):
    plan = _rich_plan()
    mutation(plan)

    with pytest.raises(ValueError):
        client_module.build_mujoco_payload(
            plan,
            ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6'],
            [0.0] * 6,
        )


@pytest.mark.parametrize('object_type', ['carton_box', 'mouse_compound'])
def test_build_mujoco_payload_rejects_category_specific_object_types(object_type):
    with pytest.raises(ValueError, match='obb_box'):
        client_module.build_mujoco_payload(
            _rich_plan(),
            ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6'],
            [0.0] * 6,
            {'object_model': {'type': object_type}},
        )


@pytest.mark.parametrize(
    'legacy',
    (
        {'mass_kg': 0.08},
        {'friction': [1.2, 0.08, 0.02]},
    ),
)
def test_build_mujoco_payload_rejects_fixed_object_dynamics(legacy):
    with pytest.raises(ValueError, match='category-specific'):
        client_module.build_mujoco_payload(
            _rich_plan(),
            ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6'],
            [0.0] * 6,
            {'object_model': {'type': 'obb_box', **legacy}},
        )


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
        expected_candidate_source='graspnet',
        expected_candidate_source_lineage=['graspnet'],
    )

    assert result.ok
    assert result.code == ''
    assert result.reason == ''
    assert result.score == pytest.approx(92.5)


def test_validate_mujoco_gate_response_requires_structured_dynamic_lift():
    response = _passing_response()
    response.pop('lift_evidence')

    result = client_module.validate_mujoco_gate_response(
        response,
        '0123456789abcdef01234567',
        80,
    )

    assert not result.ok
    assert result.code == 'WSL_UNAVAILABLE'


@pytest.mark.parametrize(
    ('field', 'value', 'expected_code'),
    (
        ('object_lift_m', 0.0, 'MUJOCO_LIFT_FAILED'),
        ('lost_contact_samples', 4, 'MUJOCO_CONTACT_FAILED'),
        ('max_lost_contact_streak', 4, 'MUJOCO_CONTACT_FAILED'),
    ),
)
def test_validate_mujoco_gate_response_rejects_weak_lift_evidence(
    field,
    value,
    expected_code,
):
    response = _passing_response()
    response['lift_evidence'][field] = value
    if field == 'lost_contact_samples':
        response['lift_evidence']['two_sided_lift_samples'] = 36

    result = client_module.validate_mujoco_gate_response(
        response,
        '0123456789abcdef01234567',
        80,
    )

    assert not result.ok
    assert result.code == expected_code


@pytest.mark.parametrize(
    'mutation',
    (
        lambda response: response.pop('candidate_source'),
        lambda response: response.update(candidate_source='tabletop_geometry'),
        lambda response: response.update(candidate_source_lineage=[]),
        lambda response: response.update(
            candidate_source_lineage=['graspnet', 'tabletop_geometry']
        ),
    ),
)
def test_validate_mujoco_gate_response_rejects_mismatched_provenance(mutation):
    response = _passing_response()
    mutation(response)

    result = client_module.validate_mujoco_gate_response(
        response,
        '0123456789abcdef01234567',
        80,
        expected_candidate_source='graspnet',
        expected_candidate_source_lineage=['graspnet'],
    )

    assert not result.ok
    assert result.code == 'CANDIDATE_SOURCE_MISMATCH'


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


def test_validate_mujoco_gate_response_preserves_preflight_failure_without_provenance():
    response = _passing_response()
    response.pop('candidate_source')
    response.pop('candidate_source_lineage')
    response.update({key: False for key in SAFETY_KEYS})
    response['score'] = 0.0
    response['failure_code'] = 'PLAN_INVALID'
    response['failure_reason'] = 'schema_version must be 2'

    result = client_module.validate_mujoco_gate_response(
        response,
        '0123456789abcdef01234567',
        80,
        expected_candidate_source='tabletop_geometry',
        expected_candidate_source_lineage=['tabletop_geometry'],
    )

    assert not result.ok
    assert result.code == 'PLAN_INVALID'
    assert result.reason == 'schema_version must be 2'


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


@pytest.mark.parametrize(
    'nonfinite',
    [float('nan'), float('inf'), float('-inf')],
    ids=('nan', 'positive-infinity', 'negative-infinity'),
)
def test_validate_mujoco_gate_response_rejects_nonfinite_hidden_extra(
    nonfinite,
):
    response = _passing_response()
    response['unexpected_diagnostic'] = nonfinite

    result = client_module.validate_mujoco_gate_response(
        response,
        '0123456789abcdef01234567',
        80,
    )

    assert not result.ok
    assert result.code == 'WSL_UNAVAILABLE'
    assert 'strict-JSON' in result.reason


@pytest.mark.parametrize(
    'constant',
    ('NaN', 'Infinity', '-Infinity'),
)
def test_http_client_rejects_nonstandard_json_constants(monkeypatch, constant):
    class FakeResponse:
        def __enter__(self):
            return self

        def __exit__(self, *_args):
            return False

        def read(self):
            return (
                '{"plan_id":"plan","score":95,'
                '"simulation_ok":true,"ik_success":true,'
                '"collision_free":true,"contact_success":true,'
                '"lift_success":true,"unexpected":%s}' % constant
            ).encode('utf-8')

    monkeypatch.setattr(
        client_module.urllib.request,
        'urlopen',
        lambda *_args, **_kwargs: FakeResponse(),
    )
    client = client_module.MujocoDigitalTwinClient(
        'http://127.0.0.1:8000',
        timeout_sec=0.01,
    )

    with pytest.raises(ValueError, match='non-standard JSON constant'):
        client.simulate_grasp({'schema_version': 3})


if __name__ == '__main__':
    unittest.main()


def _operator_mass_case():
    plan = _rich_plan()
    plan.model_choice = 'unknown_tabletop'
    plan.target_track_id = 'g4-t18'
    evidence = dict(source='operator_estimate', operator_statement='重量为10g左右',
        evidence_sha256='a'*64, plan_id=plan.plan_id, target_track_id=plan.target_track_id,
        snapshot_stamp_ns=str(plan.header.stamp.to_nsec()), estimated_mass_kg=.01)
    return plan, dict(target_mass_evidence=evidence)


def test_target_mass_uses_twice_operator_estimate_without_altering_other_contracts():
    plan, config = _operator_mass_case()
    default = client_module.build_mujoco_payload(plan, ['j'], [0.])
    actual = client_module.build_mujoco_payload(plan, ['j'], [0.], config)
    assert actual['object_model']['mass_kg'] == pytest.approx(.02)
    assert actual['object_model']['friction'] == default['object_model']['friction']
    assumptions = actual['object_model']['dynamics_assumptions']
    assert assumptions['source'] == 'plan_bound_operator_mass_estimate'
    assert not assumptions['measured_upper_bound']
    assert assumptions['estimate_uncertainty_multiplier'] == 2.
    assert actual['gripper'] == default['gripper']
    assert actual['trajectory'] == default['trajectory']
    assert actual['object_model']['size_xyz_m'] == default['object_model']['size_xyz_m']
    assert 'mass_kg' not in config


@pytest.mark.parametrize('field,value', [('plan_id','other'),('target_track_id','g4-t19'),
    ('snapshot_stamp_ns','123'),('source','default'),('evidence_sha256',''),
    ('estimated_mass_kg',float('nan')),('estimated_mass_kg',0),('estimated_mass_kg',True),
    ('estimated_mass_kg',.26)])
def test_operator_mass_cannot_cross_plan_or_exceed_operating_range(field,value):
    plan,config = _operator_mass_case()
    config['target_mass_evidence'][field] = value
    with pytest.raises((ValueError,TypeError)):
        client_module.build_mujoco_payload(plan,['j'],[0.],config)


def test_carton_does_not_accept_unknown_operator_mass_override():
    plan, config = _operator_mass_case()
    plan.model_choice = 'carton_segment'
    with pytest.raises(ValueError):
        client_module.build_mujoco_payload(plan,['j'],[0.],config)


# Captured from WSL MuJoCo 3.2.3: loaded opposed closure succeeds, lift loses it.
def _recorded_lift_failure():
    import json
    return json.loads((ROOT / 'tests/fixtures/mujoco_lift_contact_loss.json').read_text())


def _validate_recorded(response, **options):
    return client_module.validate_mujoco_gate_response(
        response, '14357aa4286c1b2b7eb8b03f', 80,
        expected_candidate_source='tabletop_geometry',
        expected_candidate_source_lineage=['tabletop_geometry'], **options)


def test_lift_diagnostic_policy_preserves_actual_wsl_failure_and_can_be_reenabled():
    from copy import deepcopy
    response = _recorded_lift_failure()
    original = deepcopy(response)
    assert not _validate_recorded(response).ok
    result = _validate_recorded(response, require_lift_success=False)
    assert result.ok
    assert result.code == 'MUJOCO_LIFT_DIAGNOSTIC_ONLY'
    assert result.score == 55.0
    assert response == original
    assert not _validate_recorded(response, require_lift_success=True).ok


@pytest.mark.parametrize('change', [
    lambda r: r.update(ik_success=False),
    lambda r: r.update(collision_free=False),
    lambda r: r.update(plan_id='other-plan'),
    lambda r: r.update(candidate_source='graspnet', candidate_source_lineage=['graspnet']),
    lambda r: r.update(score=float('nan')),
    lambda r: r.update(failure_code='SNAPSHOT_TOO_OLD'),
    lambda r: r.update(simulation_ok='false'),
    lambda r: r['lift_evidence'].pop('settled_gripper_state'),
    lambda r: r['lift_evidence'].update(preload_settle_samples=0),
    lambda r: r['lift_evidence'].update(lift_sample_count=0),
    lambda r: r['lift_evidence'].update(max_prescribed_joint_speed_rad_s=0.081),
    lambda r: r['lift_evidence']['settled_gripper_state'].update(left_object_normal_force_n=0),
    lambda r: r['lift_evidence']['settled_gripper_state'].update(finger_object_contacts=[]),
    lambda r: r['lift_evidence']['settled_gripper_state']['finger_object_contacts'][0].update(normal_base=[0, 0, 1]),
    lambda r: r['ik_results'][0].update(orientation_error=0.19),
])
def test_lift_diagnostic_policy_never_bypasses_non_lift_failures(change):
    response = _recorded_lift_failure()
    change(response)
    assert not _validate_recorded(response, require_lift_success=False).ok


@pytest.mark.parametrize('value', ['false', 0, None])
def test_lift_policy_requires_explicit_boolean(value):
    result = _validate_recorded(_recorded_lift_failure(), require_lift_success=value)
    assert not result.ok
    assert result.code == 'MUJOCO_GATE_CONFIG_INVALID'


def test_lift_policy_only_changes_unknown_object_plans():
    plan = _rich_plan()
    config = {'unknown_lift_gate_enabled': False}
    assert client_module.mujoco_lift_gate_enabled(plan, config) is True
    plan.model_choice = 'unknown_tabletop'
    assert client_module.mujoco_lift_gate_enabled(plan, {}) is True
    assert client_module.mujoco_lift_gate_enabled(plan, config) is False
