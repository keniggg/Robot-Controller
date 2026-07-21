#!/usr/bin/env python3
from copy import deepcopy
import pathlib
import sys
import types

import pytest
from geometry_msgs.msg import Pose
from alicia_flexible_grasp_supervisor.msg import Grasp6DPlan


ROOT = pathlib.Path(__file__).resolve().parents[1]
for path in (ROOT, ROOT / 'src'):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from alicia_flexible_grasp.grasp import rich_plan_integrity as integrity  # noqa: E402
from alicia_flexible_grasp.vision.mujoco_digital_twin_client import (  # noqa: E402
    build_mujoco_payload,
)


ARM_JOINT_NAMES = [
    'Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6'
]
GRIPPER_CONFIG = {
    'name': 'Alicia_D_v5_6_gripper_50mm',
    'max_inner_gap_m': 0.050,
}


class FakeStamp:
    def __init__(self, seconds=123.25):
        self.seconds = float(seconds)

    def to_nsec(self):
        return int(round(self.seconds * 1_000_000_000.0))

    def to_sec(self):
        return self.seconds


def _pose(x=0.0):
    pose = Pose()
    pose.position.x = float(x)
    pose.position.y = -0.02
    pose.position.z = 0.20
    pose.orientation.w = 1.0
    return pose


def valid_plan(**overrides):
    stamp = FakeStamp()
    geometry = types.SimpleNamespace(
        header=types.SimpleNamespace(frame_id='base_link', stamp=stamp),
        valid=True,
        label='object',
        source_mode='instance_mask',
        pose_base=_pose(0.30),
        size_xyz_m=types.SimpleNamespace(x=0.20, y=0.10, z=0.08),
        support_normal_base=types.SimpleNamespace(x=0.0, y=0.0, z=1.0),
        support_offset_m=0.0,
    )
    values = {
        'header': types.SimpleNamespace(frame_id='base_link', stamp=stamp),
        'valid': True,
        'score': 0.91,
        'poses': [_pose(value) for value in (0.20, 0.25, 0.30, 0.30)],
        'candidate_source': 'graspnet',
        'candidate_source_lineage': ['graspnet'],
        'has_candidate_model_width': True,
        'candidate_width_m': 0.039,
        'required_open_width_m': 0.044,
        'object_geometry': geometry,
        'model_choice': 'object_segment',
        'plan_id': '0123456789abcdef01234567',
    }
    values.update(overrides)
    return types.SimpleNamespace(**values)


def test_ros_message_exposes_candidate_source_contract_fields():
    plan = Grasp6DPlan()

    assert hasattr(plan, 'candidate_source')
    assert hasattr(plan, 'candidate_source_lineage')
    assert hasattr(plan, 'has_candidate_model_width')


def test_plan_id_binds_candidate_source_and_lineage():
    plan = valid_plan()
    original = integrity.compute_plan_id(plan)

    plan.candidate_source_lineage = ['graspnet', 'tabletop_geometry']

    assert integrity.compute_plan_id(plan) != original


def test_plan_id_binds_candidate_model_width_presence_and_value():
    graspnet = valid_plan()
    changed_width = deepcopy(graspnet)
    changed_width.candidate_width_m = 0.040
    geometry = valid_plan(
        candidate_source='tabletop_geometry',
        candidate_source_lineage=['tabletop_geometry'],
        has_candidate_model_width=False,
        candidate_width_m=0.0,
    )

    assert integrity.compute_plan_id(changed_width) != integrity.compute_plan_id(
        graspnet
    )
    assert integrity.compute_plan_id(geometry) != integrity.compute_plan_id(
        graspnet
    )


def test_geometry_plan_encodes_absent_model_width_without_fabricating_one():
    plan = valid_plan(
        candidate_source='tabletop_geometry',
        candidate_source_lineage=['tabletop_geometry'],
        has_candidate_model_width=False,
        candidate_width_m=0.0,
    )

    assert integrity.validate_candidate_model_width(plan) is None
    payload = build_mujoco_payload(
        plan,
        ARM_JOINT_NAMES,
        [0.0] * len(ARM_JOINT_NAMES),
        GRIPPER_CONFIG,
    )

    assert payload['schema_version'] == 3
    assert payload['candidate_source'] == 'tabletop_geometry'
    assert payload['candidate_source_lineage'] == ['tabletop_geometry']
    assert payload['candidate_width_m'] is None
    assert payload['object_model']['type'] == 'obb_box'


@pytest.mark.parametrize(
    ('source', 'lineage'),
    (
        ('', []),
        ('unknown', ['unknown']),
        ('graspnet', []),
        ('tabletop_geometry', ['graspnet']),
        ('graspnet', ['tabletop_geometry', 'graspnet']),
        ('graspnet', ['graspnet', 'graspnet']),
    ),
)
def test_invalid_candidate_source_fails_closed(source, lineage):
    with pytest.raises(ValueError):
        integrity.validate_candidate_source(source, lineage)


@pytest.mark.parametrize(
    'plan',
    (
        valid_plan(has_candidate_model_width=False, candidate_width_m=0.0),
        valid_plan(has_candidate_model_width=True, candidate_width_m=0.0),
        valid_plan(candidate_width_m=float('nan')),
        valid_plan(
            candidate_source='tabletop_geometry',
            candidate_source_lineage=['tabletop_geometry'],
            has_candidate_model_width=True,
            candidate_width_m=0.039,
        ),
        valid_plan(
            candidate_source='tabletop_geometry',
            candidate_source_lineage=['tabletop_geometry'],
            has_candidate_model_width=False,
            candidate_width_m=0.001,
        ),
    ),
)
def test_mismatched_candidate_source_and_model_width_fail_closed(plan):
    with pytest.raises(ValueError):
        integrity.validate_candidate_model_width(plan)
