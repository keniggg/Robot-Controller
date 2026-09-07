#!/usr/bin/env python3
from copy import deepcopy
import io
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
        target_track_id='g0-t1',
        source_mode='instance_mask',
        pose_base=_pose(0.30),
        size_xyz_m=types.SimpleNamespace(x=0.20, y=0.10, z=0.08),
        support_normal_base=types.SimpleNamespace(x=0.0, y=0.0, z=1.0),
        support_offset_m=0.0,
    )
    values = {
        'header': types.SimpleNamespace(frame_id='base_link', stamp=stamp),
        'valid': True,
        'target_track_id': 'g0-t1',
        'refinement_status': 'NOT_EVALUATED',
        'refinement_inlier_count': 0,
        'refinement_overlap_fraction': 0.0,
        'refinement_rmse_m': 0.0,
        'refinement_translation_m': 0.0,
        'refinement_rotation_deg': 0.0,
        'refinement_source_clipped': False,
        'fused_view_count': 0,
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


def valid_refined_ros_plan():
    """Build a wire-serializable plan with measured 3D refinement evidence."""
    plan = Grasp6DPlan()
    plan.header.frame_id = 'base_link'
    plan.header.stamp.secs = 123
    plan.header.stamp.nsecs = 250_000_000
    plan.valid = True
    plan.target_track_id = 'g7-t42'
    plan.refinement_status = 'VALID_3D'
    plan.refinement_inlier_count = 80
    plan.refinement_overlap_fraction = 0.6173
    plan.refinement_rmse_m = 0.0037
    plan.refinement_translation_m = 0.0123
    plan.refinement_rotation_deg = 7.31
    plan.refinement_source_clipped = True
    plan.fused_view_count = 3
    plan.score = 0.91
    plan.poses = [_pose(value) for value in (0.20, 0.25, 0.30, 0.30)]
    plan.candidate_source = 'graspnet'
    plan.candidate_source_lineage = ['graspnet']
    plan.has_candidate_model_width = True
    plan.candidate_width_m = 0.039
    plan.required_open_width_m = 0.044
    plan.model_choice = 'object_segment'

    geometry = plan.object_geometry
    geometry.header.frame_id = plan.header.frame_id
    geometry.header.stamp = plan.header.stamp
    geometry.valid = True
    geometry.label = 'object'
    geometry.target_track_id = plan.target_track_id
    geometry.source_mode = 'instance_mask'
    geometry.pose_base = _pose(0.30)
    geometry.size_xyz_m.x = 0.20
    geometry.size_xyz_m.y = 0.10
    geometry.size_xyz_m.z = 0.08
    geometry.support_normal_base.z = 1.0
    geometry.support_offset_m = 0.0137
    plan.plan_id = integrity.compute_plan_id(plan)
    return plan


def neutral_refinement_ros_plan():
    plan = valid_refined_ros_plan()
    plan.refinement_status = 'NOT_EVALUATED'
    plan.refinement_inlier_count = 0
    plan.refinement_overlap_fraction = 0.0
    plan.refinement_rmse_m = 0.0
    plan.refinement_translation_m = 0.0
    plan.refinement_rotation_deg = 0.0
    plan.refinement_source_clipped = False
    plan.fused_view_count = 0
    plan.plan_id = integrity.compute_plan_id(plan)
    return plan


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


@pytest.mark.parametrize(
    ('field', 'changed_value'),
    (
        ('target_track_id', 'g7-t43'),
        ('refinement_status', 'INVALID_3D'),
        ('refinement_inlier_count', 81),
        ('refinement_overlap_fraction', 0.7139),
        ('refinement_rmse_m', 0.0039),
        ('refinement_translation_m', 0.0149),
        ('refinement_rotation_deg', 8.27),
        ('refinement_source_clipped', False),
        ('fused_view_count', 4),
    ),
)
def test_plan_id_binds_track_and_each_refinement_authority_field(
        field, changed_value):
    plan = valid_refined_ros_plan()
    original = plan.plan_id

    setattr(plan, field, changed_value)
    if field == 'target_track_id':
        plan.object_geometry.target_track_id = changed_value

    assert integrity.compute_plan_id(plan) != original


def test_valid_3d_plan_id_survives_ros_wire_round_trip():
    plan = valid_refined_ros_plan()
    wire = io.BytesIO()

    plan.serialize(wire)
    received = Grasp6DPlan()
    received.deserialize(wire.getvalue())

    assert received.refinement_status == 'VALID_3D'
    assert integrity.validate_refinement_evidence(received)[0] == 'VALID_3D'
    assert received.plan_id == plan.plan_id
    assert integrity.compute_plan_id(received) == plan.plan_id
    for field in (
        'refinement_overlap_fraction',
        'refinement_rmse_m',
        'refinement_translation_m',
        'refinement_rotation_deg',
    ):
        assert getattr(received, field) == integrity.float32_wire_value(
            getattr(plan, field)
        )


INVALID_REFINEMENT_CASES = (
    pytest.param('refined', 'refinement_status', 'VALID', id='unsupported-status'),
    *(
        pytest.param('refined', field, value, id='%s-%s' % (field, label))
        for field in (
            'refinement_overlap_fraction',
            'refinement_rmse_m',
            'refinement_translation_m',
            'refinement_rotation_deg',
        )
        for label, value in (
            ('nan', float('nan')),
            ('positive-inf', float('inf')),
            ('negative', -0.001),
        )
    ),
    pytest.param(
        'refined', 'refinement_overlap_fraction', 1.01,
        id='overlap-greater-than-one',
    ),
    pytest.param(
        'refined', 'refinement_rotation_deg', 180.01,
        id='rotation-greater-than-180',
    ),
    pytest.param(
        'refined', 'refinement_inlier_count', 0,
        id='valid-3d-zero-inliers',
    ),
    pytest.param(
        'refined', 'refinement_overlap_fraction', 0.0,
        id='valid-3d-zero-overlap',
    ),
    pytest.param(
        'refined', 'fused_view_count', 0,
        id='valid-3d-zero-views',
    ),
    *(
        pytest.param('neutral', field, value, id='not-evaluated-%s' % field)
        for field, value in (
            ('refinement_inlier_count', 1),
            ('refinement_overlap_fraction', 0.1),
            ('refinement_rmse_m', 0.001),
            ('refinement_translation_m', 0.001),
            ('refinement_rotation_deg', 1.0),
            ('refinement_source_clipped', True),
            ('fused_view_count', 1),
        )
    ),
)


@pytest.mark.parametrize(
    ('starting_state', 'field', 'invalid_value'),
    INVALID_REFINEMENT_CASES,
)
def test_invalid_refinement_authority_fails_closed(
        starting_state, field, invalid_value):
    plan = (
        neutral_refinement_ros_plan()
        if starting_state == 'neutral'
        else valid_refined_ros_plan()
    )
    setattr(plan, field, invalid_value)

    with pytest.raises(ValueError):
        integrity.compute_plan_id(plan)
    assert not integrity.plan_id_matches_content(plan)


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
