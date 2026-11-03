"""Replay two actual empty closes; no ROS node, publisher or hardware I/O."""
from copy import deepcopy
from dataclasses import asdict, replace
import hashlib
import json
from pathlib import Path
import sys

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))

from alicia_flexible_grasp.grasp.gripper_geometry import (
    ANALYTICAL_FINGER_SIZE_XYZ_M,
    ANALYTICAL_PALM_SIZE_XYZ_M,
    GripperGeometry,
    evaluate_bilateral_surface_evidence_and_bounds,
)
from alicia_flexible_grasp.grasp.measured_contact import evaluate_measured_contact
from alicia_flexible_grasp.robot.observation_path_guard import SerialUrdfFk
from alicia_flexible_grasp.vision.multiview_surface import FusedTargetSurface
from alicia_flexible_grasp.vision.target_observation import TargetTrackIdentity


@pytest.fixture(scope='module')
def fixture():
    return json.loads((ROOT / 'tests/fixtures/measured_contact_20260926.json').read_text())


@pytest.fixture
def gripper():
    return GripperGeometry(.05, .002, ANALYTICAL_FINGER_SIZE_XYZ_M,
                           ANALYTICAL_PALM_SIZE_XYZ_M, .003)


def check(record, gripper, actual=None, **changes):
    values = deepcopy(record['frozen_contact'])
    values.update(changes)
    return evaluate_measured_contact(
        gripper=gripper,
        actual_T_base_tool0=(record['actual_T_base_tool0'] if actual is None else actual),
        **values,
    )


@pytest.mark.parametrize('name', ['gen11', 'gen18'])
def test_recorded_fk_counts_and_measured_surface_reproduce_frozen_evidence(fixture, gripper, name):
    record = fixture['records'][name]
    model = fixture['robot_description']
    assert hashlib.sha256(model.encode()).hexdigest() == fixture['model_sha256']
    fk = SerialUrdfFk(model, fixture['joint_names'])
    q = (np.asarray(record['measured_counts']) - 2048) * (2. * np.pi / 4096)
    np.testing.assert_allclose(fk(q), record['actual_T_base_tool0'], atol=1e-12)
    geometry = record['frozen_object_geometry']
    points = np.array(geometry['object_points_base_m'])
    identity = TargetTrackIdentity(**record['identity'])
    surface = FusedTargetSurface(identity, points, np.zeros(len(points), dtype=np.int64),
                                 (record['snapshot_stamp_ns'],))
    frozen = record['frozen_contact']
    planned = np.asarray(frozen['planned_T_base_tool0'])
    evidence, bounds = evaluate_bilateral_surface_evidence_and_bounds(
        surface, frozen['contact_center_base'], planned[:3, 1], planned[:3, 2],
        gripper, 12, support_normal_base=frozen['support_normal_base'],
    )
    actual = asdict(evidence)
    expected = record['bilateral_surface_evidence']
    assert actual.keys() == expected.keys()
    for key, value in expected.items():
        # Replay may differ by a last-bit BLAS operation. This tolerance is
        # sub-femtometre; identifiers/counts and physical admission stay exact.
        if isinstance(value, float):
            assert actual[key] == pytest.approx(value, rel=0., abs=1e-15)
        else:
            assert actual[key] == value
    np.testing.assert_allclose(bounds, frozen['contact_height_bounds_m'], atol=1e-12)


@pytest.mark.parametrize('name', ['gen11', 'gen18'])
def test_actual_empty_close_is_rejected_despite_old_endpoint_tolerance_pass(fixture, gripper, name):
    record = fixture['records'][name]
    old = record['endpoint_report']
    assert old['code'] == 'PREGRASP_MEASURED_CONVERGED'
    assert old['steps_completed'] == 0
    assert old['position_error_m'] < .010
    assert old['orientation_error_rad'] < np.deg2rad(5)
    result = check(record, gripper)
    assert not result.ok
    assert result.code == 'MEASURED_CONTACT_PATCH_MISS'
    expected = record['expected_actual_result']
    assert result.actual_overlap_m == pytest.approx(expected['actual_overlap_m'], abs=1e-12)
    assert result.planned_overlap_m == pytest.approx(expected['planned_overlap_m'], abs=1e-12)
    assert result.minimum_support_clearance_m == pytest.approx(
        expected['minimum_support_clearance_m'], abs=1e-12)
    assert result.actual_overlap_m < result.minimum_contact_overlap_m
    assert fixture['physical_grasp_success'] is False


@pytest.mark.parametrize('name', ['gen11', 'gen18'])
def test_original_planned_pose_passes_same_frozen_requirement(fixture, gripper, name):
    record = fixture['records'][name]
    result = check(record, gripper, record['frozen_contact']['planned_T_base_tool0'])
    assert result.ok
    assert result.code == 'MEASURED_CONTACT_GEOMETRY_OK'
    assert result.minimum_contact_overlap_m == .002
    assert result.actual_overlap_m == pytest.approx(
        record['expected_planned_result']['actual_overlap_m'], abs=1e-12)
    assert 'physical grip unverified' in result.reason


def test_sufficient_contact_does_not_waive_support_clearance(fixture, gripper):
    record = fixture['records']['gen18']
    actual = np.asarray(record['frozen_contact']['planned_T_base_tool0']).copy()
    actual[:3, 3] -= .001 * np.asarray(record['frozen_contact']['support_normal_base'])
    result = check(record, gripper, actual)
    assert result.actual_overlap_m >= .002
    assert not result.ok
    assert result.code == 'MEASURED_CONTACT_SUPPORT_CLEARANCE'


def test_invalid_planned_band_cannot_be_repaired_by_actual_pose(fixture, gripper):
    record = fixture['records']['gen18']
    result = check(record, gripper, minimum_contact_overlap_m=.003)
    assert not result.ok
    assert result.code == 'FROZEN_CONTACT_EVIDENCE_INVALID'


@pytest.mark.parametrize('change', [
    {'minimum_contact_overlap_m': 0.0},
    {'minimum_contact_overlap_m': float('nan')},
    {'contact_height_bounds_m': [.002, -.002]},
    {'contact_height_bounds_m': [.002, .002]},
    {'contact_height_bounds_m': [0., float('inf')]},
    {'contact_center_base': [0., 0., float('nan')]},
    {'support_normal_base': [0., 0., 2.]},
    {'support_offset_m': float('nan')},
    {'opening_width_m': .051},
    {'opening_width_m': -.001},
])
def test_invalid_contact_evidence_is_rejected(fixture, gripper, change):
    result = check(fixture['records']['gen18'], gripper, **change)
    assert not result.ok
    assert result.code == 'MEASURED_CONTACT_INPUT_INVALID'
    # No NaN/Infinity diagnostics are emitted on invalid input.
    json.dumps(asdict(result), allow_nan=False)


def test_height_axis_cannot_silently_change_with_actual_tool(fixture, gripper):
    record = fixture['records']['gen18']
    actual_axis = np.asarray(record['actual_T_base_tool0'])[:3, 2]
    result = check(record, gripper, contact_height_axis_base=actual_axis)
    assert result.code == 'MEASURED_CONTACT_INPUT_INVALID'


@pytest.mark.parametrize('mutation', ['reflection', 'nonhomogeneous', 'nan'])
def test_actual_pose_must_be_a_finite_rigid_transform(fixture, gripper, mutation):
    record = fixture['records']['gen18']
    actual = np.array(record['actual_T_base_tool0'])
    if mutation == 'reflection':
        actual[:3, 0] *= -1
    elif mutation == 'nonhomogeneous':
        actual[3, 0] = .001
    else:
        actual[0, 3] = float('nan')
    assert check(record, gripper, actual).code == 'MEASURED_CONTACT_INPUT_INVALID'


def test_collision_model_cannot_shrink_to_make_contact_pass(fixture, gripper):
    smaller = replace(gripper, finger_size_xyz_m=gripper.finger_size_xyz_m * .5)
    assert check(fixture['records']['gen18'], smaller).code == 'MEASURED_CONTACT_INPUT_INVALID'
