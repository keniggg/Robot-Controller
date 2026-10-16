#!/usr/bin/env python3
import ast
import importlib.util
import hashlib
import json
import pathlib

import numpy as np
import pytest


ROOT = pathlib.Path(__file__).resolve().parents[1]
TOOL = ROOT / 'tools' / 'benchmark_tabletop_geometry.py'
FIXTURE = ROOT / 'tests' / 'fixtures' / 'carton_tabletop_cloud.json'
SPEC = importlib.util.spec_from_file_location('benchmark_tabletop_geometry', TOOL)
BENCHMARK = importlib.util.module_from_spec(SPEC) if TOOL.is_file() else None
if SPEC is not None and BENCHMARK is not None:
    SPEC.loader.exec_module(BENCHMARK)


def test_tabletop_fixture_benchmark_tool_exists():
    assert TOOL.is_file()


@pytest.mark.parametrize(
    'value',
    (None, 0.0, -1.0, float('nan'), float('inf'), '438.6'),
)
def test_capture_intrinsics_reject_invalid_focal_or_depth_scale(value):
    camera = {
        'fx': 438.6,
        'fy': 438.4,
        'cx': 317.8,
        'cy': 245.4,
        'depth_scale': 0.001,
    }
    camera['fx'] = value

    with pytest.raises(ValueError, match='fx'):
        BENCHMARK.validate_intrinsics(camera)


def test_capture_intrinsics_accept_finite_principal_point():
    intrinsics = BENCHMARK.validate_intrinsics({
        'fx': 438.6,
        'fy': 438.4,
        'cx': 0.0,
        'cy': -2.0,
        'depth_scale': 0.001,
    })

    assert intrinsics == {
        'fx': 438.6,
        'fy': 438.4,
        'cx': 0.0,
        'cy': -2.0,
        'depth_scale': 0.001,
    }


def test_back_projection_converts_16uc1_and_applies_base_transform():
    depth = np.array([[0, 1000], [2000, 3000]], dtype=np.uint16)
    mask = np.array([[255, 255], [0, 255]], dtype=np.uint8)
    transform = np.eye(4)
    transform[:3, 3] = [0.1, -0.2, 0.3]

    points = BENCHMARK.back_project_masked_depth(
        depth,
        mask,
        '16UC1',
        {'fx': 1.0, 'fy': 1.0, 'cx': 0.0, 'cy': 0.0,
         'depth_scale': 0.001},
        transform,
    )

    np.testing.assert_allclose(
        points,
        [[1.1, -0.2, 1.3], [3.1, 2.8, 3.3]],
    )


def test_back_projection_accepts_32fc1_metres_and_rejects_other_encodings():
    depth = np.array([[0.25, np.nan]], dtype=np.float32)
    mask = np.array([[1, 1]], dtype=np.uint8)
    intrinsics = {
        'fx': 2.0, 'fy': 2.0, 'cx': 0.0, 'cy': 0.0,
        'depth_scale': 100.0,
    }

    points = BENCHMARK.back_project_masked_depth(
        depth, mask, '32FC1', intrinsics, np.eye(4)
    )

    np.testing.assert_allclose(points, [[0.0, 0.0, 0.25]])
    with pytest.raises(ValueError, match='encoding'):
        BENCHMARK.back_project_masked_depth(
            depth, mask, '8UC1', intrinsics, np.eye(4)
        )


def test_cleanup_filters_support_and_obb_then_voxels_and_bounds_to_512():
    coords = np.arange(-0.014, 0.0141, 0.002)
    points = np.array(
        [(x, y, z) for x in coords for y in coords
         for z in np.arange(0.004, 0.0141, 0.002)],
        dtype=float,
    )
    points = np.vstack((
        points,
        [[0.0, 0.0, 0.001], [0.016, 0.0, 0.010], [np.nan, 0.0, 0.010]],
    ))

    cleaned = BENCHMARK.clean_target_points(
        points,
        obb_center_base=np.zeros(3),
        R_base_obb=np.eye(3),
        obb_size_xyz_m=np.array([0.020, 0.020, 0.020]),
        support_normal_base=np.array([0.0, 0.0, 1.0]),
        support_offset_m=0.0,
        min_height_m=0.004,
    )

    assert cleaned.shape == (512, 3)
    assert np.all(cleaned[:, 2] >= 0.004)
    assert np.all(np.abs(cleaned) <= 0.015 + 1e-12)
    voxels = np.floor(cleaned / 0.002).astype(np.int64)
    assert len(np.unique(voxels, axis=0)) == len(cleaned)


def test_cleanup_rejects_capture_with_fewer_than_120_points():
    with pytest.raises(ValueError, match='at least 120'):
        BENCHMARK.clean_target_points(
            np.zeros((119, 3)),
            obb_center_base=np.zeros(3),
            R_base_obb=np.eye(3),
            obb_size_xyz_m=np.ones(3),
            support_normal_base=np.array([0.0, 0.0, 1.0]),
            support_offset_m=0.0,
            min_height_m=0.0,
        )


def test_audit_bytes_require_matching_snapshot_and_successful_geometry_row():
    audit_bytes = json.dumps({
        'report_version': 3,
        'snapshot_transform': {'snapshot_stamp_ns': 123456789},
        'rows': [{
            'candidate_source': 'tabletop_geometry',
            'analytical_result': {'ok': True},
        }],
    }, sort_keys=True).encode('utf-8')

    digest = BENCHMARK.validate_planning_audit_bytes(
        audit_bytes, 123456789
    )

    assert digest == hashlib.sha256(audit_bytes).hexdigest()
    with pytest.raises(ValueError, match='snapshot stamp'):
        BENCHMARK.validate_planning_audit_bytes(audit_bytes, 123456788)


def test_audit_bytes_reject_non_strict_json_and_failed_geometry_row():
    with pytest.raises(ValueError, match='strict JSON'):
        BENCHMARK.validate_planning_audit_bytes(
            b'{"snapshot_transform":{"snapshot_stamp_ns":1},'
            b'"rows":[],"value":NaN}',
            1,
        )
    failed = json.dumps({
        'report_version': 3,
        'snapshot_transform': {'snapshot_stamp_ns': 1},
        'rows': [{
            'candidate_source': 'tabletop_geometry',
            'analytical_result': {'ok': False},
        }],
    }).encode('utf-8')
    with pytest.raises(ValueError, match='successful tabletop_geometry'):
        BENCHMARK.validate_planning_audit_bytes(failed, 1)


def test_gate_audit_path_must_be_nonempty():
    with pytest.raises(ValueError, match='non-empty'):
        BENCHMARK.validate_gate_audit_path('   ')


def test_fixture_validator_rejects_extra_keys_and_invalid_rotation():
    with pytest.raises(ValueError, match='exact top-level keys'):
        BENCHMARK.validate_fixture({'unexpected': True})


def test_support_point_is_derived_from_plane_offset():
    point = BENCHMARK.support_point_from_plane(
        np.array([0.0, 0.0, 1.0]), -0.125
    )

    np.testing.assert_allclose(point, [0.0, 0.0, 0.125])


def test_benchmark_uses_production_tabletop_configuration():
    config = BENCHMARK.load_production_tabletop_config()

    assert config.max_inner_gap_m == 0.050
    assert config.angle_step_deg == 5.0
    assert config.angle_dedup_deg == 1.0
    assert config.jaw_clearance_each_side_m == 0.002
    assert config.min_contact_band_points == 6
    assert config.contact_band_fraction == 0.12
    assert config.max_candidates == 32
    assert config.approach_tilt_degrees == ()


def test_benchmark_runs_ten_warmups_then_requested_timed_iterations():
    calls = []
    ticks = iter([0.000, 0.010, 0.010, 0.040, 0.040, 0.060])

    median_ms, p95_ms = BENCHMARK.benchmark_generator(
        {'fixture': True},
        iterations=3,
        warmups=10,
        generate=lambda fixture: calls.append(fixture),
        clock=lambda: next(ticks),
    )

    assert len(calls) == 13
    assert median_ms == pytest.approx(20.0)
    assert p95_ms == pytest.approx(29.0)


def test_cli_requires_exactly_one_fixture_mode():
    with pytest.raises(SystemExit):
        BENCHMARK.parse_args([])
    with pytest.raises(SystemExit):
        BENCHMARK.parse_args([
            '--fixture', 'input.json',
            '--dump-tabletop-fixture', 'output.json',
        ])


def test_capture_always_stops_preview_after_capture_failure(tmp_path):
    class FailingRuntime:
        def __init__(self):
            self.triggers = []

        def set_preview(self, trigger):
            self.triggers.append(trigger)

        def capture(self, timeout_sec):
            assert timeout_sec == 7.0
            raise RuntimeError('synthetic capture failure')

    runtime = FailingRuntime()

    with pytest.raises(RuntimeError, match='synthetic capture failure'):
        BENCHMARK.capture_tabletop_fixture(
            tmp_path / 'must-not-exist.json',
            timeout_sec=7.0,
            runtime=runtime,
        )

    assert runtime.triggers == [True, False]
    assert not (tmp_path / 'must-not-exist.json').exists()


def test_geometry_and_depth_stamps_must_match_within_sync_slop():
    BENCHMARK.require_stamp_match(1_000_000_000, 1_079_999_999)
    with pytest.raises(ValueError, match='synchronizer tolerance'):
        BENCHMARK.require_stamp_match(1_000_000_000, 1_080_000_001)


def test_pose_quaternion_is_converted_to_base_obb_rotation():
    half_sqrt = np.sqrt(0.5)
    rotation = BENCHMARK.quaternion_xyzw_to_rotation(
        [0.0, 0.0, half_sqrt, half_sqrt]
    )

    np.testing.assert_allclose(
        rotation,
        [[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]],
        atol=1e-12,
    )


def load_fixture():
    if not FIXTURE.is_file():
        pytest.skip(
            'requires a genuine RealSense capture; '
            'do not substitute synthetic points'
        )
    return BENCHMARK.load_fixture(FIXTURE)


def test_realsense_carton_fixture_has_a_width_valid_topdown_candidate():
    fixture = load_fixture()
    result = BENCHMARK.generate_tabletop_proposals_from_fixture(fixture)

    assert result.ok
    best = result.proposals[0]
    assert best.required_open_width_m < 0.050
    assert np.dot(
        best.insertion_axis_base, fixture['support_normal_base']
    ) < -0.99
    assert abs(np.dot(
        best.jaw_axis_base, fixture['support_normal_base']
    )) < 0.01


def test_production_generator_is_bounded():
    fixture = load_fixture()
    result = BENCHMARK.generate_tabletop_proposals_from_fixture(fixture)

    assert len(result.proposals) <= 24


def test_capture_tool_has_only_the_preview_service_interface():
    tree = ast.parse(TOOL.read_text())
    service_names = []
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call) or not isinstance(node.func, ast.Attribute):
            continue
        if node.func.attr != 'ServiceProxy' or not node.args:
            continue
        if isinstance(node.args[0], ast.Constant):
            service_names.append(node.args[0].value)

    assert service_names == ['/grasp_6d/request_plan']
    source = TOOL.read_text()
    for forbidden in (
        '/grasp/start',
        '/grasp/stop',
        '/supervisor/move_to_joints',
        '/supervisor/cartesian_jog',
        '/supervisor/torque',
        '/grasp/execute',
    ):
        assert forbidden not in source
