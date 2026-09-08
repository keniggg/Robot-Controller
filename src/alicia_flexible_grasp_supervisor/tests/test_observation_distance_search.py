"""Offline distance search using the 2026-09-07 19:06 frozen observation."""

import importlib.util
import pathlib
import sys
import types

import numpy as np
import pytest


ROOT = pathlib.Path(__file__).resolve().parents[1]
for path in (ROOT, ROOT / 'src'):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))
spec = importlib.util.spec_from_file_location(
    'observation_distance_remote', ROOT / 'scripts' / 'remote_grasp6d_node.py'
)
remote = importlib.util.module_from_spec(spec)
spec.loader.exec_module(remote)

# Reduced directly from the real execution audit, source stamp
# 1788833187757686853, SHA256
# 3b1ca8a4da60534f88d91d087cf5d8a697b8f1d322b93ffd3658725641f138c7.
# The original 0.200 m / +90 degree branch required 116.936 s on Joint6.
BASE_CAMERA = np.asarray([
    [-0.13950346895525867, 0.9527993799114034, -0.269651856640918, -0.05757966215926949],
    [-0.7213261033962179, -0.28434347731452225, -0.6315357784543281, -0.21136808685175437],
    [-0.678400644684745, 0.10640549116061038, 0.7269459654900023, 0.29612760968668184],
    [0.0, 0.0, 0.0, 1.0],
])
OBB_AXES = np.asarray([
    [-0.9881784814520107, -0.1528179510139203, -0.012245923530107953],
    [0.1491533213143756, -0.9767973211001011, 0.15368890731785634],
    [-0.03544820920848924, 0.15004555088230062, 0.9880433984012738],
])


def recorded_scene(rolls=(0.0,)):
    # Construct definitions only: no ROS node, service, publisher or TF lookup.
    node = remote.RemoteGrasp6DNode.__new__(remote.RemoteGrasp6DNode)
    node.grasp_config = {
        'observation_camera_target_nominal_distance_m': 0.200,
        'observation_camera_target_min_distance_m': 0.180,
        'observation_camera_target_max_distance_m': 0.220,
    }
    node.gripper_geometry = remote.GripperGeometry(
        0.05, 0.002, [0.0434, 0.0286, 0.0600],
        [0.1175, 0.1550, 0.0774], 0.003,
    )
    node.gripper_tool_jaw_axis = 'y'
    node.gripper_tool_finger_length_axis = 'z'
    node.gripper_physical_open_width_m = 0.05
    node.observation_envelope_gate_enabled = True
    node.observation_camera_roll_offsets_deg = rolls
    tool_camera = remote.quaternion_matrix([
        0.005929097277682129, -0.7167581240677906,
        0.018061735450252286, 0.6970627024169479,
    ])
    tool_camera[:3, 3] = [
        -0.08327671107922277, 0.004843742371779117,
        -0.12907560357461276,
    ]
    node._tool_from_camera_matrix = lambda: tool_camera
    geometry = types.SimpleNamespace(
        center_base=np.asarray([
            -0.12405989758892534, -0.40261513657769304, 0.08704304903962377,
        ]),
        axes_base=OBB_AXES,
        size_xyz_m=np.asarray([
            0.049320761911874664, 0.03876635001193422, 0.020615491201148604,
        ]),
        support_normal_base=OBB_AXES[:, 2],
        support_offset_m=-0.015336311988516163,
    )
    prepared = types.SimpleNamespace(
        pose_estimator=types.SimpleNamespace(T_base_camera_link=BASE_CAMERA),
        stamp=remote.rospy.Time(1788833187, 757686853),
    )
    reference = node._frozen_observation_reference_pose(prepared)
    return node, geometry, prepared, reference


def variants(node, geometry, prepared, reference):
    return node._far_field_observation_variants(
        reference, geometry, -geometry.support_normal_base, None,
        reference, prepared,
    )


def test_recorded_carton_admits_unrolled_view_inside_existing_distance_band():
    node, geometry, prepared, reference = recorded_scene()
    passing, rejected = variants(node, geometry, prepared, reference)

    # The nominal unrolled view collides; the existing band contains a safe
    # unrolled endpoint. This proves endpoint safety, not MoveIt reachability.
    assert any(
        row['observation_view']['actual_camera_target_distance_m']
        == pytest.approx(0.210)
        for row in passing
    )
    safe = next(row for row in passing if np.isclose(
        row['observation_view']['actual_camera_target_distance_m'], 0.210,
    ))
    assert safe['camera_roll_offset_deg'] == 0.0
    assert safe['observation_envelope'].minimum_support_clearance_m == pytest.approx(
        0.0059, abs=0.0001,
    )
    assert safe['observation_view']['nominal_camera_target_distance_m'] == 0.200
    assert safe['observation_view']['center_residual_m'] < 1e-8
    assert any(row['camera_target_distance_m'] == pytest.approx(0.200)
               and row['failure_code'] == 'OBSERVATION_SUPPORT_COLLISION'
               for row in rejected)


def test_distance_and_roll_search_is_bounded_and_nominal_first():
    node, geometry, prepared, reference = recorded_scene(
        remote.DEFAULT_OBSERVATION_CAMERA_ROLL_OFFSETS_DEG,
    )
    passing, rejected = variants(node, geometry, prepared, reference)
    repeat, repeat_rejected = variants(node, geometry, prepared, reference)
    assert len(passing) + len(rejected) == 70
    pairs = [(row['camera_target_distance_m'], row['camera_roll_offset_deg'])
             for row in passing]
    assert pairs == [(row['camera_target_distance_m'], row['camera_roll_offset_deg'])
                     for row in repeat]
    assert rejected == repeat_rejected
    nominal_rolls = [roll for distance, roll in pairs if np.isclose(distance, 0.2)]
    assert nominal_rolls == [-60.0, 75.0, -75.0, 90.0, -90.0, 180.0]
    assert pairs[:6] == [(0.2, roll) for roll in nominal_rolls]
    assert len({row['preference_index'] for row in passing}) == len(passing)
    assert [row['preference_index'] for row in passing] == sorted(
        row['preference_index'] for row in passing
    )
    for row in passing:
        assert 0.18 - 1e-12 <= row['camera_target_distance_m'] <= 0.22 + 1e-12
        assert row['observation_envelope'].ok
        assert row['observation_view']['object_yaw_used'] is False


def test_largest_roll_lattice_cannot_exceed_five_distances_per_roll():
    node, geometry, prepared, reference = recorded_scene(
        tuple(float(value) for value in range(-180, 181, 15)),
    )
    passing, rejected = variants(node, geometry, prepared, reference)
    assert len(passing) + len(rejected) == 125
    assert len({(row['camera_target_distance_m'], row['camera_roll_offset_deg'])
                for row in (*passing, *rejected)}) == 125


def test_single_distance_band_does_not_repeat_roll_planning():
    node, geometry, prepared, reference = recorded_scene(
        remote.DEFAULT_OBSERVATION_CAMERA_ROLL_OFFSETS_DEG,
    )
    node.grasp_config['observation_camera_target_min_distance_m'] = 0.200
    node.grasp_config['observation_camera_target_max_distance_m'] = 0.200
    passing, rejected = variants(node, geometry, prepared, reference)
    assert len(passing) == 6
    assert len(passing) + len(rejected) == 14
    assert all(row['camera_target_distance_m'] == 0.200
               for row in (*passing, *rejected))


@pytest.mark.parametrize('minimum,nominal,maximum,expected', [
    (0.18, 0.20, 0.22, (0.20, 0.19, 0.21, 0.18, 0.22)),
    (0.20, 0.20, 0.20, (0.20,)),
    (0.18, 0.18, 0.22, (0.18, 0.20, 0.22)),
    (0.18, 0.22, 0.22, (0.22, 0.20, 0.18)),
])
def test_distance_samples_stay_inside_band_without_duplicate_planning(
    minimum, nominal, maximum, expected,
):
    assert remote.observation_camera_distance_variants(
        nominal, minimum, maximum,
    ) == pytest.approx(expected)


@pytest.mark.parametrize('minimum,nominal,maximum', [
    (0.18, 0.17, 0.22), (0.18, 0.23, 0.22), (0.22, 0.20, 0.18),
    (0.0, 0.20, 0.22), (0.18, float('nan'), 0.22),
    (0.18, 0.20, float('inf')),
])
def test_invalid_distance_policy_cannot_generate_observation_authority(
    minimum, nominal, maximum,
):
    node, geometry, prepared, reference = recorded_scene()
    node.grasp_config.update({
        'observation_camera_target_nominal_distance_m': nominal,
        'observation_camera_target_min_distance_m': minimum,
        'observation_camera_target_max_distance_m': maximum,
    })
    with pytest.raises(remote.CandidateContractError):
        variants(node, geometry, prepared, reference)
