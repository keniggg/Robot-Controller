"""No ROS master or actuation: prospective contracts and stopping bounds."""
from types import SimpleNamespace as NS
from pathlib import Path
import json

import numpy as np
import pytest

from alicia_flexible_grasp.robot.observation_path_guard import (
    ObservationPathError, segment_coefficients, _polynomial,
)
from alicia_flexible_grasp.robot.observation_tracking_contract import (
    commanded_stop_reserve, tracking_contract_candidates,
    validate_command_derivatives, MAX_VELOCITY_RAD_S, MAX_ACCELERATION_RAD_S2,
)
from test_observation_path_guard import point, trajectory


@pytest.mark.parametrize('duration', [.02, .1, .3, .5])
@pytest.mark.parametrize('velocity', [-.08, 0., .08])
@pytest.mark.parametrize('acceleration', [-.3, 0., .3])
def test_commanded_stop_reserve_encloses_noetic_symmetric_quintic(duration, velocity, acceleration):
    start = point([1.], 0., [velocity], [acceleration])
    # Exact StopTrajectoryBuilder construction before resampling at T.
    end = point([1.], 2*duration, [-velocity], [0.])
    c = segment_coefficients(start, end, 2*duration, 1)
    bound = commanded_stop_reserve(duration)
    for s in np.linspace(0., .5, 101):
        assert abs(_polynomial(c, s)[0]-1.) <= bound+1e-14


@pytest.mark.parametrize('invalid', [0., -.1, .51, True, float('nan'), float('inf'), '0.5'])
def test_invalid_stop_duration_is_not_a_zero_reserve(invalid):
    with pytest.raises(ObservationPathError):
        commanded_stop_reserve(invalid)


def test_candidates_bind_stricter_path_limits_and_stopping_reserve_per_joint():
    cfg = {'J1': {'trajectory': .12}, 'J2': {'trajectory': .04}}
    rows = tracking_contract_candidates(cfg, ['J2', 'J1'], .5)
    assert rows[0]['path_position_tolerance_rad'] == [.04, .09]
    assert rows[-1]['path_position_tolerance_rad'] == [.035, .035]
    for row in rows:
        np.testing.assert_allclose(np.array(row['joint_error_bounds_rad']),
            np.array(row['path_position_tolerance_rad'])
            + commanded_stop_reserve(.5) + np.pi/4096., atol=1e-14)
        assert row['certifies_hardware_tracking_stopping_or_calibration'] is False
    assert len(tracking_contract_candidates({'J1': {'trajectory': .02}}, ['J1'], .5)) == 1


def test_final_bridge_velocity_is_checked_not_just_waypoint_velocities():
    plan = trajectory([point([0.], 0., [0.], [0.]), point([.1], 4., [0.], [0.])])
    audit = validate_command_derivatives(plan, point([0.], v=[0.], a=[0.]))
    assert audit['maximum_velocity_rad_s'][0] < MAX_VELOCITY_RAD_S
    assert audit['maximum_acceleration_rad_s2'][0] < MAX_ACCELERATION_RAD_S2
    with pytest.raises(ObservationPathError, match='derivatives exceed'):
        validate_command_derivatives(plan, point([1.], v=[0.], a=[0.]))


def test_acceleration_limit_rejects_short_curve_even_when_position_change_is_small():
    plan = trajectory([point([0.], 0., [0.], [0.]), point([.001], .1, [0.], [0.])])
    with pytest.raises(ObservationPathError, match='derivatives exceed'):
        validate_command_derivatives(plan, point([0.], v=[0.], a=[0.]))


@pytest.mark.parametrize('yaw', [-.1, 0., .7, 1.4, 2.1])
def test_same_policy_proves_distinct_base_azimuths_without_target_offset_rule(yaw):
    from alicia_flexible_grasp.robot.observation_path_guard import SerialUrdfFk
    from alicia_flexible_grasp.robot.observation_preview import candidate_scene, qualify_candidate_path
    from test_observation_path_guard import gripper
    data = json.loads((Path(__file__).parent/'fixtures/observation_contract_20260919.json').read_text())
    root = Path(__file__).resolve().parents[3]
    fk = SerialUrdfFk((root/'src/real-arm/alicia_d_descriptions/urdf/alicia_duo_with_gripper.urdf').read_text(),
                      ['Joint%d'%i for i in range(1, 7)])
    q0, q1 = np.array(data['start_q']), np.array(data['goal_q'])
    original_start, original_goal = fk(q0), fk(q1)
    q0[0] += yaw; q1[0] += yaw
    c, s = np.cos(yaw), np.sin(yaw)
    rotation = np.array([[c, -s, 0.], [s, c, 0.], [0., 0., 1.]])
    np.testing.assert_allclose(fk(q0)[:3, 3], rotation@original_start[:3, 3], atol=1e-12)
    np.testing.assert_allclose(fk(q1)[:3, 3], rotation@original_goal[:3, 3], atol=1e-12)
    g = data['geometry']
    geometry = NS(center_base=rotation@np.array(g['obb_center_base_m']),
        axes_base=rotation@np.array(g['R_base_obb']), size_xyz_m=g['obb_size_xyz_m'],
        support_normal_base=rotation@np.array(g['support_normal_base']), support_offset_m=g['support_offset_m'])
    frozen = candidate_scene(geometry, round(data['source_stamp_sec']*1e9))
    plan = trajectory([point(q0.tolist(), 0., [0.]*6, [0.]*6),
                       point(q1.tolist(), data['duration_sec'], [0.]*6, [0.]*6)], fk.names)
    hold = point(q0.tolist(), v=[0.]*6, a=[0.]*6)
    validate_command_derivatives(plan, hold)
    contracts = tracking_contract_candidates({n: {'trajectory': .12} for n in fk.names}, fk.names, .5)
    report = qualify_candidate_path(plan, hold, frozen, fk, gripper(), data['opening_m'],
                                    contracts[1]['joint_error_bounds_rad'])
    assert report['ok'], report
    assert report['continuous_path']['following_support']['minimum_support_clearance_lower_bound_m'] >= .003
    # Five FK-reachable configurations are coverage samples, not a proof of
    # full-arm collision feasibility, all workspace poses, or grasp success.
