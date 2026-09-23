import json
import pytest
from alicia_flexible_grasp.robot.observation_timing import observation_timing

BASE = (.20, .30)
SELECTION = dict(mode='unknown', strategy='two_stage', generation=12, stamp_ns='12000')


def test_baseline_and_carton_keep_their_exact_configuration():
    for selection, config in [(None, {}), (SELECTION, {}),
                              (dict(SELECTION, mode='carton'), dict(enabled=True))]:
        assert observation_timing(selection, config, BASE) == (('baseline',), BASE)


def test_unknown_uses_faster_cruise_and_gentler_acceleration():
    key, values = observation_timing(json.dumps(SELECTION), dict(enabled=True), BASE)
    assert values == (.30, .20)
    assert key[:4] == ('unknown', 'two_stage', 12, 12000)
    assert observation_timing(dict(SELECTION, generation=13), dict(enabled=True), BASE)[0] != key


@pytest.mark.parametrize('field,value', [('velocity_scaling', x) for x in
    [True, '0.3', None, float('nan'), float('inf'), 0., .009, .321]] +
    [('acceleration_scaling', x) for x in [True, '0.2', float('nan'), -.1, .31]])
def test_bad_scaling_rejected(field, value):
    with pytest.raises(ValueError):
        observation_timing(SELECTION, dict(enabled=True, **{field:value}), BASE)


@pytest.mark.parametrize('changes', [dict(generation=True), dict(generation=0),
    dict(stamp_ns='1.2'), dict(stamp_ns=True), dict(strategy='invalid'), dict(mode='invalid')])
def test_invalid_mode_identity_rejected(changes):
    with pytest.raises(ValueError):
        observation_timing(dict(SELECTION, **changes), dict(enabled=True), BASE)


@pytest.fixture
def recorded_plan():
    from pathlib import Path
    import rospy
    from moveit_msgs.msg import RobotTrajectory
    from trajectory_msgs.msg import JointTrajectoryPoint
    data = json.loads((Path(__file__).parent/'fixtures'/'observation_trajectory_20260923.json').read_text())[0]
    plan = RobotTrajectory()
    plan.joint_trajectory.joint_names = data['names']
    for row in data['points']:
        point = JointTrajectoryPoint(positions=row['q'], velocities=row['v'], accelerations=row['a'])
        point.time_from_start = rospy.Duration.from_sec(row['t'])
        plan.joint_trajectory.points.append(point)
    return plan


def test_actual_observation_path_is_faster_with_lower_acceleration_and_zero_end_rates(recorded_plan):
    import numpy as np
    from copy import deepcopy
    from types import SimpleNamespace
    from alicia_flexible_grasp.robot.observation_timing import smooth_collinear_observation
    from alicia_flexible_grasp.robot.observation_tracking_contract import command_derivative_bounds
    before = deepcopy(recorded_plan)
    result = smooth_collinear_observation(recorded_plan, {n:.08 for n in before.joint_trajectory.joint_names})
    assert result is not None and recorded_plan == before
    assert [list(p.positions) for p in result.joint_trajectory.points] == [list(p.positions) for p in before.joint_trajectory.points]
    hold = SimpleNamespace(positions=before.joint_trajectory.points[0].positions, velocities=[0.]*6, accelerations=[0.]*6)
    old = command_derivative_bounds(before, hold)
    new = command_derivative_bounds(result, hold)
    assert result.joint_trajectory.points[-1].time_from_start.to_sec() < .90*before.joint_trajectory.points[-1].time_from_start.to_sec()
    assert max(new['maximum_velocity_rad_s']) <= .08
    assert max(new['maximum_acceleration_rad_s2']) < max(old['maximum_acceleration_rad_s2'])
    for point in (result.joint_trajectory.points[0], result.joint_trajectory.points[-1]):
        assert point.velocities == [0.]*6 and point.accelerations == [0.]*6
    # Enclose the complete interpolant, not just waypoint speeds. A signed
    # dominant-joint velocity must never reverse anywhere on the curve.
    from alicia_flexible_grasp.robot.observation_path_guard import segment_coefficients, _polynomial_bounds
    points = result.joint_trajectory.points
    sign = np.sign(np.asarray(points[-1].positions)-points[0].positions)
    for a,b in zip(points,points[1:]):
        duration = (b.time_from_start-a.time_from_start).to_sec()
        c = segment_coefficients(a,b,duration,6)
        velocity = c[:,1:]*np.arange(1,c.shape[1])/duration*sign[:,None]
        for lo,hi in zip(np.linspace(0.,1.,65)[:-1],np.linspace(0.,1.,65)[1:]):
            lower,upper = _polynomial_bounds(velocity,lo,hi)
            assert np.min(lower) >= -1e-9


def test_curved_or_reversing_paths_keep_existing_retimer(recorded_plan):
    from alicia_flexible_grasp.robot.observation_timing import smooth_collinear_observation
    limits = {n:.08 for n in recorded_plan.joint_trajectory.joint_names}
    recorded_plan.joint_trajectory.points[2].positions = list(recorded_plan.joint_trajectory.points[2].positions)
    recorded_plan.joint_trajectory.points[2].positions[0] += .00001
    assert smooth_collinear_observation(recorded_plan, limits) is None


def test_smooth_retimer_preserves_per_joint_caps_and_duration_floor(recorded_plan):
    from types import SimpleNamespace
    from alicia_flexible_grasp.robot.observation_timing import smooth_collinear_observation
    from alicia_flexible_grasp.robot.observation_tracking_contract import command_derivative_bounds
    limits = {n:.08 for n in recorded_plan.joint_trajectory.joint_names}
    limits['Joint5'] = .02
    result = smooth_collinear_observation(recorded_plan, limits, minimum_duration=30.)
    hold = SimpleNamespace(positions=result.joint_trajectory.points[0].positions, velocities=[0.]*6, accelerations=[0.]*6)
    bounds = command_derivative_bounds(result,hold)
    assert result.joint_trajectory.points[-1].time_from_start.to_sec() >= 30.-1e-9
    assert all(v <= limits[n]+1e-9 for n,v in zip(result.joint_trajectory.joint_names,bounds['maximum_velocity_rad_s']))
