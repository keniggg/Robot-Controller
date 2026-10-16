"""Observation-only solve allowance through real gateway and planner methods."""
from copy import deepcopy
from types import SimpleNamespace as NS

import pytest
from moveit_msgs.msg import MoveItErrorCodes

from test_observation_branch_hint import fixture, offline_robot_description
from test_gen20_observation_planning_error import noetic_result
from test_motion_gateway_controller_start import MotionGateway, motion_gateway_node
from test_moveit_planner_pose_feedback import make_pose


def gateway_fixture(monkeypatch, *, fresh=False):
    observation, manipulator, clock, _ = fixture()
    contact, contact_manipulator, _, _ = fixture()
    contact.observation_branch_hints_enabled = False
    noetic_result(contact_manipulator, success=True, code=MoveItErrorCodes.SUCCESS, empty=False)
    observation.strict_execution_max_joint_velocity_rad_s = .08
    observation.strict_execution_velocity_scaling = .2
    observation.strict_execution_acceleration_scaling = .3
    node = MotionGateway.__new__(MotionGateway)
    node.planner = contact
    if fresh:
        node.manipulator_group, node.gripper_group, node.velocity = 'alicia', 'hand', .3
        monkeypatch.setattr(motion_gateway_node, 'MoveItPlanner', lambda *a: observation)
    else:
        node._observation_planner = observation
    node._configure_controller_reference_guard = lambda planner: None
    # Preview evidence itself has separate collision/visibility tests. Keep
    # this pure planning allowance test independent of telemetry or hardware.
    node._observation_preview_path_evidence = lambda planner, target: 'offline-evidence'
    monkeypatch.setattr(motion_gateway_node.rospy, 'loginfo', lambda *a, **k: None)
    monkeypatch.setattr(motion_gateway_node.rospy, 'logwarn', lambda *a, **k: None)
    req = NS(target=make_pose(), execute=False)
    observation.expected_pose = deepcopy(req.target)
    return node, observation, manipulator, contact, contact_manipulator, clock, req


@pytest.mark.parametrize('fresh', [False, True])
@pytest.mark.parametrize('configured', [None, .05, .6, 1.])
def test_observation_allowance_is_applied_and_contact_stays_at_quarter_second(
        monkeypatch, offline_robot_description, fresh, configured):
    if configured is not None:
        offline_robot_description['/robot/observation_pose_planning_time'] = configured
    node, observation, manipulator, contact, contact_manipulator, clock, req = gateway_fixture(
        monkeypatch, fresh=fresh)
    old_time = manipulator.get_planning_time()
    old_clock = clock[0]
    expected = 1. if configured is None else configured

    response = node.handle_observation_pose_strict(req)

    assert response.success, response.message
    assert 'observation_planning_time_sec=%.3f' % expected in response.message
    assert observation.strict_pose_planning_time == expected
    assert manipulator.planning_time_updates[-2:] == [expected, old_time]
    assert manipulator.get_planning_time() == old_time
    assert manipulator.plan_calls == 1 and not manipulator.executed_plans
    assert node.planner is contact and contact.strict_pose_planning_time == .25
    assert clock[0] == old_clock
    contact_old_time = contact_manipulator.get_planning_time()
    contact_response = node.handle_pose_strict(req)
    assert contact_response.success, contact_response.message
    assert contact_manipulator.planning_time_updates[-2:] == [.25, contact_old_time]
    assert contact_manipulator.plan_calls == 1 and not contact_manipulator.executed_plans
    assert 'observation_planning_time_sec=' not in contact_response.message


@pytest.mark.parametrize('invalid', [True, None, '1.0', 0., .049, 1.001,
                                     float('nan'), float('inf')])
def test_invalid_observation_allowance_cannot_plan_or_reuse_a_cached_path(
        monkeypatch, offline_robot_description, invalid):
    node, observation, manipulator, contact, _, _, req = gateway_fixture(monkeypatch)
    observation._last_pose_plan = object()
    offline_robot_description['/robot/observation_pose_planning_time'] = invalid

    response = node.handle_observation_pose_strict(req)

    assert not response.success and response.message.startswith('OBSERVATION_TIMING_INVALID: ')
    assert 'observation_pose_planning_time' in response.message
    assert observation._last_pose_plan is None
    assert manipulator.plan_calls == 0 and not manipulator.planning_time_updates
    assert node.planner is contact and contact.strict_pose_planning_time == .25
    assert not manipulator.executed_plans


@pytest.mark.parametrize('failure', ['typed_timeout', 'transport_exception'])
def test_failed_observation_solve_restores_movegroup_time_without_retry(monkeypatch, failure):
    node, observation, manipulator, contact, _, _, req = gateway_fixture(monkeypatch)
    old_time = manipulator.get_planning_time()
    if failure == 'typed_timeout':
        noetic_result(manipulator)
    else:
        def fail():
            manipulator.plan_calls += 1
            raise RuntimeError('offline MoveGroup transport failure')
        manipulator.plan = fail

    response = node.handle_observation_pose_strict(req)

    assert not response.success
    assert 'observation_planning_time_sec=1.000' in response.message
    assert manipulator.planning_time_updates[-2:] == [1., old_time]
    assert manipulator.get_planning_time() == old_time
    assert manipulator.plan_calls == 1 and not manipulator.executed_plans
    assert node.planner is contact and contact.strict_pose_planning_time == .25
    assert observation._last_pose_plan is None


def test_raised_gateway_handler_restores_contact_planner(monkeypatch):
    node, observation, manipulator, contact, _, _, req = gateway_fixture(monkeypatch)
    def fail(request):
        assert node.planner is observation and observation.strict_pose_planning_time == 1.
        raise RuntimeError('offline handler failure')
    with pytest.raises(RuntimeError, match='offline handler failure'):
        node._with_observation_planner(req, fail)
    assert node.planner is contact and contact.strict_pose_planning_time == .25
    assert not manipulator.executed_plans
