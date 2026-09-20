from copy import deepcopy
import threading
from types import SimpleNamespace as NS
from unittest import mock

import numpy as np
import pytest
import rospy
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from test_motion_gateway_controller_start import MotionGateway, motion_gateway_node as gateway
from test_stationary_following import XML, NAMES
from alicia_flexible_grasp.robot.observation_path_guard import FrozenObservationScene


def fixture():
    node = MotionGateway.__new__(MotionGateway)
    node.joint_names = NAMES + ['right_finger']
    node._planner_lock = threading.RLock()
    node._tracking_probe_context = mock.Mock(return_value=('scene', 1, 2))
    node._validate_tracking_probe_context = mock.Mock()
    node._command_reference_snapshot = mock.Mock(return_value=NS(positions=[0.]*6))
    node._fresh_accepted_reference = lambda: [0.]*6+[.05]
    node._ensure_trajectory_controllers_started = mock.Mock(return_value=(True, 'ready'))
    node._synchronize_trajectory_controller_to_feedback = mock.Mock(return_value=(True, 'ready'))
    planner = NS(observation_path_guard_required=False, observation_path_validator=None)
    planner.plan_and_execute_joint_probe = mock.Mock(return_value=(True, 'planned'))
    node._ensure_planner = lambda: planner
    return node, planner


def test_plan_only_probe_never_synchronizes_or_executes():
    node, planner = fixture()
    result = node.handle_tracking_probe(NS(positions=[.005]*6, execute=False))
    assert result.success
    node._synchronize_trajectory_controller_to_feedback.assert_not_called()
    node._ensure_trajectory_controllers_started.assert_not_called()
    planner.plan_and_execute_joint_probe.assert_called_once_with([.005]*6, execute=False, start_joint_positions=[0.]*6)


@pytest.mark.parametrize('goal', [[0.]*6, [.01]*6, [.005]*5, [.005]*7, [float('nan')]*6])
def test_invalid_probe_never_reaches_controller_or_planner(goal):
    node, planner = fixture()
    assert not node.handle_tracking_probe(NS(positions=goal, execute=True)).success
    planner.plan_and_execute_joint_probe.assert_not_called()
    node._synchronize_trajectory_controller_to_feedback.assert_not_called()


def test_probe_execution_installs_final_geometry_guard_then_restores_planner():
    node, planner = fixture()
    def execute(goal, execute, **options):
        assert execute and planner.observation_path_guard_required
        assert callable(planner.observation_path_validator)
        return False, 'synthetic failure'
    planner.plan_and_execute_joint_probe = execute
    result = node.handle_tracking_probe(NS(positions=[.005]*6, execute=True))
    assert not result.success
    node._synchronize_trajectory_controller_to_feedback.assert_called_once()
    assert planner.observation_path_guard_required is False
    assert planner.observation_path_validator is None


def test_changed_sdk_during_handoff_never_reaches_execution():
    node, planner = fixture()
    node._command_reference_snapshot.side_effect = [NS(positions=[0.]*6), NS(positions=[.003]*6)]
    result = node.handle_tracking_probe(NS(positions=[.005]*6, execute=True))
    assert not result.success
    planner.plan_and_execute_joint_probe.assert_not_called()


def test_manual_active_task_or_missing_scene_rejects_before_sync():
    node, planner = fixture()
    node._tracking_probe_context.side_effect = ValueError('no idle scene authority')
    assert not node.handle_tracking_probe(NS(positions=[.005]*6, execute=True)).success
    node._synchronize_trajectory_controller_to_feedback.assert_not_called()
    planner.plan_and_execute_joint_probe.assert_not_called()


def path_fixture(monkeypatch):
    node, _ = fixture()
    node._observation_measured_opening = mock.Mock(return_value=.05)
    node._observation_controller_hold = lambda names: NS(positions=(0.,)*6,
        velocities=(0.,)*6, accelerations=(0.,)*6)
    planner = NS(manipulator=NS(get_planning_frame=lambda: 'base_link',
                               get_end_effector_link=lambda: 'tool0'))
    scene = FrozenObservationScene('diagnostic', 'frozen', 1, np.array([0.,0.,1.]),
        1., np.array([2.,2.,2.]), np.eye(3), np.array([.05]*3))
    plan = RobotTrajectory()
    plan.joint_trajectory.joint_names = list(NAMES)
    for q, time_sec in [([0.]*6, 0.), ([.005]*6, 3.)]:
        p = JointTrajectoryPoint()
        p.positions, p.velocities, p.accelerations = q, [0.]*6, [0.]*6
        p.time_from_start = rospy.Duration(time_sec)
        plan.joint_trajectory.points.append(p)
    monkeypatch.setattr(gateway.rospy, 'get_param', lambda name, default=None:
                        XML if name == '/robot_description' else default)
    return node, planner, scene, plan


def test_final_probe_spline_has_nominal_and_conditional_margin_proofs(monkeypatch):
    node, planner, scene, plan = path_fixture(monkeypatch)
    audit = node._validate_tracking_probe_path((scene,1,2), [0.]*6, [.005]*6, plan, planner)
    assert audit['diagnostic_only']
    assert audit['endpoint_support_bound']['ok']
    assert audit['endpoint_target_separation_bound']['ok']
    assert len(audit['trajectory_sha256']) == 64


def test_final_probe_cannot_leave_small_neighborhood(monkeypatch):
    node, planner, scene, plan = path_fixture(monkeypatch)
    plan.joint_trajectory.points[-1].positions = [.013]*6
    with pytest.raises(ValueError, match='local .012 rad box'):
        node._validate_tracking_probe_path((scene,1,2), [0.]*6, [.013]*6, plan, planner)


def test_final_probe_cannot_change_goal_after_joint_planning(monkeypatch):
    node, planner, scene, plan = path_fixture(monkeypatch)
    with pytest.raises(ValueError, match='joint goal changed'):
        node._validate_tracking_probe_path((scene,1,2), [0.]*6, [.00501]*6, plan, planner)


def test_final_probe_rejects_opening_change_during_proof(monkeypatch):
    node, planner, scene, plan = path_fixture(monkeypatch)
    node._observation_measured_opening.side_effect = [.05, .04]
    with pytest.raises(ValueError, match='opening/reference changed'):
        node._validate_tracking_probe_path((scene,1,2), [0.]*6, [.005]*6, plan, planner)


@pytest.mark.parametrize('failure', ['manual', 'active', 'disconnected', 'unknown_idle',
                                    'requires_inactive', 'stale', 'old_epoch', 'no_scene'])
def test_real_context_gate_rejects_missing_or_revoked_diagnostic_authority(monkeypatch, failure):
    node, _ = fixture()
    node._observation_context_lock = threading.RLock()
    node._control_reference_epoch_ns = 9_000_000_000
    node._manual_control_active = lambda: failure == 'manual'
    node._observation_task_connection_available = lambda: failure != 'disconnected'
    node._observation_task_state_stamp_ns = 0 if failure == 'unknown_idle' else 9_500_000_000
    node._observation_task_requires_inactive = failure == 'requires_inactive'
    source = 8_000_000_000 if failure == 'old_epoch' else (9_100_000_000 if failure == 'stale' else 9_900_000_000)
    node._observation_contexts = NS(active=failure == 'active', revocation=3,
        entries={} if failure == 'no_scene' else {1: NS(source_ns=source)})
    monkeypatch.setattr(gateway.rospy, 'get_time', lambda: 10.)
    monkeypatch.setattr(gateway.rospy, 'get_param', lambda name, default=None:
                        {'plan_validity_sec': .5} if name == '/grasp' else default)
    with pytest.raises(ValueError):
        MotionGateway._tracking_probe_context(node)


def test_final_probe_cannot_ignore_frozen_support_collision(monkeypatch):
    node, planner, scene, plan = path_fixture(monkeypatch)
    from dataclasses import replace
    scene = replace(scene, offset=0.)
    with pytest.raises(ValueError):
        node._validate_tracking_probe_path((scene,1,2), [0.]*6, [.005]*6, plan, planner)


def test_final_probe_named_order_is_independent_of_canonical_request_order(monkeypatch):
    node, planner, scene, plan = path_fixture(monkeypatch)
    goal = [0., .005, .004, 0., .003, 0.]
    plan.joint_trajectory.points[-1].positions = list(goal)
    plan.joint_trajectory.joint_names.reverse()
    for p in plan.joint_trajectory.points:
        p.positions = list(reversed(p.positions))
    audit = node._validate_tracking_probe_path((scene,1,2), [0.]*6, goal, plan, planner)
    assert audit['diagnostic_only']


@pytest.mark.parametrize('limit', [0., -1., float('nan'), 121.])
def test_scene_freshness_configuration_cannot_disable_probe_gate(monkeypatch, limit):
    monkeypatch.setattr(gateway.rospy, 'get_param', lambda name, default=None:
                        {'plan_validity_sec': limit} if name == '/grasp' else default)
    with pytest.raises(ValueError):
        MotionGateway._tracking_probe_scene_max_age()
