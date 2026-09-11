from copy import deepcopy
from types import SimpleNamespace as NS

import pytest

from alicia_flexible_grasp.robot.actuation_bootstrap import bounded_observation_prefix
from test_moveit_planner_pose_feedback import JointPlan, FakeManipulator, make_pose
import test_moveit_planner_pose_feedback as planner_tests
import test_motion_gateway_controller_start as gateway_tests
from test_motion_gateway_controller_start import motion_gateway_node
from test_grasp_task_sequence import grasp_task_node


def path(*rows):
    return JointPlan([[v, v/2, 0, 0, 0, 0] for v in rows], duration_sec=4)


def test_prefix_is_on_checked_path_and_does_not_mutate_or_retain_full_path():
    original = path(0, .01, .02, .1, .4)
    result = bounded_observation_prefix(original)
    assert [p.positions[0] for p in result.joint_trajectory.points] == pytest.approx([0, .01, .02, .025])
    assert result.joint_trajectory.points[-1].positions[1] == pytest.approx(.0125)
    assert original.joint_trajectory.points[-1].positions[0] == .4
    assert all(p.velocities == [] and p.accelerations == [] for p in result.joint_trajectory.points)


@pytest.mark.parametrize('rows', [(0, .005), (0, -.01, .04), (0, float('nan')), (0,)])
def test_invalid_short_or_reversing_prefix_cannot_execute(rows):
    with pytest.raises(ValueError):
        bounded_observation_prefix(path(*rows))


@pytest.mark.parametrize('bound', [.019, .03, float('nan')])
def test_prefix_cannot_expand_bound_or_reduce_required_response(bound):
    with pytest.raises(ValueError):
        bounded_observation_prefix(path(0, .5), bound)


def prefix_planner(current=None):
    manipulator = FakeManipulator(current_joint_values=current or [0]*6)
    planner = planner_tests.MoveItPlannerPoseFeedbackTest().make_planner(manipulator)
    planner.strict_execution_retime_enabled = True
    planner._retime_strict_execution_plan = lambda p: (deepcopy(p), 'retimed same geometry')
    planner._remember_pose_plan(make_pose(), path(0, .1), 'strict pose')
    return planner, manipulator


def test_prefix_consumes_cache_and_executes_only_bounded_motion():
    planner, manipulator = prefix_planner()
    ok, _ = planner.execute_cached_observation_prefix(make_pose())
    assert ok
    assert planner._last_pose_plan is None
    assert len(manipulator.executed_plans) == 1
    sent = manipulator.executed_plans[0]
    assert sent.joint_trajectory.points[-1].positions[0] == pytest.approx(.025)
    assert not planner.execute_cached_observation_prefix(make_pose())[0]


@pytest.mark.parametrize('failure', ['changed_start', 'wrong_pose', 'retime_failure', 'retime_disabled'])
def test_prefix_rejects_invalid_authority_before_motion(failure):
    planner, manipulator = prefix_planner([.01, 0, 0, 0, 0, 0] if failure == 'changed_start' else None)
    target = make_pose(x=1) if failure == 'wrong_pose' else make_pose()
    if failure == 'retime_failure':
        planner._retime_strict_execution_plan = lambda p: (None, 'failed')
    if failure == 'retime_disabled':
        planner.strict_execution_retime_enabled = False
    assert not planner.execute_cached_observation_prefix(target)[0]
    assert not manipulator.executed_plans


@pytest.mark.parametrize('status', ['', 'DISABLED:NOT_REQUESTED', 'UNCONFIRMED:ENCODER_RESPONSE_TIMEOUT', 'OVERHEAT_BLOCKED:TEMPERATURE'])
def test_gateway_never_bootstraps_disabled_faulted_or_missing_state(status):
    gateway = gateway_tests.MotionGatewayControllerStartTest().make_gateway()
    gateway._fresh_actuation_status = lambda: status
    result = gateway.handle_observation_actuation(NS(execute=True, target=make_pose()))
    assert not result.success
    assert gateway.controller_checks == 0
    assert gateway.planner.calls == []


def test_gateway_requires_confirmation_after_exact_prefix(monkeypatch):
    gateway = gateway_tests.MotionGatewayControllerStartTest().make_gateway()
    events = []
    state = ['PENDING:POSITIVE_ENABLE_REQUESTED']
    gateway._fresh_actuation_status = lambda: state[0]
    gateway.joint_cmd = NS(last_positions=[0]*7, publish=lambda q: events.append(('sync', q)))
    def execute(target):
        events.append(('prefix', target))
        state[0] = 'CONFIRMED:MEASURED_DIRECTIONAL_RESPONSE'
        return True, 'prefix finished'
    gateway.planner.execute_cached_observation_prefix = execute
    monkeypatch.setattr(motion_gateway_node.rospy, 'is_shutdown', lambda: False)
    monkeypatch.setattr(motion_gateway_node.rospy, 'get_time', lambda: 10)
    result = gateway.handle_observation_actuation(NS(execute=True, target=make_pose()))
    assert result.success
    assert [e[0] for e in events] == ['sync', 'prefix']
    assert gateway.planner.calls[0][1:] == (False, False)


@pytest.mark.parametrize('bootstrap_ok', [True, False])
def test_start_binds_far_plan_before_bootstrap_and_never_continues_failure(monkeypatch, bootstrap_ok):
    node = grasp_task_node.GraspTaskNode.__new__(grasp_task_node.GraspTaskNode)
    node.active = False
    node.latest_actuation_status = 'PENDING:POSITIVE_ENABLE_REQUESTED'
    node.latest_actuation_status_time = grasp_task_node.rospy.Time.from_sec(10)
    config = dict(use_grasp6d_plan=True, require_actuation_confirmation=True,
                  auto_confirm_actuation_from_observation=True)
    monkeypatch.setattr(grasp_task_node.rospy, 'get_param', lambda key, default=None: config if key == '/grasp' else default)
    monkeypatch.setattr(grasp_task_node.rospy.Time, 'now', lambda: grasp_task_node.rospy.Time.from_sec(10.1))
    events = []
    plan = NS(diagnostic='FAR_FIELD_OBSERVATION_PLAN')
    def copy(*args):
        events.append('bind')
        return NS(ok=True), plan
    node._copy_requested_grasp6d_plan = copy
    node._freeze_execution_plan = lambda p: events.append('freeze') or p
    node._bootstrap_bound_actuation = lambda p,c: events.append('bootstrap') or bootstrap_ok
    node.execute = lambda **kw: events.append('execute') or True
    node._set_near_field_active = lambda *a: None
    node._clear_bound_execution_plan = lambda: None
    node.set_state = lambda *a: None
    result = node.start_cb(NS(execute=True, plan_id='bound'))
    assert result.success is bootstrap_ok
    assert events == ['bind', 'freeze', 'bootstrap'] + (['execute'] if bootstrap_ok else [])
    assert not node.active and not node._start_inflight


def test_task_does_not_bootstrap_stale_pending_or_fault_state(monkeypatch):
    node = grasp_task_node.GraspTaskNode.__new__(grasp_task_node.GraspTaskNode)
    config = dict(use_grasp6d_plan=True, auto_confirm_actuation_from_observation=True)
    monkeypatch.setattr(grasp_task_node.rospy.Time, 'now', lambda: grasp_task_node.rospy.Time.from_sec(13))
    node.latest_actuation_status_time = grasp_task_node.rospy.Time.from_sec(10)
    node.latest_actuation_status = 'PENDING:POSITIVE_ENABLE_REQUESTED'
    assert not node._actuation_bootstrap_allowed(config)
    node.latest_actuation_status_time = grasp_task_node.rospy.Time.from_sec(12.9)
    assert node._actuation_bootstrap_allowed(config)
    node.latest_actuation_status = 'UNCONFIRMED:ENCODER_RESPONSE_TIMEOUT'
    assert not node._actuation_bootstrap_allowed(config)
