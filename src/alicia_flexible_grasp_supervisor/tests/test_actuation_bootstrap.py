from copy import deepcopy
from types import SimpleNamespace as NS
import math

import numpy as np
import pytest

from alicia_flexible_grasp.robot.actuation_bootstrap import (
    bounded_observation_prefix, validate_bounded_observation_prefix,
)
from alicia_flexible_grasp.robot.observation_path_guard import segment_coefficients, _polynomial
from test_moveit_planner_pose_feedback import JointPlan, FakeManipulator, make_pose
import test_moveit_planner_pose_feedback as planner_tests
import test_motion_gateway_controller_start as gateway_tests
from test_motion_gateway_controller_start import motion_gateway_node
from test_grasp_task_sequence import grasp_task_node


@pytest.fixture(autouse=True)
def isolate_live_ros_parameters(monkeypatch):
    # These unit tests may run while the operator is using the real GUI.
    # Read local defaults; never consume or change the live mode parameter.
    monkeypatch.setattr(motion_gateway_node.rospy, 'get_param',
                        lambda key, default=None: default)


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


def stationary(q):
    return NS(positions=list(q), velocities=[0.]*6, accelerations=[0.]*6)


def test_real_sdk_offsets_clip_against_desired_without_rebasing_measured_path():
    # Three, five and eight SDK counts: the observed J2/J3/J5 discrepancy.
    offsets = np.array([0., 3., 5., 0., 8., 0.])*2.*math.pi/4096.
    measured = np.array([-1.82, .51, -.30, 0., -.23, .0046])
    desired = stationary(measured+offsets)
    direction = np.array([.2, -.1, -.3, 0., -.5, .05])
    original = JointPlan([measured, measured+.01*direction, measured+direction], duration_sec=4.)
    before = deepcopy(original)
    result = bounded_observation_prefix(original, controller_reference=desired,
                                        measured_origin=measured)
    endpoint = np.array(result.joint_trajectory.points[-1].positions)
    assert endpoint[4]-desired.positions[4] == pytest.approx(-.025, abs=2e-10)
    assert abs(endpoint[4]-measured[4]) < .013
    assert np.max(abs(endpoint-measured)) < .025
    progress = (endpoint-measured)[direction != 0.]/direction[direction != 0.]
    assert np.max(progress)-np.min(progress) < 1e-12
    assert original.joint_trajectory.points[-1].positions == before.joint_trajectory.points[-1].positions
    assert result.joint_trajectory.points[0].positions == list(measured)
    validate_bounded_observation_prefix(result, desired, measured)
    old = bounded_observation_prefix(original)
    with pytest.raises(ValueError, match='continuous curve exceeds'):
        validate_bounded_observation_prefix(old, desired, measured)


@pytest.mark.parametrize('measured,desired,endpoint', [
    (0., -.006, .019),     # controller origin limits the prefix
    (-.002, 0., .023),    # current measured origin limits the prefix
    (0., .001, .025),     # measured bound, sufficient command delta remains
])
def test_both_fixed_origins_limit_the_same_source_curve(measured, desired, endpoint):
    result = bounded_observation_prefix(path(0., .1),
        controller_reference=stationary([desired]*6), measured_origin=[measured]*6)
    assert result.joint_trajectory.points[-1].positions[0] == pytest.approx(endpoint, abs=2e-10)
    validate_bounded_observation_prefix(result, stationary([desired]*6), [measured]*6)


@pytest.mark.parametrize('desired', [.003, .01, .024])
def test_direction_without_required_command_delta_is_unavailable(desired):
    with pytest.raises(ValueError, match='insufficient directional command delta'):
        bounded_observation_prefix(JointPlan([[0.]*6, [.1]*6], duration_sec=1.),
                                   controller_reference=stationary([desired]*6))


def test_preexisting_offset_alone_cannot_claim_directional_response():
    with pytest.raises(ValueError, match='insufficient directional command delta'):
        bounded_observation_prefix(JointPlan([[0.]*6, [.001]*6], duration_sec=1.),
                                   controller_reference=stationary([.025]*6))


def test_command_direction_must_progress_from_actual_measured_start():
    with pytest.raises(ValueError, match='insufficient directional command delta'):
        bounded_observation_prefix(JointPlan([[0.]*6, [.0005]*6], duration_sec=1.),
            controller_reference=stationary([-.023]*6), measured_origin=[.001]*6)


def test_displaced_controller_bridge_cannot_be_clipped_away():
    with pytest.raises(ValueError, match='origins or controller bridge'):
        bounded_observation_prefix(path(0., .1),
                                   controller_reference=stationary([.026]*6))


def test_source_spline_first_excursion_is_cut_on_curve_not_endpoint_chord():
    original = JointPlan([[0.]*6, [.01, .01, 0., 0., 0., 0.]], duration_sec=1.)
    original.joint_trajectory.points[0].velocities = [.15, 0., 0., 0., 0., 0.]
    original.joint_trajectory.points[1].velocities = [-.15, 0., 0., 0., 0., 0.]
    coefficients = segment_coefficients(*original.joint_trajectory.points, 1., 6)
    assert _polynomial(coefficients, .5)[0] > .025  # both knots were inside
    result = bounded_observation_prefix(original)
    endpoint = result.joint_trajectory.points[-1].positions
    assert endpoint[0] == pytest.approx(.025, abs=2e-10)
    assert 0. < endpoint[1] < .01  # a linear chord would give equal q1/q2
    lo, hi = 0., .5
    for _ in range(50):
        mid = (lo+hi)/2.
        if _polynomial(coefficients, mid)[0] < endpoint[0]: lo = mid
        else: hi = mid
    assert endpoint == pytest.approx(_polynomial(coefficients, lo), abs=1e-9)


def test_source_spline_reversal_between_monotone_knots_is_rejected():
    original = path(0., .024)
    original.joint_trajectory.points[0].velocities = [-.001, 0., 0., 0., 0., 0.]
    original.joint_trajectory.points[1].velocities = [0.]*6
    with pytest.raises(ValueError, match='reverses a joint'):
        bounded_observation_prefix(original)


def test_final_spline_extrema_are_not_licensed_by_bounded_knots():
    result = path(0., .024)
    result.joint_trajectory.points[0].velocities = [.1, 0., 0., 0., 0., 0.]
    result.joint_trajectory.points[1].velocities = [-.1, 0., 0., 0., 0., 0.]
    with pytest.raises(ValueError, match='continuous curve exceeds'):
        validate_bounded_observation_prefix(result, stationary([0.]*6), [0.]*6)


def test_final_actual_desired_bridge_can_fail_when_measured_bridge_passes():
    result = JointPlan([[0.]*6, [.024, .024, 0., 0., 0., 0.]], duration_sec=1.)
    for point in result.joint_trajectory.points:
        point.velocities = [0.]*6
        point.accelerations = [0.]*6
    result.joint_trajectory.points[-1].velocities[0] = -.011
    validate_bounded_observation_prefix(result, stationary([0.]*6), [0.]*6)
    with pytest.raises(ValueError, match='continuous curve exceeds'):
        validate_bounded_observation_prefix(result, stationary([.01, 0., 0., 0., 0., 0.]), [0.]*6)


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


def test_post_retime_spline_overshoot_never_reaches_cad_or_submission():
    from test_controller_reference_timing import setup
    planner, manipulator, _ = setup('prefix')
    def retime(plan):
        result = deepcopy(plan)
        for point in result.joint_trajectory.points:
            point.velocities = [0.]*6
            point.accelerations = [0.]*6
        result.joint_trajectory.points[-1].velocities = [-.1]*6
        return result, 'synthetic retimed overshoot'
    planner._retime_strict_execution_plan = retime
    planner.observation_path_guard_required = True
    planner.observation_path_validator = lambda plan: pytest.fail('invalid prefix reached CAD')
    ok, message = planner.execute_cached_observation_prefix(make_pose())
    assert not ok and 'final continuous curve exceeds' in message
    assert not manipulator.executed_plans and planner._last_pose_plan is None


def test_reference_changed_during_retime_cannot_rebase_clipped_prefix():
    from test_controller_reference_timing import setup
    planner, manipulator, _ = setup('prefix')
    original = planner.controller_reference_provider([])
    def retime(plan):
        changed = deepcopy(original)
        changed.positions[0] += 1e-6
        planner.controller_reference_provider = lambda names: changed
        return deepcopy(plan), 'same geometry, changed external reference'
    planner._retime_strict_execution_plan = retime
    ok, message = planner.execute_cached_observation_prefix(make_pose())
    assert not ok and 'controller reference changed after speed proof' in message
    assert not manipulator.executed_plans and planner._last_pose_plan is None


@pytest.mark.parametrize('status', ['', 'DISABLED:NOT_REQUESTED', 'UNCONFIRMED:ENCODER_RESPONSE_TIMEOUT', 'UNCONFIRMED:ENCODER_RESPONSE_LOST', 'OVERHEAT_BLOCKED:TEMPERATURE'])
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
    gateway._wait_for_stationary_arm_feedback = lambda: (True, 'settled')
    def execute(target):
        events.append(('prefix', target))
        state[0] = 'CONFIRMED:MEASURED_DIRECTIONAL_RESPONSE'
        return True, 'prefix finished'
    gateway.planner.execute_cached_observation_prefix = execute
    monkeypatch.setattr(motion_gateway_node.rospy, 'is_shutdown', lambda: False)
    monkeypatch.setattr(motion_gateway_node.rospy, 'get_time', lambda: 10)
    result = gateway.handle_observation_actuation(NS(execute=True, target=make_pose()))
    assert result.success
    assert [e[0] for e in events] == ['prefix']
    assert gateway.controller_syncs == 1
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
    for reason in ('ENCODER_RESPONSE_TIMEOUT', 'ENCODER_RESPONSE_LOST'):
        node.latest_actuation_status = 'UNCONFIRMED:' + reason
        assert not node._actuation_bootstrap_allowed(config)


@pytest.mark.parametrize('behavior', ['settles', 'moving', 'stale'])
def test_bootstrap_planning_waits_for_measured_settling(monkeypatch, behavior):
    gateway = gateway_tests.MotionGatewayControllerStartTest().make_gateway()
    clock = [10.0]
    gateway._manual_control_active = lambda: False
    gateway.joint_cmd = NS(last_state_time_sec=clock[0])
    gateway._current_arm_positions_snapshot = lambda: [
        min(clock[0]-10.0, .3) if behavior == 'settles' else clock[0]-10.0
    ] + [0]*5
    def advance(dt):
        clock[0] += dt
        if behavior != 'stale': gateway.joint_cmd.last_state_time_sec = clock[0]
    monkeypatch.setattr(motion_gateway_node.rospy, 'get_time', lambda: clock[0])
    monkeypatch.setattr(motion_gateway_node.rospy, 'sleep', advance)
    monkeypatch.setattr(motion_gateway_node.rospy, 'is_shutdown', lambda: False)
    ok, message = gateway._wait_for_stationary_arm_feedback()
    assert ok is (behavior == 'settles')
    if ok: assert clock[0] >= 10.55
    else: assert 'FEEDBACK_' in message
