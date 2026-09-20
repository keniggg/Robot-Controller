"""Offline planner handoff proofs using real ROS trajectory messages, no ROS I/O."""
from copy import deepcopy
from types import SimpleNamespace as NS

import pytest
import rospy
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from alicia_flexible_grasp.robot.actuation_bootstrap import bounded_observation_prefix
from alicia_flexible_grasp.robot.moveit_planner import MoveItPlanner
from alicia_flexible_grasp.robot import observation_path_guard as guard
import test_moveit_planner_pose_feedback as feedback_fakes
from test_moveit_planner_pose_feedback import FakeManipulator, make_pose


NAMES = ['Joint%d' % i for i in range(1, 7)]


def point(q, seconds, velocities=None, accelerations=None):
    result = JointTrajectoryPoint()
    result.positions = list(q)
    result.velocities = [0.] * 6 if velocities is None else list(velocities)
    result.accelerations = [0.] * 6 if accelerations is None else list(accelerations)
    result.time_from_start = rospy.Duration.from_sec(seconds)
    return result


def setup(route='strict'):
    plan = RobotTrajectory()
    plan.joint_trajectory.joint_names = list(NAMES)
    plan.joint_trajectory.points = [point([0.] * 6, 0.), point([.06] * 6, .1)]
    manipulator = FakeManipulator(current_joint_values=[0.] * 6)
    planner = feedback_fakes.MoveItPlannerPoseFeedbackTest().make_planner(manipulator)
    planner.strict_execution_retime_enabled = True
    planner._retime_strict_execution_plan = lambda p: (deepcopy(p), 'retimed')
    planner.strict_execution_max_joint_velocity_rad_s = .08
    planner.strict_execution_joint_velocity_limits_rad_s = {'Joint2': .02}
    planner.controller_reference_guard_required = True
    planner.controller_reference_provider = lambda names: NS(
        positions=[.01] * 6, velocities=[0.] * 6, accelerations=[0.] * 6)
    planner._remember_pose_plan(make_pose(), plan,
                                'cartesian' if route == 'cartesian' else 'strict pose')
    return planner, manipulator, plan


def execute(planner, route):
    if route == 'prefix':
        return planner.execute_cached_observation_prefix(make_pose())
    if route == 'cartesian':
        return planner.move_to_pose_linear(make_pose(), execute=True)
    return planner.execute_cached_strict_pose(make_pose())


@pytest.mark.parametrize('route', ['strict', 'cartesian', 'prefix'])
def test_every_execution_route_proves_final_bridge_before_cad_and_rechecks_reference(route):
    planner, manipulator, original = setup(route)
    expected = bounded_observation_prefix(original) if route == 'prefix' else original
    events = []
    reference = planner.controller_reference_provider(NAMES)
    def provide(names):
        assert names == NAMES
        events.append('reference')
        return deepcopy(reference)
    def cad(path):
        events.append('cad')
        # The CAD callback must see the final slowed trajectory, not the
        # unsafe original controller-desired to first-positive-time bridge.
        audit = guard.validate_observation_trajectory_velocity(
            path, reference, planner._controller_reference_velocity_limits(NAMES))
        assert path.joint_trajectory.points[-1].time_from_start.to_sec() > .1
        return audit
    planner.controller_reference_provider = provide
    planner.observation_path_guard_required = True
    planner.observation_path_validator = cad
    original_execute = manipulator.execute
    def record_execute(path, wait=True):
        events.append('execute')
        return original_execute(path, wait=wait)
    manipulator.execute = record_execute
    ok, message = execute(planner, route)
    assert ok, message
    assert events == ['reference', 'cad', 'reference', 'execute']
    actual = manipulator.executed_plans[0]
    assert MoveItPlanner._same_joint_path_geometry(expected, actual, tolerance=0.)
    assert actual.joint_trajectory.points[0].time_from_start.to_sec() == 0.
    assert planner._last_controller_reference_audit['applied_time_scale'] > 1.
    assert planner._last_controller_reference_audit['trajectory_sha256'] == guard.wire_digest(actual)
    if route != 'prefix':
        assert 'submitted trajectory duration=%.3fs' % (
            actual.joint_trajectory.points[-1].time_from_start.to_sec()) in message
        assert 'controller_reference_time_scale=%.6f' % (
            planner._last_controller_reference_audit['applied_time_scale']) in message
    assert planner._last_pose_plan is None
    assert not manipulator.go_calls and not manipulator.start_states


@pytest.mark.parametrize('route', ['strict', 'cartesian', 'prefix'])
@pytest.mark.parametrize('failure', ['missing', 'stale_after_cad', 'changed_after_cad'])
def test_unavailable_or_changed_reference_never_executes_or_stops(route, failure):
    planner, manipulator, _ = setup(route)
    provider = planner.controller_reference_provider
    if failure == 'missing':
        planner.controller_reference_provider = None
    else:
        def cad(path):
            if failure == 'stale_after_cad':
                def stale(_names):
                    raise ValueError('SDK reference stale or manual ownership changed')
                planner.controller_reference_provider = stale
            else:
                reference = provider(NAMES)
                reference.positions[2] += 1e-12
                planner.controller_reference_provider = lambda _names: reference
            return {'trajectory_sha256': guard.wire_digest(path)}
        planner.observation_path_guard_required = True
        planner.observation_path_validator = cad
    ok, message = execute(planner, route)
    assert not ok and 'CONTROLLER_REFERENCE_INVALID' in message
    assert not manipulator.executed_plans and not manipulator.stopped
    assert not manipulator.go_calls and planner._last_pose_plan is None


@pytest.mark.parametrize('field,value', [
    ('positions', [0.] * 5), ('positions', [float('nan')] * 6),
    ('velocities', [.000001] * 6), ('accelerations', [.000001] * 6),
    ('accelerations', []),
])
def test_only_complete_finite_stationary_reference_can_be_time_scaled(field, value):
    planner, manipulator, _ = setup()
    reference = planner.controller_reference_provider(NAMES)
    setattr(reference, field, value)
    planner.controller_reference_provider = lambda _names: reference
    planner._stretch_trajectory_timing = lambda *_: pytest.fail('invalid reference must not stretch')
    ok, message = execute(planner, 'strict')
    assert not ok and 'CONTROLLER_REFERENCE_INVALID' in message
    assert not manipulator.executed_plans and not manipulator.stopped


@pytest.mark.parametrize('failure', ['budget', 'bad_audit', 'second_proof', 'geometry_mutation'])
def test_only_complete_overspeed_proof_can_stretch_and_reproof_cannot_be_bypassed(monkeypatch, failure):
    planner, manipulator, _ = setup()
    validate = guard.validate_observation_trajectory_velocity
    calls = []
    def proof(*args, **kwargs):
        calls.append(1)
        if failure == 'budget' or (failure == 'second_proof' and len(calls) == 2):
            raise guard.ObservationPathError('continuous proof budget exhausted')
        if failure == 'bad_audit':
            return {'trajectory_sha256': 'not-the-plan'}
        return validate(*args, **kwargs)
    monkeypatch.setattr(guard, 'validate_observation_trajectory_velocity', proof)
    if failure in ('budget', 'bad_audit'):
        planner._stretch_trajectory_timing = lambda *_: pytest.fail('no completed overspeed proof')
    elif failure == 'geometry_mutation':
        stretch = planner._stretch_trajectory_timing
        def mutate(plan, scale):
            result, error = stretch(plan, scale)
            result.joint_trajectory.points[-1].positions[0] += 1e-12
            return result, error
        planner._stretch_trajectory_timing = mutate
    ok, message = execute(planner, 'strict')
    assert not ok and 'CONTROLLER_BRIDGE_INVALID' in message
    assert not manipulator.executed_plans and not manipulator.stopped
    if failure == 'second_proof':
        assert len(calls) == 2


@pytest.mark.parametrize('mutation', ['time', 'position', 'limits', 'joint_names', 'cad_hash'])
def test_final_cad_or_provider_cannot_change_proved_trajectory_or_velocity_limits(mutation):
    planner, manipulator, _ = setup()
    planner.observation_path_guard_required = True
    def cad(plan):
        digest = guard.wire_digest(plan)
        if mutation == 'time':
            plan.joint_trajectory.points[-1].time_from_start = rospy.Duration.from_sec(.0001)
        elif mutation == 'position':
            plan.joint_trajectory.points[-1].positions[0] += .1
        elif mutation == 'joint_names':
            plan.joint_trajectory.joint_names.reverse()
        elif mutation == 'cad_hash':
            digest = 'not-the-final-timed-path'
        else:
            planner.strict_execution_joint_velocity_limits_rad_s['Joint2'] = .01
        return {'trajectory_sha256': digest}
    planner.observation_path_validator = cad
    ok, message = execute(planner, 'strict')
    assert not ok and 'CONTROLLER_BRIDGE_INVALID' in message
    assert not manipulator.executed_plans and not manipulator.stopped


def test_final_provider_mutation_is_checked_after_reference_read():
    planner, manipulator, _ = setup()
    planner.observation_path_guard_required = True
    provider = planner.controller_reference_provider
    def cad(plan):
        def mutate(names):
            plan.joint_trajectory.points[-1].positions[0] += .01
            return provider(names)
        planner.controller_reference_provider = mutate
        return {'trajectory_sha256': guard.wire_digest(plan)}
    planner.observation_path_validator = cad
    ok, message = execute(planner, 'strict')
    assert not ok and 'CONTROLLER_BRIDGE_INVALID' in message
    assert not manipulator.executed_plans


def test_bridge_reference_does_not_replace_fresh_measured_retime_start():
    planner, manipulator, original = setup()
    planner._retime_strict_execution_plan = MoveItPlanner._retime_strict_execution_plan.__get__(planner)
    actual = NS(joint_state=NS(name=NAMES, position=[0.] * 6))
    planner.robot = NS(get_current_state=lambda: actual)
    seen = []
    def retime(state, plan, **_kwargs):
        seen.append(state)
        return deepcopy(plan)
    manipulator.retime_trajectory = retime
    # A different stationary desired reference is valid for timing, but
    # MoveIt still receives actual q=0 and all path positions stay unchanged.
    planner.controller_reference_provider = lambda _names: NS(
        positions=[.04] * 6, velocities=[0.] * 6, accelerations=[0.] * 6)
    ok, message = execute(planner, 'strict')
    assert ok, message
    assert seen == [actual]
    assert MoveItPlanner._same_joint_path_geometry(original, manipulator.executed_plans[0], 0.)
    assert manipulator.current_joint_values == [0.] * 6
    assert not manipulator.start_states


def test_stale_measured_start_still_fails_before_reference_proof():
    planner, manipulator, _ = setup()
    planner._retime_strict_execution_plan = MoveItPlanner._retime_strict_execution_plan.__get__(planner)
    planner.robot = NS(get_current_state=lambda: NS())
    manipulator.retime_trajectory = lambda _state, p, **_kwargs: deepcopy(p)
    manipulator.current_joint_values = [.2] * 6
    planner.controller_reference_provider = lambda _names: pytest.fail('stale actual start must reject first')
    ok, message = execute(planner, 'strict')
    assert not ok and 'trajectory start mismatch' in message
    assert not manipulator.executed_plans and not manipulator.stopped


def test_uniform_stretch_scales_nonzero_waypoint_derivatives_not_joint_positions():
    planner, _, plan = setup()
    plan.joint_trajectory.points[-1].velocities = [.03] * 6
    plan.joint_trajectory.points[-1].accelerations = [.02] * 6
    original_digest = guard.wire_digest(plan)
    result, evidence, error = planner._prepare_controller_reference_timing(plan)
    assert not error, error
    scale = planner._last_controller_reference_audit['applied_time_scale']
    assert scale > 1.
    assert list(result.joint_trajectory.points[-1].velocities) == pytest.approx([.03 / scale] * 6)
    assert list(result.joint_trajectory.points[-1].accelerations) == pytest.approx([.02 / scale ** 2] * 6)
    assert MoveItPlanner._same_joint_path_geometry(plan, result, 0.)
    assert guard.wire_digest(plan) == original_digest
    assert not planner._controller_reference_execution_error(result, evidence)


@pytest.mark.parametrize('kind', ['missing_joint', 'extra_gripper', 'bad_limit'])
def test_arm_name_and_limit_contract_is_fail_closed(kind):
    planner, manipulator, plan = setup()
    if kind == 'missing_joint':
        plan.joint_trajectory.joint_names[-1] = 'Joint1'
    elif kind == 'extra_gripper':
        plan.joint_trajectory.joint_names.append('right_finger')
    else:
        planner.strict_execution_joint_velocity_limits_rad_s['Joint2'] = float('nan')
        result, evidence, message = planner._prepare_controller_reference_timing(plan)
        assert result is None and evidence is None and 'CONTROLLER_BRIDGE_INVALID' in message
        return
    ok, message = execute(planner, 'strict')
    assert not ok and 'CONTROLLER_BRIDGE_INVALID' in message
    assert not manipulator.executed_plans


def test_opt_in_default_keeps_legacy_unconfigured_planners_unchanged():
    planner, manipulator, _ = setup('cartesian')
    del planner.controller_reference_guard_required
    planner.controller_reference_provider = lambda _names: pytest.fail('disabled guard must not call provider')
    ok, message = execute(planner, 'cartesian')
    assert ok, message
    assert len(manipulator.executed_plans) == 1


@pytest.mark.parametrize('route', ['strict', 'cartesian', 'prefix'])
def test_completion_callback_receives_only_deepcopied_successfully_executed_final_trajectory(route):
    planner, manipulator, _ = setup(route)
    seen = []
    def completed(plan):
        assert len(manipulator.executed_plans) == 1
        executed = manipulator.executed_plans[0]
        assert plan is not executed
        assert guard.wire_digest(plan) == guard.wire_digest(executed)
        assert guard.wire_digest(plan) == planner._last_controller_reference_audit['trajectory_sha256']
        seen.append(deepcopy(plan))
        # Completion observers cannot modify the actual submitted trajectory.
        plan.joint_trajectory.points[-1].positions[0] = 100.
    planner.controller_reference_completion_callback = completed
    ok, message = execute(planner, route)
    assert ok, message
    assert len(seen) == 1
    assert manipulator.executed_plans[0].joint_trajectory.points[-1].positions[0] < .1
    assert planner._last_pose_plan is None


@pytest.mark.parametrize('route', ['strict', 'cartesian', 'prefix'])
@pytest.mark.parametrize('failure', ['false', 'exception'])
def test_completion_callback_never_runs_after_execute_failure_or_exception(route, failure):
    planner, manipulator, _ = setup(route)
    planner.controller_reference_completion_callback = lambda _p: pytest.fail('failed execution has no completion provenance')
    if failure == 'false':
        manipulator.execute_result = False
    else:
        def raise_error(_plan, wait=True):
            raise RuntimeError('synthetic controller execution error')
        manipulator.execute = raise_error
    ok, message = execute(planner, route)
    assert not ok, message
    assert planner._last_pose_plan is None


@pytest.mark.parametrize('route', ['strict', 'cartesian', 'prefix'])
def test_completion_callback_exception_is_reported_as_already_executed_not_admission_failure(route):
    planner, manipulator, _ = setup(route)
    def broken(_plan):
        raise ValueError('completion metadata unavailable')
    planner.controller_reference_completion_callback = broken
    ok, message = execute(planner, route)
    assert not ok and message.startswith('EXECUTION_COMPLETED_PROVENANCE_FAILED:')
    assert 'execution completed but provenance failed' in message
    assert 'CONTROLLER_REFERENCE_INVALID' not in message and 'CONTROLLER_BRIDGE_INVALID' not in message
    assert len(manipulator.executed_plans) == 1
    assert planner._last_pose_plan is None


def test_hardware_tolerance_success_after_controller_failure_is_not_completion_provenance():
    planner, manipulator, _ = setup()
    manipulator.execute_result = False
    planner._cached_plan_goal_reached_with_hardware_tolerance = lambda _p: (True, 'hardware within tolerance')
    seen = []
    planner.controller_reference_completion_callback = lambda p: seen.append(p)
    ok, message = execute(planner, 'strict')
    assert ok and 'controller reported failure' in message
    assert not seen


def test_contract_retimes_acceleration_instead_of_discarding_short_joint_path():
    from alicia_flexible_grasp.robot.observation_tracking_contract import (
        command_derivative_bounds, validate_command_derivatives,
    )
    planner, _, plan = setup()
    # A small displacement with low peak speed, but excessive bridge
    # acceleration: the previous velocity-only proof admitted its timing.
    planner.strict_execution_joint_velocity_limits_rad_s = {}
    planner.controller_reference_provider = lambda names: NS(
        positions=[0.]*6, velocities=[0.]*6, accelerations=[0.]*6)
    plan.joint_trajectory.points = [point([0.]*6, 0.), point([.002]*6, .1)]
    before = deepcopy(plan)
    old, _, error = planner._prepare_controller_reference_timing(plan)
    assert not error
    assert max(command_derivative_bounds(old, planner.controller_reference_provider(NAMES))['maximum_acceleration_rad_s2']) > .3
    planner.observation_tracking_contract_required = True
    new, evidence, error = planner._prepare_controller_reference_timing(plan)
    assert not error, error
    assert MoveItPlanner._same_joint_path_geometry(before, new, tolerance=0.)
    assert new.joint_trajectory.points[-1].time_from_start.to_sec() > .1
    assert plan == before
    validate_command_derivatives(new, evidence['reference'])
    assert evidence['trajectory_sha256'] == guard.wire_digest(new)
