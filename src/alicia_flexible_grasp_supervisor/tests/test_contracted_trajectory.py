"""Offline action wire binding and abort/preemption behavior; no robot I/O."""
from copy import deepcopy
from types import SimpleNamespace as NS

import pytest
from actionlib_msgs.msg import GoalStatus

from alicia_flexible_grasp.robot.contracted_trajectory import (
    bound_action_goal, ContractedTrajectoryExecutor,
)
from alicia_flexible_grasp.robot.observation_path_guard import ObservationPathError, wire_digest
from alicia_flexible_grasp.robot.observation_tracking_contract import tracking_contract_candidates
from test_observation_path_guard import point, trajectory


def example():
    names = ['Joint%d'%i for i in range(1, 7)]
    plan = trajectory([point([0.]*6, 0., [0.]*6, [0.]*6),
                       point([.05]*6, 3., [0.]*6, [0.]*6)], names)
    cfg = {n: {'trajectory': .12, 'goal': .035} for n in names}
    cfg.update(goal_time=8., stopped_velocity_tolerance=.03)
    contract = tracking_contract_candidates(cfg, names, .5)[1]
    audit = dict(trajectory_sha256=wire_digest(plan), execution_tracking_contract=contract,
        certified_minimum_clearance_m=.01,
        following_support=dict(joint_error_bounds_rad=contract['joint_error_bounds_rad'],
            minimum_support_clearance_lower_bound_m=.01, includes_controller_bridge=True))
    return plan, audit, cfg


def test_exact_action_has_proven_per_joint_path_and_original_goal_contract():
    plan, audit, cfg = example()
    goal = bound_action_goal(plan, audit, cfg, .5)
    assert goal.trajectory == plan.joint_trajectory
    assert [t.name for t in goal.path_tolerance] == list(plan.joint_trajectory.joint_names)
    assert [t.position for t in goal.path_tolerance] == [.06]*6
    assert [t.position for t in goal.goal_tolerance] == [.035]*6
    assert goal.goal_time_tolerance.to_sec() == 8.


@pytest.mark.parametrize('mutation', ['path', 'contract', 'proof_errors', 'clearance', 'bridge', 'config', 'stop'])
def test_changed_plan_proof_or_controller_contract_is_never_sent(mutation):
    plan, audit, cfg = example()
    stop = .5
    if mutation == 'path': plan.joint_trajectory.points[-1].positions[0] += 1e-9
    elif mutation == 'contract': audit['execution_tracking_contract']['path_position_tolerance_rad'][0] = .12
    elif mutation == 'proof_errors': audit['following_support']['joint_error_bounds_rad'] = [.12]*6
    elif mutation == 'clearance': audit['following_support']['minimum_support_clearance_lower_bound_m'] = .0029
    elif mutation == 'bridge': audit['following_support']['includes_controller_bridge'] = False
    elif mutation == 'config': cfg['Joint1']['trajectory'] = .04
    elif mutation == 'stop': stop = .4
    with pytest.raises(ObservationPathError): bound_action_goal(plan, audit, cfg, stop)


class Transport:
    def __init__(self, state, code=0, completed=True):
        self.state, self.code, self.completed = state, code, completed
        self.events, self.now = [], 0.
    def get_num_connections(self): return 1
    def send_goal(self, goal): self.events.append(('send', deepcopy(goal)))
    def wait_for_result(self, duration):
        self.now += duration.to_sec()
        return self.completed
    def get_state(self): return self.state
    def get_result(self): return NS(error_code=self.code, SUCCESSFUL=0, error_string='controller result')
    def cancel_goal(self): self.events.append(('cancel', None))
    def publish(self, msg): self.events.append(('hold', deepcopy(msg)))


@pytest.mark.parametrize('state,code,expected', [
    (GoalStatus.SUCCEEDED, 0, ['send']),
    (GoalStatus.ABORTED, -4, ['send', 'hold']),
    (GoalStatus.ABORTED, -5, ['send', 'hold']),
    (GoalStatus.SUCCEEDED, -4, ['send', 'hold']),
    (GoalStatus.PREEMPTED, 0, ['send']),
    (GoalStatus.RECALLED, 0, ['send']),
    (GoalStatus.REJECTED, -1, ['send']),
])
def test_failed_finished_action_needs_explicit_hold_but_preemption_keeps_other_owner(state, code, expected):
    goal = bound_action_goal(*example(), .5)
    io = Transport(state, code)
    executor = ContractedTrajectoryExecutor(io, io, clock=lambda: io.now)
    ok, reason = executor.execute(goal, before_send=lambda: None, still_authorized=lambda: True)
    assert ok is (state == GoalStatus.SUCCEEDED and code == 0)
    assert [k for k, _ in io.events] == expected
    for kind, message in io.events:
        if kind == 'hold': assert not message.points
    assert not executor.active


def test_manual_takeover_cancels_own_goal_without_publishing_over_new_owner():
    goal = bound_action_goal(*example(), .5)
    io = Transport(GoalStatus.ACTIVE, completed=False)
    executor = ContractedTrajectoryExecutor(io, io, clock=lambda: io.now)
    calls = iter([True, True, False])
    ok, _ = executor.execute(goal, before_send=lambda: None, still_authorized=lambda: next(calls))
    assert not ok
    assert [k for k, _ in io.events] == ['send', 'cancel']


def test_timeout_cancels_and_installs_hold_without_changing_torque():
    goal = bound_action_goal(*example(), .5)
    io = Transport(GoalStatus.ACTIVE, completed=False)
    executor = ContractedTrajectoryExecutor(io, io, clock=lambda: io.now)
    ok, _ = executor.execute(goal, before_send=lambda: None, still_authorized=lambda: True)
    assert not ok
    assert [k for k, _ in io.events] == ['send', 'cancel', 'hold']


def test_final_reference_rejection_submits_nothing():
    goal = bound_action_goal(*example(), .5)
    io = Transport(GoalStatus.SUCCEEDED)
    executor = ContractedTrajectoryExecutor(io, io)
    def reject(): raise ObservationPathError('reference changed')
    with pytest.raises(ObservationPathError, match='reference changed'):
        executor.execute(goal, before_send=reject, still_authorized=lambda: True)
    assert not io.events


def test_partial_send_failure_cancels_and_requests_hold():
    goal = bound_action_goal(*example(), .5)
    io = Transport(GoalStatus.ACTIVE)
    original = io.send_goal
    def partial_send(message):
        original(message)
        raise RuntimeError('transport failed after publish')
    io.send_goal = partial_send
    executor = ContractedTrajectoryExecutor(io, io)
    with pytest.raises(RuntimeError, match='after publish'):
        executor.execute(goal, before_send=lambda: None, still_authorized=lambda: True)
    assert [k for k, _ in io.events] == ['send', 'cancel', 'hold']


@pytest.mark.parametrize('route', ['strict', 'cartesian', 'prefix'])
@pytest.mark.parametrize('case', ['success', 'abort', 'missing_executor', 'changed_reference', 'changed_context'])
def test_all_planner_routes_bind_action_and_never_fall_back_after_contract_failure(route, case):
    from test_controller_reference_timing import setup, execute
    planner, manipulator, _ = setup(route)
    _, _, cfg = example()
    contract = tracking_contract_candidates(cfg, ['Joint%d'%i for i in range(1,7)], .5)[1]
    def audit(path):
        return dict(trajectory_sha256=wire_digest(path), controller_constraints_snapshot=cfg,
            execution_tracking_contract=contract, certified_minimum_clearance_m=.01,
            following_support=dict(joint_error_bounds_rad=contract['joint_error_bounds_rad'],
                minimum_support_clearance_lower_bound_m=.01, includes_controller_bridge=True))
    planner.observation_tracking_contract_required = True
    planner.observation_path_guard_required = True
    planner.observation_path_validator = audit
    planner.observation_execution_authorized = lambda: True
    def revalidate(path, evidence):
        if case == 'changed_context': raise ObservationPathError('scene revoked')
        if case == 'changed_reference':
            old = planner.controller_reference_provider(path.joint_trajectory.joint_names)
            old.positions[0] += .001
            planner.controller_reference_provider = lambda names: old
    planner.observation_submission_revalidate = revalidate
    io = Transport(GoalStatus.ABORTED if case == 'abort' else GoalStatus.SUCCEEDED,
                   code=-4 if case == 'abort' else 0)
    if case != 'missing_executor':
        planner.observation_trajectory_executor = ContractedTrajectoryExecutor(io, io, clock=lambda: io.now)
    ok, message = execute(planner, route)
    assert ok is (case == 'success'), message
    assert not manipulator.executed_plans and not manipulator.go_calls
    assert planner._last_pose_plan is None
    if case in ('success', 'abort'):
        assert [t.position for t in io.events[0][1].path_tolerance] == [.06]*6
        assert [kind for kind, _ in io.events] == (['send'] if ok else ['send', 'hold'])
    else:
        assert not io.events


@pytest.mark.parametrize('mutation', [None, 'scene', 'manual', 'policy', 'constraints', 'stop', 'geometry', 'opening', 'model', 'trajectory'])
def test_gateway_revalidates_frozen_inputs_at_final_submission(monkeypatch, mutation):
    import hashlib
    import threading
    from test_motion_gateway_controller_start import MotionGateway, motion_gateway_node
    plan, audit, cfg = example()
    params = {'/robot/observation_tracking_contract_enabled': True,
              '/alicia_controller/constraints': cfg,
              '/alicia_controller/stop_trajectory_duration': .5,
              '/grasp_6d/remote/gripper_geometry': {'max_inner_gap_m': .05},
              '/robot_description': '<robot/>'}
    audit.update(controller_constraints_snapshot=deepcopy(cfg),
                 gripper_geometry_snapshot=deepcopy(params['/grasp_6d/remote/gripper_geometry']),
                 opening_width_m=.049, model_sha256=hashlib.sha256(b'<robot/>').hexdigest())
    gateway = MotionGateway.__new__(MotionGateway)
    gateway._observation_context_lock = threading.RLock()
    gateway._require_live_observation_task = lambda: None
    gateway._observation_execution_authorized = lambda: mutation != 'manual'
    gateway._observation_measured_opening = lambda: .048 if mutation == 'opening' else .049
    def context_check(scene, revision):
        if mutation == 'scene': raise ObservationPathError('scene revoked')
    gateway._observation_contexts = NS(validate_capture=context_check)
    if mutation == 'policy': params['/robot/observation_tracking_contract_enabled'] = False
    elif mutation == 'constraints': params['/alicia_controller/constraints']['Joint1']['trajectory'] = .2
    elif mutation == 'stop': params['/alicia_controller/stop_trajectory_duration'] = .4
    elif mutation == 'geometry': params['/grasp_6d/remote/gripper_geometry']['max_inner_gap_m'] = .06
    elif mutation == 'model': params['/robot_description'] += ' '
    elif mutation == 'trajectory': plan.joint_trajectory.points[-1].positions[0] += .001
    monkeypatch.setattr(motion_gateway_node.rospy, 'get_param', lambda name, default=None: params.get(name, default))
    if mutation is None:
        gateway._revalidate_observation_contract_submission((object(), 1), plan, audit)
    else:
        with pytest.raises(ObservationPathError):
            gateway._revalidate_observation_contract_submission((object(), 1), plan, audit)
