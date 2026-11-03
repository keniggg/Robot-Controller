"""Offline gateway/action integration; synthetic geometry is not hardware evidence."""
from copy import deepcopy
import json
import threading
from types import SimpleNamespace as NS

import numpy as np
import pytest
import rospy
from actionlib_msgs.msg import GoalStatus

from alicia_flexible_grasp.grasp.gripper_geometry import GripperGeometry
from alicia_flexible_grasp.robot.contact_tracking_contract import POLICY
from alicia_flexible_grasp.robot.contracted_trajectory import bound_action_goal, ContractedTrajectoryExecutor
from alicia_flexible_grasp.robot.endpoint_correction import CorrectionStep, sdk_counts
from alicia_flexible_grasp.robot.observation_path_guard import (
    FrozenObservationScene, ObservationPathError, ObservationSupportBox,
)
from alicia_flexible_grasp.robot.observation_tracking_contract import qualify_contract_path
from alicia_flexible_grasp.robot.stationary_following import ARM_NAMES, SDK_QUANTUM_RAD as Q
from test_contact_tracking_contract import inputs
from test_contracted_trajectory import Transport
from test_motion_gateway_controller_start import MotionGateway, motion_gateway_node
from test_stationary_following import XML, message


def gateway_fixture(monkeypatch, clearance=.015):
    path, hold, fk, sdk, accepted, constraints, stop = inputs()
    gripper = GripperGeometry(.05, .002, np.array([.0434, .0286, .06]),
                              np.array([.1175, .155, .0774]), .003)
    # Select a synthetic table height where the original isotropic allowance
    # fails but the strictly smaller, explicitly enforced allowance may pass.
    normal = np.array([0., 0., 1.])
    zero_plane = ObservationSupportBox(fk, gripper, .05, normal, 0.)
    offset = clearance-zero_plane.enclosure(np.zeros(6), np.zeros(6))[1]
    scene = FrozenObservationScene('synthetic-contact', 'test-scene', 9_000_000_000,
        normal, offset, np.array([2., 2., 2.]), np.eye(3), np.array([.05]*3))
    context = dict(scene=scene, revocation=1, epoch=9_000_000_000, pose_index=2,
                   model=XML, plan=NS(model_choice='unknown_tabletop'),
                   initial_sdk_counts=(2048,)*6, deadline=float('inf'))
    params = {'/robot_description': XML, '/grasp_6d/remote/gripper_geometry': {},
        '/alicia_controller/constraints': constraints, '/alicia_controller/stop_trajectory_duration': stop,
        '/robot/observation_tracking_contract_enabled': True}
    state = dict(now=10.05, manual=False)
    monkeypatch.setattr(motion_gateway_node.rospy, 'get_time', lambda: state['now'])
    monkeypatch.setattr(motion_gateway_node.rospy, 'get_param', lambda key, default=None: params.get(key, default))
    node = MotionGateway.__new__(MotionGateway)
    node.joint_names = list(ARM_NAMES)+['right_finger']
    node._observation_context_lock = threading.RLock()
    node._endpoint_feedback_lock = threading.RLock()
    node._endpoint_feedback_history = dict(sdk=sdk, accepted=accepted)
    node._require_live_observation_task = lambda: None
    node._manual_control_active = lambda: state['manual']
    node._observation_execution_authorized = lambda: not state['manual']
    node._validate_pregrasp_context = lambda ctx: None
    node._observation_measured_opening = lambda: .05
    node._observation_controller_hold = lambda names: deepcopy(hold)
    node._observation_sync_hold_evidence = {'synthetic_test': True}
    node._unknown_observation_view_audit = lambda *a: None
    node._revalidate_unknown_observation_view = lambda *a: None
    node._observation_contexts = NS(validate_capture=lambda *a: None)
    node._pregrasp_reference = lambda: (2048,)*6
    planner = NS(observation_tracking_contract_required=True,
                 manipulator=NS(get_planning_frame=lambda: 'base_link',
                                get_end_effector_link=lambda: 'tool0'),
                 _observation_branch_signature=lambda: (fk.model_sha256,))
    step = CorrectionStep((2048,)*6, tuple(sdk_counts(path.joint_trajectory.points[-1].positions)),
                          tuple(sdk_counts(accepted[-1].position[:6])),
                          sdk[-1].header.stamp.to_nsec(), accepted[-1].header.stamp.to_nsec())
    return node, context, planner, path, hold, fk, gripper, constraints, state, params, step


def validate(fixture, *, include_context=True):
    node, context, planner, path = fixture[:4]
    options = {'contact_tracking_context': context} if include_context else {}
    return node._validate_frozen_observation_path((context['scene'], 1), path, planner,
        context_validator=lambda: None, **options)


def complete_audit(fixture):
    node, context, planner, path, _, _, _, _, _, _, step = fixture
    return node._validate_pregrasp_step_path(context, step, path, planner)


def test_original_isotropic_failure_reaches_only_complete_same_geometry_fallback(monkeypatch):
    f = gateway_fixture(monkeypatch)
    node, context, planner, path, hold, fk, gripper, constraints = f[:8]
    old = qualify_contract_path(path, hold, context['scene'], fk, gripper, .05,
                                constraints, .5, trajectory_stop_reserve=True)
    assert not old['ok'] and old['failure_location'] == 'start'
    audit = complete_audit(f)
    assert audit['execution_tracking_contract']['policy'] == POLICY
    assert audit['compensation_pose_index'] == 2
    assert len(audit['tracking_contract_search']) == len(old['tracking_contract_search'])+1
    assert all(row['ok'] is False for row in audit['tracking_contract_search'][:-1])
    assert audit['following_support']['includes_controller_bridge'] is True
    assert audit['following_support']['minimum_support_clearance_lower_bound_m'] >= .003
    assert audit['certified_minimum_clearance_m'] >= .003
    assert audit['scene_sha256'] == context['scene'].digest
    assert audit['model_sha256'] == fk.model_sha256
    goal = bound_action_goal(path, audit, constraints, .5, reference=hold)
    assert [t.position for t in goal.path_tolerance] == audit['execution_tracking_contract']['path_position_tolerance_rad']
    assert all(0. < g.position <= p.position for p, g in zip(goal.path_tolerance, goal.goal_tolerance))
    assert goal.trajectory == path.joint_trajectory


def test_large_clearance_keeps_original_contract_and_needs_no_new_raw_source(monkeypatch):
    f = gateway_fixture(monkeypatch, clearance=1.)
    f[0]._endpoint_feedback_history = dict(sdk=[], accepted=[])
    audit = complete_audit(f)
    assert audit['execution_tracking_contract']['policy'] != POLICY
    assert not any(row.get('policy') == POLICY for row in audit['tracking_contract_search'])


@pytest.mark.parametrize('mutation', ['no_context', 'pose0', 'pose1', 'known', 'other_scene', 'five_counts'])
def test_contact_fallback_cannot_escape_phase_target_or_count_scope(monkeypatch, mutation):
    f = gateway_fixture(monkeypatch)
    if mutation == 'pose0': f[1]['pose_index'] = 0
    if mutation == 'pose1': f[1]['pose_index'] = 1
    if mutation == 'known': f[1]['plan'].model_choice = 'known'
    if mutation == 'other_scene':
        f[1]['scene'] = deepcopy(f[1]['scene'])
        object.__setattr__(f[1]['scene'], 'digest', 'changed')
        # Explicitly pass the independent original scene to exercise binding.
        with pytest.raises(ObservationPathError, match='SCOPE_INVALID'):
            f[0]._validate_frozen_observation_path(
                (NS(**dict(f[1]['scene'].__dict__, digest='original')), 1), f[3], f[2],
                context_validator=lambda: None, contact_tracking_context=f[1])
        return
    if mutation == 'five_counts': f[3].joint_trajectory.points[-1].positions[5] = 5*Q
    with pytest.raises(ObservationPathError):
        validate(f, include_context=mutation != 'no_context')


def test_failed_tighter_geometry_is_not_replaced_by_nominal_contact_pass(monkeypatch):
    f = gateway_fixture(monkeypatch, clearance=.0031)
    with pytest.raises(ObservationPathError):
        complete_audit(f)


@pytest.mark.parametrize('mutation', ['none', 'outward_feedback', 'stale', 'epoch', 'hold', 'configuration'])
def test_submission_rechecks_raw_provenance_without_widening_frozen_contract(monkeypatch, mutation):
    f = gateway_fixture(monkeypatch)
    node, context, _, path, hold, _, _, _, state, params = f[:10]
    audit = complete_audit(f)
    frozen = deepcopy(audit)
    if mutation == 'outward_feedback':
        for row in node._endpoint_feedback_history['accepted']: row.position[1] -= Q
    if mutation == 'stale': state['now'] = 11.
    if mutation == 'epoch': context['epoch'] = 10_000_000_000
    if mutation == 'hold':
        hold.positions[0] += .0001
        node._observation_controller_hold = lambda names: deepcopy(hold)
    if mutation == 'configuration': params['/alicia_controller/constraints']['Joint1']['trajectory'] = .04
    if mutation == 'none':
        node._revalidate_pregrasp_tracking_submission(context, path, audit)
        assert context['active_contact_tracking_contract'] == audit['execution_tracking_contract']
    else:
        with pytest.raises(ObservationPathError):
            node._revalidate_pregrasp_tracking_submission(context, path, audit)
        assert context.get('active_contact_tracking_contract') is None
    assert audit == frozen


def armed(monkeypatch):
    f = gateway_fixture(monkeypatch)
    audit = complete_audit(f)
    f[0]._revalidate_pregrasp_tracking_submission(f[1], f[3], audit)
    goal = bound_action_goal(f[3], audit, f[7], .5, reference=f[4])
    return f, goal


@pytest.mark.parametrize('fault', ['stale', 'duplicate', 'exception'])
def test_runtime_monitor_failure_uses_owned_hold_and_keeps_torque_contract(monkeypatch, fault):
    f, goal = armed(monkeypatch)
    node, context, state = f[0], f[1], f[8]
    io = Transport(GoalStatus.ACTIVE, completed=False)
    executor = ContractedTrajectoryExecutor(io, io, clock=lambda: io.now)
    def monitor():
        if any(kind == 'send' for kind, _ in io.events):
            if fault == 'stale': state['now'] = 11.
            elif fault == 'duplicate':
                node._endpoint_feedback_history['accepted'].append(node._endpoint_feedback_history['accepted'][-1])
            else: raise RuntimeError('monitor callback failure')
        node._contact_tracking_feedback_monitor(context)
    with pytest.raises((ObservationPathError, RuntimeError)):
        executor.execute(goal, before_send=lambda: None, still_authorized=lambda: True,
                         execution_monitor=monitor)
    assert [kind for kind, _ in io.events] == ['send', 'cancel', 'hold']
    assert not io.events[-1][1].points
    assert executor.active is False


def test_manual_takeover_has_priority_over_stale_monitor_and_publishes_no_hold(monkeypatch):
    f, goal = armed(monkeypatch)
    node, context, state = f[0], f[1], f[8]
    io = Transport(GoalStatus.ACTIVE, completed=False)
    original = io.send_goal
    def send(value):
        original(value)
        state.update(manual=True, now=11.)
    io.send_goal = send
    executor = ContractedTrajectoryExecutor(io, io, clock=lambda: io.now)
    ok, _ = executor.execute(goal, before_send=lambda: None,
        still_authorized=lambda: not state['manual'],
        execution_monitor=lambda: node._contact_tracking_feedback_monitor(context))
    assert ok is False
    assert [kind for kind, _ in io.events] == ['send', 'cancel']


def test_success_result_does_not_hide_feedback_expiry_during_result_wait(monkeypatch):
    f, goal = armed(monkeypatch)
    node, context, state = f[0], f[1], f[8]
    io = Transport(GoalStatus.SUCCEEDED)
    original = io.wait_for_result
    def wait(duration):
        state['now'] = 11.
        return original(duration)
    io.wait_for_result = wait
    executor = ContractedTrajectoryExecutor(io, io, clock=lambda: io.now)
    with pytest.raises(ObservationPathError):
        executor.execute(goal, before_send=lambda: None, still_authorized=lambda: True,
            execution_monitor=lambda: node._contact_tracking_feedback_monitor(context))
    assert [kind for kind, _ in io.events] == ['send', 'cancel', 'hold']


def test_post_settle_transient_violation_is_latched_even_after_inside_feedback(monkeypatch):
    f, _ = armed(monkeypatch)
    node, context, state = f[0], f[1], f[8]
    context['contact_tracking_completed_after_ns'] = 10_000_000_000
    goal = np.array(context['contact_tracking_goal_positions'])
    tolerance = np.array(context['active_contact_tracking_contract']['goal_position_tolerance_rad'])
    bad = goal.copy(); bad[0] += tolerance[0]+Q
    node._endpoint_feedback_history['accepted'] += [message(10.1, 'sdk_measured', bad, True),
                                                  message(10.2, 'sdk_measured', goal, True)]
    state['now'] = 10.25
    with pytest.raises(ObservationPathError, match='goal envelope exceeded'):
        node._contact_tracking_feedback_monitor(context)
    node._endpoint_feedback_history['accepted'] = [message(10.3, 'sdk_measured', goal, True)]
    state['now'] = 10.35
    with pytest.raises(ObservationPathError, match='goal envelope exceeded'):
        node._contact_tracking_feedback_monitor(context)


def test_no_contact_contract_leaves_other_execution_monitoring_unchanged(monkeypatch):
    f = gateway_fixture(monkeypatch)
    f[0]._endpoint_feedback_history = dict(sdk=[], accepted=[])
    f[8]['now'] = 99.
    assert f[0]._contact_tracking_feedback_monitor(f[1]) is None


@pytest.mark.parametrize('lost_source', ['sdk', 'accepted'])
def test_history_eviction_cannot_hide_gap_from_last_monitored_source(monkeypatch, lost_source):
    f, _ = armed(monkeypatch)
    node, context, state = f[0], f[1], f[8]
    assert context['contact_tracking_monitored_stamps']
    for source in ('sdk', 'accepted'):
        old = node._endpoint_feedback_history[source][-1]
        replacement = []
        # The other source has complete continuity; only the selected source
        # lost the interval before its latest (otherwise valid, fresh) sample.
        times = [11.8, 11.9] if source == lost_source else np.arange(10.1, 11.91, .1)
        for stamp in times:
            replacement.append(message(stamp, old.header.frame_id, old.position[:6], source == 'accepted'))
        node._endpoint_feedback_history[source] = replacement
    state['now'] = 11.95
    with pytest.raises(ObservationPathError, match='continuity lost: '+lost_source):
        node._contact_tracking_feedback_monitor(context)


@pytest.mark.parametrize('owner', ['automatic', 'manual'])
@pytest.mark.parametrize('publisher_failure', [False, True])
def test_completed_action_settle_failure_keeps_original_error_and_owner_priority(
        monkeypatch, owner, publisher_failure):
    # Execute the real outer gateway handler with injected planning/feedback
    # transports; the inner complete path+submission chain is tested above.
    from test_pregrasp_gateway import make_gateway
    node, planner, plan, state, calls, _ = make_gateway(monkeypatch)
    context = node._pregrasp_context(plan)
    plan.model_choice = 'known-fixture'
    plan.poses[2] = deepcopy(plan.poses[0])
    context['pose_index'] = 2
    node._pregrasp_context = lambda plan, index=0: context
    node._pregrasp_still_authorized = lambda _: owner == 'automatic'
    io = Transport(GoalStatus.SUCCEEDED)
    planner.observation_trajectory_executor = NS(hold_publisher=io)
    original_execute = planner.plan_and_execute_joint_probe
    def execute(*args, **kwargs):
        result = original_execute(*args, **kwargs)
        context['active_contact_tracking_contract'] = {'policy': POLICY}
        return result
    planner.plan_and_execute_joint_probe = execute
    original_snapshot = node._endpoint_following_snapshot
    def snapshot(*args, **kwargs):
        if kwargs.get('after_ns'):
            context['contact_tracking_fault'] = 'CONTACT_TRACKING_FEEDBACK_INVALID: settle source expired'
            raise ObservationPathError(context['contact_tracking_fault'])
        return original_snapshot(*args, **kwargs)
    node._endpoint_following_snapshot = snapshot
    if publisher_failure:
        def broken_publish(msg):
            io.events.append(('hold', deepcopy(msg)))
            raise RuntimeError('hold transport unavailable')
        io.publish = broken_publish
    result = node._handle_compensate_endpoint(NS(plan=plan, execute=True), 2)
    report = json.loads(result.message)
    assert not result.success and 'settle source expired' in report['code']
    assert report['steps_completed'] == 1 and report['post_command_feedback_valid'] is False
    assert len(calls) == 1
    assert [kind for kind, _ in io.events] == (['hold'] if owner == 'automatic' else [])
    assert report['no_implicit_rollback_or_torque_disable'] is True
