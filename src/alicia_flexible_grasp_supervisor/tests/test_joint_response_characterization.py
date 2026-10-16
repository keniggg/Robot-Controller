"""The larger diagnostic retains physical bounds and cannot rearm itself."""
from types import SimpleNamespace as NS
from unittest import mock

import pytest

from test_gateway_tracking_probe import fixture, path_fixture
from test_motion_gateway_controller_start import motion_gateway_node as gateway
from alicia_flexible_grasp.robot.stationary_following import SDK_QUANTUM_RAD


def characterization_fixture(monkeypatch, enabled=True):
    node, planner = fixture()
    node._tracking_probe_context.return_value = (NS(plan_id='fresh'), 123, 4)
    parameters = {'~enable_joint_response_characterization': enabled}
    monkeypatch.setattr(gateway.rospy, 'get_param',
                        lambda key, default=None: parameters.get(key, default))
    monkeypatch.setattr(gateway.rospy, 'set_param',
                        lambda key, value: parameters.__setitem__(key, value))
    return node, planner, parameters


def request(count=7, execute=True):
    return NS(positions=[0., count*SDK_QUANTUM_RAD, 0., 0., 0., 0.], execute=execute)


@pytest.mark.parametrize('enabled', [False, 'true', 1])
def test_explicit_opt_in_required_before_controller_or_motion(monkeypatch, enabled):
    node, planner, _ = characterization_fixture(monkeypatch, enabled)
    assert not node.handle_joint_response_characterization(request()).success
    planner.plan_and_execute_joint_probe.assert_not_called()
    node._synchronize_trajectory_controller_to_feedback.assert_not_called()


def test_planning_only_cannot_consume_epoch_or_move(monkeypatch):
    node, planner, parameters = characterization_fixture(monkeypatch, False)
    assert node.handle_joint_response_characterization(request(execute=False)).success
    assert '/supervisor/joint_response_characterization_episode' not in parameters
    node._synchronize_trajectory_controller_to_feedback.assert_not_called()
    assert planner.plan_and_execute_joint_probe.call_args.kwargs == {
        'execute': False, 'start_joint_positions': [0.]*6}


def test_production_four_count_cap_unchanged(monkeypatch):
    node, planner, _ = characterization_fixture(monkeypatch)
    assert not node.handle_tracking_probe(request()).success
    planner.plan_and_execute_joint_probe.assert_not_called()


@pytest.mark.parametrize('count', [-7, 7])
def test_signed_characterization_preserves_start_and_planning_has_no_authority(monkeypatch, count):
    node, planner, parameters = characterization_fixture(monkeypatch, False)
    req = request(count, execute=False)
    assert node.handle_joint_response_characterization(req).success
    assert planner.plan_and_execute_joint_probe.call_args.args[0] == req.positions
    assert planner.plan_and_execute_joint_probe.call_args.kwargs == {
        'execute': False, 'start_joint_positions': [0.]*6}
    assert '/supervisor/joint_response_characterization_episode' not in parameters
    node._synchronize_trajectory_controller_to_feedback.assert_not_called()


@pytest.mark.parametrize('count', [8, -8, 0])
def test_characterization_cannot_exceed_seven_counts(monkeypatch, count):
    node, planner, _ = characterization_fixture(monkeypatch)
    assert not node.handle_joint_response_characterization(request(count)).success
    planner.plan_and_execute_joint_probe.assert_not_called()


@pytest.mark.parametrize('planner_ok', [False, True])
def test_one_submission_consumes_epoch_even_on_failure(monkeypatch, planner_ok):
    node, planner, parameters = characterization_fixture(monkeypatch)
    def execute(goal, execute, **options):
        assert execute and planner.observation_path_guard_required
        assert callable(planner.observation_path_validator)
        options['before_execute']()
        assert parameters['/supervisor/joint_response_characterization_episode']['epoch_ns'] == '123'
        return planner_ok, 'result'
    planner.plan_and_execute_joint_probe = mock.Mock(side_effect=execute)
    assert node.handle_joint_response_characterization(request()).success is planner_ok
    assert not node.handle_joint_response_characterization(request()).success
    assert planner.plan_and_execute_joint_probe.call_count == 1
    assert planner.observation_path_guard_required is False
    assert planner.observation_path_validator is None


def test_consumed_epoch_survives_new_gateway_instance(monkeypatch):
    node, planner, parameters = characterization_fixture(monkeypatch)
    parameters['/supervisor/joint_response_characterization_episode'] = {'epoch_ns': '123'}
    result = node.handle_joint_response_characterization(request())
    assert not result.success and 'EPOCH_CONSUMED' in result.message
    node._synchronize_trajectory_controller_to_feedback.assert_not_called()
    planner.plan_and_execute_joint_probe.assert_not_called()


def test_changed_reference_still_rejects_after_handoff(monkeypatch):
    node, planner, _ = characterization_fixture(monkeypatch)
    node._command_reference_snapshot.side_effect = [NS(positions=[0.]*6), NS(positions=[.003]*6)]
    assert not node.handle_joint_response_characterization(request()).success
    planner.plan_and_execute_joint_probe.assert_not_called()


def test_seven_count_trajectory_uses_original_continuous_box_and_clearance(monkeypatch):
    node, planner, scene, plan = path_fixture(monkeypatch)
    goal = request().positions
    plan.joint_trajectory.points[-1].positions = goal
    audit = node._validate_tracking_probe_path((scene, 1, 2), [0.]*6, goal, plan, planner)
    assert audit['endpoint_support_bound']['ok']
    assert audit['endpoint_target_separation_bound']['ok']


def test_stale_or_active_context_still_prevents_characterization(monkeypatch):
    node, planner, _ = characterization_fixture(monkeypatch)
    node._tracking_probe_context.side_effect = ValueError('stale/active')
    assert not node.handle_joint_response_characterization(request()).success
    planner.plan_and_execute_joint_probe.assert_not_called()


def test_rejection_before_submission_does_not_consume_physical_episode(monkeypatch):
    node, planner, parameters = characterization_fixture(monkeypatch)
    planner.plan_and_execute_joint_probe.return_value = (False, 'final path rejected')
    assert not node.handle_joint_response_characterization(request()).success
    assert '/supervisor/joint_response_characterization_episode' not in parameters
