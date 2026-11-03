"""Collision transport must preserve requests, rejection and cancellation."""
from copy import deepcopy
from types import SimpleNamespace as NS

import numpy as np
import pytest
from moveit_msgs.msg import RobotState, RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from alicia_flexible_grasp.robot import contact_linear_compensation as module
from alicia_flexible_grasp.robot.observation_path_guard import ObservationPathError
from alicia_flexible_grasp.robot.stationary_following import ARM_NAMES


def case(monkeypatch, *, invalid_at=None, fault_at=None, close_fault=None):
    state = RobotState()
    state.joint_state.name = ['right_finger'] + list(reversed(ARM_NAMES))
    state.joint_state.position = [.0498] + [0.] * 6
    path = RobotTrajectory()
    path.joint_trajectory.joint_names = list(ARM_NAMES)
    for x in (0., .004, .006):
        path.joint_trajectory.points.append(
            JointTrajectoryPoint(positions=[x, 0., 0., 0., 0., 0.]))
    proxies = []
    def connect(name, kind, *, persistent):
        assert name == '/check_state_validity'
        assert kind is module.GetStateValidity and persistent is True
        requests = []
        proxy = NS(requests=requests, closed=0)
        def call(request):
            requests.append(deepcopy(request))
            if fault_at is not None and len(requests) == fault_at[0]:
                raise fault_at[1]
            return NS(valid=len(requests) != invalid_at)
        def close_proxy():
            proxy.closed += 1
            if close_fault is not None:
                raise close_fault
        proxy.close = close_proxy
        proxy.call = call
        proxies.append(proxy)
        class Handle:
            __call__ = staticmethod(call)
            close = staticmethod(close_proxy)
        return Handle()
    monkeypatch.setattr(module.rospy, 'wait_for_service', lambda *a, **k: None)
    monkeypatch.setattr(module.rospy, 'ServiceProxy', connect)
    node = module.ContactLinearCompensationGateway()
    node.manipulator_group = 'alicia'
    return node, state, path, proxies


def test_every_original_sample_and_nonarm_joint_still_reaches_moveit(monkeypatch):
    node, state, path, proxies = case(monkeypatch)
    original = deepcopy(state)
    assert node._contact_command_collision_check(path, state) == 5
    proxy, = proxies
    assert proxy.closed == 1
    # Keep both checks of the shared waypoint, preserving the original
    # temporal scene-query behavior rather than caching its earlier reply.
    np.testing.assert_allclose(
        [r.robot_state.joint_state.position[-1] for r in proxy.requests],
        [0., .002, .004, .004, .006])
    assert all(r.group_name == 'alicia' for r in proxy.requests)
    assert all(r.robot_state.joint_state.position[0] == .0498 for r in proxy.requests)
    assert state == original


def test_collision_on_a_repeated_state_is_rechecked_and_stops(monkeypatch):
    node, state, path, proxies = case(monkeypatch, invalid_at=4)
    with pytest.raises(ObservationPathError, match='COMPENSATED_PATH_COLLISION'):
        node._contact_command_collision_check(path, state)
    assert len(proxies[0].requests) == 4 and proxies[0].closed == 1


@pytest.mark.parametrize('close_fails', [False, True])
def test_transport_failure_is_not_retried_or_hidden_by_cleanup(monkeypatch, close_fails):
    error = RuntimeError('connection lost after two checked samples')
    node, state, path, proxies = case(monkeypatch, fault_at=(3, error),
        close_fault=ValueError('close failure') if close_fails else None)
    with pytest.raises(RuntimeError) as caught:
        node._contact_command_collision_check(path, state)
    assert caught.value is error
    assert len(proxies) == 1 and len(proxies[0].requests) == 3
    assert proxies[0].closed == 1


@pytest.mark.parametrize('close_fails', [False, True])
def test_phase_cancellation_after_rpc_closes_and_keeps_original_exception(monkeypatch, close_fails):
    cancelled = RuntimeError('original fixed phase expired')
    node, state, path, proxies = case(monkeypatch,
        close_fault=ValueError('close failure') if close_fails else None)
    callbacks = []
    def remaining():
        callbacks.append(None)
        if len(callbacks) == 3:
            raise cancelled
    with pytest.raises(RuntimeError) as caught:
        node._contact_command_collision_check(path, state, deadline_check=remaining)
    assert caught.value is cancelled
    assert len(proxies[0].requests) == 1 and proxies[0].closed == 1


def test_new_curve_uses_a_fresh_connection_and_queries_all_states(monkeypatch):
    node, state, path, proxies = case(monkeypatch)
    assert node._contact_command_collision_check(path, state) == 5
    assert node._contact_command_collision_check(path, state) == 5
    assert len(proxies) == 2 and proxies[0] is not proxies[1]
    assert all(len(p.requests) == 5 and p.closed == 1 for p in proxies)


@pytest.mark.parametrize('cancel_at', [1, 2, 3])
def test_baseexception_cancellation_never_opens_or_continues_unchecked_rpc(monkeypatch, cancel_at):
    class Cancelled(BaseException):
        pass
    error = Cancelled('original owner retired')
    node, state, path, proxies = case(monkeypatch, close_fault=ValueError('close failure'))
    callbacks = []
    def remaining():
        callbacks.append(None)
        if len(callbacks) == cancel_at:
            raise error
    with pytest.raises(Cancelled) as caught:
        node._contact_command_collision_check(path, state, deadline_check=remaining)
    assert caught.value is error
    if cancel_at == 1:
        assert proxies == []
    else:
        assert len(proxies[0].requests) == (1 if cancel_at == 3 else 0)
        assert proxies[0].closed == 1


def test_original_request_budget_still_rejects_before_any_rpc(monkeypatch):
    node, state, path, proxies = case(monkeypatch)
    path.joint_trajectory.points[1].positions[0] = 9.
    with pytest.raises(ObservationPathError, match='COLLISION_CHECK_BUDGET'):
        node._contact_command_collision_check(path, state)
    assert not proxies[0].requests and proxies[0].closed == 1


def test_cleanup_failure_cannot_return_an_accepted_curve(monkeypatch):
    error = RuntimeError('close failure')
    node, state, path, proxies = case(monkeypatch, close_fault=error)
    with pytest.raises(RuntimeError) as caught:
        node._contact_command_collision_check(path, state)
    assert caught.value is error
    assert len(proxies[0].requests) == 5 and proxies[0].closed == 1
