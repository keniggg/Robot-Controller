"""Exact stale XMLRPC refusal and negative cases; no ROS/device/process action."""
import importlib.util
from pathlib import Path
import socket
from types import SimpleNamespace
import xmlrpc.client

import pytest


PATH = Path(__file__).with_name('visual_registration_20261004.py')
spec = importlib.util.spec_from_file_location('visual_registration_test', PATH)
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)


class Pending(RuntimeError):
    pass


def helper(error=None, alive=False, pid=456):
    def node_pid(*_):
        if error is not None:
            raise error
        return pid
    return SimpleNamespace(node_pid=node_pid, alive=lambda _: alive, CameraRegistrationPending=Pending)


OLD = dict(pid=183122, start_ticks=4557096)


def test_actual_xmlrpc_http_refused_is_pending_only_after_old_identity_proved_dead(monkeypatch):
    # Exercise the same stdlib XMLRPC->HTTP->socket exception path as the
    # deployment report, without opening a real socket or contacting ROS.
    calls = []
    def refused(*args, **kwargs):
        calls.append('connect')
        raise ConnectionRefusedError(111, 'Connection refused')
    monkeypatch.setattr(socket, 'create_connection', refused)
    h = helper()
    h.node_pid = lambda *_: xmlrpc.client.ServerProxy('http://127.0.0.1:12345').getPid('/offline_test')
    h.alive = lambda original: calls.append(('alive', original.copy())) or False
    assert m.registered_or_pending(None, '/supervisor_camera', OLD, h) is None
    assert calls[0] == 'connect' and calls[-1] == ('alive', OLD)


@pytest.mark.parametrize('name', sorted(m.VISUAL_NODES))
def test_success_or_unknown_registration_preserves_original_semantics(name):
    assert m.registered_or_pending(None, name, OLD, helper(pid=789)) == 789
    assert m.registered_or_pending(None, name, OLD, helper(error=Pending('unknown node'))) is None


def test_refusal_with_alive_original_owner_is_not_downgraded():
    with pytest.raises(ConnectionRefusedError):
        m.registered_or_pending(None, '/supervisor_camera', OLD,
                                helper(error=ConnectionRefusedError(111, 'refused'), alive=True))


@pytest.mark.parametrize('error', [TimeoutError('timeout'), ConnectionResetError('reset'),
                                 OSError('transport failed'), RuntimeError('lookup failed')])
def test_other_errors_are_never_downgraded_even_when_old_owner_dead(error):
    with pytest.raises(type(error)):
        m.registered_or_pending(None, '/supervisor_camera', OLD, helper(error=error))


@pytest.mark.parametrize('identity', [None, {}, {'pid': 1}, {'pid': True, 'start_ticks': 1},
                                   {'pid': 1, 'start_ticks': 0}])
def test_no_unverified_identity_can_claim_dead_owner(identity):
    with pytest.raises(RuntimeError, match='identity required'):
        m.registered_or_pending(None, '/supervisor_camera', identity, helper())


def test_hardware_or_unrelated_node_never_uses_visual_exception_policy():
    with pytest.raises(RuntimeError, match='identity required'):
        m.registered_or_pending(None, '/alicia_d_driver_node', OLD,
                                helper(error=ConnectionRefusedError(111, 'refused')))


def test_failure_to_prove_old_owner_dead_propagates():
    h = helper(error=ConnectionRefusedError(111, 'refused'))
    def unreadable(_): raise PermissionError('identity unreadable')
    h.alive = unreadable
    with pytest.raises(PermissionError):
        m.registered_or_pending(None, '/supervisor_camera', OLD, h)
