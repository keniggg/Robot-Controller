"""Exercise router transactions without creating a ROS node or connection."""
import ast
import json
import pathlib
import sys
import threading
import time
import types
import xmlrpc.client

import pytest

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))
from alicia_grasp_modes.policy import ModePolicy


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def response(success, message=''):
    return types.SimpleNamespace(success=success, message=message)


def router_fixture():
    """Compile the real router class with inert transport, not a rewritten copy."""
    tree = ast.parse((ROOT / 'scripts' / 'mode_router.py').read_text())
    class_node = next(node for node in tree.body
                      if isinstance(node, ast.ClassDef) and node.name == 'ModeRouter')
    parameters = {}

    def set_param(name, value):
        # Reproduce the actual XML-RPC encoding boundary of rospy.set_param.
        xmlrpc.client.dumps((value,), allow_none=False)
        parameters[name] = value

    rospy = types.SimpleNamespace(
        Time=types.SimpleNamespace(now=lambda: types.SimpleNamespace(
            to_nsec=lambda: 1790040471603499175)),
        wait_for_service=lambda *args, **kwargs: None,
        set_param=set_param,
        get_param=lambda name, default=None: parameters.get(name, default),
        logerr=lambda *args: None,
        loginfo=lambda *args: None,
    )
    namespace = dict(json=json, threading=threading, time=time,
                     rospy=rospy, ModePolicy=ModePolicy,
                     String=lambda data: types.SimpleNamespace(data=data),
                     StartGraspResponse=response)
    module = ast.Module(body=[class_node], type_ignores=[])
    exec(compile(module, str(ROOT / 'scripts' / 'mode_router.py'), 'exec'), namespace)
    router = namespace['ModeRouter'].__new__(namespace['ModeRouter'])
    router.lock = threading.RLock()
    router.policy = ModePolicy(state_known=True)
    router.policy.begin_switch('unknown', 1790040471603499174)
    router.target_receipt = time.monotonic()
    router.last_source_stamp = 0
    router.source_status = {}
    router.last_status = ''
    router.call_started_ns = 0
    router.service_uncertain = False
    router.completion_seen = False
    router.selection_pub = Publisher()
    router.detector_pub = Publisher()
    router.status_pub = Publisher()
    router.request_plan = lambda trigger: response(True)

    def acknowledge():
        selection = dict(parameters['/grasp_mode/selection'])
        selection['stamp_ns'] = int(selection['stamp_ns'])
        return response(True, json.dumps(dict(selection, ready=True)))

    router.reset_remote = acknowledge
    router.reset_task = acknowledge
    router.reset_task = acknowledge
    return router, parameters


def test_switch_preserves_exact_nanoseconds_through_xmlrpc_and_remote_ack():
    router, parameters = router_fixture()
    router.apply_switch()
    assert router.policy.accepting
    assert not router.policy.switching
    assert int(parameters['/grasp_mode/selection']['stamp_ns']) == 1790040471603499174
    committed = json.loads(router.selection_pub.messages[-1].data)
    assert int(committed['stamp_ns']) == 1790040471603499174


@pytest.mark.parametrize('node', ['remote', 'task'])
@pytest.mark.parametrize('failure', ['reset_refused', 'wrong_ack', 'wrong_strategy'])
def test_incomplete_remote_reset_never_commits_selection(failure, node):
    router, _parameters = router_fixture()
    if failure == 'reset_refused':
        reset = lambda: response(False, 'worker still draining')
    else:
        reset = lambda: response(True, json.dumps({
            'ready': True, 'mode': 'unknown',
            'strategy': 'direct' if failure == 'wrong_strategy' else 'two_stage',
            'generation': 1 if failure == 'wrong_strategy' else 99,
            'stamp_ns': 1790040471603499174}))
    setattr(router, 'reset_' + node, reset)
    router.apply_switch()
    assert not router.policy.accepting
    assert not router.selection_pub.messages


def test_old_task_without_direct_strategy_ack_cannot_commit_selection():
    router, _parameters = router_fixture()
    router.policy.strategy = 'direct'
    def obsolete_task():
        selected = router.policy.selection()
        selected['strategy'] = 'two_stage'
        return response(True, json.dumps(dict(selected, ready=True)))
    router.reset_task = obsolete_task
    router.apply_switch()
    assert not router.policy.accepting
    assert not router.selection_pub.messages
    assert 'task reset acknowledgement mismatch' in router.last_status


def test_ambiguous_start_transport_failure_does_not_release_switch_authority():
    router, _parameters = router_fixture()
    router.policy.finish_switch(True)
    router.policy.observe_plan('/grasp_6d/plan_enriched', True, 'current',
                               1790040471603499175, 'unknown_tabletop', 'unknown_tabletop')

    def disconnect_after_send(*_args):
        # The request can reach the task while its ROS response is lost.
        raise RuntimeError('connection dropped after request write')

    router.start_proxy = disconnect_after_send
    result = router.start(types.SimpleNamespace(execute=True, plan_id='current'))
    assert not result.success
    outcome = router.policy.begin_switch('carton', 1790040471603499176)
    assert outcome != 'SWITCHING'


def test_delayed_inactive_state_does_not_override_local_start_reservation():
    router, _parameters = router_fixture()
    router.policy.finish_switch(True)
    router.policy.reserved = True
    router.on_state(types.SimpleNamespace(active=False, message='ready',
        header=types.SimpleNamespace(stamp=types.SimpleNamespace(to_nsec=lambda: 1))))
    assert router.policy.begin_switch('carton', 1790040471603499176) == 'QUEUED'


def test_ambiguous_response_clears_only_after_this_calls_execution_slot_release():
    router, _parameters = router_fixture()
    router.policy.finish_switch(True)
    router.policy.reserved = True
    router.service_uncertain = True
    router.call_started_ns = 1790040471603499175
    def state(stamp, message):
        return types.SimpleNamespace(active=False, message=message,
            header=types.SimpleNamespace(stamp=types.SimpleNamespace(to_nsec=lambda: stamp)))
    router.on_state(state(1790040471603499174, 'execution slot released: success'))
    assert router.policy.reserved
    router.on_state(state(1790040471603499176, 'stop requested'))
    assert router.policy.reserved
    router.on_state(state(1790040471603499177, 'execution slot released: success'))
    assert not router.policy.reserved
    assert not router.service_uncertain
