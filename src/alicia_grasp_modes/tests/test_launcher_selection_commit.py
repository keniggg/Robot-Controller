"""Actual router transaction -> actual launcher dispatch, with inert transport."""
import ast
import importlib.util
import json
import os
from pathlib import Path
import sys
import threading
import types

import pytest

WORK = Path(__file__).resolve().parents[3]
MODES = WORK / 'src/alicia_grasp_modes'
sys.path.insert(0, str(MODES / 'src'))
from alicia_grasp_modes.selection import parse_selection

spec = importlib.util.spec_from_file_location('router_test_contract', str(MODES / 'tests/test_router_contract.py'))
router_contract = importlib.util.module_from_spec(spec)
spec.loader.exec_module(router_contract)
LAUNCHER = Path(os.environ.get('GEN10_TEST_LAUNCHER', WORK / '.runtime/run_reboot_realigned_grasp_20261010.py'))


class Clock:
    def __init__(self):
        self.now = 0.
        self.step = lambda: None

    def monotonic(self):
        return self.now

    def sleep(self, duration):
        self.step()
        self.now += duration


class Bus:
    def __init__(self, router, parameters, prior):
        self.router = router
        self.parameters = parameters
        self.parameters['/grasp_mode/selection'] = prior
        self.cached = {
            '/grasp_mode/selection': json.dumps(prior),
            '/grasp_mode/status': json.dumps(dict(prior, state='ready')),
        }
        self.callbacks = {}
        self.subscriptions = []
        self.published = []
        self.transform = lambda topic, value: value
        self.select_count = 0
        self.runner_count = 0
        self.worker = None
        self.blocked = threading.Event()
        self.release = threading.Event()
        self.failure = None
        self.selected = router.policy.selection()
        for short in ('selection', 'status'):
            topic = '/grasp_mode/' + short
            setattr(router, short + '_pub', types.SimpleNamespace(
                publish=lambda msg, t=topic: self.publish(t, msg)))

    def publish(self, topic, message):
        value = self.transform(topic, json.loads(message.data))
        if value is None:
            return
        text = json.dumps(value)
        self.cached[topic] = text
        self.published.append((topic, value))
        for callback in list(self.callbacks.get(topic, [])):
            callback(types.SimpleNamespace(data=text))

    def Subscriber(self, topic, _kind, callback, **kwargs):
        self.callbacks.setdefault(topic, []).append(callback)
        item = types.SimpleNamespace(unregistered=False)
        def unregister():
            item.unregistered = True
            self.callbacks[topic].remove(callback)
        item.unregister = unregister
        self.subscriptions.append(item)
        if topic in self.cached:
            callback(types.SimpleNamespace(data=self.cached[topic]))
        return item

    def get_param(self, name):
        return self.parameters[name]

    def is_shutdown(self):
        return False

    def ServiceProxy(self, name, kind):
        assert name == '/grasp_mode/select'
        def select(mode, strategy):
            assert (mode, strategy) == ('unknown', 'observe_return')
            self.select_count += 1
            self.worker = threading.Thread(target=self.router.apply_switch)
            self.worker.start()
            assert self.blocked.wait(2), 'real router did not reach reset boundary'
            # apply_switch has already written the parameter, but has not
            # committed the latched selection or the matching ready status.
            assert parse_selection(self.parameters['/grasp_mode/selection']) == self.selected
            assert not self.router.policy.accepting
            return types.SimpleNamespace(success=True, message='SWITCHING')
        return select

    def dispatch_runner(self, command):
        self.runner_count += 1
        assert self.router.policy.accepting, 'runner dispatched before reset ACKs completed'
        assert not self.router.policy.switching
        selected = json.loads(command[command.index('--expected-selection-json') + 1])
        assert selected == self.selected
        assert json.loads(self.cached['/grasp_mode/selection']) == self.selected
        assert json.loads(self.cached['/grasp_mode/status'])['state'] in ('ready', 'waiting_for_target')
        return 0

    def release_reset(self):
        assert self.runner_count == 0
        self.release.set()
        self.worker.join(2)
        assert not self.worker.is_alive()


@pytest.fixture
def harness(monkeypatch):
    router, parameters = router_contract.router_fixture()
    router.policy.strategy = 'observe_return'
    router.policy.generation = 2
    selected = router.policy.selection()
    prior = dict(selected, generation=1, stamp_ns=selected['stamp_ns'] - 1)
    bus = Bus(router, parameters, prior)
    clock = Clock()
    clock.step = bus.release_reset
    spec = importlib.util.spec_from_file_location('launcher_under_test', str(LAUNCHER))
    launcher = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(launcher)
    monkeypatch.setattr(launcher, 'time', clock)
    monkeypatch.setitem(sys.modules, 'std_msgs.msg', types.SimpleNamespace(String=object))

    def block(node='remote', failure=None):
        original = getattr(router, 'reset_' + node)
        def reset():
            bus.blocked.set()
            assert bus.release.wait(2), 'test failed to release reset boundary'
            if failure == 'refused':
                return router_contract.response(False, 'worker still draining')
            if failure == 'mismatch':
                return router_contract.response(True, json.dumps(dict(selected, ready=True, generation=99)))
            return original()
        setattr(router, 'reset_' + node, reset)

    def run():
        # Compile the unmodified selection-to-dispatch statements from the
        # actual launcher main. Preflight and hardware transports stay inert.
        tree = ast.parse(LAUNCHER.read_text())
        main = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'main')
        def assigned(node, name):
            return isinstance(node, ast.Assign) and any(isinstance(t, ast.Name) and t.id == name for t in node.targets)
        start = next(i for i, n in enumerate(main.body) if assigned(n, 'response'))
        end = next(i for i, n in enumerate(main.body) if assigned(n, 'result'))
        namespace = dict(vars(launcher), rospy=bus, SetGraspMode=object, prior=prior,
                         authorization={}, save=lambda: None, parse_selection=parse_selection,
                         subprocess=types.SimpleNamespace(call=bus.dispatch_runner))
        exec(compile(ast.Module(body=main.body[start:end+1], type_ignores=[]), str(LAUNCHER), 'exec'), namespace)
        return namespace

    yield types.SimpleNamespace(bus=bus, clock=clock, router=router, selected=selected,
                                prior=prior, block=block, run=run)
    bus.release.set()
    if bus.worker:
        bus.worker.join(2)


@pytest.mark.parametrize('node', ['remote', 'task'])
def test_real_launcher_waits_for_both_actual_router_acks(harness, node):
    h = harness
    h.block(node)
    result = h.run()
    assert h.bus.select_count == h.bus.runner_count == 1
    assert result['authorization']['selection_commit']['latched_selection_matched'] is True
    assert result['authorization']['selection_commit']['router_status']['state'] == 'waiting_for_target'
    assert h.clock.now == .05
    assert all(s.unregistered for s in h.bus.subscriptions)


@pytest.mark.parametrize('node', ['remote', 'task'])
@pytest.mark.parametrize('failure', ['refused', 'mismatch'])
def test_real_router_failure_is_reported_without_runner_or_retry(harness, node, failure):
    h = harness
    h.block(node, failure)
    with pytest.raises(RuntimeError, match='Selection reset failed: ' + node):
        h.run()
    assert h.bus.select_count == 1 and h.bus.runner_count == 0
    assert h.clock.now == .05
    assert all(s.unregistered for s in h.bus.subscriptions)


@pytest.mark.parametrize('missing', ['selection', 'status'])
def test_old_latched_message_cannot_complete_new_selection(harness, missing):
    h = harness
    h.block()
    h.bus.transform = lambda topic, value: None if topic == '/grasp_mode/' + missing else value
    with pytest.raises(RuntimeError, match='commit acknowledgement missing; no automatic retry'):
        h.run()
    assert h.bus.runner_count == 0 and h.bus.select_count == 1
    assert 8.0 <= h.clock.now < 8.1


@pytest.mark.parametrize('field,value', [('generation', 3), ('stamp_ns', 1790040471603499175),
                                        ('strategy', 'two_stage'), ('mode', 'carton')])
def test_wrong_committed_identity_cannot_release_runner(harness, field, value):
    h = harness
    h.block()
    h.bus.transform = lambda topic, payload: dict(payload, **{field: value}) if topic.endswith('/selection') else payload
    with pytest.raises(RuntimeError, match='commit acknowledgement missing'):
        h.run()
    assert h.bus.runner_count == 0 and h.bus.select_count == 1


def test_future_parameter_cannot_rebind_attempt(harness):
    h = harness
    h.block()
    def replace_after_ack():
        h.bus.release_reset()
        h.bus.parameters['/grasp_mode/selection'] = dict(h.selected, generation=3, stamp_ns=h.selected['stamp_ns'] + 1)
    h.clock.step = replace_after_ack
    with pytest.raises(AssertionError):
        h.run()
    assert h.bus.runner_count == 0 and h.bus.select_count == 1


def test_switching_status_even_with_new_latch_is_not_commit(harness):
    h = harness
    h.block()
    h.bus.transform = lambda topic, payload: dict(payload, state='switching') if topic.endswith('/status') else payload
    with pytest.raises(RuntimeError, match='commit acknowledgement missing'):
        h.run()
    assert h.bus.runner_count == 0


def test_ready_status_also_accepts_without_requiring_new_detection(harness):
    h = harness
    h.block('task')
    h.bus.transform = lambda topic, payload: dict(payload, state='ready') if topic.endswith('/status') else payload
    result = h.run()
    assert h.bus.runner_count == 1
    assert result['authorization']['selection_commit']['router_status']['state'] == 'ready'


@pytest.mark.parametrize('boundary', ['deadline', 'shutdown'])
def test_final_parameter_read_cannot_admit_after_wait_boundary(harness, boundary):
    h = harness
    h.block()
    original = h.bus.get_param
    reads = [0]
    shutdown = [False]
    def delayed_read(name):
        value = original(name)
        reads[0] += 1
        if reads[0] == 3:
            if boundary == 'deadline':
                h.clock.now = 8.
            else:
                shutdown[0] = True
        return value
    h.bus.get_param = delayed_read
    h.bus.is_shutdown = lambda: shutdown[0]
    with pytest.raises(RuntimeError, match='commit acknowledgement missing'):
        h.run()
    assert reads[0] == 3
    assert h.bus.runner_count == 0 and h.bus.select_count == 1
