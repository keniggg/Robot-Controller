"""Real session/replan/worker wrappers, deterministic locks; no ROS or hardware."""
from copy import deepcopy
import json
import threading
import time
import types

import pytest

from test_remote_modes import configured, module, base, select_unknown
from test_observe_return_remote import setup_observe_return, deficit, prefix_session


class ObservedLock:
    """Keep the actual mutex, recording acquisition/wait edges only."""
    def __init__(self, lock, events, name):
        self.lock, self.events, self.name = lock, events, name

    def acquire(self, *args, **kwargs):
        thread = threading.current_thread().name
        self.events.setdefault((thread, self.name, 'waiting'), threading.Event()).set()
        value = self.lock.acquire(*args, **kwargs)
        if value:
            self.events.setdefault((thread, self.name, 'acquired'), threading.Event()).set()
        return value

    def release(self):
        return self.lock.release()

    def __enter__(self):
        self.acquire()
        return self

    def __exit__(self, *args):
        self.release()


def test_session_cancels_real_worker_without_replan_holding_control_hostage(configured, monkeypatch):
    node, params, clock = setup_observe_return(configured)
    assert node._observe_return_fallback(types.SimpleNamespace(remote_diagnostics=deficit()))
    session = prefix_session(node)
    clock[0] = 15_000_000_000
    identity = node._current_stream_target_identity()
    surface = node._observe_return_saved[0]
    prefix = deepcopy(node._observe_return_prefix)
    ticket = types.SimpleNamespace(generation=node._stream_generation, target_epoch=node.target_instance_epoch)
    events = {}
    for name in ('control', 'work'):
        attr = '_mode_'+name+'_lock'
        setattr(node, attr, ObservedLock(getattr(node, attr), events, name))
    entered = threading.Event()
    emergency_release = threading.Event()
    cancelled = threading.Event()
    callback_returned = threading.Event()
    results, errors, unexpected_base_replan = [], [], []

    def heavy_plan(self, actual_ticket):
        assert actual_ticket is ticket
        entered.set()
        # Model successive planner phases with real ticket rechecks. This is
        # deliberately not a claim that an uninterruptible RPC can be stopped.
        while not emergency_release.wait(.002):
            self._require_stream_ticket_current(actual_ticket)
        self._require_stream_ticket_current(actual_ticket)
        raise base.StreamResultCancelled('TEST_CLEANUP')

    monkeypatch.setattr(base.RemoteGrasp6DNode, '_prepare_and_predict', heavy_plan)
    # A visual-deficit prefix uses the diagnostic worker body. Keep that
    # real branch selection and isolate only its expensive inner planner.
    monkeypatch.setattr(node, '_prepare_diagnostic_observation_prediction',
                        lambda actual_ticket: heavy_plan(node, actual_ticket))
    # The concurrency subject is the real outer worker/control wrappers, not
    # RGB-D inference. Avoid fabricating a measurement packet for that layer.
    monkeypatch.setattr(node, '_capture_diagnostic_observation_view_source', lambda ticket: None)
    monkeypatch.setattr(node, '_capture_silhouette_source', lambda ticket: None)
    def old_replan_body(self, request):
        unexpected_base_replan.append(request)
        return types.SimpleNamespace(success=True, message='stub original replan body')
    monkeypatch.setattr(base.RemoteGrasp6DNode, 'replan_execution_cb', old_replan_body)

    def work():
        try:
            node._prepare_and_predict(ticket)
        except base.StreamResultCancelled as exc:
            errors.append(exc.code)
            cancelled.set()
        except Exception as exc:
            errors.append(type(exc).__name__+':'+str(exc))

    def replan():
        results.append(node.replan_execution_cb(types.SimpleNamespace(trigger=True)))

    def receive():
        try:
            node._observe_return_session_cb(types.SimpleNamespace(data=json.dumps(session)))
        finally:
            callback_returned.set()

    worker = threading.Thread(target=work, name='gen23-worker')
    replanner = threading.Thread(target=replan, name='gen23-replan')
    receiver = threading.Thread(target=receive, name='gen23-session')
    try:
        worker.start()
        assert entered.wait(1.), errors
        replanner.start()
        # Deterministically wait until actual replan reached its work wait.
        end = time.monotonic()+1.
        while ('gen23-replan', 'work', 'waiting') not in events and time.monotonic() < end:
            time.sleep(.001)
        assert events[('gen23-replan', 'work', 'waiting')].is_set()
        receiver.start()
        assert callback_returned.wait(.5), dict(
            paused=node._observe_return_paused,
            old_worker_cancelled=cancelled.is_set(),
            session_acks=len(node._observe_return_ack_pub.messages),
            session_waiting_control=events.get(('gen23-session', 'control', 'waiting'), threading.Event()).is_set(),
            session_acquired_control=events.get(('gen23-session', 'control', 'acquired'), threading.Event()).is_set())
        ack = json.loads(node._observe_return_ack_pub.messages[-1].data)
        assert ack['accepted'], ack
        assert cancelled.wait(.2)
        assert node._observe_return_paused
        assert errors == ['OBSERVE_RETURN_PHASE_PAUSED']
        assert node._current_stream_target_identity() == identity
        assert node._observe_return_saved[0] is surface
        assert node._observe_return_origin_prefix.plan_id == prefix.plan_id
        assert node._observe_return_origin_prefix.header.stamp == prefix.header.stamp
        assert node._poll_stream_snapshot() is False
        assert node.submit_stream_snapshot(object()) is False
        replanner.join(.5)
        assert len(results) == 1 and results[0].success is False
        assert unexpected_base_replan == []
    finally:
        emergency_release.set()
        for thread in (worker, replanner, receiver):
            if thread.ident is not None:
                thread.join(2.)
                assert not thread.is_alive(), thread.name


@pytest.mark.parametrize('strategy', ['carton', 'direct', 'observe_return'])
def test_unpaused_replan_preserves_original_entry(configured, monkeypatch, strategy):
    node, params = configured
    if strategy == 'observe_return':
        node, params, _ = setup_observe_return(configured)
    elif strategy == 'direct':
        select_unknown(params)
        params[module.SELECTION_PARAM]['strategy'] = 'direct'
        assert node.reset_mode_cb(None).success
    called = []
    request = types.SimpleNamespace(trigger=True)
    expected = types.SimpleNamespace(success=True, message='original replan result')
    def original(self, req):
        assert self is node
        called.append(req)
        return expected
    monkeypatch.setattr(base.RemoteGrasp6DNode, 'replan_execution_cb', original)
    assert node.replan_execution_cb(request) is expected
    assert called == [request]


@pytest.mark.parametrize('changed', ['selection', 'identity', 'near_field_phase',
                                   'generation', 'observation_phase', 'session', 'paused'])
def test_waiting_replan_cannot_promote_request_after_context_changes(configured, monkeypatch, changed):
    node, params, _ = setup_observe_return(configured)
    events, called, result, errors = {}, [], [], []
    for name in ('control', 'work'):
        attr = '_mode_'+name+'_lock'
        setattr(node, attr, ObservedLock(getattr(node, attr), events, name))
    monkeypatch.setattr(base.RemoteGrasp6DNode, 'replan_execution_cb',
                        lambda self, req: called.append(req))
    def replan():
        try:
            result.append(node.replan_execution_cb(types.SimpleNamespace(trigger=True)))
        except Exception as exc:
            errors.append(exc)
    worker = threading.Thread(target=replan, name='waiting-replan')
    node._mode_work_lock.acquire()
    try:
        worker.start()
        end = time.monotonic()+1.
        key = ('waiting-replan', 'work', 'waiting')
        while key not in events and time.monotonic() < end:
            time.sleep(.001)
        assert key in events
        # A bounded acquire also catches an accidental return to the old
        # control-before-work inversion without hanging the test process.
        assert node._mode_control_lock.acquire(timeout=.5)
        try:
            if changed == 'selection':
                params[module.SELECTION_PARAM] = dict(params[module.SELECTION_PARAM], generation=3)
            elif changed == 'identity':
                node.target_instance_epoch += 1
            elif changed == 'near_field_phase':
                node._near_field_phase_id += 1
            elif changed == 'generation':
                node._stream_generation += 1
            elif changed == 'observation_phase':
                node._observe_return_phase = 'observing'
            elif changed == 'session':
                node._observe_return_session = {'session_id': 'replacement'}
            else:
                node._observe_return_paused = True
        finally:
            node._mode_control_lock.release()
    finally:
        node._mode_work_lock.release()
        if worker.ident is not None:
            worker.join(2.)
            assert not worker.is_alive()
    assert errors == []
    assert called == []
    assert len(result) == 1 and result[0].success is False
    assert result[0].message == ('OBSERVE_RETURN_PHASE_PAUSED' if changed == 'paused'
                                 else 'REPLAN_CONTEXT_CHANGED')
