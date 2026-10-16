"""Independent retry commit-boundary regressions; actual MRO, no ROS graph."""
from copy import deepcopy
import threading
from types import SimpleNamespace as NS

import pytest

from test_gen22_qualified_rgbd_remote import (
    context, pending_reached_retry, real_wait_clock)
from test_remote_modes import base


@pytest.mark.parametrize('changed', ['phase', 'task', 'session', 'session_contents',
    'configuration', 'epoch', 'selection', 'manual_callback'])
def test_final_context_cannot_commit_a_changed_retry_owner(context, monkeypatch, changed):
    c = context
    source, color, depth, msg, captured = pending_reached_retry(c, monkeypatch)
    n = c.node
    n.color_cb(color)
    n.depth_cb(depth)
    original = n._measurement_remote_context
    calls = []

    def check(request):
        result = original(request)
        calls.append(True)
        if len(calls) == 3:  # Probe, pre-resolution, final registration commit.
            if changed == 'phase':
                replacement = deepcopy(msg)
                replacement.phase_id += 1
                n._station_compute_phase = replacement
            elif changed == 'task':
                n.robot_execution_active = False
                n._station_compute_task_state = 'FAILED'
            elif changed == 'session':
                n._observe_return_session = deepcopy(n._observe_return_session)
            elif changed == 'session_contents':
                n._observe_return_session['deadline_ns'] -= 1
            elif changed == 'epoch':
                n._measurement_remote_epoch_ns += 1
            elif changed == 'selection':
                n._mode_selection = dict(n._mode_selection, generation=999)
            elif changed == 'manual_callback':
                n._initial_preflight_manual_cb(NS(data=True))
            else:
                n.planning_snapshot_max_age_sec += 1.
        return result

    monkeypatch.setattr(n, '_measurement_remote_context', check)
    assert n._observe_return_retry_reached_view()
    assert len(calls) == 3 and len(captured) == 1
    assert n._observe_return_phase == 'observing'
    assert n._observe_return_reached_retry is None


def test_manual_takeover_keeps_seen_token_and_cannot_be_undone_by_phase_redelivery(context, monkeypatch):
    c = context
    source, color, depth, msg, captured = pending_reached_retry(c, monkeypatch)
    n = c.node
    pending = n._observe_return_reached_retry
    n._initial_preflight_manual_cb(NS(data=True))
    n._initial_preflight_manual_cb(NS(data=False))
    assert n._observe_return_reached_retry is None
    assert pending['key'] in n._observe_return_reached_retry_seen
    n.color_cb(color)
    n.depth_cb(depth)
    n.near_field_state_cb(deepcopy(msg))
    assert n._observe_return_reached_retry is None
    assert not captured and n._observe_return_phase == 'observing'


@pytest.mark.parametrize('callback_kind', ['phase', 'task'])
def test_real_owner_callback_can_finish_during_final_context_and_reject_commit(context, monkeypatch, callback_kind):
    c = context
    source, color, depth, msg, captured = pending_reached_retry(c, monkeypatch)
    n = c.node
    n.color_cb(color)
    n.depth_cb(depth)
    original = n._measurement_remote_context
    calls, threads, failures = [], [], []
    finished = threading.Event()
    replacement = deepcopy(msg)
    replacement.phase_id += 1
    replacement.active = False
    monkeypatch.setattr(n, 'stop_streaming', lambda: None)

    def callback():
        try:
            if callback_kind == 'phase':
                n.near_field_state_cb(replacement)
            else:
                n.grasp_state_cb(NS(state='FAILED', active=False))
        except BaseException as exc:
            failures.append(exc)
        finally:
            finished.set()

    def check(request):
        result = original(request)
        calls.append(True)
        if len(calls) == 3:
            worker = threading.Thread(target=callback, daemon=True)
            threads.append(worker)
            worker.start()
            assert finished.wait(1.), 'final context retained commit locks'
        return result

    monkeypatch.setattr(n, '_measurement_remote_context', check)
    try:
        assert n._observe_return_retry_reached_view()
    finally:
        for worker in threads:
            worker.join(2.)
    assert len(calls) == 3 and threads and not failures
    assert not any(t.is_alive() for t in threads)
    assert len(captured) == 1 and n._observe_return_phase == 'observing'
    assert n._observe_return_reached_retry is None


@pytest.mark.parametrize('callback_kind', ['phase', 'task'])
def test_real_owner_callback_cannot_mutate_during_final_registration(context, monkeypatch, callback_kind):
    c = context
    source, color, depth, msg, captured = pending_reached_retry(c, monkeypatch)
    n = c.node
    n.color_cb(color)
    n.depth_cb(depth)
    original = base.RemoteGrasp6DNode._register_near_field_surface
    calls, threads, failures = [], [], []
    entered, finished = threading.Event(), threading.Event()
    replacement = deepcopy(msg)
    replacement.phase_id += 1
    replacement.active = False
    # Cleanup belongs outside the short owner update critical section; it is
    # independently exercised below. Retain the actual base state callback.
    monkeypatch.setattr(n, 'stop_streaming', lambda: None)

    def callback():
        entered.set()
        try:
            if callback_kind == 'phase':
                n.near_field_state_cb(replacement)
            else:
                n.grasp_state_cb(NS(state='FAILED', active=False))
        except BaseException as exc:
            failures.append(exc)
        finally:
            finished.set()

    def register(self, observation):
        calls.append(True)
        worker = threading.Thread(target=callback, daemon=True)
        threads.append(worker)
        worker.start()
        assert entered.wait(1.)
        assert not finished.wait(.05), 'owner callback bypassed registration locks'
        assert n._station_compute_phase == msg
        assert n._station_compute_task_state == 'PLAN_PREGRASP'
        assert n.robot_execution_active
        return original(self, observation)

    monkeypatch.setattr(base.RemoteGrasp6DNode, '_register_near_field_surface', register)
    try:
        assert n._observe_return_retry_reached_view()
    finally:
        for worker in threads:
            worker.join(2.)
    assert len(calls) == 1 and threads and not failures
    assert finished.is_set() and not any(t.is_alive() for t in threads)
    assert len(captured) == 1 and n._latest_registration_evidence.result.ok
    if callback_kind == 'phase':
        assert n._station_compute_phase == replacement
    else:
        assert n._station_compute_task_state == 'FAILED'
        assert not n.robot_execution_active


@pytest.mark.parametrize('callback_kind', ['phase', 'task'])
def test_owner_callback_releases_short_guard_before_wait_or_cleanup(context, monkeypatch, callback_kind):
    c = context
    source, color, depth, msg, captured = pending_reached_retry(c, monkeypatch)
    n = c.node
    acquired = threading.Event()
    threads = []

    def slow_work(*args):
        # A second thread models input cancellation while reconstruction or
        # inactive-task cleanup is waiting. Both short owner locks must be free.
        def input_callback():
            with n._stream_condition, n._geometry_state_guard():
                acquired.set()
        worker = threading.Thread(target=input_callback, daemon=True)
        threads.append(worker)
        worker.start()
        assert acquired.wait(1.), 'slow callback retained registration locks'

    if callback_kind == 'phase':
        monkeypatch.setattr(n, '_observe_return_near_field_cb', slow_work)
    else:
        monkeypatch.setattr(n, 'stop_streaming', slow_work)
    try:
        if callback_kind == 'phase':
            n.near_field_state_cb(msg)
        else:
            n.grasp_state_cb(NS(state='FAILED', active=False))
    finally:
        for worker in threads:
            worker.join(2.)
    assert acquired.is_set() and not any(t.is_alive() for t in threads)
