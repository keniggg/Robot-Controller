"""Real rospy serial dispatch versus a held planning lock; no ROS master."""
from copy import deepcopy
import threading
from types import SimpleNamespace as NS

import pytest
from rospy.topics import _SubscriberImpl

from test_station_compute_wait_remote import station, arm
from test_observation_measurement_remote import context, at
from test_initial_preflight_remote import initial, request as initial_request
from alicia_grasp_modes import station_compute_wait_remote as implementation
from alicia_grasp_modes.observe_return_measurement import station_requalification_key


def test_real_serial_transport_ingress_remains_live_under_planning_lock(station, monkeypatch):
    c = station; n = c.node; arm(c)
    callbacks = []
    monkeypatch.setattr(implementation.rospy, 'Subscriber',
                        lambda topic, kind, cb, **kw: callbacks.append(cb))
    monkeypatch.setattr(implementation.rospy, 'Timer', lambda *a, **kw: NS())
    n._setup_station_compute_io()
    assert len(callbacks) == 1
    c.clock[0] += .85
    now = n._ros_source_clock_ns()
    messages = []
    for delta in (200000000, 100000000, 0):
        msg = deepcopy(n.latest_joint_state)
        msg.header.frame_id = 'sdk_measured'; msg.header.stamp = at(now-delta)
        messages.append(msg)
    entered = threading.Event(); finished = threading.Event(); errors = []
    def callback(msg):
        entered.set()
        callbacks[0](msg)
    transport = NS(callbacks=[(callback, None)], statistics_logger=None,
                   _invoke_callback=lambda msg, cb, args: cb(msg))
    def receive():
        try:
            _SubscriberImpl.receive_callback(transport, messages, None)
        except Exception as exc:
            errors.append(exc)
        finally:
            finished.set()
    worker = threading.Thread(target=receive, daemon=True)
    with n._stream_condition, n._geometry_state_guard():
        worker.start(); assert entered.wait(1.)
        completed_while_locked = finished.wait(.2)
        cached_latest = max((ns for ns, q in n._station_compute_feedback), default=0)
        try:
            n._station_compute_check_joints(c.binding)
            gate_reason = None
        except ValueError as exc:
            gate_reason = str(exc)
    worker.join(1.)
    assert not worker.is_alive() and not errors
    assert completed_while_locked, 'TCPROS dispatch blocked behind planning locks; gate=%s' % gate_reason
    assert cached_latest == now
    assert gate_reason is None


def ingress(node, ns, q=None, malformed=False):
    from sensor_msgs.msg import JointState
    msg = JointState()
    msg.header.frame_id = 'sdk_measured'; msg.header.stamp = at(ns)
    msg.name = ['joint%d' % i for i in range(1, 7)]
    msg.position = [0.]*6 if q is None else q
    if malformed:
        msg.name[-1] = msg.name[0]
    node._station_compute_feedback_ingress_cb(msg)


@pytest.mark.parametrize('fault', ['moved_returned', 'malformed', 'overrun'])
def test_station_deferred_checks_retain_every_unvalidated_fault(station, fault):
    c = station; n = c.node; parent = arm(c)
    # This case verifies permanent ingress-fault retention after the existing
    # one-shot station requalification has already been used. First-use drift
    # is separately covered by test_station_requalification_remote.
    n._station_requalification_used.add(station_requalification_key(parent))
    ns = n._ros_source_clock_ns(); c.clock[0] += .1
    with n._stream_condition:
        if fault == 'moved_returned':
            ingress(n, ns+100, [0., .005, 0., 0., 0., 0.])
            ingress(n, ns+200)
        elif fault == 'malformed':
            ingress(n, ns+100, malformed=True); ingress(n, ns+200)
        else:
            for i in range(129): ingress(n, ns+i+1)
        assert not n._station_compute_retired
    n._station_compute_accepted_cb(None)  # Actual timer validation entry.
    assert n._station_compute_retired
    reason = n._station_compute_feedback_diagnostic['reason']
    assert ('moved' if fault == 'moved_returned' else
            'invalid_feedback' if fault == 'malformed' else 'overrun') in reason


@pytest.mark.parametrize('fault', ['moved_returned', 'malformed', 'overrun'])
def test_initial_preflight_final_gate_sees_ingress_fault_before_timer(initial, fault):
    s = initial; n = s.n; assert initial_request(s)['accepted']
    ns = n._ros_source_clock_ns(); s.clock[0] += .1
    if fault == 'moved_returned':
        ingress(n, ns+100, [0., .005, 0., 0., 0., 0.]); ingress(n, ns+200)
    elif fault == 'malformed':
        ingress(n, ns+100, malformed=True); ingress(n, ns+200)
    else:
        for i in range(129): ingress(n, ns+i+1)
    with pytest.raises(ValueError, match='moved|invalid_feedback|overrun'):
        n._initial_preflight_context(verify_owner=True)
    reply = initial_request(s, 'commit')
    assert not reply['accepted']
    assert n._initial_preflight_remote['state'] != 'committed'


@pytest.mark.parametrize('fault', ['empty', 'stale', 'future', 'no_post_station_samples'])
def test_station_failure_diagnostic_identifies_real_source_gate(station, fault):
    c=station; n=c.node; arm(c)
    ns=n._ros_source_clock_ns()
    if fault == 'empty': n._station_compute_feedback.clear()
    elif fault == 'stale': c.clock[0] += .801
    elif fault == 'future': ingress(n,ns+1000000000)
    else:
        n._station_compute_feedback.clear()
        ingress(n,n._measurement_remote_request['source_stamps_ns'][-1]-1)
    with pytest.raises(ValueError, match='feedback_'+fault):
        n._station_compute_check_joints(c.binding)
    diagnostic=n._station_compute_feedback_diagnostic
    assert diagnostic['reason'].endswith(fault)
    assert diagnostic['validation_now_ns'] == n._ros_source_clock_ns()
    assert diagnostic['control_epoch_ns'] == c.binding['control_epoch_ns']


def test_owner_change_does_not_admit_prior_epoch_or_buffer_fault(station):
    c=station;n=c.node;arm(c);now=n._ros_source_clock_ns()
    old=n._station_compute_feedback
    old.snapshot(owner=('station','old'), epoch_ns=0, since_ns=0)
    for i in range(129): ingress(n,now+i+1)
    with pytest.raises(ValueError,match='overrun'):
        old.snapshot(owner=('station','old'))
    # The real reset installs an independent buffer; an in-flight old ingress
    # can only write its captured old buffer, never the new selection's cache.
    n._reset_station_compute_wait()
    old.append((now+300,[0.]*6))
    assert list(n._station_compute_feedback) == []
    ingress(n,now)
    rows,diag=n._station_compute_feedback.snapshot(
        owner=('station','new'),epoch_ns=now+1,since_ns=now+1)
    assert rows == []


def test_timer_delay_does_not_require_new_station_permissions(station):
    c=station;n=c.node;arm(c)
    binding=deepcopy(c.binding);epoch=n._measurement_remote_epoch_ns
    for i in range(3):
        c.clock[0] += .1; ingress(n,n._ros_source_clock_ns())
    n._station_compute_accepted_cb(None)
    assert not n._station_compute_retired
    assert n._station_compute_reference['contract']['binding'] == binding
    n._measurement_remote_epoch_ns=epoch+1
    with pytest.raises(ValueError,match='owner_changed'):
        n._station_compute_context(n._station_compute_reference['contract'])


def test_station_snapshot_arrival_uses_post_snapshot_clock(station, monkeypatch):
    c=station;n=c.node;arm(c)
    original=n._station_compute_feedback.snapshot
    def arriving(**kwargs):
        c.clock[0] += .05
        ingress(n,n._ros_source_clock_ns())
        return original(**kwargs)
    monkeypatch.setattr(n._station_compute_feedback,'snapshot',arriving)
    n._station_compute_check_joints(c.binding)
    assert n._station_compute_feedback_diagnostic['latest_source_age_ns'] == 0


def test_initial_snapshot_arrival_uses_post_snapshot_clock(initial, monkeypatch):
    s=initial;n=s.n;assert initial_request(s)['accepted']
    original=n._station_compute_feedback.snapshot
    def arriving(**kwargs):
        s.clock[0] += .05
        ingress(n,n._ros_source_clock_ns())
        return original(**kwargs)
    monkeypatch.setattr(n._station_compute_feedback,'snapshot',arriving)
    n._initial_preflight_context()
    assert n._station_compute_feedback_diagnostic['latest_source_age_ns'] == 0


def test_rejected_motion_stays_rejected_after_bad_row_leaves_buffer(station):
    c=station;n=c.node;parent=arm(c);ns=n._ros_source_clock_ns();c.clock[0]+=.1
    n._station_requalification_used.add(station_requalification_key(parent))
    ingress(n,ns+100,[.005,0.,0.,0.,0.,0.]);ingress(n,ns+200)
    with n._stream_condition, n._geometry_state_guard():
        with pytest.raises(ValueError,match='moved'):n._station_compute_check_joints(c.binding)
    for i in range(129):ingress(n,ns+300+i)
    with n._stream_condition, n._geometry_state_guard():
        with pytest.raises(ValueError,match='moved'):n._station_compute_check_joints(c.binding)
