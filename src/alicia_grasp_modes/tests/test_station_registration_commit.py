"""Real remote callback expiry during post-gate commit; no ROS control."""
from copy import deepcopy
from types import SimpleNamespace as NS
import pytest
from test_observation_measurement_remote import context
from test_station_compute_wait_remote import station, arm, pending, clock, row_at, deliver, base


def expire_in_original_object(monkeypatch, c, ns, *, delta=810_000_000, mutate=None):
    original=base.RemoteGrasp6DNode.object_cb
    def delayed(n,msg):
        result=original(n,msg)
        if msg.header.stamp.to_nsec()==ns:
            clock(c,ns+delta)
            if mutate:mutate(c)
        return result
    monkeypatch.setattr(base.RemoteGrasp6DNode,'object_cb',delayed)


def test_original_gate_expiry_discards_without_retiring_and_new_frame_recovers(station,monkeypatch):
    c=station;n=c.node;old=pending(c)
    saved=deepcopy(n._station_compute_reference)
    ns=n._ros_source_clock_ns()+100_000_000
    clock(c,ns+10_000_000)
    expire_in_original_object(monkeypatch,c,ns)
    deliver(n,row_at(c,ns,recovery=old))
    assert not n._station_compute_retired
    assert n._station_compute_pending
    assert n._station_compute_reference['contract']==saved['contract']
    assert n._station_compute_reference['identity_monotonic']==saved['identity_monotonic']
    assert n._station_compute_cutoff_ns>=ns
    assert not n.geometry_pub.messages[-1].valid
    rejected=NS(payload=(NS(stamp_ns=ns,sample_stamp_ns=(ns,)),))
    assert n._station_compute_ticket_code(rejected)=='TARGET_PROCESSING_PENDING'
    fresh_ns=n._ros_source_clock_ns()+100_000_000
    clock(c,fresh_ns+10_000_000)
    deliver(n,row_at(c,fresh_ns,recovery=old))
    assert not n._station_compute_retired and not n._station_compute_pending
    assert n._station_compute_reference['contract']['source_ns']==fresh_ns
    assert n._current_stream_target_identity()==c.identity
    assert n._station_compute_ticket_code(rejected)=='TARGET_PROCESSING_PENDING'


def test_expired_commit_cannot_renew_wait_without_new_evidence(station,monkeypatch):
    c=station;n=c.node;old=pending(c)
    saved=deepcopy(n._station_compute_reference)
    ns=n._ros_source_clock_ns()+100_000_000
    clock(c,ns+10_000_000)
    expire_in_original_object(monkeypatch,c,ns)
    deliver(n,row_at(c,ns,recovery=old))
    assert not n._station_compute_retired and n._station_compute_pending
    assert n._station_compute_reference['identity_monotonic']==saved['identity_monotonic']
    clock(c,old['identity_deadline_ns']+1_000_000)
    n._station_compute_poll()
    assert n._station_compute_retired and n._station_compute_reference is None


@pytest.mark.parametrize('fault',['manual','epoch','low_confidence','identity_deadline'])
def test_commit_expiry_does_not_hide_real_loss(station,monkeypatch,fault):
    c=station;n=c.node;old=pending(c)
    ns=n._ros_source_clock_ns()+100_000_000
    clock(c,ns+10_000_000)
    row=row_at(c,ns,recovery=old)
    mutate=None;delta=810_000_000
    if fault=='manual':mutate=lambda c:c.params.__setitem__('/gui/joint_direct_mode',True)
    elif fault=='epoch':mutate=lambda c:setattr(c.node,'_measurement_remote_epoch_ns',c.binding['control_epoch_ns']+1)
    elif fault=='low_confidence':row['object'].confidence=.1
    else:delta=old['identity_deadline_ns']-ns+1_000_000
    expire_in_original_object(monkeypatch,c,ns,delta=delta,mutate=mutate)
    deliver(n,row)
    assert n._station_compute_retired


@pytest.mark.parametrize('start_pending', [False, True])
@pytest.mark.parametrize('offset', [300_000_000, 100_000_000])
def test_cached_successor_uses_current_clock_and_only_fresh_source_recovers(station,monkeypatch,offset,start_pending):
    c=station;n=c.node;old=pending(c) if start_pending else arm(c)
    first_ns=n._ros_source_clock_ns()+100_000_000
    next_ns=first_ns+offset
    rows=[row_at(c,ns,recovery=old) for ns in (first_ns,next_ns)]
    # Preserve the real callback/cache path while batching messages received
    # before a worker releases its lock, as in the recorded failure.
    join=n._station_compute_join_locked
    with monkeypatch.context() as queued:
        queued.setattr(n,'_station_compute_join_locked',lambda:None)
        clock(c,first_ns+10_000_000)
        deliver(n,rows[0])
        clock(c,next_ns-10_000_000)
        clock(c,next_ns+10_000_000)
        deliver(n,rows[1])
    original=base.RemoteGrasp6DNode.object_cb
    processed=[]
    def delayed(node,msg):
        processed.append(msg.header.stamp.to_nsec())
        result=original(node,msg)
        if msg.header.stamp.to_nsec()==first_ns:
            clock(c,first_ns+950_000_000)
        return result
    monkeypatch.setattr(base.RemoteGrasp6DNode,'object_cb',delayed)
    with n._mode_control_lock,n._stream_condition,n._geometry_state_guard():join()
    assert not n._station_compute_retired, n.geometry_pub.messages[-1].failure_reason
    if offset==300_000_000:
        assert processed==[first_ns,next_ns]
        assert not n._station_compute_pending
        assert n._station_compute_reference['contract']['source_ns']==next_ns
    else:
        # The second source expired while processing the first: it must never
        # be fed into the original gates using an earlier loop timestamp.
        assert processed==[first_ns]
        assert n._station_compute_pending
        assert n._station_compute_reference['contract']==old
    assert n._station_compute_cutoff_ns>=first_ns
    rejected=NS(payload=(NS(stamp_ns=first_ns,sample_stamp_ns=(first_ns,)),))
    assert n._station_compute_ticket_code(rejected)=='TARGET_PROCESSING_PENDING'


def test_first_registration_expiring_in_original_gate_cannot_create_wait(station,monkeypatch):
    c=station;n=c.node
    ns=c.ready['status']['source_stamp_ns']
    expire_in_original_object(monkeypatch,c,ns)
    deliver(n,c.ready)
    assert n._station_compute_reference is None
    assert not n._station_compute_pending


def test_original_identity_rejection_clearing_caches_does_not_recover(station,monkeypatch):
    c=station;n=c.node;old=pending(c)
    ns=n._ros_source_clock_ns()+100_000_000
    clock(c,ns+10_000_000)
    row=row_at(c,ns,recovery=old)
    row['object'].pose_base.pose.position.x+=.09
    # This passes the base jump filter but changes the actual target epoch;
    # the original callback clears frame caches as a side effect.
    expire_in_original_object(monkeypatch,c,ns)
    deliver(n,row)
    assert n._station_compute_retired
    assert n._current_stream_target_identity()!=c.identity
    assert n._station_compute_reference is None
