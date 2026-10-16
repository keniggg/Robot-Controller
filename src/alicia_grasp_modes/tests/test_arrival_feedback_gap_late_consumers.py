"""Exact discarded-source receipts, real consumer callbacks, inert transport."""
from copy import deepcopy
from itertools import permutations

import pytest

from test_observation_measurement_remote import context, at
from test_station_compute_wait_remote import station as remote_station, arm, row_at, clock, feedback
from test_unknown_pending_identity import deliver
from test_task_modes import configured
from test_station_compute_wait_task import station as task_station
import test_station_compute_wait_task as task_case


def gap_receipt(parent, bad_ns):
    before = bad_ns-30_000_000
    after = before+226_932_289
    evidence = dict(accepted_before=[before, parent['binding']['station_q_rad']],
        accepted_after=[after, parent['binding']['station_q_rad']],
        maximum_span_ns=200_000_000, span_ns=after-before, bracket_qualified=False)
    return dict(discarded_source_stamp_ns=bad_ns, discarded_source_stamps_ns=[bad_ns],
        feedback_gap_evidence=deepcopy(evidence),
        discarded_source_evidence=[dict(source_stamp_ns=bad_ns, feedback_gap_evidence=deepcopy(evidence))],
        source_qualified=False, identity_verified=False, metric_geometry_authority=False,
        motion_authorized=False, reference_refreshed=False)


def rejected(row, parent, receipt):
    row=deepcopy(row)
    row['object'].detected=False
    row['mask'].data=bytes(len(row['mask'].data))
    row['status'].update(state='processing_deadline_pending',
        detail='station_accepted_feedback_bracket_pending_revalidation', target_lost=False, first_loss=None,
        station_compute_wait=deepcopy(parent), station_compute_recovery_chain=[deepcopy(parent)],
        station_feedback_gap=deepcopy(receipt))
    return row


def recover_remote(c, delay=.6):
    old_row=deepcopy(c.ready); parent=arm(c); n=c.node
    n._configured_near_field_strategy=lambda:'single_snapshot_direct'
    n.near_field_planning_snapshot_frames=3
    bad=parent['source_ns']+250_000_000
    receipt=gap_receipt(parent,bad)
    old=rejected(row_at(c,bad),parent,receipt)
    ns=parent['source_ns']+round(delay*1e9)
    feedback(c,ns-10_000_000);feedback(c,ns+10_000_000);clock(c,ns+50_000_000)
    fresh=row_at(c,ns,recovery=parent)
    fresh['status']['station_feedback_gap_recovery']=receipt
    deliver(n,fresh)
    assert not n._station_compute_pending and not n._station_compute_retired
    assert n._station_compute_reference['contract']['source_ns']==ns
    return old,old_row,parent,ns


@pytest.mark.parametrize('delay',[.6,.95])
@pytest.mark.parametrize('order',list(permutations(('object','mask','status'))))
def test_remote_gap_receipts_delayed_until_after_complete_new_ready(remote_station,delay,order):
    c=remote_station;old,_,_,ns=recover_remote(c,delay)
    saved=deepcopy(c.node._station_compute_reference)
    deliver(c.node,old,order)
    assert not c.node._station_compute_pending and not c.node._station_compute_retired
    assert c.node.latest_object.detected and c.node.latest_object.header.stamp.to_nsec()==ns
    assert c.node._station_compute_reference==saved


def recover_task(c):
    old_ready=task_case.frame(c);task_case.deliver(c,old_ready)
    parent=deepcopy(c.node._station_task_record['contract'])
    bad=parent['source_ns']+250_000_000;receipt=gap_receipt(parent,bad)
    old=rejected(task_case.frame(c,bad),parent,receipt)
    ns=parent['source_ns']+600_000_000
    task_case.advance(c,ns+50_000_000)
    task_case.feedback(c,ns-10_000_000);task_case.feedback(c,ns+10_000_000)
    fresh=task_case.frame(c,ns,parent)
    fresh['status']['station_feedback_gap_recovery']=receipt
    task_case.deliver(c,fresh)
    assert not c.node._station_task_record['pending'] and not c.node._station_task_retired
    assert c.node._station_task_record['contract']['source_ns']==ns
    return old,old_ready,parent,ns


@pytest.mark.parametrize('order',list(permutations(('object','mask','status'))))
def test_task_gap_receipts_delayed_until_after_complete_new_ready(task_station,order):
    c=task_station;old,_,_,ns=recover_task(c)
    saved=deepcopy(c.node._station_task_record)
    task_case.deliver(c,old,order)
    assert not c.node._station_task_retired and not c.node._station_task_record['pending']
    assert c.node.latest_obj.header.stamp.to_nsec()==ns
    assert c.node._station_task_record==saved


@pytest.mark.parametrize('fault',['true_loss','changed_ready','changed_object','changed_mask'])
def test_remote_tombstone_never_hides_real_failure_or_changed_history(remote_station,fault):
    c=remote_station;pending,ready,_,_=recover_remote(c)
    if fault=='true_loss':
        pending['status']['target_lost']=True;deliver(c.node,pending,('status',))
    elif fault=='changed_ready':
        ready['status']['station_compute_wait']['binding']['source_id']='f'*64
        deliver(c.node,ready,('status',))
    elif fault=='changed_object':
        ready['object'].confidence-=.01;deliver(c.node,ready,('object',))
    else:
        ready['mask'].data=bytes(len(ready['mask'].data));deliver(c.node,ready,('mask',))
    assert c.node._station_compute_retired


@pytest.mark.parametrize('fault',['true_loss','changed_ready','changed_object','changed_mask'])
def test_task_tombstone_never_hides_real_failure_or_changed_history(task_station,fault):
    c=task_station;pending,ready,_,_=recover_task(c)
    if fault=='true_loss':
        pending['status']['target_lost']=True;task_case.deliver(c,pending,('status',))
    elif fault=='changed_ready':
        ready['status']['station_compute_wait']['binding']['source_id']='f'*64
        task_case.deliver(c,ready,('status',))
    elif fault=='changed_object':
        ready['object'].confidence-=.01;task_case.deliver(c,ready,('object',))
    else:
        ready['mask'].data=bytes(len(ready['mask'].data));task_case.deliver(c,ready,('mask',))
    assert c.node._station_task_retired
