"""Original source lifetime and actual task/remote callbacks; inert transport."""
from copy import deepcopy
from itertools import permutations
from types import SimpleNamespace as NS

import pytest

from alicia_grasp_modes.station_compute_reacquisition import (
    build_station_support_gap, validate_station_support_gap)
from test_station_compute_reacquisition import contract
from test_observation_measurement_remote import context
from test_station_compute_wait_remote import station as remote_station
import test_station_compute_wait_remote as remote
from test_unknown_pending_identity import deliver as remote_deliver
from test_task_modes import configured
from test_station_compute_wait_task import station as task_station
import test_station_compute_wait_task as task


def receipt(parent, offset=300_000_000, first=None):
    return build_station_support_gap(parent, parent['source_ns']+offset,
        'support_plane_inlier_ratio_low', first_source_ns=first)


def rejected(row, parent, gap):
    row = deepcopy(row)
    row['object'].detected = False
    row['mask'].data = bytes(len(row['mask'].data))
    row['status'].update(state='processing_deadline_pending',
        detail='support_measurement_pending_revalidation', target_lost=False, first_loss=None,
        station_compute_wait=deepcopy(parent), station_compute_recovery_chain=[deepcopy(parent)],
        station_support_gap=deepcopy(gap))
    return row


def register_owner(which, c):
    if which == 'task':
        task.deliver(c, task.frame(c))
        parent = deepcopy(c.node._station_task_record['contract'])
    else:
        parent = remote.arm(c)
    return parent


def advance(which, c, ns):
    if which == 'task':
        task.advance(c, ns)
    else:
        remote.clock(c, ns)


def row(which, c, ns, parent=None):
    if which == 'task':
        return task.frame(c, ns, parent)
    return remote.row_at(c, ns, recovery=parent)


def deliver(which, c, row, order=('object', 'mask', 'status')):
    if which == 'task':
        task.deliver(c, row, order)
    else:
        remote_deliver(c.node, row, order)


def record(which, c):
    return c.node._station_task_record if which == 'task' else c.node._station_compute_reference


def pending(which, c):
    r = record(which, c)
    return r['pending'] if which == 'task' and r else c.node._station_compute_pending if which == 'remote' else False


def retired(which, c):
    return c.node._station_task_retired if which == 'task' else c.node._station_compute_retired


def tick(which, c):
    if which == 'task':
        c.node._station_compute_task_tick()
    else:
        c.node._station_compute_poll()


def fresh(which, c, parent, gap, offset=600_000_000):
    ns = parent['source_ns']+offset
    advance(which, c, ns+40_000_000)
    fn = task.feedback if which == 'task' else remote.feedback
    fn(c, ns-10_000_000); fn(c, ns+10_000_000)
    result = row(which, c, ns, parent)
    result['status']['station_support_gap_recovery'] = deepcopy(gap)
    return result


@pytest.fixture(params=['task', 'remote'])
def owner(request):
    name = request.param
    return name, request.getfixturevalue(name+'_station')


def test_gap_is_original_geometry_deadline_not_five_seconds(contract):
    gap = receipt(contract)
    assert gap['deadline_ns'] == contract['geometry_deadline_ns']
    assert gap['deadline_ns'] < contract['identity_deadline_ns']
    assert validate_station_support_gap(gap, contract, gap['source_ns']) == gap
    assert all(gap[k] is False for k in ('metric_geometry_authority','motion_authorized','reference_refreshed'))
    repeated = receipt(contract, 600_000_000, gap['first_source_ns'])
    assert repeated['deadline_ns'] == gap['deadline_ns']
    assert repeated['first_source_ns'] == gap['first_source_ns']


@pytest.mark.parametrize('key,value', [
    ('version', True), ('deadline_ns', 99), ('parent_source_ns', 1), ('source_id', 'f'*64),
    ('source_ns', True), ('first_source_ns', 1.), ('metric_geometry_authority', True),
    ('motion_authorized', 0), ('reference_refreshed', True), ('reason', 'identity_conflict'),
    ('policy', 'processing_deadline_pending')])
def test_receipt_cannot_forge_parent_clock_authority_or_reason(contract, key, value):
    gap = receipt(contract); gap[key] = value
    with pytest.raises(ValueError):
        validate_station_support_gap(gap, contract, contract['source_ns']+400_000_000)


@pytest.mark.parametrize('offset', [800_000_000, 800_000_001, 4_000_000_000])
def test_gap_expiry_cannot_be_replaced_by_parent_identity_window(contract, offset):
    with pytest.raises(ValueError):
        validate_station_support_gap(receipt(contract), contract, contract['source_ns']+offset)


@pytest.mark.parametrize('offset', [0, 200_000_000, 300_000_000, 750_000_001])
def test_recovery_must_follow_rejected_source_and_not_be_future(contract, offset):
    with pytest.raises(ValueError):
        validate_station_support_gap(receipt(contract), contract, contract['source_ns']+750_000_000,
            source_ns=contract['source_ns']+offset)


@pytest.mark.parametrize('order', list(permutations(('object','mask','status'))))
def test_actual_pending_withdraws_old_geometry_and_new_three_topic_join_recovers(owner, order):
    which,c=owner;parent=register_owner(which,c);gap=receipt(parent)
    advance(which,c,parent['source_ns']+400_000_000)
    bad=rejected(row(which,c,gap['source_ns']),parent,gap)
    deliver(which,c,bad,order)
    assert pending(which,c) and not retired(which,c)
    hold=deepcopy(record(which,c)['support_gap'])
    assert hold['receipt']['deadline_ns']==parent['geometry_deadline_ns']
    ready=fresh(which,c,parent,gap)
    for key in order[:-1]:
        deliver(which,c,ready,(key,));assert pending(which,c)
    deliver(which,c,ready,order[-1:])
    assert not pending(which,c) and not retired(which,c)
    assert record(which,c)['contract']['source_ns']==ready['status']['source_stamp_ns']
    assert 'support_gap' not in record(which,c)
    # The original false/status/mask arriving after recovery cannot revive it.
    deliver(which,c,bad,order)
    assert not pending(which,c) and not retired(which,c)
    assert record(which,c)['contract']['source_ns']==ready['status']['source_stamp_ns']


@pytest.mark.parametrize('order', list(permutations(('object','mask','status'))))
def test_recovery_receipt_requires_full_proof_even_when_pending_notice_was_missed(owner, order):
    which,c=owner;parent=register_owner(which,c);gap=receipt(parent)
    ready=fresh(which,c,parent,gap)
    deliver(which,c,ready,order)
    assert not pending(which,c) and not retired(which,c)
    assert record(which,c)['contract']['source_ns']==ready['status']['source_stamp_ns']


@pytest.mark.parametrize('fault', ['no_receipt','no_parent','source_before_gap','different_first',
    'extended_deadline','changed_owner','real_loss'])
def test_pending_cannot_recover_without_exact_parent_and_support_receipt(owner,fault):
    which,c=owner;parent=register_owner(which,c);gap=receipt(parent)
    advance(which,c,parent['source_ns']+400_000_000)
    deliver(which,c,rejected(row(which,c,gap['source_ns']),parent,gap))
    ready=fresh(which,c,parent,gap)
    if fault=='no_receipt':ready['status'].pop('station_support_gap_recovery')
    elif fault=='no_parent':ready['status'].pop('station_compute_recovery')
    elif fault=='source_before_gap':ready['status']['station_support_gap_recovery']['source_ns']=ready['status']['source_stamp_ns']
    elif fault=='different_first':ready['status']['station_support_gap_recovery']['first_source_ns']+=1
    elif fault=='extended_deadline':ready['status']['station_support_gap_recovery']['deadline_ns']+=1
    elif fault=='changed_owner':ready['status']['station_compute_recovery']['binding']['control_epoch_ns']+=1
    else:ready['status']['target_lost']=True;ready['status']['first_loss']={'reason':'identity_conflict'}
    deliver(which,c,ready)
    assert retired(which,c) or pending(which,c)
    assert record(which,c) is None or record(which,c)['contract']['source_ns']==parent['source_ns']


def test_duplicate_pending_cannot_extend_either_clock(owner):
    which,c=owner;parent=register_owner(which,c);gap=receipt(parent)
    advance(which,c,parent['source_ns']+400_000_000)
    bad=rejected(row(which,c,gap['source_ns']),parent,gap);deliver(which,c,bad)
    cap=record(which,c)['support_gap']['monotonic_deadline']
    advance(which,c,parent['source_ns']+700_000_000)
    repeated=receipt(parent,600_000_000,gap['first_source_ns'])
    deliver(which,c,rejected(row(which,c,repeated['source_ns']),parent,repeated))
    assert record(which,c)['support_gap']['monotonic_deadline']==cap
    advance(which,c,parent['geometry_deadline_ns']);tick(which,c)
    assert retired(which,c)


def test_late_complete_ready_cannot_register_under_five_second_identity(owner):
    which,c=owner;parent=register_owner(which,c);gap=receipt(parent)
    advance(which,c,parent['source_ns']+400_000_000)
    deliver(which,c,rejected(row(which,c,gap['source_ns']),parent,gap))
    ready=fresh(which,c,parent,gap,offset=810_000_000)
    deliver(which,c,ready)
    assert retired(which,c) or pending(which,c)
    assert record(which,c) is None or record(which,c)['contract']['source_ns']==parent['source_ns']


@pytest.mark.parametrize('key', ['phase_deadline_ns', 'session_deadline_ns'])
def test_parent_phase_and_session_can_only_shorten_gap(contract,key):
    from alicia_grasp_modes.station_compute_reacquisition import build_contract
    kwargs={k:deepcopy(contract[k]) for k in ('source_ns','maximum_frame_age_sec',
        'session_deadline_ns','phase_deadline_ns','binding')}
    kwargs[key]=contract['source_ns']+500_000_000
    parent=build_contract(**kwargs);gap=receipt(parent)
    assert gap['deadline_ns']==kwargs[key]
    with pytest.raises(ValueError):validate_station_support_gap(gap,parent,kwargs[key])


@pytest.mark.parametrize('fault',['manual','epoch','phase','monotonic_only'])
def test_live_owner_and_monotonic_failures_cannot_be_rescued_by_support_receipt(owner,monkeypatch,fault):
    which,c=owner;parent=register_owner(which,c);gap=receipt(parent)
    advance(which,c,parent['source_ns']+400_000_000)
    deliver(which,c,rejected(row(which,c,gap['source_ns']),parent,gap))
    if fault=='manual':c.params['/gui/joint_direct_mode']=True
    elif fault=='epoch':
        if which=='task':c.node._following_epoch_ns+=1
        else:c.node._measurement_remote_epoch_ns+=1
    elif fault=='phase':
        if which=='task':c.node._published_near_field_phase.phase_id+=1
        else:c.phase.phase_id+=1
    else:
        cap=record(which,c)['support_gap']['monotonic_deadline']
        from alicia_grasp_modes import station_compute_wait_remote as impl
        monkeypatch.setattr(impl.time,'monotonic',lambda:cap)
    tick(which,c)
    assert retired(which,c)


def test_support_receipt_cannot_create_initial_station_without_parent(owner):
    which,c=owner
    old=row(which,c,100_000_000_000) if which=='task' else deepcopy(c.ready)
    parent=old['status']['station_compute_wait'];gap=receipt(parent)
    # No old object/mask/status was joined in this test.
    ready=fresh(which,c,parent,gap)
    deliver(which,c,ready)
    assert record(which,c) is None


def test_recovery_original_object_gate_cannot_commit_after_fixed_deadline(owner,monkeypatch):
    which,c=owner;parent=register_owner(which,c);gap=receipt(parent)
    advance(which,c,parent['source_ns']+400_000_000)
    deliver(which,c,rejected(row(which,c,gap['source_ns']),parent,gap))
    ready=fresh(which,c,parent,gap)
    if which=='task':
        cls=task.base.GraspTaskNode;name='obj_cb'
    else:
        cls=remote.base.RemoteGrasp6DNode;name='object_cb'
    original=getattr(cls,name)
    called=[]
    def consume_clock(node,msg):
        result=original(node,msg)
        if node is c.node and msg.header.stamp==ready['object'].header.stamp:
            called.append(True)
            limit=gap['deadline_ns']+10_000_000
            if which=='task':
                c.mono[0]+=(limit-c.now[0])/1e9;c.now[0]=limit
            else:c.clock[0]=limit/1e9
        return result
    monkeypatch.setattr(cls,name,consume_clock)
    deliver(which,c,ready)
    assert called
    assert record(which,c) is None or record(which,c)['contract']['source_ns']==parent['source_ns']
    assert retired(which,c) or pending(which,c)
    if which=='task':assert c.node.latest_obj is None
    else:assert not getattr(c.node.latest_preview_rich_plan,'valid',False)
