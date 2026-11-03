"""Router/consumer reset transactions with real rospy parameter cache code.

Only the XML-RPC server transport is inert. MasterProxy, ParamServerCache,
rospy.get_param and the actual router/consumer reset entries are exercised.
"""
from contextlib import contextmanager
from copy import deepcopy
import json
from types import SimpleNamespace as NS
from unittest.mock import patch

import pytest
import rospy
import rospy.client
import rospy.msproxy
import rospy.impl.paramserver

from test_remote_modes import configured as remote_fixture, module as remote_module
from test_task_modes import configured as task_fixture, module as task_module
from test_router_contract import router_fixture


KEY='/grasp_mode/selection'
OLD=dict(mode='unknown',strategy='observe_return',generation=9,
         stamp_ns='1791695267403706550')
NEW=dict(mode='unknown',strategy='observe_return',generation=10,
         stamp_ns='1791695929288217306')


def setup_owners(monkeypatch):
    remote,rparams=remote_fixture.__wrapped__(monkeypatch)
    remote._ros_source_clock_ns=lambda:int(NEW['stamp_ns'])+1_000_000_000
    rparams[KEY]=deepcopy(OLD)
    assert remote.reset_mode_cb(None).success
    task,tparams=task_fixture.__wrapped__(monkeypatch)
    tparams[KEY]=deepcopy(OLD)
    assert task.reset_mode_cb(None).success
    router,master=router_fixture()
    master.update(rparams);master.update(tparams);master[KEY]=deepcopy(OLD)
    router.policy.mode='unknown';router.policy.strategy='observe_return'
    router.policy.generation=10;router.policy.stamp_ns=int(NEW['stamp_ns'])
    caches={};proxies={};reads=[];responses=[]
    fault={'read':None}
    for owner in ('remote','task'):
        caches[owner]=rospy.impl.paramserver.ParamServerCache()
        proxy=rospy.msproxy.MasterProxy.__new__(rospy.msproxy.MasterProxy)
        def get_param(caller,name,_owner=owner):
            assert isinstance(caller,str)
            reads.append((_owner,name))
            if name==KEY and fault['read'] is not None:
                return fault['read'](_owner)
            return (1,'inert authoritative master',deepcopy(master.get(name,{})))
        proxy.target=NS(getParam=get_param,
            subscribeParam=lambda caller,uri,name:(1,'initial subscription',deepcopy(OLD)))
        proxies[owner]=proxy
        with patch.object(rospy.impl.paramserver,'get_param_server_cache',
                          lambda _owner=owner:caches[_owner]):
            assert proxy.get_param_cached(KEY)==OLD

    monkeypatch.setattr(rospy,'get_param',rospy.client.get_param)

    @contextmanager
    def consumer(owner):
        with patch.object(rospy.client,'_param_server',proxies[owner]), \
             patch.object(rospy.impl.paramserver,'get_param_server_cache',lambda:caches[owner]), \
             patch.object(rospy,'get_master',lambda:proxies[owner]):
            yield remote if owner=='remote' else task

    def reset(owner):
        with consumer(owner) as node:
            result=node.reset_mode_cb(None)
        responses.append((owner,result))
        return result
    router.reset_remote=lambda:reset('remote')
    router.reset_task=lambda:reset('task')
    return NS(router=router,master=master,caches=caches,proxies=proxies,
        nodes=dict(remote=remote,task=task),reads=reads,responses=responses,
        consumer=consumer,reset=reset,fault=fault)


def test_router_resets_both_consumers_before_committing_despite_stale_cache(monkeypatch):
    c=setup_owners(monkeypatch)
    c.router.apply_switch()
    assert c.router.policy.accepting,c.router.last_status
    assert [owner for owner,_ in c.responses]==['remote','task']
    for owner,result in c.responses:
        ack=json.loads(result.message)
        assert result.success and ack['ready']
        assert ack['generation']==10 and ack['stamp_ns']==int(NEW['stamp_ns'])
        assert c.nodes[owner]._mode_selection==dict(NEW,stamp_ns=int(NEW['stamp_ns']))
        assert c.nodes[owner]._mode_ready
        assert c.caches[owner].get(KEY)==OLD
        assert (owner,KEY) in c.reads
    assert json.loads(c.router.selection_pub.messages[-1].data)['generation']==10


@pytest.mark.parametrize('owner',['remote','task'])
def test_consumer_reset_does_not_acknowledge_cached_previous_generation(monkeypatch,owner):
    c=setup_owners(monkeypatch);c.master[KEY]=deepcopy(NEW)
    result=c.reset(owner)
    assert result.success,result.message
    assert json.loads(result.message)['generation']==10
    assert c.caches[owner].get(KEY)==OLD


@pytest.mark.parametrize('owner',['remote','task'])
@pytest.mark.parametrize('fault',['missing','malformed','transport'])
def test_authoritative_failure_cannot_fall_back_to_ready_cached_owner(monkeypatch,owner,fault):
    c=setup_owners(monkeypatch)
    def broken(_owner):
        if fault=='transport':raise RuntimeError('offline master unavailable')
        if fault=='missing':return (-1,'missing',{})
        return (1,'malformed',dict(NEW,generation=True))
    c.fault['read']=broken
    result=c.reset(owner)
    assert not result.success
    assert c.nodes[owner]._mode_selection['generation']==9
    # Failure before entering the reset transaction grants no new ACK and does
    # not erase the old local state; the router cannot commit the switch.
    assert c.nodes[owner]._mode_ready


@pytest.mark.parametrize('owner',['remote','task'])
@pytest.mark.parametrize('failure',['changed','transport'])
def test_master_change_during_reset_prevents_ready_ack(monkeypatch,owner,failure):
    c=setup_owners(monkeypatch);c.master[KEY]=deepcopy(NEW)
    node=c.nodes[owner]
    hook='_reset_station_compute_wait' if owner=='remote' else '_reset_station_compute_task'
    original=getattr(node,hook)
    def changed():
        original()
        if failure=='changed':
            c.master[KEY]=dict(NEW,generation=11,stamp_ns=str(int(NEW['stamp_ns'])+1))
        else:
            def unavailable(_owner):raise RuntimeError('master lost during reset')
            c.fault['read']=unavailable
    monkeypatch.setattr(node,hook,changed)
    result=c.reset(owner)
    assert not result.success
    assert not node._mode_ready


@pytest.mark.parametrize('owner',['remote','task'])
def test_same_generation_ready_fast_return_rechecks_master(monkeypatch,owner):
    c=setup_owners(monkeypatch)
    count=[0]
    def change_after_first_read(_owner):
        count[0]+=1
        return (1,'read',deepcopy(OLD if count[0]==1 else NEW))
    c.fault['read']=change_after_first_read
    result=c.reset(owner)
    assert not result.success
    assert count[0]>=2


@pytest.mark.parametrize('owner',['remote','task'])
def test_active_execution_reset_rejection_preserves_existing_authority(monkeypatch,owner):
    c=setup_owners(monkeypatch);c.master[KEY]=deepcopy(NEW);node=c.nodes[owner]
    field='robot_execution_active' if owner=='remote' else 'active'
    setattr(node,field,True)
    result=c.reset(owner)
    assert not result.success
    assert getattr(node,field) and node._mode_ready
    assert node._mode_selection['generation']==9


def test_late_remote_selection_uses_master_and_never_rolls_ready_owner_back(monkeypatch):
    c=setup_owners(monkeypatch);c.master[KEY]=deepcopy(NEW)
    with c.consumer('remote') as node:
        node.selection_cb(NS(data=json.dumps(NEW)))
        assert node._mode_ready and node._mode_selection['generation']==10
        node.selection_cb(NS(data=json.dumps(OLD)))
        assert node._mode_ready and node._mode_selection['generation']==10
    assert c.caches['remote'].get(KEY)==OLD


def test_ready_hot_selection_check_does_not_make_a_master_call(monkeypatch):
    c=setup_owners(monkeypatch)
    with c.consumer('remote') as node:
        c.reads.clear()
        assert node._selection_is_current()
        assert c.reads==[]


@pytest.mark.parametrize('entry',['request','stream'])
def test_immediate_start_after_reset_uses_master_despite_delayed_cache(monkeypatch,entry):
    c=setup_owners(monkeypatch);c.router.apply_switch()
    assert c.router.policy.accepting
    with c.consumer('remote') as node:
        if entry=='request':
            result=node.request_plan_cb(NS(trigger=True))
            assert result.success,result.message
        else:
            assert node.start_streaming()
        assert node.streaming_enabled
        assert c.caches['remote'].get(KEY)==OLD
        # A control request does not create fresh cache/source evidence.
        before=len(c.reads)
        assert not node._selection_is_current()
        assert len(c.reads)==before


@pytest.mark.parametrize('entry',['request','stream'])
@pytest.mark.parametrize('failure',['changed','transport','not_ready'])
def test_start_rejects_invalid_master_or_uncompleted_reset(monkeypatch,entry,failure):
    c=setup_owners(monkeypatch);c.master[KEY]=deepcopy(NEW)
    assert c.reset('remote').success
    node=c.nodes['remote']
    if failure=='changed':
        c.master[KEY]=dict(NEW,generation=11,stamp_ns=str(int(NEW['stamp_ns'])+1))
    elif failure=='transport':
        def unavailable(_owner):raise RuntimeError('master unavailable at start')
        c.fault['read']=unavailable
    else:node._mode_ready=False
    before=node._stream_generation
    with c.consumer('remote'):
        result=node.request_plan_cb(NS(trigger=True)) if entry=='request' else node.start_streaming()
    assert not (result.success if entry=='request' else result)
    assert not node.streaming_enabled and node._stream_generation==before


@pytest.mark.parametrize('already_running',[False,True])
def test_master_changes_between_start_checks_never_acknowledges_success(monkeypatch,already_running):
    c=setup_owners(monkeypatch);c.master[KEY]=deepcopy(NEW)
    assert c.reset('remote').success
    with c.consumer('remote') as node:
        if already_running:assert node.start_streaming()
        before=node._stream_generation
        count=[0]
        def changed(_owner):
            count[0]+=1
            return (1,'raced master',deepcopy(NEW if count[0]==1 else
                dict(NEW,generation=11,stamp_ns=str(int(NEW['stamp_ns'])+1))))
        c.fault['read']=changed
        result=node.request_plan_cb(NS(trigger=True))
        assert not result.success,result.message
        assert count[0]>=2
        assert node._stream_generation==before
        assert node.streaming_enabled is already_running


def test_stop_request_remains_available_without_current_owner(monkeypatch):
    c=setup_owners(monkeypatch)
    with c.consumer('remote') as node:
        assert node.start_streaming()
        def unavailable(_owner):raise RuntimeError('master unavailable during stop')
        c.fault['read']=unavailable
        result=node.request_plan_cb(NS(trigger=False))
        assert result.success and not node.streaming_enabled
