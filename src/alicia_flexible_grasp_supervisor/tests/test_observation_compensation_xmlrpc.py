"""Actual compensation service with the ROS parameter XML-RPC wire boundary.

Planning and motion remain inert fixture operations.  The callback, service,
before_execute consumption and XML-RPC encoder/decoder are production paths.
"""
from copy import deepcopy
import json
from types import SimpleNamespace as NS
from xmlrpc.client import dumps, loads

import pytest
import rospy

from test_observation_compensation_gateway import gateway_case


EPISODE_PREFIX = '/supervisor/observation_compensation_episodes/'
GEN9_EPOCH_NS = 1791694455560621129


def xmlrpc_case(monkeypatch, epoch_ns, failure=None):
    n,p,plan,s,params,calls,receipt,cfg = gateway_case(
        monkeypatch, moving=True, epoch_ns=epoch_ns)
    writes = []
    epochs = []
    snapshot = n._endpoint_following_snapshot

    def measured(fk, epoch, context, **kwargs):
        epochs.append((epoch, context['epoch']))
        return snapshot(fk, epoch, context, **kwargs)

    n._endpoint_following_snapshot = measured

    def set_param(key, value):
        assert key.startswith(EPISODE_PREFIX)
        assert s['moves'] == 0, 'consumption must precede any motion'
        transmitted = deepcopy(value)
        if failure == 'serialize':
            transmitted['transport_failure'] = 2**31
        wire = dumps(('/offline_compensation_test', key, transmitted),
                     methodname='setParam')
        args, method = loads(wire)
        assert method == 'setParam' and args[1] == key
        writes.append((key, args[2]))
        if failure == 'write':
            raise RuntimeError('fixture parameter server write failed')
        params[key] = args[2]
        if failure == 'write_then_raise':
            raise RuntimeError('fixture parameter server reply lost')

    monkeypatch.setattr(rospy, 'set_param', set_param)
    return n,plan,s,params,receipt,writes,epochs


@pytest.mark.parametrize('epoch_ns', [2**31-1, 2**31, GEN9_EPOCH_NS])
def test_real_service_preserves_epoch_through_xmlrpc_and_consumes_once(
        monkeypatch, epoch_ns):
    n,plan,s,params,receipt,writes,epochs = xmlrpc_case(monkeypatch, epoch_ns)
    result = n.handle_compensate_observation(NS(plan=plan, execute=True))
    report = json.loads(result.message)
    assert result.success, report
    assert s['moves'] == 1 and report['physical_motion_submitted']
    assert report['complete_execution_path_checked']
    assert len(writes) == 1
    key, stored = writes[0]
    assert stored['epoch_ns'] == str(epoch_ns)
    assert type(stored['epoch_ns']) is str
    assert int(stored['epoch_ns']) == epoch_ns
    assert stored['plan_id'] == plan.plan_id
    assert stored['lease_id'] == receipt['lease_id']
    assert stored['state'] == 'consumed_before_submission'
    assert params[key] == stored
    assert epochs and all(type(v) is int and v == epoch_ns
                          for values in epochs for v in values)
    assert type(n._control_reference_epoch_ns) is int
    assert n._control_reference_epoch_ns == epoch_ns
    assert type(receipt['control_epoch_ns']) is int
    assert report['control_epoch_ns'] == epoch_ns
    assert type(report['control_epoch_ns']) is int
    repeated = n.handle_compensate_observation(NS(plan=plan, execute=True))
    assert not repeated.success and 'EPISODE_CONSUMED' in repeated.message
    assert s['moves'] == 1 and len(writes) == 1


@pytest.mark.parametrize('failure', ['serialize', 'write', 'write_then_raise'])
def test_parameter_transport_failure_never_submits_motion(monkeypatch, failure):
    n,plan,s,params,receipt,writes,epochs = xmlrpc_case(
        monkeypatch, GEN9_EPOCH_NS, failure=failure)
    result = n.handle_compensate_observation(NS(plan=plan, execute=True))
    report = json.loads(result.message)
    assert not result.success
    expected = {'serialize':'int exceeds XML-RPC limits',
                'write':'parameter server write failed',
                'write_then_raise':'parameter server reply lost'}[failure]
    assert expected in report['reason'], report
    assert report['complete_execution_path_checked']
    assert not report['physical_motion_submitted'] and s['moves'] == 0
    if failure == 'serialize':
        assert not writes
    else:
        assert len(writes) == 1
        assert writes[0][1]['epoch_ns'] == str(GEN9_EPOCH_NS)
    if failure == 'write_then_raise':
        repeated = n.handle_compensate_observation(NS(plan=plan, execute=True))
        assert not repeated.success and 'EPISODE_CONSUMED' in repeated.message
        assert s['moves'] == 0 and len(writes) == 1
    else:
        assert not any(key.startswith(EPISODE_PREFIX) for key in params)
