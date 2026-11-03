"""Recorded R4 metadata/latency boundary; no reconstructed hardware authority.

The actual station wrapper/live feedback is tested separately in
 test_session_ack_contract_wait. This probe does NOT claim to reconstruct
non-wire qualified/handoff monotonic caches, or full robot state/actions.
"""
from copy import deepcopy
import hashlib
import json
import os
from pathlib import Path
from types import SimpleNamespace as NS

import pytest
from std_msgs.msg import String
from alicia_flexible_grasp_supervisor.msg import NearFieldPlanningPhase
from test_observe_return_task import context, task, module, at_ns

N = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_contact_admission_20261013T011103Z')
WIRE = N/'verification/attempt4_producer_timeline_r1/wire.json'
EXPECTED = {176:'624426854dc5ee7e87ac107118b32cbab4cb8ccb595d9d741b29354baac9821e',
181:'39bca4a1cd258ef7ca0eb2c12838aec6f032846b4a6724b8cf30597221f11fe8',
587:'c8b1229d027fd2a6ce6a47a9b87908afd996a6d6349571fb84b19468c7c5c8',
1001:'ca8ea7358f8d20ea5676154f80b93726d5a750fa1a75b54a12ca24f9b3618b5a',
1086:'71e189296a4ce11dedaf121880541f6186b43c26c366ce2fea16d5fa9802f27b'}
# The phase digest is retained from its original sealed wire entry below.
EXPECTED[587] = 'c8b1229d027fd2a6ce6a47a9b87908afd996a6d6349571fb84cfb19468c7c5c8'


def raw(row):
    payload = Path(row['file']).read_bytes()
    assert hashlib.sha256(payload).hexdigest() == row['sha256'] == EXPECTED[row['index']]
    if row['datatype'] == 'std_msgs/String':
        msg = String().deserialize(payload)
        assert json.loads(msg.data) == row['message']
        return msg
    msg = NearFieldPlanningPhase().deserialize(payload)
    assert msg.header.stamp.to_nsec() == row['message']['header']['stamp']
    assert msg.deadline.to_nsec() == row['message']['deadline']
    return msg


@pytest.mark.parametrize('scenario', ['recorded_rejected_ack_boundary', 'counterfactual_accepted_same_latency',
                                     'recorded_task_failure_boundary'])
def test_original_r4_session_raw_messages_and_latency(context, monkeypatch, scenario):
    c = context
    rows = {r['index']: r for r in json.loads(WIRE.read_text()) if r['index'] in EXPECTED}
    decoded = {i:raw(r) for i,r in rows.items()}
    first, request = [json.loads(decoded[i].data) for i in (176,1001)]
    c.selection = deepcopy(first['selection'])
    c.params[module.SELECTION_PARAM] = deepcopy(c.selection)
    c.node._mode_bound_selection = deepcopy(c.selection)
    c.node._mode_selection = deepcopy(c.selection)
    c.node._observe_return_session = deepcopy(first)
    c.node._observe_return_acks.clear()
    c.now[0] = int(first['stamp_ns'])
    origin = c.now[0]
    sent, warns = [], []
    monkeypatch.setattr(task.time, 'monotonic', lambda: (c.now[0]-origin)/1e9)
    monkeypatch.setattr(module.rospy, 'logwarn', lambda *args: warns.append(args))
    pending = [rows[181]['received_ns'], decoded[181]]
    def publish(msg):
        sent.append(json.loads(msg.data))
    c.node._observe_return_pub = NS(publish=publish)
    def wait(seconds):
        target = c.now[0] + round(seconds*1e9)
        if pending[0] is not None and target >= pending[0]:
            c.now[0] = pending[0]
            event = pending[1]
            pending[0] = None
            if callable(event): event()
            else: c.node._observe_return_ack_cb(event)
        else:
            c.now[0] = target
    monkeypatch.setattr(c.node._observe_return_condition, 'wait', wait)
    # Original initial accepted ACK establishes the real session's fixed cap.
    initial = c.node._observe_return_publish(deepcopy(first))
    assert initial == rows[181]['message'] and not warns
    c.node._published_near_field_phase = decoded[587]
    c.node._near_field_active = True
    c.now[0] = rows[1001]['received_ns']
    start = c.now[0]
    c.node._observe_return_acks.clear()
    pending[:] = [rows[1086]['received_ns'], decoded[1086]]
    if scenario == 'counterfactual_accepted_same_latency':
        from alicia_grasp_modes.observe_return_session import parse_session
        # Explicit conditional construction, NOT a recorded accepted reply.
        constructed = dict(parse_session(request,c.selection,c.now[0],first),accepted=True,reason='OK')
        pending[1] = String(data=json.dumps(constructed))
    elif scenario == 'recorded_task_failure_boundary':
        # R4 original FAILED header. It is an outcome of its original 5s
        # timeout, not an independent cause we silently remove from history.
        pending[:] = [1791884713327177524, lambda: setattr(c.node,'active',False)]
    error = None
    try:
        result = c.node._observe_return_publish(deepcopy(request))
    except ValueError as exc:
        error = str(exc)
    baseline = os.environ['GEN16_ACK_EXPECT_BASELINE'] == '1'
    if baseline:
        assert error == 'OBSERVE_RETURN_REMOTE_ACK_TIMEOUT'
        assert c.now[0]-start == 5_000_000_000
    elif scenario == 'counterfactual_accepted_same_latency':
        assert error is None and result == constructed
        assert c.now[0] == rows[1086]['received_ns'] and len(warns)==1
    elif scenario == 'recorded_rejected_ack_boundary':
        assert error == 'OBSERVE_RETURN_REMOTE_REJECTED: OBSERVE_RETURN_TARGET_CHANGED'
        assert c.now[0] == rows[1086]['received_ns'] and len(warns)==1
    else:
        assert error == 'OBSERVE_RETURN_OWNER_CHANGED'
        assert c.now[0] == 1791884713327177524
    assert sent == [first,request]
    assert c.gateways == [] and c.replans == []
    out = Path(os.environ['GEN16_ACK_PROBE_OUTPUT'])
    (out/(scenario+'.json')).write_text(json.dumps(dict(scenario=scenario,
        baseline=baseline,error=error,elapsed_ns=c.now[0]-start,
        original_ack_latency_ns=rows[1086]['received_ns']-start,
        original_ack_accepted=False,counterfactual=scenario=='counterfactual_accepted_same_latency',
        original_raw_sha256={str(i):r['sha256'] for i,r in rows.items()},
        wire_path=str(WIRE),wire_sha256=hashlib.sha256(WIRE.read_bytes()).hexdigest(),
        limitations=['metadata caller boundary only; original task/hardware memory not on wire',
          'qualified monotonic cache and station handoff memory not reconstructed',
          'actual station wrapper/SDK live feedback is separately exercised by author tests',
          'recorded rejected ACK never converted to accepted; constructed accepted case explicitly conditional'],
        physical_motion=False),indent=2)+'\n')
