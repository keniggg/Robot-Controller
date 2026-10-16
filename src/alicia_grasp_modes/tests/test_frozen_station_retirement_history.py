"""Actual freeze/three-owner retirement; synthetic RGBD and recorded geometry.

The existing fixture supplies the geometric audit report, inert RPC/transport
and action sentinel. Real registration, audit commit, freeze and retirement
validators run; these tests never establish physical grasp success.
"""
from copy import deepcopy
import json
import hashlib
import inspect
from pathlib import Path

import numpy as np
import pytest
from std_msgs.msg import Header, String
from alicia_flexible_grasp_supervisor.msg import GraspState
from test_gen11_contact_compute_admission_independent import (
    context, computed, audit, freeze, retirement, feed_feedback, custom_frame,
    publication, at, nsnow)


def frozen_gap(monkeypatch, tmp_path):
    c = computed(context(monkeypatch))
    root = Path(__file__).resolve().parents[1]/'src/alicia_grasp_modes'
    loaded = {}
    for method, name in [(c.node._contact_station_live_context, 'contact_station_remote.py'),
                         (c.node._station_contact_preserve_history, 'station_compute_wait_remote.py')]:
        path = Path(inspect.getsourcefile(method)).resolve()
        assert path == root/name
        loaded[str(path)] = hashlib.sha256(path.read_bytes()).hexdigest()
    (tmp_path/'loaded_sources.json').write_text(json.dumps(loaded, indent=2)+'\n')
    audit(c, tmp_path)
    c.frozen = freeze(c)
    c.original_contract = deepcopy(c.node._station_compute_reference['contract'])
    c.clock[0] += .82
    c.joint(c.clock[0]); feed_feedback(c)
    c.node._station_compute_poll()
    assert c.node._station_compute_pending and c.node._station_compute_soft_wait
    assert not c.node._station_compute_retired
    assert c.node._station_compute_work_certificate(c.ticket)
    custom_frame(c.producer, c.clock, c.joint, target_depth=.35)
    feed_feedback(c)
    c.fresh = publication(c.producer)
    assert c.fresh['status']['state'] == 'ready'
    assert c.fresh['object'].detected and c.producer.first_loss is None
    assert c.task._station_contact_is_admitted(c.frozen)
    return c


def hold(c):
    state = GraspState(state='MOVE_PREGRASP', active=True)
    c.producer._station_compute_state_cb(state); c.node.grasp_state_cb(state)
    c.node._station_compute_poll()
    assert c.node._station_contact_history == dict(contract=c.original_contract)
    assert c.node._station_compute_reference is None
    assert c.node._station_compute_pending and c.node._station_compute_soft_wait
    assert not c.node._station_compute_work_certificate(c.ticket)
    assert c.node._station_compute_ticket_code(c.ticket) == 'TARGET_PROCESSING_PENDING'


@pytest.mark.parametrize('tick', ['before_prepare', 'deferred'])
def test_frozen_clock_gap_exact_retirement_reaches_original_action_only_once(monkeypatch, tmp_path, tick):
    c = frozen_gap(monkeypatch, tmp_path)
    plan = deepcopy(c.frozen)
    publications = len(c.task._frozen_contact_pub.messages)
    retirement(c, c.frozen, tick_at=tick)
    n = c.node
    assert n._station_contact_history is None and n._station_compute_reference is None
    assert n._station_compute_pending  # No READY or computation permission was restored.
    assert not n._station_compute_work and not n._station_compute_work_certificate(c.ticket)
    assert not n._station_compute_retired  # Identity was not declared lost.
    assert c.task._bound_execution_plan == plan
    assert len(c.task._frozen_contact_pub.messages) == publications == 1
    assert len(c.motion_sentinel) == 1
    for request in c.handshakes:
        n._contact_station_request_cb(String(data=json.dumps(request)))
        assert json.loads(n._contact_station_ack_pub.messages[-1].data)['accepted'] is False
    assert len(c.motion_sentinel) == 1


@pytest.mark.parametrize('fault', ['preview_only', 'uncommitted', 'audit_changed', 'request_digest',
    'manual', 'control_epoch', 'generation', 'phase', 'phase_deadline', 'session_deadline',
    'target_loss', 'first_loss', 'false_object', 'empty_mask', 'historical_object', 'hard_pending'])
def test_historical_retirement_never_hides_real_fault(monkeypatch, tmp_path, fault):
    c = frozen_gap(monkeypatch, tmp_path)
    hold(c)
    n = c.node
    original = n._contact_station_request_cb
    injected = []
    def receive(msg):
        req = json.loads(msg.data)
        if not injected:
            injected.append(True)
            proof = n._station_compute_work[c.ticket.request_id]
            if fault == 'preview_only': proof['contact_published'] = False
            elif fault == 'uncommitted': proof.pop('contact_audit')
            elif fault == 'audit_changed': n._latest_gate_audit_reference = dict(n._latest_gate_audit_reference, report_sha256='f'*64)
            elif fault == 'request_digest':
                req['plan_sha256'] = 'f'*64; msg = String(data=json.dumps(req))
            elif fault == 'manual': c.params['/gui/joint_direct_mode'] = True
            elif fault == 'control_epoch':
                n._measurement_remote_epoch_cb(Header(stamp=at(n._measurement_remote_epoch_ns+1), frame_id='sdk_reference_epoch'))
            elif fault == 'generation':
                before = n._stream_generation
                assert n._advance_target_instance_epoch('MODEL_RELOADED', preserve_identity=True)
                assert n._stream_generation != before
            elif fault == 'phase':
                phase = deepcopy(c.phase); phase.active = False
                n.near_field_state_cb(phase)
            elif fault == 'phase_deadline': c.clock[0] = c.phase.deadline.to_sec()
            elif fault == 'session_deadline': c.clock[0] = c.session['deadline_ns']/1e9
            elif fault in ('target_loss', 'first_loss'):
                status = deepcopy(c.fresh['status'])
                if fault == 'target_loss': status['target_lost'] = True
                else: status['first_loss'] = dict(reason='true independent source loss')
                n._station_compute_status_cb(status)
                assert n._station_compute_retired and n._station_contact_history is None
            elif fault in ('false_object', 'historical_object'):
                obj = deepcopy(c.fresh['object'])
                if fault == 'false_object': obj.detected = False
                else:
                    obj = deepcopy(c.ready['object']); obj.pose_base.pose.position.x += .01
                n.object_cb(obj)
                assert n._station_compute_retired and n._station_contact_history is None
            elif fault == 'empty_mask':
                msgmask = deepcopy(c.fresh['mask'])
                pixels = n.bridge.imgmsg_to_cv2(msgmask, desired_encoding='mono8')
                empty = n.bridge.cv2_to_imgmsg(np.zeros_like(pixels), encoding='mono8')
                empty.header = deepcopy(msgmask.header); n.mask_cb(empty)
                assert n._station_compute_retired and n._station_contact_history is None
            else:
                n._station_compute_quarantine(c.fresh['object'].header.stamp.to_nsec())
                assert n._station_compute_retired and n._station_contact_history is None
        original(msg)
        if fault in ('phase_deadline', 'session_deadline'):
            assert not n._contact_station_ack_pub.messages
            raise ValueError('original expired-request parser emitted no ACK')
    monkeypatch.setattr(n, '_contact_station_request_cb', receive)
    with pytest.raises((AssertionError, ValueError, RuntimeError)):
        retirement(c, c.frozen)
    assert injected
    assert not getattr(c, 'motion_sentinel', [])
    assert all(not json.loads(m.data)['accepted'] for m in n._contact_station_ack_pub.messages)


@pytest.mark.parametrize('fault', ['manual', 'generation', 'target_loss', 'plan_digest'])
def test_actual_prepare_does_not_hide_change_before_actual_retire(monkeypatch, tmp_path, fault):
    c = frozen_gap(monkeypatch, tmp_path)
    hold(c)
    n = c.node; original = n._contact_station_request_cb
    prepared = []
    def receive(msg):
        req = json.loads(msg.data)
        if req['state'] == 'retire':
            assert prepared
            if fault == 'manual': c.params['/gui/joint_direct_mode'] = True
            elif fault == 'generation': n._advance_target_instance_epoch('MODEL_RELOADED', preserve_identity=True)
            elif fault == 'target_loss':
                status = deepcopy(c.fresh['status']); status['target_lost'] = True
                n._station_compute_status_cb(status)
            else:
                n._station_compute_work[c.ticket.request_id]['contact_proposal'].poses[0].position.x += .001
        original(msg)
        if req['state'] == 'prepare':
            assert json.loads(n._contact_station_ack_pub.messages[-1].data)['accepted']
            prepared.append(True)
    monkeypatch.setattr(n, '_contact_station_request_cb', receive)
    with pytest.raises((AssertionError, ValueError, RuntimeError)):
        retirement(c, c.frozen)
    assert prepared and not getattr(c, 'motion_sentinel', [])
    assert not json.loads(n._contact_station_ack_pub.messages[-1].data)['accepted']
