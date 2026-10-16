"""Real callbacks, serialized plans and source-paired test images, never ROS.

The producer's three-frame proof is tested in its own suite; this file tests
the consumer's exact receipt binding and final real-source join.
"""
from copy import deepcopy
import itertools
import json
from types import SimpleNamespace as NS

import pytest
from std_msgs.msg import String, Header

from test_locked_rgb_hold import locked
from test_unknown_pending_identity import consumer_for, publication, deliver
from test_remote_modes import module, base, Publisher
from alicia_grasp_modes.observe_return_measurement import measurement_ack
from alicia_flexible_grasp.robot.observation_path_guard import committed_plan_digest


def at(ns):
    return module.rospy.Time(ns // 10**9, ns % 10**9)


def request(node, value):
    node._measurement_remote_request_cb(String(data=json.dumps(value)))
    return json.loads(node._measurement_remote_ack_pub.messages[-1].data)


@pytest.fixture
def context(monkeypatch):
    producer, clock, feed, joint, params = locked(monkeypatch)
    node = consumer_for(producer, clock, params)
    # Both owners share the same exact ROS clock. The older consumer fixture
    # rounds its float clock whereas rospy.Time.from_sec truncates it.
    node._ros_source_clock_ns = lambda: module.rospy.Time.now().to_nsec()
    row = publication(producer)
    deliver(node, row)
    identity = node._current_stream_target_identity()
    now = node._ros_source_clock_ns()
    source = row['object'].header.stamp.to_nsec()
    plan = base.Grasp6DPlan()
    plan.valid = True
    plan.plan_id = 'real-wire-plan'
    plan.header.frame_id = 'base_link'
    plan.header.stamp = at(source)
    plan.model_choice = 'unknown_tabletop'
    plan.target_track_id = identity.track_id
    plan.diagnostic = 'FAR_FIELD_OBSERVATION_PLAN'
    node._observe_return_prefix = deepcopy(plan)
    node._observe_return_origin_prefix = deepcopy(plan)
    node._observe_return_phase = 'observing'
    node._observe_return_paused = True
    node._observe_return_saved = object()
    node._measurement_remote_ack_pub = Publisher()
    node._measurement_remote_epoch_ns = producer._rgb_interest_epoch
    session = dict(version=2, session_id='s', selection=deepcopy(node._mode_selection),
        phase='observing', target_epoch=identity.epoch, target_track_id=identity.track_id,
        origin_plan_id=plan.plan_id, origin_snapshot_stamp_ns=source,
        view_plan_id=plan.plan_id, view_source_stamp_ns=source, view_index=1,
        deadline_ns=now + 60_000_000_000)
    node._observe_return_session = session
    req = dict(session, version=1, state='executing', lease_id='lease', request_id='start',
        plan_sha256=committed_plan_digest(plan), control_epoch_ns=node._measurement_remote_epoch_ns,
        stamp_ns=now, motion_kind='observation', arrival_stamp_ns=0, capture_deadline_ns=0,
        metric_geometry_authority=False, motion_authorized=False)
    req.pop('phase')
    return NS(node=node, clock=clock, ready=row, request=req, identity=identity, params=params,
              producer=producer, joint=joint)


def begin(c):
    ack = request(c.node, c.request)
    assert ack['accepted'], ack
    return c.node._measurement_remote_request


def arrival(c):
    previous = begin(c)
    c.clock[0] += 5.
    now = c.node._ros_source_clock_ns()
    req = dict(previous, state='requalifying', request_id='arrived', stamp_ns=now,
        arrival_stamp_ns=now, capture_deadline_ns=now + 3_000_000_000,
        settle=dict(epoch_ns=previous['control_epoch_ns'],
            stationary_window_start_ns=now-300_000_000, stationary_window_end_ns=now-1))
    ack = request(c.node, req)
    assert ack['accepted'], ack
    return c.node._measurement_remote_request


def fresh(c, req):
    row = deepcopy(c.ready)
    sources = [req['arrival_stamp_ns'] + i * 100_000_000 for i in (1, 2, 3)]
    c.clock[0] = (sources[-1] + 10_000_000) / 1e9
    ns = sources[-1]
    for name in ('object', 'mask'):
        row[name].header.stamp = at(ns)
    for key in ('source_stamp_ns', 'color_stamp_ns', 'depth_stamp_ns', 'tf_lookup_stamp_ns'):
        row['status'][key] = ns
    row['status']['observation_measurement'] = dict(request=deepcopy(req), state='qualified',
        qualified=True, source_stamps_ns=sources, metric_geometry_authority=False, motion_authorized=False,
        measurement_evaluated=True, exact_tf_consumed=True)
    row['ack'] = measurement_ack(req, 'unknown', accepted=True, qualified=True, source_stamps_ns=sources)
    return row, sources


def deliver_one(c, row, key):
    if key == 'ack':
        c.node._measurement_remote_ack_cb(String(data=json.dumps(row['ack'])))
    elif key == 'status':
        c.node._silhouette_source_status_cb(String(data=json.dumps(row['status'])))
    else:
        getattr(c.node, key + '_cb')(deepcopy(row[key]))


def qualified_ack(c):
    return [json.loads(m.data) for m in c.node._measurement_remote_ack_pub.messages
            if json.loads(m.data).get('qualified')]


@pytest.mark.parametrize('order', list(itertools.permutations(('object', 'mask', 'status', 'ack'))))
def test_three_frame_owner_proof_joins_only_actual_final_frame_then_waits_for_task(context, order):
    c = context
    saved = c.node._observe_return_saved
    req = arrival(c)
    row, sources = fresh(c, req)
    old_ns = c.node.latest_object.header.stamp.to_nsec()
    for key in order[:-1]:
        deliver_one(c, row, key)
        assert c.node.latest_object.header.stamp.to_nsec() == old_ns
        assert not qualified_ack(c)
    deliver_one(c, row, order[-1])
    assert qualified_ack(c)[-1]['source_stamps_ns'] == sources
    assert c.node.latest_object.header.stamp.to_nsec() == sources[-1]
    assert c.node._current_stream_target_identity() == c.identity
    assert c.node._observe_return_saved is saved
    assert c.node._observe_return_paused and c.node._poll_stream_snapshot() is False
    assert c.node._measurement_remote_request['state'] == 'requalifying'
    c.clock[0] += .01
    done = dict(req, state='qualified', request_id='done', stamp_ns=c.node._ros_source_clock_ns(),
                source_stamps_ns=sources)
    assert request(c.node, done)['accepted']
    assert not c.node._measurement_remote_active()
    assert c.node._observe_return_paused and c.node._poll_stream_snapshot() is False
    assert c.node._observe_return_saved is saved


def test_valid_moving_lease_never_refreshes_old_source_or_publishes_geometry(context):
    c = context
    begin(c)
    old = deepcopy(c.node.latest_object)
    old_time = c.node.latest_object_time
    saved = c.node._observe_return_saved
    count = len(c.node.geometry_pub.messages)
    c.clock[0] += 20.
    pending = deepcopy(c.ready['status'])
    pending.update(state='unavailable', detail='observation_measurement_executing')
    pending['observation_measurement'] = dict(request=c.request, state='executing', qualified=False,
        metric_geometry_authority=False, motion_authorized=False, source_stamps_ns=[])
    c.node._silhouette_source_status_cb(String(data=json.dumps(pending)))
    assert c.node._poll_stream_snapshot() is False
    assert c.node.latest_object == old and c.node.latest_object_time == old_time
    assert c.node._current_stream_target_identity() == c.identity
    assert c.node._observe_return_saved is saved
    assert len(c.node.geometry_pub.messages) == count


@pytest.mark.parametrize('fault', ['epoch', 'session', 'source', 'plan_digest', 'selection',
    'not_paused', 'diagnostic', 'lost', 'direct', 'stale_reference'])
def test_admission_cannot_borrow_other_authority_or_revive_an_old_target(context, fault):
    c = context
    r = deepcopy(c.request)
    if fault == 'epoch': r['control_epoch_ns'] -= 1
    elif fault == 'session': r['session_id'] = 'wrong'
    elif fault == 'source': r['view_source_stamp_ns'] -= 1
    elif fault == 'plan_digest': r['plan_sha256'] = 'f'*64
    elif fault == 'selection': r['selection']['generation'] += 1
    elif fault == 'not_paused': c.node._observe_return_paused = False
    elif fault == 'diagnostic': c.node._observe_return_diagnostic_observation_only = True
    elif fault == 'lost': c.node._unknown_identity_retired_selection = deepcopy(c.node._mode_selection)
    elif fault == 'direct': c.node._observe_return_phase = 'initial_contact'
    elif fault == 'stale_reference': c.clock[0] += 1.
    assert request(c.node, r)['accepted'] is False
    assert not c.node._measurement_remote_active()


@pytest.mark.parametrize('fault', ['object_false', 'producer_lost', 'epoch', 'expiry', 'revoked'])
def test_true_retirement_remains_irreversible(context, fault):
    c = context
    req = begin(c)
    if fault == 'object_false':
        obj = deepcopy(c.ready['object']);obj.detected = False;c.node.object_cb(obj)
    elif fault == 'producer_lost':
        s = deepcopy(c.ready['status']);s['target_lost'] = True
        c.node._silhouette_source_status_cb(String(data=json.dumps(s)))
    elif fault == 'epoch':
        c.node._measurement_remote_epoch_cb(Header(stamp=at(req['control_epoch_ns']+1),
            frame_id='sdk_reference_epoch'))
    elif fault == 'expiry':
        c.clock[0] = req['deadline_ns']/1e9;c.node._poll_stream_snapshot()
    else:
        c.clock[0] += .1
        request(c.node, dict(req, state='revoked', request_id='stop', stamp_ns=c.node._ros_source_clock_ns()))
    assert c.node._current_stream_target_identity() != c.identity
    assert c.node._unknown_identity_retired_selection == c.node._mode_selection
    deliver(c.node, c.ready)
    assert not c.node.latest_object.detected
    assert not c.node._measurement_remote_active()


@pytest.mark.parametrize('fault', ['anchor', 'empty_mask', 'frame', 'source', 'unqualified',
    'no_owner_proof', 'list_mismatch', 'old_source', 'authority', 'late', 'low_confidence',
    'tf_not_consumed', 'confirmation_before_arrival', 'confirmation_unordered'])
def test_invalid_or_incomplete_arrival_cannot_confirm_geometry(context, fault):
    c = context
    req = arrival(c)
    row, _ = fresh(c, req)
    if fault == 'anchor':row['status']['anchor_position_base'][0] += .001
    elif fault == 'empty_mask':row['mask'].data = bytes(len(row['mask'].data))
    elif fault == 'frame':row['mask'].header.frame_id = 'wrong'
    elif fault == 'source':row['object'].header.stamp = at(row['object'].header.stamp.to_nsec()-1)
    elif fault == 'unqualified':row['ack']['qualified'] = False
    elif fault == 'list_mismatch':row['status']['observation_measurement']['source_stamps_ns'][0] += 1
    elif fault == 'old_source':row['ack']['source_stamps_ns'][0] = req['arrival_stamp_ns']
    elif fault == 'authority':row['status']['observation_measurement']['motion_authorized'] = True
    elif fault == 'late':c.clock[0] += 1.
    elif fault == 'low_confidence':row['object'].confidence = 0.
    elif fault == 'tf_not_consumed':row['status']['observation_measurement']['exact_tf_consumed'] = False
    elif fault == 'confirmation_before_arrival':
        row['status']['observation_measurement']['source_stamps_ns'][0]=req['arrival_stamp_ns']-1
        row['ack']['source_stamps_ns'][0]=req['arrival_stamp_ns']-1
    elif fault == 'confirmation_unordered':
        row['status']['observation_measurement']['source_stamps_ns'][0]=row['ack']['source_stamps_ns'][1]
        row['ack']['source_stamps_ns'][0]=row['ack']['source_stamps_ns'][1]
    for key in ('object', 'mask', 'status', 'ack'):
        if not (fault == 'no_owner_proof' and key == 'ack'):
            deliver_one(c, row, key)
    assert not qualified_ack(c)
    assert c.node.latest_object.header.stamp.to_nsec() != row['object'].header.stamp.to_nsec() or not c.node.latest_object.detected


def test_historical_identity_evidence_does_not_expire_fresh_final_geometry(context):
    c=context;req=arrival(c);row,sources=fresh(c,req)
    # Exact gen11 first-group timing, translated into this isolated clock.
    origin=1791517407567292690
    sources=[req['arrival_stamp_ns']+ns-origin for ns in
        (1791517407718676805,1791517407986799001,1791517408292557001)]
    c.clock[0]=(req['arrival_stamp_ns']+1791517408596295118-origin)/1e9
    for name in ('object','mask'):row[name].header.stamp=at(sources[-1])
    for key in ('source_stamp_ns','color_stamp_ns','depth_stamp_ns','tf_lookup_stamp_ns'):
        row['status'][key]=sources[-1]
    row['status']['observation_measurement']['source_stamps_ns']=list(sources)
    row['ack']=measurement_ack(req,'unknown',accepted=True,qualified=True,source_stamps_ns=sources)
    now=c.node._ros_source_clock_ns()
    assert now-sources[0]>800_000_000 and now-sources[-1]<800_000_000
    for key in ('status','ack','object','mask'):deliver_one(c,row,key)
    assert qualified_ack(c)[-1]['source_stamps_ns']==sources
    assert c.node.latest_object.header.stamp.to_nsec()==sources[-1]
    assert c.node._current_stream_target_identity()==c.identity
    done=dict(req,state='qualified',request_id='done',stamp_ns=now,source_stamps_ns=sources)
    assert request(c.node,done)['accepted']


def test_final_geometry_expiring_before_task_release_still_retires(context):
    c=context;req=arrival(c);row,sources=fresh(c,req)
    for key in ('object','mask','status','ack'):deliver_one(c,row,key)
    assert qualified_ack(c)
    c.clock[0]=(sources[-1]+800_000_001)/1e9
    done=dict(req,state='qualified',request_id='late_done',
        stamp_ns=c.node._ros_source_clock_ns(),source_stamps_ns=sources)
    assert not request(c.node,done)['accepted']
    assert c.node._unknown_identity_retired_selection==c.node._mode_selection


def test_return_starts_a_new_lease_only_after_completed_handoff(context):
    c = context
    req = arrival(c); row, sources = fresh(c, req)
    for key in ('object', 'mask', 'status', 'ack'):deliver_one(c, row, key)
    c.clock[0] += .01
    done = dict(req, state='qualified', request_id='done',
        stamp_ns=c.node._ros_source_clock_ns(), source_stamps_ns=sources)
    assert request(c.node, done)['accepted']
    assert request(c.node, done)['accepted']  # exact terminal replay is inert
    assert request(c.node, c.request)['accepted'] is False
    assert c.node._measurement_remote_request['state'] == 'qualified'
    c.node._observe_return_phase = 'returning';c.node._observe_return_session['phase'] = 'returning'
    c.clock[0] += .01
    second = dict(c.request, lease_id='return-lease', request_id='return-start', motion_kind='return',
                  stamp_ns=c.node._ros_source_clock_ns())
    assert request(c.node, second)['accepted']
    assert c.node._measurement_remote_active()
    assert request(c.node, done)['accepted'] is False
    assert request(c.node, c.request)['accepted'] is False
    assert c.node._measurement_remote_request['lease_id'] == 'return-lease'
    assert c.node._current_stream_target_identity() == c.identity


def test_late_previous_phase_message_does_not_revoke_current_capture(context):
    c = context
    req = arrival(c)
    assert request(c.node, c.request)['accepted'] is False
    assert c.node._measurement_remote_request == req
    assert c.node._current_stream_target_identity() == c.identity


@pytest.mark.parametrize('frame,ns', [('other', 5_000_000_000), ('sdk_reference_epoch', 0)])
def test_epoch_subscriber_ignores_invalid_header_without_rebinding(context, frame, ns):
    c = context
    begin(c)
    original = c.node._measurement_remote_epoch_ns
    c.node._measurement_remote_epoch_cb(Header(stamp=at(ns), frame_id=frame))
    assert c.node._measurement_remote_epoch_ns == original
    assert c.node._measurement_remote_active()


def test_manual_takeover_retires_without_waiting_for_other_participants(context):
    c = context
    begin(c)
    c.params['/gui/joint_direct_mode'] = True
    assert c.node._poll_stream_snapshot() is False
    assert c.node._unknown_identity_retired_selection == c.node._mode_selection
    assert not c.node._measurement_remote_active()


@pytest.mark.parametrize('missing', ['mask', 'status'])
def test_start_ack_waits_for_delayed_reference_topic_without_timestamp_mutation(context, missing):
    c = context
    cache = c.node._unknown_identity_masks if missing == 'mask' else c.node._unknown_identity_status
    cache.clear()
    c.node._measurement_remote_request_cb(String(data=json.dumps(c.request)))
    assert not c.node._measurement_remote_ack_pub.messages
    assert not c.node._measurement_remote_active()
    before = c.node.latest_object_time
    deliver_one(c, c.ready, missing)
    assert json.loads(c.node._measurement_remote_ack_pub.messages[-1].data)['accepted']
    assert c.node._measurement_remote_active()
    assert c.node.latest_object_time == before


def test_incomplete_start_join_expires_without_claiming_acceptance(context):
    c = context
    c.node._unknown_identity_status.clear()
    c.node._measurement_remote_request_cb(String(data=json.dumps(c.request)))
    c.clock[0] += .81
    assert c.node._poll_stream_snapshot() is False
    assert not c.node._measurement_remote_active()
    assert json.loads(c.node._measurement_remote_ack_pub.messages[-1].data)['accepted'] is False


@pytest.mark.parametrize('order', list(itertools.permutations(('request', 'object', 'mask', 'status', 'ack'))))
def test_arrival_request_can_race_all_four_source_connections(context, order):
    c = context
    previous = begin(c)
    c.clock[0] += 5.
    now = c.node._ros_source_clock_ns()
    req = dict(previous, state='requalifying', request_id='arrived', stamp_ns=now,
        arrival_stamp_ns=now, capture_deadline_ns=now+3_000_000_000,
        settle=dict(epoch_ns=previous['control_epoch_ns'],
            stationary_window_start_ns=now-300_000_000, stationary_window_end_ns=now-1))
    row, sources = fresh(c, req)
    for key in order:
        if key == 'request':
            assert request(c.node, req)['accepted']
        else:
            deliver_one(c, row, key)
    assert qualified_ack(c)[-1]['source_stamps_ns'] == sources
    assert c.node._current_stream_target_identity() == c.identity
    assert c.node._observe_return_paused


@pytest.mark.parametrize('order', [('ack', 'status', 'mask', 'object'),
    ('object', 'mask', 'status', 'ack'), ('status', 'object', 'ack', 'mask')])
def test_real_unknown_three_original_audits_join_remote_without_replaying_first_two(context, order):
    from unittest.mock import Mock
    from test_observation_measurement_unknown import send, ack, arrival as producer_arrival, next_frame
    c = context
    c.producer._observation_measurement_ack_pub = Mock()
    first = begin(c)
    send(c.producer, first)
    assert ack(c.producer)['accepted']
    arrived = producer_arrival(c.producer, c.clock, c.joint, first)
    assert request(c.node, arrived)['accepted']
    old_ns = c.node.latest_object.header.stamp.to_nsec()
    for _ in range(2):
        next_frame(c.producer, c.clock, c.joint)
        pending = publication(c.producer)
        c.node._silhouette_source_status_cb(String(data=json.dumps(pending['status'])))
        assert c.node.latest_object.header.stamp.to_nsec() == old_ns
        assert not qualified_ack(c)
    next_frame(c.producer, c.clock, c.joint)
    row = publication(c.producer)
    row['ack'] = ack(c.producer)
    assert row['ack']['qualified'], row['status'].get('observation_measurement')
    for key in order:
        deliver_one(c, row, key)
    assert qualified_ack(c)[-1]['source_stamps_ns'] == row['ack']['source_stamps_ns']
    assert c.node.latest_object.header.stamp.to_nsec() == row['object'].header.stamp.to_nsec()
    assert c.node._current_stream_target_identity() == c.identity
    c.clock[0] += .01
    done = dict(arrived, state='qualified', request_id='both-owners-done',
        stamp_ns=c.node._ros_source_clock_ns(), source_stamps_ns=row['ack']['source_stamps_ns'])
    send(c.producer, done)
    assert ack(c.producer)['accepted']
    assert request(c.node, done)['accepted']
    assert c.node._observe_return_paused and not c.node._measurement_remote_active()
