"""Raw XML-RPC selection types through the real producer handoff callback.

The full synthetic RGB-D station fixture retains all producer context gates.
The historical wire replay isolates unavailable past in-memory geometry only;
it uses the recorded request byte-for-byte and the actual producer callback.
"""
from copy import deepcopy
import json

import pytest
from std_msgs.msg import String

from alicia_grasp_modes.contact_station_retirement import parse_request, ack, validate_ack
from test_contact_station_retirement import producer_context, retirement, send
from test_locked_rgb_hold import module

RECORDED_PREPARE = {'version': 1, 'state': 'prepare', 'request_id': 'ee2b2134822f53326ebd67c405d552718ccd5579cca4d915741450795e3699fa:observation:1792025730330753564:contact:1792025816005154848', 'selection': {'mode': 'unknown', 'strategy': 'observe_return', 'generation': 2, 'stamp_ns': 1792025715201812028}, 'session_id': 'ee2b2134822f53326ebd67c405d552718ccd5579cca4d915741450795e3699fa', 'target_track_id': 'g1-t4', 'target_epoch': 4, 'control_epoch_ns': 1792025612219386951, 'measurement_sha256': 'fe656d04c1a68503a0e093c0f30d524f877613d4d1480f938e62ab7b63dd63a4', 'plan_id': '26ec14e175bb95b3369dd05c', 'plan_sha256': 'da50a969b2abf68a9fd8afe54cfccc9627d45166ad325cc99de8f01dd29471f3', 'plan_source_ns': 1792025770917912483, 'phase_id': 1, 'phase_started_ns': 1792025770297081232, 'stage': 'MOVE_PREGRASP', 'stamp_ns': 1792025816005154848, 'deadline_ns': 1792025816805154848, 'metric_geometry_authority': False, 'motion_authorized': False, 'controller_motion_confirmed': False}


def raw_selection(selection):
    return dict(selection, stamp_ns=str(selection['stamp_ns']))


def test_real_producer_raw_ros_param_prepare_and_retire_keep_all_context_gates(monkeypatch):
    c = producer_context(monkeypatch)
    original = deepcopy(c.req)
    c.params['/grasp_mode/selection'] = raw_selection(c.req['selection'])
    source = c.node.last_good_source_stamp_ns
    anchor = c.node.tracker.anchor
    objects = c.node.pubs['object'].publish.call_count
    reply = send(c.node, c.req)
    validate_ack(reply, c.req, 'unknown')
    assert c.node._locked_rgb_hold_reference['observation_arrival_retry'] is c.marker
    retire = retirement(c.req)
    validate_ack(send(c.node, retire), retire, 'unknown')
    assert c.node._contact_station_version == 1
    assert 'observation_arrival_retry' not in c.node._locked_rgb_hold_reference
    assert c.node.last_good_source_stamp_ns == source and c.node.tracker.anchor is anchor
    assert c.node.pubs['object'].publish.call_count == objects
    assert c.req == original
    for response in (reply, json.loads(c.node._contact_station_ack_pub.publish.call_args[0][0].data)):
        assert response['metric_geometry_authority'] is False
        assert response['motion_authorized'] is False
        assert response['controller_motion_confirmed'] is False


def test_recorded_prepare_reaches_real_producer_ack_with_raw_xmlrpc_selection(monkeypatch):
    c = producer_context(monkeypatch)
    request = deepcopy(RECORDED_PREPARE)
    c.params['/grasp_mode/selection'] = raw_selection(request['selection'])
    c.node._rgb_interest_epoch = request['control_epoch_ns']
    now = request['stamp_ns'] + 1_000_000
    monkeypatch.setattr(module.rospy.Time, 'now', staticmethod(lambda: module.rospy.Time(now//10**9, now%10**9)))
    checked = []
    def historical_context(value):
        # No claim that recorded pixels reconstruct the old in-memory station.
        # This isolates only historical geometry from the actual wire pathway.
        assert value == request
        checked.append(value)
        return c.node._locked_rgb_hold_reference, c.marker
    monkeypatch.setattr(c.node, '_contact_station_context', historical_context)
    c.node._contact_station_request_cb(String(data=json.dumps(request)))
    assert c.node._contact_station_ack_pub.publish.call_count == 1
    reply = json.loads(c.node._contact_station_ack_pub.publish.call_args[0][0].data)
    validate_ack(reply, request, 'unknown')
    assert checked == [request]
    assert c.node._contact_station_prepared == request
    assert getattr(c.node, '_contact_station_version', 0) == 0
    assert request == RECORDED_PREPARE


@pytest.mark.parametrize('phase', ['prepare', 'retire'])
def test_canonical_integer_and_decimal_text_param_have_identical_receipts(phase):
    request = deepcopy(RECORDED_PREPARE)
    previous = None
    if phase == 'retire':
        previous = deepcopy(request)
        request = retirement(request)
    before = deepcopy(request)
    now = request['stamp_ns'] + 1_000_000
    canonical = parse_request(request, request['selection'], now, previous)
    raw = parse_request(request, raw_selection(request['selection']), now, previous)
    assert raw == canonical == before
    assert request == before
    assert ack(raw, 'unknown') == ack(canonical, 'unknown')


@pytest.mark.parametrize('value', [True, False, 1792025715201812028.0,
    '1792025715201812028.0', '-1792025715201812028', '', None])
def test_invalid_param_stamp_never_produces_positive_producer_ack(monkeypatch, value):
    c = producer_context(monkeypatch)
    c.params['/grasp_mode/selection'] = dict(c.req['selection'], stamp_ns=value)
    c.node._contact_station_request_cb(String(data=json.dumps(c.req)))
    assert c.node._contact_station_ack_pub.publish.call_count == 0
    assert getattr(c.node, '_contact_station_prepared', None) is None
    assert getattr(c.node, '_contact_station_version', 0) == 0
    assert c.node._locked_rgb_hold_reference['observation_arrival_retry'] is c.marker


@pytest.mark.parametrize('changed', ['stamp', 'generation', 'mode', 'strategy'])
def test_raw_parameter_mismatch_is_not_normalized_into_an_acceptance(monkeypatch, changed):
    c = producer_context(monkeypatch)
    raw = raw_selection(c.req['selection'])
    if changed == 'stamp': raw['stamp_ns'] = str(int(raw['stamp_ns']) + 1)
    elif changed == 'generation': raw['generation'] += 1
    elif changed == 'mode': raw.update(mode='carton', strategy='two_stage')
    else: raw['strategy'] = 'direct'
    c.params['/grasp_mode/selection'] = raw
    c.node._contact_station_request_cb(String(data=json.dumps(c.req)))
    assert c.node._contact_station_ack_pub.publish.call_count == 0
    assert getattr(c.node, '_contact_station_prepared', None) is None
    assert getattr(c.node, '_contact_station_version', 0) == 0


@pytest.mark.parametrize('authority', ['metric_geometry_authority', 'motion_authorized', 'controller_motion_confirmed'])
def test_raw_parameter_support_never_accepts_new_authority(monkeypatch, authority):
    c = producer_context(monkeypatch)
    c.params['/grasp_mode/selection'] = raw_selection(c.req['selection'])
    c.req[authority] = True
    c.node._contact_station_request_cb(String(data=json.dumps(c.req)))
    assert c.node._contact_station_ack_pub.publish.call_count == 0
    assert getattr(c.node, '_contact_station_prepared', None) is None
    assert getattr(c.node, '_contact_station_version', 0) == 0
