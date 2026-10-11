"""Bag-only observation RGB-D remains exact, bounded, and non-authorizing."""
import ast
from collections import OrderedDict
from copy import deepcopy
import json
from pathlib import Path
import sys
from types import SimpleNamespace as NS

import pytest


SCRIPT = Path(__file__).resolve().parents[1] / 'record_grasp_keyframes_readonly.py'
TREE = ast.parse(SCRIPT.read_text())
NODES = [node for node in TREE.body if
    isinstance(node, ast.ClassDef) and node.name == 'ObservationRgbdWindow'
    or isinstance(node, ast.Assign) and any(isinstance(t, ast.Name)
        and t.id == 'OBSERVATION_RGBD_TOPICS' for t in node.targets)]
SCOPE = dict(json=json, OrderedDict=OrderedDict)
exec(compile(ast.Module(body=NODES, type_ignores=[]), str(SCRIPT), 'exec'), SCOPE)
Window = SCOPE['ObservationRgbdWindow']
TOPICS = SCOPE['OBSERVATION_RGBD_TOPICS']
COLOR, DEPTH = tuple(TOPICS)


def request(stamp=1_000_000_000):
    return dict(version=1, state='executing', lease_id='lease', request_id='executing',
        selection=dict(mode='unknown', strategy='observe_return', generation=2, stamp_ns=500_000_000),
        session_id='session', target_epoch=2, target_track_id='track', origin_plan_id='origin',
        origin_snapshot_stamp_ns=600_000_000, view_plan_id='view', view_source_stamp_ns=800_000_000,
        view_index=1, plan_sha256='a'*64, control_epoch_ns=550_000_000,
        deadline_ns=60_000_000_000, stamp_ns=stamp, motion_kind='observation',
        arrival_stamp_ns=0, capture_deadline_ns=0, metric_geometry_authority=False, motion_authorized=False)


def image(stamp, data):
    return NS(header=NS(stamp=NS(to_nsec=lambda: stamp), frame_id='camera_link', seq=37),
              width=640, height=480, data=data)


def start():
    window = Window(900_000_000)
    assert window.update_request(json.dumps(request()), 1_000_000_000, 10.)
    return window


def pair(window, stamp, wall, order=(COLOR, DEPTH)):
    messages = {COLOR: image(stamp, b'original rgb'), DEPTH: image(stamp, b'original depth')}
    results = [window.offer(topic, messages[topic], stamp+1, wall) for topic in order]
    return results[-1], messages


@pytest.mark.parametrize('order', [(COLOR, DEPTH), (DEPTH, COLOR)])
def test_records_two_exact_original_messages_without_object_or_mask(order):
    window = start()
    result, messages = pair(window, 1_100_000_000, 10.1, order)
    rows, metadata = result
    assert len(rows) == 2 and window.count == 1
    for original, destination in TOPICS.items():
        found = dict(rows)[destination]
        assert found is messages[original]
        assert found.header.stamp.to_nsec() == 1_100_000_000 and found.header.seq == 37
    assert metadata['original_topics'] == {dst: src for src, dst in TOPICS.items()}
    assert not metadata['execution_source'] and not metadata['metric_geometry_authority']
    assert not metadata['motion_authorized']


def test_mismatched_stamps_are_never_nearest_paired_or_filled():
    window = start()
    assert window.offer(COLOR, image(1_100_000_000, b'c'), 1_100_000_001, 10.1) is None
    assert window.offer(DEPTH, image(1_100_000_001, b'd'), 1_100_000_002, 10.1) is None
    assert window.count == 0
    assert window.offer('/perception/object_mask', image(1_100_000_000, b'm'), 1_100_000_002, 10.1) is None


def test_rate_limit_and_duplicate_delivery_never_rewrite_alias_topic_stamp():
    window = start()
    assert pair(window, 1_100_000_000, 10.1)[0] is not None
    assert pair(window, 1_100_000_000, 10.4)[0] is None
    assert pair(window, 1_200_000_000, 10.4)[0] is None
    assert pair(window, 1_400_000_000, 10.2)[0] is None
    assert pair(window, 1_400_000_000, 10.4000001)[0] is not None
    assert window.count == 2


@pytest.mark.parametrize('state', ['qualified', 'revoked'])
def test_terminal_receipt_closes_window_without_rewriting_normal_sources(state):
    window = start()
    closed = dict(request(), state=state, stamp_ns=1_200_000_000, request_id=state)
    assert window.update_request(json.dumps(closed), 1_200_000_000, 10.2)
    assert pair(window, 1_300_000_000, 10.3)[0] is None
    assert not window.update_request(json.dumps(request()), 1_400_000_000, 10.4)


@pytest.mark.parametrize('fault', ['before_recorder', 'expired', 'future', 'old_requalifying',
                                  'long', 'authority', 'different_mode', 'bool_stamp'])
def test_old_latched_or_invalid_requests_do_not_open_window(fault):
    window = Window(900_000_000)
    req = request()
    if fault == 'before_recorder': req['stamp_ns'] = 899_999_999
    elif fault == 'expired': req['deadline_ns'] = req['stamp_ns']
    elif fault == 'future': req['stamp_ns'] += 1
    elif fault == 'old_requalifying': req['state'] = 'requalifying'
    elif fault == 'long': req['deadline_ns'] = req['stamp_ns'] + 600_000_000_001
    elif fault == 'authority': req['motion_authorized'] = True
    elif fault == 'different_mode': req['selection']['mode'] = 'carton'
    elif fault == 'bool_stamp': req['stamp_ns'] = True
    assert not window.update_request(json.dumps(req), 1_000_000_000, 10.)
    assert pair(window, 1_100_000_000, 10.1)[0] is None


@pytest.mark.parametrize('clock', ['ros', 'monotonic'])
def test_fixed_deadline_stops_capture_even_if_the_other_clock_stalls(clock):
    window = start()
    ns, wall = (60_000_000_000, 10.1) if clock == 'ros' else (1_100_000_000, 69.)
    assert window.offer(COLOR, image(1_100_000_000, b'c'), ns, wall) is None
    assert not window.active(ns, wall)


def test_requalifying_uses_original_binding_and_shorter_unchangeable_deadline():
    window = start()
    req = dict(request(), state='requalifying', request_id='arrival', stamp_ns=2_000_000_000,
               arrival_stamp_ns=2_000_000_000, capture_deadline_ns=5_000_000_000)
    foreign = dict(req, target_epoch=3)
    assert not window.update_request(json.dumps(foreign), 2_000_000_000, 11.)
    assert window.update_request(json.dumps(req), 2_000_000_000, 11.)
    assert window.deadline_ns == 5_000_000_000 and window.deadline_wall == 14.
    renewed = dict(req, request_id='renew', stamp_ns=3_000_000_000, capture_deadline_ns=6_000_000_000)
    assert not window.update_request(json.dumps(renewed), 3_000_000_000, 12.)
    assert pair(window, 5_000_000_000, 12.)[0] is None


def test_expired_lease_cannot_restart_with_a_new_timestamp_or_deadline():
    window = start()
    assert not window.active(60_000_000_000, 69.)
    renewed = dict(request(), stamp_ns=61_000_000_000, deadline_ns=65_000_000_000,
                   request_id='renewed')
    assert not window.update_request(json.dumps(renewed), 61_000_000_000, 70.)


def test_original_four_way_keyframe_logic_and_stats_survive_additional_pair_stream(tmp_path, monkeypatch):
    from test_recorded_grasp_lifecycle import load_fake_ros_recorder
    module = load_fake_ros_recorder(monkeypatch, [])
    monkeypatch.setattr(module, 'read_camera_parameters', lambda: None)
    monkeypatch.setattr(module.rospy.Time, 'now', lambda: NS(to_nsec=lambda:
        1_000_000_000 + int(module.time.monotonic()*1e9)))
    subscribers, writes = {}, []
    original_bag = module.rosbag.Bag
    class CaptureBag(original_bag):
        def write(self, topic, message, stamp):
            writes.append((topic, message))
            super().write(topic, message, stamp)
    monkeypatch.setattr(module.rosbag, 'Bag', CaptureBag)
    def subscribe(topic, _type, callback, **kwargs):
        subscribers[topic] = callback
        return NS(unregister=lambda: None)
    monkeypatch.setattr(module.rospy, 'Subscriber', subscribe)
    old_sleep, events, recorded = module.rospy.sleep, [], []
    active = [None]
    def sleep(seconds):
        old_sleep(seconds)
        now = module.rospy.Time.now().to_nsec()
        if not events:
            active[0] = request(now)
            subscribers[module.MEASUREMENT_REQUEST_TOPIC](module.String(data=json.dumps(active[0])))
        elif len(events) == 2:
            terminal = dict(active[0], state='qualified', request_id='done', stamp_ns=now)
            subscribers[module.MEASUREMENT_REQUEST_TOPIC](module.String(data=json.dumps(terminal)))
        if len(events) < 3:
            color, depth = image(now, b'c'), image(now, b'd')
            recorded.append((color, depth))
            subscribers[COLOR](color); subscribers[DEPTH](depth)
            if len(events) == 1:
                subscribers['/perception/object_mask'](image(now, b'm'))
                subscribers['/perception/object'](NS(header=color.header))
                subscribers[COLOR](color); subscribers[DEPTH](depth)
        events.append(now)
    monkeypatch.setattr(module.rospy, 'sleep', sleep)
    monkeypatch.setattr(sys, 'argv', ['recorder', '--output', str(tmp_path/'keyframes.bag'),
                                    '--duration', '1.0', '--min-free-bytes', '0'])
    module.main()
    extra = [(topic, msg) for topic, msg in writes if topic in TOPICS.values()]
    assert len(extra) == 4
    assert len({(topic, msg.header.stamp.to_nsec()) for topic, msg in extra}) == len(extra)
    assert [msg for topic, msg in extra if topic == TOPICS[COLOR]] == [row[0] for row in recorded[:2]]
    assert sum(topic in module.SOURCES for topic, msg in writes) == 4
    outcome, = tmp_path.glob('*/recorder_outcome.json')
    result = json.loads(outcome.read_text())
    assert result['keyframe_count'] == 1 and result['observation_rgbd_pair_count'] == 2
    assert result['bag_closed']
