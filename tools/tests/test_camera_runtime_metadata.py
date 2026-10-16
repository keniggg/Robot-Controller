"""Offline parameter evidence tests; no ROS master, camera or arm is contacted."""
import ast
import copy
import json
from pathlib import Path
import sys
import threading
import time
import types

import pytest


SCRIPT = Path(__file__).resolve().parents[1] / 'record_grasp_keyframes_readonly.py'
TREE = ast.parse(SCRIPT.read_text())
HELPERS = [node for node in TREE.body if isinstance(node, (ast.FunctionDef, ast.ClassDef))
           and node.name in {'camera_runtime_metadata_snapshot', 'CameraRuntimeMetadataReader', 'read_camera_parameters'}]
SCOPE = {'json': json, 'threading': threading}
exec(compile(ast.Module(body=HELPERS, type_ignores=[]), str(SCRIPT), 'exec'), SCOPE)
snapshot = SCOPE['camera_runtime_metadata_snapshot']
Reader = SCOPE['CameraRuntimeMetadataReader']


def active_parameters():
    color = dict(width=640, height=480, fx=395.7682800292969, fy=395.6027526855469,
                 cx=317.9961242675781, cy=244.833984375, model='inverse_brown_conrady',
                 coeffs=[-.0537299253, .0583461151, .0021220003, .0007625872, -.0197049230])
    return {
        'frame_id': 'camera_link', 'width': 640, 'height': 480,
        'camera_info_topic': '/supervisor/camera/color/camera_info',
        'depth_scale': 9.999999747378752e-5, 'intrinsics_source': 'sdk_active_color_profile',
        'runtime_profile': {
            'source': 'realsense_active_profile', 'schema_version': 1,
            'device': {'serial_number': 'test-device'}, 'sdk_version': '2.56.5',
            'geometry_intrinsics_source': 'sdk_active_color_profile',
            'geometry_intrinsics': color, 'color_intrinsics': copy.deepcopy(color),
            'native_depth_intrinsics': dict(color, fx=389.4533996582031,
                                           model='brown_conrady', coeffs=[0.] * 5),
            'alignment': {'enabled': True, 'target_stream': 'color'},
            'depth_to_color_extrinsics': {
                'rotation_column_major': [1., 0., 0., 0., 1., 0., 0., 0., 1.],
                'translation_m': [-.0000507, .0000217, .0000955]},
            'depth_scale_m_per_unit': 9.999999747378752e-5,
            'depth_scale_source': 'sdk_depth_sensor',
            'color_projection_correction': {'enabled': False},
            'stationary_temporal': {'active': False},
        },
    }


def interval(second=1):
    return {'started': {'utc': '2026-09-27T00:00:%02d.000Z' % second,
                        'ros_time_ns': second * 10**9},
            'finished': {'utc': '2026-09-27T00:00:%02d.005Z' % second,
                         'ros_time_ns': second * 10**9 + 5_000_000}}


def test_preserves_full_geometry_without_inventing_camera_info_or_image_time():
    parameters = active_parameters()
    original = copy.deepcopy(parameters)
    content, serialized = snapshot(parameters, interval())
    payload = json.loads(serialized)
    assert payload['status'] == 'available'
    assert payload['parameters'] == original == parameters
    assert payload['parameter_read_interval'] == interval()
    assert payload['image_time_association'] == 'none'
    assert 'not_image_or_exposure_time' in payload['time_semantics']
    assert 'camera_liveness_not_checked' in payload['availability_scope']
    assert payload['unit_conventions']['depth_scale'] == 'metres_per_depth_unit'
    assert 'parameter_read_interval' not in json.loads(content)
    assert 'header' not in payload and 'CameraInfo' not in payload


def test_deduplicates_content_while_preserving_profile_switches_and_recovery():
    parameters = active_parameters()
    previous, first = snapshot(parameters, interval())
    # Key ordering and new read time cannot manufacture a profile change.
    reordered = dict(reversed(list(parameters.items())))
    unchanged, payload = snapshot(reordered, interval(2), previous)
    assert unchanged == previous and payload is None
    parameters['runtime_profile']['geometry_intrinsics']['fx'] += .25
    changed, payload = snapshot(parameters, interval(3), previous)
    assert payload is not None and changed != previous
    assert json.loads(payload)['parameters']['runtime_profile']['geometry_intrinsics']['fx'] == pytest.approx(396.0182800292969)
    missing, payload = snapshot(None, interval(4), changed)
    assert json.loads(payload)['status'] == 'unavailable'
    recovered, payload = snapshot(parameters, interval(5), missing)
    assert recovered == changed and json.loads(payload)['status'] == 'available'


@pytest.mark.parametrize('parameters, reason', [
    (None, 'camera_parameter_missing_or_invalid'),
    ([], 'camera_parameter_missing_or_invalid'),
    ({'fx': 99., 'depth_scale': .001}, 'active_profile_missing'),
    ({'runtime_profile': {'source': 'simulation'}}, 'active_profile_unavailable'),
    ({'runtime_profile': {'source': 'realsense_active_profile'}}, 'active_geometry_missing'),
])
def test_unavailable_is_explicit_and_preserves_readable_parameters(parameters, reason):
    content, serialized = snapshot(parameters, interval())
    payload = json.loads(serialized)
    assert payload['status'] == 'unavailable'
    assert payload['unavailable_reason'] == reason
    assert payload['parameters'] == parameters
    assert snapshot(parameters, interval(2), content)[1] is None


def test_parameter_read_failure_is_evidence_and_later_retry_can_recover():
    failed, serialized = snapshot(None, interval(), read_error='OSError: offline')
    payload = json.loads(serialized)
    assert payload['status'] == 'unavailable'
    assert payload['unavailable_reason'] == 'parameter_read_failed'
    assert payload['read_error'] == 'OSError: offline'
    assert snapshot(None, interval(2), failed, read_error='OSError: offline')[1] is None
    assert snapshot(active_parameters(), interval(3), failed)[1] is not None


@pytest.mark.parametrize('bad_value', [float('nan'), float('inf'), object()])
def test_malformed_parameter_payload_does_not_raise_or_claim_active_geometry(bad_value):
    parameters = active_parameters()
    parameters['runtime_profile']['geometry_intrinsics']['fx'] = bad_value
    _, serialized = snapshot(parameters, interval())
    payload = json.loads(serialized)
    assert payload['status'] == 'unavailable'
    assert payload['unavailable_reason'] == 'parameter_not_json_serializable'
    assert payload['parameters'] is None


def test_startup_snapshot_precedes_images_and_missing_profile_does_not_block_keyframes(
        tmp_path, monkeypatch):
    # This loader supplies fake modules and an in-memory clock; it never imports
    # real ROS clients or opens a device. Exercise main's actual ordering too.
    from test_recorded_grasp_lifecycle import load_fake_ros_recorder
    module = load_fake_ros_recorder(monkeypatch, [])
    writes = []
    original_bag = module.rosbag.Bag

    class CaptureBag(original_bag):
        def write(self, topic, message, stamp):
            writes.append((topic, getattr(message, 'data', None)))
            super().write(topic, message, stamp)

    monkeypatch.setattr(module.rosbag, 'Bag', CaptureBag)
    first_subscription = []

    def subscribe(topic, _message_type, callback, **_kwargs):
        first_subscription.append(len(writes))
        if topic in module.SOURCES:
            callback(types.SimpleNamespace(header=types.SimpleNamespace(
                stamp=types.SimpleNamespace(to_nsec=lambda: 123))))
        return types.SimpleNamespace(unregister=lambda: None)

    def get_param(name, default):
        if name == '/camera':
            if module.time.monotonic() < .5:
                raise OSError('parameter server temporarily unavailable')
            parameters = active_parameters()
            if module.time.monotonic() > 1.5:
                parameters['runtime_profile']['geometry_intrinsics']['fx'] += .25
            return parameters
        return default

    original_sleep = module.rospy.sleep
    monkeypatch.setattr(module.rospy, 'sleep',
                        lambda seconds: (time.sleep(.002), original_sleep(seconds)))
    monkeypatch.setattr(module.rospy, 'Subscriber', subscribe)
    monkeypatch.setattr(module.rospy, 'get_param', get_param)
    monkeypatch.setattr(module, 'read_camera_parameters', lambda: get_param('/camera', None))
    monkeypatch.setattr(sys, 'argv', ['recorder', '--output', str(tmp_path / 'keyframes.bag'),
                                    '--duration', '2.8', '--min-free-bytes', '0'])
    module.main()
    topic = '/grasp_record/camera_runtime_metadata'
    assert writes[0][0] == topic and first_subscription[0] == 1
    metadata = [json.loads(payload) for name, payload in writes if name == topic]
    assert len(metadata) == 3
    assert [item['status'] for item in metadata] == ['unavailable', 'available', 'available']
    assert metadata[-1]['parameters']['runtime_profile']['geometry_intrinsics']['fx'] == pytest.approx(396.0182800292969)
    assert set(module.SOURCES) <= {name for name, _ in writes}
    outcome_path, = tmp_path.glob('*/recorder_outcome.json')
    assert json.loads(outcome_path.read_text())['keyframe_count'] == 1


def test_single_worker_stays_bounded_during_hang_then_reports_recovered_read():
    entered = threading.Event()
    release = threading.Event()
    calls = []

    def blocked_read():
        calls.append(1)
        entered.set()
        release.wait()
        return active_parameters()

    reader = Reader(blocked_read, lambda: interval()['started'])
    worker = reader._worker
    try:
        assert worker.daemon and reader.request()
        assert entered.wait(.5)
        start = time.monotonic()
        sample = reader.poll(wait_seconds=.02)
        assert time.monotonic() - start < .3
        parameters, read_interval, read_error = sample
        assert parameters is None and read_error == 'parameter_read_pending'
        assert read_interval['finished'] is None
        content, payload = snapshot(parameters, read_interval, read_error=read_error)
        assert json.loads(payload)['unavailable_reason'] == 'parameter_read_pending'
        for _ in range(50):
            assert reader.request() is False
            pending = reader.poll()
            assert snapshot(pending[0], pending[1], content, pending[2])[1] is None
            assert reader._worker is worker
        assert len(calls) == 1
        release.set()
        recovered = reader.poll(wait_seconds=.5)
        assert recovered[0] == active_parameters() and recovered[2] is None
        assert recovered[1]['finished'] is not None
        assert json.loads(snapshot(recovered[0], recovered[1], content)[1])['status'] == 'available'
        # A subsequent independent read can run on the same recovered worker.
        assert reader.request()
        assert reader.poll(wait_seconds=.5)[0] == active_parameters()
        assert len(calls) == 2 and reader._worker is worker
    finally:
        reader.close()
        release.set()
        worker.join(timeout=.5)
    assert not worker.is_alive()


def test_hung_startup_read_does_not_block_ready_continuous_keyframes_or_shutdown(
        tmp_path, monkeypatch):
    from test_recorded_grasp_lifecycle import load_fake_ros_recorder
    module = load_fake_ros_recorder(monkeypatch, [])
    release = threading.Event()
    entered = threading.Event()
    reads = []
    writes = []
    source_callbacks = {}
    startup_delay = []
    workers = []
    original_reader = module.CameraRuntimeMetadataReader
    original_bag = module.rosbag.Bag
    real_started = time.monotonic()

    def make_reader(*args):
        reader = original_reader(*args)
        workers.append(reader._worker)
        return reader

    class CaptureBag(original_bag):
        def write(self, topic, message, stamp):
            writes.append((topic, getattr(message, 'data', None)))
            super().write(topic, message, stamp)

    def get_param(name, default):
        if name == '/camera':
            reads.append(1)
            entered.set()
            release.wait()
            return active_parameters()
        return default

    def subscribe(topic, _message_type, callback, **_kwargs):
        if not startup_delay:
            startup_delay.append(time.monotonic() - real_started)
        if topic in module.SOURCES:
            source_callbacks[topic] = callback
        return types.SimpleNamespace(unregister=lambda: None)

    original_sleep = module.rospy.sleep
    frame = [1000]

    def sleep_and_send_frames(seconds):
        # The parameter read remains blocked for the recorder's entire run.
        assert entered.is_set() and not release.is_set()
        assert list(tmp_path.glob('*/recorder_ready.json'))
        frame[0] += 1
        stamp_ns = frame[0]
        for callback in source_callbacks.values():
            callback(types.SimpleNamespace(header=types.SimpleNamespace(
                stamp=types.SimpleNamespace(to_nsec=lambda: stamp_ns))))
        original_sleep(seconds)
        time.sleep(.001)

    monkeypatch.setattr(module, 'CameraRuntimeMetadataReader', make_reader)
    monkeypatch.setattr(module.rosbag, 'Bag', CaptureBag)
    monkeypatch.setattr(module.rospy, 'get_param', get_param)
    monkeypatch.setattr(module, 'read_camera_parameters', lambda: get_param('/camera', None))
    monkeypatch.setattr(module.rospy, 'Subscriber', subscribe)
    monkeypatch.setattr(module.rospy, 'sleep', sleep_and_send_frames)
    monkeypatch.setattr(sys, 'argv', ['recorder', '--output', str(tmp_path / 'keyframes.bag'),
                                    '--duration', '3.2', '--min-free-bytes', '0'])
    try:
        module.main()
        assert startup_delay[0] < 1.
        assert time.monotonic() - real_started < 1.5
        assert len(workers) == 1 and workers[0].daemon and workers[0].is_alive()
        assert reads == [1]
        camera_metadata = [json.loads(payload) for topic, payload in writes
                           if topic == '/grasp_record/camera_runtime_metadata']
        assert len(camera_metadata) == 1
        assert camera_metadata[0]['unavailable_reason'] == 'parameter_read_pending'
        assert camera_metadata[0]['parameter_read_interval']['finished'] is None
        outcome_path, = tmp_path.glob('*/recorder_outcome.json')
        outcome = json.loads(outcome_path.read_text())
        assert outcome['keyframe_count'] >= 10 and outcome['bag_closed']
        writes_before_release = len(writes)
    finally:
        release.set()
        for worker in workers:
            worker.join(timeout=.5)
    assert all(not worker.is_alive() for worker in workers)
    assert len(writes) == writes_before_release  # Late results cannot touch a closed bag.


@pytest.mark.parametrize('reply, expected', [
    ((1, 'ok', {'runtime_profile': {'source': 'realsense_active_profile'}}),
     {'runtime_profile': {'source': 'realsense_active_profile'}}),
    ((-1, 'Parameter /camera is not set', None), None),
])
def test_camera_read_uses_a_private_xmlrpc_connection_not_rospy_shared_proxy(
        monkeypatch, reply, expected):
    import xmlrpc.client
    events = []
    rosgraph = types.ModuleType('rosgraph')
    rosgraph.get_master_uri = lambda: 'http://unused.example:11311'
    monkeypatch.setitem(sys.modules, 'rosgraph', rosgraph)
    monkeypatch.setitem(SCOPE, 'rospy', types.SimpleNamespace(
        get_name=lambda: '/readonly-recorder',
        get_param=lambda *_args: pytest.fail('must not use shared rospy proxy')))

    class PrivateProxy:
        def __init__(self, uri):
            events.append(('private_proxy', uri))
        def __enter__(self):
            return self
        def __exit__(self, *_args):
            events.append(('connection_closed',))
        def getParam(self, caller_id, key):
            events.append(('getParam', caller_id, key))
            return reply

    monkeypatch.setattr(xmlrpc.client, 'ServerProxy', PrivateProxy)
    assert SCOPE['read_camera_parameters']() == expected
    assert events == [('private_proxy', 'http://unused.example:11311'),
                      ('getParam', '/readonly-recorder', '/camera'),
                      ('connection_closed',)]


def test_main_wires_private_camera_reader_and_keeps_original_error_read():
    # Check the actual callback wiring so a mocked worker test cannot conceal a
    # regression back to rospy.get_param('/camera') and its shared master lock.
    main = next(node for node in TREE.body if isinstance(node, ast.FunctionDef)
                and node.name == 'main')
    constructors = [node for node in ast.walk(main) if isinstance(node, ast.Call)
                    and isinstance(node.func, ast.Name)
                    and node.func.id == 'CameraRuntimeMetadataReader']
    assert len(constructors) == 1
    assert isinstance(constructors[0].args[0], ast.Name)
    assert constructors[0].args[0].id == 'read_camera_parameters'
    parameters = [node.args[0].value for node in ast.walk(main)
                  if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
                  and node.func.attr == 'get_param'
                  and isinstance(node.args[0], ast.Constant)]
    assert parameters == ['/grasp_6d/runtime_execution_error']
