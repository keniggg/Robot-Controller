"""Recorder metadata compatibility with fake ROS, messages and bag transport."""
from copy import deepcopy
import json
import sys
import types

import pytest

from test_recorded_grasp_lifecycle import load_fake_ros_recorder, archive


STATE_TOPICS = ('/alicia_d/physical_reference_state',
                '/bessica_d_hw_interface/physical_reference_state')
MAPPING_KEY = '/grasp/unknown_physical_reference_mapping_enabled'


@pytest.fixture
def recorder_case(tmp_path, monkeypatch):
    bags = []

    def make(*, messages_available, mapping_enabled):
        events = []
        module = load_fake_ros_recorder(monkeypatch, events)
        subscriptions, originals, before, writes, parameter_reads = [], {}, {}, [], []
        package = types.ModuleType('alicia_d_driver')
        package.__path__ = []
        messages = types.ModuleType('alicia_d_driver.msg')
        package.msg = messages

        class PhysicalReferenceState:
            def __init__(self, **values):
                self.__dict__.update(values)

        if messages_available:
            messages.PhysicalReferenceState = PhysicalReferenceState
        monkeypatch.setitem(sys.modules, 'alicia_d_driver', package)
        monkeypatch.setitem(sys.modules, 'alicia_d_driver.msg', messages)

        # Camera-reader timing/threads are covered by their own tests. This
        # fake keeps these startup-order cases deterministic and network-free.
        class QuietCameraReader:
            def __init__(self, *args): events.append('camera_reader_created')
            def request(self): return True
            def poll(self, **kwargs): return None
            def close(self): events.append('camera_reader_closed')

        original_bag = module.rosbag.Bag

        class CaptureBag(original_bag):
            def __init__(self, *args, **kwargs):
                super().__init__(*args, **kwargs)
                bags.append(self)

            def write(self, topic, message, stamp):
                writes.append((topic, message))
                super().write(topic, message, stamp)

        def get_param(name, default):
            parameter_reads.append(name)
            return mapping_enabled if name == MAPPING_KEY else default

        def subscribe(topic, message_type, callback, **kwargs):
            subscriptions.append((topic, message_type))
            if topic in STATE_TOPICS:
                assert not list(tmp_path.glob('*/recorder_ready.json'))
                index = STATE_TOPICS.index(topic)
                message = message_type(
                    header=types.SimpleNamespace(seq=19+index, frame_id='physical_reference_v1',
                        stamp=types.SimpleNamespace(to_nsec=lambda value=123+index: value)),
                    binding_id='synthetic-binding', owner_token='synthetic-owner',
                    goal_token='synthetic-goal', epoch_ns=101, phase=6 if index == 0 else 4,
                    code='synthetic-driver-fault' if index == 0 else 'synthetic-hardware-state',
                    bias_rad=[-.009203884727313847, .019941750242513323, .01, 0., 0., 0.],
                    physical_reference_rad=[.1, .2, .3, .4, .5, .6],
                    wire_positions_rad=[.11, .22, .31, .4, .5, .6],
                    sdk_counts=[2048, 2050, 2052, 2054, 2056, 2058],
                    goal_observed=True, goal_terminal=True, goal_status=4,
                    goal_error_code=-5, output_gated=True)
                originals[topic] = message
                before[topic] = deepcopy(message.__dict__)
                callback(message)
            return types.SimpleNamespace(unregister=lambda: events.append(('unregistered', topic)))

        monkeypatch.setattr(module, 'CameraRuntimeMetadataReader', QuietCameraReader)
        monkeypatch.setattr(module, 'read_camera_parameters', lambda: None)
        monkeypatch.setattr(module.rosbag, 'Bag', CaptureBag)
        monkeypatch.setattr(module.rospy, 'get_param', get_param)
        monkeypatch.setattr(module.rospy, 'Subscriber', subscribe)
        for name in ('Publisher', 'Service', 'ServiceProxy'):
            monkeypatch.setattr(module.rospy, name,
                lambda *a, **k: pytest.fail('recorder must not publish or call services'), raising=False)
        # These tests never install process signal handlers.
        monkeypatch.setattr(module.signal, 'signal', lambda *args: None)
        monkeypatch.setattr(sys, 'argv', ['recorder', '--output', str(tmp_path/'keyframes.bag'),
                                       '--duration', '.2', '--min-free-bytes', '0'])
        return types.SimpleNamespace(module=module, events=events,
            subscriptions=subscriptions, originals=originals, before=before,
            writes=writes, parameter_reads=parameter_reads, state_type=PhysicalReferenceState, bags=bags)

    yield make
    # Defensive teardown of fake resources if an assertion fails. The missing
    # dependency test separately requires that no resource was created at all.
    for bag in bags:
        if not bag.stream.closed:
            bag.stream.close()


@pytest.mark.parametrize('mapping_enabled', [True, False])
def test_both_typed_writer_receipts_are_preserved_before_ready(
        tmp_path, recorder_case, mapping_enabled):
    case = recorder_case(messages_available=True, mapping_enabled=mapping_enabled)
    case.module.main()
    assert [(topic, kind) for topic, kind in case.subscriptions if topic in STATE_TOPICS] == [
        (topic, case.state_type) for topic in STATE_TOPICS]
    captured = [(topic, message) for topic, message in case.writes if topic in STATE_TOPICS]
    assert [topic for topic, message in captured] == list(STATE_TOPICS)
    for topic, message in captured:
        assert message is case.originals[topic]
        assert message.__dict__ == case.before[topic]
        assert ('unregistered', topic) in case.events
    assert len(list(tmp_path.glob('*/recorder_ready.json'))) == 1
    manifest = json.loads(next(tmp_path.glob('*/manifest.json')).read_text())
    assert manifest['result'] == 'unknown' and manifest['recorder_closed'] is True


def test_missing_typed_messages_preserve_disabled_legacy_recording(tmp_path, recorder_case):
    case = recorder_case(messages_available=False, mapping_enabled=False)
    case.module.main()
    assert not any(topic in STATE_TOPICS for topic, kind in case.subscriptions)
    assert not any(topic in STATE_TOPICS for topic, message in case.writes)
    assert case.parameter_reads.count(MAPPING_KEY) == 1
    assert '/grasp_6d/runtime_execution_error' in case.parameter_reads
    assert len(list(tmp_path.glob('*/recorder_ready.json'))) == 1
    manifest = json.loads(next(tmp_path.glob('*/manifest.json')).read_text())
    assert manifest['result'] == 'unknown' and manifest['recorder_closed'] is True


@pytest.mark.parametrize('managed', [False, True])
def test_enabled_mapping_without_typed_messages_rejects_before_resources_or_ready(
        tmp_path, recorder_case, capsys, monkeypatch, managed):
    case = recorder_case(messages_available=False, mapping_enabled=True)
    if managed:
        directory = archive.begin_record(tmp_path, config={'min_free_bytes': 0})
        manifest_before = (directory/'manifest.json').read_bytes()
        monkeypatch.setattr(sys, 'argv', ['recorder', '--record-dir', str(directory),
            '--output', str(directory/'keyframes.bag'), '--duration', '.2', '--min-free-bytes', '0'])
    with pytest.raises(RuntimeError, match='mapped reference recording requires the built driver messages'):
        case.module.main()
    assert case.parameter_reads == [MAPPING_KEY]
    assert not list(tmp_path.glob('*/recorder_ready.json'))
    assert 'KEYFRAME_RECORDING_READY' not in capsys.readouterr().out
    assert case.subscriptions == []
    assert case.bags == []
    assert 'camera_reader_created' not in case.events
    assert not list(tmp_path.glob('*/keyframes.bag'))
    if managed:
        assert (directory/'manifest.json').read_bytes() == manifest_before
    else:
        assert not list(tmp_path.glob('*/manifest.json'))
