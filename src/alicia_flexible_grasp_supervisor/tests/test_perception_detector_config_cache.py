"""Configuration polling must not repeatedly discover ROS model packages."""
from types import SimpleNamespace
from unittest import mock

import numpy as np
import pytest

from test_perception_detector_selection import (
    FakeBridge,
    FakeMessagePublisher,
    FakePublisher,
    FakeRospy,
    NoFrameBuffer,
    load_perception_node,
)


@pytest.fixture
def configured_node():
    module = load_perception_node()
    config = {
        'enabled': True,
        'detector': 'yolov8',
        'object_label': 'target',
        'yolo_model_choice': 'custom',
        'yolo_reload_generation': 1,
        'yolo_models': {
            'custom': {
                'model_path': 'models/custom.pt',
                'task': 'detect',
                'target_class_mode': 'description',
            },
        },
    }
    module.rospy = FakeRospy(config)
    module.rospy.get_param = mock.Mock(wraps=module.rospy.get_param)
    module.select_yolo_model = mock.Mock(wraps=module.select_yolo_model)
    module.resolve_yolo_model_path = mock.Mock(side_effect=lambda path: '/resolved/' + path)
    module.YOLOv8ObjectDetector = mock.Mock()
    node = module.PerceptionNode.__new__(module.PerceptionNode)
    node.detector = None
    node.detector_signature = None
    node.detector_status_pub = FakePublisher()
    node.pub_obj = FakeMessagePublisher()
    node.pub_detected = FakeMessagePublisher()
    node.pub_raw_detected = FakeMessagePublisher()
    node.stabilizer = module.DetectionStabilizer()
    node.frames = NoFrameBuffer()
    return module, config, node


@pytest.mark.parametrize('enabled', [True, False])
def test_unchanged_no_frame_polls_read_config_without_rediscovering_models(configured_node, enabled):
    module, config, node = configured_node
    config['enabled'] = enabled
    node.refresh_detector(force=True)
    detector = node.detector

    for _ in range(24):
        assert node._poll_detector_and_process_latest_frame() is False

    assert module.rospy.get_param.call_count == 25
    assert node.frames.polls == 24
    assert node.detector is detector
    module.select_yolo_model.assert_called_once()
    module.resolve_yolo_model_path.assert_called_once_with('models/custom.pt')
    module.YOLOv8ObjectDetector.assert_called_once()
    assert len(node.detector_status_pub.messages) == 2


def test_nested_profile_edit_invalidates_snapshot(configured_node):
    module, config, node = configured_node
    node.refresh_detector(force=True)
    config['yolo_models']['custom']['model_path'] = 'models/replacement.pt'

    node.refresh_detector()
    node.refresh_detector()

    assert module.resolve_yolo_model_path.call_args_list == [
        mock.call('models/custom.pt'), mock.call('models/replacement.pt'),
    ]
    assert module.YOLOv8ObjectDetector.call_count == 2
    assert module.YOLOv8ObjectDetector.call_args.kwargs['model_path'] == '/resolved/models/replacement.pt'


@pytest.mark.parametrize('request', ['force', 'generation'])
def test_explicit_reload_rechecks_unchanged_model_selection(configured_node, request):
    module, config, node = configured_node
    node.refresh_detector(force=True)
    if request == 'generation':
        config['yolo_reload_generation'] += 1
    node.refresh_detector(force=request == 'force')
    node.refresh_detector()

    assert module.select_yolo_model.call_count == 2
    assert module.resolve_yolo_model_path.call_count == 2
    assert module.YOLOv8ObjectDetector.call_count == 2


def test_failed_path_stays_unavailable_until_explicit_reload(configured_node):
    module, config, node = configured_node
    module.resolve_yolo_model_path.side_effect = FileNotFoundError('missing weights')
    node.refresh_detector(force=True)
    for _ in range(12):
        assert node._poll_detector_and_process_latest_frame() is False

    module.resolve_yolo_model_path.assert_called_once()
    module.YOLOv8ObjectDetector.assert_not_called()
    assert node.detector is None
    assert node.detector_error == 'missing weights'
    assert node.detector_status_pub.messages[-1] == 'error:custom:missing weights'
    assert node.pub_obj.messages[-1].detected is False
    assert node.pub_detected.messages[-1].data is False
    assert node.pub_raw_detected.messages[-1].data is False

    module.resolve_yolo_model_path.side_effect = lambda path: '/resolved/' + path
    config['yolo_reload_generation'] += 1
    assert node._poll_detector_and_process_latest_frame() is False

    assert module.resolve_yolo_model_path.call_count == 2
    module.YOLOv8ObjectDetector.assert_called_once()
    assert node.detector is not None
    assert node.detector_error == ''


def test_failed_profile_selection_is_cached_and_config_fix_retries(configured_node):
    module, config, node = configured_node
    config['yolo_models']['custom']['task'] = 'invalid'
    node.refresh_detector(force=True)
    node.refresh_detector()

    module.select_yolo_model.assert_called_once()
    module.resolve_yolo_model_path.assert_not_called()
    assert node.detector is None
    assert node.pub_obj.messages[-1].detected is False

    config['yolo_models']['custom']['task'] = 'segment'
    node.refresh_detector()

    assert module.select_yolo_model.call_count == 2
    module.resolve_yolo_model_path.assert_called_once()
    assert node.detector is not None
    assert node.detector_task == 'segment'
    assert node.require_instance_mask is True


@pytest.mark.parametrize('key,value', [
    ('enabled', False),
    ('object_label', 'unknown object'),
    ('yolo_conf', 0.72),
    ('tracking_max_jump_px', 35.0),
])
def test_runtime_configuration_changes_remain_observable(configured_node, key, value):
    module, config, node = configured_node
    node.refresh_detector(force=True)
    config[key] = value
    node.refresh_detector()
    node.refresh_detector()

    assert module.resolve_yolo_model_path.call_count == 2
    assert module.YOLOv8ObjectDetector.call_count == 2
    assert node.enabled == config['enabled']
    assert node.label == config['object_label']
    assert node.stabilizer.max_jump_px == config.get('tracking_max_jump_px', 90.0)
    assert module.YOLOv8ObjectDetector.call_args.kwargs['conf'] == config.get('yolo_conf', 0.35)


def test_non_detector_config_change_updates_cache_without_reloading(configured_node):
    module, config, node = configured_node
    node.refresh_detector(force=True)
    config['detect_hz'] = 8.0
    for _ in range(12):
        node.refresh_detector()

    assert module.resolve_yolo_model_path.call_count == 2
    module.YOLOv8ObjectDetector.assert_called_once()


@pytest.mark.parametrize('state', ['disabled', 'unavailable'])
def test_cached_config_keeps_negative_frame_and_mask_publication(configured_node, state):
    module, config, node = configured_node
    config['enabled'] = state != 'disabled'
    config['yolo_models']['custom']['task'] = 'segment'
    if state == 'unavailable':
        module.YOLOv8ObjectDetector.side_effect = RuntimeError('cannot load model')
    node.frames = module.LatestFrameBuffer()
    node.bridge = FakeBridge()
    node.pub_mask = FakeMessagePublisher()
    node._refresh_camera_params = lambda: None
    node.refresh_detector(force=True)
    node.pub_obj.messages.clear()
    node.pub_detected.messages.clear()
    node.pub_raw_detected.messages.clear()
    node.frames.update_depth(np.ones((2, 2), dtype=np.uint16))

    for frame_index in range(3):
        stamp = SimpleNamespace(secs=10, nsecs=frame_index)
        node.frames.update_color(np.zeros((2, 2, 3), dtype=np.uint8), stamp, 'camera_latest')
        assert node._poll_detector_and_process_latest_frame() is True
        assert node.pub_obj.messages[-1].detected is False
        assert node.pub_detected.messages[-1].data is False
        assert node.pub_raw_detected.messages[-1].data is False
        mask = node.pub_mask.messages[-1]
        assert np.count_nonzero(mask.data) == 0
        assert mask.header.stamp is stamp
        assert mask.header.frame_id == 'camera_latest'

    assert len(node.pub_obj.messages) == 3
    module.select_yolo_model.assert_called_once()
    module.resolve_yolo_model_path.assert_called_once()
    module.YOLOv8ObjectDetector.assert_called_once()
    if state == 'disabled':
        module.YOLOv8ObjectDetector.return_value.detect.assert_not_called()
