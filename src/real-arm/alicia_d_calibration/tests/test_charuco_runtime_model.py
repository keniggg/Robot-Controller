"""Current-stream RGB model tests: pure SDK math and fake callbacks, no ROS node."""
from copy import deepcopy
import importlib.util
import json
from pathlib import Path
import threading
import time
from types import SimpleNamespace as NS
from unittest.mock import Mock
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import pyrealsense2 as rs

SCRIPT = Path(__file__).resolve().parents[1] / 'scripts' / 'charuco_tracker.py'
SPEC = importlib.util.spec_from_file_location('charuco_runtime_test', SCRIPT)
T = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(T)


def parameters():
    color = dict(width=640, height=480, fx=395.7682800292969, fy=395.6027526855469,
                 cx=317.9961242675781, cy=244.833984375, model='inverse_brown_conrady',
                 coeffs=[-.05372992530465126, .05834611505270004,
                         .002122000325471163, .0007625872385688126, -.019704923033714294])
    return dict(width=640, height=480, frame_id='camera_link',
                color_topic='/supervisor/camera/color/image_raw',
                runtime_profile=dict(source='realsense_active_profile', schema_version=1,
                    sdk_version='2.56.5', device={'serial_number': 'test-only-no-device'},
                    color_intrinsics=color, geometry_intrinsics=dict(color, fx=999.),
                    color_projection_correction={'enabled': False}))


def evidence(params=None):
    return T.runtime_color_model_evidence(
        parameters() if params is None else params, 'camera_link',
        '/supervisor/camera/color/image_raw')


def model():
    return T.RuntimeColorModel(evidence(), rs)


def board_points():
    # A tilted synthetic planar board with 48 known metric corners.
    return np.asarray([[.015*x, .015*y, 0.] for y in range(6) for x in range(8)])


@pytest.mark.parametrize('change', ['missing', 'simulation', 'color_missing', 'dimensions',
                                   'frame', 'topic', 'correction', 'sdk_missing',
                                   'serial_missing', 'model', 'coefficients', 'nan',
                                   'zero_focal', 'boolean_focal'])
def test_profile_failures_never_fall_back_to_top_level_or_depth_geometry(change):
    params = parameters()
    profile = params['runtime_profile']
    if change == 'missing': params = {}
    elif change == 'simulation': profile['source'] = 'simulation'
    elif change == 'color_missing': del profile['color_intrinsics']
    elif change == 'dimensions': params['width'] = 1280
    elif change == 'frame': params['frame_id'] = 'camera_color_optical_frame'
    elif change == 'topic': params['color_topic'] = '/different'
    elif change == 'correction': profile['color_projection_correction']['enabled'] = True
    elif change == 'sdk_missing': profile['sdk_version'] = 'unavailable'
    elif change == 'serial_missing': profile['device']['serial_number'] = ''
    elif change == 'model': profile['color_intrinsics']['model'] = 'modified_brown_conrady'
    elif change == 'coefficients': profile['color_intrinsics']['coeffs'] = [0.]
    elif change == 'nan': profile['color_intrinsics']['fx'] = float('nan')
    elif change == 'zero_focal': profile['color_intrinsics']['fy'] = 0.
    elif change == 'boolean_focal': profile['color_intrinsics']['fx'] = True
    with pytest.raises(ValueError): evidence(params)


def test_rgb_model_and_source_are_frozen_without_mutating_parameters():
    params = parameters()
    before = deepcopy(params)
    result = T.RuntimeColorModel(evidence(params), rs)
    assert params == before
    assert result.intrinsics.fx == pytest.approx(395.7682800292969)
    assert result.evidence['source_field'] == 'runtime_profile.color_intrinsics'
    assert result.backend_sdk_version == params['runtime_profile']['sdk_version']
    params['runtime_profile']['color_intrinsics']['fx'] += 10.
    assert result.evidence['color_intrinsics']['fx'] == before['runtime_profile']['color_intrinsics']['fx']


@pytest.mark.parametrize('version', [None, '2.55.1'])
def test_backend_version_must_match_producer(version):
    with pytest.raises(ValueError, match='SDK_VERSION'):
        T.RuntimeColorModel(evidence(), NS(__version__=version))


def test_sdk_ray_projection_roundtrip_uses_original_pixels():
    current = model()
    pixels = np.asarray([[0., 0.], [639., 0.], [639., 479.], [0., 479.], [320., 240.]])
    rays = current.normalized_points(pixels).reshape(-1, 2)
    projected = current.project(np.c_[rays, np.ones(len(rays))], np.zeros(3), np.zeros(3))
    np.testing.assert_allclose(projected, pixels, atol=.0001)


def test_known_pose_recovers_with_sdk_rays_and_pnp_lm_identity_intrinsics(monkeypatch):
    current = model()
    xyz = board_points()
    rotation = np.array([.20, -.15, .05])
    translation = np.array([-.06, -.04, .25])
    pixels = current.project(xyz, rotation, translation)
    pnp, refine = T.cv2.solvePnP, T.cv2.solvePnPRefineLM
    arguments = []
    def checked_pnp(points, rays, matrix, distortion, **kwargs):
        arguments.append((matrix, distortion))
        return pnp(points, rays, matrix, distortion, **kwargs)
    def checked_refine(points, rays, matrix, distortion, *args):
        arguments.append((matrix, distortion))
        return refine(points, rays, matrix, distortion, *args)
    monkeypatch.setattr(T.cv2, 'solvePnP', checked_pnp)
    monkeypatch.setattr(T.cv2, 'solvePnPRefineLM', checked_refine)
    rvec, tvec, rms = current.solve(xyz, pixels)
    np.testing.assert_allclose(rvec.reshape(3), rotation, atol=2e-6)
    np.testing.assert_allclose(tvec.reshape(3), translation, atol=2e-7)
    assert rms < .0001 and len(arguments) == 2
    for matrix, distortion in arguments:
        np.testing.assert_array_equal(matrix, np.eye(3))
        assert distortion is None


@pytest.mark.parametrize('bad', ['behind', 'nan', 'inf'])
def test_sdk_projection_rejects_invalid_camera_points(bad):
    xyz = board_points()
    translation = [0., 0., -1. if bad == 'behind' else float(bad)]
    with pytest.raises(ValueError): model().project(xyz, np.zeros(3), translation)


def test_projection_rejects_any_corner_behind_not_only_board_origin():
    with pytest.raises(ValueError):
        model().project([[0., 0., 0.], [0., 0., -1.]], np.zeros(3), [0., 0., .3])


def fake_tracker(monkeypatch, responses=None):
    tracker = T.CharucoTracker.__new__(T.CharucoTracker)
    tracker.intrinsics_source = 'realsense_runtime'
    tracker.runtime_model = None
    tracker.runtime_model_rejected = tracker.runtime_model_error = None
    tracker.camera_frame, tracker.board_frame = 'camera_link', 'charuco_board'
    tracker.image_topic = '/supervisor/camera/color/image_raw'
    tracker.camera_matrix = tracker.dist_coeffs = None
    tracker.max_input_age_sec, tracker.stale_drop_count = .35, 0
    tracker.last_pose_ok, tracker.last_reprojection_rms_px = False, None
    tracker.last_marker_count, tracker.last_charuco_count = 10, 48
    tracker.last_marker_corners, tracker.last_marker_ids = [], None
    tracker.last_edge_clearance_px, tracker.detection_roi = 30., None
    tracker.board_size, tracker.dictionary_id = (12, 9), 'DICT_5X5_100'
    tracker.square_length, tracker.marker_length, tracker.detection_scale = .015, .01125, 1.
    params = parameters()
    sequence = iter(responses) if responses is not None else None
    tracker.runtime_callback_lock = threading.Lock()
    tracker.runtime_parameter_reader = NS(closed=False, read=lambda: next(sequence) if sequence else deepcopy(params))
    tracker.publish_tf = Mock()
    tracker.quality_pub, tracker.result_pub = Mock(), Mock()
    tracker.bridge = NS(imgmsg_to_cv2=lambda *_: np.zeros((480, 640, 3), dtype=np.uint8),
                        cv2_to_imgmsg=lambda *_: NS(header=None))
    xyz = board_points()
    pixels = model().project(xyz, [.20, -.15, .05], [-.06, -.04, .25]).astype(np.float32).reshape(-1, 1, 2)
    ids = np.arange(len(xyz), dtype=np.int32).reshape(-1, 1)
    tracker.board = NS(matchImagePoints=lambda *_: (xyz, pixels))
    tracker.detect_charuco_board = Mock(return_value=(pixels, ids))
    monkeypatch.setattr(T.rospy, 'logwarn_throttle', Mock())
    monkeypatch.setattr(T.rospy, 'loginfo_throttle', Mock())
    monkeypatch.setattr(T.rospy.Time, 'now', staticmethod(lambda: T.rospy.Time(100, 123)))
    image = NS(width=640, height=480, header=NS(frame_id='camera_link', stamp=T.rospy.Time(100, 123)))
    return tracker, image, params


def quality(tracker):
    return json.loads(tracker.quality_pub.publish.call_args.args[0].data)


def test_current_stream_callback_publishes_bound_model_and_exact_source_stamp(monkeypatch):
    tracker, image, _ = fake_tracker(monkeypatch)
    tracker.callback(image, None)
    tracker.publish_tf.assert_called_once()
    payload = quality(tracker)
    assert payload['pose_ok'] and payload['reprojection_rms_px'] < .0001
    assert payload['source_stamp_ns'] == image.header.stamp.to_nsec()
    assert payload['camera_model_fingerprint'] == tracker.runtime_model.fingerprint
    assert payload['backend_sdk_version'] == '2.56.5'
    assert payload['board_size'] == [12, 9] and payload['marker_length_m'] == .01125
    assert payload['square_length_m'] == .015 and payload['dictionary_id'] == 'DICT_5X5_100'


@pytest.mark.parametrize('change', ['width', 'frame', 'missing', 'reader_error'])
def test_invalid_runtime_input_never_emits_pose_or_uses_yaml_camera_info(monkeypatch, change):
    tracker, image, params = fake_tracker(monkeypatch)
    tracker.camera_matrix = np.eye(3) * 438.  # Must not become an implicit fallback.
    if change == 'width': image.width = 1280
    elif change == 'frame': image.header.frame_id = 'wrong_frame'
    elif change == 'missing': tracker.runtime_parameter_reader = NS(closed=False, read=lambda: {})
    else: tracker.runtime_parameter_reader = NS(closed=False, read=Mock(side_effect=ValueError('offline')))
    tracker.callback(image, NS(K=np.eye(3).reshape(-1).tolist(), D=[0.]*5))
    tracker.publish_tf.assert_not_called()
    assert quality(tracker)['pose_ok'] is False


def test_profile_change_during_detection_is_latched_before_tf_and_stays_rejected(monkeypatch):
    original, changed = parameters(), parameters()
    changed['runtime_profile']['color_intrinsics']['fx'] += 1.
    tracker, image, _ = fake_tracker(monkeypatch, [original, changed, original])
    tracker.callback(image, None)
    tracker.publish_tf.assert_not_called()
    assert quality(tracker)['camera_model_change_latched'] is True
    tracker.callback(image, None)
    tracker.publish_tf.assert_not_called()
    assert quality(tracker)['pose_ok'] is False


def test_missing_second_read_and_expired_image_do_not_publish_a_new_tf(monkeypatch):
    tracker, image, _ = fake_tracker(monkeypatch, [parameters(), {}])
    tracker.callback(image, None)
    tracker.publish_tf.assert_not_called()
    assert quality(tracker)['pose_ok'] is False
    tracker, image, _ = fake_tracker(monkeypatch)
    tracker._drop_if_stale = Mock(side_effect=[False, True])
    tracker.callback(image, None)
    tracker.publish_tf.assert_not_called()


def test_irrelevant_depth_preset_does_not_change_color_model_fingerprint(monkeypatch):
    original, changed = parameters(), parameters()
    changed['runtime_profile']['stationary_temporal'] = {'active': True}
    changed['runtime_profile']['mode_depth_preset'] = {'preset': 5}
    tracker, image, _ = fake_tracker(monkeypatch, [original, changed])
    tracker.callback(image, None)
    tracker.publish_tf.assert_called_once()


def test_sdk_overlay_and_rms_cannot_use_opencv_distortion_projection(monkeypatch):
    tracker, image, _ = fake_tracker(monkeypatch)
    monkeypatch.setattr(T.cv2, 'projectPoints', Mock(side_effect=AssertionError('wrong backend')))
    monkeypatch.setattr(T.cv2, 'drawFrameAxes', Mock(side_effect=AssertionError('wrong backend')))
    tracker.callback(image, None)
    tracker.publish_tf.assert_called_once()
    corners, ids = tracker.detect_charuco_board.return_value
    assert tracker._reprojection_rms_px(corners, ids, [.20, -.15, .05], [-.06, -.04, .25]) < .0001
    assert tracker._can_draw_axes_safely(np.array([.20, -.15, .05]), np.array([-.06, -.04, .25]), (480, 640, 3))
    monkeypatch.setattr(T.cv2, 'line', Mock(side_effect=AttributeError('overlay unavailable')))
    tracker.callback(image, None)
    assert quality(tracker)['pose_ok'] is True


def test_legacy_pnp_path_does_not_require_or_call_sdk_runtime_reader(monkeypatch):
    tracker, image, _ = fake_tracker(monkeypatch)
    tracker.intrinsics_source = 'legacy'
    tracker.runtime_parameter_reader = NS(closed=False, read=Mock(side_effect=AssertionError('legacy RPC')))
    tracker.camera_matrix = np.array([[400., 0, 320.], [0, 400., 240.], [0, 0, 1.]])
    tracker.dist_coeffs = np.zeros(5)
    tracker.board = T.aruco.CharucoBoard((12, 9), .015, .01125, T.aruco.getPredefinedDictionary(T.aruco.DICT_5X5_100))
    points = tracker.board.getChessboardCorners()
    pixels = T.cv2.projectPoints(points, np.zeros(3), np.array([-.06, -.04, .25]), tracker.camera_matrix, tracker.dist_coeffs)[0]
    ids = np.arange(len(points), dtype=np.int32).reshape(-1, 1)
    tracker.detect_charuco_board = Mock(return_value=(pixels, ids))
    tracker.callback(image, None)
    tracker.publish_tf.assert_called_once()
    tracker.runtime_parameter_reader.read.assert_not_called()
    assert 'camera_model_fingerprint' not in quality(tracker)


def test_old_board_api_can_match_corner_ids_without_changing_board_coordinates(monkeypatch):
    tracker, _, _ = fake_tracker(monkeypatch)
    tracker.board = NS(chessboardCorners=board_points())
    corners = np.array([[[1., 2.]], [[3., 4.]]])
    selected, image = tracker._board_points(corners, np.array([[2], [7]]))
    np.testing.assert_array_equal(selected, board_points()[[2, 7]])
    np.testing.assert_array_equal(image, corners)


def test_reader_hang_does_not_block_callback_or_spawn_more_workers():
    entered, released = threading.Event(), threading.Event()
    calls = []
    def blocked():
        calls.append(1)
        entered.set()
        released.wait()
        return {'old': True}
    reader = T.RuntimeParameterReader(blocked)
    try:
        started = time.monotonic()
        with pytest.raises(ValueError, match='TIMEOUT'): reader.read(.02)
        assert time.monotonic()-started < .25
        assert entered.wait(.2)
        for _ in range(5):
            with pytest.raises(ValueError, match='PENDING'): reader.read(.02)
        assert len(calls) == 1 and reader._worker.is_alive()
    finally:
        reader.close()
        released.set()
        reader._worker.join(.3)
    assert not reader._worker.is_alive()


def test_reader_discards_late_result_and_reads_current_parameters():
    entered, release = threading.Event(), threading.Event()
    values = iter([{'old': True}, {'new': True}])
    def read():
        value = next(values)
        if 'old' in value:
            entered.set()
            release.wait()
        return value
    reader = T.RuntimeParameterReader(read)
    try:
        with pytest.raises(ValueError, match='TIMEOUT'): reader.read(.02)
        assert entered.wait(.2)
        release.set()
        assert reader._ready.wait(.2)
        assert reader.read(.2) == {'new': True}
    finally:
        reader.close()
        reader._worker.join(.3)


def test_parameter_rpc_is_private_read_only_and_has_connection_timeout(monkeypatch):
    import xmlrpc.client
    seen = {}
    class Proxy:
        def __init__(self, uri, transport):
            seen['uri'] = uri
            connection = transport.make_connection('localhost:11311')
            seen['timeout'] = connection.timeout
        def __enter__(self): return self
        def __exit__(self, *_): pass
        def getParam(self, caller, namespace):
            seen['request'] = (caller, namespace)
            return 1, '', parameters()
    monkeypatch.setattr(xmlrpc.client, 'ServerProxy', Proxy)
    monkeypatch.setattr(T.rospy, 'get_param', Mock(side_effect=AssertionError('shared proxy')))
    result = T.read_runtime_camera_parameters('/camera', '/test', 'http://localhost:11311')
    assert result == parameters()
    assert seen == {'uri': 'http://localhost:11311', 'timeout': .2, 'request': ('/test', '/camera')}


def test_current_launch_selects_sdk_runtime_without_changing_other_camera_launches():
    launch = SCRIPT.parents[1] / 'launch' / 'd405_current_stream_handeye_invariance.launch'
    root = ET.parse(launch).getroot()
    tracker = next(n for n in root.findall('node') if n.attrib['type'] == 'charuco_tracker.py')
    params = {n.attrib['name']: n.attrib['value'] for n in tracker.findall('param')}
    args = {n.attrib['name']: n.attrib['default'] for n in root.findall('arg')}
    assert params['intrinsics_source'] == 'realsense_runtime'
    assert params['camera_parameter_namespace'] == '/camera'
    assert params['dictionary_id'] == '$(arg dictionary_id)' and args['camera_info_yaml'] == ''
    prefix = tracker.attrib['launch-prefix'].replace('$(dirname)', str(launch.parent))
    assert Path(prefix).resolve().is_file()
    for name in ('usb_charuco_eyeonhand.launch', 'd405_v4l2_charuco_eyeonhand.launch'):
        assert 'realsense_runtime' not in (launch.parent / name).read_text()


class EventAfterWait:
    """Deterministically schedule a race after ready but before result consume."""
    def __init__(self, hook):
        self.event, self.hook = threading.Event(), hook
    def set(self): self.event.set()
    def clear(self): self.event.clear()
    def is_set(self): return self.event.is_set()
    def wait(self, timeout=None):
        ready = self.event.wait(timeout)
        if ready: self.hook()
        return ready


def test_close_after_rpc_ready_before_consume_never_returns_success():
    reader = T.RuntimeParameterReader(lambda: {'result': 1})
    reader._ready = EventAfterWait(reader.close)
    try:
        with pytest.raises(ValueError, match='CLOSED'):
            reader.read(.2)
    finally:
        reader.close()
        reader._worker.join(.3)


def test_second_consumer_cannot_steal_ready_result_or_submit_new_request():
    calls, blocked = [], []
    def read():
        calls.append(1)
        return {'result': len(calls)}
    reader = T.RuntimeParameterReader(read)
    def concurrent_reader():
        with pytest.raises(ValueError, match='READER_BUSY'):
            reader.read(.01)
        blocked.append(True)
    reader._ready = EventAfterWait(concurrent_reader)
    try:
        assert reader.read(.2) == {'result': 1}
        assert calls == [1] and blocked == [True]
    finally:
        reader.close()
        reader._worker.join(.3)


def test_runtime_callback_drops_busy_frame_without_parameter_read(monkeypatch):
    tracker, image, _ = fake_tracker(monkeypatch)
    tracker.runtime_parameter_reader.read = Mock(side_effect=AssertionError('busy callback'))
    tracker.runtime_callback_lock.acquire()
    try:
        tracker.callback(image, None)
        tracker.publish_tf.assert_not_called()
        tracker.quality_pub.publish.assert_not_called()
    finally:
        tracker.runtime_callback_lock.release()


@pytest.mark.parametrize('stop', ['closed', 'shutdown'])
def test_stop_after_second_parameter_check_prevents_valid_pose(monkeypatch, stop):
    tracker, image, params = fake_tracker(monkeypatch)
    count = []
    def read():
        count.append(1)
        if len(count) == 2:
            if stop == 'closed': tracker.runtime_parameter_reader.closed = True
            else: monkeypatch.setattr(T.rospy, 'is_shutdown', lambda: True)
        return deepcopy(params)
    tracker.runtime_parameter_reader.read = read
    tracker.callback(image, None)
    tracker.publish_tf.assert_not_called()
    assert quality(tracker)['pose_ok'] is False
    assert quality(tracker)['camera_model_error'] == 'RUNTIME_CAMERA_CLOSED_OR_SHUTDOWN'


@pytest.mark.parametrize('mode', ['legacy', 'realsense_runtime'])
def test_initialization_selects_only_the_explicit_source_with_no_ros_clients(monkeypatch, mode):
    monkeypatch.setattr(T.rospy, 'init_node', Mock())
    monkeypatch.setattr(T.rospy, 'get_param', lambda key, default=None:
                        mode if key == '~intrinsics_source' else default)
    monkeypatch.setattr(T.rospy, 'Publisher', Mock())
    direct = Mock()
    paired = Mock()
    monkeypatch.setattr(T.rospy, 'Subscriber', direct)
    monkeypatch.setattr(T.message_filters, 'Subscriber', paired)
    monkeypatch.setattr(T.message_filters, 'TimeSynchronizer', Mock())
    monkeypatch.setattr(T.rospy, 'loginfo', Mock())
    monkeypatch.setattr(T.rospy, 'on_shutdown', Mock())
    monkeypatch.setattr(T.tf2_ros, 'TransformBroadcaster', Mock())
    monkeypatch.setattr(T, 'RuntimeParameterReader', Mock(return_value=NS(close=Mock())))
    tracker = T.CharucoTracker()
    if mode == 'realsense_runtime':
        direct.assert_called_once()
        paired.assert_not_called()
        assert tracker.camera_info_sub is None and tracker.camera_matrix is None
    else:
        direct.assert_not_called()
        assert paired.call_count == 2
        T.RuntimeParameterReader.assert_not_called()
    if tracker.runtime_frame_worker is not None:
        tracker._close_runtime_pipeline()
        tracker.runtime_frame_worker._worker.join(.3)
        assert not tracker.runtime_frame_worker._worker.is_alive()


def test_runtime_sdk_projection_failure_cannot_publish_pose(monkeypatch):
    tracker, image, _ = fake_tracker(monkeypatch)
    monkeypatch.setattr(rs, 'rs2_project_point_to_pixel', lambda *_: [float('nan'), 0.])
    tracker.callback(image, None)
    tracker.publish_tf.assert_not_called()
    assert quality(tracker)['pose_ok'] is False
    assert quality(tracker)['reprojection_rms_px'] is None


def test_legacy_old_aruco_api_still_receives_original_camera_model(monkeypatch):
    tracker, _, _ = fake_tracker(monkeypatch)
    tracker.intrinsics_source = 'legacy'
    tracker.camera_matrix = np.array([[438., 0, 320.], [0, 438., 240.], [0, 0, 1.]])
    tracker.dist_coeffs = np.zeros(5)
    called = []
    def old_api(corners, ids, board, matrix, distortion, *args, **kwargs):
        called.append((matrix, distortion))
        return True, np.zeros((3, 1)), np.array([[0.], [0.], [.3]])
    monkeypatch.setattr(T.aruco, 'estimatePoseCharucoBoard', old_api, raising=False)
    tracker._reprojection_rms_px = Mock(return_value=.5)
    pose, _, _ = tracker.estimate_pose(*tracker.detect_charuco_board.return_value)
    assert pose is not None and len(called) == 1
    assert called[0][0] is tracker.camera_matrix and called[0][1] is tracker.dist_coeffs


@pytest.mark.parametrize('stage', ['overlay', 'conversion'])
def test_stop_during_debug_rendering_cannot_publish_valid_quality(monkeypatch, stage):
    tracker, image, _ = fake_tracker(monkeypatch)
    if stage == 'overlay':
        def overlay(*_): tracker.runtime_parameter_reader.closed = True
        tracker._draw_runtime_axes = overlay
    else:
        def convert(*_):
            monkeypatch.setattr(T.rospy, 'is_shutdown', lambda: True)
            return NS(header=None)
        tracker.bridge.cv2_to_imgmsg = convert
    tracker.callback(image, None)
    # TF was published before the simulated shutdown; no new valid quality may follow it.
    tracker.publish_tf.assert_called_once()
    assert tracker.last_pose_ok is False and quality(tracker)['pose_ok'] is False
    assert quality(tracker)['camera_model_error'] == 'RUNTIME_CAMERA_CLOSED_OR_SHUTDOWN'


def test_latest_image_worker_replaces_pending_source_without_concurrent_processing():
    first_entered, release_first, latest_done = (threading.Event() for _ in range(3))
    original, replaced, latest = (object() for _ in range(3))
    calls, errors = [], []
    active = [0]
    def process(image):
        active[0] += 1
        assert active[0] == 1
        calls.append(image)
        try:
            if image is original:
                first_entered.set()
                assert release_first.wait(.5)
            if image is latest:
                latest_done.set()
        finally:
            active[0] -= 1
    worker = T.LatestImageWorker(process, errors.append)
    try:
        assert worker.submit(original) and first_entered.wait(.3)
        assert worker.submit(replaced) and worker.submit(latest)
        assert calls == [original]  # submission never starts concurrent detection
        release_first.set()
        assert latest_done.wait(.3)
    finally:
        worker.close()
        release_first.set()
        worker._worker.join(.3)
    assert calls == [original, latest] and not errors
    assert not worker._worker.is_alive()


def test_latest_image_worker_close_drops_pending_and_rejects_new_work():
    entered, release = threading.Event(), threading.Event()
    calls, errors = [], []
    def process(image):
        calls.append(image)
        entered.set()
        assert release.wait(.5)
    worker = T.LatestImageWorker(process, errors.append)
    try:
        assert worker.submit('in-flight') and entered.wait(.3)
        assert worker.submit('queued')
        worker.close()
        assert worker.submit('after-close') is False
        release.set()
        worker._worker.join(.3)
    finally:
        worker.close()
        release.set()
        worker._worker.join(.3)
    assert calls == ['in-flight'] and not errors
    assert not worker._worker.is_alive()


def test_runtime_image_transport_submits_original_source_without_processing(monkeypatch):
    tracker, image, _ = fake_tracker(monkeypatch)
    tracker.runtime_frame_worker = NS(submit=Mock())
    tracker.callback = Mock()
    tracker.image_callback(image)
    tracker.runtime_frame_worker.submit.assert_called_once_with(image)
    tracker.callback.assert_not_called()
    assert image.header.stamp.to_nsec() == 100000000123


def test_runtime_shutdown_closes_model_before_discarding_pending_images(monkeypatch):
    tracker, _, _ = fake_tracker(monkeypatch)
    order = []
    tracker.runtime_parameter_reader = NS(close=lambda: order.append('model'))
    tracker.runtime_frame_worker = NS(close=lambda: order.append('pending'))
    tracker._close_runtime_pipeline()
    assert order == ['model', 'pending']


@pytest.fixture(scope='module')
def rectified_parameters():
    from alicia_flexible_grasp.vision.rgbd_rectification import PairedPinholeRectification
    params = parameters()
    mapper = PairedPinholeRectification(params['runtime_profile']['color_intrinsics'], rs)
    output = mapper.contract['output_intrinsics']
    params.update(deepcopy(output), intrinsics_source='sdk_color_rectified_pinhole_canvas',
                  pair_rectification={'enabled': True})
    params['runtime_profile'].update(geometry_intrinsics=deepcopy(output),
        geometry_intrinsics_source='sdk_color_rectified_pinhole_canvas',
        rectification=deepcopy(mapper.contract), geometry_activation_stamp_ns='100000000000')
    return params


def test_raw_runtime_evidence_remains_exactly_the_original_contract():
    expected = dict(source='realsense_runtime', source_parameter='/camera',
        source_field='runtime_profile.color_intrinsics', color_intrinsics=parameters()['runtime_profile']['color_intrinsics'],
        camera_frame='camera_link', image_topic='/supervisor/camera/color/image_raw',
        serial_number='test-only-no-device', sdk_version='2.56.5', color_projection_correction=False,
        backend='realsense_sdk_rays_normalized_pnp', pixel_error_model='realsense_sdk_projection_original_pixels',
        image_time_association='parameter_snapshot_not_capture_metadata')
    assert evidence() == expected


def test_rectified_evidence_binds_output_rays_native_model_map_valid_domain_and_activation(rectified_parameters):
    params = deepcopy(rectified_parameters)
    before = deepcopy(params)
    record = evidence(params)
    assert params == before
    assert record['source_field'] == 'runtime_profile.geometry_intrinsics'
    assert record['color_intrinsics']['width'] == 653 and record['color_intrinsics']['height'] == 489
    assert record['color_intrinsics']['model'] == 'none' and record['color_intrinsics']['coeffs'] == [0.] * 5
    assert record['raw_color_intrinsics']['width'] == 640
    assert record['raw_color_intrinsics']['model'] == 'inverse_brown_conrady'
    assert record['rectification'] == params['runtime_profile']['rectification']
    assert record['geometry_activation_stamp_ns'] == '100000000000'


@pytest.mark.parametrize('change', ['missing_contract', 'digest', 'bitmap', 'raw_intrinsics',
    'geometry_intrinsics', 'top_K', 'dimensions', 'sdk_version', 'activation_missing',
    'activation_float', 'activation_noncanonical', 'activation_overflow'])
def test_rectified_model_rejects_incomplete_or_inconsistent_source(change, rectified_parameters):
    params = deepcopy(rectified_parameters)
    profile = params['runtime_profile']
    if change == 'missing_contract': del profile['rectification']
    elif change == 'digest': profile['rectification']['digest'] = '0' * 64
    elif change == 'bitmap': profile['rectification']['rgb_valid_mask_zlib_base64'] = 'broken'
    elif change == 'raw_intrinsics': profile['color_intrinsics']['cx'] += 1.
    elif change == 'geometry_intrinsics': profile['geometry_intrinsics']['cx'] += 1.
    elif change == 'top_K': params['fx'] += 1.
    elif change == 'dimensions': params['width'] = 640
    elif change == 'sdk_version': profile['sdk_version'] = '2.55.1'
    elif change == 'activation_missing': del profile['geometry_activation_stamp_ns']
    elif change == 'activation_float': profile['geometry_activation_stamp_ns'] = 1e11
    elif change == 'activation_noncanonical': profile['geometry_activation_stamp_ns'] = '0100000000000'
    elif change == 'activation_overflow': profile['geometry_activation_stamp_ns'] = str(2**63)
    with pytest.raises(ValueError): evidence(params)


def test_actual_sdk_rectified_board_pose_uses_none_rays_without_second_distortion(rectified_parameters, monkeypatch):
    raw = model()
    current = T.RuntimeColorModel(evidence(rectified_parameters), rs)
    xyz, rotation, translation = board_points(), np.array([.20, -.15, .05]), np.array([-.06, -.04, .25])
    native_pixels = raw.project(xyz, rotation, translation)
    rays = raw.normalized_points(native_pixels).reshape(-1, 2)
    intr = current.evidence['color_intrinsics']
    ideal_pixels = rays * [intr['fx'], intr['fy']] + [intr['cx'], intr['cy']]
    deproject = rs.rs2_deproject_pixel_to_point
    models = []
    def checked(intrinsics, pixel, depth):
        models.append(intrinsics.model)
        assert list(intrinsics.coeffs) == [0.] * 5
        return deproject(intrinsics, pixel, depth)
    monkeypatch.setattr(rs, 'rs2_deproject_pixel_to_point', checked)
    rvec, tvec, rms = current.solve(xyz, ideal_pixels)
    np.testing.assert_allclose(rvec.reshape(3), rotation, atol=2e-6)
    np.testing.assert_allclose(tvec.reshape(3), translation, atol=2e-7)
    assert rms < .0001 and models == [rs.distortion.none] * len(xyz)


def fake_rectified_tracker(monkeypatch, rectified_parameters, responses=None):
    params = deepcopy(rectified_parameters)
    tracker, image, _ = fake_tracker(monkeypatch, responses or [params, params])
    tracker.detection_roi_padding_px = 45.
    image.width, image.height = params['width'], params['height']
    tracker.bridge.imgmsg_to_cv2 = lambda *_: np.zeros((image.height, image.width, 3), dtype=np.uint8)
    current = T.RuntimeColorModel(evidence(params), rs)
    pixels = current.project(board_points(), [.20, -.15, .05], [-.06, -.04, .25]).astype(np.float32).reshape(-1, 1, 2)
    ids = np.arange(len(pixels), dtype=np.int32).reshape(-1, 1)
    tracker.board.matchImagePoints = lambda *_: (board_points(), pixels)
    tracker.detect_charuco_board.return_value = pixels, ids
    return tracker, image, params


def test_rectified_callback_publishes_original_board_standards_and_bound_model(monkeypatch, rectified_parameters):
    tracker, image, params = fake_rectified_tracker(monkeypatch, rectified_parameters)
    tracker.callback(image, None)
    tracker.publish_tf.assert_called_once()
    payload = quality(tracker)
    assert payload['pose_ok'] and payload['source_stamp_ns'] == image.header.stamp.to_nsec()
    assert payload['camera_model']['rectification']['digest'] == params['runtime_profile']['rectification']['digest']
    assert payload['board_size'] == [12, 9] and payload['dictionary_id'] == 'DICT_5X5_100'
    assert payload['square_length_m'] == .015 and payload['marker_length_m'] == .01125


@pytest.mark.parametrize('delta', [-1, 0])
def test_old_queued_frame_is_rejected_even_with_same_canvas_shape(monkeypatch, rectified_parameters, delta):
    tracker, image, _ = fake_rectified_tracker(monkeypatch, rectified_parameters)
    stamp_ns = int(rectified_parameters['runtime_profile']['geometry_activation_stamp_ns']) + delta
    image.header.stamp = T.rospy.Time(stamp_ns // 10**9, stamp_ns % 10**9)
    tracker.callback(image, None)
    tracker.publish_tf.assert_not_called()
    tracker.detect_charuco_board.assert_not_called()
    assert quality(tracker)['pose_ok'] is False
    assert tracker.runtime_model is None


@pytest.mark.parametrize('activation', ['100000000050', '100000000500'])
def test_same_model_new_activation_during_detection_blocks_tf_and_requires_restart(monkeypatch, rectified_parameters, activation):
    changed = deepcopy(rectified_parameters)
    changed['runtime_profile']['geometry_activation_stamp_ns'] = activation
    tracker, image, _ = fake_rectified_tracker(monkeypatch, rectified_parameters,
        [deepcopy(rectified_parameters), changed, deepcopy(rectified_parameters)])
    tracker.callback(image, None)
    tracker.publish_tf.assert_not_called()
    assert tracker.runtime_model_rejected == 'RUNTIME_CAMERA_MODEL_CHANGED_RESTART_REQUIRED'
    tracker.callback(image, None)
    tracker.publish_tf.assert_not_called()


def test_bitmap_boundary_checks_full_subpixel_cell_and_detection_support(rectified_parameters):
    current = T.RuntimeColorModel(evidence(rectified_parameters), rs)
    valid = current.rgb_valid
    # An actual observed cell directly beside real padded RGB: rounding to the
    # valid side alone must not qualify a corner whose interpolation uses both.
    y, x = np.argwhere(valid[:, 1:] & ~valid[:, :-1])[0]
    current.validate_rgb_points([[x+1., y]])
    with pytest.raises(ValueError, match='SAMPLE_UNOBSERVED'):
        current.validate_rgb_points([[x+.75, y]])
    with pytest.raises(ValueError, match='SAMPLE_UNOBSERVED'):
        current.validate_rgb_points([[x, y]])
    current.validate_rgb_detection_region([], np.array([[[320., 240.]]]), 45.)
    # The corner center is valid, but its existing detection ROI includes the
    # synthetic padding. It cannot supply calibration identity evidence.
    with pytest.raises(ValueError, match='DETECTION_SUPPORT_UNOBSERVED'):
        current.validate_rgb_detection_region([], np.array([[[x+1., y]]]), 45.)


def test_unobserved_marker_support_cannot_publish_pose(monkeypatch, rectified_parameters):
    tracker, image, params = fake_rectified_tracker(monkeypatch, rectified_parameters)
    current = T.RuntimeColorModel(evidence(params), rs)
    y, x = np.argwhere(~current.rgb_valid)[0]
    tracker.last_marker_corners = [np.array([[[x, y], [x+1, y], [x+1, y+1], [x, y+1]]], dtype=float)]
    tracker.last_marker_ids = np.array([[1]], dtype=np.int32)
    tracker.callback(image, None)
    tracker.publish_tf.assert_not_called()
    assert quality(tracker)['pose_ok'] is False
    assert 'RGB_' in quality(tracker)['camera_model_error']
