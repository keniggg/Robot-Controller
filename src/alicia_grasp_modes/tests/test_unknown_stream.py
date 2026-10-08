"""Ingress remains live during expensive image processing; stamps stay exact."""
import importlib.util
from pathlib import Path
import threading
import types

import pytest


path = Path(__file__).resolve().parents[1] / 'scripts' / 'unknown_tabletop_perception.py'
spec = importlib.util.spec_from_file_location('unknown_stream_test', path)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def node():
    value = module.UnknownTabletopPerception.__new__(module.UnknownTabletopPerception)
    value.lock, value.input_lock = threading.RLock(), threading.RLock()
    value.mode, value.generation, value.selection_ns = 'unknown', 7, 100
    value.latest, value.last_consumed_ns, value.last_receive_wall = None, 110, 0.
    value.first_loss, value.last_good_source_stamp_ns = None, 0
    return value


def image(stamp):
    return types.SimpleNamespace(header=types.SimpleNamespace(
        stamp=types.SimpleNamespace(to_nsec=lambda:stamp)))


def test_latest_pair_is_received_while_processing_lock_is_busy(monkeypatch):
    monkeypatch.setattr(module.rospy.Time, 'now', staticmethod(lambda:module.rospy.Time(0,200)))
    n = node()
    first, second = image(120), image(130)
    done = threading.Event()
    def receive():
        n.frame(first, first)
        n.frame(second, second)
        done.set()
    with n.lock:
        thread = threading.Thread(target=receive)
        thread.start()
        received_without_processing_unlock = done.wait(1.)
    thread.join(timeout=1.)
    assert received_without_processing_unlock
    assert n.latest == (second, second)
    assert n.last_receive_wall > 0


@pytest.mark.parametrize('color,depth', [(100,100),(110,110),(120,121)])
def test_input_lock_does_not_weaken_stamp_or_generation_boundaries(color, depth):
    n = node()
    n.frame(image(color), image(depth))
    assert n.latest is None and n.last_receive_wall == 0.


def test_inactive_mode_does_not_accept_buffered_images():
    n = node()
    n.mode = 'carton'
    n.frame(image(120), image(120))
    assert n.latest is None


def test_late_delivery_cannot_replace_a_newer_unprocessed_pair(monkeypatch):
    monkeypatch.setattr(module.rospy.Time, 'now', staticmethod(lambda:module.rospy.Time(0,200)))
    n = node()
    newest = image(130)
    n.frame(newest, newest)
    n.frame(image(120), image(120))
    assert n.latest == (newest, newest)


def test_failed_tracked_refinement_withholds_raw_mask_and_recovers_next_frame(monkeypatch):
    import numpy as np
    from cv_bridge import CvBridge
    from unittest.mock import Mock
    n = node()
    n.max_age, n.tf_wait, n.minimum_quality = .8, .15, .5
    n.base_frame, n.camera_frame = 'base_link', 'camera_link'
    n.config, n.rgb_instance_refinement = module.Config(), True
    n.bridge, n.last_shape = CvBridge(), (20, 20)
    n.anchor_source_stamp_ns = 120
    raw = types.SimpleNamespace(mask=np.full((20,20),255,np.uint8), quality=.9,
        center_uv=[10.,10.], bbox=(0,0,20,20), depth_m=.3,
        position_camera=[.3,0.,0.], position_base=[.3,0.,0.])
    n.tracker = types.SimpleNamespace(anchor=raw, last=raw, lost=False, plane_base=np.array([0.,0.,1.,0.]),
        choose=lambda *_:(raw,'tracked'), unavailable=Mock())
    n.buffer = types.SimpleNamespace(lookup_transform=lambda *_:object())
    n.status = Mock()
    n.pubs = {key:Mock() for key in ('object','object_mask','object_detected','raw_object_detected',
                                    'object_pose_camera','object_pose_base')}
    monkeypatch.setattr(module.rospy.Time, 'now', staticmethod(lambda:module.rospy.Time(10)))
    monkeypatch.setattr(module.rospy, 'get_param', lambda *_:dict(align_depth_to_color=True,fx=400,fy=400,cx=10,cy=10))
    monkeypatch.setattr(module, 'transform_matrix', lambda _:np.eye(4))
    monkeypatch.setattr(module, 'segment_with_support_fallback',
        lambda *_,**__:types.SimpleNamespace(metrics={'support_current_audit_available':True}))
    def rejected(*_):
        raise ValueError('rgb_instance_reaches_search_boundary')
    monkeypatch.setattr(module, 'refine_candidate', rejected)
    def pair(ns):
        rgb = n.bridge.cv2_to_imgmsg(np.zeros((20,20,3),np.uint8), encoding='bgr8')
        depth = n.bridge.cv2_to_imgmsg(np.full((20,20),.3,np.float32), encoding='32FC1')
        for msg in (rgb,depth):
            msg.header.stamp = module.rospy.Time(9,ns)
            msg.header.frame_id = 'camera_link'
        return rgb,depth
    n.latest = pair(900000000)
    n.process(None)
    assert not n.pubs['object'].publish.call_args[0][0].detected
    published = n.pubs['object_mask'].publish.call_args[0][0]
    assert not np.any(n.bridge.imgmsg_to_cv2(published, desired_encoding='mono8'))
    n.pubs['object_pose_base'].publish.assert_not_called()
    n.tracker.unavailable.assert_not_called()
    assert n.tracker.anchor is raw
    assert n.first_loss is None
    assert 'rgb_depth_refinement_rejected' in n.status.call_args[0][1]
    monkeypatch.setattr(module, 'refine_candidate', lambda *_:raw)
    n.latest = pair(950000000)
    n.process(None)
    assert n.pubs['object'].publish.call_args[0][0].detected
    assert n.pubs['object'].publish.call_args[0][0].header.stamp == module.rospy.Time(9,950000000)


def diagnostic_node(monkeypatch):
    import numpy as np
    from cv_bridge import CvBridge
    from unittest.mock import Mock
    n = node()
    n.base_frame, n.camera_frame = 'base_link', 'camera_link'
    n.bridge, n.last_shape = CvBridge(), (4, 4)
    n.anchor_source_stamp_ns, n.last_good_source_stamp_ns = 120, 130
    anchor = types.SimpleNamespace(position_base=np.array([.2,.1,.05]),
        extent_m=np.array([.02,.01,.01]), support_footprint_base=np.zeros((4,3)))
    n.tracker = types.SimpleNamespace(anchor=anchor, lost=False,
        plane_base=np.array([0.,0.,1.,0.]))
    n.tracker.unavailable = Mock(side_effect=lambda:setattr(n.tracker,'lost',True))
    n.pubs = {key:Mock() for key in ('object','object_mask','object_detected',
        'raw_object_detected','detector_status')}
    monkeypatch.setattr(module.rospy.Time,'now',staticmethod(lambda:module.rospy.Time(10)))
    monkeypatch.setattr(module.rospy,'logwarn',Mock())
    return n


def test_loss_status_keeps_first_frame_cause_and_stays_lost(monkeypatch):
    import json
    from sensor_msgs.msg import Image
    n=diagnostic_node(monkeypatch)
    rgb,depth=Image(),Image()
    for msg in (rgb,depth):
        msg.header.stamp=module.rospy.Time(9)
        msg.header.frame_id='camera_link'
    n.unavailable('ambiguous_locked_instances',rgb.header.stamp,(rgb,depth))
    first=dict(n.first_loss)
    n.unavailable('target_lost_requires_new_generation',module.rospy.Time(10),(rgb,depth))
    status=json.loads(n.pubs['detector_status'].publish.call_args[0][0].data)
    assert n.tracker.lost and n.tracker.unavailable.call_count==2
    assert status['first_loss']==first
    assert first['reason']=='ambiguous_locked_instances'
    assert first['source_stamp_ns']==9_000_000_000
    assert status['detail']=='target_lost_requires_new_generation'


def test_source_less_timeout_records_time_separately_and_selection_owns_reset(monkeypatch):
    import json
    from std_msgs.msg import String
    from unittest.mock import Mock
    n=diagnostic_node(monkeypatch)
    n.unavailable('exact_rgbd_stream_stale')
    assert n.first_loss['source_stamp_ns'] is None
    assert n.first_loss['observed_stamp_ns']==10_000_000_000
    n.selection(String(data=json.dumps(dict(mode='unknown',generation=7,stamp_ns=200))))
    assert n.first_loss is not None  # Duplicate selection must not erase the cause.
    def reset():
        n.tracker.anchor=None
        n.tracker.lost=False
    n.tracker.reset=Mock(side_effect=reset)
    n.acquisition=types.SimpleNamespace(reset=Mock())
    n.selection(String(data=json.dumps(dict(mode='unknown',generation=8,stamp_ns=300))))
    assert n.first_loss is None and n.last_good_source_stamp_ns==0


def rgb_interest_node(monkeypatch):
    """Real RGB detector/support fit; fake transport only, no ROS master."""
    import numpy as np
    from cv_bridge import CvBridge
    from unittest.mock import Mock
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Header
    n = node()
    n.max_age, n.tf_wait, n.minimum_quality = .8, .15, .5
    n.base_frame, n.camera_frame = 'base_link', 'camera_link'
    n.config, n.rgb_instance_refinement = module.Config(), True
    n.bridge, n.last_shape = CvBridge(), (240, 320)
    n.anchor_source_stamp_ns = 0
    n.tracker, n.acquisition = module.TargetTracker(n.config), module.InitialAcquisition(n.config)
    n._initialize_rgb_interest({})
    n.pubs = {key: Mock() for key in ('object', 'object_mask', 'object_detected',
        'raw_object_detected', 'object_pose_camera', 'object_pose_base', 'detector_status', 'rgb_reference_status')}
    clock = [10.]
    monkeypatch.setattr(module.time, 'monotonic', lambda: clock[0])
    monkeypatch.setattr(module.rospy.Time, 'now', staticmethod(lambda: module.rospy.Time.from_sec(clock[0])))
    camera = dict(align_depth_to_color=True, fx=400., fy=400., cx=160., cy=120.,
                  depth_scale=.0001, width=320, height=240, frame_id='camera_link')
    parameters = {'/camera': camera, '/robot_description': '<robot name="test"/>',
        '/grasp_mode/selection': dict(mode='unknown', strategy='observe_return',
            generation=n.generation, stamp_ns=n.selection_ns), '/handeye': {}, '/unknown_perception': {}}
    monkeypatch.setattr(module.rospy, 'get_param', lambda key, default=None: parameters.get(key, default))
    monkeypatch.setattr(module.rospy, 'get_param_cached', lambda key, default=None: parameters.get(key, default))
    monkeypatch.setattr(module.rospy, 'logwarn_throttle', Mock())
    transform = np.array([[0., -1., 0., .1], [0., 0., 1., -.2],
                          [-1., 0., 0., .4], [0., 0., 0., 1.]])
    n.buffer = types.SimpleNamespace(lookup_transform=lambda *args: args)
    monkeypatch.setattr(module, 'transform_matrix', lambda args:
                        transform.copy() if args[0] == 'base_link' else np.eye(4))
    n._rgb_interest_epoch_cb(Header(stamp=module.rospy.Time(1), frame_id='sdk_reference_epoch'))

    def joint(stamp, q=None):
        message = JointState()
        message.header.stamp = module.rospy.Time.from_sec(stamp)
        message.header.frame_id = 'sdk_measured'
        message.name, message.position = list(module.JOINT_NAMES), list(np.zeros(6) if q is None else q)
        n._rgb_interest_joint_cb(message)

    def feed(*, bracket=True, q=None, gap=.10, target_depth=0.):
        stamp = clock[0] - .1
        rgb = np.full((240, 320, 3), (60, 90, 190), np.uint8)
        rgb[106:134, 148:173] = 20
        depth = np.full((240, 320), .4, np.float32)
        depth[106:134, 148:173] = target_depth
        messages = (n.bridge.cv2_to_imgmsg(rgb, encoding='bgr8'),
                    n.bridge.cv2_to_imgmsg(depth, encoding='32FC1'))
        for message in messages:
            message.header.stamp = module.rospy.Time.from_sec(stamp)
            message.header.frame_id = n.camera_frame
        joint(stamp-gap/2, q)
        if bracket:
            joint(stamp+gap/2, q)
        n.latest = messages
        n.process(None)
        return messages

    return n, clock, feed, joint, parameters


def test_initial_rgb_interest_is_separate_from_detected_target_and_mask(monkeypatch):
    import json
    import numpy as np
    n, clock, feed, _, _ = rgb_interest_node(monkeypatch)
    for i in range(3):
        clock[0] = 10. + .6 * i
        feed()
        report = json.loads(n.pubs['detector_status'].publish.call_args[0][0].data)
        assert report['state'] == 'unavailable' and report['detail'] == 'no_center_instance'
        assert report['target_locked'] is False and report['target_lost'] is False
        assert not n.pubs['object'].publish.call_args[0][0].detected
        mask = n.pubs['object_mask'].publish.call_args[0][0]
        assert not np.any(n.bridge.imgmsg_to_cv2(mask, 'mono8'))
        assert n.tracker.anchor is None and not n.tracker.lost and n.acquisition.samples == []
        interest = report['observation_interest']
        assert interest['rgb_track']['consistent_frames'] == i+1
        assert interest['observation_interest_ready'] is (i == 2)
    assert interest['interest_volume']['support_provenance']['spatial_quadrants'] == 4
    assert interest['interest_volume']['usable_as_collision_geometry'] is False
    assert interest['motion_authorized'] is False
    assert interest['contact_geometry_authorized'] is False
    assert interest['metric_geometry_authority'] is False
    n.pubs['object_pose_base'].publish.assert_not_called()
    n.pubs['object_pose_camera'].publish.assert_not_called()


def test_rgb_work_is_capped_at_two_hz_without_reusing_ready_payload(monkeypatch):
    from unittest.mock import Mock
    n, clock, feed, _, _ = rgb_interest_node(monkeypatch)
    original = module.propose_rgb_observation_regions
    spy = Mock(side_effect=original)
    monkeypatch.setattr(module, 'propose_rgb_observation_regions', spy)
    for offset in (0., .2, .4, .6, .8):
        clock[0] = 10. + offset
        feed()
    assert spy.call_count == 2
    assert n._rgb_observation_interest['reason'] == 'rgb_observation_rate_limited'
    assert n._rgb_observation_interest['observation_interest_ready'] is False
    assert 'interest_volume' not in n._rgb_observation_interest


@pytest.mark.parametrize('kind', ['missing_after', 'wide_bracket', 'changed_camera', 'changed_config'])
def test_rgb_interest_source_failures_preserve_original_no_target_result(monkeypatch, kind):
    n, _, feed, _, parameters = rgb_interest_node(monkeypatch)
    if kind == 'changed_camera':
        parameters['/camera']['width'] = 321
    if kind == 'changed_config':
        parameters['/unknown_perception'] = {'foreground_min_m': .001}
    feed(bracket=kind != 'missing_after', gap=.201 if kind == 'wide_bracket' else .1)
    assert not n._rgb_observation_interest['observation_interest_ready']
    assert 'rgb_track' not in n._rgb_observation_interest
    assert n.tracker.anchor is None
    assert not n.pubs['object'].publish.call_args[0][0].detected


@pytest.mark.parametrize('event', ['epoch', 'motion', 'stale', 'selection'])
def test_weak_interest_is_retired_on_input_changes(monkeypatch, event):
    from std_msgs.msg import Header, String
    import json
    n, clock, feed, joint, _ = rgb_interest_node(monkeypatch)
    for offset in (0., .6, 1.2):
        clock[0] = 10.+offset
        feed()
    assert n._rgb_observation_interest['observation_interest_ready']
    if event == 'epoch':
        n._rgb_interest_epoch_cb(Header(stamp=module.rospy.Time(11), frame_id='sdk_reference_epoch'))
        assert not n._rgb_interest_feedback
    elif event == 'motion':
        joint(clock[0], [0., 0., 0., .02, 0., 0.])
    elif event == 'stale':
        n.unavailable('exact_rgbd_stream_stale')
    else:
        n.selection(String(data=json.dumps(dict(mode='unknown', generation=8, stamp_ns=300))))
    assert not n._rgb_observation_interest['observation_interest_ready']
    assert 'interest_volume' not in n._rgb_observation_interest
    assert n.tracker.anchor is None


def test_rgb_tracking_restarts_on_camera_calibration_change(monkeypatch):
    n, clock, feed, _, parameters = rgb_interest_node(monkeypatch)
    feed()
    clock[0] += .6
    feed()
    assert n._rgb_observation_interest['rgb_track']['consistent_frames'] == 2
    parameters['/camera']['fx'] = 401.
    clock[0] += .6
    feed()
    assert n._rgb_observation_interest['rgb_track']['consistent_frames'] == 1
    assert not n._rgb_observation_interest['observation_interest_ready']


def test_rgb_interest_callbacks_are_bounded_and_do_not_take_processing_lock(monkeypatch):
    n, _, _, joint, _ = rgb_interest_node(monkeypatch)
    done = threading.Event()

    def receive():
        for i in range(200):
            joint(2. + i * .01)
        done.set()

    with n.lock:
        worker = threading.Thread(target=receive)
        worker.start()
        available = done.wait(1.)
    worker.join(timeout=1.)
    assert available
    assert len(n._rgb_interest_feedback) == 128


def test_source_change_during_rgb_work_discards_inflight_evidence(monkeypatch):
    from std_msgs.msg import Header
    n, _, feed, _, _ = rgb_interest_node(monkeypatch)
    original = module.propose_rgb_observation_regions

    def changed(*args, **kwargs):
        result = original(*args, **kwargs)
        n._rgb_interest_epoch_cb(Header(stamp=module.rospy.Time(2), frame_id='sdk_reference_epoch'))
        return result

    monkeypatch.setattr(module, 'propose_rgb_observation_regions', changed)
    feed()
    assert not n._rgb_observation_interest['observation_interest_ready']
    assert 'rgb_track' not in n._rgb_observation_interest
    assert n.tracker.anchor is None


def test_source_expiring_during_rgb_work_cannot_publish_ready_evidence(monkeypatch):
    n, clock, feed, _, _ = rgb_interest_node(monkeypatch)
    original = module.propose_rgb_observation_regions

    def delayed(*args, **kwargs):
        result = original(*args, **kwargs)
        clock[0] += .81
        return result

    monkeypatch.setattr(module, 'propose_rgb_observation_regions', delayed)
    feed()
    assert n._rgb_observation_interest['reason'] == 'source_exceeds_existing_frame_limit'
    assert not n._rgb_observation_interest['observation_interest_ready']
    assert 'interest_volume' not in n._rgb_observation_interest
    assert not n.pubs['object'].publish.call_args[0][0].detected


def test_ambiguous_frame_retires_volume_until_three_new_rgb_samples(monkeypatch):
    from dataclasses import replace
    n, clock, feed, _, _ = rgb_interest_node(monkeypatch)
    for offset in (0., .6, 1.2):
        clock[0] = 10.+offset
        feed()
    assert n._rgb_observation_interest['observation_interest_ready']
    original = module.propose_rgb_observation_regions
    monkeypatch.setattr(module, 'propose_rgb_observation_regions', lambda *args, **kwargs:
        replace(original(*args, **kwargs), selected_index=None, reason='ambiguous_rgb_regions'))
    clock[0] += .6
    feed()
    assert n._rgb_observation_interest['rgb_track']['consistent_frames'] == 0
    assert not n._rgb_observation_interest['observation_interest_ready']
    assert 'interest_volume' not in n._rgb_observation_interest
    monkeypatch.setattr(module, 'propose_rgb_observation_regions', original)
    for index in range(3):
        clock[0] += .6
        feed()
        assert n._rgb_observation_interest['rgb_track']['consistent_frames'] == index+1
        assert n._rgb_observation_interest['observation_interest_ready'] is (index == 2)


def test_model_digest_is_updated_without_parsing_urdf(monkeypatch):
    n, clock, feed, _, parameters = rgb_interest_node(monkeypatch)
    feed()
    first = n._rgb_interest_model_cache
    clock[0] += .6
    feed()
    assert n._rgb_interest_model_cache is first
    parameters['/robot_description'] = '<robot name="changed"/>'
    clock[0] += .6
    feed()
    assert n._rgb_interest_model_cache[1] != first[1]
    assert n._rgb_observation_interest['rgb_track']['consistent_frames'] == 1


def test_support_coverage_failure_cannot_be_replaced_by_rgb_region(monkeypatch):
    n, clock, feed, _, _ = rgb_interest_node(monkeypatch)
    monkeypatch.setattr(module, 'measured_support_coverage', lambda *args:
                        dict(spatial_quadrants=2, measured_support_pixel_count=1000))
    for index in range(3):
        clock[0] = 10.+.6*index
        feed()
    assert 'SUPPORT_PROVENANCE_INVALID' in n._rgb_observation_interest['reason']
    assert not n._rgb_observation_interest['observation_interest_ready']
    assert 'interest_volume' not in n._rgb_observation_interest
    assert not n.pubs['object'].publish.call_args[0][0].detected


def test_rgb_interest_cannot_use_a_transported_old_support_plane(monkeypatch):
    n, clock, feed, _, _ = rgb_interest_node(monkeypatch)
    original = module.segment_with_support_fallback

    def stale_plane(*args, **kwargs):
        result = original(*args, **kwargs)
        result.metrics['support_source'] = 'locked_base_plane'
        return result

    monkeypatch.setattr(module, 'segment_with_support_fallback', stale_plane)
    for index in range(3):
        clock[0] = 10.+.6*index
        feed()
    assert n._rgb_observation_interest['reason'] == 'observation_requires_same_frame_support_fit'
    assert not n._rgb_observation_interest['observation_interest_ready']
    assert n.tracker.anchor is None


@pytest.mark.parametrize('parameter', ['/unknown_perception', '/handeye'])
def test_in_place_cached_parameter_change_during_rgb_work_is_rejected(monkeypatch, parameter):
    n, _, feed, _, parameters = rgb_interest_node(monkeypatch)
    if parameter == '/handeye':
        parameters[parameter]['rotation'] = {'angles': [0., 0., 0.]}
    original = module.propose_rgb_observation_regions

    def mutate_cached_dict(*args, **kwargs):
        result = original(*args, **kwargs)
        if parameter == '/handeye':
            parameters[parameter]['rotation']['angles'][2] = .01
        else:
            parameters[parameter]['foreground_min_m'] = .002
        return result

    monkeypatch.setattr(module, 'propose_rgb_observation_regions', mutate_cached_dict)
    feed()
    assert n._rgb_observation_interest['reason'] == 'observation_source_context_changed'
    assert not n._rgb_observation_interest['observation_interest_ready']
    assert 'source_context' not in n._rgb_observation_interest
    assert 'interest_volume' not in n._rgb_observation_interest
    assert n.tracker.anchor is None


def test_source_context_has_no_mutable_cached_parameter_aliases(monkeypatch):
    n, _, _, _, parameters = rgb_interest_node(monkeypatch)
    parameters['/handeye'] = {'matrix': [[1., 0.], [0., 1.]]}
    parameters['/unknown_perception'] = {'nested': {'values': [1, 2]}}
    parameters['/camera']['runtime_profile'] = {'geometry_intrinsics': {'coeffs': [0., 0.]}}
    context = n._rgb_interest_source_context(module.rospy.Time.from_sec(9.9))
    frozen_id = module.digest(context)
    parameters['/handeye']['matrix'][0][1] = .2
    parameters['/unknown_perception']['nested']['values'].append(3)
    parameters['/camera']['runtime_profile']['geometry_intrinsics']['coeffs'][0] = .1
    parameters['/grasp_mode/selection']['generation'] += 1
    assert module.digest(context) == frozen_id
    assert module.digest(n._rgb_interest_source_context(module.rospy.Time.from_sec(9.9))) != frozen_id


def test_cached_parameter_mutation_during_third_frame_volume_retires_confirmation(monkeypatch):
    n, clock, feed, _, parameters = rgb_interest_node(monkeypatch)
    feed()
    clock[0] += .6
    feed()
    original = module.measured_support_coverage

    def changed(*args, **kwargs):
        coverage = original(*args, **kwargs)
        parameters['/unknown_perception']['foreground_min_m'] = .001
        return coverage

    monkeypatch.setattr(module, 'measured_support_coverage', changed)
    clock[0] += .6
    feed()
    assert n.config.foreground_min_m == .004
    assert n._rgb_observation_interest['reason'] == 'observation_source_context_changed'
    assert not n._rgb_observation_interest['observation_interest_ready']
    assert 'interest_volume' not in n._rgb_observation_interest


@pytest.mark.parametrize('failure', ['quality', 'small_core', 'fragmented_component', 'few_points'])
def test_partial_depth_initialization_does_not_erase_consistent_rgb_interest(monkeypatch, failure):
    from dataclasses import replace
    import json
    n, clock, feed, _, _ = rgb_interest_node(monkeypatch)
    if failure == 'quality':
        original = module.segment_with_support_fallback

        def low_quality(*args, **kwargs):
            result = original(*args, **kwargs)
            result.candidates = [replace(candidate, quality=.2) for candidate in result.candidates]
            return result

        monkeypatch.setattr(module, 'segment_with_support_fallback', low_quality)
    else:
        def incomplete_refinement(_rgb, _depth, _k, _transform, candidate, _cfg):
            if failure == 'small_core':
                raise ValueError('rgb_seed_core_too_small')
            if failure == 'fragmented_component':
                raise ValueError('rgb_seed_component_ambiguous_or_fragmented')
            return replace(candidate, metrics=dict(candidate.metrics, valid_points=66))

        monkeypatch.setattr(module, 'refine_candidate', incomplete_refinement)
    for index in range(3):
        clock[0] = 10.+.6*index
        feed(target_depth=.38)
        assert n.tracker.anchor is None
        assert not n.pubs['object'].publish.call_args[0][0].detected
        assert n._rgb_observation_interest['rgb_track']['consistent_frames'] == index+1
        assert n._rgb_observation_interest['observation_interest_ready'] is (index == 2)
        status = json.loads(n.pubs['detector_status'].publish.call_args[0][0].data)
        assert status['detail'] != 'no_center_instance'
    assert n.acquisition.samples == []
    assert n._rgb_observation_interest['motion_authorized'] is False


def test_geometry_confirmation_retires_weak_evidence_only_after_actual_anchor(monkeypatch):
    n, clock, feed, _, _ = rgb_interest_node(monkeypatch)
    for index in range(3):
        clock[0] = 10.+.6*index
        feed()
    assert n._rgb_observation_interest['observation_interest_ready']
    for index in range(3):
        clock[0] += .6
        feed(target_depth=.38)
        if index < 2:
            assert n.tracker.anchor is None
            assert not n.pubs['object'].publish.call_args[0][0].detected
            assert n._rgb_observation_interest['observation_interest_ready']
        else:
            assert n.tracker.anchor is not None
            assert n.pubs['object'].publish.call_args[0][0].detected
            assert not n._rgb_observation_interest['observation_interest_ready']
            assert n._rgb_observation_interest['reason'] == 'measured_target_available'


@pytest.mark.parametrize('failure', ['depth_ambiguity', 'support_failure'])
def test_initial_depth_ambiguity_or_invalid_support_retires_existing_rgb_interest(monkeypatch, failure):
    from dataclasses import replace
    import numpy as np
    n, clock, feed, _, _ = rgb_interest_node(monkeypatch)
    for index in range(3):
        clock[0] = 10.+.6*index
        feed()
    assert n._rgb_observation_interest['observation_interest_ready']
    original = module.segment_with_support_fallback

    def failed_depth(*args, **kwargs):
        if failure == 'support_failure':
            raise ValueError('support_plane_inlier_ratio_low')
        result = original(*args, **kwargs)
        assert len(result.candidates) == 1
        duplicate = replace(result.candidates[0], center_uv=result.candidates[0].center_uv+np.array([2., 0.]))
        result.candidates.append(duplicate)
        return result

    monkeypatch.setattr(module, 'segment_with_support_fallback', failed_depth)
    clock[0] += .6
    feed(target_depth=.38)
    assert not n._rgb_observation_interest['observation_interest_ready']
    assert 'interest_volume' not in n._rgb_observation_interest
    assert n.tracker.anchor is None
    assert not n.pubs['object'].publish.call_args[0][0].detected


def test_rgb_preservation_cannot_override_an_existing_target_anchor(monkeypatch):
    n, clock, feed, _, _ = rgb_interest_node(monkeypatch)
    for index in range(3):
        clock[0] = 10.+.6*index
        source = feed(target_depth=.38)
    assert n.tracker.anchor is not None
    n._rgb_observation_interest = dict(observation_interest_ready=True,
        source_stamp_ns=source[0].header.stamp.to_nsec(), generation=n.generation)
    n.unavailable('unconfirmed_depth', source[0].header.stamp, source,
                  lose=False, preserve_rgb_interest=True)
    assert not n._rgb_observation_interest['observation_interest_ready']
    assert n._rgb_observation_interest['reason'] == 'unconfirmed_depth'


def test_depth_fragment_failures_can_alternate_without_restarting_rgb_confirmation(monkeypatch):
    from dataclasses import replace
    n, clock, feed, _, _ = rgb_interest_node(monkeypatch)
    mode = ['none']
    original_segment = module.segment_with_support_fallback
    original_refine = module.refine_candidate

    def segment(*args, **kwargs):
        result = original_segment(*args, **kwargs)
        if mode[0] == 'quality':
            result.candidates = [replace(candidate, quality=.2) for candidate in result.candidates]
        return result

    def refine(rgb, depth, k, transform, candidate, cfg):
        if mode[0] == 'core':
            raise ValueError('rgb_seed_core_too_small')
        if mode[0] == 'component':
            raise ValueError('rgb_seed_component_ambiguous_or_fragmented')
        if mode[0] == 'points':
            return replace(candidate, metrics=dict(candidate.metrics, valid_points=66))
        return original_refine(rgb, depth, k, transform, candidate, cfg)

    monkeypatch.setattr(module, 'segment_with_support_fallback', segment)
    monkeypatch.setattr(module, 'refine_candidate', refine)
    for index, mode[0] in enumerate(('none', 'core', 'points', 'quality', 'component', 'none')):
        clock[0] = 10. + .6 * index
        feed(target_depth=0. if mode[0] == 'none' else .38)
        assert n.tracker.anchor is None
        assert n._rgb_observation_interest['rgb_track']['consistent_frames'] == min(index+1, 3)
        assert n._rgb_observation_interest['observation_interest_ready'] is (index >= 2)
        assert not n.pubs['object'].publish.call_args[0][0].detected


def test_no_failure_reason_implicitly_preserves_rgb_interest(monkeypatch):
    n, clock, feed, _, _ = rgb_interest_node(monkeypatch)
    for index in range(3):
        clock[0] = 10.+.6*index
        source = feed()
    assert n._rgb_observation_interest['observation_interest_ready']
    n.unavailable('no_center_instance', source[0].header.stamp, source, lose=False)
    assert not n._rgb_observation_interest['observation_interest_ready']
    assert 'interest_volume' not in n._rgb_observation_interest


def test_rgb_interest_expiring_before_unavailable_status_is_retired(monkeypatch):
    n, clock, feed, _, _ = rgb_interest_node(monkeypatch)
    for index in range(2):
        clock[0] = 10.+.6*index
        feed()
    original = n.unavailable

    def delayed(*args, **kwargs):
        clock[0] += .81
        return original(*args, **kwargs)

    monkeypatch.setattr(n, 'unavailable', delayed)
    clock[0] += .6
    feed()
    assert n._rgb_observation_interest['reason'] == 'source_exceeds_existing_frame_limit'
    assert not n._rgb_observation_interest['observation_interest_ready']
    assert 'interest_volume' not in n._rgb_observation_interest
