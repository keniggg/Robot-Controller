import importlib.util
import json
from dataclasses import asdict
from pathlib import Path
import threading
import types

import pytest
import numpy as np


SCRIPT = Path(__file__).resolve().parents[1] / 'scripts' / 'mode_aware_remote_grasp6d.py'
spec = importlib.util.spec_from_file_location('mode_remote_under_test', str(SCRIPT))
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
base = module.original


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


@pytest.fixture
def configured(monkeypatch):
    params = {
        module.SELECTION_PARAM: {'mode': 'carton', 'generation': 1, 'stamp_ns': '1000000000'},
        '/perception': {'detector': 'simple_hsv', 'yolo_model_choice': 'carton'},
    }
    monkeypatch.setattr(module.rospy, 'get_param', lambda name, default=None: params.get(name, default))
    node = module.ModeAwareRemoteGrasp6DNode.__new__(module.ModeAwareRemoteGrasp6DNode)
    node._initialize_mode_boundary()
    node._initialize_streaming_state(start_worker=False, source_clock=lambda: 10.0)
    node._object_lock = threading.Lock()
    node._geometry_state_lock = threading.RLock()
    node.enabled = True
    node.robot_execution_active = False
    node.frames = base.SynchronizedRgbdBuffer()
    node.planning_snapshot_max_age_sec = 0.35
    node.planning_snapshot_max_inference_latency_sec = 3.5
    node.latest_joint_state = None
    node.latest_object = None
    node.latest_object_time = None
    for name in ('plan_pub', 'rich_plan_pub', 'preview_plan_pub', 'preview_rich_plan_pub',
                 'status_pub', 'pipeline_metrics_pub', 'geometry_pub'):
        setattr(node, name, Publisher())
    assert node.reset_mode_cb(None).success
    return node, params


def select_unknown(params):
    params[module.SELECTION_PARAM] = {
        'mode': 'unknown', 'generation': 2, 'stamp_ns': '2000000000'}


@pytest.mark.parametrize('value', [None, {}, {'mode': 'other', 'generation': 1, 'stamp_ns': 1},
                                    {'mode': 'unknown', 'generation': True, 'stamp_ns': 1},
                                    {'mode': 'unknown', 'generation': 1, 'stamp_ns': 1.1}])
def test_selection_rejects_ambiguous_values(value):
    with pytest.raises(ValueError):
        module.parse_selection(value)


def test_unknown_requires_mask_and_keeps_carton_config_unchanged(configured):
    node, params = configured
    assert node._active_profile_requires_mask() is False
    select_unknown(params)
    result = node.reset_mode_cb(None)
    assert result.success, result.message
    assert json.loads(result.message)['generation'] == 2
    assert node._active_profile_requires_mask() is True
    assert node._last_model_choice == 'unknown_tabletop'
    assert params['/perception']['yolo_model_choice'] == 'carton'
    assert not node.streaming_enabled
    params[module.SELECTION_PARAM] = {'mode': 'carton', 'generation': 3, 'stamp_ns': '3000000000'}
    assert node.reset_mode_cb(None).success
    assert node._active_profile_requires_mask() is False
    assert node._last_model_choice == 'carton'


def test_reset_clears_frames_preview_geometry_and_target_identity(configured):
    node, params = configured
    old_frames = node.frames
    old_identity = node._current_stream_target_identity()
    node.latest_object = object()
    node._near_field_fused_surface = object()
    node._latest_target_observation = object()
    node._near_field_request_plan_cache = {'old': object()}
    node.latest_preview_rich_plan = object()
    select_unknown(params)
    assert node.reset_mode_cb(None).success
    assert node.frames is not old_frames
    assert node.frames._entry_retention_sec == pytest.approx(3.85)
    assert node._current_stream_target_identity() != old_identity
    assert node.latest_object is None
    assert node._near_field_fused_surface is None
    assert node._latest_target_observation is None
    assert node._near_field_request_plan_cache == {}
    assert node.latest_preview_rich_plan is None
    assert node.latest_rich_plan is None
    assert not node.rich_plan_pub.messages[-1].valid
    assert not node.preview_rich_plan_pub.messages[-1].valid
    assert not node.preview_plan_pub.messages[-1].poses
    assert node.last_submitted_stamp_ns == 2000000000


def test_rgb_unknown_masks_never_fill_missing_depth_and_carton_restores_original(configured):
    node, params = configured
    original_area = node.mask_internal_hole_max_area_px
    select_unknown(params)
    assert node.reset_mode_cb(None).success
    assert node.mask_internal_hole_max_area_px == 0
    params[module.SELECTION_PARAM] = {'mode':'carton', 'generation':3, 'stamp_ns':'3000000000'}
    assert node.reset_mode_cb(None).success
    assert node.mask_internal_hole_max_area_px == original_area


def test_source_cutoff_and_reset_readiness_gate_callbacks(configured, monkeypatch):
    node, params = configured
    received = []
    monkeypatch.setattr(base.RemoteGrasp6DNode, 'object_cb', lambda self, msg: received.append(msg))
    def message(ns):
        return types.SimpleNamespace(header=types.SimpleNamespace(stamp=module.rospy.Time(0, ns)))
    node.object_cb(message(999999999))
    node.object_cb(message(1000000000))
    assert received == []
    fresh = message(1000000001)
    node.object_cb(fresh)
    assert received == [fresh]
    node._mode_ready = False
    node.object_cb(message(1000000002))
    assert received == [fresh]


def test_parameter_commit_blocks_inference_until_matching_reset(configured):
    node, params = configured
    select_unknown(params)
    response = node.request_plan_cb(types.SimpleNamespace(trigger=True))
    assert not response.success
    assert not node.streaming_enabled
    assert node.reset_mode_cb(None).success
    assert node.request_plan_cb(types.SimpleNamespace(trigger=True)).success
    assert node.streaming_enabled


def test_active_execution_refuses_mode_reset_without_changing_enable_state(configured):
    node, params = configured
    old = dict(node._mode_selection)
    node.robot_execution_active = True
    select_unknown(params)
    result = node.reset_mode_cb(None)
    assert not result.success
    assert 'during execution' in result.message
    assert node._mode_selection == old
    assert node.robot_execution_active


def test_old_selection_and_same_generation_mutation_rejected(configured):
    node, params = configured
    params[module.SELECTION_PARAM]['mode'] = 'unknown'
    assert not node.reset_mode_cb(None).success
    select_unknown(params)
    assert node.reset_mode_cb(None).success
    params[module.SELECTION_PARAM] = {'mode': 'carton', 'generation': 1, 'stamp_ns': '3000000000'}
    assert not node.reset_mode_cb(None).success


def test_reset_cancels_old_inference_and_drains_late_cache_writes(configured, monkeypatch):
    node, params = configured
    assert node.start_streaming()
    ticket = types.SimpleNamespace(generation=node._stream_generation,
                                   target_epoch=node.target_instance_epoch)
    entered = threading.Event()
    release = threading.Event()
    epoch_changed = threading.Event()
    failures = []
    reset_results = []
    def delayed_prepare(self, old_ticket):
        entered.set()
        assert release.wait(3)
        # Simulate a late write before the original code's next ticket check.
        self._latest_target_observation = 'old result'
        self._require_stream_ticket_current(old_ticket)
    monkeypatch.setattr(base.RemoteGrasp6DNode, '_prepare_and_predict', delayed_prepare)
    original_advance = node._advance_target_instance_epoch
    def advance(*args, **kwargs):
        result = original_advance(*args, **kwargs)
        epoch_changed.set()
        return result
    node._advance_target_instance_epoch = advance
    def prepare():
        try:
            node._prepare_and_predict(ticket)
        except base.StreamResultCancelled as exc:
            failures.append(exc)
    worker = threading.Thread(target=prepare)
    worker.start()
    assert entered.wait(2)
    select_unknown(params)
    resetter = threading.Thread(target=lambda: reset_results.append(node.reset_mode_cb(None)))
    resetter.start()
    assert epoch_changed.wait(2)
    assert not node._mode_ready
    assert reset_results == []
    release.set()
    worker.join(3)
    resetter.join(3)
    assert not worker.is_alive() and not resetter.is_alive()
    assert len(failures) == 1
    assert reset_results[0].success, reset_results[0].message
    assert node._latest_target_observation is None
    assert node._mode_selection['generation'] == 2
    assert node._mode_ready


def test_delayed_latched_selection_cannot_revert_atomic_parameter(configured):
    node, params = configured
    previous = dict(node._mode_selection)
    select_unknown(params)
    assert node.reset_mode_cb(None).success
    node.selection_cb(types.SimpleNamespace(data=json.dumps(previous)))
    assert node._mode_selection['mode'] == 'unknown'
    assert node._mode_selection['generation'] == 2


def test_unknown_rejects_carton_objects_even_with_new_source_stamp(configured, monkeypatch):
    node, params = configured
    select_unknown(params)
    assert node.reset_mode_cb(None).success
    received = []
    monkeypatch.setattr(base.RemoteGrasp6DNode, 'object_cb', lambda self, msg: received.append(msg))
    msg = types.SimpleNamespace(header=types.SimpleNamespace(stamp=module.rospy.Time(3)),
                                detected=True, label='carton')
    node.object_cb(msg)
    assert received == []
    msg.label = 'unknown_small'
    node.object_cb(msg)
    assert received == [msg]


def test_snapshot_collection_does_not_hold_input_callback_lock(configured, monkeypatch):
    node, _ = configured
    admitted = threading.Event()
    msg = types.SimpleNamespace(header=types.SimpleNamespace(stamp=module.rospy.Time(3)))
    monkeypatch.setattr(base.RemoteGrasp6DNode, 'color_cb', lambda self, msg: admitted.set())
    def collect(self):
        callback = threading.Thread(target=lambda: self.color_cb(msg))
        callback.start()
        completed = admitted.wait(2)
        callback.join(2)
        assert completed
        return True
    monkeypatch.setattr(base.RemoteGrasp6DNode, '_poll_stream_snapshot', collect)
    assert node._poll_stream_snapshot()


def test_reset_reports_tombstone_publish_failure_as_not_ready(configured):
    node, params = configured
    def fail(_msg):
        raise RuntimeError('publisher unavailable')
    node.rich_plan_pub.publish = fail
    select_unknown(params)
    result = node.reset_mode_cb(None)
    assert not result.success
    assert 'publisher unavailable' in result.message
    assert not node._mode_ready


def test_old_preview_cannot_publish_after_reset_completes(configured):
    node, params = configured
    assert node.start_streaming()
    ticket = types.SimpleNamespace(generation=node._stream_generation,
                                   target_epoch=node.target_instance_epoch)
    select_unknown(params)
    assert node.reset_mode_cb(None).success
    count = len(node.preview_rich_plan_pub.messages)
    late = base.Grasp6DPlan()
    late.valid = True
    with pytest.raises(base.StreamResultCancelled):
        node._publish_preview_plan(late, base.PoseArray(), ticket=ticket)
    assert len(node.preview_rich_plan_pub.messages) == count
    assert not node.preview_rich_plan_pub.messages[-1].valid


def test_reset_releases_input_lock_while_draining_reached_view_callback(configured, monkeypatch):
    node, params = configured
    phase_entered = threading.Event()
    epoch_changed = threading.Event()
    callback_lock_available = threading.Event()
    results = []
    original_advance = node._advance_target_instance_epoch
    def advance(*args, **kwargs):
        result = original_advance(*args, **kwargs)
        epoch_changed.set()
        return result
    node._advance_target_instance_epoch = advance
    def reached_view(self, msg):
        phase_entered.set()
        assert epoch_changed.wait(2)
        # A pending camera callback must remain able to acquire its lock and
        # retire while reset waits for this reached-view callback to drain.
        assert self._mode_control_lock.acquire(timeout=2)
        try:
            assert not self._mode_ready
            callback_lock_available.set()
        finally:
            self._mode_control_lock.release()
    monkeypatch.setattr(base.RemoteGrasp6DNode, 'near_field_state_cb', reached_view)
    msg = types.SimpleNamespace(header=types.SimpleNamespace(stamp=module.rospy.Time(3)))
    phase = threading.Thread(target=lambda: node.near_field_state_cb(msg), daemon=True)
    phase.start()
    assert phase_entered.wait(2)
    select_unknown(params)
    resetter = threading.Thread(target=lambda: results.append(node.reset_mode_cb(None)), daemon=True)
    resetter.start()
    phase.join(3)
    resetter.join(3)
    assert callback_lock_available.is_set()
    assert not phase.is_alive() and not resetter.is_alive()
    assert results[0].success, results[0].message


def test_carton_frozen_input_config_is_returned_unchanged(configured, monkeypatch):
    node, _ = configured
    config = base.FrozenGraspNetInputConfig(
        mode=base.CONTEXT_ROI, context_margin_px=24, context_expand_ratio=0.25)
    monkeypatch.setattr(base.RemoteGrasp6DNode, '_freeze_graspnet_input_config', lambda self: config)
    assert node._freeze_graspnet_input_config() is config


def test_unknown_freezes_only_mode_local_crop_fields(configured):
    node, params = configured
    node.graspnet_input_mode = base.CONTEXT_ROI
    node.graspnet_input_context_margin_px = 24
    node.graspnet_input_context_expand_ratio = 0.25
    carton = node._freeze_graspnet_input_config()
    select_unknown(params)
    assert node.reset_mode_cb(None).success
    unknown = node._freeze_graspnet_input_config()
    expected = asdict(carton)
    expected.update(context_margin_px=12, context_expand_ratio=0.15)
    assert asdict(unknown) == expected
    params['/unknown_perception/graspnet_context_margin_px'] = 10
    params['/unknown_perception/graspnet_context_expand_ratio'] = 0.12
    override = node._freeze_graspnet_input_config()
    assert override.context_margin_px == 10
    assert override.context_expand_ratio == 0.12
    assert unknown.context_margin_px == 12
    assert unknown.context_expand_ratio == 0.15
    assert unknown.min_target_fraction == carton.min_target_fraction == 0.15
    assert unknown.min_target_points == carton.min_target_points == 120
    assert unknown.min_support_points == carton.min_support_points == 200
    assert unknown.min_total_points == carton.min_total_points == 320
    assert unknown.target_guard_px == carton.target_guard_px == 2
    assert unknown.candidate_target_gate_enabled is True
    assert params['/perception']['yolo_model_choice'] == 'carton'


def test_unknown_crop_builds_audited_input_without_relaxing_evidence_gates(configured):
    node, params = configured
    node.graspnet_input_mode = base.CONTEXT_ROI
    node.graspnet_input_context_margin_px = 24
    node.graspnet_input_context_expand_ratio = 0.25
    intrinsics = types.SimpleNamespace(width=160, height=120, fx=200., fy=200.,
                                       cx=80., cy=60., depth_scale=0.001)
    node._camera_intrinsics = lambda: intrinsics
    depth = np.full((120, 160), 800, dtype=np.uint16)
    mask = np.zeros((120, 160), dtype=np.uint8)
    mask[50:70, 70:90] = 255
    depth[mask > 0] = 750
    target = np.where(mask > 0, depth, 0).astype(np.uint16)
    snapshot = types.SimpleNamespace(source_mode='instance_mask', object_mask=mask,
                                     depth_raw=depth, bbox=(70, 50, 20, 20),
                                     color_bgr=np.zeros((120, 160, 3), dtype=np.uint8))
    def build(config):
        return node._build_frozen_graspnet_input(
            snapshot, target, config, commit_audit=False,
            support_plane_point_camera=np.asarray([0.8, 0., 0.]),
            support_plane_normal_camera=np.asarray([1., 0., 0.]))
    carton = node._freeze_graspnet_input_config()
    with pytest.raises(base.GraspNetInputContextError) as failure:
        build(carton)
    assert failure.value.code == 'TARGET_FRACTION_INSUFFICIENT'
    select_unknown(params)
    assert node.reset_mode_cb(None).success
    unknown = node._freeze_graspnet_input_config()
    _, audit = build(unknown)
    assert audit['context_margin_px'] == 12
    assert audit['context_expand_ratio'] == 0.15
    assert audit['frozen_config'] == asdict(unknown)
    assert audit['min_target_fraction'] == 0.15
    assert audit['target_fraction'] >= 0.15


def test_legacy_selection_defaults_to_two_stage_and_strategy_is_bound():
    value = module.parse_selection({'mode': 'carton', 'generation': 1, 'stamp_ns': '1'})
    assert value['strategy'] == 'two_stage'
    with pytest.raises(ValueError):
        module.parse_selection(dict(value, strategy='skip_checks'))


def test_direct_begins_fresh_contact_phase_without_reached_or_registration_claim(configured):
    node, params = configured
    select_unknown(params)
    params[module.SELECTION_PARAM]['strategy'] = 'direct'
    assert node.reset_mode_cb(None).success
    node._ros_source_clock_ns = lambda: 10_000_000_000
    node.near_field_direct_timeout_sec = 30.0
    assert node.start_streaming()
    assert node.near_field_planning_active
    assert node._configured_near_field_strategy() == 'single_snapshot_direct'
    assert node._near_field_phase_id > 0
    assert node._near_field_phase_started_sec == 10.0
    assert node._near_field_phase_deadline_sec == 40.0
    assert node.last_submitted_stamp_ns == 10_000_000_000
    assert node._near_field_reference_view is None
    assert node._near_field_reference_center_base is None
    assert node._latest_registration_evidence is None
    assert node._direct_near_field_submission_generation is None
    # An idempotent streaming request cannot renew the single-shot budget.
    node._ros_source_clock_ns = lambda: 20_000_000_000
    assert node.start_streaming() is False
    assert node._near_field_phase_deadline_sec == 40.0


def test_direct_phase_ignores_task_far_field_notification(configured, monkeypatch):
    node, params = configured
    select_unknown(params)
    params[module.SELECTION_PARAM]['strategy'] = 'direct'
    assert node.reset_mode_cb(None).success
    called = []
    monkeypatch.setattr(base.RemoteGrasp6DNode, 'near_field_state_cb', lambda self, msg: called.append(msg))
    node.near_field_state_cb(types.SimpleNamespace(
        header=types.SimpleNamespace(stamp=module.rospy.Time(3)), active=False))
    assert called == []


def test_strategy_mutation_requires_new_generation_and_reset(configured):
    node, params = configured
    params[module.SELECTION_PARAM]['strategy'] = 'direct'
    assert not node.request_plan_cb(types.SimpleNamespace(trigger=True)).success
    result = node.reset_mode_cb(None)
    assert not result.success
    assert 'without a new generation' in result.message


def test_direct_audit_identifies_current_view_contact_source(configured):
    node, params = configured
    select_unknown(params)
    params[module.SELECTION_PARAM]['strategy'] = 'direct'
    assert node.reset_mode_cb(None).success
    prepared = types.SimpleNamespace(
        snapshot=types.SimpleNamespace(stamp_ns=3000000000), near_field=True,
        ticket=types.SimpleNamespace(request_id=1, generation=1, target_epoch=1, snapshot_stamp_sec=3.0))
    report, summary, lineage = node._build_streaming_gate_audit_report(
        prepared, types.SimpleNamespace(selected=None), {}, 'NO_CANDIDATE', base_report={})
    assert report['execution_strategy'] == 'direct'
    assert report['planning_origin'] == 'current_view_contact_snapshot'
    assert report['observation_motion_performed'] is False
    assert report['snapshot_stamp_ns'] == 3000000000
    assert report['grasp_mode_selection'] == node._mode_selection
    assert summary['status'] == 'NO_CANDIDATE'
    assert lineage is None


def test_direct_stop_and_restart_requires_new_source_frames_and_ticket_generation(configured):
    node, params = configured
    select_unknown(params)
    params[module.SELECTION_PARAM]['strategy'] = 'direct'
    assert node.reset_mode_cb(None).success
    node._ros_source_clock_ns = lambda: 10_000_000_000
    assert node.start_streaming()
    prior_generation = node._stream_generation
    node._direct_near_field_submission_generation = prior_generation
    node._near_field_fused_surface = object()
    assert node.stop_streaming()
    assert not node.streaming_enabled
    assert not node.near_field_planning_active
    assert node._near_field_phase_id == 0
    assert node._near_field_phase_deadline_sec == 0.0
    assert node._near_field_fused_surface is None
    assert node._direct_near_field_submission_generation is None
    node._ros_source_clock_ns = lambda: 20_000_000_000
    assert node.start_streaming()
    assert node._stream_generation > prior_generation
    assert node.last_submitted_stamp_ns == 20_000_000_000
    assert node._near_field_phase_started_sec == 20.0


def test_direct_task_completion_retires_computation_phase_only(configured):
    node, params = configured
    select_unknown(params)
    params[module.SELECTION_PARAM]['strategy'] = 'direct'
    assert node.reset_mode_cb(None).success
    node._ros_source_clock_ns = lambda: 10_000_000_000
    assert node.start_streaming()
    node.grasp_state_cb(types.SimpleNamespace(active=True))
    assert node.streaming_enabled
    node.grasp_state_cb(types.SimpleNamespace(active=False))
    assert not node.streaming_enabled
    assert not node.near_field_planning_active
    assert node._near_field_phase_deadline_sec == 0.0


def test_direct_to_two_stage_reset_removes_contact_phase_and_delegates(configured, monkeypatch):
    node, params = configured
    select_unknown(params)
    params[module.SELECTION_PARAM]['strategy'] = 'direct'
    assert node.reset_mode_cb(None).success
    node._ros_source_clock_ns = lambda: 10_000_000_000
    assert node.start_streaming()
    params[module.SELECTION_PARAM] = dict(params[module.SELECTION_PARAM], strategy='two_stage',
                                         generation=3, stamp_ns='11000000000')
    assert node.reset_mode_cb(None).success
    assert not node.near_field_planning_active
    assert node._near_field_phase_deadline_sec == 0.0
    assert node._near_field_reference_view is None
    calls = []
    monkeypatch.setattr(base.RemoteGrasp6DNode, 'near_field_state_cb', lambda self, msg: calls.append(msg))
    msg = types.SimpleNamespace(header=types.SimpleNamespace(stamp=module.rospy.Time(12)), active=True)
    node.near_field_state_cb(msg)
    assert calls == [msg]


def test_two_stage_task_completion_does_not_change_original_streaming_behavior(configured):
    node, _ = configured
    assert node.start_streaming()
    node.grasp_state_cb(types.SimpleNamespace(active=True))
    node.grasp_state_cb(types.SimpleNamespace(active=False))
    assert node.streaming_enabled


def direct_buffer_node(configured):
    node, params = configured
    clock = [10_000_000_000]
    node._ros_source_clock_ns = lambda: clock[0]
    node._safe_time = lambda value=None: (value if value is not None else
                                         module.rospy.Time(clock[0] // 10**9, clock[0] % 10**9))
    node._initialize_streaming_state(start_worker=False, source_clock=lambda: clock[0] / 1e9)
    select_unknown(params)
    params[module.SELECTION_PARAM]['strategy'] = 'direct'
    assert node.reset_mode_cb(None).success
    node.near_field_direct_timeout_sec = 30.0
    node.planning_snapshot_frames = 5
    node.near_field_planning_snapshot_frames = 3
    node.planning_snapshot_timeout_sec = 0.0
    node.planning_snapshot_max_span_sec = 12.0
    node.planning_mask_min_iou = 0.85
    node.planning_mask_max_centroid_shift_px = 5.0
    node.planning_max_joint_delta_rad = 0.01
    node.mask_erosion_px = 2
    node.depth_mad_scale = 3.5
    node.depth_mad_absolute_floor_m = 0.002
    node.mask_internal_hole_max_area_px = 25
    node.rate_hz = 3.0
    assert node.start_streaming()
    clock[0] = 10_700_000_000
    return node, clock


def fill_real_buffer(node, masks, stamps=None, joints=None):
    node.frames.update_joints(np.zeros(6) if joints is None else joints)
    stamps = stamps or [10_100_000_000, 10_200_000_000, 10_300_000_000]
    for stamp_ns, mask in zip(stamps, masks):
        stamp = module.rospy.Time(stamp_ns // 10**9, stamp_ns % 10**9)
        depth = np.full(mask.shape, 600, dtype=np.uint16)
        color = np.zeros((*mask.shape, 3), dtype=np.uint8)
        msg = types.SimpleNamespace(detected=True, label='unknown_small', bbox_x=20, bbox_y=20,
                                    bbox_width=20, bbox_height=20,
                                    header=types.SimpleNamespace(stamp=stamp, frame_id='camera_link'))
        node.frames.update_color(color, stamp, 'camera_link')
        node.frames.update_depth(depth, stamp, 'camera_link')
        node.frames.update_mask(mask, stamp, 'camera_link')
        node.frames.update_object(msg, stamp, node.target_instance_epoch,
                                  node._current_stream_target_identity())


def test_direct_real_buffer_fuser_submits_fresh_three_frame_snapshot_without_robot_active(configured):
    node, _clock = direct_buffer_node(configured)
    mask = np.zeros((60, 80), dtype=np.uint8)
    mask[20:40, 20:40] = 255
    fill_real_buffer(node, [mask] * 3)
    assert not node.robot_execution_active
    assert node._poll_stream_snapshot()
    ticket = node._stream_worker_ticket
    snapshot, config = ticket.payload
    assert snapshot.ok
    assert snapshot.quality.fused_frames == 3
    assert snapshot.sample_stamp_ns == (10100000000, 10200000000, 10300000000)
    assert snapshot.target_identity == node._current_stream_target_identity()
    assert snapshot.source_mode == 'instance_mask'
    assert config.candidate_target_gate_enabled
    assert node._direct_near_field_submission_generation == node._stream_generation
    assert node._poll_stream_snapshot() is False


def test_direct_real_buffer_unstable_masks_report_original_gate_and_terminal_once(configured):
    node, clock = direct_buffer_node(configured)
    masks = []
    for offset in (0, 3, 0):
        mask = np.zeros((60, 80), dtype=np.uint8)
        mask[20:40, 20 + offset:40 + offset] = 255
        masks.append(mask)
    fill_real_buffer(node, masks)
    assert node._poll_stream_snapshot() is False
    assert node._stream_worker_ticket is None
    last = node._direct_snapshot_last_rejection
    assert last['code'] == 'DEPTH_UNSTABLE'
    assert 'mask IoU' in last['reason'] and '0.850' in last['reason']
    deadline = node._near_field_phase_deadline_sec
    clock[0] = int(deadline * 1e9)
    assert node._poll_stream_snapshot() is False
    terminal = node.preview_rich_plan_pub.messages[-1]
    assert not terminal.valid
    assert terminal.candidate_source == 'near_field_terminal'
    assert terminal.diagnostic.startswith('NEAR_FIELD_DIRECT_TIMEOUT:')
    assert 'mask IoU' in terminal.diagnostic
    assert node._near_field_phase_deadline_sec == deadline
    count = len(node.preview_rich_plan_pub.messages)
    node._poll_stream_snapshot()
    assert len(node.preview_rich_plan_pub.messages) == count


def test_direct_real_buffer_rejects_prephase_frames_and_reports_missing_count(configured):
    node, _clock = direct_buffer_node(configured)
    mask = np.zeros((60, 80), dtype=np.uint8)
    mask[20:40, 20:40] = 255
    fill_real_buffer(node, [mask] * 3, stamps=[9_100_000_000, 9_200_000_000, 9_300_000_000])
    assert node._poll_stream_snapshot() is False
    assert node._stream_worker_ticket is None
    assert node._direct_snapshot_last_rejection['code'] == 'SNAPSHOT_INCOMPLETE'
    assert 'need 3 exact fresh frames' in node._direct_snapshot_last_rejection['reason']


def direct_observation(node, ns=11_000_000_000):
    points = np.array([(x,y,.02) for x in np.linspace(-.02,.02,15)
                       for y in np.linspace(-.01,.01,10)])
    return base.TargetObservation(identity=node._current_stream_target_identity(),
        stamp_ns=ns, frame_id='base_link', points_base=points,
        support_normal_base=(0,0,1), support_offset_m=0,
        bbox_xywh=(280,220,60,30), image_shape_hw=(480,640),
        edge_clearance_px=220, source_kind='instance_mask', source_label='unknown_small')


def test_direct_current_surface_retains_only_real_single_view_points(configured):
    node, params = configured
    select_unknown(params)
    params[module.SELECTION_PARAM]['strategy'] = 'direct'
    assert node.reset_mode_cb(None).success
    node._ros_source_clock_ns = lambda: 10_000_000_000
    node.near_field_direct_timeout_sec = 30.
    node.start_streaming()
    observation = direct_observation(node)
    node._latest_target_observation = observation
    node._register_near_field_surface(observation)
    surface, reference = node._snapshot_multiview_surface()
    assert surface is not None and reference is not None
    assert surface.view_stamps_ns == (observation.stamp_ns,)
    assert set(map(tuple, surface.points_base)).issubset(set(map(tuple, observation.points_base)))
    assert set(surface.view_indices) == {0}
    assert node._latest_registration_evidence is None
    audit = node._multiview_surface_audit()
    assert audit['active'] and audit['evidence_kind'] == 'current_view_only'
    assert audit['cross_view_registration_performed'] is False
    node.stop_streaming()
    assert node._active_multiview_surface() == (None, None)


def test_direct_surface_rejects_before_phase_and_old_target(configured):
    node, params = configured
    select_unknown(params)
    params[module.SELECTION_PARAM]['strategy'] = 'direct'
    assert node.reset_mode_cb(None).success
    node._ros_source_clock_ns = lambda: 10_000_000_000
    node.near_field_direct_timeout_sec = 30.
    node.start_streaming()
    with pytest.raises(ValueError, match='current phase and target'):
        node._register_near_field_surface(direct_observation(node, 9_000_000_000))
    old = direct_observation(node)
    node._advance_target_instance_epoch('TEST_NEW_TARGET', preserve_identity=False)
    with pytest.raises(ValueError, match='current phase and target'):
        node._register_near_field_surface(old)
    assert node._active_multiview_surface() == (None, None)


def test_unknown_compares_real_motion_between_equally_informative_views(configured, monkeypatch):
    node, params = configured
    select_unknown(params)
    assert node.reset_mode_cb(None).success
    node._stable_variant_runtime = {}
    def candidate(index, translation, duration, turn, deficit=0.):
        node._stable_variant_runtime[(index,0)] = {'soft_evidence':{
            'observation_translation_delta_m':translation,
            'observation_side_evidence_deficit_m':deficit,
            'observation_projected_side_evidence_m':.003}}
        return types.SimpleNamespace(track_id=index,variant_index=0,pre_moveit_score=0.,
            moveit_result=types.SimpleNamespace(reason=str(duration),joint_max_delta_rad=turn,joint_path_cost=turn))
    monkeypatch.setattr(node,'_parse_plan_metrics',lambda text:{'observation_screened_duration_sec':float(text)})
    short_translation = candidate(1,.01,60,3.1)
    short_motion = candidate(2,.04,25,.7)
    insufficient_view = candidate(3,.005,5,.1,.002)
    assert node._select_far_field_observation([short_translation,short_motion,insufficient_view]) is short_motion
    assert node._select_far_field_observation([]) is None
    node._mode_selection['mode'] = 'carton'
    assert node._select_far_field_observation([short_translation,short_motion,insufficient_view]) is short_translation


@pytest.mark.parametrize('delta,admitted', [(.7,True), (np.pi/2,True), (np.pi,False)])
def test_unknown_rejects_large_observation_turn_without_changing_carton(configured, monkeypatch, delta, admitted):
    node, params = configured
    select_unknown(params)
    assert node.reset_mode_cb(None).success
    checked = base.MoveItResult(reachable=True, joint_path_cost=delta,
                               joint_max_delta_rad=delta, reason='strict checked path')
    monkeypatch.setattr(base.RemoteGrasp6DNode, '_observation_following_support_evaluation',
                        lambda *a: (checked, {'ok':True, 'screened_duration_sec':20.}))
    result, report = node._observation_following_support_evaluation(object(), checked, object())
    assert result.reachable is admitted
    assert report['ok'] is admitted
    if not admitted:
        assert result.failure_code == 'UNKNOWN_OBSERVATION_TURN_LIMIT'
    node._mode_selection['mode'] = 'carton'
    result, report = node._observation_following_support_evaluation(object(), checked, object())
    assert result is checked and report['ok']


def test_unknown_observation_preferences_refresh_without_global_parameter_write(configured, monkeypatch):
    node, params = configured
    select_unknown(params)
    params[module.SELECTION_PARAM]['strategy'] = 'two_stage'
    assert node.reset_mode_cb(None).success
    def staged(original_node):
        original_node.grasp_config = {'observation_camera_target_max_distance_m':.22, 'sentinel':True}
        return 'unchanged_continuous_config'
    monkeypatch.setattr(base.RemoteGrasp6DNode, '_stage_runtime_params', staticmethod(staged))
    assert node._stage_runtime_params(node) == 'unchanged_continuous_config'
    assert node.grasp_config['observation_camera_target_max_distance_m'] == .30
    node._mode_selection['mode'] = 'carton'
    node._stage_runtime_params(node)
    assert node.grasp_config == {'observation_camera_target_max_distance_m':.22, 'sentinel':True}


def test_unknown_reduces_only_observation_seed_work_without_mutating_contact_config(configured, monkeypatch):
    from alicia_flexible_grasp.grasp.tabletop_geometry_candidates import TabletopGeometryConfig
    node, params = configured
    select_unknown(params)
    assert node.reset_mode_cb(None).success
    config = TabletopGeometryConfig(max_candidates=32)
    node.tabletop_geometry_config = config
    calls = []
    def generate(staged, geometry, snapshot, contact, label):
        calls.append((staged, staged.tabletop_geometry_config.max_candidates, contact))
        return ('candidate',), {'materialized_count':1}
    monkeypatch.setattr(base.RemoteGrasp6DNode, '_generate_tabletop_candidates', generate)
    result, audit = node._generate_tabletop_candidates(object(), contact_execution_phase=False)
    assert result == ('candidate',) and audit['unknown_observation_seed_limit'] == 8
    assert calls[-1][0] is not node and calls[-1][1:] == (8,False)
    assert node.tabletop_geometry_config is config
    node._generate_tabletop_candidates(object(), contact_execution_phase=True)
    assert calls[-1][0] is node and calls[-1][1:] == (32,True)
    node._mode_selection['mode'] = 'carton'
    node._generate_tabletop_candidates(object(), contact_execution_phase=False)
    assert calls[-1][0] is node and calls[-1][1:] == (32,False)


def test_direct_final_geometry_evidence_preserves_original_execution_schema(configured, monkeypatch):
    from copy import deepcopy
    node, params = configured
    select_unknown(params)
    params[module.SELECTION_PARAM]['strategy'] = 'direct'
    assert node.reset_mode_cb(None).success
    ticket = types.SimpleNamespace(request_id=7,snapshot_stamp_sec=3.)
    row = dict(candidate_source='tabletop_geometry',source_index=4,variant_index=0,
        selected=True,final_score=.2,
        tracking=dict(track_id=8,hit_count=1,window_count=1,hit_request_ids=[7]),
        lineage_binding=dict(source_request_id=7,source_snapshot_stamp_sec=3.,
            source_raw_candidate_index=4,source_variant_index=0,evaluation_request_id=7,
            evaluation_snapshot_stamp_sec=3.,evaluation_variant_index=0),
        moveit=dict(reachable=True,collision_free=True,within_joint_limits=True,
            ik_valid=True,planning_success=True,failure_code='',evidence_code='',
            joint_path_cost=.2,joint_max_delta_rad=.1))
    report = dict(outcome={'valid_plan':True},plan_id='actual-plan-test',selected=deepcopy(row),
                  stable_evaluations=[deepcopy(row)],lineage=[deepcopy(row)],rows=[deepcopy(row)])
    assert base.RemoteGrasp6DNode._continuous_execution_schema_error(report,ticket) == ''
    monkeypatch.setattr(base.RemoteGrasp6DNode,'_build_streaming_gate_audit_report',
        lambda *a,**kw:(deepcopy(report),{},deepcopy(row)))
    gate = {'ok':True,'required_open_width_m':.025}
    node._stable_variant_runtime={(8,0):{'final_registered_geometry_gate':gate}}
    prepared=types.SimpleNamespace(snapshot=types.SimpleNamespace(stamp_ns=3_000_000_000))
    result,_,lineage=node._build_streaming_gate_audit_report(prepared,
        types.SimpleNamespace(selected=types.SimpleNamespace(track_id=8,variant_index=0)))
    assert base.RemoteGrasp6DNode._continuous_execution_schema_error(result,ticket) == ''
    assert result['selected']['final_geometry_gate'] == gate
    assert result['selected'] == result['stable_evaluations'][0] == result['lineage'][0] == lineage
    result['lineage'][0]['final_geometry_gate']['ok']=False
    assert base.RemoteGrasp6DNode._continuous_execution_schema_error(result,ticket)


@pytest.mark.parametrize('contact_ok', [False, True])
def test_unknown_direct_search_rejects_failed_simulation_before_publication(configured, monkeypatch, contact_ok):
    from dataclasses import dataclass, field, fields
    node, params = configured
    select_unknown(params)
    params[module.SELECTION_PARAM]['strategy']='direct'
    assert node.reset_mode_cb(None).success
    node._near_field_phase_id=1
    node._require_stream_ticket_current=lambda ticket:None
    node.mujoco_config={'enabled':True,'execution_gate_enabled':True,'min_score':80}
    node.latest_joint_state=types.SimpleNamespace(name=['Joint1'],position=[0.])
    @dataclass
    class Candidate:
        track_id:int=1
        variant_index:int=0
        moveit_result:object=None
        final_score:object=None
        soft_features:object=field(default_factory=lambda:base.SoftCandidateFeatures(**{
            f.name:0. for f in fields(base.SoftCandidateFeatures)}))
        score_weights:object=field(default_factory=base.SoftScoreWeights)
    candidate=Candidate()
    result=base.MoveItResult(reachable=True,joint_path_cost=.2,joint_max_delta_rad=.1,reason='checked')
    monkeypatch.setattr(base.RemoteGrasp6DNode,'_check_direct_registered_candidate',lambda self,c:result)
    node._stable_variant_runtime={(1,0):{'prepared':types.SimpleNamespace(ticket=object())}}
    plan=types.SimpleNamespace(plan_id='plan-one',candidate_source='tabletop_geometry',candidate_source_lineage=['tabletop_geometry'])
    def build(c):
        assert isinstance(c.final_score, float) and np.isfinite(c.final_score)
        assert c.moveit_result is result
        return {'rich_plan':plan}
    node._build_selected_preview_bundle=build
    monkeypatch.setattr(base,'build_mujoco_payload',lambda *a:{'plan_id':plan.plan_id})
    response=dict(plan_id=plan.plan_id,candidate_source=plan.candidate_source,
        candidate_source_lineage=plan.candidate_source_lineage,simulation_ok=contact_ok,
        ik_success=True,collision_free=True,contact_success=contact_ok,lift_success=contact_ok,
        score=100 if contact_ok else 55,
        lift_evidence=dict(contract_version=1,object_lift_m=.03,commanded_lift_m=.03,minimum_lift_m=.02,
            two_sided_lift_samples=10,lost_contact_samples=0,lift_sample_count=10,
            max_lost_contact_streak=0,contact_loss_grace_samples=0))
    calls=[]
    node._mujoco_client_factory=lambda *a,**kw:types.SimpleNamespace(simulate_grasp=lambda payload:(calls.append(payload) or response))
    observed=node._check_direct_registered_candidate(candidate)
    assert observed.reachable is contact_ok
    assert len(calls)==1
    audit=node._stable_variant_runtime[(1,0)]['mujoco_selection']
    assert audit['passed'] is contact_ok
    assert audit['plan_id']=='plan-one'
    if not contact_ok:assert 'MUJOCO_CONTACT_FAILED' in observed.reason
    node.mujoco_selection_max_candidates=1
    assert not node._check_direct_registered_candidate(candidate).reachable
    assert len(calls)==1


def test_direct_retired_request_still_emits_terminal_at_phase_deadline(configured):
    node,clock=direct_buffer_node(configured)
    node._direct_near_field_submission_generation=node._stream_generation
    node._stream_worker_busy=False
    clock[0]=int(node._near_field_phase_deadline_sec*1e9)
    node._poll_stream_snapshot()
    terminal=node.preview_rich_plan_pub.messages[-1]
    assert terminal.candidate_source=='near_field_terminal'
    assert 'NEAR_FIELD_DIRECT_TIMEOUT' in terminal.diagnostic


@pytest.mark.parametrize('busy,ready,active',[(True,False,False),(False,True,False),(False,False,True)])
def test_direct_deadline_does_not_replace_inflight_work_or_executable_plan(configured,busy,ready,active):
    node,clock=direct_buffer_node(configured)
    node._direct_near_field_submission_generation=node._stream_generation
    node._stream_worker_busy=busy
    node.robot_execution_active=active
    node.latest_preview_rich_plan=types.SimpleNamespace(valid=ready)
    clock[0]=int(node._near_field_phase_deadline_sec*1e9)
    count=len(node.preview_rich_plan_pub.messages)
    node._poll_stream_snapshot()
    assert len(node.preview_rich_plan_pub.messages)==count


def test_unknown_requires_visible_pregrasp_and_restores_carton_policy(configured):
    node, params = configured
    original_policy = dict(node._carton_visibility_settings)
    select_unknown(params)
    assert node.reset_mode_cb(None).success
    assert node.camera_visibility_gate_enabled is True
    assert node.camera_visibility_min_depth_m >= .070
    assert node.camera_visibility_max_depth_m <= .500
    params[module.SELECTION_PARAM] = {
        'mode': 'carton', 'generation': 3, 'stamp_ns': '3000000000'}
    assert node.reset_mode_cb(None).success
    assert all(getattr(node, key) == value for key, value in original_policy.items())


@pytest.mark.parametrize('case,expected', [
    ('out_of_frame_sequence', False), ('visible_sequence', True)])
def test_real_gen9_pregrasp_view_is_checked_after_pose_resolution(configured, monkeypatch, case, expected):
    from geometry_msgs.msg import PoseStamped
    from dataclasses import dataclass
    fixture = json.loads((Path(__file__).parent / 'fixtures' /
                         'unknown_pregrasp_view_20260923.json').read_text())
    node, params = configured
    select_unknown(params)
    assert node.reset_mode_cb(None).success
    transform = base.quaternion_matrix(fixture['tool_from_camera_quaternion'])
    transform[:3, 3] = fixture['tool_from_camera_translation']
    node._tool_from_camera_matrix = lambda: transform
    node._camera_intrinsics = lambda: types.SimpleNamespace(**fixture['camera'])
    node._planning_object_snapshot = lambda: (None, None)
    node.camera_visibility_require_approach = False
    stages = {}
    for item in fixture[case]:
        pose = PoseStamped()
        for key, value in zip('xyz', item['position_m']):
            setattr(pose.pose.position, key, value)
        for key, value in zip('xyzw', item['quaternion_xyzw']):
            setattr(pose.pose.orientation, key, value)
        stages[item['stage']] = pose
    sequence = types.SimpleNamespace(**stages)
    runtime = {'prepared': types.SimpleNamespace(
        geometry=types.SimpleNamespace(center_base=np.array(fixture['target_base'])))}

    @dataclass
    class GeometryGate:
        ok: bool = True
        failure_code: str = ''
        failure_reason: str = ''
        failed_gate: str = ''

    geometry_gate = GeometryGate()
    monkeypatch.setattr(base.RemoteGrasp6DNode, '_resolved_sequence_geometry_gate',
                        lambda *args: geometry_gate)
    result = node._resolved_sequence_geometry_gate(runtime, sequence)
    assert result.ok is expected
    assert runtime['unknown_pregrasp_visibility']['ok'] is expected
    if not expected:
        assert result.failure_code == 'UNKNOWN_PREGRASP_NOT_VISIBLE'
        assert runtime['unknown_pregrasp_visibility']['metrics'][0]['v'] > 480
    # Registration must be checked in its corrected coordinates, not the
    # original in-frame center. This tests the final-resolution boundary.
    correction = np.eye(4)
    correction[:3, 3] = [1., 0., 0.]
    runtime['contact_registration_transform'] = correction
    assert not node._resolved_sequence_geometry_gate(runtime, sequence).ok
    node._mode_selection = dict(node._mode_selection, mode='carton')
    assert node._resolved_sequence_geometry_gate(runtime, sequence) is geometry_gate


@pytest.mark.parametrize('mode,enabled,minimum,maximum', [
    ('unknown', True, .070, .500), ('carton', False, .01, .9)])
def test_runtime_refresh_preserves_unknown_view_contract(monkeypatch, mode, enabled, minimum, maximum):
    node = types.SimpleNamespace(_mode_selection={'mode': mode}, grasp_config={})
    def refresh(staged):
        staged.camera_visibility_gate_enabled = False
        staged.camera_visibility_diagnostic_enabled = False
        staged.camera_visibility_min_depth_m = .01
        staged.camera_visibility_max_depth_m = .9
        return 'refreshed'
    monkeypatch.setattr(base.RemoteGrasp6DNode, '_stage_runtime_params', refresh)
    assert module.ModeAwareRemoteGrasp6DNode._stage_runtime_params(node) == 'refreshed'
    assert node.camera_visibility_gate_enabled is enabled
    assert node.camera_visibility_min_depth_m == minimum
    assert node.camera_visibility_max_depth_m == maximum


def real_pregrasp_fixture_node(configured):
    from geometry_msgs.msg import PoseStamped
    fixture = json.loads((Path(__file__).parent / 'fixtures' /
                         'unknown_pregrasp_view_20260923.json').read_text())
    node, params = configured
    select_unknown(params)
    assert node.reset_mode_cb(None).success
    node.near_field_planning_active = True
    transform = base.quaternion_matrix(fixture['tool_from_camera_quaternion'])
    transform[:3, 3] = fixture['tool_from_camera_translation']
    node._tool_from_camera_matrix = lambda: transform
    node._camera_intrinsics = lambda: types.SimpleNamespace(**fixture['camera'])
    node._planning_object_snapshot = lambda: (None, None)
    node.camera_visibility_require_approach = False
    # The real next attempt required a 54 px object-footprint margin.
    node.camera_visibility_margin_px = 54
    stages = {}
    for item in fixture['out_of_frame_sequence']:
        pose = PoseStamped()
        for key, value in zip('xyz', item['position_m']):
            setattr(pose.pose.position, key, value)
        for key, value in zip('xyzw', item['quaternion_xyzw']):
            setattr(pose.pose.orientation, key, value)
        stages[item['stage']] = pose
    return node, base.Grasp6DSequence(**stages), fixture


def test_actual_out_of_frame_pose_uses_bounded_planar_offset_without_changing_contact(configured):
    from copy import deepcopy
    node, sequence, fixture = real_pregrasp_fixture_node(configured)
    before = deepcopy(sequence)
    target = np.asarray(fixture['target_base'])
    assert not node._candidate_visibility_metrics(sequence.grasp, target, sequence)[0]
    resolved = node._visible_unknown_pregrasp(sequence, types.SimpleNamespace(center_base=target,
        support_normal_base=np.array(fixture["geometry"]["support_normal_base"])))
    assert node._candidate_visibility_metrics(resolved.grasp, target, resolved)[0]
    assert sequence == before
    for stage in ['pregrasp', 'approach', 'grasp', 'lift']:
        assert getattr(resolved, stage).pose.orientation == getattr(sequence, stage).pose.orientation
    for stage in ['approach', 'grasp', 'lift']:
        assert getattr(resolved, stage) == getattr(sequence, stage)
    offset = np.array(resolved.unknown_pregrasp_view['offset_base_m'])
    assert 0.0 < np.linalg.norm(offset) <= .040
    assert abs(np.dot(offset, fixture['geometry']['support_normal_base'])) < 1e-12
    separation = base.pose_matrix(resolved.pregrasp)[:3, 3] - base.pose_matrix(resolved.grasp)[:3, 3]
    assert np.linalg.norm(separation) <= node._effective_adaptive_stage_limits().pregrasp_max_m
    a, b = base.pose_matrix(sequence.pregrasp), base.pose_matrix(resolved.pregrasp)
    assert np.allclose(a[:3, 1], b[:3, 1])  # jaw closing axis is unchanged
    g = fixture['geometry']
    gripper = base.GripperGeometry(.050, .002,
        np.array([.0434, .0286, .0600]), np.array([.1175, .1550, .0774]), .003)
    envelope = base.evaluate_open_gripper_observation_envelope(
        gripper=gripper, T_base_tool0=b, opening_width_m=.050,
        support_normal_base=g['support_normal_base'], support_offset_m=g['support_offset_m'],
        obb_center_base=g['obb_center_base_m'], R_base_obb=g['R_base_obb'],
        obb_size_xyz_m=g['obb_size_xyz_m'])
    assert envelope.ok, envelope
    audit = node._execution_sequence_audit(resolved, True)
    assert audit['pregrasp_view_adjustment'] == resolved.unknown_pregrasp_view
    assert audit['stages'][0]['position_m'] == [resolved.pregrasp.pose.position.x,
        resolved.pregrasp.pose.position.y, resolved.pregrasp.pose.position.z]
    assert node._visible_unknown_pregrasp(resolved, types.SimpleNamespace(center_base=target,
        support_normal_base=np.array(fixture["geometry"]["support_normal_base"]))) is resolved


def test_pregrasp_view_offset_cannot_relax_range_or_modify_carton(configured):
    node, sequence, fixture = real_pregrasp_fixture_node(configured)
    geometry = types.SimpleNamespace(center_base=np.asarray(fixture['target_base']),
        support_normal_base=np.array(fixture['geometry']['support_normal_base']))
    node.camera_visibility_max_depth_m = .10
    assert node._visible_unknown_pregrasp(sequence, geometry) is sequence
    assert not node._candidate_visibility_metrics(sequence.grasp, geometry.center_base, sequence)[0]
    node._mode_selection = dict(node._mode_selection, mode='carton')
    assert node._visible_unknown_pregrasp(sequence, geometry) is sequence
    node._mode_selection = dict(node._mode_selection, mode='unknown')
    node.near_field_planning_active = False
    assert node._visible_unknown_pregrasp(sequence, geometry) is sequence


@pytest.mark.parametrize('mode,turn,accepted', [
    ('unknown', 3.03543331164935, False), ('unknown', .8, True),
    ('unknown', float('nan'), False), ('carton', 3.03543331164935, True)])
def test_final_actual_large_wrist_turn_is_rejected_before_publication(configured, monkeypatch, mode, turn, accepted):
    node, _ = configured
    node._mode_selection = dict(node._mode_selection, mode=mode)
    result = types.SimpleNamespace(joint_max_delta_rad=turn)
    response = (object(), result, {})
    monkeypatch.setattr(base.RemoteGrasp6DNode, '_strict_check_final_near_field_plan',
                        lambda *args: response)
    if accepted:
        assert node._strict_check_final_near_field_plan(None, None, {}) == response
    else:
        with pytest.raises(base.CandidateContractError, match='joint turn'):
            node._strict_check_final_near_field_plan(None, None, {})
