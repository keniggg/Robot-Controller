#!/usr/bin/env python3
import importlib.util
import json
import math
import pathlib
import sys
import threading
import time
import types

import pytest


ROOT = pathlib.Path(__file__).resolve().parents[1]
for path in (ROOT, ROOT / 'src'):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))


SCRIPT = ROOT / 'scripts' / 'remote_grasp6d_node.py'
spec = importlib.util.spec_from_file_location(
    'remote_grasp6d_streaming_node', str(SCRIPT)
)
remote_node = importlib.util.module_from_spec(spec)
spec.loader.exec_module(remote_node)

from alicia_flexible_grasp.grasp.grasp6d_pipeline import (  # noqa: E402
    SoftCandidateFeatures,
    SoftScoreWeights,
    soft_candidate_cost,
)
from alicia_flexible_grasp.grasp.grasp6d_stability import (  # noqa: E402
    CandidateObservation,
    CandidateTracker,
    StableCandidate,
    TrackingConfig,
)
from alicia_flexible_grasp.grasp.gripper_geometry import (  # noqa: E402
    CandidateGateResult,
)
from alicia_flexible_grasp.vision.remote_grasp6d_client import (  # noqa: E402
    RemoteGraspCandidate,
)
from alicia_flexible_grasp.vision.latest_only_inference import (  # noqa: E402
    InferenceTicket,
)


class MutableClock:
    def __init__(self, value):
        self.value = float(value)

    def __call__(self):
        return self.value


class RecordingPublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class BlockingPrediction:
    def __init__(self):
        self.entered = threading.Event()
        self.release = threading.Event()

    def __call__(self, ticket):
        self.entered.set()
        if not self.release.wait(2.0):
            raise RuntimeError('test did not release blocked prediction')
        return types.SimpleNamespace(
            ticket=ticket,
            candidates=(),
            remote_diagnostics={},
            remote_performance={},
            ros_prepare_ms=1.0,
            transport_ms=2.0,
            decode_ms=3.0,
        )


def snapshot(stamp_sec):
    return types.SimpleNamespace(
        stamp_sec=float(stamp_sec),
        stamp_ns=int(round(float(stamp_sec) * 1e9)),
    )


def streaming_node(clock=None, prepare=None, start_worker=True):
    node = remote_node.RemoteGrasp6DNode.__new__(remote_node.RemoteGrasp6DNode)
    node.enabled = True
    node.status_pub = RecordingPublisher()
    node.pipeline_metrics_pub = RecordingPublisher()
    node.preview_plan_pub = RecordingPublisher()
    node.preview_rich_plan_pub = RecordingPublisher()
    node._prepare_and_predict = prepare or (
        lambda ticket: types.SimpleNamespace(
            ticket=ticket,
            candidates=(),
            remote_diagnostics={},
            remote_performance={},
            ros_prepare_ms=0.0,
            transport_ms=0.0,
            decode_ms=0.0,
        )
    )
    node._accept_prediction_calls = []
    node._accept_prediction = node._accept_prediction_calls.append
    node._initialize_streaming_state(
        result_max_age_sec=1.2,
        performance_window_size=100,
        source_clock=clock or MutableClock(10.0),
        start_worker=start_worker,
    )
    return node


def wait_until(predicate, timeout=2.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.005)
    raise AssertionError('condition did not become true')


def test_request_service_starts_and_stops_without_waiting_for_inference():
    node = streaming_node()
    try:
        started = node.request_plan_cb(types.SimpleNamespace(trigger=True))
        assert started.success is True
        assert node.streaming_enabled is True

        stopped = node.request_plan_cb(types.SimpleNamespace(trigger=False))
        assert stopped.success is True
        assert node.streaming_enabled is False
    finally:
        node.shutdown_streaming_worker()


def test_busy_inference_replaces_pending_and_drops_stopped_result():
    clock = MutableClock(10.0)
    prediction = BlockingPrediction()
    node = streaming_node(clock=clock, prepare=prediction)
    try:
        node.start_streaming()
        node.submit_stream_snapshot(snapshot(9.8))
        assert prediction.entered.wait(1.0)

        node.submit_stream_snapshot(snapshot(9.9))
        node.submit_stream_snapshot(snapshot(10.0))
        assert node.inference_coordinator.pending_count == 1

        node.stop_streaming()
        prediction.release.set()
        wait_until(
            lambda: bool(node.pipeline_metrics)
            and node.pipeline_metrics[-1]['completed'] == 3
        )

        assert node._accept_prediction_calls == []
        assert [item['request_id'] for item in node.pipeline_metrics] == [2, 3, 1]
        assert [item['drop_reason'] for item in node.pipeline_metrics] == [
            'PENDING_REPLACED',
            'GENERATION_STALE',
            'GENERATION_STALE',
        ]
        assert node.pipeline_metrics[-1]['drop_reason'] == 'GENERATION_STALE'
        assert node.pipeline_metrics[-1]['submitted'] == 3
        assert node.pipeline_metrics[-1]['started'] == 1
        assert node.pipeline_metrics[-1]['replaced'] == 1
        assert node.pipeline_metrics[-1]['stale'] == 3
    finally:
        prediction.release.set()
        node.shutdown_streaming_worker()


def test_pipeline_metrics_are_finite_strict_json_with_rolling_percentiles():
    history = list(range(1, 101))
    metrics = remote_node.build_pipeline_metrics(
        event='request_completed',
        request_id=101,
        generation=3,
        target_epoch=8,
        snapshot_stamp_sec=20.0,
        status='NO_CANDIDATE',
        drop_reason='',
        counters={
            'submitted': 101,
            'started': 100,
            'completed': 100,
            'accepted': 99,
            'expired': 1,
            'stale': 0,
            'replaced': 4,
            'busy': 5,
        },
        pending_replacements=4,
        ros_prepare_ms=1.0,
        transport_ms=2.0,
        decode_ms=3.0,
        remote_performance={
            'preprocess_ms': 4.0,
            'inference_ms': 5.0,
            'postprocess_ms': 6.0,
            'server_total_ms': 15.0,
            'gpu_allocated_mb': 1000.0,
            'gpu_reserved_mb': 1200.0,
            'gpu_peak_allocated_mb': 1300.0,
        },
        end_to_end_ms=25.0,
        result_age_ms=100.0,
        latency_history_ms=history,
        funnel={
            'stage_counts': {'returned': {'entered': 3, 'passed': 0, 'rejected': 3}},
            'rejection_counts': {'COLLISION': 3},
            'rejection_ratios': {'COLLISION': 1.0},
            'primary_failure': 'COLLISION',
        },
    )

    encoded = remote_node.strict_metrics_json(metrics)
    decoded = json.loads(encoded)

    assert decoded['latency_p50_ms'] == pytest.approx(50.5)
    assert decoded['latency_p95_ms'] == pytest.approx(95.05)
    assert decoded['primary_failure'] == 'COLLISION'
    assert set(('submitted', 'started', 'completed', 'accepted', 'expired',
                'stale', 'replaced', 'busy')) <= decoded.keys()
    assert all(
        math.isfinite(value)
        for value in decoded.values()
        if isinstance(value, float)
    )
    assert encoded == json.dumps(
        decoded, allow_nan=False, sort_keys=True, separators=(',', ':')
    )


def test_bounded_metrics_json_hard_limits_arbitrarily_long_strings():
    metrics = {
        'event': 'x' * 20000,
        'request_id': 7,
        'generation': 3,
        'target_epoch': 2,
        'status': '状态' * 20000,
        'drop_reason': 'reason' * 20000,
        'primary_failure': 'failure' * 20000,
        'stage_counts': {},
        'rejection_counts': {},
        'rejection_ratios': {},
    }

    encoded = remote_node.bounded_metrics_json(metrics, max_bytes=512)

    assert len(encoded.encode('utf-8')) <= 512
    assert json.loads(encoded)['request_id'] == 7


@pytest.mark.parametrize('invalidate_action', ['stop_restart', 'target_epoch'])
def test_queued_active_ticket_is_drained_before_new_pending_is_promoted(
    invalidate_action,
):
    clock = MutableClock(10.0)
    node = streaming_node(clock=clock, start_worker=False)
    try:
        node.start_streaming()
        node.submit_stream_snapshot(snapshot(9.8))
        old_ticket = node._stream_worker_ticket
        if invalidate_action == 'stop_restart':
            node.stop_streaming()
            node.start_streaming()
        else:
            node._advance_target_instance_epoch('test target changed')
        assert node._stream_worker_ticket is old_ticket

        node.submit_stream_snapshot(snapshot(9.9))
        assert node.inference_coordinator.pending_count == 1
        node._stream_worker = threading.Thread(
            target=node._stream_worker_main,
            daemon=True,
        )
        node._stream_worker.start()

        wait_until(lambda: len(node._accept_prediction_calls) == 1)
        assert node._accept_prediction_calls[0].ticket.request_id == 2
        assert node.pipeline_metrics[-1]['request_id'] == 2
        assert node.pipeline_metrics[-1]['drop_reason'] == ''
    finally:
        node.shutdown_streaming_worker()


def test_stop_during_acceptance_does_not_double_drop_or_predict_promoted_ticket():
    clock = MutableClock(10.0)
    prepare_entered = threading.Event()
    prepare_release = threading.Event()
    accept_entered = threading.Event()
    accept_release = threading.Event()
    prepared_ids = []

    def prepare(ticket):
        prepared_ids.append(ticket.request_id)
        if ticket.request_id == 1:
            prepare_entered.set()
            assert prepare_release.wait(1.0)
        return types.SimpleNamespace(
            ticket=ticket,
            candidates=(),
            remote_diagnostics={},
            remote_performance={},
            ros_prepare_ms=0.0,
            transport_ms=0.0,
            decode_ms=0.0,
        )

    node = streaming_node(clock=clock, prepare=prepare)

    def accept(_prepared):
        accept_entered.set()
        assert accept_release.wait(1.0)

    node._accept_prediction = accept
    try:
        node.start_streaming()
        node.submit_stream_snapshot(snapshot(9.8))
        assert prepare_entered.wait(1.0)
        node.submit_stream_snapshot(snapshot(9.9))
        prepare_release.set()
        assert accept_entered.wait(1.0)

        node.stop_streaming()
        accept_release.set()
        wait_until(
            lambda: [item['request_id'] for item in node.pipeline_metrics]
            == [1, 2]
        )

        assert prepared_ids == [1]
        assert node.pipeline_metrics[1]['drop_reason'] == 'GENERATION_STALE'
        assert sum(
            item['request_id'] == 2 for item in node.pipeline_metrics
        ) == 1
    finally:
        prepare_release.set()
        accept_release.set()
        node.shutdown_streaming_worker()


def test_remote_prediction_uses_protocol3_ticket_correlation():
    class RecordingClient:
        def __init__(self):
            self.calls = []
            self.last_diagnostics = {'returned': 1}
            self.last_performance = {'server_total_ms': 8.0}

        def predict(self, color, depth, intrinsics, **kwargs):
            self.calls.append((color, depth, intrinsics, kwargs))
            return ['candidate']

    node = remote_node.RemoteGrasp6DNode.__new__(remote_node.RemoteGrasp6DNode)
    node.client = RecordingClient()
    node.max_candidates = 20
    node.candidate_width_tolerance_m = 0.003
    ticket = InferenceTicket(
        request_id=41,
        generation=2,
        snapshot_stamp_sec=123.25,
        target_epoch=7,
        payload=None,
        submitted_monotonic_sec=123.25,
    )
    graspnet_input = types.SimpleNamespace(color_bgr='rgb', depth_raw='depth')

    result = node._predict_remote(
        ticket,
        graspnet_input,
        intrinsics='K',
        frame_id='camera_link',
    )

    kwargs = node.client.calls[0][3]
    assert kwargs['request_id'] == 41
    assert kwargs['snapshot_stamp_sec'] == 123.25
    assert kwargs['stamp_sec'] == 123.25
    assert result[0] == ('candidate',)
    assert result[1] == {'returned': 1}
    assert result[2] == {'server_total_ms': 8.0}


def test_tracker_empty_batch_always_carries_explicit_target_identity():
    class RecordingTracker:
        def __init__(self):
            self.calls = []

        def update(self, request_id, observations, target_identity=None):
            self.calls.append((request_id, tuple(observations), target_identity))
            return []

    node = remote_node.RemoteGrasp6DNode.__new__(remote_node.RemoteGrasp6DNode)
    node.tracker = RecordingTracker()

    stable = node._update_candidate_tracker(
        request_id=9,
        observations=(),
        target_identity=(4, 'carton', 'carton_segment'),
    )

    assert stable == []
    assert node.tracker.calls == [
        (9, (), (4, 'carton', 'carton_segment'))
    ]


def test_normal_distance_approach_and_visibility_are_soft_not_hard():
    node = remote_node.RemoteGrasp6DNode.__new__(remote_node.RemoteGrasp6DNode)
    common_gate = {
        'request_id': 9,
        'snapshot_stamp_sec': 20.0,
        'target_identity': (4, 'carton', 'carton_segment'),
        'track_id': 3,
        'variant_index': 0,
        'center_base_xyz': (0.12, 0.0, 0.2),
        'tool0_position_xyz': (0.12, 0.0, 0.2),
        'quaternion_xyzw': (0.0, 0.0, 0.0, 1.0),
        'approach_base_xyz': (0.0, 0.0, -1.0),
        'target_present': True,
        'same_target_instance': True,
        'target_absolute_distance_m': 0.12,
        'target_absolute_limit_m': 0.15,
        'required_open_width_m': 0.04,
        'physical_open_width_m': 0.05,
        'depth_valid': True,
        'transform_valid': True,
        'geometry_valid': True,
        'collision_free': True,
        'snapshot_context_revision': 'request-9',
    }
    assert node._candidate_safety_gate(**common_gate).ok is True

    preferred = node._candidate_soft_features(
        model_score=0.8,
        cloud_distance_m=0.005,
        center_distance_m=0.01,
        downward_approach_cos=0.9,
        visibility_center_cost=0.1,
        support_margin_m=0.01,
        jaw_tilt_cos=0.95,
        geometry_margin_m=0.01,
        stability_hit_ratio=0.8,
    )
    degraded = node._candidate_soft_features(
        model_score=0.8,
        cloud_distance_m=0.08,
        center_distance_m=0.12,
        downward_approach_cos=0.2,
        visibility_center_cost=0.9,
        support_margin_m=0.01,
        jaw_tilt_cos=0.95,
        geometry_margin_m=0.01,
        stability_hit_ratio=0.8,
    )

    assert soft_candidate_cost(
        degraded, SoftScoreWeights()
    ).total > soft_candidate_cost(preferred, SoftScoreWeights()).total


@pytest.mark.parametrize(
    ('overrides', 'expected_code'),
    [
        ({'depth_valid': False}, 'DEPTH_INVALID'),
        ({'target_present': False}, 'TARGET_LOST'),
        ({'required_open_width_m': 0.051}, 'GRIPPER_TOO_NARROW'),
        ({'collision_free': False}, 'COLLISION'),
        ({'transform_valid': False}, 'TRANSFORM_INVALID'),
    ],
)
def test_streaming_hard_facts_remain_rejected(overrides, expected_code):
    node = remote_node.RemoteGrasp6DNode.__new__(remote_node.RemoteGrasp6DNode)
    values = {
        'request_id': 9,
        'snapshot_stamp_sec': 20.0,
        'target_identity': (4, 'carton', 'carton_segment'),
        'track_id': 3,
        'variant_index': 0,
        'center_base_xyz': (0.12, 0.0, 0.2),
        'tool0_position_xyz': (0.12, 0.0, 0.2),
        'quaternion_xyzw': (0.0, 0.0, 0.0, 1.0),
        'approach_base_xyz': (0.0, 0.0, -1.0),
        'target_present': True,
        'same_target_instance': True,
        'target_absolute_distance_m': 0.02,
        'target_absolute_limit_m': 0.15,
        'required_open_width_m': 0.04,
        'physical_open_width_m': 0.05,
        'depth_valid': True,
        'transform_valid': True,
        'geometry_valid': True,
        'collision_free': True,
        'snapshot_context_revision': 'request-9',
    }
    values.update(overrides)

    decision = node._candidate_safety_gate(**values)

    assert decision.ok is False
    assert decision.code == expected_code


def test_stream_poll_is_nonblocking_and_only_submits_an_advanced_window(monkeypatch):
    class RecordingFrames:
        def __init__(self):
            self.calls = []
            self.samples = [object(), object(), object()]

        def wait_for_samples(self, *args, **kwargs):
            self.calls.append((args, kwargs))
            return list(self.samples)

    clock = MutableClock(10.0)
    node = streaming_node(clock=clock)
    try:
        node.frames = RecordingFrames()
        node.planning_snapshot_frames = 3
        node.planning_snapshot_max_age_sec = 0.35
        node.planning_snapshot_max_span_sec = 3.0
        node.planning_snapshot_max_inference_latency_sec = 1.2
        node.planning_mask_min_iou = 0.85
        node.planning_mask_max_centroid_shift_px = 5.0
        node.planning_max_joint_delta_rad = 0.01
        node.mask_erosion_px = 2
        node.mask_internal_hole_max_area_px = 25
        node.depth_mad_scale = 3.5
        node.depth_mad_absolute_floor_m = 0.002
        node._snapshot_depth_config = lambda: (0.001, 0.03, 2.0)
        node._freeze_graspnet_input_config = lambda: types.SimpleNamespace(
            requires_instance_mask=False
        )
        node._active_profile_requires_mask = lambda: False
        fused = snapshot(9.9)
        fused.ok = True
        monkeypatch.setattr(remote_node, 'fuse_stable_samples', lambda *_a, **_k: fused)

        node.start_streaming()
        assert node._poll_stream_snapshot() is True
        assert node._poll_stream_snapshot() is False

        first_args, first_kwargs = node.frames.calls[0]
        assert first_args[1] == 0.0
        assert first_kwargs['newest_after_ns'] == 0
        assert node.last_submitted_stamp_ns == 9_900_000_000
    finally:
        node.shutdown_streaming_worker()


def test_preview_publication_never_mutates_execution_publishers():
    node = remote_node.RemoteGrasp6DNode.__new__(remote_node.RemoteGrasp6DNode)
    node.preview_plan_pub = RecordingPublisher()
    node.preview_rich_plan_pub = RecordingPublisher()
    node.plan_pub = RecordingPublisher()
    node.rich_plan_pub = RecordingPublisher()
    rich = types.SimpleNamespace(plan_id='preview-1')
    legacy = types.SimpleNamespace(poses=['preview'])

    node._publish_preview_plan(rich, legacy)

    assert [item.plan_id for item in node.preview_rich_plan_pub.messages] == [
        'preview-1'
    ]
    assert node.preview_plan_pub.messages[0].poses == ['preview']
    assert node.rich_plan_pub.messages == []
    assert node.plan_pub.messages == []


def matching_observation(request_id):
    return CandidateObservation(
        request_id=request_id,
        snapshot_stamp_sec=20.0 + request_id * 0.01,
        target_epoch=4,
        target_label='carton',
        model_choice='carton_segment',
        center_base_xyz=(0.1, 0.0, 0.2),
        tool0_position_xyz=(0.1, 0.0, 0.2),
        quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
        approach_base_xyz=(0.0, 0.0, -1.0),
        required_open_width_m=0.04,
        model_width_m=0.038,
        model_score=0.8,
        geometry_margin_m=0.008,
        pre_moveit_score=0.0,
        payload={'request_id': request_id},
    )


def prepared_prediction(request_id):
    ticket = InferenceTicket(
        request_id=request_id,
        generation=1,
        snapshot_stamp_sec=20.0 + request_id * 0.01,
        target_epoch=4,
        payload=None,
        submitted_monotonic_sec=20.0,
    )
    return types.SimpleNamespace(
        ticket=ticket,
        snapshot=types.SimpleNamespace(
            object_msg=types.SimpleNamespace(detected=True, label='carton')
        ),
        candidates=('raw',),
        remote_diagnostics={},
        remote_performance={},
    )


def test_third_matching_prediction_enters_current_recheck_and_moveit_top_n(
    monkeypatch,
):
    node = remote_node.RemoteGrasp6DNode.__new__(remote_node.RemoteGrasp6DNode)
    node.target_instance_epoch = 4
    node._last_model_choice = 'carton_segment'
    node.tracker = CandidateTracker(TrackingConfig(window_size=5, min_hits=3))
    node.moveit_top_n = 5
    node._activate_prepared_geometry = lambda _prepared: True
    node._evaluate_local_candidates = lambda prepared: (
        (matching_observation(prepared.ticket.request_id),),
        {'stage_counts': {}, 'rejection_counts': {}, 'rejection_ratios': {}},
    )
    recheck_calls = []
    node._recheck_and_score_stable = lambda prepared, stable: (
        recheck_calls.append((prepared.ticket.request_id, tuple(stable)))
        or tuple(stable)
    )
    bounded_calls = []

    def fake_bounded(candidates, checker, top_n):
        del checker
        bounded_calls.append((tuple(candidates), top_n))
        return types.SimpleNamespace(
            selected=tuple(candidates)[0],
            checked=tuple(candidates),
            reachable=tuple(candidates),
            funnel=types.SimpleNamespace(
                to_dict=lambda: {
                    'stage_counts': {
                        'moveit_checked': {
                            'entered': 1,
                            'passed': 1,
                            'rejected': 0,
                        }
                    },
                    'rejection_counts': {},
                    'rejection_ratios': {},
                    'primary_failure': None,
                }
            ),
        )

    monkeypatch.setattr(remote_node, 'bounded_moveit_select', fake_bounded)
    node._check_moveit_stable_candidate = lambda _candidate: None
    published = []
    node._publish_selected_preview = published.append

    first = node._accept_prediction(prepared_prediction(1))
    second = node._accept_prediction(prepared_prediction(2))
    third = node._accept_prediction(prepared_prediction(3))

    assert first['status'] == 'STABILITY_PENDING'
    assert second['status'] == 'STABILITY_PENDING'
    assert published == [bounded_calls[0][0][0]]
    assert recheck_calls[0][0] == 3
    assert len(recheck_calls[0][1]) == 1
    assert bounded_calls[0][1] == 5
    assert third['status'] == 'PREVIEW_READY'


def test_current_recheck_binds_conservative_latest_required_width():
    node = remote_node.RemoteGrasp6DNode.__new__(remote_node.RemoteGrasp6DNode)
    node.target_instance_epoch = 4
    node.grasp_config = {
        'tool_approach_axis': 'z',
        'pregrasp_distance_m': 0.08,
        'final_approach_offset_m': 0.015,
        'lift_height_m': 0.05,
    }
    node.gripper_physical_open_width_m = 0.05
    node.target_absolute_sanity_distance_m = 0.15
    node.soft_score_weights = SoftScoreWeights()
    node.camera_visibility_gate_enabled = False
    node.camera_visibility_diagnostic_enabled = False
    latest_gate = CandidateGateResult(
        ok=True,
        failure_code='',
        failure_reason='',
        required_open_width_m=0.045,
        center_distance_m=0.0,
        support_clearance_m=0.01,
        jaw_alignment=1.0,
        motion_cost=0.0,
        geometry_cost=0.0,
        failed_gate='',
        passed_gate_count=6,
    )
    node._evaluate_candidate_geometry = lambda *_args: latest_gate
    camera_candidate = RemoteGraspCandidate(
        score=0.8,
        translation_m=(0.1, 0.0, 0.2),
        quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
        width_m=0.038,
        depth_m=0.03,
        tool0_translation_m=(0.1, 0.0, 0.2),
    )
    features = SoftCandidateFeatures(
        model_score=0.8,
        cloud_distance_m=0.01,
        center_distance_m=0.01,
        downward_approach_cos=1.0,
        visibility_center_cost=0.0,
        support_margin_m=0.01,
        jaw_tilt_cos=1.0,
        geometry_margin_m=0.01,
        joint_path_cost=0.0,
        joint_max_delta_rad=0.0,
        stability_hit_ratio=0.6,
        position_dispersion_m=0.0,
        orientation_dispersion_rad=0.0,
    )
    payload = remote_node.LocalCandidatePayload(
        raw_candidate_index=0,
        variant_index=0,
        raw_candidate=camera_candidate,
        camera_candidate=camera_candidate,
        grasp_pose=None,
        geometry_gate=latest_gate,
        soft_features=features,
        score_components={},
    )
    stable = StableCandidate(
        track_id=1,
        hit_count=3,
        window_count=5,
        hit_request_ids=(1, 2, 3),
        request_id=3,
        snapshot_stamp_sec=19.9,
        target_epoch=4,
        target_label='carton',
        model_choice='carton_segment',
        center_base_xyz=(0.1, 0.0, 0.2),
        tool0_position_xyz=(0.1, 0.0, 0.2),
        quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
        approach_base_xyz=(0.0, 0.0, -1.0),
        required_open_width_m=0.04,
        model_width_m=0.038,
        model_score=0.8,
        geometry_margin_m=0.01,
        pre_moveit_score=0.0,
        payload=payload,
    )
    ticket = InferenceTicket(
        request_id=4,
        generation=1,
        snapshot_stamp_sec=20.0,
        target_epoch=4,
        payload=None,
        submitted_monotonic_sec=20.0,
    )
    prepared = types.SimpleNamespace(
        ticket=ticket,
        stamp=remote_node.rospy.Time.from_sec(20.0),
        geometry=types.SimpleNamespace(center_base=(0.1, 0.0, 0.2)),
        pose_estimator=types.SimpleNamespace(transform_sha256='current-tf'),
        snapshot=types.SimpleNamespace(
            object_msg=types.SimpleNamespace(detected=True, label='carton')
        ),
        model_choice='carton_segment',
    )

    scored = node._recheck_and_score_stable(prepared, (stable,))

    assert len(scored) == 2
    assert all(
        item.stable_candidate.required_open_width_m == pytest.approx(0.045)
        for item in scored
    )
    assert all(
        item.latest_safety.required_open_width_m == pytest.approx(0.045)
        for item in scored
    )
