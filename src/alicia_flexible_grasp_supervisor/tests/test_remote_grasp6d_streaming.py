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
    ExecutionPlanController,
    MoveItResult,
    PromotionDecision,
    SafetyGateInput,
    ScoredStableCandidate,
    SoftCandidateFeatures,
    SoftScoreWeights,
    bounded_moveit_select,
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


class FailingPublisher:
    def publish(self, _message):
        raise RuntimeError('publisher unavailable')


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
    assert set(('submitted', 'started', 'completed', 'accepted', 'failed',
                'expired', 'stale', 'replaced', 'busy')) <= decoded.keys()
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
    decoded = json.loads(encoded)
    assert decoded['request_id'] == 7
    assert decoded['metrics_truncated'] is True


def test_metrics_truncation_preserves_flag_original_totals_and_audit_hash():
    metrics = {
        'event': 'request_completed',
        'request_id': 7,
        'generation': 3,
        'target_epoch': 2,
        'status': 'PREVIEW_READY',
        'error': 'x' * 20000,
        'submitted': 11,
        'completed': 10,
        'stage_counts': {
            'stage-%03d' % index: {
                'entered': 10,
                'passed': 5,
                'rejected': 5,
            }
            for index in range(80)
        },
        'rejection_counts': {
            'FAILURE-%03d' % index: index + 1
            for index in range(80)
        },
        'rejection_ratios': {
            'FAILURE-%03d' % index: 0.5
            for index in range(80)
        },
        'audit_reference': {
            'report_path': '/tmp/task5-audit.json',
            'report_sha256': 'a' * 64,
            'row_count': 160,
        },
    }

    decoded = json.loads(
        remote_node.bounded_metrics_json(metrics, max_bytes=1200)
    )

    assert decoded['metrics_truncated'] is True
    assert decoded['metrics_original_totals'] == {
        'rejection_code_count': 80,
        'rejection_total': sum(range(1, 81)),
        'stage_count': 80,
        'stage_entered_total': 800,
        'stage_passed_total': 400,
        'stage_rejected_total': 400,
    }
    assert decoded['submitted'] == 11
    assert decoded['completed'] == 10
    assert decoded['audit_reference']['report_sha256'] == 'a' * 64


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
        wait_until(
            lambda: bool(node.pipeline_metrics)
            and node.pipeline_metrics[-1]['request_id'] == 2
        )
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


def test_worker_records_post_completion_cancellation_as_stale_not_accepted():
    clock = MutableClock(10.0)
    accept_entered = threading.Event()
    accept_release = threading.Event()
    node = streaming_node(clock=clock)

    def accept(prepared):
        accept_entered.set()
        assert accept_release.wait(1.0)
        node._require_stream_ticket_current(prepared.ticket)

    node._accept_prediction = accept
    try:
        node.start_streaming()
        node.submit_stream_snapshot(snapshot(9.8))
        assert accept_entered.wait(1.0)

        node.stop_streaming()
        accept_release.set()
        wait_until(lambda: len(node.pipeline_metrics) == 1)

        terminal = node.pipeline_metrics[0]
        assert terminal['request_id'] == 1
        assert terminal['status'] == 'GENERATION_STALE'
        assert terminal['drop_reason'] == 'GENERATION_STALE'
        assert terminal['accepted'] == 0
        assert terminal['stale'] == 1
        assert terminal['completed'] == terminal['submitted'] == 1
    finally:
        accept_release.set()
        node.shutdown_streaming_worker()


def test_shutdown_emits_one_terminal_drop_per_queued_request_without_remote():
    prepared_ids = []
    node = streaming_node(
        clock=MutableClock(10.0),
        prepare=lambda ticket: prepared_ids.append(ticket.request_id),
        start_worker=False,
    )
    node.start_streaming()
    node.submit_stream_snapshot(snapshot(9.8))
    node.submit_stream_snapshot(snapshot(9.9))

    assert node.shutdown_streaming_worker() is True

    assert prepared_ids == []
    assert sorted(item['request_id'] for item in node.pipeline_metrics) == [1, 2]
    assert all(
        item['drop_reason'] == 'GENERATION_STALE'
        for item in node.pipeline_metrics
    )
    assert all(
        sum(
            other['request_id'] == item['request_id']
            for other in node.pipeline_metrics
        ) == 1
        for item in node.pipeline_metrics
    )
    assert node._pipeline_counters['completed'] == 2
    assert node._pipeline_counters['submitted'] == 2
    assert node._pipeline_counters['stale'] == 2


def test_streaming_terminal_and_metrics_histories_are_bounded():
    node = streaming_node(
        clock=MutableClock(10.0),
        start_worker=False,
    )
    node.pipeline_metrics = []
    node._initialize_streaming_state(
        result_max_age_sec=1.2,
        performance_window_size=3,
        source_clock=MutableClock(10.0),
        start_worker=False,
    )

    for request_id in range(1, 26):
        node._request_telemetry[request_id] = {
            'request_id': request_id,
            'generation': 1,
            'target_epoch': 1,
            'snapshot_stamp_sec': 9.5,
            'submitted_sec': 9.5,
        }
        node._emit_pending_drop_metrics(request_id, 'GENERATION_STALE')

    assert len(node.pipeline_metrics) == 3
    assert len(node._terminal_request_ids) <= 8
    assert node._request_telemetry == {}


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


def test_remote_prediction_prefers_request_local_immutable_bundle():
    class BundleClient:
        last_diagnostics = {'stale': 'must not be read'}
        last_performance = {'server_total_ms': 999.0}

        def __init__(self):
            self.calls = []

        def predict(self, *_args, **_kwargs):
            raise AssertionError('legacy shared-state predict was used')

        def predict_bundle(self, color, depth, intrinsics, **kwargs):
            self.calls.append((color, depth, intrinsics, kwargs))
            return types.SimpleNamespace(
                candidates=('candidate',),
                diagnostics={'returned': 1},
                performance={'server_total_ms': 8.0},
                encode_ms=4.0,
                transport_ms=5.0,
                decode_ms=6.0,
            )

    node = remote_node.RemoteGrasp6DNode.__new__(remote_node.RemoteGrasp6DNode)
    node.client = BundleClient()
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

    assert result == (
        ('candidate',),
        {'returned': 1},
        {'server_total_ms': 8.0},
        4.0,
        5.0,
        6.0,
    )


def test_worker_latency_is_measured_after_accept_side_effects_finish():
    clock = MutableClock(10.0)
    node = streaming_node(clock=clock)

    def accept(_prepared):
        clock.value = 12.0

    node._accept_prediction = accept
    try:
        node.start_streaming()
        node.submit_stream_snapshot(snapshot(9.8))
        wait_until(lambda: len(node.pipeline_metrics) == 1)

        terminal = node.pipeline_metrics[0]
        assert terminal['end_to_end_ms'] == pytest.approx(2000.0)
        assert terminal['result_age_ms'] == pytest.approx(2200.0)
    finally:
        node.shutdown_streaming_worker()


@pytest.mark.parametrize(
    ('failure_stage', 'expected_status'),
    [
        ('predict', 'PREDICT_FAILED'),
        ('accept', 'ACCEPT_FAILED'),
    ],
)
def test_pipeline_exceptions_have_one_failed_terminal_and_conserve_counts(
    failure_stage,
    expected_status,
):
    def fail_predict(_ticket):
        raise RuntimeError('synthetic predict failure')

    node = streaming_node(
        clock=MutableClock(10.0),
        prepare=fail_predict if failure_stage == 'predict' else None,
    )
    if failure_stage == 'accept':
        def fail_accept(_prepared):
            raise RuntimeError('synthetic accept failure')

        node._accept_prediction = fail_accept
    try:
        node.start_streaming()
        node.submit_stream_snapshot(snapshot(9.8))
        wait_until(lambda: len(node.pipeline_metrics) == 1)

        terminal = node.pipeline_metrics[0]
        assert terminal['status'] == expected_status
        assert terminal['drop_reason'] == expected_status
        assert terminal['completed'] == 1
        assert terminal['accepted'] == 0
        assert terminal['failed'] == 1
        assert terminal['expired'] == 0
        assert terminal['stale'] == 0
        assert terminal['completed'] == (
            terminal['accepted']
            + terminal['failed']
            + terminal['expired']
            + terminal['stale']
        )
        assert sum(
            item['request_id'] == terminal['request_id']
            for item in node.pipeline_metrics
        ) == 1
    finally:
        node.shutdown_streaming_worker()


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


def test_zero_locally_valid_candidates_report_primary_failure_not_stability():
    class RetainingTracker:
        def __init__(self):
            self.calls = []

        def update(self, request_id, observations, target_identity=None):
            self.calls.append((request_id, tuple(observations), target_identity))
            return ('stable-from-an-earlier-request',)

    node = remote_node.RemoteGrasp6DNode.__new__(remote_node.RemoteGrasp6DNode)
    node.target_instance_epoch = 4
    node._last_model_choice = 'carton_segment'
    node._stream_condition = threading.Condition(threading.RLock())
    node._stream_shutdown = threading.Event()
    node.streaming_enabled = True
    node._stream_generation = 1
    node.tracker = RetainingTracker()
    node._activate_prepared_geometry = lambda _prepared: True
    node._evaluate_local_candidates = lambda _prepared: (
        (),
        {
            'input_count': 0,
            'stage_counts': {
                'locally_valid': {
                    'entered': 0,
                    'passed': 0,
                    'rejected': 0,
                }
            },
            'rejection_counts': {'REMOTE_NO_CANDIDATES': 1},
            'rejection_ratios': {'REMOTE_NO_CANDIDATES': 1.0},
            'primary_failure': 'REMOTE_NO_CANDIDATES',
        },
    )
    node._recheck_and_score_stable = lambda *_args: pytest.fail(
        'old stable candidate must not be reused for a zero-valid request'
    )

    result = node._accept_prediction(prepared_prediction(1))

    assert result['status'] == 'REMOTE_NO_CANDIDATES:1'
    assert result['funnel']['primary_failure'] == 'REMOTE_NO_CANDIDATES'
    assert node.tracker.calls[0][1] == ()
    assert node._stable_variant_runtime == {}


def test_empty_remote_batch_creates_explicit_no_candidates_rejection():
    node = remote_node.RemoteGrasp6DNode.__new__(remote_node.RemoteGrasp6DNode)
    node.require_candidate_depth = True
    node.model_grasp_to_tool_quaternion = (
        remote_node.STRICT_MODEL_GRASP_TO_TOOL_QUATERNION
    )
    node.grasp_config = {'tool_approach_axis': 'z'}
    node.soft_score_weights = SoftScoreWeights()
    prepared = types.SimpleNamespace(
        candidates=(),
        remote_diagnostics={
            'raw_candidates': 0,
            'after_nms': 0,
            'after_collision': 0,
        },
        snapshot=types.SimpleNamespace(
            object_msg=types.SimpleNamespace(label='carton')
        ),
        ticket=types.SimpleNamespace(target_epoch=4),
        model_choice='carton_segment',
        geometry=types.SimpleNamespace(center_base=(0.1, 0.0, 0.2)),
        pose_estimator=types.SimpleNamespace(transform_sha256='request-4'),
    )

    observations, funnel = node._evaluate_local_candidates(prepared)

    assert observations == ()
    assert funnel['primary_failure'] == 'REMOTE_NO_CANDIDATES'
    assert funnel['rejection_counts'] == {'REMOTE_NO_CANDIDATES': 1}
    assert funnel['rejection_ratios'] == {'REMOTE_NO_CANDIDATES': 1.0}


def test_generic_strict_moveit_failure_does_not_invent_hard_states(monkeypatch):
    node = remote_node.RemoteGrasp6DNode.__new__(remote_node.RemoteGrasp6DNode)
    node._candidate_plan_metrics = {}
    node._position_only_rejected_count = 0
    node._orientation_fallback_rejected_count = 0
    response = types.SimpleNamespace(
        success=False,
        message='collision joint limit IK planning all mentioned here',
    )
    monkeypatch.setattr(remote_node.rospy, 'wait_for_service', lambda *_a, **_k: None)
    monkeypatch.setattr(
        remote_node.rospy,
        'ServiceProxy',
        lambda *_a, **_k: lambda *_args: response,
    )
    pose = types.SimpleNamespace(
        pose=types.SimpleNamespace(
            position=types.SimpleNamespace(x=0.0, y=0.0, z=0.0),
            orientation=types.SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
        )
    )

    result = node._strict_moveit_result(pose)

    assert result.reachable is False
    assert result.failure_code == 'MOVEIT_UNREACHABLE'
    assert result.collision_free is None
    assert result.within_joint_limits is None
    assert result.ik_valid is None
    assert result.planning_success is None


def test_unstructured_planning_failure_code_is_normalized_to_unreachable(
    monkeypatch,
):
    node = remote_node.RemoteGrasp6DNode.__new__(remote_node.RemoteGrasp6DNode)
    node._candidate_plan_metrics = {}
    node._position_only_rejected_count = 0
    node._orientation_fallback_rejected_count = 0
    response = types.SimpleNamespace(
        success=False,
        message='generic SetTargetPose failure',
        failure_code='MOVEIT_PLANNING_FAILED',
    )
    monkeypatch.setattr(
        remote_node.rospy,
        'wait_for_service',
        lambda *_args, **_kwargs: None,
    )
    monkeypatch.setattr(
        remote_node.rospy,
        'ServiceProxy',
        lambda *_args, **_kwargs: lambda *_call_args: response,
    )
    pose = types.SimpleNamespace(
        pose=types.SimpleNamespace(
            position=types.SimpleNamespace(x=0.0, y=0.0, z=0.0),
            orientation=types.SimpleNamespace(
                x=0.0, y=0.0, z=0.0, w=1.0
            ),
        )
    )

    result = node._strict_moveit_result(pose)

    assert result.failure_code == 'MOVEIT_UNREACHABLE'
    assert result.collision_free is None
    assert result.within_joint_limits is None
    assert result.ik_valid is None
    assert result.planning_success is None


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


def test_submit_atomically_rejects_snapshot_from_previous_target_identity():
    node = streaming_node(clock=MutableClock(30.0), start_worker=False)
    try:
        node.latest_object = types.SimpleNamespace(detected=True, label='carton')
        node._last_model_choice = 'carton_segment'
        node.start_streaming()
        current_identity = (
            node.target_instance_epoch,
            'carton',
            'carton_segment',
        )
        stale = snapshot(29.8)
        stale.target_epoch = current_identity[0] - 1
        stale.target_identity = (
            current_identity[0] - 1,
            current_identity[1],
            current_identity[2],
        )

        assert node.submit_stream_snapshot(stale) is False
        assert node.inference_coordinator.pending_count == 0
        assert node.last_submitted_stamp_ns == 0

        fresh = snapshot(29.9)
        fresh.target_epoch = current_identity[0]
        fresh.target_identity = current_identity
        assert node.submit_stream_snapshot(fresh) is True
        assert node._stream_worker_ticket.snapshot_stamp_sec == 29.9
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


def test_stop_barrier_prevents_old_accept_from_committing_tracker_state():
    node = streaming_node(clock=MutableClock(40.0), start_worker=False)

    class RecordingTracker:
        def __init__(self):
            self.calls = []

        def update(self, *args, **kwargs):
            self.calls.append((args, kwargs))
            return []

    try:
        del node._accept_prediction
        node.start_streaming()
        node.submit_stream_snapshot(snapshot(39.8))
        ticket = node._stream_worker_ticket
        tracker = RecordingTracker()
        node.tracker = tracker
        node._activate_prepared_geometry = lambda _prepared: True

        def stop_during_local_evaluation(_prepared):
            node.stop_streaming()
            return (matching_observation(ticket.request_id),), {
                'stage_counts': {},
                'rejection_counts': {},
                'rejection_ratios': {},
            }

        node._evaluate_local_candidates = stop_during_local_evaluation
        prepared = prepared_prediction(ticket.request_id)
        prepared.ticket = ticket

        with pytest.raises(remote_node.StreamResultCancelled):
            node._accept_prediction(prepared)

        assert tracker.calls == []
    finally:
        node.shutdown_streaming_worker()


def test_stop_barrier_prevents_each_stale_moveit_call():
    node = streaming_node(clock=MutableClock(50.0), start_worker=False)
    try:
        node.start_streaming()
        node.submit_stream_snapshot(snapshot(49.8))
        ticket = node._stream_worker_ticket
        stale_runtime = {
            (3, 1): {
                'prepared': types.SimpleNamespace(ticket=ticket),
                'grasp_pose': object(),
            }
        }
        calls = []
        node._strict_moveit_result = lambda pose: calls.append(pose)
        node.stop_streaming()
        # Stop deliberately clears production runtime.  Restore only the old
        # ticket in this fixture to exercise the per-call stale barrier.
        node._stable_variant_runtime = stale_runtime

        with pytest.raises(remote_node.StreamResultCancelled):
            node._check_moveit_stable_candidate(
                types.SimpleNamespace(track_id=3, variant_index=1)
            )

        assert calls == []
    finally:
        node.shutdown_streaming_worker()


@pytest.mark.parametrize(
    ('invalidate_action', 'expected_code'),
    [
        ('stop', 'GENERATION_STALE'),
        ('target_epoch', 'TARGET_EPOCH_STALE'),
    ],
)
def test_stop_during_inflight_moveit_discards_result_and_metrics(
    monkeypatch,
    invalidate_action,
    expected_code,
):
    node = streaming_node(clock=MutableClock(50.0), start_worker=False)
    entered = threading.Event()
    release = threading.Event()
    outcomes = []
    try:
        node.start_streaming()
        node.submit_stream_snapshot(snapshot(49.8))
        ticket = node._stream_worker_ticket
        pose = remote_node.PoseStamped()
        node._candidate_plan_metrics = {}
        node._position_only_rejected_count = 0
        node._orientation_fallback_rejected_count = 0
        node._stable_variant_runtime = {
            (3, 1): {
                'prepared': types.SimpleNamespace(ticket=ticket),
                'grasp_pose': pose,
            }
        }
        response = types.SimpleNamespace(
            success=True,
            message=(
                'planned joint_path_cost=1.25 joint_max_delta=0.35'
            ),
        )

        def strict_rpc(*_args):
            entered.set()
            if not release.wait(2.0):
                raise RuntimeError('test did not release MoveIt RPC')
            return response

        monkeypatch.setattr(
            remote_node.rospy,
            'wait_for_service',
            lambda *_args, **_kwargs: None,
        )
        monkeypatch.setattr(
            remote_node.rospy,
            'ServiceProxy',
            lambda *_args, **_kwargs: strict_rpc,
        )

        def invoke_checker():
            try:
                outcomes.append(
                    node._check_moveit_stable_candidate(
                        types.SimpleNamespace(track_id=3, variant_index=1)
                    )
                )
            except Exception as exc:
                outcomes.append(exc)

        thread = threading.Thread(target=invoke_checker)
        thread.start()
        assert entered.wait(1.0)
        if invalidate_action == 'stop':
            node.stop_streaming()
        else:
            node._advance_target_instance_epoch('TEST_TARGET_CHANGED')
        release.set()
        thread.join(1.0)

        assert not thread.is_alive()
        assert len(outcomes) == 1
        assert isinstance(outcomes[0], remote_node.StreamResultCancelled)
        assert outcomes[0].code == expected_code
        assert node._candidate_plan_metrics == {}
    finally:
        release.set()
        node.shutdown_streaming_worker()


def test_missing_stable_runtime_is_generic_moveit_check_error():
    node = streaming_node(clock=MutableClock(50.0), start_worker=False)
    try:
        result = node._check_moveit_stable_candidate(
            types.SimpleNamespace(track_id=404, variant_index=0)
        )

        assert result.reachable is False
        assert result.failure_code == 'MOVEIT_CHECK_ERROR'
        assert result.collision_free is None
        assert result.within_joint_limits is None
        assert result.ik_valid is None
        assert result.planning_success is None
    finally:
        node.shutdown_streaming_worker()


def test_top_n_uses_node_strict_moveit_checker_for_only_three_candidates():
    node = streaming_node(clock=MutableClock(50.0), start_worker=False)
    try:
        node.start_streaming()
        node.submit_stream_snapshot(snapshot(49.8))
        ticket = node._stream_worker_ticket
        candidates = []
        runtime = {}
        for track_id in range(5):
            center = (0.1 + 0.001 * track_id, 0.0, 0.2)
            stable = StableCandidate(
                track_id=track_id,
                hit_count=3,
                window_count=5,
                hit_request_ids=(1, 2, 3),
                request_id=3,
                snapshot_stamp_sec=49.8,
                target_epoch=ticket.target_epoch,
                target_label='carton',
                model_choice='carton_segmentation',
                center_base_xyz=center,
                tool0_position_xyz=center,
                quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
                approach_base_xyz=(0.0, 0.0, -1.0),
                required_open_width_m=0.04,
                model_width_m=0.04,
                model_score=0.8,
                geometry_margin_m=0.01,
                pre_moveit_score=0.0,
                position_dispersion_m=0.002,
                orientation_dispersion_rad=0.03,
                payload=None,
            )
            safety = SafetyGateInput(
                depth_valid=True,
                transform_valid=True,
                target_present=True,
                same_target_instance=True,
                target_absolute_distance_m=0.01,
                target_absolute_limit_m=0.2,
                required_open_width_m=0.04,
                physical_open_width_m=0.05,
                geometry_valid=True,
                collision_free=True,
                request_id=3,
                snapshot_stamp_sec=49.8,
                target_epoch=ticket.target_epoch,
                target_label='carton',
                model_choice='carton_segmentation',
                track_id=track_id,
                variant_index=0,
                center_base_xyz=center,
                tool0_position_xyz=center,
                quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
                approach_base_xyz=(0.0, 0.0, -1.0),
                snapshot_context_revision='ctx-3',
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
                position_dispersion_m=0.002,
                orientation_dispersion_rad=0.03,
            )
            candidates.append(
                ScoredStableCandidate(
                    stable_candidate=stable,
                    variant_index=0,
                    latest_safety=safety,
                    soft_features=features,
                    score_weights=SoftScoreWeights(),
                    evaluation_request_id=3,
                    evaluation_snapshot_stamp_sec=49.8,
                    evaluation_context_revision='ctx-3',
                )
            )
            runtime[(track_id, 0)] = {
                'prepared': types.SimpleNamespace(ticket=ticket),
                'grasp_pose': object(),
            }
        node._stable_variant_runtime = runtime
        calls = []

        def strict_checker(pose):
            calls.append(pose)
            return (
                MoveItResult(
                    reachable=False,
                    joint_path_cost=0.0,
                    joint_max_delta_rad=0.0,
                    reason='not reachable',
                    failure_code='MOVEIT_UNREACHABLE',
                ),
                {},
                '',
            )

        node._strict_moveit_evaluation = strict_checker

        selection = bounded_moveit_select(
            candidates,
            node._check_moveit_stable_candidate,
            top_n=3,
        )

        assert selection.selected is None
        assert len(calls) == 3
        assert len(selection.checked) == 3
        assert selection.funnel.rejection_counts == {
            'MOVEIT_UNREACHABLE': 3
        }
    finally:
        node.shutdown_streaming_worker()


def test_stop_barrier_prevents_stale_preview_and_atomic_audit_commit(tmp_path):
    node = streaming_node(clock=MutableClock(60.0), start_worker=False)
    try:
        node.start_streaming()
        node.submit_stream_snapshot(snapshot(59.8))
        ticket = node._stream_worker_ticket
        node.gate_audit_enabled = True
        node.gate_audit_output_path = str(tmp_path / 'planning.json')
        node.mujoco_audit_output_path = str(tmp_path / 'mujoco.json')
        node.stop_streaming()

        with pytest.raises(remote_node.StreamResultCancelled):
            node._publish_preview_plan(
                types.SimpleNamespace(plan_id='stale'),
                types.SimpleNamespace(poses=['stale']),
                ticket=ticket,
            )
        with pytest.raises(remote_node.StreamResultCancelled):
            node._write_gate_audit_report({'rows': []}, ticket=ticket)

        assert node.preview_rich_plan_pub.messages == []
        assert node.preview_plan_pub.messages == []
        assert not pathlib.Path(node.gate_audit_output_path).exists()
    finally:
        node.shutdown_streaming_worker()


def test_activation_rechecks_cancellation_before_global_geometry_commit():
    node = streaming_node(clock=MutableClock(70.0), start_worker=False)
    try:
        node.start_streaming()
        node.submit_stream_snapshot(snapshot(69.8))
        ticket = node._stream_worker_ticket
        node._planning_snapshot_active = False
        node._planning_object_msg = 'original-object'
        node._planning_object_time = 'original-time'
        node._current_prepared_prediction = 'original-prepared'
        cache_clears = []
        activations = []

        def stop_before_commit(_generation):
            node.stop_streaming()
            return False, ''

        node._geometry_invalidation_state = stop_before_commit
        node._clear_geometry_cache = lambda: cache_clears.append(True)
        node._activate_geometry = lambda *_args, **_kwargs: (
            activations.append(True) or True
        )
        node.camera_visibility_gate_enabled = False
        node.camera_visibility_diagnostic_enabled = False
        prepared = types.SimpleNamespace(
            ticket=ticket,
            request_invalidation_generation=3,
            snapshot=types.SimpleNamespace(object_msg='new-object'),
            stamp='new-time',
            geometry=object(),
            pose_estimator=types.SimpleNamespace(T_base_optical=object()),
            graspnet_input_audit={},
        )

        with pytest.raises(remote_node.StreamResultCancelled):
            node._activate_prepared_geometry(prepared)

        assert node._planning_snapshot_active is False
        assert node._planning_object_msg == 'original-object'
        assert node._planning_object_time == 'original-time'
        assert node._current_prepared_prediction == 'original-prepared'
        assert cache_clears == []
        assert activations == []
    finally:
        node.shutdown_streaming_worker()


def test_preview_publication_and_latest_audit_share_one_token_commit():
    node = streaming_node(clock=MutableClock(80.0), start_worker=False)
    try:
        node.start_streaming()
        node.submit_stream_snapshot(snapshot(79.8))
        ticket = node._stream_worker_ticket
        commits = []

        node._publish_preview_plan(
            types.SimpleNamespace(plan_id='preview-current'),
            types.SimpleNamespace(poses=['preview-current']),
            ticket=ticket,
            commit_callback=lambda: commits.append(ticket.request_id),
        )

        assert commits == [ticket.request_id]
        assert node.latest_preview_rich_plan.plan_id == 'preview-current'
        node.stop_streaming()
        with pytest.raises(remote_node.StreamResultCancelled):
            node._publish_preview_plan(
                types.SimpleNamespace(plan_id='preview-stale'),
                types.SimpleNamespace(poses=['preview-stale']),
                ticket=ticket,
                commit_callback=lambda: commits.append('stale'),
            )
        assert commits == [ticket.request_id]
    finally:
        node.shutdown_streaming_worker()


def test_streaming_audit_commits_request_local_report_under_token(tmp_path):
    node = streaming_node(clock=MutableClock(90.0), start_worker=False)
    try:
        node.start_streaming()
        node.submit_stream_snapshot(snapshot(89.8))
        ticket = node._stream_worker_ticket
        node.gate_audit_enabled = True
        node.gate_audit_output_path = str(tmp_path / 'planning.json')
        node.mujoco_audit_output_path = str(tmp_path / 'mujoco.json')
        node.gate_audit_pub = RecordingPublisher()
        node._stable_variant_runtime = {}
        node._active_gate_audit_report = {
            'marker': 'wrong-shared-report',
            'summary': {},
            'rows': [],
        }
        base_report = {
            'marker': 'request-local-report',
            'summary': {},
            'rows': [],
        }
        prepared = types.SimpleNamespace(ticket=ticket)

        node._finalize_streaming_gate_audit(
            prepared,
            None,
            {'stage_counts': {}, 'rejection_counts': {}},
            'STABILITY_PENDING',
            base_report=base_report,
        )

        written = json.loads(
            pathlib.Path(node.gate_audit_output_path).read_text()
        )
        assert written['marker'] == 'request-local-report'
        assert node._active_gate_audit_report['marker'] == (
            'request-local-report'
        )
        assert len(node.gate_audit_pub.messages) == 1
    finally:
        node.shutdown_streaming_worker()


def test_streaming_audit_keeps_selected_lineage_when_current_rows_are_empty(
    tmp_path,
):
    node = streaming_node(clock=MutableClock(95.0), start_worker=False)
    try:
        node.start_streaming()
        node.submit_stream_snapshot(snapshot(94.8))
        active_ticket = node._stream_worker_ticket
        ticket = InferenceTicket(
            request_id=4,
            generation=active_ticket.generation,
            snapshot_stamp_sec=94.8,
            target_epoch=active_ticket.target_epoch,
            payload=active_ticket.payload,
            submitted_monotonic_sec=active_ticket.submitted_monotonic_sec,
        )
        node.gate_audit_enabled = True
        node.gate_audit_output_path = str(tmp_path / 'planning.json')
        node.mujoco_audit_output_path = str(tmp_path / 'mujoco.json')
        node.gate_audit_pub = RecordingPublisher()
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
            position_dispersion_m=0.002,
            orientation_dispersion_rad=0.03,
        )
        payload = remote_node.LocalCandidatePayload(
            raw_candidate_index=4,
            variant_index=0,
            raw_candidate=None,
            camera_candidate=None,
            grasp_pose=None,
            geometry_gate=None,
            soft_features=features,
            score_components={},
        )
        stable = types.SimpleNamespace(
            request_id=3,
            snapshot_stamp_sec=93.3,
            hit_count=3,
            window_count=5,
            hit_request_ids=(1, 2, 3),
        )
        evaluated = types.SimpleNamespace(
            track_id=7,
            variant_index=1,
            payload=payload,
            stable_candidate=stable,
            soft_features=features,
            score_weights=SoftScoreWeights(),
            moveit_result=None,
            pre_moveit_score=0.0,
            final_score=None,
            evaluation_request_id=ticket.request_id,
            evaluation_snapshot_stamp_sec=ticket.snapshot_stamp_sec,
        )
        node._stable_variant_runtime = {
            (7, 1): {
                'prepared': types.SimpleNamespace(ticket=ticket),
                'scored_candidate': evaluated,
            }
        }
        selection = types.SimpleNamespace(
            checked=(evaluated,),
            selected=evaluated,
        )
        base_report = {'summary': {}, 'rows': []}
        prepared = types.SimpleNamespace(ticket=ticket)

        node._finalize_streaming_gate_audit(
            prepared,
            selection,
            {'stage_counts': {}, 'rejection_counts': {}},
            'PREVIEW_READY',
            base_report=base_report,
        )

        written = json.loads(
            pathlib.Path(node.gate_audit_output_path).read_text()
        )
        expected_binding = {
            'source_request_id': 3,
            'source_snapshot_stamp_sec': 93.3,
            'source_raw_candidate_index': 4,
            'source_variant_index': 0,
            'evaluation_request_id': ticket.request_id,
            'evaluation_snapshot_stamp_sec': ticket.snapshot_stamp_sec,
            'evaluation_variant_index': 1,
        }
        assert written['rows'] == []
        assert written['selected']['lineage_binding'] == expected_binding
        assert written['lineage'][0]['lineage_binding'] == expected_binding
        assert written['lineage'][0]['selected'] is True
    finally:
        node.shutdown_streaming_worker()


def test_current_raw_index_collision_cannot_rebind_old_stable_lineage(tmp_path):
    node = streaming_node(clock=MutableClock(96.0), start_worker=False)
    try:
        node.start_streaming()
        node.submit_stream_snapshot(snapshot(95.8))
        active_ticket = node._stream_worker_ticket
        ticket = InferenceTicket(
            request_id=4,
            generation=active_ticket.generation,
            snapshot_stamp_sec=95.8,
            target_epoch=active_ticket.target_epoch,
            payload=active_ticket.payload,
            submitted_monotonic_sec=active_ticket.submitted_monotonic_sec,
        )
        node.gate_audit_enabled = True
        node.gate_audit_output_path = str(tmp_path / 'planning.json')
        node.mujoco_audit_output_path = str(tmp_path / 'mujoco.json')
        node.gate_audit_pub = RecordingPublisher()
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
            position_dispersion_m=0.002,
            orientation_dispersion_rad=0.03,
        )
        payload = remote_node.LocalCandidatePayload(
            raw_candidate_index=0,
            variant_index=0,
            raw_candidate=None,
            camera_candidate=None,
            grasp_pose=None,
            geometry_gate=None,
            soft_features=features,
            score_components={},
        )
        stable = types.SimpleNamespace(
            request_id=3,
            snapshot_stamp_sec=93.3,
            hit_count=3,
            window_count=5,
            hit_request_ids=(1, 2, 3),
        )
        evaluated = types.SimpleNamespace(
            track_id=7,
            variant_index=1,
            payload=payload,
            stable_candidate=stable,
            soft_features=features,
            score_weights=SoftScoreWeights(),
            moveit_result=None,
            pre_moveit_score=0.0,
            final_score=None,
            evaluation_request_id=ticket.request_id,
            evaluation_snapshot_stamp_sec=ticket.snapshot_stamp_sec,
        )
        node._stable_variant_runtime = {
            (7, 1): {
                'prepared': types.SimpleNamespace(ticket=ticket),
                'scored_candidate': evaluated,
            }
        }
        selection = types.SimpleNamespace(
            checked=(evaluated,),
            selected=evaluated,
        )
        prepared = types.SimpleNamespace(ticket=ticket)

        node._finalize_streaming_gate_audit(
            prepared,
            selection,
            {'stage_counts': {}, 'rejection_counts': {}},
            'PREVIEW_READY',
            base_report={
                'summary': {},
                'rows': [{'candidate_index': 0, 'variant_index': 1}],
            },
        )

        written = json.loads(
            pathlib.Path(node.gate_audit_output_path).read_text()
        )
        assert written['selected']['lineage_binding'][
            'source_request_id'
        ] == 3
        assert written['rows'][0].get('tracking') is None
        assert written['rows'][0].get('lineage_binding') is None
        assert written['rows'][0]['selected'] is False
    finally:
        node.shutdown_streaming_worker()


@pytest.mark.parametrize(
    ('current_status', 'current_rows', 'failure_code'),
    [
        ('REMOTE_NO_CANDIDATES:1', [], 'REMOTE_NO_CANDIDATES'),
        (
            'DEPTH_INVALID:1',
            [{'candidate_index': 0, 'variant_index': 0}],
            'DEPTH_INVALID',
        ),
    ],
)
def test_zero_valid_audit_excludes_previous_preview_runtime(
    tmp_path,
    current_status,
    current_rows,
    failure_code,
):
    node = streaming_node(clock=MutableClock(96.0), start_worker=False)
    try:
        node.start_streaming()
        node.submit_stream_snapshot(snapshot(95.8))
        active = node._stream_worker_ticket
        source_ticket = InferenceTicket(
            request_id=3,
            generation=active.generation,
            snapshot_stamp_sec=95.3,
            target_epoch=active.target_epoch,
            payload=active.payload,
            submitted_monotonic_sec=active.submitted_monotonic_sec,
        )
        current_ticket = InferenceTicket(
            request_id=4,
            generation=active.generation,
            snapshot_stamp_sec=95.8,
            target_epoch=active.target_epoch,
            payload=active.payload,
            submitted_monotonic_sec=active.submitted_monotonic_sec,
        )
        node.gate_audit_enabled = True
        node.gate_audit_output_path = str(tmp_path / 'planning.json')
        node.mujoco_audit_output_path = str(tmp_path / 'mujoco.json')
        node.gate_audit_pub = RecordingPublisher()
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
            position_dispersion_m=0.002,
            orientation_dispersion_rad=0.03,
        )
        payload = remote_node.LocalCandidatePayload(
            raw_candidate_index=4,
            variant_index=0,
            raw_candidate=None,
            camera_candidate=None,
            grasp_pose=None,
            geometry_gate=None,
            soft_features=features,
            score_components={},
        )
        stable = types.SimpleNamespace(
            request_id=3,
            snapshot_stamp_sec=95.3,
            hit_count=3,
            window_count=5,
            hit_request_ids=(1, 2, 3),
        )
        evaluated = types.SimpleNamespace(
            track_id=7,
            variant_index=1,
            payload=payload,
            stable_candidate=stable,
            soft_features=features,
            score_weights=SoftScoreWeights(),
            moveit_result=None,
            pre_moveit_score=0.0,
            final_score=None,
            evaluation_request_id=3,
            evaluation_snapshot_stamp_sec=95.3,
        )
        node._stable_variant_runtime = {
            (7, 1): {
                'prepared': types.SimpleNamespace(ticket=source_ticket),
                'scored_candidate': evaluated,
            }
        }
        source_selection = types.SimpleNamespace(
            checked=(evaluated,),
            selected=evaluated,
        )
        node._finalize_streaming_gate_audit(
            types.SimpleNamespace(ticket=source_ticket),
            source_selection,
            {'stage_counts': {}, 'rejection_counts': {}},
            'PREVIEW_READY',
            base_report={
                'summary': {},
                'rows': [{'candidate_index': 4, 'variant_index': 1}],
            },
        )
        previous = json.loads(
            pathlib.Path(node.gate_audit_output_path).read_text()
        )
        assert previous['selected']['lineage_binding'][
            'evaluation_request_id'
        ] == 3

        node._finalize_streaming_gate_audit(
            types.SimpleNamespace(ticket=current_ticket),
            None,
            {
                'stage_counts': {
                    'locally_valid': {
                        'entered': len(current_rows),
                        'passed': 0,
                        'rejected': len(current_rows),
                    }
                },
                'rejection_counts': {failure_code: 1},
                'primary_failure': failure_code,
            },
            current_status,
            base_report={'summary': {}, 'rows': current_rows},
        )

        current = json.loads(
            pathlib.Path(node.gate_audit_output_path).read_text()
        )
        assert current['request_id'] == 4
        assert current['stable_evaluations'] == []
        assert current['selected'] is None
        assert all(
            item.get('lineage_binding') is None
            for item in current['candidate_row_lineage']
        )
    finally:
        node.shutdown_streaming_worker()


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
    node._stream_condition = threading.Condition(threading.RLock())
    node._stream_shutdown = threading.Event()
    node.streaming_enabled = True
    node._stream_generation = 1
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
    node._camera_candidate_cloud_distance = lambda _candidate: 0.007
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
        position_dispersion_m=0.002,
        orientation_dispersion_rad=0.03,
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
    assert all(
        item.soft_features.cloud_distance_m == pytest.approx(0.007)
        for item in scored
    )
    assert all(
        item.soft_features.position_dispersion_m == pytest.approx(0.002)
        for item in scored
    )
    assert all(
        item.soft_features.orientation_dispersion_rad == pytest.approx(0.03)
        for item in scored
    )


def promotion_plan(
    plan_id,
    x=0.1,
    target_x=0.2,
    quaternion=(0.0, 0.0, 0.0, 1.0),
):
    pose = types.SimpleNamespace(
        position=types.SimpleNamespace(x=float(x), y=0.0, z=0.3),
        orientation=types.SimpleNamespace(
            x=float(quaternion[0]),
            y=float(quaternion[1]),
            z=float(quaternion[2]),
            w=float(quaternion[3]),
        ),
    )
    geometry_pose = types.SimpleNamespace(
        position=types.SimpleNamespace(
            x=float(target_x), y=0.0, z=0.1
        ),
        orientation=types.SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
    )
    return types.SimpleNamespace(
        valid=True,
        plan_id=str(plan_id),
        header=types.SimpleNamespace(frame_id='base_link', stamp=20.0),
        poses=[pose, pose, pose, pose],
        object_geometry=types.SimpleNamespace(pose_base=geometry_pose),
    )


def promotion_node(clock=None):
    node = remote_node.RemoteGrasp6DNode.__new__(remote_node.RemoteGrasp6DNode)
    node._geometry_state_lock = threading.RLock()
    node._geometry_invalidation_generation = 0
    node._last_geometry_invalidation_code = ''
    node.robot_execution_active = False
    node.execution_plan_controller = ExecutionPlanController(
        replan_cooldown_sec=1.0,
        selection_hysteresis_ratio=0.12,
        candidate_consecutive_invalidations=2,
        replan_position_delta_m=0.012,
        replan_orientation_delta_deg=12.0,
        replan_target_drift_m=0.025,
    )
    node._stream_source_clock = clock or MutableClock(10.0)
    node.plan_pub = RecordingPublisher()
    node.rich_plan_pub = RecordingPublisher()
    node.preview_plan_pub = RecordingPublisher()
    node.preview_rich_plan_pub = RecordingPublisher()
    node.latest_rich_plan = None
    node.latest_plan = None
    node.latest_preview_rich_plan = None
    node._execution_promotion_audit_ready = lambda _plan: True
    return node


def promotion_transaction_node(tmp_path):
    node = streaming_node(clock=MutableClock(10.0), start_worker=False)
    node._geometry_state_lock = threading.RLock()
    node._geometry_invalidation_generation = 0
    node._last_geometry_invalidation_code = ''
    node.robot_execution_active = False
    node.execution_plan_controller = ExecutionPlanController()
    node.plan_pub = RecordingPublisher()
    node.rich_plan_pub = RecordingPublisher()
    node.latest_rich_plan = None
    node.latest_plan = None
    node.gate_audit_enabled = True
    node.gate_audit_output_path = str(tmp_path / 'planning.json')
    node.mujoco_audit_output_path = str(tmp_path / 'mujoco.json')
    node.gate_audit_pub = RecordingPublisher()
    node._stable_variant_runtime = {}
    node.start_streaming()
    node.submit_stream_snapshot(snapshot(9.8))
    ticket = node._stream_worker_ticket
    plan = promotion_plan('plan-A')
    node._latest_streaming_audit = {
        'request_id': ticket.request_id,
        'plan_id': plan.plan_id,
    }
    proposal = {
        'rich_plan': plan,
        'signature': 'target:4:carton:carton_segment',
        'score': 1.0,
        'ticket': ticket,
        'expected_generation': 0,
    }
    prepared = types.SimpleNamespace(ticket=ticket)
    selection = types.SimpleNamespace(checked=(), selected=object())
    local_funnel = {
        'input_count': 1,
        'stage_counts': {
            'locally_valid': {'entered': 1, 'passed': 1, 'rejected': 0}
        },
        'rejection_counts': {},
    }
    moveit_funnel = {
        'stage_counts': {
            'moveit_reachable': {'entered': 1, 'passed': 1, 'rejected': 0}
        },
        'rejection_counts': {},
    }
    return (
        node,
        prepared,
        selection,
        proposal,
        local_funnel,
        moveit_funnel,
    )


def test_initial_preview_promotion_publishes_execution_once():
    node = promotion_node()
    preview = promotion_plan('plan-A')

    first = node._maybe_promote_preview(
        preview, signature='candidate-A', score=1.0
    )
    second = node._maybe_promote_preview(
        preview, signature='candidate-A', score=1.0
    )

    assert first.promote is True
    assert second.promote is False
    assert [item.plan_id for item in node.rich_plan_pub.messages] == ['plan-A']
    assert len(node.plan_pub.messages) == 1
    assert node.latest_rich_plan.plan_id == 'plan-A'
    assert node.execution_plan_controller.execution_plan_id == 'plan-A'


def test_active_execution_keeps_authority_while_new_preview_is_visible():
    node = promotion_node()
    first = promotion_plan('plan-A')
    challenger = promotion_plan('preview-B', x=0.2)
    node._maybe_promote_preview(first, signature='candidate-A', score=1.0)
    node.grasp_state_cb(types.SimpleNamespace(active=True))

    node._publish_preview_plan(
        challenger,
        remote_node.rich_plan_to_legacy(challenger),
    )
    decision = node._maybe_promote_preview(
        challenger, signature='candidate-B', score=0.5
    )

    assert decision.code == 'EXECUTION_FROZEN'
    assert [item.plan_id for item in node.preview_rich_plan_pub.messages] == [
        'preview-B'
    ]
    assert [item.plan_id for item in node.rich_plan_pub.messages] == ['plan-A']
    assert node.latest_rich_plan.plan_id == 'plan-A'


def test_execution_publication_failure_does_not_commit_controller():
    node = promotion_node()
    node.plan_pub = FailingPublisher()

    decision = node._maybe_promote_preview(
        promotion_plan('plan-A'), signature='candidate-A', score=1.0
    )

    assert decision.promote is False
    assert decision.code == 'PLAN_PUBLICATION_FAILED'
    assert node.execution_plan_controller.execution_plan_id is None
    assert node.rich_plan_pub.messages == []
    assert node.latest_rich_plan is None


def test_execution_promotion_without_bound_final_audit_is_rejected():
    node = promotion_node()
    node._execution_promotion_audit_ready = lambda _plan: False

    decision = node._maybe_promote_preview(
        promotion_plan('plan-A'), signature='candidate-A', score=1.0
    )

    assert decision.promote is False
    assert decision.code == 'PLAN_AUDIT_NOT_READY'
    assert node.rich_plan_pub.messages == []
    assert node.plan_pub.messages == []
    assert node.execution_plan_controller.execution_plan_id is None


def test_final_audit_is_bound_before_execution_authority_publish(tmp_path):
    setup = promotion_transaction_node(tmp_path)
    node, prepared, selection, proposal, local_funnel, moveit_funnel = setup
    audit_path = pathlib.Path(node.gate_audit_output_path)

    class AuditBeforeRichPublisher(RecordingPublisher):
        def publish(self, message):
            assert audit_path.is_file()
            report = json.loads(audit_path.read_text())
            assert report['plan_id'] == message.plan_id
            assert report['outcome']['valid_plan'] is True
            assert report['promotion']['promote'] is True
            assert report['pipeline_funnel']['stage_counts']['promoted'][
                'passed'
            ] == 1
            super().publish(message)

    node.rich_plan_pub = AuditBeforeRichPublisher()
    try:
        funnel, decision = node._finalize_promotion_transaction(
            prepared,
            selection,
            proposal,
            local_funnel,
            moveit_funnel,
            1,
            'PREVIEW_READY',
            {'summary': {}, 'rows': []},
        )

        assert decision.promote is True
        assert funnel['stage_counts']['promoted']['passed'] == 1
        assert node.execution_plan_controller.execution_plan_id == 'plan-A'
        assert [item.plan_id for item in node.rich_plan_pub.messages] == [
            'plan-A'
        ]
    finally:
        node.shutdown_streaming_worker()


def test_failed_execution_publish_rewrites_audit_as_unpublished(tmp_path):
    setup = promotion_transaction_node(tmp_path)
    node, prepared, selection, proposal, local_funnel, moveit_funnel = setup
    node.plan_pub = FailingPublisher()
    try:
        funnel, decision = node._finalize_promotion_transaction(
            prepared,
            selection,
            proposal,
            local_funnel,
            moveit_funnel,
            1,
            'PREVIEW_READY',
            {'summary': {}, 'rows': []},
        )

        report = json.loads(
            pathlib.Path(node.gate_audit_output_path).read_text()
        )
        assert decision.code == 'PLAN_PUBLICATION_FAILED'
        assert report['outcome']['valid_plan'] is False
        assert report['outcome']['code'] == 'PLAN_PUBLICATION_FAILED'
        assert report['promotion']['code'] == 'PLAN_PUBLICATION_FAILED'
        assert funnel['stage_counts']['promoted']['passed'] == 0
        assert node.execution_plan_controller.execution_plan_id is None
        assert node.rich_plan_pub.messages == []
    finally:
        node.shutdown_streaming_worker()


def test_generation_rejection_rewrites_audit_as_unpublished(tmp_path):
    setup = promotion_transaction_node(tmp_path)
    node, prepared, selection, proposal, local_funnel, moveit_funnel = setup
    proposal['expected_generation'] = 1
    try:
        funnel, decision = node._finalize_promotion_transaction(
            prepared,
            selection,
            proposal,
            local_funnel,
            moveit_funnel,
            1,
            'PREVIEW_READY',
            {'summary': {}, 'rows': []},
        )

        report = json.loads(
            pathlib.Path(node.gate_audit_output_path).read_text()
        )
        assert decision.code == 'PLAN_PUBLICATION_FAILED'
        assert report['outcome']['valid_plan'] is False
        assert funnel['stage_counts']['promoted']['passed'] == 0
        assert node.execution_plan_controller.execution_plan_id is None
        assert node.plan_pub.messages == []
        assert node.rich_plan_pub.messages == []
    finally:
        node.shutdown_streaming_worker()


def test_blocked_audit_io_does_not_block_stream_stop_or_publish_execution(
    tmp_path, monkeypatch
):
    setup = promotion_transaction_node(tmp_path)
    node, prepared, selection, proposal, local_funnel, moveit_funnel = setup
    fsync_entered = threading.Event()
    release_fsync = threading.Event()
    original_fsync = remote_node.os.fsync

    def blocking_fsync(file_descriptor):
        fsync_entered.set()
        if not release_fsync.wait(2.0):
            raise RuntimeError('test did not release audit fsync')
        return original_fsync(file_descriptor)

    monkeypatch.setattr(remote_node.os, 'fsync', blocking_fsync)
    transaction_errors = []

    def run_transaction():
        try:
            node._finalize_promotion_transaction(
                prepared,
                selection,
                proposal,
                local_funnel,
                moveit_funnel,
                1,
                'PREVIEW_READY',
                {'summary': {}, 'rows': []},
            )
        except Exception as exc:
            transaction_errors.append(exc)

    transaction = threading.Thread(target=run_transaction)
    transaction.start()
    assert fsync_entered.wait(1.0)
    stop_done = threading.Event()
    stop_thread = threading.Thread(
        target=lambda: (node.stop_streaming(), stop_done.set())
    )
    stop_thread.start()
    try:
        assert stop_done.wait(0.2), 'stop_streaming blocked on audit fsync'
    finally:
        release_fsync.set()
        transaction.join(2.0)
        stop_thread.join(2.0)
        node.shutdown_streaming_worker()

    assert any(
        isinstance(exc, remote_node.StreamResultCancelled)
        for exc in transaction_errors
    )
    assert node.rich_plan_pub.messages == []
    assert node.execution_plan_controller.execution_plan_id is None


def test_failed_publish_correction_survives_stop_during_correction_io(
    tmp_path, monkeypatch
):
    setup = promotion_transaction_node(tmp_path)
    node, prepared, selection, proposal, local_funnel, moveit_funnel = setup
    node.plan_pub = FailingPublisher()
    second_fsync_entered = threading.Event()
    release_second_fsync = threading.Event()
    original_fsync = remote_node.os.fsync
    fsync_calls = {'count': 0}

    def block_second_fsync(file_descriptor):
        fsync_calls['count'] += 1
        if fsync_calls['count'] == 2:
            second_fsync_entered.set()
            if not release_second_fsync.wait(2.0):
                raise RuntimeError('test did not release correction fsync')
        return original_fsync(file_descriptor)

    monkeypatch.setattr(remote_node.os, 'fsync', block_second_fsync)
    transaction_errors = []

    def run_transaction():
        try:
            node._finalize_promotion_transaction(
                prepared,
                selection,
                proposal,
                local_funnel,
                moveit_funnel,
                1,
                'PREVIEW_READY',
                {'summary': {}, 'rows': []},
            )
        except Exception as exc:
            transaction_errors.append(exc)

    transaction = threading.Thread(target=run_transaction)
    transaction.start()
    assert second_fsync_entered.wait(1.0)
    assert node.stop_streaming() is True
    release_second_fsync.set()
    transaction.join(2.0)
    try:
        report = json.loads(
            pathlib.Path(node.gate_audit_output_path).read_text()
        )
        assert transaction_errors == []
        assert report['outcome']['valid_plan'] is False
        assert report['outcome']['code'] == 'PLAN_PUBLICATION_FAILED'
        assert node.execution_plan_controller.execution_plan_id is None
        assert node.rich_plan_pub.messages == []
    finally:
        node.shutdown_streaming_worker()


def test_replan_service_rejects_false_and_active_without_touching_streaming():
    node = promotion_node()
    node.streaming_enabled = True
    node._maybe_promote_preview(
        promotion_plan('plan-A'), signature='candidate-A', score=1.0
    )

    false_request = node.replan_execution_cb(
        types.SimpleNamespace(trigger=False)
    )
    node.grasp_state_cb(types.SimpleNamespace(active=True))
    active_request = node.replan_execution_cb(
        types.SimpleNamespace(trigger=True)
    )

    assert false_request.success is False
    assert active_request.success is False
    assert node.execution_plan_controller.explicit_replan_requested is False
    assert node.streaming_enabled is True


def test_replan_service_idle_request_allows_better_preview_after_cooldown():
    clock = MutableClock(10.0)
    node = promotion_node(clock=clock)
    node._maybe_promote_preview(
        promotion_plan('plan-A'), signature='candidate-A', score=1.0
    )
    clock.value = 11.1

    response = node.replan_execution_cb(types.SimpleNamespace(trigger=True))
    decision = node._maybe_promote_preview(
        promotion_plan('plan-B', x=0.15),
        signature='candidate-B',
        score=0.5,
    )

    assert response.success is True
    assert decision.promote is True
    assert [item.plan_id for item in node.rich_plan_pub.messages] == [
        'plan-A',
        'plan-B',
    ]
    assert node.execution_plan_controller.execution_plan_id == 'plan-B'


def test_preview_failure_only_updates_invalid_streak_not_execution_topics():
    node = promotion_node()
    node._maybe_promote_preview(
        promotion_plan('plan-A'), signature='candidate-A', score=1.0
    )

    first = node._observe_execution_candidate_invalid(now_sec=10.1)
    second = node._observe_execution_candidate_invalid(now_sec=10.2)

    assert first.invalidate is False
    assert second.invalidate is True
    assert [item.plan_id for item in node.rich_plan_pub.messages] == ['plan-A']
    assert len(node.plan_pub.messages) == 1
    assert node.latest_rich_plan.plan_id == 'plan-A'


def test_stale_ticket_cannot_mutate_execution_invalid_streak():
    node = streaming_node(clock=MutableClock(10.0), start_worker=False)
    try:
        node._geometry_state_lock = threading.RLock()
        node.robot_execution_active = False
        node.execution_plan_controller = ExecutionPlanController()
        node.execution_plan_controller.commit_execution(
            'plan-A', 'candidate-A', score=1.0, now_sec=9.0
        )
        node.start_streaming()
        node.submit_stream_snapshot(snapshot(9.8))
        ticket = node._stream_worker_ticket
        node.stop_streaming()

        with pytest.raises(remote_node.StreamResultCancelled):
            node._observe_execution_candidate_invalid(
                now_sec=10.1,
                ticket=ticket,
            )

        assert node.execution_plan_controller.invalid_streak == 0
    finally:
        node.shutdown_streaming_worker()


def test_stale_ticket_cannot_append_promotion_to_streaming_audit():
    node = streaming_node(clock=MutableClock(10.0), start_worker=False)
    try:
        node.start_streaming()
        node.submit_stream_snapshot(snapshot(9.8))
        ticket = node._stream_worker_ticket
        node._latest_streaming_audit = {'plan_id': 'preview-A'}
        node.stop_streaming()

        with pytest.raises(remote_node.StreamResultCancelled):
            node._record_preview_promotion(
                ticket,
                PromotionDecision(
                    False,
                    'EXECUTION_HELD',
                    'a valid execution plan already exists',
                ),
            )

        assert node._latest_streaming_audit == {'plan_id': 'preview-A'}
    finally:
        node.shutdown_streaming_worker()


def test_target_signature_ignores_track_and_parallel_jaw_variant_identity():
    first = types.SimpleNamespace(
        target_epoch=4,
        target_label='carton',
        model_choice='carton_segment',
        track_id=1,
    )
    rebuilt = types.SimpleNamespace(
        target_epoch=4,
        target_label='carton',
        model_choice='carton_segment',
        track_id=99,
    )

    assert remote_node.RemoteGrasp6DNode._preview_target_signature(
        first
    ) == remote_node.RemoteGrasp6DNode._preview_target_signature(rebuilt)


def test_recovered_equivalent_preview_breaks_nonconsecutive_invalid_streak():
    node = promotion_node()
    signature = 'target:4:carton:carton_segment'
    node._maybe_promote_preview(
        promotion_plan('plan-A'), signature=signature, score=1.0
    )

    node._observe_execution_candidate_invalid(now_sec=10.1)
    recovered = node._maybe_promote_preview(
        promotion_plan('preview-A2', x=0.105, target_x=0.205),
        signature=signature,
        score=0.5,
    )
    node._observe_execution_candidate_invalid(now_sec=10.2)

    assert recovered.code == 'EXECUTION_HELD'
    assert node.execution_plan_controller.invalid_streak == 1
    assert node.execution_plan_controller.replacement_authorized is False
    assert [item.plan_id for item in node.rich_plan_pub.messages] == ['plan-A']


def test_pipeline_funnel_records_successful_execution_promotion():
    merged = remote_node.RemoteGrasp6DNode._merge_pipeline_funnel(
        {
            'input_count': 1,
            'stage_counts': {},
            'rejection_counts': {},
            'rejection_ratios': {},
        },
        stable_count=1,
        preview_count=1,
        promotion_count=1,
    )

    assert merged['stage_counts']['promoted'] == {
        'entered': 1,
        'passed': 1,
        'rejected': 0,
    }


@pytest.mark.parametrize(
    'challenger',
    [
        promotion_plan('plan-B-position', x=0.113),
        promotion_plan(
            'plan-B-orientation',
            quaternion=(
                math.sin(math.radians(13.0) / 2.0),
                0.0,
                0.0,
                math.cos(math.radians(13.0) / 2.0),
            ),
        ),
        promotion_plan('plan-B-target', target_x=0.226),
    ],
)
def test_same_target_drift_authorizes_idle_replan(challenger):
    clock = MutableClock(10.0)
    node = promotion_node(clock=clock)
    signature = 'target:4:carton:carton_segment'
    node._maybe_promote_preview(
        promotion_plan('plan-A'), signature=signature, score=1.0
    )
    clock.value = 11.1

    decision = node._maybe_promote_preview(
        challenger,
        signature=signature,
        score=0.5,
    )

    assert decision.promote is True
    assert decision.code == 'PROMOTE_REPLAN'
    assert len(node.rich_plan_pub.messages) == 2


def test_parallel_jaw_half_turn_is_zero_orientation_drift():
    current = promotion_plan('plan-A')
    symmetric = promotion_plan(
        'preview-symmetric', quaternion=(0.0, 0.0, 1.0, 0.0)
    )

    _position, orientation, _target = (
        remote_node.RemoteGrasp6DNode._execution_plan_drift(
            current, symmetric
        )
    )

    assert orientation == pytest.approx(0.0, abs=1e-9)


def test_request_local_promotion_count_ignores_unrelated_decision():
    node = streaming_node(clock=MutableClock(10.0), start_worker=False)
    try:
        node.start_streaming()
        node.submit_stream_snapshot(snapshot(9.8))
        ticket = node._stream_worker_ticket
        promoted = PromotionDecision(
            True,
            'PROMOTE_INITIAL',
            'no valid execution plan exists',
        )
        node._latest_streaming_audit = {'plan_id': 'preview-A'}
        node._record_preview_promotion(ticket, promoted)
        node._latest_promotion_decision = PromotionDecision(
            False,
            'EXECUTION_FROZEN',
            'robot execution is active',
        )

        assert node._request_promotion_count(ticket) == 1
    finally:
        node.shutdown_streaming_worker()
