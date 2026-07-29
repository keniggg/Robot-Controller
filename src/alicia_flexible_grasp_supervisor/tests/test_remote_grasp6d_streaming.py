#!/usr/bin/env python3
import importlib.util
import json
import math
import pathlib
import sys
import threading
import time
import types

import numpy as np
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
    BoundedMoveItSelection,
    CandidateStageFunnel,
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
    GripperGeometry,
    evaluate_open_gripper_observation_envelope,
)
from alicia_flexible_grasp.grasp.tabletop_geometry_candidates import (  # noqa: E402
    TabletopGeometryConfig,
    generate_tabletop_proposals,
    materialize_tabletop_candidates,
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


class BlockingPublisher(RecordingPublisher):
    def __init__(self, should_block):
        super().__init__()
        self.should_block = should_block
        self.entered = threading.Event()
        self.release = threading.Event()
        self._blocked_once = False

    def publish(self, message):
        if not self._blocked_once and self.should_block(message):
            self._blocked_once = True
            self.entered.set()
            if not self.release.wait(2.0):
                raise RuntimeError('test did not release blocked publisher')
        super().publish(message)


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
    node.robot_execution_active = False
    node.near_field_planning_active = False
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


def test_explicit_near_field_phase_resets_tracker_and_invalidates_prior_epoch():
    node = streaming_node(start_worker=False)
    assert node.start_streaming() is True
    original_epoch = node.target_instance_epoch
    original_tracker = node.tracker
    node._stable_variant_runtime = {(7, 1): {'phase': 'far_field'}}

    node.grasp_state_cb(types.SimpleNamespace(active=True))

    assert node.robot_execution_active is True
    assert node.near_field_planning_active is False
    assert node.target_instance_epoch == original_epoch
    assert node.tracker is original_tracker

    node.near_field_state_cb(remote_node.Bool(data=True))

    assert node.near_field_planning_active is True
    assert node.target_instance_epoch == original_epoch + 1
    assert node.tracker is not original_tracker
    assert node._stable_variant_runtime == {}

    near_field_epoch = node.target_instance_epoch
    near_field_tracker = node.tracker
    node.near_field_state_cb(remote_node.Bool(data=True))

    assert node.target_instance_epoch == near_field_epoch
    assert node.tracker is near_field_tracker

    node.near_field_state_cb(remote_node.Bool(data=False))

    assert node.near_field_planning_active is False
    assert node.target_instance_epoch == near_field_epoch + 1
    node.shutdown_streaming_worker()


def seed_stream_execution_authority(node, authority_ticket=None):
    node._geometry_state_lock = threading.RLock()
    node.execution_plan_controller = ExecutionPlanController()
    node.execution_plan_controller.commit_execution(
        'old-plan',
        'old-signature',
        score=0.1,
        now_sec=10.0,
    )
    node.latest_rich_plan = types.SimpleNamespace(
        valid=True,
        plan_id='old-plan',
    )
    node.latest_plan = types.SimpleNamespace(poses=('old-pose',))
    node.rich_plan_pub = RecordingPublisher()
    node.plan_pub = RecordingPublisher()
    node._active_execution_audit_report = {
        'plan_id': 'old-plan',
        'outcome': {'valid_plan': True},
    }
    node._latest_execution_audit_reference = {
        'report_sha256': 'old-audit',
        'atomic_committed': True,
    }
    if authority_ticket is not None:
        node._execution_authority_ticket = (
            int(authority_ticket.request_id),
            int(authority_ticket.generation),
            int(authority_ticket.target_epoch),
            float(authority_ticket.snapshot_stamp_sec),
        )


def assert_execution_authority_revoked(node, failure_code):
    assert node.execution_plan_controller.execution_plan_id is None
    assert node.latest_rich_plan is None
    assert node.latest_plan is None
    assert node._active_execution_audit_report is None
    assert node._latest_execution_audit_reference == {}
    assert node.rich_plan_pub.messages[-1].valid is False
    assert failure_code in node.rich_plan_pub.messages[-1].diagnostic
    assert node.plan_pub.messages[-1].poses == []


def assert_execution_authority_retained(node, plan_id='old-plan'):
    assert node.execution_plan_controller.execution_plan_id == plan_id
    assert node.latest_rich_plan.plan_id == plan_id
    assert node.latest_plan.poses == ('old-pose',)
    assert node._active_execution_audit_report['plan_id'] == plan_id
    assert node._latest_execution_audit_reference['report_sha256'] == 'old-audit'
    assert not node.rich_plan_pub.messages
    assert not node.plan_pub.messages


def wait_until(predicate, timeout=2.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.005)
    raise AssertionError('condition did not become true')


def continuous_runtime_values(**overrides):
    values = {
        'request_hz': 1.5,
        'result_max_age_sec': 1.2,
        'stability_window_size': 5,
        'stability_min_hits': 3,
        'tracking_position_threshold_m': 0.025,
        'tracking_orientation_threshold_deg': 25.0,
        'tracking_approach_threshold_deg': 20.0,
        'tracking_width_threshold_m': 0.008,
        'target_instance_association_threshold_m': 0.08,
        'target_absolute_sanity_distance_m': 0.15,
        'moveit_top_n': 5,
        'replan_position_delta_m': 0.012,
        'replan_orientation_delta_deg': 12.0,
        'replan_target_drift_m': 0.025,
        'replan_cooldown_sec': 1.0,
        'selection_hysteresis_ratio': 0.12,
        'candidate_consecutive_invalidations': 2,
        'performance_window_size': 100,
    }
    values.update(overrides)
    return values


def tabletop_box_cloud(size_xyz):
    xs = np.linspace(-size_xyz[0] / 2.0, size_xyz[0] / 2.0, 21)
    ys = np.linspace(-size_xyz[1] / 2.0, size_xyz[1] / 2.0, 17)
    zs = np.linspace(0.0, size_xyz[2], 7)
    points = []
    for x_value in xs:
        for z_value in zs:
            points.extend(
                (
                    (x_value, -size_xyz[1] / 2.0, z_value),
                    (x_value, size_xyz[1] / 2.0, z_value),
                )
            )
    for y_value in ys:
        for z_value in zs:
            points.extend(
                (
                    (-size_xyz[0] / 2.0, y_value, z_value),
                    (size_xyz[0] / 2.0, y_value, z_value),
                )
            )
    return np.asarray(points, dtype=float)


def tabletop_geometry(size_xyz=(0.051, 0.035, 0.011)):
    size = np.asarray(size_xyz, dtype=float)
    return types.SimpleNamespace(
        ok=True,
        center_base=np.asarray([0.0, 0.0, size[2] / 2.0]),
        axes_base=np.eye(3),
        size_xyz_m=size,
        support_normal_base=np.asarray([0.0, 0.0, 1.0]),
        support_offset_m=0.0,
        object_points_base=tabletop_box_cloud(size),
    )


def tabletop_gripper():
    return GripperGeometry(
        max_inner_gap_m=0.050,
        jaw_clearance_each_side_m=0.002,
        finger_size_xyz_m=np.asarray([0.0434, 0.0286, 0.0600]),
        palm_size_xyz_m=np.asarray([0.1175, 0.1550, 0.0774]),
        support_clearance_m=0.003,
    )


def configure_identity_handeye(node):
    node.handeye_translation_xyz = (0.0, 0.0, 0.0)
    node.handeye_rotation_xyzw = (0.0, 0.0, 0.0, 1.0)
    node.camera_visibility_min_depth_m = 0.035
    node.camera_visibility_max_depth_m = 1.20


def tabletop_candidates_for(geometry):
    result = generate_tabletop_proposals(
        object_points_base=geometry.object_points_base,
        obb_center_base=geometry.center_base,
        R_base_obb=geometry.axes_base,
        obb_size_xyz_m=geometry.size_xyz_m,
        support_point_base=np.zeros(3),
        support_normal_base=geometry.support_normal_base,
        config=TabletopGeometryConfig(max_candidates=8),
    )
    return tuple(
        candidate
        for proposal in result.proposals
        for candidate in materialize_tabletop_candidates(
            proposal,
            np.zeros(3),
            geometry.support_normal_base,
            tabletop_gripper(),
        )
    )[:8]


@pytest.mark.parametrize(
    ('field', 'bad_value'),
    [
        ('request_hz', True),
        ('request_hz', '2.0'),
        ('request_hz', float('nan')),
        ('request_hz', float('inf')),
        ('request_hz', 0.0),
        ('request_hz', 0.09),
        ('request_hz', 5.01),
        ('result_max_age_sec', False),
        ('result_max_age_sec', 0.0),
        ('tracking_position_threshold_m', -0.01),
        ('tracking_orientation_threshold_deg', 181.0),
        ('tracking_approach_threshold_deg', 0.0),
        ('tracking_width_threshold_m', float('-inf')),
        ('target_instance_association_threshold_m', -0.001),
        ('target_absolute_sanity_distance_m', 0.0),
        ('target_absolute_sanity_distance_m', 0.0009),
        ('replan_position_delta_m', -0.001),
        ('replan_orientation_delta_deg', 181.0),
        ('replan_target_drift_m', float('nan')),
        ('replan_cooldown_sec', -0.1),
        ('selection_hysteresis_ratio', 1.01),
        ('candidate_consecutive_invalidations', True),
        ('candidate_consecutive_invalidations', 2.0),
        ('candidate_consecutive_invalidations', 0),
        ('performance_window_size', False),
        ('performance_window_size', 4.0),
        ('performance_window_size', 0),
    ],
)
def test_continuous_runtime_config_rejects_nonfinite_wrong_type_and_bounds(
    field, bad_value
):
    with pytest.raises(remote_node.CandidateContractError) as raised:
        remote_node.validate_continuous_runtime_config(
            continuous_runtime_values(**{field: bad_value})
        )

    assert raised.value.code == 'CONTINUOUS_CONFIG_INVALID'
    assert field in str(raised.value)


@pytest.mark.parametrize(
    ('field', 'bad_value'),
    [
        ('stability_window_size', 4),
        ('stability_window_size', 6),
        ('stability_window_size', 5.0),
        ('stability_min_hits', 2),
        ('stability_min_hits', 4),
        ('stability_min_hits', 3.0),
    ],
)
def test_continuous_runtime_config_enforces_production_stability_contract(
    field, bad_value
):
    with pytest.raises(remote_node.CandidateContractError) as raised:
        remote_node.validate_continuous_runtime_config(
            continuous_runtime_values(**{field: bad_value})
        )

    assert raised.value.code == 'CONTINUOUS_CONFIG_INVALID'
    assert 'window_size=5' in str(raised.value)
    assert 'min_hits=3' in str(raised.value)


@pytest.mark.parametrize(('raw', 'expected'), [(1, 3), (5, 5), (99, 24)])
def test_continuous_runtime_config_strict_integer_moveit_top_n_clamps(
    raw, expected
):
    config = remote_node.validate_continuous_runtime_config(
        continuous_runtime_values(moveit_top_n=raw)
    )
    assert config.moveit_top_n == expected


@pytest.mark.parametrize('bad_value', [True, 5.0, '5', float('nan')])
def test_continuous_runtime_config_rejects_noninteger_moveit_top_n(bad_value):
    with pytest.raises(remote_node.CandidateContractError):
        remote_node.validate_continuous_runtime_config(
            continuous_runtime_values(moveit_top_n=bad_value)
        )


def test_runtime_continuous_apply_is_atomic_and_preserves_execution_state():
    node = streaming_node(clock=MutableClock(10.0), start_worker=False)
    node._geometry_state_lock = threading.RLock()
    node.rate_hz = 1.5
    node.target_instance_association_threshold_m = 0.08
    node.target_absolute_sanity_distance_m = 0.15
    node.moveit_top_n = 5
    node.execution_plan_controller = ExecutionPlanController()
    node.execution_plan_controller.commit_execution(
        'plan-A', 'target-A', score=1.0, now_sec=5.0
    )
    node.execution_plan_controller.invalid_streak = 1
    node.execution_plan_controller.explicit_replan_requested = True
    node.execution_plan_controller._drift_replan_requested = True
    old_controller = node.execution_plan_controller
    old_tracker = node.tracker
    node._stable_variant_runtime = {'history': object()}
    old_runtime = node._stable_variant_runtime
    config = remote_node.validate_continuous_runtime_config(
        continuous_runtime_values(
            request_hz=2.0,
            result_max_age_sec=0.9,
            target_instance_association_threshold_m=0.06,
            target_absolute_sanity_distance_m=0.12,
            moveit_top_n=8,
            replan_position_delta_m=0.02,
            replan_orientation_delta_deg=15.0,
            replan_target_drift_m=0.03,
            replan_cooldown_sec=1.5,
            selection_hysteresis_ratio=0.2,
            candidate_consecutive_invalidations=4,
            performance_window_size=7,
        )
    )

    node._apply_continuous_runtime_config(config)

    assert node.rate_hz == pytest.approx(2.0)
    assert node.inference_coordinator._result_max_age_sec == pytest.approx(0.9)
    assert node.moveit_top_n == 8
    assert node.target_instance_association_threshold_m == pytest.approx(0.06)
    assert node.target_absolute_sanity_distance_m == pytest.approx(0.12)
    assert node.pipeline_metrics.maxlen == 7
    assert node._latency_history_ms.maxlen == 7
    assert node.tracker is old_tracker
    assert node._stable_variant_runtime is old_runtime
    assert node.execution_plan_controller is old_controller
    assert old_controller.execution_plan_id == 'plan-A'
    assert old_controller.execution_signature == 'target-A'
    assert old_controller.execution_score == pytest.approx(1.0)
    assert old_controller.invalid_streak == 1
    assert old_controller.explicit_replan_requested is True
    assert old_controller._drift_replan_requested is True
    assert old_controller.last_promotion_sec == pytest.approx(5.0)
    assert old_controller.replan_position_delta_m == pytest.approx(0.02)
    assert old_controller.replan_orientation_delta_deg == pytest.approx(15.0)
    assert old_controller.replan_target_drift_m == pytest.approx(0.03)
    assert old_controller.replan_cooldown_sec == pytest.approx(1.5)
    assert old_controller.selection_hysteresis_ratio == pytest.approx(0.2)
    assert old_controller.candidate_consecutive_invalidations == 4


def test_tracking_threshold_change_clears_only_tracking_history_under_lock():
    node = streaming_node(clock=MutableClock(10.0), start_worker=False)
    node._geometry_state_lock = threading.RLock()
    node.rate_hz = 1.5
    node.target_instance_association_threshold_m = 0.08
    node.target_absolute_sanity_distance_m = 0.15
    node.moveit_top_n = 5
    node.execution_plan_controller = ExecutionPlanController()
    old_tracker = node.tracker
    node._stable_variant_runtime = {'history': object()}
    config = remote_node.validate_continuous_runtime_config(
        continuous_runtime_values(
            tracking_position_threshold_m=0.03,
            tracking_orientation_threshold_deg=30.0,
            tracking_approach_threshold_deg=22.0,
            tracking_width_threshold_m=0.009,
        )
    )

    node._apply_continuous_runtime_config(config)

    assert node.tracker is not old_tracker
    assert node._tracking_config.position_threshold_m == pytest.approx(0.03)
    assert node._tracking_config.orientation_threshold_deg == pytest.approx(
        30.0
    )
    assert node._tracking_config.approach_threshold_deg == pytest.approx(22.0)
    assert node._tracking_config.width_threshold_m == pytest.approx(0.009)
    assert node._stable_variant_runtime == {}


def test_unchanged_performance_window_preserves_deque_identity():
    node = streaming_node(clock=MutableClock(10.0), start_worker=False)
    node._geometry_state_lock = threading.RLock()
    node.rate_hz = 1.5
    node.target_instance_association_threshold_m = 0.08
    node.target_absolute_sanity_distance_m = 0.15
    node.moveit_top_n = 5
    node.execution_plan_controller = ExecutionPlanController()
    latency_history = node._latency_history_ms
    metrics_history = node.pipeline_metrics
    config = remote_node.validate_continuous_runtime_config(
        continuous_runtime_values(performance_window_size=100)
    )

    node._apply_continuous_runtime_config(config)

    assert node._latency_history_ms is latency_history
    assert node.pipeline_metrics is metrics_history


def test_performance_window_resize_linearizes_with_terminal_append():
    node = streaming_node(clock=MutableClock(10.0), start_worker=False)
    node._geometry_state_lock = threading.RLock()
    node.rate_hz = 1.5
    node.target_instance_association_threshold_m = 0.08
    node.target_absolute_sanity_distance_m = 0.15
    node.moveit_top_n = 5
    node.execution_plan_controller = ExecutionPlanController()
    append_entered = threading.Event()
    release_append = threading.Event()

    class BlockingAppendDeque(remote_node.deque):
        def append(self, value):
            append_entered.set()
            if not release_append.wait(2.0):
                raise RuntimeError('test did not release latency append')
            super().append(value)

    node._latency_history_ms = BlockingAppendDeque(maxlen=100)
    node._request_telemetry[1] = {
        'request_id': 1,
        'generation': 1,
        'target_epoch': 1,
        'snapshot_stamp_sec': 9.5,
        'submitted_sec': 9.5,
    }
    emitted = []
    emit_thread = threading.Thread(
        target=lambda: emitted.append(
            node._emit_pending_drop_metrics(1, 'GENERATION_STALE')
        )
    )
    config = remote_node.validate_continuous_runtime_config(
        continuous_runtime_values(performance_window_size=7)
    )
    resize_done = threading.Event()
    resize_thread = threading.Thread(
        target=lambda: (
            node._apply_continuous_runtime_config(config),
            resize_done.set(),
        )
    )

    emit_thread.start()
    assert append_entered.wait(1.0)
    resize_thread.start()
    try:
        assert not resize_done.wait(0.2), (
            'performance resize bypassed an in-flight terminal append'
        )
    finally:
        release_append.set()
        emit_thread.join(2.0)
        resize_thread.join(2.0)
        node.shutdown_streaming_worker()

    assert emitted[0]['request_id'] == 1
    assert node._latency_history_ms.maxlen == 7
    assert list(node._latency_history_ms) == [pytest.approx(500.0)]
    assert [item['request_id'] for item in node.pipeline_metrics] == [1]
    assert node.pipeline_metrics[0]['latency_p95_ms'] == pytest.approx(500.0)


def test_duplicate_terminal_claim_does_not_change_latency_or_metrics():
    node = streaming_node(clock=MutableClock(10.0), start_worker=False)
    try:
        with node._stream_condition:
            first = node._claim_terminal_metrics_snapshot_locked(
                7,
                'GENERATION_STALE',
                end_to_end_ms=25.0,
            )
            second = node._claim_terminal_metrics_snapshot_locked(
                7,
                'GENERATION_STALE',
                end_to_end_ms=99.0,
            )

        assert first is not None
        assert second is None
        assert list(node._latency_history_ms) == [25.0]
        assert node._pipeline_counters['completed'] == 1
        assert node.pipeline_metrics == remote_node.deque(maxlen=100)
    finally:
        node.shutdown_streaming_worker()


def test_worker_duplicate_terminal_does_not_pollute_latency_history():
    prediction = BlockingPrediction()
    node = streaming_node(clock=MutableClock(10.0), prepare=prediction)
    try:
        node.start_streaming()
        node.submit_stream_snapshot(snapshot(9.8))
        assert prediction.entered.wait(1.0)
        with node._stream_condition:
            assert node._claim_terminal_request_locked(
                1, 'GENERATION_STALE'
            )

        prediction.release.set()
        wait_until(lambda: not node._stream_worker_busy)

        assert list(node._latency_history_ms) == []
        assert list(node.pipeline_metrics) == []
        assert node._pipeline_counters['completed'] == 1
    finally:
        prediction.release.set()
        node.shutdown_streaming_worker()


def test_spin_rebuilds_rate_on_change_without_catchup_schedule(monkeypatch):
    node = types.SimpleNamespace(
        rate_hz=1.5,
        enabled=False,
        streaming_enabled=False,
    )
    created_rates = []
    shutdown_checks = {'count': 0}

    class Rate:
        def __init__(self, hz):
            created_rates.append(float(hz))

        def sleep(self):
            node.rate_hz = 2.5

    def is_shutdown():
        shutdown_checks['count'] += 1
        return shutdown_checks['count'] > 2

    monkeypatch.setattr(remote_node.rospy, 'Rate', Rate)
    monkeypatch.setattr(remote_node.rospy, 'is_shutdown', is_shutdown)

    remote_node.RemoteGrasp6DNode.spin(node)

    assert created_rates == [1.5, 2.5]


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


def test_submit_snapshot_after_coordinator_stop_is_dropped_not_crashed():
    node = streaming_node(start_worker=False)
    try:
        node.start_streaming()
        with node._stream_condition:
            node.inference_coordinator.stop()

        assert node.submit_stream_snapshot(snapshot(9.8)) is False
        assert node._pipeline_counters['submitted'] == 0
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


def test_hundred_stream_requests_emit_bounded_strict_metrics_with_exact_p95():
    clock = MutableClock(1000.0)

    def prepare(ticket):
        clock.value = (
            float(ticket.submitted_monotonic_sec)
            + float(ticket.request_id) / 1000.0
        )
        return types.SimpleNamespace(
            ticket=ticket,
            candidates=(),
            remote_diagnostics={},
            remote_performance={},
            ros_prepare_ms=1.0,
            transport_ms=2.0,
            decode_ms=3.0,
        )

    node = streaming_node(clock=clock, prepare=prepare)
    node.moveit_top_n = 5
    node._accept_prediction = lambda _prepared: {
        'status': 'PREVIEW_READY',
        'funnel': {
            'stage_counts': {
                'moveit_checked': {
                    'entered': 5,
                    'passed': 3,
                    'rejected': 2,
                },
            },
            'rejection_counts': {'MOVEIT_UNREACHABLE': 2},
            'rejection_ratios': {'MOVEIT_UNREACHABLE': 0.4},
            'primary_failure': None,
        },
    }

    try:
        assert node.start_streaming() is True
        for request_id in range(1, 101):
            clock.value = 1000.0 + float(request_id)
            assert node.submit_stream_snapshot(snapshot(clock.value)) is True
            wait_until(
                lambda expected=request_id: bool(node.pipeline_metrics)
                and node.pipeline_metrics[-1]['request_id'] == expected
            )

        wait_until(lambda: not node._stream_worker_busy)
        metrics_history = list(node.pipeline_metrics)
        assert len(metrics_history) == 100
        assert all(
            0 <= item['submitted'] - item['completed'] <= 2
            for item in metrics_history
        )

        metrics = metrics_history[-1]
        assert metrics['submitted'] == 100
        assert metrics['started'] == 100
        assert metrics['completed'] == 100
        assert metrics['accepted'] == 100
        assert metrics['latency_p95_ms'] == pytest.approx(95.05)
        assert (
            metrics['stage_counts']['moveit_checked']['entered']
            <= node.moveit_top_n
        )
        assert sum(metrics['rejection_counts'].values()) >= 0

        def numeric_values(value):
            if isinstance(value, dict):
                for nested in value.values():
                    yield from numeric_values(nested)
            elif isinstance(value, (list, tuple)):
                for nested in value:
                    yield from numeric_values(nested)
            elif isinstance(value, (int, float)) and not isinstance(value, bool):
                yield float(value)

        assert all(math.isfinite(value) for value in numeric_values(metrics))
        encoded = remote_node.strict_metrics_json(metrics)
        decoded = json.loads(encoded)
        assert encoded == json.dumps(
            decoded,
            allow_nan=False,
            sort_keys=True,
            separators=(',', ':'),
        )
        assert node.inference_coordinator.pending_count == 0
    finally:
        node.shutdown_streaming_worker()


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
    node.hybrid_merge_config = remote_node.MergeConfig()
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


def test_real_graspnet_normalization_and_analytical_gate_use_prepared_geometry():
    node = remote_node.RemoteGrasp6DNode.__new__(remote_node.RemoteGrasp6DNode)
    node.require_candidate_depth = True
    node.model_grasp_to_tool_quaternion = np.asarray(
        remote_node.STRICT_MODEL_GRASP_TO_TOOL_QUATERNION,
        dtype=float,
    )
    node.candidate_frame_convention = 'opencv_optical'
    node.grasp_config = {
        'tool_approach_axis': 'z',
        'pregrasp_distance_m': 0.08,
        'final_approach_offset_m': 0.020,
        'lift_height_m': 0.05,
    }
    node.gripper_geometry = tabletop_gripper()
    node.gripper_tool_jaw_axis = 'y'
    node.gripper_tool_finger_length_axis = 'z'
    node.gripper_physical_open_width_m = 0.05
    node.target_instance_epoch = 4
    node.target_absolute_sanity_distance_m = 0.15
    node.soft_score_weights = SoftScoreWeights()
    node.camera_visibility_gate_enabled = False
    node.camera_visibility_diagnostic_enabled = False
    node._record_geometry_gate_result = lambda _gate: None
    size = np.asarray([0.040, 0.040, 0.040])
    prepared_geometry = types.SimpleNamespace(
        ok=True,
        center_base=np.asarray([0.0, 0.0, 0.080]),
        axes_base=np.eye(3),
        size_xyz_m=size,
        support_normal_base=np.asarray([0.0, 0.0, 1.0]),
        support_offset_m=0.0,
        object_points_base=(
            tabletop_box_cloud(size) + np.asarray([0.0, 0.0, 0.050])
        ),
    )
    live_geometry = types.SimpleNamespace(
        ok=False,
        center_base=np.asarray([9.0, 9.0, 9.0]),
    )
    node._latest_geometry_estimate = live_geometry
    stamp = remote_node.rospy.Time.from_sec(20.0)
    base_from_optical = np.eye(4)
    base_from_optical[:3, :3] = remote_node.OPTICAL_TO_ROS_CAMERA
    estimator = remote_node.FrozenSnapshotCandidatePoseEstimator(
        base_from_optical,
        stamp,
        'camera_link',
        raw_candidate_convention='opencv_optical',
    )
    prepared = types.SimpleNamespace(
        ticket=InferenceTicket(
            request_id=4,
            generation=1,
            snapshot_stamp_sec=20.0,
            target_epoch=4,
            payload=None,
            submitted_monotonic_sec=20.0,
        ),
        stamp=stamp,
        geometry=prepared_geometry,
        pose_estimator=estimator,
        snapshot=types.SimpleNamespace(
            object_msg=types.SimpleNamespace(detected=True, label='carton')
        ),
        model_choice='carton_segment',
    )
    raw_candidate = RemoteGraspCandidate(
        score=0.9,
        translation_m=(0.0, -0.080, 0.0),
        quaternion_xyzw=remote_node.quaternion_from_matrix(
            np.block([
                [
                    np.asarray(
                        [
                            [0.0, 1.0, 0.0],
                            [1.0, 0.0, 0.0],
                            [0.0, 0.0, -1.0],
                        ]
                    ),
                    np.zeros((3, 1)),
                ],
                [np.zeros((1, 3)), np.ones((1, 1))],
            ])
        ),
        width_m=0.012,
        depth_m=0.030,
    )

    normalized = node._normalize_graspnet_candidate(
        prepared,
        raw_candidate,
        0,
    )
    safety = node._normalized_candidate_safety(prepared, normalized)

    assert normalized.geometry_gate.ok is True
    assert normalized.geometry_gate.required_open_width_m > 0.0
    assert normalized.audit['adaptive_orientation_projection'][
        'projected'
    ] is False
    assert safety.ok is True
    assert node._latest_geometry_estimate is live_geometry


def test_graspnet_orientation_projection_is_minimal_and_preserves_contact():
    node = remote_node.RemoteGrasp6DNode.__new__(remote_node.RemoteGrasp6DNode)
    node.grasp_config = {'tool_approach_axis': 'z'}
    node.gripper_tool_jaw_axis = 'y'
    node.gripper_tool_finger_length_axis = 'z'
    node._adaptive_stage_profiles = lambda *_args, **_kwargs: (
        types.SimpleNamespace(tilt_deg=0.0),
        types.SimpleNamespace(tilt_deg=30.0),
    )
    stamp = remote_node.rospy.Time.from_sec(20.0)
    base_from_optical = np.eye(4)
    base_from_optical[:3, :3] = remote_node.OPTICAL_TO_ROS_CAMERA
    estimator = remote_node.FrozenSnapshotCandidatePoseEstimator(
        base_from_optical,
        stamp,
        'camera_link',
        raw_candidate_convention='opencv_optical',
    )
    assert np.allclose(estimator.T_base_camera_link, np.eye(4))
    prepared = types.SimpleNamespace(
        stamp=stamp,
        pose_estimator=estimator,
        geometry=types.SimpleNamespace(
            support_normal_base=np.asarray([0.0, 0.0, 1.0]),
        ),
        snapshot=types.SimpleNamespace(),
    )
    jaw = np.asarray([0.0, 1.0, 0.0])
    raw_tilt_deg = 60.0
    raw_insertion = np.asarray([
        math.sin(math.radians(raw_tilt_deg)),
        0.0,
        -math.cos(math.radians(raw_tilt_deg)),
    ])
    rotation = remote_node.semantic_axes_to_tool_rotation(
        insertion_axis_base=raw_insertion,
        jaw_axis_base=jaw,
        tool_jaw_axis='y',
        tool_finger_length_axis='z',
    )
    transform = np.eye(4)
    transform[:3, :3] = rotation
    center = np.asarray([0.01, -0.02, 0.10])
    depth_m = 0.03
    transform[:3, 3] = center + depth_m * raw_insertion
    candidate = RemoteGraspCandidate(
        score=0.91,
        translation_m=center.copy(),
        quaternion_xyzw=remote_node.quaternion_from_matrix(transform),
        width_m=0.024,
        height_m=0.018,
        depth_m=depth_m,
        tool0_translation_m=transform[:3, 3].copy(),
    )
    grasp_pose = remote_node.make_pose_stamped(
        'base_link',
        transform[:3, 3],
        candidate.quaternion_xyzw,
        stamp=stamp,
    )

    projected, projected_pose, recovered_center, audit = (
        node._project_graspnet_stage_orientation(
            prepared,
            candidate,
            grasp_pose,
            center,
        )
    )

    projected_insertion = node._pose_approach_base_xyz(projected_pose)
    projected_tilt_deg = math.degrees(
        math.acos(
            np.clip(
                float(np.dot(projected_insertion, [0.0, 0.0, -1.0])),
                -1.0,
                1.0,
            )
        )
    )
    projected_tool0 = remote_node.pose_matrix(projected_pose)[:3, 3]
    assert audit['projected'] is True
    assert audit['raw_tilt_deg'] == pytest.approx(60.0)
    assert audit['projected_tilt_deg'] == pytest.approx(30.0)
    assert audit['maximum_admissible_tilt_deg'] == pytest.approx(30.0)
    assert audit['orientation_correction_deg'] == pytest.approx(30.0)
    assert projected_tilt_deg == pytest.approx(30.0)
    assert np.allclose(recovered_center, center)
    assert np.allclose(projected.translation_m, center)
    assert np.linalg.norm(projected_tool0 - center) == pytest.approx(depth_m)
    assert projected.score == pytest.approx(candidate.score)
    assert projected.width_m == pytest.approx(candidate.width_m)
    assert projected.depth_m == pytest.approx(candidate.depth_m)


def test_graspnet_orientation_projection_fails_closed_without_downward_axis():
    node = remote_node.RemoteGrasp6DNode.__new__(remote_node.RemoteGrasp6DNode)
    node.grasp_config = {'tool_approach_axis': 'z'}
    node.gripper_tool_jaw_axis = 'y'
    node.gripper_tool_finger_length_axis = 'z'
    node._adaptive_stage_profiles = lambda *_args, **_kwargs: (
        types.SimpleNamespace(tilt_deg=0.0),
        types.SimpleNamespace(tilt_deg=30.0),
    )
    prepared = types.SimpleNamespace(
        geometry=types.SimpleNamespace(
            support_normal_base=np.asarray([0.0, 1.0, 0.0]),
        ),
        snapshot=types.SimpleNamespace(),
    )
    rotation = remote_node.semantic_axes_to_tool_rotation(
        insertion_axis_base=np.asarray([1.0, 0.0, 0.0]),
        jaw_axis_base=np.asarray([0.0, 1.0, 0.0]),
        tool_jaw_axis='y',
        tool_finger_length_axis='z',
    )
    transform = np.eye(4)
    transform[:3, :3] = rotation
    candidate = RemoteGraspCandidate(
        score=0.9,
        translation_m=np.zeros(3),
        quaternion_xyzw=remote_node.quaternion_from_matrix(transform),
        width_m=0.02,
        depth_m=0.03,
        tool0_translation_m=np.asarray([0.03, 0.0, 0.0]),
    )
    grasp_pose = remote_node.make_pose_stamped(
        'base_link',
        candidate.tool0_translation_m,
        candidate.quaternion_xyzw,
    )

    with pytest.raises(
        remote_node.CandidateContractError,
        match='no downward insertion direction exists',
    ) as caught:
        node._project_graspnet_stage_orientation(
            prepared,
            candidate,
            grasp_pose,
            np.zeros(3),
        )

    assert caught.value.code == 'GRASPNET_STAGE_PROFILE_UNAVAILABLE'


def test_local_pipeline_keeps_tabletop_candidate_when_graspnet_hits_table(
    monkeypatch,
):
    node = remote_node.RemoteGrasp6DNode.__new__(remote_node.RemoteGrasp6DNode)
    node.require_candidate_depth = True
    node.model_grasp_to_tool_quaternion = (
        remote_node.STRICT_MODEL_GRASP_TO_TOOL_QUATERNION
    )
    node.grasp_config = {
        'tool_approach_axis': 'z',
        'pregrasp_distance_m': 0.08,
        'final_approach_offset_m': 0.015,
        'lift_height_m': 0.05,
    }
    node.soft_score_weights = SoftScoreWeights()
    node.hybrid_merge_config = remote_node.MergeConfig()
    node.gripper_geometry = tabletop_gripper()
    node.gripper_tool_jaw_axis = 'y'
    node.gripper_tool_finger_length_axis = 'z'
    node.gripper_physical_open_width_m = 0.05
    node.target_instance_epoch = 4
    node.target_absolute_sanity_distance_m = 0.15
    node.camera_visibility_gate_enabled = False
    node.camera_visibility_diagnostic_enabled = False
    geometry = tabletop_geometry()
    passing_gate = CandidateGateResult(
        ok=True,
        failure_code='',
        failure_reason='',
        required_open_width_m=0.039,
        center_distance_m=0.0,
        support_clearance_m=0.003,
        jaw_alignment=1.0,
        motion_cost=0.0,
        geometry_cost=0.0,
        failed_gate='',
        passed_gate_count=6,
    )
    monkeypatch.setattr(
        remote_node,
        'evaluate_explicit_candidate',
        lambda **_kwargs: passing_gate,
    )
    monkeypatch.setattr(
        node,
        '_normalize_graspnet_candidate',
        lambda *_args: (_ for _ in ()).throw(
            remote_node.CandidateContractError(
                'GRIPPER_SWEEP_COLLISION',
                'synthetic table collision',
            )
        ),
        raising=False,
    )
    ticket = InferenceTicket(
        request_id=9,
        generation=1,
        snapshot_stamp_sec=20.0,
        target_epoch=4,
        payload=None,
        submitted_monotonic_sec=20.0,
    )
    prepared = types.SimpleNamespace(
        ticket=ticket,
        stamp=remote_node.rospy.Time.from_sec(20.0),
        snapshot=types.SimpleNamespace(
            object_msg=types.SimpleNamespace(detected=True, label='carton')
        ),
        geometry=geometry,
        pose_estimator=types.SimpleNamespace(transform_sha256='frozen-cloud'),
        candidates=('graspnet-table-collision',),
        tabletop_candidates=tabletop_candidates_for(geometry),
        remote_diagnostics={
            'raw_candidates': 1,
            'after_nms': 1,
            'after_collision': 1,
        },
        remote_failure_code='',
        remote_failure_reason='',
        model_choice='carton_segment',
    )

    observations, funnel = node._evaluate_local_candidates(prepared)

    assert funnel['source_counts']['graspnet']['locally_valid'] == 0
    assert funnel['source_counts']['tabletop_geometry']['locally_valid'] >= 1
    assert any(
        item.candidate_source == 'tabletop_geometry'
        for item in observations
    )
    assert all(item.model_width_m is None for item in observations)
    assert len(observations) <= 8


@pytest.mark.parametrize(
    ('geometry', 'expected_code'),
    [
        (
            types.SimpleNamespace(
                **{
                    **tabletop_geometry().__dict__,
                    'support_normal_base': np.zeros(3),
                }
            ),
            'SUPPORT_PLANE_INVALID',
        ),
        (tabletop_geometry((0.060, 0.055, 0.011)), 'NO_FIT_DIRECTION'),
    ],
)
def test_prepared_geometry_generation_fails_closed_with_stable_code(
    geometry, expected_code
):
    node = remote_node.RemoteGrasp6DNode.__new__(remote_node.RemoteGrasp6DNode)
    node.tabletop_geometry_enabled = True
    node.tabletop_geometry_config = TabletopGeometryConfig(max_candidates=8)
    node.gripper_geometry = tabletop_gripper()
    node.gripper_tool_jaw_axis = 'y'
    node.gripper_tool_finger_length_axis = 'z'

    candidates, diagnostics = node._generate_tabletop_candidates(geometry)

    assert candidates == ()
    assert diagnostics['failure_code'] == expected_code


def test_adaptive_stage_profiles_ignore_object_label_and_workspace_position():
    node = remote_node.RemoteGrasp6DNode.__new__(remote_node.RemoteGrasp6DNode)
    node.gripper_geometry = tabletop_gripper()
    node.gripper_tool_finger_length_axis = 'z'
    node.candidate_min_downward_approach_cos = 0.65
    node.candidate_max_final_approach_lateral_m = 0.010
    geometry = tabletop_geometry()
    shift = np.asarray([0.23, -0.41, 0.17])
    translated = types.SimpleNamespace(
        **{
            **geometry.__dict__,
            'center_base': np.asarray(geometry.center_base) + shift,
            'object_points_base': (
                np.asarray(geometry.object_points_base) + shift
            ),
            'support_offset_m': (
                float(geometry.support_offset_m)
                - float(np.dot(geometry.support_normal_base, shift))
            ),
        }
    )
    carton_snapshot = types.SimpleNamespace(
        object_msg=types.SimpleNamespace(label='carton'),
        quality=types.SimpleNamespace(
            depth_mad_m=0.009,
            depth_repeatability_m=0.0015,
        ),
    )
    bottle_snapshot = types.SimpleNamespace(
        object_msg=types.SimpleNamespace(label='bottle'),
        quality=types.SimpleNamespace(
            depth_mad_m=0.004,
            depth_repeatability_m=0.0015,
        ),
    )

    carton = node._adaptive_stage_profiles(geometry, carton_snapshot)
    bottle = node._adaptive_stage_profiles(translated, bottle_snapshot)

    assert carton == bottle
    assert len(carton) > 1
    assert all(
        profile.lateral_sweep_m <= 0.010 + 1e-12
        for profile in carton
    )


def test_near_field_profiles_consume_fresh_measured_execution_error(
    monkeypatch,
):
    node = remote_node.RemoteGrasp6DNode.__new__(
        remote_node.RemoteGrasp6DNode
    )
    node.gripper_geometry = tabletop_gripper()
    node.gripper_tool_finger_length_axis = 'z'
    node.candidate_min_downward_approach_cos = 0.65
    node.candidate_max_final_approach_lateral_m = 0.020
    node.runtime_execution_error_max_age_sec = 180.0
    geometry = tabletop_geometry()
    snapshot = types.SimpleNamespace(
        quality=types.SimpleNamespace(
            depth_mad_m=0.009,
            depth_repeatability_m=0.0015,
        ),
    )

    monkeypatch.setattr(
        remote_node.rospy,
        'get_param',
        lambda name, default=None: (
            {
                'stamp_sec': 100.0,
                'position_error_m': 0.0228,
            }
            if name == '/grasp_6d/runtime_execution_error'
            else default
        ),
    )
    monkeypatch.setattr(
        remote_node.rospy.Time,
        'now',
        staticmethod(lambda: remote_node.rospy.Time.from_sec(101.0)),
    )

    node.robot_execution_active = True
    node.near_field_planning_active = False
    baseline = node._adaptive_stage_profiles(geometry, snapshot)

    node.near_field_planning_active = True
    measured = node._adaptive_stage_profiles(geometry, snapshot)

    assert baseline[0].execution_position_error_m == pytest.approx(0.0)
    assert measured[0].execution_position_error_m == pytest.approx(0.0228)
    assert measured[0].pregrasp_distance_m > baseline[0].pregrasp_distance_m
    assert measured[-1].tilt_deg < baseline[-1].tilt_deg


def test_contact_sequence_lifts_along_live_support_normal():
    node = remote_node.RemoteGrasp6DNode.__new__(
        remote_node.RemoteGrasp6DNode
    )
    node.gripper_geometry = tabletop_gripper()
    node.gripper_tool_finger_length_axis = 'z'
    node.candidate_min_downward_approach_cos = 0.65
    node.candidate_max_final_approach_lateral_m = 0.010
    node.robot_execution_active = False
    support = np.asarray([0.12, -0.16, 0.9797958971], dtype=float)
    support /= np.linalg.norm(support)
    geometry = types.SimpleNamespace(
        axes_base=np.eye(3),
        size_xyz_m=np.asarray([0.040, 0.035, 0.020]),
        support_normal_base=support,
        object_points_base=tabletop_box_cloud((0.040, 0.035, 0.020)),
    )
    grasp_pose = remote_node.make_pose_stamped(
        'base_link',
        np.asarray([0.2, -0.3, 0.1]),
        np.asarray([0.0, 0.0, 0.0, 1.0]),
        stamp=remote_node.rospy.Time.from_sec(10.0),
    )

    sequence, profile = node._make_contact_sequence(
        grasp_pose,
        geometry,
        insertion_axis_base=-support,
    )

    grasp_xyz = remote_node.pose_matrix(sequence.grasp)[:3, 3]
    lift_xyz = remote_node.pose_matrix(sequence.lift)[:3, 3]
    np.testing.assert_allclose(
        lift_xyz - grasp_xyz,
        profile.lift_height_m * support,
        atol=1e-12,
    )


def test_centered_observation_pose_uses_camera_distance_and_centers_target():
    grasp_pose = remote_node.make_pose_stamped(
        'base_link',
        np.asarray([0.0, 0.0, 0.0]),
        np.asarray([0.0, 0.0, 0.0, 1.0]),
        stamp=remote_node.rospy.Time.from_sec(10.0),
    )
    target = np.asarray([0.10, 0.06, 0.0])

    observation, audit = (
        remote_node.centered_observation_pose_at_camera_distance(
            grasp_pose,
            target,
            np.eye(4),
            0.20,
            0.19,
            0.21,
        )
    )

    observation_xyz = remote_node.pose_matrix(observation)[:3, 3]
    np.testing.assert_allclose(observation_xyz, [-0.10, 0.06, 0.0])
    u, v, depth = remote_node.project_base_target_at_tool_pose(
        observation,
        target,
        np.eye(4),
        remote_node.CameraIntrinsics(
            640,
            480,
            600.0,
            600.0,
            320.0,
            240.0,
            0.001,
        ),
    )
    assert u == pytest.approx(320.0)
    assert v == pytest.approx(240.0)
    assert depth == pytest.approx(0.20)
    assert audit['actual_camera_target_distance_m'] == pytest.approx(0.20)
    assert audit['min_camera_target_distance_m'] == pytest.approx(0.19)
    assert audit['max_camera_target_distance_m'] == pytest.approx(0.21)
    assert audit[
        'derived_tool0_to_grasp_tool0_distance_m'
    ] == pytest.approx(math.sqrt(0.10 ** 2 + 0.06 ** 2))
    assert audit['center_residual_m'] == pytest.approx(0.0, abs=1e-12)
    assert (
        audit['selection_rule']
        == 'configured_nominal_camera_target_distance'
    )


def test_centered_observation_pose_rejects_nominal_outside_success_range():
    grasp_pose = remote_node.make_pose_stamped(
        'base_link',
        np.asarray([0.0, 0.0, 0.0]),
        np.asarray([0.0, 0.0, 0.0, 1.0]),
        stamp=remote_node.rospy.Time.from_sec(10.0),
    )

    with pytest.raises(remote_node.CandidateContractError) as caught:
        remote_node.centered_observation_pose_at_camera_distance(
            grasp_pose,
            np.asarray([0.10, 0.11, 0.0]),
            np.eye(4),
            0.18,
            0.19,
            0.21,
        )

    assert caught.value.code == 'OBSERVATION_VIEW_GEOMETRY_INVALID'


def test_200mm_observation_real_plan_fixture_increases_table_clearance():
    orientation = np.asarray(
        [
            0.7787295814561642,
            0.5887404775969284,
            -0.14461075927228956,
            0.16140823184317438,
        ]
    )
    grasp_pose = remote_node.make_pose_stamped(
        'base_link',
        np.asarray(
            [
                -0.1215109865272657,
                -0.45201426390555255,
                0.05700888776151803,
            ]
        ),
        orientation,
    )
    old_observation = remote_node.make_pose_stamped(
        'base_link',
        np.asarray(
            [
                -0.0918402400634709,
                -0.3571621596383056,
                0.06808702480510659,
            ]
        ),
        orientation,
    )
    target = np.asarray(
        [
            -0.11806379974380408,
            -0.441570058070009,
            0.054769604796107964,
        ]
    )
    tool_from_camera = remote_node.quaternion_matrix(
        [
            0.0229081321,
            -0.6787668594,
            0.0278470194,
            0.7334680031,
        ]
    )
    tool_from_camera[:3, 3] = [
        -0.0866127223,
        -0.0063407198,
        -0.1034120675,
    ]
    new_observation, audit = (
        remote_node.centered_observation_pose_at_camera_distance(
            grasp_pose,
            target,
            tool_from_camera,
            0.200,
            0.190,
            0.210,
        )
    )
    gripper = GripperGeometry(
        max_inner_gap_m=0.050,
        jaw_clearance_each_side_m=0.002,
        finger_size_xyz_m=np.asarray([0.0434, 0.0286, 0.0600]),
        palm_size_xyz_m=np.asarray([0.1175, 0.1550, 0.0774]),
        support_clearance_m=0.003,
    )
    support_normal = np.asarray(
        [
            -0.06761897336170623,
            0.06130672642656254,
            0.9958258681799582,
        ]
    )
    support_normal /= np.linalg.norm(support_normal)
    obb_rotation = remote_node.quaternion_matrix(
        [
            0.003930067374661929,
            -0.045515057733144966,
            -0.7321585531099706,
            0.6796004614462041,
        ]
    )[:3, :3]
    envelope_kwargs = {
        'gripper': gripper,
        'opening_width_m': 0.050,
        'support_normal_base': support_normal,
        'support_offset_m': -0.025322463363409042,
        'obb_center_base': target,
        'R_base_obb': obb_rotation,
        'obb_size_xyz_m': np.asarray(
            [
                0.055169997074668284,
                0.03624848213702317,
                0.020261329136952385,
            ]
        ),
        'tool_jaw_axis': 'y',
        'tool_finger_length_axis': 'z',
    }
    old_result = evaluate_open_gripper_observation_envelope(
        T_base_tool0=remote_node.pose_matrix(old_observation),
        **envelope_kwargs,
    )
    new_result = evaluate_open_gripper_observation_envelope(
        T_base_tool0=remote_node.pose_matrix(new_observation),
        **envelope_kwargs,
    )

    assert old_result.ok is True
    assert new_result.ok is True
    assert old_result.minimum_support_clearance_m == pytest.approx(
        0.017840662,
        abs=1e-9,
    )
    assert new_result.minimum_support_clearance_m == pytest.approx(
        0.063109846,
        abs=1e-9,
    )
    assert (
        new_result.minimum_support_clearance_m
        - old_result.minimum_support_clearance_m
    ) > 0.045
    assert audit['actual_camera_target_distance_m'] == pytest.approx(0.200)
    assert audit[
        'derived_tool0_to_grasp_tool0_distance_m'
    ] == pytest.approx(0.128335548, abs=1e-9)


def test_tabletop_generation_materializes_geometry_derived_stage_profiles():
    node = remote_node.RemoteGrasp6DNode.__new__(remote_node.RemoteGrasp6DNode)
    node.tabletop_geometry_enabled = True
    node.tabletop_geometry_config = TabletopGeometryConfig(max_candidates=24)
    node.gripper_geometry = tabletop_gripper()
    node.gripper_tool_jaw_axis = 'y'
    node.gripper_tool_finger_length_axis = 'z'
    node.candidate_min_downward_approach_cos = 0.65
    node.candidate_max_final_approach_lateral_m = 0.010

    candidates, diagnostics = node._generate_tabletop_candidates(
        tabletop_geometry(),
        contact_execution_phase=False,
    )

    profiles = diagnostics['adaptive_stage_profiles']
    assert candidates
    assert diagnostics['plan_phase'] == remote_node.FAR_FIELD_OBSERVATION_PLAN
    assert len(profiles) == 5
    assert profiles[0]['tilt_deg'] == pytest.approx(0.0)
    assert profiles[-1]['lateral_sweep_m'] <= 0.010 + 1e-12
    assert any(
        abs(profile['tilt_deg'] - 15.0) > 1e-3
        for profile in profiles[1:]
    )
    assert all(
        'adaptive_stage_profile' in candidate.audit
        for candidate in candidates
    )
    materialized_tilts = {
        round(float(candidate.audit['approach_tilt_deg']), 6)
        for candidate in candidates
    }
    assert {
        round(float(profile['tilt_deg']), 6)
        for profile in profiles
    }.issubset(materialized_tilts)


def test_tabletop_contact_boundary_resolves_live_uncertainty_and_is_stratified():
    node = remote_node.RemoteGrasp6DNode.__new__(remote_node.RemoteGrasp6DNode)
    node.tabletop_geometry_enabled = True
    node.tabletop_geometry_config = TabletopGeometryConfig(max_candidates=24)
    node.gripper_geometry = tabletop_gripper()
    node.gripper_tool_jaw_axis = 'y'
    node.gripper_tool_finger_length_axis = 'z'
    node.candidate_min_downward_approach_cos = 0.65
    node.candidate_max_final_approach_lateral_m = 0.010
    snapshot = types.SimpleNamespace(
        quality=types.SimpleNamespace(
            depth_mad_m=0.009,
            depth_repeatability_m=0.0015,
        ),
    )

    candidates, diagnostics = node._generate_tabletop_candidates(
        tabletop_geometry((0.040, 0.035, 0.011)),
        snapshot=snapshot,
    )

    assert candidates
    live_uncertainty_m = 0.003
    assert diagnostics['required_contact_patch_overlap_m'] == pytest.approx(
        live_uncertainty_m
    )
    assert diagnostics['depth_spatial_mad_m'] == pytest.approx(0.009)
    assert diagnostics['depth_repeatability_m'] == pytest.approx(0.0015)
    assert diagnostics['contact_overlap_requirement_source'] == (
        'live_perception_uncertainty_only'
    )
    assert diagnostics['contact_boundary_profiles']
    for profile in diagnostics['contact_boundary_profiles']:
        for branch in profile['branches']:
            assert branch['required_contact_patch_overlap_m'] == pytest.approx(
                live_uncertainty_m
            )
            assert branch['sample_count'] >= 1
            assert branch['maximum_observed_contact_patch_overlap_m'] is not None
            assert branch['maximum_observed_tilt_deg'] is not None
    assert any(
        item.audit['approach_tilt_deg'] > 0.0
        for item in candidates
    )
    assert all(
        item.audit['contact_patch_overlap_m'] >= live_uncertainty_m - 1e-9
        for item in candidates
    )
    assert all(
        item.audit['contact_patch_signal_to_uncertainty'] >= 1.0 - 1e-9
        and item.audit['contact_patch_uncertainty_margin_m'] >= -1e-9
        and 0.0 <= item.audit['contact_patch_coverage_fraction'] <= 1.0
        for item in candidates
    )
    first_by_proposal = {}
    for item in candidates:
        proposal_index = int(item.audit['proposal_source_index'])
        first_by_proposal.setdefault(proposal_index, item)
    for proposal_index, item in first_by_proposal.items():
        polarity_index = (
            0
            if float(item.audit['approach_tilt_polarity']) < 0.0
            else 1
        )
        branch_index = 2 * polarity_index + int(item.variant_index)
        assert branch_index == proposal_index % 4


def test_far_field_defers_contact_while_near_field_uses_snapshot_uncertainty():
    node = remote_node.RemoteGrasp6DNode.__new__(remote_node.RemoteGrasp6DNode)
    node.tabletop_geometry_enabled = True
    node.tabletop_geometry_config = TabletopGeometryConfig(max_candidates=24)
    node.gripper_geometry = tabletop_gripper()
    node.gripper_tool_jaw_axis = 'y'
    node.gripper_tool_finger_length_axis = 'z'
    node.candidate_min_downward_approach_cos = 0.65
    node.candidate_max_final_approach_lateral_m = 0.010
    geometry = tabletop_geometry((0.040, 0.035, 0.011))
    snapshot = types.SimpleNamespace(
        quality=types.SimpleNamespace(
            depth_mad_m=0.012,
            depth_repeatability_m=0.003,
        ),
    )

    contact_candidates, contact_diagnostics = (
        node._generate_tabletop_candidates(
            geometry,
            snapshot=snapshot,
            contact_execution_phase=True,
        )
    )
    observation_candidates, observation_diagnostics = (
        node._generate_tabletop_candidates(
            geometry,
            snapshot=snapshot,
            contact_execution_phase=False,
        )
    )

    assert contact_candidates
    assert contact_diagnostics['required_contact_patch_overlap_m'] == (
        pytest.approx(0.006)
    )
    assert contact_diagnostics['contact_overlap_requirement_source'] == (
        'live_perception_uncertainty_only'
    )
    assert all(
        item.audit['contact_patch_uncertainty_m'] == pytest.approx(0.006)
        and item.audit['contact_patch_signal_to_uncertainty'] >= 1.0 - 1e-9
        for item in contact_candidates
    )
    assert observation_candidates
    assert observation_diagnostics['plan_phase'] == (
        remote_node.FAR_FIELD_OBSERVATION_PLAN
    )
    assert observation_diagnostics['contact_execution_gate_deferred'] is True
    assert observation_diagnostics[
        'contact_overlap_requirement_source'
    ] == 'deferred_to_near_field_contact_plan'
    assert observation_diagnostics['required_contact_patch_overlap_m'] == (
        pytest.approx(0.0)
    )
    assert all(
        item.audit['contact_patch_required_overlap_m']
        == pytest.approx(0.0)
        for item in observation_candidates
    )
    assert all(
        item.audit['contact_patch_overlap_m'] >= 0.0
        for item in observation_candidates
    )
    assert all(
        item.audit['contact_patch_signal_to_uncertainty'] is None
        and item.audit['contact_overlap_decision_rule']
        == 'deferred_to_near_field_contact_plan'
        for item in observation_candidates
    )


def test_contact_overlap_requirement_fails_closed_without_live_profile():
    with pytest.raises(
        remote_node.CandidateContractError,
        match='live adaptive-stage uncertainty',
    ):
        remote_node.RemoteGrasp6DNode._phase_contact_overlap_requirement_m(
            True,
            None,
        )


def test_contact_overlap_requirement_excludes_cartesian_tracking_residual():
    profile = types.SimpleNamespace(
        depth_uncertainty_m=0.0162,
        execution_position_error_m=0.0132,
        contact_overlap_requirement_m=0.002,
    )

    assert remote_node.RemoteGrasp6DNode._phase_contact_overlap_requirement_m(
        True,
        profile,
    ) == pytest.approx(0.002)

    assert (
        remote_node.RemoteGrasp6DNode
        ._phase_contact_overlap_requirement_m(False, None)
        == pytest.approx(0.0)
    )


def test_tabletop_bounded_batch_represents_each_live_proposal_before_repeats():
    node = remote_node.RemoteGrasp6DNode.__new__(remote_node.RemoteGrasp6DNode)
    node.tabletop_geometry_enabled = True
    node.tabletop_geometry_config = TabletopGeometryConfig(max_candidates=5)
    node.gripper_geometry = tabletop_gripper()
    node.gripper_tool_jaw_axis = 'y'
    node.gripper_tool_finger_length_axis = 'z'
    node.candidate_min_downward_approach_cos = 0.65
    node.candidate_max_final_approach_lateral_m = 0.010

    candidates, diagnostics = node._generate_tabletop_candidates(
        tabletop_geometry((0.040, 0.035, 0.011))
    )

    proposal_indices = [
        int(candidate.audit['proposal_source_index'])
        for candidate in candidates
    ]
    assert diagnostics['proposal_count'] > 1
    assert len(candidates) == 5
    assert len(set(proposal_indices)) == diagnostics['proposal_count']
    assert diagnostics['materialized_proposal_count'] == len(
        set(proposal_indices)
    )
    assert diagnostics['bounded_proposal_count'] == len(set(proposal_indices))
    assert len(set(proposal_indices[:diagnostics['proposal_count']])) == (
        diagnostics['proposal_count']
    )


def test_target_epoch_invalidation_clears_tracks_from_both_sources():
    tracker = CandidateTracker(TrackingConfig(window_size=5, min_hits=3))
    graspnet = matching_observation(1)
    geometry = CandidateObservation(
        **{
            **matching_observation(1).__dict__,
            'model_width_m': None,
            'model_score': None,
            'payload': {'source': 'geometry'},
            'candidate_source': 'tabletop_geometry',
            'source_lineage': ('tabletop_geometry',),
        }
    )
    tracker.update(
        1,
        (graspnet, geometry),
        target_identity=(4, 'carton', 'carton_segment'),
    )

    stable = tracker.update(
        2,
        (),
        target_identity=(5, 'carton', 'carton_segment'),
    )

    assert stable == []
    assert tracker.track_count == 0


def test_source_funnel_exposes_bounded_hybrid_outcomes():
    funnel = remote_node.RemoteGrasp6DNode._merge_pipeline_funnel(
        {
            'input_count': 3,
            'stage_counts': {
                'locally_valid': {'entered': 3, 'passed': 2, 'rejected': 1},
            },
            'source_counts': {
                'graspnet': {'input': 1, 'locally_valid': 1},
                'tabletop_geometry': {'input': 2, 'locally_valid': 1},
            },
            'rejection_counts': {},
        },
    )

    assert funnel['source_counts']['graspnet'] == {
        'generated': 1,
        'locally_valid': 1,
        'stable': 0,
        'preview': 0,
        'promoted': 0,
    }
    assert funnel['source_counts']['tabletop_geometry'] == {
        'generated': 2,
        'locally_valid': 1,
        'stable': 0,
        'preview': 0,
        'promoted': 0,
    }


@pytest.mark.parametrize(
    'failure_code',
    [
        'WSL_UNAVAILABLE',
        'WSL_PREDICT_FAILED',
        'GRASPNET_PROTOCOL_INVALID',
    ],
)
@pytest.mark.parametrize('early_return', ['observations_empty', 'stable_empty'])
def test_remote_failure_only_records_invalid_before_local_pipeline_early_return(
    monkeypatch,
    failure_code,
    early_return,
):
    node = streaming_node(clock=MutableClock(20.0), start_worker=False)
    try:
        node.start_streaming()
        node.submit_stream_snapshot(snapshot(19.8))
        ticket = node._stream_worker_ticket
        seed_stream_execution_authority(node)
        prepared = remote_node.PreparedPrediction(
            ticket=ticket,
            snapshot=types.SimpleNamespace(
                object_msg=types.SimpleNamespace(detected=True, label='carton')
            ),
            stamp=remote_node.rospy.Time.from_sec(19.8),
            geometry=tabletop_geometry(),
            pose_estimator=types.SimpleNamespace(transform_sha256='frozen-tf'),
            graspnet_input=None,
            candidates=(),
            remote_diagnostics={},
            remote_performance={},
            tabletop_candidates=(),
            remote_failure_code=failure_code,
            remote_failure_reason='remote prediction failed',
            model_choice='carton_segment',
        )
        node._activate_prepared_geometry = lambda _prepared: True
        node._run_candidate_gate_audit = lambda *_args, **_kwargs: {
            'summary': {},
            'rows': [],
        }
        observations = (
            ()
            if early_return == 'observations_empty'
            else ('geometry-observation',)
        )
        stable = ('retained-stable',) if not observations else ()
        local_funnel = {
            'input_count': 0,
            'stage_counts': {
                'locally_valid': {
                    'entered': len(observations),
                    'passed': len(observations),
                    'rejected': 0,
                }
            },
            'source_counts': {},
            'rejection_counts': {failure_code: 1},
            'rejection_ratios': {failure_code: 1.0},
            'primary_failure': failure_code,
        }
        node._evaluate_local_candidates = lambda _prepared: (
            observations,
            local_funnel,
        )
        node._update_candidate_tracker = lambda **_kwargs: stable
        node._recheck_and_score_stable = lambda *_args: pytest.fail(
            'early return must not recheck stable candidates'
        )
        def finalize_audit(*_args, **kwargs):
            callback = kwargs.get('lifecycle_commit_callback')
            if callback is not None:
                callback()

        node._finalize_streaming_gate_audit = finalize_audit
        node._finalize_promotion_transaction = lambda *_args, **_kwargs: (
            pytest.fail('remote failure must not enter MuJoCo promotion')
        )
        node.stop_streaming = lambda *_args, **_kwargs: pytest.fail(
            'remote failure revocation must not stop streaming or the robot'
        )
        monkeypatch.setattr(
            remote_node.rospy,
            'wait_for_service',
            lambda *_args, **_kwargs: pytest.fail(
                'remote failure revocation must not wait for a ROS service'
            ),
        )
        monkeypatch.setattr(
            remote_node.rospy,
            'ServiceProxy',
            lambda *_args, **_kwargs: pytest.fail(
                'remote failure revocation must not create a ROS service'
            ),
        )

        result = remote_node.RemoteGrasp6DNode._accept_prediction(
            node,
            prepared,
        )

        assert result['funnel']['primary_failure'] == failure_code
        assert node.execution_plan_controller.invalid_streak == 1
        assert_execution_authority_retained(node)
    finally:
        node.shutdown_streaming_worker()


def test_worker_remote_error_without_prepared_prediction_keeps_execution(
    monkeypatch,
):
    def fail_before_prepared(_ticket):
        error = RuntimeError(
            'remote failed and tabletop produced no candidates'
        )
        error.__cause__ = ConnectionError('connection refused')
        raise error

    node = streaming_node(
        clock=MutableClock(20.0),
        prepare=fail_before_prepared,
        start_worker=True,
    )
    seed_stream_execution_authority(node)
    node.stop_streaming = lambda *_args, **_kwargs: pytest.fail(
        'worker failure revocation must not stop streaming or the robot'
    )
    monkeypatch.setattr(
        remote_node.rospy,
        'wait_for_service',
        lambda *_args, **_kwargs: pytest.fail(
            'worker failure revocation must not wait for a ROS service'
        ),
    )
    monkeypatch.setattr(
        remote_node.rospy,
        'ServiceProxy',
        lambda *_args, **_kwargs: pytest.fail(
            'worker failure revocation must not create a ROS service'
        ),
    )
    try:
        node.start_streaming()
        node.submit_stream_snapshot(snapshot(19.8))
        wait_until(lambda: len(node.pipeline_metrics) == 1)

        assert node.pipeline_metrics[0]['status'] == 'PREDICT_FAILED'
        assert node.execution_plan_controller.invalid_streak == 1
        assert_execution_authority_retained(node)
    finally:
        node.shutdown_streaming_worker()


def test_stale_ticket_cannot_revoke_newer_execution_authority():
    node = streaming_node(clock=MutableClock(20.0), start_worker=False)
    try:
        node.start_streaming()
        node.submit_stream_snapshot(snapshot(19.7))
        stale_ticket = node._stream_worker_ticket
        newer_ticket = InferenceTicket(
            request_id=stale_ticket.request_id + 1,
            generation=stale_ticket.generation,
            snapshot_stamp_sec=19.8,
            target_epoch=stale_ticket.target_epoch,
            payload=None,
            submitted_monotonic_sec=19.8,
        )
        seed_stream_execution_authority(node, authority_ticket=newer_ticket)
        initial_rich_messages = len(node.rich_plan_pub.messages)
        initial_plan_messages = len(node.plan_pub.messages)

        revoked = node._revoke_execution_authority_for_ticket(
            stale_ticket,
            'WSL_UNAVAILABLE',
            'stale remote failure',
        )

        assert revoked is False
        assert node.execution_plan_controller.execution_plan_id == 'old-plan'
        assert node.latest_rich_plan.plan_id == 'old-plan'
        assert node.latest_plan.poses == ('old-pose',)
        assert len(node.rich_plan_pub.messages) == initial_rich_messages
        assert len(node.plan_pub.messages) == initial_plan_messages
    finally:
        node.shutdown_streaming_worker()


def test_target_loss_during_robot_execution_keeps_frozen_authority():
    node = streaming_node(clock=MutableClock(20.0), start_worker=False)
    try:
        seed_stream_execution_authority(node)
        node.robot_execution_active = True
        node.geometry_pub = RecordingPublisher()

        invalid_geometry = node._invalidate_geometry(
            'TARGET_LOST',
            'target object is not detected',
        )

        assert 'TARGET_LOST' in invalid_geometry.failure_reason
        assert node._geometry_invalidation_generation == 1
        assert node.latest_object_geometry.failure_reason == (
            invalid_geometry.failure_reason
        )
        assert node.geometry_pub.messages[-1].failure_reason == (
            invalid_geometry.failure_reason
        )
        assert_execution_authority_retained(node)
    finally:
        node.shutdown_streaming_worker()


@pytest.mark.parametrize(
    'failure_code',
    [
        'WSL_UNAVAILABLE',
        'WSL_PREDICT_FAILED',
        'GRASPNET_PROTOCOL_INVALID',
    ],
)
def test_remote_failure_preview_keeps_old_execution_without_actuation(
    monkeypatch,
    failure_code,
):
    node = streaming_node(clock=MutableClock(20.0), start_worker=False)
    try:
        node.start_streaming()
        node.submit_stream_snapshot(snapshot(19.8))
        ticket = node._stream_worker_ticket
        node.moveit_top_n = 5
        seed_stream_execution_authority(node)
        node._latest_geometry_estimate = tabletop_geometry()
        prepared = remote_node.PreparedPrediction(
            ticket=ticket,
            snapshot=types.SimpleNamespace(
                object_msg=types.SimpleNamespace(detected=True, label='carton')
            ),
            stamp=remote_node.rospy.Time.from_sec(19.8),
            geometry=tabletop_geometry(),
            pose_estimator=types.SimpleNamespace(transform_sha256='frozen-tf'),
            graspnet_input=None,
            candidates=(),
            remote_diagnostics={},
            remote_performance={},
            tabletop_candidates=('geometry-candidate',),
            remote_failure_code=failure_code,
            remote_failure_reason='connection refused',
            model_choice='carton_segment',
        )
        node._activate_prepared_geometry = lambda _prepared: True
        node._run_candidate_gate_audit = lambda *_args, **_kwargs: {
            'summary': {},
            'rows': [],
        }
        local_funnel = {
            'input_count': 1,
            'stage_counts': {
                'locally_valid': {'entered': 1, 'passed': 1, 'rejected': 0}
            },
            'source_counts': {
                'graspnet': {'input': 0, 'locally_valid': 0},
                'tabletop_geometry': {'input': 1, 'locally_valid': 1},
            },
            'rejection_counts': {failure_code: 1},
            'rejection_ratios': {failure_code: 1.0},
            'primary_failure': failure_code,
        }
        node._evaluate_local_candidates = lambda _prepared: (
            ('geometry-observation',),
            local_funnel,
        )
        node._update_candidate_tracker = lambda **_kwargs: ('stable',)
        node._recheck_and_score_stable = lambda *_args: ('scored',)
        selection = types.SimpleNamespace(
            selected='selected',
            checked=('selected',),
            reachable=('selected',),
            funnel=types.SimpleNamespace(
                to_dict=lambda: {
                    'stage_counts': {
                        'moveit_reachable': {
                            'entered': 1,
                            'passed': 1,
                            'rejected': 0,
                        }
                    },
                    'rejection_counts': {},
                }
            ),
        )
        monkeypatch.setattr(
            remote_node,
            'bounded_moveit_select',
            lambda *_args, **_kwargs: selection,
        )
        node._publish_selected_preview = lambda _selected: {
            'rich_plan': object(),
            'signature': 'geometry-preview',
            'score': 0.0,
            'ticket': ticket,
            'expected_generation': 1,
        }
        node._finalize_promotion_transaction = lambda *_args, **_kwargs: (
            pytest.fail('remote failure must not enter MuJoCo promotion')
        )
        node.stop_streaming = lambda *_args, **_kwargs: pytest.fail(
            'remote failure invalidation must not stop or actuate the robot'
        )
        monkeypatch.setattr(
            remote_node.rospy,
            'wait_for_service',
            lambda *_args, **_kwargs: pytest.fail(
                'remote failure invalidation must not call a ROS service'
            ),
        )
        monkeypatch.setattr(
            remote_node.rospy,
            'ServiceProxy',
            lambda *_args, **_kwargs: pytest.fail(
                'remote failure invalidation must not create a ROS service'
            ),
        )
        finalized = []
        node._finalize_streaming_gate_audit = (
            lambda *_args, **kwargs: finalized.append(
                kwargs.get('promotion_decision')
            )
        )

        result = remote_node.RemoteGrasp6DNode._accept_prediction(
            node,
            prepared,
        )

        assert result['status'] == 'PREVIEW_READY'
        assert result['funnel']['stage_counts']['preview']['passed'] == 1
        assert result['funnel']['stage_counts']['promoted']['passed'] == 0
        assert result['funnel']['primary_failure'] == failure_code
        assert finalized[0].code == failure_code
        assert node.execution_plan_controller.invalid_streak == 1
        assert_execution_authority_retained(node)
    finally:
        node.shutdown_streaming_worker()


def test_prepared_prediction_deep_freezes_request_evidence():
    diagnostics = {
        'transport': {'attempts': [1, 2]},
        'samples': np.asarray([1.0, 2.0]),
    }
    performance = {'latency': {'components_ms': [3.0, 4.0]}}
    input_audit = {'mask': {'shape': [480, 640]}}

    prepared = remote_node.PreparedPrediction(
        ticket=object(),
        snapshot=object(),
        stamp=object(),
        geometry=object(),
        pose_estimator=object(),
        graspnet_input=object(),
        candidates=(),
        remote_diagnostics=diagnostics,
        remote_performance=performance,
        graspnet_input_audit=input_audit,
    )
    diagnostics['transport']['attempts'].append(3)
    diagnostics['samples'][0] = 99.0
    performance['latency']['components_ms'][0] = 99.0
    input_audit['mask']['shape'][0] = 1

    assert prepared.remote_diagnostics['transport']['attempts'] == (1, 2)
    assert prepared.remote_diagnostics['samples'].tolist() == [1.0, 2.0]
    assert prepared.remote_performance['latency']['components_ms'] == (
        3.0,
        4.0,
    )
    assert prepared.graspnet_input_audit['mask']['shape'] == (480, 640)
    with pytest.raises(TypeError):
        prepared.remote_diagnostics['transport']['attempts'] = ()
    with pytest.raises(ValueError):
        prepared.remote_diagnostics['samples'][0] = 0.0


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


@pytest.mark.parametrize(
    'near_field_planning_active, require_all_after_ns',
    [(False, False), (True, True)],
)
def test_stream_poll_waits_for_fresh_window_and_submits_only_once(
    monkeypatch,
    near_field_planning_active,
    require_all_after_ns,
):
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
        node.near_field_planning_active = near_field_planning_active
        node.rate_hz = 2.0
        node.planning_snapshot_timeout_sec = 4.0
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
        assert first_args[1] == 0.5
        assert first_kwargs['newest_after_ns'] == 0
        assert first_kwargs['require_all_after_ns'] is require_all_after_ns
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
        sequence = types.SimpleNamespace(
            pregrasp=remote_node.PoseStamped(),
            approach=remote_node.PoseStamped(),
            grasp=pose,
            lift=remote_node.PoseStamped(),
        )
        node._candidate_plan_metrics = {}
        node._position_only_rejected_count = 0
        node._orientation_fallback_rejected_count = 0
        node._stable_variant_runtime = {
            (3, 1): {
                'prepared': types.SimpleNamespace(ticket=ticket),
                'grasp_pose': pose,
                'observation_sequence': types.SimpleNamespace(
                    pregrasp=remote_node.PoseStamped(),
                ),
                'sequence': sequence,
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


def test_far_field_moveit_checks_only_observation_pose():
    node = streaming_node(clock=MutableClock(50.0), start_worker=False)
    try:
        node.start_streaming()
        node.submit_stream_snapshot(snapshot(49.8))
        ticket = node._stream_worker_ticket
        grasp_pose = remote_node.PoseStamped()
        observation_pose = remote_node.PoseStamped()
        node._stable_variant_runtime = {
            (3, 1): {
                'prepared': types.SimpleNamespace(ticket=ticket),
                'grasp_pose': grasp_pose,
                'observation_sequence': types.SimpleNamespace(
                    pregrasp=observation_pose,
                    approach=remote_node.PoseStamped(),
                    grasp=grasp_pose,
                    lift=remote_node.PoseStamped(),
                ),
                'sequence': types.SimpleNamespace(
                    pregrasp=remote_node.PoseStamped(),
                    approach=remote_node.PoseStamped(),
                    grasp=grasp_pose,
                    lift=remote_node.PoseStamped(),
                ),
            }
        }
        checked = []

        def strict_checker(pose):
            checked.append(pose)
            return (
                MoveItResult(
                    reachable=False,
                    joint_path_cost=0.0,
                    joint_max_delta_rad=0.0,
                    reason='observation unreachable',
                    failure_code='MOVEIT_UNREACHABLE',
                ),
                {},
                '',
            )

        node._strict_moveit_evaluation = strict_checker

        result = node._check_moveit_stable_candidate(
            types.SimpleNamespace(track_id=3, variant_index=1)
        )

        assert checked == [observation_pose]
        assert result.reachable is False
        assert result.failure_code == 'MOVEIT_UNREACHABLE'
        assert result.reason == 'observation unreachable'
    finally:
        node.shutdown_streaming_worker()


def test_request_frozen_near_field_checks_ordered_sequence_after_state_changes():
    node = streaming_node(clock=MutableClock(50.0), start_worker=False)
    try:
        node.start_streaming()
        node.submit_stream_snapshot(snapshot(49.8))
        ticket = node._stream_worker_ticket
        reached = promotion_plan('reached')
        reached.poses[0].position.x = 0.31
        node.latest_rich_plan = reached
        node.robot_execution_active = False
        new_pregrasp = remote_node.PoseStamped()
        new_pregrasp.pose.position.x = 0.72
        approach_pose = remote_node.PoseStamped()
        approach_pose.pose.position.x = 0.41
        grasp_pose = remote_node.PoseStamped()
        grasp_pose.pose.position.x = 0.42
        lift_pose = remote_node.PoseStamped()
        node._stable_variant_runtime = {
            (3, 1): {
                'prepared': types.SimpleNamespace(
                    ticket=ticket,
                    near_field=True,
                ),
                'grasp_pose': grasp_pose,
                'sequence': types.SimpleNamespace(
                    pregrasp=new_pregrasp,
                    approach=approach_pose,
                    grasp=grasp_pose,
                    lift=lift_pose,
                ),
            }
        }
        checked = []
        node._strict_moveit_sequence_evaluation = lambda stages: (
            checked.extend(stages)
            or MoveItResult(
                reachable=True,
                joint_path_cost=0.6,
                joint_max_delta_rad=0.1,
                reason='ordered sequence reachable',
                collision_free=True,
                within_joint_limits=True,
                ik_valid=True,
                planning_success=True,
            ),
            {'joint_path_cost': 0.6, 'joint_max_delta': 0.1},
        )

        result = node._check_moveit_stable_candidate(
            types.SimpleNamespace(track_id=3, variant_index=1)
        )

        assert result.reachable is True
        assert result.joint_path_cost == pytest.approx(0.6)
        assert result.joint_max_delta_rad == pytest.approx(0.1)
        assert result.evidence_code == ''
        assert checked == [
            ('pregrasp', new_pregrasp),
            ('approach', approach_pose),
            ('grasp', grasp_pose),
            ('lift', lift_pose),
        ]
        assert node._candidate_plan_metrics[
            node._pose_key(grasp_pose)
        ] == {
            'joint_path_cost': 0.6,
            'joint_max_delta': 0.1,
        }
    finally:
        node.shutdown_streaming_worker()


def test_near_field_strict_sequence_checks_linear_lift_like_execution():
    node = streaming_node(clock=MutableClock(50.0), start_worker=False)
    original_wait = remote_node.rospy.wait_for_service
    original_proxy = remote_node.rospy.ServiceProxy
    observed = []
    remote_node.rospy.wait_for_service = (
        lambda name, timeout=None: observed.append(('wait', name, timeout))
    )

    def service_proxy(name, service_type):
        observed.append(('proxy', name, service_type))

        def invoke(request):
            observed.append(
                (
                    'invoke',
                    list(request.stage_names),
                    list(request.linear),
                )
            )
            return types.SimpleNamespace(
                success=True,
                failure_code='',
                failed_stage='',
                joint_path_cost=0.4,
                joint_max_delta=0.1,
                message='ordered sequence planned',
            )

        return invoke

    remote_node.rospy.ServiceProxy = service_proxy
    try:
        stages = tuple(
            (name, remote_node.PoseStamped())
            for name in ('pregrasp', 'approach', 'grasp', 'lift')
        )
        result, metrics = node._strict_moveit_sequence_evaluation(stages)
    finally:
        remote_node.rospy.wait_for_service = original_wait
        remote_node.rospy.ServiceProxy = original_proxy
        node.shutdown_streaming_worker()

    assert result.reachable is True
    assert result.collision_free is True
    assert result.within_joint_limits is True
    assert result.ik_valid is True
    assert result.planning_success is True
    assert result.failure_code == ''
    assert result.evidence_code == ''
    assert metrics == {
        'joint_path_cost': pytest.approx(0.4),
        'joint_max_delta': pytest.approx(0.1),
    }
    invoke = [item for item in observed if item[0] == 'invoke']
    assert invoke == [
        (
            'invoke',
            ['pregrasp', 'approach', 'grasp', 'lift'],
            [False, True, True, True],
        )
    ]


def test_free_space_sequence_resolver_preserves_xyz_and_contact_orientations(
    monkeypatch,
):
    node = streaming_node(clock=MutableClock(50.0), start_worker=False)
    observed = []
    try:
        stages = []
        for index, name in enumerate(
            ('pregrasp', 'approach', 'grasp', 'lift')
        ):
            pose = remote_node.PoseStamped()
            pose.header.frame_id = 'base_link'
            pose.pose.position.x = 0.1 * index
            pose.pose.position.y = -0.2
            pose.pose.position.z = 0.3
            pose.pose.orientation.w = 1.0
            stages.append((name, pose))
        resolved = [remote_node.deepcopy(pose) for _name, pose in stages]
        resolved[0].pose.orientation.z = 0.1
        resolved[0].pose.orientation.w = math.sqrt(0.99)
        resolved[3].pose.orientation.y = 0.2
        resolved[3].pose.orientation.w = math.sqrt(0.96)

        def service_proxy(name, service_type):
            observed.append((name, service_type))

            def invoke(request):
                observed.append(
                    (
                        list(request.stage_names),
                        list(request.linear),
                        list(request.resolve_orientation),
                    )
                )
                return types.SimpleNamespace(
                    success=True,
                    failure_code='',
                    failed_stage='',
                    resolved_targets=resolved,
                    joint_path_cost=0.8,
                    joint_max_delta=0.4,
                    max_position_error=0.0002,
                    message=(
                        'policy=deterministic_geodesic_collision_ik '
                        'candidates_tested=12 '
                        'max_repeatability_error=0.000000000'
                    ),
                )

            return invoke

        monkeypatch.setattr(
            remote_node.rospy,
            'wait_for_service',
            lambda *_args, **_kwargs: None,
        )
        monkeypatch.setattr(
            remote_node.rospy,
            'ServiceProxy',
            service_proxy,
        )
        monkeypatch.setattr(
            remote_node.rospy,
            'get_param',
            lambda _name, default=None: default,
        )

        sequence, audit, code, reason = node._resolve_free_space_sequence(
            stages
        )

        assert sequence is not None, reason
        assert code == ''
        assert audit['available'] is True
        assert audit['policy'] == 'deterministic_geodesic_collision_ik'
        assert audit['joint_path_cost'] == pytest.approx(0.8)
        assert audit['joint_max_delta'] == pytest.approx(0.4)
        assert audit['max_position_error'] == pytest.approx(0.0002)
        assert sequence.pregrasp.pose.orientation.z == pytest.approx(0.1)
        assert sequence.lift.pose.orientation.y == pytest.approx(0.2)
        assert sequence.approach.pose.orientation.w == pytest.approx(1.0)
        assert sequence.grasp.pose.orientation.w == pytest.approx(1.0)
        assert observed[-1] == (
            ['pregrasp', 'approach', 'grasp', 'lift'],
            [False, True, True, False],
            [True, False, False, True],
        )
    finally:
        node.shutdown_streaming_worker()


def test_near_field_fallback_rechecks_resolved_geometry_before_strict_plan():
    node = streaming_node(clock=MutableClock(50.0), start_worker=False)
    try:
        node.start_streaming()
        node.submit_stream_snapshot(snapshot(49.8))
        ticket = node._stream_worker_ticket
        original = remote_node.Grasp6DSequence(
            pregrasp=remote_node.PoseStamped(),
            approach=remote_node.PoseStamped(),
            grasp=remote_node.PoseStamped(),
            lift=remote_node.PoseStamped(),
        )
        original.grasp.pose.position.x = 0.42
        resolved = remote_node.deepcopy(original)
        resolved.pregrasp.pose.orientation.z = 0.1
        resolved.pregrasp.pose.orientation.w = math.sqrt(0.99)
        resolved.lift.pose.orientation.y = 0.2
        resolved.lift.pose.orientation.w = math.sqrt(0.96)
        prepared = types.SimpleNamespace(
            ticket=ticket,
            near_field=True,
            geometry=object(),
        )
        node._stable_variant_runtime = {
            (3, 1): {
                'prepared': prepared,
                'grasp_pose': original.grasp,
                'sequence': original,
                'scored_candidate': object(),
            }
        }
        strict_calls = []

        def strict_sequence(stages):
            strict_calls.append(tuple(stages))
            if len(strict_calls) == 1:
                return (
                    MoveItResult(
                        reachable=False,
                        joint_path_cost=0.2,
                        joint_max_delta_rad=0.1,
                        reason='original pregrasp unreachable',
                        failure_code='MOVEIT_UNREACHABLE',
                    ),
                    {'joint_path_cost': 0.2, 'joint_max_delta': 0.1},
                )
            return (
                MoveItResult(
                    reachable=True,
                    joint_path_cost=0.7,
                    joint_max_delta_rad=0.3,
                    reason='resolved sequence reachable',
                    collision_free=True,
                    within_joint_limits=True,
                    ik_valid=True,
                    planning_success=True,
                ),
                {'joint_path_cost': 0.7, 'joint_max_delta': 0.3},
            )

        passing_gate = CandidateGateResult(
            ok=True,
            failure_code='',
            failure_reason='',
            required_open_width_m=0.04,
            center_distance_m=0.0,
            support_clearance_m=0.003,
            jaw_alignment=1.0,
            motion_cost=0.0,
            geometry_cost=0.0,
            failed_gate='',
            passed_gate_count=6,
        )
        geometry_calls = []
        node._strict_moveit_sequence_evaluation = strict_sequence
        node._resolve_free_space_sequence = lambda _stages: (
            resolved,
            {
                'available': True,
                'policy': 'deterministic_geodesic_collision_ik',
            },
            '',
            'resolved',
        )
        node._resolved_sequence_geometry_gate = (
            lambda runtime, sequence: (
                geometry_calls.append((runtime, sequence)) or passing_gate
            )
        )

        result = node._check_moveit_stable_candidate(
            types.SimpleNamespace(track_id=3, variant_index=1)
        )

        assert result.reachable is True
        assert result.joint_path_cost == pytest.approx(0.7)
        assert len(strict_calls) == 2
        assert len(geometry_calls) == 1
        runtime = node._stable_variant_runtime[(3, 1)]
        assert runtime['sequence'] is resolved
        assert runtime['geometry_gate'] is passing_gate
        assert runtime['orientation_resolution']['available'] is True
        assert [
            stage[1]
            for stage in strict_calls[1]
        ] == [
            resolved.pregrasp,
            resolved.approach,
            resolved.grasp,
            resolved.lift,
        ]
    finally:
        node.shutdown_streaming_worker()


def test_near_field_exact_sequence_cache_reuses_only_attested_safe_work():
    node = streaming_node(clock=MutableClock(50.0), start_worker=False)
    try:
        node.start_streaming()
        node.submit_stream_snapshot(snapshot(49.8))
        ticket = node._stream_worker_ticket
        node.latest_joint_state = types.SimpleNamespace(
            name=['Joint%d' % index for index in range(1, 7)],
            position=[0.1 * index for index in range(6)],
        )
        node._candidate_plan_metrics = {}
        original = remote_node.Grasp6DSequence(
            pregrasp=remote_node.PoseStamped(),
            approach=remote_node.PoseStamped(),
            grasp=remote_node.PoseStamped(),
            lift=remote_node.PoseStamped(),
        )
        original.grasp.pose.position.x = 0.42
        resolved = remote_node.deepcopy(original)
        resolved.pregrasp.pose.orientation.z = 0.1
        resolved.pregrasp.pose.orientation.w = math.sqrt(0.99)
        resolved.lift.pose.orientation.y = 0.2
        resolved.lift.pose.orientation.w = math.sqrt(0.96)
        prepared = types.SimpleNamespace(
            ticket=ticket,
            near_field=True,
            geometry=object(),
        )
        node._stable_variant_runtime = {
            (3, 0): {
                'prepared': prepared,
                'grasp_pose': original.grasp,
                'sequence': remote_node.deepcopy(original),
                'scored_candidate': object(),
            },
            (4, 0): {
                'prepared': prepared,
                'grasp_pose': remote_node.deepcopy(original.grasp),
                'sequence': remote_node.deepcopy(original),
                'scored_candidate': object(),
            },
        }
        strict_calls = []

        def strict_sequence(stages):
            strict_calls.append(tuple(stages))
            is_resolved = (
                abs(stages[0][1].pose.orientation.z - 0.1) < 1e-12
            )
            if not is_resolved:
                return (
                    MoveItResult(
                        reachable=False,
                        joint_path_cost=0.2,
                        joint_max_delta_rad=0.1,
                        reason='original pregrasp unreachable',
                        failure_code='MOVEIT_UNREACHABLE',
                    ),
                    {'joint_path_cost': 0.2, 'joint_max_delta': 0.1},
                )
            return (
                MoveItResult(
                    reachable=True,
                    joint_path_cost=0.7,
                    joint_max_delta_rad=0.3,
                    reason='resolved sequence reachable',
                    collision_free=True,
                    within_joint_limits=True,
                    ik_valid=True,
                    planning_success=True,
                ),
                {'joint_path_cost': 0.7, 'joint_max_delta': 0.3},
            )

        resolver_calls = []

        def resolve(stages):
            resolver_calls.append(tuple(stages))
            return (
                remote_node.deepcopy(resolved),
                {
                    'available': True,
                    'policy': 'deterministic_geodesic_collision_ik',
                    'policy_attested': True,
                },
                '',
                'resolved',
            )

        passing_gate = CandidateGateResult(
            ok=True,
            failure_code='',
            failure_reason='',
            required_open_width_m=0.04,
            center_distance_m=0.0,
            support_clearance_m=0.003,
            jaw_alignment=1.0,
            motion_cost=0.0,
            geometry_cost=0.0,
            failed_gate='',
            passed_gate_count=6,
        )
        geometry_calls = []
        node._strict_moveit_sequence_evaluation = strict_sequence
        node._resolve_free_space_sequence = resolve
        node._resolved_sequence_geometry_gate = (
            lambda runtime, sequence: (
                geometry_calls.append((runtime, sequence)) or passing_gate
            )
        )

        first = node._check_moveit_stable_candidate(
            types.SimpleNamespace(track_id=3, variant_index=0)
        )
        second = node._check_moveit_stable_candidate(
            types.SimpleNamespace(track_id=4, variant_index=0)
        )

        assert first.reachable is True
        assert second.reachable is True
        # Original strict failures remain independent stochastic attempts.
        # Only the deterministic resolver and an already-proven strict
        # success may be reused at the exact unchanged joint state.
        assert len(strict_calls) == 3
        assert len(resolver_calls) == 1
        assert len(geometry_calls) == 2
        second_cache = node._stable_variant_runtime[(4, 0)][
            'request_plan_cache'
        ]
        assert second_cache['original_strict_sequence']['hit'] is False
        assert second_cache[
            'free_space_orientation_resolution'
        ]['hit'] is True
        assert second_cache['resolved_strict_sequence']['hit'] is True
    finally:
        node.shutdown_streaming_worker()


def test_near_field_cache_reuses_attested_deterministic_unreachable_result():
    node = streaming_node(clock=MutableClock(50.0), start_worker=False)
    try:
        node.start_streaming()
        node.submit_stream_snapshot(snapshot(49.8))
        ticket = node._stream_worker_ticket
        node.latest_joint_state = types.SimpleNamespace(
            name=['Joint%d' % index for index in range(1, 7)],
            position=[0.0] * 6,
        )
        stages = tuple(
            (name, remote_node.PoseStamped())
            for name in ('pregrasp', 'approach', 'grasp', 'lift')
        )
        prepared = types.SimpleNamespace(ticket=ticket)
        calls = []

        def resolve(received):
            calls.append(tuple(received))
            return (
                None,
                {
                    'available': False,
                    'policy': 'deterministic_geodesic_collision_ik',
                    'policy_attested': True,
                    'failed_stage': 'approach',
                },
                'MOVEIT_UNREACHABLE',
                'Cartesian planning fraction is below the hard bound',
            )

        node._resolve_free_space_sequence = resolve
        first_runtime = {}
        second_runtime = {}

        first = node._cached_free_space_sequence_resolution(
            stages,
            prepared,
            first_runtime,
        )
        second = node._cached_free_space_sequence_resolution(
            stages,
            prepared,
            second_runtime,
        )

        assert first == second
        assert len(calls) == 1
        assert first_runtime['request_plan_cache'][
            'free_space_orientation_resolution'
        ]['stored'] is True
        assert second_runtime['request_plan_cache'][
            'free_space_orientation_resolution'
        ]['hit'] is True
    finally:
        node.shutdown_streaming_worker()


@pytest.mark.parametrize(
    'failed_stage',
    ['pregrasp', 'approach', 'grasp', 'lift'],
)
def test_active_near_field_moveit_rejects_unreachable_remaining_stage(
    failed_stage,
):
    node = streaming_node(clock=MutableClock(50.0), start_worker=False)
    try:
        node.start_streaming()
        node.submit_stream_snapshot(snapshot(49.8))
        ticket = node._stream_worker_ticket
        node.latest_rich_plan = promotion_plan('reached')
        node.robot_execution_active = True
        node.near_field_planning_active = True
        approach_pose = remote_node.PoseStamped()
        grasp_pose = remote_node.PoseStamped()
        node._stable_variant_runtime = {
            (3, 1): {
                'prepared': types.SimpleNamespace(ticket=ticket),
                'grasp_pose': grasp_pose,
                'sequence': types.SimpleNamespace(
                    pregrasp=remote_node.PoseStamped(),
                    approach=approach_pose,
                    grasp=grasp_pose,
                    lift=remote_node.PoseStamped(),
                ),
            }
        }
        checked = []

        def strict_checker(stages):
            checked.extend(name for name, _pose in stages)
            return (
                MoveItResult(
                    reachable=False,
                    joint_path_cost=0.2,
                    joint_max_delta_rad=0.1,
                    reason='strict sequence %s unreachable: planning failed'
                    % failed_stage,
                    failure_code='MOVEIT_UNREACHABLE',
                ),
                {'joint_path_cost': 0.2, 'joint_max_delta': 0.1},
            )

        node._strict_moveit_sequence_evaluation = strict_checker

        result = node._check_moveit_stable_candidate(
            types.SimpleNamespace(track_id=3, variant_index=1)
        )

        assert result.reachable is False
        assert result.failure_code == 'MOVEIT_UNREACHABLE'
        assert result.reason == (
            'strict sequence %s unreachable: planning failed'
            % failed_stage
        )
        assert tuple(checked) == (
            'pregrasp',
            'approach',
            'grasp',
            'lift',
        )
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
            grasp_pose = remote_node.PoseStamped()
            runtime[(track_id, 0)] = {
                'prepared': types.SimpleNamespace(ticket=ticket),
                'grasp_pose': grasp_pose,
                'observation_sequence': types.SimpleNamespace(
                    pregrasp=remote_node.PoseStamped(),
                ),
                'sequence': types.SimpleNamespace(
                    pregrasp=remote_node.PoseStamped(),
                    approach=remote_node.PoseStamped(),
                    grasp=grasp_pose,
                    lift=remote_node.PoseStamped(),
                ),
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


def test_near_field_moveit_continuation_gate_reserves_server_lifetime():
    node = remote_node.RemoteGrasp6DNode.__new__(
        remote_node.RemoteGrasp6DNode
    )
    node.mujoco_server_max_snapshot_age_sec = 120.0
    node.mujoco_selection_snapshot_reserve_sec = 30.0
    clock = MutableClock(109.999)
    node._execution_plan_validity_now_sec = clock
    prepared = types.SimpleNamespace(
        ticket=types.SimpleNamespace(snapshot_stamp_sec=20.0)
    )

    continue_checking = node._near_field_moveit_continuation_gate(
        prepared
    )

    assert continue_checking() is True
    clock.value = 110.0
    assert continue_checking() is False


@pytest.mark.parametrize(
    ('max_age_sec', 'reserve_sec'),
    [
        (float('nan'), 30.0),
        (120.0, float('nan')),
        (120.0, 120.0),
        (20.0, 30.0),
    ],
)
def test_near_field_moveit_continuation_gate_fails_closed_on_bad_contract(
    max_age_sec,
    reserve_sec,
):
    node = remote_node.RemoteGrasp6DNode.__new__(
        remote_node.RemoteGrasp6DNode
    )
    node.mujoco_server_max_snapshot_age_sec = max_age_sec
    node.mujoco_selection_snapshot_reserve_sec = reserve_sec
    node._execution_plan_validity_now_sec = lambda: 25.0
    prepared = types.SimpleNamespace(
        ticket=types.SimpleNamespace(snapshot_stamp_sec=20.0)
    )

    assert (
        node._near_field_moveit_continuation_gate(prepared)()
        is False
    )


def test_remote_health_binds_mujoco_server_snapshot_age(monkeypatch):
    node = remote_node.RemoteGrasp6DNode.__new__(
        remote_node.RemoteGrasp6DNode
    )
    node.mujoco_server_max_snapshot_age_sec = 30.0
    node.require_candidate_depth = True
    node.client = types.SimpleNamespace(
        server_url='http://127.0.0.1:8000',
        health=lambda: {
            'ok': True,
            'backend': 'graspnet_baseline',
            'loaded': True,
            'protocol_version': 3,
            'candidate_fields': ['depth_m'],
            'digital_twin': {
                'max_snapshot_age_sec': 120.0,
            },
        },
    )
    node.status_pub = RecordingPublisher()
    monkeypatch.setattr(remote_node.rospy, 'loginfo', lambda *_args: None)

    node._check_remote_health()

    assert node.mujoco_server_max_snapshot_age_sec == 120.0
    assert node.status_pub.messages == []


@pytest.mark.parametrize(
    'near_field, ready_request_id',
    [(False, 3), (True, 2)],
)
def test_phase_evidence_count_enters_current_recheck_and_moveit_top_n(
    monkeypatch,
    near_field,
    ready_request_id,
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
    node.candidate_max_joint_delta_rad = 1.8
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

    def fake_bounded(
        candidates,
        checker,
        top_n,
        max_joint_delta_rad=0.0,
        ranking_key=None,
        exhaustive=False,
        first_reachable_by_rank=False,
        continue_checking=None,
        continuation_stop_reason='',
    ):
        del checker
        bounded_calls.append(
            (
                tuple(candidates),
                top_n,
                max_joint_delta_rad,
                ranking_key,
                exhaustive,
                first_reachable_by_rank,
                continue_checking,
                continuation_stop_reason,
            )
        )
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

    predictions = [prepared_prediction(request_id) for request_id in (1, 2, 3)]
    for prepared in predictions:
        prepared.near_field = near_field
    first = node._accept_prediction(predictions[0])
    second = node._accept_prediction(predictions[1])
    third = (
        node._accept_prediction(predictions[2])
        if ready_request_id == 3
        else None
    )

    assert first['status'] == 'STABILITY_PENDING'
    if ready_request_id == 3:
        assert second['status'] == 'STABILITY_PENDING'
        assert third['status'] == 'PREVIEW_READY'
    else:
        assert second['status'] == 'PREVIEW_READY'
        assert third is None
    assert published == [bounded_calls[0][0][0]]
    assert recheck_calls[0][0] == ready_request_id
    assert len(recheck_calls[0][1]) == 1
    assert bounded_calls[0][1] == 5
    assert bounded_calls[0][2] == 1.8
    if near_field:
        assert bounded_calls[0][3] is None
    else:
        assert callable(bounded_calls[0][3])
    assert bounded_calls[0][4] is near_field
    assert bounded_calls[0][5] is (not near_field)
    if near_field:
        assert callable(bounded_calls[0][6])
    else:
        assert bounded_calls[0][6] is None
    assert (
        bounded_calls[0][7]
        == 'MUJOCO_SNAPSHOT_RESERVE_REACHED'
    )

def test_direct_near_field_uses_current_request_and_first_reachable_without_mujoco(
    monkeypatch,
):
    node = remote_node.RemoteGrasp6DNode.__new__(
        remote_node.RemoteGrasp6DNode
    )
    node.target_instance_epoch = 4
    node._last_model_choice = 'carton_segment'
    node._stream_condition = threading.Condition(threading.RLock())
    node._stream_shutdown = threading.Event()
    node.streaming_enabled = True
    node._stream_generation = 1
    node.tracker = CandidateTracker(
        TrackingConfig(window_size=5, min_hits=3)
    )
    node.moveit_top_n = 5
    node.candidate_max_joint_delta_rad = 1.8
    node.near_field_strategy = 'single_snapshot_direct'
    node.near_field_direct_timeout_sec = 30.0
    node._execution_plan_validity_now_sec = lambda: 21.0
    node._activate_prepared_geometry = lambda _prepared: True
    current = (
        matching_observation(1),
        matching_observation(1),
    )
    node._evaluate_local_candidates = lambda _prepared: (
        current,
        {
            'input_count': 2,
            'stage_counts': {
                'locally_valid': {
                    'entered': 2,
                    'passed': 2,
                    'rejected': 0,
                }
            },
            'rejection_counts': {},
            'rejection_ratios': {},
            'primary_failure': None,
        },
    )
    rechecked = []
    node._recheck_and_score_stable = lambda _prepared, stable: (
        rechecked.append(tuple(stable)) or tuple(stable)
    )
    node._dedupe_scored_tabletop_candidates_for_moveit = (
        lambda scored: tuple(scored)
    )
    captured = {}

    def direct_selection(
        candidates,
        checker,
        top_n,
        max_joint_delta_rad=0.0,
        ranking_key=None,
        exhaustive=False,
        first_reachable_by_rank=False,
        continue_checking=None,
        continuation_stop_reason='',
    ):
        del checker
        captured.update(
            {
                'candidates': tuple(candidates),
                'top_n': top_n,
                'max_joint_delta_rad': max_joint_delta_rad,
                'ranking_key': ranking_key,
                'exhaustive': exhaustive,
                'first_reachable_by_rank': first_reachable_by_rank,
                'continue_checking': continue_checking,
                'continuation_stop_reason': continuation_stop_reason,
            }
        )
        return types.SimpleNamespace(
            selected=tuple(candidates)[0],
            checked=(tuple(candidates)[0],),
            reachable=(tuple(candidates)[0],),
            terminated_early=False,
            termination_reason='',
            funnel=types.SimpleNamespace(
                to_dict=lambda: {
                    'stage_counts': {
                        'moveit_checked': {
                            'entered': 1,
                            'passed': 1,
                            'rejected': 0,
                        },
                        'moveit_reachable': {
                            'entered': 1,
                            'passed': 1,
                            'rejected': 0,
                        },
                    },
                    'rejection_counts': {},
                    'rejection_ratios': {},
                    'primary_failure': None,
                }
            ),
        )

    monkeypatch.setattr(
        remote_node,
        'bounded_moveit_select',
        direct_selection,
    )
    node._check_moveit_stable_candidate = lambda _candidate: None
    node._screen_near_field_selection_with_mujoco = (
        lambda *_args, **_kwargs: pytest.fail(
            'direct near-field mode must not call MuJoCo selection'
        )
    )
    published = []
    node._publish_selected_preview = published.append
    prepared = prepared_prediction(1)
    prepared.near_field = True

    result = node._accept_prediction(prepared)

    assert result['status'] == 'PREVIEW_READY'
    assert len(rechecked) == 1
    assert len(rechecked[0]) == len(current)
    assert all(
        isinstance(candidate, StableCandidate)
        for candidate in rechecked[0]
    )
    assert all(candidate.hit_count == 1 for candidate in rechecked[0])
    assert all(candidate.window_count == 1 for candidate in rechecked[0])
    assert all(
        candidate.hit_request_ids == (prepared.ticket.request_id,)
        for candidate in rechecked[0]
    )
    assert all(
        candidate.position_dispersion_m == 0.0
        for candidate in rechecked[0]
    )
    assert all(
        candidate.orientation_dispersion_rad == 0.0
        for candidate in rechecked[0]
    )
    assert captured['candidates'] == rechecked[0]
    assert captured['top_n'] == len(current)
    assert callable(captured['ranking_key'])
    assert captured['exhaustive'] is False
    assert captured['first_reachable_by_rank'] is True
    assert callable(captured['continue_checking'])
    assert captured['continue_checking']() is True
    assert (
        captured['continuation_stop_reason']
        == 'NEAR_FIELD_DIRECT_TIMEOUT'
    )
    assert published == [rechecked[0][0]]
    assert (
        result['funnel']['snapshot_evidence'][
            'disjoint_window_required'
        ]
        is False
    )
    assert result['funnel']['tracking_evidence']['required_hits'] == 1


def test_direct_near_field_empty_current_request_has_exact_status():
    node = remote_node.RemoteGrasp6DNode.__new__(
        remote_node.RemoteGrasp6DNode
    )
    node.target_instance_epoch = 4
    node._last_model_choice = 'carton_segment'
    node._stream_condition = threading.Condition(threading.RLock())
    node._stream_shutdown = threading.Event()
    node.streaming_enabled = True
    node._stream_generation = 1
    node.tracker = CandidateTracker(
        TrackingConfig(window_size=5, min_hits=3)
    )
    node.near_field_strategy = 'single_snapshot_direct'
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
            'rejection_counts': {},
            'rejection_ratios': {},
            'primary_failure': None,
        },
    )
    node._observe_execution_candidate_invalid = lambda **_kwargs: None
    prepared = prepared_prediction(1)
    prepared.near_field = True

    result = node._accept_prediction(prepared)

    assert result['status'] == 'NEAR_FIELD_NO_HARD_SAFE_CANDIDATE'
    assert (
        result['funnel']['snapshot_evidence'][
            'disjoint_window_required'
        ]
        is False
    )


def test_direct_near_field_deadline_is_exactly_thirty_seconds():
    node = remote_node.RemoteGrasp6DNode.__new__(
        remote_node.RemoteGrasp6DNode
    )
    node.near_field_direct_timeout_sec = 30.0
    clock = MutableClock(49.999)
    node._execution_plan_validity_now_sec = clock
    prepared = types.SimpleNamespace(
        ticket=types.SimpleNamespace(snapshot_stamp_sec=20.0)
    )

    continue_checking = node._direct_near_field_deadline_gate(prepared)

    assert continue_checking() is True
    clock.value = 50.0
    assert continue_checking() is False


@pytest.mark.parametrize(
    'terminated_early,termination_reason,expected_status',
    (
        (
            True,
            'NEAR_FIELD_DIRECT_TIMEOUT',
            'NEAR_FIELD_DIRECT_TIMEOUT',
        ),
        (False, '', 'NEAR_FIELD_NO_REACHABLE_CANDIDATE'),
    ),
)
def test_direct_near_field_no_selection_has_exact_terminal_status(
    monkeypatch,
    terminated_early,
    termination_reason,
    expected_status,
):
    node = remote_node.RemoteGrasp6DNode.__new__(
        remote_node.RemoteGrasp6DNode
    )
    node.target_instance_epoch = 4
    node._last_model_choice = 'carton_segment'
    node._stream_condition = threading.Condition(threading.RLock())
    node._stream_shutdown = threading.Event()
    node.streaming_enabled = True
    node._stream_generation = 1
    node.tracker = CandidateTracker(
        TrackingConfig(window_size=5, min_hits=3)
    )
    node.moveit_top_n = 5
    node.candidate_max_joint_delta_rad = 1.8
    node.near_field_strategy = 'single_snapshot_direct'
    node.near_field_direct_timeout_sec = 30.0
    node._execution_plan_validity_now_sec = lambda: 21.0
    node._activate_prepared_geometry = lambda _prepared: True
    node._evaluate_local_candidates = lambda _prepared: (
        (matching_observation(1),),
        {
            'input_count': 1,
            'stage_counts': {
                'locally_valid': {
                    'entered': 1,
                    'passed': 1,
                    'rejected': 0,
                }
            },
            'rejection_counts': {},
            'rejection_ratios': {},
            'primary_failure': None,
        },
    )
    node._recheck_and_score_stable = (
        lambda _prepared, stable: tuple(stable)
    )
    node._dedupe_scored_tabletop_candidates_for_moveit = (
        lambda scored: tuple(scored)
    )
    node._check_moveit_stable_candidate = lambda _candidate: None
    node._publish_selected_preview = lambda _candidate: pytest.fail(
        'a direct near-field terminal failure cannot publish a preview'
    )
    node._observe_execution_candidate_invalid = lambda **_kwargs: None

    def no_selection(*_args, **_kwargs):
        return types.SimpleNamespace(
            selected=None,
            checked=(),
            reachable=(),
            terminated_early=terminated_early,
            termination_reason=termination_reason,
            funnel=types.SimpleNamespace(
                to_dict=lambda: {
                    'stage_counts': {
                        'moveit_reachable': {
                            'entered': 0,
                            'passed': 0,
                            'rejected': 0,
                        }
                    },
                    'rejection_counts': {},
                    'rejection_ratios': {},
                    'primary_failure': None,
                }
            ),
        )

    monkeypatch.setattr(
        remote_node,
        'bounded_moveit_select',
        no_selection,
    )
    prepared = prepared_prediction(1)
    prepared.near_field = True

    result = node._accept_prediction(prepared)

    assert result['status'] == expected_status


def test_snapshot_budget_terminal_status_survives_primary_failure_count(
    monkeypatch,
):
    node = remote_node.RemoteGrasp6DNode.__new__(
        remote_node.RemoteGrasp6DNode
    )
    node.target_instance_epoch = 4
    node._last_model_choice = 'carton_segment'
    node._stream_condition = threading.Condition(threading.RLock())
    node._stream_shutdown = threading.Event()
    node.streaming_enabled = True
    node._stream_generation = 1
    node.tracker = CandidateTracker(
        TrackingConfig(window_size=5, min_hits=3)
    )
    node.moveit_top_n = 5
    node.candidate_max_joint_delta_rad = 1.8
    node.mujoco_server_max_snapshot_age_sec = 120.0
    node.mujoco_selection_snapshot_reserve_sec = 30.0
    node._execution_plan_validity_now_sec = lambda: 20.0
    node._activate_prepared_geometry = lambda _prepared: True
    node._evaluate_local_candidates = lambda prepared: (
        (matching_observation(prepared.ticket.request_id),),
        {
            'input_count': 20,
            'stage_counts': {
                'locally_valid': {
                    'entered': 20,
                    'passed': 20,
                    'rejected': 0,
                }
            },
            'rejection_counts': {},
            'rejection_ratios': {},
            'primary_failure': None,
        },
    )
    node._recheck_and_score_stable = lambda _prepared, stable: tuple(stable)
    node._screen_near_field_selection_with_mujoco = (
        lambda _prepared, selection: (selection, '')
    )
    node._publish_selected_preview = lambda _candidate: pytest.fail(
        'an early-terminated unreachable subset cannot publish a preview'
    )
    node._observe_execution_candidate_invalid = lambda **_kwargs: None

    def stopped_selection(*_args, **_kwargs):
        return types.SimpleNamespace(
            selected=None,
            checked=tuple(range(17)),
            reachable=(),
            terminated_early=True,
            termination_reason='MUJOCO_SNAPSHOT_RESERVE_REACHED',
            funnel=types.SimpleNamespace(
                to_dict=lambda: {
                    'input_count': 20,
                    'stage_counts': {
                        'moveit_checked': {
                            'entered': 17,
                            'passed': 17,
                            'rejected': 0,
                        },
                        'moveit_reachable': {
                            'entered': 17,
                            'passed': 0,
                            'rejected': 17,
                        },
                    },
                    'rejection_counts': {'MOVEIT_UNREACHABLE': 17},
                    'rejection_ratios': {
                        'MOVEIT_UNREACHABLE': 17.0 / 20.0
                    },
                    'primary_failure': 'MOVEIT_UNREACHABLE',
                }
            ),
        )

    monkeypatch.setattr(
        remote_node,
        'bounded_moveit_select',
        stopped_selection,
    )

    first = prepared_prediction(1)
    second = prepared_prediction(2)
    first.near_field = True
    second.near_field = True

    assert node._accept_prediction(first)['status'] == 'STABILITY_PENDING'
    result = node._accept_prediction(second)

    assert result['status'] == 'MUJOCO_SNAPSHOT_RESERVE_REACHED'
    assert result['funnel']['primary_failure'] == 'MOVEIT_UNREACHABLE'
    assert result['funnel']['rejection_counts']['MOVEIT_UNREACHABLE'] == 17


def test_current_recheck_binds_conservative_latest_required_width():
    node = remote_node.RemoteGrasp6DNode.__new__(remote_node.RemoteGrasp6DNode)
    configure_identity_handeye(node)
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
    node.camera_visibility_gate_enabled = True
    node.camera_visibility_diagnostic_enabled = True
    visibility_sequences = []

    def record_visibility(_pose, _target, sequence=None):
        visibility_sequences.append(sequence)
        return True, [
            {
                'stage': 'pregrasp',
                'u': 320.0,
                'v': 240.0,
                'depth_m': 0.2,
                'center_cost': 0.0,
                'margin_x_px': 36,
                'margin_y_px': 36,
            }
        ], 'visible'

    node._candidate_visibility_metrics = record_visibility
    node.gripper_tool_jaw_axis = 'y'
    node.gripper_tool_finger_length_axis = 'z'
    node.gripper_geometry = tabletop_gripper()
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
    evaluated_geometries = []

    def record_request_geometry(
        _raw_candidate,
        _camera_candidate,
        _grasp_pose,
        _sequence,
        geometry,
        contact_execution_phase=True,
    ):
        assert contact_execution_phase is False
        evaluated_geometries.append(geometry)
        return latest_gate

    node._evaluate_candidate_geometry = record_request_geometry
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
    source_payload = remote_node.LocalCandidatePayload(
        raw_candidate_index=0,
        variant_index=0,
        raw_candidate=camera_candidate,
        camera_candidate=camera_candidate,
        grasp_pose=None,
        geometry_gate=latest_gate,
        soft_features=features,
        score_components={},
    )
    initial_gate = CandidateGateResult(
        **{
            **latest_gate.__dict__,
            'required_open_width_m': 0.04,
        }
    )
    transform = np.eye(4)
    transform[:3, 3] = (0.1, 0.0, 0.2)
    normalized = remote_node.NormalizedPlanningCandidate(
        candidate_source='graspnet',
        source_index=0,
        variant_index=0,
        source_lineage=('graspnet',),
        contact_center_base=(0.1, 0.0, 0.2),
        T_base_tool0=transform,
        insertion_axis_base=(0.0, 0.0, -1.0),
        jaw_axis_base=(0.0, 1.0, 0.0),
        required_open_width_m=0.04,
        model_width_m=0.038,
        model_score=0.8,
        source_local_score=-0.8,
        common_physical_cost=0.0,
        geometry_gate=initial_gate,
        grasp_sequence=None,
        payload=source_payload,
        audit={},
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
        payload=normalized,
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
        geometry=types.SimpleNamespace(
            center_base=(0.1, 0.0, 0.2),
            object_points_base=(
                tabletop_box_cloud((0.03, 0.02, 0.01))
                + np.asarray([0.1, 0.0, 0.195])
            ),
            support_normal_base=(0.0, 0.0, 1.0),
        ),
        pose_estimator=types.SimpleNamespace(transform_sha256='current-tf'),
        snapshot=types.SimpleNamespace(
            object_msg=types.SimpleNamespace(detected=True, label='carton')
        ),
        near_field=False,
        model_choice='carton_segment',
    )

    scored = node._recheck_and_score_stable(prepared, (stable,))

    assert len(scored) == 2
    assert evaluated_geometries == [prepared.geometry, prepared.geometry]
    assert all(
        item.stable_candidate.required_open_width_m == pytest.approx(0.045)
        for item in scored
    )
    assert all(
        item.latest_safety.required_open_width_m == pytest.approx(0.045)
        for item in scored
    )
    assert all(item.latest_safety.depth_required is True for item in scored)
    assert all(item.latest_safety.depth_valid is True for item in scored)
    assert all(item.latest_safety.visibility_required is True for item in scored)
    assert all(item.latest_safety.visibility_valid is True for item in scored)
    assert all(item.soft_features.model_score == 0.0 for item in scored)
    assert all(item.soft_features.contact_balance > 0.0 for item in scored)
    assert all(
        item.soft_features.position_dispersion_m == pytest.approx(0.002)
        for item in scored
    )
    assert all(
        item.soft_features.orientation_dispersion_rad == pytest.approx(0.03)
        for item in scored
    )
    observation_sequences = {
        id(value['observation_sequence'])
        for value in node._stable_variant_runtime.values()
    }
    contact_sequences = {
        id(value['sequence'])
        for value in node._stable_variant_runtime.values()
    }
    assert {id(value) for value in visibility_sequences} == observation_sequences
    assert observation_sequences.isdisjoint(contact_sequences)


def test_tabletop_stable_recheck_uses_fused_pose_without_depth(monkeypatch):
    node = remote_node.RemoteGrasp6DNode.__new__(remote_node.RemoteGrasp6DNode)
    configure_identity_handeye(node)
    node.target_instance_epoch = 4
    node.grasp_config = {
        'tool_approach_axis': 'z',
        'pregrasp_distance_m': 0.08,
        'final_approach_offset_m': 0.015,
        'lift_height_m': 0.05,
    }
    node.gripper_geometry = tabletop_gripper()
    node.gripper_tool_jaw_axis = 'y'
    node.gripper_tool_finger_length_axis = 'z'
    node.gripper_physical_open_width_m = 0.05
    node.target_absolute_sanity_distance_m = 0.15
    node.soft_score_weights = SoftScoreWeights()
    node.camera_visibility_gate_enabled = False
    node.camera_visibility_diagnostic_enabled = False
    geometry = tabletop_geometry()
    gate = CandidateGateResult(
        ok=True,
        failure_code='',
        failure_reason='',
        required_open_width_m=0.039,
        center_distance_m=0.0,
        support_clearance_m=0.003,
        jaw_alignment=1.0,
        motion_cost=0.0,
        geometry_cost=0.0,
        failed_gate='',
        passed_gate_count=6,
    )
    monkeypatch.setattr(
        remote_node,
        'evaluate_explicit_candidate',
        lambda **_kwargs: gate,
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
        geometry=geometry,
        pose_estimator=types.SimpleNamespace(transform_sha256='current-tf'),
        snapshot=types.SimpleNamespace(
            object_msg=types.SimpleNamespace(detected=True, label='carton')
        ),
        model_choice='carton_segment',
    )
    normalized = node._normalize_tabletop_candidate(
        prepared,
        tabletop_candidates_for(geometry)[0],
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
        center_base_xyz=normalized.contact_center_base,
        tool0_position_xyz=normalized.T_base_tool0[:3, 3],
        quaternion_xyzw=remote_node.quaternion_from_matrix(
            normalized.T_base_tool0
        ),
        approach_base_xyz=normalized.insertion_axis_base,
        required_open_width_m=normalized.required_open_width_m,
        model_width_m=None,
        model_score=None,
        geometry_margin_m=0.003,
        pre_moveit_score=normalized.common_physical_cost,
        position_dispersion_m=0.002,
        orientation_dispersion_rad=0.03,
        payload=normalized,
        candidate_source='tabletop_geometry',
        source_lineage=('tabletop_geometry',),
    )
    monkeypatch.setattr(
        remote_node,
        'validate_graspnet_depth_m',
        lambda *_args, **_kwargs: pytest.fail(
            'tabletop recheck must never request GraspNet depth'
        ),
    )

    scored = node._recheck_and_score_stable(prepared, (stable,))

    assert len(scored) == 2
    # Both exact parallel-jaw wrist symmetries must retain a safety record that
    # binds to the materialized pose. The tabletop reprojection must not make
    # variant 1 apply tool Rz(pi) a second time before MoveIt.
    selection = bounded_moveit_select(
        scored,
        lambda _candidate: MoveItResult(
            reachable=True,
            joint_path_cost=1.0,
            joint_max_delta_rad=1.0,
            reason='reachable',
            collision_free=True,
            within_joint_limits=True,
            ik_valid=True,
            planning_success=True,
        ),
        top_n=3,
        max_joint_delta_rad=0.0,
    )
    assert len(selection.checked) == 2
    assert len(selection.reachable) == 2
    assert 'SAFETY_BINDING_MISMATCH' not in selection.funnel.rejection_counts
    assert all(item.latest_safety.depth_required is False for item in scored)
    assert all(item.latest_safety.depth_valid is None for item in scored)
    assert all(item.stable_candidate.model_width_m is None for item in scored)
    fusion_audits = [
        value['fusion_vs_latest_observation']
        for value in node._stable_variant_runtime.values()
    ]
    assert len(fusion_audits) == 2
    assert all(item['available'] is True for item in fusion_audits)
    assert all(
        item['source_is_current_evaluation'] is False
        for item in fusion_audits
    )
    assert all(
        math.isfinite(item['position_delta_m'])
        and item['position_delta_m'] >= 0.0
        for item in fusion_audits
    )
    assert all(
        math.isfinite(item['rotation_delta_deg'])
        and item['rotation_delta_deg'] >= 0.0
        for item in fusion_audits
    )


def test_fusion_vs_latest_observation_audit_is_diagnostic_only():
    node = remote_node.RemoteGrasp6DNode.__new__(remote_node.RemoteGrasp6DNode)
    node.grasp_config = {
        'tool_approach_axis': 'z',
        'pregrasp_distance_m': 0.08,
        'final_approach_offset_m': 0.015,
        'lift_height_m': 0.05,
    }
    node.gripper_geometry = tabletop_gripper()
    node.gripper_tool_jaw_axis = 'y'
    node.gripper_tool_finger_length_axis = 'z'
    node.gripper_physical_open_width_m = 0.05
    node.target_absolute_sanity_distance_m = 0.15
    node.soft_score_weights = SoftScoreWeights()
    node.camera_visibility_gate_enabled = False
    node.camera_visibility_diagnostic_enabled = False
    geometry = tabletop_geometry()
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
        geometry=geometry,
        pose_estimator=types.SimpleNamespace(transform_sha256='current-tf'),
        snapshot=types.SimpleNamespace(
            object_msg=types.SimpleNamespace(detected=True, label='carton')
        ),
        near_field=True,
        model_choice='carton_segment',
    )
    normalized = node._normalize_tabletop_candidate(
        prepared,
        tabletop_candidates_for(geometry)[0],
    )
    latest_transform = np.asarray(normalized.T_base_tool0, dtype=float)
    latest_quaternion = remote_node.quaternion_from_matrix(latest_transform)
    offset = np.asarray([0.001, -0.002, 0.003], dtype=float)
    fused_pose = remote_node.make_pose_stamped(
        'base_link',
        latest_transform[:3, 3] + offset,
        latest_quaternion,
        stamp=remote_node.rospy.Time.from_sec(20.0),
    )

    audit = node._fusion_vs_latest_observation_audit(
        fused_grasp_pose=fused_pose,
        latest_observation=normalized,
        evaluation_variant_index=0,
        source_request_id=4,
        evaluation_request_id=4,
    )

    assert audit['available'] is True
    assert audit['source_is_current_evaluation'] is True
    assert audit['fused_minus_latest_position_m'] == pytest.approx(offset)
    assert audit['position_delta_m'] == pytest.approx(np.linalg.norm(offset))
    assert audit['rotation_delta_deg'] == pytest.approx(0.0)


def test_common_soft_features_rewards_real_opening_margin_not_support_floor():
    node = remote_node.RemoteGrasp6DNode.__new__(remote_node.RemoteGrasp6DNode)
    node.gripper_physical_open_width_m = 0.050
    node.camera_visibility_gate_enabled = False
    node.camera_visibility_diagnostic_enabled = False
    geometry = types.SimpleNamespace(
        object_points_base=np.asarray(
            [
                [-0.010, 0.000, 0.000],
                [0.010, 0.000, 0.000],
                [0.000, -0.010, 0.000],
                [0.000, 0.010, 0.000],
            ],
            dtype=float,
        ),
        center_base=np.asarray([0.0, 0.0, 0.0], dtype=float),
        support_normal_base=np.asarray([0.0, 0.0, 1.0], dtype=float),
    )
    prepared = types.SimpleNamespace(geometry=geometry)

    def features(required_width):
        return node._common_soft_features(
            prepared=prepared,
            contact_center_base=np.asarray([0.0, 0.0, 0.0], dtype=float),
            grasp_pose=None,
            approach_axis_base=np.asarray([0.0, 0.0, -1.0], dtype=float),
            jaw_axis_base=np.asarray([1.0, 0.0, 0.0], dtype=float),
            gate=types.SimpleNamespace(
                support_clearance_m=0.003,
                required_open_width_m=required_width,
            ),
        )

    narrow = features(0.036)
    near_limit = features(0.046)

    assert narrow.geometry_margin_m == pytest.approx(0.014)
    assert near_limit.geometry_margin_m == pytest.approx(0.004)
    assert remote_node.source_neutral_candidate_cost(
        narrow, SoftScoreWeights()
    ).total < remote_node.source_neutral_candidate_cost(
        near_limit, SoftScoreWeights()
    ).total


def test_tabletop_stable_recheck_refreshes_width_from_current_cloud():
    node = remote_node.RemoteGrasp6DNode.__new__(remote_node.RemoteGrasp6DNode)
    configure_identity_handeye(node)
    node.target_instance_epoch = 4
    node.grasp_config = {
        'tool_approach_axis': 'z',
        'pregrasp_distance_m': 0.08,
        'final_approach_offset_m': 0.015,
        'lift_height_m': 0.05,
    }
    node.gripper_geometry = tabletop_gripper()
    node.gripper_tool_jaw_axis = 'y'
    node.gripper_tool_finger_length_axis = 'z'
    node.gripper_physical_open_width_m = 0.05
    node.target_absolute_sanity_distance_m = 0.15
    node.soft_score_weights = SoftScoreWeights()
    node.camera_visibility_gate_enabled = False
    node.camera_visibility_diagnostic_enabled = False
    geometry = tabletop_geometry()
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
        geometry=geometry,
        pose_estimator=types.SimpleNamespace(transform_sha256='current-tf'),
        snapshot=types.SimpleNamespace(
            object_msg=types.SimpleNamespace(detected=True, label='carton')
        ),
        model_choice='carton_segment',
    )
    normalized = node._normalize_tabletop_candidate(
        prepared,
        tabletop_candidates_for(geometry)[0],
    )
    stale_width = normalized.required_open_width_m + 0.001
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
        center_base_xyz=normalized.contact_center_base,
        tool0_position_xyz=normalized.T_base_tool0[:3, 3],
        quaternion_xyzw=remote_node.quaternion_from_matrix(
            normalized.T_base_tool0
        ),
        approach_base_xyz=normalized.insertion_axis_base,
        required_open_width_m=stale_width,
        model_width_m=None,
        model_score=None,
        geometry_margin_m=0.003,
        pre_moveit_score=normalized.common_physical_cost,
        position_dispersion_m=0.002,
        orientation_dispersion_rad=0.03,
        payload=normalized,
        candidate_source='tabletop_geometry',
        source_lineage=('tabletop_geometry',),
    )

    scored = node._recheck_and_score_stable(prepared, (stable,))

    assert len(scored) == 2
    assert all(item.latest_safety.collision_free is True for item in scored)
    assert all(
        item.stable_candidate.required_open_width_m == pytest.approx(stale_width)
        for item in scored
    )
    assert all(
        runtime['geometry_gate'].required_open_width_m == pytest.approx(stale_width)
        for runtime in node._stable_variant_runtime.values()
    )


def test_tabletop_stable_recheck_reprojects_tracker_drift_to_support_clearance():
    node = remote_node.RemoteGrasp6DNode.__new__(remote_node.RemoteGrasp6DNode)
    configure_identity_handeye(node)
    node.target_instance_epoch = 4
    node.grasp_config = {
        'tool_approach_axis': 'z',
        'pregrasp_distance_m': 0.08,
        'final_approach_offset_m': 0.015,
        'lift_height_m': 0.05,
    }
    node.gripper_geometry = tabletop_gripper()
    node.gripper_tool_jaw_axis = 'y'
    node.gripper_tool_finger_length_axis = 'z'
    node.gripper_physical_open_width_m = 0.05
    node.target_absolute_sanity_distance_m = 0.15
    node.soft_score_weights = SoftScoreWeights()
    node.camera_visibility_gate_enabled = False
    node.camera_visibility_diagnostic_enabled = False
    geometry = tabletop_geometry()
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
        geometry=geometry,
        pose_estimator=types.SimpleNamespace(transform_sha256='current-tf'),
        snapshot=types.SimpleNamespace(
            object_msg=types.SimpleNamespace(detected=True, label='carton')
        ),
        model_choice='carton_segment',
    )
    normalized = node._normalize_tabletop_candidate(
        prepared,
        tabletop_candidates_for(geometry)[0],
    )
    drifted_tool0 = np.array(
        normalized.T_base_tool0[:3, 3],
        dtype=float,
        copy=True,
    )
    drifted_tool0[2] -= 0.00025
    fused_orientation_with_axis_drift = remote_node.quaternion_multiply(
        remote_node.quaternion_from_matrix(normalized.T_base_tool0),
        remote_node.quaternion_from_euler(0.010, 0.0, 0.0),
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
        center_base_xyz=normalized.contact_center_base,
        tool0_position_xyz=drifted_tool0,
        quaternion_xyzw=fused_orientation_with_axis_drift,
        approach_base_xyz=normalized.insertion_axis_base,
        required_open_width_m=normalized.required_open_width_m,
        model_width_m=None,
        model_score=None,
        geometry_margin_m=0.003,
        pre_moveit_score=normalized.common_physical_cost,
        position_dispersion_m=0.002,
        orientation_dispersion_rad=0.03,
        payload=normalized,
        candidate_source='tabletop_geometry',
        source_lineage=('tabletop_geometry',),
    )

    scored = node._recheck_and_score_stable(prepared, (stable,))

    assert len(scored) == 2
    assert all(item.latest_safety.collision_free is True for item in scored)
    assert all(
        item.stable_candidate.tool0_position_xyz[2] > drifted_tool0[2]
        for item in scored
    )
    assert all(
        runtime['geometry_gate'].support_clearance_m + 1e-9
        >= node.gripper_geometry.support_clearance_m
        for runtime in node._stable_variant_runtime.values()
    )
    jaw_local, _jaw_index = remote_node.parse_tool_axis('y')
    assert all(
        abs(float(np.dot(
            remote_node.pose_matrix(runtime['grasp_pose'])[:3, :3]
            @ jaw_local,
            geometry.support_normal_base,
        )))
        <= 1e-10
        for runtime in node._stable_variant_runtime.values()
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
    node._execution_plan_validity_now_sec = lambda: max(
        20.0,
        float(node._stream_source_clock()),
    )
    node.plan_pub = RecordingPublisher()
    node.rich_plan_pub = RecordingPublisher()
    node.preview_plan_pub = RecordingPublisher()
    node.preview_rich_plan_pub = RecordingPublisher()
    node.latest_rich_plan = None
    node.latest_plan = None
    node.latest_preview_rich_plan = None
    node._latest_preview_proposal = None
    node._latest_preview_gate_audit_report = None
    node._execution_promotion_audit_ready = lambda _plan: True
    return node


def seed_execution(node, plan, signature='candidate-A', score=1.0):
    legacy = remote_node.rich_plan_to_legacy(plan)
    node.latest_rich_plan = remote_node.deepcopy(plan)
    node.latest_plan = remote_node.deepcopy(legacy)
    node.rich_plan_pub.publish(remote_node.deepcopy(plan))
    node.plan_pub.publish(remote_node.deepcopy(legacy))
    node.execution_plan_controller.commit_execution(
        plan.plan_id,
        signature,
        score=score,
        now_sec=node._promotion_now_sec(),
    )


def promotion_transaction_node(tmp_path):
    node = streaming_node(clock=MutableClock(10.0), start_worker=False)
    node._geometry_state_lock = threading.RLock()
    node._geometry_invalidation_generation = 0
    node._last_geometry_invalidation_code = ''
    node.robot_execution_active = False
    node._execution_plan_validity_now_sec = lambda: 20.0
    node.execution_plan_controller = ExecutionPlanController()
    node.plan_pub = RecordingPublisher()
    node.rich_plan_pub = RecordingPublisher()
    node.latest_rich_plan = None
    node.latest_plan = None
    node.gate_audit_enabled = True
    node.gate_audit_output_path = str(tmp_path / 'planning.json')
    node.mujoco_audit_output_path = str(tmp_path / 'mujoco.json')
    node.gate_audit_pub = RecordingPublisher()
    node.start_streaming()
    node.submit_stream_snapshot(snapshot(9.8))
    active_ticket = node._stream_worker_ticket
    ticket = InferenceTicket(
        request_id=4,
        generation=active_ticket.generation,
        snapshot_stamp_sec=9.8,
        target_epoch=active_ticket.target_epoch,
        payload=active_ticket.payload,
        submitted_monotonic_sec=active_ticket.submitted_monotonic_sec,
    )
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
    stable = StableCandidate(
        track_id=7,
        hit_count=3,
        window_count=5,
        hit_request_ids=(1, 2, 3),
        request_id=3,
        snapshot_stamp_sec=9.7,
        target_epoch=ticket.target_epoch,
        target_label='carton',
        model_choice='carton_segment',
        center_base_xyz=(0.1, 0.0, 0.3),
        tool0_position_xyz=(0.1, 0.0, 0.3),
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
    safety = SafetyGateInput(
        depth_valid=True,
        transform_valid=True,
        target_present=True,
        same_target_instance=True,
        target_absolute_distance_m=0.0,
        target_absolute_limit_m=0.15,
        required_open_width_m=0.04,
        physical_open_width_m=0.05,
        geometry_valid=True,
        collision_free=True,
        request_id=ticket.request_id,
        snapshot_stamp_sec=ticket.snapshot_stamp_sec,
        target_epoch=ticket.target_epoch,
        target_label='carton',
        model_choice='carton_segment',
        track_id=7,
        variant_index=1,
        center_base_xyz=(0.1, 0.0, 0.3),
        tool0_position_xyz=(0.1, 0.0, 0.3),
        quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
        approach_base_xyz=(0.0, 0.0, -1.0),
        snapshot_context_revision='ctx-4',
    )
    moveit = MoveItResult(
        reachable=True,
        joint_path_cost=0.2,
        joint_max_delta_rad=0.1,
        reason='strict plan success',
        collision_free=True,
        within_joint_limits=True,
        ik_valid=True,
        planning_success=True,
    )
    evaluated = ScoredStableCandidate(
        stable_candidate=stable,
        variant_index=1,
        latest_safety=safety,
        soft_features=features,
        score_weights=SoftScoreWeights(),
        evaluation_request_id=ticket.request_id,
        evaluation_snapshot_stamp_sec=ticket.snapshot_stamp_sec,
        evaluation_context_revision='ctx-4',
        moveit_result=moveit,
        final_score=1.0,
    )
    node._stable_variant_runtime = {
        (7, 1): {
            'prepared': types.SimpleNamespace(ticket=ticket),
            'scored_candidate': evaluated,
            'soft_evidence': {
                'source': 'latest-rgbd',
                'observation_incidence_angle_deg': 18.0,
                'observation_projected_side_evidence_m': 0.006,
                'observation_side_uncertainty_m': 0.003,
                'observation_side_evidence_deficit_m': 0.0,
            },
            'observation_envelope': {
                'ok': True,
                'failure_code': '',
                'failure_reason': '',
                'minimum_support_clearance_m': 0.015,
            },
            'observation_view': {
                'construction': (
                    'camera_optical_axis_fixed_camera_target_distance'
                ),
                'nominal_camera_target_distance_m': 0.200,
                'min_camera_target_distance_m': 0.190,
                'max_camera_target_distance_m': 0.210,
                'actual_camera_target_distance_m': 0.200,
                'center_residual_m': 0.0,
            },
        }
    }
    prepared = types.SimpleNamespace(ticket=ticket)
    selection = types.SimpleNamespace(
        checked=(evaluated,),
        reachable=(evaluated,),
        selected=evaluated,
    )
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


def test_preview_promotion_helper_is_provisional_and_cannot_publish():
    node = promotion_node()
    preview = promotion_plan('plan-A')

    first = node._maybe_promote_preview(
        preview, signature='candidate-A', score=1.0
    )
    second = node._maybe_promote_preview(
        preview, signature='candidate-A', score=1.0
    )

    assert first.promote is True
    assert second.promote is True
    assert node.rich_plan_pub.messages == []
    assert node.plan_pub.messages == []
    assert node.latest_rich_plan is None
    assert node.execution_plan_controller.execution_plan_id is None


def test_active_execution_keeps_authority_while_new_preview_is_visible():
    node = promotion_node()
    first = promotion_plan('plan-A')
    challenger = promotion_plan('preview-B', x=0.2)
    seed_execution(node, first)
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


def test_provisional_helper_never_calls_execution_publishers():
    node = promotion_node()
    node.plan_pub = FailingPublisher()

    decision = node._maybe_promote_preview(
        promotion_plan('plan-A'), signature='candidate-A', score=1.0
    )

    assert decision.promote is True
    assert decision.code == 'PROMOTE_INITIAL'
    assert node.execution_plan_controller.execution_plan_id is None
    assert node.rich_plan_pub.messages == []
    assert node.latest_rich_plan is None


def test_provisional_helper_cannot_consume_or_bypass_execution_audit():
    node = promotion_node()
    node._execution_promotion_audit_ready = lambda _plan: False

    decision = node._maybe_promote_preview(
        promotion_plan('plan-A'), signature='candidate-A', score=1.0
    )

    assert decision.promote is True
    assert decision.code == 'PROMOTE_INITIAL'
    assert node.rich_plan_pub.messages == []
    assert node.plan_pub.messages == []
    assert node.execution_plan_controller.execution_plan_id is None


def test_final_audit_is_bound_before_execution_authority_publish(tmp_path):
    setup = promotion_transaction_node(tmp_path)
    node, prepared, selection, proposal, local_funnel, moveit_funnel = setup
    audit_path = pathlib.Path(
        '{}.execution'.format(node.gate_audit_output_path)
    )

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
        preview_report = json.loads(
            pathlib.Path(node.gate_audit_output_path).read_text()
        )
        assert preview_report['promotion']['code'] == 'PROMOTED'
        assert preview_report['promotion']['promote'] is True
        assert preview_report['pipeline_funnel']['stage_counts']['promoted'][
            'passed'
        ] == 1
        assert preview_report['selected']['observation_envelope']['ok'] is True
        assert preview_report['selected']['observation_view'][
            'actual_camera_target_distance_m'
        ] == pytest.approx(0.200)
        assert preview_report['selected']['observation_side_evidence'][
            'observation_side_evidence_deficit_m'
        ] == pytest.approx(0.0)
        execution_report = json.loads(audit_path.read_text())
        assert execution_report['selected']['observation_envelope']['ok'] is True
        assert execution_report['selected']['observation_view'][
            'actual_camera_target_distance_m'
        ] == pytest.approx(0.200)
        assert execution_report['selected']['observation_side_evidence'][
            'observation_projected_side_evidence_m'
        ] == pytest.approx(0.006)
        assert node._latest_promotion_decision.code == 'PROMOTED'
        assert [item.plan_id for item in node.rich_plan_pub.messages] == [
            'plan-A'
        ]
    finally:
        node.shutdown_streaming_worker()


def test_execution_audit_excludes_unreachable_checked_without_final_score(
    tmp_path,
):
    setup = promotion_transaction_node(tmp_path)
    node, prepared, selection, proposal, local_funnel, moveit_funnel = setup
    selected = selection.selected
    unreachable_moveit = MoveItResult(
        reachable=False,
        joint_path_cost=0.0,
        joint_max_delta_rad=0.0,
        reason='strict plan failed',
        collision_free=True,
        within_joint_limits=True,
        ik_valid=True,
        planning_success=False,
        failure_code='MOVEIT_UNREACHABLE',
    )
    rejected_stable = remote_node.replace(
        selected.stable_candidate,
        track_id=8,
    )
    rejected = remote_node.replace(
        selected,
        stable_candidate=rejected_stable,
        variant_index=0,
        moveit_result=unreachable_moveit,
        final_score=None,
    )
    selection.checked = (rejected, selected)
    selection.reachable = (selected,)
    selection.selected = selected
    node._stable_variant_runtime[(8, 0)] = {
        'prepared': types.SimpleNamespace(ticket=prepared.ticket),
        'scored_candidate': rejected,
        'soft_evidence': {'source': 'latest-rgbd'},
    }
    try:
        _funnel, decision = node._finalize_promotion_transaction(
            prepared,
            selection,
            proposal,
            local_funnel,
            moveit_funnel,
            2,
            'PREVIEW_READY',
            {'summary': {}, 'rows': []},
        )

        report = json.loads(
            pathlib.Path(
                '{}.execution'.format(node.gate_audit_output_path)
            ).read_text()
        )
        assert decision.promote is True
        assert report['outcome']['valid_plan'] is True
        assert len(report['stable_evaluations']) == 1
        assert report['stable_evaluations'][0]['selected'] is True
        assert report['stable_evaluations'][0]['moveit']['reachable'] is True
        assert report['stable_evaluations'][0]['final_score'] == 1.0
    finally:
        node.shutdown_streaming_worker()


def test_streaming_audit_records_exact_moveit_sequence_and_pre_call_joints(
    tmp_path,
):
    setup = promotion_transaction_node(tmp_path)
    node, prepared, selection, _proposal, local_funnel, _moveit_funnel = setup
    prepared.near_field = True
    replay_points = np.asarray(
        [
            [0.10, -0.20, 0.02],
            [0.12, -0.20, 0.02],
            [0.10, -0.16, 0.04],
        ],
        dtype=float,
    )
    prepared.geometry = types.SimpleNamespace(
        ok=True,
        failure_code='',
        failure_reason='',
        center_base=np.asarray([0.11, -0.18, 0.03], dtype=float),
        axes_base=np.eye(3, dtype=float),
        size_xyz_m=np.asarray([0.02, 0.04, 0.02], dtype=float),
        support_normal_base=np.asarray([0.0, 0.0, 1.0], dtype=float),
        support_offset_m=0.0,
        support_inlier_ratio=0.75,
        object_points_base=replay_points,
        source_mode='instance_mask',
    )
    grasp_pose = remote_node.make_pose_stamped(
        'base_link',
        (0.1, -0.2, 0.1),
        (0.0, 0.0, 0.0, 1.0),
        stamp=9.8,
    )
    sequence = remote_node.make_grasp_sequence_from_grasp_pose(
        grasp_pose,
        pregrasp_distance_m=0.08,
        approach_offset_m=0.03,
        lift_height_m=0.05,
        approach_direction_base=(0.0, 0.0, -1.0),
        pregrasp_direction_base=(0.0, 0.0, -1.0),
        lift_direction_base=(0.0, 0.0, 1.0),
    )
    profile = types.SimpleNamespace(
        tilt_deg=0.0,
        pregrasp_distance_m=0.08,
        approach_offset_m=0.03,
        lift_height_m=0.05,
        lateral_sweep_m=0.0,
        object_height_m=0.02,
        depth_uncertainty_m=0.004,
        execution_position_error_m=0.0,
        contact_overlap_requirement_m=0.002,
    )
    joint_state = types.SimpleNamespace(
        header=types.SimpleNamespace(frame_id='base_link', stamp=9.7),
        name=['Joint1', 'Joint2'],
        position=[0.1, -0.2],
    )
    runtime = node._stable_variant_runtime[(7, 1)]
    runtime.update({
        'sequence': sequence,
        'adaptive_stage_profile': profile,
        'moveit_input_joint_state': node._joint_state_audit(joint_state),
    })
    try:
        report, _summary, _selected = (
            node._build_streaming_gate_audit_report(
                prepared,
                selection,
                local_funnel,
                'PREVIEW_READY',
                base_report={'summary': {}, 'rows': []},
            )
        )
        evaluation = report['stable_evaluations'][0]
        sequence_audit = evaluation['execution_sequence']
        assert sequence_audit['available'] is True
        assert sequence_audit['kind'] == 'near_field_contact'
        assert [
            stage['stage'] for stage in sequence_audit['stages']
        ] == ['pregrasp', 'approach', 'grasp', 'lift']
        assert [
            stage['position_m'][2] for stage in sequence_audit['stages']
        ] == pytest.approx([0.18, 0.13, 0.10, 0.15])
        assert all(
            stage['quaternion_xyzw']
            == pytest.approx([0.0, 0.0, 0.0, 1.0])
            for stage in sequence_audit['stages']
        )
        assert evaluation['adaptive_stage_profile'] == {
            'tilt_deg': 0.0,
            'pregrasp_distance_m': 0.08,
            'approach_offset_m': 0.03,
            'lift_height_m': 0.05,
            'lateral_sweep_m': 0.0,
            'object_height_m': 0.02,
            'depth_uncertainty_m': 0.004,
            'execution_position_error_m': 0.0,
            'contact_overlap_requirement_m': 0.002,
            'pregrasp_direction_mode': 'support_normal',
            'lateral_sweep_reference': 'final_approach',
        }
        joint_audit = evaluation['moveit_input_joint_state']
        assert joint_audit['available'] is True
        assert joint_audit['service_start_state_exact'] is False
        assert joint_audit['stamp_ns'] == 9_700_000_000
        assert joint_audit['name'] == ['Joint1', 'Joint2']
        assert joint_audit['position_rad'] == pytest.approx([0.1, -0.2])
        replay = report['replay_geometry']
        assert replay['available'] is True
        assert replay['frame_id'] == 'base_link'
        assert replay['source_mode'] == 'instance_mask'
        assert replay['obb_center_base_m'] == pytest.approx(
            [0.11, -0.18, 0.03]
        )
        assert np.asarray(replay['R_base_obb']) == pytest.approx(np.eye(3))
        assert replay['obb_size_xyz_m'] == pytest.approx([0.02, 0.04, 0.02])
        assert replay['support_normal_base'] == pytest.approx([0.0, 0.0, 1.0])
        assert replay['support_offset_m'] == pytest.approx(0.0)
        assert replay['support_inlier_ratio'] == pytest.approx(0.75)
        assert replay['object_points_count'] == 3
        assert np.asarray(replay['object_points_base_m']) == pytest.approx(
            replay_points
        )
        assert replay['object_points_sha256'] == remote_node.array_sha256(
            replay_points
        )
    finally:
        node.shutdown_streaming_worker()


def test_promotion_source_stable_counts_use_full_tracker_pool(tmp_path):
    setup = promotion_transaction_node(tmp_path)
    node, prepared, selection, proposal, local_funnel, moveit_funnel = setup
    selected = selection.selected
    geometry_stable = remote_node.replace(
        selected.stable_candidate,
        track_id=8,
        candidate_source='tabletop_geometry',
        source_lineage=('tabletop_geometry',),
    )
    second_graspnet_stable = remote_node.replace(
        selected.stable_candidate,
        track_id=9,
    )
    full_stable_pool = (
        selected.stable_candidate,
        geometry_stable,
        second_graspnet_stable,
    )
    try:
        funnel, _decision = node._finalize_promotion_transaction(
            prepared,
            selection,
            proposal,
            local_funnel,
            moveit_funnel,
            len(full_stable_pool),
            'PREVIEW_READY',
            {'summary': {}, 'rows': []},
            stable_candidates=full_stable_pool,
        )

        counts = funnel['source_counts']
        assert funnel['stage_counts']['stable']['passed'] == 3
        assert counts['graspnet']['stable'] == 2
        assert counts['tabletop_geometry']['stable'] == 1
        assert sum(item['stable'] for item in counts.values()) == 3
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
            pathlib.Path(
                '{}.execution'.format(node.gate_audit_output_path)
            ).read_text()
        )
        assert decision.code == 'PLAN_PUBLICATION_FAILED'
        assert report['outcome']['valid_plan'] is False
        assert report['outcome']['code'] == 'PLAN_PUBLICATION_FAILED'
        assert report['promotion']['code'] == 'PLAN_PUBLICATION_FAILED'
        assert funnel['stage_counts']['promoted']['passed'] == 0
        assert node.execution_plan_controller.execution_plan_id is None
        preview_report = json.loads(
            pathlib.Path(node.gate_audit_output_path).read_text()
        )
        assert preview_report['promotion']['promote'] is False
        assert preview_report['promotion']['code'] == (
            'PLAN_PUBLICATION_FAILED'
        )
        assert preview_report['pipeline_funnel']['stage_counts']['promoted'][
            'passed'
        ] == 0
        assert node._request_promotion_count(prepared.ticket) == 0
        assert node._latest_promotion_decision.code == (
            'PLAN_PUBLICATION_FAILED'
        )
        assert len(node.rich_plan_pub.messages) == 1
        assert node.rich_plan_pub.messages[-1].valid is False
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
            pathlib.Path(
                '{}.execution'.format(node.gate_audit_output_path)
            ).read_text()
        )
        assert decision.code == 'PLAN_STALE'
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
    correction_fsync_entered = threading.Event()
    release_correction_fsync = threading.Event()
    original_fsync = remote_node.os.fsync
    fsync_calls = {'count': 0}

    def block_correction_fsync(file_descriptor):
        fsync_calls['count'] += 1
        if fsync_calls['count'] == 3:
            correction_fsync_entered.set()
            if not release_correction_fsync.wait(2.0):
                raise RuntimeError('test did not release correction fsync')
        return original_fsync(file_descriptor)

    monkeypatch.setattr(remote_node.os, 'fsync', block_correction_fsync)
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
    assert correction_fsync_entered.wait(1.0)
    assert node.stop_streaming() is True
    release_correction_fsync.set()
    transaction.join(2.0)
    try:
        report = json.loads(
            pathlib.Path(
                '{}.execution'.format(node.gate_audit_output_path)
            ).read_text()
        )
        assert transaction_errors == []
        assert report['outcome']['valid_plan'] is False
        assert report['outcome']['code'] == 'PLAN_PUBLICATION_FAILED'
        assert node.execution_plan_controller.execution_plan_id is None
        assert len(node.rich_plan_pub.messages) == 1
        assert node.rich_plan_pub.messages[-1].valid is False
    finally:
        node.shutdown_streaming_worker()


def test_gate_audit_publisher_failure_compensates_execution_audit(tmp_path):
    setup = promotion_transaction_node(tmp_path)
    node, prepared, selection, proposal, local_funnel, moveit_funnel = setup
    node.gate_audit_pub = FailingPublisher()
    execution_path = pathlib.Path(
        '{}.execution'.format(node.gate_audit_output_path)
    )
    try:
        with pytest.raises(remote_node.CandidateContractError):
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

        report = json.loads(execution_path.read_text())
        assert report['outcome']['valid_plan'] is False
        assert report['outcome']['code'] == remote_node.PLANNING_AUDIT_FAILED
        assert node.plan_pub.messages == []
        assert node.rich_plan_pub.messages == []
        assert node.execution_plan_controller.execution_plan_id is None
    finally:
        node.shutdown_streaming_worker()


def test_promotion_rejects_selected_lineage_without_strict_moveit(tmp_path):
    setup = promotion_transaction_node(tmp_path)
    node, prepared, selection, proposal, local_funnel, moveit_funnel = setup
    selected = selection.selected
    incomplete = remote_node.replace(
        selected,
        moveit_result=remote_node.replace(
            selected.moveit_result,
            planning_success=None,
        ),
    )
    selection.checked = (incomplete,)
    selection.reachable = (incomplete,)
    selection.selected = incomplete
    node._stable_variant_runtime[(7, 1)]['scored_candidate'] = incomplete
    try:
        with pytest.raises(remote_node.CandidateContractError) as exc_info:
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

        assert exc_info.value.code == remote_node.PLANNING_AUDIT_FAILED
        assert node.plan_pub.messages == []
        assert node.rich_plan_pub.messages == []
        assert node.execution_plan_controller.execution_plan_id is None
    finally:
        node.shutdown_streaming_worker()


def test_promotion_accepts_generic_strict_service_success(tmp_path):
    setup = promotion_transaction_node(tmp_path)
    node, prepared, selection, proposal, local_funnel, moveit_funnel = setup
    selected = selection.selected
    generic_moveit = remote_node.replace(
        selected.moveit_result,
        collision_free=None,
        within_joint_limits=None,
        ik_valid=None,
        planning_success=None,
        failure_code='',
        evidence_code='STRICT_SERVICE_SUCCESS',
    )
    generic = remote_node.replace(selected, moveit_result=generic_moveit)
    selection.checked = (generic,)
    selection.reachable = (generic,)
    selection.selected = generic
    node._stable_variant_runtime[(7, 1)]['scored_candidate'] = generic
    try:
        _funnel, decision = node._finalize_promotion_transaction(
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
        assert node.execution_plan_controller.execution_plan_id == 'plan-A'
    finally:
        node.shutdown_streaming_worker()


@pytest.mark.parametrize(
    'failure_code,evidence_code',
    [
        ('MOVEIT_UNREACHABLE', ''),
        ('', 'STRICT_SERVICE_SUCCESS'),
    ],
)
def test_promotion_rejects_contradictory_structured_moveit_success(
    tmp_path,
    failure_code,
    evidence_code,
):
    setup = promotion_transaction_node(tmp_path)
    node, prepared, selection, proposal, local_funnel, moveit_funnel = setup
    selected = selection.selected
    contradictory_moveit = remote_node.replace(
        selected.moveit_result,
        failure_code=failure_code,
        evidence_code=evidence_code,
    )
    contradictory = remote_node.replace(
        selected,
        moveit_result=contradictory_moveit,
    )
    selection.checked = (contradictory,)
    selection.reachable = (contradictory,)
    selection.selected = contradictory
    node._stable_variant_runtime[(7, 1)][
        'scored_candidate'
    ] = contradictory
    try:
        with pytest.raises(remote_node.CandidateContractError) as exc_info:
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

        assert exc_info.value.code == remote_node.PLANNING_AUDIT_FAILED
        assert node.rich_plan_pub.messages == []
        assert node.execution_plan_controller.execution_plan_id is None
    finally:
        node.shutdown_streaming_worker()


def test_promotion_uses_only_selected_lineage_when_duplicate_tracks_exist(
    tmp_path,
):
    setup = promotion_transaction_node(tmp_path)
    node, prepared, selection, proposal, local_funnel, moveit_funnel = setup
    selected = selection.selected
    duplicate_stable = remote_node.replace(
        selected.stable_candidate,
        track_id=8,
    )
    duplicate = remote_node.replace(
        selected,
        stable_candidate=duplicate_stable,
    )
    selection.checked = (selected, duplicate)
    selection.reachable = (selected, duplicate)
    node._stable_variant_runtime[(8, 1)] = {
        'prepared': types.SimpleNamespace(ticket=prepared.ticket),
        'scored_candidate': duplicate,
        'soft_evidence': {'source': 'latest-rgbd'},
    }
    try:
        _funnel, decision = node._finalize_promotion_transaction(
            prepared,
            selection,
            proposal,
            local_funnel,
            moveit_funnel,
            2,
            'PREVIEW_READY',
            {'summary': {}, 'rows': []},
        )

        report = json.loads(
            pathlib.Path(
                '{}.execution'.format(node.gate_audit_output_path)
            ).read_text()
        )
        assert decision.promote is True
        assert node.execution_plan_controller.execution_plan_id == 'plan-A'
        assert len(report['stable_evaluations']) == 1
        assert report['stable_evaluations'][0]['selected'] is True
        assert report['stable_evaluations'][0]['tracking']['track_id'] == 7
    finally:
        node.shutdown_streaming_worker()


@pytest.mark.parametrize(
    'missing_evidence',
    ['selected_lineage', 'tracking', 'lineage_binding', 'final_score'],
)
def test_promotion_rejects_incomplete_selected_execution_evidence(
    tmp_path,
    monkeypatch,
    missing_evidence,
):
    setup = promotion_transaction_node(tmp_path)
    node, prepared, selection, proposal, local_funnel, moveit_funnel = setup
    original = node._build_streaming_gate_audit_report

    def incomplete_report(*args, **kwargs):
        report, summary, selected = original(*args, **kwargs)
        if missing_evidence == 'selected_lineage':
            report['selected'] = None
        elif missing_evidence == 'tracking':
            report['stable_evaluations'][0]['tracking'].pop('hit_count')
            report['selected']['tracking'].pop('hit_count')
        elif missing_evidence == 'lineage_binding':
            report['stable_evaluations'][0]['lineage_binding'].pop(
                'evaluation_request_id'
            )
            report['selected']['lineage_binding'].pop(
                'evaluation_request_id'
            )
        else:
            report['stable_evaluations'][0]['final_score'] = None
            report['selected']['final_score'] = None
        report['lineage'] = remote_node.deepcopy(
            report['stable_evaluations']
        )
        return report, summary, selected

    monkeypatch.setattr(
        node,
        '_build_streaming_gate_audit_report',
        incomplete_report,
    )
    try:
        with pytest.raises(remote_node.CandidateContractError) as exc_info:
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

        assert exc_info.value.code == remote_node.PLANNING_AUDIT_FAILED
        assert node.rich_plan_pub.messages == []
        assert node.execution_plan_controller.execution_plan_id is None
    finally:
        node.shutdown_streaming_worker()


def test_blocked_gate_audit_publish_does_not_block_stop_or_authority(
    tmp_path,
):
    setup = promotion_transaction_node(tmp_path)
    node, prepared, selection, proposal, local_funnel, moveit_funnel = setup
    node.gate_audit_pub = BlockingPublisher(lambda _message: True)
    errors = []
    results = []

    def run_transaction():
        try:
            results.append(
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
            )
        except Exception as exc:
            errors.append(exc)

    transaction = threading.Thread(target=run_transaction)
    transaction.start()
    assert node.gate_audit_pub.entered.wait(1.0)
    try:
        assert node.stop_streaming() is True
    finally:
        node.gate_audit_pub.release.set()
        transaction.join(2.0)

    execution_report = json.loads(
        pathlib.Path(
            '{}.execution'.format(node.gate_audit_output_path)
        ).read_text()
    )
    assert errors == []
    assert results[0][1].promote is False
    assert results[0][1].code == 'GENERATION_STALE'
    assert execution_report['outcome']['valid_plan'] is False
    assert node.rich_plan_pub.messages == []
    assert node.execution_plan_controller.execution_plan_id is None
    node.shutdown_streaming_worker()


def test_blocked_gate_audit_publish_allows_hard_invalidation(tmp_path):
    setup = promotion_transaction_node(tmp_path)
    node, prepared, selection, proposal, local_funnel, moveit_funnel = setup
    node.gate_audit_pub = BlockingPublisher(lambda _message: True)
    errors = []

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
            errors.append(exc)

    transaction = threading.Thread(target=run_transaction)
    transaction.start()
    assert node.gate_audit_pub.entered.wait(1.0)
    invalidation_done = threading.Event()
    invalidation = threading.Thread(
        target=lambda: (
            node._invalidate_geometry('TARGET_LOST', 'target lost'),
            invalidation_done.set(),
        )
    )
    invalidation.start()
    try:
        assert invalidation_done.wait(0.2)
    finally:
        node.gate_audit_pub.release.set()
        transaction.join(2.0)
        invalidation.join(2.0)

    execution_report = json.loads(
        pathlib.Path(
            '{}.execution'.format(node.gate_audit_output_path)
        ).read_text()
    )
    assert execution_report['outcome']['valid_plan'] is False
    assert node.execution_plan_controller.execution_plan_id is None
    assert node.rich_plan_pub.messages[-1].valid is False
    bounded_message = node.gate_audit_pub.messages[-1]
    bounded = json.loads(
        getattr(bounded_message, 'data', bounded_message)
    )
    execution_payload = pathlib.Path(
        '{}.execution'.format(node.gate_audit_output_path)
    ).read_bytes()
    assert bounded['report_sha256'] == remote_node.hashlib.sha256(
        execution_payload
    ).hexdigest()
    node.shutdown_streaming_worker()


@pytest.mark.parametrize('blocked_channel', ['legacy', 'rich'])
def test_blocked_execution_publish_allows_stream_stop_and_recovers(
    tmp_path,
    blocked_channel,
):
    setup = promotion_transaction_node(tmp_path)
    node, prepared, selection, proposal, local_funnel, moveit_funnel = setup
    blocker = BlockingPublisher(
        lambda message: bool(getattr(message, 'valid', True))
    )
    if blocked_channel == 'legacy':
        node.plan_pub = blocker
    else:
        node.rich_plan_pub = blocker
    results = []

    def run_transaction():
        results.append(
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
        )

    transaction = threading.Thread(target=run_transaction)
    transaction.start()
    assert blocker.entered.wait(1.0)
    try:
        assert node.stop_streaming() is True
    finally:
        blocker.release.set()
        transaction.join(2.0)

    execution_report = json.loads(
        pathlib.Path(
            '{}.execution'.format(node.gate_audit_output_path)
        ).read_text()
    )
    assert results[0][1].promote is False
    assert results[0][1].code == 'GENERATION_STALE'
    assert execution_report['outcome']['valid_plan'] is False
    assert node.execution_plan_controller.execution_plan_id is None
    assert node._request_promotion_count(prepared.ticket) == 0
    preview_path = pathlib.Path(node.gate_audit_output_path)
    if preview_path.exists():
        preview_report = json.loads(preview_path.read_text())
        assert preview_report['promotion']['promote'] is False
        assert preview_report['pipeline_funnel']['stage_counts']['promoted'][
            'passed'
        ] == 0
    if blocked_channel == 'legacy':
        assert node.plan_pub.messages[-1].poses == []
    rich_messages = list(node.rich_plan_pub.messages)
    assert not rich_messages or rich_messages[-1].valid is False
    node.shutdown_streaming_worker()


@pytest.mark.parametrize('state_change', ['target_epoch', 'robot_active'])
def test_blocked_rich_publish_state_change_cannot_commit_authority(
    tmp_path,
    state_change,
):
    setup = promotion_transaction_node(tmp_path)
    node, prepared, selection, proposal, local_funnel, moveit_funnel = setup
    blocker = BlockingPublisher(
        lambda message: bool(getattr(message, 'valid', False))
    )
    node.rich_plan_pub = blocker
    results = []

    def run_transaction():
        results.append(
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
        )

    transaction = threading.Thread(target=run_transaction)
    transaction.start()
    assert blocker.entered.wait(1.0)
    if state_change == 'target_epoch':
        with node._stream_condition:
            node.target_instance_epoch += 1
    else:
        node.grasp_state_cb(types.SimpleNamespace(active=True))
    blocker.release.set()
    transaction.join(2.0)

    assert results[0][1].promote is False
    assert node.execution_plan_controller.execution_plan_id is None
    assert node._request_promotion_count(prepared.ticket) == 0
    assert node.rich_plan_pub.messages[-1].valid is False
    node.shutdown_streaming_worker()


@pytest.mark.parametrize('blocked_channel', ['legacy', 'rich'])
def test_blocked_execution_publish_allows_hard_invalidation_and_recovers(
    tmp_path,
    blocked_channel,
):
    setup = promotion_transaction_node(tmp_path)
    node, prepared, selection, proposal, local_funnel, moveit_funnel = setup
    blocker = BlockingPublisher(
        lambda message: bool(getattr(message, 'valid', True))
    )
    if blocked_channel == 'legacy':
        node.plan_pub = blocker
    else:
        node.rich_plan_pub = blocker
    errors = []

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
            errors.append(exc)

    transaction = threading.Thread(target=run_transaction)
    transaction.start()
    assert blocker.entered.wait(1.0)
    invalidation_done = threading.Event()
    invalidation = threading.Thread(
        target=lambda: (
            node._invalidate_geometry('TARGET_LOST', 'target lost'),
            invalidation_done.set(),
        )
    )
    invalidation.start()
    try:
        assert invalidation_done.wait(0.2), (
            'hard invalidation blocked on {} publisher'.format(
                blocked_channel
            )
        )
    finally:
        blocker.release.set()
        transaction.join(2.0)
        invalidation.join(2.0)

    execution_report = json.loads(
        pathlib.Path(
            '{}.execution'.format(node.gate_audit_output_path)
        ).read_text()
    )
    assert execution_report['outcome']['valid_plan'] is False
    assert node.execution_plan_controller.execution_plan_id is None
    rich_messages = list(node.rich_plan_pub.messages)
    assert not rich_messages or rich_messages[-1].valid is False
    node.shutdown_streaming_worker()


def test_held_preview_does_not_overwrite_execution_audit(tmp_path):
    setup = promotion_transaction_node(tmp_path)
    node, prepared, selection, proposal, local_funnel, moveit_funnel = setup
    execution_path = pathlib.Path(
        '{}.execution'.format(node.gate_audit_output_path)
    )
    try:
        _funnel, promoted = node._finalize_promotion_transaction(
            prepared,
            selection,
            proposal,
            local_funnel,
            moveit_funnel,
            1,
            'PREVIEW_READY',
            {'summary': {}, 'rows': []},
        )
        assert promoted.promote is True
        execution_a = json.loads(execution_path.read_text())

        proposal_b = dict(proposal)
        proposal_b['rich_plan'] = promotion_plan('preview-B', x=0.105)
        proposal_b['score'] = 2.0
        node._latest_streaming_audit = {
            'request_id': prepared.ticket.request_id,
            'plan_id': 'preview-B',
        }
        _funnel, held = node._finalize_promotion_transaction(
            prepared,
            selection,
            proposal_b,
            local_funnel,
            moveit_funnel,
            1,
            'PREVIEW_READY',
            {'summary': {}, 'rows': []},
        )

        preview_b = json.loads(
            pathlib.Path(node.gate_audit_output_path).read_text()
        )
        execution_after_b = json.loads(execution_path.read_text())
        assert held.promote is False
        assert preview_b['plan_id'] == 'preview-B'
        assert execution_after_b == execution_a
        assert execution_after_b['plan_id'] == 'plan-A'
        assert execution_after_b['outcome']['valid_plan'] is True
    finally:
        node.shutdown_streaming_worker()


def test_incomplete_same_plan_replan_keeps_existing_execution_audit(
    tmp_path,
    monkeypatch,
):
    setup = promotion_transaction_node(tmp_path)
    node, prepared, selection, proposal, local_funnel, moveit_funnel = setup
    execution_path = pathlib.Path(
        '{}.execution'.format(node.gate_audit_output_path)
    )
    try:
        _funnel, promoted = node._finalize_promotion_transaction(
            prepared,
            selection,
            proposal,
            local_funnel,
            moveit_funnel,
            1,
            'PREVIEW_READY',
            {'summary': {}, 'rows': []},
        )
        assert promoted.promote is True
        execution_a = json.loads(execution_path.read_text())

        node._stream_source_clock.value = 12.0
        requested = node.execution_plan_controller.request_replan(False)
        assert requested.code == 'REPLAN_REQUESTED'
        proposal_b = dict(proposal)
        proposal_b['score'] = 0.5
        original = node._build_streaming_gate_audit_report

        def incomplete_report(*args, **kwargs):
            report, summary, selected = original(*args, **kwargs)
            if kwargs.get('execution_authority', False):
                report['stable_evaluations'][0]['final_score'] = None
                report['selected']['final_score'] = None
                report['lineage'] = remote_node.deepcopy(
                    report['stable_evaluations']
                )
            return report, summary, selected

        monkeypatch.setattr(
            node,
            '_build_streaming_gate_audit_report',
            incomplete_report,
        )
        with pytest.raises(remote_node.CandidateContractError):
            node._finalize_promotion_transaction(
                prepared,
                selection,
                proposal_b,
                local_funnel,
                moveit_funnel,
                1,
                'PREVIEW_READY',
                {'summary': {}, 'rows': []},
            )

        execution_after = json.loads(execution_path.read_text())
        assert execution_after == execution_a
        assert execution_after['outcome']['valid_plan'] is True
        assert node.execution_plan_controller.execution_plan_id == 'plan-A'
    finally:
        node.shutdown_streaming_worker()


def test_failed_replan_restores_previous_execution_audit(tmp_path):
    setup = promotion_transaction_node(tmp_path)
    node, prepared, selection, proposal, local_funnel, moveit_funnel = setup
    execution_path = pathlib.Path(
        '{}.execution'.format(node.gate_audit_output_path)
    )
    try:
        _funnel, promoted = node._finalize_promotion_transaction(
            prepared,
            selection,
            proposal,
            local_funnel,
            moveit_funnel,
            1,
            'PREVIEW_READY',
            {'summary': {}, 'rows': []},
        )
        assert promoted.promote is True
        execution_a = json.loads(execution_path.read_text())

        node._stream_source_clock.value = 12.0
        node.execution_plan_controller.request_replan(False)
        proposal_b = dict(proposal)
        proposal_b['rich_plan'] = promotion_plan('plan-B', x=0.15)
        proposal_b['score'] = 0.5
        node._latest_streaming_audit = {
            'request_id': prepared.ticket.request_id,
            'plan_id': 'plan-B',
        }
        node.plan_pub = FailingPublisher()
        _funnel, failed = node._finalize_promotion_transaction(
            prepared,
            selection,
            proposal_b,
            local_funnel,
            moveit_funnel,
            1,
            'PREVIEW_READY',
            {'summary': {}, 'rows': []},
        )

        assert failed.code == 'PLAN_PUBLICATION_FAILED'
        assert json.loads(execution_path.read_text()) == execution_a
        assert node.execution_plan_controller.execution_plan_id == 'plan-A'
        assert [item.plan_id for item in node.rich_plan_pub.messages] == [
            'plan-A',
            'plan-A',
        ]
    finally:
        node.shutdown_streaming_worker()


def test_replan_service_rejects_false_and_active_without_touching_streaming():
    node = promotion_node()
    node.streaming_enabled = True
    seed_execution(node, promotion_plan('plan-A'))

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
    seed_execution(node, promotion_plan('plan-A'))
    clock.value = 11.1

    response = node.replan_execution_cb(types.SimpleNamespace(trigger=True))
    decision = node._maybe_promote_preview(
        promotion_plan('plan-B', x=0.15),
        signature='candidate-B',
        score=0.5,
    )

    assert response.success is True
    assert decision.promote is True
    assert [item.plan_id for item in node.rich_plan_pub.messages] == ['plan-A']
    assert node.execution_plan_controller.execution_plan_id == 'plan-A'


def test_replan_service_promotes_cached_preview_when_streaming_is_stopped():
    clock = MutableClock(10.0)
    node = promotion_node(clock=clock)
    seed_execution(node, promotion_plan('plan-A'), signature='target-A')
    preview = promotion_plan('plan-B', x=0.15)
    node.latest_preview_rich_plan = remote_node.deepcopy(preview)
    node._latest_preview_proposal = {
        'plan_id': preview.plan_id,
        'signature': 'target-B',
        'score': 0.5,
        'expected_generation': 0,
    }
    node.streaming_enabled = False
    clock.value = 11.1

    response = node.replan_execution_cb(types.SimpleNamespace(trigger=True))

    assert response.success is True
    assert 'cached Preview' in response.message
    assert node.latest_rich_plan.plan_id == 'plan-B'
    assert node.execution_plan_controller.execution_plan_id == 'plan-B'
    assert node.execution_plan_controller.execution_signature == 'target-B'
    assert [item.plan_id for item in node.rich_plan_pub.messages] == [
        'plan-A',
        'plan-B',
    ]


def test_replan_service_promotes_cached_preview_while_streaming_runs():
    clock = MutableClock(10.0)
    node = promotion_node(clock=clock)
    seed_execution(node, promotion_plan('plan-A'), signature='target-A')
    preview = promotion_plan('plan-B', x=0.15)
    node.latest_preview_rich_plan = remote_node.deepcopy(preview)
    node._latest_preview_proposal = {
        'plan_id': preview.plan_id,
        'signature': 'target-B',
        'score': 0.5,
        'expected_generation': 0,
    }
    node.streaming_enabled = True
    clock.value = 11.1

    response = node.replan_execution_cb(types.SimpleNamespace(trigger=True))

    assert response.success is True
    assert 'cached Preview' in response.message
    assert node.latest_rich_plan.plan_id == 'plan-B'
    assert node.execution_plan_controller.execution_plan_id == 'plan-B'
    assert node.execution_plan_controller.execution_signature == 'target-B'
    assert [item.plan_id for item in node.rich_plan_pub.messages] == [
        'plan-A',
        'plan-B',
    ]


def test_cached_replan_and_worker_share_execution_publication_transaction():
    node = promotion_node()
    node._execution_publication_lock = threading.RLock()
    cached_entered = threading.Event()
    release_cached = threading.Event()
    worker_entered = threading.Event()
    cached_results = []
    worker_results = []

    def cached_transaction():
        cached_entered.set()
        assert release_cached.wait(2.0)
        return PromotionDecision(
            True,
            'PROMOTED_CACHED_PREVIEW',
            'cached authority committed',
        )

    def worker_transaction(*_args):
        worker_entered.set()
        return PromotionDecision(
            False,
            'PROMOTION_SUPERSEDED',
            'older worker transaction superseded',
        )

    node._try_promote_cached_preview_after_replan_serialized = (
        cached_transaction
    )
    node._publish_execution_with_token_serialized = worker_transaction

    cached_thread = threading.Thread(
        target=lambda: cached_results.append(
            node._try_promote_cached_preview_after_replan()
        )
    )
    worker_thread = threading.Thread(
        target=lambda: worker_results.append(
            node._publish_execution_with_token(
                None,
                '',
                0.0,
                0.0,
                None,
                None,
            )
        )
    )
    cached_thread.start()
    assert cached_entered.wait(1.0)
    worker_thread.start()
    try:
        assert not worker_entered.wait(0.1)
    finally:
        release_cached.set()
        cached_thread.join(2.0)
        worker_thread.join(2.0)

    assert not cached_thread.is_alive()
    assert not worker_thread.is_alive()
    assert cached_results[0].promote is True
    assert worker_results[0].code == 'PROMOTION_SUPERSEDED'
    assert worker_entered.is_set()


def test_replan_policy_mutation_waits_for_worker_publication_transaction():
    node = promotion_node()
    node._execution_publication_lock = threading.RLock()
    worker_entered = threading.Event()
    release_worker = threading.Event()
    replan_entered = threading.Event()
    worker_results = []
    replan_results = []

    def worker_transaction(*_args):
        worker_entered.set()
        assert release_worker.wait(2.0)
        return PromotionDecision(
            True,
            'PROMOTED',
            'worker authority committed',
        )

    def replan_transaction(_req):
        replan_entered.set()
        node.execution_plan_controller.request_replan(robot_active=False)
        return types.SimpleNamespace(success=True, message='requested')

    node._publish_execution_with_token_serialized = worker_transaction
    node._replan_execution_cb_serialized = replan_transaction
    worker_thread = threading.Thread(
        target=lambda: worker_results.append(
            node._publish_execution_with_token(
                None,
                '',
                0.0,
                0.0,
                None,
                None,
            )
        )
    )
    replan_thread = threading.Thread(
        target=lambda: replan_results.append(
            node.replan_execution_cb(types.SimpleNamespace(trigger=True))
        )
    )
    worker_thread.start()
    assert worker_entered.wait(1.0)
    replan_thread.start()
    try:
        assert not replan_entered.wait(0.1)
        assert (
            node.execution_plan_controller.explicit_replan_requested is False
        )
    finally:
        release_worker.set()
        worker_thread.join(2.0)
        replan_thread.join(2.0)

    assert not worker_thread.is_alive()
    assert not replan_thread.is_alive()
    assert worker_results[0].promote is True
    assert replan_results[0].success is True
    assert replan_entered.is_set()
    assert node.execution_plan_controller.explicit_replan_requested is True


def cached_preview_audit_report(plan_id='plan-B'):
    selected = {
        'candidate_source': 'graspnet',
        'source_lineage': ['graspnet'],
        'source_index': 4,
        'candidate_index': 4,
        'variant_index': 0,
        'selected': True,
        'tracking': {
            'track_id': 7,
            'hit_count': 3,
            'window_count': 5,
            'hit_request_ids': [1, 2, 3],
        },
        'lineage_binding': {
            'source_request_id': 8,
            'source_snapshot_stamp_sec': 20.0,
            'source_raw_candidate_index': 4,
            'source_variant_index': 0,
            'evaluation_request_id': 9,
            'evaluation_snapshot_stamp_sec': 20.0,
            'evaluation_variant_index': 0,
        },
        'final_score': 0.5,
        'moveit': {
            'reachable': True,
            'collision_free': True,
            'within_joint_limits': True,
            'ik_valid': True,
            'planning_success': True,
            'failure_code': '',
            'evidence_code': '',
            'joint_path_cost': 0.1,
            'joint_max_delta_rad': 0.2,
        },
    }
    return {
        'mode': 'continuous_preview',
        'request_id': 9,
        'generation': 1,
        'target_epoch': 2,
        'snapshot_stamp_sec': 20.0,
        'plan_id': str(plan_id),
        'pipeline_funnel': {'stage_counts': {}},
        'summary': {},
        'rows': [remote_node.deepcopy(selected)],
        'stable_evaluations': [
            remote_node.deepcopy(selected),
            {'candidate_source': 'graspnet', 'source_index': 9},
        ],
        'lineage': [remote_node.deepcopy(selected)],
        'selected': remote_node.deepcopy(selected),
        'outcome': {'valid_plan': False, 'preview_valid': True},
    }


def test_cached_preview_promotion_writes_execution_audit(tmp_path):
    node = promotion_node()
    node.gate_audit_enabled = True
    node.gate_audit_output_path = str(tmp_path / 'preview.json')
    node.mujoco_audit_output_path = str(tmp_path / 'mujoco.json')
    node.gate_audit_pub = RecordingPublisher()
    node._execution_promotion_audit_ready = (
        remote_node.RemoteGrasp6DNode._execution_promotion_audit_ready
        .__get__(node, remote_node.RemoteGrasp6DNode)
    )
    node._active_gate_audit_report = cached_preview_audit_report('plan-B')
    decision = PromotionDecision(
        True,
        'PROMOTE_REPLAN',
        'explicit execution replan requested',
    )

    ready, reason = node._write_cached_preview_execution_audit(
        promotion_plan('plan-B'),
        decision,
    )

    execution_report = json.loads(
        pathlib.Path(
            '{}.execution'.format(node.gate_audit_output_path)
        ).read_text()
    )
    assert ready is True
    assert reason == ''
    assert execution_report['mode'] == 'continuous_execution'
    assert execution_report['plan_id'] == 'plan-B'
    assert execution_report['outcome']['valid_plan'] is True
    assert execution_report['promotion']['code'] == 'PROMOTE_REPLAN'
    assert execution_report['stable_evaluations'] == [
        execution_report['selected']
    ]


def test_cached_preview_promotion_uses_bound_audit_after_active_audit_overwrite(
    tmp_path,
):
    node = promotion_node()
    node.gate_audit_enabled = True
    node.gate_audit_output_path = str(tmp_path / 'preview.json')
    node.mujoco_audit_output_path = str(tmp_path / 'mujoco.json')
    node.gate_audit_pub = RecordingPublisher()
    node._execution_promotion_audit_ready = (
        remote_node.RemoteGrasp6DNode._execution_promotion_audit_ready
        .__get__(node, remote_node.RemoteGrasp6DNode)
    )
    node._latest_preview_gate_audit_report = (
        cached_preview_audit_report('plan-B')
    )
    node._active_gate_audit_report = {
        'mode': 'continuous_preview',
        'request_id': 10,
        'generation': 1,
        'target_epoch': 2,
        'snapshot_stamp_sec': 21.0,
        'plan_id': '',
        'summary': {},
        'rows': [],
        'stable_evaluations': [],
        'lineage': [],
        'selected': None,
        'outcome': {'valid_plan': False, 'preview_valid': False},
    }
    decision = PromotionDecision(
        True,
        'PROMOTE_REPLAN',
        'explicit execution replan requested',
    )

    ready, reason = node._write_cached_preview_execution_audit(
        promotion_plan('plan-B'),
        decision,
    )

    assert ready is True, reason
    assert reason == ''
    execution_report = json.loads(
        pathlib.Path(
            '{}.execution'.format(node.gate_audit_output_path)
        ).read_text()
    )
    assert execution_report['plan_id'] == 'plan-B'


def test_expired_execution_plan_allows_fresh_preview_promotion():
    clock = MutableClock(50.0)
    node = promotion_node(clock=clock)
    node.execution_plan_validity_sec = 30.0
    expired = promotion_plan('plan-A')
    expired.header.stamp = 10.0
    seed_execution(node, expired)

    decision = node._maybe_promote_preview(
        promotion_plan('plan-B', x=0.13),
        signature='candidate-B',
        score=0.5,
    )

    assert decision.promote is True
    assert decision.code == 'PROMOTE_INITIAL'
    assert node.execution_plan_controller.execution_plan_id is None
    assert node.latest_rich_plan is None
    assert node.latest_plan is None
    assert node._latest_promotion_decision.code == 'EXECUTION_EXPIRED'


def test_epoch_stamped_execution_plan_expires_against_ros_time_not_monotonic():
    clock = MutableClock(50.0)
    node = promotion_node(clock=clock)
    node.execution_plan_validity_sec = 30.0
    node._execution_plan_validity_now_sec = lambda: 1784955350.0
    expired = promotion_plan('plan-A')
    expired.header.stamp = 1784955000.0
    seed_execution(node, expired)

    decision = node._maybe_promote_preview(
        promotion_plan('plan-B', x=0.13),
        signature='candidate-B',
        score=0.5,
    )

    assert decision.promote is True
    assert decision.code == 'PROMOTE_INITIAL'
    assert node.execution_plan_controller.execution_plan_id is None
    assert node.latest_rich_plan is None
    assert node.latest_plan is None
    assert node._latest_promotion_decision.code == 'EXECUTION_EXPIRED'


def test_preview_failure_only_updates_invalid_streak_not_execution_topics():
    node = promotion_node()
    seed_execution(node, promotion_plan('plan-A'))

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
    seed_execution(
        node,
        promotion_plan('plan-A'),
        signature=signature,
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
    seed_execution(
        node,
        promotion_plan('plan-A'),
        signature=signature,
    )
    clock.value = 11.1

    decision = node._maybe_promote_preview(
        challenger,
        signature=signature,
        score=0.5,
    )

    assert decision.promote is True
    assert decision.code == 'PROMOTE_REPLAN'
    assert len(node.rich_plan_pub.messages) == 1


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


def test_far_field_rank_resolves_live_side_uncertainty_before_distance():
    node = remote_node.RemoteGrasp6DNode.__new__(
        remote_node.RemoteGrasp6DNode
    )
    candidates = [
        types.SimpleNamespace(
            track_id=1,
            variant_index=0,
            pre_moveit_score=0.1,
        ),
        types.SimpleNamespace(
            track_id=2,
            variant_index=0,
            pre_moveit_score=0.2,
        ),
        types.SimpleNamespace(
            track_id=3,
            variant_index=0,
            pre_moveit_score=0.3,
        ),
    ]
    node._stable_variant_runtime = {
        (1, 0): {
            'soft_evidence': {
                'observation_side_evidence_deficit_m': 0.001,
                'observation_projected_side_evidence_m': 0.002,
                'observation_translation_delta_m': 0.010,
            }
        },
        (2, 0): {
            'soft_evidence': {
                'observation_side_evidence_deficit_m': 0.0,
                'observation_projected_side_evidence_m': 0.005,
                'observation_translation_delta_m': 0.040,
            }
        },
        (3, 0): {
            'soft_evidence': {
                'observation_side_evidence_deficit_m': 0.0,
                'observation_projected_side_evidence_m': 0.004,
                'observation_translation_delta_m': 0.020,
            }
        },
    }

    ranked = sorted(
        candidates,
        key=node._far_field_observation_moveit_rank_key,
    )

    assert [candidate.track_id for candidate in ranked] == [3, 2, 1]
    assert node._select_far_field_observation(candidates).track_id == 3


def test_far_field_rank_never_prefers_missing_side_evidence():
    node = remote_node.RemoteGrasp6DNode.__new__(
        remote_node.RemoteGrasp6DNode
    )
    evidenced = types.SimpleNamespace(
        track_id=1,
        variant_index=0,
        pre_moveit_score=100.0,
    )
    missing = types.SimpleNamespace(
        track_id=2,
        variant_index=0,
        pre_moveit_score=0.0,
    )
    node._stable_variant_runtime = {
        (1, 0): {
            'soft_evidence': {
                'observation_side_evidence_deficit_m': 0.0,
                'observation_projected_side_evidence_m': 0.003,
                'observation_translation_delta_m': 0.100,
            }
        },
        (2, 0): {'soft_evidence': {}},
    }

    ranked = sorted(
        [missing, evidenced],
        key=node._far_field_observation_moveit_rank_key,
    )

    assert [candidate.track_id for candidate in ranked] == [1, 2]


def test_observation_side_evidence_uses_live_height_and_uncertainty():
    node = remote_node.RemoteGrasp6DNode.__new__(
        remote_node.RemoteGrasp6DNode
    )
    configure_identity_handeye(node)
    geometry = tabletop_geometry(size_xyz=(0.051, 0.035, 0.011))
    geometry.support_normal_base.setflags(write=False)
    stage_profile = types.SimpleNamespace(depth_uncertainty_m=0.003)

    def observation_at_camera_incidence(incidence_deg):
        rotation = np.eye(4, dtype=float)
        rotation[:3, :3] = remote_node.quaternion_matrix(
            remote_node.quaternion_from_euler(
                0.0,
                math.radians(90.0 - incidence_deg),
                0.0,
            )
        )[:3, :3]
        return remote_node.make_pose_stamped(
            'base_link',
            np.array([0.0, 0.0, 0.2]),
            remote_node.quaternion_from_matrix(rotation),
            stamp=remote_node.rospy.Time.from_sec(20.0),
        )

    top_down = node._observation_side_evidence(
        geometry,
        observation_at_camera_incidence(0.0),
        stage_profile,
    )
    tilted = node._observation_side_evidence(
        geometry,
        observation_at_camera_incidence(30.0),
        stage_profile,
    )

    assert top_down['observation_incidence_angle_deg'] == pytest.approx(0.0)
    assert top_down['observation_object_height_m'] == pytest.approx(0.011)
    assert top_down['observation_projected_side_evidence_m'] == pytest.approx(
        0.0
    )
    assert top_down['observation_side_evidence_deficit_m'] == pytest.approx(
        0.003
    )
    assert tilted['observation_incidence_angle_deg'] == pytest.approx(30.0)
    assert tilted['observation_projected_side_evidence_m'] == pytest.approx(
        0.011 * math.sin(math.radians(30.0))
    )
    assert tilted['observation_side_evidence_deficit_m'] == pytest.approx(0.0)


def _mujoco_selection_candidate(track_id, final_score):
    return types.SimpleNamespace(
        track_id=int(track_id),
        variant_index=0,
        final_score=float(final_score),
        evaluation_snapshot_stamp_sec=20.0,
        stable_candidate=types.SimpleNamespace(
            candidate_source='tabletop_geometry',
        ),
    )


def _mujoco_selection_node(candidates):
    node = remote_node.RemoteGrasp6DNode.__new__(
        remote_node.RemoteGrasp6DNode
    )
    node.mujoco_config = {
        'enabled': True,
        'execution_gate_enabled': True,
        'server_url': 'http://127.0.0.1:8000',
        'timeout_sec': 20.0,
        'min_score': 80,
    }
    node.mujoco_selection_gate_enabled = True
    node.mujoco_selection_max_candidates = 10
    node.mujoco_selection_time_budget_sec = 80.0
    node.latest_joint_state = types.SimpleNamespace(
        name=['Joint%d' % index for index in range(1, 7)],
        position=[0.0] * 6,
    )
    node._stable_variant_runtime = {
        (candidate.track_id, candidate.variant_index): {}
        for candidate in candidates
    }
    node._require_stream_ticket_current = lambda _ticket: None
    node._build_selected_preview_bundle = lambda candidate: {
        'rich_plan': types.SimpleNamespace(
            plan_id='plan-%d' % candidate.track_id,
            candidate_source='tabletop_geometry',
            candidate_source_lineage=['tabletop_geometry'],
        )
    }
    node._mujoco_client_factory = lambda _url, timeout_sec: types.SimpleNamespace(
        simulate_grasp=lambda payload: {'plan_id': payload['plan_id']}
    )
    return node


def test_near_field_mujoco_selection_tries_next_reachable_candidate(
    monkeypatch,
):
    first = _mujoco_selection_candidate(1, 0.1)
    second = _mujoco_selection_candidate(2, 0.2)
    node = _mujoco_selection_node((first, second))
    monkeypatch.setattr(
        remote_node,
        'build_mujoco_payload',
        lambda plan, _names, _positions, _cfg: {
            'plan_id': plan.plan_id,
        },
    )

    def validate(_response, plan_id, _score, **_kwargs):
        if plan_id == 'plan-1':
            return types.SimpleNamespace(
                ok=False,
                code='MUJOCO_CONTACT_FAILED',
                reason='contact lost during prescribed lift',
                score=55.0,
            )
        return types.SimpleNamespace(
            ok=True,
            code='',
            reason='',
            score=91.0,
        )

    monkeypatch.setattr(
        remote_node,
        'validate_mujoco_gate_response',
        validate,
    )
    funnel = CandidateStageFunnel(2)
    selection = BoundedMoveItSelection(
        selected=first,
        checked=(first, second),
        reachable=(second, first),
        funnel=funnel,
        configured_top_n=2,
    )
    prepared = types.SimpleNamespace(
        near_field=True,
        ticket=object(),
    )

    screened, status = node._screen_near_field_selection_with_mujoco(
        prepared,
        selection,
    )

    assert screened.selected is second
    assert status == 'MUJOCO_SELECTION_PASSED'
    assert node._stable_variant_runtime[(1, 0)]['mujoco_selection'][
        'code'
    ] == 'MUJOCO_CONTACT_FAILED'
    assert node._stable_variant_runtime[(2, 0)]['mujoco_selection'][
        'passed'
    ] is True
    stage = funnel.to_dict()['stage_counts']['mujoco_screened']
    assert stage == {'entered': 2, 'passed': 1, 'rejected': 1}


def test_near_field_mujoco_selection_stops_on_authority_failure(monkeypatch):
    first = _mujoco_selection_candidate(1, 0.1)
    second = _mujoco_selection_candidate(2, 0.2)
    node = _mujoco_selection_node((first, second))
    monkeypatch.setattr(
        remote_node,
        'build_mujoco_payload',
        lambda plan, _names, _positions, _cfg: {
            'plan_id': plan.plan_id,
        },
    )
    monkeypatch.setattr(
        remote_node,
        'validate_mujoco_gate_response',
        lambda *_args, **_kwargs: types.SimpleNamespace(
            ok=False,
            code='PLAN_ID_MISMATCH',
            reason='response does not bind the requested plan',
            score=0.0,
        ),
    )
    funnel = CandidateStageFunnel(2)
    selection = BoundedMoveItSelection(
        selected=first,
        checked=(first, second),
        reachable=(first, second),
        funnel=funnel,
        configured_top_n=2,
    )

    screened, status = node._screen_near_field_selection_with_mujoco(
        types.SimpleNamespace(near_field=True, ticket=object()),
        selection,
    )

    assert screened.selected is None
    assert status == 'PLAN_ID_MISMATCH'
    assert 'mujoco_selection' in node._stable_variant_runtime[(1, 0)]
    assert 'mujoco_selection' not in node._stable_variant_runtime[(2, 0)]
    stage = funnel.to_dict()['stage_counts']['mujoco_screened']
    assert stage == {'entered': 1, 'passed': 0, 'rejected': 1}


def test_mujoco_response_audit_preserves_strict_contact_loss_evidence():
    response = {
        'plan_id': 'plan-7',
        'failure_code': 'MUJOCO_CONTACT_FAILED',
        'failure_reason': 'contact lost during prescribed lift',
        'score': 54.0,
        'simulation_ok': False,
        'ik_success': True,
        'collision_free': True,
        'contact_success': False,
        'lift_success': False,
        'used_joint_state_source': 'request.current_joint_state',
        'diagnosis': [
            'initial bilateral contact established',
            'right contact lost at lift alpha 0.083',
        ],
        'ik_results': [
            {'stage': 'pregrasp', 'success': True},
            {'stage': 'lift', 'success': True},
        ],
        'lift_evidence': {
            'samples': [
                {
                    'alpha': 0.083,
                    'left_contact': True,
                    'right_contact': False,
                    'object_lift_m': 0.000055,
                }
            ],
            'maximum_object_lift_m': 0.000055,
        },
    }

    audit = remote_node.RemoteGrasp6DNode._mujoco_response_audit(response)
    response['lift_evidence']['samples'][0]['right_contact'] = True

    assert audit['strict_json'] is True
    assert len(audit['response_sha256']) == 64
    assert audit['diagnosis'][1].endswith('lift alpha 0.083')
    assert audit['ik_results'][1] == {'stage': 'lift', 'success': True}
    assert audit['lift_evidence']['samples'][0]['right_contact'] is False
    assert audit['used_joint_state_source'] == (
        'request.current_joint_state'
    )
