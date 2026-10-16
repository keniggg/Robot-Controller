"""Input provenance survives rejected results without trusting their output."""

import json
import types

import numpy as np
import pytest

import test_remote_grasp6d_streaming as fixtures
from alicia_flexible_grasp.vision.rgbd_snapshot import DepthQuality, SnapshotResult


remote = fixtures.remote_node


@pytest.fixture(autouse=True)
def prohibit_ros_services(monkeypatch):
    def forbidden(*_args, **_kwargs):
        pytest.fail('input evidence tests must not access a ROS service')

    monkeypatch.setattr(remote.rospy, 'wait_for_service', forbidden)
    monkeypatch.setattr(remote.rospy, 'ServiceProxy', forbidden)


def frozen_snapshot(node, newest_ns):
    return SnapshotResult(
        ok=True, failure_code='', failure_reason='',
        color_bgr=np.zeros((2, 2, 3), dtype=np.uint8),
        depth_raw=np.zeros((2, 2), dtype=np.uint16),
        target_depth_raw=np.zeros((2, 2), dtype=np.uint16),
        object_mask=np.ones((2, 2), dtype=np.uint8),
        bbox=(0, 0, 2, 2), object_msg=types.SimpleNamespace(detected=True),
        stamp_sec=newest_ns / 1e9, stamp_ns=newest_ns, frame_id='camera_link',
        quality=DepthQuality(3, 4, 4, 1.0, .1, 0.0), source_mode='instance_mask',
        target_epoch=node.target_instance_epoch,
        target_identity=node._current_stream_target_identity(),
        sample_stamp_ns=(newest_ns - 100_000_003, newest_ns - 1, newest_ns),
    )


def expected_evidence(snapshot):
    return {
        'snapshot_stamp_ns': snapshot.stamp_ns,
        'source_stamp_ns': list(snapshot.sample_stamp_ns),
        'unique_source_frames': 3,
        'source_span_ms': 100.000003,
    }


@pytest.mark.parametrize('drop_metadata', [False, True])
@pytest.mark.parametrize('failure', ['expired', 'predict_failed'])
def test_failed_worker_reports_frozen_input_without_trusting_prediction(
    failure, drop_metadata,
):
    clock = fixtures.MutableClock(1789904374.1)
    node = fixtures.streaming_node(clock=clock)

    def prepare(ticket):
        if drop_metadata:
            with node._stream_condition:
                node._request_telemetry.pop(ticket.request_id)
        if failure == 'predict_failed':
            raise RuntimeError('preparation failed before producing output')
        clock.value += 2.0
        return types.SimpleNamespace(
            ticket=ticket,
            snapshot=types.SimpleNamespace(sample_stamp_ns=(123, 456)),
            near_field=True,
            remote_diagnostics={'snapshot_evidence': {'source_stamp_ns': [987]}},
            remote_performance={'server_total_ms': 999.0, 'inference_ms': 888.0},
        )

    node._prepare_and_predict = prepare
    try:
        node.start_streaming()
        snapshot = frozen_snapshot(node, 1_789_904_374_069_951_715)
        assert node.submit_stream_snapshot(snapshot)
        fixtures.wait_until(lambda: len(node.pipeline_metrics) == 1
                            and len(node.pipeline_metrics_pub.messages) == 1)

        metric = node.pipeline_metrics[0]
        assert metric['status'] == (
            'RESULT_EXPIRED' if failure == 'expired' else 'PREDICT_FAILED'
        )
        assert metric['snapshot_evidence'] == expected_evidence(snapshot)
        assert 'near_field' not in metric['snapshot_evidence']
        assert 'disjoint_window_required' not in metric['snapshot_evidence']
        assert metric['wsl_total_ms'] == metric['wsl_inference_ms'] == 0.0
        assert node._accept_prediction_calls == []
        encoded = json.loads(node.pipeline_metrics_pub.messages[0].data)
        assert encoded['snapshot_evidence'] == expected_evidence(snapshot)
        assert isinstance(encoded['snapshot_evidence']['source_stamp_ns'][-1], int)
    finally:
        node.shutdown_streaming_worker()


def test_pending_replacement_and_phase_change_keep_each_admitted_source_window():
    prediction = fixtures.BlockingPrediction()
    node = fixtures.streaming_node(
        clock=fixtures.MutableClock(10.0), prepare=prediction,
    )
    try:
        node.start_streaming()
        inputs = [frozen_snapshot(node, value) for value in
                  (9_700_000_003, 9_800_000_005, 9_900_000_007)]
        assert node.submit_stream_snapshot(inputs[0])
        assert prediction.entered.wait(1.0)
        admitted = node._request_telemetry[1]['snapshot_evidence']
        with pytest.raises(TypeError):
            admitted['source_stamp_ns'] = (999,)
        assert isinstance(admitted['source_stamp_ns'], tuple)
        assert node.submit_stream_snapshot(inputs[1])
        assert node.submit_stream_snapshot(inputs[2])

        node.near_field_state_cb(fixtures.near_field_phase(
            True, phase_id=17, start_sec=10.0, deadline_sec=30.0,
        ))
        prediction.release.set()
        fixtures.wait_until(lambda: len(node.pipeline_metrics) == 3)

        metrics = {item['request_id']: item for item in node.pipeline_metrics}
        assert metrics[1]['status'] == 'GENERATION_STALE'
        assert metrics[2]['status'] == 'PENDING_REPLACED'
        assert metrics[3]['status'] == 'TARGET_EPOCH_STALE'
        for request_id, snapshot in enumerate(inputs, 1):
            assert metrics[request_id]['snapshot_evidence'] == expected_evidence(snapshot)
            assert 'near_field' not in metrics[request_id]['snapshot_evidence']
        assert node.near_field_planning_active
        assert node._near_field_phase_id == 17
        assert node._accept_prediction_calls == []
        assert node._request_telemetry == {}
    finally:
        prediction.release.set()
        node.shutdown_streaming_worker()


def test_accepted_metrics_keep_local_phase_fields_and_authoritative_input_window():
    node = fixtures.streaming_node()
    node._accept_prediction = lambda _prepared: {
        'status': 'PREVIEW_READY',
        'funnel': {
            'snapshot_evidence': {
                'near_field': True,
                'disjoint_window_required': False,
                'source_stamp_ns': [123],
                'source_span_ms': 999.0,
                'unique_source_frames': 1,
            },
        },
    }
    try:
        node.start_streaming()
        snapshot = frozen_snapshot(node, 9_900_000_007)
        assert node.submit_stream_snapshot(snapshot)
        fixtures.wait_until(lambda: len(node.pipeline_metrics) == 1)

        metric = node.pipeline_metrics[0]
        assert metric['status'] == 'PREVIEW_READY'
        assert metric['snapshot_evidence'] == dict(
            expected_evidence(snapshot), near_field=True,
            disjoint_window_required=False,
        )
    finally:
        node.shutdown_streaming_worker()
