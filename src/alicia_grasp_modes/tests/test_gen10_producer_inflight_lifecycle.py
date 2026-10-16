"""Independent real producer callback/worker interleavings; no ROS master."""
import threading

import pytest

from test_station_compute_wait_producer import armed
from test_locked_rgb_hold import custom_frame, latest_status, module
from test_observation_measurement_unknown import send, ack
from test_supplement_measurement_admission import next_request


@pytest.mark.parametrize('fault', ['none', 'epoch', 'manual', 'deadline'])
def test_old_worker_exception_after_executing_ack_obeys_current_lease(monkeypatch, fault):
    node, clock, joint, params, marker, first = armed(monkeypatch)
    old = node._locked_rgb_hold_reference
    entered, release = threading.Event(), threading.Event()
    errors = []
    original_timing = node._frame_timing_stage
    object_count = node.pubs['object'].publish.call_count

    def timing(stage, *args, **kwargs):
        result = original_timing(stage, *args, **kwargs)
        if stage == 'post_segment':
            entered.set()
            assert release.wait(3.)
            raise ValueError('independent_old_inflight_computation_failed')
        return result

    def worker():
        try:
            custom_frame(node, clock, joint, target_depth=.35)
        except Exception as exc:
            errors.append(exc)

    monkeypatch.setattr(node, '_frame_timing_stage', timing)
    base = old['source_stamp_ns'] * 1e-9
    clock[0] = base + .5
    thread = threading.Thread(target=worker)
    thread.start()
    assert entered.wait(3.)
    try:
        clock[0] = base + .7
        joint(clock[0] - .001)
        request = next_request(node)
        send(node, request)
        assert ack(node)['accepted'] is True, ack(node)
        assert node._station_compute_registration is None
        assert node._observation_measurement['request'] == request
        if fault == 'epoch':
            node._rgb_interest_epoch += 1
        elif fault == 'manual':
            params['/gui/joint_direct_mode'] = True
        elif fault == 'deadline':
            clock[0] = request['deadline_ns'] / 1e9 + .001
    finally:
        release.set()
        thread.join(3.)
    assert not thread.is_alive() and errors == [], errors
    if fault == 'none':
        assert node.first_loss is None and not node.tracker.lost
        assert node._observation_measurement['request'] == request
        assert node.pubs['object'].publish.call_count == object_count
        assert latest_status(node)['state'] == 'pending'
        assert latest_status(node)['detail'] == 'observation_measurement_pending'
    else:
        assert node.first_loss is not None and node.tracker.lost
        assert node._observation_measurement is None
        assert ack(node)['accepted'] is False
        assert 'observation_measurement_' in latest_status(node)['detail']
