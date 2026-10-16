"""Real final input_lock contention after actual PRE face freeze; no ROS I/O."""
import inspect
from pathlib import Path
import threading

import pytest

from alicia_flexible_grasp.vision.rgbd_rectification import PairedPinholeRectification
from test_unknown_stream import rgb_interest_node, module
from test_observation_measurement_unknown import send, ack
from test_measured_face_source_binding import request_for


def final_lock_case(monkeypatch, mutation):
    assert Path(module.__file__).name == 'unknown_tabletop_perception.py'
    node, clock, feed, joint, params = rgb_interest_node(monkeypatch)
    # The legacy locked() fixture deliberately has no rectification bitmap,
    # hence no optional face proof. Use a real identity geometry contract so
    # actual source-local extraction, rather than legacy fallback, completes.
    camera = params['/camera']
    mapper = PairedPinholeRectification(dict(camera, model='none', coeffs=[0.]*5))
    camera['runtime_profile_geometry'] = dict(geometry_intrinsics=mapper.output_intrinsics,
        rectification=mapper.contract, geometry_activation_stamp_ns=1)
    for _ in range(3):
        feed(target_depth=.35)
        clock[0] += .2
    assert node.tracker.anchor is not None and not node.tracker.lost
    node._locked_rgb_hold_reference.pop('initial_station_current_support', None)
    request = request_for(node, clock, params)
    previous_source = node.last_good_source_stamp_ns
    original_lock = node.input_lock
    main_thread = threading.get_ident()
    copy_complete, final_entered, allow_acquire, will_acquire = [threading.Event() for _ in range(4)]
    frozen, freeze_failures = [], []
    original_freeze = module.freeze_measured_face

    def actual_freeze(*args, **kwargs):
        try:
            result = original_freeze(*args, **kwargs)
        except BaseException as exc:
            freeze_failures.append(str(exc))
            raise
        frozen.append(result)
        copy_complete.set()
        return result

    monkeypatch.setattr(module, 'freeze_measured_face', actual_freeze)

    class ObservedLock:
        def __enter__(self):
            # Inner context/feedback locks are not this final direct callback
            # commit lock. The fixture does not simulate a same-thread RLock.
            caller = inspect.currentframe().f_back.f_code.co_name
            if (threading.get_ident() != main_thread and copy_complete.is_set()
                    and caller == '_observation_measurement_request_cb'):
                final_entered.set()
                assert allow_acquire.wait(5.), 'test never released acquisition barrier'
                will_acquire.set()
            return original_lock.__enter__()

        def __exit__(self, *args):
            return original_lock.__exit__(*args)

    node.input_lock = ObservedLock()
    failures = []

    def executing_callback():
        try:
            send(node, request)
        except BaseException as exc:
            failures.append(exc)

    worker = threading.Thread(target=executing_callback, daemon=True)
    worker.start()
    try:
        assert final_entered.wait(5.), dict(reason='actual face freeze did not reach final commit lock',
            frozen_count=len(frozen), freeze_failures=freeze_failures,
            worker_exceptions=failures, ack=ack(node))
        assert len(frozen) == 1 and frozen[0].source.source_ns == previous_source
        with original_lock:
            allow_acquire.set()
            assert will_acquire.wait(5.)
            assert worker.is_alive(), 'worker must be blocked behind main thread input_lock'
            if mutation == 'epoch':
                node._rgb_interest_epoch += 1
            elif mutation == 'sdk_out_and_back':
                joint(clock[0] - .02, [.005, 0., 0., 0., 0., 0.])
                joint(clock[0] - .01, [0.] * 6)
            elif mutation == 'source_expiry':
                clock[0] += .801
            elif mutation == 'lease_expiry':
                clock[0] += 61.
            elif mutation != 'unchanged':
                raise AssertionError(mutation)
        worker.join(5.)
        assert not worker.is_alive() and not failures, failures
        if mutation == 'unchanged':
            assert ack(node)['accepted'] and not ack(node)['qualified']
            assert node._observation_measurement is not None
            assert node._observation_measurement['confirmation'].appearance_reference.measured_face is frozen[0]
            assert node._observation_measurement['confirmation'].source_stamps_ns == []
        else:
            assert not ack(node)['accepted'], ack(node)
            assert getattr(node, '_observation_measurement', None) is None
            assert request['lease_id'] not in getattr(node, '_observation_measurement_seen', ())
        assert node.last_good_source_stamp_ns == previous_source
        assert ack(node)['source_stamps_ns'] == []
    finally:
        allow_acquire.set()
        worker.join(5.)


def test_actual_completed_copy_can_commit_after_contention_when_owner_unchanged(monkeypatch):
    final_lock_case(monkeypatch, 'unchanged')


@pytest.mark.parametrize('mutation', ['epoch', 'sdk_out_and_back', 'source_expiry', 'lease_expiry'])
def test_completed_face_copy_cannot_cross_final_lock_revocation(monkeypatch, mutation):
    final_lock_case(monkeypatch, mutation)
