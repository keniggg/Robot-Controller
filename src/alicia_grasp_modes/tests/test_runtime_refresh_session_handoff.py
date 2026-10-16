"""A real session callback cancels real runtime staging, without ROS or motion."""
import json
import threading
import types

from test_remote_modes import configured, module, base
from test_observe_return_remote import setup_observe_return, deficit, prefix_session
from test_observe_return_session_handoff_concurrency import ObservedLock
from test_remote_grasp6d_node import make_processing_node


def test_parameter_refresh_cannot_undo_real_session_pause(configured, monkeypatch):
    node, params, clock = setup_observe_return(configured)
    assert node._observe_return_fallback(types.SimpleNamespace(remote_diagnostics=deficit()))
    session = prefix_session(node)
    clock[0] = 15_000_000_000
    # Complete the ordinary constructor-free fixture with existing runtime
    # configuration defaults, retaining every actual session/stream lock.
    defaults = make_processing_node()
    for name, value in defaults.__dict__.items():
        if name != '_refresh_runtime_params':
            node.__dict__.setdefault(name, value)
    node.rate_hz = 1.5
    node.target_instance_association_threshold_m = .08
    node.target_absolute_sanity_distance_m = .15
    node.moveit_top_n = 5
    node.candidate_frame_convention = 'opencv_optical'
    node.tabletop_geometry_enabled = True
    node.tabletop_geometry_config = base.TabletopGeometryConfig()
    node.hybrid_merge_config = base.MergeConfig()
    node.candidate_max_target_distance_m = .04
    node.candidate_min_relative_z_m = -.015
    node.candidate_max_relative_z_m = .08
    identity = node._current_stream_target_identity()
    surface = node._observe_return_saved[0]
    ticket = types.SimpleNamespace(generation=node._stream_generation,
        target_epoch=node.target_instance_epoch)
    events = {}
    node._mode_work_lock = ObservedLock(node._mode_work_lock, events, 'work')
    blocked, release, cancelled = threading.Event(), threading.Event(), threading.Event()
    errors = []
    original_get = module.rospy.get_param

    def get_param(name, default=None):
        if threading.current_thread().name == 'refresh-worker' and name == '/grasp_6d/remote/max_candidates':
            blocked.set()
            if not release.wait(2.):
                raise RuntimeError('runtime staging not released')
        return original_get(name, default)
    monkeypatch.setattr(module.rospy, 'get_param', get_param)
    def work():
        try:
            with node._mode_work_lock:
                node._refresh_runtime_params()
                node._require_stream_ticket_current(ticket)
                errors.append('obsolete worker entered next planning stage')
        except base.StreamResultCancelled as exc:
            errors.append(exc.code)
            cancelled.set()
        except Exception as exc:
            errors.append(repr(exc))
    worker = threading.Thread(target=work, name='refresh-worker')
    receiver = threading.Thread(target=lambda: node._observe_return_session_cb(
        types.SimpleNamespace(data=json.dumps(session))), name='session-receiver')
    try:
        worker.start()
        assert blocked.wait(2.), errors
        receiver.start()
        # Poll only until the callback reaches its actual work-drain barrier;
        # the worker remains blocked in the real get_param staging call.
        import time
        end = time.monotonic() + 2.
        while ('session-receiver', 'work', 'waiting') not in events and time.monotonic() < end:
            threading.Event().wait(.001)
        assert events[('session-receiver', 'work', 'waiting')].is_set()
        assert node._observe_return_paused
        release.set()
        worker.join(2.); receiver.join(2.)
        assert not worker.is_alive() and not receiver.is_alive(), errors
        assert cancelled.is_set(), errors
        assert errors == ['OBSERVE_RETURN_PHASE_PAUSED']
        ack = json.loads(node._observe_return_ack_pub.messages[-1].data)
        assert ack['accepted'], ack
        assert node._observe_return_paused
        assert node._current_stream_target_identity() == identity
        assert node._observe_return_saved[0] is surface
        assert not node._poll_stream_snapshot()
        assert not node.submit_stream_snapshot(object())
    finally:
        release.set()
        worker.join(2.)
        if receiver.ident is not None:
            receiver.join(2.)
