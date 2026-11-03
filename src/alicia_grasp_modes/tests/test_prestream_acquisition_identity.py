"""Real acquisition precedes inference startup; evidence is not replayed geometry."""
from copy import deepcopy
import itertools
import threading

import pytest

from test_locked_rgb_hold import locked, custom_frame
from test_support_measurement_hold import support_gap_depth
from test_unknown_pending_identity import publication, deliver, module, base, Publisher


def inactive_consumer(producer, clock, params):
    n = module.ModeAwareRemoteGrasp6DNode.__new__(module.ModeAwareRemoteGrasp6DNode)
    n._initialize_mode_boundary()
    n._initialize_streaming_state(start_worker=False, source_clock=lambda: clock[0])
    n._ros_source_clock_ns = lambda: round(clock[0]*1e9)
    n._object_lock = threading.RLock()
    n._geometry_state_lock = threading.RLock()
    n.enabled = True
    n.robot_execution_active = False
    n.frames = base.SynchronizedRgbdBuffer(source_clock_ns=n._ros_source_clock_ns)
    n.planning_snapshot_max_age_sec = .35
    n.planning_snapshot_max_inference_latency_sec = 3.5
    n.latest_joint_state = n.latest_object = n.latest_object_time = None
    n.bridge = producer.bridge
    for key in ('plan_pub', 'rich_plan_pub', 'preview_plan_pub', 'preview_rich_plan_pub',
                'status_pub', 'pipeline_metrics_pub', 'geometry_pub'):
        setattr(n, key, Publisher())
    assert n.reset_mode_cb(None).success
    assert not n.streaming_enabled
    return n


@pytest.mark.parametrize('order', list(itertools.permutations(('object', 'mask', 'status', 'start'))))
def test_acquired_topics_across_startup_join_new_identity_without_replaying_old_geometry(monkeypatch, order):
    p, clock, feed, joint, params = locked(monkeypatch, initial_station=True)
    n = inactive_consumer(p, clock, params)
    row = publication(p)
    before = n._current_stream_target_identity()
    for action in order:
        if action == 'start':
            assert n.start_streaming()
            assert n._current_stream_target_identity() != before
        else:
            deliver(n, row, (action,))
        if not n.streaming_enabled:
            assert n._unknown_identity_template is None
            assert n._unknown_identity_reference is None
    assert n._unknown_identity_template['provenance'] == row['status']['initial_station_identity_template']
    assert n._unknown_identity_template['identity'] == n._current_stream_target_identity()
    assert n._unknown_identity_reference['source_ns'] == row['status']['source_stamp_ns']
    assert n._unknown_identity_reference['deadline_ns'] == row['status']['source_stamp_ns']+800_000_000
    identity = n._current_stream_target_identity()
    # The first real typed gap after start keeps identity; original callbacks
    # still withhold ObjectPose/geometry and cancel obsolete planning tickets.
    custom_frame(p, clock, joint, depth_image=support_gap_depth())
    deliver(n, publication(p))
    assert n._unknown_identity_pending is not None
    assert n._current_stream_target_identity() == identity
    assert not n.latest_object.detected and not n.geometry_pub.messages[-1].valid
    clock[0] += .15
    feed(target_depth=.35)
    fresh = publication(p)
    deliver(n, fresh)
    assert n._unknown_identity_pending is None and n.latest_object.detected
    assert n._current_stream_target_identity() == identity
    assert n._unknown_identity_reference['source_ns'] == fresh['status']['source_stamp_ns']
    assert not n.rich_plan_pub.messages[-1].valid


@pytest.mark.parametrize('change', ['expired', 'missing_object', 'missing_mask', 'mask_hash',
                                   'selection', 'ordinary_ready', 'wrong_anchor'])
def test_startup_never_manufactures_acquisition_or_renews_its_source_age(monkeypatch, change):
    p, clock, _, _, params = locked(monkeypatch, initial_station=True)
    n = inactive_consumer(p, clock, params)
    row = publication(p)
    if change == 'mask_hash': row['status']['initial_station_identity_template']['refined_mask_sha256'] = 'f'*64
    elif change == 'ordinary_ready': row['status']['detail'] = 'tracked'
    elif change == 'wrong_anchor': row['status']['anchor_source_stamp_ns'] += 1
    order = tuple(k for k in ('object', 'mask', 'status')
                  if k != ('object' if change == 'missing_object' else 'mask' if change == 'missing_mask' else ''))
    deliver(n, row, order)
    if change == 'expired': clock[0] += .81
    elif change == 'selection': params[module.SELECTION_PARAM]['generation'] += 1
    started = n.start_streaming()
    assert started is (change != 'selection')
    assert n._unknown_identity_reference is None and n._unknown_identity_template is None
    assert not n.rich_plan_pub.messages[-1].valid


def test_prestream_true_and_false_keep_original_base_behavior(monkeypatch):
    p, clock, _, _, params = locked(monkeypatch, initial_station=True)
    n = inactive_consumer(p, clock, params)
    row = publication(p)
    deliver(n, row)
    assert n.latest_object.detected and n._unknown_identity_reference is None
    absent = deepcopy(row['object']); absent.detected = False
    n.object_cb(absent)
    assert not n.latest_object.detected and n._unknown_identity_pending is None
    assert n.start_streaming()
    assert n._unknown_identity_reference is None and n._unknown_identity_template is None


def test_mode_reset_discards_all_old_acquisition_evidence(monkeypatch):
    p, clock, _, _, params = locked(monkeypatch, initial_station=True)
    n = inactive_consumer(p, clock, params)
    deliver(n, publication(p))
    assert n._unknown_identity_objects and n._unknown_identity_masks
    params[module.SELECTION_PARAM]['generation'] += 1
    params[module.SELECTION_PARAM]['stamp_ns'] = str(round(clock[0]*1e9))
    assert n.reset_mode_cb(None).success
    assert not n._unknown_identity_objects and not n._unknown_identity_masks and not n._unknown_identity_status
    assert n.start_streaming()
    assert n._unknown_identity_reference is None and n._unknown_identity_template is None


def test_recorded_gen19_status_to_stream_delay_keeps_original_source_and_deadline(monkeypatch):
    p, clock, _, _, params = locked(monkeypatch, initial_station=True)
    n = inactive_consumer(p, clock, params)
    row = publication(p)
    deliver(n, row)
    stamp = row['status']['source_stamp_ns']
    # Exact sealed status-receive to inference-start interval, 165.811061 ms.
    # Synthetic producer pixels/clock remain explicitly synthetic.
    clock[0] += (1791527943765328168 - 1791527943599517107) * 1e-9
    assert n.start_streaming()
    assert n._unknown_identity_reference['source_ns'] == stamp
    assert n._unknown_identity_reference['deadline_ns'] == stamp + 800_000_000
    assert n._unknown_identity_template['provenance']['source_stamp_ns'] == stamp
    assert not n.rich_plan_pub.messages[-1].valid
