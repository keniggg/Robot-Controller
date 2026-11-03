"""Real producer receipts through the remote callbacks, without ROS/hardware."""
from copy import deepcopy
import itertools
import json
import threading
from types import SimpleNamespace as NS

import pytest
from std_msgs.msg import String

from test_locked_rgb_hold import locked, custom_frame, latest_status
from test_support_measurement_hold import support_gap_depth
from test_remote_modes import module, base, Publisher


def consumer_for(producer, clock, params):
    """Also used by the independent recorded-message replay."""
    node = module.ModeAwareRemoteGrasp6DNode.__new__(module.ModeAwareRemoteGrasp6DNode)
    node._initialize_mode_boundary()
    node._initialize_streaming_state(start_worker=False, source_clock=lambda: clock[0])
    node._ros_source_clock_ns = lambda: round(clock[0] * 1e9)
    node._object_lock = threading.RLock()
    node._geometry_state_lock = threading.RLock()
    node.enabled = True
    node.robot_execution_active = False
    node.frames = base.SynchronizedRgbdBuffer(source_clock_ns=node._ros_source_clock_ns)
    node.planning_snapshot_max_age_sec = .35
    node.planning_snapshot_max_inference_latency_sec = 3.5
    node.latest_joint_state = None
    node.latest_object = node.latest_object_time = None
    node.bridge = producer.bridge
    for key in ('plan_pub', 'rich_plan_pub', 'preview_plan_pub', 'preview_rich_plan_pub',
                'status_pub', 'pipeline_metrics_pub', 'geometry_pub'):
        setattr(node, key, Publisher())
    assert node.reset_mode_cb(None).success
    assert node.start_streaming()
    return node


def publication(producer):
    return dict(object=deepcopy(producer.pubs['object'].publish.call_args[0][0]),
                mask=deepcopy(producer.pubs['object_mask'].publish.call_args[0][0]),
                status=deepcopy(latest_status(producer)))


def deliver(node, row, order=('mask', 'object', 'status')):
    for key in order:
        if key == 'status':
            node._silhouette_source_status_cb(String(data=json.dumps(row[key])))
        else:
            getattr(node, key + '_cb')(deepcopy(row[key]))


@pytest.fixture
def scene(monkeypatch):
    producer, clock, feed, joint, params = locked(monkeypatch)
    node = consumer_for(producer, clock, params)
    ready = publication(producer)
    deliver(node, ready)
    assert node._unknown_identity_reference is not None
    identity = node._current_stream_target_identity()
    custom_frame(producer, clock, joint, depth_image=support_gap_depth())
    pending = publication(producer)
    assert pending['status']['source_qualification']['kind'] == 'support_measurement_gap'
    return NS(node=node, producer=producer, clock=clock, feed=feed, joint=joint, params=params,
              ready=ready, pending=pending, identity=identity)


def recovery(scene):
    scene.clock[0] += .2
    scene.feed(target_depth=.35)
    row = publication(scene.producer)
    assert row['object'].detected
    assert row['status']['source_qualification']['state'] == 'recovered_unique_measured_frame'
    return row


@pytest.mark.parametrize('order', list(itertools.permutations(('mask', 'object', 'status'))))
def test_support_gap_preserves_only_identity_and_recovers_exact_new_measurement(scene, order):
    n = scene.node
    deadline = n._near_field_phase_deadline_sec
    deliver(n, scene.pending, order)
    assert n._current_stream_target_identity() == scene.identity
    assert n._unknown_identity_pending is not None
    assert n.latest_object.detected is False
    assert not n.geometry_pub.messages[-1].valid
    assert not n.rich_plan_pub.messages[-1].valid
    assert n._poll_stream_snapshot() is False
    assert n._near_field_phase_deadline_sec == deadline
    deliver(n, recovery(scene), order)
    assert n._unknown_identity_pending is None
    assert n.latest_object.detected
    assert n._current_stream_target_identity() == scene.identity
    assert n._near_field_phase_deadline_sec == deadline
    # Recovery reinstates a measured object, never an executable plan.
    assert not n.rich_plan_pub.messages[-1].valid


def test_false_before_status_immediately_quarantines_and_never_authorizes(scene):
    n = scene.node
    n.object_cb(scene.pending['object'])
    assert n._unknown_identity_pending['unpaired']
    assert not n.latest_object.detected and not n.geometry_pub.messages[-1].valid
    snapshot = NS(stamp_ns=scene.pending['object'].header.stamp.to_nsec()+1)
    assert n.submit_stream_snapshot(snapshot) is False
    assert n._poll_stream_snapshot() is False
    deliver(n, scene.pending, ('mask', 'status'))
    assert not n._unknown_identity_pending['unpaired']
    assert n._current_stream_target_identity() == scene.identity


def test_original_deadline_is_not_renewed_by_repeated_gap_or_no_frame(scene):
    n = scene.node
    deadline = n._unknown_identity_reference['deadline_ns']
    deliver(n, scene.pending)
    scene.clock[0] += .2
    custom_frame(scene.producer, scene.clock, scene.joint, depth_image=support_gap_depth())
    deliver(n, publication(scene.producer))
    assert n._unknown_identity_pending['deadline_ns'] == deadline
    scene.clock[0] = deadline/1e9 + .000001
    n._unknown_identity_poll()
    assert n._unknown_identity_pending is None
    assert n._current_stream_target_identity().epoch == scene.identity.epoch+1
    assert not n.geometry_pub.messages[-1].valid
    assert n.geometry_pub.messages[-1].failure_reason.startswith('TARGET_LOST:')


@pytest.mark.parametrize('field', ['kind', 'reference', 'source_id', 'anchor', 'selection', 'authority', 'source'])
def test_invalid_typed_receipt_never_retains_identity(scene, field):
    row = deepcopy(scene.pending)
    q = row['status']['source_qualification']
    if field == 'kind': q['kind'] = 'any_false_hold'
    elif field == 'reference': q['reference_source_stamp_ns'] -= 1
    elif field == 'source_id': q['source_id'] = 'invalid'
    elif field == 'anchor': row['status']['anchor_position_base'][0] += .01
    elif field == 'selection': row['status']['selection_stamp_ns'] += 1
    elif field == 'authority': q['motion_authorized'] = True
    else: row['status']['depth_stamp_ns'] += 1
    deliver(scene.node, row)
    if field == 'selection':
        # A status belonging to another selection never completes this join.
        scene.clock[0] = scene.node._unknown_identity_reference['deadline_ns']/1e9+.001
        scene.node._unknown_identity_poll()
    assert scene.node._current_stream_target_identity() != scene.identity
    assert not scene.node.latest_object.detected


@pytest.mark.parametrize('change', ['ordinary_ready', 'uncommitted', 'source_id', 'anchor', 'wrong_mask', 'empty_mask'])
def test_recovery_requires_exact_committed_real_source(scene, change):
    n = scene.node
    deliver(n, scene.pending)
    row = recovery(scene)
    if change == 'ordinary_ready': row['status'].pop('source_qualification')
    elif change == 'uncommitted': row['status']['source_qualification']['state'] = 'qualified_unique_measured_recovery'
    elif change == 'source_id': row['status']['source_qualification']['source_id'] = 'f'*64
    elif change == 'anchor': row['status']['anchor_extent_m'][0] += .001
    elif change == 'wrong_mask': row['mask'].header.stamp = module.rospy.Time(1)
    else: row['mask'].data = bytes(len(row['mask'].data))
    deliver(n, row)
    assert not n.latest_object.detected
    if change == 'wrong_mask':
        assert n._unknown_identity_pending is not None
    else:
        assert n._current_stream_target_identity() != scene.identity


@pytest.mark.parametrize('reservation', ['session', 'prefix', 'authority', 'controller', 'active', 'publication_started'])
def test_scope_does_not_change_authorized_or_executing_false_behavior(scene, reservation):
    n = scene.node
    if reservation == 'session': n._observe_return_session = {'deadline_ns': 10**15}
    elif reservation == 'prefix': n._observe_return_prefix = object()
    elif reservation == 'authority': n._execution_authority_ticket = object()
    elif reservation == 'controller': n.execution_plan_controller = NS(has_execution=True, clear_execution=lambda:None)
    elif reservation == 'active': n.robot_execution_active = True
    else: n._unknown_identity_publication_started = True
    deliver(n, scene.pending)
    assert n._unknown_identity_pending is None
    assert n._current_stream_target_identity() != scene.identity


def test_old_queued_and_inflight_tickets_cannot_resurrect_after_recovery(scene):
    n = scene.node
    old_ns = scene.ready['object'].header.stamp.to_nsec()
    old_snapshot = NS(stamp_ns=old_ns)
    active = n.inference_coordinator.submit((old_snapshot, None), old_ns/1e9, scene.identity.epoch).ticket_to_start
    queued = n.inference_coordinator.submit((old_snapshot, None), old_ns/1e9, scene.identity.epoch)
    n._pending_request_id = queued.pending_request_id
    deliver(n, scene.pending)
    assert n.inference_coordinator.pending_count == 0
    with pytest.raises(base.StreamResultCancelled, match='TARGET_MEASUREMENT_PENDING'):
        n._prepare_and_predict(active)
    deliver(n, recovery(scene))
    with pytest.raises(base.StreamResultCancelled, match='TARGET_MEASUREMENT_PENDING'):
        n._accept_prediction(NS(ticket=active))
    assert not n.submit_stream_snapshot(old_snapshot)
    assert not n.rich_plan_pub.messages[-1].valid


def test_recovery_true_waits_for_status_and_mask(scene):
    n = scene.node
    deliver(n, scene.pending)
    row = recovery(scene)
    deliver(n, row, ('object', 'status'))
    assert n._unknown_identity_pending is not None and not n.latest_object.detected
    deliver(n, row, ('mask',))
    assert n.latest_object.detected and n._unknown_identity_pending is None


def test_generic_false_without_typed_receipt_expires_instead_of_holding_forever(scene):
    n = scene.node
    deliver(n, scene.pending, ('object',))
    scene.clock[0] += 1.
    n._unknown_identity_poll()
    assert n._current_stream_target_identity() != scene.identity
    assert n._unknown_identity_pending is None


@pytest.mark.parametrize('expire_in_callback', [False, True])
def test_expired_selection_cannot_be_revived_by_late_true_or_recovery(scene, expire_in_callback):
    n = scene.node
    deliver(n, scene.pending)
    row = recovery(scene)  # Producer completed this measurement on time.
    deadline = n._unknown_identity_pending['deadline_ns']
    scene.clock[0] = deadline/1e9 + .000001
    if not expire_in_callback:
        n._unknown_identity_poll()
    deliver(n, row)
    assert n._current_stream_target_identity().epoch == scene.identity.epoch + 1
    assert not n.latest_object.detected
    assert n._unknown_identity_retired_selection == n._mode_selection
    deliver(n, row, ('status', 'object', 'mask'))
    assert not n.latest_object.detected
    assert n._current_stream_target_identity().epoch == scene.identity.epoch + 1
    assert not n.submit_stream_snapshot(NS(stamp_ns=row['object'].header.stamp.to_nsec()))


@pytest.mark.parametrize('value', [None, float('nan'), .69])
def test_recovery_does_not_replace_original_support_quality_with_true_flag(scene, value):
    n = scene.node
    deliver(n, scene.pending)
    row = recovery(scene)
    audit = row['status']['source_qualification']['support_audit']
    audit['support_current_audit_inlier_ratio'] = value
    deliver(n, row)
    assert not n.latest_object.detected
    assert n._unknown_identity_retired_selection == n._mode_selection


@pytest.mark.parametrize('value', [.84, float('nan'), True])
def test_recovery_rejects_inconsistent_rgb_compatibility_claim(scene, value):
    n = scene.node
    deliver(n, scene.pending)
    row = recovery(scene)
    row['status']['source_qualification']['rgb_identity']['candidates'][0]['mask_iou'] = value
    deliver(n, row)
    assert not n.latest_object.detected
    assert n._unknown_identity_retired_selection == n._mode_selection


def test_recovery_expiring_during_original_object_acceptance_rolls_back(scene, monkeypatch):
    n = scene.node
    deliver(n, scene.pending)
    row = recovery(scene)
    deadline = n._unknown_identity_pending['deadline_ns']
    accept = base.RemoteGrasp6DNode.object_cb
    def slow_accept(self, obj):
        accept(self, obj)
        if obj.detected:
            scene.clock[0] = deadline/1e9+.000001
    monkeypatch.setattr(base.RemoteGrasp6DNode, 'object_cb', slow_accept)
    deliver(n, row)
    assert n._current_stream_target_identity() != scene.identity
    assert not n.latest_object.detected
    assert not n.rich_plan_pub.messages[-1].valid


def test_new_recovery_source_can_submit_but_mixed_old_sources_cannot(scene, monkeypatch):
    n = scene.node
    n._direct_near_field_submission_generation = n._stream_generation
    original_deadline = n._near_field_phase_deadline_sec
    deliver(n, scene.pending)
    assert n._direct_near_field_submission_generation is None
    row = recovery(scene)
    deliver(n, row)
    new_ns = row['object'].header.stamp.to_nsec()
    old_ns = scene.ready['object'].header.stamp.to_nsec()
    calls = []
    monkeypatch.setattr(base.RemoteGrasp6DNode, 'submit_stream_snapshot',
        lambda self, snapshot, **kwargs: calls.append(snapshot) or True)
    assert not n.submit_stream_snapshot(NS(stamp_ns=new_ns, sample_stamp_ns=(old_ns, new_ns)))
    fresh = NS(stamp_ns=new_ns, sample_stamp_ns=(new_ns,))
    assert n.submit_stream_snapshot(fresh)
    assert calls == [fresh]
    assert n._near_field_phase_deadline_sec == original_deadline


def test_false_during_first_preview_send_cannot_be_misclassified_as_unpublished(scene, monkeypatch):
    n = scene.node
    ticket = NS(generation=n._stream_generation, target_epoch=n.target_instance_epoch,
                snapshot_stamp_sec=scene.clock[0], payload=(NS(stamp_ns=round(scene.clock[0]*1e9)), None))
    plan = base.Grasp6DPlan()
    plan.valid = True
    def send_then_false(_message):
        n.object_cb(scene.pending['object'])
    monkeypatch.setattr(n.preview_plan_pub, 'publish', send_then_false)
    with pytest.raises(base.StreamResultCancelled, match='TARGET_EPOCH_STALE'):
        n._publish_preview_plan(plan, base.PoseArray(), ticket=ticket)
    assert n._unknown_identity_publication_started
    assert n._unknown_identity_pending is None
    assert n._current_stream_target_identity() != scene.identity
    assert not n.preview_rich_plan_pub.messages[-1].valid


def test_pending_callback_does_not_wait_for_inference_work_lock(scene):
    n = scene.node
    finished = threading.Event()
    errors = []
    def deliver_false():
        try:
            deliver(n, scene.pending)
        except Exception as exc:
            errors.append(exc)
        finally:
            finished.set()
    with n._mode_work_lock:
        thread = threading.Thread(target=deliver_false)
        thread.start()
        completed = finished.wait(1.)
    thread.join(1.)
    assert completed and not errors
    assert n._unknown_identity_pending is not None
