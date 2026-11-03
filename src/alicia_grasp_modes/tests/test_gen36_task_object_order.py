"""Task-side object delivery order through the original one-view executor.

The GEN35 fixture still owns producer, remote, fusion, acceptance, stationary
feedback and execution. Only task-bound topic deliveries are perturbed here;
all actuator/SDK/FK boundaries remain explicit offline simulators.
"""
from copy import deepcopy
import json

import pytest

import test_gen35_minimal_chain_replay as chain
from test_unknown_pending_identity import publication


@pytest.fixture(autouse=True)
def forbid_external_transport(monkeypatch):
    # This uses BaseException so broad production RPC handlers cannot turn an
    # accidental network call into a passing rejection-path assertion.
    chain.forbid_external_transport.__wrapped__(monkeypatch)
    import rospy
    def forbidden(*args, **kwargs):
        raise chain.UnsimulatedExternalCall('unmodeled ROS endpoint attempted: '+repr(args))
    for name in ('init_node', 'Publisher', 'Subscriber', 'Service'):
        monkeypatch.setattr(rospy, name, forbidden)


def replay_task_object_order(monkeypatch, tmp_path, *, fault='old_positive',
                             order=('status', 'ack', 'object', 'mask'), perturb=False,
                             validity=3.):
    original_arrived = chain.arrived_same_scene
    injected = []
    def arrived(*args, **kwargs):
        c = original_arrived(*args, **kwargs)
        c.params['/grasp'] = dict(c.params.get('/grasp', {}),
                                 target_observation_validity_sec=validity)
        rows = []
        feed = c.feed
        def remember_feed():
            result = feed()
            rows.append(publication(c.producer))
            return result
        c.feed = remember_feed
        obj_cb = c.task.obj_cb
        def ordered_object(msg):
            obj_cb(msg)
            request = c.task._observation_measurement_request or {}
            if request.get('state') != 'requalifying' or injected:
                return
            assert len(rows) == 3
            assert msg == rows[-1]['object']
            before = deepcopy(c.task.latest_obj)
            stale = deepcopy(rows[-2]['object'])
            assert stale.detected and stale.header.stamp < msg.header.stamp
            assert stale.pose_base.header.stamp == stale.header.stamp
            # This is a genuine earlier producer publication, not a fabricated
            # timestamp. At 3s it is alive; at the runtime 1.5s policy it has
            # expired in transport. The retained arrival remains fresh in both.
            assert c.task._observed_target_matches_plan(c.origin, stale, c.cfg)
            now_ns = chain.producer_module.rospy.Time.now().to_nsec()
            assert 1.5 < (now_ns-stale.header.stamp.to_nsec())*1e-9 < 3.
            assert 0 <= (now_ns-msg.header.stamp.to_nsec())*1e-9 < .8
            source_receipt = getattr(c.task, '_latest_obj_received_monotonic', None)
            change = deepcopy(stale if fault == 'old_positive' else msg)
            if fault in ('current_negative', 'newer_target_change'):
                newer = chain._stamp(msg.header.stamp.to_nsec()+10_000_000)
                change.header.stamp = change.pose_base.header.stamp = change.pose_camera.header.stamp = newer
            if fault == 'current_negative':
                change.detected = False
            elif fault in ('same_source_conflict', 'newer_target_change'):
                change.pose_base.pose.position.x += .20 if fault != 'same_source_conflict' else .01
            elif fault == 'revoked':
                c.task._clear_grasp6d_authority()
                change = stale
            elif fault == 'expired':
                c.advance((request['capture_deadline_ns']-now_ns)*1e-9+.01)
                change = stale
            elif fault == 'unassociated_old_positive':
                change = deepcopy(stale)
                change.pose_base.pose.position.x += .20
            elif fault == 'owner_changed':
                c.params['/gui/joint_direct_mode'] = True
                change = stale
            elif fault == 'unbound_old_source':
                change = deepcopy(stale)
                # A previously captured producer frame whose source is outside
                # this view's lower bound cannot claim this lease's protection.
                request['view_source_stamp_ns'] = stale.header.stamp.to_nsec()+1
            obj_cb(change)
            injected.append(dict(kind=fault, ack_source_ns=msg.header.stamp.to_nsec(),
                delayed_source_ns=stale.header.stamp.to_nsec(),
                age_sec=(now_ns-stale.header.stamp.to_nsec())*1e-9,
                validity_sec=validity,
                receipt_age_preserved=(getattr(c.task, '_latest_obj_received_monotonic', None) == source_receipt),
                before_source_ns=before.header.stamp.to_nsec(),
                after_source_ns=(c.task.latest_obj.header.stamp.to_nsec()
                                 if c.task.latest_obj is not None else None)))
            if perturb and fault == 'old_positive':
                obj_cb(deepcopy(stale))
                obj_cb(deepcopy(msg))
        c.task.obj_cb = ordered_object
        return c
    monkeypatch.setattr(chain, 'arrived_same_scene', arrived)
    c = chain.unified_replay(monkeypatch, tmp_path, order, perturb=perturb)
    c.object_order_injections = injected
    (tmp_path/'task_object_order.json').write_text(json.dumps(dict(
        injected=injected, request_states=[r['state'] for r in c.requests],
        states=[str(s) for s in c.states], device_calls=c.device_calls,
        frozen_count=len(c.task._frozen_contact_pub.messages),
        latest_obj_source_ns=(c.task.latest_obj.header.stamp.to_nsec()
                              if c.task.latest_obj is not None else None)),
        default=str, indent=2))
    return c


@pytest.mark.parametrize('validity', [3., 1.5])
def test_delayed_previous_positive_cannot_erase_completed_arrival_source(monkeypatch, tmp_path, validity):
    c = replay_task_object_order(monkeypatch, tmp_path, validity=validity)
    assert_one_contact(c)


def assert_one_contact(c):
    assert len(c.object_order_injections) == 1
    injection = c.object_order_injections[0]
    assert injection['before_source_ns'] == injection['ack_source_ns']
    assert injection['after_source_ns'] == injection['ack_source_ns'], injection
    assert injection['receipt_age_preserved'], injection
    assert c.states[-1].state == 'HOLDING', c.states[-1]
    assert 'GRASP_HOLD_UNVERIFIED' in c.states[-1].message
    assert sum(x[0] == 'motion' and x[1] == c.origin.plan_id for x in c.device_calls) == 1
    assert sum(x[0] == 'motion' and x[1] == c.contact.plan_id for x in c.device_calls) == 3
    assert len(c.task._frozen_contact_pub.messages) == 1
    assert len(c.phases) == 1
    assert [r['state'] for r in c.requests] == ['executing', 'requalifying', 'qualified']


@pytest.mark.parametrize('order', [('mask', 'object', 'ack', 'status'),
                                 ('ack', 'status', 'mask', 'object')])
def test_delayed_stale_object_with_reordered_duplicate_messages_reaches_one_close(monkeypatch, tmp_path, order):
    c = replay_task_object_order(monkeypatch, tmp_path, validity=1.5, order=order, perturb=True)
    assert_one_contact(c)
    assert sum(x[0] == 'gripper' and x[2] < .05 for x in c.device_calls) == 1
    assert c.injected_events == ['qualified_then_late_pending_and_duplicate_ack',
        'late_ancestor_clock_preview', 'duplicate_preview_audit', 'duplicate_start_rejected']


@pytest.mark.parametrize('fault', ['current_negative', 'newer_target_change',
    'same_source_conflict', 'revoked', 'expired', 'owner_changed',
    'unassociated_old_positive', 'unbound_old_source'])
def test_real_invalidation_during_arrival_still_prevents_contact_and_close(monkeypatch, tmp_path, fault):
    c = replay_task_object_order(monkeypatch, tmp_path, fault=fault, validity=1.5)
    assert c.states[-1].state == 'FAILED', c.states[-1]
    assert sum(x[0] == 'motion' for x in c.device_calls) == 1
    assert not any(x[0] == 'gripper' and x[2] < .05 for x in c.device_calls)
    assert not c.task._frozen_contact_pub.messages
    assert 'qualified' not in [r['state'] for r in c.requests]
