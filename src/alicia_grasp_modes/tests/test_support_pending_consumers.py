"""Real consumer boundaries for a support-pending false/empty publication.

No ROS master or robot is used. Existing close-range occlusion may retain an
already frozen plan; these tests do not mislabel that as new authorization.
"""
from copy import deepcopy
import importlib.util
import json
from pathlib import Path
import threading
from types import SimpleNamespace as NS
from unittest.mock import Mock

import numpy as np
import pytest
from std_msgs.msg import Bool, String

from test_locked_rgb_hold import locked, custom_frame, latest_status, module
from test_support_measurement_hold import support_gap_depth
from test_router_contract import router_fixture, Publisher, ModePolicy
from alicia_flexible_grasp.vision.rgbd_snapshot import SynchronizedRgbdBuffer
from alicia_flexible_grasp.vision.target_observation import TargetTrackIdentity
from alicia_flexible_grasp.robot import rgb_acquisition_gateway as gateway
from alicia_flexible_grasp.robot.observation_path_guard import ObservationPathError


TESTS = Path(__file__).resolve().parents[2] / 'alicia_flexible_grasp_supervisor/tests'


def helper(name, filename):
    spec = importlib.util.spec_from_file_location(name, TESTS / filename)
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


remote = helper('support_consumer_remote_fixture', 'test_remote_grasp6d_node.py')
task = helper('support_consumer_task_fixture', 'test_grasp_task_sequence.py')


@pytest.fixture
def pending(monkeypatch):
    node, clock, _, joint, _ = locked(monkeypatch)
    visible = deepcopy(node.pubs['object'].publish.call_args[0][0])
    source = custom_frame(node, clock, joint, depth_image=support_gap_depth())
    obj = node.pubs['object'].publish.call_args[0][0]
    mask = node.pubs['object_mask'].publish.call_args[0][0]
    status = latest_status(node)
    assert not obj.detected and not np.any(node.bridge.imgmsg_to_cv2(mask))
    assert not status['source_qualification']['metric_geometry_authority']
    assert not status['source_qualification']['motion_authorized']
    return NS(node=node, clock=clock, source=source, visible=visible,
              obj=obj, mask=mask, status=status)


def test_router_forwards_false_empty_and_cannot_start_a_cached_plan(pending):
    router, params = router_fixture()
    router.policy = ModePolicy(mode='unknown', strategy='observe_return',
        generation=pending.node.generation, stamp_ns=pending.node.selection_ns,
        accepting=True, state_known=True)
    params.update({'/camera/width': pending.mask.width, '/camera/height': pending.mask.height})
    namespace = router.on_frame.__func__.__globals__
    namespace['Bool'] = Bool
    namespace['rospy'].Time.now = lambda: module.rospy.Time.from_sec(pending.clock[0])
    for name in ('mask_pub', 'obj_pub', 'detected_pub', 'raw_pub', 'cam_pub', 'base_pub'):
        setattr(router, name, Publisher())
    router.start_proxy = Mock()
    router.policy.observe_plan('/grasp_6d/plan_enriched', True, 'previous-contact',
        pending.visible.header.stamp.to_nsec(), 'unknown_tabletop', 'unknown_tabletop')
    router.on_frame('unknown', pending.obj, pending.mask)
    assert router.obj_pub.messages[-1].detected is False
    assert not any(router.mask_pub.messages[-1].data)
    assert router.target_receipt == 0
    assert json.loads(router.status_pub.messages[-1].data)['state'] == 'waiting_for_target'
    result = router.start(NS(execute=True, plan_id='previous-contact'))
    assert result.success is False and result.message == 'TARGET_UNAVAILABLE'
    router.start_proxy.assert_not_called()
    assert not router.policy.reserved


def test_remote_invalidates_geometry_epoch_and_plan_on_pending(pending):
    node = remote.make_processing_node()
    node._initialize_streaming_state(source_clock=lambda: pending.clock[0], start_worker=False)
    node.streaming_enabled = True
    node._object_lock = threading.RLock()
    node.latest_object = pending.visible
    node.latest_object_time = pending.visible.header.stamp
    old_identity = node._current_stream_target_identity()
    old_ticket = NS(generation=node._stream_generation, target_epoch=node.target_instance_epoch)
    node.object_cb(pending.obj)
    assert node.latest_object.detected is False
    assert node._current_stream_target_identity().epoch == old_identity.epoch + 1
    assert node._stream_ticket_cancellation_code_locked(old_ticket) == 'TARGET_EPOCH_STALE'
    assert not node.geometry_pub.messages[-1].valid
    assert node.geometry_pub.messages[-1].failure_reason.startswith('TARGET_LOST:')
    assert not node.rich_plan_pub.messages[-1].valid
    assert not node.plan_pub.messages[-1].poses
    assert node.previous_object_axes_base is None


@pytest.mark.parametrize('require_mask', [False, True])
def test_same_stamp_rgbd_cannot_turn_pending_into_a_planning_sample(pending, require_mask):
    stamp = pending.obj.header.stamp.to_nsec()
    buffer = SynchronizedRgbdBuffer(source_clock_ns=lambda: stamp + 100_000_000)
    identity = TargetTrackIdentity.from_stream(0, 1)
    buffer.update_joints([0.] * 6)
    for key, message in zip(('color', 'depth'), pending.source):
        array = pending.node.bridge.imgmsg_to_cv2(message)
        getattr(buffer, 'update_' + key)(array, message.header.stamp, message.header.frame_id)
    buffer.update_mask(pending.node.bridge.imgmsg_to_cv2(pending.mask),
                       pending.mask.header.stamp, pending.mask.header.frame_id)
    buffer.update_object(pending.obj, pending.obj.header.stamp,
                         target_epoch=1, target_identity=identity)
    result = buffer.wait_for_exact_sample(stamp, identity, timeout_sec=0.,
        require_mask=require_mask, max_age_sec=.8, max_inference_latency_sec=.8)
    assert result is None


def test_rgb_gateway_drops_ready_interest_and_refuses_pending(pending):
    node = gateway.RgbAcquisitionGateway()
    node._rgb_lock = threading.RLock()
    node._rgb_pairs = {}
    node._rgb_reference_ack = None
    node._rgb_ready_status = dict(target_locked=False,
        observation_interest=dict(observation_interest_ready=True, source_stamp_ns=1))
    node._rgb_acquisition_status_cb(String(data=json.dumps(pending.status)))
    assert node._rgb_ready_status is None
    with pytest.raises(ObservationPathError, match='FRESH_READY_RGB_D_SOURCE_REQUIRED'):
        node._rgb_capture('a' * 64)


@pytest.mark.parametrize('active,occlusion,revoked,retained', [
    (True, True, False, True), (True, False, False, False),
    (False, True, False, False), (True, True, True, False)])
def test_bound_occlusion_only_retains_existing_authority(pending, active, occlusion, revoked, retained):
    factory = task.GraspTaskSequenceTest()
    node = task.grasp_task_node.GraspTaskNode.__new__(task.grasp_task_node.GraspTaskNode)
    plan = factory._rich_plan(stamp_sec=pending.clock[0] - .2)
    node.latest_grasp6d_plan = plan
    node.latest_obj = pending.visible
    node.latest_obj_time = pending.visible.header.stamp
    node.latest_visual_obj = pending.visible
    node.latest_visual_obj_time = pending.visible.header.stamp
    node.active = active
    node._freeze_execution_plan(plan, allow_target_occlusion=occlusion)
    node._execution_authority_revoked = revoked
    frozen = deepcopy(node._bound_execution_plan)
    frozen_digest = node._bound_execution_plan_digest
    node.obj_cb(pending.obj)
    assert node.latest_visual_obj is None
    assert node._bound_execution_plan == frozen
    assert node._bound_execution_plan_digest == frozen_digest
    assert node._bound_target_occlusion_allowed is occlusion
    if retained:
        assert node.latest_obj is pending.visible
        assert not node._execution_authority_revoked
    else:
        assert node.latest_obj is None
        assert node._execution_authority_revoked
