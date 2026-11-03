"""Near-contact loss must not cancel an admitted fixed-target correction."""
from copy import deepcopy
import time
from unittest import mock

import pytest
import rospy

from test_pregrasp_gateway import real_context, make_gateway, NS, GraspStages
from alicia_flexible_grasp.robot.observation_path_guard import (
    CommittedObservationContexts, FrozenObservationScene)
from alicia_flexible_grasp.grasp.rich_plan_integrity import compute_plan_id


def admitted(monkeypatch, strategy="direct"):
    node, old, params = real_context(monkeypatch)
    registry = CommittedObservationContexts()
    registry.set_active(True)
    node._observation_contexts = registry
    params['/grasp_mode/selection'] = dict(mode='unknown', strategy=strategy, generation=3,
                                         stamp_ns='8000000000')
    plan = old['plan']
    plan.model_choice = 'unknown_tabletop'
    plan.plan_id = compute_plan_id(plan)
    context = node._pregrasp_context(plan)
    msg = deepcopy(plan)
    msg.valid = False
    msg.diagnostic = 'TARGET_LOST: object occluded'
    msg.header.stamp = rospy.Time.from_sec(9.9)
    msg.target_track_id = ''
    monkeypatch.setattr(rospy, 'logwarn_throttle', mock.Mock())
    monkeypatch.setattr(rospy, 'logwarn', mock.Mock())
    return node, context, msg, params


@pytest.mark.parametrize("strategy", ["direct", "observe_return"])
def test_loss_during_compensation_preserves_same_frozen_geometry_and_authority(monkeypatch, strategy):
    node, context, msg, _ = admitted(monkeypatch, strategy)
    before = context['scene'].digest
    for _ in range(3):
        node._committed_observation_plan_cb(msg)
        assert node._pregrasp_still_authorized(context)
    assert node._observation_contexts.revocation == context['revocation']
    assert not node._observation_contexts.blocked
    assert context['scene'].digest == before
    assert node._compensation_visual_context is context


@pytest.mark.parametrize('fault', [
    'no_episode', 'manual', 'stage', 'epoch', 'revoked', 'config', 'deadline',
    'actuation', 'disconnect', 'selection', 'two_stage', 'nonvisual',
    'different_track', 'valid_different_track', 'future', 'old', 'frame',
])
def test_loss_cannot_override_other_reasons_to_revoke(monkeypatch, fault):
    node, context, msg, params = admitted(monkeypatch)
    if fault == 'no_episode': node._compensation_visual_context = None
    if fault == 'manual': node._manual_control_active = lambda: True
    if fault == 'stage': node._pregrasp_task_stage = GraspStages.SUCCESS
    if fault == 'epoch': node._control_reference_epoch_ns += 1
    if fault == 'revoked': node._observation_contexts.clear()
    if fault == 'config': params['/grasp']['new_setting'] = True
    if fault == 'deadline': context['deadline'] = time.monotonic()-1
    if fault == 'actuation': node._fresh_actuation_status = lambda: 'UNCONFIRMED:ENCODER_RESPONSE_TIMEOUT'
    if fault == 'disconnect': node._require_live_observation_task.side_effect = ValueError('disconnected')
    if fault == 'selection': params['/grasp_mode/selection']['generation'] += 1
    if fault == 'two_stage':
        params['/grasp_mode/selection']['strategy'] = 'two_stage'
        context['selection'] = deepcopy(params['/grasp_mode/selection'])
    if fault == 'nonvisual': msg.diagnostic = 'INVALID_GEOMETRY: malformed'
    if fault == 'different_track': msg.target_track_id = 'replacement'
    if fault == 'valid_different_track':
        msg.valid = True
        msg.target_track_id = 'replacement'
    if fault == 'future': msg.header.stamp = rospy.Time.from_sec(11.)
    if fault == 'old': msg.header.stamp = rospy.Time.from_sec(8.)
    if fault == 'frame': msg.header.frame_id = 'camera_link'
    previous = node._observation_contexts.revocation
    node._committed_observation_plan_cb(msg)
    assert node._observation_contexts.revocation > previous
    assert node._observation_contexts.blocked
    assert not node._pregrasp_still_authorized(context)


def test_completed_compensation_clears_visual_exception(monkeypatch):
    node, _, plan, _, _, _ = make_gateway(monkeypatch, day='20260920_wrist')
    import threading
    node._observation_context_lock = threading.RLock()
    original = node._pregrasp_context
    def capture(value):
        context = original(value)
        node._compensation_visual_context = context
        return context
    node._pregrasp_context = capture
    result = node.handle_compensate_pregrasp(NS(plan=plan, execute=True))
    assert result.success, result.message
    assert node._compensation_visual_context is None
