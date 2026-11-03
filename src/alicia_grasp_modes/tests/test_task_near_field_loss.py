"""Loss cancels only an observation's near-field evidence wait, never hardware."""
from copy import deepcopy
import json
from types import SimpleNamespace as NS

import pytest

from test_task_modes import configured, module, base


def status(selection, observed=4000000000):
    return dict(mode='unknown', generation=selection['generation'],
        selection_stamp_ns=int(selection['stamp_ns']), state='unavailable',
        target_locked=True, target_lost=True, anchor_source_stamp_ns=2500000000,
        first_loss=dict(generation=selection['generation'], selection_stamp_ns=int(selection['stamp_ns']),
            observed_stamp_ns=observed, anchor_source_stamp_ns=2500000000,
            reason='exact_rgbd_stream_stale', diagnostic_only=True, target_recovered=False))


def prepare(configured, monkeypatch):
    node, params = configured
    params[module.SELECTION_PARAM] = dict(params[module.SELECTION_PARAM], strategy='two_stage',
                                         generation=3, stamp_ns='2200000000')
    assert node.reset_mode_cb(None).success
    node._mode_bound_selection = deepcopy(node._mode_selection)
    now = [5.]
    monkeypatch.setattr(module.rospy.Time, 'now', staticmethod(lambda: module.rospy.Time.from_sec(now[0])))
    monkeypatch.setattr(module.rospy, 'is_shutdown', lambda: False)
    monkeypatch.setattr(module.rospy, 'logwarn', lambda *a: None)
    node.active = True
    node._near_field_phase_started_sec = 5.
    node._near_field_phase_deadline_sec = 95.
    plan = NS(model_choice='unknown_tabletop', diagnostic=base._FAR_FIELD_OBSERVATION_PLAN,
              plan_id='a'*24, header=NS(stamp=module.rospy.Time(3)))
    config = dict(near_field_replan_enabled=True, near_field_replan_required=True,
                  near_field_strategy='single_snapshot_direct', near_field_replan_timeout_sec=90.)
    streams, states = [], []
    node._set_near_field_preview_stream = lambda _cfg, value: streams.append(value) or True
    node.set_state = lambda *args, **kwargs: states.append(args)
    def forbidden(*a, **kw): pytest.fail('loss triggered contact/arm/gripper execution')
    node._freeze_execution_plan = forbidden
    node._simulate_grasp6d_plan_if_required = forbidden
    node._execute_clear_view_reacquisition = forbidden
    node._invoke_plan_bound_action = forbidden
    return node, params, plan, config, streams, states, now, forbidden


def deliver(node, message):
    node._unknown_support_status_cb(NS(data=json.dumps(message)))


@pytest.mark.parametrize('required', [True, False])
def test_bound_loss_before_wait_finishes_without_old_plan_fallback_or_motion(configured, monkeypatch, required):
    node, _, plan, config, streams, states, _, forbidden = prepare(configured, monkeypatch)
    config['near_field_replan_required'] = required
    deliver(node, status(node._mode_bound_selection))
    node._copy_near_field_preview_candidate = forbidden
    result = node._maybe_rebind_near_field_grasp6d_plan(config, {}, plan,
        move_pose=forbidden, strict_execute_pose=forbidden)
    assert result is None
    assert streams == [False]  # existing inference cancellation only
    assert states[-1][0] == base.GraspStages.FAILED
    assert states[-1][1].startswith('TARGET_LOST: exact_rgbd_stream_stale')


def test_loss_during_90_second_wait_exits_at_next_poll(configured, monkeypatch):
    node, _, plan, config, streams, states, now, forbidden = prepare(configured, monkeypatch)
    reads = []
    node._copy_near_field_preview_candidate = lambda *a: (
        reads.append(True) or base.PlanValidationResult(False, 'FINAL_REFINE_3D_INVALID', 'old Preview'), None)
    def sleep(_):
        now[0] += .05
        deliver(node, status(node._mode_bound_selection, module.rospy.Time.now().to_nsec()))
    monkeypatch.setattr(module.rospy, 'sleep', sleep)
    result = node._maybe_rebind_near_field_grasp6d_plan(config, {}, plan,
        move_pose=forbidden, strict_execute_pose=forbidden)
    assert result is None
    assert now[0] == pytest.approx(5.05) and reads == [True]
    assert streams == [True, False]
    assert 'TARGET_LOST:' in states[-1][1] and 'TIMEOUT' not in states[-1][1]


def test_loss_during_candidate_validation_is_rechecked_before_contact_freeze(configured, monkeypatch):
    node, _, plan, config, streams, states, _, forbidden = prepare(configured, monkeypatch)
    def copy_candidate(*a):
        deliver(node, status(node._mode_bound_selection))
        return base.PlanValidationResult(True), NS(plan_id='b'*24)
    node._copy_near_field_preview_candidate = copy_candidate
    result = node._maybe_rebind_near_field_grasp6d_plan(config, {}, plan,
        move_pose=forbidden, strict_execute_pose=forbidden)
    assert result is None and streams == [True, False]
    assert states[-1][1].startswith('TARGET_LOST:')


@pytest.mark.parametrize('mutation', ['generation', 'selection', 'first_generation', 'first_selection',
    'future', 'malformed_time', 'missing_first_loss', 'not_lost', 'not_locked', 'json_list', 'bad_json'])
def test_malformed_or_old_selection_cannot_revoke_current_wait(configured, monkeypatch, mutation):
    node, _, plan, _, _, _, _, _ = prepare(configured, monkeypatch)
    payload = status(node._mode_bound_selection)
    if mutation == 'generation': payload['generation'] -= 1
    elif mutation == 'selection': payload['selection_stamp_ns'] -= 1
    elif mutation == 'first_generation': payload['first_loss']['generation'] -= 1
    elif mutation == 'first_selection': payload['first_loss']['selection_stamp_ns'] -= 1
    elif mutation == 'future': payload['first_loss']['observed_stamp_ns'] = 5000000001
    elif mutation == 'malformed_time': payload['first_loss']['observed_stamp_ns'] = True
    elif mutation == 'missing_first_loss': del payload['first_loss']
    elif mutation == 'not_lost': payload['target_lost'] = False
    elif mutation == 'not_locked': payload['target_locked'] = False
    elif mutation == 'json_list': payload = []
    if mutation == 'bad_json': node._unknown_support_status_cb(NS(data='{'))
    else: deliver(node, payload)
    assert node._near_field_wait_failure(plan, {}) is None


def test_ready_or_missing_status_does_not_clear_same_selection_first_loss(configured, monkeypatch):
    node, _, plan, _, _, _, _, _ = prepare(configured, monkeypatch)
    payload = status(node._mode_bound_selection)
    deliver(node, payload)
    first = deepcopy(node._unknown_target_loss)
    payload.update(target_lost=False, state='ready')
    deliver(node, payload)
    assert node._unknown_target_loss == first
    assert node._near_field_wait_failure(plan, {}).code == 'TARGET_LOST'


@pytest.mark.parametrize('phase,mode,strategy', [
    ('CONTACT_EXECUTION_PLAN', 'unknown', 'two_stage'),
    ('CONTACT_EXECUTION_PLAN', 'unknown', 'direct'),
    ('FAR_FIELD_OBSERVATION_PLAN', 'carton', 'two_stage'),
    ('FAR_FIELD_OBSERVATION_PLAN', 'unknown', 'direct'),
])
def test_contact_occlusion_and_other_flows_do_not_gain_new_failure_gate(configured, monkeypatch, phase, mode, strategy):
    node, _, plan, _, _, _, _, _ = prepare(configured, monkeypatch)
    deliver(node, status(node._mode_bound_selection))
    plan.diagnostic = phase
    node._mode_bound_selection.update(mode=mode, strategy=strategy)
    assert node._near_field_wait_failure(plan, {}) is None


def test_explicit_new_selection_does_not_inherit_old_loss(configured, monkeypatch):
    node, params, plan, _, _, _, _, _ = prepare(configured, monkeypatch)
    old = status(node._mode_bound_selection)
    deliver(node, old)
    node.active = False
    node._mode_bound_selection = None
    params[module.SELECTION_PARAM] = dict(mode='unknown', strategy='two_stage', generation=4,
                                         stamp_ns='4500000000')
    assert node.reset_mode_cb(None).success
    node._mode_bound_selection = deepcopy(node._mode_selection)
    deliver(node, old)
    assert node._unknown_target_loss is None
    assert node._near_field_wait_failure(plan, {}) is None


def test_bound_contact_loss_finishes_final_reference_wait_without_twenty_seconds(configured, monkeypatch):
    node,_,plan,_,streams,states,now,forbidden=prepare(configured,monkeypatch)
    plan.diagnostic='CONTACT_EXECUTION_PLAN'
    deliver(node,status(node._mode_bound_selection))
    node._set_near_field_active=forbidden
    node._request_near_field_preview_stream=forbidden
    result=node._maybe_final_refine_grasp6d_plan(dict(
        final_visual_refine_enabled=True,final_visual_refine_timeout_sec=20.,
        near_field_strategy='single_snapshot_direct'),{},plan,
        move_pose=forbidden,strict_execute_pose=forbidden)
    assert result is None and now[0]==5.
    assert streams==[False]
    assert states[-1][0]==base.GraspStages.FAILED
    assert states[-1][1].startswith('TARGET_LOST: exact_rgbd_stream_stale')


def test_loss_during_post_arrival_reference_wait_exits_next_poll(configured,monkeypatch):
    node,_,plan,_,_,_,now,_=prepare(configured,monkeypatch)
    plan.diagnostic='CONTACT_EXECUTION_PLAN'
    def sleep(_):
        now[0]+=.05
        deliver(node,status(node._mode_bound_selection,module.rospy.Time.now().to_nsec()))
    monkeypatch.setattr(module.rospy,'sleep',sleep)
    monkeypatch.setattr(base.time,'monotonic',lambda:now[0])
    result=node._wait_for_final_refine_reference(plan,5_000_000_000,25.,.05)
    assert result is None and now[0]==pytest.approx(5.05)


def test_old_selection_loss_cannot_abort_a_new_final_reference_wait(configured,monkeypatch):
    node,_,plan,_,_,_,_,_=prepare(configured,monkeypatch)
    deliver(node,status(node._mode_bound_selection))
    node._mode_bound_selection=dict(node._mode_bound_selection,generation=4,stamp_ns=4_000_000_000)
    assert node._final_visual_wait_failure(plan) is None
    node._mode_bound_selection=None
    assert node._final_visual_wait_failure(plan) is None


def test_bound_loss_is_reported_by_post_move_confirmation_without_new_phase(configured,monkeypatch):
    node,_,plan,_,_,_,now,forbidden=prepare(configured,monkeypatch)
    plan.diagnostic='CONTACT_EXECUTION_PLAN'
    deliver(node,status(node._mode_bound_selection))
    node._set_near_field_active=forbidden
    result=node._confirm_final_center_alignment(plan,dict(
        final_visual_refine_post_move_confirm_enabled=True,
        final_visual_refine_post_move_confirm_timeout_sec=20.,
        final_visual_refine_post_move_confirm_required_samples=1,
        final_visual_refine_post_move_confirm_max_jitter_m=.006,
        final_visual_refine_post_move_confirm_max_residual_m=.006))
    assert not result.ok and result.code=='TARGET_LOST'
    assert now[0]==5.


@pytest.mark.parametrize('confirmation',[False,True])
def test_loss_after_reference_stops_preview_wait_without_motion(configured,monkeypatch,confirmation):
    node,_,plan,_,streams,states,now,forbidden=prepare(configured,monkeypatch)
    plan.diagnostic='CONTACT_EXECUTION_PLAN'
    node._wait_for_final_refine_reference=lambda *args:NS()
    node._set_near_field_active=lambda *args,**kwargs:True
    def request(*args):
        deliver(node,status(node._mode_bound_selection))
        return True
    node._request_near_field_preview_stream=request
    node._copy_near_field_preview_candidate=forbidden
    if confirmation:
        result=node._confirm_final_center_alignment(plan,dict(
            final_visual_refine_post_move_confirm_enabled=True,
            final_visual_refine_post_move_confirm_timeout_sec=20.,
            final_visual_refine_post_move_confirm_required_samples=1,
            final_visual_refine_post_move_confirm_max_jitter_m=.006,
            final_visual_refine_post_move_confirm_max_residual_m=.006))
        assert not result.ok and result.code=='TARGET_LOST'
    else:
        result=node._maybe_final_refine_grasp6d_plan(dict(
            final_visual_refine_enabled=True,final_visual_refine_timeout_sec=20.,
            near_field_strategy='single_snapshot_direct'),{},plan,
            move_pose=forbidden,strict_execute_pose=forbidden)
        assert result is None and states[-1][1].startswith('TARGET_LOST:')
        assert streams==[False]
    assert now[0]==5.
