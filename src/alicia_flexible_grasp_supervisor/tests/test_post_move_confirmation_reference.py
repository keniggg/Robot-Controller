"""Final confirmation cannot register against an object from before arrival."""
from unittest.mock import Mock
import pytest
from test_final_refine_reference import setup_node, module


def config(timeout=.3, samples=1):
    return dict(final_visual_refine_post_move_confirm_enabled=True,
        final_visual_refine_post_move_confirm_timeout_sec=timeout,
        final_visual_refine_post_move_confirm_required_samples=samples,
        final_visual_refine_post_move_confirm_max_jitter_m=.006,
        final_visual_refine_post_move_confirm_max_residual_m=.006)


def test_pre_arrival_cached_object_cannot_start_confirmation_phase(monkeypatch):
    node,helper,clock=setup_node(monkeypatch)
    node.latest_grasp6d_preview_plan=None
    node._set_near_field_active=Mock(return_value=True)
    node._request_near_field_preview_stream=Mock(return_value=True)
    result=node._confirm_final_center_alignment(helper._rich_plan(stamp_sec=10.),config())
    assert not result.ok and result.code=='FINAL_REFINE_POST_MOVE_CONFIRM_REFERENCE_UNAVAILABLE'
    assert clock[0]<=12.3+1e-9
    node._set_near_field_active.assert_not_called()
    node._request_near_field_preview_stream.assert_not_called()


@pytest.mark.parametrize('expired_stage',['none','reference','phase','stream','validation'])
def test_fresh_reference_and_all_confirmation_work_share_original_deadline(monkeypatch,expired_stage):
    node,helper,clock=setup_node(monkeypatch)
    plan=helper._mark_valid_3d(helper._rich_plan(stamp_sec=10.))
    preview=helper._mark_valid_3d(helper._rich_plan(stamp_sec=12.26))
    node.latest_grasp6d_preview_plan=preview
    selected=helper._object(stamp_sec=12.20)
    waits=[];phases=[];requests=[]
    def fresh(plan_arg,minimum,deadline,poll):
        waits.append((minimum,deadline))
        clock[0]=12.3 if expired_stage=='reference' else 12.25
        # Deliberately leave latest_obj pointing to the pre-arrival cache.
        return selected
    def phase(*args,**kwargs):
        phases.append(kwargs)
        if expired_stage=='phase':clock[0]=12.3
        return True
    def stream(*args):
        requests.append(args)
        if expired_stage=='stream':clock[0]=12.3
        return True
    def valid(*args,**kwargs):
        if expired_stage=='validation':clock[0]=12.3
        return module.PlanValidationResult(True)
    node._wait_for_final_refine_reference=fresh
    node._set_near_field_active=phase
    node._request_near_field_preview_stream=stream
    monkeypatch.setattr(module,'validate_execution_plan',valid)
    monkeypatch.setattr(module,'validate_final_refinement_execution',lambda *_:module.PlanValidationResult(True))
    result=node._confirm_final_center_alignment(plan,config())
    assert waits==[(12_000_000_000,12.3)]
    if expired_stage=='reference':
        assert not phases and not requests
    else:
        assert len(phases)==1 and phases[0]['reference_object'] is selected
        assert phases[0]['absolute_deadline_sec']==12.3
        assert phases[0]['budget_sec']==pytest.approx(.05)
        assert selected is not node.latest_obj
    if expired_stage=='phase':assert not requests
    assert result.ok == (expired_stage=='none')


def test_repeated_confirmation_phases_do_not_renew_the_deadline(monkeypatch):
    node,helper,clock=setup_node(monkeypatch)
    plan=helper._mark_valid_3d(helper._rich_plan(stamp_sec=10.))
    frames=[helper._mark_valid_3d(helper._rich_plan(stamp_sec=t)) for t in (12.05,12.15)]
    node.latest_grasp6d_preview_plan=frames[0]
    calls=[];phases=[]
    def fresh(_plan,minimum,deadline,poll):
        calls.append((minimum,deadline))
        clock[0]+=.02
        return helper._object(stamp_sec=clock[0]-.01)
    def phase(*_,**kwargs):phases.append(kwargs);return True
    node._wait_for_final_refine_reference=fresh
    node._set_near_field_active=phase
    node._request_near_field_preview_stream=lambda *_:True
    monkeypatch.setattr(module,'validate_execution_plan',lambda *_,**__:module.PlanValidationResult(True))
    monkeypatch.setattr(module,'validate_final_refinement_execution',lambda *_:module.PlanValidationResult(True))
    def sleep(duration):
        clock[0]+=duration
        node.latest_grasp6d_preview_plan=frames[1]
    monkeypatch.setattr(module.rospy,'sleep',sleep)
    result=node._confirm_final_center_alignment(plan,config(samples=2))
    assert result.ok
    assert len(calls)==2 and calls[0]==calls[1]==(12_000_000_000,12.3)
    assert phases[0]['absolute_deadline_sec']==phases[1]['absolute_deadline_sec']==12.3
    assert phases[1]['budget_sec']<phases[0]['budget_sec']
