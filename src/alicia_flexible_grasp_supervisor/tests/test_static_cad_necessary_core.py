"""A box core cannot prove collision of a different validated CAD volume."""
from copy import deepcopy
from types import SimpleNamespace as NS
import pytest
from static_cad_contract_fixtures import gg,G,XML,core_fixture,remote

@pytest.mark.parametrize('body',['left_finger','right_finger'])
@pytest.mark.parametrize('world',range(3))
def test_validated_cad_does_not_inherit_old_box_core_inevitability(body,world):
    node,p=core_fixture(body,world);original=deepcopy(p)
    old=node._contact_goal_inevitable_collision(p,G,.05,.001)
    assert old['rejected']
    actual=node._contact_goal_inevitable_collision(p,G,.05,.001,robot_description=XML)
    assert actual['checked'] and not actual['rejected']
    assert not actual['execution_authorized'] and not actual['path_verified']
    assert p==original

@pytest.mark.parametrize('description',[None,'','<robot name="unsupported"/>'])
def test_unsupported_model_preserves_necessary_box_rejection(description):
    node,p=core_fixture()
    report=node._contact_goal_inevitable_collision(p,G,.05,.001,robot_description=description)
    assert report['rejected'] and not report['path_verified']

@pytest.mark.parametrize('world',range(3))
def test_validated_finger_model_never_disables_palm_core(world):
    node,p=core_fixture('palm',world)
    report=node._contact_goal_inevitable_collision(p,G,.05,.001,robot_description=XML)
    assert report['rejected']
    assert next(b for b in report['bodies'] if b['body']=='palm')['inevitable_overlap']
    assert not report['execution_authorized'] and not report['path_verified']

@pytest.mark.parametrize('axes',[('x','z'),('-y','z'),('y','-z')])
def test_axis_mismatch_retains_conservative_necessary_check(axes):
    node,p=core_fixture()
    report=node._contact_goal_inevitable_collision(p,G,.05,.001,robot_description=XML,
        tool_jaw_axis=axes[0],tool_finger_length_axis=axes[1])
    assert report['rejected'] and not report['execution_authorized']

def test_early_gate_uses_committed_description_and_continues_to_full_checker(monkeypatch):
    r=remote();node,p=core_fixture()
    node.contact_sequence_precheck_enabled=True
    runtime=dict(prepared=NS(model_choice='unknown_tabletop'),registration_support_lift_m=0.)
    node._build_selected_preview_bundle=lambda *a,**kw:dict(rich_plan=p)
    node._stable_variant_runtime={(1,0):runtime}
    node._geometry_robot_description=XML
    def get_param(key,default=None):
        assert key != '/robot_description', 'geometry must use the committed model snapshot'
        return default
    monkeypatch.setattr(r.rospy,'get_param',get_param)
    calls=[];sentinel=NS(reachable=False)
    node._check_moveit_stable_candidate=lambda c:(calls.append('full_checker') or sentinel)
    result=node._check_direct_registered_candidate(NS(track_id=1,variant_index=0))
    assert result is sentinel and calls==['full_checker']
    assert runtime['contact_goal_necessary_check']['rejected'] is False
    node._geometry_robot_description='<robot name="unsupported"/>'
    result=node._check_direct_registered_candidate(NS(track_id=1,variant_index=0))
    assert result.reachable is False and calls==['full_checker']
    assert runtime['contact_goal_necessary_check']['rejected'] is True

