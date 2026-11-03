"""Public geometry evaluators retain whole stage checks with an optional CAD binding."""
from copy import deepcopy
import numpy as np
import pytest
from static_cad_contract_fixtures import gg,G,XML,context,candidate_fixture,explicit_carton_fixture,transformed_public

@pytest.mark.parametrize('kind',['graspnet','explicit'])
@pytest.mark.parametrize('world',range(3))
@pytest.mark.parametrize('width',[.024,.035,.044])
def test_public_evaluators_cover_all_stages_with_validated_description(monkeypatch,kind,world,width):
    if kind=='explicit':
        v=explicit_carton_fixture()
        v['target_points_base'][:,1]*=width/.035
        v['obb_size_xyz_m'][1]=width
        v['required_open_width_m']=width+.004
        evaluate=gg.evaluate_explicit_candidate
    else:
        v=candidate_fixture();v['obb_size_xyz_m'][1]=width
        evaluate=gg.evaluate_candidate
    v=transformed_public(v,world)
    calls=[];real=gg._check_boxes
    def checked(*args,**kw):
        ctx=kw.get('finger_cad_context')
        assert ctx is not None and ctx.model_sha256==context().model_sha256
        calls.append((args,kw))
        return real(*args,**kw)
    monkeypatch.setattr(gg,'_check_boxes',checked)
    result=evaluate(**v,robot_description=XML)
    assert result.ok,(result.failed_gate,result.failure_reason)
    assert len(calls)>4
    assert any(c[1].get('allow_finger_contact', c[0][10] if len(c[0])>10 else False) for c in calls)

@pytest.mark.parametrize('kind',['graspnet','explicit'])
@pytest.mark.parametrize('description',[None,'<robot name="unsupported"/>'])
def test_optional_unsupported_description_preserves_full_original_result(kind,description):
    v=candidate_fixture() if kind=='graspnet' else explicit_carton_fixture()
    evaluate=gg.evaluate_candidate if kind=='graspnet' else gg.evaluate_explicit_candidate
    assert evaluate(**v)==evaluate(**v,robot_description=description)

@pytest.mark.parametrize('kind',['graspnet','explicit'])
def test_support_and_palm_rejections_are_not_removed_from_public_evaluation(kind):
    v=candidate_fixture() if kind=='graspnet' else explicit_carton_fixture()
    evaluate=gg.evaluate_candidate if kind=='graspnet' else gg.evaluate_explicit_candidate
    # Only an open pregrasp is moved; the target/contact/width evidence stays valid.
    v['pregrasp_T_base_tool']=v['grasp_T_base_tool'].copy()
    _,palm_center,_=next(b for b in gg._stage_boxes(
        v['pregrasp_T_base_tool'],G,.05,'y','z') if b[0]=='palm')
    v['pregrasp_T_base_tool'][:3,3]+=v['obb_center_base']-palm_center
    # Isolate target penetration from a low synthetic support plane.
    v['support_offset_m']=1.
    result=evaluate(**v,robot_description=XML)
    assert not result.ok and 'palm' in result.failure_reason, result
    # Clearance remains mandatory even if the target is well separated.
    v=candidate_fixture() if kind=='graspnet' else explicit_carton_fixture()
    v['pregrasp_T_base_tool'][2,3]=-.1
    result=evaluate(**v,robot_description=XML)
    assert not result.ok and 'support' in result.failure_reason

@pytest.mark.parametrize('kind',['graspnet','explicit'])
def test_required_opening_limit_not_relaxed(kind):
    if kind=='explicit':
        v=explicit_carton_fixture();v['target_points_base'][:,1]*=.06/.035
        v['obb_size_xyz_m'][1]=.06;v['required_open_width_m']=.064
        evaluate=gg.evaluate_explicit_candidate
    else:
        v=candidate_fixture();v['obb_size_xyz_m'][1]=.06;evaluate=gg.evaluate_candidate
    result=evaluate(**v,robot_description=XML)
    assert not result.ok and result.failed_gate=='jaw_width', (result.failed_gate,result.failure_reason)

