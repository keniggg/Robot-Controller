from copy import deepcopy
import json
from types import SimpleNamespace as NS
import pytest
from alicia_grasp_modes.support_observation import support_contradiction, contradiction_after_plan
from test_task_modes import configured, module, base


def status(selection=None):
    selection = selection or dict(mode='unknown',generation=9,stamp_ns=1790566155063051939)
    # Detector wire schema with the failure measured in the 2026-09-28 run.
    stamp=int(selection['stamp_ns'])+70000000000
    return dict(mode='unknown',state='unavailable',target_locked=True,exact_rgbd=True,
        generation=selection['generation'],selection_stamp_ns=int(selection['stamp_ns']),
        source_stamp_ns=stamp,color_stamp_ns=stamp,depth_stamp_ns=stamp,tf_lookup_stamp_ns=stamp,
        detail='perception_failure: support_plane_local_distance_inconsistent: 0.009180 > 0.004000 m')


def test_real_magnitude_remains_a_positive_contradiction():
    selected=dict(mode='unknown',generation=9,stamp_ns=1790566155063051939)
    fault=support_contradiction(status(selected),selected)
    assert fault['value']==.009180 and fault['limit']==.004
    assert contradiction_after_plan(fault,selected,1790566160674229621)
    assert not contradiction_after_plan(fault,selected,fault['source_stamp_ns'])


@pytest.mark.parametrize('key,value',[
    ('generation',8),('selection_stamp_ns',1),('source_stamp_ns',1),
    ('color_stamp_ns',1),('depth_stamp_ns',1),('tf_lookup_stamp_ns',1),
    ('exact_rgbd',False),('target_locked',False),('state','ready'),('generation',True),
    ('detail','perception_failure: locked_support_plane_no_longer_visible'),
    ('detail','perception_failure: locked_target_behind_camera'),
    ('detail','perception_failure: support_plane_local_distance_inconsistent: nan > 0.004000 m'),
    ('detail','perception_failure: support_plane_local_distance_inconsistent: 0.003 > 0.004000 m'),
    ('detail','perception_failure: support_plane_local_distance_inconsistent: 0.009 > 0.004000 deg'),
])
def test_unbound_or_missing_vision_does_not_manufacture_contradiction(key,value):
    selected=dict(mode='unknown',generation=9,stamp_ns=1790566155063051939)
    payload=status(selected);payload[key]=value
    assert support_contradiction(payload,selected) is None


def test_normal_inconsistency_is_kept_with_its_own_original_limit():
    selected=dict(mode='unknown',generation=9,stamp_ns=1790566155063051939)
    payload=status(selected);payload['detail']='perception_failure: support_plane_normal_inconsistent: 9.000000 > 8.000000 deg'
    fault=support_contradiction(payload,selected)
    assert fault['unit']=='deg' and fault['limit']==8.


def test_drift_then_occlusion_blocks_close_without_sending_any_action(configured,monkeypatch):
    from test_support_source_qualification import install, messages
    node,_=configured;node._mode_bound_selection=deepcopy(node._mode_selection)
    payload=status(node._mode_selection)
    install(node, *messages(payload['source_stamp_ns']))
    node._unknown_support_status_cb(NS(data=json.dumps(payload)))
    payload['detail']='perception_failure: locked_support_plane_no_longer_visible'
    node._unknown_support_status_cb(NS(data=json.dumps(payload)))
    plan=NS(model_choice='unknown_tabletop',diagnostic='CONTACT_EXECUTION_PLAN',
            header=NS(stamp=module.rospy.Time(3)))
    monkeypatch.setattr(base.GraspTaskNode,'_validate_bound_plan_locked',lambda *a:base.PlanValidationResult(True))
    actions=[]
    valid,response=node._invoke_plan_bound_action(plan,{},'gripper close',lambda:actions.append('close'))
    assert not valid.ok and valid.code=='SUPPORT_GEOMETRY_CONTRADICTION'
    assert response is None and actions==[]
    assert node._unknown_support_fault['value']==.00918
    # A newer observation/plan is not silently rewritten or rejected by the
    # older event; all original plan checks still execute.
    plan.header.stamp=module.rospy.Time(73)
    assert node._validate_bound_plan_locked(plan,{}).ok


def test_new_generation_does_not_inherit_old_target_fault(configured,monkeypatch):
    node,params=configured;node._mode_bound_selection=deepcopy(node._mode_selection)
    node._unknown_support_status_cb(NS(data=json.dumps(status(node._mode_selection))))
    node._mode_bound_selection=None
    params[module.SELECTION_PARAM]=dict(mode='unknown',strategy='direct',generation=3,stamp_ns='100000000000')
    assert node.reset_mode_cb(None).success
    node._mode_bound_selection=deepcopy(node._mode_selection)
    plan=NS(model_choice='unknown_tabletop',diagnostic='CONTACT_EXECUTION_PLAN',header=NS(stamp=module.rospy.Time(101)))
    monkeypatch.setattr(base.GraspTaskNode,'_validate_bound_plan_locked',lambda *a:base.PlanValidationResult(True))
    assert node._validate_bound_plan_locked(plan,{}).ok
