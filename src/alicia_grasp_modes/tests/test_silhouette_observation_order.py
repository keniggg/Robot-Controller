from copy import deepcopy
from dataclasses import replace
from types import SimpleNamespace as NS

import numpy as np
import pytest

from test_remote_modes import configured, module, base
from test_silhouette_observation_planning import context
from alicia_grasp_modes.silhouette_observation_order import endpoint_schedule_metrics, POLICY


def transform(angle, translation=(0., 0., 0.)):
    c,s=np.cos(angle),np.sin(angle)
    result=np.eye(4);result[:3,:3]=[[1,0,0],[0,c,-s],[0,s,c]]
    result[:3,3]=translation
    return result


def test_same_camera_axis_different_tool_roll_has_actual_pi_rotation():
    reference=np.eye(4);candidate=transform(np.pi)
    np.testing.assert_array_equal(reference[:3,0],candidate[:3,0])
    assert endpoint_schedule_metrics(reference,candidate,.02)['tool_so3_delta_rad']==pytest.approx(np.pi)
    assert endpoint_schedule_metrics(reference,reference,.02)['tool_so3_delta_rad']==0.


def test_equal_rotation_support_margin_precedes_shorter_translation():
    # A short move with almost no plane clearance need not consume the limited
    # strict-proof budget before the same orientation with ample clearance.
    near=endpoint_schedule_metrics(np.eye(4),transform(.3,(.02,0,0)),.004)
    far=endpoint_schedule_metrics(np.eye(4),transform(.3,(.10,0,0)),.030)
    assert far['sort_key']<near['sort_key']
    shortest=endpoint_schedule_metrics(np.eye(4),transform(.3,(.01,0,0)),.030)
    assert shortest['sort_key']<far['sort_key']


@pytest.mark.parametrize('invalid',[np.full((4,4),np.nan),np.zeros((4,4)),np.diag([-1,1,1,1])])
def test_non_rigid_reference_never_creates_preference(invalid):
    with pytest.raises(ValueError):endpoint_schedule_metrics(invalid,np.eye(4),.02)


def test_real_promotion_uses_frozen_reference_not_live_joint_or_roll_labels(configured,monkeypatch,tmp_path):
    node,prepared,checks,variant=context(configured,monkeypatch,tmp_path)
    reference=base.pose_matrix(variant['sequence'].pregrasp)
    specs=[(.4,.005,.02,0),(.2,.005,.01,180),(.2,.030,.12,90),(.2,.030,.04,-90)]
    variants=[]
    for index,(angle,clearance,shift,label) in enumerate(specs):
        v=deepcopy(variant);matrix=reference@transform(angle,(shift,0,0))
        v['sequence'].pregrasp=base.make_pose_stamped('base_link',matrix[:3,3],
            base.quaternion_from_matrix(matrix),stamp=prepared.stamp)
        v['observation_envelope']=base.ObservationEnvelopeResult(True,'','',clearance)
        v['camera_roll_offset_deg']=label;v['preference_index']=index
        variants.append(v)
    monkeypatch.setattr(node,'_far_field_observation_variants',lambda *_:(tuple(variants),()))
    node.latest_joint_state=NS(position=[999.]*6)  # Not used as the source reference.
    result=node._accept_prediction(prepared)
    assert result['status']=='PREVIEW_READY'
    actual=[next(i for i,v in enumerate(variants) if target is v['sequence'].pregrasp) for target,_ in checks]
    assert actual==[3,2,1,0]
    assert len(actual)==len(variants)
    runtime=node._stable_variant_runtime[(0,0)]
    audit=runtime['observation_orientation_search']
    assert audit['check_order']==POLICY
    assert audit['scheduling']['pruned_candidate_count']==0
    for v in variants:
        a=v['sequence'].observation_view_audit
        assert a['endpoint_schedule_proves_joint_motion'] is False
        assert a['endpoint_schedule_proves_path_visibility'] is False
        assert a['endpoint_schedule_is_global_optimum'] is False
        assert a['original_strict_check_budget_unchanged'] is True
    # Identical FK orientation could also have a different IK branch; this
    # scheduler deliberately has no joint-space qualification claim.
    assert audit['scheduling']['proves_joint_motion_or_path_visibility'] is False


@pytest.mark.parametrize('corruption',['missing','request','reference','stamp','endpoint'])
def test_missing_or_stale_reference_fails_closed(configured,monkeypatch,tmp_path,corruption):
    node,prepared,_,variant=context(configured,monkeypatch,tmp_path)
    node._accept_prediction(prepared)
    runtime=node._stable_variant_runtime[(0,0)]
    if corruption=='missing':runtime.pop('diagnostic_observation_reference')
    elif corruption=='request':runtime['diagnostic_observation_reference']['request_id']+=1
    elif corruption=='reference':runtime['grasp_pose']=deepcopy(runtime['grasp_pose']);runtime['grasp_pose'].pose.position.x+=.001
    elif corruption=='stamp':runtime['grasp_pose']=deepcopy(runtime['grasp_pose']);runtime['grasp_pose'].header.stamp=module.rospy.Time(1)
    elif corruption=='endpoint':variant['observation_envelope']=replace(variant['observation_envelope'],ok=False,failure_code='COLLISION',failure_reason='collision')
    with pytest.raises(base.CandidateContractError) as exc:
        node._order_observation_checks((variant,))
    assert exc.value.code=='SILHOUETTE_OBSERVATION_REFERENCE_INVALID'
