from copy import deepcopy
from dataclasses import replace
from types import SimpleNamespace as NS, MethodType
import json

import numpy as np
import pytest

from test_remote_modes import configured, base, module
from test_silhouette_observation_planning import context
from test_observation_path_guard import point, trajectory
from alicia_flexible_grasp.robot.observation_path_guard import SerialUrdfFk
from alicia_flexible_grasp.robot.observation_preview import encode_path_evidence
from alicia_flexible_grasp.vision.rgbd_snapshot import RgbdSample
from alicia_grasp_modes.silhouette_observation_view_source import measured_view_source
from alicia_grasp_modes.silhouette_observation_planning import SilhouetteObservationPlanningMixin


def view_context(configured,monkeypatch,tmp_path):
    node,prepared,checks,variant=context(configured,monkeypatch,tmp_path)
    raw=np.full((240,320),300,np.uint16);raw[106:134,145:175]=280
    color=np.zeros((240,320,3),np.uint8);mask=np.zeros(raw.shape,np.uint8);mask[106:134,145:175]=255
    snapshot=replace(prepared.snapshot,color_bgr=color,depth_raw=np.full_like(raw,900),
        target_depth_raw=raw,object_mask=mask,bbox=(145,106,30,28))
    ticket=replace(prepared.ticket,payload=(snapshot,None))
    C=np.eye(4);C[:3,:3]=[[0,0,1],[-1,0,0],[0,-1,0]]
    geometry=replace(prepared.geometry,center_base=np.array([.28,0.,0.]),axes_base=np.eye(3),
        size_xyz_m=np.array([.02,.02,.02]),support_normal_base=np.array([-1.,0.,0.]),support_offset_m=.3)
    prepared=replace(prepared,ticket=ticket,snapshot=snapshot,geometry=geometry,
        pose_estimator=NS(_T_base_optical=C,T_base_camera_link=np.eye(4)))
    sample=RgbdSample(color.copy(),raw.copy(),mask.copy(),snapshot.bbox,deepcopy(snapshot.object_msg),
        snapshot.stamp_sec,snapshot.frame_id,np.zeros(6),snapshot.stamp_ns,snapshot.target_epoch,snapshot.target_identity)
    camera=NS(width=320,height=240,fx=240.,fy=240.,cx=160.,cy=120.,depth_scale=.001)
    monkeypatch.setattr(node,'_camera_intrinsics',lambda:camera)
    monkeypatch.setattr(node,'_tool_from_camera_matrix',lambda:np.eye(4))
    for name in ('_capture_diagnostic_observation_view_source','_bind_diagnostic_observation_view_source',
                 '_require_diagnostic_observation_view_source','_diagnostic_observation_view_evaluation'):
        monkeypatch.setattr(node,name,MethodType(getattr(SilhouetteObservationPlanningMixin,name),node))
    node.frames=NS(wait_for_exact_sample=lambda *a,**k:sample)
    node._bind_diagnostic_observation_view_source(prepared,dict(sample=sample,intrinsics=camera,tool_from_camera=np.eye(4),
        view_limits=node._observation_view_limit_configuration()))
    names=['Joint%d'%i for i in range(1,7)]
    xml='<robot name="offline">'+''.join('<joint name="%s" type="revolute"><parent link="%s"/><child link="%s"/><axis xyz="0 0 1"/></joint>'%(n,'base_link' if i==0 else 'link%d'%i,'tool0' if i==5 else 'link%d'%(i+1)) for i,n in enumerate(names))+'</robot>'
    configured[1]['/robot_description']=xml
    fk=SerialUrdfFk(xml,names)
    return node,prepared,sample,camera,fk


def encoded_result(prepared,fk,points=None,hold=None,target=None):
    points=points or [point([0.]*6,0.,[0.]*6,[0.]*6),point([.02,0,0,0,0,0],1.,[0.]*6,[0.]*6)]
    plan=trajectory(points,fk.names)
    target=(base.make_pose_stamped('base_link',[0.,0.,0.],[0.,0.,0.,1.],stamp=prepared.stamp)
        if target is None else target)
    hold=point([0.]*6,v=[0.]*6,a=[0.]*6) if hold is None else hold
    token=encode_path_evidence(target,plan,hold,.05,'preserved',123,fk.model_sha256)
    reason='observation_joint_names='+json.dumps(list(fk.names))+' observation_goal_joint_positions='+json.dumps(list(points[-1].positions))+' observation_robot_description_sha256='+fk.model_sha256+' observation_path_evidence='+token
    return base.MoveItResult(True,.1,.02,reason,True,True,True,True),target


def test_raw_support_excludes_fused_depth_and_records_exact_source(configured,monkeypatch,tmp_path):
    node,prepared,sample,camera,_=view_context(configured,monkeypatch,tmp_path)
    source=node._require_diagnostic_observation_view_source(prepared)
    assert np.allclose(source['points_base'][:,0],.3)
    assert len(source['points_base'])==sample.depth_raw.size-np.count_nonzero(sample.object_mask)
    assert source['audit']['source_stamp_ns']==prepared.snapshot.stamp_ns
    assert source['audit']['synthetic_support_points'] is False
    assert source['audit']['projection_prediction_only'] is True
    assert not source['points_base'].flags.writeable
    assert np.all(prepared.snapshot.depth_raw==900)


@pytest.mark.parametrize('fault',['newer','other_identity','wrong_payload','wrong_image','not_uint16','wrong_plane'])
def test_support_source_never_substitutes_later_or_fused_data(configured,monkeypatch,tmp_path,fault):
    _,prepared,sample,camera,_=view_context(configured,monkeypatch,tmp_path)
    if fault=='newer':sample.stamp_ns+=1
    elif fault=='other_identity':sample.target_epoch+=1
    elif fault=='wrong_payload':prepared=replace(prepared,ticket=replace(prepared.ticket,payload=(replace(prepared.snapshot),None)))
    elif fault=='wrong_image':sample.color_bgr[0,0]=1
    elif fault=='not_uint16':sample.depth_raw=sample.depth_raw.astype(float)
    elif fault=='wrong_plane':prepared=replace(prepared,geometry=replace(prepared.geometry,support_offset_m=.32))
    with pytest.raises(ValueError):measured_view_source(sample,prepared,camera,np.eye(4))


def test_candidate_decodes_real_polynomial_hold_and_measured_support(configured,monkeypatch,tmp_path):
    node,prepared,_,_,fk=view_context(configured,monkeypatch,tmp_path)
    result,target=encoded_result(prepared,fk)
    report=node._diagnostic_observation_view_evaluation(prepared,result,target)
    assert report['ok']
    assert report['target_trajectory']['sample_count']>3
    assert report['endpoint_support']['ok']
    assert report['candidate_path_only_not_execution_authority']
    assert report['source']['source_stamp_ns']==prepared.snapshot.stamp_ns


@pytest.mark.parametrize('fault',['camera','handeye','cache_request','missing_path','goal_mismatch','midpath_exit','actual_hold_exit','support_missing'])
def test_view_rejection_precedes_expensive_support_proof(configured,monkeypatch,tmp_path,fault):
    node,prepared,_,camera,fk=view_context(configured,monkeypatch,tmp_path)
    result,target=encoded_result(prepared,fk)
    if fault=='camera':camera.fx+=1.
    elif fault=='handeye':
        changed=np.eye(4);changed[0,3]=.001
        monkeypatch.setattr(node,'_tool_from_camera_matrix',lambda:changed)
    elif fault=='cache_request':node._diagnostic_observation_view_prepared=replace(prepared)
    elif fault=='missing_path':result=replace(result,reason='checked')
    elif fault=='goal_mismatch':result=replace(result,reason=result.reason.replace('[0.02, 0, 0, 0, 0, 0]','[0.03, 0, 0, 0, 0, 0]'))
    elif fault=='midpath_exit':
        result,target=encoded_result(prepared,fk,[point([0.]*6,0.,[0.]*6,[0.]*6),
            point([1.,0,0,0,0,0],1.,[0.]*6,[0.]*6),point([0.]*6,2.,[0.]*6,[0.]*6)])
    elif fault=='actual_hold_exit':result,target=encoded_result(prepared,fk,hold=point([1.,0,0,0,0,0],v=[0.]*6,a=[0.]*6))
    elif fault=='support_missing':
        source=base._thaw_request_evidence(node._diagnostic_observation_view_source)
        source['points_base']=source['points_base'][:3]
        from alicia_grasp_modes.silhouette_observation_view_source import array_fingerprint
        source['audit']['support_points_sha256']=array_fingerprint(source['points_base'])
        node._diagnostic_observation_view_source=base._freeze_request_evidence(source)
    monkeypatch.setattr(base.RemoteGrasp6DNode,'_observation_following_support_evaluation',lambda *_:pytest.fail('expensive following proof entered'))
    actual,report=node._observation_following_support_evaluation(prepared,result,target)
    assert not actual.reachable
    assert actual.failure_code=='SILHOUETTE_OBSERVATION_PATH_VIEW_INVALID'
    assert report['failure_location']=='path_view'
    assert not any(p.valid for p in node.rich_plan_pub.messages)


def test_disabled_diagnostic_path_guard_is_not_silently_enabled(configured,monkeypatch,tmp_path):
    node,prepared,_,_=context(configured,monkeypatch,tmp_path)
    node.observation_following_guard_required=False
    with pytest.raises(base.CandidateContractError) as error:node._accept_prediction(prepared)
    assert error.value.code=='SILHOUETTE_OBSERVATION_VIEW_GUARD_REQUIRED'
    assert node.observation_following_guard_required is False


def test_capture_fails_without_exact_sample_and_never_asks_for_latest(configured,monkeypatch,tmp_path):
    node,prepared,_,_,_=view_context(configured,monkeypatch,tmp_path)
    calls=[]
    node.frames=NS(wait_for_exact_sample=lambda *a,**k:calls.append((a,k)))
    with pytest.raises(base.CandidateContractError) as error:
        node._capture_diagnostic_observation_view_source(prepared.ticket)
    assert error.value.code=='SILHOUETTE_OBSERVATION_VIEW_SOURCE_INVALID'
    assert len(calls)==1 and calls[0][0]==(prepared.snapshot.stamp_ns,prepared.snapshot.target_identity)
    assert calls[0][1]['timeout_sec']==0.


def test_new_bound_prepared_replaces_cache_and_invalidates_old_reference(configured,monkeypatch,tmp_path):
    node,prepared,sample,camera,_=view_context(configured,monkeypatch,tmp_path)
    old=node._diagnostic_observation_view_source
    new=replace(prepared)
    node._bind_diagnostic_observation_view_source(new,dict(sample=sample,intrinsics=camera,tool_from_camera=np.eye(4),
        view_limits=node._observation_view_limit_configuration()))
    assert node._diagnostic_observation_view_source is not old
    assert node._diagnostic_observation_view_prepared is new
    assert node._require_diagnostic_observation_view_source(new)
    with pytest.raises(ValueError):node._require_diagnostic_observation_view_source(prepared)


def test_calibration_change_during_expensive_proof_cannot_promote(configured,monkeypatch,tmp_path):
    node,prepared,_,camera,fk=view_context(configured,monkeypatch,tmp_path)
    result,target=encoded_result(prepared,fk)
    def changed(_node,_prepared,_result,_target=None):
        camera.fx+=1.
        return _result,dict(ok=True)
    monkeypatch.setattr(base.RemoteGrasp6DNode,'_observation_following_support_evaluation',changed)
    with pytest.raises(ValueError,match='camera, handeye or source points changed'):
        node._observation_following_support_evaluation(prepared,result,target)
    assert not any(p.valid for p in node.rich_plan_pub.messages)


def test_capture_uses_request_owned_raw_source_after_shared_cache_expiry(configured,monkeypatch,tmp_path):
    from alicia_flexible_grasp.vision.rgbd_snapshot import FrozenRgbdSample
    node,prepared,sample,camera,_=view_context(configured,monkeypatch,tmp_path)
    retained=FrozenRgbdSample(**{name:getattr(sample,name) for name in RgbdSample.__dataclass_fields__})
    snapshot=replace(prepared.snapshot,original_sample=retained)
    ticket=replace(prepared.ticket,payload=(snapshot,None))
    node.frames=NS(wait_for_exact_sample=lambda *a,**k:pytest.fail('expired shared-cache re-admission attempted'))
    captured=node._capture_diagnostic_observation_view_source(ticket)
    assert captured['sample'] is retained
    assert np.array_equal(captured['sample'].depth_raw,sample.depth_raw)
    assert not np.array_equal(captured['sample'].depth_raw,snapshot.depth_raw)


def test_mismatched_retained_source_cannot_fall_back_to_shared_cache(configured,monkeypatch,tmp_path):
    from alicia_flexible_grasp.vision.rgbd_snapshot import FrozenRgbdSample
    node,prepared,sample,camera,_=view_context(configured,monkeypatch,tmp_path)
    retained=FrozenRgbdSample(**{name:getattr(sample,name) for name in RgbdSample.__dataclass_fields__})
    retained=replace(retained,stamp_ns=retained.stamp_ns+1)
    snapshot=replace(prepared.snapshot,original_sample=retained)
    ticket=replace(prepared.ticket,payload=(snapshot,None))
    node.frames=NS(wait_for_exact_sample=lambda *a,**k:pytest.fail('mismatched source replaced'))
    with pytest.raises(base.CandidateContractError,match='does not match'):
        node._capture_diagnostic_observation_view_source(ticket)
