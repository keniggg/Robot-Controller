"""The same source-bound view gate applies to every unknown auxiliary view."""
from copy import deepcopy
from dataclasses import replace
from types import SimpleNamespace as NS
import threading

import numpy as np
import pytest

from test_remote_modes import configured, base
from test_silhouette_observation_view_source import view_context, encoded_result
from alicia_grasp_modes.silhouette_observation_planning import INFORMATION_ORDER_POLICY as POLICY


def generic_context(configured,monkeypatch,tmp_path):
    node,prepared,sample,camera,fk=view_context(configured,monkeypatch,tmp_path)
    node._observe_return_diagnostic_observation_only=False
    # A normal missing-side view has no silhouette diagnosis or envelope.
    prepared=replace(prepared,remote_diagnostics={})
    node._bind_diagnostic_observation_view_source(prepared,
        dict(sample=sample,intrinsics=camera,tool_from_camera=np.eye(4),
        view_limits=node._observation_view_limit_configuration()))
    return node,prepared,sample,camera,fk


def test_missing_side_path_uses_same_raw_source_and_preserves_all_old_pose_fields(configured,monkeypatch,tmp_path):
    node,prepared,_,_,fk=generic_context(configured,monkeypatch,tmp_path)
    pose=base.make_pose_stamped('base_link',[0.,0.,0.],[0,0,0,1],stamp=prepared.stamp)
    variants=[]
    for index,angle in enumerate((.10,.02)):
        matrix=fk([angle,0,0,0,0,0])
        view=base.make_pose_stamped('base_link',matrix[:3,3],base.quaternion_from_matrix(matrix),stamp=prepared.stamp)
        sequence=NS(pregrasp=view,approach=deepcopy(view),grasp=deepcopy(view),lift=deepcopy(view),
            observation_view_audit=dict(orientation_source='measured_missing_side_geodesic',
                information_utility=dict(utility=.5)))
        variants.append(dict(sequence=sequence,observation_view=sequence.observation_view_audit,
            observation_envelope=base.ObservationEnvelopeResult(True,'','',.02),
            observation_side_evidence={},camera_roll_offset_deg=0.,camera_tilt_offset_deg=0.,
            preference_index=index))
    candidate=NS(track_id=31,variant_index=1,candidate_source='tabletop_geometry')
    # Contact yaw is deliberately unrelated to the frozen identity tool pose.
    contact=base.make_pose_stamped('base_link',[.2,.1,.1],[0.,0.,1.,0.],stamp=prepared.stamp)
    runtime=dict(prepared=prepared,grasp_pose=contact,sequence=variants[0]['sequence'],
        observation_sequence=variants[0]['sequence'],observation_variants=tuple(variants),soft_evidence={})
    node._stable_variant_runtime={(31,1):runtime}
    checked=[]
    def strict(target,observation=False):
        checked.append(target)
        result,_=encoded_result(prepared,fk,target=target)
        result=replace(result,reason=result.reason+' observation_screened_duration_sec=2.0')
        return result,{},{}
    monkeypatch.setattr(node,'_strict_moveit_evaluation',strict)
    outcome=node._check_moveit_stable_candidate(candidate)
    assert outcome.reachable
    assert checked==[variants[1]['sequence'].pregrasp,variants[0]['sequence'].pregrasp]
    assert runtime['observation_orientation_search']['check_order']==POLICY
    assert node._active_unknown_observation_check_runtime is None
    assert all(set(('pregrasp','approach','grasp','lift')).issubset(vars(v['sequence'])) for v in variants)
    assert runtime['grasp_pose'] is contact
    assert all(row['following_support']['unknown_observation_view']['ok']
        for row in runtime['observation_orientation_search']['variant_results'])


@pytest.mark.parametrize('fault',['path','source','disabled'])
def test_plain_tabletop_observation_cannot_bypass_guard(configured,monkeypatch,tmp_path,fault):
    node,prepared,_,_,fk=generic_context(configured,monkeypatch,tmp_path)
    result,target=encoded_result(prepared,fk)
    if fault=='source':node._diagnostic_observation_view_prepared=replace(prepared)
    if fault=='path':result=replace(result,reason='checked without path')
    if fault=='disabled':
        candidate=NS(track_id=17,variant_index=2)
        node._stable_variant_runtime={(17,2):dict(prepared=prepared)}
        node.observation_following_guard_required=False
        with pytest.raises(base.CandidateContractError) as error:node._check_moveit_stable_candidate(candidate)
        assert error.value.code=='UNKNOWN_OBSERVATION_VIEW_GUARD_REQUIRED'
    else:
        monkeypatch.setattr(base.RemoteGrasp6DNode,'_observation_following_support_evaluation',
            lambda *_:pytest.fail('following proof entered before mandatory unknown view check'))
        outcome,report=node._observation_following_support_evaluation(prepared,result,target)
        assert not outcome.reachable and outcome.failure_code=='UNKNOWN_OBSERVATION_PATH_VIEW_INVALID'
        assert report['failure_location']=='path_view'
    assert not any(p.valid for p in node.rich_plan_pub.messages)


def test_regular_prepare_captures_original_before_inference_and_binds_final_prepared(configured,monkeypatch,tmp_path):
    node,prepared,sample,camera,_=generic_context(configured,monkeypatch,tmp_path)
    order=[]
    monkeypatch.setattr(node,'_capture_diagnostic_observation_view_source',lambda ticket:order.append('capture') or
        dict(sample=sample,intrinsics=camera,tool_from_camera=np.eye(4),
        view_limits=node._observation_view_limit_configuration()))
    def predict(_node,ticket):
        assert order==['capture'];order.append('predict')
        return prepared
    monkeypatch.setattr(base.RemoteGrasp6DNode,'_prepare_and_predict',predict)
    result=node._prepare_and_predict(prepared.ticket)
    assert order==['capture','predict']
    assert node._diagnostic_observation_view_prepared is result
    assert node._require_diagnostic_observation_view_source(result)['binding']['request_id']==prepared.ticket.request_id


def test_contact_and_non_unknown_do_not_enter_observation_projection(configured,monkeypatch,tmp_path):
    node,prepared,_,_,fk=generic_context(configured,monkeypatch,tmp_path)
    result,target=encoded_result(prepared,fk)
    monkeypatch.setattr(node,'_diagnostic_observation_view_evaluation',lambda *_:pytest.fail('view guard entered contact'))
    node._observe_return_phase='initial_contact'
    actual,_=node._observation_following_support_evaluation(replace(prepared,near_field=True),result,target)
    assert actual.reachable
    node._mode_selection['mode']='carton'
    actual,_=node._observation_following_support_evaluation(prepared,result,target)
    assert actual.reachable


def test_real_worker_pending_requests_do_not_replace_active_accept_cloud():
    # Use the production worker/coordinator, not a synthetic scheduling loop.
    # Building a measured cloud belongs to prepare. A fresh pending snapshot
    # cannot enter prepare while the previous accept is doing a long check.
    from test_remote_grasp6d_streaming import streaming_node, snapshot, MutableClock, wait_until
    prepare_entered=threading.Event();prepare_release=threading.Event()
    accept_entered=threading.Event();accept_release=threading.Event()
    prepared_ids=[];accepted_ids=[];single_cache={}
    def prepare(ticket):
        prepared_ids.append(ticket.request_id)
        if ticket.request_id==1:
            prepare_entered.set();assert prepare_release.wait(2.)
        cloud=object()
        single_cache.clear();single_cache.update(request_id=ticket.request_id,cloud=cloud)
        return NS(ticket=ticket,view_source=cloud,remote_diagnostics={},remote_performance={})
    worker=streaming_node(clock=MutableClock(10.),prepare=prepare)
    def accept(prepared):
        if prepared.ticket.request_id==1:
            accept_entered.set();assert accept_release.wait(2.)
        worker._require_stream_ticket_current(prepared.ticket)
        assert single_cache['cloud'] is prepared.view_source
        assert single_cache['request_id']==prepared.ticket.request_id
        accepted_ids.append(prepared.ticket.request_id)
    worker._accept_prediction=accept
    try:
        worker.start_streaming();worker.submit_stream_snapshot(snapshot(9.8))
        assert prepare_entered.wait(1.)
        worker.submit_stream_snapshot(snapshot(9.9))
        prepare_release.set();assert accept_entered.wait(1.)
        first=single_cache['cloud']
        worker.submit_stream_snapshot(snapshot(9.95))
        assert prepared_ids==[1] and single_cache['cloud'] is first
        assert worker.inference_coordinator.pending_count==1
        accept_release.set()
        wait_until(lambda:3 in accepted_ids)
        assert accepted_ids[0]==1 and accepted_ids[-1]==3
        assert prepared_ids==[1,3]  # Unprepared request 2 replaced by latest 3.
        assert single_cache['request_id']==3 and single_cache['cloud'] is not first
    finally:
        prepare_release.set();accept_release.set();worker.shutdown_streaming_worker()


@pytest.mark.parametrize('key,value',[
    ('camera_visibility_margin_px',40.),('camera_visibility_min_depth_m',.09),
    ('camera_visibility_max_depth_m',.45)])
def test_candidate_visibility_limit_change_invalidates_frozen_request(configured,monkeypatch,tmp_path,key,value):
    node,prepared,_,_,fk=generic_context(configured,monkeypatch,tmp_path)
    result,target=encoded_result(prepared,fk)
    configured[1]['/grasp_6d/remote/'+key]=value
    outcome,report=node._observation_following_support_evaluation(prepared,result,target)
    assert not outcome.reachable and outcome.failure_code=='UNKNOWN_OBSERVATION_PATH_VIEW_INVALID'
    assert report['measured_source']['view_limits']['effective_visibility_limits']['roi_margin_px']==36.


def test_tighter_source_limits_reach_real_candidate_guard(configured,monkeypatch,tmp_path):
    configured[1].update({'/grasp_6d/remote/camera_visibility_margin_px':40.,
        '/grasp_6d/remote/camera_visibility_min_depth_m':.09,
        '/grasp_6d/remote/camera_visibility_max_depth_m':.4})
    node,prepared,_,_,fk=generic_context(configured,monkeypatch,tmp_path)
    result,target=encoded_result(prepared,fk)
    report=node._diagnostic_observation_view_evaluation(prepared,result,target)
    assert report['target_trajectory']['required_image_margin_px']==40.
    assert report['target_trajectory']['minimum_camera_depth_m']==.09
    assert report['target_trajectory']['maximum_camera_depth_m']==.4
    source=report['source']['view_limits']
    assert source['requested_visibility_limits_snapshot']['camera_visibility_margin_px']==40.
    assert source['effective_visibility_limits']['roi_margin_px']==40.


def test_missing_original_limit_capture_does_not_borrow_current_setting(configured,monkeypatch,tmp_path):
    node,prepared,sample,camera,_=generic_context(configured,monkeypatch,tmp_path)
    with pytest.raises(base.CandidateContractError) as error:
        node._bind_diagnostic_observation_view_source(prepared,
            dict(sample=sample,intrinsics=camera,tool_from_camera=np.eye(4)))
    assert error.value.code=='UNKNOWN_OBSERVATION_VIEW_SOURCE_INVALID'
