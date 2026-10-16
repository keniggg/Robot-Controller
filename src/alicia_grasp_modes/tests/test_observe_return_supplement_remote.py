"""Supplemental observation needs completed current search and real new evidence."""
from copy import deepcopy
from dataclasses import replace
from types import SimpleNamespace as NS
import json

import numpy as np
import pytest

from alicia_grasp_modes.active_observation_policy import registered_information_gain, evidence_digest, distinct_view
from alicia_grasp_modes import observe_return_remote as remote
from test_observe_return_remote import (configured, setup_observe_return, prefix_session, send,
    append_measured_view, contact_preview, direct_observation, base, module, deficit)
from test_observe_return_visual import recorded_zero_safe_request


def gained(identity, stamp):
    old=np.array([[0.,y,.02] for y in (.001,.011,.021)])
    before=NS(identity=identity,points_base=old,view_stamps_ns=(stamp-1,))
    after=NS(identity=identity,points_base=np.vstack((old,old+[.01,0.,0.])),
        view_stamps_ns=(stamp-1,stamp),view_indices=np.array([0,0,0,1,1,1]))
    return registered_information_gain(before,after,NS(ok=True),depth_uncertainty_m=.001)


def continuation(configured,monkeypatch):
    node,p=recorded_zero_safe_request(configured)
    stamp=p.snapshot.stamp_ns
    identity=node._current_stream_target_identity()
    node._observe_return_phase='observing_contact'
    node._observe_return_session=dict(version=2,phase='observing',session_id='session',
        selection=deepcopy(node._mode_selection),stamp_ns=stamp-1,
        deadline_ns=stamp+600_000_000_000,origin_plan_id='origin',origin_snapshot_stamp_ns=stamp-2,
        target_epoch=identity.epoch,target_track_id=identity.track_id,
        max_observations=3,view_index=1,view_plan_id='origin',view_source_stamp_ns=stamp-2,
        view_request_id='initial',previous_gain_sha256='0'*64)
    node._observe_return_step_gain=gained(identity,stamp)
    surface, reference = node._active_multiview_surface()
    node._latest_target_observation = base.TargetObservation(identity, stamp, 'base_link',
        reference.points_base, reference.support_normal_base, reference.support_offset_m,
        (12, 12, 10, 10), (480, 640), 12, 'instance_mask')
    node._latest_registration_evidence = base.RegistrationEvidence(identity, stamp,
        base.RegistrationResult(True, 'REGISTERED', np.eye(4), 100, .8, .001, 0., 0.), False)
    node._active_gate_audit_report['grasp_mode_selection']=deepcopy(node._mode_selection)
    p.remote_diagnostics['tabletop_geometry']['adaptive_stage_profiles']=[
        dict(depth_uncertainty_m=.001, execution_position_error_m=0.)]
    monkeypatch.setattr(node,'_require_stream_ticket_current_locked',lambda ticket:None)
    return node,p


def test_full_failed_search_and_independent_current_deficit_prepare_only_an_inert_view(configured,monkeypatch):
    node,p=continuation(configured,monkeypatch)
    before=node._active_multiview_surface()[0]
    deadline=node._observe_return_session['deadline_ns']
    assert node._observe_return_prepare_supplement(p,'NEAR_FIELD_NO_HARD_SAFE_CANDIDATE')
    request=node._observe_return_pending_supplement
    assert request['view_index']==2 and request['previous_view_index']==1
    assert request['previous_gain_sha256']==evidence_digest(request['previous_gain'])
    assert not request['contact_or_motion_authorized']
    assert node._observe_return_session['deadline_ns']==deadline
    assert node._observe_return_saved[0] is before
    assert node._observe_return_phase=='observation_prefix'
    assert not node.near_field_planning_active
    assert node._observe_return_visited_camera_origins == [p.pose_estimator.T_base_camera_link[:3,3].tolist()]
    assert node.status_pub.messages[-1].data.startswith('OBSERVE_RETURN_SUPPLEMENTAL_VIEW_REQUIRED')


@pytest.mark.parametrize('fault', ['collision','ik','timeout','stale','lost','transport','no_gap',
    'incomplete','stale_audit','selected','no_gain','forged_gain','budget','expired','v1','diagnostic',
    'registration','foreign_registration','missing_noise'])
def test_error_alone_or_missing_current_evidence_never_prepares_new_view(configured,monkeypatch,fault):
    node,p=continuation(configured,monkeypatch);code='NEAR_FIELD_NO_HARD_SAFE_CANDIDATE'
    if fault in ('collision','ik','timeout','stale','lost'):
        code=dict(collision='CONTACT_COLLISION',ik='MOVEIT_IK_FAILED',timeout='NEAR_FIELD_DIRECT_TIMEOUT',
            stale='SOURCE_FRAME_STALE',lost='TARGET_LOST')[fault]
    elif fault=='transport':p.remote_failure_code='WSL_UNAVAILABLE'
    elif fault=='no_gap':p.remote_diagnostics['tabletop_geometry']['surface_evidence_rejections']=[]
    elif fault=='incomplete':node._active_gate_audit_report['moveit_selection']['terminated_early']=True
    elif fault=='stale_audit':node._active_gate_audit_report['snapshot_stamp_ns']-=1
    elif fault=='selected':node._active_gate_audit_report['selected']={'plan_id':'winner'}
    elif fault=='no_gain':node._observe_return_step_gain['ok']=False
    elif fault=='forged_gain':node._observe_return_step_gain['new_occupied_voxel_count']=0
    elif fault=='budget':node._observe_return_session['view_index']=3
    elif fault=='expired':node._observe_return_session['deadline_ns']=p.snapshot.stamp_ns
    elif fault=='v1':node._observe_return_session['version']=1
    elif fault=='diagnostic':node._observe_return_diagnostic_observation_only=True
    elif fault=='registration':node._latest_registration_evidence=replace(
        node._latest_registration_evidence,result=replace(node._latest_registration_evidence.result,ok=False))
    elif fault=='foreign_registration':node._latest_registration_evidence=replace(
        node._latest_registration_evidence,stamp_ns=p.snapshot.stamp_ns-1)
    elif fault=='missing_noise':p.remote_diagnostics['tabletop_geometry']['adaptive_stage_profiles']=[]
    before=node._active_multiview_surface()[0]
    assert not node._observe_return_prepare_supplement(p,code)
    assert node._observe_return_phase=='observing_contact'
    assert node._observe_return_pending_supplement is None
    assert node._active_multiview_surface()[0] is before


def v2_session(configured,monkeypatch):
    node,params,clock=setup_observe_return(configured)
    assert node._observe_return_fallback(NS(remote_diagnostics=deficit()))
    session=prefix_session(node)
    pose=base.make_pose_stamped('base_link',[.2,0.,.2],[0.,0.,0.,1.])
    node._observe_return_prefix.poses=[pose.pose]
    monkeypatch.setattr(node,'_tool_from_camera_matrix',lambda:np.eye(4))
    session.update(version=2,max_observations=3,view_index=1,view_plan_id=session['origin_plan_id'],
        view_source_stamp_ns=session['origin_snapshot_stamp_ns'],view_request_id='initial',previous_gain_sha256='0'*64)
    clock[0]=15_000_000_000
    assert send(node,session)['accepted']
    return node,params,clock,session


def test_v2_current_view_can_change_without_overwriting_original_return_prefix(configured,monkeypatch):
    node,params,clock,session=v2_session(configured,monkeypatch)
    original=deepcopy(node._observe_return_origin_prefix)
    node._observe_return_phase='observation_prefix'
    view=deepcopy(original);view.plan_id='new-view';view.header.stamp=module.rospy.Time.from_sec(16.)
    view.poses[0].position.y=.1
    node._observe_return_accept_view_plan(view)
    message=dict(session,view_index=2,view_plan_id=view.plan_id,view_source_stamp_ns=16_000_000_000,
        view_request_id='next-request',previous_gain_sha256='a'*64,stamp_ns=17_000_000_000)
    node._observe_return_pending_supplement={key:message[key] for key in (
        'session_id','view_index','view_request_id','previous_gain_sha256','deadline_ns','max_observations',
        'origin_plan_id','origin_snapshot_stamp_ns','target_epoch','target_track_id')}
    clock[0]=18_000_000_000
    assert send(node,message)['accepted']
    assert node._observe_return_origin_prefix.plan_id==original.plan_id
    assert node._observe_return_prefix.plan_id==view.plan_id
    assert len(node._observe_return_visited_camera_origins)==2
    assert node._observe_return_session['deadline_ns']==session['deadline_ns']
    assert not send(node,message)['accepted']


@pytest.mark.parametrize('fault',['view_plan','view_source','request','gain','origin','deadline'])
def test_remote_ack_requires_exact_new_plan_and_pending_request(configured,monkeypatch,fault):
    node,params,clock,session=v2_session(configured,monkeypatch)
    original=deepcopy(node._observe_return_origin_prefix)
    node._observe_return_phase='observation_prefix'
    view=deepcopy(original);view.plan_id='new-view';view.header.stamp=module.rospy.Time.from_sec(16.)
    node._observe_return_accept_view_plan(view)
    message=dict(session,view_index=2,view_plan_id=view.plan_id,view_source_stamp_ns=16_000_000_000,
        view_request_id='next-request',previous_gain_sha256='a'*64,stamp_ns=17_000_000_000)
    node._observe_return_pending_supplement={key:message[key] for key in (
        'session_id','view_index','view_request_id','previous_gain_sha256','deadline_ns','max_observations',
        'origin_plan_id','origin_snapshot_stamp_ns','target_epoch','target_track_id')}
    if fault=='view_plan':message['view_plan_id']='different'
    elif fault=='view_source':message['view_source_stamp_ns']+=1
    elif fault=='request':message['view_request_id']='different'
    elif fault=='gain':message['previous_gain_sha256']='b'*64
    elif fault=='origin':message['origin_plan_id']='different'
    elif fault=='deadline':message['deadline_ns']+=1
    clock[0]=18_000_000_000
    assert not send(node,message)['accepted']
    assert node._observe_return_session['view_index']==1


def test_valid_contact_after_registered_view_is_not_blocked_by_no_novel_voxels(configured,monkeypatch):
    node,params,clock,session=v2_session(configured,monkeypatch)
    node._observe_return_advance('observing_contact',20_000_000_000,80_000_000_000,active=True,paused=False)
    observation=append_measured_view(node,22_000_000_000)
    contact_preview(node,observation)
    node._observe_return_step_gain=dict(ok=False,code='OBSERVATION_NO_INFORMATION_GAIN')
    clock[0]=24_000_000_000
    assert node._observe_return_ready_contact(dict(session,stamp_ns=23_000_000_000))


def test_v2_reached_registration_records_no_gain_but_allows_complete_contact_search(configured,monkeypatch):
    node,params,clock,session=v2_session(configured,monkeypatch)
    node._observe_return_view_uncertainty_m=.001
    obs=direct_observation(node,19_000_000_000)
    monkeypatch.setattr(node,'_resolve_reached_view_observation',lambda message:obs)
    phase=NS(active=True,phase_id=1,
        header=NS(stamp=module.rospy.Time.from_sec(20.),frame_id='base_link'),
        deadline=module.rospy.Time.from_sec(70.))
    clock[0]=21_000_000_000
    node.near_field_state_cb(phase)
    assert node._observe_return_phase=='observing_contact'
    assert node._latest_registration_evidence.result.ok
    assert not node._observe_return_paused
    assert node._observe_return_step_gain['ok'] is False
    assert node._observe_return_step_gain['source_stamp_ns']==obs.stamp_ns


def test_supplement_audit_is_not_an_external_new_attempt_entry(configured,monkeypatch):
    from alicia_grasp_modes.runner_contract import observe_return_audit_error
    node,p=continuation(configured,monkeypatch)
    assert node._observe_return_prepare_supplement(p,'NEAR_FIELD_NO_HARD_SAFE_CANDIDATE')
    report={'plan_id':'a'*24}
    node._observe_return_audit(report,p)
    metadata=report['observe_return']
    assert metadata['phase']=='supplemental_observation'
    assert metadata['view_source_stamp_ns']==p.snapshot.stamp_ns
    assert metadata['view_plan_id']==report['plan_id']
    assert metadata['origin_snapshot_stamp_ns']==node._observe_return_session['origin_snapshot_stamp_ns']
    assert report['observation_motion_performed'] is True
    assert observe_return_audit_error(report)  # Never grants a new top-level attempt.


def test_initial_source_ray_cannot_be_reused_after_a_different_observation(configured,monkeypatch):
    node,p=recorded_zero_safe_request(configured)
    monkeypatch.setattr(node,'_require_stream_ticket_current_locked',lambda ticket:None)
    opportunity=node._observe_return_mixed_visual_failure(p,'NEAR_FIELD_NO_HARD_SAFE_CANDIDATE')
    assert node._observe_return_fallback(p,visual_opportunity=opportunity)
    camera=p.pose_estimator.T_base_camera_link[:3,3]
    assert node._observe_return_visited_camera_origins == [camera.tolist()]
    center=node._observe_return_view_gaps[0]['target_center_base_m']
    node._observe_return_visited_camera_origins.append((camera+[.1,.1,0.]).tolist())
    assert not distinct_view(camera,center,node._observe_return_visited_camera_origins,.001)
