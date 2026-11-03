"""Atomic measured refresh and strict successor contracts; no live transports."""
from copy import deepcopy
from dataclasses import replace
from types import SimpleNamespace as NS
import json

import numpy as np
import pytest

from test_remote_modes import configured, base
from test_observe_return_remote import setup_observe_return
from test_r27_observation_source_refresh import measured_fixture, materializer_configuration
from alicia_grasp_modes import observe_return_remote as remote
from alicia_grasp_modes.active_observation_policy import evidence_digest
from alicia_grasp_modes.observation_source_refresh import request_binding


def refresh_context(configured, monkeypatch):
    node, params, clock = setup_observe_return(configured)
    saved, observation, geometry = measured_fixture()
    identity = node._current_stream_target_identity()
    saved = (replace(saved[0], identity=identity), replace(saved[1], identity=identity))
    observation = replace(observation, identity=identity)
    materializer_configuration(node)
    node._observe_return_saved = saved
    node._near_field_fused_surface, node._near_field_reference_view = saved
    node._observe_return_phase = 'observation_prefix'
    node.near_field_planning_active = False
    node._observe_return_trigger = 'BILATERAL_SURFACE_EVIDENCE_MISSING'
    node._observe_return_trigger_evidence = dict(policy='historical_contact_result',
        request_id=1,generation=node._stream_generation,target_epoch=identity.epoch,
        snapshot_stamp_ns=saved[1].stamp_ns, materialized_count=16)
    node._observe_return_initial_stamp = saved[1].stamp_ns
    node._observe_return_view_gaps = ({'historical': True},)
    node._observe_return_view_uncertainty_m = .002
    node._observe_return_visited_camera_origins = [[0.,-.15,.16]]
    node._regular_box_snapshot = None
    node._observe_return_paused = False
    node._execution_authority_ticket = None
    node.latest_rich_plan = node.latest_preview_rich_plan = None
    snapshot = NS(stamp_ns=observation.stamp_ns, target_identity=identity,
        quality=NS(depth_mad_m=.001,depth_repeatability_m=.001), object_msg=NS(detected=True))
    ticket = NS(request_id=3, generation=node._stream_generation,target_epoch=identity.epoch)
    transform = np.eye(4); transform[:3,3] = [0.,-.15,.16]
    prepared = NS(ticket=ticket,snapshot=snapshot,geometry=geometry,
        pose_estimator=NS(_T_base_optical=transform), target_observation=observation)
    source = {'binding': request_binding(ticket,snapshot)}
    node._diagnostic_observation_view_source = source
    node._diagnostic_observation_view_prepared = prepared
    node._latest_target_observation = observation
    node._latest_raw_geometry_source = dict(snapshot=snapshot,observation=observation,
        geometry=geometry,transform=transform)
    node._require_diagnostic_observation_view_source = lambda p: source
    node._require_stream_ticket_current_locked = lambda t: None
    node._selection_is_current = lambda: True
    node._observation_source_refresh_pending = dict(request_id=2,generation=ticket.generation,
        target_epoch=identity.epoch,target_track_id=identity.track_id,
        snapshot_stamp_ns=saved[1].stamp_ns+1,owner_sha256=node._observation_refresh_owner(),
        search_complete=False, failure_code='OBSERVATION_SEARCH_INCOMPLETE')
    return node, prepared


def stable_state(node):
    return deepcopy({key: getattr(node,key,None) for key in (
        '_observe_return_phase','_near_field_phase_id','_stream_generation',
        '_near_field_phase_started_sec','_near_field_phase_deadline_sec',
        '_observe_return_initial_contact_deadline_ns','_observe_return_session',
        '_observe_return_pending_supplement','_observe_return_visited_camera_origins',
        '_observe_return_initial_stamp')})


def test_real_zero_materialization_commits_once_cancels_request_and_preserves_history(configured, monkeypatch):
    node, prepared = refresh_context(configured, monkeypatch)
    before = stable_state(node); original_surface, reference = node._observe_return_saved
    with pytest.raises(base.StreamResultCancelled, match='TARGET_PROCESSING_PENDING'):
        node._observe_return_refresh_measurement(prepared)
    assert stable_state(node) == before
    assert node._observe_return_saved[1] is reference
    assert node._observe_return_saved[0].view_stamps_ns == (reference.stamp_ns,prepared.snapshot.stamp_ns)
    assert original_surface.view_stamps_ns == (reference.stamp_ns,)
    assert node._observation_source_refresh_pending is None
    trigger = node._observe_return_trigger_evidence
    assert trigger['policy'] == 'current_request_measured_view_deficit_preflight'
    assert trigger['materialized_count'] == 0 and not trigger['contact_search_completed']
    assert trigger['source_refresh']['original_trigger_evidence']['materialized_count'] == 16
    assert trigger['source_refresh']['negative_search']['search_complete'] is False
    digest = dict(trigger); expected = digest.pop('evidence_sha256')
    assert evidence_digest(digest) == expected
    assert node._observation_source_refresh_profiles
    json.dumps(trigger, allow_nan=False)
    with pytest.raises(base.StreamResultCancelled, match='TARGET_PROCESSING_PENDING'):
        node._observe_return_refresh_measurement(prepared)


@pytest.mark.parametrize('fault', ['candidate','count','bool_count','no_gap','deferred','wrong_phase','disabled','registration'])
def test_nonqualifying_recheck_keeps_prior_measurement_and_is_consumed_once(configured, monkeypatch, fault):
    node, prepared = refresh_context(configured, monkeypatch)
    saved = node._observe_return_saved; trigger = node._observe_return_trigger_evidence
    original = remote.materialize_measured_source
    calls = []
    def compute(*args, **kwargs):
        calls.append(1)
        candidates, diagnostics, profiles, uncertainty, gaps = original(*args, **kwargs)
        if fault == 'candidate': candidates=(object(),)
        elif fault == 'count': diagnostics['materialized_count']=1
        elif fault == 'bool_count': diagnostics['materialized_count']=False
        elif fault == 'no_gap': gaps=()
        elif fault == 'deferred': diagnostics['contact_execution_gate_deferred']=True
        elif fault == 'wrong_phase': diagnostics['plan_phase']='FAR_FIELD_OBSERVATION_PLAN'
        elif fault == 'disabled': diagnostics['enabled']=False
        return candidates, diagnostics, profiles, uncertainty, gaps
    monkeypatch.setattr(remote,'materialize_measured_source',compute)
    if fault == 'registration':
        monkeypatch.setattr(remote,'registered_measurement',lambda *a,**k: (_ for _ in ()).throw(ValueError('rejected')))
    assert node._observe_return_refresh_measurement(prepared) == (prepared.geometry, None)
    assert node._observe_return_saved is saved and node._observe_return_trigger_evidence is trigger
    assert node._observation_source_refresh_pending is None
    assert node._observe_return_refresh_measurement(prepared) == (prepared.geometry, None)
    assert len(calls) == (0 if fault == 'registration' else 1)


@pytest.mark.parametrize('change', ['epoch','selection','phase','control_epoch','raw_owner','raw_content',
    'source_owner','config','saved_owner','deadline','authority'])
def test_compute_completion_cannot_commit_across_any_owner_change(configured, monkeypatch, change):
    node, prepared = refresh_context(configured, monkeypatch)
    saved = node._observe_return_saved; trigger = node._observe_return_trigger_evidence
    original = remote.materialize_measured_source
    def raced(*args,**kwargs):
        result = original(*args,**kwargs)
        if change == 'epoch': node.target_instance_epoch += 1
        elif change == 'selection': node._mode_selection['generation']+=1
        elif change == 'phase': node._observe_return_phase='observing_contact'
        elif change == 'control_epoch': node._measurement_remote_epoch_ns=999
        elif change == 'raw_owner': node._latest_raw_geometry_source=dict(node._latest_raw_geometry_source)
        elif change == 'raw_content': node._latest_raw_geometry_source['transform'][0,3]+=.01
        elif change == 'source_owner': node._diagnostic_observation_view_source=dict(node._diagnostic_observation_view_source)
        elif change == 'config': node.tabletop_geometry_config=replace(node.tabletop_geometry_config,max_candidates=2)
        elif change == 'saved_owner': node._observe_return_saved=tuple(list(saved))
        elif change == 'deadline': node._near_field_phase_deadline_sec+=1
        elif change == 'authority': node._execution_authority_ticket=object()
        return result
    monkeypatch.setattr(remote,'materialize_measured_source',raced)
    with pytest.raises(base.StreamResultCancelled, match='TARGET_PROCESSING_PENDING'):
        node._observe_return_refresh_measurement(prepared)
    assert node._observe_return_trigger_evidence is trigger
    assert node._observation_source_refresh_barrier is None
    assert node._observe_return_saved[0] is saved[0]


def test_successor_registers_current_geometry_without_relabeling_or_renewing_trigger(configured, monkeypatch):
    node, prepared = refresh_context(configured, monkeypatch)
    with pytest.raises(base.StreamResultCancelled): node._observe_return_refresh_measurement(prepared)
    trigger = node._observe_return_trigger_evidence; saved=node._observe_return_saved
    before=stable_state(node)
    observation = replace(prepared.target_observation, stamp_ns=prepared.snapshot.stamp_ns+1_000_000_000)
    snapshot = NS(**dict(vars(prepared.snapshot),stamp_ns=observation.stamp_ns))
    ticket = NS(**dict(vars(prepared.ticket),request_id=prepared.ticket.request_id+1))
    successor = NS(**dict(vars(prepared),snapshot=snapshot,ticket=ticket,target_observation=observation))
    node._latest_target_observation=observation
    node._latest_raw_geometry_source=dict(snapshot=snapshot,observation=observation,
        geometry=prepared.geometry,transform=prepared.pose_estimator._T_base_optical)
    node._diagnostic_observation_view_prepared=successor
    node._diagnostic_observation_view_source={'binding':request_binding(ticket,snapshot)}
    result, enclosure=node._observe_return_refresh_measurement(successor)
    assert result is not successor.geometry and result.ok
    assert enclosure['snapshot_stamp_ns']==snapshot.stamp_ns
    assert node._observe_return_saved is saved
    assert node._observe_return_trigger_evidence is trigger and stable_state(node)==before
    assert node._latest_registration_evidence.stamp_ns==snapshot.stamp_ns
    monkeypatch.setattr(remote,'registered_measurement',lambda *a,**k: (_ for _ in ()).throw(ValueError('rejected')))
    with pytest.raises(base.StreamResultCancelled,match='TARGET_PROCESSING_PENDING'):
        node._observe_return_refresh_measurement(successor)


@pytest.mark.parametrize('fault', ['session','supplement','phase','preview','authority','precision','selection','paused'])
def test_refresh_is_limited_to_initial_measured_observation_scope(configured, monkeypatch, fault):
    node, prepared = refresh_context(configured, monkeypatch)
    if fault == 'session': node._observe_return_session={'phase':'observing'}
    elif fault == 'supplement': node._observe_return_pending_supplement={'view_index':2}
    elif fault == 'phase': node._observe_return_phase='observing_contact'
    elif fault == 'preview': node.latest_preview_rich_plan=NS(valid=True)
    elif fault == 'authority': node._execution_authority_ticket=object()
    elif fault == 'precision': node._observe_return_trigger='BOX_INFERENCE_PRECISION_VIEW_REQUIRED'
    elif fault == 'selection': node._mode_selection['strategy']='direct'
    else: node._observe_return_paused=True
    saved=node._observe_return_saved;trigger=node._observe_return_trigger_evidence
    monkeypatch.setattr(remote,'registered_measurement',lambda *a,**k:pytest.fail('ineligible scope entered ICP'))
    assert node._observe_return_refresh_measurement(prepared) == (prepared.geometry, None)
    assert node._observe_return_saved is saved and node._observe_return_trigger_evidence is trigger


@pytest.mark.parametrize('mode', ['returned','propagated','owner_changed','positive_expired','preview'])
def test_actual_original_source_reserve_search_arms_only_its_current_negative_owner(
        configured, monkeypatch, tmp_path, mode):
    from test_failed_observation_audit import reserve_context, expire_during_original_strict
    node, prepared, checks, variant, source, clock = reserve_context(configured, monkeypatch,tmp_path)
    materializer_configuration(node)
    original_deadlines=stable_state(node)
    change=((lambda: setattr(node,'_measurement_remote_epoch_ns',999)) if mode=='owner_changed'
        else (lambda:setattr(node,'latest_preview_rich_plan',NS(valid=True))) if mode=='preview' else None)
    expire_during_original_strict(node,prepared,checks,clock,monkeypatch,
        success=mode=='positive_expired',change=change)
    if mode == 'propagated':
        original_checker=node._check_moveit_stable_candidate
        def completed_then_cancel(seed):
            result=original_checker(seed)
            assert result.reachable is False and result.failure_code=='OBSERVATION_SEARCH_INCOMPLETE'
            raise base.StreamResultCancelled('OBSERVATION_SOURCE_RESERVE_REACHED')
        monkeypatch.setattr(node,'_check_moveit_stable_candidate',completed_then_cancel)
    with pytest.raises((base.CandidateContractError,base.StreamResultCancelled)) as caught:
        node._accept_prediction(prepared)
    assert caught.value.code=='OBSERVATION_SOURCE_RESERVE_REACHED'
    assert len(checks)==1
    pending=node._observation_source_refresh_pending
    if mode in ('owner_changed','preview'):
        assert pending is None
    else:
        assert pending['request_id']==prepared.ticket.request_id
        assert pending['snapshot_stamp_ns']==prepared.snapshot.stamp_ns
        assert pending['search_complete'] is False
        assert pending['search_cancelled'] is (mode in ('propagated','positive_expired'))
        assert pending['result_known'] is (mode=='returned')
        assert pending['reachable'] is (False if mode=='returned' else None)
        assert pending['owner_sha256']==node._observation_refresh_owner()
    assert not any(plan.valid for plan in node.rich_plan_pub.messages)
    assert stable_state(node)==original_deadlines


def test_actual_checker_entry_source_reserve_is_unknown_cancelled_receipt_not_negative(
        configured,monkeypatch,tmp_path):
    from test_failed_observation_audit import reserve_context
    node,prepared,checks,_,_,clock=reserve_context(configured,monkeypatch,tmp_path)
    materializer_configuration(node)
    original=node._check_moveit_stable_candidate
    def expire_before_original_checker(seed):
        clock.now=prepared.ticket.snapshot_stamp_sec+90.
        return original(seed)
    monkeypatch.setattr(node,'_check_moveit_stable_candidate',expire_before_original_checker)
    with pytest.raises(base.CandidateContractError) as caught:
        node._accept_prediction(prepared)
    assert caught.value.code=='OBSERVATION_SOURCE_RESERVE_REACHED'
    assert checks==[]
    receipt=node._observation_source_refresh_pending
    assert receipt['search_cancelled'] and receipt['search_returned'] is False
    assert receipt['result_known'] is False and receipt['reachable'] is None
    assert receipt['search_complete'] is False and receipt['unchecked_variant_count'] is None
    assert not any(plan.valid for plan in node.rich_plan_pub.messages)


@pytest.mark.parametrize('fault', ['preview','authority','executing','selection'])
def test_committed_refresh_never_returns_raw_geometry_after_scope_becomes_unavailable(
        configured, monkeypatch, fault):
    node,prepared=refresh_context(configured,monkeypatch)
    with pytest.raises(base.StreamResultCancelled):node._observe_return_refresh_measurement(prepared)
    if fault=='preview':node.latest_preview_rich_plan=NS(valid=True)
    elif fault=='authority':node._execution_authority_ticket=object()
    elif fault=='executing':node.robot_execution_active=True
    else:node._selection_is_current=lambda:False
    with pytest.raises(base.StreamResultCancelled,match='TARGET_PROCESSING_PENDING'):
        node._observe_return_refresh_measurement(prepared)


def test_prepared_successor_binds_enclosure_audit_through_original_geometry_activation(
        configured,monkeypatch,tmp_path):
    from test_silhouette_observation_planning import context
    node,old,_,_=context(configured,monkeypatch,tmp_path,ordinary=True)
    enclosure=dict(snapshot_stamp_ns=old.snapshot.stamp_ns,request_id=old.ticket.request_id,
        coordinate_frame='current_snapshot_base_estimate',prior_trigger_gaps_are_historical=True,
        source_view_stamps_ns=[10_000_000_000,old.snapshot.stamp_ns],contact_or_motion_authorized=False)
    enclosed=replace(old.geometry,size_xyz_m=old.geometry.size_xyz_m+[.001,.002,0.])
    # This test isolates the real prepare/bind/activate data handoff. Numerical
    # registration and all-point enclosure are exercised above without doubles.
    monkeypatch.setattr(node,'_observe_return_refresh_measurement',lambda p:(enclosed,enclosure))
    binds=[]
    monkeypatch.setattr(node,'_bind_diagnostic_observation_view_source',lambda p,c:binds.append(p))
    successor=node._prepare_diagnostic_observation_prediction(old.ticket)
    assert len(binds)==2 and binds[-1] is successor and binds[0].geometry is old.geometry
    assert successor.geometry is enclosed
    assert dict(successor.fused_scene_audit)['snapshot_stamp_ns']==old.snapshot.stamp_ns
    monkeypatch.setattr(node,'_activate_geometry',lambda *a,**k:True)
    monkeypatch.setattr(node,'_freeze_tool_from_camera_matrix',lambda *a:None)
    assert base.RemoteGrasp6DNode._activate_prepared_geometry(node,successor)
    assert node._latest_fused_scene_audit==enclosure
    assert node._request_multiview_surface_audit(successor)['fused_scene_geometry']==enclosure
