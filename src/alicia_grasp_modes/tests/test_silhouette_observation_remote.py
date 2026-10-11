"""Request-bound diagnostic observation does not grant any contact authority."""
from copy import deepcopy
import json
from types import SimpleNamespace as NS

import numpy as np
import pytest

from test_remote_modes import configured, module, base
from test_observe_return_remote import setup_observe_return, prefix_session, send
from test_observe_return_visual import camera_rotation_toward
from test_runner_observe_return import observe_evidence
from test_silhouette_ambiguity import true_multicolour_object, NAMES
from test_silhouette_observation import silhouette_evidence as source_evidence_fixture
from alicia_grasp_modes.silhouette_observation_geometry import conservative_silhouette_observation_geometry
from alicia_grasp_modes.silhouette_ambiguity import evaluate_silhouette_ambiguity
from alicia_grasp_modes.runner_contract import observe_return_audit_error
from alicia_grasp_modes.silhouette_observation import silhouette_view_gain


def silhouette_evidence(selection, request_id, generation, epoch, track, stamp):
    inputs, options, _, _ = true_multicolour_object()
    options['source_stamps_ns'] = {key: stamp for key in NAMES}
    measurement = evaluate_silhouette_ambiguity(*inputs, **options)
    center = np.array([0., 0., .020])
    camera = np.array([0., -.15, .15])
    result=source_evidence_fixture()
    result.update(selection=deepcopy(selection),request_id=request_id,generation=generation,
        target_epoch=epoch,target_track_id=track,snapshot_stamp_ns=stamp,source_stamp_ns=stamp,
        snapshot_source_stamps_ns=[stamp],measurement=measurement)
    result['plane_provenance'].update(verification_stamp_ns=stamp,anchor_source_stamp_ns=stamp)
    result['view_geometry'].update(target_center_base_m=center.tolist(),ambiguity_anchor_base_m=[.025,0.,.006],
        initial_camera_origin_base_m=camera.tolist(),
        initial_camera_rotation_base=camera_rotation_toward(center,camera).tolist(),
        target_height_m=.020,depth_uncertainty_m=.002,target_geometry_source_stamps_ns=[stamp],
        ambiguity_anchor_source_stamp_ns=stamp)
    result['perception_source_status']=source_status(selection,stamp,stamp)
    return result


def source_status(selection,stamp,anchor_stamp):
    return dict(state='ready',mode='unknown',generation=selection['generation'],
        selection_stamp_ns=selection['stamp_ns'],target_lost=False,exact_rgbd=True,
        source_stamp_ns=stamp,color_stamp_ns=stamp,depth_stamp_ns=stamp,tf_lookup_stamp_ns=stamp,
        color_frame_id='camera_link',depth_frame_id='camera_link',projection_frame_convention='ros_camera_link',
        anchor_source_stamp_ns=anchor_stamp,anchor_position_base=[0.,0.,.02],
        reference_support_plane_base=[0.,0.,1.,0.],
        reference_support_footprint_base=[[-.05,-.05,0.],[-.05,.05,0.],[.05,.05,0.],[.05,-.05,0.]],
        geometry_metrics=dict(support_current_audit_available=True,
            support_reference_max_distance_m=0.,support_reference_angle_deg=0.))


def collision_geometry(size=(.060,.051,.020)):
    return base.GeometryEstimate(True,'','',np.array([0.,0.,.020]),np.eye(3),np.array(size),
        np.array([0.,0.,1.]),0.,1.,np.array([[0.,0.,.02],[.01,0.,.02],[-.01,0.,.02]]),'instance_mask')


def request(configured, code='NEAR_FIELD_NO_HARD_SAFE_CANDIDATE'):
    node, params, clock = setup_observe_return(configured)
    identity = node._current_stream_target_identity()
    stamp = node._active_multiview_surface()[0].view_stamps_ns[0]
    ticket = NS(request_id=76, generation=node._stream_generation, target_epoch=identity.epoch)
    checked = 0 if code == 'NEAR_FIELD_NO_HARD_SAFE_CANDIDATE' else 3
    local = dict(entered=12, passed=0 if not checked else 3, rejected=12 if not checked else 9)
    node._active_gate_audit_report = dict(grasp_mode_selection=deepcopy(node._mode_selection),
        request_id=ticket.request_id, generation=ticket.generation, target_epoch=identity.epoch,
        snapshot_stamp_ns=stamp, selected=None,
        moveit_selection=dict(checked_count=checked, shortlist_count=checked,
            reachable_count=0, terminated_early=False),
        pipeline_funnel=dict(stage_counts=dict(locally_valid=local)))
    evidence = silhouette_evidence(node._mode_selection,ticket.request_id,ticket.generation,
                                  identity.epoch,identity.track_id,stamp)
    prepared = NS(ticket=ticket,snapshot=NS(stamp_ns=stamp,target_identity=identity),
        remote_diagnostics={'silhouette_ambiguity':evidence},remote_failure_code='',
        geometry=collision_geometry())
    return node,prepared,clock


@pytest.mark.parametrize('code',['NEAR_FIELD_NO_HARD_SAFE_CANDIDATE','NEAR_FIELD_NO_REACHABLE_CANDIDATE'])
def test_complete_failed_initial_search_admits_ambiguity_even_for_oversized_original_obb(configured,code):
    node,p,_=request(configured,code)
    before=deepcopy(p.remote_diagnostics)
    opportunity=node._observe_return_silhouette_failure(p,code)
    assert opportunity is not None
    assert opportunity['diagnostic_observation_only']
    assert not opportunity['contact_or_motion_authorized']
    assert opportunity['initial_contact_search']['checked_count']==(0 if 'HARD_SAFE' in code else 3)
    assert p.remote_diagnostics==before
    np.testing.assert_array_equal(p.geometry.size_xyz_m,[.060,.051,.020])


@pytest.mark.parametrize('fault',['selection','request','generation','epoch','stamp','winner','reachable',
    'unchecked','terminated','local_pass','local_missing','bool_count','server','phase','strategy','timeout',
    'input_source','input_hash','wrong_identity','attempted','no_evidence'])
def test_no_incomplete_foreign_or_consumed_request_can_trigger_diagnostic_observation(configured,fault):
    node,p,_=request(configured);report=node._active_gate_audit_report
    code='NEAR_FIELD_NO_HARD_SAFE_CANDIDATE'
    if fault=='selection':report['grasp_mode_selection']['generation']+=1
    elif fault in ('request','generation','epoch','stamp'):
        key=dict(request='request_id',generation='generation',epoch='target_epoch',stamp='snapshot_stamp_ns')[fault]
        report[key]+=1
    elif fault=='winner':report['selected']={'plan_id':'successful-contact'}
    elif fault=='reachable':report['moveit_selection']['reachable_count']=1
    elif fault=='unchecked':report['moveit_selection']['shortlist_count']=1
    elif fault=='terminated':report['moveit_selection']['terminated_early']=True
    elif fault=='local_pass':report['pipeline_funnel']['stage_counts']['locally_valid'].update(passed=1,rejected=11)
    elif fault=='local_missing':report['pipeline_funnel']['stage_counts']['locally_valid']['rejected']-=1
    elif fault=='bool_count':report['moveit_selection']['checked_count']=False
    elif fault=='server':p.remote_failure_code='WSL_UNAVAILABLE'
    elif fault=='phase':node._observe_return_phase='observing_contact'
    elif fault=='strategy':node._mode_selection['strategy']='direct'
    elif fault=='timeout':code='NEAR_FIELD_DIRECT_TIMEOUT'
    elif fault=='input_source':p.remote_diagnostics['silhouette_ambiguity']['source_stamp_ns']-=1
    elif fault=='input_hash':p.remote_diagnostics['silhouette_ambiguity']['measurement']['input_evidence']={}
    elif fault=='wrong_identity':p.remote_diagnostics['silhouette_ambiguity']['target_track_id']='another-target'
    elif fault=='attempted':node._observe_return_diagnostic_attempted=True
    elif fault=='no_evidence':p.remote_diagnostics={}
    assert node._observe_return_silhouette_failure(p,code) is None


def enter_diagnostic(configured,monkeypatch):
    node,p,clock=request(configured)
    # Fixture supplies a namespace ticket; production ticket currentness is
    # independently checked by the existing streaming tests. Record the check.
    calls=[]
    monkeypatch.setattr(node,'_require_stream_ticket_current_locked',lambda ticket:calls.append(ticket))
    before=node._active_multiview_surface()[0]
    opportunity=node._observe_return_silhouette_failure(p,'NEAR_FIELD_NO_HARD_SAFE_CANDIDATE')
    assert node._observe_return_silhouette_fallback(p,opportunity)
    assert calls==[p.ticket]
    assert node._observe_return_saved[0] is before
    assert node._observe_return_view_gaps==()
    assert node._observe_return_phase=='observation_prefix'
    assert node._observe_return_diagnostic_attempted
    assert node._observe_return_diagnostic_observation_only
    return node,p,clock


def test_transition_preserves_all_original_surface_and_freezes_one_source(configured,monkeypatch):
    node,p,_=enter_diagnostic(configured,monkeypatch)
    frozen=deepcopy(node._observe_return_trigger_evidence)
    p.remote_diagnostics['silhouette_ambiguity']['source_stamp_ns']+=1
    assert node._observe_return_trigger_evidence==frozen
    assert not node._observe_return_silhouette_failure(p,'NEAR_FIELD_NO_HARD_SAFE_CANDIDATE')
    report={};node._observe_return_audit(report,p)
    meta=report['observe_return']
    assert meta['trigger_code']=='RGB_DEPTH_SILHOUETTE_AMBIGUOUS'
    assert meta['diagnostic_observation_only']
    assert meta['maximum_diagnostic_observations']==1
    assert not meta['contact_and_return_authorized']
    assert meta['trigger_evidence']==frozen
    assert json.loads(json.dumps(report))==report


def test_frozen_opportunity_is_rechecked_before_transition(configured):
    node,p,_=request(configured)
    opportunity=node._observe_return_silhouette_failure(p,'NEAR_FIELD_NO_HARD_SAFE_CANDIDATE')
    node._active_gate_audit_report['selected']={'plan_id':'winner-arrived'}
    assert not node._observe_return_silhouette_fallback(p,opportunity)
    assert node._observe_return_phase=='initial_contact'
    assert not node._observe_return_diagnostic_attempted


def test_diagnostic_session_remains_paused_and_cannot_start_contact_or_return(configured,monkeypatch):
    node,p,clock=enter_diagnostic(configured,monkeypatch)
    session=prefix_session(node);clock[0]=15_000_000_000
    assert send(node,session)['accepted']
    assert node._observe_return_phase=='observing' and node._observe_return_paused
    frozen=node._observe_return_saved
    node._observe_return_near_field_cb(NS(active=True))
    assert node._observe_return_phase=='observing' and node._observe_return_paused
    assert node._observe_return_saved is frozen
    assert not node._observe_return_ready_contact({})
    clock[0]=17_000_000_000
    ack=send(node,dict(session,phase='returning',stamp_ns=16_000_000_000))
    assert not ack['accepted']
    assert ack['reason']=='DIAGNOSTIC_OBSERVATION_FORBIDS_CONTACT_AND_RETURN'


def diagnostic_report():
    selection,report,preview=observe_evidence('observation_prefix')
    meta=report['observe_return'];stamp=meta['initial_surface_stamp_ns']
    proof=dict(request_id=76,generation=13,target_epoch=meta['target_epoch'],snapshot_stamp_ns=stamp,
        contact_search_code='NEAR_FIELD_NO_HARD_SAFE_CANDIDATE',selected=False,terminated_early=False,
        checked_count=0,shortlist_count=0,reachable_count=0,locally_valid=dict(entered=12,passed=0,rejected=12),
        contact_or_motion_authorized=False)
    evidence=silhouette_evidence(selection,76,13,meta['target_epoch'],meta['target_track_id'],stamp)
    meta.update(trigger_code='RGB_DEPTH_SILHOUETTE_AMBIGUOUS',diagnostic_observation_only=True,
        maximum_diagnostic_observations=1,diagnostic_observation_attempted=True,contact_and_return_authorized=False,
        trigger_evidence=dict(policy='current_request_silhouette_initialization_ambiguity',
            initial_contact_search=proof,silhouette_ambiguity=evidence,
            diagnostic_observation_only=True,contact_or_motion_authorized=False))
    old=collision_geometry();fresh=collision_geometry((.056,.042,.020))
    combined,envelope=conservative_silhouette_observation_geometry(old,fresh,
        original_source_stamp_ns=stamp,fresh_source_stamp_ns=report['snapshot_stamp_ns'],
        max_size_m=.6,max_height_m=.5)
    report.update(request_id=77,generation=14)
    status=evidence['perception_source_status']
    envelope.update(selection=deepcopy(selection),target_epoch=meta['target_epoch'],
        target_track_id=meta['target_track_id'],original_request_id=76,original_generation=13,
        fresh_request_id=77,fresh_generation=14,perception_source_stamp_ns=report['snapshot_stamp_ns'],
        perception_reference={key:deepcopy(status[key]) for key in ('anchor_source_stamp_ns',
            'anchor_position_base','reference_support_plane_base','reference_support_footprint_base')})
    meta['collision_envelope']=envelope
    report['replay_geometry']=base.RemoteGrasp6DNode._geometry_replay_audit(None,combined)
    camera=np.eye(4);camera[:3,3]=[.20,-.05,.15]
    report['selected']['observation_view']['observe_return_silhouette_gain']=silhouette_view_gain(evidence,camera,.002)
    assert report['selected']['observation_view']['observe_return_silhouette_gain']['ok']
    return report


def test_runner_accepts_only_fully_bound_diagnostic_observation_prefix():
    assert observe_return_audit_error(diagnostic_report())==''


@pytest.mark.parametrize('fault',['missing_marker','wrong_trigger','two_observations','bool_budget','not_consumed',
    'return_authorized','missing_evidence','foreign_selection','different_frame','missing_hash',
    'unfinished_search','winner','local_missing','weak_gain','weak_moveit','bad_distance','contact_stage'])
def test_diagnostic_prefix_cannot_borrow_partial_proof_or_relax_physical_gates(fault):
    report=diagnostic_report();meta=report['observe_return'];trigger=meta['trigger_evidence']
    if fault=='missing_marker':del meta['diagnostic_observation_only']
    elif fault=='wrong_trigger':meta['trigger_code']='BILATERAL_SURFACE_EVIDENCE_MISSING'
    elif fault=='two_observations':meta['maximum_diagnostic_observations']=2
    elif fault=='bool_budget':meta['maximum_diagnostic_observations']=True
    elif fault=='not_consumed':meta['diagnostic_observation_attempted']=False
    elif fault=='return_authorized':meta['contact_and_return_authorized']=True
    elif fault=='missing_evidence':del meta['trigger_evidence']
    elif fault=='foreign_selection':trigger['silhouette_ambiguity']['selection']['generation']+=1
    elif fault=='different_frame':trigger['silhouette_ambiguity']['snapshot_stamp_ns']+=1
    elif fault=='missing_hash':trigger['silhouette_ambiguity']['measurement']['input_evidence']={}
    elif fault=='unfinished_search':trigger['initial_contact_search']['shortlist_count']=1
    elif fault=='winner':trigger['initial_contact_search']['selected']=True
    elif fault=='local_missing':trigger['initial_contact_search']['locally_valid']['rejected']-=1
    elif fault=='weak_gain':report['selected']['observation_view']['observe_return_silhouette_gain']['ok']=False
    elif fault=='weak_moveit':report['selected']['moveit']['collision_free']=False
    elif fault=='bad_distance':report['selected']['observation_view']['actual_camera_target_distance_m']=.35
    elif fault=='contact_stage':report['selected']['execution_sequence']['stages'].append(dict(stage='grasp'))
    assert observe_return_audit_error(report)


@pytest.mark.parametrize('phase',['initial_contact','returned_contact'])
def test_even_complete_contact_audit_cannot_reuse_a_diagnostic_observation_flag(phase):
    _,report,_=observe_evidence(phase)
    report['observe_return'].update(trigger_code='RGB_DEPTH_SILHOUETTE_AMBIGUOUS',diagnostic_observation_only=True)
    assert observe_return_audit_error(report,allow_returned=True)


@pytest.mark.parametrize('fault',['source','epoch','track','registration','authorized','certified','gain','noise','baseline','nan','bool'])
def test_runner_rejects_foreign_or_inconsistent_gain(fault):
    report=diagnostic_report();gain=report['selected']['observation_view']['observe_return_silhouette_gain']
    if fault=='source':gain['source_stamp_ns']+=1
    elif fault=='epoch':gain['target_epoch']+=1
    elif fault=='track':gain['target_track_id']='another-target'
    elif fault=='registration':gain['measured_registration_required']=False
    elif fault=='authorized':gain['contact_or_motion_authorized']=True
    elif fault=='certified':gain['physical_contour_certified']=True
    elif fault=='gain':gain['predicted_gain_m']+=.01
    elif fault=='noise':gain['depth_uncertainty_m']=0.
    elif fault=='baseline':gain['camera_translation_baseline_m']=.014
    elif fault=='nan':gain['predicted_gain_m']=float('nan')
    elif fault=='bool':gain['camera_translation_baseline_m']=True
    assert observe_return_audit_error(report)


@pytest.mark.parametrize('fault',['missing','old_source','fresh_source','epoch','request','generation','box_dropped',
    'replay_shrunk','points_changed','synthetic_points','digest','loose_support','anchor_changed'])
def test_runner_rejects_observation_without_its_actual_conservative_collision_geometry(fault):
    report=diagnostic_report();meta=report['observe_return'];envelope=meta['collision_envelope']
    if fault=='missing':del meta['collision_envelope']
    elif fault=='old_source':envelope['source']['original_stamp_ns']-=1
    elif fault=='fresh_source':envelope['source']['fresh_stamp_ns']+=1
    elif fault=='epoch':envelope['target_epoch']+=1
    elif fault=='request':envelope['fresh_request_id']+=1
    elif fault=='generation':envelope['original_generation']+=1
    elif fault=='box_dropped':envelope['containment']['original_corners_preserved']=False
    elif fault=='replay_shrunk':report['replay_geometry']['obb_size_xyz_m'][1]-=.01
    elif fault=='points_changed':report['replay_geometry']['object_points_sha256']='0'*64
    elif fault=='synthetic_points':envelope['measured_points']['synthetic_points_added']=8
    elif fault=='digest':envelope['original']['geometry_sha256']='0'*64
    elif fault=='loose_support':envelope['support_consistency']['maximum_local_shift_m']=.00701
    elif fault=='anchor_changed':envelope['perception_reference']['anchor_source_stamp_ns']+=1
    assert observe_return_audit_error(report)
