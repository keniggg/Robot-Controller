"""The live adapter must freeze raw input before remote inference starts."""
from copy import deepcopy
from dataclasses import replace
import json
from types import SimpleNamespace as NS

import numpy as np
import pytest

from test_remote_modes import configured, module, base
from test_silhouette_snapshot_diagnostic import bound_inputs
from test_silhouette_observation_remote import request


def runtime(configured, monkeypatch, retained=True):
    node, params = configured
    args = bound_inputs()
    if not retained:
        args['snapshot'] = replace(args['snapshot'], original_sample=None)
        args['ticket'] = replace(args['ticket'], payload=(args['snapshot'], None))
    node._mode_selection = deepcopy(args['selection'])
    node._observe_return_phase = 'initial_contact'
    intrinsics = args['intrinsics']
    params['/camera'] = dict(width=320, height=240, frame_id='camera_link', align_depth_to_color=True,
        **{key: getattr(intrinsics, key) for key in ('fx', 'fy', 'cx', 'cy', 'depth_scale')})
    monkeypatch.setattr(node, '_camera_intrinsics', lambda: intrinsics)
    monkeypatch.setattr(node, '_adaptive_stage_profiles', lambda *_: [NS(depth_uncertainty_m=.003)])
    calls = []
    node.frames = NS(wait_for_exact_sample=lambda *a, **kw: calls.append((a, kw)) or deepcopy(args['sample']))
    node._silhouette_source_status_cb(NS(data=json.dumps(args['config'].perception_status)))
    prepared = base.PreparedPrediction(ticket=args['ticket'], snapshot=args['snapshot'], stamp=None,
        geometry=args['geometry'], pose_estimator=NS(_T_base_optical=args['transform_base_optical']),
        graspnet_input=object(), candidates=(object(),), remote_diagnostics={'original': [1,2]},
        remote_performance={}, tabletop_candidates=(object(),))
    return node, args, prepared, calls


@pytest.mark.parametrize('change', ['K', 'activation', 'inflight_context'])
def test_diagnostic_capture_rejects_changed_camera_without_affecting_geometry(configured, monkeypatch, change):
    from test_silhouette_snapshot_diagnostic import rectified_diagnostic_camera
    node, args, prepared, _ = runtime(configured, monkeypatch)
    params = configured[1]
    camera = rectified_diagnostic_camera(args)
    camera['runtime_profile'] = camera.pop('runtime_profile_geometry')
    params['/camera'] = camera
    if change == 'K':
        camera['fx'] += 1
    elif change == 'activation':
        camera['runtime_profile']['geometry_activation_stamp_ns'] = args['sample'].stamp_ns
    else:
        def changed():
            camera['runtime_profile']['geometry_activation_stamp_ns'] += 1
            return args['intrinsics']
        monkeypatch.setattr(node, '_camera_intrinsics', changed)
    source = node._capture_silhouette_source(args['ticket'])
    assert source == {'error': 'SILHOUETTE_ORIGINAL_SOURCE_UNAVAILABLE'}
    actual = node._add_silhouette_source_diagnostic(prepared, source)
    report = base._thaw_request_evidence(actual.remote_diagnostics)['silhouette_ambiguity']
    assert not report['ok']
    assert actual.geometry is prepared.geometry and actual.candidates is prepared.candidates


def test_diagnostic_capture_freezes_camera_context_before_remote_latency(configured, monkeypatch):
    node, args, _, _ = runtime(configured, monkeypatch)
    params = configured[1]
    source = node._capture_silhouette_source(args['ticket'])
    before = deepcopy(source['camera_geometry'])
    params['/camera']['fx'] += 10
    assert source['camera_geometry'] == before
    assert source['camera_geometry']['fx'] == args['intrinsics'].fx


@pytest.mark.parametrize('retained', [True, False])
def test_capture_precedes_remote_call_and_attaches_only_immutable_diagnostics(configured, monkeypatch, retained):
    node, args, prepared, calls = runtime(configured, monkeypatch, retained)
    order = []
    monkeypatch.setattr(node, '_require_stream_ticket_current', lambda ticket: order.append('current'))
    def predict(self, ticket):
        assert len(calls) == (0 if retained else 1)
        order.append('predict')
        # Later source messages must not substitute for the frozen original.
        node.frames.wait_for_exact_sample = lambda *_a, **_k: pytest.fail('resampled after inference')
        status = deepcopy(args['config'].perception_status); status['source_stamp_ns'] += 1
        node._silhouette_source_status_cb(NS(data=json.dumps(status)))
        return prepared
    monkeypatch.setattr(base.RemoteGrasp6DNode, '_prepare_and_predict', predict)
    actual = node._prepare_and_predict(args['ticket'])
    assert order == ['current', 'predict', 'current', 'current']
    diag = base._thaw_request_evidence(actual.remote_diagnostics)['silhouette_ambiguity']
    assert diag['ok'], diag
    assert diag['source_stamp_ns'] == args['sample'].stamp_ns
    if retained:
        assert calls == []
        assert args['snapshot'].original_sample is not None
    else:
        assert calls[0][0] == (args['snapshot'].stamp_ns, args['snapshot'].target_identity)
        assert calls[0][1]['timeout_sec'] == 0.
        assert calls[0][1]['max_age_sec'] == node.planning_snapshot_max_age_sec
        assert calls[0][1]['max_inference_latency_sec'] == node.planning_snapshot_max_inference_latency_sec
    assert actual.geometry is prepared.geometry and actual.candidates is prepared.candidates
    assert actual.tabletop_candidates is prepared.tabletop_candidates
    assert actual.graspnet_input is prepared.graspnet_input
    with pytest.raises(TypeError): actual.remote_diagnostics['silhouette_ambiguity']['ok'] = False


@pytest.mark.parametrize('mode,strategy,phase', [('carton','observe_return','initial_contact'),
    ('unknown','direct','initial_contact'), ('unknown','two_stage','initial_contact'),
    ('unknown','observe_return','observation_prefix'), ('unknown','observe_return','returned_contact')])
def test_other_modes_and_observation_return_phases_never_capture_a_diagnostic_source(configured, monkeypatch, mode,strategy,phase):
    node,args,_,calls = runtime(configured,monkeypatch)
    node._mode_selection.update(mode=mode,strategy=strategy); node._observe_return_phase=phase
    assert node._capture_silhouette_source(args['ticket']) is None
    assert calls == []


@pytest.mark.parametrize('retained', [True, False])
@pytest.mark.parametrize('fault',['missing_sample','missing_status','wrong_status_stamp','wrong_status_generation','changed_mask'])
def test_missing_or_changed_original_source_does_not_borrow_newer_frame_or_status(configured,monkeypatch,fault,retained):
    node,args,prepared,calls=runtime(configured,monkeypatch,retained)
    source=node._capture_silhouette_source(args['ticket'])
    if fault=='missing_sample': source['sample']=None
    elif fault=='missing_status': node._silhouette_source_status.clear()
    elif fault=='wrong_status_stamp': node._silhouette_source_status[source['sample'].stamp_ns]['source_stamp_ns']+=1
    elif fault=='wrong_status_generation': node._silhouette_source_status[source['sample'].stamp_ns]['generation']+=1
    elif fault=='changed_mask':
        corrupt = source['sample'].object_mask.copy()
        corrupt[120,150] ^= 255
        source['sample'] = replace(source['sample'], object_mask=corrupt)
    actual=node._add_silhouette_source_diagnostic(prepared,source)
    diag=base._thaw_request_evidence(actual.remote_diagnostics)['silhouette_ambiguity']
    assert not diag['ok'] and not diag['execution_authority'] and not diag['observation_authority']
    assert len(calls)==(0 if retained else 1)
    assert actual.geometry is prepared.geometry and actual.candidates is prepared.candidates


def test_status_cache_is_bounded_and_retains_source_identity_without_relabeling(configured):
    node,_=configured
    for ns in range(1,101):
        node._silhouette_source_status_cb(NS(data=json.dumps(dict(source_stamp_ns=ns,
            state='ready',mode='unknown',target_lost=False))))
    assert len(node._silhouette_source_status)==64
    assert min(node._silhouette_source_status)==37
    node._silhouette_source_status_cb(NS(data='not json'))
    node._silhouette_source_status_cb(NS(data=json.dumps(dict(source_stamp_ns=101,
        state='lost',mode='unknown',target_lost=True))))
    assert max(node._silhouette_source_status)==100


def test_frozen_prepared_prediction_measurement_is_thawed_only_for_validation(configured):
    node,prepared,_=request(configured)
    prepared.remote_diagnostics=base._freeze_request_evidence(prepared.remote_diagnostics)
    opportunity=node._observe_return_silhouette_failure(prepared,'NEAR_FIELD_NO_HARD_SAFE_CANDIDATE')
    assert opportunity is not None
    assert opportunity['diagnostic_observation_only']
    with pytest.raises(TypeError): prepared.remote_diagnostics['silhouette_ambiguity']['source_stamp_ns']=0


def observation_context(configured,monkeypatch):
    from test_silhouette_observation_remote import enter_diagnostic,collision_geometry,source_status
    node,p,_=enter_diagnostic(configured,monkeypatch)
    ns=p.snapshot.stamp_ns+100000000
    snapshot=NS(stamp_ns=ns,target_identity=p.snapshot.target_identity,
        target_epoch=p.ticket.target_epoch,frame_id='camera_link',source_mode='instance_mask')
    fresh=collision_geometry((.054,.041,.020));depth=np.zeros((3,3));transform=np.eye(4)
    calls=[]
    monkeypatch.setattr(base.RemoteGrasp6DNode,'_prepare_snapshot_geometry',
        lambda self,*args: calls.append(args) or (fresh,depth,transform))
    status=source_status(node._mode_selection,ns,p.snapshot.stamp_ns)
    node._silhouette_source_status_cb(NS(data=json.dumps(status)))
    return node,p,snapshot,fresh,depth,transform,calls


def test_observation_preparation_uses_fresh_checks_then_expanded_original_collision_body(configured,monkeypatch):
    node,p,snapshot,fresh,depth,transform,calls=observation_context(configured,monkeypatch)
    frozen=node._observe_return_diagnostic_collision_geometry
    actual,d,t=node._prepare_snapshot_geometry(snapshot,None)
    assert calls==[(snapshot,None)]
    assert actual.ok and d is depth and t is transform
    assert actual.size_xyz_m[1] >= frozen.size_xyz_m[1] > fresh.size_xyz_m[1]
    np.testing.assert_array_equal(actual.object_points_base,fresh.object_points_base)
    np.testing.assert_array_equal(actual.axes_base,fresh.axes_base)
    assert not actual.object_points_base.flags.writeable
    audit=node._silhouette_observation_envelope_audit
    assert audit['source']==dict(original_stamp_ns=p.snapshot.stamp_ns,fresh_stamp_ns=snapshot.stamp_ns)
    assert audit['containment']['original_corners_preserved']
    assert audit['containment']['original_measured_points_preserved']
    assert audit['measured_points']['synthetic_points_added']==0
    ticket=NS(request_id=77,generation=node._stream_generation,target_epoch=snapshot.target_epoch)
    prepared=base.PreparedPrediction(ticket=ticket,snapshot=snapshot,stamp=None,geometry=actual,
        pose_estimator=object(),graspnet_input=object(),candidates=(),remote_diagnostics={},remote_performance={})
    prepared=node._add_silhouette_envelope_diagnostic(prepared)
    report={};node._observe_return_audit(report,prepared)
    assert report['observe_return']['initial_surface_stamp_ns']==p.snapshot.stamp_ns
    assert report['observe_return']['collision_envelope']['fresh_request_id']==77
    assert report['observe_return']['collision_envelope']['target_track_id']==snapshot.target_identity.track_id


@pytest.mark.parametrize('fault',['missing_status','old_status','lost','source','generation','reference_anchor',
    'reference_plane','reference_footprint','origin_track','origin_epoch','same_stamp','fresh_identity','support_shift'])
def test_observation_envelope_does_not_replace_freshness_identity_or_original_support_reference(configured,monkeypatch,fault):
    node,p,snapshot,fresh,depth,transform,calls=observation_context(configured,monkeypatch)
    status=node._silhouette_source_status[snapshot.stamp_ns]
    if fault=='missing_status':node._silhouette_source_status.clear()
    elif fault=='old_status':status['source_stamp_ns']-=1
    elif fault=='lost':status['target_lost']=True
    elif fault=='source':status['tf_lookup_stamp_ns']+=1
    elif fault=='generation':status['generation']+=1
    elif fault=='reference_anchor':status['anchor_source_stamp_ns']+=1
    elif fault=='reference_plane':status['reference_support_plane_base'][3]+=.001
    elif fault=='reference_footprint':status['reference_support_footprint_base'][0][0]+=.001
    elif fault=='origin_track':node._observe_return_diagnostic_collision_source['target_track_id']='different'
    elif fault=='origin_epoch':node._observe_return_diagnostic_collision_source['target_epoch']+=1
    elif fault=='same_stamp':snapshot.stamp_ns=p.snapshot.stamp_ns
    elif fault=='fresh_identity':snapshot.target_identity=None
    elif fault=='support_shift':
        bad=replace(fresh,support_offset_m=.0041)
        monkeypatch.setattr(base.RemoteGrasp6DNode,'_prepare_snapshot_geometry',lambda *_:(bad,depth,transform))
    actual,_,_=node._prepare_snapshot_geometry(snapshot,None)
    assert not actual.ok and actual.failure_code=='SILHOUETTE_OBSERVATION_ENVELOPE_INVALID'
    assert node._silhouette_observation_envelope_audit is None


@pytest.mark.parametrize('phase',['initial_contact','observing','returned_contact'])
def test_collision_union_is_exclusively_diagnostic_observation_prefix(configured,monkeypatch,phase):
    node,_,snapshot,fresh,depth,transform,calls=observation_context(configured,monkeypatch)
    node._observe_return_phase=phase
    actual,_,_=node._prepare_snapshot_geometry(snapshot,None)
    assert actual is fresh
