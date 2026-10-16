"""Independent observation source exercises real audit and promotion plumbing."""
from copy import deepcopy
from dataclasses import replace
from types import SimpleNamespace as NS, MethodType
import json

import numpy as np
import pytest

from test_remote_modes import configured, module, base, Publisher
from test_mode_runner import working_volume_view
from test_silhouette_source_runtime import observation_context
from test_silhouette_observation_remote import source_status
from test_observe_return_visual import camera_rotation_toward
from alicia_flexible_grasp.vision.rgbd_snapshot import DepthQuality
from alicia_flexible_grasp.vision.latest_only_inference import InferenceTicket
from alicia_grasp_modes.runner_contract import observe_return_audit_error
from alicia_grasp_modes.silhouette_observation import silhouette_view_gain
from alicia_grasp_modes.silhouette_observation_planning import ObservationPlanningSeed


def context(configured, monkeypatch, tmp_path, ordinary=False):
    if ordinary:
        from test_measured_observation_planning import preflight_context
        node, ticket0, source0, shape, transform, candidates, diagnostics, _ = preflight_context(configured, monkeypatch)
        assert node._maybe_prepare_observation_from_contact_geometry(
            ticket0, source0, shape, transform, candidates, diagnostics)
        initial = NS(ticket=ticket0, snapshot=source0)
        simple_snapshot = NS(stamp_ns=source0.stamp_ns+100000000)
        surface = node._observe_return_saved[0]
        geometry = base.GeometryEstimate(True, '', '', shape.center_base,
            shape.axes_base, shape.size_xyz_m, shape.support_normal_base, 0., 1.,
            surface.points_base, 'instance_mask')
        depth = np.zeros((3, 3)); calls = []
        monkeypatch.setattr(base.RemoteGrasp6DNode, '_prepare_snapshot_geometry',
            lambda self, *args: calls.append(args) or (geometry, depth, transform))
    else:
        node, initial, simple_snapshot, geometry, depth, transform, calls = observation_context(configured, monkeypatch)
    ns = simple_snapshot.stamp_ns
    stamp = module.rospy.Time(ns // 1000000000, ns % 1000000000)
    msg = base.ObjectPose();msg.header.stamp=stamp;msg.header.frame_id='camera_link'
    msg.detected=True;msg.label='unknown'
    snapshot=base.SnapshotResult(True,'','',np.zeros((3,3,3),np.uint8),depth,depth,
        np.ones((3,3),np.uint8)*255,(0,0,3,3),msg,ns/1e9,'camera_link',
        DepthQuality(1,9,9,1.,.25,0.,.001),'instance_mask',ns,
        initial.ticket.target_epoch,initial.snapshot.target_identity,(ns,))
    ticket=InferenceTicket(77,node._stream_generation,ns/1e9,snapshot.target_epoch,
        (snapshot,None),0.)
    # Source and geometry work are exercised below; isolate only ROS transport.
    monkeypatch.setattr(node,'_require_stream_ticket_current',lambda _:None)
    monkeypatch.setattr(node,'_require_stream_ticket_current_locked',lambda _:None)
    monkeypatch.setattr(node,'_refresh_runtime_params',lambda:None)
    monkeypatch.setattr(node,'_gripper_contract_mismatch_reason',lambda:'')
    monkeypatch.setattr(node,'_capture_geometry_generation',lambda:0)
    node._geometry_invalidation_generation=0
    node._last_geometry_invalidation_code=''
    if ordinary:
        node._promotion_now_sec=lambda:12.
        node._execution_plan_validity_now_sec=lambda:12.
        node._stream_source_clock=lambda:12.
    node.candidate_frame_convention='graspnet'
    node.grasp_config=dict(observation_camera_target_nominal_distance_m=.26,
        observation_camera_target_min_distance_m=.22,observation_camera_target_max_distance_m=.30)
    monkeypatch.setattr(node,'_adaptive_stage_profiles',lambda *_:[
        NS(depth_uncertainty_m=.002, execution_position_error_m=0.)])
    # Same-source cloud and real path/FOV are covered by dedicated view tests.
    # This fixture isolates the actual single-source promotion transaction.
    monkeypatch.setattr(node,'_capture_diagnostic_observation_view_source',lambda _:None)
    monkeypatch.setattr(node,'_bind_diagnostic_observation_view_source',lambda *_:None)
    binding = dict(request_id=ticket.request_id, generation=ticket.generation,
        target_epoch=ticket.target_epoch, target_track_id=snapshot.target_identity.track_id,
        snapshot_stamp_ns=ns)
    unit_source = dict(binding=binding, audit=dict(binding,
        source_kind='exact_last_raw_rgbd_sample', source_stamp_ns=ns,
        synthetic_support_points=False, projection_prediction_only=True))
    monkeypatch.setattr(node,'_require_diagnostic_observation_view_source',lambda _:unit_source if ordinary else None)
    monkeypatch.setattr(node,'_diagnostic_observation_view_evaluation',lambda *_:dict(ok=True))
    monkeypatch.setattr(node,'_frozen_observation_translation_delta_m',lambda *_:.20)
    monkeypatch.setattr(base.RemoteGrasp6DNode,'_observation_following_support_evaluation',
        lambda self,_prepared,result,_target=None:(result,dict(ok=True)))
    if ordinary:
        monkeypatch.setattr(base.RemoteGrasp6DNode, '_prepare_and_predict',
            lambda *_: pytest.fail('ordinary independent view entered contact/WSL prepare'))
    prepared=node._prepare_and_predict(ticket)
    node._require_stream_ticket_current=MethodType(base.RemoteGrasp6DNode._require_stream_ticket_current,node)
    node._require_stream_ticket_current_locked=MethodType(base.RemoteGrasp6DNode._require_stream_ticket_current_locked,node)
    assert prepared.candidates==() and prepared.tabletop_candidates==() and prepared.graspnet_input is None
    if not ordinary:
        assert prepared.geometry.size_xyz_m[1]>.05
    assert calls and prepared.geometry.object_points_base.shape == geometry.object_points_base.shape
    monkeypatch.setattr(node,'_activate_prepared_geometry',lambda _:True)
    camera=np.eye(4);camera[:3,3]=[.20,.15 if ordinary else -.05,.15]
    camera[:3,:3]=camera_rotation_toward(prepared.geometry.center_base,camera[:3,3])
    pose=base.make_pose_stamped('base_link',camera[:3,3],base.quaternion_from_matrix(camera),stamp=stamp)
    monkeypatch.setattr(node,'_frozen_observation_reference_pose',lambda _:pose)
    if ordinary:
        from alicia_grasp_modes.observe_return_visual import missing_side_view_gain
        gain = missing_side_view_gain(node._observe_return_view_gaps, camera[:3, 3], .002)
    else:
        gain=silhouette_view_gain(node._observe_return_silhouette_evidence,camera,.002)
    assert gain['ok'],gain
    view=dict(working_volume_view(prepared.snapshot.stamp_ns), **{
        'observe_return_missing_side_gain' if ordinary else 'observe_return_silhouette_gain': gain})
    if ordinary:
        from alicia_grasp_modes.observation_view_utility import predicted_information_utility
        view['information_utility'] = predicted_information_utility(
            gain['predicted_new_side_gain_m'], gain['depth_uncertainty_m'], .3, 500.)
    sequence=NS(pregrasp=pose,observation_view_audit=deepcopy(view))
    variant=dict(sequence=sequence,observation_view=deepcopy(view),
        observation_envelope=base.ObservationEnvelopeResult(True,'','',.020),
        observation_side_evidence={},observation_translation_delta_m=.20,
        camera_roll_offset_deg=0.,camera_tilt_offset_deg=0.,preference_index=0)
    monkeypatch.setattr(node,'_far_field_observation_variants',lambda *_:((variant,),()))
    if ordinary:
        # Endpoint ordering/source extraction are covered by dedicated source
        # and FK tests; this transaction has exactly one eligible view.
        monkeypatch.setattr(node, '_order_observation_checks', lambda rows: rows)
    node.gate_audit_enabled=True
    node.gate_audit_output_path=str(tmp_path/'planning.json')
    node.mujoco_audit_output_path=str(tmp_path/'mujoco.json')
    node.gate_audit_pub=Publisher()
    node._promotion_now_sec=lambda:12.
    node._execution_plan_validity_now_sec=lambda:12.
    node._stream_source_clock=lambda:12.
    node.observation_following_guard_required=True
    # Strict path checker is a ROS service; use its original caller and all
    # original downstream transaction/schema guards, injecting only response.
    checks=[]
    def strict(target,observation=False):
        checks.append((target,observation))
        result=base.MoveItResult(True,.3,.2,'checked observation_screened_duration_sec=2.0',True,True,True,True)
        return result,{},{}
    monkeypatch.setattr(node,'_strict_moveit_evaluation',strict)
    monkeypatch.setattr(node,'_commit_strict_moveit_evaluation',lambda _pose,evaluation:evaluation[0])
    # No contact gate, model candidate or tracker is part of this source.
    monkeypatch.setattr(base.RemoteGrasp6DNode,'_accept_prediction',lambda *_:pytest.fail('contact acceptance entered'))
    monkeypatch.setattr(base.RemoteGrasp6DNode,'_prepare_and_predict',lambda *_:pytest.fail('WSL prediction entered'))
    return node,prepared,checks,variant


def test_independent_source_ignores_oversized_contact_obb_and_promotes_real_single_view(configured,monkeypatch,tmp_path):
    node,prepared,checks,variant=context(configured,monkeypatch,tmp_path)
    actual=node._accept_prediction(prepared)
    assert actual['status']=='PREVIEW_READY', node._stable_variant_runtime[(0,0)]['diagnostic_observation_failure']
    assert checks and all(observation for _,observation in checks)
    preview=node.latest_preview_rich_plan
    assert preview.candidate_source=='silhouette_observation'
    assert preview.candidate_source_lineage==['silhouette_observation']
    assert len(preview.poses)==1 and preview.required_open_width_m==0.
    assert preview.candidate_width_m==0. and not preview.has_candidate_model_width
    assert not preview.contact_geometry_evidence
    assert len(base.rich_plan_to_legacy(preview).poses)==1
    assert node.latest_rich_plan.plan_id==preview.plan_id
    assert node._execution_promotion_audit_ready(preview)
    report=node._active_execution_audit_report
    assert not observe_return_audit_error(report)
    assert not node._continuous_execution_schema_error(report,prepared.ticket)
    assert report['selected']['candidate_source']=='silhouette_observation'
    assert report['pipeline_funnel']['source_counts']=={'silhouette_observation':
        dict(generated=1,locally_valid=1,stable=1,preview=1,promoted=1)}
    assert report['selected']['execution_sequence']['kind']=='far_field_observation'
    assert [r['stage'] for r in report['selected']['execution_sequence']['stages']]==['observation']
    assert report['selected']['contact_geometry_checked'] is False
    assert report['pipeline_funnel']['grasp_tracker_used'] is False
    assert json.loads((tmp_path/'planning.json.execution').read_text())==report


@pytest.mark.parametrize('failure',['strict_path','no_endpoint'])
def test_failed_view_never_publishes_execution_or_contacts(configured,monkeypatch,tmp_path,failure):
    node,prepared,checks,_=context(configured,monkeypatch,tmp_path)
    if failure=='strict_path':
        monkeypatch.setattr(node,'_strict_moveit_evaluation',lambda *a,**k:(
            base.MoveItResult(False,0.,0.,'collision',failure_code='MOVEIT_COLLISION'),{},{}))
    else:
        monkeypatch.setattr(node,'_far_field_observation_variants',lambda *_:((),({'reason':'endpoint CAD'},)))
    before=len(node.rich_plan_pub.messages)
    result=node._accept_prediction(prepared)
    assert result['status']=='SILHOUETTE_OBSERVATION_NO_SAFE_VIEW'
    assert len(node.rich_plan_pub.messages)==before
    assert not getattr(node,'latest_rich_plan',None)
    assert node._observe_return_paused
    assert not node.latest_preview_rich_plan.valid


@pytest.mark.parametrize('failure',['epoch','snapshot_identity','no_envelope','wrong_request','geometry','near_field'])
def test_invalid_source_context_cannot_publish(configured,monkeypatch,tmp_path,failure):
    node,prepared,_,_=context(configured,monkeypatch,tmp_path)
    if failure=='epoch':prepared=replace(prepared,ticket=replace(prepared.ticket,target_epoch=prepared.ticket.target_epoch+1))
    elif failure=='snapshot_identity':
        identity=base.TargetTrackIdentity.from_stream(0,prepared.ticket.target_epoch+1)
        prepared=replace(prepared,snapshot=replace(prepared.snapshot,target_epoch=identity.epoch,target_identity=identity))
    elif failure=='no_envelope':prepared=replace(prepared,remote_diagnostics={})
    elif failure=='wrong_request':prepared=replace(prepared,ticket=replace(prepared.ticket,request_id=999))
    elif failure=='geometry':prepared=replace(prepared,geometry=replace(prepared.geometry,ok=False))
    elif failure=='near_field':prepared=replace(prepared,near_field=True)
    before=len(node.rich_plan_pub.messages)
    with pytest.raises((base.CandidateContractError,base.StreamResultCancelled)):node._accept_diagnostic_observation_prediction(prepared)
    assert len(node.rich_plan_pub.messages)==before


def test_new_sequence_contains_only_observation_and_uses_original_distance_bounds(configured,monkeypatch,tmp_path):
    node,prepared,_,_=context(configured,monkeypatch,tmp_path)
    monkeypatch.setattr(node,'_tool_from_camera_matrix',lambda:np.eye(4))
    monkeypatch.setattr(node,'_contact_stage_profile',lambda *_a,**_k:pytest.fail('contact profile entered'))
    reference=node._frozen_observation_reference_pose(prepared)
    sequence=node._make_observation_sequence(reference,prepared.geometry,[0,0,-1],prepared.snapshot,
        reference,.26)
    assert set(vars(sequence))=={'pregrasp','adaptive_stage_profile','observation_view_audit'}
    assert sequence.adaptive_stage_profile.depth_uncertainty_m==.002
    assert sequence.observation_view_audit['actual_camera_target_distance_m']==pytest.approx(.26)
    with pytest.raises(ValueError):
        node._make_observation_sequence(reference,prepared.geometry,[0,0,-1],prepared.snapshot,reference,.31)


@pytest.mark.parametrize('invalidation',['camera_epoch','manual_generation'])
def test_camera_or_manual_invalidation_during_path_check_revokes_observation(configured,monkeypatch,tmp_path,invalidation):
    node,prepared,_,_=context(configured,monkeypatch,tmp_path)
    def strict(*_a,**_k):
        if invalidation=='camera_epoch':
            node._advance_target_instance_epoch('CAMERA_STREAM_STALE',preserve_identity=False)
        else:
            node.stop_streaming()
        return base.MoveItResult(True,.3,.2,'checked',True,True,True,True),{},{}
    monkeypatch.setattr(node,'_strict_moveit_evaluation',strict)
    with pytest.raises(base.StreamResultCancelled):node._accept_prediction(prepared)
    assert not any(plan.valid for plan in node.rich_plan_pub.messages)
    assert not (tmp_path/'planning.json.execution').exists()


def test_manual_generation_change_during_atomic_audit_cannot_publish_observation(configured,monkeypatch,tmp_path):
    node,prepared,_,_=context(configured,monkeypatch,tmp_path)
    original=node._write_gate_audit_report
    def write(*args,**kwargs):
        result=original(*args,**kwargs)
        if kwargs.get('reference_attr')=='_latest_execution_audit_reference':
            node.stop_streaming()
        return result
    monkeypatch.setattr(node,'_write_gate_audit_report',write)
    node._accept_prediction(prepared)
    assert not any(plan.valid for plan in node.rich_plan_pub.messages)
    assert not node._execution_promotion_audit_ready(node.latest_preview_rich_plan)


def test_new_fit_normal_does_not_replace_frozen_direction_source(configured,monkeypatch,tmp_path):
    node,prepared,_,_=context(configured,monkeypatch,tmp_path)
    evidence=node._observe_return_silhouette_evidence
    matrix=np.eye(4);matrix[:3,:3]=evidence['view_geometry']['initial_camera_rotation_base']
    reference=base.make_pose_stamped('base_link',evidence['view_geometry']['initial_camera_origin_base_m'],
        base.quaternion_from_matrix(matrix),stamp=prepared.stamp)
    monkeypatch.setattr(node,'_tool_from_camera_matrix',lambda:np.eye(4))
    node.observation_camera_tilt_offsets_deg=(10.,20.,30.)
    node.observation_camera_roll_offsets_deg=(0.,90.,180.)
    original=np.asarray(evidence['view_geometry']['support_normal_base'])
    fresh=original+np.array([0.0006,0.,0.]);fresh/=np.linalg.norm(fresh)
    assert not np.allclose(fresh,original,atol=1e-6,rtol=0.)
    node._observation_reference_tilt_variants(reference,fresh)
    assert node._observe_return_directed_views
    for _rotation,metadata in node._observe_return_directed_views:
        assert metadata['source_stamp_ns']==evidence['source_stamp_ns']
        assert metadata['predicted_gain_m']>metadata['depth_uncertainty_m']+1e-6
    # The generator does not relabel a fresh fit as original source evidence.
    np.testing.assert_array_equal(evidence['view_geometry']['support_normal_base'],original)
