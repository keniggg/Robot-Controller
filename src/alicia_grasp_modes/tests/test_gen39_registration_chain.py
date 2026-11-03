"""GEN39 real source-to-policy-to-task chain with explicit device simulators.

The original GEN35 unified fixture owns acquisition admission, actual source
geometry, final qualification, publication, task freeze, and closing gates.
No qualified model, registration result, contact proof, or codec is stubbed.
"""
from copy import deepcopy
import json
import hashlib
from pathlib import Path
from types import SimpleNamespace as NS
from dataclasses import replace

import numpy as np
import pytest

import test_gen35_minimal_chain_replay as chain
from test_gen38_cuboid_chain import cuboid_scene


@pytest.fixture(autouse=True)
def prohibit_unsimulated_transports(monkeypatch):
    import socket
    import rospy
    def forbidden(*args, **kwargs):
        raise chain.UnsimulatedExternalCall('GEN39 unsimulated boundary '+repr(args))
    for name in ('connect','connect_ex','sendto','bind','listen'):
        monkeypatch.setattr(socket.socket,name,forbidden)
    monkeypatch.setattr(socket,'create_connection',forbidden)
    for name in ('init_node','Publisher','Subscriber','Service','ServiceProxy',
                 'wait_for_service','set_param','delete_param'):
        monkeypatch.setattr(rospy,name,forbidden)


def _install_real_reached_source_adapter(monkeypatch):
    """Replace only GEN35's hand-constructed reached observation fixture.

    The production exact-frame resolver receives actual captured RGB-D and
    previously delivered mask/object, then runs its actual snapshot fuser and
    geometry preparation. Its own raw observation/cache owner is returned.
    Worker station snapshots still use the original three-frame admission.
    """
    station=chain.station_samples
    def adapted(c):
        node=c.remote
        resolve=node._resolve_reached_view_observation
        def exact_reached(msg):
            stamp=msg.reference_source_stamp.to_nsec()
            pairs=[pair for pair in c.sources if pair[0].header.stamp.to_nsec()==stamp]
            assert len(pairs)==1
            color,depth=pairs[0]
            # This is the accepted encoder simulator already used by the
            # producer, not a manufactured SDK/FK/temperature observation.
            node.frames.update_joints(c.producer._rgb_interest_feedback[-1][1])
            node.frames.update_color(node.bridge.imgmsg_to_cv2(color,'bgr8'),color.header.stamp,color.header.frame_id)
            node.frames.update_depth(node.bridge.imgmsg_to_cv2(depth,'passthrough'),depth.header.stamp,depth.header.frame_id)
            # The old fixture assigned a manually constructed observation.
            # Force the real exact-source resolver to own its preparation.
            node._latest_target_observation=None
            observation=resolve(msg)
            assert observation is not None
            assert observation is node._latest_raw_geometry_source['observation']
            assert observation.stamp_ns==stamp
            c.gen39_reached_source_stamp_ns=stamp
            return observation
        with monkeypatch.context() as patch:
            patch.setattr(node,'_resolve_reached_view_observation',exact_reached)
            return station(c)
    monkeypatch.setattr(chain,'station_samples',adapted)


def _watch_real_policy(monkeypatch, *, optimizer_proposal_simulator=False):
    from test_remote_modes import base
    from alicia_flexible_grasp.vision.regular_box_inference import regular_box_geometry_sha256
    original=base.append_box_consistent_registered_view
    _install_real_reached_source_adapter(monkeypatch)
    calls=[]
    def watched(surface,reference,moving,geometry,model,*args,**kwargs):
        before=surface.points_base.copy()
        assert model.source['stamp_ns']==moving.stamp_ns
        assert model.source['geometry_sha256']==regular_box_geometry_sha256(geometry)
        assert np.array_equal(moving.points_base,geometry.object_points_base)
        if optimizer_proposal_simulator:
            from alicia_flexible_grasp.vision import multiview_surface as mv
            # Explicit optimizer-provider simulator only. The proposed rigid
            # correction is checked by every original production gate.
            def proposal(ref,source,config=None):
                T=np.eye(4);T[:3,3]=cuboid_scene().geometry.axes_base[:,0]*.006
                checked=mv.evaluate_registration_transform(ref,source,T,config)
                assert checked.ok,checked
                return checked
            with monkeypatch.context() as patch:
                patch.setattr(mv,'register_surface_view',proposal)
                result,fused,audit=original(surface,reference,moving,geometry,model,*args,**kwargs)
        else:
            result,fused,audit=original(surface,reference,moving,geometry,model,*args,**kwargs)
        np.testing.assert_array_equal(surface.points_base,before)
        calls.append(dict(raw_model=model,raw_geometry=geometry,moving=moving,
                          before=surface,result=result,fused=fused,audit=deepcopy(audit)))
        return result,fused,audit
    monkeypatch.setattr(base,'append_box_consistent_registered_view',watched)
    return calls


def _assert_final_chain(c,calls):
    from alicia_flexible_grasp.grasp.contact_evidence import decode_contact_evidence
    from alicia_flexible_grasp.grasp.rich_plan_integrity import compute_plan_id
    from alicia_flexible_grasp.vision.regular_box_inference import regular_box_geometry_sha256
    assert calls,'actual preparation never entered the new source-bound policy'
    source=c.current_row['object'].header.stamp.to_nsec()
    current=[row for row in calls if row['moving'].stamp_ns==source]
    assert len(current)==1
    call=current[0]
    reached_source=c.requests[-1]['source_stamps_ns'][-1]
    reached=[row for row in calls if row['moving'].stamp_ns==reached_source]
    assert len(reached)==1,'ACK reached source bypassed source-bound registration policy'
    assert reached_source==c.gen39_reached_source_stamp_ns<source
    assert reached[0]['raw_model'].source['geometry_sha256']==regular_box_geometry_sha256(reached[0]['raw_geometry'])
    final=decode_contact_evidence(c.contact,required=True)['box_model_evidence']
    assert final['schema_version']==3
    assert final['source']['stamp_ns']==source
    actual=c.remote._regular_box_snapshot['geometry']
    assert final['source']['geometry_sha256']==regular_box_geometry_sha256(actual)
    assert final['model_sha256']!=call['raw_model'].model_sha256
    assert final['source']['geometry_sha256']!=call['raw_model'].source['geometry_sha256']
    assert c.contact.plan_id==compute_plan_id(c.contact)
    assert [row['state'] for row in c.requests]==['executing','requalifying','qualified']
    assert len(c.task._frozen_contact_pub.messages)==1
    assert json.loads(c.task._frozen_contact_pub.messages[0].data)['plan_id']==c.contact.plan_id
    assert c.execution_result is False and c.states[-1].state=='HOLDING'
    assert 'GRASP_HOLD_UNVERIFIED' in c.states[-1].message
    assert len([r for r in c.device_calls if r[0]=='gripper' and r[2]<.05])==1
    assert not any(r[0]=='lift' for r in c.device_calls)
    return call,final


@pytest.mark.parametrize('order',[
    ('status','ack','object','mask'),
    ('mask','object','status','ack'),
    ('ack','mask','status','object'),
])
def test_real_source_policy_and_final_binding_traverse_perturbed_unified_chain(monkeypatch,tmp_path,order):
    monkeypatch.setenv('ROS_LOG_DIR',str(tmp_path/'runtime_logs'))
    monkeypatch.setattr(chain,'ray_scene',cuboid_scene)
    calls=_watch_real_policy(monkeypatch)
    c=chain.unified_replay(monkeypatch,tmp_path,order,perturb=True)
    call,final=_assert_final_chain(c,calls)
    # This symmetric same-scene control enters the new policy through the
    # real worker, but its original registration already explains the box.
    assert call['audit']['selected_hypothesis']=='original_icp'
    assert call['audit']['selection_reason']=='ORIGINAL_ICP_BOX_CONSISTENT'
    assert call['audit']['hypotheses']['original_icp']['consistency']['ok']
    assert 'identity' not in call['audit']['hypotheses']
    assert 'qualified_then_late_pending_and_duplicate_ack' in c.injected_events
    assert 'duplicate_start_rejected' in c.injected_events
    (tmp_path/'gen39_chain_policy.json').write_text(json.dumps(dict(
        choice=call['audit'],raw_model_sha256=call['raw_model'].model_sha256,
        final_model_sha256=final['model_sha256'],source_stamp_ns=final['source']['stamp_ns'],
        injected_events=c.injected_events,physical_grasp_verified=False,
        external_device_providers='explicit simulators'),indent=2)+'\n')


@pytest.mark.parametrize('order',[
    ('status','ack','object','mask'),
    ('mask','object','status','ack'),
    ('ack','mask','status','object'),
])
def test_explicit_optimizer_proposal_simulator_identity_branch_runs_full_perturbed_chain(monkeypatch,tmp_path,order):
    """Handoff evidence, not a claim that real ICP naturally finds this T."""
    monkeypatch.setenv('ROS_LOG_DIR',str(tmp_path/'runtime_logs'))
    monkeypatch.setattr(chain,'ray_scene',cuboid_scene)
    calls=_watch_real_policy(monkeypatch,optimizer_proposal_simulator=True)
    c=chain.unified_replay(monkeypatch,tmp_path,order,perturb=True)
    call,final=_assert_final_chain(c,calls)
    audit=call['audit']
    assert audit['hypotheses']['original_icp']['registration']['ok']
    assert not audit['hypotheses']['original_icp']['consistency']['ok']
    assert audit['hypotheses']['identity']['registration']['ok']
    assert audit['hypotheses']['identity']['consistency']['ok']
    assert audit['selected_hypothesis']=='identity'
    assert np.array_equal(call['result'].transform_base,np.eye(4))
    assert 'duplicate_start_rejected' in c.injected_events
    (tmp_path/'gen39_simulated_optimizer_identity_chain.json').write_text(json.dumps(dict(
        choice=audit,optimizer_provider='EXPLICIT SIMULATOR: fixed 6mm proposal, production all-gate evaluation',
        natural_optimizer_branch_claimed=False,source_stamp_ns=final['source']['stamp_ns'],
        raw_model_sha256=call['raw_model'].model_sha256,final_model_sha256=final['model_sha256'],
        physical_grasp_verified=False),indent=2)+'\n')


def test_without_identity_choice_same_proposal_fails_at_final_full_geometry(monkeypatch):
    """Source qualification and original gates cannot bypass final history."""
    from test_remote_modes import base
    from alicia_flexible_grasp.vision import multiview_surface as mv
    monkeypatch.setattr(chain,'ray_scene',cuboid_scene)
    _install_real_reached_source_adapter(monkeypatch)
    calls=[]
    def keep_proposal(surface,reference,moving,geometry,model,config=None,voxel_size_m=.0025):
        T=np.eye(4);T[:3,3]=cuboid_scene().geometry.axes_base[:,0]*.006
        result,fused=mv.append_registered_view(surface,reference,moving,config,voxel_size_m,
            transform_base=T)
        assert result.ok
        calls.append((geometry,model,result))
        return result,fused,dict(diagnostic_only=True,execution_authorized=False,
            selected_hypothesis='counterfactual_keep_explicit_simulated_optimizer_proposal')
    monkeypatch.setattr(base,'append_box_consistent_registered_view',keep_proposal)
    c=chain.arrived_same_scene(monkeypatch)
    with pytest.raises(AssertionError):
        chain.same_source_contact(c)
    assert calls and calls[0][1].schema_version==3
    result=c.remote._regular_box_snapshot['result']
    assert not result.ok and result.code=='BOX_MEASURED_GEOMETRY_CONFLICT'
    assert not hasattr(c,'contact')


_RECORD_FIXTURE=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/'
    'full_latest_gen36_restart_20261017T012037Z/verification/gen38_actual_attempt1_source_replay_r1')


@pytest.mark.skipif(not (_RECORD_FIXTURE/'seal_r1.json').is_file(),
    reason='explicit closed-record source fixture is not installed; no historical data is manufactured')
def test_closed_source_two_view_counterexample_uses_real_icp_and_requalifies_full_geometry(tmp_path):
    """Minimal real-ICP branch counterexample, not a historical 3-view replay.

    The original unified scene is symmetric and keeps original ICP. This case
    uses the exact initial1419 and actual current869 with new source-generated
    models. Missing historical ACK history is explicitly outside this case.
    """
    import cv2
    from alicia_flexible_grasp.vision.object_geometry import GeometryEstimate
    from alicia_flexible_grasp.vision.target_observation import TargetTrackIdentity
    from alicia_flexible_grasp.vision.multiview_surface import SurfaceView,fused_surface_from_view
    from alicia_flexible_grasp.vision.box_surface_registration import append_box_consistent_registered_view
    from alicia_flexible_grasp.vision.fused_scene_geometry import enclose_registered_surface,_array_digest
    from alicia_flexible_grasp.vision.regular_box_inference import qualify_regular_box_top,regular_box_geometry_sha256
    from alicia_flexible_grasp.grasp.gripper_geometry import (GripperGeometry,
        ANALYTICAL_FINGER_SIZE_XYZ_M,ANALYTICAL_PALM_SIZE_XYZ_M,evaluate_inferred_box_contact)
    def read(p):return json.loads(p.read_text())
    root=_RECORD_FIXTURE
    seal=read(root/'seal_r1.json')
    for name in ('near_qualification_replay.json','exact_replay_geometry.npz','surface_reference.json'):
        assert hashlib.sha256((root/name).read_bytes()).hexdigest()==seal['files'][name]['sha256']
    recorded=read(root/'near_qualification_replay.json');g=recorded['input_geometry']
    # Prefix recovery is already bound by exact union/order/shape+dtype digest.
    points=np.load(root/'exact_replay_geometry.npz')['current_raw_points']
    assert len(points)==869 and _array_digest(points)=='3afdb0cc5a8e477821f3df6cb3c103bcc21a606aefc180495fe5b8b21f5c4fbe'
    full=np.array(g['object_points_base_m']);np.testing.assert_array_equal(points,full[:869])
    # Original raw OBB, before the recorded fused enclosure expanded it.
    center=np.array([-.0870455837175115,-.3667367900376436,.014318268671965407])
    size=np.array([.05099959155314199,.03438796764750491,.02068150663375746])
    geometry=GeometryEstimate(True,'','',center,g['R_base_obb'],size,g['support_normal_base'],
        g['support_offset_m'],g['support_inlier_ratio'],points,'instance_mask')
    stamp=1792214632454550027;raw={}
    for role,ext in [('color','.png'),('mask','.png'),('depth','.npy')]:
        p=root/'source_images'/(str(stamp)+'_'+role+ext)
        raw[role]=np.load(p,allow_pickle=False) if role=='depth' else cv2.imread(str(p),cv2.IMREAD_UNCHANGED)
        assert hashlib.sha256(raw[role].tobytes()).hexdigest()==recorded['historical']['retained_pixel_sha256'][role+'_sha256']
    c=recorded['K'];K=NS(**{key:c[key] for key in ('fx','fy','cx','cy','width','height')})
    T=np.array(recorded['T_base_optical']);identity=TargetTrackIdentity(4,'g1-t4')
    qualified=qualify_regular_box_top(raw['depth'],raw['mask'],K,c['depth_scale'],T,geometry,
        identity,stamp,'instance_mask',color_bgr=raw['color'])
    assert qualified.ok and qualified.model.schema_version==3
    reference_data=read(root/'surface_reference.json')['reference']
    reference=SurfaceView(identity,reference_data['stamp_ns'],reference_data['points_base'],
        reference_data['support_normal_base'],reference_data['support_offset_m'],reference_data['edge_clearance_px'])
    assert len(reference.points_base)==1419
    before=fused_surface_from_view(reference,.0015)
    moving=SurfaceView(identity,stamp,points,geometry.support_normal_base,geometry.support_offset_m,
        recorded['historical']['edge_clearance_px'])
    registration,after,audit=append_box_consistent_registered_view(before,reference,moving,
        geometry,qualified.model,voxel_size_m=.0015)
    assert registration.ok and np.array_equal(registration.transform_base,np.eye(4))
    assert audit['selected_hypothesis']=='identity'
    assert not audit['hypotheses']['original_icp']['consistency']['ok']
    assert audit['hypotheses']['identity']['registration']['ok']
    assert audit['hypotheses']['identity']['consistency']['ok']
    assert len(before.view_stamps_ns)==1 and len(after.view_stamps_ns)==2
    # Selected identity keeps the complete accepted history in this frame.
    fused,enclosure=enclose_registered_surface(geometry,after,expected_identity=identity)
    final=qualify_regular_box_top(raw['depth'],raw['mask'],K,c['depth_scale'],T,fused,
        identity,stamp,'instance_mask',color_bgr=raw['color'])
    assert final.ok
    assert final.model.source['geometry_sha256']==regular_box_geometry_sha256(fused)
    assert final.model.source['geometry_sha256']!=qualified.model.source['geometry_sha256']
    assert final.model.model_sha256!=qualified.model.model_sha256
    gripper=GripperGeometry(.05,.002,ANALYTICAL_FINGER_SIZE_XYZ_M,ANALYTICAL_PALM_SIZE_XYZ_M,.003)
    evidence=evaluate_inferred_box_contact(final.model,final.model.center_base,final.model.axes_base[:,1],
        final.model.axes_base[:,2],gripper,opening_fit_clearance_each_side_m=.0005)
    required=evidence.inferred_width_m+evidence.width_uncertainty_m+.001
    assert required==pytest.approx(.05053246495155868,abs=1e-9)
    assert required+.002>.05 and not evidence.ok
    (tmp_path/'gen39_actual_two_view_choice.json').write_text(json.dumps(dict(choice=audit,
        required_open_width_m=required,independent_reserve_m=.002,opening_still_rejected=True,
        role='NEW offline two-view reconstruction, not actual preappend three-view history',
        physical_grasp_verified=False),indent=2)+'\n')
