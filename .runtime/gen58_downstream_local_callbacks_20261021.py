"""Bounded GEN57 real-callback export/local geometry continuation, no prediction."""
from copy import deepcopy
from dataclasses import asdict,is_dataclass
import hashlib
import io
import json
from pathlib import Path
import traceback
from types import SimpleNamespace as NS

class OfflinePredictionBoundary(BaseException):
    pass

def run(ctx,h):
    import numpy as np
    from test_remote_modes import base
    from alicia_flexible_grasp.vision.multiview_surface import SurfaceView,fused_surface_from_view
    from alicia_flexible_grasp.vision.target_observation import TargetTrackIdentity
    remote=ctx['remote'];producer=ctx['producer'];out=ctx['output'];mp=ctx['mp'];rospy=ctx['rospy']
    rows=ctx['rows'];now=ctx['now'];parameters=ctx['parameters'];frame_data=ctx['frame_data']
    def plain(v):
        if is_dataclass(v):return plain(asdict(v))
        if isinstance(v,np.ndarray):return v.tolist()
        if isinstance(v,np.generic):return v.item()
        if isinstance(v,dict):return {str(k):plain(x) for k,x in v.items()}
        if isinstance(v,(tuple,list)):return [plain(x) for x in v]
        if isinstance(v,float) and not np.isfinite(v):return str(v)
        return v
    def save(name,v):return h.save(name,plain(v))
    def wire(m):
        b=io.BytesIO();m.serialize(b);return b.getvalue()
    exports=[]
    for kind in ('object','object_mask'):
        for msg in producer.pubs[kind].messages:
            blob=wire(msg);stamp=msg.header.stamp.to_nsec();name='%s_%d.bin'%(kind,stamp)
            (out/name).write_bytes(blob)
            exports.append(dict(kind=kind,source_stamp_ns=stamp,path=str(out/name),message_type=msg._type,
                wire_sha256=h.digest_bytes(blob),origin='same-run actual production producer callback publication'))
            if kind=='object_mask':
                array=producer.bridge.imgmsg_to_cv2(msg,'mono8')
                np.save(out/('mask_%d.npy'%stamp),array,allow_pickle=False)
    phases=ctx['phases'];assert len(phases)==1
    blob=wire(phases[0]);(out/'task_contact_replan_phase.bin').write_bytes(blob)
    save('actual_callback_export_manifest.json',dict(rows=exports,phase_sha256=h.digest_bytes(blob),
        original_r8_result_sha256=h.sha(h.DEFAULT_INPUT.parent/'owner_handoff_r8/result.json'),
        snapshot_sha256=h.sha(out/'source_manifest.json'),source_inputs=ctx['inputs'],
        all_real_objects=True,new_export_not_original_online_result=True))
    report=dict(mode='bounded actual callback local continuation',stage='parameter_namespace_reconstruction',
        prediction_called=False,network_used=False,motion_executed=False,moveit_called=False,
        physics_or_execution_verified=False,local_rejection=None,contract_gap=None)
    def deny(*a,**k):
        report['prediction_called']=True
        raise OfflinePredictionBoundary('new exact-input inference response is unavailable; no remote call')
    remote._predict_remote=deny
    try:
        # Construct the ROS namespace view of the byte-bound launch leaves;
        # apply the actual later recorded runtime mappings, never defaults.
        frozen=json.loads(ctx['args'].context_parameters.read_text())
        root={}
        def put(path,value):
            parts=[x for x in path.split('/') if x];d=root
            for x in parts[:-1]:d=d.setdefault(x,{})
            d[parts[-1]]=deepcopy(value)
        for key,value in sorted(frozen.items(),key=lambda item:(len(item[0].split('/')),item[0])):put(key,value)
        for key in ('/camera','/unknown_perception','/grasp_mode/selection','/grasp','/gui/joint_direct_mode'):
            put(key,parameters[key])
        for key,value in ctx['config_record'].items():
            if key not in ('/camera',):put(key,value)
        missing=object()
        def get(path,default=missing):
            d=root
            try:
                for part in path.split('/'):
                    if part:d=d[part]
                return deepcopy(d)
            except (KeyError,TypeError):
                if default is missing:raise KeyError(path)
                return deepcopy(default)
        mp.setattr(rospy,'get_param',get);mp.setattr(rospy,'get_param_cached',get)
        mp.setattr(rospy,'get_master',lambda:NS(getParam=lambda key:(1,'frozen parameter tree',get(key))))
        save('downstream_parameters.json',dict(parameters=root,all_resolved_parameters_sha256=h.sha(ctx['args'].context_parameters),
            runtime_override_sha256=h.sha(ctx['RECORD']/'perception_execution_configuration.json') if 'RECORD' in ctx else ctx['inputs']))
        # Execute only the production constructor's pure gripper assignment.
        # consumer_for deliberately omits I/O construction and this data field.
        import ast
        production_path=Path(base.__file__)
        tree=ast.parse(production_path.read_text())
        cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='RemoteGrasp6DNode')
        init=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='__init__')
        names={'gripper_geometry','max_candidates','candidate_max_relative_z_m',
            'candidate_max_target_distance_m','candidate_min_relative_z_m','candidate_target_gate_enabled',
            'geometry_min_size_m','camera_visibility_min_depth_m','camera_depth_scale',
            'gripper_tool_jaw_axis','gripper_tool_finger_length_axis','candidate_width_tolerance_m'}
        assignments=[n for n in init.body if isinstance(n,ast.Assign) and any(
            isinstance(t,ast.Attribute) and isinstance(t.value,ast.Name) and t.value.id=='self'
            and t.attr in names for t in n.targets)]
        assert len(assignments)==len(names)
        env=dict(vars(base),self=remote,remote_cfg=get('/grasp_6d/remote'),cam_cfg=get('/camera'),
            gripper_geometry_cfg=get('/grasp_6d/remote/gripper_geometry'))
        exec(compile(ast.Module(body=assignments,type_ignores=[]),str(production_path),'exec'),env)
        save('pure_constructor_assignment.json',dict(source=str(production_path),source_sha256=h.sha(production_path),
            assignments=[dict(line=n.lineno,ast=ast.dump(n)) for n in assignments],
            fields={name:getattr(remote,name) for name in sorted(names)},no_constructor_or_io_executed=True))
        remote._refresh_runtime_params()
        remote.tf_buffer=NS(lookup_transform=ctx['tf_lookup'])
        # This pure constructor field is not refreshed by _stage_runtime_params.
        remote.near_field_direct_timeout_sec=float(get('/grasp/near_field_replan_timeout_sec'))
        assert np.isfinite(remote.near_field_direct_timeout_sec) and remote.near_field_direct_timeout_sec>0.

        report['stage']='restore_original_initial_surface'
        audit_path=h.E/'grasp_records/20261021T020508.229978Z_585a1fd70e7e4886a808da45a7aba646/keyframes_audits/235ae3ec56a3e65cfc842ce6f0b032802f2a7716981513ed39687b9a500964d3.json'
        audit=json.loads(audit_path.read_text());g=audit['replay_geometry'];pts=np.asarray(g['object_points_base_m'],dtype=np.float64)
        assert h.array_sha(pts)==g['object_points_sha256']
        bbox=audit['graspnet_input']['detected_bbox_xywh'];x,y,w,height=bbox
        image_shape=audit['graspnet_input']['input_depth_shape']
        clearance=min(x,y,image_shape[1]-x-w,image_shape[0]-y-height)
        identity=remote._current_stream_target_identity();assert identity==TargetTrackIdentity(audit['target_epoch'],audit['observe_return']['target_track_id'])
        reference=SurfaceView(identity,audit['snapshot_stamp_ns'],pts,np.asarray(g['support_normal_base']),g['support_offset_m'],clearance)
        surface=fused_surface_from_view(reference,remote.geometry_voxel_size_m)
        assert len(surface.points_base)==audit['multiview_surface']['point_count']
        assert list(surface.view_stamps_ns)==audit['multiview_surface']['source_stamps_ns']
        remote._near_field_reference_view=reference;remote._near_field_fused_surface=surface
        remote._near_field_surface_phase_id=remote._near_field_phase_id
        remote._near_field_surface_generation=remote._stream_generation
        remote._observe_return_saved=(surface,reference)
        from alicia_grasp_modes.observation_information_uncertainty import observation_information_uncertainty_m
        remote._observe_return_view_uncertainty_m=observation_information_uncertainty_m(
            audit['remote_diagnostics']['tabletop_geometry']['adaptive_stage_profiles'])
        remote._observe_return_initial_stamp=audit['snapshot_stamp_ns']

        # The historical session is 'observing'. Production session transition
        # explicitly suspends near-field geometry while the arm observes
        # (observe_return_remote.py _observe_return_session_cb: active=False).
        # The old r8 handoff-only fixture never needed this geometric flag.
        assert remote._observe_return_session['phase']=='observing'
        remote.near_field_planning_active=False
        remote._near_field_phase_started_sec=0.
        remote._near_field_phase_deadline_sec=0.

        remote._latest_target_observation=None;remote._latest_raw_geometry_source=None
        remote._latest_registration_evidence=None;remote._latest_box_registration_choice=None
        save('restored_reference_surface.json',dict(source=str(audit_path),source_sha256=h.sha(audit_path),
            reference=reference,fused_surface=surface,derived_edge_clearance_from_audited_detected_bbox=clearance,
            exact_initial_rgbd_not_reread=True,initial_source_rebuilt_from_full_audited_measurements=True,
            current_phase=remote._observe_return_phase,phase_id=remote._near_field_phase_id,
            near_field_planning_active=remote.near_field_planning_active,source_generation=remote._stream_generation))
        report['stage']='reached_exact_source_rgbd_feed'
        source=phases[0].reference_source_stamp.to_nsec()
        assert source==h.ARRIVALS[-1]
        # Replay the actual callback messages again with exact matching pixels.
        # The virtual timestamp is retained, not a claim about elapsed hardware time.
        now[0]=phases[0].header.stamp.to_nsec();ctx['legacy_clock'][0]=now[0]/1e9
        feedback=rows[source]['feedback_before'];jointrow=next(r for r in ctx['joint_rows'] if r['data']['header']['stamp']['ns']==feedback[0])
        remote.joint_cb(ctx['joint_message'](jointrow))
        color,depth=ctx['image_pair'](source);remote.color_cb(color);remote.depth_cb(depth)
        for kind,callback in (('object_mask',remote.mask_cb),('object',remote.object_cb)):
            msg=next(m for m in producer.pubs[kind].messages if m.header.stamp.to_nsec()==source)
            callback(deepcopy(msg))
        before=len(remote.preview_rich_plan_pub.messages)
        report['stage']='actual_reached_near_field_callback'
        save('reached_callback_entry.json',dict(selection_current=remote._selection_is_current(),
            phase=remote._observe_return_phase,diagnostic_only=remote._observe_return_diagnostic_observation_only,
            current_ns=now[0],phase_start_ns=phases[0].header.stamp.to_nsec(),
            phase_deadline_ns=phases[0].deadline.to_nsec(),phase_frame_id=phases[0].header.frame_id,
            near_field_timeout_sec=remote.near_field_direct_timeout_sec,session=remote._observe_return_session))
        captured_preparations=[]
        original_prepare=remote._prepare_snapshot_geometry
        def observe_prepare(*a,**kw):
            result=original_prepare(*a,**kw)
            captured_preparations.append(dict(raw_source=remote._latest_raw_geometry_source,result=result))
            return result
        # Observer returns the original result unchanged and only retains its
        # immutable source before the production phase transition retires it.
        remote._prepare_snapshot_geometry=observe_prepare
        try:remote.near_field_state_cb(deepcopy(phases[0]))
        finally:remote._prepare_snapshot_geometry=original_prepare
        evidence=getattr(remote,'_latest_registration_evidence',None)
        observation=getattr(remote,'_latest_target_observation',None)
        save('reached_callback_result.json',dict(phase=remote._observe_return_phase,
            reached_phase_id=getattr(remote,'_observe_return_reached_phase_id',None),
            registration_evidence=evidence,raw_observation=observation,
            latest_fused_scene_audit=getattr(remote,'_latest_fused_scene_audit',None),
            information_gain=getattr(remote,'_observe_return_step_gain',None),
            terminal_diagnostics=[m.diagnostic for m in remote.preview_rich_plan_pub.messages[before:]],
            reached_retry_present=getattr(remote,'_observe_return_reached_retry',None) is not None))
        if evidence is None:
            report['contract_gap']='actual reached callback did not produce registration evidence; see exact result and traceback if any'
            return report
        if not evidence.result.ok:
            report['local_rejection']=evidence.result.code
            return report
        report['registration_passed']=True
        if getattr(remote,'_observe_return_reached_phase_id',None)!=phases[0].phase_id:
            report['stage']='after_registration_information_gain'
            report['contract_gap']='Actual phase callback did not complete information-gain/source commit; see terminal diagnostics.'
            return report
        report['reached_callback_committed']=True
        report['stage']='registered_measured_fused_enclosure'
        from alicia_flexible_grasp.vision.fused_scene_geometry import enclose_registered_surface
        assert len(captured_preparations)==1
        measured=captured_preparations[0]['raw_source']
        assert measured['observation'].stamp_ns==source
        snapshot_surface,snapshot_reference=remote._snapshot_multiview_surface()
        enclosed,enclosure=enclose_registered_surface(measured['geometry'],snapshot_surface,
            expected_identity=identity,max_size_m=remote.geometry_max_size_m,
            max_height_m=remote.geometry_max_height_m)
        save('fused_enclosure_result.json',dict(raw_geometry=measured['geometry'],
            raw_snapshot_source_stamps=list(measured['snapshot'].sample_stamp_ns),
            source_transform=measured['transform'],snapshot_surface=snapshot_surface,
            snapshot_reference=snapshot_reference,planning_geometry=enclosed,enclosure=enclosure,
            production_function='enclose_registered_surface',no_synthetic_points=True))
        report['fused_enclosure_passed']=True

        # The local materializer consumes actual measured execution uncertainty.
        # The original dynamic value was not captured in the startup parameter map.
        if get('/grasp_6d/runtime_execution_error',None) is None:
            report['stage']='before_adaptive_tabletop_materialization'
            report['contract_gap']='Original post-observation /grasp_6d/runtime_execution_error is absent from retained parameter snapshots; do not silently replace it with zero.'
            return report
        report['contract_gap']='Continuation requires exact postqualification stable-frame snapshot export; no synthetic snapshot inserted.'
        return report
    except Exception as exc:
        report['exception_type']=type(exc).__name__;report['exception']=str(exc);report['traceback']=traceback.format_exc()
        report['contract_gap']='local callback continuation needs additional measured context; no mock acceptance inserted'
        return report
