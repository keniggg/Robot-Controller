"""Pure historical R18 source fusion, original materializer and geometry gates.

No rejected qualification gate is skipped. ROS message classes and pure original
node methods are imported with every network/device/ROS operation forbidden.
This reconstructs historical calculation inputs; no fresh execution authority,
IK, arm path, registration re-fit, visibility or physical grip is claimed.
"""
import argparse
import ast
from copy import deepcopy
from dataclasses import asdict, replace
import gzip
import io
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import platform
import socket
import struct
import sys
import threading
import time
import traceback
from types import ModuleType, SimpleNamespace as NS

import numpy as np
import yaml

LIVE = Path(__file__).resolve().parents[1]
M = Path('/media/zhuyupei/SanStand')
E = M/'实验记录/Robot-Controller'
NV = E/'ros_log/full_latest_gen41_restart_20261018T011222Z/verification'
G = E/'grasp_records/20261018T023244.888815Z_d5686c81c6144223b5adb93e60ca7cee'
I = NV/'gen43_actual_source_inspection_r1'
BASE = 'src/alicia_flexible_grasp_supervisor/scripts/remote_grasp6d_node.py'
BOX = 'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/cuboid_rgb_interval.py'
STAMP = 1792290823025146722
AUDIT = 'b7d9a682a3f359f3963ecbf9e6fd95ecffa3768c8a55e81176f35da8b3451945.json'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def plain(value):
    if isinstance(value, dict) or hasattr(value, 'items'):
        return {str(k): plain(v) for k,v in value.items()}
    if isinstance(value, (list,tuple)):
        return [plain(v) for v in value]
    if hasattr(value, 'tolist'):
        return value.tolist()
    return value


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--workspace', required=True, type=Path)
    p.add_argument('--output', required=True, type=Path)
    a = p.parse_args()
    work, out = a.workspace.resolve(strict=True), a.output.resolve()
    assert work in (LIVE/'.runtime/gen42_source_before_gen43', LIVE/'.runtime/gen43_mask_candidate')
    assert M.is_mount() and not out.exists()
    out.relative_to(NV.resolve(strict=True))
    assert out.parent.stat().st_dev == M.stat().st_dev
    out.mkdir()
    (out/'tmp').mkdir()
    os.environ.update(ROS_LOG_DIR=str(out/'ros'), TMPDIR=str(out/'tmp'))
    os.environ['ROS_HOME'] = str(out/'ros_home')
    platform.architecture = lambda *args, **kwargs: (str(struct.calcsize('P')*8)+'bit','ELF')
    platform._syscmd_uname = lambda option, default='': default
    socket.has_ipv6 = False
    attempts, inputs, reads = [], {}, []
    record = dict(passed=False, request_id=18, snapshot_stamp_ns=STAMP,
        workspace=str(work), helper_sha256=sha(__file__), physical_grasp_verified=False,
        execution_authorized=False, ROS_initialized=False, ROS_operations_called=False,
        hardware_contacted=False, temperature_queried=False, motion_commands_sent=False,
        diagnostic_gate_bypass_used=False, registration_recomputed=False,
        IK_checked=False, whole_arm_path_checked=False, camera_visibility_checked=False,
        full_contact_sequence_precheck_completed=False, continuous_joint_contact_curve_checked=False,
        future_constant_bias_prediction_checked=False, actual_nominal_endpoint_feedback_used=False,
        current_execution_readiness_checked=False, historical_measurements_only=True)

    def save(name,value):
        assert M.is_mount() and out.stat().st_dev == M.stat().st_dev
        with (out/name).open('x') as f:
            json.dump(plain(value), f, ensure_ascii=False, indent=2, allow_nan=False)
            f.write('\n'); f.flush(); os.fsync(f.fileno())

    def read(path):
        inputs[str(path)] = sha(path)
        return json.loads(Path(path).read_bytes())

    class Boundary(BaseException):
        pass

    def forbidden(*args,**kwargs):
        attempts.append(repr(args)[:500])
        raise Boundary('offline boundary: '+repr(args)[:300])

    for name in ('connect','connect_ex','sendto','bind','listen'):
        setattr(socket.socket,name,forbidden)
    socket.create_connection = socket.getaddrinfo = forbidden
    def guard(event,args):
        if event in ('subprocess.Popen','os.system','os.posix_spawn','os.exec'):
            forbidden(event,args)
        if event=='open' and args and isinstance(args[0],(str,bytes,os.PathLike)):
            path=os.path.realpath(os.fsdecode(args[0]))
            if path.startswith('/dev/') and path not in ('/dev/null','/dev/zero','/dev/random','/dev/urandom') and not path.startswith(('/dev/shm/','/dev/fd/','/dev/pts/')):
                forbidden('device_open',path)
    sys.addaudithook(guard)
    try:
        release_path=NV/'gen42_release_manifest.json'
        assert sha(release_path)=='4ce3b5142b18e9968deff461d39a202f0fe6f91572394744abad67d29c4810dc'
        release=read(release_path)
        expected=dict(release['source_sha256'])
        if work.name=='gen43_mask_candidate':
            expected[BOX]='50e00a5b751ba0470f98c4215922bb6f05471784a364181c297ad5f617751d7f'
        assert all(sha(work/n)==h for n,h in expected.items())
        sys.path.insert(0,str(work/'src/alicia_flexible_grasp_supervisor/src'))
        sys.path.append(str(LIVE/'devel/lib/python3/dist-packages'))
        sys.path.append(str(LIVE.parents[1]/'devel/lib/python3/dist-packages'))
        transformation_path=Path('/opt/ros/noetic/lib/python3/dist-packages/tf/transformations.py')
        namespace=ModuleType('tf'); namespace.__path__=[str(transformation_path.parent)]
        sys.modules['tf']=namespace
        spec=importlib.util.spec_from_file_location('tf.transformations',transformation_path)
        transformations=importlib.util.module_from_spec(spec)
        sys.modules['tf.transformations']=transformations; spec.loader.exec_module(transformations)
        inputs[str(transformation_path)]=sha(transformation_path)
        import rospy
        for name in ('init_node','Publisher','Subscriber','Service','ServiceProxy','wait_for_service','set_param','delete_param','get_master'):
            setattr(rospy,name,forbidden)
        for name in ('loginfo','logwarn','logerr','logdebug','loginfo_throttle','logwarn_throttle','logerr_throttle','logdebug_throttle'):
            setattr(rospy,name,lambda *args,**kwargs: None)
        rospy.Time.now=staticmethod(lambda: rospy.Time(*divmod(STAMP,10**9)))
        source=work/BASE
        spec=importlib.util.spec_from_file_location('gen43_original_remote_calculation',source)
        remote=importlib.util.module_from_spec(spec); sys.modules[spec.name]=remote; spec.loader.exec_module(remote)
        from alicia_flexible_grasp.vision.rgbd_snapshot import RgbdSample, fuse_stable_samples
        from alicia_flexible_grasp.vision.target_observation import TargetTrackIdentity, observation_from_snapshot
        from alicia_flexible_grasp.vision.object_geometry import GeometryEstimate
        from alicia_flexible_grasp.vision.multiview_surface import FusedTargetSurface, SurfaceView, RegistrationResult, RegistrationConfig
        from alicia_flexible_grasp.vision.regular_box_inference import qualify_regular_box_snapshot, enclose_inferred_box_geometry
        from alicia_flexible_grasp.grasp.contact_height_placement import place_contact_height
        from alicia_flexible_grasp.grasp.contact_execution import plan_contact_evaluator, plan_aperture_evaluator
        from alicia_flexible_grasp.grasp.gripper_geometry import evaluate_explicit_candidate
        from alicia_flexible_grasp.grasp.rich_plan_integrity import compute_plan_id
        from sensor_msgs.msg import JointState
        config_path=work/'src/alicia_flexible_grasp_supervisor/config/grasp_params.yaml'
        inputs[str(config_path)]=sha(config_path)
        config=yaml.safe_load(config_path.read_text())
        geometry_cfg=config['grasp_6d']['remote']['gripper_geometry']
        archived=read(G/'perception_execution_configuration.json')['parameters']
        audit=read(G/'keyframes_audits'/AUDIT)
        meta=read(I/'request18'/str(STAMP)/'metadata.json')
        sdk_path=NV/'gen43_request18_sdk_snapshot_r1/result.json'
        assert sha(sdk_path)=='ba023d03ff6a7d647c70db297365c7eabe025f27ff34ebace00a280adec7d8d1'
        sdk=read(sdk_path)
        errors=[dict(json.loads(r['data']),bag_receive_ns=r['bag_stamp_ns']) for r in sdk['runtime_execution_error_snapshots']
            if r['bag_stamp_ns'] <= STAMP and json.loads(r['data'])]
        assert errors
        error=errors[-1]
        assert error['position_error_m']==.013084139731526678
        remote_cfg=deepcopy(config['grasp_6d']['remote'])
        remote_cfg['multiview']=archived['/grasp_6d/remote/multiview']
        camera=dict(meta['intrinsics'])
        K=NS(**camera)
        urdf=work/'deployment/2026-10-08/runtime/candidate_models/expanded_metric_20260929/alicia_expanded_metric_candidate.urdf'
        inputs[str(urdf)]=sha(urdf)
        params={'/grasp':archived['/grasp'],'/grasp_6d/remote':remote_cfg,
            '/grasp_6d/runtime_execution_error':error,'/camera':camera,'/gripper':config['gripper'],
            '/robot_description':urdf.read_text(),'/perception':config.get('perception',{}),
            '/grasp/unknown_require_measured_bilateral_contact':archived['/grasp']['unknown_require_measured_bilateral_contact']}
        def get_param(name,default=None):
            reads.append(name)
            if name in params: return deepcopy(params[name])
            for root in ('/grasp_6d/remote','/grasp','/camera','/gripper','/perception'):
                if name.startswith(root+'/'):
                    value=params[root]
                    try:
                        for part in name[len(root)+1:].split('/'): value=value[part]
                        return deepcopy(value)
                    except (KeyError,TypeError): return deepcopy(default)
            raise ValueError('unprovided historical parameter '+name)
        rospy.get_param=rospy.get_param_cached=get_param
        node=remote.RemoteGrasp6DNode.__new__(remote.RemoteGrasp6DNode)
        env=dict(vars(remote),self=node,remote_cfg=remote_cfg,gcfg=archived['/grasp'],
            grasp_cfg=archived['/grasp'],cam_cfg=camera,gripper_cfg=config['gripper'],
            gripper_geometry_cfg=dict(remote_cfg['gripper_geometry']),default_soft_weights=remote.SoftScoreWeights(),
            soft_weight_cfg=remote_cfg.get('soft_score_weights',{}))
        names={'gripper_geometry','opening_fit_clearance_each_side_m','camera_depth_scale',
            'tabletop_geometry_enabled','tabletop_geometry_config','hybrid_merge_config','adaptive_stage_limits',
            'runtime_execution_error_max_age_sec','gripper_tool_jaw_axis','gripper_tool_finger_length_axis',
            'soft_score_weights','gripper_physical_open_width_m','candidate_min_downward_approach_cos',
            'candidate_max_final_approach_lateral_m','planning_mask_min_iou','planning_mask_max_centroid_shift_px',
            'planning_max_joint_delta_rad','mask_erosion_px','mask_internal_hole_max_area_px','depth_mad_scale',
            'depth_mad_absolute_floor_m','grasp_config'}
        tree=ast.parse(source.read_text())
        init=next(f for c in tree.body if isinstance(c,ast.ClassDef) and c.name=='RemoteGrasp6DNode'
            for f in c.body if isinstance(f,ast.FunctionDef) and f.name=='__init__')
        configured=[]
        for statement in init.body:
            if not isinstance(statement,ast.Assign): continue
            attrs=[n.attr for target in statement.targets for n in ast.walk(target)
                if isinstance(n,ast.Attribute) and isinstance(n.value,ast.Name) and n.value.id=='self']
            if attrs and all(n in names for n in attrs):
                exec(compile(ast.Module(body=[statement],type_ignores=[]),str(source),'exec'),env)
                configured.append(dict(line=statement.lineno,names=attrs))
        node.grasp_config=deepcopy(archived['/grasp'])
        mode_path=work/'src/alicia_grasp_modes/scripts/mode_aware_remote_grasp6d.py'
        mode_tree=ast.parse(mode_path.read_text())
        reset=next(f for c in mode_tree.body if isinstance(c,ast.ClassDef) for f in c.body
            if isinstance(f,ast.FunctionDef) and f.name=='_reset_mode')
        node._carton_internal_hole_area_px=node.mask_internal_hole_max_area_px
        node._mode_selection=deepcopy(audit['grasp_mode_selection'])
        assert node._mode_selection['mode']=='unknown' and node._mode_selection['strategy']=='observe_return'
        assignment=next(n for n in ast.walk(reset) if isinstance(n,ast.Assign)
            and any(isinstance(t,ast.Attribute) and t.attr=='mask_internal_hole_max_area_px' for t in n.targets))
        exec(compile(ast.Module(body=[assignment],type_ignores=[]),str(mode_path),'exec'),dict(vars(remote),self=node,selected=node._mode_selection))
        assert node.mask_internal_hole_max_area_px==0
        identity=TargetTrackIdentity(audit['target_epoch'],audit['multiview_surface']['target_track_id'])
        stamps=tuple(audit['pipeline_funnel']['snapshot_evidence']['source_stamp_ns'])
        samples, source_joints=[] ,[]
        for stamp in stamps:
            folder=I/'request18'/str(stamp)
            data=read(folder/'metadata.json')
            arrays={}
            for role in ('color_bgr','depth_raw','mask_uint8'):
                path=folder/(role+'.npy'); inputs[str(path)]=sha(path)
                arrays[role]=np.load(path,allow_pickle=False)
                assert hashlib.sha256(np.ascontiguousarray(arrays[role]).tobytes()).hexdigest()==data['files'][role]['decoded_array_sha256']
            js=max((r for r in sdk['joint_samples'] if r['topic']=='/alicia_d/accepted_joint_states'
                and r['frame_id']=='sdk_measured' and r['header_stamp_ns']<=stamp),key=lambda r:r['header_stamp_ns'])
            age=stamp-js['header_stamp_ns']; assert 0<=age<200_000_000
            source_joints.append(dict(js,source_stamp_ns=stamp,source_minus_joint_header_ns=age))
            obj=data['object_message']['value']; qmap=dict(zip(js['names'],js['positions']))
            bbox=tuple(obj[k] for k in ('bbox_x','bbox_y','bbox_width','bbox_height'))
            samples.append(RgbdSample(arrays['color_bgr'],arrays['depth_raw'],arrays['mask_uint8'],bbox,
                NS(detected=obj['detected'],label=obj['label'],confidence=obj['confidence']),stamp/1e9,
                data['files']['color_bgr']['frame_id'],np.asarray([qmap['Joint'+str(i)] for i in range(1,7)]),stamp,identity.epoch,identity))
        # Original node's explicit pre-initialization config branch, with the
        # historical camera scale. All actual target depths are interior to
        # both recorded camera range and this original planning range.
        depth_scale,depth_min,depth_max=node._snapshot_depth_config()
        snapshot=fuse_stable_samples(samples,True,node.planning_mask_min_iou,
            node.planning_mask_max_centroid_shift_px,node.planning_max_joint_delta_rad,
            node.mask_erosion_px,depth_scale,depth_min,depth_max,node.depth_mad_scale,
            node.depth_mad_absolute_floor_m,internal_hole_max_area_px=node.mask_internal_hole_max_area_px)
        assert snapshot.ok,(snapshot.failure_code,snapshot.failure_reason)
        assert snapshot.sample_stamp_ns==stamps and snapshot.quality.fused_frames==3
        original_repeat=audit['remote_diagnostics']['tabletop_geometry']['depth_repeatability_m']
        assert snapshot.quality.depth_repeatability_m==original_repeat,(snapshot.quality.depth_repeatability_m,original_repeat)
        assert snapshot.sample_mask_sha256==tuple(hashlib.sha256(np.ascontiguousarray((s.object_mask>0).astype(np.uint8)).tobytes()).hexdigest() for s in samples)
        assert snapshot.sample_mask_sha256[-1]==audit['remote_diagnostics']['tabletop_geometry']['regular_box_inference']['final_sample_binary_mask_sha256']
        geo=audit['replay_geometry']
        geometry=GeometryEstimate(True,'','',geo['obb_center_base_m'],geo['R_base_obb'],geo['obb_size_xyz_m'],
            geo['support_normal_base'],geo['support_offset_m'],geo['support_inlier_ratio'],geo['object_points_base_m'],geo['source_mode'])
        transform=np.asarray(audit['snapshot_transform']['T_base_optical'])
        result=qualify_regular_box_snapshot(snapshot,K,K.depth_scale,transform,geometry,identity,STAMP)
        association=deepcopy(meta['intrinsics_evidence'])
        association.update(parameter_read_finished_ns=association['parameter_read_interval']['finished']['ros_time_ns'],
            source_image_stamp_ns=STAMP,bag_receive_ns=association['bag_record_stamp_ns'],
            calibration_time_association='parameter read completed before source; bag receive after source; no per-image CameraInfo calibration association')
        record.update(snapshot_quality=asdict(snapshot.quality),three_source_fusion_recreated=True,
            original_repeatability_exact=True,historical_fused_pixel_array_exactness_claimed=False,
            parameter_basis='sealed startup YAML plus archived grasp/multiview; original constructor defaults; no complete live remote dump',
            archived_mode_selection=node._mode_selection,source_joints=source_joints,source_stamps_ns=stamps,
            raw_retained_source_stamp_ns=snapshot.original_sample.stamp_ns,qualification_ok=result.ok,
            qualification_code=result.code,qualification_audit=result.audit,camera_parameter_timing=association,
            original_runtime_execution_error=error,original_configured_assignments=configured,
            depth_fusion_range_basis=dict(scale=depth_scale,min_m=depth_min,max_m=depth_max,
                original_method='_snapshot_depth_config pre-initialized branch; no active ROS node',
                source_nonzero_target_raw_ranges_m=[[float(np.min(s.depth_raw[(s.object_mask>0)&(s.depth_raw>0)]))*depth_scale,
                    float(np.max(s.depth_raw[(s.object_mask>0)&(s.depth_raw>0)]))*depth_scale] for s in samples]))
        if result.ok:
            planning,enclosure=enclose_inferred_box_geometry(geometry,result.model)
            refs=audit['multiview_surface']['fused_scene_geometry']['surface_replay']['references']
            receipts={}
            for frame,row in refs.items():
                path=Path(row['store_root'])/row['relative_path']; inputs[str(path)]=sha(path)
                assert sha(path)==row['gzip_sha256']
                raw=gzip.decompress(path.read_bytes()); assert hashlib.sha256(raw).hexdigest()==row['object_sha256']
                receipts[frame]=json.loads(raw)
            reference=receipts['reference']; ref=reference['reference']
            node._stream_condition=threading.Condition(threading.RLock())
            node._geometry_state_lock=threading.RLock()
            node._stream_generation=reference['request_binding']['generation']
            node._near_field_phase_id=reference['request_binding']['phase_id']
            node.near_field_planning_active=True
            node._current_stream_target_identity=lambda: identity
            node._near_field_fused_surface=FusedTargetSurface(identity,reference['points_base'],reference['view_indices'],reference['view_stamps_ns'],view_sources=reference['view_sources'])
            node._near_field_reference_view=SurfaceView(identity,ref['stamp_ns'],ref['points_base'],ref['support_normal_base'],ref['support_offset_m'],ref['edge_clearance_px'])
            node._near_field_surface_generation=node._stream_generation
            node._near_field_surface_phase_id=node._near_field_phase_id
            node._latest_target_observation=observation_from_snapshot(snapshot,geometry,transform)
            reg=audit['multiview_surface']['latest_registration']
            registration=RegistrationResult(reg['ok'],reg['code'],reg['snapshot_to_reference_transform'],reg['inlier_count'],
                reg['overlap_fraction'],reg['rmse_m'],reg['support_normal_angle_deg'],reg['support_plane_separation_m'])
            node._latest_registration_evidence=remote.RegistrationEvidence(identity,STAMP,registration,reg['source_clipped'])
            node.multiview_registration_config=RegistrationConfig(**archived['/grasp_6d/remote/multiview'])
            current=sdk['nearest_sdk_measured']; joint=JointState()
            joint.header.stamp=rospy.Time(*divmod(current['header_stamp_ns'],10**9))
            joint.header.frame_id=current['frame_id']; joint.name=current['names']; joint.position=current['positions']
            node.latest_joint_state=joint
            tool_audit=node._inferred_contact_current_tool_audit()
            assert tool_audit['available'],tool_audit
            node._regular_box_snapshot=dict(lifecycle=node._multiview_lifecycle_token(),observation=node._latest_target_observation,
                result=result,geometry=geometry,planning_geometry=planning,enclosure=enclosure,current_tool=tool_audit)
            actual_surface,_=node._snapshot_multiview_surface()
            np.testing.assert_array_equal(actual_surface.points_base,np.asarray(receipts['snapshot']['points_base']))
            assert node._current_inferred_box() is result.model
            candidates,diagnostics=node._generate_tabletop_candidates(planning,snapshot=snapshot,
                contact_execution_phase=True,target_label='unknown_small',refine_obb_neighbor_angles=True)
            save('materialization_diagnostics.json',diagnostics)
            save('materialized_candidates.json',[dict(source_index=c.source_index,variant_index=c.variant_index,
                T_base_tool0=c.T_base_tool0,contact_center_base=c.contact_center_base,audit=dict(c.audit)) for c in candidates])
            prepared=NS(geometry=planning,snapshot=snapshot,stamp=rospy.Time(*divmod(STAMP,10**9)),near_field=True,
                target_observation=node._latest_target_observation)
            gates=[]
            for item in candidates:
                row=dict(source_index=item.source_index,variant_index=item.variant_index,initial_transform=item.T_base_tool0)
                try:
                    normalized=node._normalize_tabletop_candidate(prepared,item)
                    row.update(initial_analytic_gate=asdict(normalized.geometry_gate),normalization_ok=True)
                except remote.CandidateContractError as exc:
                    row.update(normalization_ok=False,code=exc.code,reason=str(exc))
                    gates.append(row); continue
                surface,evidence,bounds,axis=node._current_bilateral_surface_measurement(item.contact_center_base,item.T_base_tool0[:3,:3])
                profile=node._contact_stage_profile(planning,item.insertion_axis_base,snapshot=snapshot)
                minimum=node._phase_contact_overlap_requirement_m(True,profile)
                placed,height=place_contact_height(translation_evidence_limit_m=node._contact_height_evidence_limit(),
                    **node._contact_height_geometry(item.T_base_tool0,planning,item.contact_center_base,bounds,axis,minimum))
                row['contact_height_placement']=height
                if placed is None:
                    row.update(final_analytic_ok=False,code='CONTACT_HEIGHT_REGISTRATION_MARGIN',reason=height['reason'])
                    gates.append(row); continue
                placed_item=replace(item,T_base_tool0=placed)
                final=node._normalize_tabletop_candidate(prepared,placed_item)
                header=deepcopy(final.grasp_sequence.grasp.header)
                geometry_msg=remote.geometry_estimate_to_message(planning,stamp=prepared.stamp,target_identity=identity)
                encoded=node._encode_current_contact_evidence(evidence,contact_center_base=item.contact_center_base,
                    contact_height_axis_base=axis,contact_height_bounds_m=bounds,minimum_contact_overlap_m=minimum,
                    grasp_pose=final.grasp_sequence.grasp,required_open_width_m=final.required_open_width_m)
                plan=remote.build_rich_plan(final,geometry_msg,header,'unknown_tabletop',encoded)
                plan.diagnostic=remote.CONTACT_EXECUTION_PLAN
                registration_applied=node._apply_registration_evidence_to_plan(plan,prepared.target_observation)
                assert registration_applied and plan.refinement_status=='VALID_3D'
                runtime=dict(prepared=prepared,scored_candidate=NS(stable_candidate=NS(
                    center_base_xyz=item.contact_center_base,required_open_width_m=final.required_open_width_m),payload=final),
                    adaptive_stage_profile=profile)
                try:
                    node._prepare_final_registered_contact_plan(plan,prepared,runtime)
                except remote.CandidateContractError as exc:
                    row.update(final_analytic_ok=False,code=exc.code,reason=str(exc),
                        registration_applied=True,final_registered_runtime=runtime)
                    row['final_registered_runtime']={k:v for k,v in runtime.items() if k not in ('prepared','scored_candidate','adaptive_stage_profile')}
                    gates.append(row); continue
                plan.plan_id=compute_plan_id(plan)
                registered_sequence=node._rich_plan_execution_sequence(plan)
                current_open=float(current['positions'][current['names'].index('right_finger')])
                consumer=plan_contact_evaluator(plan,geometry_cfg,current_open,
                    translation_evidence_limit_m=node._contact_height_evidence_limit(),
                    require_measured_bilateral_contact=False)
                frozen_contact=consumer(remote.pose_matrix(registered_sequence.grasp))
                apertures=[]
                for pose_index,target in enumerate((registered_sequence.pregrasp,registered_sequence.approach,registered_sequence.grasp)):
                    aperture=plan_aperture_evaluator(plan,geometry_cfg,current_open,pose_index,
                        require_measured_bilateral_contact=False)(remote.pose_matrix(target))
                    apertures.append(dict(stage_index=pose_index,report=aperture,
                        actual_measurement=False,evaluation_input='planned stage pose only'))
                assert frozen_contact['ok'] and frozen_contact['height_registration_margin']['ok']
                assert all(r['report']['ok'] for r in apertures)
                necessary=node._contact_goal_inevitable_collision(plan,node.gripper_geometry,
                    float(config['gripper']['open_position_m']),float(config['gripper']['open_skip_tolerance_m']))
                buffer=io.BytesIO(); plan.serialize(buffer)
                payload_name='geometry_only_rich_plan_%s_%s.bin'%(item.source_index,item.variant_index)
                with (out/payload_name).open('xb') as stream: stream.write(buffer.getvalue())
                row.update(registration_applied=True,final_registered_runtime={k:v for k,v in runtime.items()
                    if k not in ('prepared','scored_candidate','adaptive_stage_profile')},
                    frozen_contact_consumer_report=frozen_contact,frozen_aperture_consumer_reports=apertures,
                    geometry_only_rich_plan_payload=payload_name,geometry_only_rich_plan_payload_sha256=sha(out/payload_name),
                    final_registered_four_stage_T_base_tool0=[remote.pose_matrix(p) for p in (registered_sequence.pregrasp,registered_sequence.approach,registered_sequence.grasp,registered_sequence.lift)],
                    final_analytic_ok=bool(final.geometry_gate.ok),final_analytic_gate=asdict(final.geometry_gate),
                    final_transform=placed,necessary_endpoint_collision=necessary,plan_id=plan.plan_id,
                    four_stage_T_base_tool0=[remote.pose_matrix(p) for p in (final.grasp_sequence.pregrasp,final.grasp_sequence.approach,final.grasp_sequence.grasp,final.grasp_sequence.lift)],
                    plan_geometry_only=True,plan_published=False,IK_or_full_arm_path_verified=False)
                gates.append(row)
            save('analytic_contact_gates.json',gates)
            record.update(model_sha256=result.model.model_sha256,planning_enclosure=enclosure,
                planning_geometry_size_xyz_m=planning.size_xyz_m,original_materializer_used=True,original_final_registered_geometry_gate_used=True,
                materialized_candidate_count=len(candidates),normalization_pass_count=sum(r.get('normalization_ok',False) for r in gates),
                final_analytic_pass_count=sum(r.get('final_analytic_ok',False) for r in gates),
                necessary_endpoint_rejected_count=sum(r.get('necessary_endpoint_collision',{}).get('rejected',False) for r in gates),
                historical_current_tool_audit=tool_audit,historical_open_encoder_m=current['positions'][current['names'].index('right_finger')],
                original_actual_surface_receipt_matched=True,new_adaptive_profiles_recomputed=True,
                geometry_only_plan_generated=any('plan_id' in r for r in gates),plan_published=False,
                frozen_nominal_contact_consumer_pass_count=sum(r.get('frozen_contact_consumer_report',{}).get('ok',False) for r in gates),
                frozen_nominal_aperture_consumer_pass_count=sum(sum(x['report']['ok'] for x in r.get('frozen_aperture_consumer_reports',[])) for r in gates),
                no_executable_plan_claim=True)
        record['passed']=True
        assert all(sha(work/n)==h for n,h in expected.items())
        assert all(sha(path)==h for path,h in inputs.items())
        assert attempts==[]
    except BaseException as exc:
        record.update(error=repr(exc),traceback=traceback.format_exc())
    finally:
        loaded={}
        for name,module in list(sys.modules.items()):
            path=getattr(module,'__file__',None)
            if name.startswith('alicia_flexible_grasp.') and path and str(Path(path).resolve()).startswith(str(work/'src')):
                loaded[str(Path(path).resolve())]=sha(path)
        record.update(boundary_attempts=attempts,parameter_reads=reads,source_and_inputs_sha256=inputs,
            isolated_loaded_source_sha256=loaded,recorded_ns=time.time_ns())
        save('result.json',record)
        print(json.dumps(dict(result=str(out/'result.json'),sha256=sha(out/'result.json'),passed=record['passed'],
            qualification_ok=record.get('qualification_ok'),materialized=record.get('materialized_candidate_count'),
            normalization_pass=record.get('normalization_pass_count'),final_analytic_pass=record.get('final_analytic_pass_count'),error=record.get('error'))))
    return 0 if record['passed'] else 1


if __name__=='__main__':
    raise SystemExit(main())
