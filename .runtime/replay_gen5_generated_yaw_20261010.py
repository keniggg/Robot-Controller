#!/usr/bin/env python3
"""Exact frozen-input candidate generation; no ROS actions or motion authority."""
import json,sys,argparse,hashlib,math
from pathlib import Path
from types import SimpleNamespace as NS
import numpy as np
import yaml
import test_remote_grasp6d_streaming as f
r=f.remote_node
r.rospy.logwarn=lambda *args,**kwargs:None
import importlib.util
sys.modules['_alicia_original_remote_grasp6d']=r
spec=importlib.util.spec_from_file_location('gen5_mode_replay',Path('.runtime/gen5_contact_yaw_staging_20261010/src/alicia_grasp_modes/scripts/mode_aware_remote_grasp6d.py'))
mode=importlib.util.module_from_spec(spec);spec.loader.exec_module(mode)
from alicia_flexible_grasp.robot.contact_sequence_precheck import contact_precheck_gripper
from alicia_flexible_grasp.grasp.gripper_geometry import _stage_boxes,_obb_overlap
from alicia_flexible_grasp.grasp.contact_height_placement import place_contact_height
from alicia_flexible_grasp.vision.multiview_surface import FusedTargetSurface, SurfaceView
BASE=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
REC=BASE/'grasp_records/20261010T020909.515250Z_ec46b035f0d74e2688bb217beb3da64a'
NEW=BASE/'ros_log/full_latest_restart_20261010T002347Z'
def run():
 audit=json.loads((REC/'keyframes_audits/bb75081aa6bcc160e641481385310be3b02152af763da52f964c570d7afcb42b.json').read_text())
 source=np.load(NEW/'gen5_handoff_analysis/contact_height_placement/exact_snapshot_reconstruction/fused_reconstruction.npz')
 raw=audit['replay_geometry']; geo=NS(center_base=np.array(raw['obb_center_base_m']),axes_base=np.array(raw['R_base_obb']),size_xyz_m=np.array(raw['obb_size_xyz_m']),object_points_base=np.array(raw['object_points_base_m']),support_normal_base=np.array(raw['support_normal_base']),support_offset_m=raw['support_offset_m'])
 node=mode.ModeAwareRemoteGrasp6DNode.__new__(mode.ModeAwareRemoteGrasp6DNode)
 node._observe_return_diagnostic_observation_only=False
 node._mode_selection=dict(mode='unknown',strategy='observe_return');node._observe_return_phase='observing_contact';node.near_field_planning_active=True;node._measurement_remote_request={'state':'qualified'}
 node.gripper_geometry=f.tabletop_gripper();node.gripper_tool_jaw_axis='y';node.gripper_tool_finger_length_axis='z'
 node.tabletop_geometry_enabled=True;node.tabletop_geometry_config=f.TabletopGeometryConfig(angle_step_deg=5.,angle_dedup_deg=1.,max_candidates=32,width_projection_trim_fraction=.01)
 node.gripper_physical_open_width_m=.05;node.opening_fit_clearance_each_side_m=.0005
 cfg=yaml.safe_load(Path('src/alicia_flexible_grasp_supervisor/config/grasp_params.yaml').read_text())
 node.tabletop_geometry_enabled,node.tabletop_geometry_config,node.hybrid_merge_config=r.load_tabletop_geometry_config(cfg['grasp_6d']['remote'],node.gripper_geometry,node.opening_fit_clearance_each_side_m)
 node.grasp_config=cfg['grasp'];node.adaptive_stage_limits=r.load_adaptive_stage_config(cfg['grasp_6d']['remote'])
 node.candidate_min_downward_approach_cos=.65;node.candidate_max_final_approach_lateral_m=.02
 node.soft_score_weights=f.SoftScoreWeights();node.target_absolute_sanity_distance_m=.15
 node.camera_visibility_gate_enabled=False;node.camera_visibility_diagnostic_enabled=False
 node._runtime_execution_position_error_m=lambda:.014261509777136161
 f.configure_identity_handeye(node)
 identity=f.TargetTrackIdentity(16,'g5-t16');stamps=tuple(audit['multiview_surface']['source_stamps_ns']);C=source['registration_transform']
 surface=FusedTargetSurface(identity,source['reference_points'],source['view_indices'],stamps)
 initial=json.loads((REC/'keyframes_audits/21e913cd61d73fde06e3aadb30ed3e077e13cc977b8a900527d1a6ce290d843d.json').read_text())['replay_geometry']
 n_ref=np.array(initial['support_normal_base']); d_ref=initial['support_offset_m']
 ref=SurfaceView(identity,stamps[0],np.array(initial['object_points_base_m']),n_ref,d_ref,12)
 reg=audit['multiview_surface']['latest_registration']; result=r.RegistrationResult(True,'REGISTERED',C,reg['inlier_count'],reg['overlap_fraction'],reg['rmse_m'],reg['support_normal_angle_deg'],reg['support_plane_separation_m'],reg['translation_m'])
 node._active_multiview_surface=lambda:(surface,ref);f.bind_surface_snapshot(node,surface,ref,result)
 node.multiview_registration_config=r.RegistrationConfig()
 snapshot=NS(quality=NS(depth_repeatability_m=audit['remote_diagnostics']['tabletop_geometry']['depth_repeatability_m'],depth_mad_m=audit['remote_diagnostics']['tabletop_geometry']['depth_spatial_mad_m']),target_identity=identity,object_msg=NS(detected=True,label='unknown'),stamp_ns=stamps[-1])
 node.target_instance_epoch=16
 prepared=NS(geometry=geo,stamp=r.rospy.Time(*divmod(stamps[-1],10**9)),near_field=True,snapshot=snapshot,target_observation=node._latest_target_observation,model_choice='unknown_tabletop',pose_estimator=NS(transform_sha256='recorded-source-transform'),ticket=NS(request_id=41,snapshot_stamp_sec=stamps[-1]/1e9,target_epoch=16))
 candidates,diag=node._generate_tabletop_candidates(geo,snapshot=snapshot)
 rows=[];normalized_all=[]
 for item in candidates:
  row=dict(angle=item.audit['sampled_angle_deg'],tilt=item.audit['approach_tilt_deg'],variant=item.variant_index,source=item.source_index)
  try:
   normalized=node._normalize_tabletop_candidate(prepared,item);normalized_all.append(normalized)
   row['required_open_width_m']=normalized.required_open_width_m
   stable=NS(center_base_xyz=normalized.contact_center_base,required_open_width_m=normalized.required_open_width_m,model_choice='unknown_tabletop',approach_base_xyz=normalized.insertion_axis_base)
   grasp=node._reproject_tabletop_stable_pose_for_support_clearance(normalized.grasp_sequence.grasp,stable,geo)
   T=r.pose_matrix(grasp); _,ev,bounds,axis=node._current_bilateral_surface_measurement(stable.center_base_xyz,T[:3,:3])
   seq,profile=node._make_contact_sequence(grasp,geo,insertion_axis_base=T[:3,2],snapshot=snapshot)
   placed,report=place_contact_height(translation_evidence_limit_m=.0026767676132463574,**node._contact_height_geometry(T,geo,stable.center_base_xyz,bounds,axis,node._phase_contact_overlap_requirement_m(True,profile)))
   row.update(bilateral=ev.ok,counts=[ev.negative_jaw_points,ev.positive_jaw_points],height_ok=placed is not None,height_report=report)
   if placed is not None:
    grasp=r.make_pose_stamped('base_link',placed[:3,3],r.quaternion_from_matrix(placed),stamp=grasp.header.stamp)
    seq,profile=node._make_contact_sequence(grasp,geo,insertion_axis_base=placed[:3,2],snapshot=snapshot)
    runtime=dict(prepared=prepared,grasp_pose=seq.grasp,adaptive_stage_profile=profile,scored_candidate=NS(payload=normalized,stable_candidate=stable))
    plan=f.bound_surface_plan(prepared.target_observation);plan.candidate_source='tabletop_geometry';plan.model_choice='unknown_tabletop';plan.diagnostic=r.CONTACT_EXECUTION_PLAN
    plan.poses=[r.deepcopy(getattr(seq,k).pose) for k in ('pregrasp','approach','grasp','lift')]
    assert node._apply_registration_evidence_to_plan(plan,prepared.target_observation)
    node._prepare_final_registered_contact_plan(plan,prepared,runtime)
    Tfinal=r.pose_matrix(node._rich_plan_execution_sequence(plan).grasp)
    # Final gate transforms frozen OBB by the same registration as the poses.
    center=C[:3,:3]@geo.center_base+C[:3,3];axes=C[:3,:3]@geo.axes_base
    row.update(final_geometry_ok=runtime['final_registered_geometry_gate']['ok'],final_margin=runtime.get('final_contact_height_margin'),collisions=[name for name,c,s in _stage_boxes(Tfinal,node.gripper_geometry,.04975,'y','z') if _obb_overlap(c,Tfinal[:3,:3],s,center,axes,geo.size_xyz_m)],final_T=Tfinal.tolist())
  except Exception as ex:row.update(error=type(ex).__name__+': '+str(ex))
  rows.append(row)
 merged=r.merge_hybrid_candidates(tuple(normalized_all),node.hybrid_merge_config)
 observations=[r.CandidateObservation(request_id=41,snapshot_stamp_sec=stamps[-1]/1e9,target_epoch=16,target_track_id='g5-t16',target_label='unknown',model_choice='unknown_tabletop',center_base_xyz=v.contact_center_base,tool0_position_xyz=v.T_base_tool0[:3,3],quaternion_xyzw=r.quaternion_from_matrix(v.T_base_tool0),approach_base_xyz=v.insertion_axis_base,required_open_width_m=v.required_open_width_m,model_width_m=v.model_width_m,model_score=v.model_score,geometry_margin_m=.05-v.required_open_width_m,pre_moveit_score=v.common_physical_cost,payload=v,candidate_source=v.candidate_source,source_lineage=v.source_lineage) for v in merged]
 stable=node._single_request_stable_candidates(observations)
 scored=node._recheck_and_score_stable(prepared,stable)
 unique=node._dedupe_scored_tabletop_candidates_for_moveit(scored)
 unique=node._dedupe_exact_contact_sequences_for_moveit(unique,prepared)
 ranked=sorted(unique,key=node._direct_near_field_moveit_rank_key)
 pipeline=dict(normalized_count=len(normalized_all),merged_count=len(merged),single_snapshot_stable_count=len(stable),rechecked_count=len(scored),unique_count=len(unique),ordered=[dict(track=v.track_id,variant=v.variant_index,angle=v.payload.audit['sampled_angle_deg'],tilt=v.payload.audit['approach_tilt_deg'],width=v.stable_candidate.required_open_width_m,rank_key=node._direct_near_field_moveit_rank_key(v)) for v in ranked])
 source_files=[REC/'keyframes_audits/bb75081aa6bcc160e641481385310be3b02152af763da52f964c570d7afcb42b.json',REC/'keyframes_audits/21e913cd61d73fde06e3aadb30ed3e077e13cc977b8a900527d1a6ce290d843d.json',NEW/'gen5_handoff_analysis/contact_height_placement/exact_snapshot_reconstruction/fused_reconstruction.npz']
 provenance=[dict(path=str(path),sha256=hashlib.sha256(path.read_bytes()).hexdigest()) for path in source_files]
 new_rows=[row for row in rows if abs(row['angle']-52.295652091777455)<1e-9 and row.get('final_geometry_ok') and row.get('collisions')==[]]
 assert len(new_rows)==4, 'bounded new yaw failed original geometry chain'
 assert all(row.get('height_ok') and row.get('bilateral') for row in new_rows)
 assert len(merged)==len(normalized_all), 'same-source merge removed a generated candidate'
 assert len(ranked)>=4 and all(abs(v.payload.audit['sampled_angle_deg']-52.295652091777455)<1e-9 for v in ranked[:4]), 'new yaw did not retain first width stratum'
 assert any(abs(row['angle']-50.)<1e-9 and row.get('collisions') for row in rows), 'recorded old direction no longer reproduces endpoint overlap'
 return dict(provenance=provenance,pipeline=pipeline,motion_authorized=False,physical_actions=0,diagnostics=diag,rows=rows,limitations=['Actual ModeAware candidate generation, merge, single-snapshot stable adaptation, original height/registered geometry and deduplication; no IK/path or SDK authority.', 'Visibility disabled only in offline diagnostic construction; production configuration unchanged.', 'Frozen current-tool pose is absent; equal-width wrist tie ordering is not asserted. Existing OBB-aperture primary bins put all four new directions before old 50deg independent of that tie.', 'The qualified-phase fixture selects the recorded observed-contact branch; this replay does not recreate lease admission or hardware authority.'])
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args();d=run();d=json.loads(json.dumps(d),parse_constant=lambda value:None);a.output.write_text(json.dumps(d,indent=2,allow_nan=False)+'\n');print(json.dumps(dict(pipeline=d['pipeline'],count=len(d['rows']),rows=[{k:v for k,v in row.items() if k in ('angle','tilt','height_ok','bilateral','final_geometry_ok','collisions','error')} for row in d['rows']]),indent=2))
