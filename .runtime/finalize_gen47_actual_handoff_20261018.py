#!/usr/bin/env python3
"""Read-only G47 closed-attempt handoff attribution; no ROS/bag/devices."""
from pathlib import Path
import json,hashlib,math,time
from monitor_gen45_actual_handoff_20261018 import read_rows,numeric_time,json_after
W=Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
E=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller');N=E/'ros_log/full_latest_gen41_restart_20261018T011222Z';V=N/'verification'
G=E/'grasp_records/20261018T085552.871836Z_7d7d31b2570b433b8ee05398edc47d73';O=V/'gen47_actual_handoff_monitor_r1'
evidence={}
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def pin(p,expected=None):
 h=sha(p)
 if expected:assert h==expected,(str(p),h,expected)
 evidence[str(p)]=h;return h
def load(p,expected=None):pin(p,expected);return json.loads(Path(p).read_text())
def write(name,obj):
 p=O/name;raw=(json.dumps(obj,ensure_ascii=False,indent=2,allow_nan=False)+'\n').encode()
 with p.open('xb')as f:f.write(raw)
 assert p.read_bytes()==raw
 evidence[str(p)]=sha(p);return dict(path=str(p),sha256=sha(p))
snapshot=load(O/'snapshot_1792313977648650671.json','63b747dff7d063e3e943f025663758f29d117b7c0aa067f3be3158199b611307')
diagnostic=load(O/'diagnostics_1792313977634179015.json','0f0b6e13032eb4dc8ce7c143d5d1296da6e9a76fa0b83580f301377831b0504c')
observation=load(G/'keyframes_audits/1c6773cb6e38e1f5629f664a321702f958bb2fe1118c2fef5777e8f2bedf5d9f.json','1c6773cb6e38e1f5629f664a321702f958bb2fe1118c2fef5777e8f2bedf5d9f')
station=load(G/'keyframes_audits/19674cf4fc7b10aa290adff95422e8911eaafd18a8ab55c242dd5ca1f959cda2.json','19674cf4fc7b10aa290adff95422e8911eaafd18a8ab55c242dd5ca1f959cda2')
manifest=load(G/'manifest.json');command=load(G/'command_result.json');result_summary=load(G/'result_summary.json');manual=load(N/'gen47_runtime/after_attempt_manual_r1/result.json')
assert manifest['result']==command['result']==result_summary['result']=='failure'
assert manual['verified_inactive'] is True and manual['generation']==7
assert diagnostic['selection']==dict(generation=7,mode='unknown',stamp_ns=1792313754948979616,strategy='observe_return')
assert station['request_id']==16 and station['target_epoch']==4 and station['snapshot_stamp_ns']==1792313803719039440
atomic=N.parent/'planning_audits/grasp6d_gate_audit_latest.json'
raw=atomic.read_bytes();assert hashlib.sha256(raw).hexdigest()=='3aee0f2f181f1552cd264d90c6dc1a71d8ff32f5ff5413b2f5e7a57c77ce8f98';a=json.loads(raw)
assert {k for k in set(station)|set(a) if station.get(k)!=a.get(k)}=={'pipeline_metrics'}
(O/'request16_atomic_audit_3aee0f2f.json').write_bytes(raw);pin(O/'request16_atomic_audit_3aee0f2f.json')
consumer=N/'gen47_runtime/compute_gateway_deployment_r1/consumers.log';rows,source_range=read_rows(consumer,4*1024*1024)
metrics={};samples=[];late_loss=[]
for text,ref in rows:
 t=numeric_time(text)
 if t is None or t<diagnostic['selection']['stamp_ns']/1e9:continue
 q=json_after(text,'remote 6D pipeline metrics: ')
 if q and q.get('event')=='request_completed':metrics[q['request_id']]=dict(stamp_sec=t,metrics=q,evidence=ref)
 s=json_after(text,'remote 6D GraspNet candidate rejection samples: ')
 if s is not None and 1792313804<t<1792313807.1:samples=s
 if 'Unknown target first loss' in text:late_loss.append(dict(stamp_sec=t,line=text,evidence=ref))
assert metrics[2]['metrics']['status']=='PREVIEW_READY'
assert metrics[16]['metrics']['status']=='NEAR_FIELD_SURFACE_VIEW_REQUIRED'
assert not any(r['metrics'].get('status')=='ACCEPT_FAILED' for r in diagnostic['request_preparation_events'])
assert len(samples)==8
initial=observation['observe_return']['trigger_evidence'];ib=initial['regular_box_inference'];table=station['remote_diagnostics']['tabletop_geometry'];box=table['regular_box_inference'];cuboid=box['cuboid_rgb_interval']
assert ib['ok'] is False and box['ok'] is False and table['regular_box_planning_enclosure'] is None
assert max(ib['cuboid_rgb_interval']['full_silhouette_rgb_distances_px'])>math.sqrt(8)
assert max(cuboid['full_silhouette_rgb_distances_px'])<=math.sqrt(8) and max(cuboid['corner_rgb_distances_px'])<=math.sqrt(8)
assert cuboid['top_mask_coverage']>=.88 and box['cuboid_qualification_reason']=='common cuboid family has unresolved top/support silhouette visibility'
assert table['proposal_count']==30 and len(table['proposal_reach_rejections'])==22 and len(table['contact_boundary_profiles'])==8 and len(table['surface_evidence_rejections'])==8
assert table['materialized_count']==table['materialized_total_count']==0
assert len(table['sampled_angles_deg'])==38
boundary_max=max(b['maximum_observed_contact_patch_overlap_m'] for row in table['contact_boundary_profiles'] for b in row['branches'] if b['maximum_observed_contact_patch_overlap_m'] is not None)
assert boundary_max==0.0019768228047266556
required_overlap=table['required_contact_patch_overlap_m'];assert required_overlap==.002 and boundary_max<required_overlap
for rel in ['src/alicia_grasp_modes/scripts/mode_aware_remote_grasp6d.py','src/alicia_grasp_modes/src/alicia_grasp_modes/silhouette_observation_planning.py','src/alicia_grasp_modes/scripts/unknown_tabletop_perception.py','src/alicia_flexible_grasp_supervisor/scripts/remote_grasp6d_node.py','src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/cuboid_rgb_interval.py','src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/regular_box_inference.py']:
 pin(W/rel)
local=write('request16_local_geometry_evidence.json',dict(kind='exact_recorded_g47_request16_local_geometry_evidence',request_id=16,snapshot_stamp_ns=station['snapshot_stamp_ns'],audit_sha256=evidence[str(G/'keyframes_audits/19674cf4fc7b10aa290adff95422e8911eaafd18a8ab55c242dd5ca1f959cda2.json')],tabletop_geometry=table,pipeline_funnel=station['pipeline_funnel'],learned_rejection_samples=samples,learned_samples_scope='Only first8 are serialized in production log. Full12 partition is reconstructed from original source/count semantics; do not invent remaining4 detailed exception messages.',actual_request_metrics=metrics[16]))
first_failure=snapshot['first_Task_failure'];assert first_failure['stamp_sec']==1792313807.093189
submit=next(r for r in snapshot['events'] if r['kind']=='actual_submitted_trajectory');assert submit['plan_id']=='d488a32a969d4cb076975ac1' and submit['duration_sec']==30.604
cause=dict(kind='gen47_actual_first_uncompleted_handoff_cause',record=str(G),selection=diagnostic['selection'],result='failure',initial_contact_preflight=dict(request_id=1,snapshot_stamp_ns=initial['snapshot_stamp_ns'],target_epoch=initial['target_epoch'],target_track_id=initial['target_track_id'],contact_search_completed=False,learned_candidates_requested=False,regular_box=ib,measured_missing_sides=initial['measured_missing_sides'],conclusion='Initial box does not qualify: complete cuboid RGB silhouette maximum4.663787697px exceeds unchanged sqrt8 raster gate; missing common bilateral height triggers a measured observation request before learned inference.'),handoffs_completed=dict(original_prepared_identity_rejection_repeated=False,request2_status='PREVIEW_READY',observation_plan_id=submit['plan_id'],actual_observation_submission=submit,qualified_station_stamp_sec=1792313802.201816,waiting_nearfield_stamp_sec=1792313802.717455,station_registration=dict(ok=station['multiview_surface']['latest_registration']['ok'],code=station['multiview_surface']['latest_registration']['code'],inlier_count=station['multiview_surface']['latest_registration']['inlier_count'],rmse_m=station['multiview_surface']['latest_registration']['rmse_m'],fused_point_count=station['multiview_surface']['point_count'],enclosed_point_count=station['multiview_surface']['fused_scene_geometry']['enclosed_point_count'],synthetic_points_added=station['multiview_surface']['fused_scene_geometry']['synthetic_points_added'])),first_uncompleted_handoff=dict(stage='fresh registered station geometry -> locally valid contact plan',request_id=16,stream_generation=7,target_epoch=4,target_track_id='g1-t4',snapshot_stamp_ns=station['snapshot_stamp_ns'],snapshot_sources_ns=station['pipeline_funnel']['snapshot_evidence']['source_stamp_ns'],request_completed=metrics[16]['evidence'],request_completed_stamp_sec=metrics[16]['stamp_sec'],Task_failure=first_failure),station_regular_box=dict(ok=False,code=box['code'],reason=box['reason'],cuboid_qualification_reason=box['cuboid_qualification_reason'],whole_hull_mask_fraction=cuboid['silhouette_mask_coverage'],whole_hull_mask_pixels=[cuboid['silhouette_mask_intersection_pixels'],cuboid['silhouette_mask_eroded_polygon_pixels']],top_mask_fraction=cuboid['top_mask_coverage'],top_mask_pixels=[cuboid['top_mask_intersection_pixels'],cuboid['top_mask_eroded_polygon_pixels']],maximum_complete_rgb_distance_px=max(cuboid['full_silhouette_rgb_distances_px']),maximum_corner_arm_distance_px=max(cuboid['corner_rgb_distances_px']),raster_gate_px=math.sqrt(8),coverage_gate=.88,failed_original_function='cuboid_rgb_interval._assignment_stability',function_semantics='Family interval must preserve which footprint boundaries are physically top/support silhouette. Camera projection interval overlaps an assignment boundary; it is not a low mask coverage failure.',numeric_assignment_margin_available=False,missing_intermediate='axis/yaw family interval is not serialized before this failure; no numeric interval margin is claimed',box_enclosure_created=False),station_tabletop=dict(sampled_angles=38,early_surface_evidence_rejections=8,proposals=30,reach_rejected_proposals=22,maximum_side_reach_m=.025,proposals_entering_contact_materialization=8,boundary_maximum_observed_contact_patch_overlap_m=boundary_max,required_contact_patch_overlap_m=required_overlap,boundary_overlap_shortfall_m=required_overlap-boundary_max,materialized_total=0,variant_rejection_counts=table['rejection_counts'],variant_counts_not_proposal_partition=True,adaptive_stage_profile_count=len(table['adaptive_stage_profiles']),live_depth_uncertainty_m=table['adaptive_stage_profiles'][0]['depth_uncertainty_m'],execution_position_error_m=table['adaptive_stage_profiles'][0]['execution_position_error_m'],conclusion='With no qualified box family, unchanged fused measured bilateral route produces zero candidates. Both missing bilateral evidence and contact patch/collision filters reject materialized variants; max boundary overlap alone is not a complete sole-cause proof.'),learned_fallback=dict(returned_candidates=12,locally_valid=0,first_reject_partition=dict(GRASPNET_STAGE_PROFILE_UNAVAILABLE=6,BILATERAL_SURFACE_EVIDENCE_MISSING=2,GRIPPER_SWEEP_COLLISION=4),top_level_funnel_counts=station['pipeline_funnel']['rejection_counts'],one_extra_source_summary='Base generator adds one tabletop failure_code to funnel rejection Counter; 6+3+4=13 combines12 learned firstrejects and1tabletop summary.',primary_failure=station['pipeline_funnel']['primary_failure'],primary_failure_is_counter_mode_not_missing_profile=True,original_profiles_present=5,hard_maximum_adaptive_tilt_deg=31.808096663632114,all_available_sample_details=local),later_target_loss=late_loss,later_target_loss_interpretation='The inspected first-loss log is after Task FAILED and the far_field phase reset. It is subsequent retirement, not the first failure and not proof of a physical object change.',new_stage0_and_close=dict(contact_plan_accepted=False,zero_step_pregrasp_reached=False,current_CAD_stage0_reached=False,held_approach_reached=False,descent_reached=False,stage2_reached=False,close_reached=False),provider_claims=dict(data='Original sealed runtime audit/logs, no synthetic scene introduced',numerical_replay_executed=False,ROS_or_hardware_access=False,temperature_read=False,source_or_record_modified=False),local_geometry_evidence=local,evidence_sha256=evidence)
cause_ref=write('first_handoff_cause.json',cause)
final=dict(kind='gen47_actual_handoff_monitor_final',timestamp_ns=time.time_ns(),selection=diagnostic['selection'],record=str(G),record_result='failure',record_closed=True,first_cause=cause_ref,GEN47_identity_fix_actual_handoff_passed=True,physical_grasp_verified=False,observation_executed=True,observation_plan_id=submit['plan_id'],contact_plan_accepted=False,first_Task_failure=first_failure,completed_request_count=diagnostic['completed_request_count'],request_accept_failed_count=0,expected_initial_transition='Request1 OBSERVE_RETURN_MEASURED_VIEW_REQUIRED / GENERATION_STALE is the intended measured-view route transition, not GEN46 instance-identity regression.',manual_recovery=dict(path=str(N/'gen47_runtime/after_attempt_manual_r1/result.json'),sha256=evidence[str(N/'gen47_runtime/after_attempt_manual_r1/result.json')],verified_inactive=True,generation=7),source_or_record_modified=False,ROS_or_hardware_access=False,temperature_read=False,evidence_sha256=evidence)
final_ref=write('result.json',final)
seal=write('seal.json',dict(result=final_ref,first_cause=cause_ref,local_evidence=local,helper_sha256=sha(Path(__file__)),evidence_sha256=evidence))
print(json.dumps(dict(result=final_ref,cause=cause_ref,seal=seal),ensure_ascii=False))
