"""Bind the completed G52 failure; no ROS or device access."""
import json,hashlib,time
from pathlib import Path
W=Path(__file__).resolve().parents[1];E=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller');N=E/'ros_log/full_latest_gen41_restart_20261018T011222Z';V=N/'verification';G=E/'grasp_records/20261018T143337.794533Z_bfea8721dba04526980035ef69ebe860';O=V/'gen52_actual_handoff_monitor_r1'
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def read(p):return json.loads(Path(p).read_text())
def ref(p):return dict(path=str(p),sha256=sha(p))
def main():
 assert Path('/media/zhuyupei/SanStand').is_mount()
 s=read(O/'snapshot_1792334098359607700.json');d=read(O/'station_snapshot_1792334098363033740.json');ap=G/'keyframes_audits/404d30752dbb490c83ce54c9d307676d52bb75355b3683a09825d294e293b402.json';assert sha(ap)==ap.stem
 a=read(ap);tg=a['remote_diagnostics']['tabletop_geometry'];box=tg['regular_box_inference'];m=read(G/'manifest.json');assert m['state']=='complete'and m['recorder_closed']and m['result']=='failure'
 diag=d['station_task_diagnostics'][0];events=diag['diagnostic']['before_retire']['recent_join_events'];joins=[x for x in events if x['kind']=='join_accepted'];rejected=[x for x in events if x['kind']=='check_rejected'];last=diag['diagnostic']['before_retire']['current_reference'];assert len(joins)==15 and len(rejected)==1
 assert not diag['diagnostic']['before_retire']['pending_before']and diag['stamp_sec']>s['first_Task_failure']['stamp_sec']and last['identity_deadline_ns']>int(s['first_Task_failure']['stamp_sec']*1e9)
 metric=None
 for line in (N/'gen52_runtime/compute_gateway_deployment_r1/consumers.log').read_text().splitlines():
  if 'remote 6D pipeline metrics: 'in line:
   x=json.loads(line.split('remote 6D pipeline metrics: ',1)[1])
   if x.get('request_id')==18 and x.get('event')=='request_completed':metric=x
 assert metric is not None
 manual=N/'gen52_runtime/after_attempt_manual_r1/result.json'
 result=dict(kind='GEN52_actual_single_attempt_handoff_final_review',passed=True,passed_meaning='Original closed failure and first uncompleted handoff evidence bound, not grasp success.',record=str(G),record_manifest=ref(G/'manifest.json'),record_state=m['state'],actual_result=m['result'],recorder_closed=True,selection=s['selection'],initial_observation_plan=s['initial_observation_plan'],task_first_failure=s['first_Task_failure'],execution_slot_released=s['latest_task_phase'],runner_result=s['runner_result'],
  actual_progress=dict(observation_moved=True,settled_station_and_new_measurement_qualified=True,range_check_completed=True,nearfield_contact_planning_started=True,CONTACT_plan_accepted=False,strict_service_deliveries=len(s['candidate_local_rejections']),pose0_contact_motion=False,held_approach=False,descent=False,close=False),
  first_uncompleted_handoff='Producer contact generation/materialization yielded no local hard-safe plan. No strict consumer request was delivered or rejected.',
  producer=dict(request_id=a['request_id'],generation=a['generation'],target_epoch=a['target_epoch'],snapshot_stamp_ns=a['snapshot_stamp_ns'],sample_stamps_ns=a['pipeline_funnel']['snapshot_evidence']['source_stamp_ns'],original_audit=ref(ap),
   box_qualification=dict(ok=box['ok'],code=box['code'],reason=box['reason'],complete_rgb_residual_max_px=max(box['cuboid_rgb_interval']['full_silhouette_rgb_distances_px']),four_px_raster_gate_passed_before_measured_conflict=True,measured_geometry_conflict=box['measured_geometry_conflict']),
   tabletop={k:tg[k]for k in('proposal_count','materialized_count','materialized_proposal_count','materialized_total_count','failure_code','failure_reason','proposal_reach_rejections','surface_evidence_rejections')},
   learned_first_rejections=dict(actual_raw_candidates=a['remote_diagnostics']['raw_candidates'],actual_returned=a['remote_diagnostics']['returned'],reported_counts=metric['rejection_counts'],counting_caution='Raw12 learned candidates are not13; additional tabletop/center summary counts share the funnel and metric ratios>1 do not count candidates.'),
   pipeline_request_completed={k:metric.get(k)for k in('status','drop_reason','end_to_end_ms','primary_failure','snapshot_evidence','stage_counts')}),
  station_handoff=dict(diagnostic_stamp_sec=diag['stamp_sec'],diagnostic_reason=diag['diagnostic']['reason'],diagnostic_after_Task_failure=True,recent_join_accepted=len(joins),recent_check_rejected=len(rejected),recent_check_rejected_reason=rejected[0]['reason'],last_reference=last,identity_not_expired_at_Task_failure=True,last_reference_geometry_pending_before_retire=False,interpretation='owner_changed retirement follows original Task failure/phase-active withdrawal; it is not the first cause or the prior scheduling-expiry branch.'),
  replay_inputs=dict(exact_original_pixels=ref(O/'exact_box_inputs_r1/result.json'),historical_point_coordinate_review=ref(O/'point1056_origin_r1/result.json'),original_snapshot=ref(O/'snapshot_1792334098359607700.json'),station_snapshot=ref(O/'station_snapshot_1792334098363033740.json')),
  manual_restoration=ref(manual)if manual.exists()else dict(status='root_owned_receipt_not_yet_visible'),
  boundaries=dict(no_ROS_device_or_motion_commands=True,no_temperature_read=True,no_source_or_parameters_changed=True,history_not_rewritten=True,registration_inverse_arithmetic_exact_but_ICP_physical_truth_not_proven=True,no_geometric_threshold_relaxed=True,physical_grasp_success=False),helper_sha256=sha(__file__),created_ns=time.time_ns())
 p=O/'final_review_r1.json';assert not p.exists();p.write_text(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False)+'\n');assert read(p)==result
 print(json.dumps(dict(result=str(p),sha256=sha(p),manifest_sha256=sha(G/'manifest.json'),task_failure=s['first_Task_failure']['message'],tabletop_proposals=tg['proposal_count'],reach_count=len(tg['proposal_reach_rejections']),surface_count=len(tg['surface_evidence_rejections']),raw=a['remote_diagnostics']['raw_candidates']),ensure_ascii=False))
if __name__=='__main__':main()
