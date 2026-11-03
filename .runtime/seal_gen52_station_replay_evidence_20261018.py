"""Read-back of exact-wire replay and independently reviewed endpoint necessary gate."""
import hashlib,json,time
from pathlib import Path
W=Path(__file__).resolve().parents[1]
V=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z/verification')
I=V/'gen52_station_expiry_evidence_r1';O=I/'comparison_review_r1'
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def read(p):return json.loads(Path(p).read_text())
def ref(p):return dict(path=str(p),sha256=sha(p))
def save(p,v):p.write_text(json.dumps(v,ensure_ascii=False,indent=2,allow_nan=False)+'\n');assert read(p)==v
def main():
 assert Path('/media/zhuyupei/SanStand').is_mount()and not O.exists();O.mkdir()
 fan=read(I/'mask_fanin_r1/result.json');joined=read(fan['fanin']['path'])
 assert fan['ready_count']==fan['full_triplet_count']==fan['matched_count']==154
 assert all(x['recorder_all_three_before_geometry_expiry']for x in joined)
 dependencies=[I/'extraction.json',I/'mask_fanin_r1/result.json',I/'callback_context_r1/result.json',I/'task_original_callback_replay_r5/result.json']
 rows=[]
 for variant,scenario in [('baseline','delayed'),('candidate','delayed'),('baseline','duplicate'),('candidate','duplicate'),('baseline','reordered'),('candidate','reordered'),('candidate','ordered'),('candidate','epoch')]:
  d=I/('task_schedules_r1_'+variant+'_'+scenario);r=read(d/'result.json');t=read(d/'trace.json');a=read(d/'admission_costs.json');e=read(d/'join_exceptions.json')
  assert r['completed']and not r['errors']and not r['external_boundary_attempts']
  assert sha(r['loaded_task_source']['path'])==r['loaded_task_source']['sha256']
  retired=[x for x in t if x['tag']=='retire'];last=t[-1]
  if scenario=='epoch':assert r['retired']and retired[0]['reason']=='station_task_control_epoch_changed'and last['authority_revoked']
  elif variant=='candidate':assert not r['retired']and r['final_source']==1792330054187593221 and not last['authority_revoked']
  else:assert r['retired']and retired[0]['active_source_ns']==1792330050083012580 and retired[0]['reason']=='station_compute_wait_expired_or_future'
  rows.append(dict(variant=variant,scenario=scenario,result=ref(d/'result.json'),loaded_task_source=r['loaded_task_source'],trace=ref(d/'trace.json'),exceptions=ref(d/'join_exceptions.json'),costs=ref(d/'admission_costs.json'),retired=r['retired'],final_source_ns=r['final_source'],final_geometry_pending=last['pending'],final_authority_revoked=last['authority_revoked'],retire_ns=None if not retired else retired[0]['now_ns'],retired_source_ns=None if not retired else retired[0]['active_source_ns'],reason=None if not retired else retired[0]['reason'],admission_context_calls=len(a),injected_admission_context_total_ns=sum(x['injected_admission_context_cost_ns']for x in a),join_reject_count=len(e)))
  dependencies.extend([d/'result.json',d/'trace.json',d/'join_exceptions.json',d/'admission_costs.json',d/'helper_source.py'])
 original=read(I/'task_original_callback_replay_r5/result.json');assert original['completed']and not original['retired']and original['final_source']==1792330054187593221
 actual_failure_ns=1792330055166587829
 result=dict(passed=True,scope='exact_G51_wire_fanin_and_Task_callback_scheduling_sensitivity_only',
  historical=dict(record='/media/zhuyupei/SanStand/实验记录/Robot-Controller/grasp_records/20261018T132526.747381Z_787872d637b749b29ea988feb844e2e8',task_failure_source_ns=actual_failure_ns,reason='TARGET_LOST: station_compute_wait_expired_or_future',READY_count=154,all154_recorded_complete_detected_matching_mask_triplets=True,all154_recorder_completion_before_geometry_deadline=True,last_three_fanin=[{k:x[k]for k in ('source_ns','status_bag_timestamp_ns','object_bag_timestamps_ns','mask_bag_timestamps_ns','geometry_deadline_ns','identity_deadline_ns','recorder_latest_ns')}for x in joined[-3:]],
    Task_callback_ingress_and_lock_processing_not_recorded=True,historical_expired_registered_contract_source_proven=False,
    source_0050083012580_is_counterfactual_reproduced_pin_not_observed_historical_pin=True,
    producer_READY_and_recorded_transport_complete_but_actual_Task_admission_gap_remains_unobservable=True),
  original_ordered_baseline_result=ref(I/'task_original_callback_replay_r5/result.json'),
  comparison=rows,
  controlled_schedule=dict(mask_sources_delayed_350ms=[1792330051304566860,1792330051879674911],admission_context_cost_ns=100000000,admission_cost_start_ns=1792330051900000000,
    reordered_status_additional_delay_ns=1000000,duplicate_raw_message_delay_ns=1000000,
    SDK_provider='Original accepted feedback appends independently at recorder timestamps while the modeled admission work holds the planning lock. No fabricated positions or future feedback.',
    epoch_negative='Synthetic real following_epoch callback changes control_epoch_ns by one at1792330054700000000; all original target messages remain untouched.',
    exact_historical_subscriber_schedule_or_RPC_cost_claimed=False),
  established_defect='Expired READY cache entries repeatedly perform live admission/config/SDK checks before cheap age rejection. Under bounded processing delay they consume the fresh READY window, preserving the old pin until its unchanged identity expires.',
  candidate_change='Pure validate_registration moved ahead of live _station_task_context. All original current-owner/SDK/source/recovery/final-registration checks still run before new registration; no duration/threshold increase.',
  boundaries=dict(no_movement_no_service_calls=True,no_production_or_old_tests_changed=True,no_temperature_read=True,no_ROS_initialization_or_hardware=True,
    initialization='Recorded original owner/session/phase/qualified lease/plan/camera and accepted SDK history prebound into real original Task fixture; preceding physical observation not replayed.',
    original_callback_order_is_unobserved=True,candidate_end_geometry_pending_cannot_authorize_motion=True,
    real_grasp_success=False,full_regression_or_deployment_not_reviewed_here=True),
  dependency_sha256={str(p):sha(p)for p in dependencies},helper_sha256=sha(__file__),created_ns=time.time_ns())
 save(O/'result.json',result)
 R=V/'gen52_contact_terminal_independent_static_review_r1';assert not R.exists();R.mkdir()
 C=W/'.runtime/gen52_contact_endpoint_candidate'
 paths=[C/'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot/contact_sequence_precheck.py',C/'src/alicia_flexible_grasp_supervisor/tests/test_gen52_contact_terminal_collision_screen.py',C/'src/alicia_flexible_grasp_supervisor/tests/fixtures/gen52_g51_contact_collision_inputs.json']
 review=dict(passed=True,review_scope='static_same_proposed_curve_terminal_necessary_collision_filter_and_new_tests',blocking_findings=[],
    findings=['Canonical FK/scene/gripper required; fallback retains original checker for noncanonical/invalid shapes.','Same _stage_boxes/analytical padding/_obb_overlap against the same frozen target envelope as original continuous validator.','Only an exact terminal overlap rejects; no endpoint pass accepts a full path or any other IK branch.','Retiming must preserve joint geometry; every survivor still runs original nominal and predicted full continuous gates plus whole-arm collision service and remaining deadline checks.','Replay receipt marks endpoint-only, first-collision ordering unclaimed; control/deadline before and between screens is preserved.'],
    test_review=['Three exact recorded terminal counterexamples and typed replay sink.','Clear endpoint/interior collision invokes real original kernel and rejects.','Clear endpoints invoke both real original continuous kernels and original whole-arm boundary.','Deadline/control failures before and between screens cannot become success.'],
    material_limitations=['This is necessary rejection of the existing proposed curve, not proof all IK branches/poses are infeasible.','No tests were executed by this static review; no physical future path was reconstructed as measured success.'],
    reviewed_files_sha256={str(p):sha(p)for p in paths},dependency_sha256={str(W/'.runtime/gen52_baseline_source/src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot/observation_path_guard.py'):sha(W/'.runtime/gen52_baseline_source/src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot/observation_path_guard.py'),str(O/'result.json'):sha(O/'result.json')},
    production_or_tests_modified=False,tests_or_ROS_or_hardware_run=False,temperature_read=False,created_ns=time.time_ns())
 save(R/'result.json',review)
 print(json.dumps(dict(comparison=ref(O/'result.json'),endpoint_review=ref(R/'result.json')),ensure_ascii=False))
if __name__=='__main__':main()
