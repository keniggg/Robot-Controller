#!/usr/bin/env python3
"""Summarize immutable failures and the separate diagnostic profile honestly."""
import hashlib,json
from pathlib import Path
F=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_contact_admission_20261013T011103Z/verification/gen14_arrival_independent')
def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
rounds={};deps={}
for revision in (1,2):
 rows=[]
 for phase in (0,1,2):
  path=F/f'three_owner_v5_phase{phase}_r{revision}/result.json'
  report=json.loads(path.read_text());deps[str(path)]=sha(path)
  rows.append(dict(timer_phase=report['timer_phase'],passed=report['passed'],error=report['error'],
   source_unchanged=report['source_unchanged'],source_sha256=report['source_sha256'],
   script_sha256=report['script_sha256'],actual_task_phases=report['actual_task_phases'],
   qualified_acks=[a for a in report['acks']if a.get('accepted') and a.get('qualified')],
   accepted_qualified_requests=[a for a in report['acks']if a.get('accepted') and a.get('state')=='qualified'],
   frame_results=[{key:r[key]for key in ('source_ns','processing_start_ns','completion_ns','wall_sec','source_age_sec','state','detail','measurement_reason','accepted_sources','measurement_qualified')}for r in report['rows']],
   remote_fusion=report['fusion'],clock_monotonicity=report['clock_monotonicity'],
   sdk_ingress_equivalence=report.get('sdk_ingress_equivalence')))
 rounds[str(revision)]=rows
profile_path=F/'frozen_r2_cprofile_phase0/result.json';profile=json.loads(profile_path.read_text());deps[str(profile_path)]=sha(profile_path)
sdk_path=F/'sdk_ingress_independent/result.json';deps[str(sdk_path)]=sha(sdk_path)
init_path=F/'remote_initial_fusion_r1/result.json';deps[str(init_path)]=sha(init_path)
profile_summary=[]
selected={'_check_measurement_binding','_rgb_interest_source_context','_rgb_bracketed_feedback','_prepare_locked_rgb_hold_reference','refine_train_only_proposal','_samples','freeze_reference','extract_measured_face','_camera_configuration_contract','deepcopy'}
for item in profile['cpu_profiles']:
 profile_summary.append(dict(label=item['label'],total_self_seconds=item['total_self_seconds'],
  selected_functions=[r for r in item['functions']if r['function']in selected]))
receipt=dict(passed=False,budget_acceptance_passed=False,scope='Preserved fixed timer phase matrix; no deployment readiness or grasp success',
 predeclared_timer_phases=[0,1,2],fraction_of_6hz_period=[0.,1./3,2./3],rounds=rounds,
 interpretation=dict(
  r1='Three real failures preserved; replay charged repeated SDK history reconstruction on each clock read, which was a harness artifact.',
  r2='Incremental original receive ingress independently proved equivalent; all three fixed timer phases still genuinely fail original source/capture deadlines or the original source3680062055 identity gate. No qualifying dual ACK or phase/fusion success occurred.',
  historical_fusion='Original historical initial three-frame RGBD fusion is reconstructed through production code and matches recorded center/camera origin exactly and height within 3.47e-18m; this seeds only historical surface, not arrival qualification.',
  cprofile='Separate single predeclared phase0 diagnosis on frozen r2 source. Profiling overhead remains charged but these durations are not budget acceptance evidence. Third-frame hold was not reached in this profile.'),
 profile_summary=profile_summary,dependencies_sha256=deps,
 source_unchanged_in_all_runs=all(row['source_unchanged'] for rows in rounds.values()for row in rows),
 no_ros_master=True,motion_commands_sent=False,script_sha256=sha(Path(__file__)))
output=F/'three_owner_r2_budget_receipt.json';assert not output.exists();output.write_text(json.dumps(receipt,indent=2)+'\n');print(output,sha(output))
