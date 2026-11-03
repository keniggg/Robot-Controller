"""Update all current-state aliases after verified shared-reference deployment."""
from pathlib import Path
import json,time
w=Path(__file__).resolve().parents[1]
r=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_restart_20261010T002347Z')
assert Path('/media/zhuyupei/SanStand').is_mount()
d=r/'shared_reference_deployment/result.json'; m=r/'verification/gen10_shared_reference_combined_final/deployment_sources.json'
dep=json.loads(d.read_text());man=json.loads(m.read_text())
assert dep['deployment_passed'] and man['passed'] and man['reviewed']
paths=[r/'current_progress.json',w/'.runtime/latest_full_ros_start.json',w/'.runtime/current_full_latest_ros_20261008.json',w/'.runtime/codex_reload_handoff_20261009.json',r/'codex_ui_analysis/task_resume_checkpoint.json']
for i,p in enumerate(paths):
    data=json.loads(p.read_text())
    backup=r/'shared_reference_deployment'/('progress_before_'+str(i)+'.json')
    if not backup.exists():backup.write_text(json.dumps(data,ensure_ascii=False,indent=2)+'\n')
    data.update(updated_ns=time.time_ns(),active_work='gen11_post_enable_verification',user_message='继续处理',paused_by_user=False,next_action_requires_user_resume=False,automatic_grasp_running=False,automatic_validation_running=False,active_record=None,active_record_dir=None,current_record=None,current_record_closed=True,last_generation=10,last_record_dir=str(r.parent.parent/'grasp_records/20261010T044441.309012Z_081c1da83cbe47cc9da10a94a4578168'),last_record_result='failure',last_result='failure',latest_grasp_result='failure',physical_success_verified=False,physical_grasp_verified=False,manual_control=True,gui_manual=True,pending_stage=None,pending_source_deployed=True,pending_targeted_report=None,pending_combined_report=str(m.parent/'result.json'),combined_regressions_passed=man['unique_combined_tests'],latest_compute_unit=dep['unit'],deployment_verified=True,new_alignment_confirmation_received=False,latest_alignment_is_unused=False,current_alignment_pose_is_current=False,alignment_positions_are_historical_start_measurement=True,new_alignment_ready_pending_fix=False,awaiting_user_alignment=False,awaiting_alignment_confirmation=False,awaiting_actual_new_alignment=False,alignment_control_state='historical actual confirmation; verify fresh same pose across logged positive-enable epoch',next_action='Verify fresh same pose and logged positive-enable epoch; new selection11 through original actuation bootstrap and all path gates.',current_issue='Shared reference classification repaired and deployed. Positive enable is only PENDING until real directional response; grasp success still unverified.',next_resume_context='Shared-reference repaired sources deployed; hardware preserved. Continue fresh same-pose receipt then GEN11; no active experiment yet.',resume_steps=['Verify current measured same pose across recorded authorized positive enable','Run new recorded selection with existing bootstrap and original execution gates'])
    for key in ('source_manifest','latest_source_manifest','current_manifest'):data[key]=str(m)
    for key in ('runtime_deployment','latest_deployment','latest_compute_deployment','latest_runtime_baseline'):data[key]=str(d)
    p.write_text(json.dumps(data,ensure_ascii=False,indent=2)+'\n')
print('updated five current-state pointers')
