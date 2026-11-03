"""Synchronize resume pointers to sealed gen7 evidence and actual new alignment."""
import hashlib
import json
import time
from pathlib import Path

work = Path(__file__).resolve().parents[1]
ext = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
assert ext.parents[1].is_mount()
session = ext / 'ros_log/full_latest_restart_20261010T002347Z'
index = json.loads((work / '.runtime/current_realigned_20261008.json').read_text())
alignment = Path(index['alignment_receipt'])
assert session in alignment.parents
receipt = json.loads(alignment.read_text())
record = ext / 'grasp_records/20261010T030141.809371Z_7724ffb5910c4565bf751bd9c281954c'
deployment = session / 'callback_handoff_deployment/result.json'
deployed = json.loads(deployment.read_text())
assert deployed['deployment_passed']
manifest = session / 'verification/gen6_callback_handoff_combined/deployment_sources.json'
sources = json.loads(manifest.read_text())
assert (session / 'gen7_handoff_analysis/observers_closed_seal.json').is_file()
common = dict(
    updated_ns=time.time_ns(), current_generation=7, last_generation=7,
    current_selection=receipt['selection'],
    automatic_grasp_started=False, automatic_grasp_running=False,
    automatic_validation_running=False, current_record_closed=True,
    active_record=None, active_grasp_record=None,
    last_record_result='failure', latest_grasp_result='failure', last_result='failure',
    last_record_dir=str(record), latest_record=str(record),
    current_record=str(record), latest_grasp_record=str(record),
    latest_grasp_outcome='second_observation_admission_retired_optional_compute_contract',
    latest_failure_reason='observation_arrival_station_invalidated_during_recovery',
    physical_success_verified=False, physical_grasp_verified=False,
    gui_state_evidence=str(session/'gen8_alignment_before_repair/pose_difference.json'),
    awaiting_user_alignment=False, awaiting_alignment_confirmation=False,
    awaiting_actual_new_alignment=False, new_alignment_ready_pending_fix=True,
    new_alignment_confirmation_received=True, latest_alignment_is_unused=True,
    alignment_receipt=str(alignment), current_alignment_session=str(alignment.parent),
    alignment_message_id=receipt['message_id'], latest_actual_alignment_message=receipt['message_id'],
    current_alignment_received_ns=receipt['receipt_ns'],
    current_alignment_positions_rad=receipt['topics']['/alicia_d/accepted_joint_states']['positions'],
    alignment_consumed_by_record=None, alignment_control_state='user_alignment_received_pending_fix',
    alignment_positions_are_historical_start_measurement=False,
    current_deployment=str(deployment), latest_deployment=str(deployment),
    latest_compute_deployment=str(deployment), current_deployment_unit=deployed['unit'],
    latest_compute_unit=deployed['unit'], latest_runtime_baseline=str(deployment),
    current_manifest=str(manifest), current_deployment_manifest=str(manifest),
    latest_source_manifest=str(manifest), current_source_count=len(sources['source_sha256']),
    current_deployment_manifest_sha256=hashlib.sha256(manifest.read_bytes()).hexdigest(),
    combined_regressions_passed=sources['unique_combined_tests'], deployment_verified=True,
    current_issue='A phase switch retires an optional compute contract while a frame is in flight; this is incorrectly escalated to target loss before second observation motion.',
    active_work='Verify isolated producer retirement repair and exact first-view contact-band evidence.',
    next_action='Review and deploy compute-only repair, verify current pose against actual latest receipt, then record gen8.',
    next_resume_context='Live source141; gen7 first observation qualified, then second-view admission failed before second movement. Actual latest alignment is unconsumed; see alignment_receipt.',
    resume_steps=['Finish frozen gen7 regression and independent review', 'Reload only three compute consumers', 'Compare fresh q and epoch with actual latest receipt', 'Record a new autonomous selection'],
    paused_by_user=False, next_action_requires_user_resume=False,
)
for path in (session/'current_progress.json', work/'.runtime/latest_full_ros_start.json',
             work/'.runtime/current_full_latest_ros_20261008.json',
             work/'.runtime/codex_reload_handoff_20261009.json',
             session/'codex_ui_analysis/task_resume_checkpoint.json'):
    data = json.loads(path.read_text())
    data.update(common)
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2)+'\n')
doc = work/'docs/operations/2026-10-08-full-latest-ros-startup.md'
heading = '### 2026-10-09 LA：gen7第二补视准入的计算合同退休竞争'
if heading not in doc.read_text():
    with doc.open('a') as f:
        f.write('\n\n'+heading+'\n\n'
            '- gen7 已封存 failure。首补视实测最大关节位移1.168893rad，三帧到位、双ACK、CAD审核通过，进入接触规划；source141旧回调超龄问题在本轮未复现。\n'
            '- 第二补视准入时，在途post_segment仍持有被MOVE_PREGRASP回调退休的station-compute合同，被误判station_changed并提升为目标丢失。当前源龄0.176193s；到位至失败208条实测反馈六轴范围均0，无第二运动、接近或闭爪。原始88消息fixture与观察器封存于gen7_handoff_analysis。\n'
            '- 第一补视的新增轴邻近角度确实参与搜索。精确四源1762点融合复建显示，窄边两侧共同高度约0.286mm，交集只剩7/4点；总点多不代表双侧接触带充分。继续核查观察面及滤波影响，未放宽接触门限。\n'
            '- 用户真实新确认call_a0682891287d4429ac21a5cfbe10eb96:0已捕获realigned_20261010T030640Z/alignment_receipt.json，尚未用于新实验。待修复组合验证与独审后部署，仅重载计算节点。仍无实物夹持成功。\n')
(session/doc.name).write_bytes(doc.read_bytes())
print(json.dumps({'updated_pointers':5, 'generation':7, 'new_alignment':str(alignment)},ensure_ascii=False))
