#!/usr/bin/env python3
"""Persist the user's offline-only instruction; no ROS or hardware imports."""
import hashlib
import json
import os
import time
from pathlib import Path

W = Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
M = Path('/media/zhuyupei/SanStand')
N = M / '实验记录/Robot-Controller/ros_log/full_latest_restart_20261010T002347Z'
assert os.path.ismount(str(M)), 'EXPERIMENT_STORAGE_UNAVAILABLE'
now = time.time_ns()
out = N / 'gen13_handoff_analysis/offline_boundary'
out.mkdir(exist_ok=True)

def save(p, obj):
    p.write_text(json.dumps(obj, ensure_ascii=False, indent=2) + '\n')

receipt = {
    'recorded_ns': now,
    'user_instruction': '我先关闭机械臂，你先离线处理看看是不是其他地方导致的这个问题',
    'user_reports_external_force_play': '我手轻推机械臂腕部发现存在松动，各个关节并不是锁定得很死的状态，是能有一点幅度摆动的',
    'mode': 'offline_only',
    'poweroff_intent_reported_by_user': True,
    'power_state_independently_verified': False,
    'proven_mechanical_cause_of_plane_error': False,
    'hardware_commands_sent': False,
    'ros_access_permitted_for_current_work': False,
    'new_explicit_ready_authorization_required_before_online_work': True,
    'previous_alignment_epoch_and_pose_are_historical': True,
    'hold_observation_completed_before_instruction': str(N / 'gen13_handoff_analysis/current_hold_readonly/result.json'),
    'hold_observation_limit': 'Published reference constant; measured J3 span 0.0260776734 rad. External-force timing not recorded; not evidence of spontaneous drift or servo torque. SDK command topic in observer did not yield data.',
    'analysis_scope': ['camera projection and rectification', 'depth filter history', 'support comparison math', 'sealed feedback and TF provenance'],
}
save(out / 'user_instruction.json', receipt)
context_path = N / 'gen13_handoff_analysis/mechanical_hypothesis_user_context.json'
context = json.loads(context_path.read_text())
save(out / 'mechanical_context_before.json', context)
context.update({
    'updated_ns': now,
    'later_direct_user_observation': receipt['user_reports_external_force_play'],
    'user_reported_play_observed_under_external_force': True,
    'confirmed_mechanical_looseness': False,
    'confirmed_mechanical_looseness_field_meaning': 'No independently established hardware defect or causal explanation for GEN13 plane error; does not deny the user-reported physical play.',
    'latest_offline_instruction': str(out / 'user_instruction.json'),
})
save(context_path, context)
manifest = N / 'verification/gen10_shared_reference_combined_final/deployment_sources.json'
deployment = N / 'shared_reference_deployment/result.json'
record = M / '实验记录/Robot-Controller/grasp_records/20261010T050849.339487Z_fa4a10a8358a4fa2b80214f1f626b5c7'
pointers = [N / 'current_progress.json', W / '.runtime/latest_full_ros_start.json',
            W / '.runtime/current_full_latest_ros_20261008.json', W / '.runtime/codex_reload_handoff_20261009.json',
            N / 'codex_ui_analysis/task_resume_checkpoint.json']
for i, p in enumerate(pointers):
    obj = json.loads(p.read_text())
    save(out / ('progress_before_%d.json' % i), obj)
    obj.update({
        'updated_ns': now, 'active_work': 'gen13_offline_software_cause_audit',
        'offline_only': True, 'user_message': receipt['user_instruction'],
        'offline_boundary_receipt': str(out / 'user_instruction.json'),
        'paused_by_user': False, 'automatic_grasp_started': False, 'automatic_grasp_running': False,
        'automatic_validation_running': False, 'new_alignment_ready_pending_fix': False,
        'current_alignment_pose_is_current': False, 'latest_alignment_is_unused': False,
        'new_alignment_confirmation_received': False,
        'alignment_positions_are_historical_start_measurement': True,
        'alignment_control_state': 'Historical GEN13 authorization consumed; later manual force and poweroff instruction invalidate continuation.',
        'current_generation': 13, 'last_generation': 13,
        'current_selection': {'mode': 'unknown', 'strategy': 'observe_return', 'generation': 13, 'stamp_ns': '1791608930773618936', 'historical': True},
        'current_record': str(record), 'latest_record': str(record), 'latest_grasp_record': str(record),
        'last_record_dir': str(record), 'active_record': None, 'active_grasp_record': None,
        'active_record_dir': None, 'current_record_closed': True,
        'latest_grasp_result': 'failure', 'latest_grasp_outcome': 'second_observation_raw_support_offset_exceeds_4mm',
        'latest_failure_reason': 'OBSERVATION_MEASUREMENT_FAILED: OBSERVATION_MEASUREMENT_ACK_TIMEOUT; 17 post-arrival frames fail support local distance 5.543–5.904 mm > 4 mm',
        'current_issue': 'Offline audit of persistent second-view support offset. User reports force-induced joint play; causal mechanical defect unproven.',
        'current_deployment': str(deployment), 'latest_deployment': str(deployment),
        'current_deployment_unit': 'alicia-observation-measurement-1791608246796590772',
        'current_deployment_manifest': str(manifest), 'current_manifest': str(manifest),
        'current_deployment_manifest_sha256': hashlib.sha256(manifest.read_bytes()).hexdigest(),
        'current_source_count': 153, 'runtime_identity_fields_are_last_known_not_rechecked': True,
        'control_epoch_is_historical': True, 'gui_state_is_last_known_not_rechecked': True,
        'mechanical_looseness_confirmed': False, 'user_reported_play_observed_under_external_force': True,
        'physical_grasp_verified': False, 'physical_success_verified': False,
        'next_action': 'Complete offline source and sealed-record analysis. No ROS calls, device access, enables, reloads, or motion until new explicit user readiness authorization.',
        'next_resume_context': 'User is powering off arm. All former alignment/epoch/GUI fields are historical; continue offline work only.',
        'resume_steps': ['Complete and document offline analysis', 'Wait for a new explicit user request to restore online operation before accessing ROS or hardware'],
    })
    save(p, obj)

doc = W / 'docs/operations/2026-10-08-full-latest-ros-startup.md'
section = '''

### 2026-10-09 LA：用户关机，转纯离线排查

- 用户随后补充“我手轻推机械臂腕部发现存在松动，各个关节并不是锁定得很死的状态，是能有一点幅度摆动的”。这更新了先前“仅推测”的事实：已有用户报告的外力下摆动，但尚未独立确认机械故障，也未证明它造成 GEN13 平面偏移。不得把先前假设字段当作用户仍未观察到摆动。
- 在本次关机指令之前完成的 10 秒只读观测，83 份 control_reference 均相同，84 份 accepted_joint_states 中 Joint3 跨度 0.0260776734 rad（约 1.49°）。没有外力时间/力值记录，不能据此认定自行漂移；CONFIRMED 表示此前实测方向响应，不是电机保持力矩测量。该观察器未收到 SDK command，不能把 control_reference 等同本次原始线缆命令。
- 用户最新明确“我先关闭机械臂，你先离线处理看看是不是其他地方导致的这个问题”。从此只读源码和已封存记录，允许离线复算/修复验证；不访问 ROS 或设备，不发使能/失能、运动或重载命令。实际断电状态未主动查询。原对准、epoch 和 GUI 状态全部视为历史，下一次在线工作须有新的明确准备授权；离线工作继续。
- 正确运行模型与实录关节/TF 的一致性已核验；独立桌面 RGB-D 配准因重复文字纹理，跨站训练/留出匹配未满足预设门限，不能据此确认标定错误或机械故障。第二站同站留出帧稳定也不能排除受力后静态偏移。第二站角度超出历史独立标定验证覆盖是适用范围限制，不是要求重标定的结论。
- 当前并行离线排查相机投影/整流、深度滤波历史及支撑平面比较数学；生产源码清单仍为 153，原 4 mm/4° 门限不改。本次用户指令、前后进度索引及只读观测局限保存于 gen13_handoff_analysis/offline_boundary。
'''
with doc.open('a') as f:
    f.write(section)
(N / doc.name).write_bytes(doc.read_bytes())
print(json.dumps({'receipt': str(out / 'user_instruction.json'), 'pointers_updated': len(pointers), 'doc': str(doc), 'offline_only': True}, ensure_ascii=False))
