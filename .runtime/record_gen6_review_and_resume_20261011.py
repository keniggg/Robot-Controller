"""Record finished GEN6 and independent terminal review; no ROS operations."""
import hashlib
import json
from pathlib import Path
import time

W = Path(__file__).resolve().parents[1]
E = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
N = E/'ros_log/full_latest_no_skin_20261011T000625Z'
assert Path('/media/zhuyupei/SanStand').is_mount()
A = N/'realigned_20261011T024307Z'
receipt = json.loads((A/'alignment_receipt.json').read_text())
record = str(E/'grasp_records/20261011T023740.243842Z_0e7a6e5935a847a5ae5fcb14772f78a7')
checkpoints = [N/'current_progress.json', W/'.runtime/latest_full_ros_start.json',
 W/'.runtime/current_full_latest_ros_20261008.json', W/'.runtime/codex_reload_handoff_20261009.json',
 N/'codex_ui_analysis/task_resume_checkpoint.json']
reference = json.loads(checkpoints[0].read_text())
alignment_keys = [k for k in reference if 'alignment' in k]
for p in checkpoints:
 d = json.loads(p.read_text())
 d.update({k:reference[k] for k in alignment_keys})
 d.update(paused_by_user=False, automatic_grasp_started=False, automatic_grasp_running=False,
  current_generation=6, current_alignment_session=str(A), latest_alignment_is_unused=True,
  active_work='gen6_observation_search_coverage_and_terminal_reason_fix',
  current_issue='Ordinary information-driven supplemental view search bypasses diagnostic coverage; terminal cause is then masked by task phase validation.',
  next_action='Finish search coverage and terminal reason fixes, deploy compute nodes, verify current alignment and perform GEN7',
  active_record=None, current_record=record, current_record_dir=record, last_record=record,
  latest_finished_record=record, last_result='failure', latest_result='failure',
  last_failure_reason='UNKNOWN_OBSERVATION_NO_SAFE_VIEW: bounded strict search checked 4/220 similar poses; task masked this with NEAR_FIELD_PLAN_PHASE_INVALID',
  physical_grasp_success=False, approach_executed=False, gripper_closed=False,
  manual_control=True, gui_manual=True, gui_manual_restored=True,
  pending_source_changes_deployed=False, runner_exec_session_id=None, observer_exec_session_ids=[],
  updated_ns=time.time_ns())
 p.write_text(json.dumps(d,ensure_ascii=False,indent=2)+'\n')

s=W/'.runtime/gen6_terminal_reason_20261011'
delivery_path=s/'delivery.json'
delivery=json.loads(delivery_path.read_text())
sources={**delivery['files'],**delivery['test_files']}
for f,digest in sources.items():
 assert hashlib.sha256((s/f).read_bytes()).hexdigest()==digest
review=dict(reviewed=True, blocking_findings=[], reviewer='root (not patch author)',
 source_sha256=sources, delivery_sha256=hashlib.sha256(delivery_path.read_bytes()).hexdigest(),
 findings=[
 'Production diff only adds six exact producer terminal codes to the existing whitelist.',
 'Existing invalid-preview flag, typed source, minimum snapshot timestamp and direct-mode checks remain before terminal handling. No plan is made executable.',
 'Actual parser and consumer-loop regressions preserve producer cause and reject stale, malformed, unsupported or wrong-source messages.',
 'Messages have no request ID; existing minimum source-stamp window remains the actual boundary. No stronger cross-request protocol claim.',
 '4.1mm to7.1mm negative-fixture update matches previously deployed7mm support hard limit; added4.1mm pass verifies existing warning-band behavior. No production threshold change.'
 ], evidence=delivery['verification'], offline_only=True, deployment_performed=False)
out=N/'gen6_analysis/terminal_reason_independent_review.json'
assert not out.exists()
out.write_text(json.dumps(review,ensure_ascii=False,indent=2)+'\n')
doc=W/'docs/operations/2026-10-08-full-latest-ros-startup.md'
with doc.open('a') as h:
 h.write('''\n\n#### GEN6 结果与普通补视搜索覆盖遗漏（2026-10-11 UTC）\n\n- GEN6记录 grasp_records/20261011T023740.243842Z_0e7a6e5935a847a5ae5fcb14772f78a7 已封失败，GUI恢复手动，观察器退出；未接近或闭爪。首补视及纠偏入口通过，实际视野40.161px，纠偏服务返回no_motion，随后新三帧和双端qualified通过。GEN5交接时效修复获得实机验证；24px放宽、真实纠偏运动及预抓取6mm偏好尚未实机触发。\n- req12接触搜索39角度中16缺双侧证据、23产生提案，22被夹爪扫掠/单侧25mm范围拒绝；剩余35°共同实测接触带8.610–9.045mm仅0.435mm，小于2mm。严格接触路径尚未开始，不能说本轮又发生预抓取路径碰撞或证明40mm窄边物体不可抓。\n- req13共有220观察候选，3.006秒初始严格预算只检查4个tilt45°/roll−15°、188–190mm距离的相近候选，均完整路径失败；其余216未证不可达。旧距离/方向coverage只在not information_driven诊断分支启用，普通信息驱动补视被漏掉；且任一正向端点IK提示会关闭旧coverage，提示后完整路径失败仍无覆盖恢复。正在同时修这两种入口，保留原预算、全部候选和严格门。\n- producer于1791686326.537发布UNKNOWN_OBSERVATION_NO_SAFE_VIEW/OBSERVATION_SEARCH_INCOMPLETE，task于6326.688误报NEAR_FIELD_PLAN_PHASE_INVALID。另修任务已存在的终止码白名单使首因原样传递；不把原因显示修复称为路径成功。时线 gen6_analysis/observation_handoff_and_terminal_timeline.json，SHA256 db1183979a84b02c33a9e2f978227c50fea6d42bad43a9594f4e9463f93fbd6a。\n- 用户新真实确认call_e2aced1c023d4fb48e3b8fdd9b5d6e48:0已保存realigned_20261011T024307Z，epoch1791677188993077035，目标居中、当前图像未见手；尚未消费。所有新修复仍离线暂存，当前部署仍gen5_handoff_compute_deployment，未失能、未查温度、未启动电子皮肤。\n''')
(N/doc.name).write_bytes(doc.read_bytes())
print(json.dumps(dict(checkpoints=len(checkpoints), review=str(out), alignment_unused=str(A)),ensure_ascii=False))
