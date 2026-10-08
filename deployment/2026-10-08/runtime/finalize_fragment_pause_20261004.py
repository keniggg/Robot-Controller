"""Save the completed offline work and explicit user pause, without ROS actions."""
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path

WORK = Path(__file__).resolve().parents[1]
EXT = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
RUN = EXT/'ros_log/full_ros_resume_20261004T002930Z'
manifest = json.loads((WORK/'.runtime/rgb_acquisition_validation_20261004.json').read_text())
if not EXT.parent.parent.is_mount():
    raise RuntimeError('SanStand unavailable; no fallback')
if manifest['test_exit_code'] != 0 or manifest['changed_during_test']:
    raise RuntimeError('Final successful immutable regression required')
for name, expected in manifest['source_sha256'].items():
    if hashlib.sha256((WORK/name).read_bytes()).hexdigest() != expected:
        raise RuntimeError('Source changed after final test: '+name)
now = datetime.now(timezone.utc).isoformat()
result = dict(recorded_at_utc=now, work_status='paused_by_user_after_offline_repairs',
    pause_request='你解决完问题后就先暂停，等我回来唤醒你',
    task_complete=False, offline_code_and_tests_complete=True,
    physical_grasp_complete=False, physical_prefix_verified=False,
    pending_source_fix_loaded=False, dispatcher_loaded=False, rgb_prefix_gateway_loaded=False,
    temperature_off_policy_loaded=True,
    temperature_policy_record=str(RUN/'driver_temperature_off_20261004T084125Z/result.json'),
    final_regression=manifest['test_result'], final_regression_log=manifest['test_log'],
    source_sha256=manifest['source_sha256'], test_sha256=manifest['test_sha256'],
    last_known_selection_generation=10, generation_10_lost=True,
    new_selection_started_after_pause_request=False,
    robot_motion_commands_sent_after_pause_request=False,
    torque_disable_commands_sent=False,
    historical_cross_view_support_error_globally_resolved=False,
    original_user_confirmation='call_5erp6BWEU3KQ4mhVrVNKeNUq:0',
    latest_readonly_original_intent_qualification=str(EXT/'ros_log/rgb_observation_new_trial_20261004T084737.784476Z/result.json'),
    next_steps_after_user_wakes=[
        'Read saved progress and verify actual current scene/control ownership; no old plan reuse.',
        'Load tested perception and RGB gateway only after user resumes; gen10 lost must not be revived.',
        'If original RGB, measured pose, source context and maintenance edge still qualify, establish a fresh selection through guarded entry; otherwise establish actual new alignment evidence.',
        'Record exact source pair and full attempt to SanStand before any newly authorized action.',
        'Verify genuine actuation response, initial-view-first dispatch, bounded supplemental observation and physical grasp.'])
path = RUN/'paused_offline_repairs_20261004.json'
path.write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
for progress in (RUN/'current_progress.json', WORK/'.runtime/latest_full_ros_start.json'):
    value = json.loads(progress.read_text())
    value.update(updated_at_utc=now, work_status=result['work_status'],
        pause_requested_by_user=True, automatic_resume_allowed=False,
        task_complete=False, offline_code_and_tests_complete=True,
        final_offline_regression=manifest['test_result'], final_offline_regression_log=manifest['test_log'],
        pause_record=str(path), software_temperature_monitoring_enabled=False,
        temperature_poll_rate_hz=0.0,
        pending_source_fix_loaded=False, dispatcher_loaded=False, rgb_prefix_gateway_loaded=False,
        current_blocker='User requested pause after offline repair. Final offline regression passed and all sources saved; no new deployment/selection/motion after pause request. Temperature-off driver is already loaded and verified. Gen10 remains lost; newer pending-source/dispatcher/RGB-prefix code is not loaded. Real actuation was pending at last readback; fresh physical response and full grasp remain unverified. Do not resume automatically or claim cross-view/physical grasp completion.')
    progress.write_text(json.dumps(value, ensure_ascii=False, indent=2)+'\n')
doc = WORK/'docs/verification/2026-10-04-thin-target-rgb-observation.md'
heading = '## 最终离线封版与用户要求暂停'
if heading not in doc.read_text():
    with doc.open('a') as stream:
        stream.write('\n'+heading+'\n\n')
        stream.write('最终联合回归：**'+manifest['test_result']+'**；完整原始日志 `'+manifest['test_log']+'`，测试期间及保存前源码摘要一致。结果和冻结摘要写入 `paused_offline_repairs_20261004.json`。\n\n')
        stream.write('维护入口新增一次显式驱动重载回执分支：旧确认、原图、旧代次链逐项保留并复验，只承认实际记录的旧→新控制 epoch；当前 RGB 直接匹配最早原图，仍使用新 epoch 后真实双侧反馈、原 200 ms/0.8 s/0.004 rad 限制。该维护结果不能递归延续。084737Z 只读实测匹配通过（IoU约0.841），未切换选择或移动。第七代旧 ledger 有两次观察；用户之后重新对准，只有第八至第十代没有新的预约，不能将第七代写为无动作。\n\n')
        stream.write('新增串行入口优先实际合格三维目标；未锁定且新鲜 RGB 兴趣已就绪时进入原有限补观察；locked hold/pending 等待原有效期内真恢复或 lost，不授予弱入口权限。补观察退出码0或相机动作完成不等于获得目标，必须核对实际新鲜 measured handoff 才重新抓取规划。两个 CLI 接受精确 selection 绑定；日志 I/O 后再次读取配置并核验来源/选择/epoch/lost/handoff，关闭启动竞态。RGB 与 observe_return 仍各有原预算，不能声称合计最多三次。\n\n')
        stream.write('独立 RGB owner 的 pending 运动确认前缀现在有离线实现：一次性能力仅属于本次 RPC、epoch、reference 和同预算已预约动作，实际连续控制曲线分别相对真实 desired 起点和实测起点限制在0.025 rad内；沿用完整场景、原参考及全部成员取景、跟随和停止范围证明。回调接收到的故障或确认退化立即锁存撤销，不因稍后恢复而复活。前缀只有真实方向运动确认并停稳才返回成功，明确原请求观察终点未到达；下一次必须新帧、新关节位置、重新规划并再次计费，不继续旧完整轨迹。该前缀没有实机执行证据。\n\n')
        stream.write('用户随后明确要求修复后暂停，等待唤醒。已完成代码、离线测试、独立复核和文件保存；此指令之后没有新选择、部署或机械臂动作。在线仅温度关闭驱动已生效，感知仍为此前b3dc版本，网关/分流新修复未加载，第十代 lost 未复活；实际抓取及新前缀实机效果待用户唤醒后验证。没有发送关节失能命令，没有写入全任务完成标记，也没有将历史跨视角19–20 mm问题记为全局解决。\n')
print(json.dumps(dict(paused=True, report=str(path), regression=manifest['test_result'],
    physical_validation_pending=True), ensure_ascii=False))
