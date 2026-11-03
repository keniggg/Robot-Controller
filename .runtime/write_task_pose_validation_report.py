from pathlib import Path
from datetime import datetime,timezone
import json,hashlib,shutil,os,numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path.cwd();c=json.load(open(ROOT/'.runtime/current_handeye_task_pose_validation.json'));out=Path(c['session']);assert os.path.ismount('/media/zhuyupei/SanStand');r=json.load(open(out/'task_pose_validation_result.json'));live=json.load(open(out/'final_live_readback.json'));robust=json.load(open(out/'worst_pose_window_robustness.json'));depth=json.load(open(out/'depth_vs_pnp_diagnostic.json'));fit=json.load(open(out/'post_validation_fit_diagnostic.json'));rows=[(i,g) for i,s in enumerate(r['segments'],1) for g in s['groups']];rows.sort(key=lambda p:p[1]['source_stamp_ns']);q=np.array([g['q_deg'] for _,g in rows]);tcp=np.array([np.array(g['T_base_tool'])[:3,3] for _,g in rows]);times={g['source_stamp_ns']:j+1 for j,(_,g) in enumerate(rows)}
fig,axes=plt.subplots(2,1,figsize=(11,7),sharex=True,constrained_layout=True)
for i,s in enumerate(r['segments'],1):
 gx=[times[g['source_stamp_ns']] for g in s['groups']];gy=[g['translation_error_mm'] for g in s['groups']];axes[0].plot(gx,gy,'o-',label='Epoch '+str(i),markersize=4)
axes[0].axhline(5,color='#b42318',ls='--',label='Maximum-error limit: 5 mm');axes[0].set_ylabel('Fixed-board displacement (mm)');axes[0].set_title('Frozen hand-eye verification: fixed first reference within each epoch');axes[0].legend(ncol=4,fontsize=8);axes[0].grid(alpha=.2)
for axis,label in [(1,'Joint 2'),(2,'Joint 3'),(4,'Joint 5')]:axes[1].plot(np.arange(1,len(rows)+1),q[:,axis],'.-',label=label)
axes[1].set_ylabel('Measured joint angle (deg)');axes[1].set_xlabel('Archived observation window (time order; includes return views)');axes[1].legend(ncol=3);axes[1].grid(alpha=.2);fig.savefig(out/'validation_overview.png',dpi=170);plt.close(fig)
allnovel=sum(json.load(open(Path(s['session'])/'session.json'))['active_sample_count'] for s in c['sampling_segments']);worst=max((g for _,g in rows),key=lambda g:g['translation_error_mm']);current_depth=[v for v in depth['samples'] if str(out) in v['capture']];events=[json.loads(line) for line in (out/'automatic_motion.jsonl').read_text().splitlines()];stopped=[e for e in events if e['event']=='task_pose_validation_interrupted'];positive=[e for e in events if e['event']=='positive_enable_request'];requests=[e for e in events if e['event']=='move_requested'];settled=[e for e in events if e['event']=='move_settled']
coverage=dict(all_observation_joint_min_deg=q.min(axis=0).tolist(),all_observation_joint_max_deg=q.max(axis=0).tolist(),all_observation_tcp_xyz_span_mm=(np.ptp(tcp,axis=0)*1000).tolist(),all_nominal_observation_slots_archived=True,original_return_nominal_goal_reached=False,original_return_actual_observation_retained=True,contact_or_closed_grasp_pose_tested=False,physical_grasp_success=False)
(out/'coverage_and_motion_summary.json').write_text(json.dumps(dict(coverage=coverage,positive_enable_requests=len(positive),bounded_move_requests=len(requests),bounded_move_settled=len(settled),interruption_events=stopped,validation_windows=33,novel_gui_admitted_windows=allnovel,repeat_return_windows=33-allnovel,final_live=live),ensure_ascii=False,indent=2)+'\n')
lines=['# 固定手眼参数的自主多姿态检验','',f'结果：**未通过（FAIL）**。完成 33 组观测、498 帧原始 RGB-D/TF/关节反馈核对，证据配对错误为 0。28 组由上位机兼容采样服务接收，另外 5 组是重复返回窗口，单独保存，不算新增独立姿态。','',
'当前仍使用 2026-09-27 验证通过的参数，整个检验过程未重新拟合或替换运行中的手眼变换。最终参数服务器、实时 `tool0 -> camera_link` 和 YAML SHA256 完全一致。', '',
'- 参数文件：`'+c['frozen_handeye']['calibration_file']+'`',
'- SHA256：`'+c['frozen_yaml_sha256']+'`',
'- 平移（m）：`'+str(c['frozen_handeye']['translation_xyz'])+'`',
'- 四元数 xyzw：`'+str(c['frozen_handeye']['rotation_xyzw'])+'`','',
'## 同代次验收','',
'每个驱动代次均以该分段第一组已归档观测作为固定参考，参考采集早于后续姿态。没有用后续数据重新居中参考或改变参数。原门限保持：位置 RMS ≤ 3 mm、最大 ≤ 5 mm；角度 RMS ≤ 1°、最大 ≤ 2°。使能恢复共触发 5 次控制代次变化，因此数据分为 6 段；跨代次绝对差只作诊断。','',
'| 分段 | 参考后观测数 | 位置 RMS (mm) | 最大 (mm) | 角度 RMS / 最大 (°) | 判定 |','|---|---:|---:|---:|---:|---|']
for i,s in enumerate(r['segments'],1):
 a=s['all_nonreference_summary']
 if a:lines.append('| %d | %d | %.3f | %.3f | %.3f / %.3f | %s |'%(i,a['count'],a['translation_rms_m']*1000,a['translation_max_m']*1000,a['orientation_rms_deg'],a['orientation_max_deg'],s['within_epoch_verdict']))
 else:lines.append('| %d | 0 | — | — | — | 样本不足 |'%i)
lines += ['', '分段 2 的最大误差 5.572 mm；分段 5 为 6.144 mm，均超过 5 mm。分段 5 的 RMS 3.835 mm 也超过 3 mm。样本数量不足的分段不会标记为通过。','',
'超限具有窗口内稳定性：分段 2 参考与最差姿态的全部帧组合为 5.488–5.575 mm；分段 5 为 5.880–6.220 mm，全部超过 5 mm。对应两个窗口均值差为 5.533 / 6.100 mm。','',
'![按代次固定参考的漂移和实测关节角](validation_overview.png)','',
'## 实际运动和覆盖','',
'用户已明确授权自主调姿。本轮通过上位机已有直控协议逐关节小步调姿，夹爪保持打开；没有发布关闭使能命令。驱动 PID 10012 未重启。5 次 `/demonstration=False` 在该驱动中为正向使能恢复，原反馈新鲜度、保护状态和 MoveIt 碰撞约束继续生效。','',
'先完成肘腕姿态和往返检验，再以实际空抓关节姿态为依据补采。观测到的关节 2 范围 %.3f–%.3f°，关节 3 范围 %.3f–%.3f°，关节 5 范围 %.3f–%.3f°。'%(q[:,1].min(),q[:,1].max(),q[:,2].min(),q[:,2].max(),q[:,4].min(),q[:,4].max()),'',
'空抓时关节 2 / 3 / 5 为约 -43.066 / 74.004 / -66.270°。本轮较深肘观测达到约 -28.564 / 73.037 / -85.869°，属于接近任务关节构型的抬高观察位，不是闭爪接触位。因此不能声称覆盖了实际接触高度或整个工作空间。','',
'原计划第 20 个返回位受完整视野和 40 mm 粗几何余量限制，没有精确返回名义角度；记录了实际停止姿态并标记 `geometry_limited_return`。31 个计划观测槽位均有记录，其中该槽位明确为替代观测。后续补充路径完成往返。','',
'调姿期间记录到肩部反向运动约 0.9–1.2° 的指令/反馈偏差及多次使能恢复。采样使用实测角度，不以名义指令角度作为标定输入。调姿工具随后将肩肘反向指令限制在 3°，并在发令前为完整 4.8° 实测响应范围核对几何/碰撞；接触任务的控制容差和全局速度参数没有因此改变。所有中断和重试均保留。','',
'## 结论边界和附加诊断','',
'昨天的通过结果对应昨天实际覆盖的区域。今天的结果表明，当前“机械臂关节/运动学 + 手眼 + 相机”整条坐标链在新增区域和往返姿态下仍有超限偏差。仅凭该结果不能把误差全部归因于手眼参数，也不能证明它完全解释了之前空抓中的约 13 mm 桌面偏移。','',
'同帧深度与棋盘 PnP 的中位差在 %.3f–%.3f mm 范围，深度/PnP 比值中位约 %.5f；棋盘实物尺度没有独立计量，所以未据此修改深度比例。'%(min(v['median_depth_minus_pnp_m']*1000 for v in current_depth),max(v['median_depth_minus_pnp_m']*1000 for v in current_depth),np.median([v['median_depth_over_pnp'] for v in current_depth])),'',
'对样本最多的同代次 16 组数据另做了事后拟合诊断。候选交叉验证 RMS 约 1.95–2.06 mm、最大约 3.59–3.85 mm；这些数据已用于本轮验证，不能再被当成新候选的独立验证。该诊断没有部署任何参数。现用参数在事后重新居中的诊断参考下也会得到 RMS 1.86 mm / 最大 3.72 mm，因此不能用这种参考变化取代本轮固定参考的超限结论。','',
'抓取问题仍未解决，尚未再次抓取，也未添加固定向下 13 mm 偏置。后续需要分离关节/运动学、手眼及深度贡献，并对任何候选修正进行新采集的独立验证。','',
'## 最终状态和记录','',
'机械臂已停稳、夹爪打开（反馈 0.04975 m），驱动真实运动确认、反馈就绪、无保护锁定，抓取任务不活动；保持手动直控模式及现用手眼。','',
'- [完整结果及每组数据](task_pose_validation_result.json)',
'- [超限窗口稳健性](worst_pose_window_robustness.json)',
'- [深度/PnP 诊断](depth_vs_pnp_diagnostic.json)',
'- [事后拟合诊断，未部署](post_validation_fit_diagnostic.json)',
'- [运动与覆盖汇总](coverage_and_motion_summary.json)',
'- [最终实时状态](final_live_readback.json)',
'- [全部调姿与中断日志](automatic_motion.jsonl)',
'- [采样分段索引](sampling_segments.json)','',
'所有原始窗口、被拒绝窗口、bag、日志均直接保存在 SanStand；local_only，不上传、不删除历史。','']
report='\n'.join(lines);(out/'REPORT.md').write_text(report)
doc=ROOT/'docs/operations/2026-09-28-task-pose-handeye-validation.md';doc.write_text('# 自主任务姿态手眼检验\n\n本轮完成 33 组观测、498 帧原始数据核对。固定昨天通过的手眼参数，按 6 个控制代次分别以首组为固定参考：存在同代次最大 5.572 / 6.144 mm 偏差，超出 5 mm；更深肘构型分段 RMS 3.835 mm 超出 3 mm，结论 FAIL。全帧窗口比较仍全部超过 5 mm。\n\n没有更换手眼参数。机械臂已停稳并保持使能，驱动 PID 10012 不变，夹爪打开。记录了原计划第 20 个返回姿态的几何受限替代观测，未把它记为名义目标到位；补充抓取构型观察和回程完成。此结果未完成抓取、未证明单一根因，也未覆盖真实接触高度。\n\n详细报告：[SanStand 实验报告]('+str(out/'REPORT.md')+')。原始数据、拒绝窗口、命令和反馈都保留；现用 TF、YAML、参数服务器三方一致。\n')
mirror=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/文档/operations');mirror.mkdir(exist_ok=True,parents=True);shutil.copy2(doc,mirror/doc.name)
source=out/'source_snapshot';source.mkdir(exist_ok=True);files=['.runtime/handeye_auto_motion.py','.runtime/collect_task_pose_sweep.py','.runtime/plan_task_pose_sweep.py','.runtime/audit_task_pose_sweep.py','.runtime/rotate_task_pose_sampler.py','.runtime/audit_board_depth_consistency.py','.runtime/diagnose_task_pose_handeye_fit.py','.runtime/write_task_pose_validation_report.py','tools/handeye_exact_sample_server.py'];hashes={}
for name in files:
 p=ROOT/name;target=source/name.replace('/','__');shutil.copy2(p,target);hashes[name]=hashlib.sha256(p.read_bytes()).hexdigest()
(out/'source_sha256.json').write_text(json.dumps(hashes,indent=2)+'\n')
status_path=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/grasp_resume_20260928T011952Z/unknown_speed_repair_status.json');status=json.load(open(status_path));status.update(status='task_pose_verification_complete_failed_current_handeye_unchanged',physical_grasp_success=False,task_pose_validation_session=str(out),task_pose_validation_verdict='FAIL',task_pose_validation_groups=33,task_pose_validation_frames=498,task_pose_max_within_epoch_error_mm=worst['translation_error_mm'],task_pose_report=str(out/'REPORT.md'),handeye_replaced_during_verification=False);status_path.write_text(json.dumps(status,ensure_ascii=False,indent=2)+'\n')
print(json.dumps(dict(report=str(out/'REPORT.md'),doc=str(doc),all_groups=len(rows),admitted=allnovel,worst_mm=worst['translation_error_mm'],coverage=coverage),ensure_ascii=False,indent=2))
