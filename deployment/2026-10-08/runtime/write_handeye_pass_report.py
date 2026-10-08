#!/usr/bin/env python3
import json,os,hashlib,sys
from pathlib import Path
from datetime import datetime,timezone
from collections import Counter
import numpy as np,rosbag,yaml
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/real-arm/alicia_d_calibration/scripts'))
from handeye_invariance_metrics import transform_from_translation_quaternion
assert os.path.ismount('/media/zhuyupei/SanStand')
final=Path(json.loads((ROOT/'.runtime/current_handeye_auto_final.json').read_text())['directory'])
r=json.loads((final/'result.json').read_text());assert r['verdict']=='PASS' and not r['archive_failures']
train=Path(r['archive_training']['session']); fit=Path(json.loads((ROOT/'.runtime/current_handeye_auto_fit.json').read_text())['directory'])
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
assert sha(fit/'results.json')==r['candidate_sha256']
f=json.loads((fit/'results.json').read_text()); candidate=next(train.glob('candidate_*.yaml')); y=yaml.safe_load(candidate.read_text());tr=y['transformation']; xyz=[tr[x] for x in ['x','y','z']];quat=[tr[x] for x in ['qx','qy','qz','qw']]
X=transform_from_translation_quaternion(xyz,quat);assert np.allclose(X,r['frozen_candidate']['T_tool_camera'],rtol=0,atol=1e-8)
(final/'validated_handeye.yaml').write_bytes(candidate.read_bytes())
with rosbag.Bag(str(train/'automatic_motion_telemetry.bag')) as b:
 counts=Counter();positive=negative=0; first=last=None
 for topic,msg,t in b.read_messages():
  counts[topic]+=1; first=t.to_sec() if first is None else first;last=t.to_sec()
  if topic=='/demonstration':
   negative+=int(msg.data);positive+=int(not msg.data)
 telemetry={'bag':str(train/'automatic_motion_telemetry.bag'),'sha256':sha(train/'automatic_motion_telemetry.bag'),'start_unix':first,'end_unix':last,'topic_message_counts':dict(counts),'positive_enable_messages':positive,'disable_messages':negative,'coverage_note':'Recorder began after first four training groups; earlier startup/motions are in ros.log and automatic_motion.jsonl.'}
(final/'telemetry_audit.json').write_text(json.dumps(telemetry,ensure_ascii=False,indent=2)+'\n')
assert negative==0
sources=['tools/handeye_exact_sample_server.py','src/real-arm/alicia_d_calibration/scripts/charuco_tracker.py','src/real-arm/alicia_d_calibration/scripts/handeye_sample_audit.py','src/real-arm/alicia_d_calibration/scripts/handeye_invariance_metrics.py','.runtime/handeye_auto_motion.py','.runtime/handeye_auto_collect.py','.runtime/compute_auto_handeye.py','.runtime/finish_auto_handeye_validation.py','.runtime/write_handeye_pass_report.py']
attestation={'created_at':datetime.now(timezone.utc).isoformat(),'validated':True,'deployed':False,'parent_frame':'tool0','child_frame':'camera_link','transform_convention':'T_tool_camera maps camera_link coordinates into tool0','translation_xyz_m':xyz,'quaternion_xyzw':quat,'camera_model_fingerprint':r['archive_training']['camera_model_fingerprint'],'training_groups':22,'independent_groups':7,'return_groups':1,'candidate_refitted_with_validation':False,'result_file':str(final/'result.json'),'result_sha256':sha(final/'result.json'),'source_candidate':str(candidate),'yaml_sha256':sha(final/'validated_handeye.yaml'),'frozen_fit_sha256':sha(fit/'results.json'),'scope':r['scope'],'source_files':{s:sha(ROOT/s) for s in sources}}
(final/'validated_handeye.json').write_text(json.dumps(attestation,ensure_ascii=False,indent=2)+'\n')
cv=r['training_cross_validation'];ind=r['independent_validation'];ret=r['return_repeatability'];allv=r['all_validation_summary']
def row(label,d):return '| %s | %d | %.3f | %.3f | %.3f | %.3f |'%(label,d['count'],d['translation_rms_m']*1000,d['translation_max_m']*1000,d['orientation_rms_deg'],d['orientation_max_deg'])
lines=['# 2026-09-27 自动手眼标定通过记录','',
'本次使用当前 D405 活动内参自动采集 22 组训练姿态，完成五折验证；冻结 Tsai-Lenz 参数后，再采集 7 组独立验证姿态和 1 组返回姿态。原 3/5 mm 平移 RMS/最大值、1/2° 旋转 RMS/最大值标准全部通过。原始数据审计 0 项异常。','',
'新参数已保存为通过验证的参数文件，尚未部署到运行 TF；现用 TF 仍是旧参数。通过范围限于本次观测区域与姿态，不代表已验证整个抓取工作空间、绝对桌面高度或小目标接触裕度。','',
'| 数据集 | 姿态数 | 位置 RMS (mm) | 位置最大 (mm) | 角度 RMS (°) | 角度最大 (°) |','|---|---:|---:|---:|---:|---:|',row('训练姿态五折留出',cv),row('独立新姿态',ind),row('返回姿态对冻结参考',ret),row('独立验证合计',allv),'| 原标准 | — | ≤3 | ≤5 | ≤1 | ≤2 |','',
'误差计算为每组实测 `T_base_tool × T_tool_camera × T_camera_board` 相对冻结训练集平均标定板位姿的差；独立验证不重新居中、不重新拟合。返回姿态一行也是对该冻结参考的误差，并非外部仪器测得的机械重复定位精度。','',
'## 数据与算法','',
'- 训练 32 次采样尝试中，22 组通过采样门并全部用于拟合；10 次未接纳尝试完整保留，未按拟合误差删掉已接纳样本。验证 8 次均接纳，独立验证数据不参与候选选择或拟合。',
'- 训练原始同源帧 377 帧，验证 126 帧，共 503 帧；连续帧用于静稳性和来源审核，每组固定选中间帧作为一个姿态，未把连续帧充作独立姿态。',
'- 核对 RGB、深度、质量、标定板 TF 的同源时间戳、每组 bag SHA256、内参指纹、关节 epoch 及关节反馈。机器人 TF 查询消息头存在 0/+1 ns 序列化差异（训练 183/194 帧，验证 72/54 帧），源时间和矩阵一致；并非毫秒级不同步。',
'- 每组仍要求至少 8 帧、跨度至少 1 秒及原静稳性门；角点不少于 35，重投影 RMS 不大于 1.5 px，图像边缘距离不少于 12 px。',
'- 预先规划的一个训练姿态及一个验证姿态因预测板出视野未执行；重复姿态在入库时拒绝。这些决定发生在候选误差评估前，记录均保留。',
'- 五折按姿态固定分组 k::5。Tsai、Park、Horaud、Daniilidis 通过全部四项，Andreff 未通过；按预定规则选择通过者中位置 RMS 最小的 Tsai。',
'- 训练实际末端 XYZ 跨度约 34.34 / 28.01 / 40.85 mm，最大两姿态转角 43.25°；平移方程条件数 2.634。该覆盖范围不构成全工作空间精度认证。','',
'## 当前相机模型与板','',
'- D405 序列号 260522277974，SDK 2.56.5，640×480，camera_link。',
'- fx=395.7682800292969，fy=395.6027526855469，cx=317.9961242675781，cy=244.833984375。',
'- inverse_brown_conrady；D=[-0.05372992530465126, 0.05834611505270004, 0.002122000325471163, 0.0007625872385688126, -0.019704923033714294]。',
'- CC200：12×9 格，15 mm 方格、11.25 mm marker，DICT_5X5_100，legacy pattern。',
'- PnP 使用 SDK 归一化反投影和原像素域重投影；训练/验证绑定同一模型指纹：`'+attestation['camera_model_fingerprint']+'`。','',
'## 采集过程中的处理','',
'- 原图角点数量不足时，启用已有检测器的 1.5 倍检测缩放，角点坐标再映回原图。内参不缩改，角点和误差门限不放宽。部分早期训练组使用 1 倍检测，所有组均通过同一原图质量门。',
'- 新建控制进程时等待真正的驱动订阅连接，修复只看到本进程监测订阅者即误判连接已建立造成的首条命令丢失。',
'- 自动动作沿用上位机关节控制协议及使能握手，单关节每步至多 4°；采样采用实测稳定反馈，不为追逐名义角度死区反复补偿。',
'- 使用上位机手眼模块相同的采样、计算、保存后端；后台计算结果与冻结候选矩阵一致。没有修改真实驱动协议、使能语义或验收阈值。',
'- 采样后端新增自动姿态来源和 training/validation 角色元数据，既有 28 项测试通过。','',
'## 已验证参数','',
'父坐标 tool0，子坐标 camera_link；平移单位 m，四元数顺序 xyzw。','',
'```json',json.dumps({'translation_xyz_m':xyz,'quaternion_xyzw':quat},ensure_ascii=False,indent=2),'```','',
'## 运行与保存状态','',
'- ROS 完整主栈、上位机、相机、手眼跟踪与采样后端继续运行。完成快照为 feedback_ready=true、actuation_status=CONFIRMED:MEASURED_DIRECTIONAL_RESPONSE、protection_latched=false，关节反馈持续更新。未发送关闭关节使能命令。',
'- 本次单独的只读运动 rosbag 记录器已封存。它在第 4 组后启用，其覆盖区间关闭使能消息为 0；更早的启动和动作由主 ros.log 与运动 JSONL 记录。主 ROS 日志继续写入。',
'- 后端原始 candidate JSON 的 validated=false 保留为计算当时快照；本次最终通过凭证是同目录 `validated_handeye.json` 与 `result.json`，新副本 `validated_handeye.yaml` 与原候选字节相同。运行 TF 未替换。',
'- 存储采用 SanStand local_only，不上传、不删除历史。','',
'## 文件索引','',
'- [通过参数 YAML](validated_handeye.yaml)',
'- [通过凭证](validated_handeye.json)',
'- [最终机器可读报告](result.json)',
'- [各验证姿态误差](validation_groups.csv)',
'- [运行状态快照](completion_runtime_snapshot.json)',
'- [封存运动记录审核](telemetry_audit.json)',
'- 训练目录：`'+str(train)+'`',
'- 独立验证目录：`'+r['archive_validation']['session']+'`',
'- 冻结拟合及 GUI 计算核对：`'+str(fit)+'`','']
report='\n'.join(lines);(final/'report.md').write_text(report)
doc=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/文档/2026-09-27-automatic-handeye-calibration-pass.md')
doc.write_text('# 2026-09-27 自动手眼标定通过\n\n22 组训练、7 组独立验证、1 组返回姿态全部完成；当前区域通过原 3/5 mm、1/2° 标准，运行 TF 尚未替换。\n\n完整报告：['+str(final/'report.md')+']('+str(final/'report.md')+')\n')
manifest={p.name:sha(p) for p in final.iterdir() if p.is_file() and p.name!='SHA256SUMS.json'}
(final/'SHA256SUMS.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n')
print(json.dumps({'report':str(final/'report.md'),'validated_yaml':str(final/'validated_handeye.yaml'),'disable_messages_in_recorded_interval':negative,'candidate_deployed':False,'final_artifacts':len(manifest)},ensure_ascii=False))
