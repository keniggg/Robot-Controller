#!/usr/bin/env python3
from pathlib import Path
import json, csv, os
out=Path(Path('.runtime/current_handeye_audit_path.txt').read_text().strip())
assert os.path.ismount('/media/zhuyupei/SanStand')
a=json.loads((out/'audit.json').read_text());r=json.loads((out/'archive_timestamp_resolution.json').read_text());groups=a['groups'];park=a['algorithms']['OpenCV/Park'];pg=park['group_5fold']['per_group'];s=park['group_5fold']['summary']
assert a['active_pose_groups']==21 and a['unique_rgbd_source_frames']==347
assert r['live_gui_matches_snapshot'] and r['session_manifest_unchanged']
assert not any(x.get('cross_validation',{}).get('passes_thresholds',False) for x in a['algorithms'].values())
summary={'verdict':'FAIL','scope':'21-group current-intrinsics hand-eye calibration audit','sample_quality_passed':True,'calibration_accuracy_passed':False,'independent_validation_done':False,'deployed':False,'pose_groups':21,'source_frames':347,'best_translation_rms_algorithm':'OpenCV/Park','five_fold_summary':s,'thresholds':a['protocol']['thresholds'],'groups_over_5mm':[v['gui_index_1based'] for v in pg if v['translation_error_mm']>5],'archive_resolution_file':'archive_timestamp_resolution.json','report':'report.md'}
(out/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n')
with (out/'per_group_park_5fold.csv').open('w') as f:
 w=csv.writer(f);w.writerow(['gui_index_1based','capture','translation_error_mm','orientation_error_deg','fold'])
 for g,v in zip(groups,pg):w.writerow([g['gui_index_1based'],g['capture'],v['translation_error_mm'],v['orientation_error_deg'],v['fold']])
def row(label,m):return '| %s | %.3f | %.3f | %.3f | %.3f |'%(label,m['translation_rms_m']*1000,m['translation_max_m']*1000,m['orientation_rms_deg'],m['orientation_max_deg'])
lines=['# 21 组新内参手眼标定样本核验','',f'审计时间：{a["created_at_utc"]}。采集会话：`{out.parent.name}`。','',
'**结论：未通过标定精度要求。** 21 组数量及单组采样质量达标，但五种算法均未通过既有误差门限。本次只读核验并离线计算，没有部署新手眼、发布标定 TF 或发送机械臂运动命令。','',
'## 数据与参数核对','',
'- 21 组有效姿态，共 347 个不同 RGBD 源帧；每组 9–27 帧。连续帧只用于检查静止性，不当作独立姿态增加样本量。',
'- 26 次采集中的 5 次已被采集服务拒绝（2 次姿态差异不足、3 次 accepted 反馈过期），未纳入本次拟合。未根据拟合误差再删除任何已接纳样本。',
'- 21 个 bag 的 SHA256 与会话及 capture.json 一致。RGB、深度、棋盘质量、棋盘 TF 与源帧戳对应；归档矩阵与 bag TF 数值一致。上位机中 21 个样本与冻结的归档代表帧全部一致。',
'- 所有样本均绑定同一个当前模型指纹；窗口前后相机配置一致。640×480，fx=395.768280、fy=395.602753，inverse_brown_conrady，SDK 2.56.5，颜色投影二次修正关闭。PnP 使用 SDK 射线模型。模型绑定来自参数快照、质量消息及启动来源记录，并非相机每帧硬件曝光元数据。',
'- 棋盘配置为 CC200：12×9 格，格长 15 mm、标记长 11.25 mm、DICT_5X5_100。此处验证的是软件配置一致，不能替代实物尺寸测量。',
'- 角点数 56–88（要求 ≥35）；距图像边缘最小 58.330 px（要求 ≥12）；重投影 RMS 0.090–0.376 px（要求 ≤1.5）。',
'- 组内工具最大变化 0.200 mm / 0.0879°（门限 1.5 mm / 0.25°）；棋盘观测最大变化 0.392 mm / 0.2449°（门限 2 mm / 0.5°）。',
'- 原始机器人 TF 头部有 174 帧与图像完全同戳，173 帧比图像晚 1 ns。严格整数相等检查因此报出 173 项；逐项确认没有其他不匹配，矩阵误差为零。保留原始审计和时间戳解释文件，未改写源戳。这一纳秒级差异不解释当前毫米级标定残差。accepted 关节消息保留其真实时间，不能称为与图像完全同戳。','',
'## 固定五折交叉验证','',
'每组使用采集服务已选定的中间帧。第 k 折测试零基索引 k::5，其余组拟合相机到工具的变换；标定板在基座下的参考位姿也只从训练组求均值。所有 21 组均参与一次测试。以下位置误差为三维欧氏误差，角度误差为旋转差，并非抓取高度误差。门限未放宽。','',
'| 算法 | 位置 RMS / mm | 位置最大 / mm | 角度 RMS / ° | 角度最大 / ° |',
'|---|---:|---:|---:|---:|',
'| 原有门限 | ≤3.000 | ≤5.000 | ≤1.000 | ≤2.000 |']
for name,v in a['algorithms'].items():lines.append(row(name,v['cross_validation']['summary']))
lines.extend(['',
'Park 的位置 RMS 最小，但四项指标仍全部超限；有 17/21 组位置误差超过 5 mm，误差并非只集中于单个样本。GUI 第 17 组（capture_022）最大，位置 14.219 mm、角度 2.211°；第 8、10、18、19 组的位置误差也超过 10 mm。不要仅凭这些排序删除样本后宣称通过。','',
'## 稳健性与姿态覆盖','',
'| Park 复核方式 | 位置 RMS / mm | 位置最大 / mm | 角度 RMS / ° | 角度最大 / ° |',
'|---|---:|---:|---:|---:|',
row('全量拟合的内部残差',park['in_sample']),row('每组连续帧取均值后五折验证',park['group_mean_5fold']['summary']),row('按采集顺序分为五个连续测试块',park['contiguous_5fold']['summary']),row('逐组留一验证',park['leave_one_group_out']['summary']),'',
'组均值没有明显改善结果，支持误差并非主要来自单帧抖动。连续时间块验证更差，提示姿态相关或随采集顺序变化的误差值得检查；此结果本身不能区分标定板移动、安装变形、几何模型或机器人反馈误差。','',
'工具在基座 XYZ 的跨度分别为 144.765 / 74.467 / 55.379 mm，姿态两两最大旋转差 88.816°。六个关节跨度约为 25.75 / 32.61 / 61.44 / 68.38 / 63.11 / 67.68°。平移求解矩阵的三个奇异值为 9.715 / 9.221 / 3.960，条件数 2.453，未出现明显秩亏。覆盖有多个方向，不能将失败简单解释为只有 21 组或全是重复姿态。此诊断并非实际抓取空间覆盖的独立验收。','',
'## 后续处理依据','',
'1. 保留这批原始数据作为排查依据，当前结果不应部署用于抓取。优先核对实物格长、棋盘与机器人基座的相对固定性，以及相机支架在不同姿态下是否松动。',
'2. 对可重复到达的姿态做往返重复观测，比较 accepted 关节反馈、工具 FK 与棋盘观测的重复性，区分机械回差/变形、机器人几何误差和视觉尺度问题。现有记录尚不能锁定唯一原因。',
'3. 修正原因后再补采或重新采集，并保留采集前即指定的独立新姿态用于验收。本批 21 组全部标记为 training，五折交叉验证不等于一套新采集的独立验证数据。即使以后通过此门限，仍需单独核验实际抓取区域的高度与接触余量。','',
'## 复核文件','',
'- `protocol.json`：拟合前冻结的输入指纹、门限、五折分组及敏感性检查方案。',
'- `session_snapshot.json` 与 `capture_*.json`：冻结的会话和样本元数据。原始 `sources.bag` 位于上一级会话的各 capture 目录。',
'- `audit.json`：完整算法候选、每折、每组、覆盖和原始严格核对结果；其中 archive_failures 的纳秒戳项须结合 `archive_timestamp_resolution.json` 阅读。',
'- `archive_timestamp_resolution.json`：1 ns TF 戳差异解释及上位机样本只读比对。',
'- `per_group_park_5fold.csv`：21 组 Park 五折误差明细。',
'- `selected_frames.jpg`：各组代表帧联系表，便于人工复查。',
'- 审计脚本在项目 `.runtime/audit_handeye_21.py`；完整输出只写 SanStand。所有候选矩阵都是未验收的离线计算结果。',''])
(out/'report.md').write_text('\n'.join(lines))
print(out/'report.md');print(json.dumps(summary,ensure_ascii=False,indent=2))
