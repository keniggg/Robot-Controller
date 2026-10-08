#!/usr/bin/env python3
"""Read saved captures only; stop before PnP when pixel-model provenance is incomplete."""
import hashlib
import json
from datetime import datetime, timezone
from pathlib import Path
import rosbag

BASE=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
ROOT=BASE/'ros_log/ros_live_20260920_session/charuco_repair'
OUT=BASE/'ros_log/unknown_grasp_20260923/hover_speed_failure_20260927/handeye_consistency_correct_model'
REPO=Path(__file__).resolve().parents[1]
if not Path('/media/zhuyupei/SanStand').is_mount():raise SystemExit('SanStand unavailable; no fallback')
OUT.mkdir(parents=True,exist_ok=True)

def sha(p):
 h=hashlib.sha256()
 with p.open('rb') as stream:
  for b in iter(lambda:stream.read(2**20),b''):h.update(b)
 return h.hexdigest()

def evidence(p,kind):return {'path':str(p),'sha256':sha(p),'kind':kind}

sources=[evidence(ROOT/'capture_exact.py','archived recorder source, not a producer binary hash'),
 evidence(ROOT/'validate_color_module.py','offline remap validation reads original bags, writes derived JSON'),
 evidence(ROOT/'replay_original_background.py','offline depth remap reads another source bag, writes derived JSON'),
 evidence(ROOT/'device_profiles_readonly.json','historical device query: 438.6 profile; no captured active SDK version'),
 evidence(ROOT.parent/'b1e1e120-b55d-11f1-b976-23ae9c4bf53a/supervisor_camera-8.log','producer connection/startup log, no producer code hash or active K/D profile'),
 evidence(REPO/'docs/verification/2026-09-20-cross-view-repair-diagnosis.md','narrative evidence: optional color remap was not enabled in running producer')]
rows=[]
for capfile in sorted(ROOT.glob('pose_*/capture.json')):
 cap=json.loads(capfile.read_text());bagfile=capfile.parent/'sources.bag';stamps={s['source_stamp_ns'] for s in cap['samples']};cam=cap['camera_runtime_parameters']
 rgb={};depth={};headers=[]
 with rosbag.Bag(str(bagfile),'r') as bag:
  topics=sorted(bag.get_type_and_topic_info().topics)
  for c in bag._get_connections():
   if 'image_raw' in c.topic:headers.append({'topic':c.topic,'header_keys':sorted(c.header),'original_callerid_retained':'callerid' in c.header})
  for topic,msg,_ in bag.read_messages(topics=['/supervisor/camera/color/image_raw','/supervisor/camera/depth/image_raw']):
   value={'encoding':msg.encoding,'width':msg.width,'height':msg.height,'frame_id':msg.header.frame_id}
   (depth if '/depth/' in topic else rgb)[msg.header.stamp.to_nsec()]=value
 rows.append({'label':cap['label'],'accepted_stationary_window':cap['accepted_stationary_window'],
   'capture_file':evidence(capfile,'capture metadata'),'bag_file':evidence(bagfile,'saved raw Image messages, no per-frame CameraInfo'),
   'sample_count':len(stamps),'first_source_stamp_ns':min(stamps),'last_source_stamp_ns':max(stamps),
   'source_start_utc':datetime.fromtimestamp(min(stamps)/1e9,timezone.utc).isoformat(),
   'source_end_utc':datetime.fromtimestamp(max(stamps)/1e9,timezone.utc).isoformat(),
   'all_samples_have_exact_rgb_depth':stamps<=rgb.keys() and stamps<=depth.keys(),
   'first_rgb_metadata':rgb[min(stamps)],'topics':topics,'image_connection_headers':headers,
   'camera_config_K':{k:cam.get(k) for k in ('fx','fy','cx','cy')},
   'runtime_profile_present':'runtime_profile' in cam,
   'color_projection_correction_key_present':'color_projection_correction' in cam,
   'producer_source_hash_in_capture':any('hash' in k and ('producer' in k or 'pipeline' in k) for k in cap),
   'saved_yaml_sha256':cap['camera_yaml_sha256'],'embedded_yaml_sha256':hashlib.sha256(cap['calibration_file_content'].encode()).hexdigest()})
accepted=[r for r in rows if r['accepted_stationary_window']]
report={'status':'STOPPED_BEFORE_FITTING_INPUT_PIXEL_MODEL_NOT_PROVEN',
 'operator_confirmation':'User explicitly confirms original deployed handeye was calibrated with old intrinsics before intrinsics change; accepted as provenance, not reopened here.',
 'offline_only':True,'camera_or_ros_node_opened':False,'candidate_generation_performed':False,'deployment_allowed':False,
 'checked_capture_groups':len(rows),'accepted_capture_groups':len(accepted),'accepted_source_frames':sum(r['sample_count'] for r in accepted),
 'independent_heldout_reserved':'09_return_validation (not fitted)',
 'positive_evidence':[
   'Recorder stores original subscribed Image messages directly at exact integer source timestamps.',
   'Reviewed color-projection and focal/depth-remap scripts open bags read-only and write separate derived JSON; no source-bag writeback in those scripts.',
   'Producer log shows supervisor_camera present across capture windows and capture subscriber connections.',
   'Contemporaneous narrative states optional color projection code remained disabled in running camera.',
   'All accepted groups retain RGB, depth, T_base_tool, and board source stamps; source pixels are available for reprocessing once ray provenance is established.'],
 'strict_proof_missing':[
   'No capture-bound active SDK profile, SDK version, or executed producer/pipeline source or binary hash.',
   'No per-frame CameraInfo or intrinsics/preprocessing metadata in source bags; repacked bag connection headers omit original callerid.',
   'Missing color_projection_correction key does not prove false for an unknown executed producer version.',
   'Historical saved device query reports 438.6 and lacks active SDK/version binding; later official SDK 395.768/395.603 profile is not directly linked to these September 20 pixels.',
   'No archived mapping certificate or matched before/after raw-image pair establishes that applying current inverse-Brown rays to these historical pixels is valid.'],
 'judgment':'Evidence supports no remapping by the reviewed offline scripts, but does not strictly certify upstream pixel-to-ray model. Stop as requested; do not generate an assumed calibration.',
 'minimal_required_existing_evidence':'A capture-period active profile plus executed producer/preprocessing hash/config, or an equivalent source-bound proof of raw color pixel projection and any remap. This report does not request new motion or acquisition.',
 'correct_model_computation_if_proven':'Use SDK rs2_deproject_pixel_to_point on verified original pixel model, normalize rays, solvePnP with K=I and D=0, transform optical to camera_link, reserve pose09 and perform pose-group holdout. Never pass inverse-Brown coefficients directly as OpenCV Brown D.',
 'sources':sources,'captures':rows}
(OUT/'input_provenance_audit.json').write_text(json.dumps(report,indent=2,ensure_ascii=False,allow_nan=False)+'\n')
text='''# 近期手眼录包像素来源核验：暂不拟合

用户已确认原部署手眼使用旧内参，本报告接受这一事实。目标是核验近期保存图像能否严格使用当前官方 SDK 内参和 inverse Brown 射线重新计算；全程离线，不部署外参或操作硬件。

**结论：未达到严格输入证明，已停止在 PnP/手眼拟合之前。** 没有生成新的外参候选，也没有把第九组用于训练。

10 组记录中 9 组通过当时的静止窗口条件，共 153 个原始源帧。原始 RGBD、整数源戳和 T_base_tool 均保留，所有合格样本能与 RGBD 精确配对。它们具备重新处理的原始数据基础，但当前像素到射线模型还缺绑定证据。

可以证明已审阅的离线重映射脚本没有写回源 bag：capture_exact 直接保存 Image；validate_color_module 和 replay_original_background 只读原始 bag，写到单独派生 JSON。当时运行说明还明确记载 color projection 修正未接入运行相机，同一 supervisor_camera 的连接日志贯穿采样。这些证据支持“原 bag 没有被这些离线分析改写”。

不能由此严格认证上游相机像素模型。所有 capture 只有旧 438.6 K 和 tracker YAML 哈希，没有 active runtime_profile、SDK 版本、运行 producer/pipeline 源码或二进制哈希，也没有显式 remap=false。bag 没有 CameraInfo/逐帧投影参数，重新封装的连接头未保留原 callerid。当时单独设备查询仍给 438.6，缺少 SDK 版本；后来 official SDK 的 395.768/395.603 不能自动绑定这些旧帧。缺 key 不能当成明确关闭，旧 YAML 哈希也不是图像 producer 哈希。

因此不盲用 395 到未经严格确认的历史像素，也不把有畸变模型误用的旧重放数字当作当前组合已验收。需要补齐既有采集期 active profile 与 producer/preprocessing 来源证明，或等价的原像素映射证据，之后才可按 SDK inverse Brown 去投影为 normalized rays，以 K=I、D=0 做 PnP，再正确转换 optical→camera_link，做按姿态组留出；第九返回组必须独立保留。

本轮没有要求新采样、没有运行采集脚本，没有相机/ROS/机械臂命令。逐组元数据、源文件路径和 SHA256 见 input_provenance_audit.json。复现脚本位于工作树 .runtime/audit_handeye_pixel_provenance_20260927.py。
'''
(OUT/'input_provenance_audit.md').write_text(text)
print(json.dumps({k:report[k] for k in ['status','checked_capture_groups','accepted_capture_groups','accepted_source_frames','candidate_generation_performed','deployment_allowed']},ensure_ascii=False));print(OUT)
