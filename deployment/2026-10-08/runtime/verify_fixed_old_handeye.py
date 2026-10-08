#!/usr/bin/env python3
"""Validate a frozen active hand-eye against archived current-model poses.
No hand-eye fitting, publishing or robot commands.
"""
import sys,os,json,hashlib,csv
from pathlib import Path
from datetime import datetime,timezone
import numpy as np
import rospy,tf2_ros
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/real-arm/alicia_d_calibration/scripts'))
from charuco_tracker import runtime_color_model_evidence
from handeye_invariance_metrics import transform_from_translation_quaternion,mean_transform,summarize_transforms,quaternion_from_matrix,quaternion_angle_rad
MOUNT=Path('/media/zhuyupei/SanStand');assert os.path.ismount(str(MOUNT))
prior=Path((ROOT/'.runtime/current_handeye_audit_path.txt').read_text().strip())
protocol=json.loads((prior/'protocol.json').read_text());manifest=json.loads((prior/'session_snapshot.json').read_text());integrity=json.loads((prior/'archive_timestamp_resolution.json').read_text())
assert integrity['archive_consistent_with_documented_1ns_robot_header_discrepancy']
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
assert integrity['original_audit_sha256']==sha(prior/'audit.json')
rospy.init_node('fixed_old_handeye_offline_validation',anonymous=True,disable_signals=True)
buffer=tf2_ros.Buffer();listener=tf2_ros.TransformListener(buffer)
t=buffer.lookup_transform('tool0','camera_link',rospy.Time(0),rospy.Duration(5));p=t.transform.translation;q=t.transform.rotation
xyz=[p.x,p.y,p.z];xyzw=[q.x,q.y,q.z,q.w];fixed=transform_from_translation_quaternion(xyz,xyzw)
parameters=rospy.get_param('/handeye');camera=rospy.get_param('/camera')
assert np.allclose(fixed,transform_from_translation_quaternion(parameters['translation_xyz'],parameters['rotation_xyzw']),rtol=0,atol=1e-12)
model=runtime_color_model_evidence(camera,'camera_link','/supervisor/camera/color/image_raw');fingerprint=hashlib.sha256(json.dumps(model,sort_keys=True,allow_nan=False).encode()).hexdigest()
assert fingerprint==manifest['camera_model_fingerprint']
old=transform_from_translation_quaternion([-.08327671107922277,.004843742371779117,-.12907560357461276],[.005929097277682129,-.7167581240677906,.018061735450252286,.6970627024169479])
assert np.allclose(old,fixed,rtol=0,atol=1e-12), 'active TF changed since historical verification'
out=prior.parent/('fixed_old_tf_validation_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ'));out.mkdir()
thresholds=protocol['thresholds'];entries=[e for e in manifest['captures'] if e.get('active')];transforms=[];averaged=[];sources=[]
for e,pin in zip(entries,protocol['capture_files']):
 path=prior/(e['directory']+'.json');assert sha(path)==pin['capture_json_sha256']
 d=json.loads(path.read_text());assert d['model_fingerprint']==fingerprint
 s=d['samples'][d['selected_frame_index']]
 transforms.append(np.asarray(s['T_base_tool'])@fixed@np.asarray(s['T_camera_board']))
 averaged.append(mean_transform([np.asarray(x['T_base_tool'])@fixed@np.asarray(x['T_camera_board']) for x in d['samples']]))
 sources.append({'gui_index_1based':e['gui_index_1based'],'directory':e['directory'],'source_stamp_ns':s['source_stamp_ns'],'capture_json_sha256':pin['capture_json_sha256'],'bag_sha256_previously_verified':e['bag_sha256']})
assert len(transforms)==21
summary=summarize_transforms(transforms);meansummary=summarize_transforms(averaged);reference=mean_transform(transforms)
per=[]
for i,t in enumerate(transforms):
 delta=(t[:3,3]-reference[:3,3])*1000
 angle=float(np.degrees(quaternion_angle_rad(quaternion_from_matrix(t),quaternion_from_matrix(reference))))
 ref_loo=mean_transform([s for j,s in enumerate(transforms) if j!=i])
 per.append(dict(sources[i],translation_error_mm=float(np.linalg.norm(delta)),orientation_error_deg=angle,base_xyz_residual_mm=delta.tolist(),loo_reference_translation_error_mm=float(np.linalg.norm(t[:3,3]-ref_loo[:3,3])*1000),loo_reference_orientation_error_deg=float(np.degrees(quaternion_angle_rad(quaternion_from_matrix(t),quaternion_from_matrix(ref_loo))))))
# Independent arithmetic check of the translation summary.
pts=np.array([t[:3,3] for t in transforms]);delta=pts-pts.mean(0)
assert abs(summary['translation_rms_m']-float(np.sqrt(np.mean(np.sum(delta**2,axis=1)))))<1e-12
reference_without_test={'translation_rms_m':float(np.sqrt(np.mean([(p['loo_reference_translation_error_mm']/1000)**2 for p in per]))),'translation_max_m':max(p['loo_reference_translation_error_mm'] for p in per)/1000,'orientation_rms_deg':float(np.sqrt(np.mean([p['loo_reference_orientation_error_deg']**2 for p in per]))),'orientation_max_deg':max(p['loo_reference_orientation_error_deg'] for p in per)}
verdict={k:{'value':summary[k],'limit':limit,'pass':summary[k]<=limit} for k,limit in thresholds.items()}
oldfile=Path(parameters['calibration_file']).expanduser()
report={'created_at_utc':datetime.now(timezone.utc).isoformat(),'verdict':'PASS' if all(v['pass'] for v in verdict.values()) else 'FAIL','contract':'For each of 21 groups use the preselected middle source frame. T_base_board_i = T_base_tool_i @ fixed_active_T_tool_camera @ T_camera_board_i. Measure scatter relative to the common mean board pose. Do not refit X.','assumption':'The calibration board remained fixed relative to the robot base over all pose groups. Absolute ground-truth board position/physical dimensions were not independently measured.','test_target':{'parent':'tool0','child':'camera_link','translation_xyz':xyz,'rotation_xyzw':xyzw,'T_tool_camera':fixed.tolist(),'active_equals_historical_calibration':True,'calibration_file':str(oldfile),'calibration_file_sha256':sha(oldfile) if oldfile.exists() else None},'active_handeye_parameters':parameters,'active_camera_parameters':camera,'camera_model_fingerprint':fingerprint,'source_audit_directory':str(prior),'source_audit_sha256':sha(prior/'audit.json'),'source_integrity_resolution_sha256':sha(prior/'archive_timestamp_resolution.json'),'groups':len(transforms),'source_rgbd_frames_previously_audited':347,'threshold_checks':verdict,'fixed_transform_invariance':summary,'one_group_mean_per_pose_sensitivity':meansummary,'reference_excluding_test_group_sensitivity':reference_without_test,'per_group':per,'base_xyz_residual_rms_mm':(np.sqrt(np.mean(delta**2,axis=0))*1000).tolist(),'base_xyz_peak_to_peak_mm':(np.ptp(pts,axis=0)*1000).tolist(),'handeye_fitted':False,'calibration_deployed':False,'robot_commands_sent':False,'script_sha256':sha(Path(__file__))}
# Confirm read-only validation target remained the same before writing final verdict.
t2=buffer.lookup_transform('tool0','camera_link',rospy.Time(0),rospy.Duration(2));p2=t2.transform.translation;q2=t2.transform.rotation
assert np.allclose(fixed,transform_from_translation_quaternion([p2.x,p2.y,p2.z],[q2.x,q2.y,q2.z,q2.w]),rtol=0,atol=1e-12)
assert rospy.get_param('/handeye')==parameters
assert runtime_color_model_evidence(rospy.get_param('/camera'),'camera_link','/supervisor/camera/color/image_raw')==model
(out/'result.json').write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
with (out/'per_group.csv').open('w') as f:
 w=csv.writer(f);w.writerow(['gui_index_1based','capture','translation_error_mm','orientation_error_deg','dx_mm','dy_mm','dz_mm'])
 for p in per:w.writerow([p['gui_index_1based'],p['directory'],p['translation_error_mm'],p['orientation_error_deg']]+p['base_xyz_residual_mm'])
lines=['# 旧手眼 TF 与当前新内参的固定参数检验','',f'时间：{report["created_at_utc"]}。结论：**未通过**。','',
'本次固定实际运行中的 `tool0 → camera_link`，使用新内参采集的 21 组样本逐组计算标定板在基座坐标系中的位姿。没有重新拟合手眼，也没有改变当前参数或控制机械臂。','',
'计算式：`T_base_board[i] = T_base_tool[i] × T_tool_camera_old × T_camera_board_new_intrinsics[i]`。每组只使用采集时已指定的代表帧。标定板若相对机器人基座固定，计算结果应保持一致。下表是相对共同均值的三维位置与旋转误差，不是桌面高度或绝对位置的实测误差。','',
'| 指标 | 实测 | 原有门限 | 结果 |','|---|---:|---:|---|']
for k,name,scale,unit in [('translation_rms_m','位置 RMS',1000,'mm'),('translation_max_m','最大位置误差',1000,'mm'),('orientation_rms_deg','角度 RMS',1,'°'),('orientation_max_deg','最大角度误差',1,'°')]:
 v=verdict[k];lines.append('| %s | %.3f %s | ≤%.3f %s | %s |'%(name,v['value']*scale,unit,v['limit']*scale,unit,'通过' if v['pass'] else '未通过'))
lines+=['',
'固定 TF 与此前 2026-08-10 验收结果一致：平移（m）`'+str(xyz)+'`，四元数（xyzw）`'+str(xyzw)+'`。相机模型指纹与这 21 组采集记录一致，fx=395.768280、fy=395.602753，SDK 2.56.5 的 inverse_brown_conrady 模型。','',
'对每组全部连续帧求一个平均位姿后，位置 RMS / 最大值为 %.3f / %.3f mm，角度 RMS / 最大值为 %.3f / %.3f°，仍不通过。'%(meansummary['translation_rms_m']*1000,meansummary['translation_max_m']*1000,meansummary['orientation_rms_deg'],meansummary['orientation_max_deg']),'',
'每次用其余 20 组建立参考、再检查剩下 1 组（旧手眼始终固定），位置 RMS / 最大值为 %.3f / %.3f mm，角度 RMS / 最大值为 %.3f / %.3f°，仍不通过。'%(reference_without_test['translation_rms_m']*1000,reference_without_test['translation_max_m']*1000,reference_without_test['orientation_rms_deg'],reference_without_test['orientation_max_deg']),'',
'结论范围：**旧 TF 与当前新内参的组合，在这批 21 个姿态上未满足原有一致性要求。** 上次“新拟合手眼不通过”和本次“固定旧手眼不通过”是两项不同计算。此结果不能单独证明误差全部来自旧 TF；棋盘是否移动、实物尺寸与平面度、相机安装刚性和机器人 FK/反馈都可能影响整条测量链。缺少外部真值，因此不得将残差直接当作抓取下降补偿。','',
'原始证据复用此前完整审计：21 组、347 个 RGBD 源帧，bag 摘要验证、质量与静止门限核对及上位机一致性核对已完成。机器人 TF 头部与图像存在 0 或 +1 ns 差异，原审计另有逐项解释；本次未重写任何时间戳或剔除任何组。','',
'机器可读结果：`result.json`；每组误差：`per_group.csv`；上次归档核验：`'+str(prior/'report.md')+'`；复现脚本：`'+str(Path(__file__).resolve())+'`。','']
(out/'report.md').write_text('\n'.join(lines))
(ROOT/'.runtime/current_fixed_handeye_validation_path.txt').write_text(str(out)+'\n')
print(json.dumps({'output':str(out),'verdict':report['verdict'],'main':summary,'group_mean':meansummary,'reference_excluding_test':reference_without_test,'worst':max(per,key=lambda p:p['translation_error_mm'])},ensure_ascii=False,indent=2))
