#!/usr/bin/env python3
"""Read existing immutable evidence and write a scope opinion; no ROS or fitting."""
import hashlib
import json
import os
from pathlib import Path
import numpy as np

MOUNT = Path('/media/zhuyupei/SanStand')
LOG = MOUNT / '实验记录/Robot-Controller/ros_log'
RUN = LOG / 'restart_grasp_20260929T013841Z'
FIT = LOG / 'grasp_repair_resume_20260928T120606Z/expanded_metric_training_once'
PROJECT = Path(__file__).resolve().parents[1]
OUT = RUN / 'candidate_geometry_scope_review'


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read(path):
    return json.loads(path.read_text())


def main():
    if not os.path.ismount(MOUNT):
        raise RuntimeError('SanStand unavailable; no fallback')
    if OUT.exists():
        raise RuntimeError('Do not overwrite scope review')
    source_paths = {
        'historical_acceptance': LOG / 'handeye_auto_20260927T102700Z/final_validation_20260927T110130Z/result.json',
        'local_support_configuration': PROJECT / 'src/alicia_flexible_grasp_supervisor/config/grasp_params.yaml',
        'acceptance_summary': FIT / 'acceptance_summary.json',
        'candidate_manifest': FIT / 'candidate_model/manifest.json',
        'candidate_verification': FIT / 'candidate_model/verification.json',
        'independent_review': RUN / 'expanded_metric_independent_review/report.json',
        'prospective_report': RUN / 'prospective_all_with_small_j3_partial/report.json',
        'prospective_frames': RUN / 'prospective_all_with_small_j3_partial/selected_frames.json',
        'small_j3_report': RUN / 'prospective_small_j3/report.json',
        'old_failed_terminal': LOG / 'bounded_shoulder_elbow_validation_20260929T025136Z_167696/terminal.json',
        'new_small_j3_terminal': LOG / 'bounded_j3_small_response_20260929T032235Z_244644/terminal.json',
        'evaluator': PROJECT / '.runtime/evaluate_prospective_metric_candidate.py',
    }
    historical = read(source_paths['historical_acceptance'])
    limits = {k: v['limit'] for k, v in historical['threshold_checks'].items()}
    assert limits == dict(translation_rms_m=.003, translation_max_m=.005,
                          orientation_rms_deg=1., orientation_max_deg=2.)
    configuration = source_paths['local_support_configuration'].read_text()
    assert 'maximum_support_normal_angle_deg: 4.0' in configuration
    assert 'maximum_support_offset_delta_m: 0.004' in configuration
    report = read(source_paths['prospective_report'])
    assert report['evaluator_source_sha256'] == digest(source_paths['evaluator']) == '42295547dd7aadfc11df2ff75c3a99212f8fa50a367bd667f67fa083f6c21bcb'
    manifest = read(source_paths['candidate_manifest'])
    for k in ('candidate_model_sha256', 'candidate_handeye_sha256', 'frozen_fit_sha256'):
        assert manifest[k] == report[k]
    for pathkey, hashkey in (('candidate_urdf_path', 'candidate_model_sha256'),
                             ('candidate_handeye_path', 'candidate_handeye_sha256'),
                             ('frozen_fit_path', 'frozen_fit_sha256')):
        assert digest(Path(manifest[pathkey])) == manifest[hashkey]
    candidate = report['models']['candidate']['all_validation']
    assert candidate['historical_3mm_5mm_1deg_2deg_numeric_pass']
    assert candidate['local_4mm_4deg_numeric_pass']
    frames = read(source_paths['prospective_frames'])
    q = np.degrees(np.array([row['q'][:6] for row in frames['baseline'] + frames['validation']]))
    independent = read(source_paths['independent_review'])
    prior = read(source_paths['old_failed_terminal'])
    small = read(source_paths['new_small_j3_terminal'])
    assert prior['status'] == 'failed' and prior['commands_published'] == 2
    assert small['status'] == 'completed' and small['commands_published'] == 1
    assert report['status'] == 'partial_command_phase_coverage'
    assert report['control_contract_outcome'] == 'FAILED_PREVIOUS_COMMAND_CONTRACT_NOT_CLEARED'
    missing = [p for p in report['coverage']['command_phases'] if not p['multiframe_coverage_complete']]
    assert len(missing) == 1 and missing[0]['phase'] == '02_validation_J3_positive' and missing[0]['qualified_frame_count'] == 0
    summary = read(source_paths['acceptance_summary'])
    result = dict(
        schema=1, review_only=True, refit_performed=False, ROS_access=False,
        activation_review_modified=False, motion_authorized=False, new_acceptance_thresholds=False,
        opinion='GEOMETRIC_EVIDENCE_SUPPORTS_IDLE_PAIR_LOAD_AND_READ_ONLY_COMPUTE_VERIFICATION_ONLY',
        interpretation='This evidence opinion is not an execute flag or a claim of successful future grasp. Historical failed command outcome is retained but is not an impossible prerequisite requiring historical failure to become success.',
        frozen_pair={k: manifest[k] for k in ('candidate_model_sha256', 'candidate_handeye_sha256', 'candidate_pair_sha256', 'frozen_fit_sha256')},
        original_geometric_limits=limits,
        existing_local_support_limits=dict(maximum_support_normal_angle_deg=4., maximum_support_offset_delta_m=.004),
        prospective=dict(validation_frame_count=report['validation_frame_count'], fixed_baseline_frame_count=report['baseline_frame_count'],
            metrics=report['models'], original_coverage_status=report['status'], missing_original_phase=missing,
            distinct_quantized_posture_count=int(len(set(map(tuple, q)))), joint_min_deg=q.min(0).tolist(),
            joint_max_deg=q.max(0).tolist(), joint_span_deg=np.ptp(q, axis=0).tolist(),
            caveat='264 correlated stationary frames, 11 quantized postures; not 264 independent motion experiments. Candidate transformed frozen raw observations collected under original runtime model.'),
        independent8=independent['independent8'],
        archived_grasp_visible_support=dict(**summary['failed_grasp_support_drift'],
            maximum_same_visible_patch_bound_mm=independent['max_common_patch_local_difference_bound_mm'],
            maximum_normal_change_deg=independent['max_common_patch_normal_change_deg']),
        old_control_diagnostic=dict(status='FAILED_RETAINED', terminal=prior, missing_phase=missing[0],
            retrospective_success_required_for_idle_geometry_load=False),
        small_control_diagnostic=dict(status='COMPLETED_FOR_ITS_OWN_SINGLE_SMALL_INPUT_ONLY', terminal=small,
            fills_old_missing_phase=False, clears_old_failure=False),
        future_grasp=dict(status='NOT_YET_PHYSICALLY_VALIDATED_WITH_CANDIDATE', motion_authorized=False,
            limitation='Visible board/table invariance is not independent fingertip metrology, contact, holding, lift or dynamic following validation.'),
        source_evidence={k: dict(path=str(v), sha256=digest(v)) for k, v in source_paths.items()},
        generator_sha256=digest(Path(__file__)),
    )
    text = f'''# 冻结候选模型的验收范围复核

证据意见：**冻结 URDF 与手眼参数已具备进入“系统空闲时成对加载、随后只读核对 FK/TF 与模型身份”的几何依据。此意见不授权任何新运动，不代表抓取已修复，也不修改 activation_review。** 原失败诊断应继续记为失败；要求历史失败本身变成成功不是原几何验收标准，不能据此永久否决上述有限加载。

| 分开的验收问题 | 结论 | 证据范围 |
|---|---|---|
| 候选几何是否符合原标准 | 已取得符合原数值标准的独立观测与回放证据 | 新固定板 264 帧、旧 8 帧留出、旧空抓的共视桌面 |
| 原 +22 字 J3 诊断是否成功 | 失败，保持不变 | J3 发布后 J4 超出原 held 轴边界；该阶段没有合格静止窗 |
| 新 +8 字 J3 诊断是否成功 | 仅该次小输入完成 | 一条命令、独立新阶段及完整归档；不补旧缺窗 |
| 未来候选模型抓取是否成功 | 尚未验证 | 没有候选运行模型下的真实夹持/提升结果 |

## 原标准及其含义

原始手眼验收记录 `{source_paths['historical_acceptance']}` 的 threshold_checks 明确是：位置 RMS ≤3 mm、最大值 ≤5 mm；角度 RMS ≤1°、最大值 ≤2°。此前 2.244 mm、0.591°是那一组实测结果，不是新候选必须优于的门限。`docs/operations/2026-09-28-task-pose-handeye-validation.md` 也据 3/5 mm 判定原任务姿态验证失败。

现有 `grasp_params.yaml` multiview 配置原有支撑面局部差 ≤4 mm、法向差 ≤4°。这是局部支撑注册标准；不能把板原点平移误差直接称为完整接触域的支撑面证明。本复核分别列出板几何数值和原共视桌面区域的保守差界，未新增阈值，也不要求每一个子指标都严格优于旧模型。

## 冻结模型的证据

| 验证数据 | 位置 RMS / 最大值（mm） | 角度 RMS / 最大值（°） | 说明 |
|---|---:|---:|---|
| 新固定板 264 帧，原模型 | 0.640439 / 1.599616 | 0.090046 / 0.197789 | 同一个首命令前固定 Y |
| 新固定板 264 帧，冻结候选 | 0.473389 / 1.438767 | 0.087890 / 0.189115 | 未重拟合，符合原 3/5 mm、1/2°及板数值 4/4 比较 |
| 原 8 帧留出，冻结候选 | 0.591864 / 0.822819 | 0.140173 / 0.257268 | 参考来自原 22 帧训练组；留出帧未参与拟合 |

真正 08:42 空抓的共视桌面回放中，闭爪前约 −9.94…−10.01 mm 的旧模型高度漂移，候选为 +0.262…+0.329 mm。六阶段、18 个原选定区域的锚点高度差最大 0.911152 mm；独立复核将整个相同可见区域的倾角影响计入，最大局部差保守界 2.541297 mm、法向差最大 1.046346°，在原局部 4 mm / 4°范围内。像素区域、平面拟合、实际关节和相机内参未重新选择或调整。此证据仅覆盖当时可见桌面，不外推到不可见电机接触点。

候选拟合仍是已冻结的一次 45 样本、10 参数结果；原 task 的 16/2/5 组已算训练，不能重复当留出。旧 8 帧及空抓日志虽未进入拟合目标，但已反复用于模型开发比较，不能称全新盲测。新 264 帧在候选冻结后取得，使用原运行模型采集的 raw q、同源图像/板 TF 对候选离线计算；候选尚未在实机规划运动中使用。

模型表示另经 1114 组合法 q 验证：新 URDF FK(q) 与原 FK(q+固定零位) 等价至约 1e−15 m，手眼相机链也一致；check_urdf、生产 FK 解析通过。仅 Joint2…5 的 origin 旋转吸收零位，原命令/反馈、axis、limits、mesh、frame 不变。加载必须使用已冻结的一对文件，不能只替换 X 或把零位加到下发关节命令。

## 覆盖范围与历史失败

新固定板数据是 24 帧固定基准、264 帧合格验证图，来自 11 个量化关节姿态；同一静止窗的帧相关，不是 264 次独立动作。包括基准的六轴范围跨度为 `{np.ptp(q, axis=0).tolist()}` 度：J1/J5 没有展开，J2/J3/J4/J6 仅约 1.76–2.02°。这支持当前局部模型一致性，不足以声称完整工作空间、速度、负载和接触高度均已验证。

`bounded_shoulder_elbow_validation_20260929T025136Z_167696` 仍是失败：J2 完成；+22 字 J3 后，J4 相对该次初始 +0.087890625°变化 −0.439453125°，超过原 ±0.4° held 界，J4 SDK 未改变。该 J3 phase 仍为 0 合格帧。后续被动观察和本次 +8 字成功是各自独立阶段，不能补齐旧 phase，也不能宣称根因消失。

汇总报告继续保持 `partial_command_phase_coverage` 和 `FAILED_PREVIOUS_COMMAND_CONTRACT_NOT_CLEARED`。它们描述历史动作/覆盖完整性；所保留的合格几何观测可以独立用于模型数值验收。不能将“整段控制实验未全部完成”等同于“全部几何证据无效”，也不能将几何达标反过来覆盖该控制故障。

## 有限加载意见

本证据支持的下一步仅为：在已有空闲编排下成对加载这两个固定模型文件，以当前同源关节读数核对 RSP/MoveGroup/客户端 FK、TF、模型哈希和新代次/缓存来源。其目的是验证计算消费者确实使用同一新模型；不允许借此次几何意见提交轨迹、闭爪、提升、返回或复试旧 +22 动作。此复核本身不启动该编排，也不更改发布许可。

既有故障保留、加载脚本的可回滚性、实时反馈状态与模型身份仍由现有编排按自身契约处理；这不要求历史日志改成通过。本轮没有新增运行或几何阈值。加载后的实际抓取仍须根据当时的新识别、补偿、路径和反馈证据执行既有任务流程，不能用本报告替代真实抓取结果。

固定 URDF SHA256：`{manifest['candidate_model_sha256']}`；固定 X SHA256：`{manifest['candidate_handeye_sha256']}`。所有引用路径与 SHA256 见同目录 `scope_review.json`。ROS、activation_review、生产参数和原实验文件均未修改。
'''
    OUT.mkdir()
    (OUT / 'scope_review.json').write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False) + '\n')
    (OUT / 'scope_review.md').write_text(text)
    eligibility = dict(
        schema=1,
        scope='idle_pair_reload_and_readonly_compute_verification',
        geometry_evidence_eligible=True,
        evidence_opinion_only=True,
        deployment_or_execution_permission_granted_by_this_file=False,
        motion_authorized=False,
        control_repair_claimed=False,
        physical_grasp_validated=False,
        refit_performed=False,
        model_parameter_count=10,
        frozen_model_definition='X six DOF and Joint2..5 zero offsets only; K, lengths, scales, Joint1/6 offsets, raw commands, feedback and limits unchanged',
        **result['frozen_pair'],
        candidate_urdf_path=manifest['candidate_urdf_path'],
        candidate_handeye_path=manifest['candidate_handeye_path'],
        candidate_manifest=dict(path=str(source_paths['candidate_manifest']), sha256=digest(source_paths['candidate_manifest'])),
        combined_report=dict(path=str(source_paths['prospective_report']), sha256=digest(source_paths['prospective_report']),
            status=report['status'], control_contract_outcome=report['control_contract_outcome'],
            validation_frame_count=report['validation_frame_count']),
        scope_review=dict(path=str(OUT/'scope_review.json'), sha256=digest(OUT/'scope_review.json')),
        original_geometric_limits=limits,
        candidate_prospective_metrics=candidate,
        existing_local_support_limits=result['existing_local_support_limits'],
        archived_visible_patch_metrics=result['archived_grasp_visible_support'],
        coverage=dict(distinct_quantized_posture_count=result['prospective']['distinct_quantized_posture_count'],
            joint_span_deg=result['prospective']['joint_span_deg'], correlated_frames=True,
            full_workspace_validated=False, candidate_used_for_hardware_motion=False),
        preserved_history=dict(original_failure_cleared=False, missing_phase_filled=False,
            original_failed_terminal=dict(path=str(source_paths['old_failed_terminal']), sha256=digest(source_paths['old_failed_terminal']),
                status=prior['status'], commands_published=prior['commands_published']),
            missing_original_command_phase=missing[0], later_small_j3_is_separate=True),
        scope_exclusions=['new trajectory publication', 'joint enable/disable or command changes',
            'gripper closing', 'grasp or lift authorization', 'clearing the prior diagnostic failure',
            'substituting a later small-command phase for the earlier missing phase'],
        new_acceptance_thresholds=False,
    )
    (OUT / 'idle_pair_reload_geometry_eligibility.json').write_text(json.dumps(eligibility, ensure_ascii=False, indent=2, allow_nan=False) + '\n')
    (OUT / Path(__file__).name).write_bytes(Path(__file__).read_bytes())
    print(json.dumps(dict(output=str(OUT), opinion=result['opinion'], thresholds=limits,
                          historical_failure_preserved=True, new_motion_authorized=False), ensure_ascii=False))


if __name__ == '__main__':
    main()
