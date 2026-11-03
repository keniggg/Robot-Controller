"""Package sealed offline evidence and update documentation without ROS access."""
import hashlib
import json
import os
import time
from pathlib import Path

W = Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
M = Path('/media/zhuyupei/SanStand')
assert os.path.ismount(str(M)), 'EXPERIMENT_STORAGE_UNAVAILABLE'
N = M / '实验记录/Robot-Controller/ros_log/full_latest_restart_20261010T002347Z'
A = N / 'gen13_handoff_analysis'

def sha(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()

def write(p, data):
    p.write_text(json.dumps(data, ensure_ascii=False, indent=2) + '\n')

evidence = []
for directory in ['offline_camera_chain', 'offline_depth_processing', 'offline_support_math',
                  'offline_support_domains', 'offline_encoder_quantization']:
    for p in sorted((A / directory).glob('*.json')):
        json.loads(p.read_text())
        evidence.append({'path': str(p), 'sha256': sha(p)})
manifest = N / 'verification/gen10_shared_reference_combined_final/deployment_sources.json'
sources = json.loads(manifest.read_text())['source_sha256']
differences = [{'path': rel, 'expected_sha256': expected,
                'actual_sha256': sha(W / rel) if (W / rel).exists() else None}
               for rel, expected in sources.items()
               if not (W / rel).exists() or sha(W / rel) != expected]
assert not differences, differences

summary = {
    'recorded_ns': time.time_ns(), 'mode': 'offline_only',
    'disposition': 'offline_software_checks_completed_no_confirmed_causal_code_defect',
    'physical_grasp_succeeded': False,
    'failure': 'GEN13 second observation: 17 post-arrival frames have raw support local distance 5.543–5.904 mm > unchanged 4 mm.',
    'findings': [
        {'area': 'projection_and_rectification',
         'result': 'Independent map digests, inverse mapping, paired headers and geometry agree. Independent fit second-view difference 5.790 mm; recovered source SDK rays 5.806 mm. Pixel rounding does not explain discrepancy.'},
        {'area': 'filter_history',
         'result': 'Recorded periodic snapshots show regions cleared during both motions and warming again. 82 offline regressions pass. This does not establish physical filtered depth accuracy.'},
        {'area': 'support_math_and_reference',
         'result': 'Original comparison and independent distance math agree. 514 states keep frozen reference bitwise unchanged; initial candidate reconstructed. Signs, normalization, common-frame transforms and boundary cases pass.'},
        {'area': 'support_sampling_domain',
         'result': 'Equal initial cell weighting in recorded common base-XY domains still gives second-station plane difference 5.348–5.555 mm. Different support regions alone are not sufficient under this recorded coordinate model.',
         'limits': 'Recorded TF defines the grid; cells are not independent physical correspondences. Coordinatewise medians, view-dependent Tukey weights and possible extrapolation remain.'},
        {'area': 'feedback_timing',
         'result': 'Using any accepted q within one second before/after the source changes the same-plane comparison by at most 0.1062 mm in this record.'},
        {'area': 'encoder_quantization_sensitivity',
         'result': 'Conditional independent half-count errors at two stations: 4096 corner pairs change a point comparison by up to 1.8771 mm; most favorable corner reduces maximum local difference to 3.9709 mm. Quantization alone does not explain all 5.7675 mm, but can affect the 4 mm boundary.',
         'limits': 'Half-count centering is an analysis assumption, not verified encoder precision or rounding behavior. Extreme combinations are not observed errors. Mechanical compliance, systematic zero errors and calibration are outside this model.'},
    ],
    'unresolved': [
        'Native depth before SDK alignment/spatial processing was not recorded; actual sensor/view-dependent or processing bias cannot be separated offline.',
        'User-reported motion under external force is real user evidence, but the amount not represented by accepted encoder feedback is unknown.',
        'Recorded FK/TF consistency does not establish physical extrinsic/robot accuracy. Existing evidence does not justify a new calibration fit.',
    ],
    'next_diagnostic_when_user_explicitly_restores_online_work': {
        'status': 'not_started',
        'purpose': 'Distinguish sensor/processing depth change from camera pose discrepancy before another grasp retry.',
        'minimum_missing_evidence': 'Source-paired native pre-align/pre-spatial depth and final processed depth, corresponding RGB, native intrinsics/extrinsics/scale and frame timestamps, accepted joint feedback; compare one station first. Any later motion needs then-current authorization and valid checks.',
        'no_automatic_calibration_or_threshold_change': True,
    },
    'source_verification': {'manifest': str(manifest), 'manifest_sha256': sha(manifest),
                            'count': len(sources), 'all_unchanged': not differences},
    'restrictions': {'ros_access': False, 'device_commands': False, 'live_reloads': False,
                     'temperature_access': False, 'production_or_threshold_changes': False},
    'evidence': evidence,
}
out = A / 'offline_audit_summary.json'
assert not out.exists(), 'preserve previous final summary'
write(out, summary)
pointers = [N / 'current_progress.json', W / '.runtime/latest_full_ros_start.json',
            W / '.runtime/current_full_latest_ros_20261008.json', W / '.runtime/codex_reload_handoff_20261009.json',
            N / 'codex_ui_analysis/task_resume_checkpoint.json']
for p in pointers:
    obj = json.loads(p.read_text())
    obj.update({'updated_ns': summary['recorded_ns'], 'active_work': 'gen13_offline_audit_completed',
                'offline_audit_summary': str(out), 'offline_audit_summary_sha256': sha(out),
                'offline_only': True, 'automatic_grasp_running': False, 'automatic_validation_running': False,
                'current_alignment_pose_is_current': False, 'physical_grasp_verified': False,
                'new_explicit_ready_authorization_required_before_online_work': True,
                'current_issue': 'Second-view support discrepancy remains unresolved. No causal code defect reproduced; conditional encoder quantization can affect threshold, native depth evidence missing.',
                'next_action': 'Offline analysis complete for available evidence. Keep hardware untouched; await explicit user readiness before collecting missing native/processed paired depth. Do not reuse old alignment/epoch or start another automatic grasp.'})
    write(p, obj)

doc = W / 'docs/operations/2026-10-08-full-latest-ros-startup.md'
section = '''

### GEN13 纯离线软件原因排查结果

本轮只使用封存记录与源码，未访问 ROS、机械臂或温度，未重载节点；已部署清单 153 个文件逐项 SHA256 均保持。结论不是“抓取已修好”，而是将第二站持续支撑偏差与已修复的时效交接问题分开，并限定哪些软件原因有证据排除。

| 检查项 | 结果及边界 |
|---|---|
| 相机投影、整流、单位与帧绑定 | 59 份几何配置一致，独立重建两映射的 SHA 与录制相符；独立拟合仍 5.790 mm，绕开像素取整的源 SDK 射线仍 5.806 mm。不是扩画布内参或像素取整造成的约 5.7 mm 偏移。相机四源文件另与该次 camera_transition.json 绑定。 |
| 时间滤波跨站历史 | 两次运动的周期元数据均显示区域清空，到位后重新预热；82 项逻辑回归通过。不能据此证明真实深度经过 spatial 后无偏差。 |
| 平面比较和冻结参考 | 514 条状态的原参考逐位不变；初始唯一候选可重建。符号/缩放、换基、边界与凸域检验通过；第二站的字面法向/竖直高度差也大于 4 mm。 |
| 不同桌面采样区域 | 在记录 TF 定义的共同 XY 栅格区域拟合，第二站仍 5.348–5.555 mm。区域变化单独不足以解释偏移；共同栅格不是独立物理同点证据，鲁棒重加权和外推仍有限制。 |
| 反馈时间对应 | 初始和第二站源帧前后各 1 秒的实录关节替换，对同一相机平面的差值最大影响约 0.106 mm，不能解释这次全部偏移。 |
| 编码器量化 | 假定两站各轴独立 ±半个计数，4096 个角点组合最多改变约 1.877 mm；最有利组合将最大局部差降至 3.971 mm。不能解释全部 5.767 mm，但不能排除对 4 mm 边界判定有贡献。半计数居中与极端组合是分析假设，不是实测误差或完整硬件精度上限。 |

尚缺对齐前、spatial 前的原生深度，因此现有记录不能把传感器随视角变化的误差、处理偏差、未被编码器反映的结构形变和物理外参误差唯一分开。用户报告的外力下关节摆动已记录，但不是本次偏移的已证实原因。当前没有依据改标定或放宽 4 mm，更不以本次离线通过代表实物抓取成功。

下一步缺失证据是同源原生深度与最终处理深度的成对记录及对应相机几何/时间戳、实测关节。该诊断尚未启动；保持用户要求的关机/离线边界，收到新的在线准备授权后才处理。原对准和 epoch 不得续用。

完整证据清单、来源 SHA、条件与未解决项见外盘 `gen13_handoff_analysis/offline_audit_summary.json`；各专题原报告与共同区域/量化独立复核完整保留，local_only 不上传、不删除。
'''
with doc.open('a') as f:
    f.write(section)
(N / doc.name).write_bytes(doc.read_bytes())
print(json.dumps({'summary':str(out),'sha256':sha(out),'reports':len(evidence),
                  'sources_unchanged':len(sources),'doc_mirror_equal':sha(doc)==sha(N/doc.name)},ensure_ascii=False))
