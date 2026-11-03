#!/usr/bin/env python3
"""Closed-record FK/remaining-budget analysis. No ROS node/services or motion.

The reconstructed object is used only for the existing pure search functions;
no synthetic sample is represented as a live stationary or execution contract.
"""
import hashlib
import json
import math
from pathlib import Path

import numpy as np
from scipy.optimize import minimize
from alicia_flexible_grasp.robot.pregrasp_compensation import (
    ARM_NAMES, PregraspCompensation, Q, SerialUrdfFk, pose_matrix,
)

ROOT = Path(__file__).resolve().parents[1]
ARCHIVE = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
TRIAL = ARCHIVE / 'ros_log/feedforward_speed_grasp_20260929T064011Z'
TIMING = TRIAL / 'timing_analysis/report.json'
STATIONARY = TRIAL / 'timing_analysis/stationarity_and_count_budget.json'
MODEL = ROOT / '.runtime/candidate_models/expanded_metric_20260929/alicia_expanded_metric_candidate.urdf'
MODEL_SHA = '7812d6b69e854ed9f37b4e3cbd813ecd53dab3504065b2d04d287abc6fd852e0'


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    if not Path('/media/zhuyupei/SanStand').is_mount():
        raise RuntimeError('SanStand required; no system-disk fallback')
    assert sha(MODEL) == MODEL_SHA
    timing = json.loads(TIMING.read_text())
    stationary = json.loads(STATIONARY.read_text())
    episode = next(e for e in timing['analysis']['feedforward_episodes'] if e['pose_index'] == 1)
    source = next(e for e in stationary['episodes'] if e['pose_index'] == 1)
    p, r = episode['proposal'], episode['measured_result']
    assert p['plan_id'] == r['plan_id'] == '62e92d05507d1e48a0f49773'
    assert p['initial_sdk_counts'] == source['original_32_count_origin']
    assert r['sdk_counts'] == source['target_counts']
    assert r['measured_counts'] == source['after_actual_counts']
    fk = SerialUrdfFk(MODEL.read_text(), ARM_NAMES)
    goal = pose_matrix(r['fixed_goal_pose'])
    measured = np.asarray(source['post_action_stationary_replay']['accepted_positions_rad'])
    before = np.asarray(p['measured_start_positions_rad'])
    command = np.asarray(r['last_step_response']['command_delta_counts'])
    response = np.asarray(r['last_step_response']['measured_delta_counts'])
    changed = command != 0
    directional = response * np.sign(command)
    strong, weak = changed & (directional >= 2), changed & (directional < 2)
    assert not (np.any(directional[changed] < 0) or
                np.any(abs(response[changed]) > abs(command[changed])+2) or
                np.any(abs(response[~changed]) > 2))

    # Mathematical state of a hypothetical same-episode continuation, NOT a
    # new constructor/episode, current admission or executable CorrectionStep.
    c = PregraspCompensation.__new__(PregraspCompensation)
    c.fk, c.goal, c.goal_values = fk, goal, tuple(r['fixed_goal_pose'])
    c.tolerances, c.endpoint_acceptance = (.006, math.radians(5)), None
    c.initial_counts, c.expected = tuple(p['initial_sdk_counts']), tuple(r['sdk_counts'])
    c.steps = 1  # The full guide already consumed the first original step.
    c.held_axes = set(c.HELD_AXES) | set(int(i) for i in np.flatnonzero(weak))
    c.response_gains = np.ones(6)
    c.response_gains[strong] = np.minimum(1., directional[strong] / abs(command[strong]))
    current_error, current_angle = c.residual(measured)
    guide, guide_error = c._remaining_budget_guide(measured)
    trim = c._guided_step(measured, np.asarray(r['measured_counts']), guide, current_error)
    assert not np.any(trim)
    assert np.isclose(current_error, r['position_error_m'], atol=1e-12)
    sensitivity = []
    for i in range(6):
        d = np.zeros(6); d[i] = Q
        sensitivity.append(((fk(measured+d)[:3, 3]-fk(measured-d)[:3, 3])/2).tolist())
    unit_prediction = before + command*Q

    # A further conditional numerical diagnostic; neither a global optimum
    # proof nor a path/response certificate. Preserve the same original box.
    axes = [0, 3, 5]
    origin, sdk = np.asarray(c.initial_counts), np.asarray(c.expected)
    reach = (c.MAX_STEPS-c.steps)*c.MAX_STEP_COUNTS
    low = np.maximum(origin-c.MAX_TOTAL_COUNTS-sdk, -reach)
    high = np.minimum(origin+c.MAX_TOTAL_COUNTS-sdk, reach)
    def predicted(d):
        delta = np.zeros(6); delta[axes] = d
        return measured + delta*c.response_gains*Q
    optimizations = []
    for seed in ([0, 0, 0], [-30, -30, -40], [30, 30, 0], [-30, 30, -20], [30, -30, -20]):
        result = minimize(lambda d: c.residual(predicted(d))[0], np.asarray(seed, dtype=float),
                          method='SLSQP', bounds=[(low[i], high[i]) for i in axes],
                          constraints=[dict(type='ineq', fun=lambda d: c.tolerances[1]-c.residual(predicted(d))[1])],
                          options=dict(ftol=1e-12, maxiter=150))
        optimizations.append(dict(success=bool(result.success), axes=[ARM_NAMES[i] for i in axes],
                                  delta_counts=result.x.tolist(), residual=c.residual(predicted(result.x))))
    report = dict(
        schema='closed_pose1_remaining_original_budget_v1', plan_id=r['plan_id'],
        scope='Pure FK and existing search replay of a closed failed trial; no production change.',
        hardware_commands_sent=0, ros_service_calls=0, executable_path_proven=False,
        sources={str(path): sha(path) for path in (TIMING, STATIONARY, MODEL, Path(__file__),
            ROOT/'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot/pregrasp_compensation.py',
            ROOT/'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot/pregrasp_feedforward.py')},
        model_sha256=MODEL_SHA, original_sdk_counts=c.initial_counts, current_sdk_counts=c.expected,
        before_actual_counts=source['before_actual_counts'], after_actual_counts=r['measured_counts'],
        command_delta_counts=command.tolist(), actual_response_counts=response.tolist(),
        position_error_before_m=c.residual(before)[0], position_error_after_m=current_error,
        goal_minus_actual_xyz_m=(goal[:3, 3]-fk(measured)[:3, 3]).tolist(),
        unit_response_prediction_error_m=c.residual(unit_prediction)[0],
        actual_minus_unit_prediction_xyz_m=(fk(measured)[:3, 3]-fk(unit_prediction)[:3, 3]).tolist(),
        local_fk_tool_xyz_per_encoder_count_m=sensitivity,
        same_episode_hypothesis=dict(original_step_budget=12, steps_already_used=1,
            original_total_count_radius=32, original_time_limit_sec=60,
            time_and_stationarity_not_renewed=True, held_joints=[ARM_NAMES[i] for i in sorted(c.held_axes)],
            response_gains=c.response_gains.tolist(), remaining_command_low_counts=low.tolist(),
            remaining_command_high_counts=high.tolist(), remaining_guide_counts=guide.tolist(),
            guide_predicted_error_m=guide_error, proposed_small_step_counts=trim.tolist(),
            result='PREGRASP_NO_BOUNDED_IMPROVING_STEP',
            guide_rejection='J1 -2 target is only 1 count from actual, below existing minimum-response separation'),
        bounded_continuous_diagnostic=dict(global_optimality_proven=False, physical_response_proven=False,
                                         paths_not_checked=True, starts=optimizations),
        measured_stationarity=source['post_action_stationary_replay'],
        extended_tail=stationary.get('extended_terminal_tail'),
        causality='J3 directional under-response explains the conditional FK miss; speed causation is not identified. Older .002 trial also under-responded, with different targets and starts.',
        software_proposal='A future response-aware tail could reuse the SAME feedforward.contract after accounting for its first step, preserving original origin/deadline/12-step cap, holding weak axes, and certifying each <=4-count command. This record produces no admissible improving step; do not implement or execute a retry as a cure for this case.',
        outcome='Neither the actual one-shot feedforward nor a hypothetical unchanged-contract tail proves convergence of this old pose1. No threshold, count budget or consumed episode may be reset.')
    out = TRIAL/'remaining_budget_analysis'; out.mkdir(exist_ok=True)
    (out/'report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
    (out/'report.md').write_text('''# 闭袋 pose1 收敛审计

本轮单次前馈未满足原 6 mm 到位门：FK 位置残差 6.786 → 6.451 mm。目标减实际的 z 差为 −6.394 mm。

J3 指令 −8 字，仅响应 −1 字；J6 指令 +32 字、响应 +22 字。SDK−actual 偏差在动作中改变，原“偏差保持不变”的前馈预测 5.226 mm 没有实现。实际比该预测高 2.108 mm；J3 少响应 7 字乘本姿态约 0.303 mm/字的竖直灵敏度，约为 2.123 mm。该数值解释不证明摩擦、回差或速率中哪一项是硬件根因。旧慢速试验也有欠响应，不能将原因归于本次提速。

按原 PregraspCompensation 响应规则假设延续同一 episode，J3 因不足 2 字必须保持，J2/J5 继续保持。J6 已耗尽 SDK 正方向 +32 字预算，不能按实际只走 22 字再“补 10 字”。原预算搜索只给 J1 −2 字、预测 6.403 mm；该目标距实际仅 1 字，现有最小响应条件拒绝，实际提议为全零。五起点连续有界数值诊断也只到约 6.335 mm；这不是全局不可达证明，更不是路径或硬件响应证明。

动作后原停稳窗有效；随后 8 秒也没有位置改善。因此继续等待或直接追加现有细调循环，没有证据能让本条旧 pose1 收敛。未来可把同一 contract 的已用 1 步、原始 SDK 原点、剩余时间、弱轴保持及响应增益接入小步算法，但本条记录仍须明确失败，不重置/增加预算、不放宽到位门、不恢复已结束任务。

此报告仅离线数学复核；未改生产代码、参数或节点，未发送动作。完整输入哈希、搜索结果与停稳证据见同目录 report.json。
''', encoding='utf-8')
    print(json.dumps(dict(report=str(out/'report.json'), report_sha256=sha(out/'report.json'),
                         current_residual_m=current_error, guide=guide.tolist(),
                         proposed_step=trim.tolist(), main_result=report['outcome']), ensure_ascii=False))


if __name__ == '__main__':
    main()
