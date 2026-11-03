#!/usr/bin/env python3
"""Offline saved-bag audit. No ROS node, publisher, service, or action client.

Reports commanded/measured geometry and the previously rejected error box.
Nothing here qualifies hardware dynamics or authorizes a new motion.
"""
import argparse
import ast
import hashlib
import json
import math
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import rosbag
from genpy import Time

from alicia_flexible_grasp.grasp.contact_execution import frozen_contact_geometry
from alicia_flexible_grasp.robot.observation_path_guard import (
    FrozenObservationScene, ObservationSupportBox, SerialUrdfFk,
    observation_following_support_bound,
)
from alicia_flexible_grasp.robot.observation_tracking_contract import (
    SDK_HALF_COUNT_RAD, commanded_stop_reserve,
)
from alicia_flexible_grasp.robot.stationary_following import ARM_NAMES, SDK_QUANTUM_RAD

ROOT = Path(__file__).resolve().parents[1]
STORE = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
RECORD = STORE / 'grasp_records/20261006T045405.115795Z_146ce792cb7b4aeaa7263b99d08f1c12'
PARAMETERS = STORE / 'ros_log/grasp_support_fixed_20261006T040304Z/contact_margin_replay/current_runtime_parameters.json'
ROBOT = 'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot/'
SOURCES = {
    'linear_descent': ROBOT + 'contact_linear_compensation.py',
    'endpoint_compensation': ROBOT + 'pregrasp_gateway.py',
    'gateway_tracking_dispatch': 'src/alicia_flexible_grasp_supervisor/scripts/motion_gateway_node.py',
    'tracking_budget': ROBOT + 'observation_tracking_contract.py',
    'support_box': ROBOT + 'observation_path_guard.py',
    'sequence_precheck': ROBOT + 'contact_sequence_precheck.py',
    'endpoint_feedforward': ROBOT + 'pregrasp_feedforward.py',
    'lift_continuity': ROBOT + 'lift_path_continuity.py',
    'lift_gateway': ROBOT + 'lift_linear_gateway.py',
    'lift_action_contract': ROBOT + 'lift_moveit_adapter.py',
}


def sha(path):
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(chunk)
    return h.hexdigest()


def evidence(path, full_hash=True):
    stat = path.stat()
    item = dict(path=str(path), size=stat.st_size, mtime_ns=stat.st_mtime_ns)
    if full_hash:
        item['sha256'] = sha(path)
    return item


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path, required=True)
    args = parser.parse_args()
    output = args.output_dir.resolve()
    if STORE.resolve() not in output.parents:
        raise ValueError('Experimental output must remain under SanStand Robot-Controller')
    if not Path('/media/zhuyupei/SanStand').is_mount():
        raise ValueError('SanStand is not mounted; no system-disk fallback')
    if output.exists():
        raise ValueError('Use a new report directory; do not overwrite existing evidence')

    result_path = RECORD / 'result_summary.json'
    summary = json.loads(result_path.read_text())
    failure = json.loads(summary['failure_reason'].split(': ', 1)[1])
    audit = ast.literal_eval(failure['code'].split('OBSERVATION_PATH_INVALID: ', 1)[1])
    start = audit['start_support']
    assert audit['ok'] is False and audit['failure_location'] == 'start'
    assert failure['steps_completed'] == 0
    params = json.loads(PARAMETERS.read_text())['parameters']
    plans, goals, accepted = [], [], None
    with rosbag.Bag(str(RECORD / 'keyframes.bag'), 'r') as bag:
        for topic, msg, when in bag.read_messages(topics=[
                '/grasp_6d/plan_enriched', '/alicia_controller/follow_joint_trajectory/goal']):
            if topic.endswith('/goal'):
                trajectory = msg.goal.trajectory
                goals.append(dict(
                    stamp_ns=when.to_nsec(), goal_id=msg.goal_id.id,
                    duration_sec=trajectory.points[-1].time_from_start.to_sec(),
                    point_count=len(trajectory.points),
                    joint_names=list(trajectory.joint_names),
                    endpoint_rad=list(trajectory.points[-1].positions),
                    path_position_tolerance_rad={x.name: x.position for x in msg.goal.path_tolerance},
                    goal_position_tolerance_rad={x.name: x.position for x in msg.goal.goal_tolerance}))
            elif msg.plan_id == failure['plan_id']:
                plans.append(msg)
        accepted_second = failure['accepted_stamp_ns'] // 1000000000
        for topic, msg, when in bag.read_messages(
                topics=['/alicia_d/accepted_joint_states'],
                start_time=Time(accepted_second - 1), end_time=Time(accepted_second + 2)):
            if msg.header.stamp.to_nsec() == failure['accepted_stamp_ns']:
                assert accepted is None
                accepted = msg
    assert plans and len(goals) == 4
    assert accepted is not None and accepted.header.frame_id == 'sdk_measured'
    accepted_map = dict(zip(accepted.name, accepted.position))
    assert set(ARM_NAMES + ('right_finger',)).issubset(accepted_map)
    plan = plans[-1]
    scene = FrozenObservationScene.from_contact_plan(plan)
    fk = SerialUrdfFk(params['/robot_description'], ARM_NAMES)
    assert fk.model_sha256 == failure['model_sha256'] == start['robot_description_sha256']
    # Bind the exact seven-axis accepted feedback used by the failed report.
    opening = float(accepted_map['right_finger'])
    gripper = frozen_contact_geometry(
        plan, params['/grasp_6d/remote/gripper_geometry'], opening)['gripper']
    checker = ObservationSupportBox(fk, gripper, opening, scene.normal, scene.offset)
    positions = {
        'accepted_measured': (np.array(failure['measured_counts']) - 2048) * SDK_QUANTUM_RAD,
        'sdk_command_quantized': (np.array(failure['sdk_counts']) - 2048) * SDK_QUANTUM_RAD,
        'controller_hold': np.array(start['goal_joint_positions_rad']),
        'rejected_box_witness': np.array(start['counterexample_joint_positions_rad']),
    }
    assert np.max(np.abs(positions['accepted_measured'] -
                         [accepted_map[name] for name in ARM_NAMES])) < 1e-12
    clearances = {name: float(checker.enclosure(q, np.zeros(6))[1])
                  for name, q in positions.items()}
    assert math.isclose(clearances['rejected_box_witness'],
                        start['counterexample_model_clearance_m'], abs_tol=1e-12)
    assert math.isclose(clearances['accepted_measured'],
                        failure['measured_contact']['minimum_support_clearance_m'], abs_tol=1e-12)
    assert np.max(np.abs(np.array(goals[-1]['endpoint_rad']) - positions['controller_hold'])) < 1e-12
    actual_bounds = np.array(start['joint_error_bounds_rad'])
    witness_delta = positions['rejected_box_witness'] - positions['controller_hold']
    assert np.all(np.abs(witness_delta) <= actual_bounds + 1e-14)
    original_minimum_path = min(
        min(row['path_position_tolerance_rad']) for row in audit['tracking_contract_search'])
    assert original_minimum_path == .035
    stop_reserve = float(actual_bounds[0] - original_minimum_path - SDK_HALF_COUNT_RAD)
    # Diagnostic necessary condition at the same frozen start: ideal zero
    # derivatives, no proposal to shorten stop duration or change tolerances.
    ideal_stop = commanded_stop_reserve(.5, velocity=0., acceleration=0.)
    floor = original_minimum_path + SDK_HALF_COUNT_RAD + ideal_stop
    floor_report = observation_following_support_bound(
        fk, positions['controller_hold'], np.full(6, floor),
        scene.normal, scene.offset, gripper, opening)
    assert floor_report['ok'] is False
    assert 'counterexample_joint_positions_rad' in floor_report

    report = dict(
        created_utc=datetime.now(timezone.utc).isoformat(),
        scope='saved_bag_and_frozen_model_readonly_audit', motion_authorized=False,
        hardware_tracking_or_stopping_certified=False, physical_collision_observed=False,
        real_grasp_success=False, runtime_parameters_were_read_live=False,
        result='contract_discontinuity_reproduced_with_model_counterexample',
        source_evidence=dict(result=evidence(result_path), parameters=evidence(PARAMETERS),
                             bag=evidence(RECORD / 'keyframes.bag'), script=evidence(Path(__file__))),
        source_files={name: evidence(ROOT / relative) for name, relative in SOURCES.items()},
        plan_id=plan.plan_id, source_stamp_ns=scene.source_ns, scene_sha256=scene.digest,
        model_sha256=fk.model_sha256, epoch_ns=failure['epoch_ns'],
        accepted_source_stamp_ns=accepted.header.stamp.to_nsec(),
        support_plane=dict(normal=scene.normal.tolist(), offset_m=scene.offset),
        opening_width_m=opening, required_support_clearance_m=gripper.support_clearance_m,
        measured_position_error_m=failure['position_error_m'],
        measured_contact_height_margin_m=failure['measured_contact']['height_registration_margin']['maximum_symmetric_normal_translation_m'],
        required_height_margin_m=failure['measured_contact']['height_registration_margin']['translation_evidence_limit_m'],
        positions_rad={name: q.tolist() for name, q in positions.items()},
        model_minimum_support_clearance_m=clearances,
        sdk_minus_measured_rad=(positions['sdk_command_quantized']-positions['accepted_measured']).tolist(),
        controller_hold_minus_quantized_sdk_rad=(positions['controller_hold']-positions['sdk_command_quantized']).tolist(),
        action_goals=goals, original_path_audit=audit,
        rejected_budget=dict(path_tolerance_rad=original_minimum_path,
                             sdk_half_count_rad=SDK_HALF_COUNT_RAD,
                             commanded_stop_reserve_rad=stop_reserve,
                             total_error_half_width_rad=actual_bounds.tolist()),
        ideal_zero_command_derivative_diagnostic=floor_report,
        findings=[
            'Descent validates exact measured-nominal and once-offset command curves without following-error boxes; final correction adds the observation tracking box.',
            'The box is centered on the same controller desired pose reached by descent; recorded SDK differs from that pose by less than half a count.',
            'The rejected witness is inside the allowed model box; its reproduced clearance is negative. Actual accepted pose has positive clearance. These statements are consistent.',
            'Zero commanded stopping reserve still leaves a concrete violating start pose under the original minimum path tolerance. Retiming alone cannot make this start qualify.',
            'No second SDK-minus-measured offset was found in this failure path. This does not certify hand-eye, URDF, or actual servo dynamics.',
            'Existing lift pair and contact sequence proofs bind full nominal/shifted geometry; they explicitly do not certify hardware following or stopping swept volumes.',
            'Lift custom contracted action transport is reusable infrastructure, but replacing the final error-box gate with its configured-limit-only contract would weaken the current guarantee.',
            'Stationary endpoint response tests and per-goal tolerance monitoring are not continuous hardware tracking or stopping bounds.',
        ],
        next_requirements=[
            'Agree and implement one explicit contact-phase contract spanning descent and correction; retain unchanged final contact/support/registration gates.',
            'Reuse frozen model/scene/SDK bridge and complete nominal-shifted path binding, not an unbound scalar endpoint result.',
            'Any smaller directional or phase-specific residual envelope needs independent continuous response/latency/stopping evidence across relevant axes, directions, loads and poses; present records do not provide this bound.',
            'If that evidence is unavailable, keep the reported limitation rather than assert static samples establish it or silently remove the error-box check.',
        ])
    output.mkdir(parents=True)
    (output / 'report.json').write_text(json.dumps(report, indent=2, ensure_ascii=False) + '\n')
    lines = [
        '# 末段接触路径合同只读审查', '',
        '结论：下降与最终补偿使用了不同的轨迹合格合同；本轮被拒绝的是补偿起点的条件误差盒，不是已观察到实机穿桌。没有找到可以直接替代此误差盒的已验证动态残差上界。', '',
        '| 同一冻结模型中的姿态 | 指爪最低支撑余量 |', '|---|---:|',
    ]
    for name, value in clearances.items():
        lines.append('| %s | %.6f mm |' % (name, value * 1000.))
    lines += ['', '原最低门为 %.3f mm。即使 commanded stop reserve 理想为零，原最小 path tolerance + SDK 半量子的起点盒仍有 %.6f mm 的反例，因此提慢速度不能单独解决本轮起点失败。' %
              (gripper.support_clearance_m * 1000., floor_report['counterexample_model_clearance_m'] * 1000.), '',
              '下降：`contact_linear_compensation.py` 对 measured nominal 和一次 SDK−accepted 偏移后的整条指令曲线做几何验证，随后关闭 observation tracking contract，走 MoveIt 动作。最终补偿：`pregrasp_gateway.py` 要求 tracking contract，有限搜索 .09/.06/.05/.035 rad 原候选，并添加指令停止余量与 SDK 半量子。最后一次搜索在 start 阶段即失败，未发送该补偿动作。', '',
              '本轮下降 goal 的终点与被拒补偿的 controller hold 在 1e−12 rad 内一致。模型 SHA、支撑面和计划来源一致； SDK−accepted 仅作一次全路径偏置。实际 following 偏差与保守误差盒并非同一对象，不能用一次停稳小偏差替换误差盒。', '',
              '已有可复用结构：`lift_path_continuity.validate_lift_pair` 的完整 nominal/shifted 路径、SDK bridge 和冻结绑定；`lift_moveit_adapter.bound_lift_action_goal` 的合约化发送和配置快照。其明确声明 `hardware_following_swept_volume_certified=false`、`hardware_stopping_swept_volume_certified=false`。`contact_sequence_precheck` 也明确 `future_following_verified=false`。这些是连续指令几何证据，不是实际机械臂连续运动范围保证。', '',
              '建议：统一下降与最终补偿的接触阶段合同，保留原接触、桌面、注册余量门；复用已实现的冻结几何和运输绑定。若需要更紧的方向性残差包络，必须另有连续响应、反馈延迟和停止过程证据。当前静止样本不足以建立该上界，不能通过移除误差盒、任意缩小限值或只新增更早拒绝来声称抓取已修复。', '',
              '报告由只读保存数据生成，无 ROS 控制，无新动作，无温度读取。精确输入 SHA、源文件 SHA、四条已发送动作及反例均在 report.json。', '']
    (output / 'report.md').write_text('\n'.join(lines))
    print(json.dumps(dict(output_dir=str(output), result=report['result'],
                          actual_clearance_m=clearances['accepted_measured'],
                          rejected_witness_m=clearances['rejected_box_witness'],
                          ideal_stop_witness_m=floor_report['counterexample_model_clearance_m']), ensure_ascii=False))


if __name__ == '__main__':
    main()
