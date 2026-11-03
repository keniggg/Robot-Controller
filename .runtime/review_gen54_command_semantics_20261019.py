#!/usr/bin/env python3
"""Independent file-only semantics audit. Never imports robot/ROS modules."""
import ast
import datetime
import gzip
import hashlib
import json
import math
from pathlib import Path

W = Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
E = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
V = E / 'ros_log/full_latest_gen41_restart_20261018T011222Z/verification'
OUT = V / 'gen54_command_semantics_independent_review_r1'
ROBOT = 'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot/'
PINNED = {
    'gen54_collision_geometry_r1/registered_ik_continuous_all_existing.json': '421333752aceb5f47a2b3607b060d5ddbc107181f9d053f5c8a0b1a8a1af2779',
    'gen54_collision_geometry_r1/raw_command_geometry_all_existing.json': '5280af78eb82c4123fa8c9f59c9cdbda9665ea64fdc09900b4022440ad89a40e',
    'gen54_collision_geometry_r1/height_joint_window.json': '1e05af3bba2006cc341e5beddad2d3f9cba6b45cf0587db748e1062b52c49cf1',
    'gen54_collision_geometry_r1/height_provenance_residuals.json': '2479224c19e4e724b29e83b66224c9bdb3e943fb350e3f87fd69cc26ec5213a6',
    'gen52_contact_collision_review_r1/child_dual_curve_semantics_review.json': '9248d5072ef6781b5061fa1a09cffc3fc0c0bc899a2cede9839d36c78a6cf257',
    'gen52_contact_collision_review_r1/child_bias_aware_tracking_design.json': 'a81891c8ecc94b36f570efc022c434653ca3ed292f1d1c91739e828396623988',
    'gen52_contact_collision_review_r1/actual_residual_box_counterexamples.json': 'e37d9719c3e0c8d2cdd70f08feef4e28aa91cb7cba0f87d5c6609ef14138e709',
}
SOURCE_NAMES = [
    ROBOT + name + '.py' for name in (
        'contact_linear_compensation', 'contact_sequence_precheck', 'stationary_following',
        'observation_path_guard', 'moveit_planner', 'contracted_trajectory',
        'observation_tracking_contract', 'contact_tracking_contract', 'pregrasp_gateway',
        'observation_preview')
] + [
    'src/alicia_flexible_grasp_supervisor/scripts/motion_gateway_node.py',
    'src/real-arm/alicia_d_driver/src/alicia_d_driver_node.cpp',
    'src/alicia_flexible_grasp_supervisor/tests/fixtures/contact_following_20260925.json',
    'src/alicia_flexible_grasp_supervisor/tests/test_contact_linear_compensation.py',
    'docs/verification/2026-09-25-contact-following-continuity.md',
    'docs/operations/2026-09-30-observe-return-and-contact-precheck.md',
]


def sha(data):
    return hashlib.sha256(data).hexdigest()


def reference(path):
    data = path.read_bytes()
    return {'path': str(path), 'sha256': sha(data), 'bytes': len(data)}


def main():
    assert Path('/media/zhuyupei/SanStand').is_mount(), 'external disk unavailable'
    assert not OUT.exists(), 'immutable audit output already exists'
    evidence = {}
    docs = {}
    for name, expected in PINNED.items():
        path = V / name
        evidence[name] = reference(path)
        assert evidence[name]['sha256'] == expected, name
        docs[name] = json.loads(path.read_text())
    sources = {name: reference(W / name) for name in SOURCE_NAMES}
    raw = docs['gen54_collision_geometry_r1/raw_command_geometry_all_existing.json']
    curves = docs['gen54_collision_geometry_r1/registered_ik_continuous_all_existing.json']
    height = docs['gen54_collision_geometry_r1/height_joint_window.json']
    residuals = docs['gen54_collision_geometry_r1/height_provenance_residuals.json']
    receipt_ref = raw['original_receipt_reference']
    receipt_path = Path(receipt_ref['store_root']) / receipt_ref['relative_path']
    compressed = receipt_path.read_bytes()
    assert sha(compressed) == receipt_ref['gzip_sha256']
    receipt_bytes = gzip.decompress(compressed)
    assert sha(receipt_bytes) == receipt_ref['object_sha256']
    receipt = json.loads(receipt_bytes)
    sample = receipt['following_sample']
    b = [s - a for s, a in zip(sample['sdk_positions_rad'], sample['accepted_positions_rad'])]
    assert max(abs(a - z) for a, z in zip(b, raw['sdk_minus_accepted_bias_rad'])) < 1e-15
    assert sample['measurement_kind'] == 'stationary_sdk_following_not_executed_endpoint'
    assert not sample['certifies_command_response_or_calibration']
    assert not receipt['future_following_verified'] and not receipt['execution_authorized']
    assert len(curves['rows']) == len(raw['rows']) == 24
    assert curves['original_receipt_reference'] == receipt_ref
    for row in curves['rows']:
        assert row['ik_complete'] and len(row['new_offline_nominal_curve']) == 15
        assert row['curves']['nominal']['continuous_ok']
        assert not row['curves']['frozen_bias_command']['continuous_ok']
        assert not row['compensation_derivation']['hardware_response_verified']
        assert max(abs(x-y) for x, y in zip(b, row['compensation_derivation']['sdk_minus_encoder_rad'])) < 1e-14
    matrix_delta = max(row['predicted_encoder_max_matrix_difference'] for row in raw['rows'])
    assert matrix_delta < 1e-14
    collisions = [body['padded_sat']['overlap_m'] for row in raw['rows']
                  for body in row['raw_command']['bodies'] if body['padded_overlap']]
    fixture = json.loads((W / SOURCE_NAMES[-4]).read_text())
    # Use only explicit joint feedback from this archived fixture.
    historical = fixture['approach_samples_before_driver_trim']
    sdk = historical['/alicia_d/sdk_command']['message']
    accepted = historical['/alicia_d/accepted_joint_states']['message']
    measured_b = [x-y for x, y in zip(sdk['position'], accepted['position'][:6])]
    hypothetical_b = fixture['rows'][0]['hypothetical_constant_bias_rad']
    quantum = 2.*math.pi/4096.
    design = docs['gen52_contact_collision_review_r1/child_bias_aware_tracking_design.json']
    history_source_comparison = [dict(path=p, reviewed_sha256=h,
        current_sha256=sha(Path(p).read_bytes()), unchanged=sha(Path(p).read_bytes()) == h)
        for p, h in design['source_sha256'].items()]
    function_lines = {}
    wanted = {'compensate_cartesian_path', '_handle_contact_linear', '_endpoint_following_snapshot',
        'validate_contact_prediction', 'predict_contact_command_path', 'validate_selected_pregrasp_path',
        'stationary_following_error', 'validate_observation_trajectory', '_execute_observation_contract',
        'execute_contracted_trajectory', 'build_contact_tracking_contract', 'commanded_stop_reserve',
        '_contact_tracking_feedback_monitor', '_validate_pregrasp_context'}
    for name in SOURCE_NAMES:
        if name.endswith('.py'):
            for node in ast.walk(ast.parse((W / name).read_text())):
                if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)) and node.name in wanted:
                    function_lines[name + ':' + node.name] = {'line': node.lineno, 'end_line': node.end_lineno}
    contact_text = (W / (ROBOT + 'contact_linear_compensation.py')).read_text()
    guard_text = (W / (ROBOT + 'observation_path_guard.py')).read_text()
    planner_text = (W / (ROBOT + 'moveit_planner.py')).read_text()
    assert 'planner.observation_tracking_contract_required = False' in contact_text
    assert 'self.manipulator.execute(execution_plan, wait=True)' in planner_text
    assert 'support_box = ObservationSupportBox(' in guard_text
    height_pass = [row for row in height['rows'] if row.get('both_nominal_and_command_continuous_pass')]
    assert len(height_pass) == height['valid_both_continuous_count'] == 4
    residual_rows = []
    for row in residuals['rows']:
        probes = row['finite_bias_interpolation_endpoint_probes']
        assert len(probes) == 17 and sum(not x['ok'] for x in probes) == 13
        residual_rows.append({
            'source_index': row['source_index'], 'variant_index': row['variant_index'],
            'angle_deg': row['angle_deg'], 'height_translation_m': row['selected_upshift_m'],
            'joint_bias_interpolation_endpoint_states': len(probes),
            'colliding_interpolation_states': sum(not x['ok'] for x in probes),
            'residual_corner_probes': [{k: group[k] for k in (
                'counts_each_axis', 'total_corners', 'failed_corners', 'first_failure')}
                for group in row['residual_endpoint_probes']],
        })
    result = {
        'kind': 'gen54_independent_command_coordinate_and_execution_contract_review',
        'created_at_utc': datetime.datetime.now(datetime.timezone.utc).isoformat(),
        'review_completed': True,
        'scope': 'read_only_source_history_and_existing_frozen_offline_results',
        'conclusion': 'Raw compensated-command FK is an explicit conservative reference-curve policy, not measured actual motion or a proved physical following envelope. No sign error or duplicated bias application is established. Current evidence does not authorize dropping its collision check. Preserve both original checks while improving candidate geometry within the existing contract.',
        'coordinate_semantics': {
            'b0': 'q_sdk_successfully_transmitted - q_accepted_encoder, from one frozen stationary window',
            'command': 'q_command(t) = q_nominal(t) + b0',
            'conditional_actual': 'If the same offset persists, q_actual(t) = q_command(t) - b0 = q_nominal(t). This is conditional, not measured future motion.',
            'raw_reference': 'FK(q_command) represents the command coordinates and, physically, the zero-following-error counterfactual. Its collision cannot be called an observed or predicted actual collision without an explicit response model.',
            'residual_model': 'Define r(t) = q_command(t) - q_accepted(t) - b0; then q_accepted(t) = q_nominal(t) - r(t). A valid replacement would certify the entire residual/time/stop set around this actual curve.',
            'two_curves_limitation': 'Two continuous reference curves do not certify all intermediate or time-varying following offsets under nonlinear FK. No inspected source derives a future bias-collapse interval from these samples.',
        },
        'original_receipt': {'reference': receipt_ref, 'compressed_reference': reference(receipt_path),
            'binding': receipt['binding'], 'measurement_kind': sample['measurement_kind'],
            'source_stamps': raw['following_sample_source_stamps'], 'opening_m': receipt['opening_m'],
            'bias_rad': b, 'bias_sdk_counts': [round(x/quantum) for x in b],
            'current_or_future_feedback_claimed': False},
        'existing_offline_counterexample': {
            'rows': len(raw['rows']), 'all_nominal_15_waypoint_curves_pass': True,
            'all_raw_command_curves_rejected': True,
            'raw_command_translation_norm_range_m': [min(x['command_translation_delta_norm_m'] for x in raw['rows']), max(x['command_translation_delta_norm_m'] for x in raw['rows'])],
            'absolute_jaw_direction_delta_range_m': [min(abs(x['command_translation_delta_in_nominal_tool_m'][1]) for x in raw['rows']), max(abs(x['command_translation_delta_in_nominal_tool_m'][1]) for x in raw['rows'])],
            'colliding_padded_sat_overlap_range_m': [min(collisions), max(collisions)],
            'max_observed_points_inside_raw_finger': max(body['observed_points_inside_raw_box'] for row in raw['rows'] for body in row['raw_command']['bodies'] if 'finger' in body['body']),
            'conditional_frozen_bias_matrix_difference_max': matrix_delta,
            'scope_limits': ['new offline virtual starts and IK curves', 'historical feedback only', 'not original failed hardware trajectory', 'no actual controller-hold bridge', 'no whole-arm collision proof', 'no physical execution or grip success'],
        },
        'unchanged_strict_feasible_height_evidence': {
            'reference': evidence['gen54_collision_geometry_r1/height_joint_window.json'],
            'rows_passing_both_original_continuous_checks': len(height_pass),
            'rows': [{'source_index': x['source_index'], 'variant_index': x['variant_index'],
                      'angle_deg': x['angle_deg'], 'selected_upshift_m': x['selected_upshift_m'],
                      'nominal_segments': x['curves']['nominal']['continuous']['segments'],
                      'command_segments': x['curves']['raw_command']['continuous']['segments']}
                     for x in height_pass],
            'implication': 'Existing diagnostic evidence supports searching the original legal height interval while retaining both checks; it is not live execution approval, full IK exhaustion, or a frozen production implementation.',
        },
        'freshness_and_execution_findings': [
            'Driver publishes SDK positions only after successful raw-frame transmission, decoded from the sent quantized wire command; sdk_transmitted explicitly does not claim motor arrival. sdk_measured comes from accepted complete parser feedback and retains its source stamp.',
            'Stationary sampling enforces canonical joints, matching model/epoch, source continuity, a shared >=0.3 s window, stationary SDK and accepted span <=1 SDK count, and source age <=0.5 s both at capture and after validation. This proves one local stationary offset only.',
            'Contact planning freezes model, scene/selection, epoch, original deadline, opening and sample; cache use consumes the plan, rechecks fresh SDK/accepted within half a quantum, verifies geometry after retiming and the actual stationary controller hold, and revalidates before submission.',
            'Long contact-linear execution sets observation_tracking_contract_required=False and follows ordinary manipulator.execute(wait=True). Entry sampling, ordinary controller path/goal tolerances and measured final arrival are not an in-motion bias-residual certificate.',
            'The existing contact tracking contract is restricted to bounded quantized microsteps (pose index 2 and <=4 SDK counts); scope expansion alone cannot make it valid for these longer contact curves.',
            'Existing commanded_stop_reserve bounds the specified Noetic command polynomial, not physical servo stopping. A fresh measured endpoint or successful controller action does not cover missed in-motion excursions.',
        ],
        'critical_geometry_interface_gap': {
            'function': 'validate_observation_trajectory',
            'finding': 'joint_error_bounds_rad is consumed by ObservationSupportBox for support-plane clearance only. Target-OBB checks use reference FK plus interpolation displacement; they do not propagate that following-error box to target collisions.',
            'implication': 'Passing a residual vector to this existing parameter cannot certify a target-clear actual tube. Any new policy must cover target fingers/palm and support geometry over residual, phase, quantization, feedback age, detection delay and stopping motion.',
            'original_checks_unchanged': True,
        },
        'current_height_candidate_uncertainty_counterexamples': {
            'reference': evidence['gen54_collision_geometry_r1/height_provenance_residuals.json'],
            'rows': residual_rows,
            'conclusion': 'Concrete current-candidate endpoint witnesses disprove the claim that passing both reference curves covers the interpolated bias set. Some small independent joint residuals also collide. Clear finite corners do not certify their interiors. These are geometric counterexamples, not observations of executed motion.',
            'replacement_or_new_hardware_robustness_certified': False,
        },
        'historical_constant_bias_limit': {
            'fixture_reference': sources[SOURCE_NAMES[-4]], 'fixture_provenance': fixture['provenance'],
            'hypothetical_pregrasp_bias_sdk_counts': [round(x/quantum) for x in hypothetical_b],
            'recorded_post_approach_bias_sdk_counts': [round(x/quantum) for x in measured_b],
            'difference_sdk_counts': [round((x-y)/quantum) for x, y in zip(measured_b, hypothetical_b)],
            'meaning': 'Historical actual SDK/accepted endpoint samples differ from the fixture frozen-bias assumption. This rejects a universal constant-bias guarantee; it supplies no bound for the present candidate or future hardware.',
            'historical_controller_completions': 'Prior documented approach/grasp completion and small FK endpoint errors include operator-confirmed air grasps; they are not calibration or physical grip success evidence.',
            'prior_design_source_comparison': history_source_comparison,
            'old_residual_box_scope': 'GEN51 diagnostic corners at one SDK-count radius include collision witnesses; clear quarter/half-count corners never certified interiors and cannot define a new GEN54 tolerance.',
        },
        'requirements_before_any_future_policy_replacement': [
            'A separately reviewed physical actual-state model and conservative residual set, bound to fresh same-goal SDK/accepted/controller desired feedback, model, epoch, scene, opening and original authority/deadline.',
            'Continuous geometry for all Link6/7/8 target and support states in that set, including the real controller bridge, quantization, phase uncertainty, feedback transport/callback age, monitoring interval and detection delay.',
            'A justified physical stopping/detection reserve and active same-goal in-motion residual checks; ordinary raw desired-minus-actual JTC tolerances are not automatically bias-residual tolerances.',
            'On mismatch/stale feedback/ownership or manual revocation: invalidate the consumed plan, cancel only the owned goal under the existing ownership rule, no automatic rearm/replay/recovery, and require a new fresh plan. No joint disable commands.',
        ],
        'source_references': sources, 'function_lines': function_lines, 'evidence_references': evidence,
        'helper_reference': reference(Path(__file__)),
        'actions': {'production_changed': False, 'tests_run': False, 'new_replay_run': False,
            'ROS_initialized': False, 'hardware_accessed': False, 'temperature_queried_or_values_diagnosed': False,
            'joint_disable_or_motion_published': False, 'deployment_performed': False},
        'execution_authorized': False, 'physical_grip_verified': False,
    }
    # Detect concurrent input mutation before sealing the file-only review.
    assert all(reference(W / p) == r for p, r in sources.items())
    assert all(reference(V / p) == r for p, r in evidence.items())
    OUT.mkdir()
    encoded = json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True) + '\n'
    (OUT / 'result.json').write_text(encoded)
    summary = '''双曲线语义独立审查完成。q_command = q_nominal + (SDK−accepted) 的符号与用途正确；在旧偏差保持不变的条件下，预测 encoder 等于 nominal。原始 command 的 FK 不是实测轨迹，12.8–13.1 mm 是两条参考曲线的几何差，不能报告为实际运动残差或真实碰撞。

现有 raw-command 检查属于明确的保守参考曲线合同，尚未证明是完整跟随包络；双曲线也不覆盖所有中间和时变偏差。长 contact-linear 未启用 bias-aware 运动中监视，入口静止窗口和终点到位不能保证全程偏差恒定，因此不能直接删除 raw-command 检查。现有 joint_error_bounds_rad 仅扩展支撑面检查，不扩展目标 OBB；不能把它当完整目标碰撞误差包络接口。

原 24 条离线曲线均 nominal 过、raw-command 拒；它们是虚拟起点和冻结历史偏差，未执行。新高度窗口诊断已有 4 条通过保留的两条连续检查，支持在原合法几何范围内修正候选，不支持放宽 strict、物理成功或部署结论。当前高度候选的 17 个 bias 插值端点每例有 13 个碰撞；67.5° 的 ±1 count 角点 7/64 碰撞，68° 的 ±0.5 count 角点 3/64 碰撞，直接说明 dual-pass 不覆盖完整偏差范围，清晰角点也不能证明内部。旧样本已出现与冻结偏差假设不同的实际终点，历史控制完成/空抓也不构成物理成功证明。

本审查只读取源码和既有证据、核验 SHA256 与算术，没有导入 ROS、运行回放/测试、修改生产、部署、查询温度或发布运动。\n'''
    (OUT / 'summary.txt').write_text(summary)
    print(json.dumps({'result': reference(OUT / 'result.json'), 'summary': reference(OUT / 'summary.txt')}, ensure_ascii=False))


if __name__ == '__main__':
    main()
