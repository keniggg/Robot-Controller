#!/usr/bin/env python3
"""Read-only record diagnosis; no ROS imports, model forgery, or gate mutation."""
import ast
import hashlib
import json
import math
from pathlib import Path
import socket
import time

import numpy as np

W = Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
E = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
C = E / 'ros_log/full_latest_gen29_restart_20261016T000728Z'
N = E / 'ros_log/full_latest_gen22_restart_20261015T003700Z/verification'
R = E / 'grasp_records/20261016T121453.757769Z_87f1feb8ab9847419f85b9ab3d7237fb'
D = C / 'verification/gen35_actual_attempt1_geometry_r1'
PREFIX = 'src/alicia_flexible_grasp_supervisor/'
REL = [PREFIX + p for p in (
    'src/alicia_flexible_grasp/grasp/gripper_geometry.py',
    'src/alicia_flexible_grasp/grasp/tabletop_geometry_candidates.py',
    'src/alicia_flexible_grasp/grasp/contact_execution.py',
    'src/alicia_flexible_grasp/vision/regular_box_inference.py',
    'src/alicia_flexible_grasp/vision/outer_rgb_rectangle.py',
    'scripts/remote_grasp6d_node.py', 'config/grasp_params.yaml')]

def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()

def load(path):
    return json.loads(Path(path).read_text())

external_attempts = []
def forbid(*args, **kwargs):
    external_attempts.append('network')
    raise AssertionError('No network/ROS/RPC permitted')
socket.socket = forbid
socket.create_connection = forbid
socket.getaddrinfo = forbid

assert Path('/media/zhuyupei/SanStand').is_mount()
D.mkdir(exist_ok=False)
started = time.monotonic()
manifest = load(R / 'manifest.json')
assert manifest['state'] == 'complete' and manifest['result'] == 'failure'
assert manifest['recorder_closed'] is True
manifest_files = []
for item in manifest['files']:
    path = R / item['path']
    actual = sha(path)
    assert actual == item['sha256'] and path.stat().st_size == item['size_bytes'], path
    manifest_files.append(dict(path=str(path), sha256=actual, bytes=path.stat().st_size))

release_path = N / 'gen35_release_manifest.json'
release = load(release_path)
source_before = {rel: sha(W / rel) for rel in REL}
assert all(release['source_sha256'][rel] == value for rel, value in source_before.items())
audit_path = R / 'keyframes_audits/1a2bda4538b3516d34934347bace57621290bccc6b808aef779b2bd19d3d0919.json'
a = load(audit_path)
t = a['remote_diagnostics']['tabletop_geometry']
model = t['regular_box_inference']
enclosure = t['regular_box_planning_enclosure']
rows = t['surface_evidence_rejections']
narrow = next(r for r in rows if r['failure_code'] == 'BOX_INFERENCE_PRECISION_VIEW_REQUIRED')
v = narrow['targeted_view']
assert a['grasp_mode_selection']['generation'] == 7
assert a['request_id'] == 28 and a['snapshot_stamp_ns'] == 1792152937641056060
assert model['source_binding_valid'] and model['ok']
assert v['model_sha256'] == model['model_sha256'] == enclosure['model_sha256']
assert t['proposal_count'] == t['materialized_count'] == 0
assert a['moveit_selection']['checked_count'] == 0

g_source = (W / REL[0]).read_text()
tree = ast.parse(g_source)
obb_node = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == '_obb_line_interval')
namespace = {'np': np}
exec(compile(ast.Module(body=[obb_node], type_ignores=[]), str(W / REL[0]), 'exec'), namespace)
obb_interval = namespace['_obb_line_interval']
constant = next(n.value.value for n in tree.body if isinstance(n, ast.Assign)
    and any(isinstance(x, ast.Name) and x.id == 'ANALYTICAL_JAW_CLEARANCE_EACH_SIDE_M' for x in n.targets))
assert constant == .002
fit = (v['required_open_upper_m'] - v['inferred_width_m'] - v['width_uncertainty_m']) / 2.
assert abs(fit - .0005) < 1e-12
edge_component = 2. * v['empirical_edge_uncertainty_m']
offset_component = 2. * v['commanded_center_jaw_offset_m']
yaw_component = v['width_uncertainty_m'] - edge_component - offset_component
assert math.isclose(v['precision_reserve_m'], max(constant, v['width_uncertainty_m']), abs_tol=1e-14)
assert v['contact_position_uncertainty_m'] == v['empirical_edge_uncertainty_m']
assert v['aperture_margin_m'] < constant
center = np.mean(np.asarray(v['target_edges_or_corners_base']), axis=0)
interval = obb_interval(center, np.asarray(v['jaw_axis_base']),
    np.asarray(enclosure['planning_center_base']), np.asarray(enclosure['planning_axes_base']),
    np.asarray(enclosure['planning_size_xyz_m']))
reach = [-interval[0] + fit, interval[1] + fit]
assert max(reach) < .025

old_path = C / 'verification/gen35_recorded_failure_replay_r2/result.json'
old = load(old_path)
assert sha(old_path) == 'bee4bd50779c5ae6d28a1af913f9cab68d422562f6fe5496bb413bc1104eb8e6'
old_input_verification = {path: sha(path) == digest for path, digest in old['input_sha256'].items()}
assert all(old_input_verification.values())
old_source_verification = {path: sha(path) == digest for path, digest in old['source_after'].items()}
assert all(old_source_verification.values())
assert old['passed'] and old['actual_diagnostics']['materialized_count'] == 0
old_reach = old['actual_diagnostics']['proposal_reach_rejections'][0]
assert max(old_reach['negative_reach_m'], old_reach['positive_reach_m']) > .025
assert old['balanced_side_reach_m'] > .025

audit_identities = []
for item in manifest['files']:
    if item['path'].startswith('keyframes_audits/'):
        value = load(R / item['path'])
        audit_identities.append(dict(path=item['path'], sha256=item['sha256'],
            selection=value.get('grasp_mode_selection'), request_id=value.get('request_id'),
            source_stamp_ns=value.get('snapshot_stamp_ns'),
            current_selection=value.get('grasp_mode_selection') == a['grasp_mode_selection'],
            phase=value.get('observe_return', {}).get('phase')))
source_after = {rel: sha(W / rel) for rel in REL}
assert source_before == source_after
result = dict(
    kind='gen35_actual_attempt1_geometry_diagnosis', schema_version=1, passed=True,
    passed_semantics='input/source integrity and stated diagnostic checks passed; actual grasp failed',
    actual_grasp_success=False, production_modified=False, live_ros_contacted=False,
    temperature_queried=False, motion_commands_sent=0, external_boundary_attempts=external_attempts,
    script_sha256=sha(__file__), manifest_sha256=sha(R / 'manifest.json'),
    record_dir=str(R), record_result=manifest['result'], record_failure_reason=manifest['failure_reason'],
    manifest_files_verified=manifest_files,
    release_manifest=dict(path=str(release_path), sha256=sha(release_path)),
    deployment=dict(path=str(C / 'gen35_runtime_deployment.json'), sha256=sha(C / 'gen35_runtime_deployment.json')),
    source_before=source_before, source_after=source_after, source_unchanged=True,
    contact_audit=dict(path=str(audit_path), sha256=sha(audit_path), selection=a['grasp_mode_selection'],
        request_id=a['request_id'], internal_generation=a['generation'], target_epoch=a['target_epoch'],
        source_stamp_ns=a['snapshot_stamp_ns'], observe_return=a['observe_return']),
    all_recorded_audit_identities=audit_identities,
    first_tabletop_rejection=dict(field='remote_diagnostics.tabletop_geometry',
        failure_code=t['failure_code'], failure_reason=t['failure_reason'], proposal_count=t['proposal_count'],
        materialized_count=t['materialized_count'], surface_evidence_rejections=rows,
        proposal_reach_rejections_present='proposal_reach_rejections' in t),
    qualified_model_audit=model, planning_enclosure=enclosure,
    registration=dict(field='multiview_surface.latest_registration', value=a['multiview_surface']['latest_registration']),
    source_snapshot_evidence=a['pipeline_funnel']['snapshot_evidence'],
    learned_candidate_funnel=a['pipeline_funnel'], moveit_selection=a['moveit_selection'],
    rejection_count_semantics='12 learned local rejections + 1 empty tabletop branch summary; not 13 learned candidates',
    precision_formula=dict(
        nominal_m=v['inferred_width_m'], yaw_projection_increase_m=yaw_component,
        twice_empirical_edge_uncertainty_m=edge_component, twice_commanded_center_offset_m=offset_component,
        uncertainty_m=v['width_uncertainty_m'], fit_clearance_each_side_m=fit,
        required_m=v['required_open_upper_m'], aperture_m=v['physical_opening_m'], margin_m=v['aperture_margin_m'],
        original_extra_reserve_m=v['precision_reserve_m'], fixed_clearance_reserve_m=constant,
        same_uncertainty_used_in_required_and_extra_reserve=True,
        independent_current_center_error_measured=False,
        edge_alias='contact_position_uncertainty_m == empirical_edge_uncertainty_m; not independent observations',
        fixed_clearance_semantics='existing fixed software gripper-clearance contract, compared with total aperture margin; distinct from the 50 mm physical maximum',
        model_policy='v1 rgb_edges_on_measured_top_plane keeps extra empirical uncertainty reserve; validated v2 joint-outer policy uses fixed clearance only'),
    limited_counterfactual=dict(
        changed_assumption='remove only repeated empirical U from extra reserve; retain fixed 2 mm clearance, full U in required opening, fit, measured points and full union enclosure',
        production_mutated=False, hypothetical_reserve_m=constant,
        original_aperture_margin_m=v['aperture_margin_m'], original_precision_comparison_still_rejects=True,
        contact_evaluator_semantics='same precision comparison in evaluate_inferred_box_contact remains false; no typed model or success evidence forged',
        actual_obb_function_ast_extracted=True, function_file=REL[0], function_name='_obb_line_interval',
        derived_nominal_center_base=center.tolist(), center_derivation='mean of 8 symmetric model-centered target face corners; not canonical model reconstruction',
        obb_interval_m=list(interval), full_union_finger_reach_m=reach, maximum_side_reach_m=.025,
        early_union_finger_reach_would_pass=True, collision_or_strict_path_passed=None,
        strict_path_tested=False, strict_path_not_tested_reason='unchanged fixed-clearance precision comparison still rejects before materialization; strict-path success cannot be inferred',
        full_pipeline_replayed=False),
    old_gen33_failure_preserved=dict(path=str(old_path), sha256=sha(old_path),
        input_hashes_still_match=old_input_verification, source_hashes_still_match=old_source_verification,
        previous_real_current_generator_replay=old_reach, historical_reach=old['historical_reach'],
        total_required_reach_m=old['total_required_reach_m'], balanced_side_reach_m=old['balanced_side_reach_m'],
        still_rejects=True, repeated_replay=False, exact_historical_model=False),
    limitations=[
        'This diagnoses one sealed record and a bounded arithmetic/pure-OBB counterfactual, not a new full task or strict MoveIt replay.',
        'Canonical current model body is not stored in this audit. No substituted model is claimed to have its historical SHA.',
        'No current-scene observation or robot state is used to rewrite this closed record.',
        'The historical latched generation-6 audit in the record is excluded from generation-7 first-cause attribution.',
        'Contact proposal rejection does not prove physical impossibility. Actual collision/IK/path feasibility remains untested.',
        'v1 versus v2 qualifier selection will be diagnosed separately without modifying thresholds or production.'
    ], wall_sec=time.monotonic()-started)
(D / 'result.json').write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False) + '\n')
md = '''# GEN35 首轮：候选生成拒绝与余量公式

本轮已封口为失败。接触请求 28、选择 generation 7、目标 epoch 7 / g9-t7、源时间 1792152937641056060。登记成功（738 inliers、overlap 0.805677、RMSE 0.792153 mm），盒形合格。第一次导致盒形路线无候选的门是 `BOX_INFERENCE_PRECISION_VIEW_REQUIRED`；proposal=0/materialized=0，尚未进入 union finger-reach 或严格路径检查。12 个学习候选也全部本地拒绝；统计中第 13 个拒绝项是空 tabletop 分支摘要，不是额外候选。

窄边名义宽 40.426775 mm；U=偏航投影增加 1.769263 + 两侧边缘误差 6.397316 + 指令中心偏移 0 = 8.166579 mm。开口需求已为 nominal+U+两侧 fit 0.5 mm = 49.593354 mm，50 mm 剩余 0.406646 mm。随后 v1 额外要求总剩余不小于 max(固定净空 2 mm, 同一 U 8.166579 mm)。因此该额外 U 确实重复使用已计入开口需求的同一经验预算；`contact_position_uncertainty_m` 也是同一 edge 值的别名，不是另一份中心测量误差。

2 mm 是既有软件夹爪净空合同，函数明确对总开口余量比较，不能称为机械硬件极限或本帧独立测得的中心误差。只去掉重复的额外 U，保留该固定净空及所有原几何约束，0.406646 mm 仍小于 2 mm，接触证据仍拒绝；这项有限改动本身不能使本轮继续。

另用未修改的 `_obb_line_interval` 纯函数及完整 union 包络，模型中心由审计中八个对称面角点求均值得到；两侧 finger reach 均为 24.796677 mm，低于 25 mm。这里只证明更下游的早期 reach 门未必再拒绝，不是完整碰撞或路径通过。由于保留的固定净空门仍拒绝，本诊断不绕过它去声明严格路径可行。

旧 GEN33 的真实失败回放仍保留：完整 union + fit 总宽 50.198907 mm，偏置中心一侧 26.630877 mm；即使居中每侧仍 25.099453 mm > 25 mm。其原回放的输入和全部 44 项源码 SHA 已再次核验，没有通过删除测量点或缩包络让旧失败消失。

当前源码、旧证据和本轮全部 12 份 manifest 文件（含封口 bag）已按 SHA/大小核验。无 ROS、RPC、温度查询、运动或生产修改。完整数值、字段路径、源 SHA、审计身份筛选及限制见 `result.json`。v1/v2 分支原因另行只读诊断；本回执不把局部计算当实物抓取成功。
'''
(D / 'diagnosis.md').write_text(md)
receipt = {'result_sha256': sha(D / 'result.json'), 'diagnosis_sha256': sha(D / 'diagnosis.md'),
    'script_sha256': sha(__file__), 'source_unchanged': source_before == source_after,
    'manifest_files_verified': len(manifest_files), 'network_attempts': external_attempts}
(D / 'receipt.json').write_text(json.dumps(receipt, indent=2) + '\n')
print(json.dumps({'directory': str(D), **receipt}, ensure_ascii=False))
