#!/usr/bin/env python3
"""Summarize sealed gen21 preview evidence without importing ROS or SDK code."""
import argparse
import collections
import hashlib
import json
import math
from pathlib import Path


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--capture', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    root = args.capture
    summary = json.loads((root / 'summary.json').read_text())
    messages = [json.loads(line) for line in (root / 'messages.jsonl').open()]
    gate_hashes = set()
    metrics = []
    for row in messages:
        if row['kind'] in ('gate_audit', 'metrics'):
            message = json.loads(row['message']['data'])
            if row['kind'] == 'gate_audit':
                gate_hashes.add(message['report_sha256'])
            else:
                metrics.append(message)
    records = []
    selection = dict(generation=21, mode='unknown', strategy='observe_return',
                     stamp_ns=1791347266720888853)
    for path in root.glob('audit_*.json'):
        sha = digest(path)
        assert sha == path.stem[len('audit_'):]
        assert sha in gate_hashes, 'saved audit has no exact gate-message reference'
        audit = json.loads(path.read_text())
        assert audit['grasp_mode_selection'] == selection
        profiles = audit['remote_diagnostics']['tabletop_geometry']['adaptive_stage_profiles']
        maximum_tilt = max(float(profile['tilt_deg']) for profile in profiles)
        learned = [row for row in audit['rows']
                   if row['candidate_source'] == 'graspnet' and row['variant_index'] == 0]
        minimum_tilts = [dict(source_index=row['source_index'],
                             minimum_insertion_tilt_deg=math.degrees(math.asin(
                                 min(1.0, abs(float(row['jaw_normal_cos']))))))
                         for row in learned]
        rejected = [row for row in minimum_tilts
                    if row['minimum_insertion_tilt_deg'] > maximum_tilt + 1e-9]
        funnel = audit['pipeline_funnel']
        assert len(rejected) == funnel['rejection_counts']['GRASPNET_STAGE_PROFILE_UNAVAILABLE']
        searches = []
        checked = []
        for index, lineage in enumerate(audit['lineage']):
            search = lineage.get('observation_orientation_search') or {}
            rows = search.get('variant_results') or []
            if not rows:
                continue
            triple = [(row.get('camera_roll_offset_deg'), row.get('camera_tilt_offset_deg'),
                       row.get('camera_target_distance_m')) for row in rows]
            searches.append(dict(
                lineage_index=index, configured=len(rows),
                checked=sum(bool(row.get('checked')) for row in rows),
                skipped_reasons=dict(collections.Counter(row.get('reason') for row in rows
                                                         if not row.get('checked'))),
                roll_tilt_distance_list_sha256=hashlib.sha256(
                    json.dumps(triple, separators=(',', ':')).encode()).hexdigest(),
                equality_scope='only three scalar view descriptors; not pose/context/authority equality',
                moveit_result=lineage.get('moveit')))
            for row in rows:
                if not row.get('checked'):
                    continue
                following = row.get('following_support') or {}
                view = following.get('view_qualification') or {}
                endpoint = view.get('endpoint_support') or {}
                goal = following.get('goal_support') or {}
                checked.append(dict(
                    lineage_index=index,
                    view={key: row.get(key) for key in (
                        'camera_roll_offset_deg', 'camera_tilt_offset_deg',
                        'camera_target_distance_m')},
                    failure_code=row.get('failure_code'), reason=row.get('reason'),
                    failure_location=following.get('failure_location'),
                    path_minimum_depth_m=view.get('minimum_depth_m'),
                    original_minimum_camera_depth_m=view.get('minimum_camera_depth_m'),
                    endpoint_qualifying_quadrants=endpoint.get('qualifying_quadrants'),
                    endpoint_required_quadrants=endpoint.get('minimum_quadrants'),
                    endpoint_quadrant_pixels=endpoint.get('quadrant_pixels'),
                    goal_nominal_support_clearance_m=goal.get('nominal_minimum_support_clearance_m'),
                    goal_counterexample_support_clearance_m=goal.get('counterexample_model_clearance_m'),
                    required_support_clearance_m=goal.get('required_support_clearance_m')))
        matching_metrics = [row for row in metrics
                            if row.get('event') == 'request_completed'
                            and row.get('request_id') == audit['request_id']
                            and row.get('generation') == audit['generation']
                            and (row.get('snapshot_evidence') or {}).get('snapshot_stamp_ns')
                            == audit['snapshot_stamp_ns']]
        records.append(dict(
            request_id=audit['request_id'], audit_path=str(path), audit_sha256=sha,
            exact_gate_reference_verified=True, selection=audit['grasp_mode_selection'],
            stream_generation=audit['generation'], target_epoch=audit['target_epoch'],
            snapshot_stamp_ns=audit['snapshot_stamp_ns'], outcome=audit['outcome'],
            profile_tilts_deg=[row['tilt_deg'] for row in profiles],
            learned_candidate_count=len(learned), computed_infeasible_jaw_count=len(rejected),
            computed_infeasible_jaw_candidates=rejected,
            pipeline_funnel=funnel, observation_searches=searches, checked_views=checked,
            same_request_source_metrics=[dict(
                reference=row.get('audit_reference'),
                exact_saved_audit_hash_match=(row.get('audit_reference') or {}).get('report_sha256') == sha,
                wsl_total_ms=row.get('wsl_total_ms'),
                end_to_end_ms=row.get('end_to_end_ms')) for row in matching_metrics]))
    records.sort(key=lambda row: row['request_id'])
    assert [row['request_id'] for row in records] == [11, 15, 49, 73]
    report = dict(
        schema_version=1, kind='sealed_preview_read_only_failure_analysis',
        capture=str(root), analysis_source_sha256=digest(Path(__file__)),
        source_files={name: digest(root / name) for name in ('summary.json', 'messages.jsonl')},
        limitations=[
            'Only the four successfully copied exact-reference audits are analyzed.',
            'Summary records two oversized and one superseded audit; no replacement was inferred.',
            'Same request/source metrics may name a later audit revision; hash differences remain explicit.',
            'Matching roll/tilt/distance arrays do not prove equal endpoint poses or reusable execution authority.',
            'No ROS, WSL RPC, hardware, configuration, or production code was changed.'],
        capture_summary=summary, audits=records,
        conclusions={
            'early_profile_failure': 'Local learned-jaw insertion-cone incompatibility, not missing WSL profiles. Computed count exactly matches recorded rejection count in every saved audit.',
            'late_blocker': 'Stable tabletop candidates reach observation planning; only 2/175 and 3/175 views were checked, then original shared tilt budget prevents further checks.',
            'actual_checked_failures': 'Near views at about 94 mm fail original goal support-error-box, path camera-depth, or projected measured support coverage checks.',
            'reporting_defect': 'Primary failure is chosen by largest count across stages; earlier learned-candidate rejections can hide later blocking observation-search failures.',
            'not_proven': 'Unchecked views are not proven unreachable; this analysis does not prove any alternative path succeeds.'},
        recommendations=[
            'Keep all existing source, geometry, tracking, collision, camera-depth and time/check budgets.',
            'Use original frozen measured support endpoint evidence to avoid spending expensive strict checks on already inadequate views.',
            'Improve bounded candidate scheduling to cover information-sufficient distance families rather than consuming the budget on near-boundary roll variants.',
            'Any request-local reuse must bind complete frozen input, endpoint pose, seed, controller/model/scene and source context; scalar roll/tilt/distance identity is insufficient.',
            'Report terminal blocking stage and checked-path reasons independently from the aggregate most-common rejection.'])
    args.output.mkdir(parents=True, exist_ok=False)
    target = args.output / 'analysis.json'
    target.write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
    markdown = '''# Gen21 有限预览离线归因

WSL 正常返回候选，`GRASPNET_STAGE_PROFILE_UNAVAILABLE` 来自 ROS 本地夹持方向与插入角约束不相容。
请求 11/15/49/73 均存在 0、11.25、22.5、33.75、45 度的 profile。按保存的夹持轴与支撑面夹角重算，
最小可行插入角超过 45 度的数量为 12/11/12/8，与日志拒绝数逐项一致。

请求 49/73 已有 6 个稳定 tabletop 候选。补观察实际仅检查 2/175、3/175 个视角，随后原有共享预算耗尽。
已检查的约 94 mm 近距离视角分别被桌面误差盒、路径相机工作距离或支撑面覆盖门拒绝。
其他 9 个 lineage 显示 0/175；未检查视角不能归为不可达。

当前错误聚合按数量选主错误，因此早期 profile 拒绝可能盖过后期搜索受阻。
建议在原预算和所有门限不变的前提下，提前使用同源实测支撑预测、改进有限视角调度，
并单列最终阻断阶段。重复搜索复用必须完整绑定位姿、来源、关节起点、模型、场景和控制合同，
不能仅凭相同 roll/tilt/distance 跨候选复用位姿或权限。

四份审计均通过文件 SHA 与 gate_audit 精确引用核验，均属于 selection 21。
唯一 preview 是旧计划撤销消息；本轮没有匹配的有效预览。两份过大审计和一份被替换审计未分析。
逐帧来源、四份 SHA、实际已检查拒绝和分析限制见 analysis.json。
'''
    (args.output / 'analysis.md').write_text(markdown)
    manifest = dict(analysis_json_sha256=digest(target),
                    analysis_md_sha256=digest(args.output / 'analysis.md'),
                    analysis_source_sha256=digest(Path(__file__)))
    (args.output / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    print(json.dumps(dict(output=str(args.output), **manifest)))


if __name__ == '__main__':
    main()
