#!/usr/bin/env python3
"""Independent static review of the four frozen GEN54 production changes."""
import ast
from collections import Counter
import datetime
import difflib
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

W = Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
C = W / '.runtime/gen54_integrated_candidate'
B = W / '.runtime/gen54_baseline_source'
V = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z/verification')
OUT = V / 'gen54_geometry_candidate_independent_review_r1'
BASE = 'src/alicia_flexible_grasp_supervisor/'
GRASP = BASE + 'src/alicia_flexible_grasp/grasp/'
PRODUCTION = {
    GRASP + 'gripper_geometry.py': 'f0fa2df7513e57b5958dfd9db049ceac7b87734e96c8a55703584accce90272a',
    GRASP + 'tabletop_geometry_candidates.py': '357ca653aace057c71c3ef7d47f9956ce82f7502658428421989a4580083886f',
    GRASP + 'contact_height_placement.py': '7330126a09e532035af982a27d4cc31d39f5bdd36fe7c214c3ec24fd521e61d4',
    BASE + 'scripts/remote_grasp6d_node.py': '37573742d6be92f53f59d781ecebfd239e99b1106f672584afb5921e9abc6948',
}
OLD_TESTS = {
    BASE + 'tests/test_gripper_geometry.py': '4fe13a1336b611b4417ea2944ad1020d052b7586d994ed829f562415116dd06b',
    BASE + 'tests/test_remote_grasp6d_streaming.py': 'b6b4c8004caadc7826969568bd81a5964d65b87299d5836898cd72ea4f1d9c77',
    BASE + 'tests/test_contact_height_window_proposals.py': 'd8ed14b42ccb3a1a8681e7791490097143f7a14d97ca7bb392fbf3f4e4b52201',
}


def sha(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()


def ref(p):
    return dict(path=str(p), sha256=sha(p), bytes=p.stat().st_size)


def functions(tree):
    out = {}
    def walk(nodes, parent=''):
        for node in nodes:
            if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
                out[parent + node.name] = node
            elif isinstance(node, ast.ClassDef):
                walk(node.body, parent + node.name + '.')
    walk(tree.body)
    return out


def main():
    assert Path('/media/zhuyupei/SanStand').is_mount()
    assert not OUT.exists()
    baseline_path = B / 'runner_manifest.json'
    assert sha(baseline_path) == 'e0ea9529ff6afdff8a6794578f1e64d996cdb124ee1adf3e6732e38e44984748'
    original = json.loads(baseline_path.read_text())['source_sha256']
    assert len(original) == 1069
    actual = {p: sha(C / p) for p in original}
    changed = {p: h for p, h in actual.items() if h != original[p]}
    assert changed == dict(PRODUCTION, **OLD_TESTS)
    assert all(sha(B / p) == h for p, h in original.items())
    assert all(sha(W / p) == original[p] for p in PRODUCTION), 'production changed during review'
    references = {p: ref(C / p) for p in changed}
    ast_changes = {}
    diffs = {}
    for p in PRODUCTION:
        old, new = (B / p).read_text(), (C / p).read_text()
        old_tree, new_tree = ast.parse(old), ast.parse(new)
        compile(new_tree, str(C / p), 'exec')  # syntax only, never execute/import
        a, b = functions(old_tree), functions(new_tree)
        ast_changes[p] = sorted(k for k in set(a) | set(b)
            if k not in a or k not in b or ast.dump(a[k]) != ast.dump(b[k]))
        diffs[p] = ''.join(difflib.unified_diff(old.splitlines(True), new.splitlines(True), fromfile='GEN53/' + p, tofile='GEN54/' + p))
    assert ast_changes[GRASP + 'gripper_geometry.py'] == ['_evaluate_physical_candidate']
    assert ast_changes[GRASP + 'tabletop_geometry_candidates.py'] == ['_measured_aperture_directions', 'generate_tabletop_proposals']
    assert ast_changes[GRASP + 'contact_height_placement.py'] == ['contact_height_window_proposals']
    assert ast_changes[BASE + 'scripts/remote_grasp6d_node.py'] == [
        'RemoteGrasp6DNode._contact_height_recipe_parameters',
        'RemoteGrasp6DNode._contact_height_window_retry',
        'RemoteGrasp6DNode._generate_tabletop_candidates']
    height_path = GRASP + 'contact_height_placement.py'
    old_height = functions(ast.parse((B / height_path).read_text()))['contact_height_window_proposals']
    new_height = functions(ast.parse((C / height_path).read_text()))['contact_height_window_proposals']
    old_deltas = next(n.iter.elts for n in ast.walk(old_height)
                      if isinstance(n, ast.For) and isinstance(n.target, ast.Name) and n.target.id == 'delta')
    new_deltas = next(n.value.elts for n in ast.walk(new_height)
                      if isinstance(n, ast.Assign) and isinstance(n.targets[0], ast.Name) and n.targets[0].id == 'deltas')
    assert [ast.dump(x) for x in old_deltas] == [ast.dump(x) for x in new_deltas]
    oldtest_path = BASE + 'tests/test_contact_height_window_proposals.py'
    oldtest, newtest = (B / oldtest_path).read_text(), (C / oldtest_path).read_text()
    needle = "assert runtime['contact_height_window_search']['maximum_proposals'] == 3"
    assert oldtest.count(needle) == 1
    assert oldtest.replace(needle, needle[:-1] + '4') == newtest
    review_path = V / 'gen54_geometry_pipeline_r1/height_cardinality_only_old_test_review_r1.json'
    assert sha(review_path) == '55df9ba96aed22b77be4db213859b41c59cdea3d90b0ffc853a85471109cbf1b'
    prior = V / 'gen54_parallel_runner_compatibility_independent_review_r1/result.json'
    assert sha(prior) == 'e4c3c74c10f31edce7aa4a3e5039e0974281827f1c3a0215183c7a6faff81b81'
    focus_dir = V / 'gen54_geometry_pipeline_r1/height_related_focus_r2'
    focus_path, xml_path = focus_dir / 'result.json', focus_dir / 'pytest.xml'
    assert sha(focus_path) == 'def569074a4b1748421c212f38c644049b47593ebfe7f16b933d4644b642a364'
    assert sha(xml_path) == '250748aa3d6194937bfe20107b01609fbb2e8ce76de7f90f4012fa3172912f76'
    focus = json.loads(focus_path.read_text())
    assert focus['exit_code'] == 0 and focus['frozen_inputs_unchanged']
    assert not focus['boundary_attempts'] and not focus['full_regression']
    assert all(sha(Path(p)) == h for p, h in focus['frozen_inputs'].items())
    assert all(focus['frozen_inputs'][str(C / p)] == h for p, h in PRODUCTION.items())
    root = ET.parse(xml_path).getroot()
    cases = list(root.iter('testcase'))
    assert len(cases) == 115 and not any(list(c) for c in cases)
    per_file = dict(Counter(c.get('classname') for c in cases))
    assert per_file['tests.test_gen54_height_window'] == 16
    extra = {str(p.relative_to(C)): ref(p) for p in sorted((C / (BASE + 'tests')).glob('test_gen54_*.py'))}
    fixtures = {str(p.relative_to(C)): ref(p) for p in sorted((C / (BASE + 'tests/fixtures')).glob('gen54_*'))}
    semantic = V / 'gen54_command_semantics_independent_review_r1/result.json'
    clarification = semantic.parent / 'model_layer_clarification.json'
    assert sha(semantic) == '3844e284f7d711dce743f89ea97f0b3e09b21a7eefd0f39acb7eb4c36c9040d6'
    assert sha(clarification) == '7f54ae73d18edd2e8c534ecc1c8f45340788bd7936892c76e046e5fcf2f1c632'
    result = {
        'kind': 'GEN54_four_production_sources_independent_static_review',
        'created_at_utc': datetime.datetime.now(datetime.timezone.utc).isoformat(),
        'review_passed_for_pinned_candidate': True,
        'review_scope': 'static_production_geometry_four_sources_and_third_exact_old_test_correction',
        'production_sha256': PRODUCTION, 'source_references': references,
        'all_original_1069_sources_checked': True, 'original_changed_paths': sorted(changed),
        'all_other_original_source_bytes_unchanged': True,
        'changed_functions': ast_changes,
        'new_test_source_references': extra, 'new_fixture_source_references': fixtures,
        'findings': [
            {'area': 'open contact semantics', 'finding': 'Pregrasp/approach/grasp remain open and must not use intended closed-contact overlap exemption. The candidate gate now uses endpoint flags F,F,F,T and segment flags F,F,T; carried lift behavior is retained. Strict IK/path/feedback modules, physical dimensions, padding, support clearance and opening limits are unchanged.'},
            {'area': 'measured aperture proposal', 'finding': 'Original directions are measured first. At most four non-recursive midpoints are proposed only across original nearby complementary aperture/bilateral brackets, including the unoriented angle seam. Every added direction receives the same fresh frozen-surface bilateral and contact-bound evaluation. Geometry is relative; no recorded world coordinate or hard-coded 67.5-degree value is used.'},
            {'area': 'projection and ordering', 'finding': 'Negative full-OBB jaw projection margin is not a hard collision gate: finite fingers can clear a longer object. When a refined proposal exists, positive-margin strata are preferred while the existing maximum count, per-stratum priority and round-robin direction diversity remain. This ranks planning proposals only, without reusing a strict result.'},
            {'area': 'height default preservation', 'finding': 'The public helper defaults upper_interior_reserve_m to None. All three original expressions, order, transforms and unchanged-limit postchecks remain: 0.5*(hi-limit), 0.75*(hi-limit), 0.75*(lo+limit). The existing connected interval must already contain the unchanged empirical evidence interval.'},
            {'area': 'fourth height proposal', 'finding': 'Opt-in requires a finite positive real reserve and excludes booleans. It adds at most hi-limit-reserve, only when distinct and above all three original deltas. The production caller supplies the existing 0.0005 m GRIPPER_CONTRACT_TOLERANCE_M as extra interior clearance; it never decreases the hand-eye/evidence limit or padding. Each pose rechecks the same evidence interval. Small windows retain only original proposals.'},
            {'area': 'recipe binding', 'finding': 'The reserve and maximum count both enter the exact request-local parameters dictionary. Recipe matching still binds prepared object, request/generation/target epoch, invalidation/phase/deadline, original plan and all parameters. Matching a recipe rebuilds the selected placement and reruns registered geometry, evidence freezing, plan hashing, height margin, necessary target check and full strict evaluation; it cannot reuse a motion verdict.'},
            {'area': 'deadline cancellation and bounds', 'finding': 'Retry eligibility remains limited to registered unknown-tabletop direct-near-field predicted-command grasp collisions. The existing caller deadline is reused for every proposal and checked before and after strict work; ticket cancellation propagates. Registration plus height shift retains its original maximum bound. Four poses move consistently along the frozen support normal; no source OBB, measured surface or target coordinates are moved.'},
            {'area': 'third old test', 'finding': 'The only additional old-test edit is one literal maximum_proposals assertion 3 to 4 in test_actual_final_admission_rehashes_only_after_new_complete_check. Original identity, limit, one-selected-proposal, geometry and plan-hash assertions remain exact. Prior two exceptions remain unchanged.'},
        ],
        'third_old_test_exception': dict(path=oldtest_path, baseline_sha256=original[oldtest_path],
            candidate_sha256=OLD_TESTS[oldtest_path], review_evidence=ref(review_path),
            exactly_one_literal_cardinality_changed=True),
        'prior_two_old_test_review': ref(prior),
        'existing_focused_validation': dict(result=ref(focus_path), xml=ref(xml_path),
            case_count=115, per_file=per_file, elapsed_sec=focus['elapsed_sec'],
            CPU_user_sec=focus['cpu_user_sec'], CPU_system_sec=focus['cpu_system_sec'],
            boundary_attempts=[], full_regression=False, current_frozen_production_and_test_hashes_match=True,
            earlier_16_case_run_not_used_as_latest_source_proof=True),
        'limitations': [
            'Static review and existing focused tests are not the required complete final regression. Final runner and scope must bind all four production paths, all three exact old-test exceptions, eight CAD dependencies and complete old/supplemental/new case inventories.',
            'Added height and direction candidates still require fresh online feedback, actual IK and the original strict admission; offline virtual curves do not authorize hardware movement or prove physical grip success.',
            'The pre-existing dual-reference policy remains a reference-curve policy, not a complete physical following tube. Current padded-guard residual counterexamples are model-level and must not be mislabeled as executed solid-mesh collision.',
        ],
        'command_semantics_review': ref(semantic), 'model_layer_clarification': ref(clarification),
        'tests_run_by_this_review': False, 'candidate_imported': False, 'source_modified': False,
        'production_or_parameters_deployed': False, 'ROS_or_hardware_accessed': False,
        'temperature_queried_or_values_diagnosed': False, 'joint_disable_or_motion_sent': False,
        'helper': ref(Path(__file__)),
    }
    assert all(sha(C / p) == h for p, h in actual.items())
    assert all(sha(Path(r['path'])) == r['sha256'] for r in list(extra.values()) + list(fixtures.values()))
    OUT.mkdir()
    for p, content in diffs.items():
        (OUT / (Path(p).name + '.diff')).write_text(content)
    (OUT / 'result.json').write_text(json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True) + '\n')
    (OUT / 'summary.txt').write_text('4 个冻结生产文件独立静态审查通过，原 1069 文件中仅 4 源码 + 3 已审测试变更。新增高度默认保持原 3 值，opt-in 第 4 点使用 hi−原 evidence limit−0.5mm extra reserve；不减少原误差、padding、opening 或 support 条件。reserve/max_count 全部绑定 request-local recipe；publication 重建仍重算 geometry/evidence/hash/height/strict，保留原 deadline、取消和注册位移边界。第三旧测试仅单项 cardinality 3→4。当前精确 4 源码与最终 height test 已由既有 115/115、8.8504 秒 focus 覆盖，没有新运行测试或部署。最终完整回归仍由 root 绑定执行；双参考曲线并非物理误差包络，实物抓取未验证。\n')
    print(json.dumps({'result': ref(OUT / 'result.json'), 'summary': ref(OUT / 'summary.txt')}, ensure_ascii=False))


if __name__ == '__main__':
    main()
