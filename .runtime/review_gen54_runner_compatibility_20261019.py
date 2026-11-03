#!/usr/bin/env python3
"""Static independent audit of the two exact test exceptions and eight CAD inputs."""
import ast
from collections import Counter
from copy import deepcopy
import datetime
import difflib
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

W = Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
E = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
V = E / 'ros_log/full_latest_gen41_restart_20261018T011222Z/verification'
C = W / '.runtime/gen54_integrated_candidate'
B = W / '.runtime/gen54_baseline_source'
OUT = V / 'gen54_parallel_runner_compatibility_independent_review_r1'
RUNNER = W / '.runtime/verify_gen54_parallel_offline_20261018.py'
RUNNER_SHA = '5f74e80fe66301f0829ee0a11cc01481735f799a24b6239ab65053e98add3b6a'
PREVIOUS = V / 'gen54_runner_cad_revision_r1/runner_before.py'
PREVIOUS_SHA = 'f98f940c6f23c3d24205845a00f2c6feaa8ce418b27c9a95b3f6117126ed5c18'
EXPECTED_FUNCTIONS = {
    'src/alicia_flexible_grasp_supervisor/tests/test_gripper_geometry.py': 'test_opening_fit_clearance_allows_near_limit_narrow_side_without_shrinking_collision_contract',
    'src/alicia_flexible_grasp_supervisor/tests/test_remote_grasp6d_streaming.py': 'test_configured_resolution_preserves_recorded_bilateral_contact',
}


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def ref(path):
    return {'path': str(path), 'sha256': sha(path), 'bytes': path.stat().st_size}


def constant(node):
    """Evaluate only the small literal whitelist used by reviewed constants."""
    if isinstance(node, ast.Constant):
        return node.value
    if isinstance(node, ast.List):
        return [constant(x) for x in node.elts]
    if isinstance(node, ast.Dict):
        return {constant(k): constant(v) for k, v in zip(node.keys, node.values)}
    if isinstance(node, ast.Name) and node.id == 'V':
        return V
    if isinstance(node, ast.BinOp) and isinstance(node.op, ast.Div):
        left, right = constant(node.left), constant(node.right)
        assert isinstance(left, Path) and isinstance(right, str)
        return left / right
    if isinstance(node, ast.Call) and isinstance(node.func, ast.Name):
        if node.func.id == 'dict':
            assert not node.args and all(k.arg is not None for k in node.keywords)
            return {k.arg: constant(k.value) for k in node.keywords}
        if node.func.id == 'str':
            assert len(node.args) == 1 and not node.keywords
            return str(constant(node.args[0]))
    raise AssertionError(ast.dump(node))


def module_map(tree):
    out = {}
    for index, node in enumerate(tree.body):
        if isinstance(node, (ast.FunctionDef, ast.ClassDef)):
            key = node.name
        elif isinstance(node, ast.Assign) and len(node.targets) == 1 and isinstance(node.targets[0], ast.Name):
            key = node.targets[0].id
        else:
            key = ast.dump(node, include_attributes=False)
        assert key not in out, (index, key)
        out[key] = ast.dump(node, include_attributes=False)
    return out


def main():
    assert Path('/media/zhuyupei/SanStand').is_mount()
    assert not OUT.exists()
    assert sha(RUNNER) == RUNNER_SHA and sha(PREVIOUS) == PREVIOUS_SHA
    before, after = PREVIOUS.read_text(), RUNNER.read_text()
    bt, at = ast.parse(before), ast.parse(after)
    bm, am = module_map(bt), module_map(at)
    changed = sorted(k for k in set(bm) | set(am) if bm.get(k) != am.get(k))
    assert changed == ['REVIEWED_OLD_TEST_CHANGES', 'SUPPLEMENTAL_CAD_INPUTS', 'current_context']
    selected = {node.targets[0].id: constant(node.value) for node in at.body
                if isinstance(node, ast.Assign) and isinstance(node.targets[0], ast.Name)
                and node.targets[0].id in ('REVIEWED_OLD_TEST_CHANGES', 'SUPPLEMENTAL_CAD_INPUTS')}
    exceptions = selected['REVIEWED_OLD_TEST_CHANGES']
    cad = selected['SUPPLEMENTAL_CAD_INPUTS']
    assert len(exceptions) == 2 and {x['path'] for x in exceptions} == set(EXPECTED_FUNCTIONS)
    assert len(cad) == 8
    baseline_path = B / 'runner_manifest.json'
    assert sha(baseline_path) == 'e0ea9529ff6afdff8a6794578f1e64d996cdb124ee1adf3e6732e38e44984748'
    baseline = json.loads(baseline_path.read_text())
    original = baseline['source_sha256']
    assert len(original) == baseline['source_count'] == 1069
    history_path = V / 'gen53_full_parallel_r1/result.json'
    xml_path = history_path.parent / 'pytest.xml'
    assert sha(history_path) == '13a428ac36f300a18a80adb00603c09f6ee70c90342d6a847afeac125883ff8a'
    assert sha(xml_path) == '3ce8c6f4e6de0be68745042e113f2e727f7b8f3cdc862e4728dca0523b8ebb78'
    history = json.loads(history_path.read_text())
    assert history['passed'] and history['source_sha256'] == original
    cases = [(c.get('file', ''), c.get('classname', ''), c.get('name', ''))
             for c in ET.parse(xml_path).getroot().iter('testcase')]
    assert len(cases) == len(set(cases)) == 9010
    assert dict(Counter(c[0] for c in cases)) == baseline['test_case_counts_by_file']
    assert len(baseline['test_case_counts_by_file']) == 322
    collection_path = V / 'gen54_old_geometry_collect_r1/result.json'
    assert sha(collection_path) == '8658aaa84ba970a2b93fda982a4527740e2aedef722df7739563b50365fee29e'
    collection = json.loads(collection_path.read_text())
    original_supplements = collection['collected_case_identities']
    assert collection['passed'] and not collection['test_bodies_executed']
    assert len(original_supplements) == len(set(map(tuple, original_supplements))) == 206
    assert not set(c[0] for c in original_supplements) & set(c[0] for c in cases)
    test_rows, pinned = [], {}
    test_snapshots = {}
    for row in exceptions:
        name = row['path']; expected_name = EXPECTED_FUNCTIONS[name]
        left, right = B / name, C / name
        assert sha(left) == row['baseline_sha256'] == original[name]
        assert sha(right) == row['candidate_sha256'] != original[name]
        evidence_path = Path(row['review_evidence']['path'])
        assert sha(evidence_path) == row['review_evidence']['sha256']
        evidence = json.loads(evidence_path.read_text())
        assert evidence['kind'] == 'GEN54_explicit_old_expectation_correction'
        for item in evidence.get('evidence', []):
            assert sha(Path(item['path'])) == item['sha256']
            pinned[item['path']] = item['sha256']
        old, new = ast.parse(left.read_text()), ast.parse(right.read_text())
        oldmap, newmap = module_map(old), module_map(new)
        changed_nodes = [k for k in set(oldmap) | set(newmap) if oldmap.get(k) != newmap.get(k)]
        assert changed_nodes == [expected_name]
        oldfun = next(x for x in old.body if isinstance(x, ast.FunctionDef) and x.name == expected_name)
        newfun = next(x for x in new.body if isinstance(x, ast.FunctionDef) and x.name == expected_name)
        for node in (oldfun, newfun):
            node.body = [ast.Pass()]
        assert ast.dump(old, include_attributes=False) == ast.dump(new, include_attributes=False)
        # Identity/decorators/parametrization/imports/setup outside that body are unchanged.
        test_rows.append(dict(row, changed_ast_function=expected_name,
            exactly_one_existing_body_changed=True, case_identity_and_decorators_unchanged=True,
            original_case_present=any(c[0] == name and c[2].split('[')[0] == expected_name
                                     for c in cases + list(map(tuple, original_supplements))),
            baseline_reference=ref(left), candidate_reference=ref(right), evidence_reference=ref(evidence_path)))
        assert test_rows[-1]['original_case_present']
        pinned[str(left)] = sha(left); pinned[str(right)] = sha(right)
        pinned[str(evidence_path)] = sha(evidence_path)
        test_snapshots[Path(name).name] = right.read_bytes()
    fixture_name = 'src/alicia_flexible_grasp_supervisor/tests/fixtures/measured_contact_rgbd_20260910.npz'
    fixture_digest = '719e94665d21d480b996b279e291ce8cc882a287137cad7c18e0b74098d22416'
    assert original[fixture_name] == sha(B / fixture_name) == sha(C / fixture_name) == fixture_digest
    cad_refs = []
    for name, digest in cad.items():
        assert name not in original
        for root in (W, C):
            path = root / name
            assert path.resolve(strict=True) == path and sha(path) == digest
            cad_refs.append(ref(path)); pinned[str(path)] = digest
    pending = {
        'final_production_scope_not_reviewed_here': 'Parent authorized a fourth existing placement module; its final bytes and helper allowlist are not bound by this review.',
        'possible_additional_old_test_change': 'Any later height-cardinality correction, even in the already listed streaming file, changes the approved hash and requires a new exact review row and runner revision.',
        'fresh_collection_and_full_regression_required': 'Prior collection cannot authorize changed runner/source bytes. New frozen collection must retain the 9010 original and 206 supplemental identities, then execute the complete final scope.',
        'installer_or_sealer_final_scope_bound': False,
        'this_review_is_not_full_regression_or_deployment_approval': True,
    }
    result = {
        'kind': 'GEN54_independent_runner_compatibility_revision_review',
        'created_at_utc': datetime.datetime.now(datetime.timezone.utc).isoformat(),
        'review_passed_for_exact_runner_revision': True,
        'review_scope': 'static_runner_two_exact_old_test_body_exceptions_and_eight_readonly_CAD_dependencies',
        'runner': ref(RUNNER), 'previous_reviewed_runner': ref(PREVIOUS),
        'changed_top_level_AST_nodes': changed,
        'all_other_AST_nodes_and_guard_definitions_unchanged': True,
        'old_test_changes': test_rows,
        'old_test_meanings': {
            'near_limit': '49 mm still passes opening fit but must fail fixed padded open-descent geometry; its 48.1 mm neighbor retains a positive full-gate check. No gripper size or padding changes.',
            'recorded': 'All six original RGBD-derived proposals retain positive bilateral/width/support checks and must fail specifically at static_envelope / GRIPPER_SWEEP_COLLISION / left_finger intrusion. Original RGBD, voxel and registration setup remain.',
        },
        'readonly_CAD_inputs': cad_refs,
        'CAD_dependency_count': 8, 'CAD_added_to_code_scope_or_import_fallback': False,
        'original_recorded_fixture': {'baseline': ref(B / fixture_name), 'candidate': ref(C / fixture_name)},
        'preserved_baseline': {'source_count': 1069, 'test_files': 322, 'cases': 9010,
                              'manifest': ref(baseline_path), 'result': ref(history_path), 'xml': ref(xml_path)},
        'supplemental_existing_tests': {'original_collection': ref(collection_path),
            'test_files': dict(Counter(c[0] for c in original_supplements)), 'case_count': 206,
            'original_case_identities': original_supplements,
            'old_collection_only_not_test_execution': True},
        'contract_review': [
            'Approved old-test changes must equal the exact two-row constant, or be empty; each binds original SHA, new SHA and immutable review evidence. Actual candidate source map must equal the expected map, so an omitted exception cannot accept edited bytes.',
            'Supplemental test SHA is checked against expected[path], which incorporates only an exact approved old-test override. All other old tests and fixtures retain their original source hashes.',
            'Eight fixed data dependencies are checked in both live project and candidate without symlink redirection. They enter compatibility_dependencies and repeated context stability checks, not the code source map or Python import allowlist.',
            'The runner still requires exact original XML case identities, frozen new/supplemental collection identities, unique file assignment, independent worker directories and provenance, clean guards, unchanged source/dependencies and full-scope readiness.',
            'No changes to worker cleanup ownership, network/subprocess/hardware guards, resource budgeting, test execution, identity comparison, XML verification or full-regression labeling.',
        ],
        'pending_final_bindings': pending,
        'no_new_test_execution_claimed': True, 'runner_imported_or_executed': False,
        'production_modified': False, 'helpers_modified': False, 'ROS_or_hardware_accessed': False,
        'temperature_queried_or_values_diagnosed': False, 'motion_or_disable_sent': False,
        'deployment_performed': False, 'helper': ref(Path(__file__)),
    }
    assert sha(RUNNER) == RUNNER_SHA and sha(PREVIOUS) == PREVIOUS_SHA
    assert all(sha(Path(path)) == digest for path, digest in pinned.items())
    OUT.mkdir()
    (OUT / 'runner.py').write_text(after)
    (OUT / 'runner.diff').write_text(''.join(difflib.unified_diff(before.splitlines(True), after.splitlines(True), fromfile='previous_reviewed', tofile='exact_two_exception_revision')))
    for name, content in test_snapshots.items():
        (OUT / name).write_bytes(content)
    (OUT / 'result.json').write_text(json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True) + '\n')
    (OUT / 'summary.txt').write_text('精确 runner 5f74e80f… 的两份旧测试例外和八项 CAD 依赖独立静态审查通过。仅 current_context 与两个常量变化，其余 AST / 隔离守卫不变。两份旧测试各只改一个原函数体，原身份、装饰器、录制输入、正向 bilateral/width/support 断言及精确碰撞拒绝保留。CAD 校验两处副本并绑定依赖，不扩代码或 import 范围。原 1069 / 322 / 9010 与补充 206 身份核实；没有重新执行测试。第四生产模块、可能新增高度旧断言例外和最终全回归/installer scope 待另外冻结，本审查不批准部署。\n')
    print(json.dumps({'result': ref(OUT / 'result.json'), 'summary': ref(OUT / 'summary.txt')}, ensure_ascii=False))


if __name__ == '__main__':
    main()
