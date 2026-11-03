#!/usr/bin/env python3
"""Bind reviewed file inventory only. Never import/run operational entries."""
import ast
import difflib
import hashlib
import json
import os
from pathlib import Path
import subprocess
import time
import xml.etree.ElementTree as ET

W = Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
M = Path('/media/zhuyupei/SanStand')
V = M / '实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z/verification'
PREP = V / 'gen54_idle_helpers_preparation_r3/result.json'
OUT = V / 'gen54_idle_helpers_preparation_r4'
REVIEW_OUT = V / 'gen54_parallel_runner_final_independent_review_r1'
RUNNER = W / '.runtime/verify_gen54_parallel_offline_20261018.py'
RUNNER_SHA = '2b9023c285ce3aadaaad37c0f734b96a567963dc384838ed3319c3858d24b5f5'
PRIOR_RUNNER = V / 'gen54_parallel_runner_compatibility_independent_review_r1/runner.py'
MANIFEST = V / 'gen54_final_candidate_preparation_r1/candidate_manifest.json'
MANIFEST_SHA = '9091b12cae1aec3fe43a579721096199fb122b9b560f8333705bb88bbf0f1f82'
GEOMETRY_REVIEW = V / 'gen54_geometry_candidate_independent_review_r1/result.json'
GEOMETRY_SHA = 'd847d08bf3fc651c441114875cab72e008d1bd1b8ebf1abb5447042352af7a59'


def sha(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def ref(p):
    return dict(path=str(p), sha256=sha(p), bytes=Path(p).stat().st_size)


def assignment(text, name):
    rows = [n for n in ast.parse(text).body if isinstance(n, ast.Assign)
            and any(isinstance(t, ast.Name) and t.id == name for t in n.targets)]
    assert len(rows) == 1
    return rows[0]


def replace(text, old, new):
    assert text.count(old) == 1, old
    return text.replace(old, new)


def assign(text, name, expression):
    node = assignment(text, name)
    return replace(text, ast.get_source_segment(text, node), name + ' = ' + expression)


def literal(node):
    if isinstance(node, ast.Constant): return node.value
    if isinstance(node, ast.List): return [literal(x) for x in node.elts]
    if isinstance(node, ast.Dict): return {literal(k): literal(v) for k, v in zip(node.keys, node.values)}
    if isinstance(node, ast.Name) and node.id == 'V': return V
    if isinstance(node, ast.BinOp) and isinstance(node.op, ast.Div):
        left, right = literal(node.left), literal(node.right)
        assert isinstance(left, Path) and isinstance(right, str)
        return left / right
    if isinstance(node, ast.Call) and isinstance(node.func, ast.Name):
        if node.func.id == 'dict':
            assert not node.args and all(k.arg is not None for k in node.keywords)
            return {k.arg: literal(k.value) for k in node.keywords}
        if node.func.id == 'str':
            assert len(node.args) == 1 and not node.keywords
            return str(literal(node.args[0]))
    raise AssertionError(ast.dump(node))


def save(path, data):
    with path.open('x') as f:
        json.dump(data, f, ensure_ascii=False, indent=2, allow_nan=False)
        f.write('\n')


def main():
    assert M.is_mount() and not os.statvfs(M).f_flag & os.ST_RDONLY
    assert not OUT.exists() and not REVIEW_OUT.exists()
    assert sha(PREP) == 'f0497d1293d5652e758e546e3750b173bcc0d50f3d8188ff1d0612cbf9410a72'
    assert sha(RUNNER) == RUNNER_SHA
    assert sha(PRIOR_RUNNER) == '5f74e80fe66301f0829ee0a11cc01481735f799a24b6239ab65053e98add3b6a'
    assert sha(MANIFEST) == MANIFEST_SHA and sha(GEOMETRY_REVIEW) == GEOMETRY_SHA
    before = json.loads(PREP.read_text()); manifest = json.loads(MANIFEST.read_text())
    reviewed = json.loads(GEOMETRY_REVIEW.read_text())
    source = manifest['source_sha256']; candidate = Path(manifest['workspace_root'])
    assert candidate == W / '.runtime/gen54_release_candidate_r1'
    assert len(source) == 1075 and not manifest['full_scope_ready']
    assert len(manifest['new_tests']) == len(manifest['new_fixture_sources']) == 3
    assert all('expected_count' not in x for x in manifest['new_tests'])
    assert all(sha(candidate / p) == h for p, h in source.items())
    prod = reviewed['production_sha256']
    assert set(prod) == set(manifest['allowed_production_paths']) and len(prod) == 4
    assert all(source[p] == h for p, h in prod.items())
    for category in ('new_test_source_references', 'new_fixture_source_references'):
        assert all(source[p] == r['sha256'] for p, r in reviewed[category].items())
    runner_text, previous_runner_text = RUNNER.read_text(), PRIOR_RUNNER.read_text()
    exact_rows = literal(assignment(runner_text, 'REVIEWED_OLD_TEST_CHANGES').value)
    prior_rows = literal(assignment(previous_runner_text, 'REVIEWED_OLD_TEST_CHANGES').value)
    assert len(exact_rows) == 3 and exact_rows[:2] == prior_rows
    assert exact_rows == manifest['approved_old_test_changes']
    third = reviewed['third_old_test_exception']
    assert all(exact_rows[2][k] == third[k] for k in ('path', 'baseline_sha256', 'candidate_sha256'))
    assert exact_rows[2]['review_evidence']['sha256'] == third['review_evidence']['sha256']
    # AST-normalize only the extra reviewed constant. No runtime function or guard may change.
    rt, pt = ast.parse(runner_text), ast.parse(previous_runner_text)
    rt_node = next(n for n in rt.body if isinstance(n, ast.Assign)
                   and isinstance(n.targets[0], ast.Name) and n.targets[0].id == 'REVIEWED_OLD_TEST_CHANGES')
    pt_node = next(n for n in pt.body if isinstance(n, ast.Assign)
                   and isinstance(n.targets[0], ast.Name) and n.targets[0].id == 'REVIEWED_OLD_TEST_CHANGES')
    rt_node.value = pt_node.value
    assert ast.dump(rt) == ast.dump(pt)
    cad = ast.literal_eval(assignment(runner_text, 'SUPPLEMENTAL_CAD_INPUTS').value)
    assert len(cad) == 8
    assert all(sha(candidate / p) == h and sha(W / p) == h for p, h in cad.items())
    for row in exact_rows:
        assert sha(Path(row['review_evidence']['path'])) == row['review_evidence']['sha256']
        assert source[row['path']] == row['candidate_sha256']
    assert all(sha(row['path']) == row['sha256'] and sha(row['template']) == row['template_sha256']
               for row in before['files'])
    texts = {Path(row['path']).name: Path(row['path']).read_text() for row in before['files']}
    for name in ('gen54_idle_reload_common_20261018.py', 'install_gen54_validated_candidate_20261018.py'):
        texts[name] = assign(texts[name], 'REVIEWED_OLD_TEST_CHANGES', repr(exact_rows))
        texts[name] = replace(texts[name],
            '# Bind the final exact reviewed old-test rows only after the second-case decision.',
            '# Exact three reviewed old-test updates; all other original test bytes stay fixed.')
    prod_expression = '{' + ', '.join(repr(p) for p in sorted(prod)) + '}'
    installer = 'install_gen54_validated_candidate_20261018.py'
    sealer = 'seal_gen54_release_20261018.py'
    texts[installer] = assign(texts[installer], 'PRODUCTION', prod_expression)
    texts[sealer] = assign(texts[sealer], 'PRODUCTION_SOURCES', 'frozenset(' + repr(tuple(sorted(prod))) + ')')
    new_test_paths = tuple(sorted(row['path'] for row in manifest['new_tests']))
    for name in (installer, sealer):
        texts[name] = assign(texts[name], 'EXPECTED_NEW_TEST_FILES', '3')
        texts[name] = replace(texts[name], 'EXPECTED_NEW_TEST_FILES = 3',
            'EXPECTED_NEW_TEST_FILES = 3\nEXPECTED_NEW_TEST_PATHS = frozenset(' + repr(new_test_paths) + ')')
    texts[installer] = replace(texts[installer], "assert candidate == W/'.runtime/gen54_integrated_candidate'",
                              "assert candidate == W/'.runtime/gen54_release_candidate_r1'")
    texts[installer] = replace(texts[installer],
        "    assert len(manifest['new_tests']) == EXPECTED_NEW_TEST_FILES\n",
        "    assert len(manifest['new_tests']) == EXPECTED_NEW_TEST_FILES\n    assert {row['path'] for row in manifest['new_tests']} == EXPECTED_NEW_TEST_PATHS\n")
    texts[sealer] = replace(texts[sealer], '        and len(new_rows) == EXPECTED_NEW_TEST_FILES\n',
        "        and len(new_rows) == EXPECTED_NEW_TEST_FILES\n        and {row['path'] for row in new_rows} == EXPECTED_NEW_TEST_PATHS\n")
    texts[installer] = replace(texts[installer], '# Bind only after the final collected candidate inventory is frozen.',
        '# Exact three new file paths are frozen; case counts still require actual collection evidence.')
    texts[sealer] = replace(texts[sealer], '# Fail closed until final GEN54 original-failure review and collected inventory are pinned.',
        '# File paths are frozen. SCOPE_EVIDENCE and exact collected case inventory remain required.')
    pins = {path: hashlib.sha256(texts[Path(path).name].encode()).hexdigest()
            for path in before['helper_sha256']}
    finalizer = 'finalize_gen54_idle_deployment_20261018.py'
    texts[finalizer] = assign(texts[finalizer], 'HELPER_SHA', repr(pins))
    for name, content in texts.items():
        if name.endswith('.py'): compile(content, name, 'exec')
        elif name.endswith('.launch'): ET.fromstring(content)
    assert ast.literal_eval(assignment(texts[sealer], 'SCOPE_EVIDENCE').value) is None
    assert sha(RUNNER) == RUNNER_SHA and all(sha(candidate / p) == h for p, h in source.items())
    REVIEW_OUT.mkdir()
    runner_review = dict(kind='GEN54_final_runner_third_exact_old_test_independent_review', passed=True,
        runner=ref(RUNNER), previous_runner=ref(PRIOR_RUNNER),
        only_AST_change='append the exact third reviewed old-test row',
        third_old_test_exception=exact_rows[2], other_runtime_functions_and_guards_unchanged=True,
        candidate_manifest=ref(MANIFEST), source_count=1075, production_source_sha256=prod,
        candidate_source_sha256=source, release_candidate_bytes_match_reviewed_integrated_candidate=True,
        CAD_dependencies_count=8, full_scope_ready=False, actual_new_case_counts_not_bound=True,
        geometry_independent_review=ref(GEOMETRY_REVIEW),
        runner_imported_or_executed=False, tests_run=False, deployment_performed=False)
    save(REVIEW_OUT / 'result.json', runner_review)
    (REVIEW_OUT / 'runner.py').write_text(runner_text)
    (REVIEW_OUT / 'runner.diff').write_text(''.join(difflib.unified_diff(previous_runner_text.splitlines(True), runner_text.splitlines(True), fromfile='two_reviewed_cases', tofile='three_reviewed_cases')))
    OUT.mkdir()
    rows = []
    for row in before['files']:
        path = Path(row['path']); template = Path(row['template'])
        assert sha(path) == row['sha256']
        previous_text = path.read_text()
        path.write_text(texts[path.name])
        rows.append(dict(row, sha256=sha(path), previous_draft_sha256=row['sha256']))
        (OUT / path.name).write_bytes(path.read_bytes())
        (OUT / (path.name + '.diff')).write_text(''.join(difflib.unified_diff(template.read_text().splitlines(True), texts[path.name].splitlines(True), fromfile=str(template), tofile=str(path))))
        (OUT / (path.name + '.revision.diff')).write_text(''.join(difflib.unified_diff(previous_text.splitlines(True), texts[path.name].splitlines(True), fromfile='previous_r3', tofile='prepared_r4')))
    subprocess.run(['bash', '-n', str(W / '.runtime/start_gen54_observation_measurement_consumers_20261018.sh')], check=True)
    assert all(sha(row['template']) == row['template_sha256'] for row in rows)
    dependencies = dict(before['dependency_sha256'])
    dependencies.update({str(RUNNER): RUNNER_SHA, str(MANIFEST): MANIFEST_SHA,
                         str(GEOMETRY_REVIEW): GEOMETRY_SHA,
                         str(REVIEW_OUT / 'result.json'): sha(REVIEW_OUT / 'result.json')})
    dependencies.update({row['review_evidence']['path']: row['review_evidence']['sha256'] for row in exact_rows})
    result = dict(before, kind='GEN54_idle_helpers_final_file_inventory_preparation',
        files=rows, helper_sha256=pins, production_allowlist=sorted(prod),
        reviewed_old_test_changes_bound=True, reviewed_old_test_changes=exact_rows,
        expected_new_test_files=3, expected_new_test_paths=list(new_test_paths),
        frozen_release_candidate=str(candidate), frozen_release_candidate_source_count=1075,
        final_test_counts_bound=False, new_case_count_guessed=False,
        pending_bindings=[
            'seal SCOPE_EVIDENCE remains None until root binds exact original-failure/review receipts',
            'actual final collection identities/counts, full-scope candidate manifest, focused result and static combined review are still required',
            'installed-workspace full regression and exact full source/file/case/worker counts remain required before sealing/reload',
        ],
        previous_preparation=str(PREP), previous_preparation_sha256=sha(PREP),
        dependency_sha256=dependencies,
        changed_since_previous_preparation=[row['path'] for row in rows if row['sha256'] != row['previous_draft_sha256']],
        preparation_helper=str(Path(__file__).resolve()), preparation_helper_sha256=sha(__file__),
        helper_main_or_import_executed=False, operational_execution_ready=False,
        created_ns=time.time_ns())
    save(OUT / 'result.json', result)
    (OUT / 'summary.txt').write_text('已将 GEN54 运维草稿绑定第四个已存在 placement 生产文件、3 个精确旧测试例外、3 个新测试路径和独立 release candidate r1。最终 runner 第三行外 AST 完全不变，1075 冻结源码与已审 candidate 精确一致。8 CAD 数据与原 9010+206 身份规则保留。只修改新 GEN54 helper 文件并更新 finalizer common 哈希；未导入或执行 installer/reload/preflight/finalizer、未访问 ROS 或硬件。新用例数量不使用预估值；SCOPE_EVIDENCE、实际 collect/full 证据仍待 root 绑定。\n')
    print(json.dumps(dict(preparation=ref(OUT / 'result.json'), runner_review=ref(REVIEW_OUT / 'result.json'),
                          operational_execution_ready=False), ensure_ascii=False))


if __name__ == '__main__':
    main()
