#!/usr/bin/env python3
"""Prepare exact reviewed-test/CAD guards only; never run operational helpers."""
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
V = M/'实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z/verification'
PREP = V/'gen54_idle_helpers_preparation_r2/result.json'
OUT = V/'gen54_idle_helpers_preparation_r3'
RUNNER = W/'.runtime/verify_gen54_parallel_offline_20261018.py'
RUNNER_SHA = 'a5f1d17433ad4c478780bc33c7fc777ecb105204981ca3a3f283bfce7a53fa93'
ORIGINAL_COLLECTION = V/'gen54_old_geometry_collect_r1/result.json'
ORIGINAL_COLLECTION_SHA = '8658aaa84ba970a2b93fda982a4527740e2aedef722df7739563b50365fee29e'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def assign(text, name):
    nodes = [node for node in ast.parse(text).body if isinstance(node, ast.Assign)
        and any(isinstance(target, ast.Name) and target.id == name for target in node.targets)]
    assert len(nodes) == 1
    return nodes[0]


def edit(text, old, new):
    assert text.count(old) == 1, old
    return text.replace(old, new)


def main():
    assert M.is_mount() and not os.statvfs(M).f_flag & os.ST_RDONLY
    assert OUT.parent.stat().st_dev == M.stat().st_dev and not OUT.exists()
    assert sha(PREP) == 'ea1e02f68719392d4dd9614d9776685246c12cf68cbb0aa47afaedb39dc5262d'
    assert sha(RUNNER) == RUNNER_SHA and sha(ORIGINAL_COLLECTION) == ORIGINAL_COLLECTION_SHA
    before = json.loads(PREP.read_text())
    assert all(sha(row['path']) == row['sha256'] and sha(row['template']) == row['template_sha256']
        for row in before['files'])
    cad = ast.literal_eval(assign(RUNNER.read_text(), 'SUPPLEMENTAL_CAD_INPUTS').value)
    assert len(cad) == 8 and all(Path(path).suffix.lower() in ('.urdf', '.stl', '.srdf') for path in cad)
    original = json.loads(ORIGINAL_COLLECTION.read_text())
    original_cases = [tuple(case) for case in original['collected_case_identities']]
    assert original['passed'] is True and original['mode'] == 'collect'
    assert original['test_bodies_executed'] is False and len(original_cases) == len(set(original_cases)) == 206
    assert {case[0] for case in original_cases} == set(before['supplemental_existing_tests'])
    texts = {Path(row['path']).name: Path(row['path']).read_text() for row in before['files']}
    constants = (
        '\n# Bind the final exact reviewed old-test rows only after the second-case decision.\n'
        'REVIEWED_OLD_TEST_CHANGES = None\n'
        'SUPPLEMENTAL_CAD_INPUTS = '+repr(cad)+'\n'
        'SUPPLEMENTAL_ORIGINAL_COLLECTION = N/\'verification/gen54_old_geometry_collect_r1/result.json\'\n'
        'SUPPLEMENTAL_ORIGINAL_COLLECTION_SHA = '+repr(ORIGINAL_COLLECTION_SHA)+'\n'
    )
    for name in ('gen54_idle_reload_common_20261018.py', 'install_gen54_validated_candidate_20261018.py'):
        text = texts[name]
        node = assign(text, 'SUPPLEMENTAL_TESTS')
        statement = ast.get_source_segment(text, node)
        texts[name] = edit(text, statement, statement+constants)
    name = 'gen54_idle_reload_common_20261018.py'
    text = texts[name]
    text = edit(text, "    new = manifest['new_tests']\n", """    require(isinstance(REVIEWED_OLD_TEST_CHANGES, list) and REVIEWED_OLD_TEST_CHANGES,
        'GEN54 reviewed old-test exception inventory is not yet bound')
    approved_rows = manifest.get('approved_old_test_changes', [])
    require(approved_rows == REVIEWED_OLD_TEST_CHANGES,
        'GEN54 old-test exceptions differ from the exact reviewed rows')
    approved = {row['path']: row for row in approved_rows}
    require(len(approved) == len(approved_rows), 'duplicate reviewed old-test paths')
    for relative, row in approved.items():
        require(relative in baseline['source_sha256'] and '/tests/test_' in relative and
            row['baseline_sha256'] == baseline['source_sha256'][relative] and
            row['candidate_sha256'] == manifest['source_sha256'][relative],
            'reviewed old-test source binding differs: '+relative)
        evidence = Path(row['review_evidence']['path']).resolve(strict=True)
        require(inside(evidence, N/'verification') and sha(evidence) == row['review_evidence']['sha256'] and
            release['evidence_sha256'].get(str(evidence)) == row['review_evidence']['sha256'],
            'reviewed old-test evidence differs: '+relative)
    require(sha(SUPPLEMENTAL_ORIGINAL_COLLECTION) == SUPPLEMENTAL_ORIGINAL_COLLECTION_SHA,
        'original supplemental geometry inventory changed')
    original_supplement = read(SUPPLEMENTAL_ORIGINAL_COLLECTION)
    original_supplement_cases = [tuple(case) for case in original_supplement['collected_case_identities']]
    require(original_supplement['mode'] == 'collect' and original_supplement['passed'] is True and
        original_supplement['test_bodies_executed'] is False and
        len(original_supplement_cases) == len(set(original_supplement_cases)) == 206,
        'original 206 supplemental case identities are incomplete')
    for relative, digest in SUPPLEMENTAL_CAD_INPUTS.items():
        path = W/relative
        require(path.resolve(strict=True) == path and relative not in manifest['source_sha256'] and
            sha(path) == digest and release['evidence_sha256'].get(str(path)) == digest,
            'readonly CAD dependency differs: '+relative)
    new = manifest['new_tests']
""")
    text = edit(text, "all(row['sha256'] == baseline['source_sha256'][row['path']]\n            for row in supplemental)", "all(row['sha256'] == (approved[row['path']]['candidate_sha256']\n            if row['path'] in approved else baseline['source_sha256'][row['path']])\n            for row in supplemental)")
    text = edit(text, "    expected_cases = list(old_cases)\n", """    require(sorted(tuple(case) for row in supplemental for case in row['case_identities']) ==
        sorted(original_supplement_cases), 'GEN54 changed an original supplemental testcase identity')
    expected_cases = list(old_cases)
""")
    texts[name] = text
    name = 'install_gen54_validated_candidate_20261018.py'
    text = texts[name]
    text = edit(text, "    assert M.is_mount() and args.expected_control_revision == 56\n", """    assert isinstance(REVIEWED_OLD_TEST_CHANGES, list) and REVIEWED_OLD_TEST_CHANGES, 'GEN54 reviewed old-test exception inventory is not yet bound'
    assert M.is_mount() and args.expected_control_revision == 56
""")
    text = edit(text, "    source = manifest['source_sha256']\n", """    source = manifest['source_sha256']
    approved_rows = manifest.get('approved_old_test_changes', [])
    assert approved_rows == REVIEWED_OLD_TEST_CHANGES
    approved = {row['path']: row for row in approved_rows}
    assert len(approved) == len(approved_rows) and not set(approved) & PRODUCTION
    for relative, row in approved.items():
        assert relative in baseline and '/tests/test_' in relative
        assert row['baseline_sha256'] == baseline[relative]
        assert row['candidate_sha256'] == source[relative] != baseline[relative]
        evidence = Path(row['review_evidence']['path']).resolve(strict=True)
        assert V in evidence.parents and sha(evidence) == row['review_evidence']['sha256']
        assert review['dependency_sha256'].get(str(evidence)) == row['review_evidence']['sha256']
        refs[str(evidence)] = row['review_evidence']['sha256']
    assert sha(SUPPLEMENTAL_ORIGINAL_COLLECTION) == SUPPLEMENTAL_ORIGINAL_COLLECTION_SHA
    original_supplement = read(SUPPLEMENTAL_ORIGINAL_COLLECTION)
    original_supplement_cases = [tuple(case) for case in original_supplement['collected_case_identities']]
    assert original_supplement['mode'] == 'collect' and original_supplement['passed'] is True
    assert original_supplement['test_bodies_executed'] is False
    assert len(original_supplement_cases) == len(set(original_supplement_cases)) == 206
    refs[str(SUPPLEMENTAL_ORIGINAL_COLLECTION)] = SUPPLEMENTAL_ORIGINAL_COLLECTION_SHA
    for relative, digest in SUPPLEMENTAL_CAD_INPUTS.items():
        assert relative not in source
        for root in (W, candidate):
            path = root/relative
            assert path.resolve(strict=True) == path and sha(path) == digest
            refs[str(path)] = digest
""")
    text = edit(text, "row['path'] in baseline and row['sha256'] == baseline[row['path']]", "row['path'] in baseline and row['sha256'] == (\n        approved[row['path']]['candidate_sha256'] if row['path'] in approved else baseline[row['path']])")
    text = edit(text, "    extra_rows = supplemental + manifest['new_tests']\n", """    assert sorted(tuple(case) for row in supplemental for case in row['case_identities']) == sorted(original_supplement_cases)
    extra_rows = supplemental + manifest['new_tests']
""")
    text = edit(text, "assert {path for path in baseline if source[path] != baseline[path]} == PRODUCTION", "assert {path for path in baseline if source[path] != baseline[path]} == PRODUCTION | set(approved)")
    text = text.replace('sorted(PRODUCTION | ADDED)', 'sorted(PRODUCTION | set(approved) | ADDED)')
    assert text.count('sorted(PRODUCTION | set(approved) | ADDED)') == 3
    texts[name] = text
    name = 'seal_gen54_release_20261018.py'
    text = texts[name]
    text = edit(text, '    W, N, BASE_SHA, PREVIOUS, PREVIOUS_SHA, HELPERS, SUPPLEMENTAL_TESTS,',
        '    W, N, BASE_SHA, PREVIOUS, PREVIOUS_SHA, HELPERS, SUPPLEMENTAL_TESTS,\n    REVIEWED_OLD_TEST_CHANGES, SUPPLEMENTAL_CAD_INPUTS,\n    SUPPLEMENTAL_ORIGINAL_COLLECTION, SUPPLEMENTAL_ORIGINAL_COLLECTION_SHA,')
    text = edit(text, '    storage(V)\n', """    require(isinstance(REVIEWED_OLD_TEST_CHANGES, list) and REVIEWED_OLD_TEST_CHANGES,
        'GEN54 reviewed old-test exception inventory is not yet bound')
    storage(V)
""")
    text = edit(text, "    supplemental_rows = manifest.get('supplemental_existing_tests', [])\n", """    require(manifest.get('approved_old_test_changes', []) == REVIEWED_OLD_TEST_CHANGES,
        'manifest old-test exceptions differ from the exact reviewed rows')
    supplemental_rows = manifest.get('supplemental_existing_tests', [])
""")
    text = edit(text, "not context['compatibility']['approved_old_test_changes'], 'historical tests changed'", "context['compatibility']['approved_old_test_changes'] ==\n            {row['path']: row for row in REVIEWED_OLD_TEST_CHANGES},\n        'historical tests differ outside the exact reviewed exceptions'")
    text = edit(text, "    require(len(context['old_expected_counts']) == 322 and\n", """    require(sha(SUPPLEMENTAL_ORIGINAL_COLLECTION) == SUPPLEMENTAL_ORIGINAL_COLLECTION_SHA,
        'original supplemental geometry inventory changed')
    original_supplement = read(SUPPLEMENTAL_ORIGINAL_COLLECTION)
    original_supplement_cases = [tuple(case) for case in original_supplement['collected_case_identities']]
    require(original_supplement['mode'] == 'collect' and original_supplement['passed'] is True and
        original_supplement['test_bodies_executed'] is False and
        len(original_supplement_cases) == len(set(original_supplement_cases)) == 206 and
        sorted(tuple(case) for row in supplemental_rows for case in row['case_identities']) ==
            sorted(original_supplement_cases), 'original supplemental testcase identities changed')
    for relative, digest in SUPPLEMENTAL_CAD_INPUTS.items():
        path = W/relative
        require(relative not in context['sources'] and path.resolve(strict=True) == path and
            sha(path) == digest and context['compatibility_dependencies'].get(str(path)) == digest,
            'readonly CAD dependency is not pinned: '+relative)
    require(len(context['old_expected_counts']) == 322 and
""")
    text = edit(text, "    for name, digest in SCOPE_EVIDENCE.items():\n", """    for row in REVIEWED_OLD_TEST_CHANGES:
        evidence = Path(row['review_evidence']['path']).resolve(strict=True)
        require(inside(evidence, V) and sha(evidence) == row['review_evidence']['sha256'] and
            dependencies.get(str(evidence)) == row['review_evidence']['sha256'],
            'independent review did not bind an old-test correction')
    for name, digest in SCOPE_EVIDENCE.items():
""")
    text = edit(text, "    evidence = dict(context['compatibility_dependencies'])\n", """    evidence = dict(context['compatibility_dependencies'])
    evidence[str(SUPPLEMENTAL_ORIGINAL_COLLECTION)] = SUPPLEMENTAL_ORIGINAL_COLLECTION_SHA
""")
    texts[name] = text
    pins = {path: hashlib.sha256(texts[Path(path).name].encode()).hexdigest()
        for path in before['helper_sha256']}
    name = 'finalize_gen54_idle_deployment_20261018.py'
    node = assign(texts[name], 'HELPER_SHA')
    texts[name] = edit(texts[name], ast.get_source_segment(texts[name], node), 'HELPER_SHA = '+repr(pins))
    for name, text in texts.items():
        if name.endswith('.py'): compile(text, name, 'exec')
        elif name.endswith('.launch'): ET.fromstring(text)
    assert sha(RUNNER) == RUNNER_SHA
    OUT.mkdir()
    rows=[]
    for row in before['files']:
        path, template = Path(row['path']), Path(row['template'])
        assert sha(path) == row['sha256']
        path.write_text(texts[path.name])
        digest = sha(path)
        rows.append(dict(row, sha256=digest, previous_draft_sha256=row['sha256']))
        (OUT/path.name).write_bytes(path.read_bytes())
        diff=''.join(difflib.unified_diff(template.read_text().splitlines(True), texts[path.name].splitlines(True), fromfile=str(template), tofile=str(path)))
        (OUT/(path.name+'.diff')).write_text(diff)
    subprocess.run(['bash', '-n', str(W/'.runtime/start_gen54_observation_measurement_consumers_20261018.sh')], check=True)
    assert all(sha(row['template']) == row['template_sha256'] for row in rows)
    result = dict(before, kind='GEN54_idle_helpers_reviewed_exception_CAD_preparation', files=rows,
        helper_sha256=pins, previous_preparation=str(PREP), previous_preparation_sha256=sha(PREP),
        readonly_CAD_inputs=cad, readonly_CAD_dependency_count=8, CAD_data_added_to_code_scope=False,
        reviewed_old_test_changes_bound=False, old_supplemental_case_identities=206,
        old_supplemental_case_identity_source=str(ORIGINAL_COLLECTION),
        old_supplemental_case_identity_source_sha256=ORIGINAL_COLLECTION_SHA,
        pending_bindings=before['pending_bindings']+['common/install REVIEWED_OLD_TEST_CHANGES remains None pending final exact old-test correction decision'],
        changed_since_first_preparation=[row['path'] for row in rows if row['sha256'] != row['previous_draft_sha256']],
        dependency_sha256={**before['dependency_sha256'], str(RUNNER):RUNNER_SHA, str(ORIGINAL_COLLECTION):ORIGINAL_COLLECTION_SHA},
        preparation_helper=str(Path(__file__).resolve()), preparation_helper_sha256=sha(__file__), created_ns=time.time_ns())
    with (OUT/'result.json').open('x') as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2, allow_nan=False); stream.write('\n')
    (OUT/'summary.md').write_text(
        '本版增加受审计旧测试例外及 8 项只读 CAD 依赖的拒绝式验证；未执行运维入口。\n\n'
        '旧测试例外只接受最终 REVIEWED_OLD_TEST_CHANGES 的完整精确路径、旧/新 SHA 和审查证据；其余旧源码与 fixtures 仍保持原字节。installer 仅将明确例外加入写入清单。原 9,010 testcase identities 和四份 supplemental 的原 206 identities 均保留，修改名称、遗漏或替换身份都会拒绝。原 206 收集记录只用于历史身份比较，不代表当前修改后的测试已运行。\n\n'
        '8 项 CAD 数据在候选和原项目各自按固定 SHA 验证，并写入审计依赖；不计入生产源码，不作为 live import fallback。运行后的 release/common 继续验证这些只读数据。\n\n'
        'REVIEWED_OLD_TEST_CHANGES、EXPECTED_NEW_TEST_FILES、SCOPE_EVIDENCE 尚未绑定，安装/封口会在写入前拒绝；候选与 runner 仍需最终独立审查和重新 collect/focus/full。原 GEN53 helpers 与所有旧验证失败记录未改。\n')
    print(json.dumps(dict(output=str(OUT), result_sha256=sha(OUT/'result.json'),
        operational_execution_ready=False, helper_count=len(rows)), ensure_ascii=False))


if __name__ == '__main__': main()
