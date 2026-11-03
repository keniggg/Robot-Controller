#!/usr/bin/env python3
"""Freeze one evidence constant after actual collection; no operational calls."""
import ast
from collections import Counter
import difflib
import hashlib
import json
from pathlib import Path
import time

W = Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
M = Path('/media/zhuyupei/SanStand')
V = M / '实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z/verification'
PREVIOUS = V / 'gen54_idle_helpers_preparation_r4/result.json'
OUT = V / 'gen54_idle_helpers_preparation_r5'
MANIFEST = V / 'gen54_final_collected_r1/candidate_manifest.json'
COLLECT = V / 'gen54_final_collect_r1/result.json'
FOCUS = V / 'gen54_final_focus_r1/result.json'
SCOPE = {
    'gen54_collision_geometry_r1/original_cases_geometry.json': '157ccff23a02a4875a49b7ad622e63e583755b26b7f2cd7fc2c1ce1642a4e2a6',
    'gen54_collision_geometry_r1/independent_review_final_r1.json': '4a05ba17dc55de306a56162d91817d19df85029f49ef15ded72de9ea2251782f',
    'gen54_collision_geometry_r1/release_candidate_binding_r1.json': '77e8a16231742b0c816efd2348f6c11978b052ce17173bdc8c18d26f20084054',
    'gen54_geometry_candidate_independent_review_r1/result.json': 'd847d08bf3fc651c441114875cab72e008d1bd1b8ebf1abb5447042352af7a59',
    'gen54_command_semantics_independent_review_r1/result.json': '3844e284f7d711dce743f89ea97f0b3e09b21a7eefd0f39acb7eb4c36c9040d6',
    'gen54_command_semantics_independent_review_r1/model_layer_clarification.json': '7f54ae73d18edd2e8c534ecc1c8f45340788bd7936892c76e046e5fcf2f1c632',
}


def sha(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def main():
    assert M.is_mount() and not OUT.exists()
    assert sha(PREVIOUS) == '07a0831c7ac8931703b85667843776f3b4703144fc2800e9c8fbf9b182646b39'
    pins = {str(MANIFEST): '78e39ce6fb2d56c95d50d672003d54f018994d48eeb222ee7c2ffd92eb3881d3',
            str(COLLECT): '1b62b02b8b3bc2bd1791c236925dbab4d176adfb6bfc21a2a561e24cd3384093',
            str(FOCUS): '949fcb61186171c55d532d283d417a6c7115ddc8fd34a1526f832a0336d7fb32'}
    pins.update({str(V / p): h for p, h in SCOPE.items()})
    assert all(sha(p) == h for p, h in pins.items())
    previous = json.loads(PREVIOUS.read_text()); manifest = json.loads(MANIFEST.read_text())
    collect = json.loads(COLLECT.read_text()); focus = json.loads(FOCUS.read_text())
    source = manifest['source_sha256']
    assert manifest['full_scope_ready'] and len(source) == 1075
    assert collect['passed'] and collect['collected_case_count'] == 245 and not collect['test_bodies_executed']
    assert collect['source_sha256'] == focus['source_sha256'] == source
    assert focus['passed'] and focus['summary'] == dict(tests=836, failures=0, errors=0, skipped=0)
    assert focus['source_unchanged'] and not focus['external_boundary_attempts'] and not focus['foreign_or_inconsistent_imports']
    assert all(focus['loaded_changed_sources_global'].values())
    new, supplement = manifest['new_tests'], manifest['supplemental_existing_tests']
    assert [r['expected_count'] for r in new] == [17, 16, 6]
    assert sum(r['expected_count'] for r in supplement) == 206
    original = json.loads((V / 'gen54_old_geometry_collect_r1/result.json').read_text())['collected_case_identities']
    assert Counter(map(tuple, original)) == Counter(tuple(c) for r in supplement for c in r['case_identities'])
    assert 322 + len(new) + len(supplement) == 329
    assert 9010 + sum(r['expected_count'] for r in new + supplement) == 9255
    assert all(sha(row['path']) == row['sha256'] and sha(row['template']) == row['template_sha256']
               for row in previous['files'])
    sealer = W / '.runtime/seal_gen54_release_20261018.py'
    before = sealer.read_text()
    assert before.count('SCOPE_EVIDENCE = None') == 1
    after = before.replace('SCOPE_EVIDENCE = None', 'SCOPE_EVIDENCE = ' + repr(SCOPE))
    compile(after, str(sealer), 'exec')
    bt, at = ast.parse(before), ast.parse(after)
    for tree in (bt, at):
        node = next(n for n in tree.body if isinstance(n, ast.Assign)
                    and isinstance(n.targets[0], ast.Name) and n.targets[0].id == 'SCOPE_EVIDENCE')
        node.value = ast.Constant(value=None)
    assert ast.dump(bt) == ast.dump(at)
    OUT.mkdir()
    sealer.write_text(after)
    files = []
    for row in previous['files']:
        path = Path(row['path']); template = Path(row['template'])
        files.append(dict(row, sha256=sha(path), previous_draft_sha256=row['sha256']))
        (OUT / path.name).write_bytes(path.read_bytes())
        (OUT / (path.name + '.diff')).write_text(''.join(difflib.unified_diff(template.read_text().splitlines(True), path.read_text().splitlines(True), fromfile=str(template), tofile=str(path))))
    revision = ''.join(difflib.unified_diff(before.splitlines(True), after.splitlines(True), fromfile='r4', tofile='r5'))
    (OUT / 'sealer_scope_only.diff').write_text(revision)
    result = dict(previous, kind='GEN54_idle_helpers_frozen_scope_preparation', files=files,
        previous_preparation=str(PREVIOUS), previous_preparation_sha256=sha(PREVIOUS),
        SCOPE_EVIDENCE=SCOPE, final_test_counts_bound=True,
        actual_scope=dict(source_count=1075, test_files=329, tests=9255,
            historical_tests=9010, supplemental_tests=206, new_tests=39,
            new_cases_by_file={r['path']: r['expected_count'] for r in new}),
        collection_reference=dict(path=str(COLLECT), sha256=sha(COLLECT)),
        focus_reference=dict(path=str(FOCUS), sha256=sha(FOCUS), tests=836, elapsed_sec=focus['elapsed_sec']),
        only_helper_code_change='SCOPE_EVIDENCE literal in the sealer; all other helpers are byte-identical to r4',
        changed_since_previous_preparation=[str(sealer)],
        pending_bindings=['root independent helper review receipt and combined static review',
                          'root-installed-workspace full regression result and exact pins before seal/reload'],
        dependency_sha256={**previous['dependency_sha256'], **pins},
        preparation_helper=str(Path(__file__).resolve()), preparation_helper_sha256=sha(__file__),
        operational_execution_ready=False, created_ns=time.time_ns())
    with (OUT / 'result.json').open('x') as f:
        json.dump(result, f, ensure_ascii=False, indent=2, allow_nan=False); f.write('\n')
    print(json.dumps(dict(result=str(OUT / 'result.json'), result_sha256=sha(OUT / 'result.json'),
                          sealer_sha256=sha(sealer), diff_sha256=sha(OUT / 'sealer_scope_only.diff'))))


if __name__ == '__main__':
    main()
