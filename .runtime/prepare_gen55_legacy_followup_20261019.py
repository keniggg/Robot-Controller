"""Freeze the two reviewed GEN55 legacy-test functions; no tests or installation.

The historical installed GEN55 manifest pins the baseline. Only D is checked
for candidate bytes: the live worktree may be running sealed GEN54 meanwhile.
Use the existing bind-collection command after fresh guarded collection.
"""
import argparse
import ast
from copy import deepcopy
import difflib
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import time

W = Path(__file__).resolve().parents[1]
D = W/'.runtime/gen55_candidate_r1'
V = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z/verification')
RUNNER = W/'.runtime/verify_gen55_parallel_offline_20261019.py'
BASE = V/'gen55_source_installation_r1/installed_candidate.json'
BASE_SHA = '38af05487563d1b19c539fa298bfba09b17b166c9c74dce7035419b7ad96deed'
FAILED_FULL = V/'gen55_full_parallel_r1/result.json'
FAILED_FULL_SHA = '0a7369d48ffff3caecdff12742fe6f0051b2c32ea6ec2e717d7c7b16f4c7d828'
FAILED_XML_SHA = 'f9c0aa3d49f8f08b14868c12482ed1a0782060dd7339c72f6e42e549402d9b13'
BACKUPS = V/'gen55_full_legacy_followup_r1/before_sources.json'
BACKUPS_SHA = 'df0a8ffdd6847c456e2c27cfde81895f7500922c8e791fab0c67a160c388462d'
T = 'src/alicia_flexible_grasp_supervisor/tests/'
R = 'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot/'
FOLLOWUP = {
    T+'test_contact_sequence_precheck.py': (
        'test_recorded_collision_is_rejected_during_candidate_check_without_motion', 32),
    T+'test_gen38_contact_replay_staging.py': (
        'test_actual_service_and_sequence_keep_failed_retimed_curve_after_exception_flattening', 10),
}


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read(path):
    return json.loads(Path(path).read_bytes())


def reference(path):
    path = Path(path).resolve(strict=True)
    return dict(path=str(path), sha256=sha(path))


def save(path, value):
    with Path(path).open('x') as stream:
        json.dump(value, stream, ensure_ascii=False, sort_keys=True, indent=2, allow_nan=False)
        stream.write('\n'); stream.flush(); os.fsync(stream.fileno())


def only_function_changed(before, after, name):
    """Keep imports, shared gateway fixtures and every other function byte exact."""
    old_text, new_text = before.read_text(), after.read_text()
    old_tree, new_tree = ast.parse(old_text), ast.parse(new_text)
    old = [n for n in old_tree.body if isinstance(n, ast.FunctionDef) and n.name == name]
    new = [n for n in new_tree.body if isinstance(n, ast.FunctionDef) and n.name == name]
    assert len(old) == len(new) == 1
    old, new = old[0], new[0]
    old_lines, new_lines = old_text.splitlines(keepends=True), new_text.splitlines(keepends=True)
    assert old_lines[:old.lineno-1] == new_lines[:new.lineno-1]
    assert old_lines[old.end_lineno:] == new_lines[new.end_lineno:]
    for field in ('args', 'decorator_list', 'returns', 'type_comment'):
        def dump(value):
            if isinstance(value, ast.AST):
                return ast.dump(value, include_attributes=False)
            if isinstance(value, list):
                return [dump(item) for item in value]
            return value
        assert dump(getattr(old, field)) == dump(getattr(new, field))
    assert ast.dump(old, include_attributes=False) != ast.dump(new, include_attributes=False)
    return ''.join(difflib.unified_diff(old_text.splitlines(True), new_text.splitlines(True),
        fromfile='installed_GEN55/'+str(after.relative_to(D)),
        tofile='reviewed_GEN55/'+str(after.relative_to(D))))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--test-review', type=Path, action='append', required=True)
    args = parser.parse_args()
    spec = importlib.util.spec_from_file_location('gen55_followup_runner_definitions', RUNNER)
    runner = importlib.util.module_from_spec(spec); spec.loader.exec_module(runner)
    serial = runner.load_serial(); output = args.output.resolve(); serial.storage(output)
    assert V in output.parents and not output.exists()
    assert sha(BASE) == BASE_SHA and sha(FAILED_FULL) == FAILED_FULL_SHA and sha(BACKUPS) == BACKUPS_SHA
    baseline, failed, backups = read(BASE), read(FAILED_FULL), read(BACKUPS)['sources']
    sources = baseline['source_sha256']; assert len(sources) == 1082
    assert baseline['runner_sha256'] == failed['runner_sha256'] == 'd095d6e7fe6432104bd2f2441d7284d79c145c738df347ff4634426f91123f2f'
    assert failed['source_sha256'] == sources and failed['candidate_manifest_sha256'] == BASE_SHA
    assert failed['passed'] is False and failed['full_regression_passed'] is False
    assert failed['scope_complete'] and failed['source_unchanged'] and failed['test_scope_files'] == 336
    xml = FAILED_FULL.parent/'pytest.xml'
    assert sha(xml) == failed['xml_sha256'] == FAILED_XML_SHA
    summary, full_cases, _ = runner.inventory(xml)
    assert summary == failed['summary'] == dict(tests=9568, failures=2, errors=0, skipped=0)
    candidate = {path:sha(D/path) for path in sources}
    changed = {path for path in sources if candidate[path] != sources[path]}
    assert changed == set(FOLLOWUP), ('Only the two reviewed test files may change', sorted(changed))
    assert len(args.test_review) == 2
    reviews = [reference(path) for path in args.test_review]
    assert all(V in Path(row['path']).parents for row in reviews)
    rows = [read(row['path']) for row in reviews]
    assert len({row['path'] for row in rows}) == 2 and {row['path'] for row in rows} == changed
    new_reviews, diffs = [], []
    for row, ref in zip(rows, reviews):
        path = row['path']; name, count = FOLLOWUP[path]
        assert runner.GEN55_TEST_REVIEW_CONTRACTS[path] == (sources[path], 1, count)
        assert row['passed'] is True
        assert row['baseline_sha256'] == sources[path] and row['candidate_sha256'] == candidate[path]
        cases = [case for case in full_cases if case[0] == path]
        changed_case = [case for case in cases if case[2] == name]
        assert len(cases) == count and len(changed_case) == 1
        assert row['changed_case_identities'] == changed_case
        assert row['original_case_count'] == count
        assert sorted(map(tuple, row['all_original_case_identities'])) == sorted(map(tuple, cases))
        assert row['unchanged_case_identities_preserved'] is True
        assert row['historical_failures_reclassified'] is False
        old = backups[str(D/path)]; before = Path(old['snapshot']).resolve(strict=True)
        assert sha(before) == old['sha256'] == sources[path]
        diffs.append(only_function_changed(before, D/path, name))
        new_reviews.append(dict(path=path, baseline_sha256=sources[path],
            candidate_sha256=candidate[path], review_evidence=ref))
    manifest = deepcopy(baseline)
    manifest['approved_gen55_test_changes'] += new_reviews
    assert len(manifest['approved_gen55_test_changes']) == 4
    collection_rows = manifest['new_tests'] + manifest['supplemental_existing_tests']
    assert len(collection_rows) == 14 and sum(row['expected_count'] for row in collection_rows) == 558
    for row in collection_rows:
        assert row['sha256'] == candidate[row['path']]
        for key in ('case_identities', 'expected_count', 'collection_evidence'):
            row.pop(key)
    historical_focus = manifest.pop('retained_focus_evidence', None)
    if historical_focus:
        manifest['historical_focus_evidence'] = historical_focus
        manifest['additional_review_evidence'] = [ref for ref in manifest['additional_review_evidence']
            if ref != historical_focus]
    for key in ('precollection_manifest', 'installed_from_manifest', 'installed_from_manifest_sha256'):
        manifest.pop(key, None)
    manifest['focus_scope'] = list(FOLLOWUP)
    for path in FOLLOWUP:
        manifest['required_imports_by_test_file'][path] = [R+name for name in
            ('finger_cad_separation.py', 'observation_path_guard.py', 'contact_sequence_precheck.py')]
    manifest.update(workspace_root=str(D), source_sha256=candidate, full_scope_ready=False,
        stage='isolated_legacy_test_followup_before_guarded_collection', created_ns=time.time_ns(),
        previous_gen55_installed_manifest=reference(BASE), historical_failed_full_result=reference(FAILED_FULL),
        runner_sha256=sha(RUNNER), freeze_helper_sha256=sha(__file__), production_modified=False, deployed=False)
    output.mkdir()
    with (output/'reviewed_changes.diff').open('x') as stream:
        stream.write(''.join(diffs)); stream.flush(); os.fsync(stream.fileno())
    manifest_path = output/'candidate_manifest.json'; save(manifest_path, manifest)
    runner.bind_manifest(manifest_path, sha(manifest_path)); context = runner.current_context(serial)
    assert len(context['scope']) == 336 and set(manifest['focus_scope']) <= set(context['scope'])
    assert sum(context['expected_counts'][path] for path in FOLLOWUP) == 42
    _, gen54_cases, _ = runner.inventory(runner.BALANCE_HISTORY.parent/'pytest.xml')
    assert len(gen54_cases) == 9255 and set(map(tuple, gen54_cases)) <= set(map(tuple, full_cases))
    assert all(sha(D/path) == digest for path,digest in candidate.items())
    assert all(sha(ref['path']) == ref['sha256'] for ref in reviews)
    save(output/'preparation.json', dict(passed=True, manifest=reference(manifest_path),
        previous_installed_manifest=reference(BASE), historical_failed_full_result=reference(FAILED_FULL),
        historical_failure_preserved=True, only_changed_paths=sorted(changed), unchanged_source_count=1080,
        changes_confined_to_reviewed_functions=True, source_count=1082, expected_file_count=336,
        expected_case_count=9568, original_GEN54_cases_retained=9255,
        recollection_files=14, historical_recollection_case_count=558, expected_focus_case_count=42,
        reviewed_changes=reference(output/'reviewed_changes.diff'), reviews=reviews,
        runner=reference(RUNNER), helper=reference(__file__),
        live_worktree_source_not_used_as_candidate_baseline=True,
        tests_executed=False, full_regression_passed=False, production_modified=False, deployed=False))
    print(json.dumps(dict(passed=True, manifest=str(manifest_path),
        manifest_sha256=sha(manifest_path)), ensure_ascii=False))


if __name__ == '__main__':
    main()
