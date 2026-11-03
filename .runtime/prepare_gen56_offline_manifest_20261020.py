"""Freeze GEN56 owner-confirmed sources and bind collect/focus evidence.

File-only: never executes tests, installs code, starts a service or contacts ROS.
Each phase creates a new external directory and immutable manifest. A changed
source needs a new freeze; an earlier failed result is never rewritten.
"""
import argparse
from collections import Counter
from copy import deepcopy
import difflib
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import time

W = Path(__file__).resolve().parents[1]
D = W/'.runtime/gen56_candidate_r1'
V = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z/verification')
RUNNER = W/'.runtime/verify_gen56_parallel_offline_20261020.py'
RUNNER_SHA = 'bac8feee8fe84431b50598b5a40adf305e8542dabedc5b99c1b5cbec517cc4ee'
BASELINE = W/'.runtime/gen56_baseline_source/runner_manifest.json'
BASELINE_SHA = '75c02bc24f59759ff62dd563350c9a1c198ded6ea0fba669e057c25460cfea2e'
PRODUCTION = frozenset((
    'src/alicia_flexible_grasp_supervisor/scripts/remote_grasp6d_node.py',
    'src/alicia_grasp_modes/scripts/mode_aware_remote_grasp6d.py',
    'src/alicia_grasp_modes/src/alicia_grasp_modes/station_compute_wait_remote.py'))
OWNERS = frozenset(('/root/gen55_astra_regression','/root/gen55_astra_collision'))


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read(path):
    return json.loads(Path(path).read_bytes())


def save(path, value):
    with Path(path).open('x') as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write('\n'); stream.flush(); os.fsync(stream.fileno())


def reference(path, expected=None):
    path = Path(path).resolve(strict=True)
    digest = sha(path)
    assert expected is None or digest == expected
    return dict(path=str(path),sha256=digest)


def runner_module():
    assert sha(RUNNER) == RUNNER_SHA and sha(BASELINE) == BASELINE_SHA
    spec = importlib.util.spec_from_file_location('gen56_manifest_frozen_runner', RUNNER)
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    return module


def output_directory(path, runner):
    output = Path(path).resolve()
    runner.load_serial().storage(output)
    assert V in output.parents and not output.exists()
    output.mkdir()
    return output


def bound_manifest(args, runner):
    reference(args.manifest,args.manifest_sha256)
    value = runner.bind_manifest(args.manifest,args.manifest_sha256)
    assert Path(value['workspace_root']).resolve(strict=True) == D
    assert value['freeze_helper_sha256'] == sha(__file__)
    return deepcopy(value), runner.current_context(runner.load_serial())


def freeze(args, runner):
    spec_ref = reference(args.spec,args.spec_sha256); spec = read(args.spec)
    assert spec['kind'] == 'GEN56_owner_confirmed_freeze_spec'
    confirmations = spec['owner_freeze_confirmations']
    assert len(confirmations) == len(OWNERS) and {r['agent'] for r in confirmations} == OWNERS
    owned = {}
    for row in confirmations:
        assert row['confirmed_frozen'] is True
        assert isinstance(row['message'],str) and row['message'].strip()
        assert type(row['recorded_ns']) is int and row['recorded_ns'] > 0
        assert row['owned_sha256'] and not set(owned)&set(row['owned_sha256'])
        owned.update(row['owned_sha256'])
    baseline = read(BASELINE); original = baseline['source_sha256']
    assert len(original) == 1082
    # Live GEN55 and its frozen baseline are both retained exactly.
    assert all(sha(W/p) == sha(BASELINE.parent/p) == h for p,h in original.items())
    changed = {p for p,h in original.items() if sha(D/p) != h}
    assert changed == PRODUCTION, ('unexpected production delta',sorted(changed^PRODUCTION))
    serial = runner.load_serial()
    new_tests = spec['new_tests']; fixtures = spec.get('new_fixture_sources',[])
    assert new_tests and len(new_tests) == len(set(new_tests))
    assert len(fixtures) == len(set(fixtures))
    assert set(new_tests).isdisjoint(fixtures)
    assert all(any(p.startswith(root+'/test_gen56_') and p.endswith('.py') for root in serial.ROOTS) for p in new_tests)
    actual_new = {str(p.relative_to(D)) for root in serial.ROOTS for p in (D/root).glob('test_gen56_*.py')}
    assert actual_new == set(new_tests), 'new test scope changed or not all owners frozen'
    for path in fixtures:
        relative = Path(path)
        assert any(path.startswith(root+'/fixtures/gen56_') for root in serial.ROOTS)
        assert relative.suffix.lower() not in ('.py','.pyc','.pyo','.so','.sh','.bash')
    additions = set(new_tests+fixtures)
    assert not additions&set(original)
    assert set(owned) == changed|additions
    assert all(not Path(p).is_absolute() and '..' not in Path(p).parts for p in owned)
    assert all(sha(D/p) == h for p,h in owned.items()), 'owner-frozen source changed'
    sources = {p:sha(D/p) for p in original}; sources.update({p:sha(D/p) for p in additions})
    focus = spec['focus_scope']; imports = spec['required_imports_by_test_file']
    assert focus and len(focus) == len(set(focus)) and set(new_tests) <= set(focus)
    assert set(imports) <= set(focus)
    assert all(set(paths) <= PRODUCTION for paths in imports.values())
    assert {p for paths in imports.values() for p in paths} == PRODUCTION
    reviews = [reference(r['path'],r['sha256']) for r in spec.get('additional_review_evidence',[])]
    manifest = dict(kind='GEN56_frozen_offline_candidate',workspace_root=str(D),variant='candidate',
        baseline_manifest=reference(BASELINE,BASELINE_SHA),runner_sha256=RUNNER_SHA,
        allowed_production_paths=sorted(PRODUCTION),production_source_changes=[dict(path=p,
            baseline_sha256=original[p],candidate_sha256=sources[p]) for p in sorted(PRODUCTION)],
        new_production_sources=[],new_tests=[dict(path=p,sha256=sources[p]) for p in new_tests],
        new_fixture_sources=[dict(path=p,sha256=sources[p]) for p in fixtures],
        approved_old_test_changes=[],approved_gen55_test_changes=[],supplemental_existing_tests=[],
        source_sha256=sources,focus_scope=focus,required_changed_sources=sorted(PRODUCTION),
        required_imports_by_test_file=imports,additional_review_evidence=reviews+[spec_ref],
        owner_freeze_spec=spec_ref,full_scope_ready=False,stage='owner_frozen_before_guarded_collection',
        freeze_helper_sha256=sha(__file__),created_ns=time.time_ns(),production_modified=False,deployed=False)
    output = output_directory(args.output,runner)
    diff = ''.join(''.join(difflib.unified_diff((BASELINE.parent/p).read_text().splitlines(True),
        (D/p).read_text().splitlines(True),fromfile='GEN55/'+p,tofile='GEN56/'+p)) for p in sorted(changed))
    (output/'production.diff').write_text(diff)
    save(output/'candidate_manifest.json',manifest)
    runner.bind_manifest(output/'candidate_manifest.json',sha(output/'candidate_manifest.json'))
    context = runner.current_context(serial)
    assert set(focus) <= set(context['scope'])
    save(output/'preparation.json',dict(passed=True,manifest=reference(output/'candidate_manifest.json'),
        source_count=len(sources),baseline_source_count=1082,baseline_case_count=9568,
        changed_existing_sources=sorted(changed),new_test_files=new_tests,new_fixtures=fixtures,
        old_test_bytes_unchanged=True,old_fixture_bytes_unchanged=True,
        new_cases_not_yet_collected=True,test_bodies_executed=False,full_regression_passed=False,
        production_modified=False,dependencies=[spec_ref,reference(RUNNER),reference(__file__)]))
    return output/'candidate_manifest.json'


def validate_collection(path, manifest_ref, manifest, context, runner):
    value = read(path); plan_path = path.parent/'plan.json'; plan = read(plan_path)
    assert value['mode'] == 'collect' and value['passed'] is True
    assert value['scope_complete'] is True and value['source_unchanged'] is True
    assert value['candidate_manifest_sha256'] == manifest_ref['sha256']
    assert Path(value['candidate_manifest_path']).resolve(strict=True) == Path(manifest_ref['path'])
    assert value['runner_sha256'] == RUNNER_SHA and value['source_sha256'] == manifest['source_sha256']
    assert not value['test_bodies_executed'] and not value['full_regression_passed']
    assert not any(value[k] for k in ('external_boundary_attempts','foreign_or_inconsistent_imports',
        'interrupted','launcher_error','offline_child_cleanup_required','residual_offline_process_groups',
        'duplicate_case_identities','missing_case_identities','unexpected_case_identities','missing_worker_results'))
    assert sha(plan_path) == value['plan_sha256'] and plan['mode'] == 'collect'
    assert runner.stable(runner.load_serial(),plan)
    assert value['worker_count'] == plan['worker_count'] == len(value['worker_result_sha256'])
    assert value['worker_returncodes'] == [0]*value['worker_count']
    assert sha(path.parent/'pytest.xml') == value['xml_sha256']
    summary,cases,_ = runner.inventory(path.parent/'pytest.xml')
    assert summary == value['summary'] == dict(tests=0,failures=0,errors=0,skipped=0) and not cases
    collected = []; nodes = []; seen = set()
    for filename,digest in value['worker_result_sha256'].items():
        child_path = Path(filename).resolve(strict=True); child = read(child_path); folder = child_path.parent
        index = child['worker_index']
        assert type(index) is int and index not in seen and 0 <= index < value['worker_count']
        assert child_path == path.parent/('worker_%d'%index)/'result.json' and sha(child_path) == digest
        seen.add(index)
        assert child['passed'] is True and child['scope_complete'] is True and child['source_unchanged'] is True
        assert child['plan_sha256'] == value['plan_sha256'] and child['mode'] == 'collect'
        assert child['returncode'] == 0 and child['direct_child_reaped'] is True
        assert not child['direct_child_cleanup_signals'] and not child['interrupted']
        assert not child['external_boundary_attempts'] and not child['foreign_or_inconsistent_imports']
        assert not child['test_bodies_executed'] and child['test_scope'] == plan['shards'][index]['scope']
        for name,key in [('pytest.xml','xml_sha256'),('pytest.log','log_sha256'),
                ('module_provenance.json','module_provenance_sha256'),('collected_items.json','collection_sha256')]:
            assert sha(folder/name) == child[key]
        expected_bootstrap = {'bootstrap/sitecustomize.py':context['guard_sha256'],
            'bootstrap/gen42_import_audit.py':context['audit_sha256'],
            'bootstrap/gen56_collect_audit.py':hashlib.sha256(runner.COLLECTION_PLUGIN.encode()).hexdigest()}
        assert child['generated_bootstrap_sha256'] == expected_bootstrap
        assert all(sha(folder/p) == h for p,h in expected_bootstrap.items())
        boundary = folder/'boundary_attempts.jsonl'
        assert not boundary.exists() or not boundary.read_text().splitlines()
        items = read(folder/'collected_items.json')
        assert [r['case_identity'] for r in items] == child['collected_case_identities']
        assert [r['nodeid'] for r in items] == child['collected_nodeids']
        collected.extend(child['collected_case_identities']); nodes.extend(child['collected_nodeids'])
    assert collected == value['collected_case_identities'] and nodes == value['collected_nodeids']
    assert len(collected) == len(set(map(tuple,collected))) == len(set(nodes)) == value['collected_case_count']
    assert set(c[0] for c in collected) == {r['path'] for r in manifest['new_tests']}
    return collected


def bind_collection(args, runner):
    manifest,context = bound_manifest(args,runner)
    assert not manifest['full_scope_ready'] and all('case_identities' not in r for r in manifest['new_tests'])
    collection_ref = reference(args.collection,args.collection_sha256)
    rows = validate_collection(Path(collection_ref['path']),reference(args.manifest),manifest,context,runner)
    for row in manifest['new_tests']:
        cases = [c for c in rows if c[0] == row['path']]; assert cases
        row.update(case_identities=cases,expected_count=len(cases),collection_evidence=collection_ref)
    manifest.update(stage='exact_collected_awaiting_focus',full_scope_ready=False,
        precollection_manifest=reference(args.manifest),created_ns=time.time_ns())
    output = output_directory(args.output,runner)
    save(output/'candidate_manifest.json',manifest)
    runner.bind_manifest(output/'candidate_manifest.json',sha(output/'candidate_manifest.json'))
    context = runner.current_context(runner.load_serial())
    old_focus = {p:context['old_expected_counts'][p] for p in manifest['focus_scope'] if p in context['old_expected_counts']}
    save(output/'preparation.json',dict(passed=True,manifest=reference(output/'candidate_manifest.json'),
        baseline_tests=9568,baseline_test_files=336,new_case_count=len(rows),new_case_counts=dict(Counter(c[0] for c in rows)),
        focus_affected_historical_case_count=sum(old_focus.values()),focus_affected_historical_files=old_focus,
        focus_new_case_count=sum(r['expected_count'] for r in manifest['new_tests']),
        full_expected_case_count=len(context['case_identities']),full_expected_files=len(context['scope']),
        full_scope_ready=False,test_bodies_executed=False,full_regression_passed=False,
        full_baseline_not_claimed_entirely_related=True,collection=collection_ref))
    return output/'candidate_manifest.json'


def full_ready(args, runner):
    manifest,context = bound_manifest(args,runner)
    assert manifest['full_scope_ready'] is False
    assert set(context['expected_counts']) == set(context['scope'])
    manifest['focus_evidence'] = reference(args.focus,args.focus_sha256)
    bindings = runner.validate_focus_evidence(manifest,context)
    # The runtime validator independently checks actual imports, worker case
    # unions and all bootstrap bytes; it does not execute tests or ROS.
    common_path = W/'.runtime/gen56_idle_reload_common_20261020.py'
    common_spec = importlib.util.spec_from_file_location('gen56_focus_result_verifier',common_path)
    common = importlib.util.module_from_spec(common_spec); common_spec.loader.exec_module(common)
    _,_,result,evidence = common.validate_result(args.focus,args.manifest,args.manifest_sha256,RUNNER,RUNNER_SHA,'focus')
    assert result['source_sha256'] == manifest['source_sha256']
    manifest.update(full_scope_ready=True,stage='focus_passed_full_ready',
        focus_manifest=reference(args.manifest),created_ns=time.time_ns())
    output = output_directory(args.output,runner)
    save(output/'candidate_manifest.json',manifest)
    runner.bind_manifest(output/'candidate_manifest.json',sha(output/'candidate_manifest.json'))
    final_context = runner.current_context(runner.load_serial())
    runner.validate_focus_evidence(manifest,final_context)
    save(output/'preparation.json',dict(passed=True,manifest=reference(output/'candidate_manifest.json'),
        focus=reference(args.focus),focus_summary=result['summary'],
        focus_evidence_sha256=bindings,independent_result_evidence_sha256=evidence,
        expected_full_cases=len(final_context['case_identities']),expected_full_files=len(final_context['scope']),
        source_count=len(final_context['sources']),source_same_as_focus=True,
        tests_executed_by_this_helper=False,full_regression_passed=False,full_regression_executed=False))
    return output/'candidate_manifest.json'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('phase',choices=('freeze','bind-collection','full-ready'))
    parser.add_argument('--output',type=Path,required=True)
    for key in ('spec','manifest','collection','focus'):
        parser.add_argument('--'+key,type=Path)
        parser.add_argument('--'+key+'-sha256')
    args = parser.parse_args(); runner = runner_module()
    needed = {'freeze':('spec',),'bind-collection':('manifest','collection'),'full-ready':('manifest','focus')}[args.phase]
    assert all(getattr(args,k) is not None and getattr(args,k+'_sha256') for k in needed)
    output = {'freeze':freeze,'bind-collection':bind_collection,'full-ready':full_ready}[args.phase](args,runner)
    print(json.dumps(dict(manifest=str(output),sha256=sha(output),phase=args.phase,
        tests_executed=False,full_regression_passed=False),ensure_ascii=False),flush=True)


if __name__ == '__main__':
    main()
