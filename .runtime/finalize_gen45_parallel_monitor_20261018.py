"""Independently verify GEN45 full XML, collection, source bytes and offline PGIDs."""
import argparse
from collections import Counter
import json
import os
from pathlib import Path
import time
import xml.etree.ElementTree as ET

from monitor_gen45_parallel_regression_20261018 import W, V, RUN, OUT, RUNNER, locations, sample, sha

BASELINE = W/'.runtime/gen45_baseline_source/manifest.json'
BASELINE_SHA = '7cb4ed47b1c2f0e43fc3309ef0c4e5fac9c23a50ce37cbb89c52bbb0ffff8d00'
HISTORY = V/'gen44_full_parallel_r2/result.json'
HISTORY_SHA = '65db4b69c47c18853ab9982839a90d68aa82c6f4b083ce6e64fa30d1a3d1d2b1'
HISTORY_XML_SHA = '344d8cb3036a81d84e6dc4bacabe6cf592c63ed2637bf64f2dd77a5a1ca9c79a'
PRODUCTION = {
    'src/alicia_flexible_grasp_supervisor/scripts/grasp_task_node.py',
    'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/grasp/contact_execution.py',
    'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot/pregrasp_gateway.py',
}
SUPPLEMENTS = {'src/alicia_flexible_grasp_supervisor/tests/'+name:count for name,count in (
    ('test_pregrasp_precision_preference.py',23), ('test_held_guide_preference_gateway.py',5),
    ('test_pregrasp_preference_path_rejection.py',17), ('test_gen14_held_approach_precheck.py',14),
    ('test_held_approach_gateway.py',47), ('test_pregrasp_feedforward.py',42))}


def read(path):
    return json.loads(Path(path).read_bytes())


def inventory(path):
    tree = ET.parse(path).getroot(); rows = list(tree.iter('testcase'))
    cases = [(r.get('file',''),r.get('classname',''),r.get('name','')) for r in rows]
    summary = {k:sum(int(s.get(k,0)) for s in tree.iter('testsuite'))
        for k in ('tests','failures','errors','skipped')}
    assert summary['tests'] == len(cases) == len(set(cases))
    for name,tag in (('failures','failure'),('errors','error'),('skipped','skipped')):
        assert summary[name] == sum(len(r.findall(tag)) for r in rows)
    return summary,cases


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run',type=Path,default=RUN)
    parser.add_argument('--output',type=Path,default=OUT)
    args = parser.parse_args(); run,out = locations(args.run,args.output)
    assert out.is_dir() and not (out/'result.json').exists()
    evidence = {}
    def keep(path,digest=None):
        path = Path(path).resolve(); actual = sha(path)
        assert digest is None or actual == digest, str(path)
        evidence[str(path)] = actual
        return read(path)
    p = keep(run/'plan.json'); r = keep(run/'result.json')
    assert all(r.get(k) is True for k in ('passed','full_regression_passed','full_regression_scope',
        'scope_complete','source_unchanged','combined_xml_matches_worker_results'))
    assert evidence[str(run/'plan.json')] == r['plan_sha256']
    assert p['mode'] == 'full' and p['context']['compatibility']['variant'] == 'candidate'
    assert 1 <= p['worker_count'] == r['worker_count'] <= 8
    assert Path(p['runner_path']).resolve() == RUNNER
    assert sha(RUNNER) == p['runner_sha256'] == r['runner_sha256']
    evidence[str(RUNNER)] = p['runner_sha256']
    manifest = keep(p['candidate_manifest_path'],p['candidate_manifest_sha256'])
    assert p['candidate_manifest_sha256'] == r['candidate_manifest_sha256']
    assert manifest['kind'] == 'GEN45_frozen_offline_candidate' and manifest['full_scope_ready'] is True
    assert manifest.get('variant','candidate') == 'candidate' and Path(manifest['workspace_root']).resolve() == W
    baseline = keep(BASELINE,BASELINE_SHA); original = baseline['source_sha256']
    assert len(original) == 1040 and len(baseline['test_scope']) == 290
    assert p['context']['original'] == original
    history = keep(HISTORY,HISTORY_SHA)
    assert Path(p['historical_result']).resolve() == HISTORY and p['historical_result_sha256'] == HISTORY_SHA
    assert history['source_sha256'] == original
    historical_xml = HISTORY.parent/'pytest.xml'; keep_xml = sha(historical_xml)
    assert keep_xml == HISTORY_XML_SHA == p['historical_xml_sha256'] == history['xml_sha256']
    assert Path(p['historical_xml']).resolve() == historical_xml
    evidence[str(historical_xml)] = keep_xml
    old_summary,old_cases = inventory(historical_xml)
    old_counts = dict(Counter(c[0] for c in old_cases))
    assert old_summary == dict(tests=8289,failures=0,errors=0,skipped=0)
    assert set(old_counts) == set(baseline['test_scope']) and len(old_counts) == 290
    sources = manifest['source_sha256']
    assert sources == p['context']['sources'] == r['source_sha256']
    assert len(sources) == r['source_count']
    assert all(not Path(name).is_absolute() and '..' not in Path(name).parts for name in sources)
    assert all(sha(W/name) == digest for name,digest in sources.items())
    changes = {name for name,digest in original.items() if sources[name] != digest}
    assert changes == PRODUCTION == set(manifest['allowed_production_paths'])
    assert not manifest.get('approved_old_test_changes') and not manifest.get('approved_fixture_changes')
    change_rows = manifest['production_source_changes']
    assert len(change_rows) == 3 and {row['path'] for row in change_rows} == changes
    assert all(row['baseline_sha256'] == original[row['path']] and
        row['candidate_sha256'] == sources[row['path']] for row in change_rows)
    supplements = manifest.get('supplemental_existing_tests',[])
    new = manifest['new_tests']; fixtures = manifest.get('new_fixture_sources',[])
    assert {row['path']:row['expected_count'] for row in supplements} == SUPPLEMENTS
    assert len(supplements) == 6 and sum(SUPPLEMENTS.values()) == 148
    new_paths = [row['path'] for row in new]; fixture_paths = [row['path'] for row in fixtures]
    assert new and len(new_paths) == len(set(new_paths)) and len(fixture_paths) == len(set(fixture_paths))
    roots = ('src/alicia_flexible_grasp_supervisor/tests','src/alicia_grasp_modes/tests')
    assert all(any(name.startswith(root+'/test_gen45_') for root in roots) and name.endswith('.py') for name in new_paths)
    assert {str(path.relative_to(W)) for root in roots for path in (W/root).glob('test_gen45_*.py')} == set(new_paths)
    assert all(any(name.startswith(root+'/fixtures/') for root in roots) and
        Path(name).suffix.lower() not in ('.py','.pyc','.pyo','.so','.sh','.bash') for name in fixture_paths)
    added = set(sources)-set(original)
    assert added == set(new_paths+fixture_paths)
    assert all(sources[row['path']] == row['sha256'] for row in new+fixtures)
    assert all(row['path'] in original and row['path'] not in old_counts and
        sources[row['path']] == row['sha256'] == original[row['path']] for row in supplements)
    expected_cases = list(old_cases); extra_counts = {}
    for row in supplements+new:
        identities = list(map(tuple,row['case_identities']))
        assert len(identities) == len(set(identities)) == row['expected_count'] > 0
        assert all(len(c) == 3 and c[0] == row['path'] for c in identities)
        collection = keep(row['collection_evidence']['path'],row['collection_evidence']['sha256'])
        assert collection['passed'] is True and collection['source_unchanged'] is True
        assert collection['mode'] == 'collect' and not collection['test_bodies_executed']
        assert not collection['full_regression_passed'] and not collection['external_boundary_attempts']
        assert not collection['foreign_or_inconsistent_imports']
        assert collection['runner_sha256'] == p['runner_sha256'] and collection['source_sha256'] == sources
        assert sorted(identities) == sorted(tuple(c) for c in collection['collected_case_identities'] if c[0] == row['path'])
        expected_cases.extend(identities); extra_counts[row['path']] = row['expected_count']
    assert len(expected_cases) == len(set(expected_cases)) == 8289+148+sum(row['expected_count'] for row in new)
    expected_scope = baseline['test_scope']+[row['path'] for row in supplements]+new_paths
    assert p['test_scope'] == expected_scope and len(expected_scope) == len(set(expected_scope))
    assert p['expected_file_count'] == r['test_scope_files'] == len(expected_scope)
    assert p['expected_case_count'] == len(expected_cases)
    assert sorted(map(tuple,p['expected_case_identities'])) == sorted(expected_cases)
    evidence[str(run/'pytest.xml')] = sha(run/'pytest.xml')
    assert evidence[str(run/'pytest.xml')] == r['xml_sha256']
    summary,cases = inventory(run/'pytest.xml')
    assert summary == r['summary'] == dict(tests=len(expected_cases),failures=0,errors=0,skipped=0)
    assert sorted(cases) == sorted(expected_cases)
    counts = dict(Counter(c[0] for c in cases)); expected_counts = dict(old_counts,**extra_counts)
    assert counts == expected_counts == p['expected_counts_by_file'] == r['test_case_counts_by_file']
    workers = []; union = []; all_pgids = set()
    expected_workers = {str(run/('worker_%d'%i)/'result.json') for i in range(p['worker_count'])}
    assert set(r['worker_result_sha256']) == expected_workers
    for i in range(p['worker_count']):
        folder = run/('worker_%d'%i); path = folder/'result.json'
        wr = keep(path,r['worker_result_sha256'][str(path)])
        assert wr['passed'] and wr['source_unchanged'] and wr['scope_complete']
        assert wr['worker_index'] == i and wr['plan_sha256'] == r['plan_sha256'] and wr['mode'] == 'full'
        assert wr['test_scope'] == p['shards'][i]['scope']
        assert wr['required_changed_sources'] == p['shards'][i]['required_changed_sources']
        assert wr['direct_child_reaped'] and not wr['direct_child_cleanup_signals']
        assert not wr['external_boundary_attempts'] and not wr['foreign_or_inconsistent_imports']
        assert all(wr['loaded_changed_sources'][x] for x in wr['required_changed_sources'])
        for name,key in (('pytest.xml','xml_sha256'),('pytest.log','log_sha256'),('module_provenance.json','module_provenance_sha256')):
            item = folder/name; assert sha(item) == wr[key]; evidence[str(item)] = wr[key]
        for name,digest in wr['generated_bootstrap_sha256'].items():
            item = folder/name; assert sha(item) == digest; evidence[str(item)] = digest
        boundary = folder/'boundary_attempts.jsonl'
        assert not boundary.exists() or not boundary.read_text().splitlines()
        if boundary.exists(): evidence[str(boundary)] = sha(boundary)
        ws,wc = inventory(folder/'pytest.xml')
        assert ws == wr['summary'] and wc == list(map(tuple,wr['case_identities']))
        assert sorted(wc) == sorted(map(tuple,p['shards'][i]['case_identities']))
        assert type(wr['worker_pgid']) is int and wr['worker_pgid'] > 1
        all_pgids.add(wr['worker_pgid']); union.extend(wc)
        workers.append(dict(index=i,pgid=wr['worker_pgid'],summary=ws,result_sha256=sha(path)))
    assert len(all_pgids) == p['worker_count'] and sorted(union) == sorted(cases)
    assert all(r['loaded_changed_sources_global'].get(name) is True for name in PRODUCTION)
    assert not r['external_boundary_attempts'] and not r['foreign_or_inconsistent_imports']
    assert not r['offline_child_cleanup_required'] and not r['offline_child_cleanup_signals']
    assert not r['residual_offline_process_groups'] and r['worker_returncodes'] == [0]*p['worker_count']
    samples = []; observed_pgids = set()
    for path in sorted(out.glob('sample_*.json')):
        value = keep(path)
        assert value['run'] == str(run) and value['plan_sha256'] == r['plan_sha256']
        assert value['helper_sha256'] == sha(W/'.runtime/monitor_gen45_parallel_regression_20261018.py')
        samples.append(dict(path=str(path),sha256=sha(path),pytest_count=value['pytest_process_count'],
            cpu_percent_sum=value['pytest_cpu_percent_sum'],rss_bytes_sum=value['pytest_rss_bytes_sum'],
            system_iowait_percent=value['system_iowait_percent']))
        observed_pgids.update(value['pytest_pgids'])
        for point in ('before','after'):
            rows = value[point]['processes']
            for child in rows:
                if child['kind'] != 'pytest': continue
                assert child['pgid'] in all_pgids and child['sid'] == child['pgid']
                parent = next((q for q in rows if q['pid'] == child['ppid']),None)
                if parent: assert parent['kind'] == 'worker' and parent['pgid'] == child['pgid'] == parent['pid']
    assert samples and observed_pgids and observed_pgids.issubset(all_pgids)
    assert not sample(run)['processes'], 'accepted run still has matching offline processes'
    residual = []
    for path in Path('/proc').iterdir():
        if not path.name.isdigit(): continue
        try:
            fields = (path/'stat').read_text().rsplit(')',1)[1].split()
            if int(fields[2]) in all_pgids: residual.append(dict(pid=int(path.name),state=fields[0],pgid=int(fields[2])))
        except (FileNotFoundError,ProcessLookupError,PermissionError): pass
    assert not residual, 'recorded worker process groups remain'
    assert all(sha(path) == digest for path,digest in evidence.items())
    result = dict(passed=True,recorded_ns=time.time_ns(),run=str(run),result_sha256=sha(run/'result.json'),
        plan_sha256=sha(run/'plan.json'),xml_sha256=sha(run/'pytest.xml'),
        independently_parsed_summary=summary,exact_file_count=len(counts),exact_source_count=len(sources),
        old_290_file_bytes_and_8289_case_identities_preserved=True,
        supplemental_case_counts=SUPPLEMENTS,supplemental_case_total=148,
        new_case_counts={row['path']:row['expected_count'] for row in new},
        original_source_count=1040,original_source_production_changes=sorted(changes),exact_new_sources=sorted(added),
        workers=workers,all_worker_XML_boundary_and_PGIDs_checked=True,samples=samples,
        sampled_offline_pgids=sorted(observed_pgids),all_worker_pgids=sorted(all_pgids),
        residual_processes=residual,no_matching_run_processes=True,all_current_source_hashes_verified=True,
        elapsed_sec=r['elapsed_sec'],historical_result_elapsed_sec=history['elapsed_sec'],historical_worker_count=history['worker_count'],
        evidence_sha256=evidence,helper_sha256=sha(__file__),no_ROS_or_hardware_contact=True,temperature_read=False,
        measurement_limits='CPU/RSS/IO are bounded samples, not complete full-run IO or peak-memory guarantees; current and historical parallel scopes differ.')
    target = out/'result.json'
    with target.open('x') as stream:
        json.dump(result,stream,ensure_ascii=False,indent=2,allow_nan=False)
        stream.write('\n');stream.flush();os.fsync(stream.fileno())
    assert read(target) == result
    print(json.dumps(dict(passed=True,result=str(target),sha256=sha(target),summary=summary,
        elapsed_sec=r['elapsed_sec'],source_count=len(sources),test_files=len(counts)),ensure_ascii=False))


if __name__ == '__main__':
    main()
