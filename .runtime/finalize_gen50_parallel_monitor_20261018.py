"""Independently verify completed GEN50 full XML, sources, guards and children."""
from collections import Counter
import hashlib
import json
import os
from pathlib import Path
import time
import xml.etree.ElementTree as ET

from monitor_gen50_parallel_regression_20261018 import RUN, OUT, W, read, sha


def inventory(path):
    tree = ET.parse(path).getroot()
    rows = list(tree.iter('testcase'))
    cases = [(row.get('file', ''), row.get('classname', ''), row.get('name', '')) for row in rows]
    summary = {key:sum(int(suite.get(key, 0)) for suite in tree.iter('testsuite'))
               for key in ('tests', 'failures', 'errors', 'skipped')}
    assert summary['tests'] == len(cases) == len(set(cases))
    for key,tag in (('failures', 'failure'), ('errors', 'error'), ('skipped', 'skipped')):
        assert summary[key] == sum(len(row.findall(tag)) for row in rows)
    return summary, cases


def main():
    assert Path('/media/zhuyupei/SanStand').is_mount() and OUT.is_dir()
    output = OUT/'result.json'
    assert not output.exists()
    plan = read(RUN/'plan.json')
    result = read(RUN/'result.json')
    assert result['passed'] and result['full_regression_passed']
    assert result['scope_complete'] and result['source_unchanged']
    assert sha(RUN/'plan.json') == result['plan_sha256']
    assert plan['mode'] == 'full' and plan['worker_count'] == result['worker_count'] == 7
    assert plan['expected_file_count'] == result['test_scope_files'] == 312
    assert plan['expected_case_count'] == 8818
    assert len(plan['context']['sources']) == result['source_count'] == 1058
    assert plan['context']['sources'] == result['source_sha256']
    assert sha(plan['runner_path']) == plan['runner_sha256'] == result['runner_sha256']
    assert sha(plan['candidate_manifest_path']) == plan['candidate_manifest_sha256'] == result['candidate_manifest_sha256']
    summary,cases = inventory(RUN/'pytest.xml')
    assert summary == result['summary'] == dict(tests=8818, failures=0, errors=0, skipped=0)
    assert sorted(cases) == sorted(map(tuple, plan['expected_case_identities']))
    counts = dict(Counter(case[0] for case in cases))
    assert len(counts) == 312 and counts == plan['expected_counts_by_file'] == result['test_case_counts_by_file']
    assert sha(RUN/'pytest.xml') == result['xml_sha256']
    historical = Path(plan['historical_result'])
    assert sha(historical) == plan['historical_result_sha256']
    old = read(historical)
    old_summary,old_cases = inventory(plan['historical_xml'])
    assert sha(plan['historical_xml']) == plan['historical_xml_sha256'] == old['xml_sha256']
    assert old_summary == dict(tests=8747, failures=0, errors=0, skipped=0)
    assert set(old_cases).issubset(cases)
    old_counts = dict(Counter(case[0] for case in old_cases))
    assert len(old_counts) == 309 and all(counts[name] == number for name,number in old_counts.items())
    original = plan['context']['original']
    sources = plan['context']['sources']
    assert len(original) == 1056 and original == old['source_sha256']
    assert all(sha(W/name) == digest for name,digest in sources.items())
    assert all(sources[name] == original[name] for name in old_counts)
    changes = sorted(name for name,digest in original.items() if sources[name] != digest)
    assert changes == sorted([
        'src/alicia_flexible_grasp_supervisor/scripts/grasp_task_node.py',
        'src/alicia_grasp_modes/scripts/unknown_tabletop_perception.py',
        'src/alicia_grasp_modes/src/alicia_grasp_modes/observe_return_measurement.py'])
    expected_new = {
        'src/alicia_grasp_modes/tests/test_gen50_arrival_capture_stationarity.py':31,
        'src/alicia_flexible_grasp_supervisor/tests/test_motion_settle.py':14,
        'src/alicia_grasp_modes/tests/test_observation_compensation_handoff.py':26}
    assert {name:counts[name] for name in set(counts)-set(old_counts)} == expected_new
    workers = []
    union = []
    for index in range(7):
        folder = RUN/('worker_%d'%index)
        path = folder/'result.json'
        worker = read(path)
        assert sha(path) == result['worker_result_sha256'][str(path)]
        assert worker['passed'] and worker['source_unchanged'] and worker['scope_complete']
        assert worker['direct_child_reaped'] and not worker['direct_child_cleanup_signals']
        assert not worker['external_boundary_attempts'] and not worker['foreign_or_inconsistent_imports']
        assert all(worker['loaded_changed_sources'][name] for name in worker['required_changed_sources'])
        assert sha(folder/'module_provenance.json') == worker['module_provenance_sha256']
        assert sha(folder/'pytest.log') == worker['log_sha256']
        assert sha(folder/'pytest.xml') == worker['xml_sha256']
        for name,digest in worker['generated_bootstrap_sha256'].items():
            assert sha(folder/name) == digest
        boundary = folder/'boundary_attempts.jsonl'
        assert not boundary.exists() or not boundary.read_text().strip()
        own_summary,own_cases = inventory(folder/'pytest.xml')
        assert own_summary == worker['summary']
        assert own_cases == list(map(tuple, worker['case_identities']))
        assert sorted(own_cases) == sorted(map(tuple, plan['shards'][index]['case_identities']))
        union.extend(own_cases)
        workers.append(dict(index=index, summary=own_summary, result_sha256=sha(path),
            provenance_sha256=sha(folder/'module_provenance.json')))
    assert sorted(union) == sorted(cases) and len(union) == len(set(union))
    assert all(result['loaded_changed_sources_global'].values())
    assert not result['external_boundary_attempts'] and not result['foreign_or_inconsistent_imports']
    assert not result['offline_child_cleanup_required'] and not result['offline_child_cleanup_signals']
    assert not result['residual_offline_process_groups'] and result['worker_returncodes'] == [0]*7
    samples = []
    pgids = set()
    coordinator_identities = set()
    for path in sorted(OUT.glob('sample_*.json')):
        value = read(path)
        assert value['source1058_sha256_verified_before']
        assert value['pytest_process_count'] == 7 and len(value['pytest_pgids']) == 7
        pgids.update(value['pytest_pgids'])
        samples.append(dict(path=str(path), sha256=sha(path),
            cpu_percent_sum=value['pytest_cpu_percent_sum'],
            rss_bytes_sum=value['pytest_rss_bytes_sum'],
            system_iowait_percent=value['system_iowait_percent']))
        for point in ('before', 'after'):
            rows = value[point]['processes']
            for row in rows:
                if row['kind'] == 'coordinator':
                    coordinator_identities.add((row['pid'], row['start_ticks']))
                if row['kind'] == 'pytest':
                    assert row['sid'] == row['pgid']
                    parent = next(item for item in rows if item['pid'] == row['ppid'])
                    assert parent['kind'] == 'worker'
                    assert parent['pid'] == parent['pgid'] == row['pgid']
    assert len(samples) >= 2 and len(pgids) == 7
    residual = []
    # Only stat identity fields are inspected here; no unrelated command lines,
    # credentials, ROS state or device information are read.
    for path in Path('/proc').iterdir():
        if not path.name.isdigit():
            continue
        try:
            fields = (path/'stat').read_text().rsplit(')', 1)[1].split()
            pid = int(path.name)
            if int(fields[2]) in pgids or (pid, int(fields[19])) in coordinator_identities:
                residual.append(dict(pid=pid, pgid=int(fields[2]), state=fields[0]))
        except (FileNotFoundError, ProcessLookupError, PermissionError):
            pass
    assert not residual
    value = dict(kind='GEN50_independent_full_regression_monitor_acceptance',
        passed=True, recorded_ns=time.time_ns(), run=str(RUN), result_sha256=sha(RUN/'result.json'),
        plan_sha256=sha(RUN/'plan.json'), xml_sha256=sha(RUN/'pytest.xml'),
        independently_parsed_summary=summary, exact_file_count=312, exact_source_count=1058,
        old_source_count=1056, old309_file_bytes_and8747_case_identities_preserved=True,
        exact_added_scope_case_counts=expected_new, original_production_changes=changes,
        exact_new_sources=sorted(set(sources)-set(original)), workers=workers,
        all7_worker_XML_and_boundaries_checked=True, samples=samples,
        sampled_offline_pgids=sorted(pgids), residual_processes=residual,
        all_current_source_hashes_verified=True, elapsed_sec=result['elapsed_sec'],
        helper_sha256=sha(__file__), no_ROS_or_hardware_access=True, temperature_read=False,
        limits='CPU/RSS/IO are two bounded sampled windows, not a complete whole-run IO profile. Exact XML/source/guard/child checks cover this completed run. Added scope includes2 existing unchanged supplementary files14+26cases and1 new31case file; Only the31case file and its fixture are newly added sources.')
    with output.open('x') as handle:
        json.dump(value, handle, ensure_ascii=False, indent=2);handle.write('\n')
    assert read(output) == value
    print(json.dumps(dict(passed=True, result=str(output), sha256=sha(output),
        summary=summary, elapsed_sec=result['elapsed_sec']), ensure_ascii=False))


if __name__ == '__main__':
    main()
