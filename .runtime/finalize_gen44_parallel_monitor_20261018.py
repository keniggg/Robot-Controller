"""Independently verify completed GEN44 XML, source bytes and offline children."""
from collections import Counter
import hashlib
import json
import os
from pathlib import Path
import time
import xml.etree.ElementTree as ET

from monitor_gen44_parallel_regression_20261018 import RUN, OUT, sample

W = Path(__file__).resolve().parents[1]


def read(path):
    return json.loads(Path(path).read_text())


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def inventory(path):
    tree = ET.parse(path).getroot()
    rows = list(tree.iter('testcase'))
    cases = [(r.get('file', ''), r.get('classname', ''), r.get('name', '')) for r in rows]
    summary = {name: sum(int(s.get(name, 0)) for s in tree.iter('testsuite'))
               for name in ('tests', 'failures', 'errors', 'skipped')}
    assert summary['tests'] == len(cases) == len(set(cases))
    for name, tag in (('failures', 'failure'), ('errors', 'error'), ('skipped', 'skipped')):
        assert summary[name] == sum(len(r.findall(tag)) for r in rows)
    return summary, cases


def main():
    assert Path('/media/zhuyupei/SanStand').is_mount() and OUT.is_dir()
    output = OUT/'result.json'
    assert not output.exists()
    p = read(RUN/'plan.json'); r = read(RUN/'result.json')
    assert r['passed'] and r['full_regression_passed'] and r['scope_complete'] and r['source_unchanged']
    assert sha(RUN/'plan.json') == r['plan_sha256']
    assert p['mode'] == 'full' and p['worker_count'] == r['worker_count'] == 8
    assert p['expected_file_count'] == r['test_scope_files'] == 290
    assert p['expected_case_count'] == 8289 and len(p['context']['sources']) == r['source_count'] == 1040
    assert p['context']['sources'] == r['source_sha256']
    assert sha(p['runner_path']) == p['runner_sha256'] == r['runner_sha256']
    assert sha(p['candidate_manifest_path']) == p['candidate_manifest_sha256'] == r['candidate_manifest_sha256']
    summary, cases = inventory(RUN/'pytest.xml')
    assert summary == r['summary'] == dict(tests=8289, failures=0, errors=0, skipped=0)
    assert sorted(cases) == sorted(map(tuple, p['expected_case_identities']))
    counts = dict(Counter(x[0] for x in cases))
    assert len(counts) == 290 and counts == p['expected_counts_by_file'] == r['test_case_counts_by_file']
    assert sha(RUN/'pytest.xml') == r['xml_sha256']
    historical = Path(p['historical_result'])
    assert sha(historical) == p['historical_result_sha256']
    old_result = read(historical)
    old_summary, old_cases = inventory(Path(p['historical_xml']))
    assert sha(p['historical_xml']) == p['historical_xml_sha256'] == old_result['xml_sha256']
    assert old_summary == dict(tests=8258, failures=0, errors=0, skipped=0)
    assert set(old_cases).issubset(cases)
    old_counts = dict(Counter(x[0] for x in old_cases))
    assert len(old_counts) == 288 and all(counts[name] == n for name, n in old_counts.items())
    original = p['context']['original']; sources = p['context']['sources']
    assert len(original) == 1036 and original == old_result['source_sha256']
    assert all(sha(W/name) == digest for name, digest in sources.items())
    assert all(sources[name] == original[name] for name in old_counts)
    changes = sorted(name for name, digest in original.items() if sources[name] != digest)
    assert changes == sorted([
        'src/alicia_flexible_grasp_supervisor/scripts/grasp_task_node.py',
        'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/grasp/contact_execution.py',
        'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/grasp/measured_contact.py'])
    added = sorted(set(sources)-set(original))
    assert added == sorted([
        'src/alicia_flexible_grasp_supervisor/tests/test_gen44_current_stage_contract.py',
        'src/alicia_flexible_grasp_supervisor/tests/test_gen44_task_gateway_path_chain.py',
        'src/alicia_flexible_grasp_supervisor/tests/fixtures/gen44_g43_stage0_replay.json',
        'src/alicia_flexible_grasp_supervisor/tests/fixtures/gen44_reconstructed_contact_path.json'])
    expected_new = {'src/alicia_flexible_grasp_supervisor/tests/test_gen44_current_stage_contract.py':22,
                    'src/alicia_flexible_grasp_supervisor/tests/test_gen44_task_gateway_path_chain.py':9}
    assert {name: counts[name] for name in set(counts)-set(old_counts)} == expected_new
    workers = []
    union = []
    for i in range(8):
        folder = RUN/('worker_%d'%i); path = folder/'result.json'; wr = read(path)
        assert sha(path) == r['worker_result_sha256'][str(path)]
        assert wr['passed'] and wr['source_unchanged'] and wr['scope_complete']
        assert wr['direct_child_reaped'] and not wr['direct_child_cleanup_signals']
        assert not wr['external_boundary_attempts'] and not wr['foreign_or_inconsistent_imports']
        assert all(wr['loaded_changed_sources'][x] for x in wr['required_changed_sources'])
        assert sha(folder/'module_provenance.json') == wr['module_provenance_sha256']
        assert sha(folder/'pytest.log') == wr['log_sha256']
        assert sha(folder/'pytest.xml') == wr['xml_sha256']
        for name, digest in wr['generated_bootstrap_sha256'].items():
            assert sha(folder/name) == digest
        boundary = folder/'boundary_attempts.jsonl'
        assert not boundary.exists() or not boundary.read_text().strip()
        ws, wc = inventory(folder/'pytest.xml')
        assert ws == wr['summary'] and wc == list(map(tuple, wr['case_identities']))
        assert sorted(wc) == sorted(map(tuple, p['shards'][i]['case_identities']))
        union.extend(wc)
        workers.append(dict(index=i, summary=ws, result_sha256=sha(path),
            provenance_sha256=sha(folder/'module_provenance.json')))
    assert sorted(union) == sorted(cases) and all(r['loaded_changed_sources_global'].values())
    assert not r['external_boundary_attempts'] and not r['foreign_or_inconsistent_imports']
    assert not r['offline_child_cleanup_required'] and not r['offline_child_cleanup_signals']
    assert not r['residual_offline_process_groups'] and r['worker_returncodes'] == [0]*8
    samples = []
    pgids = set()
    for path in sorted(OUT.glob('sample_*.json')):
        value = read(path); samples.append(dict(path=str(path), sha256=sha(path),
            pytest_count=value['pytest_process_count'], cpu_percent_sum=value['pytest_cpu_percent_sum'],
            rss_bytes_sum=value['pytest_rss_bytes_sum'], system_iowait_percent=value['system_iowait_percent']))
        pgids.update(value['pytest_pgids'])
        for point in ('before', 'after'):
            rows = value[point]['processes']
            for child in rows:
                if child['kind'] == 'pytest':
                    assert child['sid'] == child['pgid']
                    parent = next((q for q in rows if q['pid'] == child['ppid']), None)
                    if parent:
                        assert parent['kind'] == 'worker' and parent['pgid'] == child['pgid'] == parent['pid']
    assert len(pgids) == 8
    current = sample()
    assert not current['processes'], 'accepted run still has matching offline processes'
    residual = []
    for path in Path('/proc').iterdir():
        if not path.name.isdigit(): continue
        try:
            fields = (path/'stat').read_text().rsplit(')',1)[1].split()
            if int(fields[2]) in pgids: residual.append(dict(pid=int(path.name), state=fields[0], pgid=int(fields[2])))
        except (FileNotFoundError, ProcessLookupError, PermissionError):pass
    assert not residual, 'sampled offline process groups remain'
    result = dict(passed=True, recorded_ns=time.time_ns(), run=str(RUN), result_sha256=sha(RUN/'result.json'),
        plan_sha256=sha(RUN/'plan.json'), xml_sha256=sha(RUN/'pytest.xml'),
        independently_parsed_summary=summary, exact_file_count=290, exact_source_count=1040,
        original_source_count=1036, previous_contract_source_count=1038,
        current_source_count=1040, previous_contract_scope_included=True,
        old_288_file_bytes_and_8258_case_identities_preserved=True, new_case_counts=expected_new,
        original_source_production_changes=changes, exact_new_sources=added,
        workers=workers, all_eight_worker_XML_and_boundary_checked=True,
        samples=samples, sampled_offline_pgids=sorted(pgids), residual_processes=residual,
        no_matching_run_processes=True, all_current_source_hashes_verified=True,
        elapsed_sec=r['elapsed_sec'], historical_serial_elapsed_sec=old_result.get('elapsed_sec'),
        helper_sha256=sha(__file__), no_ROS_or_hardware_contact=True, temperature_read=False,
        measurement_limits='CPU/RSS/IO values are bounded samples, not a complete whole-run IO profile; historical serial and current parallel scopes differ by31 cases.')
    with output.open('x') as f: json.dump(result,f,ensure_ascii=False,indent=2);f.write('\n')
    assert read(output) == result
    print(json.dumps(dict(passed=True, result=str(output), sha256=sha(output), summary=summary,
        elapsed_sec=r['elapsed_sec'], source_count=1040, test_files=290),ensure_ascii=False))


if __name__=='__main__': main()
