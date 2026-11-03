"""Independent file-only verification of a completed GEN60 focus/full run."""
import argparse
import ast
from collections import Counter
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import time
import xml.etree.ElementTree as ET

W = Path(__file__).resolve().parents[1]
E = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z/gen58_resume_r1/observation_pose_fix/held_lattice_fix_r1')
RELEASE = E.parents[2]/'verification/gen59_release_manifest.json'
RELEASE_SHA = '5fed4cbed7ea1e824eda3bb750657970dcc1ec2d711cd041eff1a3d482ef2c6c'
PRODUCTION = 'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot/held_pregrasp_selection.py'
CONTRACT = 'src/alicia_flexible_grasp_supervisor/tests/test_gen55_held_pregrasp_reuse.py'
CONTRACT_FUNCTION = 'test_unsuccessful_solver_retains_exact_failure_and_evidence'
GUARD_SHA = '5d00e04e91cc09ed375c6dcf7ce728eaee4b0c3cfa9190f4a0ab7c17f7f9e21e'
IMPORT_AUDIT_SHA = '3172503f7e3d72c157ce49e75888a6605f1fc9a69767bee585520861ce4c4f8b'


def sha(path): return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def read(path): return json.loads(Path(path).read_bytes())


def inventory(path):
    tree = ET.parse(path).getroot()
    cases = list(tree.iter('testcase'))
    keys = [(c.get('file',''),c.get('classname',''),c.get('name','')) for c in cases]
    assert len(keys) == len(set(keys))
    summary = {key:sum(int(s.get(key,'0')) for s in tree.iter('testsuite'))
               for key in ('tests','failures','errors','skipped')}
    assert summary['tests'] == len(keys)
    assert all(summary[key] == sum(len(c.findall(tag)) for c in cases)
               for key,tag in [('failures','failure'),('errors','error'),('skipped','skipped')])
    return summary,keys


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--result',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args = parser.parse_args(); path = args.result.resolve(strict=True)
    assert E in path.parents and E in args.output.resolve().parents
    assert Path('/media/zhuyupei/SanStand').is_mount()
    assert sha(RELEASE) == RELEASE_SHA
    r = read(path); p = read(path.parent/'plan.json'); mpath = Path(r['candidate_manifest_path']); m = read(mpath)
    assert r['passed'] and r['scope_complete'] and r['source_unchanged'] and r['test_bodies_executed']
    assert r['mode'] in ('focus','full') and r['mode'] == p['mode']
    assert r['plan_sha256'] == sha(path.parent/'plan.json')
    assert r['candidate_manifest_sha256'] == p['candidate_manifest_sha256'] == sha(mpath)
    assert r['runner_sha256'] == p['runner_sha256'] == m['runner_sha256'] == sha(p['runner_path'])
    assert sha(m['infrastructure_path']) == m['infrastructure_sha256'] == '0e217bc784b24c603afa1f4bca7c977f35f9036c03b6b263c9801f749d1c2727'
    assert m['kind'] == 'GEN60_held_lattice_frozen_offline_candidate' and m['revision'] == 2
    c = Path(m['workspace_root']); release = read(RELEASE); original = release['source_sha256']
    assert len(original) == 1094 and m['baseline_live_source_sha256'] == original
    assert r['source_sha256'] == p['context']['sources'] == m['source_sha256']
    assert len(m['source_sha256']) == 1097 and len(m['copied_input_sha256']) == 1109
    assert all(sha(c/name) == digest for name,digest in m['copied_input_sha256'].items())
    assert all(sha(W/name) == digest for name,digest in original.items())
    assert sorted(name for name in original if original[name] != m['source_sha256'][name]) == sorted([PRODUCTION,CONTRACT])
    assert m['source_sha256'][PRODUCTION] == '9930078f45949c6c48458b44155598bdb54d411e33c99822b7982f818ef1e4e1'
    before,after = [ast.parse((root/CONTRACT).read_text()) for root in (W,c)]
    old = next(n for n in before.body if isinstance(n,ast.FunctionDef) and n.name == CONTRACT_FUNCTION)
    new = next(n for n in after.body if isinstance(n,ast.FunctionDef) and n.name == CONTRACT_FUNCTION)
    new.body = deepcopy(old.body)
    assert ast.dump(before,include_attributes=False) == ast.dump(after,include_attributes=False)
    assert len(m['context']['compatibility']['explicit_old_test_contract_changes']) == 1
    historical = set(map(tuple,m['context']['old_case_identities']))
    all_cases = set(map(tuple,m['context']['case_identities']))
    assert len(historical) == 9993 and historical <= all_cases and len(all_cases-historical) == 51
    assert len(all_cases) == 10044 and len(m['context']['scope']) == 352
    summary,cases = inventory(path.parent/'pytest.xml')
    assert sha(path.parent/'pytest.xml') == r['xml_sha256']
    expected = list(map(tuple,p['expected_case_identities']))
    assert set(cases) == set(expected) and len(cases) == len(expected)
    assert summary == r['summary'] == dict(tests=len(expected),failures=0,errors=0,skipped=0)
    assert r['test_case_counts_by_file'] == dict(Counter(x[0] for x in expected))
    assert r['worker_count'] == p['worker_count'] == len(r['worker_result_sha256']) <= 6
    assert all(code == 0 for code in r['worker_returncodes'])
    for key in ('external_boundary_attempts','foreign_or_inconsistent_imports','duplicate_case_identities',
                'missing_case_identities','unexpected_case_identities','missing_worker_results',
                'interrupted','launcher_error','offline_child_cleanup_required','residual_offline_process_groups',
                'live_ROS_contacted','hardware_contacted','motion_commands_sent','temperature_queried','deployed'):
        assert not r[key], key
    assert r['combined_xml_matches_worker_results']
    assert r['loaded_changed_sources_global'][PRODUCTION]
    combined=[]; loaded_entries=0; evidence={str(path):sha(path),str(mpath):sha(mpath)}
    for worker_path,digest in r['worker_result_sha256'].items():
        worker_path=Path(worker_path); assert sha(worker_path) == digest
        child = read(worker_path); folder=worker_path.parent
        assert child['passed'] and child['scope_complete'] and child['source_unchanged']
        assert child['returncode'] == 0 and child['direct_child_reaped'] and not child['direct_child_cleanup_signals']
        assert child['plan_sha256'] == r['plan_sha256']
        for key in ('foreign_or_inconsistent_imports','external_boundary_attempts','interrupted'):
            assert not child[key]
        ws,wc = inventory(folder/'pytest.xml'); combined.extend(wc)
        assert ws == child['summary'] and set(wc) == set(map(tuple,child['case_identities']))
        assert set(wc) == set(map(tuple,p['shards'][child['worker_index']]['case_identities']))
        for name,key in [('pytest.xml','xml_sha256'),('pytest.log','log_sha256'),('module_provenance.json','module_provenance_sha256')]:
            assert sha(folder/name) == child[key]; evidence[str(folder/name)] = child[key]
        assert sha(folder/'bootstrap/sitecustomize.py') == GUARD_SHA
        assert sha(folder/'bootstrap/gen42_import_audit.py') == IMPORT_AUDIT_SHA
        for name,digest in child['generated_bootstrap_sha256'].items(): assert sha(folder/name) == digest
        for row in read(folder/'module_provenance.json').values():
            source=Path(row['path']); digest=row['sha256']; loaded_entries+=1
            assert sha(source) == digest
            if c in source.parents:
                assert m['copied_input_sha256'][str(source.relative_to(c))] == digest
            elif str(source) in m['context']['generated_imports']:
                assert m['context']['generated_imports'][str(source)] == digest
            else:
                allowed=list(child['required_supplemental_imports'].values())+list(child['accepted_exact_historical_oracle'].values())
                assert row in allowed
                assert m['context']['compatibility_dependencies'].get(str(source)) == digest
        boundary=folder/'boundary_attempts.jsonl'
        assert not boundary.exists() or not boundary.read_text().strip()
        evidence[str(worker_path)] = sha(worker_path)
    assert len(combined) == len(set(combined)) and set(combined) == set(cases)
    assert sha(r['measured_workload']['path']) == r['measured_workload']['sha256']
    assert p['resource_budget']['reserved_cpu_count'] == 5
    if r['mode'] == 'full':
        assert r['full_regression_passed'] and set(cases) == all_cases
        assert r['test_scope_files'] == 352 and r['summary']['tests'] == 10044
    report=dict(passed=True,kind='GEN60_file_only_independent_verification',recorded_ns=time.time_ns(),
        verifier_sha256=sha(__file__),mode=r['mode'],full_regression_passed=r['full_regression_passed'],
        result_sha256=sha(path),manifest_sha256=sha(mpath),runner_sha256=r['runner_sha256'],
        total_cases=len(cases),historical_case_count=9993,new_cases=51,source_count=1097,
        full_scope_files=352,exact_case_union=True,duplicate_cases=[],missing_cases=[],
        explicit_contract_file_count=1,explicit_contract_function_body_count=1,
        other_historical_test_or_fixture_bytes_changed=False,active_matches_GEN59_baseline=True,
        candidate_matches_tested_snapshot=True,audited_module_entries=loaded_entries,
        independent_AST_verification=True,evidence_sha256=evidence,elapsed_sec=r['elapsed_sec'],
        measured_workload=r['measured_workload'],production_deployed=False,
        ROS_or_hardware_contacted=False,physical_grasp_verified=False)
    with args.output.open('x') as stream: json.dump(report,stream,ensure_ascii=False,indent=2); stream.write('\n')
    print(json.dumps(dict(passed=True,total_cases=len(cases),output=str(args.output),sha256=sha(args.output))))


if __name__ == '__main__': main()
