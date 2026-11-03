"""Independent read-only XML, identity, source and import audit of GEN59 r3."""
import ast
from collections import Counter
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import time
import xml.etree.ElementTree as ET

W=Path(__file__).resolve().parents[1]
N=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z')
V=N/'gen58_resume_r1/observation_pose_fix/verification'
M=V/'freeze_r3/manifest_closed.json'
M_SHA='89ac6a0f87f28c43f49dcc4c84682e2b539394b16038fdc276960efe4af0f5b9'
D=V/'full_r3'
H=N/'verification/missing_depth_prior_full_r2'
RELEASE=N/'verification/gen58_release_manifest.json'
PRODUCTION={
 'src/alicia_grasp_modes/scripts/mode_aware_remote_grasp6d.py',
 'src/alicia_grasp_modes/src/alicia_grasp_modes/observe_return_visual.py',
 'src/alicia_grasp_modes/src/alicia_grasp_modes/reachable_observation_views.py'}
CONTRACTS={
 'src/alicia_grasp_modes/tests/test_observation_fk_first_strict.py':
 ('test_first_fk_enters_original_strict_and_complete_view_gate_before_any_ik_hints',),
 'src/alicia_grasp_modes/tests/test_gen8_missing_side_distance_binding.py':(
 'test_recorded_low_slots_fail_at_actual_center_then_pass_unchanged_gate',
 'test_existing_passing_pose_is_preserved_exactly_even_if_another_gap_satisfies_it'),
 'src/alicia_grasp_modes/tests/test_gen8_precompute_gen20_equivalence.py':
 ('test_frozen_gen20_complete_result_equals_before_precomputation',),
 'src/alicia_grasp_modes/tests/test_reachable_observation_early_qualified.py':(
 'recorded', 'test_gen14_first_qualified_probe_survives_original_computation_budget',
 'test_unqualified_feasible_probes_never_become_proposals')}
NEW_TESTS={'src/alicia_grasp_modes/tests/test_observation_first_fk_comparison.py',
 'src/alicia_grasp_modes/tests/test_observation_shared_band_gain.py'}
NEW_FIXTURES={'src/alicia_grasp_modes/tests/fixtures/gen58_observation_shared_band_gain.json',
 'src/alicia_grasp_modes/tests/fixtures/gen59_fk_source.json',
 'src/alicia_grasp_modes/tests/fixtures/gen59_fk_source.npz'}


def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def read(p):return json.loads(Path(p).read_bytes())
def cases(p):
    root=ET.parse(p).getroot()
    values=[(c.get('file',''),c.get('classname',''),c.get('name','')) for c in root.iter('testcase')]
    assert len(values)==len(set(values))
    summary=dict.fromkeys(('tests','failures','errors','skipped'),0)
    for suite in root.iter('testsuite'):
        for key in summary:summary[key]+=int(suite.get(key,0))
    assert summary['tests']==len(values)
    for key,tag in (('failures','failure'),('errors','error'),('skipped','skipped')):
        assert summary[key]==sum(len(c.findall(tag)) for c in root.iter('testcase'))
    return summary,values


def main():
    assert sha(M)==M_SHA
    m=read(M);r=read(D/'result.json');p=read(D/'plan.json');C=Path(m['workspace_root']);old=read(RELEASE)
    assert old['generation']=='GEN58' and old['passed']
    assert r['passed'] and r['full_regression_passed'] and r['scope_complete'] and r['source_unchanged']
    assert r['summary']==dict(tests=9993,failures=0,errors=0,skipped=0)
    assert sha(M)==r['candidate_manifest_sha256']==p['candidate_manifest_sha256']
    assert sha(D/'plan.json')==r['plan_sha256'] and sha(D/'pytest.xml')==r['xml_sha256']
    assert r['runner_sha256']==p['runner_sha256']==m['runner_sha256']==sha(p['runner_path'])
    assert m['source_sha256']==r['source_sha256']==p['context']['sources']
    assert len(m['source_sha256'])==1094 and len(m['copied_input_sha256'])==1106
    assert len(m['context']['scope'])==350 and len(set(m['context']['scope']))==350
    assert all(sha(C/x)==h for x,h in m['copied_input_sha256'].items())
    assert old['source_sha256']==m['baseline_live_source_sha256']
    assert all(sha(W/x)==h for x,h in old['source_sha256'].items())
    assert {x for x,h in old['source_sha256'].items() if m['source_sha256'][x]!=h}==PRODUCTION|set(CONTRACTS)
    assert set(m['source_sha256'])-set(old['source_sha256'])==NEW_TESTS|NEW_FIXTURES
    for rel,names in CONTRACTS.items():
        before=ast.parse((W/rel).read_text());after=ast.parse((C/rel).read_text())
        for name in names:
            a=next(n for n in before.body if isinstance(n,ast.FunctionDef) and n.name==name)
            b=next(n for n in after.body if isinstance(n,ast.FunctionDef) and n.name==name)
            b.body=deepcopy(a.body)
        if rel.endswith('/test_reachable_observation_early_qualified.py'):
            assert after.body[0].value.value==before.body[0].value.value.replace('GEN14','GEN58')
            after.body[0]=deepcopy(before.body[0])
            def probe(tree):
                return next(n for n in tree.body if isinstance(n,ast.Assign) and len(n.targets)==1
                    and isinstance(n.targets[0],ast.Name) and n.targets[0].id=='FEASIBLE_PROBE')
            probe(after).value=deepcopy(probe(before).value)
        assert ast.dump(before)==ast.dump(after), 'Historical test change outside authorized bodies/early probe/docstring: '+rel
    hs,historical=cases(H/'pytest.xml');assert hs==dict(tests=9893,failures=0,errors=0,skipped=0)
    assert sha(H/'result.json')==p['historical_result_sha256']
    assert sha(H/'pytest.xml')==p['historical_xml_sha256']
    collection=read(V/'collect_r3/result.json');collected=list(map(tuple,collection['collected_case_identities']))
    assert collection['passed'] and len(collected)==len(set(collected))==100
    assert set(c[0] for c in collected)==NEW_TESTS and not set(historical)&set(collected)
    expected=historical+collected;summary,actual=cases(D/'pytest.xml')
    assert summary==r['summary']
    assert sorted(actual)==sorted(expected)==sorted(map(tuple,p['expected_case_identities']))==sorted(map(tuple,m['context']['case_identities']))
    for deps in (m['context']['compatibility_dependencies'],m['context']['generated_imports'],p['focus_evidence_sha256']):
        assert all(sha(path)==digest for path,digest in deps.items())
    union=[];imports=0
    for i in range(r['worker_count']):
        folder=D/('worker_%d'%i);child=read(folder/'result.json');shard=p['shards'][i]
        assert sha(folder/'result.json')==r['worker_result_sha256'][str(folder/'result.json')]
        assert child['passed'] and child['source_unchanged'] and child['scope_complete'] and child['direct_child_reaped']
        assert child['plan_sha256']==r['plan_sha256'] and child['test_scope']==shard['scope']
        assert not child['external_boundary_attempts'] and not child['foreign_or_inconsistent_imports']
        assert not child['direct_child_cleanup_signals'] and not child['interrupted']
        assert sha(folder/'pytest.xml')==child['xml_sha256'] and sha(folder/'pytest.log')==child['log_sha256']
        assert sha(folder/'module_provenance.json')==child['module_provenance_sha256']
        assert all(sha(folder/x)==h for x,h in child['generated_bootstrap_sha256'].items())
        boundary=folder/'boundary_attempts.jsonl';assert not boundary.exists() or not boundary.read_text().strip()
        child_summary,child_cases=cases(folder/'pytest.xml');assert child_summary==child['summary']
        assert sorted(child_cases)==sorted(map(tuple,shard['case_identities']))
        union+=child_cases
        provenance=read(folder/'module_provenance.json');imports+=len(provenance)
        for label,row in provenance.items():
            source=Path(row['path']);assert sha(source)==row['sha256']
            if C/'src' in source.parents:assert m['source_sha256'][str(source.relative_to(C))]==row['sha256']
            elif W/'devel' in source.parents:assert p['context']['generated_imports'][str(source)]==row['sha256']
            else:assert any(row['path']==str(C/x['relative']) and row['sha256']==x['sha256'] and x['test'] in shard['scope'] for x in p['context']['supplemental_executable_inputs'])
    assert sorted(union)==sorted(expected) and len(union)==len(set(union))
    assert all(r[k]==[] for k in ('external_boundary_attempts','duplicate_case_identities','missing_case_identities','unexpected_case_identities','missing_worker_results','residual_offline_process_groups'))
    assert not r['foreign_or_inconsistent_imports'] and not r['offline_child_cleanup_required'] and not r['launcher_error']
    assert r['worker_count']<=6 and r['resource_budget']['reserved_cpu_count']==5
    assert all(r['loaded_changed_sources_global'][x] for x in PRODUCTION)
    output=dict(passed=True,kind='GEN59_observation_pose_independent_full_receipt_verification',recorded_ns=time.time_ns(),
        full_result_sha256=sha(D/'result.json'),manifest_sha256=sha(M),verifier_sha256=sha(__file__),
        total_cases=len(actual),historical_case_count=len(historical),new_cases=len(collected),
        explicit_contract_file_count=4,explicit_contract_function_body_count=7,
        explicit_contract_regions={k:list(v) for k,v in CONTRACTS.items()},
        early_test_additional_value_changes=['FEASIBLE_PROBE','module_docstring_GEN14_to_GEN58'],
        case_identities_changed=False,other_historical_test_or_fixture_bytes_changed=False,
        source_count=len(m['source_sha256']),test_file_count=350,active_matches_GEN58_baseline=True,
        candidate_matches_tested_snapshot=True,active_source_modified=False,production_deployed=False,
        foreign_imports=0,audited_module_entries=imports,exact_case_union=True,duplicate_cases=0,missing_cases=0,
        independent_AST_verification='Only explicitly listed function bodies in four old files and early fixture probe/docstring changed; all case identities/decorators/parameters and every other executable node/assertion identical.',
        temperature_queried=False,ROS_or_hardware_contacted=False,physical_grasp_verified=False)
    with (D/'independent_verification.json').open('x') as f:json.dump(output,f,ensure_ascii=False,indent=2);f.write('\n')
    print(json.dumps(output,ensure_ascii=False))


if __name__=='__main__':main()
