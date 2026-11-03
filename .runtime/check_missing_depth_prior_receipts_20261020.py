"""Read-only independent receipt/source inventory verification for this fix."""
import ast
from collections import Counter
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import time
import xml.etree.ElementTree as ET

W=Path(__file__).resolve().parents[1]
V=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z/verification')
M=V/'missing_depth_prior_freeze_r2/manifest_closed.json'
D=V/'missing_depth_prior_full_r2'
NEW='src/alicia_grasp_modes/tests/test_missing_depth_rgb_prior.py'
PRODUCTION='src/alicia_grasp_modes/src/alicia_grasp_modes/rgb_instance.py'
CONTRACT='src/alicia_grasp_modes/tests/test_observation_arrival_probable_foreground.py'
OLD_NAME='test_only_current_valid_measured_height_adds_probable_not_hard_foreground'
NEW_NAME='test_current_depth_proposals_and_missing_prior_never_add_hard_foreground'


def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def read(p):return json.loads(Path(p).read_bytes())
def cases(p):
    tree=ET.parse(p).getroot()
    values=[(c.get('file',''),c.get('classname',''),c.get('name','')) for c in tree.iter('testcase')]
    assert len(values)==len(set(values))
    return values


def main():
    m=read(M);r=read(D/'result.json');p=read(D/'plan.json');C=Path(m['workspace_root']);old=read(V/'gen57_release_manifest.json')
    assert r['passed'] and r['full_regression_passed'] and r['scope_complete'] and r['source_unchanged']
    assert r['summary']==dict(tests=9893,failures=0,errors=0,skipped=0)
    assert sha(M)==r['candidate_manifest_sha256']==p['candidate_manifest_sha256']
    assert sha(D/'plan.json')==r['plan_sha256'] and sha(D/'pytest.xml')==r['xml_sha256']
    assert r['runner_sha256']==p['runner_sha256']==m['runner_sha256']==sha(p['runner_path'])
    assert m['source_sha256']==r['source_sha256']==p['context']['sources']
    assert all(sha(C/x)==h for x,h in m['copied_input_sha256'].items())
    assert all(sha(W/x)==h for x,h in m['source_sha256'].items())
    assert {x for x,h in old['source_sha256'].items() if m['source_sha256'][x]!=h}=={PRODUCTION,CONTRACT}
    assert set(m['source_sha256'])-set(old['source_sha256'])=={NEW}
    before=V.parent/'gen57_resume_r3/actual_attempt_r1_geometry/missing_depth_prior_fix_r1/test_observation_arrival_probable_foreground_before.py'
    assert sha(before)==old['source_sha256'][CONTRACT]
    tree=ast.parse(before.read_text());after=ast.parse((C/CONTRACT).read_text())
    functions=[n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name==OLD_NAME]
    assert len(functions)==1;function=functions[0];function.name=NEW_NAME
    comparisons=[n for n in ast.walk(function) if isinstance(n,ast.Compare)
        and isinstance(n.left,ast.Name) and n.left.id=='fault' and len(n.ops)==len(n.comparators)==1
        and isinstance(n.ops[0],ast.Eq) and isinstance(n.comparators[0],ast.Constant) and n.comparators[0].value=='valid']
    assert len(comparisons)==1
    comparisons[0].ops=[ast.In()];comparisons[0].comparators=[ast.Tuple(elts=[ast.Constant(value=x,kind=None) for x in ('valid','missing','nonfinite_depth')],ctx=ast.Load())]
    assert ast.dump(tree)==ast.dump(after), 'unexpected executable/assertion change in the historical contract file'
    historical=cases(V/'gen57_full_parallel_r2/pytest.xml');mapped=[];renames=[]
    for case in historical:
        if case[0]==CONTRACT and case[2].startswith(OLD_NAME+'['):
            revised=(case[0],case[1],case[2].replace(OLD_NAME,NEW_NAME,1));renames.append((case,revised));mapped.append(revised)
        else:mapped.append(case)
    assert len(renames)==10 and len(mapped)==9766
    collection=read(V/'missing_depth_prior_collect_r2/result.json');collected=[tuple(c) for c in collection['collected_case_identities']]
    assert collection['passed'] and len(collected)==127 and len(collected)==len(set(collected))
    assert not set(mapped)&set(collected)
    expected=mapped+collected;actual=cases(D/'pytest.xml')
    assert sorted(actual)==sorted(expected)==sorted(map(tuple,p['expected_case_identities']))
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
        child_cases=cases(folder/'pytest.xml');assert sorted(child_cases)==sorted(map(tuple,shard['case_identities']))
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
    assert r['worker_count']==6 and r['resource_budget']['reserved_cpu_count']==5
    output=dict(passed=True,kind='missing_depth_prior_independent_full_receipt_verification',recorded_ns=time.time_ns(),
        full_result_sha256=sha(D/'result.json'),manifest_sha256=sha(M),verifier_sha256=sha(__file__),
        total_cases=len(actual),historical_case_count=len(mapped),explicitly_renamed_parameter_cases=10,
        historical_label_expectations_changed=2,other_historical_test_or_fixture_bytes_changed=False,
        supplemental_existing_cases=sum(c[0]!=NEW for c in collected),new_cases=sum(c[0]==NEW for c in collected),
        source_count=len(m['source_sha256']),live_matches_tested_snapshot=True,foreign_imports=0,
        audited_module_entries=imports,exact_case_union=True,duplicate_cases=0,missing_cases=0,
        independent_AST_verification='Only the function rename and valid->(valid,missing,nonfinite_depth) condition change; all other executable nodes/assertions identical.',
        temperature_queried=False,ROS_or_hardware_contacted=False,physical_grasp_verified=False)
    with (D/'independent_verification.json').open('x') as f:json.dump(output,f,ensure_ascii=False,indent=2);f.write('\n')
    print(json.dumps(output,ensure_ascii=False))


if __name__=='__main__':main()
