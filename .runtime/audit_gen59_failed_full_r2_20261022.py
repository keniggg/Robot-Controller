"""Preserve exact failed full evidence and confirm owned computation exited."""
from collections import Counter
import hashlib
import json
from pathlib import Path
import subprocess
import time
import xml.etree.ElementTree as ET

W=Path(__file__).resolve().parents[1]
D=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z/gen58_resume_r1/observation_pose_fix/verification/full_r2')
M=D.parent/'freeze_r2/manifest_closed.json'

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def read(p):return json.loads(Path(p).read_bytes())
def inventory(p):
    root=ET.parse(p).getroot();cases=[];failures=[]
    summary=dict.fromkeys(('tests','failures','errors','skipped'),0)
    for suite in root.iter('testsuite'):
        for key in summary:summary[key]+=int(suite.get(key,0))
    for c in root.iter('testcase'):
        identity=(c.get('file',''),c.get('classname',''),c.get('name',''));cases.append(identity)
        for tag in ('failure','error','skipped'):
            for node in c.findall(tag):failures.append(dict(case_identity=identity,kind=tag,message=node.get('message'),trace=node.text))
    assert len(cases)==len(set(cases))==summary['tests']
    for key,tag in (('failures','failure'),('errors','error'),('skipped','skipped')):assert summary[key]==sum(r['kind']==tag for r in failures)
    return summary,cases,failures

def main():
    m=read(M);r=read(D/'result.json');p=read(D/'plan.json');C=Path(m['workspace_root'])
    assert not r['passed'] and not r['full_regression_passed'] and r['scope_complete'] and r['source_unchanged']
    assert r['summary']==dict(tests=9992,failures=11,errors=0,skipped=0)
    assert sha(M)==r['candidate_manifest_sha256']==p['candidate_manifest_sha256']=='3959986a6557731dc4d2c2a952bac3b6a8c0c3f2c815a26e852771a2884337f5'
    assert sha(D/'plan.json')==r['plan_sha256'] and sha(D/'pytest.xml')==r['xml_sha256']
    assert r['runner_sha256']==m['runner_sha256']==sha(p['runner_path'])
    assert all(sha(C/x)==h for x,h in m['copied_input_sha256'].items())
    assert all(sha(W/x)==h for x,h in m['baseline_live_source_sha256'].items())
    summary,cases,failures=inventory(D/'pytest.xml');assert summary==r['summary']
    expected=list(map(tuple,m['context']['case_identities']))
    assert sorted(cases)==sorted(expected)==sorted(map(tuple,p['expected_case_identities']))
    assert len(m['context']['old_case_identities'])==9893 and len(cases)-9893==99
    union=[]
    for i in range(r['worker_count']):
        folder=D/('worker_%d'%i);child=read(folder/'result.json');shard=p['shards'][i]
        assert sha(folder/'result.json')==r['worker_result_sha256'][str(folder/'result.json')]
        assert child['source_unchanged'] and child['scope_complete'] and child['direct_child_reaped']
        assert not child['external_boundary_attempts'] and not child['foreign_or_inconsistent_imports']
        assert not child['direct_child_cleanup_signals'] and not child['interrupted']
        assert sha(folder/'pytest.xml')==child['xml_sha256'] and sha(folder/'pytest.log')==child['log_sha256']
        assert sha(folder/'module_provenance.json')==child['module_provenance_sha256']
        assert all(sha(folder/x)==h for x,h in child['generated_bootstrap_sha256'].items())
        cs,cc,_=inventory(folder/'pytest.xml');assert cs==child['summary'] and sorted(cc)==sorted(map(tuple,shard['case_identities']))
        assert child['passed']==(not(cs['failures'] or cs['errors'] or cs['skipped']))
        union+=cc
        for row in read(folder/'module_provenance.json').values():
            source=Path(row['path']);assert sha(source)==row['sha256']
            if C/'src' in source.parents:assert m['source_sha256'][str(source.relative_to(C))]==row['sha256']
            elif W/'devel' in source.parents:assert p['context']['generated_imports'][str(source)]==row['sha256']
            else:assert any(str(C/x['relative'])==row['path'] and x['sha256']==row['sha256'] and x['test'] in shard['scope'] for x in p['context']['supplemental_executable_inputs'])
    assert sorted(union)==sorted(cases) and len(union)==len(set(union))
    assert not any(r[k] for k in ('external_boundary_attempts','foreign_or_inconsistent_imports','duplicate_case_identities','missing_case_identities','unexpected_case_identities','missing_worker_results','residual_offline_process_groups','offline_child_cleanup_required','launcher_error'))
    unitname='alicia-gen59-offline-full-r2.service'
    response=subprocess.run(['systemctl','--user','show',unitname,'--property=Id,ActiveState,SubState,MainPID,Result,ExecMainStatus,InvocationID'],capture_output=True,text=True,timeout=10)
    assert response.returncode==0
    unit=dict(line.split('=',1) for line in response.stdout.splitlines() if '=' in line)
    assert unit['MainPID']=='0' and unit['ExecMainStatus']=='1' and unit['Result']=='exit-code' and unit['InvocationID']=='62802f9e2e314be9aea8e4e083d50f7f'
    coordinator=read(D/'coordinator_identity.json');owned=read(D/'owned_workers.json')
    identities={(coordinator['pid'],coordinator['identity']['start_ticks']):'coordinator'}
    identities.update({(x['pid'],x['identity']['start_ticks']):'worker' for x in owned['workers']})
    assert sha(D/'workload_samples.jsonl')==r['measured_workload']['sha256']
    with (D/'workload_samples.jsonl').open() as stream:
        for line in stream:
            sample=json.loads(line)
            for row in sample['offline_process_groups']['processes']:identities.setdefault((row['pid'],row['start_ticks']),'offline_descendant')
    process_checks=[]
    for (pid,start),role in sorted(identities.items()):
        stat=Path('/proc')/str(pid)/'stat';same=False;observed=None
        if stat.exists():
            fields=stat.read_text().rsplit(')',1)[1].split();observed=dict(start_ticks=int(fields[19]),state=fields[0]);same=observed['start_ticks']==start
        assert not same
        process_checks.append(dict(pid=pid,start_ticks=start,role=role,current=observed,same_identity_remaining=same))
    report=dict(evidence_audit_passed=True,regression_passed=False,kind='GEN59_failed_full_evidence_audit',at_ns=time.time_ns(),summary=summary,elapsed_sec=r['elapsed_sec'],full_result_sha256=sha(D/'result.json'),manifest_sha256=sha(M),verifier_sha256=sha(__file__),exact_case_union=True,historical_cases=9893,new_cases=99,source_count=1092,test_files=350,source_unchanged=True,active_GEN58_unchanged=True,foreign_imports=0,external_boundary_attempts=0,failures=failures,unit=unit,process_checks=process_checks,same_identity_residual_count=0,measured_workload=r['measured_workload'],pytest_cpu_user_sec=r['pytest_cpu_user_sec'],pytest_cpu_system_sec=r['pytest_cpu_system_sec'],candidate_installed=False,source_modified=False,ROS_nodes_stopped=False,signals_sent=False)
    target=D/'failure_evidence_audit.json'
    with target.open('x') as f:json.dump(report,f,ensure_ascii=False,indent=2);f.write('\n')
    print(json.dumps(dict(evidence_audit_passed=True,regression_passed=False,summary=summary,receipt_sha256=sha(target),process_identities_checked=len(process_checks))))

if __name__=='__main__':main()
