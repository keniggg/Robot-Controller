"""Independent completed-result inventory and provenance check; no tests rerun."""
import hashlib
import importlib.util
import json
from pathlib import Path
import time

W=Path(__file__).resolve().parents[1]
E=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z/gen58_resume_r1/observation_pose_fix/held_lattice_fix_r1/seed_wait_fix_r1')
V=E/'v62';C=W/'.runtime/gen62_contact_feedback_candidate_r1'
def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def read(path):return json.loads(Path(path).read_bytes())
def main():
    assert Path('/media/zhuyupei/SanStand').is_mount()
    old=W/'.runtime/verify_gen57_parallel_offline_r2_20261020.py'
    assert sha(old)=='0e217bc784b24c603afa1f4bca7c977f35f9036c03b6b263c9801f749d1c2727'
    spec=importlib.util.spec_from_file_location('gen62_exact_xml_inventory',old)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    f=V/'full_r2';manifest=read(V/'freeze_r1/manifest_closed.json');result=read(f/'result.json');plan=read(f/'plan.json')
    assert result['passed'] and result['full_regression_passed'] and result['scope_complete'] and result['source_unchanged']
    assert result['mode']=='full' and result['test_scope_files']==355
    assert result['candidate_manifest_sha256']==sha(V/'freeze_r1/manifest_closed.json')
    assert result['plan_sha256']==sha(f/'plan.json') and result['xml_sha256']==sha(f/'pytest.xml')
    summary,cases,_=module.inventory(f/'pytest.xml')
    expected=manifest['context']['case_identities']
    assert summary==result['summary']==dict(tests=10213,failures=0,errors=0,skipped=0)
    assert len(cases)==len(set(map(tuple,cases)))==manifest['total_case_count']==10213
    assert sorted(map(tuple,cases))==sorted(map(tuple,expected))==sorted(map(tuple,plan['expected_case_identities']))
    historical=manifest['context']['old_case_identities']
    assert len(historical)==len(set(map(tuple,historical)))==10149
    assert set(map(tuple,historical))<=set(map(tuple,cases))
    assert len(set(map(tuple,cases))-set(map(tuple,historical)))==64
    baseline=read(E/'contact_feedback_verification_r1/baseline_inventory.json')
    pilot=read(E/'contact_feedback_verification_r1/pilot_r4_final_epoch/result.json')
    assert sha(E/'contact_feedback_verification_r1/baseline_inventory.json')=='dcd88a7a0fb3b2a52c710d37c9e16e547a5020b232423242696ef31997281182'
    assert sha(E/'contact_feedback_verification_r1/pilot_r4_final_epoch/result.json')=='5d1b4988d1a473d2dfe90886b0cbcc2ee8bd5a3be2ef2e8a003d7c0d73618095'
    assert sorted(map(tuple,historical))==sorted(map(tuple,baseline['case_identities']))
    assert set(map(tuple,cases))-set(map(tuple,historical))==set(map(tuple,pilot['case_identities']))
    assert result['source_sha256']==manifest['source_sha256']==plan['context']['sources']
    assert len(manifest['source_sha256'])==1103 and len(manifest['copied_input_sha256'])==1115
    assert {str(p.relative_to(C)) for p in C.rglob('*') if p.is_file()}==set(manifest['copied_input_sha256'])
    assert all(sha(C/p)==h for p,h in manifest['copied_input_sha256'].items())
    assert all(sha(W/p)==h for p,h in manifest['baseline_live_source_sha256'].items())
    assert all(sha(p)==h for p,h in manifest['context']['compatibility_dependencies'].items())
    assert all(sha(p)==h for p,h in manifest['context']['generated_imports'].items())
    assert not result['external_boundary_attempts'] and not result['foreign_or_inconsistent_imports']
    assert not result['missing_worker_results'] and not result['residual_offline_process_groups']
    assert all(result['loaded_changed_sources_global'][p] for p in manifest['context']['required_changed_sources'])
    process=read(f/'final_process_receipt.json')
    assert process['passed'] and process['verification_passed'] and process['result_sha256']==sha(f/'result.json')
    worker_cases=[];worker_reports=[]
    for relative,digest in result['worker_result_sha256'].items():
        p=Path(relative)
        if not p.is_absolute():p=f/p
        assert f in p.parents and sha(p)==digest
        worker=read(p);index=worker['worker_index'];shard=plan['shards'][index]
        assert worker['passed'] and worker['scope_complete'] and worker['source_unchanged']
        assert worker['returncode']==0 and worker['plan_sha256']==sha(f/'plan.json')
        assert worker['direct_child_reaped'] and not worker['direct_child_cleanup_signals']
        assert not worker['external_boundary_attempts'] and not worker['foreign_or_inconsistent_imports']
        assert sha(p.parent/'pytest.xml')==worker['xml_sha256']
        assert sha(p.parent/'pytest.log')==worker['log_sha256']
        assert sha(p.parent/'module_provenance.json')==worker['module_provenance_sha256']
        assert all(sha(p.parent/name)==h for name,h in worker['generated_bootstrap_sha256'].items())
        ws,wc,_=module.inventory(p.parent/'pytest.xml')
        assert ws==worker['summary']==dict(tests=shard['expected_count'],failures=0,errors=0,skipped=0)
        assert sorted(map(tuple,wc))==sorted(map(tuple,worker['case_identities']))==sorted(map(tuple,shard['case_identities']))
        assert sorted(worker['test_scope'])==sorted(shard['scope'])
        worker_cases.extend(wc)
        worker_reports.append(dict(worker_index=index,cases=len(wc),result_sha256=digest,xml_sha256=worker['xml_sha256']))
    assert len(worker_reports)==plan['worker_count']==result['worker_count']
    assert len(worker_cases)==len(set(map(tuple,worker_cases)))==10213
    assert sorted(map(tuple,worker_cases))==sorted(map(tuple,cases))
    report=dict(passed=True,kind='GEN62_independent_completed_inventory',recorded_ns=time.time_ns(),
        total_cases=len(cases),historical_cases=10149,new_cases=64,test_files=355,
        no_duplicates_omissions_or_unexpected_cases=True,original_history_preserved=True,
        candidate_copied_input_count=len(manifest['copied_input_sha256']),baseline_live_sources_unchanged=True,
        changed_import_sources_complete=True,external_boundary_attempts=0,foreign_imports=0,
        exact_owned_processes_closed=True,tests_rerun=False,production_deployed=False,
        individual_worker_xml_exact=True,worker_reports=worker_reports,
        input_sha256={str(p):sha(p) for p in (f/'result.json',f/'plan.json',f/'pytest.xml',
            f/'final_process_receipt.json',V/'freeze_r1/manifest_closed.json',Path(__file__).resolve())})
    out=f/'independent_verification.json'
    with out.open('x') as stream:json.dump(report,stream,indent=2);stream.write('\n')
    print(json.dumps(dict(passed=True,path=str(out),sha256=sha(out))))

if __name__=='__main__':main()
