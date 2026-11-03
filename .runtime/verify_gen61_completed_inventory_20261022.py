"""Independent completed-result inventory and provenance check; no tests rerun."""
import hashlib
import importlib.util
import json
from pathlib import Path
import time

W=Path(__file__).resolve().parents[1]
E=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z/gen58_resume_r1/observation_pose_fix/held_lattice_fix_r1/seed_wait_fix_r1')
V=E/'verification';C=W/'.runtime/gen61_seed_wait_candidate_r1'
def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def read(path):return json.loads(Path(path).read_bytes())
def main():
    assert Path('/media/zhuyupei/SanStand').is_mount()
    old=W/'.runtime/verify_gen57_parallel_offline_r2_20261020.py'
    assert sha(old)=='0e217bc784b24c603afa1f4bca7c977f35f9036c03b6b263c9801f749d1c2727'
    spec=importlib.util.spec_from_file_location('gen61_exact_xml_inventory',old)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    f=V/'full_r1';manifest=read(V/'freeze_r1/manifest_closed.json');result=read(f/'result.json');plan=read(f/'plan.json')
    assert result['passed'] and result['full_regression_passed'] and result['scope_complete'] and result['source_unchanged']
    assert result['mode']=='full' and result['test_scope_files']==354
    assert result['candidate_manifest_sha256']==sha(V/'freeze_r1/manifest_closed.json')
    assert result['plan_sha256']==sha(f/'plan.json') and result['xml_sha256']==sha(f/'pytest.xml')
    summary,cases,_=module.inventory(f/'pytest.xml')
    expected=manifest['context']['case_identities']
    assert summary==result['summary']==dict(tests=10149,failures=0,errors=0,skipped=0)
    assert len(cases)==len(set(map(tuple,cases)))==manifest['total_case_count']==10149
    assert sorted(map(tuple,cases))==sorted(map(tuple,expected))==sorted(map(tuple,plan['expected_case_identities']))
    historical=manifest['context']['old_case_identities']
    assert len(historical)==len(set(map(tuple,historical)))==10044
    assert set(map(tuple,historical))<=set(map(tuple,cases))
    assert len(set(map(tuple,cases))-set(map(tuple,historical)))==105
    assert result['source_sha256']==manifest['source_sha256']==plan['context']['sources']
    assert all(sha(C/p)==h for p,h in manifest['copied_input_sha256'].items())
    assert all(sha(W/p)==h for p,h in manifest['baseline_live_source_sha256'].items())
    assert not result['external_boundary_attempts'] and not result['foreign_or_inconsistent_imports']
    assert not result['missing_worker_results'] and not result['residual_offline_process_groups']
    assert all(result['loaded_changed_sources_global'][p] for p in manifest['context']['required_changed_sources'])
    process=read(f/'final_process_receipt.json')
    assert process['passed'] and process['verification_passed'] and process['result_sha256']==sha(f/'result.json')
    for relative,digest in result['worker_result_sha256'].items():
        p=Path(relative)
        if not p.is_absolute():p=f/p
        assert sha(p)==digest
    report=dict(passed=True,kind='GEN61_independent_completed_inventory',recorded_ns=time.time_ns(),
        total_cases=len(cases),historical_cases=10044,new_cases=105,test_files=354,
        no_duplicates_omissions_or_unexpected_cases=True,original_history_preserved=True,
        candidate_copied_input_count=len(manifest['copied_input_sha256']),baseline_live_sources_unchanged=True,
        changed_import_sources_complete=True,external_boundary_attempts=0,foreign_imports=0,
        exact_owned_processes_closed=True,tests_rerun=False,production_deployed=False,
        input_sha256={str(p):sha(p) for p in (f/'result.json',f/'plan.json',f/'pytest.xml',
            f/'final_process_receipt.json',V/'freeze_r1/manifest_closed.json',Path(__file__).resolve())})
    out=f/'independent_verification.json'
    with out.open('x') as stream:json.dump(report,stream,indent=2);stream.write('\n')
    print(json.dumps(dict(passed=True,path=str(out),sha256=sha(out))))

if __name__=='__main__':main()
