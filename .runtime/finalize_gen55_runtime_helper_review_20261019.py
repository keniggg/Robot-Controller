"""Verify completed focus evidence and reuse unchanged helper-unit checks only.

This process does not run pytest, initialize ROS, install, reload or actuate.
"""
import argparse
from collections import Counter
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import sys

W=Path(__file__).resolve().parents[1]
V=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z/verification')
PRIOR=V/'gen55_runtime_helpers_independent_review_r3/result.json'
PRIOR_SHA='da47d37c8dd3f1575519e7e176983c1e8112def70b68f8c97aab60b3c8414d13'
MANIFEST=V/'gen55_final_collected_r3/candidate_manifest.json'
MANIFEST_SHA='046283dca37056df08461adcf8f68737ede754627e9c2e739e5bfaa823fe39c6'
RUNNER=W/'.runtime/verify_gen55_parallel_offline_20261019.py'
RUNNER_SHA='d095d6e7fe6432104bd2f2441d7284d79c145c738df347ff4634426f91123f2f'
FOCUS=V/'gen55_integrated_focus_r2/result.json'
INSTALLER=W/'.runtime/install_and_seal_gen55_20261019.py'
INSTALLER_REVIEW=V/'gen55_installer_delta_review_r1/result.json'
INSTALLER_REVIEW_SHA='f74fd389526fd5eb41f080d0722d7fd4fb74368cf0d7504f8f2131a423c338a1'
RUNNER_REVIEW=V/'gen55_runner_delta_helper_impact_review_r1/result.json'
RUNNER_REVIEW_SHA='1d664132514faa9c0c1843bbab8112ea7f351ff6a861486fd2d5828a3990ab50'
NEGATIVE=V/'gen55_runtime_helper_real_failed_focus_review_r1/result.json'
NEGATIVE_SHA='68237fc4c5dad5ef958e96451a0d16badeaf61a7a9d40d9abe46aca805be3cc6'
ENTRYPOINT_REVIEW=V/'gen55_recorded_entrypoint_review_r1/result.json'
ENTRYPOINT_REVIEW_SHA='b6131360115fec2a0d377cd3f2f86bea57cbf1dde61c886efd37380e1774feeb'
PASSIVE_REVIEW=V/'gen55_passive_monitor_review_r1/result.json'
PASSIVE_REVIEW_SHA='fd10b1d1314b4701cb796f90b2f221f4a6308bf3f2e0ec8f0c375e635ee5fa81'
COVERAGE=V/'gen55_final_candidate_r5/prior_focus_coverage.json'
COVERAGE_SHA='500583e957964b5e2fe5538c8ac736e6994d73cfa8c733c8e05d98b4d4a5ebc1'

def sha(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def read(p):
    return json.loads(Path(p).read_bytes())

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--focus-sha256',required=True)
    parser.add_argument('--attempt',default='r1')
    args=parser.parse_args()
    assert args.attempt.isalnum() and os.path.ismount('/media/zhuyupei/SanStand')
    output=V/('gen55_runtime_helpers_final_review_'+args.attempt)
    output.mkdir()
    denied=[]
    def boundary(event,values):
        bad=event.startswith(('socket.','subprocess.')) or event in ('os.system','os.exec','os.posix_spawn','os.fork')
        if event=='open' and values and isinstance(values[0],(str,bytes,os.PathLike)):
            p=Path(os.fsdecode(values[0])).resolve()
            flags=values[2] if len(values)>2 and isinstance(values[2],int) else 0
            mode=values[1] if len(values)>1 and isinstance(values[1],str) else ''
            if flags&(os.O_WRONLY|os.O_RDWR|os.O_CREAT|os.O_TRUNC|os.O_APPEND) or any(c in mode for c in 'wax+'):
                bad=p!=output and output not in p.parents
            if str(p).startswith('/dev/') and str(p) not in ('/dev/null','/dev/random','/dev/urandom'):
                bad=True
        if bad:
            denied.append(dict(event=event,args=repr(values)[:200]))
            raise RuntimeError('review-only boundary denied '+event)
    sys.dont_write_bytecode=True
    sys.addaudithook(boundary)
    assert sha(PRIOR)==PRIOR_SHA and sha(MANIFEST)==MANIFEST_SHA and sha(RUNNER)==RUNNER_SHA
    assert sha(FOCUS)==args.focus_sha256
    prior=read(PRIOR)
    assert prior['passed'] is True and len(prior['checks'])==27 and not prior['failed_checks']
    assert not prior['boundary_attempts'] and not prior['live_ROS_contacted'] and not prior['hardware_contacted']
    assert all(row['passed'] and row['accepted']==row['expected_accept'] for row in prior['checks'])
    assert len(prior['historical_counterexamples'])==3 and all(
        row['original_gate_incorrectly_accepted'] and row['corrected_gate_rejected']
        for row in prior['historical_counterexamples'])
    references={INSTALLER_REVIEW:INSTALLER_REVIEW_SHA,RUNNER_REVIEW:RUNNER_REVIEW_SHA,
                NEGATIVE:NEGATIVE_SHA,ENTRYPOINT_REVIEW:ENTRYPOINT_REVIEW_SHA,
                PASSIVE_REVIEW:PASSIVE_REVIEW_SHA,COVERAGE:COVERAGE_SHA}
    assert all(sha(p)==h for p,h in references.items())
    installer_review=read(INSTALLER_REVIEW);runner_review=read(RUNNER_REVIEW)
    negative=read(NEGATIVE);entrypoint_review=read(ENTRYPOINT_REVIEW);passive_review=read(PASSIVE_REVIEW)
    assert installer_review['passed'] is True and len(installer_review['checks'])==4
    assert all(row['passed'] and row['accepted']==row['expected_accept'] for row in installer_review['checks'])
    assert installer_review['unaffected_prior_checks_reused']==23
    assert runner_review['passed'] is True and runner_review['changed_definitions']==['current_context']
    assert {'bootstrap_sources','inventory','load_serial','bind_manifest','worker'}<=set(runner_review['unchanged_definitions'])
    assert entrypoint_review['passed'] is True and entrypoint_review['entrypoint_called'] is False
    assert passive_review['passed'] is True and passive_review['test_bodies_reexecuted'] is False
    assert negative['passed'] is True and negative['accepted'] is False and negative['actual_focus_is_passing'] is False
    assert negative['rejection_reason']=='offline result is not complete/passing'
    superseded={str(INSTALLER),str(RUNNER)}
    assert {p for p,h in prior['dependency_sha256'].items() if sha(p)!=h}==superseded
    assert prior['dependency_sha256'][str(INSTALLER)]==installer_review['installer_old_sha256']
    assert sha(INSTALLER)==installer_review['installer_new_sha256']
    old_source=read(prior['actual_frozen_manifest_validated']['path'])['source_sha256']
    source=read(MANIFEST)['source_sha256']
    expected_delta=installer_review['exact_source_delta_from_previous_manifest']
    assert source==installer_review['source_sha256'] and set(old_source)==set(source)
    assert {p for p in source if source[p]!=old_source[p]}==set(expected_delta)
    assert all(old_source[p]==d['before'] and source[p]==d['after'] for p,d in expected_delta.items())
    dependencies={p:h for p,h in prior['dependency_sha256'].items() if p not in superseded}
    for reviewed in (installer_review,runner_review,entrypoint_review,passive_review):
        assert all(sha(p)==h for p,h in reviewed['dependency_sha256'].items())
        dependencies.update(reviewed['dependency_sha256'])
    for p,h in negative['dependency_sha256'].items():
        if p==str(RUNNER):
            assert h==prior['dependency_sha256'][p]
            assert any(d==h and sha(name)==h for name,d in runner_review['dependency_sha256'].items())
        else:
            assert sha(p)==h;dependencies[p]=h
    dependencies.update({str(p):h for p,h in references.items()})
    common_path=W/'.runtime/gen55_idle_reload_common_20261019.py'
    assert sha(common_path)==prior['dependency_sha256'][str(common_path)]
    spec=importlib.util.spec_from_file_location('reviewed_gen55_release_gate',common_path)
    common=importlib.util.module_from_spec(spec);spec.loader.exec_module(common)
    manifest,context,focus,evidence=common.validate_result(FOCUS,MANIFEST,MANIFEST_SHA,RUNNER,RUNNER_SHA,'focus')
    assert len(context['sources'])==1082 and len(context['scope'])==336 and len(context['case_identities'])==9568
    assert focus['summary']==dict(tests=1340,failures=0,errors=0,skipped=0)
    assert focus['worker_count']==7 and focus['test_scope_files']==32 and focus['full_regression_passed'] is False
    # The prior union failed. Only its two independently passing, unaffected
    # workers are retained; their 41 identities are rechecked against the new
    # frozen full scope and every actual imported byte is still identical.
    coverage=read(COVERAGE)
    assert coverage['passed'] is True and coverage['prior_union_passed'] is False
    assert coverage['full_regression_passed'] is False and coverage['retained_passed_cases']==41
    assert coverage['only_changed_source']==list(expected_delta) and coverage['unchanged_source_count']==1081
    assert coverage['all_other_source_bytes_identical'] is True
    assert coverage['final_installed_full_regression_must_execute_all_cases'] is True
    assert manifest['retained_focus_evidence']==dict(path=str(COVERAGE),sha256=COVERAGE_SHA)
    for name in ('prior_plan','prior_union_result'):
        ref=coverage[name];assert sha(ref['path'])==ref['sha256'];dependencies[ref['path']]=ref['sha256']
    old_plan=read(coverage['prior_plan']['path']);old_failed=read(coverage['prior_union_result']['path'])
    assert old_failed['passed'] is False and old_failed['summary']['failures']==2
    workspace=Path(manifest['workspace_root']).resolve(strict=True)
    runner_spec=importlib.util.spec_from_file_location('reviewed_gen55_inventory',RUNNER)
    inventory_runner=importlib.util.module_from_spec(runner_spec);runner_spec.loader.exec_module(inventory_runner)
    retained_cases=[];retained_workers=[]
    for kept in coverage['workers']:
        assert kept['all_actual_import_bytes_still_match'] is True and kept['changed_test_was_not_imported'] is True
        for name in ('worker_result','xml','import_provenance'):
            ref=kept[name];assert sha(ref['path'])==ref['sha256'];dependencies[ref['path']]=ref['sha256']
        worker_path=Path(kept['worker_result']['path']);row=read(worker_path)
        index=row['worker_index'];assert index in (0,1) and index not in retained_workers
        retained_workers.append(index)
        assert all(row[k] is True for k in ('passed','scope_complete','source_unchanged','direct_child_reaped','test_bodies_executed'))
        assert row['returncode']==0 and row['interrupted'] is False and row['mode']=='focus'
        assert not row['direct_child_cleanup_signals'] and not row['external_boundary_attempts'] and not row['foreign_or_inconsistent_imports']
        assert row['plan_sha256']==coverage['prior_plan']['sha256']
        assert row['test_scope']==old_plan['shards'][index]['scope']==[kept['test_file']]
        assert old_failed['worker_result_sha256'][str(worker_path)]==kept['worker_result']['sha256']
        expected=[c for c in context['case_identities'] if c[0]==kept['test_file']]
        summary,cases,_=inventory_runner.inventory(Path(kept['xml']['path']))
        assert summary==row['summary']==dict(tests=kept['passed_case_count'],failures=0,errors=0,skipped=0)
        assert Counter(map(tuple,cases))==Counter(map(tuple,expected))==Counter(map(tuple,kept['case_identities']))
        assert row['case_identities']==cases
        for name,key in (('pytest.xml','xml_sha256'),('pytest.log','log_sha256'),('module_provenance.json','module_provenance_sha256')):
            p=worker_path.parent/name;assert sha(p)==row[key];dependencies[str(p)]=row[key]
        provenance=read(kept['import_provenance']['path'])
        assert provenance and {d['path']:d['sha256'] for d in provenance.values()}==kept['imported_files_sha256']
        assert not any(str(workspace/p) in kept['imported_files_sha256'] for p in expected_delta)
        for filename,digest in kept['imported_files_sha256'].items():
            p=Path(filename);assert sha(p)==digest
            if workspace/'src' in p.parents:
                assert context['sources'][str(p.relative_to(workspace))]==digest
            else:
                assert context['generated_imports'].get(filename)==digest
            dependencies[filename]=digest
        required=sorted({p for test in row['test_scope'] for p in context['required_imports_by_test_file'].get(test,[])})
        assert row['required_changed_sources']==old_plan['shards'][index]['required_changed_sources']==required
        loaded={name:any(p['path']==str(workspace/name) and p['sha256']==context['sources'][name]
                    for p in provenance.values()) for name in context['required_changed_sources']}
        assert row['loaded_changed_sources']==loaded and all(loaded[name] for name in required)
        expected_bootstrap={'bootstrap/sitecustomize.py':context['guard_sha256'],
                            'bootstrap/gen42_import_audit.py':context['audit_sha256']}
        assert row['generated_bootstrap_sha256']==expected_bootstrap
        for name,digest in expected_bootstrap.items():
            p=worker_path.parent/name;assert sha(p)==digest;dependencies[str(p)]=digest
        boundary_file=worker_path.parent/'boundary_attempts.jsonl'
        assert not boundary_file.exists() or not boundary_file.read_text().splitlines()
        retained_cases.extend(cases)
    assert set(retained_workers)=={0,1} and len(retained_cases)==41
    _,fresh_cases,_=inventory_runner.inventory(FOCUS.parent/'pytest.xml')
    union=Counter(map(tuple,fresh_cases+retained_cases))
    assert len(union)==1381 and set(union.values())=={1}
    assert union==Counter(map(tuple,old_plan['expected_case_identities']))
    assert len(set(manifest['focus_scope'])|set(coverage['excluded_from_repeated_focus']))==34
    assert not (set(manifest['focus_scope'])&set(coverage['excluded_from_repeated_focus']))
    assert sha(FOCUS)==args.focus_sha256
    dependencies.update(evidence)
    dependencies.update({str(PRIOR):PRIOR_SHA,str(MANIFEST):MANIFEST_SHA,str(RUNNER):RUNNER_SHA,
                         str(FOCUS):args.focus_sha256,str(Path(__file__).resolve()):sha(__file__)})
    result=dict(kind='GEN55_final_runtime_helper_independent_review',passed=True,reviewed=True,
        dependency_sha256=dependencies,source_sha256=context['sources'],full_checks_retained=True,
        prior_unit_gate_review=dict(path=str(PRIOR),sha256=PRIOR_SHA,original_checks=27,
            unchanged_checks_reused=23,installer_delta_checks_revalidated=4,
            common_and_deployment_helpers_unchanged=True,synthetic_cases_not_real_regression=True),
        reviewed_helper_dependency_changes=dict(installer=str(INSTALLER_REVIEW),runner=str(RUNNER_REVIEW)),
        fixed_prior_gate_defects=prior['historical_counterexamples'],
        actual_failed_focus_rejection=dict(path=str(NEGATIVE),sha256=NEGATIVE_SHA,
            original_focus_passed=False,rejection_reason=negative['rejection_reason']),
        actual_focus_validation=dict(path=str(FOCUS),sha256=args.focus_sha256,summary=focus['summary'],
            worker_count=7,scope_files=32,all_worker_XML_log_bootstrap_provenance_revalidated=True),
        retained_previous_focus_validation=dict(evidence=str(COVERAGE),sha256=COVERAGE_SHA,
            cases=41,scope_files=2,original_union_passed=False,all_worker_XML_log_bootstrap_provenance_revalidated=True),
        combined_focused_inventory=dict(cases=1381,test_files=34,missing=0,duplicate=0,not_full_regression=True),
        recorded_entrypoint_review=dict(path=str(ENTRYPOINT_REVIEW),sha256=ENTRYPOINT_REVIEW_SHA),
        passive_observers_review=dict(path=str(PASSIVE_REVIEW),sha256=PASSIVE_REVIEW_SHA),
        final_frozen_scope=dict(manifest=str(MANIFEST),sha256=MANIFEST_SHA,source_files=1082,
            test_files=336,case_identities=9568),
        hardware_preservation_contract=dict(replaced_nodes=list(common.TARGETS),preserved_nodes=list(common.PRESERVE),
            gateway_respawn='original_parent_only',manual_idle_required=True,selection_generation=3,
            epoch_ns=1792373681250409915,control_revision=58),
        boundary_attempts=denied,test_bodies_reexecuted=False,full_regression_passed=False,
        installed=False,deployed=False,live_ROS_contacted=False,hardware_contacted=False,
        motion_commands_sent=False,temperature_queried=False,physical_grasp_verified=False)
    assert all(sha(p)==h for p,h in dependencies.items()) and not denied
    path=output/'result.json'
    with path.open('x') as f:
        json.dump(result,f,ensure_ascii=False,indent=2,allow_nan=False);f.write('\n');f.flush();os.fsync(f.fileno())
    assert read(path)==result
    print(json.dumps(dict(passed=True,result=str(path),sha256=sha(path),focus_summary=focus['summary']),ensure_ascii=False))

if __name__=='__main__':
    main()
