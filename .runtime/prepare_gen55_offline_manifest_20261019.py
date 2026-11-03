"""Freeze reviewed GEN55 bytes, then bind guarded collection without executing tests."""
import argparse
from copy import deepcopy
import difflib
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import time

W = Path(__file__).resolve().parents[1]
D = W/'.runtime/gen55_candidate_r1'
V = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z/verification')
RUNNER = W/'.runtime/verify_gen55_parallel_offline_20261019.py'
PRIOR = V/'gen54_installed_candidate_r1/candidate_manifest.json'
PRIOR_SHA = 'aebaa01c523bfb2f8d8351f05b4e060ed6091424f4847cfaf0c76023729a6cfe'
F = 'src/alicia_flexible_grasp_supervisor/'
T = F+'tests/'
R = F+'src/alicia_flexible_grasp/robot/'
M = 'src/alicia_grasp_modes/tests/'
PRODUCTION = [R+'held_pregrasp_selection.py',R+'observation_path_guard.py',R+'contact_sequence_precheck.py']
NEW_MODULE = R+'finger_cad_separation.py'
NEW_TESTS = [T+'test_gen55_held_pregrasp_reuse.py',T+'test_gen55_finger_cad_separation.py',T+'test_gen55_contact_cad_replay.py']
NEW_FIXTURES = [T+'fixtures/gen55_g54_held_pregrasp_inputs.json.gz',T+'fixtures/gen55_finger_cad_actual_receipts.json',T+'fixtures/gen55_gen54_contact_curves.json']
SUPPLEMENTS = [T+'test_contact_linear_compensation.py',T+'test_observation_path_guard.py',
    T+'test_observation_path_velocity.py',T+'test_held_pregrasp_diagnostics.py']
FOCUS = [T+'test_gen44_task_gateway_path_chain.py',T+'test_contact_linear_compensation.py',
    T+'test_gen52_contact_terminal_collision_screen.py',T+'test_contact_height_window_proposals.py',
    T+'test_observation_path_guard.py',T+'test_observation_path_velocity.py',
    M+'test_gen49_contact_terminal_handoff.py',M+'test_gen52_station_fresh_fanin.py',
    M+'test_gen53_station_snapshot_retry.py',M+'test_gen41_pending_handoff_chain.py']
FOCUS += [T+name+'.py' for name in ('test_held_pregrasp_selection','test_held_pregrasp_diagnostics',
    'test_gen51_shared_held_guide_fallback','test_gen45_fresh_held_path_chain',
    'test_gen14_held_approach_precheck','test_contact_pregrasp_compute_guard')]
FK_RESULT = V/'gen55_held_pregrasp_reuse_r1/result.json'
FK_RESULT_SHA = '72be42134825860b4748057ddfcf095534284d3b4c862b6a36b25b068a73145f'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read(path):
    return json.loads(Path(path).read_bytes())


def save(path,value):
    with Path(path).open('x') as stream:
        json.dump(value,stream,ensure_ascii=False,sort_keys=True,indent=2,allow_nan=False)
        stream.write('\n');stream.flush();os.fsync(stream.fileno())


def reference(path):
    path=Path(path).resolve(strict=True)
    return dict(path=str(path),sha256=sha(path))


def runner_module():
    spec=importlib.util.spec_from_file_location('gen55_offline_manifest_contract',RUNNER)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return module


def retained_focus_coverage(prior_path,sources,runner):
    """Retain two independently successful shards without relabeling the failed union."""
    prior_path=Path(prior_path).resolve(strict=True);prior=read(prior_path)
    assert prior['mode']=='focus' and prior['passed'] is False
    assert prior['scope_complete'] and prior['source_unchanged']
    assert prior['summary']==dict(tests=1381,failures=2,errors=0,skipped=0)
    assert not prior['external_boundary_attempts'] and not prior['foreign_or_inconsistent_imports']
    assert not prior['offline_child_cleanup_required'] and not prior['residual_offline_process_groups']
    old_sources=prior['source_sha256'];assert set(old_sources)==set(sources)
    changed={path for path in sources if old_sources[path]!=sources[path]}
    assert changed=={runner.GEN55_REVIEWED_TERMINAL_TEST}, 'A production/input change requires fresh relevant focused execution.'
    plan_path=prior_path.parent/'plan.json';assert sha(plan_path)==prior['plan_sha256']
    plan=read(plan_path);proofs=[];excluded=[]
    known=[(0,M+'test_gen41_pending_handoff_chain.py',14),(1,M+'test_gen53_station_snapshot_retry.py',27)]
    for index,test,count in known:
        output=prior_path.parent/('worker_%d'%index);result_path=output/'result.json';row=read(result_path)
        assert prior['worker_result_sha256'][str(result_path)]==sha(result_path)
        assert row['passed'] and row['returncode']==0 and row['direct_child_reaped'] and not row['interrupted']
        assert not row['direct_child_cleanup_signals'] and row['source_unchanged'] and row['scope_complete']
        assert row['test_scope']==plan['shards'][index]['scope']==[test]
        assert row['summary']==dict(tests=count,failures=0,errors=0,skipped=0)
        assert not row['external_boundary_attempts'] and not row['foreign_or_inconsistent_imports']
        for name,key in [('pytest.xml','xml_sha256'),('pytest.log','log_sha256'),('module_provenance.json','module_provenance_sha256')]:
            assert sha(output/name)==row[key]
        assert all(sha(output/path)==digest for path,digest in row['generated_bootstrap_sha256'].items())
        boundary=output/'boundary_attempts.jsonl';assert not boundary.exists() or not boundary.read_text().splitlines()
        summary,cases,_=runner.inventory(output/'pytest.xml')
        assert summary==row['summary'] and cases==row['case_identities']
        assert sorted(map(tuple,cases))==sorted(map(tuple,plan['shards'][index]['case_identities']))
        provenance=read(output/'module_provenance.json')
        assert not any(r['path']==str(D/runner.GEN55_REVIEWED_TERMINAL_TEST) for r in provenance.values())
        assert all(sha(r['path'])==r['sha256'] for r in provenance.values())
        proofs.append(dict(test_file=test,passed_case_count=count,case_identities=cases,
            worker_result=reference(result_path),xml=reference(output/'pytest.xml'),
            import_provenance=reference(output/'module_provenance.json'),
            imported_files_sha256={r['path']:r['sha256'] for r in provenance.values()},
            changed_test_was_not_imported=True,all_actual_import_bytes_still_match=True))
        excluded.append(test)
    return dict(passed=True,kind='GEN55_prior_successful_shard_coverage_only',prior_union_result=reference(prior_path),
        prior_union_passed=False,prior_plan=reference(plan_path),retained_passed_cases=41,workers=proofs,
        only_changed_source=list(changed),all_other_source_bytes_identical=True,
        unchanged_source_count=len(sources)-1,excluded_from_repeated_focus=excluded,
        final_installed_full_regression_must_execute_all_cases=True,full_regression_passed=False),excluded


def freeze(args,runner):
    assert sha(PRIOR)==PRIOR_SHA and sha(FK_RESULT)==FK_RESULT_SHA
    manifest=deepcopy(read(PRIOR))
    original=manifest['source_sha256']
    assert len(original)==1075 and all(sha(W/path)==digest for path,digest in original.items()), 'Production must still match GEN54 at isolated freeze.'
    test_reviews=[reference(path) for path in args.test_review]
    cad_reference=reference(args.cad_review);pipeline_reference=reference(args.pipeline_review)
    reviews=test_reviews+[cad_reference,pipeline_reference]
    old_reviews=[read(row['path']) for row in test_reviews]
    cad_review,pipeline_review=read(cad_reference['path']),read(pipeline_reference['path'])
    assert all(row['passed'] is True for row in old_reviews+[cad_review,pipeline_review])
    assert len({row['path'] for row in old_reviews})==len(old_reviews)
    for old_review in old_reviews:
        assert old_review['path'] in runner.GEN55_TEST_REVIEW_CONTRACTS
        assert old_review['baseline_sha256']==original[old_review['path']]
        assert old_review['candidate_sha256']==sha(D/old_review['path'])
    for mapping in ('owned_source_sha256','reviewed_related_sources'):
        assert all(sha(D/path)==digest for path,digest in cad_review[mapping].items())
    for asset in cad_review['required_CAD_assets']:
        assert sha(D/asset['path'])==sha(W/asset['path'])==asset['sha256']
    assert pipeline_review['test']==reference(D/NEW_TESTS[2])
    assert pipeline_review['fixture']==reference(D/NEW_FIXTURES[2])
    assert all(sha(D/R/name)==digest for name,digest in pipeline_review['kernel_sha256'].items())
    permitted=set(PRODUCTION+[row['path'] for row in old_reviews])
    changed={path for path,digest in original.items() if sha(D/path)!=digest}
    assert changed==permitted, ('Unexpected or missing GEN55 changes',sorted(changed^permitted))
    sources={path:sha(D/path) for path in original}
    sources.update({path:sha(D/path) for path in [NEW_MODULE]+NEW_TESTS+NEW_FIXTURES})
    assert len(sources)==1082
    for path in PRODUCTION:
        assert path not in manifest['allowed_production_paths']
        manifest['allowed_production_paths'].append(path)
        manifest['production_source_changes'].append(dict(path=path,
            baseline_sha256=original[path],candidate_sha256=sources[path]))
    manifest['new_production_sources']=[dict(path=NEW_MODULE,sha256=sources[NEW_MODULE],review_evidence=cad_reference)]
    manifest['approved_gen55_test_changes']=[dict(path=row['path'],baseline_sha256=row['baseline_sha256'],
        candidate_sha256=row['candidate_sha256'],review_evidence=ref) for row,ref in zip(old_reviews,test_reviews)]
    manifest['additional_review_evidence']=[pipeline_reference,reference(FK_RESULT)]
    for row in manifest['new_tests']+manifest['supplemental_existing_tests']:
        row['sha256']=sources[row['path']]
        for key in ('case_identities','expected_count','collection_evidence'):row.pop(key,None)
    manifest['new_tests'] += [dict(path=path,sha256=sources[path]) for path in NEW_TESTS]
    known={row['path'] for row in manifest['supplemental_existing_tests']}
    for path in SUPPLEMENTS:
        assert path in original and path not in known
        manifest['supplemental_existing_tests'].append(dict(path=path,sha256=sources[path]))
    manifest['new_fixture_sources'] += [dict(path=path,sha256=sources[path]) for path in NEW_FIXTURES]
    manifest['focus_scope']=list(dict.fromkeys(manifest['focus_scope']+FOCUS+NEW_TESTS))
    manifest['required_changed_sources']=list(dict.fromkeys(manifest['required_changed_sources']+PRODUCTION+[NEW_MODULE]))
    by_test=manifest['required_imports_by_test_file']
    by_test[NEW_TESTS[0]]=[PRODUCTION[0]]
    by_test[NEW_TESTS[1]]=[NEW_MODULE]
    by_test[NEW_TESTS[2]]=[NEW_MODULE,PRODUCTION[1],PRODUCTION[2]]
    by_test[runner.GEN55_REVIEWED_TEST]=[NEW_MODULE,PRODUCTION[1]]
    if runner.GEN55_REVIEWED_TERMINAL_TEST in {row['path'] for row in old_reviews}:
        by_test[runner.GEN55_REVIEWED_TERMINAL_TEST]=[NEW_MODULE,PRODUCTION[1],PRODUCTION[2]]
    by_test[T+'test_observation_path_guard.py']=[PRODUCTION[1]]
    manifest.update(kind='GEN55_frozen_offline_candidate',workspace_root=str(D),source_sha256=sources,
        full_scope_ready=False,stage='isolated_reviewed_bytes_before_guarded_collection',created_ns=time.time_ns(),
        previous_generation_manifest=reference(PRIOR),runner_sha256=sha(RUNNER),
        freeze_helper_sha256=sha(__file__),production_modified=False,deployed=False)
    for key in ('installation_evidence','isolated_manifest'):manifest.pop(key,None)
    output=args.output.resolve();serial=runner.load_serial();serial.storage(output)
    assert not output.exists();output.mkdir()
    if args.prior_focus:
        coverage,exclude=retained_focus_coverage(args.prior_focus,sources,runner)
        save(output/'prior_focus_coverage.json',coverage)
        manifest['retained_focus_evidence']=reference(output/'prior_focus_coverage.json')
        manifest['additional_review_evidence'].append(manifest['retained_focus_evidence'])
        manifest['focus_scope']=[path for path in manifest['focus_scope'] if path not in exclude]
    diff=''.join(''.join(difflib.unified_diff((W/path).read_text().splitlines(True),(D/path).read_text().splitlines(True),
        fromfile='GEN54/'+path,tofile='GEN55/'+path)) for path in sorted(changed))
    with (output/'reviewed_changes.diff').open('x') as stream:stream.write(diff)
    save(output/'candidate_manifest.json',manifest)
    runner.bind_manifest(output/'candidate_manifest.json',sha(output/'candidate_manifest.json'))
    context=runner.current_context(serial)
    assert set(manifest['focus_scope']) <= set(context['scope']), 'Every focused file must be included before collection.'
    save(output/'preparation.json',dict(passed=True,manifest=reference(output/'candidate_manifest.json'),
        source_count=len(sources),changed_prior_sources=sorted(changed),new_sources=[NEW_MODULE]+NEW_TESTS+NEW_FIXTURES,
        reviewed_changes=reference(output/'reviewed_changes.diff'),scope_files=len(context['scope']),
        collected=False,tests_executed=False,full_regression_passed=False,production_modified=False,
        dependencies=[reference(PRIOR),reference(RUNNER),reference(__file__)]+reviews+[reference(FK_RESULT)]))
    return output/'candidate_manifest.json'


def bind_collection(args,runner):
    assert sha(args.manifest)==args.manifest_sha256 and sha(args.collection)==args.collection_sha256
    manifest=deepcopy(read(args.manifest));collection=read(args.collection)
    assert collection['passed'] and collection['mode']=='collect' and collection['source_unchanged']
    assert collection['candidate_manifest_sha256']==args.manifest_sha256
    assert collection['source_sha256']==manifest['source_sha256']
    assert collection['runner_sha256']==sha(RUNNER)==manifest['runner_sha256']
    assert not collection['test_bodies_executed'] and not collection['full_regression_passed']
    assert not collection['external_boundary_attempts'] and not collection['foreign_or_inconsistent_imports']
    rows=collection['collected_case_identities'];scope=manifest['new_tests']+manifest['supplemental_existing_tests']
    assert len(rows)==len(set(map(tuple,rows)))
    assert set(c[0] for c in rows)=={row['path'] for row in scope}
    for row in scope:
        cases=[c for c in rows if c[0]==row['path']]
        assert cases
        row.update(case_identities=cases,expected_count=len(cases),collection_evidence=reference(args.collection))
    manifest.update(full_scope_ready=True,stage='isolated_reviewed_and_exact_collected',created_ns=time.time_ns(),
        precollection_manifest=reference(args.manifest))
    output=args.output.resolve();serial=runner.load_serial();serial.storage(output)
    assert not output.exists();output.mkdir();save(output/'candidate_manifest.json',manifest)
    runner.bind_manifest(output/'candidate_manifest.json',sha(output/'candidate_manifest.json'))
    context=runner.current_context(serial)
    prior_xml=runner.BALANCE_HISTORY.parent/'pytest.xml';_,prior_cases,_=runner.inventory(prior_xml)
    expected=context['case_identities'];assert set(map(tuple,prior_cases))<=set(map(tuple,expected))
    save(output/'preparation.json',dict(passed=True,manifest=reference(output/'candidate_manifest.json'),
        source_count=len(context['sources']),expected_file_count=len(context['scope']),
        expected_case_count=len(expected),retained_gen54_cases=len(prior_cases),
        added_case_count=len(expected)-len(prior_cases),historical_case_identities_preserved=True,
        collection=reference(args.collection),runner=reference(RUNNER),helper=reference(__file__),
        tests_executed=False,full_regression_passed=False,production_modified=False))
    return output/'candidate_manifest.json'


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    sub=parser.add_subparsers(dest='mode',required=True)
    first=sub.add_parser('freeze');first.add_argument('--output',type=Path,required=True)
    first.add_argument('--test-review',type=Path,action='append',required=True)
    for name in ('cad-review','pipeline-review'):first.add_argument('--'+name,type=Path,required=True)
    first.add_argument('--prior-focus',type=Path)
    second=sub.add_parser('bind-collection');second.add_argument('--output',type=Path,required=True)
    for name in ('manifest','collection'):
        second.add_argument('--'+name,type=Path,required=True);second.add_argument('--'+name+'-sha256',required=True)
    args=parser.parse_args();runner=runner_module()
    result=freeze(args,runner) if args.mode=='freeze' else bind_collection(args,runner)
    print(json.dumps(dict(passed=True,mode=args.mode,manifest=str(result),manifest_sha256=sha(result)),ensure_ascii=False))


if __name__=='__main__':
    main()
