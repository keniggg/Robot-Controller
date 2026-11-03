"""Prepare/review/seal the exact tested missing-depth source as new GEN58 metadata.

Default --plan-directory only writes a reviewable plan/preview. Explicit --seal
requires that exact plan and a hash-bound helper review. No source installation,
ROS import, network/device access, test rerun, control mutation or reload occurs.
A seal proves current file bytes; fresh process/feedback/remote/scene acceptance
remains separate. Existing GEN57 release and all prior receipts stay unchanged.
"""
import argparse
import ast
from collections import Counter
from copy import deepcopy
import difflib
import hashlib
import json
import os
from pathlib import Path
import shutil
import time
import xml.etree.ElementTree as ET

W=Path(__file__).resolve().parents[1]
MOUNT=Path('/media/zhuyupei/SanStand')
E=MOUNT/'实验记录/Robot-Controller'
N=E/'ros_log/full_latest_gen41_restart_20261018T011222Z'
V=N/'verification'
SESSION=N/'gen58_resume_r1'
DESTINATION=V/'gen58_release_manifest.json'
CANDIDATE=W/'.runtime/missing_depth_prior_candidate_r2'
MANIFEST=V/'missing_depth_prior_freeze_r2/manifest_closed.json'
FULL=V/'missing_depth_prior_full_r2/result.json'
FOCUS=V/'missing_depth_prior_focus_r2/result.json'
COLLECTION=V/'missing_depth_prior_collect_r2/result.json'
INDEPENDENT=FULL.parent/'independent_verification.json'
BASE_RELEASE=V/'gen57_release_manifest.json'
PATCH=N/'gen57_resume_r3/actual_attempt_r1_geometry/missing_depth_prior_fix_r1'
REVIEW=PATCH/'review_r1.json'
CONTRACT_REVIEW=PATCH/'test_contract_review_r1.json'
CONTRACT_CHANGE=PATCH/'test_contract_change.json'
PRODUCTION='src/alicia_grasp_modes/src/alicia_grasp_modes/rgb_instance.py'
CONTRACT='src/alicia_grasp_modes/tests/test_observation_arrival_probable_foreground.py'
NEW='src/alicia_grasp_modes/tests/test_missing_depth_rgb_prior.py'
OLD_NAME='test_only_current_valid_measured_height_adds_probable_not_hard_foreground'
NEW_NAME='test_current_depth_proposals_and_missing_prior_never_add_hard_foreground'
PINNED={
 str(BASE_RELEASE):'257e407b60d7153368f6122c531844e0d16dea2756b201fc9a0279f4c1f8e192',
 str(MANIFEST):'f2e3cf4d1798a62875173a4657eba57c78eb50a65ff065b4c8e107aef98feb55',
 str(FULL):'8a332657019ad0790a53e9a71bd5459cafd51d956b759dcfe00a89fdd7a477a9',
 str(FOCUS):'f73f2adea2ccf87e6c451681e6adcfa134ab984f7295e0e6136c97505adf73b3',
 str(COLLECTION):'03c849e8daea5cf1760ea7e791296567642b908a24b2341953494fd658c7994f',
 str(INDEPENDENT):'1800bacf740ba45208966296de430ba663764e8ccf439c5bd11b04b8c996451e',
 str(REVIEW):'bc35515c23916eafacbf6def27a71c13711bb3a168d3a24a545a3dde106f0f6f',
 str(CONTRACT_REVIEW):'d24725aa85c21ee9dc10c62fbd950e5870907d45e1fe386e84c18440f2e83a44',
 str(CONTRACT_CHANGE):'000809809d1c705fdbc0ce93bc1564250e846c647102d3e9b7ef9b666eb4f69d'}


def require(value,reason):
    if not value:raise RuntimeError(reason)
def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def read(path):return json.loads(Path(path).read_bytes())
def canonical_sha(value):return hashlib.sha256(json.dumps(value,sort_keys=True,separators=(',',':'),ensure_ascii=False,allow_nan=False).encode()).hexdigest()
def reference(path):return dict(path=str(path),sha256=sha(path))

def storage(path):
    path=Path(path).resolve();require(MOUNT.is_mount() and not os.statvfs(MOUNT).f_flag & os.ST_RDONLY,'writable SanStand required')
    require(N in path.parents and path!=N,'output outside the SanStand experiment session')
    parent=path if path.exists() else path.parent
    while not parent.exists():parent=parent.parent
    require(parent.stat().st_dev==MOUNT.stat().st_dev,'output device differs from SanStand')
    config=read(E/'grasp_records/archive_config.json');policy=read(E/'maintenance/storage_control.json')
    require(config['upload_enabled'] is False and config['delete_enabled'] is False and policy['archive_policy']=='local_only','storage policy changed')
    require(shutil.disk_usage(E).free>=config['min_free_bytes'],'SanStand below configured minimum; no fallback')

def save(path,value):
    path=Path(path);storage(path)
    raw=(json.dumps(value,ensure_ascii=False,indent=2,allow_nan=False)+'\n').encode()
    with path.open('xb') as stream:stream.write(raw);stream.flush();os.fsync(stream.fileno())
    require(path.read_bytes()==raw,'output read-back differs')
    directory=os.open(str(path.parent),os.O_DIRECTORY)
    try:os.fsync(directory)
    finally:os.close(directory)

def inventory(path):
    root=ET.parse(path).getroot();cases=[];summary=dict.fromkeys(('tests','failures','errors','skipped'),0)
    for suite in root.iter('testsuite'):
        for key in summary:summary[key]+=int(suite.get(key,0))
    for case in root.iter('testcase'):cases.append((case.get('file',''),case.get('classname',''),case.get('name','')))
    require(summary['tests']==len(cases)==len(set(cases)),'missing or duplicated XML cases')
    for key,tag in (('failures','failure'),('errors','error'),('skipped','skipped')):
        require(summary[key]==sum(len(c.findall(tag)) for c in root.iter('testcase')),'XML outcome count differs')
    return summary,cases

def contract_ast(before,after):
    tree=ast.parse(before);actual=ast.parse(after)
    funcs=[n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name==OLD_NAME]
    require(len(funcs)==1,'old contract function missing');func=funcs[0];func.name=NEW_NAME
    comparisons=[n for n in ast.walk(func) if isinstance(n,ast.Compare) and isinstance(n.left,ast.Name)
        and n.left.id=='fault' and len(n.ops)==len(n.comparators)==1 and isinstance(n.ops[0],ast.Eq)
        and isinstance(n.comparators[0],ast.Constant) and n.comparators[0].value=='valid']
    require(len(comparisons)==1,'old expected-label condition ambiguous')
    comparisons[0].ops=[ast.In()]
    comparisons[0].comparators=[ast.Tuple(elts=[ast.Constant(value=x,kind=None) for x in ('valid','missing','nonfinite_depth')],ctx=ast.Load())]
    require(ast.dump(tree)==ast.dump(actual),'unreviewed historical test executable/assertion change')

def validate():
    storage(SESSION);evidence=dict(PINNED)
    def bind(path,digest=None):
        path=Path(path).resolve(strict=True);observed=sha(path)
        require(digest is None or observed==digest,'evidence changed: '+str(path))
        require(str(path) not in evidence or evidence[str(path)]==observed,'inconsistent evidence digest')
        evidence[str(path)]=observed;return read(path) if path.suffix=='.json' else path
    for path,digest in PINNED.items():bind(path,digest)
    manifest,full,independent,base=map(read,(MANIFEST,FULL,INDEPENDENT,BASE_RELEASE))
    sources=manifest['source_sha256'];context=manifest['context']
    require(manifest['kind']=='missing_depth_prior_frozen_offline_candidate' and Path(manifest['workspace_root'])==CANDIDATE,'unexpected tested snapshot')
    require(len(sources)==1089 and len(context['scope'])==348 and len(set(context['scope']))==348,'source/test-file scope differs')
    require(sources==context['sources']==full['source_sha256'],'source maps differ')
    require(all(not Path(p).is_absolute() and '..' not in Path(p).parts for p in sources),'unsafe source path')
    require(all(sha(W/p)==h==sha(CANDIDATE/p) for p,h in sources.items()),'live/tested source bytes differ')
    require(all(sha(CANDIDATE/p)==h for p,h in manifest['copied_input_sha256'].items()),'copied fixture/oracle/CAD input changed')
    require(len(base['source_sha256'])==1088 and base['generation']=='GEN57','historical baseline differs')
    old=base['source_sha256'];require(set(old)<=set(sources),'historical source omitted')
    require({p for p in old if old[p]!=sources[p]}=={PRODUCTION,CONTRACT} and set(sources)-set(old)=={NEW},'unreviewed source delta')
    for path,digest in context['compatibility_dependencies'].items():bind(path,digest)
    runner=Path(full['candidate_manifest_path']);require(runner==MANIFEST and full['candidate_manifest_sha256']==sha(MANIFEST),'full manifest binding differs')
    plan=bind(FULL.parent/'plan.json',full['plan_sha256']);runner_path=Path(plan['runner_path'])
    require(runner_path==W/'.runtime/verify_missing_depth_prior_20261020.py','unexpected new offline runner')
    bind(runner_path,full['runner_sha256']);require(manifest['runner_sha256']==sha(runner_path),'manifest runner differs')
    bind(W/'.runtime/check_missing_depth_prior_receipts_20261020.py',independent['verifier_sha256'])
    require(independent['passed'] is True and independent['full_result_sha256']==sha(FULL)
        and independent['manifest_sha256']==sha(MANIFEST),'independent receipt binding differs')
    require(independent['historical_case_count']==9766 and independent['supplemental_existing_cases']==106 and independent['new_cases']==21
        and independent['explicitly_renamed_parameter_cases']==10 and independent['historical_label_expectations_changed']==2,'independent case partition differs')
    production_review=read(REVIEW);test_review=read(CONTRACT_REVIEW)
    require(production_review['verdict']=='no_confirmed_blocking_defect_in_reviewed_patch' and not production_review['findings'],'production review did not accept this patch')
    require(production_review['source']['after_sha256']==sources[PRODUCTION] and production_review['source']['before_sha256']==old[PRODUCTION],'production review source differs')
    before_production=Path(production_review['source']['before_path']);bind(before_production,old[PRODUCTION])
    diff=''.join(difflib.unified_diff(before_production.read_text().splitlines(True),(W/PRODUCTION).read_text().splitlines(True),fromfile='before/rgb_instance.py',tofile='after/rgb_instance.py'))
    require(diff==production_review['diff']['text'] and hashlib.sha256(diff.encode()).hexdigest()==production_review['diff']['sha256'],'reviewed production diff differs')
    require(test_review['verdict']=='legitimate_contract_expectation_update_no_test_case_removed' and not test_review['findings'],'test contract review differs')
    require(test_review['before']['sha256']==old[CONTRACT] and test_review['after']['sha256']==sources[CONTRACT],'test review source differs')
    before_contract=Path(test_review['before']['path']);bind(before_contract,old[CONTRACT]);contract_ast(before_contract.read_text(),(W/CONTRACT).read_text())
    require(sum(row['changed'] for row in test_review['expected_label_map'])==2 and len(test_review['expected_label_map'])==10,'test contract label changes differ')
    historical_result=bind(Path(base['actual_regression']),base['actual_regression_sha256'])
    historical_xml=Path(base['actual_regression']).parent/'pytest.xml';bind(historical_xml,historical_result['xml_sha256'])
    summary,historical=inventory(historical_xml);require(summary==dict(tests=9766,failures=0,errors=0,skipped=0),'historical inventory differs')
    mapping=[];mapped=[]
    for row in historical:
        if row[0]==CONTRACT and row[2].startswith(OLD_NAME+'['):
            after=(row[0],row[1],row[2].replace(OLD_NAME,NEW_NAME,1));mapping.append(dict(before=list(row),after=list(after)));mapped.append(after)
        else:mapped.append(row)
    require(len(mapping)==10 and mapping==manifest['explicit_case_identity_mapping'],'explicit historical case mapping differs')
    collection=read(COLLECTION);newcases=[tuple(row) for row in collection['collected_case_identities']]
    require(collection['passed'] is True and collection['source_sha256']==sources and len(newcases)==len(set(newcases))==127,'collection differs')
    require(sum(c[0]==NEW for c in newcases)==21 and not set(newcases)&set(mapped),'new/old collection partition differs')
    expected=mapped+newcases;require(sorted(expected)==sorted(map(tuple,context['case_identities'])),'frozen case union differs')
    require(dict(Counter(c[0] for c in expected))==context['expected_counts'] and set(context['expected_counts'])==set(context['scope']),'frozen file counts differ')
    imports=0
    for result_path,mode,case_scope in ((FULL,'full',expected),(FOCUS,'focus',[c for c in expected if c[0] in manifest['focus_scope']])):
        result=read(result_path);folder=result_path.parent
        require(result['mode']==mode and all(result[k] is True for k in ('passed','scope_complete','source_unchanged','test_bodies_executed','combined_xml_matches_worker_results')),'incomplete passing '+mode+' evidence')
        require(result['source_sha256']==sources and result['candidate_manifest_sha256']==sha(MANIFEST) and result['runner_sha256']==sha(runner_path),'result source binding differs')
        require(not result['interrupted'] and result['launcher_error'] is None and not result['offline_child_cleanup_required'],'interrupted offline result')
        require(all(not result[k] for k in ('residual_offline_process_groups','external_boundary_attempts','foreign_or_inconsistent_imports','duplicate_case_identities','missing_case_identities','unexpected_case_identities','missing_worker_results')),'offline audit failed')
        require(result['worker_returncodes']==[0]*result['worker_count'] and result['worker_count']==6,'worker return/count differs')
        execution=bind(folder/'plan.json',result['plan_sha256']);bind(folder/'pytest.xml',result['xml_sha256'])
        actual_summary,actual=inventory(folder/'pytest.xml')
        require(actual_summary==result['summary']==dict(tests=len(case_scope),failures=0,errors=0,skipped=0) and sorted(actual)==sorted(case_scope),'executed case inventory differs')
        require(sorted(map(tuple,execution['expected_case_identities']))==sorted(case_scope) and execution['context']==context,'execution plan context differs')
        seen=set();union=[]
        for filename,digest in result['worker_result_sha256'].items():
            path=Path(filename);child=bind(path,digest);index=child['worker_index'];shard=execution['shards'][index];worker=path.parent
            require(index not in seen and index in range(6) and path==folder/('worker_%d'%index)/'result.json','worker identity differs');seen.add(index)
            require(all(child[k] is True for k in ('passed','scope_complete','source_unchanged','direct_child_reaped','test_bodies_executed')) and child['returncode']==0,'worker failed')
            require(not child['interrupted'] and not child['direct_child_cleanup_signals'] and not child['external_boundary_attempts'] and not child['foreign_or_inconsistent_imports'],'worker audit failed')
            require(child['plan_sha256']==result['plan_sha256'] and child['test_scope']==shard['scope'],'worker plan/scope differs')
            for name,key in (('pytest.xml','xml_sha256'),('pytest.log','log_sha256'),('module_provenance.json','module_provenance_sha256')):bind(worker/name,child[key])
            for name,digest in child['generated_bootstrap_sha256'].items():bind(worker/name,digest)
            boundary=worker/'boundary_attempts.jsonl';require(not boundary.exists() or not boundary.read_text().strip(),'appended boundary attempt')
            if boundary.exists():bind(boundary)
            child_summary,child_cases=inventory(worker/'pytest.xml');require(child_summary==child['summary'] and sorted(child_cases)==sorted(map(tuple,shard['case_identities'])),'worker cases differ');union+=child_cases
            provenance=read(worker/'module_provenance.json');imports+=len(provenance)
            for label,row in provenance.items():
                source=Path(row['path']);require(sha(source)==row['sha256'],'imported source changed')
                if CANDIDATE/'src' in source.parents:valid=sources.get(str(source.relative_to(CANDIDATE)))==row['sha256']
                elif W/'devel' in source.parents:valid=context['generated_imports'].get(str(source))==row['sha256']
                else:valid=any(row['path']==str(CANDIDATE/item['relative']) and row['sha256']==item['sha256'] and item['test'] in shard['scope'] for item in context['supplemental_executable_inputs'])
                require(valid,'foreign imported source');evidence[str(source)]=row['sha256']
        require(seen==set(range(6)) and len(union)==len(set(union)) and sorted(union)==sorted(case_scope),'worker case union differs')
        require(result['loaded_changed_sources_global'].get(PRODUCTION) is True,'changed production module not loaded')
    require(full['full_regression_passed'] is True and full['summary']['tests']==9893 and full['source_count']==1089 and full['test_scope_files']==348,'full scope summary differs')
    require(all(sha(p)==h for p,h in evidence.items()) and all(sha(W/p)==h==sha(CANDIDATE/p) for p,h in sources.items()),'input changed during validation')
    return dict(source_sha256=sources,evidence_sha256=evidence,source_map_sha256=canonical_sha(sources),
        test_summary=full['summary'],case_identity_mapping=mapping,full_elapsed_sec=full['elapsed_sec'],
        focus_elapsed_sec=read(FOCUS)['elapsed_sec'],audited_import_entries=imports,
        full_result_legacy_kind=full['kind'],source_installation='already_matches_tested_snapshot')

def release_payload(validation,extra):
    evidence=dict(validation['evidence_sha256']);evidence.update(extra)
    return dict(schema=1,generation='GEN58',kind='GEN58_missing_depth_prior_reviewed_source_release',passed=True,reviewed=True,installed=True,
        runtime_deployed=False,runtime_reloaded=False,physical_grasp_verified=False,source_installation=validation['source_installation'],
        installed_paths=[],source_files_written=False,installed_bytes_exact_tested_candidate=True,
        source_sha256=validation['source_sha256'],source_count=1089,test_scope_files=348,test_summary=validation['test_summary'],
        baseline_release=str(BASE_RELEASE),baseline_release_sha256=sha(BASE_RELEASE),baseline_generation='GEN57',
        actual_regression=str(FULL),actual_regression_sha256=sha(FULL),focus_regression=str(FOCUS),focus_regression_sha256=sha(FOCUS),
        candidate_manifest=str(MANIFEST),candidate_manifest_sha256=sha(MANIFEST),runner=str(W/'.runtime/verify_missing_depth_prior_20261020.py'),
        runner_sha256=sha(W/'.runtime/verify_missing_depth_prior_20261020.py'),worker_count=6,full_regression_passed=True,
        full_regression_location=str(CANDIDATE),historical_tests_current_contract=9766,supplemental_existing_tests=106,new_tests=21,
        historical_test_label_expectations_changed=2,historical_parameter_case_names_mapped=10,
        historical_case_identity_mapping=validation['case_identity_mapping'],
        independent_review=str(REVIEW),independent_review_sha256=sha(REVIEW),
        test_contract_review=str(CONTRACT_REVIEW),test_contract_review_sha256=sha(CONTRACT_REVIEW),
        independent_full_verification=str(INDEPENDENT),independent_full_verification_sha256=sha(INDEPENDENT),
        evidence_sha256=evidence,new_fresh_scene_robot_and_path_acceptance_required=True,
        new_runtime_process_feedback_camera_GUI_and_remote_acceptance_required=True,
        prior_full_result_kind_preserved=validation['full_result_legacy_kind'],
        provenance_note='The inherited result kind says GEN57 because of reused executor infrastructure. This new GEN58 release binds the distinct missing-depth manifest, runner SHA and exact 9893-case source scope; the old release is unchanged.',
        motion_commands_sent=False,temperature_queried=False,ROS_or_hardware_contacted=False,tests_rerun=False)

def plan(directory):
    directory=directory.resolve();storage(directory);require(SESSION in directory.parents,'plan must be under current GEN58 session')
    require(not directory.exists() and not DESTINATION.exists(),'plan/release already exists')
    validation=validate();directory.mkdir()
    preview=release_payload(validation,{})
    save(directory/'release_preview.json',preview)
    value=dict(kind='GEN58_file_only_release_seal_plan',created_ns=time.time_ns(),helper=reference(Path(__file__).resolve()),
        destination=str(DESTINATION),validation=validation,release_preview=reference(directory/'release_preview.json'),
        required_review_schema=dict(passed=True,helper_sha256=sha(__file__),plan_sha256='<SHA256 of this plan.json>',
            source_map_sha256=validation['source_map_sha256'],file_only=True,no_runtime_claims=True,reviewer='<independent reviewer>'),
        old_release_preserved=True,source_files_written=False,runtime_reloaded=False,tests_rerun=False)
    save(directory/'plan.json',value);print(json.dumps(dict(plan=str(directory/'plan.json'),sha256=sha(directory/'plan.json'),helper_sha256=sha(__file__),destination=str(DESTINATION)),ensure_ascii=False))

def seal(args):
    plan_path=args.plan.resolve(strict=True);review_path=args.review.resolve(strict=True)
    require(SESSION in plan_path.parents and SESSION in review_path.parents,'seal plan/review outside GEN58 session')
    require(sha(plan_path)==args.plan_sha256 and sha(review_path)==args.review_sha256,'seal plan/review digest changed')
    planned=read(plan_path);reviewed=read(review_path)
    require(planned['kind']=='GEN58_file_only_release_seal_plan' and planned['helper']==reference(Path(__file__).resolve())
        and planned['destination']==str(DESTINATION),'seal plan helper/destination differs')
    require(reviewed['passed'] is True and reviewed['helper_sha256']==sha(__file__) and reviewed['plan_sha256']==args.plan_sha256
        and reviewed['source_map_sha256']==planned['validation']['source_map_sha256'] and reviewed['file_only'] is True
        and reviewed['no_runtime_claims'] is True and isinstance(reviewed['reviewer'],str) and reviewed['reviewer'].strip(),'independent helper/plan review missing')
    require(not DESTINATION.exists(),'GEN58 release already exists; never overwrite')
    validation=validate();require(validation==planned['validation'],'source/evidence changed since plan')
    preview=Path(planned['release_preview']['path']);require(sha(preview)==planned['release_preview']['sha256'] and read(preview)==release_payload(validation,{}),'reviewable release preview changed')
    extra={str(plan_path):sha(plan_path),str(preview):sha(preview),str(review_path):sha(review_path),str(Path(__file__).resolve()):sha(__file__)}
    for path,digest in reviewed.get('dependency_sha256',{}).items():require(sha(path)==digest,'helper review dependency changed');extra[path]=digest
    payload=release_payload(validation,extra);payload.update(recorded_ns=time.time_ns(),seal_helper=str(Path(__file__).resolve()),
        seal_helper_sha256=sha(__file__),seal_plan=str(plan_path),seal_plan_sha256=sha(plan_path),
        seal_helper_review=str(review_path),seal_helper_review_sha256=sha(review_path))
    require(all(sha(W/p)==h==sha(CANDIDATE/p) for p,h in validation['source_sha256'].items()) and all(sha(p)==h for p,h in payload['evidence_sha256'].items()),'source/evidence changed before seal')
    save(DESTINATION,payload)
    print(json.dumps(dict(sealed=True,release=str(DESTINATION),sha256=sha(DESTINATION),summary=payload['test_summary'],runtime_deployed=False,source_installation=payload['source_installation']),ensure_ascii=False))

def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--plan-directory',type=Path)
    parser.add_argument('--seal',action='store_true');parser.add_argument('--plan',type=Path);parser.add_argument('--plan-sha256')
    parser.add_argument('--review',type=Path);parser.add_argument('--review-sha256');args=parser.parse_args()
    if args.seal:
        require(all((args.plan,args.plan_sha256,args.review,args.review_sha256)) and args.plan_directory is None,'explicit exact plan and helper review required');seal(args)
    else:require(args.plan_directory is not None,'plan-only default requires --plan-directory');plan(args.plan_directory)

if __name__=='__main__':main()
