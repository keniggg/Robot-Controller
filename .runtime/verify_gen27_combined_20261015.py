"""Verify the explicit GEN27 patch against GEN26 and rerun the complete combined scope offline."""
import argparse,hashlib,json,os,shutil,subprocess,sys,time
from pathlib import Path
import xml.etree.ElementTree as ET
W=Path(__file__).resolve().parents[1]
M=Path('/media/zhuyupei/SanStand')
E=M/'实验记录/Robot-Controller'
N=E/'ros_log/full_latest_gen22_restart_20261015T003700Z'
PRIOR_RECORD=E/'grasp_records/20261015T051744.120111Z_0ffc956cd2e54398a35ebff62282e999'
BASE=N/'verification/gen26_release_manifest.json'
BASE_SHA='db336d8a2f2ea3c8f8ccfa35f3735fc34fd62c47f3b84abf85c03e1a00a52bd0'
FULL=BASE
FULL_SHA=BASE_SHA
SPEC=W/'.runtime/gen27_verification_spec.json'
# Explicit coordinator scope. Discovery may add tests, never production paths.
PRODUCTION_ALLOWLIST={
    'src/alicia_grasp_modes/scripts/unknown_tabletop_perception.py',
    'src/alicia_grasp_modes/src/alicia_grasp_modes/unknown_pending_identity.py',
    'src/alicia_grasp_modes/src/alicia_grasp_modes/initial_preflight_recovery.py',
    'src/alicia_grasp_modes/src/alicia_grasp_modes/initial_preflight_task.py',
}
NEW_RESOURCE_ALLOWLIST={
    'src/alicia_grasp_modes/src/alicia_grasp_modes/feedback_recovery_completion.py',
}
TEST_ROOTS=('src/alicia_grasp_modes/tests','src/alicia_flexible_grasp_supervisor/tests')
PROVENANCE=(
    'verification/gen26_combined_r2/result.json',
    'verification/gen26_final_independent_review/review.json',
    'verification/gen26_release_manifest.json',
    'gen26_deployment/result.json',
    'gen26_resume_request.json',
    'gen26_resume_request_result.json',
    'gen26_attempt_1_scene/preflight.json',
    'gen26_attempt_1_scene/current_scene.png',
    'gen26_attempt_1_scene/preflight_joint_reference.json',
    'gen26_attempt_1_scene/readiness/result.json',
    'gen26_attempt_1_scene/scene_clear_receipt.json',
    'gen26_attempt_1_scene/operator_observation.json',
    'gen26_attempt_1_after_manual/result.json',
    'verification/gen26_attempt1_initial_support_prefix/detector_status.json',
    'verification/gen26_attempt1_initial_support_prefix/receipt.json',
    'verification/gen26_attempt1_initial_support_prefix/prefix_64MiB.bag',
    'verification/gen27_recorded_feedback_gap_analysis/findings.json',
    'verification/gen27_recorded_feedback_gap_analysis/accepted_feedback.json',
)
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def read(p):return json.loads(Path(p).read_text())
def new_tests():
    return sorted(str(p.relative_to(W)) for root in TEST_ROOTS
                  for p in (W/root).glob('test_gen27*.py'))
def refreshed_spec():
    assert sha(BASE)==BASE_SHA
    base=read(BASE);sources=base['source_sha256']
    changed=[name for name,digest in sources.items() if sha(W/name)!=digest]
    production=sorted(name for name in changed if name in PRODUCTION_ALLOWLIST)
    tests=sorted(name for name in changed if '/tests/test_' in name and name.endswith('.py'))
    assert set(changed)==set(production+tests), 'Unapproved baseline changes: '+repr(sorted(set(changed)-set(production+tests)))
    added_tests=new_tests()
    assert not set(added_tests)&set(sources)
    resources=sorted(name for name in NEW_RESOURCE_ALLOWLIST if (W/name).is_file())
    assert not set(resources)&set(sources)
    provenance={str(N/name):sha(N/name) for name in PROVENANCE}
    prefix=read(N/'verification/gen26_attempt1_initial_support_prefix/receipt.json')
    assert prefix['prefix_sha256']==provenance[str(N/'verification/gen26_attempt1_initial_support_prefix/prefix_64MiB.bag')]
    assert prefix['original_modified'] is False and prefix['original_reindexed'] is False
    analysis=read(N/'verification/gen27_recorded_feedback_gap_analysis/findings.json')
    script=W/'.runtime/analyze_gen26_recorded_feedback_gap.py'
    assert sha(script)==analysis['script_sha256']
    assert all(provenance.get(path)==digest for path,digest in analysis['source_sha256'].items())
    provenance[str(script)]=analysis['script_sha256']
    baseline_directory=W/'.runtime/gen27_baseline_source'
    baseline_manifest=baseline_directory/'manifest.json'
    snapshots=read(baseline_manifest)
    assert set(snapshots)==PRODUCTION_ALLOWLIST|{'new_resources_absent_at_baseline'}
    assert set(snapshots['new_resources_absent_at_baseline'])==NEW_RESOURCE_ALLOWLIST
    for name in PRODUCTION_ALLOWLIST:
        assert snapshots[name]==sources[name] and sha(baseline_directory/name)==sources[name]
        provenance[str(baseline_directory/name)]=sources[name]
    provenance[str(baseline_manifest)]=sha(baseline_manifest)
    closed_names=('manifest.json','command_result.json','recorder_outcome.json',
                  'user_request_runner_exit.json','runner.log','authorization.json')
    for name in closed_names:
        provenance[str(PRIOR_RECORD/name)]=sha(PRIOR_RECORD/name)
    manifest=read(PRIOR_RECORD/'manifest.json')
    command=read(PRIOR_RECORD/'command_result.json')
    recorder=read(PRIOR_RECORD/'recorder_outcome.json')
    runner=read(PRIOR_RECORD/'user_request_runner_exit.json')
    assert manifest['state']=='complete' and manifest['recorder_closed'] is True
    assert manifest['result']==command['result']=='failure'
    assert manifest['failure_reason']==command['failure_reason']
    assert command['failure_reason'].startswith('NO_FRESH_MODE_PREVIEW ')
    assert command['runner_finished'] is True and 'runner.log' in command['closed_logs']
    assert recorder['bag_closed'] is True and recorder['active_seen'] is False
    assert recorder['interrupted'] is False and recorder['stop_reason']=='duration_elapsed'
    assert runner['runner_returncode']==command['evidence']['runner_returncode']==3
    assert runner['physical_success']=='unverified'
    assert command['failure_reason'] in (PRIOR_RECORD/'runner.log').read_text()
    files={row['path']:row for row in manifest['files']}
    for name in closed_names:
        if name in files:
            assert files[name]['sha256']==provenance[str(PRIOR_RECORD/name)]
    bag=files['keyframes.bag'];stat=(PRIOR_RECORD/'keyframes.bag').stat()
    assert stat.st_size==bag['size_bytes'] and stat.st_ino==prefix['source_inode']
    bag_metadata=dict(path=str(PRIOR_RECORD/'keyframes.bag'),size_bytes=bag['size_bytes'],
        manifest_sha256=bag['sha256'],inode=stat.st_ino,device=stat.st_dev,mtime_ns=stat.st_mtime_ns,
        verification='Original SHA256 recorded by completed manifest; GEN27 checks current size/inode only, without rehashing the full bag.')
    return dict(kind='gen27_feedback_gap_timely_frame_completion_scope',
        baseline=str(BASE),baseline_sha256=BASE_SHA,
        production=production,changed_tests=tests,new_tests=added_tests,
        new_resources=resources,additional_tests=[],evidence_sha256=provenance,
        prior_attempt_record=str(PRIOR_RECORD),prior_attempt_bag_metadata=bag_metadata,
        prior_attempt_evidence_note='GEN26 attempt is complete/failure with recorder_closed=true and active_seen=false. Static prefix, final small receipts and runner log are SHA-bound; original full bag hash is retained from its closed manifest, not recomputed by GEN27.',
        scope_note='Candidate inventory only; no passing test, review, release or deployment is claimed.')
def refresh_spec():
    candidate=refreshed_spec()
    # Source/config only stays in the project. Test artifacts remain external.
    tmp=SPEC.with_suffix('.json.tmp')
    with tmp.open('x') as f:
        json.dump(candidate,f,ensure_ascii=False,indent=2);f.write('\n');f.flush();os.fsync(f.fileno())
    os.replace(str(tmp),str(SPEC))
    print(json.dumps(dict(spec=str(SPEC),sha256=sha(SPEC),production=candidate['production'],
        changed_tests=candidate['changed_tests'],new_tests=candidate['new_tests'],
        new_resources=candidate['new_resources'],passed_claimed=False),ensure_ascii=False),flush=True)
def guard(out):
    assert M.is_mount() and not os.statvfs(M).f_flag & os.ST_RDONLY
    assert N/'verification' in out.resolve().parents and out.parent.stat().st_dev==M.stat().st_dev
    c=read(E/'grasp_records/archive_config.json')
    assert c['upload_enabled'] is False and c['delete_enabled'] is False
    assert read(E/'maintenance/storage_control.json')['archive_policy']=='local_only'
    assert shutil.disk_usage(E).free>=c['min_free_bytes']
def save(p,v):
    guard(p)
    with Path(p).open('x') as f:json.dump(v,f,ensure_ascii=False,indent=2);f.write('\n');f.flush();os.fsync(f.fileno())
def inventory(spec):
    assert sha(BASE)==BASE_SHA and sha(FULL)==FULL_SHA
    base,full=read(BASE),read(FULL)
    assert base['passed'] and full['passed']
    assert base['source_count']==960 and base['test_scope_files']==226 and base['test_summary']['tests']==6993
    assert spec['baseline']==str(BASE) and spec['baseline_sha256']==BASE_SHA
    assert set(spec['production'])<=PRODUCTION_ALLOWLIST
    assert set(spec.get('new_resources',[]))<=NEW_RESOURCE_ALLOWLIST
    assert spec['new_tests']==new_tests() and spec['new_tests'], 'New test scope changed or absent; refresh the spec.'
    assert spec['production'], 'GEN27 production change is not ready.'
    actual=refreshed_spec()
    for key in ('production','changed_tests','new_tests','new_resources','additional_tests','evidence_sha256',
                'prior_attempt_record','prior_attempt_bag_metadata','prior_attempt_evidence_note'):
        assert spec[key]==actual[key], key+' changed; refresh and independently review the spec.'
    changed=spec['production']+spec['changed_tests'];new=spec['new_tests']+spec.get('new_resources',[])
    assert len(set(changed+new))==len(changed+new)
    sources=dict(base['source_sha256'])
    assert set(changed)<=set(sources) and not set(new)&set(sources)
    for name,digest in sources.items():
        assert not Path(name).is_absolute() and '..' not in Path(name).parts
        if name not in changed:assert sha(W/name)==digest,name
    scope=list(dict.fromkeys(full['test_scope']+base['test_scope']+spec['changed_tests']+spec['new_tests']+spec.get('additional_tests',[])))
    for name in changed+new+scope:
        assert not Path(name).is_absolute() and '..' not in Path(name).parts
        sources[name]=sha(W/name)
    assert set(full['test_scope'])<=set(scope) and set(base['test_scope'])<=set(scope)
    assert all('/tests/test_' in name for name in scope)
    return sources,scope

def xml_summary(p):
    root=ET.parse(p).getroot()
    return {k:sum(int(s.get(k,0)) for s in root.iter('testsuite')) for k in ('tests','failures','errors','skipped')}
def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--spec',type=Path,default=SPEC)
    ap.add_argument('--output',type=Path)
    ap.add_argument('--refresh-spec',action='store_true',help='Snapshot only approved source changes and all new GEN27 tests into project spec; does not run tests.')
    a=ap.parse_args()
    if a.refresh_spec:
        assert a.output is None and a.spec.resolve()==SPEC
        refresh_spec();return 0
    if a.output is None:ap.error('--output is required for the offline combined run')
    out=a.output.resolve();guard(out);assert not out.exists()
    spec=read(a.spec);sources,scope=inventory(spec);spec_sha=sha(a.spec);runner_sha=sha(__file__)
    out.mkdir();(out/'tmp').mkdir();(out/'ros').mkdir()
    patch={'baseline':str(BASE),'baseline_sha256':BASE_SHA,'spec':spec,'spec_path':str(a.spec.resolve()),'spec_sha256':spec_sha,'files':{name:{'baseline_sha256':read(BASE)['source_sha256'].get(name),'sha256':sources[name]} for name in spec['production']+spec['changed_tests']+spec['new_tests']+spec.get('new_resources',[])}}
    save(out/'patch_manifest.json',patch)
    env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',PYTHONNOUSERSITE='1',PYTEST_DISABLE_PLUGIN_AUTOLOAD='1',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',ROS_MASTER_URI='http://127.0.0.1:1',TMPDIR=str(out/'tmp'),ROS_LOG_DIR=str(out/'ros'))
    env['PYTHONPATH']=':'.join(str(W/p) for p in ('src/alicia_flexible_grasp_supervisor/tests','src/alicia_grasp_modes/tests','src/alicia_grasp_modes/src','src/alicia_flexible_grasp_supervisor/src','devel/lib/python3/dist-packages'))+':'+env.get('PYTHONPATH','')
    env['ROS_PACKAGE_PATH']=str(W/'src')+':'+env.get('ROS_PACKAGE_PATH','')
    cmd=[sys.executable,'-m','pytest','-q',*scope,'-p','no:cacheprovider','--tb=short','--basetemp='+str(out/'pytest_tmp'),'--junitxml='+str(out/'pytest.xml'),'-o','junit_family=legacy']
    print(json.dumps(dict(started=True,source_count=len(sources),test_files=len(scope),output=str(out))),flush=True)
    started=time.monotonic()
    with (out/'pytest.log').open('x') as log:run=subprocess.run(cmd,cwd=W,env=env,stdout=log,stderr=subprocess.STDOUT)
    summary=xml_summary(out/'pytest.xml');counts=dict.fromkeys(scope,0);unassigned=[]
    for case in ET.parse(out/'pytest.xml').getroot().iter('testcase'):
        name=case.get('file','')
        if name in counts:counts[name]+=1
        else:unassigned.append(dict(file=name,classname=case.get('classname'),name=case.get('name')))
    changed=[name for name,digest in sources.items() if sha(W/name)!=digest]
    unchanged=not changed and sha(a.spec)==spec_sha and sha(__file__)==runner_sha and sha(BASE)==BASE_SHA and sha(FULL)==FULL_SHA
    unchanged=unchanged and new_tests()==spec['new_tests'] and all(sha(p)==digest for p,digest in spec['evidence_sha256'].items())
    complete=not unassigned and all(n>0 for n in counts.values()) and sum(counts.values())==summary['tests']
    passed=run.returncode==0 and unchanged and complete and summary['tests']>=read(FULL)['test_summary']['tests'] and all(summary[k]==0 for k in ('failures','errors','skipped'))
    result=dict(passed=passed,summary=summary,returncode=run.returncode,elapsed_sec=time.monotonic()-started,source_sha256=sources,source_count=len(sources),source_unchanged=unchanged,changed_source_paths=changed,test_scope=scope,test_scope_files=len(scope),test_case_counts_by_file=counts,scope_complete=complete,unassigned_testcases=unassigned,command=cmd,baseline=str(BASE),baseline_sha256=BASE_SHA,full_scope_baseline=str(FULL),full_scope_baseline_sha256=FULL_SHA,patch=str(out/'patch_manifest.json'),patch_sha256=sha(out/'patch_manifest.json'),log_sha256=sha(out/'pytest.log'),xml_sha256=sha(out/'pytest.xml'),runner_sha256=runner_sha,regression_scope='full_combined',production_worktree_tested=True,live_ros_contacted=False,motion_commands_sent=False,temperature_queried=False)
    save(out/'result.json',result);print(json.dumps(dict(passed=passed,summary=summary,result=str(out/'result.json'))),flush=True)
    return 0 if passed else 1
if __name__=='__main__':raise SystemExit(main())
