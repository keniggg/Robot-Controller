"""Verify the explicit GEN24 patch against GEN23 and rerun the complete combined scope offline."""
import argparse,hashlib,json,os,shutil,subprocess,sys,time
from pathlib import Path
import xml.etree.ElementTree as ET
W=Path(__file__).resolve().parents[1]
M=Path('/media/zhuyupei/SanStand')
E=M/'实验记录/Robot-Controller'
N=E/'ros_log/full_latest_gen22_restart_20261015T003700Z'
BASE=N/'verification/gen23_release_manifest.json'
BASE_SHA='e773a0ad15a60febfd57c9eec572119b66782a8aac953066deeab7b1b58fe214'
FULL=E/'ros_log/full_latest_gen18_restart_20261014T012216Z/verification/gen22_release_manifest.json'
FULL_SHA='a99e400819346d3552f37eb2923fae870e6a3826d57451657a52ea25cc3c6da2'
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def read(p):return json.loads(Path(p).read_text())
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
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--spec',type=Path,required=True);ap.add_argument('--output',type=Path,required=True);a=ap.parse_args()
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
    complete=not unassigned and all(n>0 for n in counts.values()) and sum(counts.values())==summary['tests']
    passed=run.returncode==0 and unchanged and complete and summary['tests']>=read(FULL)['test_summary']['tests'] and all(summary[k]==0 for k in ('failures','errors','skipped'))
    result=dict(passed=passed,summary=summary,returncode=run.returncode,elapsed_sec=time.monotonic()-started,source_sha256=sources,source_count=len(sources),source_unchanged=unchanged,changed_source_paths=changed,test_scope=scope,test_scope_files=len(scope),test_case_counts_by_file=counts,scope_complete=complete,unassigned_testcases=unassigned,command=cmd,baseline=str(BASE),baseline_sha256=BASE_SHA,full_scope_baseline=str(FULL),full_scope_baseline_sha256=FULL_SHA,patch=str(out/'patch_manifest.json'),patch_sha256=sha(out/'patch_manifest.json'),log_sha256=sha(out/'pytest.log'),xml_sha256=sha(out/'pytest.xml'),runner_sha256=runner_sha,regression_scope='full_combined',production_worktree_tested=True,live_ros_contacted=False,motion_commands_sent=False,temperature_queried=False)
    save(out/'result.json',result);print(json.dumps(dict(passed=passed,summary=summary,result=str(out/'result.json'))),flush=True)
    return 0 if passed else 1
if __name__=='__main__':raise SystemExit(main())
