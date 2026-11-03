"""Complete the one skipped real-fixture case without altering the full regression record."""
import argparse,hashlib,json,os,subprocess,sys,time
from pathlib import Path
import xml.etree.ElementTree as ET
from verify_gen24_combined_20261015 import W,E,N,sha,read,guard,save,inventory,xml_summary
NODE='src/alicia_grasp_modes/tests/test_tabletop.py::TabletopTests::test_actual_saved_rgbd_snapshot'
IDENTITY=('src.alicia_grasp_modes.tests.test_tabletop.TabletopTests','test_actual_saved_rgbd_snapshot')
SOURCE_SHA='d36b256e2c878d15292cf2ec334cac436be5d381eade338f13526356a30f1323'
DB=E/'maintenance/minimal_trace_cleanup_1791956657642651189/trace.sqlite3'
KEY='ros_log/ros_live_20260921_session/unknown_sample/source.json'
ORIGINAL=Path('/home/zhuyupei/alicia_wa_full/.ros_log/ros_live_20260921_session/unknown_sample')
DEPTH=E/'ros_log/ros_live_20260921_session/unknown_sample/depth.npy'

def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--combined',type=Path,required=True);ap.add_argument('--output',type=Path,required=True);a=ap.parse_args();out=a.output.resolve();guard(out);assert not out.exists()
    combined=read(a.combined);combined_sha=sha(a.combined)
    assert combined['returncode']==0 and combined['source_unchanged'] and combined['scope_complete']
    assert combined['summary']==dict(tests=6450,failures=0,errors=0,skipped=1)
    assert sha(a.combined.parent/'pytest.xml')==combined['xml_sha256']
    cases=list(ET.parse(a.combined.parent/'pytest.xml').getroot().iter('testcase'))
    skipped=[case for case in cases if case.find('skipped') is not None]
    assert len(cases)==6450 and len(skipped)==1 and (skipped[0].get('classname'),skipped[0].get('name'))==IDENTITY
    assert skipped[0].find('skipped').get('message')=='local measured RGB-D fixture is not distributed'
    sources,scope=inventory(read(combined['patch'])['spec']);assert sources==combined['source_sha256'] and scope==combined['test_scope']
    out.mkdir();(out/'tmp').mkdir();(out/'ros').mkdir();(out/'fixture').mkdir()
    sys.path.insert(0,str(W/'tools'));from minimal_experiment_trace import TraceStore
    store=TraceStore(DB,readonly=True)
    try:
        info=store.info(KEY);assert info['sha256']==SOURCE_SHA and info['size_bytes']==2978
        store.verify(KEY);data=b''.join(store.iter_bytes(KEY))
    finally:store.close()
    assert hashlib.sha256(data).hexdigest()==SOURCE_SHA
    target=out/'fixture/source.json';target.write_bytes(data);assert sha(target)==SOURCE_SHA
    original=read(target);assert original['color_stamp_ns']==original['depth_stamp_ns']==1790040471603499174
    assert DEPTH.is_file();depth_sha=sha(DEPTH)
    assert depth_sha=='e7837b324ac26385936007125fc8b270cb1794c931cc3c9f6a103d4aab60db36' and DEPTH.stat().st_size==614528
    links={}
    for name,destination in [('source.json',target),('depth.npy',DEPTH)]:
        link=ORIGINAL/name
        assert not link.exists() and not link.is_symlink(),str(link)
        link.symlink_to(destination);assert link.resolve()==destination.resolve()
        links[str(link)]=str(destination)
    fixture=dict(source_database=str(DB),source_original_key=KEY,source_retained_sha256=SOURCE_SHA,source_original_size=2978,recovered_source=str(target),depth_path=str(DEPTH),depth_sha256=depth_sha,depth_size_bytes=DEPTH.stat().st_size,compatibility_symlinks=links,source_replaced_or_synthesized=False,cloud_contacted=False)
    save(out/'fixture_manifest.json',fixture)
    env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',PYTHONNOUSERSITE='1',PYTEST_DISABLE_PLUGIN_AUTOLOAD='1',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',ROS_MASTER_URI='http://127.0.0.1:1',TMPDIR=str(out/'tmp'),ROS_LOG_DIR=str(out/'ros'))
    env['PYTHONPATH']=':'.join(str(W/p) for p in ('src/alicia_flexible_grasp_supervisor/tests','src/alicia_grasp_modes/tests','src/alicia_grasp_modes/src','src/alicia_flexible_grasp_supervisor/src','devel/lib/python3/dist-packages'))+':'+env.get('PYTHONPATH','')
    env['ROS_PACKAGE_PATH']=str(W/'src')+':'+env.get('ROS_PACKAGE_PATH','')
    cmd=[sys.executable,'-m','pytest','-q','--rootdir='+str(W),NODE,'-p','no:cacheprovider','--tb=short','--basetemp='+str(out/'pytest_tmp'),'--junitxml='+str(out/'pytest.xml'),'-o','junit_family=legacy']
    runner_sha=sha(__file__);start=time.monotonic()
    with (out/'pytest.log').open('x') as log:run=subprocess.run(cmd,cwd=W,env=env,stdout=log,stderr=subprocess.STDOUT)
    summary=xml_summary(out/'pytest.xml');actual=list(ET.parse(out/'pytest.xml').getroot().iter('testcase'))
    exact=len(actual)==1 and (actual[0].get('classname'),actual[0].get('name'))==IDENTITY
    unchanged=all(sha(W/name)==h for name,h in sources.items()) and sha(__file__)==runner_sha and sha(a.combined)==combined_sha and sha(target)==SOURCE_SHA and sha(DEPTH)==depth_sha
    passed=run.returncode==0 and summary==dict(tests=1,failures=0,errors=0,skipped=0) and exact and unchanged
    result=dict(passed=passed,summary=summary,returncode=run.returncode,elapsed_sec=time.monotonic()-start,test_node=NODE,test_identity=list(IDENTITY),source_sha256=sources,source_count=len(sources),source_unchanged=unchanged,combined=str(a.combined.resolve()),combined_sha256=combined_sha,original_summary=combined['summary'],completion_semantics='The original full run remains 6449 passed and 1 skipped; this executes only that exact skipped case with its original measured fixture.',unique_completed_summary=dict(tests=6450,failures=0,errors=0,skipped=0) if passed else None,fixture_manifest=str(out/'fixture_manifest.json'),fixture_manifest_sha256=sha(out/'fixture_manifest.json'),log_sha256=sha(out/'pytest.log'),xml_sha256=sha(out/'pytest.xml'),runner_sha256=runner_sha,command=cmd,live_ros_contacted=False,motion_commands_sent=False,temperature_queried=False)
    save(out/'result.json',result);print(json.dumps(dict(passed=passed,summary=summary,result=str(out/'result.json'))),flush=True)
    return 0 if passed else 1
if __name__=='__main__':raise SystemExit(main())
