"""Single read-only negative admission check on the sealed real failed focus."""
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import sys

W=Path(__file__).resolve().parents[1]
V=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z/verification')
FOCUS=V/'gen55_integrated_focus_r1/result.json'
MANIFEST=V/'gen55_final_collected_r2/candidate_manifest.json'
RUNNER=W/'.runtime/verify_gen55_parallel_offline_20261019.py'
COMMON=W/'.runtime/gen55_idle_reload_common_20261019.py'
EXPECTED={
    str(FOCUS):'b433eeb136de3018a6990cb26d6f2d4847b046267364b1a7eabec38e1f57ed8e',
    str(MANIFEST):'5c781a1a47775007f2a10cbf5f088802c331588d79ec1e36fcb065777538bd91',
    str(RUNNER):'d00bcb389355d5bad3c71728f875e8d5ba16dd5a13fecafbb6a914e3b341b7bb',
    str(COMMON):'1cefafce628d26d6775ba41e39b633551e890fbc41514042dcc9c803309724b0',
}

def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def main():
    assert os.path.ismount('/media/zhuyupei/SanStand')
    output=V/'gen55_runtime_helper_real_failed_focus_review_r1'
    output.mkdir()
    sys.dont_write_bytecode=True
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
    sys.addaudithook(boundary)
    assert all(sha(p)==digest for p,digest in EXPECTED.items())
    focus=json.loads(FOCUS.read_bytes())
    assert focus['summary']==dict(tests=1381,failures=2,errors=0,skipped=0)
    assert focus['passed'] is False and focus['interrupted'] is False and focus['scope_complete'] is True
    assert focus['worker_count']==7 and focus['test_scope_files']==34
    assert not focus['external_boundary_attempts'] and not focus['foreign_or_inconsistent_imports']
    spec=importlib.util.spec_from_file_location('gen55_actual_failed_focus_gate',COMMON)
    common=importlib.util.module_from_spec(spec);spec.loader.exec_module(common)
    try:
        common.validate_result(FOCUS,MANIFEST,EXPECTED[str(MANIFEST)],RUNNER,EXPECTED[str(RUNNER)],'focus')
    except RuntimeError as exc:
        reason=str(exc)
        assert reason=='offline result is not complete/passing', reason
    else:
        raise AssertionError('real failed focus was incorrectly admitted')
    assert all(sha(p)==digest for p,digest in EXPECTED.items()) and not denied
    dependencies=dict(EXPECTED)
    dependencies[str(Path(__file__).resolve())]=sha(__file__)
    result=dict(kind='GEN55_real_failed_focus_admission_review',passed=True,reviewed=True,
        check='actual_failed_focus_must_be_rejected',accepted=False,expected_accept=False,
        rejection_reason=reason,dependency_sha256=dependencies,
        actual_focus_summary=focus['summary'],actual_focus_is_passing=False,
        scope_complete=True,test_bodies_reexecuted=False,previous_27_unit_checks_reexecuted=False,
        boundary_attempts=denied,full_regression_passed=False,installed=False,deployed=False,
        live_ROS_contacted=False,hardware_contacted=False,motion_commands_sent=False,
        temperature_queried=False,physical_grasp_verified=False)
    path=output/'result.json'
    with path.open('x') as f:
        json.dump(result,f,ensure_ascii=False,indent=2,allow_nan=False);f.write('\n');f.flush();os.fsync(f.fileno())
    assert json.loads(path.read_bytes())==result
    print(json.dumps(dict(passed=True,result=str(path),sha256=sha(path),rejection_reason=reason),ensure_ascii=False))

if __name__=='__main__':
    main()
