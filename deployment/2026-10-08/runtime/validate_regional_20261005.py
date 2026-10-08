"""Frozen regression for support hold, warming history and opt-in regional depth."""
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import subprocess
import time

W=Path(__file__).resolve().parents[1]
M=Path('/media/zhuyupei/SanStand')
R=M/'实验记录/Robot-Controller/ros_log/full_ros_resume_20261005T012259Z'
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()

def main():
    assert M.is_mount()
    prior=json.loads((W/'.runtime/support_gap_validation_20261005.json').read_text())
    tests=prior['command'][7:]
    tests=[v for v in tests if v.endswith('.py') or (W/v).is_dir()]
    tests+=['src/alicia_flexible_grasp_supervisor/tests/'+n for n in (
        'test_stationary_depth_warmup.py','test_realsense_read_diagnostics.py',
        'test_regional_stationary_depth.py','test_regional_stationary_manager.py',
        'test_regional_stationary_contract.py')]
    tests=sorted(set(tests))
    paths=set(prior['files'])|set(tests)|{
        '.runtime/regional_stationary_shadow.py','.runtime/load_regional_shadow.py',
        '.runtime/validate_regional_20261005.py',
        'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/regional_stationary_depth.py'}
    frozen={p:sha(W/p) for p in sorted(paths) if (W/p).is_file()}
    stamp=datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    log=R/('regional_validation_'+stamp+'.log');receipt=log.with_suffix('.json')
    command=['/usr/bin/python3','-m','pytest','-q','-ra','-p','no:cacheprovider']+tests
    env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',
        ALICIA_OFFLINE_SDK_COPY_TEST='1')
    started=time.monotonic()
    with log.open('w') as stream:
        result=subprocess.run(command,cwd=W,env=env,stdout=stream,stderr=subprocess.STDOUT)
    unchanged=all(sha(W/p)==v for p,v in frozen.items())
    report=dict(tests_passed=result.returncode==0 and unchanged,reviewed=False,
        files=frozen,command=command,exit_code=result.returncode,unchanged_during_tests=unchanged,
        elapsed_sec=time.monotonic()-started,log=str(log),log_sha256=sha(log),receipt=str(receipt),
        summary=log.read_text()[-3000:])
    receipt.write_text(json.dumps(report,indent=2)+'\n')
    if report['tests_passed']:(W/'.runtime/regional_validation_20261005.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k not in ('command','files')}),flush=True)
    if not report['tests_passed']:raise SystemExit(1)

if __name__=='__main__':main()
