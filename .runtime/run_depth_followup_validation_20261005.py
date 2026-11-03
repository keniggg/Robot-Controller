"""Freeze production and bounded camera diagnostic tests before visual reload."""
from datetime import datetime,timezone
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import subprocess

W=Path(__file__).resolve().parents[1]
M=Path('/media/zhuyupei/SanStand')
R=M/'实验记录/Robot-Controller/ros_log/full_ros_resume_20261005T012259Z'

def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def main():
    assert M.is_mount() and R.is_dir(),'SanStand unavailable; no fallback'
    spec=importlib.util.spec_from_file_location('shadow_entry',W/'.runtime/run_matched_shadow_current_20261005.py')
    entry=importlib.util.module_from_spec(spec);spec.loader.exec_module(entry)
    old_path=W/'.runtime/metric_ambiguity_validation_20261005.json'
    old=json.loads(old_path.read_text())
    core_tests=sorted(set(old['test_sha256'])|{'src/alicia_flexible_grasp_supervisor/tests/test_object_geometry_observation_deficit.py'})
    groups=[core_tests,['.runtime/test_camera_preset_ab_observer.py'],['.runtime/test_run_matched_shadow_current_20261005.py']]
    names=sorted(set(old['source_sha256'])|set(old['test_sha256'])|set(entry.REQUIRED)|set(sum(groups,[])))
    hashes={n:sha(W/n) for n in names}
    tag=datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ')
    results=[]
    for i,tests in enumerate(groups):
        log=R/('depth_followup_validation_'+tag+'_'+str(i)+'.log')
        cmd=['/usr/bin/python3','-m','pytest','-q','-ra']+tests
        with log.open('w') as f:r=subprocess.run(cmd,cwd=str(W),stdout=f,stderr=subprocess.STDOUT)
        lines=log.read_text(errors='replace').strip().splitlines()
        result=dict(command=cmd,exit_code=r.returncode,result=lines[-1],log=str(log),log_sha256=sha(log))
        results.append(result);print(json.dumps(result),flush=True)
        if r.returncode:break
    changed=[n for n,h in hashes.items() if sha(W/n)!=h]
    doc=dict(schema=1,scope='matched_shadow_current_owner',reviewed=False,
        tests_passed=len(results)==len(groups) and all(x['exit_code']==0 for x in results) and not changed,
        validation_results=results,changed_during_test=changed,files=hashes,
        startup_sha256=sha(R/'startup.json'),launch_root_sha256=sha(entry.LAUNCH_ROOT),
        owner_cwd={role:'/home/zhuyupei/.ros' for role in entry.MUTABLE},
        previous_manifest=str(old_path),previous_manifest_sha256=sha(old_path),
        production_deployed=False,device_actions_performed=False)
    out=R/('depth_followup_validation_'+tag+'.json')
    out.write_text(json.dumps(doc,indent=2)+'\n')
    print(json.dumps(dict(report=str(out),passed=doc['tests_passed'],changed=changed)),flush=True)
    if not doc['tests_passed']:raise SystemExit(1)
    (W/'.runtime/depth_followup_pending_review_20261005.json').write_text(json.dumps(doc,indent=2)+'\n')

if __name__=='__main__':main()
