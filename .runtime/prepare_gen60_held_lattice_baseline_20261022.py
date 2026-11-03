#!/usr/bin/env python3
"""Copy the exact reviewed GEN59 source/input closure to an isolated GEN60 root."""
from pathlib import Path
import hashlib
import json
import os
import shutil
import time

W=Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
N=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z')
O=N/'gen58_resume_r1/observation_pose_fix'
C3=W/'.runtime/gen59_observation_pose_candidate_r3'
C60=W/'.runtime/gen60_held_lattice_candidate_r1'
E=O/'held_lattice_fix_r1'
RELEASE=N/'verification/gen59_release_manifest.json'
FREEZE=O/'verification/freeze_r3/manifest_closed.json'
FULL=O/'verification/full_r3/result.json'
PLAN=O/'verification/full_r3/plan.json'
PINS={RELEASE:'5fed4cbed7ea1e824eda3bb750657970dcc1ec2d711cd041eff1a3d482ef2c6c',
      FREEZE:'89ac6a0f87f28c43f49dcc4c84682e2b539394b16038fdc276960efe4af0f5b9',
      FULL:'f30873a4a4b36e9e0f0e5fc5e897d2e693fa2133dec82e54326cab87f677dbc8',
      PLAN:'4ca75d0907877622db2752e95cc502a76bda3997cc2097e7939b6d77e378d1c9'}

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def read(p):return json.loads(Path(p).read_text())
def check(root, mapping):
    for rel,h in mapping.items():
        path=Path(rel)
        assert not path.is_absolute() and '..' not in path.parts
        p=root/path
        assert p.is_file() and not p.is_symlink(),str(p)
        assert sha(p)==h,str(p)

def main():
    assert Path('/media/zhuyupei/SanStand').is_mount()
    for p,h in PINS.items():assert sha(p)==h,str(p)
    release,freeze,full=read(RELEASE),read(FREEZE),read(FULL)
    source=release['source_sha256'];copied=freeze['copied_input_sha256']
    assert source==freeze['source_sha256']==full['source_sha256'] and len(source)==1094 and len(copied)==1106
    assert all(copied[p]==h for p,h in source.items())
    assert full['passed'] and full['summary']==dict(tests=9993,failures=0,errors=0,skipped=0)
    plan=read(PLAN);cases=plan['expected_case_identities']
    assert full['plan_sha256']==PINS[PLAN]
    assert len(cases)==len(set(tuple(v) for v in cases))==plan['expected_case_count']==9993
    check(W,source);check(C3,copied)
    E.mkdir(exist_ok=False)
    C60.mkdir(exist_ok=False)
    total_bytes=0
    for rel,h in copied.items():
        src=C3/rel;dst=C60/rel
        dst.parent.mkdir(parents=True,exist_ok=True)
        with dst.open('xb') as f:
            b=src.read_bytes();assert hashlib.sha256(b).hexdigest()==h
            f.write(b)
        shutil.copymode(src,dst)
        total_bytes+=len(b)
    check(C60,copied);check(W,source);check(C3,copied)
    assert {str(p.relative_to(C60)) for p in C60.rglob('*') if p.is_file()}==set(copied)
    for p,h in PINS.items():assert sha(p)==h,str(p)
    result=dict(kind='GEN60_held_lattice_isolated_baseline',passed=True,recorded_ns=time.time_ns(),
        helper_path=str(Path(__file__).resolve()),helper_sha256=sha(__file__),candidate_root=str(C60),
        baseline_root=str(C3),baseline_generation='GEN59',input_sha256={str(p):h for p,h in PINS.items()},
        source_sha256=source,copied_input_sha256=copied,source_count=1094,copied_input_count=1106,
        copied_bytes=total_bytes,baseline_case_count=9993,baseline_test_scope_files=full['test_scope_files'],
        baseline_case_identities_sha256=hashlib.sha256(json.dumps(cases,sort_keys=True,separators=(',',':')).encode()).hexdigest(),
        active_source_matches_release_before_and_after=True,source_copy_matches_gen59_snapshot=True,
        resource_snapshot=dict(affinity_cpus=sorted(os.sched_getaffinity(0)),loadavg=os.getloadavg(),
            memory_info={l.split(':')[0]:l.split(':')[1].strip() for l in Path('/proc/meminfo').read_text().splitlines() if l.startswith(('MemAvailable:','MemTotal:','SwapFree:'))},
            external_free_bytes=os.statvfs(N).f_bavail*os.statvfs(N).f_frsize),
        planned_production_edit_owner='handoff_review',planned_production_scope=['src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot/held_pregrasp_selection.py'],
        current_stage='baseline_only_no_candidate_patch_verified',tests_run=False,full_regression_run=False,
        active_source_modified=False,ROS_contacted=False,motion_commands_sent=False,temperature_queried=False,
        limitations=['The GEN59 historical9993 pass validates the baseline only, notfutureGEN60 changes.',
                    'Current unresolved GEN59 evidence remains protected; noexperiment cleanup performed.',
                    'Later candidate edits require a new immutable freeze andexact retainedcase union.'])
    p=E/'candidate_baseline.json'
    with p.open('x') as f:json.dump(result,f,ensure_ascii=False,indent=2);f.write('\n')
    print(json.dumps(dict(path=str(p),sha256=sha(p),candidate_root=str(C60),source_count=1094,copied_input_count=1106,copied_bytes=total_bytes),ensure_ascii=False))

if __name__=='__main__':main()
