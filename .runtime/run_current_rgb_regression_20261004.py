"""Freeze and verify the actual sources before a reviewed runtime reload."""
from pathlib import Path
from datetime import datetime, timezone
import hashlib
import json
import subprocess

WORK=Path(__file__).resolve().parents[1]
MOUNT=Path('/media/zhuyupei/SanStand')
RUN=MOUNT/'实验记录/Robot-Controller/ros_log/full_ros_resume_20261004T002930Z'
p=WORK/'.runtime/rgb_acquisition_validation_20261004.json'
old=json.loads(p.read_text())
cmd=old['command'][:]
for test in ('src/alicia_grasp_modes/tests/test_rgb_preflight_retry.py',
             'src/alicia_flexible_grasp_supervisor/tests/test_controller_reference_timing.py',
             'src/alicia_flexible_grasp_supervisor/tests/test_moveit_planner_pose_feedback.py',
             'src/alicia_flexible_grasp_supervisor/tests/test_strict_joint_probe.py'):
    if test not in cmd:
        cmd.append(test)
paths=list(old['source_sha256'])
for source in ('src/alicia_grasp_modes/src/alicia_grasp_modes/rgb_prefix_preflight.py',
               'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot/moveit_planner.py'):
    if source not in paths:
        paths.append(source)
tests=[v for v in cmd[cmd.index('-ra')+1:] if v.endswith('.py')]
before={x:hashlib.sha256((WORK/x).read_bytes()).hexdigest() for x in paths+tests}
if not MOUNT.is_mount():
    raise RuntimeError('SanStand unavailable; no fallback')
stamp=datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
log=RUN/('rgb_prefix_regression_'+stamp+'.txt')
with log.open('w') as stream:
    result=subprocess.run(cmd,cwd=str(WORK),stdout=stream,stderr=subprocess.STDOUT)
after={x:hashlib.sha256((WORK/x).read_bytes()).hexdigest() for x in before}
if before!=after:
    raise RuntimeError('Sources changed during regression; manifest not replaced')
summary=log.read_text().splitlines()[-1]
report=dict(recorded_at_utc=datetime.now(timezone.utc).isoformat(),command=cmd,
    test_exit_code=result.returncode,test_result=summary,
    source_sha256={x:after[x] for x in paths},test_sha256={x:after[x] for x in tests},
    test_log=str(log),source_unchanged_during_test=True,previous_validation_log=old['test_log'])
out=RUN/('rgb_prefix_validation_'+stamp+'.json')
out.write_text(json.dumps(report,indent=2)+'\n')
if result.returncode==0:
    p.write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(dict(test_exit_code=result.returncode,test_result=summary,
    log=str(log),report=str(out))),flush=True)
raise SystemExit(result.returncode)
