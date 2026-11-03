"""Launch one frozen offline plan in a named durable systemd user service.

No source installation, ROS operation or hardware action is performed here.
Run the GEN55 runner without --execute first; this command records and submits
that exact plan. Unit cleanup is scoped to that one recorded user-service name.
"""
import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import time

W = Path(__file__).resolve().parents[1]
RUNNER = W/'.runtime/verify_gen55_parallel_offline_20261019.py'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def save(path, value):
    with Path(path).open('x') as stream:
        json.dump(value,stream,ensure_ascii=False,indent=2,allow_nan=False)
        stream.write('\n');stream.flush();os.fsync(stream.fileno())


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--plan',type=Path,required=True)
    parser.add_argument('--plan-sha256',required=True)
    parser.add_argument('--unit',required=True,help='Unique alicia-gen55-offline-*.service unit name.')
    args=parser.parse_args()
    assert re.fullmatch(r'alicia-gen55-offline-[a-z0-9-]+\.service',args.unit)
    plan_path=args.plan.resolve(strict=True)
    assert plan_path.name=='plan.json' and sha(plan_path)==args.plan_sha256
    plan=json.loads(plan_path.read_bytes());output=plan_path.parent
    assert plan['runner_path']==str(RUNNER) and sha(RUNNER)==plan['runner_sha256']
    spec=importlib.util.spec_from_file_location('gen55_exact_runner_definitions',RUNNER)
    runner=importlib.util.module_from_spec(spec);spec.loader.exec_module(runner)
    runner.bind_manifest(plan['candidate_manifest_path'],plan['candidate_manifest_sha256'])
    serial=runner.load_serial();serial.storage(output)
    assert Path(plan['output']).resolve()==output and runner.stable(serial,plan)
    assert plan['resource_budget']['execution_resources_sufficient']
    assert not (output/'result.json').exists() and not (output/'coordinator_identity.json').exists()
    assert not (output/'durable_launch_request.json').exists() and not list(output.glob('worker_*'))
    # Only a missing unit can be submitted. Never reuse/replace/reset another
    # service, including a completed earlier attempt with this same name.
    probe=subprocess.run(['systemctl','--user','show',args.unit,'--property=LoadState','--value'],
        text=True,capture_output=True,timeout=10)
    assert probe.stdout.strip()=='not-found', (probe.returncode,probe.stdout,probe.stderr)
    command=['systemd-run','--user','--unit='+args.unit,'--description=GEN55 frozen '+plan['mode']+' offline tests',
        '--property=Type=exec','--property=RemainAfterExit=yes','--property=KillMode=mixed','--property=TimeoutStopSec=120',
        '--property=Restart=no','--property=Nice=5','--property=WorkingDirectory='+str(W),
        '--property=StandardOutput=append:'+str(output/'coordinator.log'),
        '--property=StandardError=append:'+str(output/'coordinator.log'),
        '--setenv=PYTHONDONTWRITEBYTECODE=1','--setenv=GEN55_DURABLE_UNIT='+args.unit,
        sys.executable,'-B',str(RUNNER),'--execute-plan',str(plan_path),'--plan-sha256',args.plan_sha256]
    save(output/'durable_launch_request.json',dict(unit=args.unit,command=command,
        launcher_path=str(Path(__file__).resolve()),launcher_sha256=sha(__file__),
        plan_sha256=args.plan_sha256,runner_sha256=plan['runner_sha256'],at_ns=time.time_ns(),
        stop_command=['systemctl','--user','stop','--no-block',args.unit],
        hardware_or_ROS_authority=False,full_regression_passed=False))
    started=subprocess.run(command,text=True,capture_output=True,timeout=20)
    show=subprocess.run(['systemctl','--user','show',args.unit,
        '--property=Id,LoadState,ActiveState,SubState,MainPID,ControlGroup,InvocationID,Result,ExecMainStatus'],
        text=True,capture_output=True,timeout=10)
    result=dict(submitted=started.returncode==0,unit=args.unit,returncode=started.returncode,
        stdout=started.stdout,stderr=started.stderr,unit_snapshot=show.stdout,
        unit_show_returncode=show.returncode,unit_show_stderr=show.stderr,
        plan_sha256=args.plan_sha256,request_sha256=sha(output/'durable_launch_request.json'),at_ns=time.time_ns(),
        tests_completed=False,full_regression_passed=False)
    save(output/'durable_launch_submitted.json',result)
    print(json.dumps({k:result[k] for k in ('submitted','unit','returncode','unit_snapshot')},ensure_ascii=False),flush=True)
    return 0 if started.returncode==0 else 1


if __name__=='__main__':
    raise SystemExit(main())
