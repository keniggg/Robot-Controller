"""Read only exact offline coordinator/worker identities after completion."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import time

parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--result',type=Path,required=True)
parser.add_argument('--unit',required=True)
args=parser.parse_args()
root=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z/gen58_resume_r1/observation_pose_fix/held_lattice_fix_r1/verification')
result=args.result.resolve(strict=True)
assert root in result.parents
assert args.unit in ('alicia-gen60-held-lattice-focus-r1.service',
    'alicia-gen60-held-lattice-focus-r2.service','alicia-gen60-held-lattice-full-r2.service')
read=lambda p:json.loads(Path(p).read_bytes())
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
d=read(result); owned=read(result.parent/'owned_workers.json'); coordinator=read(result.parent/'coordinator_identity.json')
assert owned['plan_sha256']==coordinator['plan_sha256']==d['plan_sha256']
rows=[]
for role,obj in [('coordinator',coordinator)]+[('worker',x) for x in owned['workers']]:
    pid=obj['pid']; expected=obj['identity']['start_ticks']; observed=None
    try:
        text=(Path('/proc')/str(pid)/'stat').read_text(); fields=text[text.rfind(')')+2:].split(); observed=int(fields[19])
    except FileNotFoundError: pass
    rows.append(dict(role=role,pid=pid,expected_start_ticks=expected,current_start_ticks=observed,
        exact_process_residual=observed==expected))
reply=subprocess.run(['systemctl','--user','show',args.unit,
    '--property=MainPID,ActiveState,SubState,Result,ExecMainStatus,ExecMainCode,InvocationID'],capture_output=True,text=True,check=True)
props=dict(line.split('=',1) for line in reply.stdout.splitlines())
assert props['MainPID']=='0'
assert props['ExecMainStatus']==('0' if d['passed'] else '1')
assert props['InvocationID']==coordinator['systemd_invocation_id']
assert not any(r['exact_process_residual'] for r in rows) and not d['residual_offline_process_groups']
report=dict(passed=True,kind='GEN60_exact_owned_offline_process_exit_receipt',recorded_ns=time.time_ns(),
    helper_sha256=sha(__file__),result_sha256=sha(result),verification_passed=d['passed'],verification_summary=d['summary'],
    unit_name=args.unit,unit=props,owned_processes=rows,reported_residual_groups=d['residual_offline_process_groups'],
    ROS_or_hardware_touched=False,motion_commands_sent=False,temperature_queried=False)
path=result.parent/'final_process_receipt.json'
with path.open('x') as stream:json.dump(report,stream,indent=2);stream.write('\n')
print(json.dumps(dict(passed=True,output=str(path),sha256=sha(path))))
