"""Read-only exact identity audit after the recorded offline unit exits."""
import hashlib
import json
from pathlib import Path
import subprocess
import time

D=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z/gen58_resume_r1/observation_pose_fix/verification/full_r3')
UNIT='alicia-gen59-offline-full-r3.service'
INVOCATION='8dc3a308050a432b9606440638f5f6bc'

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def read(p):return json.loads(Path(p).read_bytes())
def current(pid):
    try:fields=(Path('/proc')/str(pid)/'stat').read_text().rsplit(')',1)[1].split()
    except FileNotFoundError:return None
    return dict(state=fields[0],parent_pid=int(fields[1]),pgid=int(fields[2]),start_ticks=int(fields[19]))

def main():
    r=read(D/'result.json'); independent=read(D/'independent_verification.json')
    assert r['passed'] and independent['passed'] and independent['full_result_sha256']==sha(D/'result.json')
    coordinator=read(D/'coordinator_identity.json');owned=read(D/'owned_workers.json')
    assert coordinator['durable_unit']==UNIT and coordinator['systemd_invocation_id']==INVOCATION
    result=subprocess.run(['systemctl','--user','show',UNIT,'--property=Id,ActiveState,SubState,MainPID,Result,ExecMainStatus,ExecMainCode,InvocationID,ControlGroup'],capture_output=True,text=True,timeout=10)
    assert result.returncode==0
    unit=dict(line.split('=',1) for line in result.stdout.splitlines() if '=' in line)
    assert unit['Id']==UNIT and unit['InvocationID']==INVOCATION and unit['MainPID']=='0'
    assert unit['Result']=='success' and unit['ExecMainStatus']=='0' and unit['SubState']=='exited'
    identities={(coordinator['pid'],coordinator['identity']['start_ticks']):'coordinator'}
    identities.update({(row['pid'],row['identity']['start_ticks']):'worker' for row in owned['workers']})
    with (D/'workload_samples.jsonl').open() as stream:
        for line in stream:
            sample=json.loads(line)
            for row in sample['offline_process_groups']['processes']:
                identities.setdefault((row['pid'],row['start_ticks']),'sampled_offline_descendant')
    checks=[]
    for (pid,start),role in sorted(identities.items()):
        observed=current(pid)
        same=observed is not None and observed['start_ticks']==start
        checks.append(dict(pid=pid,start_ticks=start,role=role,current=observed,same_identity_remaining=same))
    assert all(not row['same_identity_remaining'] for row in checks)
    assert not r['residual_offline_process_groups'] and not r['offline_child_cleanup_required']
    receipt=dict(passed=True,kind='GEN59_offline_final_process_receipt',at_ns=time.time_ns(),unit=unit,
        coordinator_identity_sha256=sha(D/'coordinator_identity.json'),owned_workers_sha256=sha(D/'owned_workers.json'),
        full_result_sha256=sha(D/'result.json'),independent_verification_sha256=sha(D/'independent_verification.json'),
        workload_samples_sha256=sha(D/'workload_samples.jsonl'),verifier_path=str(Path(__file__).resolve()),verifier_sha256=sha(__file__),
        process_identities=checks,same_identity_residual_count=0,unit_process_exited_with_status0=True,
        unit_remains_exited_for_audit=True,ROS_nodes_stopped=False,signals_sent=False)
    with (D/'final_process_receipt.json').open('x')as f:json.dump(receipt,f,ensure_ascii=False,indent=2);f.write('\n')
    print(json.dumps(dict(passed=True,verified_process_identities=len(checks),receipt_sha256=sha(D/'final_process_receipt.json'))))

if __name__=='__main__':main()
