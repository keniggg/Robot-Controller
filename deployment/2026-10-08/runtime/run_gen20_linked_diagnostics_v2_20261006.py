"""Bounded linked passive metadata and fresh perception capture; no grasp runner."""
import argparse,hashlib,json,os,subprocess,sys,time
from pathlib import Path
W=Path(__file__).resolve().parents[1]
E=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
p=argparse.ArgumentParser();p.add_argument('--deployment',required=True);args=p.parse_args()
assert Path('/media/zhuyupei/SanStand').is_mount()
out=E/'ros_log/gen20_linked_diagnostics_20261006_v2';out.mkdir(exist_ok=False)
metadata=E/'ros_log/gen20_support_metadata_20261006_v2'
assert not metadata.exists()
env=dict(os.environ,ROS_LOG_DIR=str(out/'ros'),PYTHONDONTWRITEBYTECODE='1',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1')
report=dict(started_wall_ns=time.time_ns(),planning_requested=False,motion_commands_sent=False,children=[])
children=[];logs=[]
try:
    log=(out/'metadata.log').open('x');logs.append(log)
    obs=subprocess.Popen([sys.executable,str(W/'.runtime/observe_support_metadata_20261006.py'),'--output',str(metadata),'--expected-generation','20','--seconds','60'],cwd=W,env=env,stdout=log,stderr=subprocess.STDOUT)
    children.append(obs);report['children'].append(dict(kind='passive_metadata',pid=obs.pid))
    deadline=time.monotonic()+8
    while not (metadata/'ready.json').is_file():
        assert obs.poll() is None,'metadata observer exited before ready'
        assert time.monotonic()<deadline,'metadata ready timeout'
        time.sleep(.02)
    report['metadata_ready']=json.loads((metadata/'ready.json').read_text())
    log=(out/'perception.log').open('x');logs.append(log)
    active=subprocess.Popen([sys.executable,str(W/'.runtime/run_gen20_support_diagnostic_20261006.py'),'--deployment',args.deployment,'--prior-diagnostic',str(E/'ros_log/gen19_support_measurement_20261006')],cwd=W,env=env,stdout=log,stderr=subprocess.STDOUT)
    children.append(active);report['children'].append(dict(kind='perception_only',pid=active.pid))
    active.wait(timeout=38);report['perception_returncode']=active.returncode
    # Still preserve passive metadata until its original deadline if fresh
    # perception failed; do not manufacture a new generation or retry.
    obs.wait(timeout=65);report['metadata_returncode']=obs.returncode
    report['completed']=True
finally:
    for proc in children:
        if proc.poll() is None:
            proc.terminate()
            try:proc.wait(timeout=3)
            except subprocess.TimeoutExpired:report.setdefault('child_exit_unconfirmed',[]).append(proc.pid)
            report.setdefault('owned_diagnostic_children_terminated',[]).append(proc.pid)
    for log in logs:log.close()
    report['finished_wall_ns']=time.time_ns()
    report['script_sha256']=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    (out/'run.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(report,ensure_ascii=False),flush=True)
sys.exit(0 if report.get('completed') and report.get('perception_returncode')==0 and report.get('metadata_returncode')==0 else 2)
