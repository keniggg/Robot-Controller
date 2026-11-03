"""Run one authorized recorded attempt with bound passive resource evidence."""
from pathlib import Path
import argparse, hashlib, json, os, subprocess, sys, time
W=Path(__file__).resolve().parents[1]
RUN=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z/gen58_resume_r1/observation_pose_fix/held_lattice_fix_r1/seed_wait_fix_r1/contact_feedback_fix_r1/actual_gen63_baseline_validation_20261023T041149Z')
R=RUN/'high_accuracy_grasp_r1/arrival_margin_repair_r1'
parser=argparse.ArgumentParser()
parser.add_argument('--attempt',default='')
args=parser.parse_args()
assert not args.attempt or Path(args.attempt).name==args.attempt
ATTEMPT=R/args.attempt if args.attempt else R
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def save(p,v):p.write_text(json.dumps(v,ensure_ascii=False,indent=2)+'\n')
def identity(pid):
    f=(Path('/proc')/str(pid)/'stat').read_text().rsplit(')',1)[1].split()
    return dict(pid=int(pid),parent_pid=int(f[1]),start_ticks=int(f[19]))
assert Path('/media/zhuyupei/SanStand').is_mount()
test=json.loads((R/'contact_lateral_full_r1/result.json').read_text());assert test['passed']
assert all(sha(W/p)==h for p,h in test['source_sha256'].items())
deployment=json.loads((R/'contact_lateral_placement_r1/online_reload_r1/result.json').read_text());assert deployment['passed'] and deployment['remote_reloaded']
targets={name:{k:int(row[k]) for k in ('pid','parent_pid','start_ticks')}
    for name,row in deployment['after_nodes'].items()}
assert all(identity(row['pid'])==row for row in targets.values())
# ROS aliases can share one process; verify every node, count each process once.
node_targets=dict(targets);aliases={};targets={};by_identity={}
for name,row in node_targets.items():
    key=(row['pid'],row['start_ticks'])
    if key in by_identity:aliases[name]=by_identity[key]
    else:by_identity[key]=name;targets[name]=row
assert (ATTEMPT/'user_instruction.json').is_file()
out=ATTEMPT/'actual_validation';out.mkdir()
save(out/'targets.json',dict(targets=targets,aliases=aliases,source=str(R/'contact_lateral_placement_r1/online_reload_r1/result.json')))
sampler=W/'tools/diagnostics/sample_bound_process_resources.py'
assert sha(sampler)=='22547c5ecf6e36d778e96dac540f38a6a3a0880f095a65d96960e6e44e148653'
env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',PYTHONNOUSERSITE='1',
    ROS_LOG_DIR=str(out/'ros'),ROS_PACKAGE_PATH=str(W/'src')+':/opt/ros/noetic/share',
    PYTHONPATH=':'.join(map(str,[W/'src/alicia_flexible_grasp_supervisor/src',W/'src/alicia_grasp_modes/src',
        W/'devel/lib/python3/dist-packages',Path('/opt/ros/noetic/lib/python3/dist-packages')])),
    LD_LIBRARY_PATH='/opt/ros/noetic/lib:/opt/ros/noetic/lib/x86_64-linux-gnu',
    OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MKL_NUM_THREADS='1')
resource_log=(out/'resource_sampler.log').open('w')
sample=subprocess.Popen([sys.executable,'-B',str(sampler),'--targets',str(out/'targets.json'),
    '--output',str(out/'resources'),'--duration-sec','900','--interval-sec','1'],
    cwd=W,env=env,stdout=resource_log,stderr=subprocess.STDOUT,start_new_session=True)
deadline=time.monotonic()+5
while not (out/'resources/manifest.json').is_file() and time.monotonic()<deadline:
    assert sample.poll() is None;time.sleep(.05)
assert (out/'resources/manifest.json').is_file()
command=[sys.executable,'-B',str(W/'tools/run_recorded_grasp.py'),'--duration','600','--',
    sys.executable,'-B',str(W/'.runtime/run_recorded_contact_lateral.py'),
    '--run',str(RUN),'--evidence',str(ATTEMPT)]
log=(out/'wrapper.log').open('w')
child=subprocess.Popen(command,cwd=W,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
save(out/'launch.json',dict(started_ns=time.time_ns(),wrapper_pid=child.pid,resource_sampler_pid=sample.pid,
    command=command,source_sha256=deployment['source_sha256'],no_extra_scene_capture=True,
    user_instruction_sha256=sha(ATTEMPT/'user_instruction.json')))
print(json.dumps(dict(started=True,wrapper_pid=child.pid,output=str(out))),flush=True)
seen={}
while child.poll() is None:
    # Only direct descendants of this exact wrapper are attributed. Nodes are
    # already bound above; a PID reuse can never inherit the old role.
    queue=[child.pid]
    while queue:
        pid=queue.pop()
        try:
            row=identity(pid);seen['attempt_'+str(pid)]=row
            children=(Path('/proc')/str(pid)/'task'/str(pid)/'children').read_text().split()
            queue.extend(map(int,children))
        except (OSError,ValueError):pass
    tmp=out/'resources/extra_targets.pending.json';save(tmp,dict(targets=seen))
    tmp.replace(out/'resources/extra_targets.json')
    time.sleep(1.)
log.close()
(out/'resources/STOP').write_text('recorded attempt finished\n')
sample.wait(timeout=10);resource_log.close()
save(out/'exit.json',dict(wrapper_returncode=child.returncode,resource_sampler_returncode=sample.returncode,
    finished_ns=time.time_ns()))
print(json.dumps(dict(wrapper_returncode=child.returncode,finished=True)),flush=True)
sys.exit(child.returncode)
