"""Replay exact terminal/clock wires through original Task methods, no ROS I/O.

The registered owner is explicitly reconstructed from recorded owner metadata;
SDK samples are stationary providers at the recorded q, not historical samples.
This probe establishes control-flow causality, never historical callback order.
"""
import argparse,base64,hashlib,io,json,os,platform,struct,sys,time
from copy import deepcopy
from pathlib import Path

W=Path(__file__).resolve().parents[1]
E=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
V=E/'ros_log/full_latest_gen41_restart_20261018T011222Z/verification'
INPUT=V/'gen49_g48_terminal_messages_r2/result.json'
BASE=W/'.runtime/gen49_baseline_source'

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def main():
 p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args()
 out=a.output.resolve();out.relative_to(V);assert not out.exists();out.mkdir();(out/'tmp').mkdir()
 os.environ.update(TMPDIR=str(out/'tmp'),ROS_HOME=str(out/'tmp'),ROS_LOG_DIR=str(out/'tmp'),ROS_MASTER_URI='http://127.0.0.1:1',ROS_PACKAGE_PATH=str(BASE/'src'),OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',PYTHONDONTWRITEBYTECODE='1')
 sys.dont_write_bytecode=True;boundaries=[]
 def guard(event,args):
  if event.startswith('socket.') or event in ('subprocess.Popen','os.system','os.exec','os.posix_spawn','os.fork','os.forkpty'):
   boundaries.append(event);raise RuntimeError('offline boundary:'+event)
  if event=='open'and isinstance(args[0],(str,bytes,os.PathLike)):
   name=os.fsdecode(args[0]);flags=args[2]if len(args)>2 and isinstance(args[2],int)else 0;mode=args[1]if len(args)>1 and isinstance(args[1],str)else ''
   if (name.startswith('/dev/')and name not in ('/dev/null','/dev/urandom','/dev/random'))or'/hwmon/'in name or name.startswith('/sys/class/thermal'):
    boundaries.append(name);raise RuntimeError('device boundary')
   if name!='/dev/null'and(flags&(os.O_WRONLY|os.O_RDWR|os.O_CREAT|os.O_TRUNC|os.O_APPEND)or any(x in mode for x in 'wax+')):
    try:Path(name).resolve().relative_to(out)
    except ValueError:boundaries.append(name);raise RuntimeError('external write boundary')
 sys.addaudithook(guard)
 u=os.uname();platform.uname=lambda:platform.uname_result(*(tuple(u)+(u.machine,)));platform.architecture=lambda *a,**k:(str(8*struct.calcsize('P'))+'bit','ELF')
 sys.path[:0]=[str(BASE/'src/alicia_grasp_modes/src'),str(BASE/'src/alicia_flexible_grasp_supervisor/src'),str(BASE/'src/alicia_grasp_modes/tests'),str(BASE/'src/alicia_flexible_grasp_supervisor/tests'),str(W/'devel/lib/python3/dist-packages'),'/opt/ros/noetic/lib/python3/dist-packages']
 from _pytest.monkeypatch import MonkeyPatch
 from test_observation_measurement_remote import context
 from test_station_compute_wait_remote import station,clock,feedback
 from test_station_compute_work_handoff import submitted
 mp=MonkeyPatch()
 try:
  c=station.__wrapped__(context.__wrapped__(mp),mp);n=c.node;t,row,ticket=submitted(c);ref=n._station_compute_reference
  before=dict(now_ns=n._ros_source_clock_ns(),source_ns=ref['contract']['source_ns'],geometry_deadline_ns=ref['contract']['geometry_deadline_ns'],identity_deadline_ns=ref['contract']['identity_deadline_ns'],pending=n._station_compute_pending,retired=n._station_compute_retired,source_context=n._station_compute_scope())
  reasons=[];orig=n._station_compute_retire
  mp.setattr(n,'_station_compute_retire',lambda reason:(reasons.append(str(reason)),orig(reason))[-1])
  now=ref['contract']['geometry_deadline_ns']+10000000;clock(c,now);feedback(c,now);poll=n._station_compute_poll()
  after=dict(now_ns=n._ros_source_clock_ns(),pending=n._station_compute_pending,retired=n._station_compute_retired,context=n._station_compute_scope(),poll=poll,retire_reasons=reasons,reference_present=n._station_compute_reference is not None,unknown_identity_pending=repr(n._unknown_identity_pending),work_exists=bool(n._station_compute_work))
  result=dict(kind='GEN49_original_remote_pending_fixture_diagnostic',before=before,after=after,boundary_attempts=boundaries,source_or_test_or_record_modified=False)
  q=out/'result.json';q.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n');print(json.dumps(result,ensure_ascii=False));print('SHA',sha(q))
 finally:mp.undo()
if __name__=='__main__':main()
