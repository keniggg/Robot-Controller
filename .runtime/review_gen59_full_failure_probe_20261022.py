"""GEN14 original-budget real solver with read-only counters; no test changes."""
import os
os.environ.update(OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MKL_NUM_THREADS='1',PYTHONDONTWRITEBYTECODE='1')
from pathlib import Path
import sys,json,hashlib,socket,time,argparse,importlib.util,copy
from types import SimpleNamespace
W=Path(__file__).resolve().parents[1];C=W/'.runtime/gen59_observation_pose_candidate_r2';N=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z/gen58_resume_r1')
a=argparse.ArgumentParser();a.add_argument('--variant',choices=['baseline','candidate_r2'],required=True);cli=a.parse_args();O=N/'observation_pose_fix/full_r2_failure_review'/cli.variant;assert Path('/media/zhuyupei/SanStand').is_mount();O.mkdir(parents=True,exist_ok=False)
code=W if cli.variant=='baseline' else C;sys.path[:0]=[str(code/'src/alicia_grasp_modes/src'),str(code/'src/alicia_flexible_grasp_supervisor/src')];blocked=[]
class NoSocket(socket.socket):
 def __new__(cls,*a,**k):blocked.append('socket');raise RuntimeError('offline socket denied')
socket.socket=NoSocket
import numpy as np
p=C/'src/alicia_grasp_modes/tests/test_reachable_observation_early_qualified.py';spec=importlib.util.spec_from_file_location('review_gen14_fixture',str(p));fixture=importlib.util.module_from_spec(spec);spec.loader.exec_module(fixture)
module=fixture.module;args=fixture.recorded.__wrapped__();original_backend=module._backend;solver=original_backend();metrics=dict(solver_entries=0,inequality_calls=0,first_endpoint=None,directions=[])
start=time.monotonic();start_cpu=time.process_time()
def counted_backend(fun,q,**kw):
 metrics['solver_entries']+=1;d=dict(entry=metrics['solver_entries'],calls=0,first_endpoint=None);metrics['directions'].append(d);f=kw['constraints'][0]['fun']
 def counted(q):
  metrics['inequality_calls']+=1;d['calls']+=1
  try:return f(q)
  except module._EndpointQualified as e:
   info=dict(inequality_calls=metrics['inequality_calls'],entry_calls=d['calls'],wall_sec=time.monotonic()-start,q=e.endpoint[0].tolist(),endpoint_checks=e.endpoint[3]);d['first_endpoint']=info
   if metrics['first_endpoint'] is None:metrics['first_endpoint']=info
   raise
 kw['constraints']=[dict(type='ineq',fun=counted)];return solver(fun,q,**kw)
module._backend=lambda:counted_backend
rows,audit=module.generate_reachable_observation_views(**args)
wall=time.monotonic()-start;cpu=time.process_time()-start_cpu
# Independent constraint-only check of old fixed probe. It never edits the real solver result.
probe=[]
def inspect_backend(fun,seed,**kw):
 if not probe:
  e=dict(q=fixture.FEASIBLE_PROBE.tolist());probe.append(e)
  try:
   vals=kw['constraints'][0]['fun'](fixture.FEASIBLE_PROBE.copy());e.update(minimum_inequality=float(np.min(vals)),negative_indices=np.flatnonzero(vals<0).tolist(),negative_values=vals[vals<0].tolist(),all_inequalities_nonnegative=bool(np.all(vals>=0)))
  except module._EndpointQualified as qualified:e.update(fully_qualified=True,endpoint_checks=qualified.endpoint[3]);raise
 return SimpleNamespace(x=seed,nit=1,success=False,message='constraint-only diagnostic; no solver claimed')
module._backend=lambda:inspect_backend
probe_rows,probe_audit=module.generate_reachable_observation_views(**args)
module._backend=original_backend
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
def plain(x):
 if isinstance(x,dict):return {str(k):plain(v) for k,v in x.items()}
 if isinstance(x,(tuple,list)):return [plain(v) for v in x]
 if isinstance(x,np.ndarray):return x.tolist()
 if isinstance(x,np.generic):return x.item()
 return x
r=dict(scope='GEN14 frozen input real2sec solver plus separate oldprobe inequality diagnostic',variant=cli.variant,real_wall_sec=wall,real_cpu_sec=cpu,real_solver_counters=metrics,audit=audit,candidates=rows,old_probe_diagnostic=probe,old_probe_direction_reports=probe_audit['directions'],artificial_clock_not_used=True,source_fixture_sha256=sha(p),metadata_sha256=sha(p.with_name('fixtures')/'gen14_fk_source.json'),arrays_sha256=sha(p.with_name('fixtures')/'gen14_fk_source.npz'),source_sha256={str(Path(module.__file__)):sha(Path(module.__file__))},numpy_version=np.__version__,no_user_site=sys.flags.no_user_site,ROS_initialized=False,socket_attempts=blocked,execution_authority=False,helper_sha256=sha(Path(__file__)))
assert not blocked
(O/'result.json').write_text(json.dumps(plain(r),indent=2,allow_nan=False)+'\n');(O/'helper_source.py').write_bytes(Path(__file__).read_bytes());print(json.dumps(dict(variant=cli.variant,status=audit['status'],rows=len(rows),real_wall_sec=wall,calls=metrics['inequality_calls'],first_endpoint_call=None if metrics['first_endpoint'] is None else metrics['first_endpoint']['inequality_calls'],old_probe=probe[0].get('negative_indices'))))
