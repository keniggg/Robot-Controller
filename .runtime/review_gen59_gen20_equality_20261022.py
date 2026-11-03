"""Inspect original deterministic equivalence contract; no tests/source changed."""
import os
os.environ.update(OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MKL_NUM_THREADS='1',PYTHONDONTWRITEBYTECODE='1')
from pathlib import Path
import sys,json,hashlib,socket,importlib.util,time,copy
W=Path(__file__).resolve().parents[1];C=W/'.runtime/gen59_observation_pose_candidate_r2';N=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z/gen58_resume_r1');O=N/'observation_pose_fix/full_r2_failure_review/gen20_comparison';assert Path('/media/zhuyupei/SanStand').is_mount();O.mkdir(exist_ok=False)
sys.path[:0]=[str(C/'src/alicia_grasp_modes/tests'),str(C/'src/alicia_grasp_modes/src'),str(C/'src/alicia_flexible_grasp_supervisor/src')];blocked=[]
class NoSocket(socket.socket):
 def __new__(cls,*a,**k):blocked.append('socket');raise RuntimeError('offline socket denied')
socket.socket=NoSocket
import numpy as np
import test_gen8_precompute_gen20_equivalence as test
args,_=test.gen20_source.__wrapped__();cases=[];sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
def diff(a,b,p=''):
 if type(a)!=type(b):return [dict(path=p,before=a,after=b)]
 if isinstance(a,dict):
  out=[]
  for k in sorted(set(a)|set(b)):
   if k not in a:out.append(dict(path=p+'/'+k,before_missing=True,after=b[k]))
   elif k not in b:out.append(dict(path=p+'/'+k,before=a[k],after_missing=True))
   else:out.extend(diff(a[k],b[k],p+'/'+k))
  return out
 if isinstance(a,list):
  if len(a)!=len(b):return [dict(path=p,before_length=len(a),after_length=len(b))]
  return [z for i,(x,y) in enumerate(zip(a,b)) for z in diff(x,y,p+'/'+str(i))]
 return [] if a==b else [dict(path=p,before=a,after=b)]
for objective in ['centered','joint_only']:
 outputs=[];times=[]
 for module in [test.baseline(),test.candidate]:
  pref=module._arrival_center_preference
  if objective=='joint_only':module._arrival_center_preference=lambda *a,_p=pref,**kw:dict(_p(*a,**kw),cost=0.)
  started=time.monotonic();result=module.generate_reachable_observation_views(**args,clock=lambda:0.);times.append(time.monotonic()-started);module._arrival_center_preference=pref;outputs.append(json.loads(test.serialized(result)))
 paths=diff(*outputs);normalized=copy.deepcopy(outputs[1]);added=[]
 for row in normalized[1]['directions']:added.append(row.pop('observation_span'))
 equal=outputs[0]==normalized
 cases.append(dict(objective=objective,status=[x[1]['status'] for x in outputs],candidate_counts=[len(x[0]) for x in outputs],tier_counts=[x[1]['tier_attempt_count'] for x in outputs],difference_paths=paths,only_new_direction_span_audit_differs=equal,added_audits=added,wall_seconds_diagnostic_only=times))
 (O/(objective+'_full_results.json')).write_text(json.dumps(outputs,indent=2,allow_nan=False)+'\n')
r=dict(scope='same original GEN20 controlled-clock complete-result comparison',cases=cases,clock_policy='Original equivalence test constant clock reused only for equation/complete-result comparison; not a2sec performance claim',source_sha256={str(Path(test.__file__)):sha(Path(test.__file__)),str(Path(test.candidate.__file__)):sha(Path(test.candidate.__file__)),str(C/'src/alicia_grasp_modes/tests/fixtures/gen8_reachable_before_precompute.py.txt'):sha(C/'src/alicia_grasp_modes/tests/fixtures/gen8_reachable_before_precompute.py.txt')},socket_attempts=blocked,execution_authority=False,helper_sha256=sha(Path(__file__)))
assert not blocked;(O/'result.json').write_text(json.dumps(r,indent=2,allow_nan=False)+'\n');(O/'helper_source.py').write_bytes(Path(__file__).read_bytes());print(json.dumps([dict(objective=c['objective'],only_added_span=c['only_new_direction_span_audit_differs'],diff_paths=[x['path'] for x in c['difference_paths']]) for c in cases]))
