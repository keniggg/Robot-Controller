"""Direct finite execution of unchanged early-mechanism test bodies, offline."""
import os
os.environ.update(OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MKL_NUM_THREADS='1',PYTHONDONTWRITEBYTECODE='1')
from pathlib import Path
import sys,json,hashlib,socket,time,traceback
W=Path(__file__).resolve().parents[1];C=W/'.runtime/gen59_observation_pose_candidate_r3';N=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z/gen58_resume_r1');O=N/'observation_pose_fix/full_r2_failure_review/r3_direct_preflight_final2';assert Path('/media/zhuyupei/SanStand').is_mount();O.mkdir(exist_ok=False)
sys.path[:0]=[str(C/'src/alicia_grasp_modes/tests'),str(C/'src/alicia_grasp_modes/src'),str(C/'src/alicia_flexible_grasp_supervisor/src')];blocked=[]
class NoSocket(socket.socket):
 def __new__(cls,*a,**k):blocked.append('socket');raise RuntimeError('offline socket denied')
socket.socket=NoSocket
import pytest
from _pytest.monkeypatch import MonkeyPatch
import test_reachable_observation_early_qualified as t
import test_observation_shared_band_gain as shared
cases=[('test_gen14_first_qualified_probe_survives_original_computation_budget',None),*[("test_early_endpoint_failure_continues_same_solver",x) for x in ['support_false','arrival_exception','arrival_false']],('test_four_rejected_early_checks_do_not_remove_original_final_audit',None),*[("test_unqualified_feasible_probes_never_become_proposals",x) for x in ['false_result','validation_exception']],*[("test_deadline_during_full_validation_cannot_publish_candidate",x) for x in ['projection','arrival']],('test_expired_budget_still_cannot_begin_solver',None),('test_original_gen14_probe_fails_retained_opposite_band_without_relaxing_gates',None)]
results=[]
for name,param in cases:
 started=time.monotonic();mp=MonkeyPatch();row=dict(name=name,parameter=param)
 try:
  if name.startswith('test_original_gen14'):getattr(shared,name)()
  else:
   recorded=t.recorded.__wrapped__();clock=t.measured_work_clock.__wrapped__(mp)
   if name=='test_gen14_first_qualified_probe_survives_original_computation_budget':getattr(t,name)(recorded,clock)
   elif name=='test_four_rejected_early_checks_do_not_remove_original_final_audit' or name=='test_expired_budget_still_cannot_begin_solver':getattr(t,name)(recorded,mp)
   else:getattr(t,name)(recorded,clock,mp,param)
  row['passed']=True
 except BaseException as e:row.update(passed=False,exception=type(e).__name__,traceback=traceback.format_exc())
 finally:mp.undo();row['wall_sec']=time.monotonic()-started;results.append(row)
 if not row['passed']:break
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest();r=dict(scope='bounded direct preflight only; not pytest collection or full regression',cases=results,passed=all(x['passed'] for x in results) and len(results)==len(cases),expected_cases=len(cases),source_sha256={str(Path(m.__file__)):sha(Path(m.__file__)) for m in [t,shared,t.module]},socket_attempts=blocked,execution_authority=False,helper_sha256=sha(Path(__file__)))
(O/'result.json').write_text(json.dumps(r,indent=2)+'\n');(O/'helper_source.py').write_bytes(Path(__file__).read_bytes());print(json.dumps(r,indent=2))
