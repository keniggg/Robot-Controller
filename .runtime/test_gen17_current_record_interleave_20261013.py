from copy import deepcopy
import threading,sys
from test_gen11_contact_compute_admission_independent import context,computed,audit,measured_row,copy_candidate,feed_feedback

def test_late_check_must_not_quarantine_new_registered_source(monkeypatch,tmp_path):
 c=computed(context(monkeypatch));audit(c,tmp_path);n=c.task
 c.clock[0]+=.82;c.joint(c.clock[0]);feed_feedback(c);n._station_compute_task_tick()
 old=n._station_task_record;assert old['pending'];entered=threading.Event();resume=threading.Event();done=threading.Event();target=[];results=[]
 original=n._station_task_context
 def context_gate(contract,*a,**kw):
  result=original(contract,*a,**kw)
  if (target and threading.get_ident()==target[0] and not entered.is_set()
      and sys._getframe(1).f_code.co_name=='_station_task_check'
      and not n._grasp6d_plan_guard()._is_owned()):
   entered.set();assert resume.wait(2)
  return result
 monkeypatch.setattr(n,'_station_task_context',context_gate)
 def run():
  target.append(threading.get_ident())
  try:results.append(copy_candidate(c))
  except BaseException as exc:results.append(exc)
  finally:done.set()
 t=threading.Thread(target=run,daemon=True);t.start()
 try:
  assert entered.wait(2)
  measured_row(c,step=.2);fresh=n._station_task_record
  assert fresh is not old and not fresh['pending']
  resume.set();assert done.wait(2);t.join()
  assert n._station_task_record is fresh
  assert not fresh['pending'],('old out-of-lock check quarantined new actual READY',old['contract']['source_ns'],fresh['contract']['source_ns'],results)
 finally:resume.set();done.wait(2);t.join(timeout=2)
