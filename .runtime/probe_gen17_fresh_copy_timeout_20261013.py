import json,sys,traceback,threading,pytest
from pathlib import Path
from alicia_grasp_modes.station_compute_wait_task import StationComputeWaitTaskMixin
q=StationComputeWaitTaskMixin._station_task_quarantine
def quarantine(self):
 f=sys._getframe(1);r=f.f_locals.get('record');current=self._station_task_record
 if r is not None and current is not r:
  print('QUARANTINE_RECORD_CHANGED',json.dumps(dict(caller=f.f_code.co_name,old=r['contract']['source_ns'],new=current['contract']['source_ns'],new_pending=current['pending'])),flush=True)
 return q(self)
StationComputeWaitTaskMixin._station_task_quarantine=quarantine
original=threading.Event.wait
def wait(self,timeout=None):
 result=original(self,timeout)
 if timeout==2 and not result:
  print('TWO_SECOND_WAIT_TIMEOUT',flush=True)
  for ident,frame in sys._current_frames().items():
   print('THREAD',ident,flush=True);traceback.print_stack(frame,limit=12)
 return result
threading.Event.wait=wait
sys.exit(pytest.main(sys.argv[1:]))
