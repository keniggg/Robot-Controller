"""Bounded original station callback scheduling probe on frozen GEN45 source.

Layer 1 reuses the original GEN41 real-remote harness and its independently
rendered producer measurements. Only G45 relative source/transport/clock timing
is transposed into that harness's clock. It does not claim recorded robot state
or an end-to-end physical replay. No production/test file is edited.
"""
from copy import deepcopy
import hashlib
import importlib
import itertools
import json
from pathlib import Path
import socket
import sys
import traceback
W=Path(__file__).resolve().parents[1];B=W/'.runtime/gen46_baseline_source'
V=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z/verification')
I=V/'gen45_station_owner_messages_r1';F=V/'gen45_station_mask_fanin_r1';O=V/'gen46_g45_callback_schedule_probe_r2'
sys.path[:0]=[str(B/'src/alicia_grasp_modes/tests'),str(B/'src/alicia_flexible_grasp_supervisor/tests'),str(B/'src/alicia_grasp_modes/src'),str(B/'src/alicia_flexible_grasp_supervisor/src'),str(W/'devel/lib/python3/dist-packages'),'/opt/ros/noetic/lib/python3/dist-packages']
attempts=[]
class OfflineSocket(socket.socket):
 def __new__(cls,*a,**k):
  attempts.append('socket');raise RuntimeError('offline callback replay forbids sockets')
socket.socket=OfflineSocket
import pytest
from _pytest.monkeypatch import MonkeyPatch
import rospy
from test_observation_measurement_remote import context
from test_station_compute_wait_remote import station,row_at,clock,feedback
from test_station_compute_work_handoff import submitted
from test_unknown_pending_identity import deliver
from test_remote_modes import base

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def save(p,d):
 p.write_text(json.dumps(d,ensure_ascii=False,indent=2,allow_nan=False)+'\n');assert json.loads(p.read_text())==d

def main():
 assert Path('/media/zhuyupei/SanStand').is_mount()and not O.exists();O.mkdir()
 frozen=json.loads((B/'manifest.json').read_text());source_map=frozen.get('source_sha256')or frozen.get('source_manifest')
 assert isinstance(source_map,dict),list(frozen)
 for rel,expected in source_map.items():assert sha(B/rel)==expected
 r=json.loads((I/'result.json').read_text());obj=json.loads(Path(r['files']['/perception/unknown/detector_status']['path']).read_text())['messages']
 actual=[]
 for m in obj:
  s=m['parsed_json']or{};c=s.get('station_compute_wait')or{}
  if s.get('state')=='ready'and c.get('source_ns',0)>=1792306915498465299 and c['binding']['selection']['generation']==5:actual.append((m['received_ns'],c['source_ns']))
 assert len(actual)==17
 actual_parent=actual[0][1]
 polls=[1792306916352817296,1792306920504162788]
 results=[];alltraces=[]
 for order in itertools.permutations(('status','object','mask')):
  for duplicate in (False,True):
   mp=MonkeyPatch();trace=[];exceptions=[];case=dict(order=list(order),duplicate=duplicate)
   try:
    def forbidden(*a,**k):attempts.append('ROS_boundary');raise RuntimeError('offline callback replay boundary')
    for name in ('init_node','Publisher','Subscriber','Service','ServiceProxy','wait_for_service','set_param','delete_param'):mp.setattr(rospy,name,forbidden)
    c=context.__wrapped__(mp);c=station.__wrapped__(c,mp);t,parentrow,ticket=submitted(c);n=c.node
    parent=deepcopy(n._station_compute_reference['contract']);shift=parent['source_ns']-actual_parent
    # Preserve every G45 inter-source and transport interval exactly, while
    # retaining original harness measurements/bindings and its source epoch.
    current_original_parent=parent
    def state(tag,**extra):
     ref=n._station_compute_reference
     trace.append(dict(tag=tag,now_ns=n._ros_source_clock_ns(),active_ref=None if ref is None else ref['contract']['source_ns'],retired=n._station_compute_retired,pending=n._station_compute_pending,private_sources={str(k):v.get('kind')for k,v in n._station_private_pending_sources.items()},work_ids=list(n._station_compute_work),cache_keys={k:sorted(getattr(n,k))for k in ('_station_compute_objects','_station_compute_masks','_station_compute_status')},**extra))
    reconcile=n._station_private_pending_reconcile
    def traced_reconcile():
     state('reconcile_entry')
     try:out=reconcile();state('reconcile_return',result=out);return out
     except BaseException:trace.append(dict(tag='reconcile_exception',traceback=traceback.format_exc()));raise
    mp.setattr(n,'_station_private_pending_reconcile',traced_reconcile)
    retire=n._station_compute_retire
    def traced_retire(reason):
     state('retire_entry',reason=str(reason),stack=traceback.format_stack());return retire(reason)
    mp.setattr(n,'_station_compute_retire',traced_retire)
    private=n._station_private_pending_input
    def traced_private(kind,ns,value):
     state('private_input_entry',input_kind=kind,input_source=ns,status=(dict(state=value.get('state'),detail=value.get('detail'))if kind=='status'else None))
     out=private(kind,ns,value);state('private_input_return',input_kind=kind,input_source=ns,result=out);return out
    mp.setattr(n,'_station_private_pending_input',traced_private)
    previous=parent;ancestors=[];lastrow=parentrow;latepolls=list(polls)
    for received,source in actual[1:]:
     when=received+shift
     for poll in list(latepolls):
      mapped=poll+shift
      if mapped<=when:
       clock(c,mapped);state('original_record_clock_point');n._station_compute_poll();latepolls.remove(poll)
     mapped=source+shift;feedback(c,mapped-10_000_000);feedback(c,mapped+10_000_000);clock(c,when)
     row=row_at(c,mapped,recovery=previous);ancestors.append(deepcopy(previous));row['status']['station_compute_recovery_chain']=deepcopy(ancestors)
     state('delivery_start',original_G45_source=source,original_G45_bagstamp=received)
     for key in order:
      deliver(n,row,(key,))
      if duplicate:deliver(n,row,(key,))
     state('delivery_end',input_source=mapped)
     previous=row['status']['station_compute_wait'];lastrow=row
    for poll in latepolls:
     mapped=poll+shift;clock(c,mapped);state('original_record_clock_point');n._station_compute_poll()
    # Original wire-exact duplicate behavior is covered separately; this layer
    # repeats the harness's identical historical measurement after old expiry.
    deliver(n,parentrow,tuple(reversed(order)));state('late_parent_duplicate_end')
    case.update(completed=True,retired=n._station_compute_retired,final_ref=None if n._station_compute_reference is None else n._station_compute_reference['contract']['source_ns'],expected_latest=actual[-1][1]+shift,old_parent=parent['source_ns'],final_private_rows={str(k):v.get('kind')for k,v in n._station_private_pending_sources.items()},private_work_retained=bool(n._station_compute_work_certificate(ticket)),boundary_attempts=len(attempts))
   except BaseException as exc:
    case.update(completed=False,error=repr(exc),traceback=traceback.format_exc());exceptions.append(repr(exc))
   finally:mp.undo()
   results.append(case);alltraces.append(dict(case=case,trace=trace));print(json.dumps(case),flush=True)
 save(O/'case_traces.json',alltraces)
 loaded={}
 for name,module in list(sys.modules.items()):
  path=getattr(module,'__file__',None)
  if path and str(B)in str(path)and Path(path).is_file():loaded[name]=dict(path=str(Path(path).resolve()),sha256=sha(path))
 passed=all(v.get('completed')and not v['retired']and v['final_ref']==v['expected_latest']for v in results)and not attempts
 result=dict(passed=passed,kind='GEN46_G45_relative_timing_original_callbacks_probe',baseline_manifest_sha256=sha(B/'manifest.json'),source_count=len(source_map),cases=results,external_boundary_attempts=attempts,module_provenance=loaded,trace_sha256=sha(O/'case_traces.json'),helper_sha256=sha(__file__),production_or_old_tests_modified=False,ROS_initialized=False,hardware_contacted=False,temperature_read=False,exact_recorded_messages_used=False,scope='Original GEN41 remote fixture/real callback and source-gate behavior, original G45 relative source/recorder clock points, no geometry worker or Task main execution. Immediate callback scheduling only; not a reconstruction of original callback timing.')
 save(O/'result.json',result);(O/'helper_source.py').write_bytes(Path(__file__).read_bytes());print(json.dumps(dict(result=str(O/'result.json'),sha256=sha(O/'result.json'),passed=passed)),flush=True)
if __name__=='__main__':main()
