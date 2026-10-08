#!/usr/bin/env python3
"""Resume only the archived pre-parameter-write consumer-exit interruption."""
import argparse,json,time
from pathlib import Path
import reload_pair as m
p=argparse.ArgumentParser(description=__doc__);p.add_argument('--execute',action='store_true');p.add_argument('--loader-sha256');p.add_argument('--output',type=Path);a=p.parse_args()
previous_path=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/model_pair_reload_20260929T043930Z/state.json')
previous_sha='8a9a6ba99e9e94ee06af3be047cc5745bd96bfcc184a7b9b5f7b803e1da164bd'
if not a.execute:
 print(json.dumps({'execute':False,'previous':str(previous_path),'previous_sha256':previous_sha,'scope':m.IDLE_SCOPE,'hardware_commands':0}));raise SystemExit(0)
if m.sha(m.__file__)!=a.loader_sha256:raise ValueError('loader source changed')
previous=m.pinned_json(previous_path,previous_sha)
if previous['status']!='FAILED_NO_FURTHER_ACTION' or previous['last_completed_event']!='gateway_frozen' or 'installed_params' in previous:raise ValueError('wrong interruption stage')
if not a.output or a.output.exists() or m.MOUNT not in a.output.resolve().parents or not m.os.path.ismount(m.MOUNT):raise ValueError('new mounted external output required')
a.output.mkdir(parents=True);io=None
try:
 with m.overall_deadline():
  ev=previous['input_evidence'];binding=m.geometry_eligibility(m.pinned_json(ev['geometry_eligibility'],ev['geometry_sha256']));m.activation_review(ev['geometry_sha256'],'geometry')
  sim=m.simulation_manifest(m.pinned_json(ev['simulation_manifest'],ev['simulation_sha256']));m.health(sim)
  urdf,h=m.staged();target={'/robot_description':urdf,'/handeye':h,'/mujoco_digital_twin/server_url':sim['server_url']}
  io=m.RosIO(a.output,previous)
  if any(not io.original_pid_exited(n) for n in previous['consumers_before'].values() if n):raise ValueError('interrupted consumer has not exited')
  gateway=previous['gateway_for_recovery']
  if not io.same_pid(gateway) or io.pid_identity(gateway['pid'])['process_state'] not in ('T','t'):raise ValueError('original frozen gateway changed')
  state=io.snapshot(1790652791028371475);m.original_pair(state['original_params'])
  for key in ('sdk_q','control_reference_q','preserved'):
   if state[key]!=previous[key]:raise ValueError('interrupted hold or policy changed: '+key)
  if [tuple(x) for x in state['controllers']]!=[tuple(x) for x in previous['controllers']]:raise ValueError('controller ownership changed')
  if state['gateway_for_recovery']!=gateway:raise ValueError('gateway recovery binding changed')
  if state['task_process'] is not None or any(state['consumers_before'].values()):raise ValueError('unexpected replacement consumer')
  state.update(input_evidence=ev,geometry_evidence_binding=binding,expected_live_epoch_ns='1790652791028371475',overall_deadline_sec=m.RELOAD_DEADLINE_SEC,resume_forward_from={'path':str(previous_path),'sha256':previous_sha},resume_wrapper_sha256=m.sha(__file__))
  io.save();m.health(sim);io.checkpoint(task_absent=True)
  io.freeze_gateway();io.event('gateway_frozen');io.stop_child(m.TASK_UNIT);io.stop_observer_if_active();io.shutdown_consumers();io.checkpoint(task_absent=True);io.event('consumers_absent')
  io.set_pair(target);io.event('pair_written');io.restart_rsp();io.start_compute();io.verify_compute(target);io.event('compute_verified')
  io.replace_gateway();io.event('gateway_rebuilt');io.checkpoint(task_absent=True)
  io.start_child(m.TASK_UNIT);io.wait_task_idle();io.start_perception();io.verify_generation();io.restore_observer();io.checkpoint();io.event('pair_reload_complete')
  print(json.dumps({'status':state['status'],'state_file':str(a.output/'state.json'),'hardware_commands_sent':0,'persistent_launch_modified':False}),flush=True)
except Exception as e:
 if io is not None and io.state:
  io.state.update(failure=str(e),failure_type=type(e).__name__);io.event('FAILED_NO_FURTHER_ACTION')
 else:(a.output/'failure.json').write_text(json.dumps({'failure':str(e),'type':type(e).__name__}))
 raise
