#!/usr/bin/env python3
"""Explicit single-command completion of a recorded, interrupted J2 diagnostic.

Never executes an archived path. Validate the original request, current epoch,
current held wire words and fresh geometry; then regenerate the return segment.
No bootstrap/enable/stop/disable/retry commands.
"""
import argparse,importlib.util,json,math,os,signal,sys
from datetime import datetime,timezone
from pathlib import Path
import numpy as np
SPEC=importlib.util.spec_from_file_location('bounded',Path(__file__).with_name('bounded_joint_response_diagnostic.py'))
m=importlib.util.module_from_spec(SPEC);SPEC.loader.exec_module(m)
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('interrupted_session',type=Path)
parser.add_argument('--execute',action='store_true')
a=parser.parse_args()
p=a.interrupted_session.resolve()
if p.parent != m.ARCHIVE.resolve():raise RuntimeError('session outside experiment archive')
plan=json.loads((p/'plan.json').read_text());terminal=json.loads((p/'terminal.json').read_text())
events=[json.loads(s) for s in (p/'events.jsonl').read_text().splitlines()]
commands=[e for e in events if e['event']=='command']
if not (terminal['status']=='failed' and terminal.get('reason','').startswith('fresh paired board quality/image/TF unavailable') and plan['joint']==2 and len(commands)==1 and commands[0]['label']=='outbound' and any(e['event']=='settled' and e['label']=='outbound' for e in events)):
 raise RuntimeError('not the expected interrupted, settled J2 outbound')
base=np.asarray(plan['starting_sdk_rad'],dtype=float);outbound=np.asarray(plan['outbound_sdk_rad'],dtype=float)
if not (base.shape==(7,) and outbound.shape==(7,) and np.isfinite(base).all() and np.isfinite(outbound).all() and all(base[j]==outbound[j] for j in [0,2,3,4,5,6]) and abs(base[1]-outbound[1])<=math.radians(2.)):
 raise RuntimeError('original segment exceeds one-axis two-degree contract')
if (m.wire_words(base)!=plan['starting_sdk_words'] or m.wire_words(outbound)!=plan['outbound_sdk_words'] or commands[0]['expected_wire_words']!=plan['outbound_sdk_words']):raise RuntimeError('archived plan/event wire words disagree')
if not os.path.ismount(str(m.MOUNT)):raise RuntimeError('archive unavailable')
out=m.ARCHIVE/('bounded_joint_return_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')+'_'+str(os.getpid()))
out.mkdir();(out/'ros').mkdir();os.environ['ROS_LOG_DIR']=str(out/'ros')
(out/'return_source.py').write_bytes(Path(__file__).read_bytes());(out/'diagnostic_source.py').write_bytes(Path(m.__file__).read_bytes())
m.rospy.init_node('resume_bounded_joint_return',anonymous=True,disable_signals=True)
args=argparse.Namespace(execute=a.execute,joint=2,step_deg=2.,bootstrap_pending=False)
d=None;result=dict(status='failed',execute=a.execute,maximum_commands=1 if a.execute else 0,interrupted_session=str(p))
def deadline(signum,frame):raise RuntimeError('59-second completion deadline exhausted')
signal.signal(signal.SIGALRM,deadline);signal.setitimer(signal.ITIMER_REAL,59.)
try:
 d=m.Diagnostic(args,out)
 held,stationary=d.await_initial()
 if d.epoch != plan['frozen_epoch_ns'] or m.wire_words(held)!=plan['outbound_sdk_words'] or held[6]!=outbound[6]:
  raise RuntimeError('live epoch/held SDK no longer matches interrupted outbound')
 old=json.loads((p/'baseline_geometry.json').read_text())
 geometry=d.snapshot('fresh_return_start')
 if not np.allclose(geometry['X'],old['T_tool_camera'],rtol=0,atol=1e-10):raise RuntimeError('handeye changed')
 if geometry['model'].fingerprint!=old['quality']['camera_model_fingerprint']:raise RuntimeError('camera model differs from original experiment')
 details=d.path(m.positions(d.accepted[-1]),base,geometry)
 m.write_json(out/'return_plan.json',dict(source_session=str(p),live_epoch_ns=d.epoch,current_sdk_words=m.wire_words(held),return_sdk_words=m.wire_words(base),fresh_geometry=details,archived_path_reused=False))
 if a.execute:
  d.execute_step('return_original_sdk',base)
  returned=d.snapshot('return_post_settle')
  original=np.asarray(old['T_base_board']);delta=returned['Y'][:3,3]-original[:3,3]
  comparison=dict(kind='same_sdk_return_completed_in_new_process',source_session=str(p),epoch_ns=d.epoch,baseline_source_stamp_ns=old['source_stamp_ns'],return_source_stamp_ns=returned['source_stamp_ns'],original_sdk_words=plan['starting_sdk_words'],final_sdk_words=m.wire_words(m.positions(d.reference[-1])),accepted_joint_delta_deg=np.degrees(returned['q'][:6]-np.asarray(old['accepted_positions_rad'])[:6]).tolist(),delta_base_board_xyz_mm=(delta*1000).tolist(),displacement_base_board_mm=float(np.linalg.norm(delta)*1000),orientation_difference_deg=math.degrees(math.acos(float(np.clip((np.trace(original[:3,:3].T@returned['Y'][:3,:3])-1)/2,-1,1)))),handeye_unchanged=True,camera_model_fingerprint=geometry['model'].fingerprint,camera_model_unchanged=True,transform_refitted=False)
  m.write_json(out/'same_sdk_return_comparison.json',comparison);d.event('same_sdk_return_observed',**comparison)
 result['status']='completed' if a.execute else 'prepared_no_motion'
except BaseException as exc:
 result['reason']=str(exc)
 if d:d.event('no_further_commands',reason=str(exc),recovery_attempted=False,stop_or_disable_sent=False)
 raise
finally:
 signal.setitimer(signal.ITIMER_REAL,0)
 if d:d.close()
 m.write_json(out/'terminal.json',result);print('SESSION '+str(out),flush=True)
