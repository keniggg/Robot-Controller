#!/usr/bin/env python3
"""Specific review-first URDF/X reload; ONLY --execute permits mutations.

No robot motion publishers, enable/disable, controller switching, or root-unit
restart. On failure stop orchestration; explicit --rollback-from is required.
"""
import argparse, contextlib, copy, hashlib, json, math, os, re, signal, subprocess, sys, threading, time
from pathlib import Path
import urllib.parse, urllib.request, xmlrpc.client
import xml.etree.ElementTree as ET
import yaml

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
STAGED=ROOT/'.runtime/candidate_models/expanded_metric_20260929'
MOUNT=Path('/media/zhuyupei/SanStand')
ROOT_UNIT='alicia-ros-live-20260928.service'
TASK_UNIT='alicia-task-postmove-20260929.service'
OBSERVER_UNIT='alicia-handeye-observer-20260928.service'
CHILD_SCRIPTS={TASK_UNIT:ROOT/'.runtime/start_postmove_task_20260929.sh',OBSERVER_UNIT:ROOT/'.runtime/start_handeye_observer_readonly.sh'}
OLD_X='6ab0e8c902fe87bad1935a69b72044930b79ce1b239728cfce6dda8e94533468'
OLD='890592a24b8ad818d995e69beb670ea4254bf84f589615bfac3c8562f62449b7'
NEW='7812d6b69e854ed9f37b4e3cbd813ecd53dab3504065b2d04d287abc6fd852e0'
NEW_X='637701accca5eb5042075c213c5e00e9b189d3cd7788d916c8c06bf89f7b1a0c'
PAIR='841ca1491fb2e05bced1a1f962749a496a9c4b9944d2833eefa2c3f1b02c3789'
NAMES=tuple('Joint%d'%i for i in range(1,7))+('right_finger',)
STOP_NODES=('/grasp_mode_router','/remote_grasp6d_node','/unknown_tabletop_perception','/perception_node','/move_group','/handeye_transform')
PROTECTED_NODES=('/alicia_d_driver_node','/bessica_d_hw_interface')
KEEP_PARAMS=('/grasp/unknown_lift_after_close_enabled','/grasp/unknown_lift_cartesian_compensation_enabled','/mujoco_digital_twin/unknown_lift_gate_enabled','/mujoco_digital_twin/unknown_object_mass_bound')
WRITE_PARAMS=frozenset(('/robot_description','/handeye','/mujoco_digital_twin/server_url'))


def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def text_sha(s):return hashlib.sha256(s.encode()).hexdigest()
def pinned_json(path,digest):
 if not path or not digest or sha(path)!=digest:raise ValueError('explicit file and matching SHA256 required')
 return json.loads(Path(path).read_text())
def dotted(data,key):
 for part in key.split('.'):data=data[part]
 return data

IDLE_SCOPE='idle_pair_reload_and_readonly_compute_verification'
FROZEN_FIT='37e558bd3749c13c5bff30c6250624b1e7cd0c21dbaf8a938d335b3dd070e672'
RELOAD_DEADLINE_SEC=180.0

class ReloadDeadlineExceeded(RuntimeError):
 """Not OSError/TimeoutError: ROS polling must not swallow the total deadline."""

@contextlib.contextmanager
def overall_deadline(seconds=RELOAD_DEADLINE_SEC):
 if threading.current_thread() is not threading.main_thread():raise RuntimeError('reload deadline requires the main thread')
 if not math.isfinite(seconds) or seconds<=0:raise ValueError('positive deadline required')
 previous=signal.getsignal(signal.SIGALRM)
 if signal.getitimer(signal.ITIMER_REAL)!=(0.0,0.0):raise RuntimeError('an existing process alarm must not be replaced')
 def expired(signum,frame):raise ReloadDeadlineExceeded('overall reload deadline expired; no further orchestration')
 signal.signal(signal.SIGALRM,expired)
 try:
  signal.setitimer(signal.ITIMER_REAL,seconds)
  yield
 finally:
  signal.setitimer(signal.ITIMER_REAL,0)
  signal.signal(signal.SIGALRM,previous)

def prepare_ros_log_dir(out):
 out=Path(out).resolve()
 if not os.path.ismount(str(MOUNT)) or MOUNT not in out.parents:raise ValueError('ROS logs require mounted SanStand output')
 log=out/'ros';log.mkdir(parents=True,exist_ok=True)
 os.environ['ROS_LOG_DIR']=str(log)
 return log

def geometry_eligibility(e):
 """Validate the independent *idle loading only* opinion, not motion success."""
 required_false=('deployment_or_execution_permission_granted_by_this_file','motion_authorized','control_repair_claimed','physical_grasp_validated','refit_performed','new_acceptance_thresholds')
 if e.get('schema')!=1 or e.get('scope')!=IDLE_SCOPE or e.get('geometry_evidence_eligible') is not True or e.get('evidence_opinion_only') is not True or any(e.get(k) is not False for k in required_false):raise ValueError('geometry eligibility must be an unchanged idle-only evidence opinion')
 identity={'candidate_model_sha256':NEW,'candidate_handeye_sha256':NEW_X,'candidate_pair_sha256':PAIR,'frozen_fit_sha256':FROZEN_FIT}
 if any(e.get(k)!=v for k,v in identity.items()):raise ValueError('geometry eligibility frozen pair mismatch')
 for k,wanted in [('candidate_urdf_path',NEW),('candidate_handeye_path',NEW_X)]:
  if sha(e[k])!=wanted:raise ValueError('geometry candidate artifact changed')
 refs={k:pinned_json(e[k]['path'],e[k]['sha256']) for k in ('candidate_manifest','combined_report','scope_review')}
 model,report,review=refs['candidate_manifest'],refs['combined_report'],refs['scope_review']
 for source_key,eligibility_key in [('candidate_manifest','candidate_manifest'),('prospective_report','combined_report')]:
  if review.get('source_evidence',{}).get(source_key)!={k:e[eligibility_key][k] for k in ('path','sha256')}:raise ValueError('independent review source binding mismatch')
 if any(model.get(k)!=v or review.get('frozen_pair',{}).get(k)!=v for k,v in identity.items()):raise ValueError('geometry review/manifest identity mismatch')
 if sha(model['frozen_fit_path'])!=FROZEN_FIT:raise ValueError('frozen model fit changed')
 if (report.get('candidate_model_sha256')!=NEW or report.get('candidate_handeye_sha256')!=NEW_X or report.get('frozen_fit_sha256')!=FROZEN_FIT or report.get('fit_performed') is not False or review.get('refit_performed') is not False or review.get('motion_authorized') is not False or review.get('opinion')!='GEOMETRIC_EVIDENCE_SUPPORTS_IDLE_PAIR_LOAD_AND_READ_ONLY_COMPUTE_VERIFICATION_ONLY'):raise ValueError('source evidence refit, pair, or scope mismatch')
 if e.get('original_geometric_limits')!={'translation_rms_m':.003,'translation_max_m':.005,'orientation_rms_deg':1.,'orientation_max_deg':2.}:raise ValueError('original geometry limits changed')
 metrics=report.get('models',{}).get('candidate',{}).get('all_validation',{})
 if metrics!=e.get('candidate_prospective_metrics'):raise ValueError('geometry metrics do not match the pinned source report')
 if metrics.get('historical_3mm_5mm_1deg_2deg_numeric_pass') is not True or metrics.get('local_4mm_4deg_numeric_pass') is not True:raise ValueError('original numerical geometry criteria not passed')
 for key,limit in [('position_rms_mm',3),('position_max_mm',5),('rotation_rms_deg',1),('rotation_max_deg',2)]:
  value=metrics.get(key)
  if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value) or not 0<=value<=limit:raise ValueError('geometry metric/criterion mismatch')
 if report.get('status')!=e['combined_report'].get('status') or report.get('control_contract_outcome')!=e['combined_report'].get('control_contract_outcome') or report.get('validation_frame_count')!=e['combined_report'].get('validation_frame_count'):raise ValueError('source report status/coverage was relabelled')
 preserved=e.get('preserved_history',{})
 if preserved.get('original_failure_cleared') is not False or preserved.get('missing_phase_filled') is not False or preserved.get('later_small_j3_is_separate') is not True:raise ValueError('historical failure/missing phase must remain unchanged')
 terminal=preserved['original_failed_terminal'];raw=pinned_json(terminal['path'],terminal['sha256'])
 if review.get('source_evidence',{}).get('old_failed_terminal')!={k:terminal[k] for k in ('path','sha256')}:raise ValueError('independent review failed-terminal binding mismatch')
 if raw.get('status')!='failed' or terminal.get('status')!='failed' or raw.get('commands_published')!=terminal.get('commands_published'):raise ValueError('original failed terminal changed')
 if str(Path(terminal['path']).parent) not in report.get('failed_executed_command_contracts',[]):raise ValueError('source report no longer preserves original failed session')
 missing=preserved['missing_original_command_phase']
 phases=report.get('coverage',{}).get('command_phases',[])
 if missing not in phases or missing.get('multiframe_coverage_complete') is not False or missing.get('qualified_frame_count')!=0:raise ValueError('original missing phase was replaced')
 exclusions={'new trajectory publication','joint enable/disable or command changes','gripper closing','grasp or lift authorization','clearing the prior diagnostic failure','substituting a later small-command phase for the earlier missing phase'}
 if not exclusions.issubset(set(e.get('scope_exclusions',[]))):raise ValueError('idle eligibility must exclude motion and history rewrites')
 epochs=sorted({int(a['epoch_ns']) for a in report.get('session_audits',[])})
 if not epochs or min(epochs)<=0:raise ValueError('historical capture epochs missing')
 return {'scope':IDLE_SCOPE,'capture_epochs_ns':[str(v) for v in epochs],'preserved_history':copy.deepcopy(preserved),'combined_report':copy.deepcopy(e['combined_report']),'motion_authorized':False}

def qualification(args):
 if args.geometry_eligibility:
  if args.prospective_report or args.prospective_sha256:raise ValueError('choose exactly one evidence interface')
  e=pinned_json(args.geometry_eligibility,args.geometry_sha256);binding=geometry_eligibility(e)
  if args.expected_live_epoch is None or args.expected_live_epoch<=0:raise ValueError('idle geometry loading requires explicit positive --expected-live-epoch')
  activation_review(args.geometry_sha256,'geometry')
  return args.expected_live_epoch,binding
 if args.geometry_sha256:raise ValueError('--geometry-sha256 requires --geometry-eligibility')
 report=pinned_json(args.prospective_report,args.prospective_sha256);epoch=prospective(report)
 if args.expected_live_epoch is not None and args.expected_live_epoch!=epoch:raise ValueError('strict prospective interface requires its capture/live epoch unchanged')
 activation_review(args.prospective_sha256)
 return epoch,{'scope':'strict_complete_prospective','capture_epochs_ns':[str(epoch)],'motion_authorized':False}

def staged():
 m=json.loads((STAGED/'staging_manifest.json').read_text())
 for name,digest in m['files'].items():
  if Path(name).name!=name or sha(STAGED/name)!=digest:raise ValueError('staged file identity mismatch')
 urdf=(STAGED/'alicia_expanded_metric_candidate.urdf').read_text();h=yaml.safe_load((STAGED/'handeye_candidate_config.yaml').read_text())['handeye']
 if text_sha(urdf)!=NEW or sha(h['calibration_file'])!=NEW_X:raise ValueError('candidate pair mismatch')
 if h.get('parent_frame')!='tool0' or h.get('camera_frame')!='camera_link':raise ValueError('candidate frame mismatch')
 return urdf,h

def prospective(report):
 if (report.get('candidate_model_sha256')!=NEW or report.get('candidate_handeye_sha256')!=NEW_X or report.get('fit_performed') is not False
     or report.get('status')!='multi_posture_prospective_numeric_evaluation'):
  raise ValueError('prospective frozen-pair multi-posture report required')
 cov=report.get('coverage',{})
 if cov.get('all_command_phases_have_multiframe_evidence') is not True or cov.get('explicitly_incomplete_archive') is not False:raise ValueError('prospective command/archive coverage incomplete')
 metrics=report.get('models',{}).get('candidate',{}).get('all_validation',{})
 if metrics.get('historical_3mm_5mm_1deg_2deg_numeric_pass') is not True or metrics.get('local_4mm_4deg_numeric_pass') is not True:raise ValueError('existing prospective numerical criteria not passed')
 for key in ('position_rms_mm','position_max_mm','rotation_rms_deg','rotation_max_deg'):
  if not isinstance(metrics.get(key),(int,float)) or not math.isfinite(metrics[key]) or metrics[key]<0:raise ValueError('invalid prospective metric')
 if any(metrics[k]>v for k,v in [('position_rms_mm',3),('position_max_mm',4),('rotation_rms_deg',1),('rotation_max_deg',2)]):raise ValueError('prospective flags/values disagree')
 epochs={int(a['epoch_ns']) for a in report.get('session_audits',[])}
 if len(epochs)!=1:raise ValueError('one prospective driver epoch required')
 return epochs.pop()

def simulation_manifest(m):
 if m.get('candidate_model_sha256')!=NEW or m.get('candidate_handeye_sha256')!=NEW_X or m.get('candidate_pair_sha256')!=PAIR:raise ValueError('simulation manifest pair mismatch')
 url=urllib.parse.urlsplit(m.get('server_url',''))
 if url.scheme!='http' or not url.hostname or url.username or url.password or url.query or url.fragment or url.path not in ('','/'):raise ValueError('explicit plain HTTP simulation server origin required')
 bindings=m.get('health_bindings',{})
 if not isinstance(bindings,dict) or not bindings or not any('sha256' in k for k in bindings) or 'digital_twin.mujoco' not in bindings:raise ValueError('explicit simulation health model SHA and version bindings required')
 if not isinstance(m.get('model_xml_sha256'),str) or len(m['model_xml_sha256'])!=64 or m['model_xml_sha256'] not in bindings.values():raise ValueError('health must expose selected model XML SHA')
 return m

def health(m):
 with urllib.request.urlopen(m['server_url'].rstrip('/')+'/health',timeout=4) as f:r=json.load(f)
 if r.get('digital_twin',{}).get('ok') is not True or r['digital_twin'].get('backend')!='mujoco':raise ValueError('real selected simulation backend unavailable')
 for key,value in m['health_bindings'].items():
  if dotted(r,key)!=value:raise ValueError('simulation health binding mismatch: '+key)
 return r

def original_pair(params):
 if text_sha(params['/robot_description'])!=OLD or sha(params['/handeye']['calibration_file'])!=OLD_X:raise ValueError('original URDF/X pair identity mismatch')
 if params['/handeye'].get('parent_frame')!='tool0' or params['/handeye'].get('camera_frame')!='camera_link':raise ValueError('original frame identity mismatch')
 return params

def verify_pair_files(params):
 model=text_sha(params['/robot_description']);wanted={OLD:OLD_X,NEW:NEW_X}.get(model)
 if wanted is None or sha(params['/handeye']['calibration_file'])!=wanted:raise ValueError('selected model/calibration files no longer form the pinned pair')
 return wanted

def activation_review(report_sha,kind='prospective'):
 r=json.loads((HERE/'activation_review.json').read_text())
 key={'prospective':'accepted_prospective_sha256','geometry':'accepted_geometry_eligibility_sha256'}[kind]
 if r.get('released') is not True or r.get('execution_ready') is not True or r.get(key)!=report_sha:
  raise ValueError('activation readiness is not released for this pinned evidence')
 if kind=='geometry' and r.get('allowed_scope')!=IDLE_SCOPE:raise ValueError('activation review exceeds idle geometry scope')
 return r

def generation_ready(parameter,published,status,old,cutoff):
 keys=('generation','stamp_ns','mode','strategy')
 if any(str(parameter.get(k))!=str(published.get(k)) or str(parameter.get(k))!=str(status.get(k)) for k in keys):return False
 return (int(parameter.get('generation',0))>old['generation'] and int(parameter.get('stamp_ns',0))>cutoff
  and parameter.get('mode')==old['mode'] and parameter.get('strategy')==old['strategy']
  and status.get('state') in ('waiting_for_target','ready') and status.get('active') is False
  and not status.get('start_reserved') and not status.get('pending'))

def recovery_empty_scene_allowed(previous,move_group_absent):
 return bool(previous and previous.get('scene_empty') is True and move_group_absent
             and previous.get('last_completed_event') in ('prepared','gateway_frozen','consumers_absent','pair_written','compute_verified','gateway_rebuilt'))

def default_plan():
 return dict(execute=False,pair_sha256=PAIR,overall_deadline_sec=RELOAD_DEADLINE_SEC,geometry_scope=IDLE_SCOPE,ros_log_dir='--output/ros before init_node',root_and_hardware_retained=[ROOT_UNIT,*PROTECTED_NODES,'ros_control alicia_controller and hand_controller','raw SDK q/words/epoch'],task_child_unit=TASK_UNIT,observer_child_unit=OBSERVER_UNIT,
  sequence=['bind strict prospective report or independent idle-geometry eligibility SHA; preserve historical capture epoch, bind expected live epoch; verify selected simulation health','require idle task/controllers and unchanged SDK hold; snapshot empty scene and old pair','SIGSTOP the one PID/starttime-verified idle motion_gateway','stop dedicated task child unit; stop readonly handeye observer child; shutdown only nonrespawn listed consumers','write candidate /robot_description, /handeye and selected simulation URL while request consumers are absent','restart RSP through original parent respawn; start compute-only scoped unit; verify fresh TF + compute_fk on same accepted q','kill the frozen old gateway only; original parent respawns a new model cache','restart original task child unit; start perception/remote/router scoped unit; require one new generation/cutoff','restart readonly observer if previously active; compare protected PIDs, controllers, SDK epoch/reference and preserved policy; no grasp start'],
  stopped_ros_nodes=list(STOP_NODES),write_parameter_allowlist=sorted(WRITE_PARAMS),preserved_policy_keys=list(KEEP_PARAMS),persistent_entry_modified=False,persistent_instruction=str(HERE/'persistent_last_include.txt'),failure='Stop orchestration; leave hardware hold untouched. Explicit rollback command rebuilds consumers around saved old pair. No automatic motion or model retry.',rollback='--execute --rollback-from <state.json> --rollback-sha256 <sha> --output <new SanStand directory>',sources={p.name:sha(p) for p in HERE.glob('*') if p.is_file() and p.suffix in ('.py','.launch','.sh','.txt','.json')})


def run_sequence(io,state,target,rollback=False):
 """One bounded ordered reload. IO mutations are only reachable from --execute."""
 io.checkpoint(task_absent=bool(rollback and state.get('task_absent_at_snapshot')));io.freeze_gateway();io.event('gateway_frozen')
 io.stop_child(TASK_UNIT);io.stop_observer_if_active();io.shutdown_consumers();io.checkpoint(task_absent=True)
 io.event('consumers_absent');io.set_pair(target);io.event('pair_written')
 io.restart_rsp();io.start_compute();io.verify_compute(target);io.event('compute_verified')
 io.replace_gateway();io.event('gateway_rebuilt');io.checkpoint(task_absent=True)
 io.start_child(TASK_UNIT);io.wait_task_idle();io.start_perception();io.verify_generation()
 io.restore_observer();io.checkpoint();io.event('rollback_complete' if rollback else 'pair_reload_complete')


class RosIO:
 def __init__(self,out,previous=None):
  prepare_ros_log_dir(out) # before ROS import/init_node configures logging
  import rospy,rosgraph
  from sensor_msgs.msg import JointState
  from std_msgs.msg import Header,String
  from actionlib_msgs.msg import GoalStatusArray
  from alicia_flexible_grasp_supervisor.msg import GraspState
  self.ros=rospy;self.MasterError=rosgraph.MasterError;self.master=rosgraph.Master('/model_pair_reload');self.out=out;self.previous=previous
  self.messages={};self.received={};self.fault=None;self.state={};self.frozen_gateway=None;self.started=time.monotonic();self.accepted_window=[];self.task_sub=None;self.task_message_cutoff=0.0;self.task_generation=0;self.task_state_lock=threading.RLock();self.sdk_state_lock=threading.RLock();self.sdk_frames={'/alicia_d/sdk_command':{},'/alicia_d/control_reference':{}}
  rospy.init_node('model_pair_reload_readonly_observer',anonymous=True,disable_signals=True)
  for topic,typ in [('/grasp_mode/selection',String),('/grasp_mode/status',String),('/joint_states',JointState),('/alicia_d/sdk_command',JointState),('/alicia_d/control_reference',JointState),('/alicia_d/accepted_joint_states',JointState),('/alicia_d/control_reference_epoch',Header),('/grasp/state',GraspState),('/alicia_controller/follow_joint_trajectory/status',GoalStatusArray),('/hand_controller/follow_joint_trajectory/status',GoalStatusArray)]:
   if topic=='/grasp/state':
    self.task_state_type=typ;self.task_sub=rospy.Subscriber(topic,typ,lambda msg,g=self.task_generation:self.receive_task_state(msg,g),queue_size=20)
   else:rospy.Subscriber(topic,typ,lambda msg,t=topic:self.receive(t,msg),queue_size=20)
 def receive_task_state(self,msg,generation):
  with self.task_state_lock:
   if generation==self.task_generation:self.receive('/grasp/state',msg)
 def renew_task_subscription(self):
  with self.task_state_lock:
   self.task_generation+=1;generation=self.task_generation;old=self.task_sub;self.task_sub=None
   self.messages.pop('/grasp/state',None);self.received.pop('/grasp/state',None);self.task_message_cutoff=time.monotonic()
  if old is not None:old.unregister()
  sub=self.ros.Subscriber('/grasp/state',self.task_state_type,lambda msg,g=generation:self.receive_task_state(msg,g),queue_size=20)
  with self.task_state_lock:self.task_sub=sub
 def receive(self,topic,msg):
  if topic in ('/alicia_d/sdk_command','/alicia_d/control_reference'):
   with self.sdk_state_lock:
    self.messages[topic]=msg;self.received[topic]=time.monotonic()
    try:
     values=self.sdk_q(msg) if topic.endswith('sdk_command') else self.reference_q(msg)
     frames=self.sdk_frames[topic];frames[msg.header.stamp.to_nsec()]=msg
     for stamp in sorted(frames)[:-4]:del frames[stamp]
     key='sdk_q' if topic.endswith('sdk_command') else 'control_reference_q'
     if self.state and values!=self.state[key]:self.fault='held SDK arm/gripper reference changed'
    except (ValueError,KeyError,TypeError) as exc:self.fault='invalid transmitted reference: '+str(exc)
   return
  self.messages[topic]=msg;self.received[topic]=time.monotonic()
  if topic=='/alicia_d/accepted_joint_states':self.accepted_window.append((time.monotonic(),self.q(msg)));self.accepted_window=self.accepted_window[-40:]
  if self.state:
   if topic.endswith('control_reference_epoch') and str(msg.stamp.to_nsec())!=str(self.state['epoch_ns']):self.fault='driver epoch changed'
   if topic=='/grasp/state' and msg.active:self.fault='task became active during idle reload'
 def wait(self,predicate,label,timeout=20):
  end=time.monotonic()+timeout
  while time.monotonic()<end:
   if self.fault:raise RuntimeError(self.fault)
   if time.monotonic()-self.started>RELOAD_DEADLINE_SEC:raise ReloadDeadlineExceeded('total reload deadline')
   try:
    if predicate():return
   except (KeyError,ConnectionError,xmlrpc.client.Error,OSError):pass
   time.sleep(.1)
  raise TimeoutError(label)
 def q(self,msg,names=NAMES):
  if len(msg.name)!=len(msg.position) or len(set(msg.name))!=len(msg.name) or set(msg.name)!=set(names):raise ValueError('joint schema requires exactly '+','.join(names))
  d=dict(zip(msg.name,msg.position));q=[float(d[n]) for n in names]
  if not all(math.isfinite(v) for v in q):raise ValueError('nonfinite raw joints')
  return q
 def sdk_q(self,msg):
  if msg.header.frame_id!='sdk_transmitted':raise ValueError('SDK transmitted arm provenance required')
  return self.q(msg,NAMES[:6])
 def reference_q(self,msg):
  if msg.header.frame_id not in ('sdk_manual','sdk_tracking','sdk_handoff_required','sdk_reference_unconfirmed'):raise ValueError('driver control_reference provenance required')
  return self.q(msg)
 def sdk_hold_pair(self,minimum_stamp_ns=0):
  # Driver publishes these beside the same successful full SDK write. The
  # legacy SDK topic is six radians; only control_reference carries width (m).
  with self.sdk_state_lock:
   sdk_frames=self.sdk_frames['/alicia_d/sdk_command'];reference_frames=self.sdk_frames['/alicia_d/control_reference']
   common=set(sdk_frames)&set(reference_frames);common={n for n in common if n>0 and n>=minimum_stamp_ns}
   if not common:return None
   stamp=max(common);sdk=sdk_frames[stamp];reference=reference_frames[stamp]
   arm=self.sdk_q(sdk);full=self.reference_q(reference)
   if arm!=full[:6]:raise ValueError('same-write SDK and control_reference arm values disagree')
   return dict(stamp_ns=str(stamp),arm_q_rad=arm,reference_q=full,reference_frame_id=reference.header.frame_id,units=['rad']*6+['m'])
 def rpc(self,uri):
  class Transport(xmlrpc.client.Transport):
   def make_connection(self,host):
    c=super().make_connection(host);c.timeout=3;return c
  return xmlrpc.client.ServerProxy(uri,transport=Transport())
 def node(self,name,optional=False):
  try:uri=self.master.lookupNode(name)
  except self.MasterError as e:
   if optional and 'unknown node' in str(e).lower():return None
   raise
  try:code,_,pid=self.rpc(uri).getPid('/model_pair_reload')
  except (OSError,EOFError,xmlrpc.client.Error):
   recovery=getattr(self,'previous',None) or {}
   old=(recovery.get('task_process') if name=='/grasp_task_node' else recovery.get('consumers_before',{}).get(name))
   if optional and old and old.get('uri')==uri and self.original_pid_exited(old):return None
   raise
  if code!=1:raise ValueError('node PID unavailable: '+name)
  return self.pid_identity(pid,uri)
 @staticmethod
 def pid_identity(pid,uri=''):
  fields=Path('/proc/%d/stat'%pid).read_text().split(') ',1)[1].split();cmd=Path('/proc/%d/cmdline'%pid).read_bytes()
  return dict(pid=int(pid),starttime=fields[19],cmd_sha256=hashlib.sha256(cmd).hexdigest(),uri=uri,process_state=fields[0])
 def same_pid(self,a):
  try:b=self.pid_identity(a['pid']);return all(a[k]==b[k] for k in ('pid','starttime','cmd_sha256'))
  except OSError:return False
 def original_pid_exited(self,identity):
  # A refused XMLRPC port or changed executable is not proof of process exit.
  try:current=self.pid_identity(identity['pid'])
  except FileNotFoundError:return True
  return current['starttime']!=identity['starttime']
 def record_consumer_exit(self,name,identity,reason,error=None):
  self.state.setdefault('consumer_shutdown_evidence',[]).append(dict(node=name,pid=identity['pid'],starttime=identity['starttime'],original_process_exited=True,reason=reason,transport_error=str(error) if error else None))
  self.save()
 def cmd(self,args,timeout=15):
  return subprocess.run(args,check=True,capture_output=True,text=True,timeout=timeout).stdout.strip()
 def system(self,*args):return self.cmd(['systemctl','--user',*args])
 def active(self,unit):return subprocess.run(['systemctl','--user','is-active','--quiet',unit],timeout=15).returncode==0
 def controllers(self):
  from controller_manager_msgs.srv import ListControllers
  r=self.ros.ServiceProxy('/controller_manager/list_controllers',ListControllers)()
  return sorted((x.name,x.state,x.type) for x in r.controller)
 def snapshot(self,expected_epoch=None):
  self.wait(lambda:all(t in self.messages for t in ['/alicia_d/sdk_command','/alicia_d/control_reference','/alicia_d/accepted_joint_states','/alicia_d/control_reference_epoch','/alicia_controller/follow_joint_trajectory/status','/hand_controller/follow_joint_trajectory/status']),'raw feedback and controller state')
  child_scripts={}
  for unit,path in CHILD_SCRIPTS.items():
   previous_binding=(self.previous or {}).get('child_scripts',{}).get(unit)
   missing=self.system('show',unit,'--property=LoadState','--value')=='not-found'
   if missing:
    if not previous_binding or previous_binding['path']!=str(path) or previous_binding['sha256']!=sha(path):raise ValueError('missing unbound child unit')
    child_scripts[unit]=previous_binding
   else:
    command=self.system('show',unit,'--property=ExecStart','--value');part=self.system('show',unit,'--property=PartOf','--value').split()
    if str(path) not in command or (unit==TASK_UNIT and part!=[ROOT_UNIT]) or (unit==OBSERVER_UNIT and part):raise ValueError('child unit identity/PartOf changed')
    if self.system('show',unit,'--property=Restart','--value')!='no':raise ValueError('child Restart policy changed')
    child_scripts[unit]=dict(path=str(path),sha256=sha(path),exec_start=command,part_of=part,restart='no')
  if not self.active(ROOT_UNIT):raise ValueError('root ROS service absent; reload never starts it')
  from moveit_msgs.srv import GetPlanningScene,GetPlanningSceneRequest
  from moveit_msgs.msg import PlanningSceneComponents
  req=GetPlanningSceneRequest();req.components.components=PlanningSceneComponents.WORLD_OBJECT_NAMES|PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
  mg=self.node('/move_group',True)
  if mg is None:
   if not recovery_empty_scene_allowed(self.previous,True):raise ValueError('missing scene service without a bound partial reload empty-scene record')
  else:
   scene=self.ros.ServiceProxy('/get_planning_scene',GetPlanningScene)(req).scene
   if scene.world.collision_objects or scene.robot_state.attached_collision_objects:raise ValueError('specific empty-scene reload recipe cannot migrate nonempty scene; existing scene left intact')
  epoch=self.messages['/alicia_d/control_reference_epoch'].stamp.to_nsec()
  if expected_epoch is not None and epoch!=expected_epoch:raise ValueError('expected/live epoch mismatch')
  prior_gateway=self.previous.get('gateway_for_recovery') if self.previous else None
  if prior_gateway and self.same_pid(prior_gateway) and self.pid_identity(prior_gateway['pid'])['process_state'] in ('T','t'):gateway=prior_gateway
  else:gateway=self.node('/motion_gateway',True)
  if gateway is None:raise ValueError('gateway absent')
  owner=[];unreachable_launch_uris=[]
  for uri in self.ros.get_param('/roslaunch/uris').values():
   try:processes=self.rpc(uri).list_processes()[2][0]
   except (OSError,EOFError,xmlrpc.client.Error) as error:
    unreachable_launch_uris.append({'uri':uri,'error':str(error)});continue
   for pname,count in processes:
    if pname.startswith(('motion_gateway-','robot_state_publisher-')):
     info=self.rpc(uri).process_info(pname)[2]
     if info.get('respawn'):owner.append((pname,info['pid']))
  if not any(n.startswith('motion_gateway-') and pid==gateway['pid'] for n,pid in owner) or (not self.previous and not any(n.startswith('robot_state_publisher-') for n,pid in owner)):raise ValueError('original RSP/gateway respawn ownership unavailable')
  selection=self.ros.get_param('/grasp_mode/selection');kept={k:self.ros.get_param(k) for k in KEEP_PARAMS}
  if kept[KEEP_PARAMS[0]] is not True or kept[KEEP_PARAMS[1]] is not True or kept[KEEP_PARAMS[2]] is not False or kept[KEEP_PARAMS[3]].get('maximum_mass_kg')!=.02:raise ValueError('expected existing lift/20g policy differs; do not overwrite')
  task=self.node('/grasp_task_node',optional=bool(self.previous))
  if task is not None:self.wait(lambda:'/grasp/state' in self.messages,'initial latched task state')
  captured=[]
  def sdk_ready():
   held=self.sdk_hold_pair(epoch)
   if held is None or not 0<=self.ros.Time.now().to_nsec()-int(held['stamp_ns'])<=500_000_000:return False
   captured.append(held);return True
  self.wait(sdk_ready,'same-write six-axis SDK and seven-axis gripper reference')
  held=captured[0]
  self.state=dict(schema=1,unreachable_launch_uris=unreachable_launch_uris,task_process=task,pair_sha256=PAIR,root_unit=ROOT_UNIT,root_pid=self.system('show',ROOT_UNIT,'--property=MainPID','--value'),protected={n:self.node(n) for n in PROTECTED_NODES},epoch_ns=str(epoch),sdk_q=held['arm_q_rad'],control_reference_q=held['reference_q'],sdk_hold_pair=held,controllers=self.controllers(),selection=selection,preserved=kept,scene_empty=True,child_scripts=child_scripts,task_absent_at_snapshot=task is None,original_params={k:self.ros.get_param(k) for k in WRITE_PARAMS},observer_was_active=(self.active(OBSERVER_UNIT) or bool(self.previous and self.previous.get('observer_was_active'))),gateway_for_recovery=gateway,consumers_before={n:self.node(n,optional=bool(self.previous)) for n in STOP_NODES},units=[],status='prepared',last_completed_event='prepared',output=str(self.out),script_sha256=sha(__file__))
  if self.previous and (self.state['root_pid']!=self.previous['root_pid'] or self.state['epoch_ns']!=str(self.previous['epoch_ns']) or any(not self.same_pid(v) for v in self.previous['protected'].values())):raise ValueError('partial rollback root/hardware/epoch identity changed')
  self.checkpoint(task_absent=bool(self.previous and not self.node('/grasp_task_node',True)))
  # No arm motion is requested: prove SDK hold and inactive action statuses persist.
  window_start=time.monotonic();until=window_start+.8
  while time.monotonic()<until:self.checkpoint(task_absent=bool(self.previous and not self.node('/grasp_task_node',True)));time.sleep(.1)
  window=[q for t,q in self.accepted_window if window_start<=t<=until]
  if len(window)<6 or any(max(q[i] for q in window)-min(q[i] for q in window)>math.radians(.15) for i in range(6)):raise ValueError('accepted joints not stationary during idle deployment window')
  current_path=Path(self.state['original_params']['/handeye']['calibration_file']);raw=current_path.read_bytes();backup=self.out/'baseline_handeye.yaml';backup.write_bytes(raw)
  self.state['calibration_backup']=str(backup);self.state['calibration_backup_sha256']=hashlib.sha256(raw).hexdigest()
  self.state['rollback_params']=copy.deepcopy(self.previous.get('rollback_params',self.previous['original_params']) if self.previous else self.state['original_params'])
  self.state['rollback_calibration_backup']=self.previous.get('rollback_calibration_backup',self.previous.get('calibration_backup')) if self.previous else str(backup)
  self.save();return self.state
 def save(self):
  p=self.out/'state.json';temp=p.with_suffix('.tmp');temp.write_text(json.dumps(self.state,indent=2));temp.replace(p)
 def event(self,name):
  self.state['status']=name
  if name!='FAILED_NO_FURTHER_ACTION':self.state['last_completed_event']=name
  self.save()
  with (self.out/'events.jsonl').open('a') as f:f.write(json.dumps(dict(event=name,time_ns=time.time_ns()))+'\n')
 def checkpoint(self,task_absent=False):
  if self.fault:raise RuntimeError(self.fault)
  if str(self.messages['/alicia_d/control_reference_epoch'].stamp.to_nsec())!=str(self.state['epoch_ns']):raise ValueError('driver epoch changed during snapshot/reload')
  now=self.ros.Time.now().to_nsec()
  for topic in ('/alicia_d/sdk_command','/alicia_d/control_reference','/alicia_d/accepted_joint_states'):
   msg=self.messages[topic]
   stamp=msg.header.stamp.to_nsec()
   if stamp<int(self.state['epoch_ns']) or not 0<=now-stamp<=500_000_000:raise ValueError('stale/pre-epoch raw feedback during reload')
  if self.sdk_q(self.messages['/alicia_d/sdk_command'])!=self.state['sdk_q']:raise ValueError('SDK arm changed')
  if self.reference_q(self.messages['/alicia_d/control_reference'])!=self.state['control_reference_q']:raise ValueError('SDK arm/gripper reference changed')
  for topic in ('/alicia_controller/follow_joint_trajectory/status','/hand_controller/follow_joint_trajectory/status'):
   if time.monotonic()-self.received[topic]>2 or any(x.status in (0,1,6,7) for x in self.messages[topic].status_list):raise ValueError('controller goal active or state unavailable')
  if task_absent and self.node('/grasp_task_node',True) is not None:raise ValueError('task appeared during absent-consumer window')
  if not task_absent:self.assert_task_idle(self.state['task_process'])
  for name,ident in self.state['protected'].items():
   if not self.same_pid(ident):raise ValueError('protected hardware process changed: '+name)
  if self.system('show',ROOT_UNIT,'--property=MainPID','--value')!=self.state['root_pid']:raise ValueError('root launch changed')
  if self.controllers()!=[tuple(x) for x in self.state['controllers']]:raise ValueError('controller ownership/state changed')
  if any(self.ros.get_param(k)!=v for k,v in self.state['preserved'].items()):raise ValueError('preserved policy changed')
 def assert_task_idle(self,identity):
  # GraspState is latched and emitted only by set_state, not a heartbeat.
  msg=self.messages.get('/grasp/state');connection=getattr(msg,'_connection_header',{}) or {}
  if msg is None or msg.active or connection.get('callerid')!='/grasp_task_node' or str(connection.get('latching'))!='1':raise ValueError('inactive latched task state from its publisher required')
  if self.received.get('/grasp/state',0)<self.task_message_cutoff or self.task_sub is None or self.task_sub.get_num_connections()!=1:raise ValueError('current task subscriber connection unavailable')
  actual=self.node('/grasp_task_node')
  if not identity or not self.same_pid(identity) or any(actual.get(k)!=identity.get(k) for k in ('pid','starttime','cmd_sha256','uri')):raise ValueError('task process identity changed')
  return True
 def freeze_gateway(self):
  ident=self.state['gateway_for_recovery']
  if not self.same_pid(ident) or ident['pid'] in [v['pid'] for v in self.state['protected'].values()]:raise ValueError('gateway identity changed')
  os.kill(ident['pid'],signal.SIGSTOP);self.frozen_gateway=ident
  self.wait(lambda:self.same_pid(ident) and self.pid_identity(ident['pid'])['process_state'] in ('T','t'),'gateway SIGSTOP barrier',3)
 def stop_child(self,unit):
  if unit not in (TASK_UNIT,OBSERVER_UNIT):raise ValueError('child unit not allowed')
  if self.system('show',unit,'--property=LoadState','--value')!='not-found':self.system('stop',unit)
 def start_child(self,unit):
  if unit not in (TASK_UNIT,OBSERVER_UNIT):raise ValueError('child unit not allowed')
  binding=self.state['child_scripts'][unit]
  if binding['path']!=str(CHILD_SCRIPTS[unit]) or sha(binding['path'])!=binding['sha256']:raise ValueError('frozen child script changed')
  if unit==TASK_UNIT:self.renew_task_subscription()
  load=self.system('show',unit,'--property=LoadState','--value')
  if load=='not-found':
   props=['--property=Restart=no']+(['--property=PartOf='+ROOT_UNIT] if unit==TASK_UNIT else [])
   self.cmd(['systemd-run','--user','--unit='+unit,*props,'/bin/bash',binding['path']])
  elif load=='loaded':
   if self.system('show',unit,'--property=ExecStart','--value')!=binding['exec_start']:raise ValueError('loaded child unit changed')
   self.system('start',unit)
  else:raise ValueError('unexpected child unit load state')
 def stop_observer_if_active(self):
  if self.active(OBSERVER_UNIT):
   if self.system('show',OBSERVER_UNIT,'--property=ExecStart','--value')!=self.state['child_scripts'][OBSERVER_UNIT]['exec_start']:raise ValueError('observer unit changed')
   self.stop_child(OBSERVER_UNIT)
 def shutdown_consumers(self):
  before=self.state['consumers_before']
  for name,n in before.items():
   if not n:continue
   if n['pid'] in [v['pid'] for v in self.state['protected'].values()]:raise ValueError('consumer process identity mismatch: '+name)
   if self.original_pid_exited(n):
    self.record_consumer_exit(name,n,'original process already exited before shutdown RPC');continue
   if not self.same_pid(n):raise ValueError('consumer process identity mismatch: '+name)
   try:self.rpc(n['uri']).shutdown('/model_pair_reload','paired model consumer reload')
   except (OSError,EOFError,xmlrpc.client.Error) as exc:
    if not self.original_pid_exited(n):raise
    self.record_consumer_exit(name,n,'shutdown transport error after proven original process exit',exc)
  self.wait(lambda:all(self.original_pid_exited(n) for n in before.values() if n),'nonrespawn consumer process exit')
  if self.previous:
   for unit in self.previous.get('units',[]):
    if not re.fullmatch(r'alicia-model-pair-(compute|perception)-[A-Za-z0-9-]+[.]service',unit):raise ValueError('unexpected previous scoped unit')
    if self.active(unit):self.system('stop',unit)
 def set_pair(self,target):
  if set(target)!=WRITE_PARAMS:raise ValueError('parameter write outside exact pair/selected simulation URL')
  verify_pair_files(target)
  for key in ('/robot_description','/handeye','/mujoco_digital_twin/server_url'):self.ros.set_param(key,target[key])
  if any(self.ros.get_param(k)!=v for k,v in target.items()):raise ValueError('pair readback mismatch')
  self.state['cutoff_ns']=str(self.ros.Time.now().to_nsec());self.state['installed_params']=target;self.save()
 def restart_rsp(self):
  old=self.node('/robot_state_publisher',True)
  if old:self.rpc(old['uri']).shutdown('/model_pair_reload','paired model reload')
  self.wait(lambda:(self.node('/robot_state_publisher',True) or {}).get('pid') not in (None,old['pid'] if old else None),'RSP parent respawn')
 def scope(self,kind,launch,args=()):
  name='alicia-model-pair-%s-%s.service'%(kind,self.out.name[-30:].replace('_','-'))
  self.cmd(['systemd-run','--user','--collect','--unit='+name,'--property=PartOf='+ROOT_UNIT,'--property=Restart=no','/bin/bash',str(HERE/'start_consumers.sh'),str(HERE/launch),str(self.out/'ros'),*args])
  self.state['units'].append(name);self.save()
 def start_compute(self):self.scope('compute','compute_consumers.launch')
 def verify_compute(self,target):
  verify_pair_files(target)
  import numpy as np,tf2_ros
  from tf.transformations import quaternion_matrix
  from moveit_msgs.srv import GetPositionFK,GetPositionFKRequest
  from alicia_flexible_grasp.robot.observation_path_guard import SerialUrdfFk
  from sensor_msgs.msg import JointState
  self.ros.wait_for_service('/compute_fk',timeout=25);buffer=tf2_ros.Buffer();listener=tf2_ros.TransformListener(buffer)
  expected=SerialUrdfFk(target['/robot_description'],NAMES[:6]);x=yaml.safe_load(Path(target['/handeye']['calibration_file']).read_text())['transformation'];X=quaternion_matrix([x[k] for k in ('qx','qy','qz','qw')]);X[:3,3]=[x[k] for k in ('x','y','z')]
  def ready():
   try:
    bt=buffer.lookup_transform('base_link','tool0',self.ros.Time(0),self.ros.Duration(.2));tc=buffer.lookup_transform('tool0','camera_link',self.ros.Time(0),self.ros.Duration(.2))
    return bt.header.stamp.to_nsec()>int(self.state['cutoff_ns']) and np.allclose(mat(tc),X,atol=1e-9,rtol=0)
   except tf2_ros.TransformException:return False
  def mat(tf):
   p=tf.transform.translation;q=tf.transform.rotation;T=quaternion_matrix([q.x,q.y,q.z,q.w]);T[:3,3]=[p.x,p.y,p.z];return T
  self.wait(ready,'fresh paired TF')
  self.wait(lambda:'/joint_states' in self.messages and self.messages['/joint_states'].header.stamp.to_nsec()>int(self.state['cutoff_ns']),'new joint_states for RSP comparison')
  jm=self.messages['/joint_states'];jt=buffer.lookup_transform('base_link','tool0',jm.header.stamp,self.ros.Duration(2))
  if not np.allclose(mat(jt),expected(self.q(jm)[:6]),atol=1e-8,rtol=0):raise ValueError('RSP did not load selected URDF at joint-state timestamp')
  q=self.q(self.messages['/alicia_d/accepted_joint_states']);req=GetPositionFKRequest();req.header.frame_id='base_link';req.fk_link_names=['tool0'];req.robot_state.joint_state=JointState(name=list(NAMES),position=q)
  result=self.ros.ServiceProxy('/compute_fk',GetPositionFK)(req)
  if result.error_code.val!=1 or len(result.pose_stamped)!=1:raise ValueError('new MoveGroup FK unavailable')
  p=result.pose_stamped[0].pose;T=quaternion_matrix([p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w]);T[:3,3]=[p.position.x,p.position.y,p.position.z]
  if not np.allclose(T,expected(q[:6]),atol=1e-8,rtol=0):raise ValueError('MoveGroup did not load selected URDF')
  verify_pair_files(target)
  self.state['compute_verified_model_sha256']=text_sha(target['/robot_description']);self.save()
 def replace_gateway(self):
  old=self.frozen_gateway
  if not old or not self.same_pid(old):raise ValueError('frozen gateway identity unavailable')
  os.kill(old['pid'],signal.SIGKILL)
  self.wait(lambda:(self.node('/motion_gateway',True) or {}).get('pid') not in (None,old['pid']),'gateway parent respawn',25)
  self.state['gateway_for_recovery']=self.node('/motion_gateway');self.frozen_gateway=None;self.save()
 def wait_task_idle(self):
  previous=self.state['task_process']
  def ready():
   if '/grasp/state' not in self.messages:return False
   current=self.node('/grasp_task_node',True)
   if current is None:return False
   if previous and self.same_pid(previous):raise ValueError('old task instance survived restart')
   self.assert_task_idle(current);self.state['task_process']=current;return True
  self.wait(ready,'restarted task latched inactive')
  self.save()
 def start_perception(self):
  sel=self.state['selection'];self.scope('perception','perception_consumers.launch',['mode:='+sel['mode'],'strategy:='+sel['strategy']])
 def verify_generation(self):
  old=self.state['selection'];cutoff=int(self.state['cutoff_ns'])
  def ready():
   s=self.ros.get_param('/grasp_mode/selection',{})
   published=json.loads(self.messages['/grasp_mode/selection'].data);status=json.loads(self.messages['/grasp_mode/status'].data)
   if status.get('generation')==s.get('generation') and status.get('state')=='error':raise ValueError('new-generation reset reported error')
   return generation_ready(s,published,status,old,cutoff)
  self.wait(ready,'new model selection cutoff',30)
 def restore_observer(self):
  if self.state['observer_was_active']:self.start_child(OBSERVER_UNIT)


def main(argv=None):
 ap=argparse.ArgumentParser(description=__doc__)
 ap.add_argument('--execute',action='store_true');ap.add_argument('--prospective-report');ap.add_argument('--prospective-sha256')
 ap.add_argument('--geometry-eligibility');ap.add_argument('--geometry-sha256');ap.add_argument('--expected-live-epoch',type=int)
 ap.add_argument('--simulation-manifest');ap.add_argument('--simulation-sha256');ap.add_argument('--rollback-from');ap.add_argument('--rollback-sha256');ap.add_argument('--output',type=Path)
 a=ap.parse_args(argv);plan=default_plan()
 if not a.execute:
  staged()
  print(json.dumps(plan,indent=2));return 0
 if not a.output or not re.fullmatch(r'[A-Za-z0-9_-]+',a.output.name) or not os.path.ismount(str(MOUNT)) or MOUNT not in a.output.resolve().parents or a.output.exists():raise ValueError('new SanStand output directory required; no fallback')
 a.output.mkdir(parents=True);io=None;started=time.monotonic()
 inputs={k:str(v) if isinstance(v,Path) else v for k,v in vars(a).items() if k not in ('execute','output')}
 try:
  with overall_deadline():
   previous=None;binding=None
   if a.expected_live_epoch is not None and a.expected_live_epoch<=0:raise ValueError('positive expected live epoch required')
   if a.rollback_from:
    if a.geometry_eligibility or a.geometry_sha256 or a.prospective_report or a.prospective_sha256:raise ValueError('rollback and forward evidence cannot be mixed')
    previous=pinned_json(a.rollback_from,a.rollback_sha256)
    if previous.get('root_unit')!=ROOT_UNIT or previous.get('pair_sha256')!=PAIR:raise ValueError('rollback source outside this reload')
    target=copy.deepcopy(previous.get('rollback_params',previous['original_params']));epoch=a.expected_live_epoch
    if epoch is not None and epoch!=int(previous['epoch_ns']):raise ValueError('rollback cannot cross a driver epoch')
    backup=previous.get('rollback_calibration_backup',previous.get('calibration_backup'))
    if not backup or sha(backup)!=OLD_X:raise ValueError('immutable original handeye backup unavailable')
    target['/handeye']['calibration_file']=backup;original_pair(target)
   else:
    urdf,h=staged();epoch,binding=qualification(a)
    sim=simulation_manifest(pinned_json(a.simulation_manifest,a.simulation_sha256));health(sim)
    target={'/robot_description':urdf,'/handeye':h,'/mujoco_digital_twin/server_url':sim['server_url']}
   io=RosIO(a.output,previous)
   state=io.snapshot(epoch)
   if previous is None:original_pair(state['original_params'])
   state['input_evidence']=inputs
   state['geometry_evidence_binding']=binding
   state['expected_live_epoch_ns']=str(epoch) if epoch is not None else str(state['epoch_ns'])
   state['overall_deadline_sec']=RELOAD_DEADLINE_SEC;io.save()
   if previous is None:health(sim) # rebind immediately before admission barrier
   run_sequence(io,state,target,rollback=bool(previous))
   print(json.dumps({'status':state['status'],'state_file':str(a.output/'state.json'),'hardware_commands_sent':0,'persistent_launch_modified':False}))
 except Exception as e:
  # Alarm is disarmed/restored before this local-file-only failure recording.
  # Never call a ROS service, signal a process or auto-rollback after failure.
  if io is not None and io.state:
   io.state['failure']=str(e);io.state['failure_type']=type(e).__name__;io.state['elapsed_sec']=time.monotonic()-started;io.event('FAILED_NO_FURTHER_ACTION')
  else:
   (a.output/'state.json').write_text(json.dumps(dict(status='FAILED_NO_FURTHER_ACTION',last_completed_event='not_started',failure=str(e),failure_type=type(e).__name__,elapsed_sec=time.monotonic()-started,input_evidence=inputs,hardware_commands_sent=0),indent=2))
  raise
 return 0
if __name__=='__main__':raise SystemExit(main())
