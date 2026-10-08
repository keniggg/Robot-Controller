import copy,importlib.util,json,sys
from pathlib import Path
from types import SimpleNamespace
import pytest
HERE=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('paired_reload_test_module',HERE/'reload_pair.py');m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)

def good_report():
 return {'candidate_model_sha256':m.NEW,'candidate_handeye_sha256':m.NEW_X,'fit_performed':False,'status':'multi_posture_prospective_numeric_evaluation','coverage':{'all_command_phases_have_multiframe_evidence':True,'explicitly_incomplete_archive':False},'models':{'candidate':{'all_validation':{'historical_3mm_5mm_1deg_2deg_numeric_pass':True,'local_4mm_4deg_numeric_pass':True,'position_rms_mm':.456,'position_max_mm':1.44,'rotation_rms_deg':.09,'rotation_max_deg':.19}}},'session_audits':[{'epoch_ns':123}]}

def test_default_review_never_ros_network_system_or_signal(monkeypatch,capsys):
 def forbidden(*a,**kw):raise AssertionError('mutation/read-network entered in review')
 monkeypatch.setattr(m,'RosIO',forbidden);monkeypatch.setattr(m.subprocess,'run',forbidden);monkeypatch.setattr(m.os,'kill',forbidden);monkeypatch.setattr(m.urllib.request,'urlopen',forbidden)
 assert m.main([])==0
 plan=json.loads(capsys.readouterr().out);assert plan['execute'] is False;assert plan['persistent_entry_modified'] is False

def test_prospective_existing_criteria_pass():assert m.prospective(good_report())==123
@pytest.mark.parametrize('mutate',[lambda r:r.update(status='partial_command_phase_coverage'),lambda r:r['coverage'].update(all_command_phases_have_multiframe_evidence=False),lambda r:r.update(fit_performed=True),lambda r:r.update(candidate_model_sha256='wrong'),lambda r:r['models']['candidate']['all_validation'].update(position_max_mm=4.01),lambda r:r['session_audits'].append({'epoch_ns':124})])
def test_prospective_rejects_invalid_or_partial(mutate):
 r=good_report();mutate(r)
 with pytest.raises(ValueError):m.prospective(r)

def test_current_activation_latch_prevents_old_wrist_only_bypass():
 assert m.prospective(good_report())==123
 with pytest.raises(ValueError,match='readiness is not released'):m.activation_review('a'*64)

def good_sim():return {'candidate_model_sha256':m.NEW,'candidate_handeye_sha256':m.NEW_X,'candidate_pair_sha256':m.PAIR,'server_url':'http://127.0.0.1:8002','model_xml_sha256':'a'*64,'health_bindings':{'digital_twin.model_xml_sha256':'a'*64,'digital_twin.mujoco':'3.2.3'}}
def test_simulation_not_hardcoded_to_wsl():assert m.simulation_manifest(good_sim())['server_url'].endswith(':8002')
@pytest.mark.parametrize('mutate',[lambda r:r.update(candidate_pair_sha256='old'),lambda r:r.update(server_url='http://x/user?token=1'),lambda r:r.update(health_bindings={}),lambda r:r.update(model_xml_sha256='b'*64)])
def test_simulation_requires_pair_model_and_version(mutate):
 r=good_sim();mutate(r)
 with pytest.raises(ValueError):m.simulation_manifest(r)

def test_old_handeye_content_change_rejected(monkeypatch):
 monkeypatch.setattr(m,'text_sha',lambda s:m.OLD);monkeypatch.setattr(m,'sha',lambda p:'wrong')
 old={'/robot_description':'old','/handeye':{'calibration_file':'original.yaml','parent_frame':'tool0','camera_frame':'camera_link'}}
 with pytest.raises(ValueError,match='identity mismatch'):m.original_pair(old)
 with pytest.raises(ValueError,match='pinned pair'):m.verify_pair_files(old)

@pytest.mark.parametrize('phase',['prepared','gateway_frozen','consumers_absent','pair_written','compute_verified','gateway_rebuilt'])
def test_partial_rollback_can_recover_absent_move_group(phase):
 assert m.recovery_empty_scene_allowed({'scene_empty':True,'last_completed_event':phase},True)
 assert not m.recovery_empty_scene_allowed({'scene_empty':False,'last_completed_event':phase},True)
 assert not m.recovery_empty_scene_allowed({'scene_empty':True,'last_completed_event':phase},False)

def test_completed_reload_does_not_blindly_recover_missing_scene():assert not m.recovery_empty_scene_allowed({'scene_empty':True,'last_completed_event':'pair_reload_complete'},True)

def test_generation_param_ahead_of_reset_is_not_success():
 old={'generation':2,'mode':'unknown','strategy':'observe_return'};new={'generation':3,'mode':'unknown','strategy':'observe_return','stamp_ns':'200'}
 status=dict(new,state='error',active=False,start_reserved=False,pending='')
 assert not m.generation_ready(new,dict(new,generation=2),status,old,100)
 assert not m.generation_ready(new,new,status,old,100)
 status['state']='waiting_for_target';assert m.generation_ready(new,new,status,old,100)
 status['start_reserved']=True;assert not m.generation_ready(new,new,status,old,100)

class FakeIO:
 def __init__(self,fail=None):self.calls=[];self.fail=fail
 def __getattr__(self,name):
  def call(*args,**kwargs):
   self.calls.append(name)
   if self.fail==name:raise RuntimeError(name)
  return call

def test_order_rebuilds_compute_before_gateway_and_task():
 io=FakeIO();m.run_sequence(io,{},{});c=io.calls
 assert c.index('freeze_gateway')<c.index('shutdown_consumers')<c.index('set_pair')<c.index('verify_compute')<c.index('replace_gateway')<c.index('start_child')<c.index('verify_generation')
 assert not any('controller' in x or 'enable' in x or 'move' in x for x in c)
@pytest.mark.parametrize('failure',['freeze_gateway','stop_child','shutdown_consumers','set_pair','restart_rsp','start_compute','verify_compute','replace_gateway','start_child','start_perception','verify_generation'])
def test_each_failure_stops_without_automatic_return_or_retry(failure):
 io=FakeIO(failure)
 with pytest.raises(RuntimeError):m.run_sequence(io,{}, {})
 assert io.calls[-1]==failure;assert io.calls.count(failure)==1

def test_node_rpc_timeout_is_not_absence():
 io=object.__new__(m.RosIO);io.MasterError=ValueError;io.master=SimpleNamespace(lookupNode=lambda n:'http://test')
 io.rpc=lambda uri:SimpleNamespace(getPid=lambda n:(_ for _ in ()).throw(TimeoutError('still alive')))
 with pytest.raises(TimeoutError):io.node('/perception_node',True)

def test_only_explicit_unknown_node_is_optional_absence():
 io=object.__new__(m.RosIO);io.MasterError=ValueError
 io.master=SimpleNamespace(lookupNode=lambda n:(_ for _ in ()).throw(ValueError('unknown node '+n)))
 assert io.node('/perception_node',True) is None
 io.master=SimpleNamespace(lookupNode=lambda n:(_ for _ in ()).throw(ValueError('master broken')))
 with pytest.raises(ValueError):io.node('/perception_node',True)

def test_launches_have_no_hardware_includes_or_policy_loads():
 for p in HERE.glob('*.launch'):
  r=m.ET.parse(p).getroot();assert not r.findall('.//include');assert not r.findall('.//rosparam')
  names=[n.get('name') for n in r.findall('.//node')]
  assert not set(names)&{'alicia_d_driver_node','bessica_d_hw_interface','controller_loader_once','robot_state_publisher','motion_gateway','grasp_task_node'}
  assert all(n.get('respawn','false')=='false' for n in r.findall('.//node'))

def test_partial_rollback_first_checkpoint_allows_verified_absent_task():
 class AbsentTaskIO(FakeIO):
  def checkpoint(self,task_absent=False):
   self.calls.append('checkpoint')
   if not task_absent and 'start_child' not in self.calls:raise RuntimeError('no fresh task message because original unit was stopped')
 io=AbsentTaskIO();m.run_sequence(io,{'task_absent_at_snapshot':True},{},rollback=True)
 assert 'set_pair' in io.calls and 'verify_generation' in io.calls
 io=AbsentTaskIO()
 with pytest.raises(RuntimeError,match='no fresh task'):m.run_sequence(io,{'task_absent_at_snapshot':True},{},rollback=False)

@pytest.mark.parametrize('unit',[m.TASK_UNIT,m.OBSERVER_UNIT])
def test_gc_transient_unit_recreated_with_only_frozen_script(unit,monkeypatch):
 io=object.__new__(m.RosIO);io.messages={};io.received={};io.state={'child_scripts':{unit:{'path':str(m.CHILD_SCRIPTS[unit]),'sha256':'same','exec_start':'original'}}};calls=[]
 monkeypatch.setattr(m,'sha',lambda p:'same')
 io.system=lambda *args:'not-found';io.cmd=lambda args:calls.append(args);io.renew_task_subscription=lambda:None
 io.start_child(unit)
 assert len(calls)==1 and calls[0][-2:]==['/bin/bash',str(m.CHILD_SCRIPTS[unit])]
 assert '--unit='+unit in calls[0]
 assert ('--property=PartOf='+m.ROOT_UNIT in calls[0])==(unit==m.TASK_UNIT)
 assert '--property=Restart=no' in calls[0]


def test_gc_child_script_change_blocks_recreation(monkeypatch):
 io=object.__new__(m.RosIO);io.state={'child_scripts':{m.TASK_UNIT:{'path':str(m.CHILD_SCRIPTS[m.TASK_UNIT]),'sha256':'old'}}}
 monkeypatch.setattr(m,'sha',lambda p:'changed')
 io.cmd=lambda *args:pytest.fail('must not recreate changed executable')
 with pytest.raises(ValueError,match='script changed'):io.start_child(m.TASK_UNIT)

ELIGIBILITY=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/restart_grasp_20260929T013841Z/candidate_geometry_scope_review/idle_pair_reload_geometry_eligibility.json')
def geometry_document():return json.loads(ELIGIBILITY.read_text())

def test_idle_geometry_accepts_bound_partial_history_without_relabelling_it():
 e=geometry_document();before=copy.deepcopy(e);digest=m.sha(ELIGIBILITY)
 binding=m.geometry_eligibility(e)
 assert binding['capture_epochs_ns']==['1790645893464309304']
 assert binding['motion_authorized'] is False
 assert binding['combined_report']['status']=='partial_command_phase_coverage'
 assert binding['preserved_history']['original_failure_cleared'] is False
 assert binding['preserved_history']['missing_phase_filled'] is False
 report=m.pinned_json(e['combined_report']['path'],e['combined_report']['sha256'])
 with pytest.raises(ValueError):m.prospective(report)
 assert e==before and m.sha(ELIGIBILITY)==digest

@pytest.mark.parametrize('key,value',[
 ('scope','grasp_and_lift'),('geometry_evidence_eligible',False),('evidence_opinion_only',False),
 ('motion_authorized',True),('refit_performed',True),('physical_grasp_validated',True),
 ('control_repair_claimed',True),('new_acceptance_thresholds',True),
 ('candidate_model_sha256','0'*64),('candidate_handeye_sha256','0'*64),
 ('candidate_pair_sha256','0'*64),('frozen_fit_sha256','0'*64),('scope_exclusions',[])])
def test_geometry_rejects_expanded_scope_refit_and_changed_pair(key,value):
 e=geometry_document();e[key]=value
 with pytest.raises(ValueError):m.geometry_eligibility(e)

@pytest.mark.parametrize('field',['combined_report','scope_review','candidate_manifest'])
def test_geometry_source_hash_mismatch_rejected(field):
 e=geometry_document();e[field]['sha256']='0'*64
 with pytest.raises(ValueError,match='SHA256'):m.geometry_eligibility(e)

@pytest.mark.parametrize('field',['original_failure_cleared','missing_phase_filled'])
def test_geometry_cannot_clear_or_fill_old_failure(field):
 e=geometry_document();e['preserved_history'][field]=True
 with pytest.raises(ValueError,match='must remain unchanged'):m.geometry_eligibility(e)

def test_geometry_missing_phase_cannot_be_substituted_by_later_small_command():
 e=geometry_document();e['preserved_history']['missing_original_command_phase']['qualified_frame_count']=24
 with pytest.raises(ValueError,match='original missing phase'):m.geometry_eligibility(e)

def test_geometry_source_status_and_metrics_cannot_be_relabelled():
 e=geometry_document();e['combined_report']['status']='multi_posture_prospective_numeric_evaluation'
 with pytest.raises(ValueError,match='relabelled'):m.geometry_eligibility(e)
 e=geometry_document();e['candidate_prospective_metrics']['position_max_mm']=0.1
 with pytest.raises(ValueError,match='pinned source report'):m.geometry_eligibility(e)

def geometry_args(epoch=1790652791028371475):
 return SimpleNamespace(geometry_eligibility=str(ELIGIBILITY),geometry_sha256=m.sha(ELIGIBILITY),
   prospective_report=None,prospective_sha256=None,expected_live_epoch=epoch)

def test_live_epoch_is_separate_from_historical_capture_and_never_written_back(monkeypatch):
 calls=[];monkeypatch.setattr(m,'activation_review',lambda *args:calls.append(args))
 before=ELIGIBILITY.read_bytes();epoch,binding=m.qualification(geometry_args())
 assert epoch==1790652791028371475
 assert binding['capture_epochs_ns']==['1790645893464309304']
 assert calls==[(m.sha(ELIGIBILITY),'geometry')]
 assert ELIGIBILITY.read_bytes()==before

@pytest.mark.parametrize('epoch',[None,0,-1])
def test_geometry_live_epoch_must_be_explicit_positive(epoch,monkeypatch):
 monkeypatch.setattr(m,'activation_review',lambda *a:pytest.fail('not qualified'))
 with pytest.raises(ValueError,match='expected-live-epoch'):m.qualification(geometry_args(epoch))

def test_strict_prospective_does_not_gain_cross_epoch_exception(monkeypatch):
 a=SimpleNamespace(geometry_eligibility=None,geometry_sha256=None,prospective_report='report',prospective_sha256='sha',expected_live_epoch=456)
 monkeypatch.setattr(m,'pinned_json',lambda *args:good_report())
 with pytest.raises(ValueError,match='capture/live epoch unchanged'):m.qualification(a)

def test_geometry_release_is_separate_and_does_not_rewrite_history(monkeypatch,tmp_path):
 p=tmp_path/'activation_review.json'
 d={'released':True,'execution_ready':True,'accepted_geometry_eligibility_sha256':'newsha','allowed_scope':m.IDLE_SCOPE,
    'latest_evidence_report':'still_partial','accepted_prospective_sha256':'other'}
 p.write_text(json.dumps(d));monkeypatch.setattr(m,'HERE',tmp_path)
 assert m.activation_review('newsha','geometry')==d
 with pytest.raises(ValueError):m.activation_review('newsha','prospective')
 d['allowed_scope']='motion';p.write_text(json.dumps(d))
 with pytest.raises(ValueError,match='exceeds'):m.activation_review('newsha','geometry')

def test_deadline_interrupts_blocked_sync_call_and_is_not_swallowed_by_polling():
 import os,time,signal
 read_fd,write_fd=os.pipe();old=signal.getsignal(signal.SIGALRM)
 io=object.__new__(m.RosIO);io.fault=None;io.started=time.monotonic();calls=[]
 def synchronous_rpc():
  calls.append('rpc');os.read(read_fd,1)
 start=time.monotonic()
 try:
  with pytest.raises(m.ReloadDeadlineExceeded):
   with m.overall_deadline(.03):io.wait(synchronous_rpc,'blocked RPC',timeout=2)
  assert time.monotonic()-start<1 and calls==['rpc']
  assert signal.getsignal(signal.SIGALRM)==old
  assert signal.getitimer(signal.ITIMER_REAL)==(0.0,0.0)
 finally:os.close(read_fd);os.close(write_fd)

def test_deadline_does_not_overwrite_existing_alarm():
 import signal
 old=signal.getsignal(signal.SIGALRM);signal.setitimer(signal.ITIMER_REAL,10)
 try:
  with pytest.raises(RuntimeError,match='existing process alarm'):
   with m.overall_deadline(.01):pytest.fail('body should not execute')
  assert signal.getitimer(signal.ITIMER_REAL)[0]>0
  assert signal.getsignal(signal.SIGALRM)==old
 finally:signal.setitimer(signal.ITIMER_REAL,0)

def test_default_review_does_not_arm_signal_or_change_ros_log_dir(monkeypatch,capsys):
 def forbidden(*a,**kw):pytest.fail('default review touched runtime')
 monkeypatch.setattr(m.signal,'signal',forbidden);monkeypatch.setattr(m.signal,'setitimer',forbidden)
 monkeypatch.setattr(m,'prepare_ros_log_dir',forbidden);monkeypatch.setenv('ROS_LOG_DIR','unchanged')
 assert m.main([])==0
 assert m.os.environ['ROS_LOG_DIR']=='unchanged'

def test_ros_log_dir_is_external_before_ros_import_and_init(monkeypatch,tmp_path):
 import types
 monkeypatch.setattr(m,'MOUNT',tmp_path);monkeypatch.setattr(m.os.path,'ismount',lambda p:True)
 out=tmp_path/'offline_fake_reload';calls=[]
 def init(*args,**kwargs):
  assert m.os.environ['ROS_LOG_DIR']==str(out/'ros')
  assert (out/'ros').is_dir();calls.append('init')
 rospy=types.ModuleType('rospy');rospy.init_node=init
 rospy.Subscriber=lambda *a,**kw:SimpleNamespace(get_num_connections=lambda:1)
 graph=types.ModuleType('rosgraph');graph.MasterError=ValueError;graph.Master=lambda *a:object()
 monkeypatch.setitem(sys.modules,'rospy',rospy);monkeypatch.setitem(sys.modules,'rosgraph',graph)
 for package,names in [('sensor_msgs',['JointState']),('std_msgs',['Header','String']),('actionlib_msgs',['GoalStatusArray']),('alicia_flexible_grasp_supervisor',['GraspState'])]:
  msg=types.ModuleType(package+'.msg')
  for name in names:setattr(msg,name,type(name,(),{}))
  monkeypatch.setitem(sys.modules,package,types.ModuleType(package));monkeypatch.setitem(sys.modules,package+'.msg',msg)
 m.RosIO(out)
 assert calls==['init']

def idle_task_io():
 io=object.__new__(m.RosIO)
 ident={'pid':123,'starttime':'456','cmd_sha256':'cmd','uri':'http://current'}
 msg=SimpleNamespace(active=False,_connection_header={'callerid':'/grasp_task_node','latching':'1'})
 io.messages={'/grasp/state':msg};io.received={'/grasp/state':1.0};io.task_message_cutoff=0.0
 io.task_sub=SimpleNamespace(get_num_connections=lambda:1);io.node=lambda *a:ident;io.same_pid=lambda x:True
 return io,ident

def test_old_latched_inactive_event_is_valid_with_same_connected_task_identity():
 io,ident=idle_task_io()
 assert io.assert_task_idle(ident) # Arbitrarily old event; it is not a heartbeat.

@pytest.mark.parametrize('mutation',[
 lambda io,ident:setattr(io.messages['/grasp/state'],'active',True),
 lambda io,ident:io.messages['/grasp/state']._connection_header.update(callerid='/other'),
 lambda io,ident:io.messages['/grasp/state']._connection_header.update(latching='0'),
 lambda io,ident:setattr(io,'task_sub',SimpleNamespace(get_num_connections=lambda:0)),
 lambda io,ident:setattr(io,'task_message_cutoff',2),
 lambda io,ident:setattr(io,'same_pid',lambda x:False),
 lambda io,ident:setattr(io,'node',lambda *a:dict(ident,pid=124)),
])
def test_latched_task_cannot_hide_active_disconnected_or_replaced_process(mutation):
 io,ident=idle_task_io();mutation(io,ident)
 with pytest.raises(ValueError):io.assert_task_idle(ident)

def test_task_active_transition_is_latched_as_reload_fault():
 io,_=idle_task_io();io.state={'some':'bound'};io.fault=None
 io.receive('/grasp/state',SimpleNamespace(active=True))
 io.receive('/grasp/state',SimpleNamespace(active=False))
 assert io.fault=='task became active during idle reload'

def test_total_deadline_records_failure_without_entering_sequence(monkeypatch,tmp_path):
 import time
 monkeypatch.setattr(m,'MOUNT',tmp_path);monkeypatch.setattr(m.os.path,'ismount',lambda p:True)
 monkeypatch.setattr(m,'staged',lambda:('unused',{}));monkeypatch.setattr(m,'qualification',lambda a:(999,{'capture_epochs_ns':['123']}))
 monkeypatch.setattr(m,'pinned_json',lambda *a:good_sim());monkeypatch.setattr(m,'health',lambda *a:None)
 real=m.overall_deadline;monkeypatch.setattr(m,'overall_deadline',lambda:real(.03))
 monkeypatch.setattr(m,'run_sequence',lambda *a,**kw:pytest.fail('sequence must not run after timeout'))
 class BlockedIO:
  def __init__(self,*args):self.state={}
  def snapshot(self,epoch):assert epoch==999;time.sleep(2)
 monkeypatch.setattr(m,'RosIO',BlockedIO)
 out=tmp_path/'bounded_failure'
 with pytest.raises(m.ReloadDeadlineExceeded):m.main(['--execute','--output',str(out)])
 state=json.loads((out/'state.json').read_text())
 assert state['status']=='FAILED_NO_FURTHER_ACTION'
 assert state['last_completed_event']=='not_started' and state['hardware_commands_sent']==0
 assert state['failure_type']=='ReloadDeadlineExceeded'

def test_independent_scope_review_must_refer_to_exact_combined_report(monkeypatch):
 e=geometry_document();original=m.pinned_json
 def changed_review(path,digest):
  r=original(path,digest)
  if path==e['scope_review']['path']:r['source_evidence']['prospective_report']['sha256']='0'*64
  return r
 monkeypatch.setattr(m,'pinned_json',changed_review)
 with pytest.raises(ValueError,match='review source binding'):m.geometry_eligibility(e)

def test_restarted_task_cannot_reuse_pre_restart_latched_message():
 io,old=idle_task_io();new=dict(old,pid=124,starttime='new',uri='http://new')
 io.state={'task_process':old};io.task_message_cutoff=2;io.node=lambda *a:new
 io.same_pid=lambda ident:ident['pid']==124;io.save=lambda:None
 io.wait=lambda predicate,*args:predicate()
 with pytest.raises(ValueError,match='current task subscriber'):io.wait_task_idle()
 io.received['/grasp/state']=3
 io.wait_task_idle()
 assert io.state['task_process']==new

def test_epoch_change_before_snapshot_state_assignment_is_not_lost():
 io=object.__new__(m.RosIO);io.messages={};io.received={};io.state={};io.fault=None
 io.receive('/alicia_d/control_reference_epoch',SimpleNamespace(stamp=SimpleNamespace(to_nsec=lambda:456)))
 assert io.fault is None # Epoch update was delivered before state was committed.
 io.state={'epoch_ns':'123'}
 with pytest.raises(ValueError,match='epoch changed during snapshot'):io.checkpoint()

def test_restarted_task_subscription_rejects_buffered_old_callback():
 import threading
 io,ident=idle_task_io();io.state={'some':'bound'};io.fault=None
 io.task_state_lock=threading.RLock();io.task_generation=0;io.task_state_type=object
 subscriptions=[];unregistered=[]
 io.task_sub=SimpleNamespace(unregister=lambda:unregistered.append(True))
 def subscribe(topic,typ,callback,**kwargs):
  subscriptions.append(callback);return SimpleNamespace(get_num_connections=lambda:1)
 io.ros=SimpleNamespace(Subscriber=subscribe)
 # A pre-restart callback is already waiting on the same short state lock.
 with io.task_state_lock:
  old=threading.Thread(target=lambda:io.receive_task_state(SimpleNamespace(active=True),0))
  old.start();io.renew_task_subscription()
 old.join(timeout=1)
 assert not old.is_alive() and unregistered==[True]
 assert '/grasp/state' not in io.messages and io.fault is None
 subscriptions[0](SimpleNamespace(active=False))
 assert io.messages['/grasp/state'].active is False
 io.receive_task_state(SimpleNamespace(active=True),0)
 assert io.messages['/grasp/state'].active is False and io.fault is None

REAL_SDK_Q=[-1.859184714917397,.47553404424454876,-.09510680884890975,-.0030679615757712823,-.34974761963792617,-.006135923151542565]
def joint_message(names,values,frame,stamp=1000):
 return SimpleNamespace(name=list(names),position=list(values),header=SimpleNamespace(frame_id=frame,stamp=SimpleNamespace(to_nsec=lambda:stamp)))
def sdk_io():
 import threading
 io=object.__new__(m.RosIO);io.messages={};io.received={};io.state={};io.fault=None;io.sdk_state_lock=threading.RLock()
 io.sdk_frames={'/alicia_d/sdk_command':{},'/alicia_d/control_reference':{}}
 return io

def test_real_sdk_six_schema_and_measured_seven_schema_keep_units():
 io=sdk_io();sdk=joint_message(m.NAMES[:6],REAL_SDK_Q,'sdk_transmitted')
 accepted=joint_message(m.NAMES,REAL_SDK_Q+[.04975],'sdk_measured')
 assert io.sdk_q(sdk)==REAL_SDK_Q
 assert io.q(accepted)==REAL_SDK_Q+[.04975]
 with pytest.raises(ValueError,match='schema'):io.q(sdk)
 with pytest.raises(ValueError):io.sdk_q(accepted)
 reordered=joint_message(list(reversed(m.NAMES)),list(reversed(REAL_SDK_Q+[.04975])),'')
 assert io.q(reordered)==REAL_SDK_Q+[.04975]

@pytest.mark.parametrize('mutate',[
 lambda msg:msg.name.append('right_finger'),
 lambda msg:msg.position.pop(),
 lambda msg:msg.name.__setitem__(1,'Joint1'),
 lambda msg:msg.position.__setitem__(0,float('nan')),
 lambda msg:setattr(msg.header,'frame_id','sdk_measured'),
])
def test_sdk_exact_six_rejects_bad_shape_names_values_or_provenance(mutate):
 io=sdk_io();msg=joint_message(m.NAMES[:6],REAL_SDK_Q,'sdk_transmitted');mutate(msg)
 with pytest.raises(ValueError):io.sdk_q(msg)

def test_snapshot_pairs_cached_exact_write_despite_asynchronous_latest_slots():
 io=sdk_io()
 io.receive('/alicia_d/sdk_command',joint_message(m.NAMES[:6],REAL_SDK_Q,'sdk_transmitted',1000))
 io.receive('/alicia_d/control_reference',joint_message(m.NAMES,REAL_SDK_Q+[.04975],'sdk_manual',1000))
 io.receive('/alicia_d/sdk_command',joint_message(m.NAMES[:6],REAL_SDK_Q,'sdk_transmitted',2000))
 # Latest SDK and reference have different stamps; the confirmed prior pair remains exact.
 held=io.sdk_hold_pair(1000)
 assert held['stamp_ns']=='1000' and held['reference_q'][-1]==.04975
 assert held['units']==['rad']*6+['m'] and held['arm_q_rad']==REAL_SDK_Q
 assert io.sdk_hold_pair(1500) is None
 io.receive('/alicia_d/control_reference',joint_message(m.NAMES,REAL_SDK_Q+[.04975],'sdk_manual',2000))
 assert io.sdk_hold_pair(1500)['stamp_ns']=='2000'

def test_reference_cannot_supply_a_different_arm_for_the_same_sdk_write():
 io=sdk_io();other=REAL_SDK_Q.copy();other[0]+=.001
 io.receive('/alicia_d/sdk_command',joint_message(m.NAMES[:6],REAL_SDK_Q,'sdk_transmitted'))
 io.receive('/alicia_d/control_reference',joint_message(m.NAMES,other+[.04975],'sdk_manual'))
 with pytest.raises(ValueError,match='disagree'):io.sdk_hold_pair()

def test_gripper_only_transmitted_change_is_latched_even_if_arm_sdk_unchanged():
 io=sdk_io();io.state={'sdk_q':REAL_SDK_Q,'control_reference_q':REAL_SDK_Q+[.04975]}
 io.receive('/alicia_d/sdk_command',joint_message(m.NAMES[:6],REAL_SDK_Q,'sdk_transmitted'))
 assert io.fault is None
 io.receive('/alicia_d/control_reference',joint_message(m.NAMES,REAL_SDK_Q+[.04970],'sdk_manual'))
 assert io.fault=='held SDK arm/gripper reference changed'


def test_sdk_pair_cache_is_bounded_to_four_messages_per_topic():
 io=sdk_io()
 for stamp in range(1,11):
  io.receive('/alicia_d/sdk_command',joint_message(m.NAMES[:6],REAL_SDK_Q,'sdk_transmitted',stamp))
 assert set(io.sdk_frames['/alicia_d/sdk_command'])=={7,8,9,10}

def shutdown_io(exited=False):
 io=object.__new__(m.RosIO);ident={'pid':321,'starttime':'old','cmd_sha256':'original','uri':'http://consumer'}
 io.state={'protected':{},'consumers_before':{'/remote_grasp6d_node':ident}};io.previous=None
 io.original_pid_exited=lambda n:exited;io.same_pid=lambda n:True;io.save=lambda:None
 io.wait=lambda predicate,*args:predicate() or (_ for _ in ()).throw(RuntimeError('still alive'))
 return io,ident

def test_shutdown_lost_ack_is_accepted_only_after_original_process_exit():
 io,ident=shutdown_io();alive=[True]
 io.original_pid_exited=lambda n:not alive[0]
 def shutdown(*args):
  alive[0]=False;raise ConnectionRefusedError('consumer exited before ack')
 io.rpc=lambda uri:SimpleNamespace(shutdown=shutdown)
 io.shutdown_consumers()
 evidence=io.state['consumer_shutdown_evidence'][0]
 assert evidence['original_process_exited'] is True and evidence['pid']==321
 assert 'transport error' in evidence['reason'] and 'before ack' in evidence['transport_error']

def test_shutdown_port_closed_but_process_alive_still_fails():
 io,ident=shutdown_io()
 io.rpc=lambda uri:SimpleNamespace(shutdown=lambda *args:(_ for _ in ()).throw(ConnectionRefusedError('port only')))
 with pytest.raises(ConnectionRefusedError):io.shutdown_consumers()
 assert 'consumer_shutdown_evidence' not in io.state

def test_already_exited_original_consumer_needs_no_rpc():
 io,ident=shutdown_io(True);io.rpc=lambda *args:pytest.fail('do not contact an exited process URI')
 io.shutdown_consumers()
 assert io.state['consumer_shutdown_evidence'][0]['original_process_exited'] is True

def test_changed_cmdline_or_pid_lookup_permission_error_is_not_exit():
 io=object.__new__(m.RosIO);ident={'pid':1,'starttime':'old','cmd_sha256':'old'}
 io.pid_identity=lambda p:dict(ident,cmd_sha256='changed')
 assert io.original_pid_exited(ident) is False
 io.pid_identity=lambda p:dict(ident,starttime='new')
 assert io.original_pid_exited(ident) is True # The original process ended before PID reuse.
 io.pid_identity=lambda p:(_ for _ in ()).throw(PermissionError('cannot inspect'))
 with pytest.raises(PermissionError):io.original_pid_exited(ident)
 io.pid_identity=lambda p:(_ for _ in ()).throw(FileNotFoundError('proc absent'))
 assert io.original_pid_exited(ident) is True


def test_stale_optional_node_requires_bound_uri_and_original_exit():
 io=object.__new__(m.RosIO);io.MasterError=ValueError
 io.master=SimpleNamespace(lookupNode=lambda n:'http://old')
 io.rpc=lambda uri:SimpleNamespace(getPid=lambda n:(_ for _ in ()).throw(ConnectionRefusedError('closed')))
 io.previous={'task_process':{'uri':'http://old','pid':123}}
 io.original_pid_exited=lambda old:True
 assert io.node('/grasp_task_node',True) is None
 io.original_pid_exited=lambda old:False
 with pytest.raises(ConnectionRefusedError):io.node('/grasp_task_node',True)
 io.original_pid_exited=lambda old:True
 io.previous['task_process']['uri']='http://different'
 with pytest.raises(ConnectionRefusedError):io.node('/grasp_task_node',True)
 with pytest.raises(ConnectionRefusedError):io.node('/grasp_task_node',False)
