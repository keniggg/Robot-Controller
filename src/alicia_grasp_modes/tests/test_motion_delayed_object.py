"""GEN6 exact old ObjectPose crosses the real task callback/action-lock seam."""
from collections import deque
from copy import deepcopy
import json
from pathlib import Path
import threading
from types import SimpleNamespace as NS

import pytest
import rosbag
import rospy
from test_task_modes import module, base

REC=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/grasp_records/20261010T024239.212466Z_bd212804e3d2427d8b64e164bf10cec0/keyframes.bag')
SOURCE=1791600172893596172
ENTRY=1791600173128938913  # Recorded independent receiver time; scheduling fixture, not claimed task log.
SUBMIT=1791600173143591880
AFTER=1791600211270000000

def at(ns):return rospy.Time(*divmod(ns,10**9))

@pytest.fixture(scope='module')
def messages():
 if not REC.exists():pytest.skip('sealed GEN6 fixture on SanStand required')
 values={}
 topics=['/perception/object','/grasp_6d/plan','/grasp_mode/observe_return_session','/grasp_mode/observe_return_measurement_request']
 with rosbag.Bag(str(REC)) as bag:
  actual=bag.get_type_and_topic_info().topics
  # Rich-plan topic name is discovered by its ROS type rather than guessed.
  topics += [t for t,v in actual.items() if v.msg_type.endswith('/Grasp6DPlan')]
  for topic,msg,when in bag.read_messages(topics=list(set(topics)),end_time=at(1791600213100000000)):
   if topic=='/perception/object' and msg.header.stamp.to_nsec() in (SOURCE,1791600172691883802,1791600212695593118):
    values[msg.header.stamp.to_nsec()]=deepcopy(msg)
   if getattr(msg,'plan_id',None)=='35ba464dc75215b81b345351' and getattr(msg,'valid',False):values['plan']=deepcopy(msg)
   if topic=='/grasp_mode/observe_return_session':values['session']=json.loads(msg.data)
   if topic=='/grasp_mode/observe_return_measurement_request':
    d=json.loads(msg.data)
    if d['state']=='executing':values['lease']=d
 assert set(('plan','session','lease',SOURCE,1791600172691883802,1791600212695593118))<=set(values),values.keys()
 return values

class ObservedLock:
 def __init__(self):self.lock=threading.RLock();self.waiting=threading.Event()
 def __enter__(self):
  if threading.current_thread().name=='delayed-source':self.waiting.set()
  self.lock.acquire();return self
 def __exit__(self,*args):self.lock.release()

@pytest.fixture
def context(messages,monkeypatch):
 m=deepcopy(messages);clock=[ENTRY];params={module.SELECTION_PARAM:m['lease']['selection'],'/gui/joint_direct_mode':False,
  '/grasp':dict(target_observation_validity_sec=8.,target_max_drift_m=.04,min_object_confidence=.5,max_object_jump_m=.12)}
 monkeypatch.setattr(rospy.Time,'now',staticmethod(lambda:at(clock[0])))
 monkeypatch.setattr(rospy,'get_param',lambda name,default=None:params.get(name,default))
 monkeypatch.setattr(rospy,'get_param_cached',lambda name,default=None:params.get(name,default))
 monkeypatch.setattr(rospy,'is_shutdown',lambda:False)
 for key in ('loginfo','logwarn','logwarn_throttle'):monkeypatch.setattr(rospy,key,lambda *a,**k:None)
 n=module.ModeAwareGraspTaskNode.__new__(module.ModeAwareGraspTaskNode);n._initialize_mode_boundary()
 n._mode_selection=deepcopy(m['lease']['selection']);n._mode_bound_selection=deepcopy(n._mode_selection);n._mode_ready=True
 n._following_evidence_lock=threading.RLock();n._following_epoch_ns=m['lease']['control_epoch_ns']
 n._following_accepted_history=deque();n._following_sdk_history=deque()
 n.active=True;n.stage=3;n.latest_obj=deepcopy(m[1791600172691883802]);n.latest_obj_time=n.latest_obj.header.stamp
 n.latest_target_geometry=deepcopy(m['plan'].object_geometry)
 n._freeze_execution_plan(m['plan']);n._grasp6d_plan_lock=ObservedLock()
 n._observe_return_session=m['session'];n._observation_measurement_request=deepcopy(m['lease'])
 n._initial_preflight_attempt=dict(ended_ns=None,physical_action_submitted=False,disqualifications=[],plan_digest=n._bound_execution_plan_digest,missing_objects=[],freeze_count=1,plan_source_ns=m['plan'].header.stamp.to_nsec(),plan_track_id=m['plan'].target_track_id,geometry_invalidations=[])
 n._initial_preflight_token=dict(state='armed',request=dict(selection=deepcopy(n._mode_selection),plan_id=m['plan'].plan_id,target_track_id=m['plan'].target_track_id,control_epoch_ns=n._following_epoch_ns,anchor_binding={}),motion_request=deepcopy(m['lease']))
 return NS(node=n,clock=clock,params=params,messages=m)

def delayed(c,msg,after_commit=None):
 n=c.node;errors=[]
 def callback():
  try:n.obj_cb(msg)
  except Exception as e:errors.append(e)
 with n._grasp6d_plan_guard():
  thread=threading.Thread(target=callback,name='delayed-source');thread.start()
  assert n._grasp6d_plan_lock.waiting.wait(2.),'actual callback did not reach shared lock'
  n._initial_preflight_token['state']='committed'
  n._initial_preflight_attempt['physical_action_submitted']=True
  n._initial_preflight_attempt['disqualifications']=['measured_pose_changed']
  c.clock[0]=AFTER
  if after_commit:after_commit()
 thread.join(2.);assert not thread.is_alive();assert not errors,errors


def test_recorded_old_positive_blocked_across_motion_is_discarded_without_authority_change(context):
 c=context;n=c.node;before=n.latest_obj;digest=n._bound_execution_plan_digest
 delayed(c,deepcopy(c.messages[SOURCE]))
 assert not n._execution_authority_revoked
 assert n.latest_obj is before and n._bound_execution_plan_digest==digest
 assert n._initial_preflight_token['superseded_object_sources']==[SOURCE]
 c.clock[0]=1791600212955167293
 fresh=deepcopy(c.messages[1791600212695593118]);n.obj_cb(fresh)
 assert n.latest_obj is fresh and not n._execution_authority_revoked

@pytest.mark.parametrize('fault',['false','new_source','stale_on_entry','foreign_plan','epoch','manual','not_submitted','hard_revoked','jump','nan','frame','confidence','expired_session','hard_disqualification'])
def test_delayed_positive_exception_does_not_hide_actual_hard_failures(context,fault):
 c=context;n=c.node;msg=deepcopy(c.messages[SOURCE]);change=lambda:None
 if fault=='false':msg.detected=False
 elif fault=='new_source':
  msg.header.stamp=at(SUBMIT+1);msg.pose_base.header.stamp=msg.header.stamp;c.clock[0]=SUBMIT+1000000
 elif fault=='stale_on_entry':c.clock[0]=SOURCE+9000000000
 elif fault=='foreign_plan':change=lambda:n._observation_measurement_request.update(view_plan_id='foreign')
 elif fault=='epoch':change=lambda:setattr(n,'_following_epoch_ns',n._following_epoch_ns+1)
 elif fault=='manual':change=lambda:c.params.update({'/gui/joint_direct_mode':True})
 elif fault=='not_submitted':change=lambda:n._initial_preflight_attempt.update(physical_action_submitted=False)
 elif fault=='hard_revoked':change=lambda:setattr(n,'_execution_authority_revoked',True)
 elif fault=='jump':msg.pose_base.pose.position.x+=.2
 elif fault=='nan':msg.pose_base.pose.position.x=float('nan')
 elif fault=='frame':msg.pose_base.header.frame_id='camera'
 elif fault=='confidence':msg.confidence=.1
 elif fault=='expired_session':change=lambda:n._observe_return_session.update(deadline_ns=AFTER-1)
 elif fault=='hard_disqualification':change=lambda:n._initial_preflight_attempt['disqualifications'].append('manual_ownership_seen')
 delayed(c,msg,change)
 assert n._execution_authority_revoked
 assert not n._initial_preflight_token.get('superseded_object_sources')


def test_late_invalid_geometry_retains_original_hard_revocation(context):
 c=context;n=c.node;n._initial_preflight_token['state']='committed';n._initial_preflight_attempt['physical_action_submitted']=True
 bad=deepcopy(c.messages['plan'].object_geometry);bad.valid=False;bad.header.stamp=at(SOURCE);bad.failure_reason='TARGET_LOST: measured target changed'
 n.target_geometry_cb(bad)
 assert n._execution_authority_revoked


def test_original_callback_seam_reproduces_recorded_stale_revocation(context,monkeypatch):
 c=context;n=c.node
 monkeypatch.setattr(n,'_initial_preflight_superseded_object',lambda *args:False)
 delayed(c,deepcopy(c.messages[SOURCE]))
 assert n._execution_authority_revoked
 assert n._last_execution_plan_event=='EXECUTION_AUTHORITY_REVOKED'


def test_committed_callback_uses_original_freshness_before_action_lock(context):
 c=context;n=c.node
 n._initial_preflight_token['state']='committed'
 n._initial_preflight_attempt['physical_action_submitted']=True
 msg=deepcopy(c.messages[SOURCE]);delayed(c,msg)
 assert not n._execution_authority_revoked
 assert n.latest_obj is msg
 assert not n._initial_preflight_token.get('superseded_object_sources')


def test_committed_refresh_does_not_wait_for_physical_action_lock(context):
 n=context.node;n._initial_preflight_token['state']='committed';done=threading.Event()
 with n._grasp6d_plan_guard():
  thread=threading.Thread(target=lambda:(n._initial_preflight_refresh(),done.set()))
  thread.start();assert done.wait(.5)
 thread.join(1.);assert not thread.is_alive()


def test_canonical_lease_and_ros_wire_hashes_remain_distinct(context):
 c=context;n=c.node
 assert c.messages['lease']['plan_sha256']=='4e3fdb606a5dd04847c9d6ab2d6f4ed9dd1d2663b446e33762e0a2524b4a8e7e'
 assert n._bound_execution_plan_digest=='e5948a0a6d1c56acb1aaeaa5854b0029156e6b334a07af7661f4b1f069c9062f'
 delayed(c,deepcopy(c.messages[SOURCE]))
 assert not n._execution_authority_revoked


def test_authoritative_selection_change_rejects_even_if_callback_is_queued(context):
 c=context;n=c.node
 def change():c.params[module.SELECTION_PARAM]=dict(n._mode_selection,generation=7)
 delayed(c,deepcopy(c.messages[SOURCE]),change)
 assert n._execution_authority_revoked
 assert not n._initial_preflight_token.get('superseded_object_sources')


def test_invalid_source_collector_never_qualifies_obsolete_positive(context):
 c=context;n=c.node
 n._initial_preflight_collector=NS(lock=threading.RLock(),fatal='source_content_changed',references={},object=lambda *args:None)
 delayed(c,deepcopy(c.messages[SOURCE]))
 assert n._execution_authority_revoked
 assert not n._initial_preflight_token.get('superseded_object_sources')
