import importlib.util,math,pathlib,sys,unittest
from unittest.mock import patch
from types import SimpleNamespace
import numpy as np
sys.path.insert(0,str(pathlib.Path(__file__).parent))
import gen4_high_posture_plan as m

class FakeGeometry:
 def __init__(self,snapshot,xml,validity):self.cache={};self.validity=validity;self.samples=[];self.fail=False
 def geometry(self,q):
  self.samples.append(np.array(q));valid=False if self.fail else bool(self.validity(q,.05))
  return dict(q_rad=list(q),clearance_m=.1,target=dict(edge_px=80.,forward_min_m=.2,forward_max_m=.4),texture=dict(edge_px=80.),geometry_pass=valid,moveit_collision_valid=valid)
 def guard_segment(self,a,b):return self.summarize([self.geometry(a),self.geometry(b)])
 summarize=staticmethod(m.base.GeometryAudit.summarize)

class Tests(unittest.TestCase):
 def setUp(self):
  self.q=np.array([-1.8,.5,-.15,-.1,-.4,.04,.04975]);self.sdk=self.q.copy();self.sdk[2]+=.04
  self.s=dict(source_stamp_ns=1,q_rad=self.q.tolist(),sdk_q_rad=self.sdk.tolist(),epoch_ns=11)
  self.a=m.HighPostureAudit(self.s,'xml',lambda q,g:True,audit_factory=FakeGeometry)
  self.a.check_model=lambda:None
 def prepare(self):self.a.audit_nominal_sequence(False)
 def advance(self,n):
  q=self.q.copy();sdk=self.sdk.copy();self.prepare()
  for i in range(n):
   step=self.a.begin_step(i,q,sdk);q[step['axis']]=step['target_sdk'][step['axis']];sdk=np.array(step['target_sdk']);self.a.complete_step(q,sdk)
  return q,sdk
 def test_sequence_all_scalar_and_exact_return(self):
  for cycles in (5,7,8):
   for sign in (-1,1):
    s=m.build_sequence(self.sdk,cycles,sign);self.assertEqual(len(s),4*cycles+4)
    for step in s:
     d=np.array(step['target_words'])-step['before_words'];self.assertEqual(np.count_nonzero(d),1);self.assertEqual(abs(d[step['axis']]),34)
     self.assertEqual([step['target_words'][i] for i in (0,2,3,5)],[m.base.words(self.sdk)[i] for i in (0,2,3,5)])
    self.assertEqual(s[-1]['target_sdk'],self.sdk.tolist())
    self.assertEqual([x['axis'] for x in s[:2*cycles]],[1,4]*cycles)
 def test_reject_nonfinite_and_invalid_cycles(self):
  for q,cycles,sign in (([float('nan')]*7,7,1),(self.sdk,6,1),(self.sdk,7,0)):
   with self.assertRaises(RuntimeError):m.build_sequence(q,cycles,sign)
 def test_full_audit_before_any_step(self):
  with self.assertRaisesRegex(RuntimeError,'missing audit'):self.a.begin_step(0,self.q,self.sdk)
  proof=self.a.audit_nominal_sequence(False);self.assertEqual(len(proof['proof_steps']),32);self.assertTrue(proof['all_pass'])
 def test_failed_full_audit_not_prepared(self):
  self.a.audit.fail=True
  with self.assertRaisesRegex(RuntimeError,'MoveIt'):self.prepare()
  self.assertFalse(self.a.prepared)
 def test_runtime_scene_rechecked(self):
  self.prepare();s=self.a.begin_step(0,self.q,self.sdk);self.a.guard_segment(self.q,s['target_sdk']);self.a.audit.fail=True
  with self.assertRaisesRegex(RuntimeError,'MoveIt'):self.a.guard_segment(self.q,s['target_sdk'])
 def test_cannot_skip_repeat_or_overlap(self):
  self.prepare()
  with self.assertRaises(RuntimeError):self.a.begin_step(1,self.q,self.sdk)
  self.a.begin_step(0,self.q,self.sdk)
  with self.assertRaises(RuntimeError):self.a.begin_step(0,self.q,self.sdk)
 def test_previous_sdk_must_match(self):
  self.prepare();bad=self.sdk.copy();bad[4]+=m.base.QUANTUM
  with self.assertRaisesRegex(RuntimeError,'previous step'):self.a.begin_step(0,self.q,bad)
 def test_global_unedited_actual_drift(self):
  self.prepare();q=self.q.copy();q[3]+=math.radians(.501)
  with self.assertRaisesRegex(RuntimeError,'globally unedited'):self.a.begin_step(0,q,self.sdk)
 def test_held_sdk_actual_offset_is_not_new_drift(self):
  self.prepare();s=self.a.begin_step(0,self.q,self.sdk);self.assertTrue(self.a.guard_segment(self.q,s['target_sdk'])['all_pass'])
 def test_per_step_other_editable_axis_drift(self):
  self.prepare();s=self.a.begin_step(0,self.q,self.sdk);q=self.q.copy();q[4]-=math.radians(.501)
  with self.assertRaisesRegex(RuntimeError,'per-step unedited'):self.a.check_during_step(q)
 def test_cannot_invent_target_or_edit_held_word(self):
  self.prepare();s=self.a.begin_step(0,self.q,self.sdk)
  for axis in (1,2,4):
   q=np.array(s['target_sdk']);q[axis]+=m.base.QUANTUM
   with self.assertRaisesRegex(RuntimeError,'frozen current step'):self.a.guard_segment(self.q,q)
 def test_recovery_actions_rejected(self):
  self.prepare();s=self.a.begin_step(0,self.q,self.sdk)
  for action in ('enable','sync','probe','timeout','clear'):
   with self.assertRaisesRegex(RuntimeError,'forbids'):m.bound_publish_target(self.a,action,s['target_sdk'],1)
 def test_publish_preserves_all_frozen_held_words(self):
  self.prepare();s=self.a.begin_step(0,self.q,self.sdk);auto=self.q.copy();auto[1]=s['target_sdk'][1]
  actual=m.bound_publish_target(self.a,'publish',auto,1);np.testing.assert_array_equal(actual,s['target_sdk'])
  with self.assertRaises(RuntimeError):m.bound_publish_target(self.a,'publish',auto,4)
 def test_margin_not_compounded(self):
  self.prepare();s=self.a.begin_step(0,self.q,self.sdk);p=self.a.guard_segment(self.q,s['target_sdk']);q=self.q.copy();q[1]=s['target_sdk'][1]+math.radians(.4);p2=self.a.guard_segment(q,s['target_sdk'])
  self.assertEqual(p['fixed_step_response_bounds_deg'],p2['fixed_step_response_bounds_deg'])
  q[1]=s['target_sdk'][1]+math.radians(.801)
  with self.assertRaisesRegex(RuntimeError,'fixed step response'):self.a.check_during_step(q)
 def test_four_degree_actual_limit(self):
  q,sdk=self.advance(4);q[1]-=math.radians(1.1);s=self.a.begin_step(4,q,sdk)
  with self.assertRaisesRegex(RuntimeError,'exceeds4deg'):self.a.guard_segment(q,s['target_sdk'])
 def test_complete_requires_final_profile_words(self):
  self.prepare();s=self.a.begin_step(0,self.q,self.sdk);q=self.q.copy();q[1]=s['target_sdk'][1]
  with self.assertRaisesRegex(RuntimeError,'exact frozen words'):self.a.complete_step(q,self.sdk)
  self.assertEqual(self.a.cursor,0);self.a.complete_step(q,s['target_sdk']);self.assertEqual(self.a.cursor,1)
 def test_measured_return_not_assumed_exact(self):
  q,sdk=self.advance(32);q[1]-=math.radians(.2);self.a.check_actual(q);self.assertNotEqual(q.tolist(),self.q.tolist());self.assertEqual(sdk.tolist(),self.sdk.tolist())
 def test_gripper_never_changes(self):
  self.prepare();bad=self.q.copy();bad[6]+=.001
  with self.assertRaisesRegex(RuntimeError,'gripper'):self.a.begin_step(0,bad,self.sdk)

 def test_five_cycles_from_second_endpoint_retains_the_same_high_domain_words(self):
  original=m.build_sequence(self.sdk,7,1)
  current=original[3]['target_sdk']
  shorter=m.build_sequence(current,5,1)
  self.assertEqual(len(shorter),24)
  for new,old in zip(shorter,original[4:28]):
   self.assertEqual(new['target_words'],old['target_words'])
   self.assertEqual(new['target_sdk'][6],old['target_sdk'][6])
  self.assertEqual(shorter[-1]['target_words'],m.base.words(current))

class ProfileFinalTests(unittest.TestCase):
 @classmethod
 def setUpClass(cls):
  path=pathlib.Path(__file__).parent/'gen4_root_cause';sys.path.insert(0,str(path));spec=importlib.util.spec_from_file_location('collector_high',str(path/'collect_high_posture.py'));cls.c=importlib.util.module_from_spec(spec);spec.loader.exec_module(cls.c)
 def test_partial_profile_cannot_complete(self):
  goal=np.array([0.,.4,0.,0.,-.4,0.,.05]);partial=goal.copy();partial[1]-=m.base.QUANTUM
  answers=iter([(goal,partial,101),(goal,goal,99),(goal,goal,102)]);calls=[]
  def get(plan):calls.append(1);return next(answers)
  r=SimpleNamespace(source_guard=get,mark=lambda *a,**k:None);motion=SimpleNamespace(guard=lambda:None)
  with patch.object(self.c.time,'sleep'),patch.object(self.c.rospy.Time,'now',return_value=SimpleNamespace(to_nsec=lambda:200)):
   self.assertEqual(self.c.wait_profile_final(r,motion,None,goal,100),200)
  self.assertEqual(len(calls),3)
 def test_profile_timeout_no_completion(self):
  goal=np.array([0.,.4,0.,0.,-.4,0.,.05]);partial=goal.copy();partial[1]-=m.base.QUANTUM
  r=SimpleNamespace(source_guard=lambda _: (goal,partial,101),mark=lambda *a,**k:None);motion=SimpleNamespace(guard=lambda:None)
  with patch.object(self.c.time,'monotonic',side_effect=[0.,0.,19.]),patch.object(self.c.time,'sleep'):
   with self.assertRaisesRegex(RuntimeError,'profile did not reach'):self.c.wait_profile_final(r,motion,None,goal,100)
class SourceBoundaryTests(ProfileFinalTests):
 def setUp(self):
  mock=patch.object(self.c.Recorder,'guard',lambda _:None);mock.start();self.addCleanup(mock.stop)
 def recorder(self):
  import threading
  r=self.c.BoundRecorder.__new__(self.c.BoundRecorder);r.lock=threading.RLock();r.epoch_ns=11;r.guard=lambda:None;r.frames={900000000:{'color':1,'depth':1}}
  r.latest_inputs={'/alicia_d/feedback_ready':(SimpleNamespace(data=True),0),'/alicia_d/actuation_status':(SimpleNamespace(data='CONFIRMED:test'),0)}
  r.joint_and_sdk=lambda:(np.zeros(7),np.zeros(7),950000000)
  p=SimpleNamespace(snapshot={'epoch_ns':11},check_during_step=lambda q:q,active=None)
  return r,p
 def test_source_epoch_is_immutable(self):
  r,p=self.recorder();r.epoch_ns=12
  with self.assertRaisesRegex(RuntimeError,'epoch'):r.source_guard(p)
 def test_no_bootstrap_on_unconfirmed(self):
  r,p=self.recorder();r.latest_inputs['/alicia_d/actuation_status']=(SimpleNamespace(data='UNCONFIRMED:recovery'),0)
  with self.assertRaisesRegex(RuntimeError,'no bootstrap'):r.source_guard(p)
 def test_camera_source_age_and_future_rejected(self):
  for stamp in (100000000,1100000000):
   r,p=self.recorder();r.frames={stamp:{'color':1,'depth':1}}
   with patch.object(self.c.rospy.Time,'now',return_value=SimpleNamespace(to_nsec=lambda:1000000000)):
    with self.assertRaisesRegex(RuntimeError,'camera source'):r.source_guard(p)
 def test_held_profile_word_cannot_change(self):
  r,p=self.recorder();p.active={'index':0};before=m.base.words(np.zeros(7));target=before.copy();target[1]+=34;p.sequence=[dict(axis=1,before_words=before,target_words=target)];sdk=np.zeros(7);sdk[4]=m.base.QUANTUM;r.joint_and_sdk=lambda:(np.zeros(7),sdk,950000000)
  with patch.object(self.c.rospy.Time,'now',return_value=SimpleNamespace(to_nsec=lambda:1000000000)):
   with self.assertRaisesRegex(RuntimeError,'unedited SDK'):r.source_guard(p)
 def test_profile_may_not_overshoot_frozen_words(self):
  r,p=self.recorder();p.active={'index':0};before=m.base.words(np.zeros(7));target=before.copy();target[1]+=34;p.sequence=[dict(axis=1,before_words=before,target_words=target)];sdk=np.zeros(7);sdk[1]=35*m.base.QUANTUM;r.joint_and_sdk=lambda:(np.zeros(7),sdk,950000000)
  with patch.object(self.c.rospy.Time,'now',return_value=SimpleNamespace(to_nsec=lambda:1000000000)):
   with self.assertRaisesRegex(RuntimeError,'outside frozen transition'):r.source_guard(p)

class ConstructorAndWaitTests(ProfileFinalTests):
 def test_post_construct_accepts_unchanged_sources(self):
  q=np.zeros(7);p=SimpleNamespace(source_q=q[:6],reference=q)
  self.c.verify_constructed_source(p,q,q,q)
 def test_post_construct_rejects_each_feedback_shift(self):
  q=np.zeros(7);p=SimpleNamespace(source_q=q[:6],reference=q);bad=q.copy();bad[1]=math.radians(.201)
  for actual,motion in ((bad,q),(q,bad)):
   with self.assertRaisesRegex(RuntimeError,'posture changed'):self.c.verify_constructed_source(p,actual,q,motion)
 def test_post_construct_rejects_sdk_or_gripper_change(self):
  q=np.zeros(7);p=SimpleNamespace(source_q=q[:6],reference=q)
  for axis,delta in ((4,m.base.QUANTUM),(6,.001)):
   sdk=q.copy();sdk[axis]=delta
   with self.assertRaisesRegex(RuntimeError,'SDK changed'):self.c.verify_constructed_source(p,q,sdk,q)
 def test_snapshot_wait_guard_propagates_authority_loss(self):
  r=self.c.BoundRecorder.__new__(self.c.BoundRecorder)
  def lost():raise RuntimeError('GUI ownership changed during snapshot wait')
  r.wait_guard_callback=lost
  with patch.object(self.c.Recorder,'guard',lambda _:None):
   with self.assertRaisesRegex(RuntimeError,'snapshot wait'):r.guard()

if __name__=='__main__':unittest.main()
