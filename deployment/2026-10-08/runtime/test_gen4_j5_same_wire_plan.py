import json,pathlib,sys,unittest
from unittest.mock import patch
import numpy as np
sys.path.insert(0,str(pathlib.Path(__file__).parent))
import gen4_j5_same_wire_plan as m
class SameWireTests(unittest.TestCase):
 def setUp(self):
  p=pathlib.Path(json.loads((m.OUT/'latest.json').read_text())['directory']);self.s=json.loads((p/'snapshot.json').read_text());self.xml=(p/'robot_description.urdf').read_text();self.valid=True;self.calls=0
  def validity(q,g):self.calls+=1;return self.valid
  self.a=m.J5SameWireAudit(self.s,self.xml,validity)
  from tf.transformations import quaternion_from_matrix
  X=np.array(self.s['T_tool_camera']);cam=dict(zip(['fx','fy','cx','cy'],self.s['K']));cam.update(width=self.s['width'],height=self.s['height'],depth_scale=self.s['depth_scale']);self.params={'/robot_description':self.xml,'/camera':cam,'/handeye':{'translation_xyz':X[:3,3].tolist(),'rotation_xyzw':quaternion_from_matrix(X).tolist()}}
  self.patch=patch('rospy.get_param',side_effect=lambda k:self.params[k]);self.patch.start()
 def tearDown(self):self.patch.stop()
 def test_four_targets_same_wire_return(self):
  original=m.base.words(self.s['sdk_q_rad']);words=[m.base.words(q) for q in self.a.absolute_sdk_targets]
  self.assertEqual([w[4]-original[4] for w in words],[-34,-68,-34,0]);self.assertEqual(words[-1],original)
  for w in words:self.assertEqual(w[:4]+w[5:],original[:4]+original[5:])
  self.assertTrue(self.a.audit_nominal_sequence()['all_pass'])
 def test_real_scene_requery_on_every_submission(self):
  self.a.guard_segment(self.a.source_q,self.a.absolute_sdk_targets[0]);n=self.calls;self.valid=False
  with self.assertRaisesRegex(RuntimeError,'MoveIt'):self.a.guard_segment(self.a.source_q,self.a.absolute_sdk_targets[0])
  self.assertGreater(self.calls,n)
 def test_nonselected_actual_drift(self):
  q=self.a.source_q.copy();q[2]+=np.radians(.501)
  with self.assertRaisesRegex(RuntimeError,'actual axis drift'):self.a.guard_segment(q,self.a.absolute_sdk_targets[0])
 def test_existing_sdk_actual_difference_is_not_new_drift(self):
  self.assertGreater(abs(self.s['sdk_q_rad'][2]-self.s['q_rad'][2]),np.radians(.5));self.assertTrue(self.a.guard_segment(self.a.source_q,self.a.absolute_sdk_targets[0])['all_pass'])
 def test_unedited_sdk_word_rejected(self):
  q=self.a.absolute_sdk_targets[0].copy();q[2]+=m.base.QUANTUM
  with self.assertRaisesRegex(RuntimeError,'unedited SDK word'):self.a.guard_segment(self.a.source_q,q)
 def test_cannot_skip_to_six_degree_target(self):
  with self.assertRaisesRegex(RuntimeError,'exceeds4deg'):self.a.guard_segment(self.a.source_q,self.a.absolute_sdk_targets[1])
 def test_fixed_absolute_bound(self):
  q=self.a.source_q.copy();q[4]=self.a.edited_axis_absolute_bounds_rad[0]-.001
  with self.assertRaisesRegex(RuntimeError,'absolute bounds'):self.a.guard_segment(q,self.a.absolute_sdk_targets[1])
 def test_extra_command_not_authorized_by_response_margin(self):
  q=self.a.absolute_sdk_targets[1].copy();q[4]-=m.base.QUANTUM;a=self.a.source_q.copy();a[4]=q[4]+.01
  with self.assertRaisesRegex(RuntimeError,'word range'):self.a.guard_segment(a,q)
 def test_model_and_gripper_mutation_rejected(self):
  q=self.a.absolute_sdk_targets[0].copy();q[6]+=.001
  with self.assertRaisesRegex(RuntimeError,'gripper'):self.a.guard_segment(self.a.source_q,q)
  self.params['/camera']['depth_scale']*=1.01
  with self.assertRaisesRegex(RuntimeError,'depth_scale'):self.a.guard_segment(self.a.source_q,self.a.absolute_sdk_targets[0])
 def test_nan_rejected(self):
  q=self.a.source_q.copy();q[0]=float('nan')
  with self.assertRaisesRegex(RuntimeError,'invalid actual'):self.a.guard_segment(q,self.a.absolute_sdk_targets[0])
if __name__=='__main__':unittest.main()
