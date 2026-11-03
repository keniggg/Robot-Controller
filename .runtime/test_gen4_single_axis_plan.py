import json,sys,pathlib,unittest
from unittest.mock import patch
import numpy as np
sys.path.insert(0,str(pathlib.Path(__file__).parent))
import gen4_single_axis_plan as p
OUT=p.OUT/'20261003T065823Z'
class PlanTests(unittest.TestCase):
 def setUp(self):
  self.s=json.load((OUT/'snapshot.json').open());self.xml=(OUT/'robot_description.urdf').read_text();self.calls=[];self.valid=True
  def validity(q,g):self.calls.append((q.copy(),g));return self.valid
  self.a=p.PreparedExecutorAudit(self.s,self.xml,validity)
  from tf.transformations import quaternion_from_matrix
  x=np.array(self.s['T_tool_camera']);self.params={'/robot_description':self.xml,'/camera':dict(zip(['fx','fy','cx','cy'],self.s['K'])),'/handeye':{'translation_xyz':x[:3,3].tolist(),'rotation_xyzw':quaternion_from_matrix(x).tolist()}}
  self.patcher=patch('rospy.get_param',side_effect=lambda k:self.params[k]);self.patcher.start()
 def tearDown(self):self.patcher.stop()
 def test_source_and_three_degree_geometry(self):
  for j in [1,4]:
   for sign in [-1,1]:
    target=self.a.source_q.copy();target[j]+=np.radians(sign*3);r=self.a.guard_segment(self.a.source_q,target);self.assertTrue(r['all_geometry_pass']);self.assertTrue(r['all_moveit_valid']);self.assertGreater(r['minimum_clearance_m'],.04)
 def test_collision_rechecked_every_guard(self):
  q=self.a.source_q;r=self.a.guard_segment(q,q);n=len(self.calls);self.assertGreater(n,0);self.valid=False
  with self.assertRaises(RuntimeError):self.a.guard_segment(q,q)
  self.assertGreater(len(self.calls),n)
 def test_current_model_and_handeye_changes_refuse(self):
  self.params['/robot_description']+=' '
  with self.assertRaisesRegex(RuntimeError,'URDF'):self.a.guard_segment(self.a.source_q,self.a.source_q)
  self.params['/robot_description']=self.xml;self.params['/handeye']['translation_xyz'][0]+=.001
  with self.assertRaisesRegex(RuntimeError,'handeye'):self.a.guard_segment(self.a.source_q,self.a.source_q)
 def test_frozen_response_envelope(self):
  q=self.a.source_q.copy();q[4]+=np.radians(4.81)
  with self.assertRaisesRegex(RuntimeError,'envelope'):self.a.guard_segment(self.a.source_q,q)
 def test_collision_missing_is_not_execution_pass(self):
  a=p.GeometryAudit(self.s,self.xml,None);r=a.guard_segment(self.a.source_q,self.a.source_q);self.assertFalse(r['all_moveit_valid'])
if __name__=='__main__':unittest.main()
