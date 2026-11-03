"""Pure mathematical audit tests; no ROS, archived images or parameter writes."""
import math
from pathlib import Path
import sys
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parent))
import audit_joint_metric_identifiability as m


def transform(t,r):
 T=np.eye(4);T[:3,3]=t;T[:3,:3]=m.matrix(m.Rotation.from_rotvec(r));return T


def synthetic():
 # Generic serial chain, with distinct axes and nonzero metric offsets.
 axes=([0,0,1],[0,1,0],[0,1,0],[1,0,0],[0,1,0],[1,0,0])
 links=([0,0,.1],[.02,0,.13],[.17,0,0],[.14,.01,0],[.03,0,0],[.04,0,0])
 xml='<robot name="metric">'
 for i,(axis,link) in enumerate(zip(axes,links)):
  parent='base_link' if i==0 else 'l%d'%i
  xml+='<joint name="Joint%d" type="revolute"><parent link="%s"/><child link="l%d"/><origin xyz="%s" rpy="0 0 0"/><axis xyz="%s"/></joint>'%(i+1,parent,i+1,' '.join(map(str,link)),' '.join(map(str,axis)))
 xml+='<joint name="tip" type="fixed"><parent link="l6"/><child link="tool0"/><origin xyz=".02 0 0" rpy="0 0 0"/></joint></robot>'
 fk=m.MetricFk(xml);X=transform([.02,-.01,.05],[.1,.2,-.1])
 true=np.array([1.,-2.,3.,.15,-.1,.2,-1.,2.,.25,-.35])
 XT=X.copy();XT[:3,3]+=true[:3]*.001;XT[:3,:3]=m.matrix(m.Rotation.from_rotvec(np.radians(true[3:6])))@X[:3,:3]
 off=np.zeros(6);off[1:5]=np.radians(true[6:10]);Y=transform([.3,.2,.4],[.2,-.2,.1])
 rng=np.random.RandomState(123);rows=[]
 for i,q in enumerate(rng.uniform(-.7,.7,(22,6))):
  C=np.linalg.inv(fk(q+off)@XT)@Y
  rows.append(dict(q=q,C=C,fk=fk,role='training',group=('train',1),capture='synthetic%d'%i))
 return rows,X,true


class MetricAuditTests(unittest.TestCase):
 def test_true_metric_solution_has_zero_residual(self):
  rows,X,z=synthetic();self.assertLess(np.linalg.norm(m.centered_residual(rows,X,z)),1e-9)
 def test_metric_fit_recovers_all_ten_parameters(self):
  rows,X,z=synthetic();f=m.fit_training(rows,X)
  self.assertTrue(f['success']);np.testing.assert_allclose(f['z'],z,atol=1e-6)
 def test_full_rank_and_finite_difference_convergence(self):
  rows,X,_=synthetic();fun=lambda z:m.centered_residual(rows,X,z)
  a=m.central_jacobian(fun,np.zeros(10),.001);b=m.central_jacobian(fun,np.zeros(10),.003)
  s=np.linalg.svd(a,compute_uv=False)
  self.assertEqual(np.sum(s>s[0]*1e-7),10);np.testing.assert_allclose(a,b,rtol=2e-7,atol=2e-7)
 def test_degenerate_repeated_pose_is_not_identifiable(self):
  rows,X,_=synthetic();rows=[rows[0]]*22
  J=m.central_jacobian(lambda z:m.centered_residual(rows,X,z),np.zeros(10),.001)
  self.assertLess(np.linalg.norm(J),1e-7)
  audit=m.audit(rows,X)
  self.assertTrue(all(entry['rank_at_relative_tolerance_1e7']==0 for info in audit.values() for entry in info['jacobian_audits']))
 def test_singletons_are_marked_uninformative(self):
  rows,X,z=synthetic();row=dict(rows[0]);row['role']='validation';row['group']=('heldout',999)
  result=m.metrics([row],X,z)[0]
  self.assertFalse(result['informative']);self.assertIsNone(result['position_pairwise_max_mm'])
 def test_estimator_rejects_validation_sample(self):
  rows,X,_=synthetic();rows[3]=dict(rows[3],role='independent_validation')
  with self.assertRaisesRegex(ValueError,'validation'):m.fit_training(rows,X)
 def test_group_translation_origin_is_eliminated(self):
  rows,X,z=synthetic();allrows=[]
  for group,shift in [('a',[.0,.0,.0]),('b',[.5,-.2,.4])]:
   for row in rows:
    r=dict(row);Y=m.predict([r],X,z)[0];Y[:3,3]+=shift
    corrected=np.zeros(6);corrected[1:5]=np.radians(z[6:10])
    XT=X.copy();XT[:3,3]+=z[:3]*.001;XT[:3,:3]=m.matrix(m.Rotation.from_rotvec(np.radians(z[3:6])))@X[:3,:3]
    r['C']=np.linalg.inv(r['fk'](r['q']+corrected)@XT)@Y;r['group']=(group,1);allrows.append(r)
  self.assertLess(np.linalg.norm(m.centered_residual(allrows,X,z)),2e-9)

if __name__=='__main__':unittest.main()
