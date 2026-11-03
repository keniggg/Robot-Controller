import math,pathlib,sys,unittest
import numpy as np
sys.path.insert(0,str(pathlib.Path(__file__).parent/'gen4_root_cause'))
import review_error_pivot as m


class Tests(unittest.TestCase):
 def test_rotation_keeps_points_on_the_physical_axis_fixed(self):
  u=np.array([1.,2.,3.]);u/=np.linalg.norm(u);p=np.array([.2,-.3,.17]);E=m.axis_transform(u,p,.03)
  pts=np.array([p-2*u,p,p+.4*u]);np.testing.assert_allclose(m.mappts(E,pts),pts,atol=1e-14)
 def test_virtual_angle_sign_and_distinct_parallel_pivots_from_independent_points(self):
  rng=np.random.RandomState(61);P=rng.uniform(-.1,.2,(90,3));P[:,2]+=.4
  F0=np.eye(4);F=np.eye(4);F[:3,3]=[.05,-.03,.18];u=np.array([1.,0.,0.]);p=np.array([0.,0.,.17]);delta=math.radians(2.5)
  Q=m.mappts(np.linalg.inv(m.axis_transform(u,p,delta)@F)@F0,P);train=np.arange(60);test=np.arange(60,90)
  estimate=m.fit_angle(F0,F,u,p,P,Q,train);self.assertAlmostEqual(estimate,delta,places=8)
  good=m.mappts(np.linalg.inv(m.axis_transform(u,p,estimate)@F)@F0,P[test]);self.assertLess(np.linalg.norm(good-Q[test]),1e-8)
  wrong=m.fit_angle(F0,F,u,np.array([0.,0.,.33]),P,Q,train)
  bad=m.mappts(np.linalg.inv(m.axis_transform(u,np.array([0.,0.,.33]),wrong)@F)@F0,P[test]);self.assertGreater(np.mean(np.linalg.norm(bad-Q[test],axis=1)),.002)
 def test_holdout_does_not_choose_angle(self):
  rng=np.random.RandomState(9);P=rng.uniform(-.2,.2,(30,3));P[:,2]+=.4;Q=P.copy();F=np.eye(4);train=np.arange(20)
  one=m.fit_angle(F,F,[1.,0.,0.],[0.,0.,.17],P,Q,train);Q[20:]+=100
  two=m.fit_angle(F,F,[1.,0.,0.],[0.,0.,.17],P,Q,train);self.assertEqual(one,two)
 def test_all_points_in_same_image_tile_stay_together(self):
  uv=np.array([[10.,10.],[20.,20.],[80.,10.],[85.,11.],[150.,20.],[155.,21.]])
  folds=m.spatial_folds(uv);self.assertEqual(folds[0],folds[1]);self.assertEqual(folds[2],folds[3]);self.assertEqual(folds[4],folds[5])


if __name__=='__main__':unittest.main()
