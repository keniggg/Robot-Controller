#!/usr/bin/env python3
import importlib.util
from pathlib import Path
import unittest
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('depth_audit',ROOT/'.runtime/analyze_depth_supported_board_motion.py')
D=importlib.util.module_from_spec(spec);spec.loader.exec_module(D)

class DepthGeometry(unittest.TestCase):
    def grid(self):
        return np.array([[x,y,0.] for y in range(1,9) for x in range(1,12)],dtype=float)

    def test_kabsch_recovers_planar_motion_without_reflection(self):
        points=self.grid()*.018
        rotation=D.GEOM.exp_so3([.03,-.06,.02]);translation=np.array([.013,-.007,.01])
        target=points@rotation.T+translation
        got,metrics=D.rigid(points,target)
        np.testing.assert_allclose(got[:3,:3],rotation,atol=1e-12)
        np.testing.assert_allclose(got[:3,3],translation,atol=1e-12)
        self.assertAlmostEqual(np.linalg.det(got[:3,:3]),1.,places=12)
        self.assertLess(metrics['rms_mm'],1e-9)

    def test_collinear_correspondences_rejected(self):
        line=np.array([[i*.01,0,0] for i in range(8)])
        with self.assertRaises(ValueError):D.rigid(line,line)

    def test_depth_plane_ransac_rejects_nonplanar_outliers(self):
        rng=np.random.default_rng(123);xy=rng.uniform(-.06,.06,(1000,2))
        points=np.column_stack([xy,.3+.1*xy[:,0]-.2*xy[:,1]])
        points[:,2]+=rng.normal(0,.0001,len(points));points[:100,2]+=.02
        n,d,metrics=D.fit_plane(points)
        expected=np.array([-.1,.2,1.]);expected/=np.linalg.norm(expected)
        self.assertGreater(n@expected,.99999)
        self.assertLess(abs(d+.3/np.sqrt(1.05)),.0001)
        self.assertLess(metrics['inlier_rms_mm'],.2)
        self.assertLess(metrics['inlier_fraction'],.92)

    def test_grid_metric_comes_from_depth_not_assumed_15mm(self):
        unit=self.grid();rot=D.GEOM.exp_so3([.2,-.1,.05])
        for scale in (.013,.017,.022):
            points=unit*scale@rot.T+np.array([.1,-.1,.3])
            result=D.grid_spacing(points,unit)
            self.assertAlmostEqual(result['similarity_square_mm'],scale*1000,places=9)
            self.assertLess(result['similarity_residual_rms_mm'],1e-9)

    def test_plane_ray_intersection_recovers_grid_scale(self):
        points=self.grid()*.019;points[:,0]-=.1;points[:,1]-=.08
        points[:,2]=.3+.1*points[:,0]-.2*points[:,1]
        n=np.array([-.1,.2,1.]);n/=np.linalg.norm(n);offset=-.3/np.sqrt(1.05)
        rays=points/points[:,2,None]
        recovered=rays*(-offset/(rays@n))[:,None]
        np.testing.assert_allclose(recovered,points,atol=1e-12)

    def test_low_kabsch_residual_does_not_prove_absolute_motion(self):
        points=self.grid()*.015
        # A common biased plane can generate a perfectly rigid spurious shift.
        target=points+np.array([0,0,.001])
        got,metrics=D.rigid(points,target)
        self.assertLess(metrics['rms_mm'],1e-9)
        self.assertAlmostEqual(np.linalg.norm(got[:3,3])*1000,1.,places=10)

    def test_relative_transform_inverse_matches_fixed_board_convention(self):
        unit=self.grid()*.015
        c0=np.eye(4);c0[:3,:3]=D.GEOM.exp_so3([.2,.1,-.1]);c0[:3,3]=[.03,.06,.3]
        c1=np.eye(4);c1[:3,:3]=D.GEOM.exp_so3([.24,.13,-.11]);c1[:3,3]=[.05,.04,.31]
        p0=unit@c0[:3,:3].T+c0[:3,3];p1=unit@c1[:3,:3].T+c1[:3,3]
        mapping,_=D.rigid(p0,p1)
        np.testing.assert_allclose(D.GEOM.inv(mapping),c0@D.GEOM.inv(c1),atol=1e-12)

if __name__=='__main__':unittest.main()
