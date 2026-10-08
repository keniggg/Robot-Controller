#!/usr/bin/env python3
import importlib.util
from pathlib import Path
import unittest
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
s=importlib.util.spec_from_file_location('anisotropy',ROOT/'.runtime/analyze_depth_grid_anisotropy.py')
A=importlib.util.module_from_spec(s);s.loader.exec_module(A)

class Anisotropy(unittest.TestCase):
    def grid(self):return np.array([[x,y,0.] for y in range(1,9) for x in range(1,12)],dtype=float)

    def test_separate_x_y_scales_survive_arbitrary_rigid_pose(self):
        unit=self.grid();rotation=A.G.exp_so3([.5,.2,-.1])
        points=(unit*np.array([.0153,.0147,1.]))@rotation.T+np.array([.03,-.02,.3])
        r=A.metrics(points,unit)
        self.assertAlmostEqual(r['joint_3d_affine_regression']['x_square_mm'],15.3,places=10)
        self.assertAlmostEqual(r['joint_3d_affine_regression']['y_square_mm'],14.7,places=10)
        self.assertAlmostEqual(r['joint_3d_affine_regression']['axis_angle_deg'],90.,places=10)
        self.assertAlmostEqual(r['adjacent_median_x_over_y'],15.3/14.7,places=10)
        self.assertAlmostEqual(r['horizontal_row_square_mm']['mean'],15.3,places=10)
        self.assertAlmostEqual(r['vertical_column_square_mm']['mean'],14.7,places=10)

    def test_shear_is_not_mistaken_for_scale(self):
        unit=self.grid();angle=np.deg2rad(89.4)
        basis=np.array([[.015,0,0],[.015*np.cos(angle),.015*np.sin(angle),0],[0,0,1]])
        r=A.metrics(unit@basis+[0,0,.3],unit)['joint_3d_affine_regression']
        self.assertAlmostEqual(r['x_over_y'],1.,places=10)
        self.assertAlmostEqual(r['axis_angle_deg'],89.4,places=10)

    def test_uniform_scale_preserves_axis_ratio_and_angle(self):
        unit=self.grid()
        for scale in (.01,.015,.022):
            r=A.metrics(unit*scale+[0,0,.3],unit)['joint_3d_affine_regression']
            self.assertAlmostEqual(r['x_over_y'],1.,places=10)
            self.assertAlmostEqual(r['axis_angle_deg'],90.,places=10)

    def test_plane_orientation_alone_can_create_apparent_anisotropy(self):
        unit=self.grid();rotation=A.G.exp_so3([.5,0,.1]);points=(unit-unit.mean(0))*.015@rotation.T+[0,0,.3]
        rows=A.normal_sensitivity(points,unit,rotation[:,2])
        ratios=[x['x_over_y'] for x in rows if abs(x['normal_rotation_deg'])==1.]
        self.assertGreater(max(ratios)-min(ratios),.01)

    def test_missing_second_dimension_rejected(self):
        unit=np.array([[i,0.,0.] for i in range(14)])
        with self.assertRaises(ValueError):A.metrics(unit*.015+[0,0,.3],unit)

if __name__=='__main__':unittest.main()
