#!/usr/bin/env python3
import importlib.util
from pathlib import Path
import unittest
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
s=importlib.util.spec_from_file_location('jointscale',ROOT/'.runtime/audit_joint_scale_rotation_sensitivity.py')
S=importlib.util.module_from_spec(s);s.loader.exec_module(S)
FK=S.Z.RotationFk((ROOT/'src/real-arm/alicia_d_descriptions/urdf/alicia_duo_with_gripper.urdf').read_text())
class ScaleAudit(unittest.TestCase):
    def data(self,truth):
        rng=np.random.RandomState(123);rows=[]
        for i in range(20):
            q=rng.uniform(-.9,.9,6);rows.append(dict(dataset='fixed',epoch=1,urdf_sha256='test',q=q,fk=FK,camera=FK(q*np.asarray(truth)).T,capture=str(i)))
        return rows,S.Z.pairs(rows,list(range(len(rows))))
    def test_six_known_scales_recovered_without_offset_fit(self):
        truth=np.array([1.02,.98,1.04,1.01,.96,1.03]);rows,pp=self.data(truth)
        fit=S.solve(rows,pp);np.testing.assert_allclose(fit.x,truth,atol=1e-6)
        self.assertLess(np.max(abs(fit.fun)),1e-6)
    def test_common_known_scale_recovered(self):
        rows,pp=self.data(1.018);fit=S.solve(rows,pp,True)
        self.assertAlmostEqual(fit.x[0],1.018,places=7)
    def test_ones_match_nominal_rotation_residual(self):
        rows,pp=self.data(np.ones(6));self.assertLess(np.max(abs(S.residual(rows,pp,[1]))),1e-8)
    def test_invalid_gain_shape_rejected(self):
        rows,pp=self.data(np.ones(6))
        with self.assertRaises(ValueError):S.residual(rows,pp,[1,1])
if __name__=='__main__':unittest.main()
