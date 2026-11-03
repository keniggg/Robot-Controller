#!/usr/bin/env python3
import importlib.util
from pathlib import Path
import unittest
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
s=importlib.util.spec_from_file_location('jointzero',ROOT/'.runtime/audit_joint_zero_rotation_sensitivity.py')
J=importlib.util.module_from_spec(s);s.loader.exec_module(J)
XML=(ROOT/'src/real-arm/alicia_d_descriptions/urdf/alicia_duo_with_gripper.urdf').read_text()
FK=J.RotationFk(XML)

class ZeroInvariants(unittest.TestCase):
    def poses(self):
        rng=np.random.RandomState(309)
        base=np.array([-1.8,.3,.4,.2,-.3,.1])
        return [base+rng.uniform(-.3,.3,6) for _ in range(16)]

    def test_j2_and_j3_offsets_only_identify_sum_in_rotation(self):
        for q in self.poses():
            qa=q.copy();qb=q.copy();qa[1]+=.13;qb[2]+=.13
            np.testing.assert_allclose(FK(qa),FK(qb),atol=1e-12)

    def test_j1_global_zero_does_not_change_relative_angle(self):
        qa,qb=self.poses()[:2];nominal=J.angle(FK(qa).T@FK(qb))
        qa[0]+=.19;qb[0]+=.19
        self.assertAlmostEqual(J.angle(FK(qa).T@FK(qb)),nominal,places=10)

    def test_j6_terminal_zero_does_not_change_relative_angle(self):
        qa,qb=self.poses()[:2];nominal=J.angle(FK(qa).T@FK(qb))
        qa[5]-=.17;qb[5]-=.17
        self.assertAlmostEqual(J.angle(FK(qa).T@FK(qb)),nominal,places=10)

    def test_known_internal_offsets_recovered_without_handeye(self):
        truth=np.array([4.,-2.,3.]);x=J.rodrigues([0,1,0],.8);y=J.rodrigues([1,0,0],.3)
        rows=[]
        for i,q in enumerate(self.poses()):
            rows.append(dict(dataset='fixed',epoch=1,urdf_sha256='test',q=q,fk=FK,
                camera=x.T@FK(q,truth).T@y,capture=str(i)))
        pp=J.pairs(rows,list(range(len(rows))));fitted=J.solve(rows,pp)
        np.testing.assert_allclose(fitted.x,truth,atol=1e-5)
        self.assertLess(np.max(abs(fitted.fun)),1e-7)

    def test_different_board_dataset_or_epoch_never_paired(self):
        qa,qb=self.poses()[:2];rows=[]
        for i,q in enumerate((qa,qb)):
            rows.append(dict(dataset='fixed',epoch=i+1,urdf_sha256='test',q=q,fk=FK,camera=FK(q).T))
        self.assertEqual(J.pairs(rows,[0,1]),[])
        rows[1]['epoch']=1;rows[1]['dataset']='moved'
        self.assertEqual(J.pairs(rows,[0,1]),[])

if __name__=='__main__':unittest.main()
