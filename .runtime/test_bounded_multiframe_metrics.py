#!/usr/bin/env python3
import importlib.util
from pathlib import Path
import unittest
import numpy as np
R=Path(__file__).resolve().parents[1]
s=importlib.util.spec_from_file_location('mf',R/'.runtime/analyze_bounded_multiframe.py');M=importlib.util.module_from_spec(s);s.loader.exec_module(M)
s=importlib.util.spec_from_file_location('geo',R/'.runtime/analyze_fixed_handeye_translation.py');G=importlib.util.module_from_spec(s);s.loader.exec_module(G)
class RelativeMetrics(unittest.TestCase):
    def row(self,a,x,b):
        c=G.inv(a@x)@b
        return dict(T_base_tool_fk=a,T_camera_board=c,T_base_board_fk=b,T_base_board_tf=b,accepted_rad=[0]*6,sdk_words=[2048]*6,epoch_ns=1)
    def test_fixed_board_camera_relative_transform_equals_fk(self):
        a=np.eye(4);x=np.eye(4);x[:3,:3]=G.exp_so3([.2,-.1,.3]);x[:3,3]=[.07,.04,.09]
        z=np.eye(4);z[:3,:3]=G.exp_so3([.3,-.2,.1]);z[:3,3]=[.4,.3,.02]
        b=a.copy();b[:3,:3]=G.exp_so3([.04,.02,-.03]);b[:3,3]=[.007,-.004,.008]
        r=M.compare([self.row(a,x,z)],[self.row(b,x,z)])
        self.assertLess(r['camera_minus_fk_translation_norm_mm']['maximum'],1e-9)
        self.assertLess(r['relative_rotation_vector_mismatch_deg']['maximum'],1e-5)
        self.assertLess(r['displacement_base_board_mm']['maximum'],1e-9)
    def test_pure_translation_norm_is_invariant_to_handeye(self):
        a=np.eye(4);b=np.eye(4);b[:3,3]=[.003,.004,.005];z=np.eye(4);z[:3,3]=[0,0,.3]
        for vector in ([.2,.4,.3],[-.7,.3,-.5]):
            x=np.eye(4);x[:3,:3]=G.exp_so3(vector);x[:3,3]=vector
            r=M.compare([self.row(a,x,z)],[self.row(b,x,z)])
            self.assertAlmostEqual(r['relative_camera_translation_norm_mm']['median'],np.sqrt(50),places=8)
    def test_missing_intermediate_is_not_fabricated(self):
        self.assertEqual(M.compare([],[])['pair_count'],0)
if __name__=='__main__':unittest.main()
