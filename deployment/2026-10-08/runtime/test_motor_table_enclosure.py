import unittest
import numpy as np
from bounded_motor_centering import table_clearance_from_cad_enclosure as evaluate

class TableEnclosureTests(unittest.TestCase):
    def test_exact_required_clearance_passes(self):
        r=evaluate([[0,0,.04]],[0],[0,0,1,0],0.)
        self.assertTrue(r['passes_40mm'])
    def test_lower_clearance_rejects(self):
        self.assertFalse(evaluate([[0,0,.039]],[0],[0,0,1,0],0.)['passes_40mm'])
    def test_radius_and_visible_motor_top_both_debited(self):
        r=evaluate([[0,0,.075]],[.02],[0,0,1,0],.02)
        self.assertAlmostEqual(r['minimum_adjacent_table_plus_visible_motor_clearance_m'],.035)
        self.assertFalse(r['passes_40mm'])
    def test_all_cells_and_corners_participate(self):
        r=evaluate([[0,0,.1],[1,2,.07],[2,3,.04]],[0,.01,.005],[0,0,1,0],.001)
        self.assertEqual(r['evaluated_cad_corners'],3)
        self.assertEqual(r['limiting_corner_base_m'],[2.,3.,.04])
        self.assertFalse(r['passes_40mm'])
    def test_positive_plane_scale_invariant(self):
        a=evaluate([[1,2,.07]],[.004],[0,0,1,-.01],.01)
        b=evaluate([[1,2,.07]],[.004],[0,0,3,-.03],.01)
        self.assertAlmostEqual(a['minimum_adjacent_table_plus_visible_motor_clearance_m'],b['minimum_adjacent_table_plus_visible_motor_clearance_m'])
    def test_base_origin_translation_does_not_inflate_new_plane(self):
        corners=np.array([[.1,.2,.08],[-.1,.21,.081]])
        normal=np.array([.01,.02,1.]);normal/=np.linalg.norm(normal)
        plane=np.r_[normal,-.004];shift=np.array([100.,-250.,30.])
        a=evaluate(corners,[.006,.007],plane,.015)
        b=evaluate(corners+shift,[.006,.007],np.r_[normal,plane[3]-normal@shift],.015)
        self.assertAlmostEqual(a['minimum_adjacent_table_plus_visible_motor_clearance_m'],b['minimum_adjacent_table_plus_visible_motor_clearance_m'],places=12)
    def test_exact_fresh_plane_can_be_better_than_old(self):
        old=evaluate([[1,2,.09]],[.004],[0,0,1,0],.01)
        n=np.array([.01,.005,1.]);n/=np.linalg.norm(n)
        fresh=evaluate([[1,2,.09]],[.004],np.r_[n,-n@np.array([1,2,0])],.01)
        self.assertTrue(fresh['passes_40mm'])
        self.assertLess(abs(old['minimum_adjacent_table_plus_visible_motor_clearance_m']-fresh['minimum_adjacent_table_plus_visible_motor_clearance_m']),.0001)
    def test_invalid_inputs_reject(self):
        for p,r,n,h in [([],[],[0,0,1,0],0),([[0,0,.1]],[-1],[0,0,1,0],0),
                         ([[0,0,.1]],[0],[0,0,0,0],0),([[0,0,.1]],[0],[0,0,-1,0],0),
                         ([[0,0,np.nan]],[0],[0,0,1,0],0),([[0,0,.1]],[0],[0,0,1,0],.041)]:
            with self.subTest(p=p,r=r,n=n,h=h):
                with self.assertRaises(RuntimeError):evaluate(p,r,n,h)
if __name__=='__main__':unittest.main()
