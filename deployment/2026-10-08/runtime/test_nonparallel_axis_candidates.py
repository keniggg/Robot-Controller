#!/usr/bin/env python3
import ast
import importlib.util
from pathlib import Path
import unittest
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('wrist_readonly',ROOT/'.runtime/prepare_nonparallel_axis_candidates.py')
W=importlib.util.module_from_spec(spec);spec.loader.exec_module(W)
XML=(ROOT/'src/real-arm/alicia_d_descriptions/urdf/alicia_duo_with_gripper.urdf').read_text()
Q=np.array([-1.93435,.51849,-.11351,-.05829,-.48014,.04909,.04975])

class ReadonlyCandidates(unittest.TestCase):
    def test_targets_edit_only_selected_axis_and_keep_original_untouched(self):
        source=Q.copy()
        for joint in (4,6):
            rows=W.candidate_targets(source,joint)
            self.assertEqual(len(rows),2)
            for sign,target in rows:
                delta=np.array(W.BASE.wire_words(target))-W.BASE.wire_words(source)
                expected=np.zeros(6,dtype=int);expected[joint-1]=sign*22
                np.testing.assert_array_equal(delta,expected)
                self.assertEqual(target[6],source[6])
            np.testing.assert_array_equal(source,Q)

    def test_other_joint_or_larger_step_rejected(self):
        for joint,step in ((2,2.),(5,2.),(4,3.),(6,0.)):
            with self.subTest(joint=joint,step=step),self.assertRaises(ValueError):W.candidate_targets(Q,joint,step)

    def test_nonparallel_axes_restore_translation_rank(self):
        base,_=W.MATH.serial_fk_and_axes(XML,Q[:6]);matrices={}
        for joint in (2,3,4,6):
            target=Q[:6].copy();target[joint-1]+=22*W.BASE.QUANTUM
            matrices[joint]=W.MATH.serial_fk_and_axes(XML,target)[0][:3,:3]-base[:3,:3]
        self.assertEqual(np.linalg.matrix_rank(np.vstack([matrices[2],matrices[3]]),tol=1e-10),2)
        for joint in (4,6):
            singular=np.linalg.svd(np.vstack([matrices[2],matrices[joint]]),compute_uv=False)
            self.assertGreater(singular[-1],.03)
            self.assertLess(singular[0]/singular[-1],1.5)

    def test_no_command_calls_and_literal_readonly_constructor(self):
        tree=ast.parse((ROOT/'.runtime/prepare_nonparallel_axis_candidates.py').read_text())
        calls=[n for n in ast.walk(tree) if isinstance(n,ast.Call)]
        for call in calls:
            if isinstance(call.func,ast.Attribute):self.assertNotIn(call.func.attr,['publish','execute_step','bootstrap_once','set_param'])
        namespaces=[c for c in calls if isinstance(c.func,ast.Name) and c.func.id=='SimpleNamespace']
        self.assertEqual(len(namespaces),1)
        kwargs={kw.arg:kw.value for kw in namespaces[0].keywords}
        self.assertIsInstance(kwargs['execute'],ast.Constant)
        self.assertIs(kwargs['execute'].value,False)
        self.assertIs(kwargs['bootstrap_pending'].value,False)

if __name__=='__main__':unittest.main()
