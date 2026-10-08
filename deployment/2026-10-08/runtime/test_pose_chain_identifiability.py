#!/usr/bin/env python3
"""Synthetic invariants and actual-URDF checks for offline diagnostic tools."""
import copy
import importlib.util
from pathlib import Path
import unittest
import numpy as np

ROOT=Path(__file__).resolve().parents[1]

def module(name):
    spec=importlib.util.spec_from_file_location(name,ROOT/'.runtime'/(''+name+'.py'))
    mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod);return mod

A=module('analyze_fixed_handeye_translation')
C=module('coupled_joint23_geometry')
XML=(ROOT/'src/real-arm/alicia_d_descriptions/urdf/alicia_duo_with_gripper.urdf').read_text()
Q=np.array([-1.93588,.52309,-.13192,-.05676,-.47707,.04909])

class Invariants(unittest.TestCase):
    def test_translation_recovers_nonparallel_true_delta(self):
        truth=np.array([.011,-.008,.019]); contrasts=[]
        for axis in np.eye(3):
            matrix=A.exp_so3(axis*.1)-np.eye(3)
            contrasts.append(dict(matrix=matrix,delta=-matrix@truth))
        fitted=A.fit(contrasts)
        self.assertEqual(fitted['rank'],3)
        np.testing.assert_allclose(fitted['correction'],truth,atol=1e-13)

    def test_parallel_rotations_are_rank_two(self):
        contrasts=[dict(matrix=A.exp_so3([0,0,x])-np.eye(3),delta=np.zeros(3)) for x in (.05,.1)]
        self.assertEqual(A.fit(contrasts)['rank'],2)

    def test_screw_axis_and_pitch(self):
        axis=np.array([.2,.3,.9]);axis/=np.linalg.norm(axis)
        point=np.array([.1,-.2,.3]);point-=axis*(axis@point)
        angle=.05;t=np.eye(4);t[:3,:3]=A.exp_so3(axis*angle)
        t[:3,3]=(np.eye(3)-t[:3,:3])@point+axis*.0002
        got=A.screw(t)
        np.testing.assert_allclose(got['axis'],axis,atol=1e-12)
        np.testing.assert_allclose(got['point'],point,atol=1e-11)
        self.assertAlmostEqual(got['axial_translation_m'],.0002)

    def test_parallel_axis_distance_rigid_frame_invariant(self):
        transforms=[]
        h=np.eye(4);h[:3,:3]=A.exp_so3([.3,-.2,.1]);h[:3,3]=[.5,-.1,.2]
        for point in (np.array([0.,0.,0.]),np.array([.2248,0.,0.])):
            t=np.eye(4);t[:3,:3]=A.exp_so3([0.,0.,.06]);t[:3,3]=(np.eye(3)-t[:3,:3])@point
            transforms.append(h@t@A.inv(h))
        got=A.pair_parallel_distance(*transforms)
        self.assertAlmostEqual(got['distance_m'],.2248,places=11)

    def test_real_urdf_counterrotation_exactly_cancels_orientation(self):
        first,_=A.serial_fk_and_axes(XML,Q)
        for sign in (-1,1):
            after=Q.copy();after[1]+=sign*np.deg2rad(4);after[2]-=sign*np.deg2rad(4)
            second,_=A.serial_fk_and_axes(XML,after)
            self.assertLess(np.linalg.norm(A.log_so3(first[:3,:3].T@second[:3,:3])),1e-12)
            self.assertAlmostEqual(np.linalg.norm(second[:3,3]-first[:3,3]),.01569137684425204,places=10)

    def test_camera_relative_norm_independent_of_arbitrary_handeye(self):
        fk=lambda q:A.serial_fk_and_axes(XML,q)[0]
        after=Q.copy();after[1]+=.06;after[2]-=.06
        board=np.eye(4);board[:3,:3]=A.exp_so3([.2,.1,-.3]);board[:3,3]=[.2,.5,.05]
        for lever in ([.02,.1,.03],[.2,-.15,.3]):
            x=np.eye(4);x[:3,:3]=A.exp_so3([.5,.1,-.3]);x[:3,3]=lever
            cbefore=A.inv(fk(Q)@x)@board;cafter=A.inv(fk(after)@x)@board
            report=C.evaluate_measured_pair(Q,after,cbefore,cafter,fk,x)
            self.assertAlmostEqual(report['invariant_length_difference_mm'],0.,places=8)
            np.testing.assert_allclose(report['AX_minus_XB_translation_mm'],0.,atol=1e-9)

    def test_uncancelled_actual_angle_is_not_hidden_by_command_intent(self):
        fk=lambda q:A.serial_fk_and_axes(XML,q)[0]
        after=Q.copy();after[1]+=.06;after[2]-=.053
        x=np.eye(4);x[:3,3]=[.1,-.1,.02]
        ca=A.inv(fk(Q)@x);cb=A.inv(fk(after)@x)
        report=C.evaluate_measured_pair(Q,after,ca,cb,fk,x)
        self.assertFalse(report['actual_fk_rotation_within_one_word'])
        self.assertGreater(report['maximum_additional_fixed_translation_leak_mm'],.6)
        self.assertAlmostEqual(report['actual_joint23_sum_delta_deg'],np.rad2deg(.007),places=8)

    def test_opposite_integer_targets_and_unchanged_held_words(self):
        q=np.r_[Q,.04975]
        for degree in (2.,3.,4.):
            targets,count=C.coupled_targets(q,degree)
            for sign,target in zip((1,-1),targets):
                np.testing.assert_array_equal(np.array(C.words(target))-C.words(q),[0,sign*count,-sign*count,0,0,0])
                self.assertEqual(target[6],q[6])

    def test_old_future_mixed_epoch_and_bad_camera_binding_rejected(self):
        good=dict(now_ns=2_000_000_000,source_stamp_ns=1_900_000_000,accepted_stamp_ns=1_950_000_000,
            sdk_stamp_ns=1_950_000_000,epoch_ns=1_000_000_000,accepted_epoch_ns=1_000_000_000,
            sdk_epoch_ns=1_000_000_000,camera_fingerprint='abc',handeye_sha256='a'*64)
        C.validate_binding(good)
        for field,value in [('source_stamp_ns',1_600_000_000),('sdk_stamp_ns',2_000_000_001),
                            ('accepted_epoch_ns',999),('camera_fingerprint','')]:
            bad=dict(good);bad[field]=value
            with self.subTest(field=field),self.assertRaises(ValueError):C.validate_binding(bad)

    def test_response_rectangle_contains_async_off_diagonal_states(self):
        binding=dict(now_ns=2_000_000_000,source_stamp_ns=1_950_000_000,accepted_stamp_ns=1_950_000_000,
            sdk_stamp_ns=1_950_000_000,epoch_ns=1_000_000_000,accepted_epoch_ns=1_000_000_000,
            sdk_epoch_ns=1_000_000_000,camera_fingerprint='abc',handeye_sha256='a'*64)
        def fk(q):
            t=np.eye(4);t[:3,3]=[0.,q[1],q[2]];return t
        def corners(tool,opening):
            # Deliberate risk only away from opposed-angle diagonal.
            z=.02 if tool[1,3]>.04 and tool[2,3]>.04 else .10
            return np.array([[0,0,z]])
        # Camera_link forward x; board normal z and board points in view.
        y=np.eye(4);y[:3,3]=[.3,0,0]
        report=C.evaluate_counterrotation_candidates(np.r_[np.zeros(6),.04],np.r_[np.zeros(6),.04],
            np.eye(4),y,binding,fk,corners,lambda p:np.column_stack((320+p[:,1]*100,240+p[:,2]*100)),
            (640,480),np.full(6,-3.),np.full(6,3.),requested_degrees=(2.,))
        self.assertFalse(report['envelope_pass'])
        self.assertTrue(any(c['q_rad'][1]>.04 and c['q_rad'][2]>.04 and not c['geometric_sample_pass'] for c in report['samples']))
        self.assertEqual(report['candidates'][0]['fixed_four_leg_order'],['positive','original_sdk','negative','original_sdk'])
        self.assertFalse(report['execution_authorized'])

if __name__=='__main__':unittest.main()
