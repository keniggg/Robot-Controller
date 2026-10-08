import contextlib,io,math,threading,types
import unittest
from unittest.mock import patch
import numpy as np
import bounded_shoulder_elbow_validation as s

class FixedContractTests(unittest.TestCase):
    def setUp(self):
        self.q=np.array([0.,.365087,-.130388,0.,0.,0.,.04975])
        self.sdk=self.q.copy();self.sdk[1:3]+=6*s.board.motor.QUANTUM
        self.c=s.fixed_contract(self.q,self.sdk,100)
    def test_two_original_sdk_onehot_targets_keep_bias_and_actual_direction(self):
        self.assertEqual(self.c.indices,[1,2]);prior=self.sdk
        q=self.q.copy()
        for n,(axis,target) in enumerate(zip(self.c.indices,self.c.targets)):
            change=np.asarray(s.board.words(target))-s.board.words(prior)
            np.testing.assert_array_equal(change,[0,-22,0,0,0,0] if n==0 else [0,0,22,0,0,0])
            self.assertLess(target[1]-q[1],0) if n==0 else self.assertGreater(target[2]-q[2],0)
            self.c.begin(self.c.labels[n],target,q,prior,100,1000+n*1000)
            q[axis]+=target[axis]-prior[axis]
            self.c.observe_sdk(target,1001+n*1000,100);self.c.complete(q,100,True);prior=target
        np.testing.assert_allclose(prior[1:3]-q[1:3],self.sdk[1:3]-self.q[1:3])
        self.assertEqual(self.c.count,2)
        with self.assertRaisesRegex(RuntimeError,'exhausted'):self.c.begin(self.c.labels[0],self.c.targets[0],q,prior,100,3000)
    def test_second_requires_previous_new_ack_and_stationary(self):
        self.c.begin(self.c.labels[0],self.c.targets[0],self.q,self.sdk,100,1000)
        with self.assertRaisesRegex(RuntimeError,'preceding'):self.c.begin(self.c.labels[1],self.c.targets[1],self.q,self.c.targets[0],100,2000)
    def test_negative_first_full_rectangle_is_unchanged(self):
        lo,hi,d=self.c.rectangle(self.q,self.c.targets[0],self.sdk,1)
        np.testing.assert_allclose(np.degrees(lo-self.q[:6]),[-.4,-4.8,-.4,-.4,-.4,-.4])
        np.testing.assert_allclose(np.degrees(hi-self.q[:6]),[.4,.5,.4,.4,.4,.4])
        self.assertEqual(d[1],-1.)
    def test_sdk_bias_that_reverses_actual_direction_rejected(self):
        ref=self.q.copy();ref[1]+=math.radians(2.3)
        c=s.fixed_contract(self.q,ref,100)
        with self.assertRaisesRegex(RuntimeError,'reverses'):c.rectangle(self.q,c.targets[0],ref,1)
    def test_epoch_and_precommand_ack_still_rejected(self):
        self.c.begin(self.c.labels[0],self.c.targets[0],self.q,self.sdk,100,1000)
        with self.assertRaisesRegex(RuntimeError,'not newer'):self.c.observe_sdk(self.c.targets[0],1000,100)
        c=s.fixed_contract(self.q,self.sdk,100)
        with self.assertRaisesRegex(RuntimeError,'epoch'):c.live(101)

class RunTests(unittest.TestCase):
    def setup_run(self,execute,source_age=1,image_delta=0.,fail_step=None):
        trace=[];q=np.r_[np.zeros(6),.04975];now=1_000_000_000
        msg=types.SimpleNamespace(header=types.SimpleNamespace(stamp=types.SimpleNamespace(to_nsec=lambda:now-1)))
        base=types.SimpleNamespace(Diagnostic=object,positions=lambda msg:q.copy(),write_json=lambda path,value:trace.append(('write',path.name,value.copy())),
            rospy=types.SimpleNamespace(Time=types.SimpleNamespace(now=lambda:types.SimpleNamespace(to_nsec=lambda:now))))
        cls=s.make_diagnostic_class(base);d=cls.__new__(cls)
        d.args=s.parse_args(['--execute'] if execute else []);d.out=s.Path('/unused');d.lock=threading.RLock()
        d.frozen_epoch=100;d.accepted=[msg];d.reference=[msg];d.publish_count=0
        d.archived_urdf_sha256='original';d.frozen_observation_x=np.eye(4);d.frozen_observation_camera='camera'
        g=dict(q=q.copy(),source_stamp_ns=now-source_age,board_observation=dict(radius_px=5.));g['q'][0]+=image_delta
        d.await_initial=lambda:trace.append(('await',))
        d.stable_hold_recording=lambda seconds:trace.append(('hold',seconds))
        d.snapshot=lambda label:trace.append(('snapshot',label)) or g
        d.guard=lambda:None
        d.path=lambda *args:trace.append(('path',d.args.joint)) or dict()
        def step(label,target):
            trace.append(('execute',label,d.args.joint))
            if d.args.joint==fail_step:raise RuntimeError('step source refused')
            return g
        d.execute_step=step;d.event=lambda name,**kw:trace.append(('event',name))
        return d,trace
    def test_default_prepares_only_first_fresh_proof_after_3s_baseline(self):
        d,trace=self.setup_run(False)
        with patch.object(s.board,'solve_board_center',side_effect=AssertionError('no optimizer')):d.run()
        self.assertEqual(trace[:3],[('await',),('hold',3.),('snapshot','baseline')])
        self.assertIn(('path',2),trace);self.assertFalse(any(row[0]=='execute' for row in trace))
        plan=next(row[2] for row in trace if row[:2]==('write','plan.json'))
        self.assertEqual(plan['fixed_increment_words'],[0,-22,22]);self.assertEqual(plan['maximum_commands'],2)
        self.assertFalse(plan['automatic_return']);self.assertFalse(plan['solver_used'])
    def test_execute_exactly_two_stages_and_no_return(self):
        d,trace=self.setup_run(True);d.run()
        self.assertEqual([row for row in trace if row[0]=='execute'],[('execute','validation_J2_negative',2),('execute','validation_J3_positive',3)])
        self.assertEqual(trace[-1],('event','finite_shoulder_elbow_validation_complete'))
    def test_failed_first_stage_does_not_send_second_or_return(self):
        d,trace=self.setup_run(True,fail_step=2)
        with self.assertRaisesRegex(RuntimeError,'source refused'):d.run()
        self.assertEqual([row[2] for row in trace if row[0]=='execute'],[2])
    def test_stale_board_source_blocks_plan_and_motion(self):
        d,trace=self.setup_run(True,source_age=350_000_001)
        with self.assertRaisesRegex(RuntimeError,'board source'):d.run()
        self.assertFalse(any(row[0] in ('path','execute','write') for row in trace))
    def test_image_current_pose_mismatch_blocks_plan_and_motion(self):
        d,trace=self.setup_run(True,image_delta=math.radians(.151))
        with self.assertRaisesRegex(RuntimeError,'settled validation start'):d.run()
        self.assertFalse(any(row[0] in ('path','execute','write') for row in trace))
    def test_source_and_command_freshness_implementation_is_inherited_unchanged(self):
        base=types.SimpleNamespace(Diagnostic=object);cls=s.make_diagnostic_class(base);original=s.board.make_diagnostic_class(base)
        for name in ('snapshot','path','execute_step','guard','write_bag','close'):
            self.assertEqual(getattr(cls,name).__code__,getattr(original,name).__code__)
    def test_cli_has_no_alternate_axis_plan_retry_or_bootstrap(self):
        self.assertFalse(s.parse_args([]).execute);self.assertFalse(s.parse_args([]).bootstrap_pending)
        for flag in ('--joint','--direction','--old-plan','--retry','--bootstrap-pending'):
            with contextlib.redirect_stderr(io.StringIO()),self.assertRaises(SystemExit):s.parse_args([flag])

if __name__=='__main__':unittest.main()
