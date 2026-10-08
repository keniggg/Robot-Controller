import contextlib,io,math,threading,types
import unittest
from unittest.mock import patch
import numpy as np
import bounded_j3_small_response as s


def qfrom(words,j4=0.):
    q=np.r_[(np.array(words)-2048)*s.board.motor.QUANTUM,.04975];q[3]=math.radians(j4)
    return q

def fixtures():
    sdk=qfrom(s.INITIAL_SDK_WORDS);actual=sdk.copy();actual[2]-=10*s.board.motor.QUANTUM
    return actual,sdk

class PinTests(unittest.TestCase):
    def test_single_8_words_reports_real_18_word_goal_difference(self):
        q,sdk=fixtures();c=s.SmallResponseContract(q,sdk,s.PINNED_EPOCH)
        self.assertEqual(c.indices,[2]);self.assertEqual(len(c.targets),1)
        target=c.targets[0];self.assertEqual(s.board.words(target),[853,2270,1999,2048,1912,2045])
        self.assertAlmostEqual(math.degrees(target[2]-q[2]),1.58203125)
        c.begin(c.labels[0],target,q,sdk,s.PINNED_EPOCH,s.PINNED_EPOCH+1000)
        q[2]+=8*s.board.motor.QUANTUM;c.observe_sdk(target,s.PINNED_EPOCH+1001,s.PINNED_EPOCH);c.complete(q,s.PINNED_EPOCH,True)
        with self.assertRaisesRegex(RuntimeError,'exhausted'):c.begin(c.labels[0],target,q,target,s.PINNED_EPOCH,s.PINNED_EPOCH+2000)
    def test_epoch_and_all_six_initial_sdk_words_are_pinned(self):
        q,sdk=fixtures()
        with self.assertRaisesRegex(RuntimeError,'epoch'):s.SmallResponseContract(q,sdk,s.PINNED_EPOCH+1)
        for axis in range(6):
            changed=sdk.copy();changed[axis]+=s.board.motor.QUANTUM
            with self.assertRaisesRegex(RuntimeError,'SDK differs'):s.SmallResponseContract(q,changed,s.PINNED_EPOCH)
    def test_j4_old_anchor_cannot_rebase_to_new_origin(self):
        q,sdk=fixtures();q[3]=math.radians(-.17578125);c=s.SmallResponseContract(q,sdk,s.PINNED_EPOCH)
        # -0.35 differs from this new origin by only .1742, yet old anchor
        # difference is .4379: the old failed-trial limit must still reject.
        moved=q.copy();moved[3]=math.radians(-.35)
        with self.assertRaisesRegex(RuntimeError,'permanent original'):c.assert_feedback(moved,s.PINNED_EPOCH)
        with self.assertRaises(RuntimeError):c.assert_feedback(q,s.PINNED_EPOCH)
    def test_current_zero_and_old_boundary_are_allowed_but_not_expanded(self):
        q,_=fixtures();s.require_original_j4_anchor(q)
        q[3]=s.OLD_J4_ANCHOR_RAD-s.OLD_J4_BOUND_RAD;s.require_original_j4_anchor(q)
        q[3]-=1e-8
        with self.assertRaises(RuntimeError):s.require_original_j4_anchor(q)
    def test_geometry_rectangle_is_unchanged_full_new_domain(self):
        q,sdk=fixtures();c=s.SmallResponseContract(q,sdk,s.PINNED_EPOCH)
        lo,hi,_=c.rectangle(q,c.targets[0],sdk,2)
        np.testing.assert_allclose(np.degrees(lo-q[:6]),[-.4,-.4,-.5,-.4,-.4,-.4])
        np.testing.assert_allclose(np.degrees(hi-q[:6]),[.4,.4,4.8,.4,.4,.4])
        self.assertLess(lo[3],s.OLD_J4_ANCHOR_RAD-s.OLD_J4_BOUND_RAD)
    def test_prior_failure_is_immutable_and_not_completed(self):
        r=s.prior_failure_record();self.assertFalse(r['original_failure_cleared']);self.assertFalse(r['original_failed_command_completed'])
        self.assertEqual(r['terminal']['status'],'failed')
        with patch.object(s,'PRIOR_SHA256',dict(s.PRIOR_SHA256,**{'terminal.json':'wrong'})):
            with self.assertRaisesRegex(RuntimeError,'artifact changed'):s.prior_failure_record()

class CallbackTests(unittest.TestCase):
    def callback(self,topic,value,contract=None):
        archived=[]
        class Collector:
            def receive(self,t,m):archived.append((t,m))
        base=types.SimpleNamespace(positions=lambda m:m.q)
        with patch.object(s.board,'make_diagnostic_class',return_value=Collector):cls=s.make_diagnostic_class(base)
        d=cls.__new__(cls);d.lock=threading.RLock();d.pair_changed=threading.Condition(d.lock);d.closed=False;d.fault=None;d.contract=contract
        msg=types.SimpleNamespace(q=value,stamp=types.SimpleNamespace(to_nsec=lambda:value))
        d.receive(topic,msg);return d,archived
    def test_transient_old_anchor_violation_latches_before_archiving(self):
        q,_=fixtures();q[3]=math.radians(-.35)
        d,rows=self.callback('/alicia_d/accepted_joint_states',q)
        self.assertIn('permanent original',d.fault);self.assertEqual(len(rows),1)
        q[3]=0.;d.receive('/alicia_d/accepted_joint_states',types.SimpleNamespace(q=q))
        self.assertIn('permanent original',d.fault)
    def test_prepared_sdk_external_change_is_latched_not_followed(self):
        _,sdk=fixtures();sdk[0]+=s.board.motor.QUANTUM
        d,rows=self.callback('/alicia_d/control_reference',sdk);self.assertIn('SDK differs',d.fault);self.assertEqual(len(rows),1)
    def test_epoch_change_latched_and_raw_still_archived(self):
        d,rows=self.callback('/alicia_d/control_reference_epoch',s.PINNED_EPOCH+1);self.assertIn('epoch',d.fault);self.assertEqual(len(rows),1)
    def test_ack_phase_does_not_mistake_allowed_new_target_for_external_change(self):
        q,sdk=fixtures();c=s.SmallResponseContract(q,sdk,s.PINNED_EPOCH);c.begin(c.labels[0],c.targets[0],q,sdk,s.PINNED_EPOCH,s.PINNED_EPOCH+1)
        d,rows=self.callback('/alicia_d/control_reference',c.targets[0],c);self.assertIsNone(d.fault)

class RunTests(unittest.TestCase):
    def fake(self,execute=False,stale=False):
        trace=[];q,sdk=fixtures();now=s.PINNED_EPOCH+10**9
        am=types.SimpleNamespace(q=q,header=types.SimpleNamespace(stamp=types.SimpleNamespace(to_nsec=lambda:now-1)))
        rm=types.SimpleNamespace(q=sdk,header=am.header)
        base=types.SimpleNamespace(Diagnostic=object,positions=lambda m:m.q.copy(),write_json=lambda p,v:trace.append(('write',v)),rospy=types.SimpleNamespace(Time=types.SimpleNamespace(now=lambda:types.SimpleNamespace(to_nsec=lambda:now))))
        cls=s.make_diagnostic_class(base);d=cls.__new__(cls);d.lock=threading.RLock();d.args=s.parse_args(['--execute'] if execute else []);d.out=s.Path('/unused');d.publish_count=0
        d.frozen_epoch=s.PINNED_EPOCH;d.accepted=[am];d.reference=[rm];d.archived_urdf_sha256='original';d.frozen_observation_x=np.eye(4);d.frozen_observation_camera='camera';d.prior_failure={'original_failure_cleared':False}
        g=dict(q=q.copy(),source_stamp_ns=now-(350000001 if stale else 1),board_observation={})
        d.await_initial=lambda:trace.append(('await',));d.stable_hold_recording=lambda t:trace.append(('hold',t));d.snapshot=lambda label:trace.append(('snapshot',label)) or g;d.guard=lambda:None
        d.path=lambda *args:trace.append(('path',)) or {};d.execute_step=lambda *args:trace.append(('execute',)) or g;d.event=lambda name,**kw:trace.append(('event',name))
        return d,trace
    def test_default_no_motion_and_3_second_baseline(self):
        d,t=self.fake();d.run();self.assertEqual(t[:3],[('await',),('hold',3.),('snapshot','baseline')]);self.assertNotIn(('execute',),t)
        p=next(x[1] for x in t if x[0]=='write');self.assertEqual(p['maximum_commands'],1);self.assertAlmostEqual(p['nominal_J3_target_minus_actual_deg'],1.58203125)
        self.assertFalse(p['old_failure_cleared']);self.assertFalse(p['automatic_retry'])
    def test_execute_calls_inherited_step_once_and_no_return(self):
        d,t=self.fake(True);d.run();self.assertEqual(t.count(('execute',)),1)
    def test_expired_source_publishes_and_prepares_nothing(self):
        d,t=self.fake(True,True)
        with self.assertRaisesRegex(RuntimeError,'board source'):d.run()
        self.assertFalse(any(x[0] in ('execute','write','path') for x in t))
    def test_geometry_and_execution_inherited_unchanged(self):
        base=types.SimpleNamespace(Diagnostic=object);cls=s.make_diagnostic_class(base);parent=s.board.make_diagnostic_class(base)
        for name in ('path','execute_step','snapshot','write_bag','close'):self.assertEqual(getattr(cls,name).__code__,getattr(parent,name).__code__)
    def test_no_cli_override_for_pins_or_anchor(self):
        self.assertFalse(s.parse_args([]).execute)
        for flag in ('--joint','--count','--epoch','--j4-anchor','--sdk','--old-plan','--retry'):
            with contextlib.redirect_stderr(io.StringIO()),self.assertRaises(SystemExit):s.parse_args([flag])

if __name__=='__main__':unittest.main()
