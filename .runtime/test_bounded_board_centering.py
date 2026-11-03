import math
import contextlib,io,threading
import types
import unittest
from unittest.mock import patch
import numpy as np
import bounded_board_centering as b

class ContractTests(unittest.TestCase):
    def setUp(self):
        self.q=np.r_[np.zeros(6),.04975]
        self.ref=self.q.copy();self.ref[:3]=np.array([3,7,6])*b.motor.QUANTUM
        self.c=b.board_contract(self.q,self.ref,100,[-30,-34,-34])
    def test_reordered_onehot_preserves_original_sdk_bias(self):
        self.assertEqual(self.c.indices,[1,2,0]);self.assertEqual(self.c.labels,['center_J2','center_J3','center_J1'])
        previous=self.ref;actual=self.q.copy()
        for count,(j,label,target) in enumerate(zip(self.c.indices,self.c.labels,self.c.targets)):
            self.assertEqual(np.flatnonzero(np.asarray(b.words(target))-b.words(previous)).tolist(),[j])
            self.c.begin(label,target,actual,previous,100,1000+count*1000)
            actual[j]+=target[j]-previous[j]
            self.c.observe_sdk(target,1100+count*1000,100);self.c.complete(actual,100,True)
            previous=target
        np.testing.assert_allclose(previous[:3]-actual[:3],self.ref[:3])
        with self.assertRaisesRegex(RuntimeError,'exhausted'):self.c.begin('center_J2',self.c.targets[0],actual,previous,100,5000)
    def test_zero_axis_omitted_without_reorder_retry(self):
        c=b.board_contract(self.q,self.ref,100,[-30,0,-34])
        self.assertEqual(c.indices,[2,0]);self.assertEqual(len(c.targets),2)
    def test_whole_directional_box_keeps_all_independent_corners(self):
        lo,hi,d=self.c.rectangle(self.q,self.c.targets[0],self.ref,1)
        np.testing.assert_allclose(np.degrees(lo),[-.4,-4.8,-.4,-.4,-.4,-.4])
        np.testing.assert_allclose(np.degrees(hi),[.4,.5,.4,.4,.4,.4])
        self.assertEqual(d.tolist(),[0,-1,0,0,0,0])
        # Nominal SDK bridge remains outside physical held domain, never enlarges it.
        self.assertGreater(self.ref[2],hi[2])
    def test_predecessor_ack_and_stationarity_required(self):
        t=self.c.targets[0];self.c.begin(self.c.labels[0],t,self.q,self.ref,100,1000)
        with self.assertRaisesRegex(RuntimeError,'not newer'):self.c.observe_sdk(t,1000,100)
    def test_new_axis_before_settled_rejected(self):
        self.c.begin(self.c.labels[0],self.c.targets[0],self.q,self.ref,100,1000)
        with self.assertRaisesRegex(RuntimeError,'preceding'):self.c.begin(self.c.labels[1],self.c.targets[1],self.q,self.c.targets[0],100,2000)
    def test_no_unreviewed_axes_or_wire_bounds(self):
        for x in ([0,-35,0],[0,-1,0],[0,2.5,0]):
            with self.assertRaises(RuntimeError):b.board_contract(self.q,self.ref,100,x)
    def test_negative_reverse_and_other_axis_hold_preserved(self):
        for axis,delta,reason in [(1,.51,'box'),(2,.41,'held')]:
            c=b.board_contract(self.q,self.ref,100,[-30,-34,-34])
            c.begin(c.labels[0],c.targets[0],self.q,self.ref,100,1000)
            q=self.q.copy();q[axis]+=math.radians(delta)
            with self.assertRaises(RuntimeError):c.assert_feedback(q,100)
    def test_current_epoch_required_even_with_same_words(self):
        with self.assertRaisesRegex(RuntimeError,'epoch'):self.c.live(101)

class ModelTests(unittest.TestCase):
    def test_fk_archive_current_must_all_match_original(self):
        import hashlib
        sha=hashlib.sha256(b'original').hexdigest()
        with patch.object(b,'ORIGINAL_URDF_SHA256',sha):
            self.assertEqual(b.require_model_identity(sha,sha,'original'),sha)
            for fk,archive,text in [('wrong',sha,'original'),(sha,'wrong','original'),(sha,sha,'candidate')]:
                with self.assertRaises(RuntimeError):b.require_model_identity(fk,archive,text)
    def test_handeye_rotation_and_translation_both_frozen_across_steps(self):
        x=np.eye(4);g=dict(X=x,model=types.SimpleNamespace(fingerprint='camera'))
        frozen,fp=b.require_frozen_observation_model(None,None,g)
        np.testing.assert_array_equal(frozen,x);self.assertIsNot(frozen,x)
        for bad in [np.diag([1,1,-1,1]),np.eye(4)+np.diag([.001,0,0,0])]:
            with self.assertRaises(RuntimeError):b.require_frozen_observation_model(frozen,fp,dict(g,X=bad))
        translated=x.copy();translated[0,3]=.001
        with self.assertRaises(RuntimeError):b.require_frozen_observation_model(frozen,fp,dict(g,X=translated))
    def test_camera_change_rejected(self):
        with self.assertRaises(RuntimeError):b.require_frozen_observation_model(np.eye(4),'old',dict(X=np.eye(4),model=types.SimpleNamespace(fingerprint='new')))

class SolveTests(unittest.TestCase):
    def geometry(self,pixel):
        sdk=types.SimpleNamespace(rs2_project_point_to_pixel=lambda intr,p:pixel)
        return dict(q=np.r_[np.zeros(6),.04975],X=np.eye(4),Y=np.eye(4),model=types.SimpleNamespace(sdk=sdk,intrinsics=None))
    def test_centered_board_is_finite_noop(self):
        g=self.geometry([320.,240.]);g['Y'][0,3]=.2
        delta,report=b.solve_board_center(lambda q:np.eye(4),g)
        np.testing.assert_array_equal(delta,[0,0,0]);self.assertEqual(report['solve_count'],0)
    def test_too_close_source_refused_before_optimizer(self):
        with self.assertRaisesRegex(RuntimeError,'120 mm'):b.solve_board_center(lambda q:np.eye(4),self.geometry([320.,240.]))
    def test_proposal_cannot_claim_success_without_image_improvement(self):
        g=self.geometry([350.,315.]);g['Y'][0,3]=.2
        with self.assertRaisesRegex(RuntimeError,'improvement'):b.solve_board_center(lambda q:np.eye(4),g)

class InterfaceTests(unittest.TestCase):
    def test_default_has_no_publisher_permission_and_no_bootstrap(self):
        a=b.parse_args([]);self.assertFalse(a.execute);self.assertFalse(a.bootstrap_pending)
    def test_only_explicit_execute_flag_allowed(self):
        self.assertTrue(b.parse_args(['--execute']).execute)
        for option in ['--retry','--bootstrap-pending','--old-plan','--joint','--enable']:
            with contextlib.redirect_stderr(io.StringIO()),self.assertRaises(SystemExit):b.parse_args([option])
    def test_inherits_exact_writer_and_publish_implementation(self):
        base=types.SimpleNamespace(Diagnostic=object)
        client=b.make_diagnostic_class(base);motor=b.motor.make_diagnostic_class(base);wrist=b.wrist.make_diagnostic_class(base)
        self.assertEqual(client.write_bag.__code__,motor.write_bag.__code__)
        self.assertEqual(client.close.__code__,motor.close.__code__)
        self.assertEqual(client.execute_step.__code__,wrist.execute_step.__code__)
        self.assertEqual(client.bootstrap_once.__code__,wrist.bootstrap_once.__code__)
        with self.assertRaisesRegex(RuntimeError,'no bootstrap'):client.__new__(client).bootstrap_once()

class RunTests(unittest.TestCase):
    def run_fake(self,execute,increments):
        trace=[];q=np.r_[np.zeros(6),.04975];msg=types.SimpleNamespace(header=types.SimpleNamespace(stamp=types.SimpleNamespace(to_nsec=lambda:1000)))
        class Parent:pass
        base=types.SimpleNamespace(Diagnostic=Parent,positions=lambda msg:q.copy(),write_json=lambda *args:trace.append(('write',args[0].name)),
            rospy=types.SimpleNamespace(Time=types.SimpleNamespace(now=lambda:types.SimpleNamespace(to_nsec=lambda:1000))))
        diagnostic=b.make_diagnostic_class(base).__new__(b.make_diagnostic_class(base))
        diagnostic.args=types.SimpleNamespace(execute=execute,joint=2);diagnostic.out=b.Path('/unused');diagnostic.lock=threading.RLock()
        diagnostic.frozen_epoch=100;diagnostic.accepted=[msg];diagnostic.reference=[msg];diagnostic.fk=lambda q:np.eye(4)
        diagnostic.archived_urdf_sha256='original';diagnostic.frozen_observation_x=np.eye(4);diagnostic.frozen_observation_camera='camera';diagnostic.publish_count=0
        g=dict(q=q.copy(),source_stamp_ns=999,board_observation=dict(radius_px=5.))
        diagnostic.await_initial=lambda:trace.append(('await',))
        diagnostic.stable_hold_recording=lambda duration:trace.append(('hold',duration))
        diagnostic.snapshot=lambda label:trace.append(('snapshot',label)) or g
        diagnostic.guard=lambda:None
        diagnostic.path=lambda *args:trace.append(('path',diagnostic.args.joint)) or dict()
        diagnostic.execute_step=lambda label,target:trace.append(('execute',label,diagnostic.args.joint)) or g
        diagnostic.event=lambda *args,**kw:trace.append(('event',args[0]))
        with patch.object(b,'solve_board_center',return_value=(np.array(increments),dict(solve_count=1))) as solve,patch.object(b.wrist,'validate_source_binding'):
            diagnostic.run();self.assertEqual(solve.call_count,1)
        self.assertEqual(trace[:3],[('await',),('hold',3.),('snapshot','baseline')])
        return trace
    def test_readonly_proves_first_step_and_never_calls_execute(self):
        trace=self.run_fake(False,[-30,-34,-34]);self.assertIn(('path',2),trace)
        self.assertFalse(any(x[0]=='execute' for x in trace));self.assertEqual(trace[-1],('event','prepared_no_motion'))
    def test_execute_finite_sequence_after_recording_with_no_return(self):
        trace=self.run_fake(True,[-30,-34,-34])
        self.assertEqual([x for x in trace if x[0]=='execute'],[('execute','center_J2',2),('execute','center_J3',3),('execute','center_J1',1)])
        self.assertEqual(trace[-1],('event','finite_board_observation_complete'))
    def test_noop_still_records_baseline_but_proves_or_moves_nothing(self):
        trace=self.run_fake(False,[0,0,0]);self.assertFalse(any(x[0] in ('path','execute') for x in trace))

if __name__=='__main__':unittest.main()
