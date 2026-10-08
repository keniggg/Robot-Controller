import math
import unittest
import numpy as np
import bounded_motor_centering as m

class ContractTests(unittest.TestCase):
    def setUp(self):
        self.q=np.zeros(7);self.q[6]=.04975
        self.sdk=self.q.copy();self.sdk[2]=m.QUANTUM*7
        self.c=m.CenteringContract(self.q,self.sdk,100,[25,29,34])
    def begin(self,i=0,q=None):
        q=self.q if q is None else q
        sdk=self.sdk if i==0 else self.c.targets[i-1]
        self.c.begin(self.c.labels[i],self.c.targets[i],q,sdk,100,1000+i*1000)
    def finish(self,i=0,q=None):
        q=self.q.copy() if q is None else q.copy()
        j=self.c.indices[i];q[j]+=float(self.c.targets[i][j]-(self.sdk if i==0 else self.c.targets[i-1])[j])
        self.c.observe_sdk(self.c.targets[i],1100+i*1000,100)
        self.c.complete(q,100,True)
        return q
    def test_three_onehot_preserve_sdk_bias(self):
        q=self.q.copy()
        for i in range(3):
            self.begin(i,q);q=self.finish(i,q)
        self.assertEqual(self.c.count,3)
        for i,t in enumerate(self.c.targets):
            prior=self.sdk if i==0 else self.c.targets[i-1]
            self.assertEqual(np.flatnonzero(np.asarray(m.words(t))-m.words(prior)).tolist(),[i])
        self.assertAlmostEqual(self.c.targets[-1][2]-q[2],self.sdk[2])
        with self.assertRaises(RuntimeError):self.begin(0,q)
    def test_next_axis_requires_complete(self):
        self.begin()
        with self.assertRaisesRegex(RuntimeError,'preceding'):self.begin(1)
    def test_sdk_wrong_held_axis(self):
        self.begin();bad=self.c.targets[0].copy();bad[2]+=m.QUANTUM
        with self.assertRaises(RuntimeError):self.c.observe_sdk(bad,1100,100)
    def test_sdk_precommand_not_ack(self):
        self.begin()
        with self.assertRaises(RuntimeError):self.c.observe_sdk(self.c.targets[0],1000,100)
    def test_reverse_half_degree(self):
        self.begin();q=self.q.copy();q[0]-=math.radians(.51)
        with self.assertRaises(RuntimeError):self.c.assert_feedback(q,100)
    def test_held_start_drift_cannot_expand_bound(self):
        start=self.q.copy();start[1]=math.radians(.1);self.begin(q=start)
        self.c.active['lower']=self.q[:6]-math.radians(.8)
        self.c.active['upper']=self.q[:6]+math.radians(.8)
        moved=start.copy();moved[1]+=math.radians(.41)
        with self.assertRaisesRegex(RuntimeError,'independent per-step'):self.c.assert_feedback(moved,100)
    def test_epoch_fault_latches(self):
        with self.assertRaises(RuntimeError):self.c.live(101)
        with self.assertRaises(RuntimeError):self.c.live(100)
    def test_no_feedback_response(self):
        self.begin();self.c.observe_sdk(self.c.targets[0],1100,100)
        with self.assertRaises(RuntimeError):self.c.complete(self.q,100,True)
    def test_not_stationary(self):
        self.begin();self.c.observe_sdk(self.c.targets[0],1100,100)
        with self.assertRaises(RuntimeError):self.c.complete(self.c.targets[0],100,False)
    def test_frozen_gripper(self):
        self.begin();q=self.q.copy();q[6]=.04
        with self.assertRaises(RuntimeError):self.c.assert_feedback(q,100)
    def test_nominal_bias_bridge_separate(self):
        self.assertGreater(abs(self.sdk[2]-self.q[2]),m.HELD)
        lo,hi,_=self.c.rectangle(self.q,self.c.targets[0],self.sdk,0)
        self.assertLess(hi[2],self.sdk[2])
    def test_outside_bound_and_reorder(self):
        with self.assertRaises(RuntimeError):m.CenteringContract(self.q,self.sdk,100,[35,0,0])
        with self.assertRaises(RuntimeError):self.begin(1)
    def test_quantization_never_expands_three_deg(self):
        x=m.quantized_deltas([m.LIMIT,-m.LIMIT,.001])
        np.testing.assert_array_equal(x,[34,-34,0])
    def test_non_integer_word_rejected(self):
        with self.assertRaises(RuntimeError):m.CenteringContract(self.q,self.sdk,100,[2.5,3,4])
    def test_no_axes(self):
        c=m.CenteringContract(self.q,self.sdk,100,[0,0,0]);self.assertEqual(c.targets,[])
    def test_full_axis_cells(self):
        lower=np.zeros(6);upper=np.r_[math.radians(.9),0.,0.,0.,0.,0.]
        cells=list(m.joint_cells(lower,upper));self.assertEqual(len(cells),3)
        np.testing.assert_allclose(cells[0][0]-cells[0][1],lower)
        np.testing.assert_allclose(cells[-1][0]+cells[-1][1],upper)
        self.assertTrue(all(np.all(h[1:]==0) for _,h in cells))

class CandidateTests(unittest.TestCase):
    def data(self,two=False):
        yy,xx=np.mgrid[10:15,10:15];uv=np.c_[xx.ravel()*2,yy.ravel()*2]
        if two:uv=np.r_[uv,uv+[30,0]]
        pb=np.c_[uv[:,0]*.001,uv[:,1]*.001,np.zeros(len(uv))]
        selected=np.ones(len(uv),bool)
        return uv,pb,selected
    def test_unique_component(self):
        uv,pb,k=self.data();r=m.unique_component(uv,pb,k,np.median(pb,axis=0),[.008,.008,.01])
        self.assertFalse(r['production_target_identity_verified'])
    def test_two_candidates_rejected(self):
        uv,pb,k=self.data(True)
        with self.assertRaises(RuntimeError):m.unique_component(uv,pb,k,np.median(pb,axis=0),[.02,.02,.01])
    def test_relocated_candidate_rejected(self):
        uv,pb,k=self.data()
        with self.assertRaises(RuntimeError):m.unique_component(uv,pb,k,[0,0,0],[.008,.008,.01])

if __name__=='__main__':unittest.main()
