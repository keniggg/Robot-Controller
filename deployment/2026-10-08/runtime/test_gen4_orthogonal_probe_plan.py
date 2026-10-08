import math
import pathlib
import sys
import unittest
from types import SimpleNamespace
import numpy as np
sys.path[:0]=[str(pathlib.Path(__file__).parent),str(pathlib.Path(__file__).parent/'gen4_root_cause')]
import gen4_orthogonal_probe_plan as m
from test_gen4_high_posture_plan import FakeGeometry
import collect_high_posture as collector


class OrthogonalTests(unittest.TestCase):
 def setUp(self):
  self.q=np.array([-1.88,.894,-.145,-.106,-.764,.04,.04975]);self.sdk=self.q.copy();self.sdk[4]+=.014
  self.s=dict(source_stamp_ns=1,q_rad=self.q.tolist(),sdk_q_rad=self.sdk.tolist(),epoch_ns=11)
  self.p=m.OrthogonalProbeAudit(self.s,'xml',lambda q,g:True,audit_factory=FakeGeometry);self.p.check_model=lambda:None
 def test_signed_repeats_restore_every_held_word(self):
  steps=m.build_sequence(self.sdk);self.assertEqual(len(steps),16)
  for i in range(0,16,2):
   out,ret=steps[i:i+2];self.assertEqual(ret['target_sdk'],self.sdk.tolist());d=np.array(out['target_words'])-m.base.words(self.sdk)
   self.assertEqual(np.count_nonzero(d),1);self.assertEqual(abs(d[out['axis']]),34)
  for axis in (1,4):
   delta=[s['target_words'][axis]-m.base.words(self.sdk)[axis] for s in steps if s['axis']==axis and s['phase']=='probe']
   self.assertEqual(delta,[-34,34,34,-34])
 def test_full_geometry_then_actual_submission(self):
  proof=self.p.audit_nominal_sequence(False);self.assertEqual(len(proof['proof_steps']),16);self.assertTrue(proof['all_pass'])
  self.p.begin_step(0,self.q,self.sdk);self.assertTrue(self.p.guard_segment(self.q,self.p.sequence[0]['target_sdk'])['all_pass'])
 def test_bounds_do_not_inherit_high_staircase(self):
  self.assertLess(np.degrees(self.p.absolute_hi[1]-self.q[1]),4)
  self.assertLess(np.degrees(self.q[1]-self.p.absolute_lo[1]),4)
  bad=self.q.copy();bad[1]+=math.radians(10)
  with self.assertRaisesRegex(RuntimeError,'whole-sequence'):self.p.check_actual(bad)
 def test_negative_step_reserved_noncompounding(self):
  self.p.audit_nominal_sequence(False);self.p.begin_step(0,self.q,self.sdk);goal=self.p.sequence[0]['target_sdk'];first=self.p.guard_segment(self.q,goal)
  q=self.q.copy();q[1]=goal[1]-math.radians(.4);second=self.p.guard_segment(q,goal)
  self.assertEqual(first['fixed_step_response_bounds_deg'],second['fixed_step_response_bounds_deg'])
 def test_unedited_word_change_rejected(self):
  self.p.audit_nominal_sequence(False);self.p.begin_step(0,self.q,self.sdk);bad=np.array(self.p.sequence[0]['target_sdk']);bad[3]+=m.base.QUANTUM
  with self.assertRaisesRegex(RuntimeError,'frozen'):self.p.guard_segment(self.q,bad)


class ReauditTests(unittest.TestCase):
 def run_audit(self,qs,sdks=None,fail=None):
  sdk=np.zeros(7);wire=sdk.copy();wire[1]=34*m.base.QUANTUM;answers=iter(qs);records=[];audited=[]
  sequence=[dict(index=0,before_words=m.base.words(sdk))]
  def source_guard(_):
   q=np.asarray(next(answers),float);return q,(next(sdks) if sdks is not None else sdk.copy()),1
  def audit(q,w):
   audited.append(q.copy())
   if fail:raise RuntimeError(fail)
   return {'q':q.tolist(),'all_pass':True}
  plan=SimpleNamespace(active={'index':0},sequence=sequence,reference=sdk,guard_segment=audit)
  recorder=SimpleNamespace(source_guard=source_guard,mark=lambda e,**kw:records.append((e,kw)))
  return collector.audit_current_submission(recorder,SimpleNamespace(guard=lambda:None),plan,wire),records,audited
 def test_one_quantum_change_discards_and_fully_reaudits(self):
  q=np.zeros(7);new=q.copy();new[4]=m.base.QUANTUM
  proof,records,audited=self.run_audit([q,new,new,new])
  self.assertEqual([e for e,_ in records],['submission_audit_discarded','actual_submission_geometry']);self.assertEqual(len(audited),2);np.testing.assert_array_equal(proof['q'],new)
 def test_continuing_motion_is_not_accepted(self):
  qs=[np.full(7,i*m.base.QUANTUM) for i in range(6)]
  with self.assertRaisesRegex(RuntimeError,'no publication'):self.run_audit(qs)
 def test_changed_sdk_is_not_a_retriable_quantum(self):
  q=np.zeros(7);sdk=q.copy();bad=sdk.copy();bad[2]=m.base.QUANTUM
  with self.assertRaisesRegex(RuntimeError,'SDK changed'):self.run_audit([q,q],iter([sdk,bad]))
 def test_failed_geometry_cannot_retry_itself_into_authority(self):
  with self.assertRaisesRegex(RuntimeError,'scene invalid'):self.run_audit([np.zeros(7)],fail='scene invalid')


if __name__=='__main__':unittest.main()
