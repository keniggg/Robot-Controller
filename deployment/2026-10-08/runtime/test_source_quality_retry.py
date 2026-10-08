"""Source selection/retention regressions; no ROS connection or device access."""
import hashlib
import json
import pathlib
import sys
import tempfile
import unittest
from unittest import mock

import numpy as np

sys.path.insert(0,str(pathlib.Path(__file__).parent))
import gen4_orthogonal_probe_plan as m


class SourceQualityRetryTests(unittest.TestCase):
 def setUp(self):
  self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
  self.directory=pathlib.Path(self.tmp.name)/'fresh_plan'
  self.xml='<robot name="complete_measured_source"/>'
  self.calls=[]

 def write_raw(self,directory,stamp):
  directory.joinpath('source_rgb.png').write_bytes(b'original_RGB_'+str(stamp).encode())
  np.savez_compressed(str(directory/'source_raw.npz'),source_stamp_ns=np.int64(stamp),depth_m=np.array([[.4]],np.float32))
  directory.joinpath('source_binding.json').write_text(json.dumps({'source_stamp_ns':stamp}))

 def complete(self,directory,stamp):
  self.write_raw(directory,stamp)
  snapshot={'source_stamp_ns':stamp,'urdf_sha256':hashlib.sha256(self.xml.encode()).hexdigest(),
            'fit_diagnostic':'completed producer with unchanged standards'}
  for name in m.SOURCE_CAPTURE_FILES:
   if name in ('source_rgb.png','source_raw.npz','source_binding.json'):continue
   if name=='snapshot.json':directory.joinpath(name).write_text(json.dumps(snapshot))
   elif name=='robot_description.urdf':directory.joinpath(name).write_text(self.xml)
   else:directory.joinpath(name).write_bytes(('full_capture_'+str(stamp)+'_'+name).encode())
  return snapshot,self.xml

 def capture_outcomes(self,outcomes):
  iterator=iter(outcomes)
  def capture(directory):
   self.calls.append(directory)
   stamp,error=next(iterator)
   if error is not None:
    self.write_raw(directory,stamp)
    raise error
   return self.complete(directory,stamp)
  return capture

 def trace(self):return json.loads((self.directory/'source_quality_retry.json').read_text())

 def test_first_valid_capture_selected_without_extra_calls(self):
  result=m.capture_with_source_quality_retry(self.directory,self.capture_outcomes([(100,None)]))
  self.assertEqual(result[0]['source_stamp_ns'],100);self.assertEqual(len(self.calls),1)
  trace=self.trace();self.assertEqual(trace['selected_attempt'],1);self.assertEqual(trace['selected_source_stamp_ns'],100)
  for name in m.SOURCE_CAPTURE_FILES:self.assertEqual((self.directory/name).read_bytes(),(self.calls[0]/name).read_bytes())

 def test_two_bad_pairs_preserved_and_third_complete_pair_promoted(self):
  bad=lambda:ValueError('support_plane_inlier_ratio_low')
  result=m.capture_with_source_quality_retry(self.directory,self.capture_outcomes([(100,bad()),(200,bad()),(300,None)]))
  self.assertEqual(result[0]['source_stamp_ns'],300);self.assertEqual(len(self.calls),3)
  trace=self.trace();self.assertEqual(trace['selected_attempt'],3);self.assertEqual([r['source_stamp_ns']for r in trace['attempts']],[100,200,300])
  self.assertEqual([r['retry']for r in trace['attempts']],[True,True,False])
  for i,path in enumerate(self.calls[:2]):
   self.assertEqual(path.joinpath('source_rgb.png').read_bytes(),b'original_RGB_'+str((i+1)*100).encode())
   record=json.loads((path/'source_capture_result.json').read_text())
   self.assertEqual(record['reason'],'support_plane_inlier_ratio_low')
   self.assertFalse(path.joinpath('snapshot.json').exists())
  for name in m.SOURCE_CAPTURE_FILES:self.assertEqual((self.directory/name).read_bytes(),(self.calls[-1]/name).read_bytes())

 def test_three_bad_pairs_stop_without_root_source_or_fourth_call(self):
  error=ValueError('support_plane_inlier_ratio_low')
  with self.assertRaisesRegex(ValueError,'^support_plane_inlier_ratio_low$'):
   m.capture_with_source_quality_retry(self.directory,self.capture_outcomes([(1,error),(2,error),(3,error),(4,None)]))
  self.assertEqual(len(self.calls),3);self.assertIsNone(self.trace()['selected_attempt'])
  self.assertFalse(self.trace()['attempts'][-1]['retry'])
  for name in m.SOURCE_CAPTURE_FILES:self.assertFalse((self.directory/name).exists())

 def test_other_errors_timeout_and_interrupt_fail_immediately(self):
  for error in (ValueError('support_plane_not_found'),RuntimeError('source feedback stale'),
                TimeoutError('full source deadline'),KeyboardInterrupt('cancelled'),
                ValueError('support_plane_inlier_ratio_low: partial'),
                RuntimeError('support_plane_inlier_ratio_low')):
   with self.subTest(error=repr(error)):
    self.directory=pathlib.Path(self.tmp.name)/('case_'+str(len(self.calls)))
    before=len(self.calls)
    with self.assertRaises(type(error)):
     m.capture_with_source_quality_retry(self.directory,self.capture_outcomes([(100,error),(200,None)]))
    self.assertEqual(len(self.calls),before+1);self.assertIsNone(self.trace()['selected_source_stamp_ns'])
    self.assertFalse(self.trace()['attempts'][0]['retry'])
    self.assertEqual(self.trace()['attempts'][0]['reason'],str(error))

 def test_error_after_ratio_failure_is_not_retried(self):
  outcomes=[(10,ValueError('support_plane_inlier_ratio_low')),(20,TimeoutError('complete source timed out')),(30,None)]
  with self.assertRaisesRegex(TimeoutError,'complete source timed out'):
   m.capture_with_source_quality_retry(self.directory,self.capture_outcomes(outcomes))
  self.assertEqual(len(self.calls),2);self.assertIsNone(self.trace()['selected_attempt'])

 def test_same_or_older_source_after_failed_pair_is_rejected(self):
  for stamp in (100,99):
   with self.subTest(stamp=stamp):
    self.directory=pathlib.Path(self.tmp.name)/('stamp_'+str(stamp));before=len(self.calls)
    with self.assertRaisesRegex(RuntimeError,'source did not advance'):
     m.capture_with_source_quality_retry(self.directory,self.capture_outcomes([(100,ValueError('support_plane_inlier_ratio_low')),(stamp,None),(300,None)]))
    self.assertEqual(len(self.calls),before+2);self.assertIsNone(self.trace()['selected_attempt'])

 def test_incomplete_return_is_not_promoted_or_retried(self):
  def capture(directory):
   self.calls.append(directory);result=self.complete(directory,100)
   (directory/'source_evidence.bag').unlink();return result
  with self.assertRaisesRegex(RuntimeError,'completed capture evidence missing: source_evidence.bag'):
   m.capture_with_source_quality_retry(self.directory,capture)
  self.assertEqual(len(self.calls),1);self.assertFalse((self.directory/'snapshot.json').exists())

 def test_failed_ratio_without_raw_evidence_cannot_retry(self):
  def capture(directory):
   self.calls.append(directory);raise ValueError('support_plane_inlier_ratio_low')
  with self.assertRaisesRegex(RuntimeError,'failed plane capture raw evidence missing'):
   m.capture_with_source_quality_retry(self.directory,capture)
  self.assertEqual(len(self.calls),1);self.assertIn('evidence_error',self.trace()['attempts'][0])

 def test_mismatching_raw_stamp_and_model_fail_without_more_captures(self):
  for kind in ('stamp','model'):
   with self.subTest(kind=kind):
    self.directory=pathlib.Path(self.tmp.name)/kind;before=len(self.calls)
    def capture(directory):
     self.calls.append(directory);result=self.complete(directory,100)
     if kind=='stamp':np.savez_compressed(str(directory/'source_raw.npz'),source_stamp_ns=np.int64(99))
     else:(directory/'robot_description.urdf').write_text('<robot name="other"/>')
     return result
    with self.assertRaisesRegex(RuntimeError,'source stamp disagreement|robot model disagreement'):
     m.capture_with_source_quality_retry(self.directory,capture)
    self.assertEqual(len(self.calls),before+1);self.assertIsNone(self.trace()['selected_attempt'])

 def test_old_root_capture_and_existing_attempt_tree_cannot_be_reused(self):
  self.directory.mkdir();(self.directory/'snapshot.json').write_text('{}')
  capture=mock.Mock()
  with self.assertRaisesRegex(RuntimeError,'already contains capture evidence'):
   m.capture_with_source_quality_retry(self.directory,capture)
  capture.assert_not_called()
  (self.directory/'snapshot.json').unlink();(self.directory/'source_capture_attempts').mkdir()
  with self.assertRaises(FileExistsError):m.capture_with_source_quality_retry(self.directory,capture)
  capture.assert_not_called()

 def test_copy_failure_fails_without_another_capture_or_false_selection(self):
  with mock.patch.object(m.shutil,'copy2',side_effect=OSError('archive write failed')):
   with self.assertRaisesRegex(OSError,'archive write failed'):
    m.capture_with_source_quality_retry(self.directory,self.capture_outcomes([(100,None),(200,None)]))
  self.assertEqual(len(self.calls),1);self.assertIsNone(self.trace()['selected_attempt'])
  self.assertEqual(self.trace()['attempts'][0]['status'],'completed_capture_rejected')

 def test_original_fit_thresholds_and_fixed_attempt_limit_preserved(self):
  cfg=m.base.tt.Config()
  self.assertEqual(cfg.support_threshold_m,.002);self.assertEqual(cfg.support_min_ratio,.70)
  self.assertEqual(m.MAX_SOURCE_CAPTURE_ATTEMPTS,3)


if __name__=='__main__':unittest.main()
