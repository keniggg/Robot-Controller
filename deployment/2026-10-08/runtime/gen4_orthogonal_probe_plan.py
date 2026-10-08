#!/usr/bin/env python3
"""Source-bound J2/J5 direction reversals around the current held posture.

 Imports and the factory are read-only. Every probe returns the same original
 SDK words, whose physical return is measured independently rather than assumed.
 """
import hashlib
import json
import math
import pathlib
import shutil
import numpy as np
import gen4_high_posture_plan as high
base=high.base

MAX_SOURCE_CAPTURE_ATTEMPTS=3
SOURCE_CAPTURE_FILES=('source_rgb.png','source_raw.npz','source_binding.json',
                      'source_target_mask.png','source.npz','snapshot.json',
                      'robot_description.urdf','camera_parameters.json','source_evidence.bag')


def _captured_source_stamp(directory):
 """Read the measured failed/successful capture stamp, never a cached source."""
 path=directory/'source_binding.json'
 if not path.is_file():raise RuntimeError('fresh capture source binding evidence missing')
 value=json.loads(path.read_text())['source_stamp_ns']
 if isinstance(value,bool) or not isinstance(value,int) or value<=0:
  raise RuntimeError('fresh capture source stamp invalid')
 return value


def capture_with_source_quality_retry(directory,capture=None):
 """At most three full fresh captures; retain original fit/geometry standards.

 Only the exact measured-plane ratio ValueError permits another capture.
 Capture/time/authority/integrity failures never retry. No command interfaces.
 Selected files describe one complete capture, never a mixture of attempts.
 The factory's existing authority, scene and path audits still run afterwards.
 """
 directory=pathlib.Path(directory);directory.mkdir(parents=True,exist_ok=True)
 if any((directory/name).exists() for name in SOURCE_CAPTURE_FILES):
  raise RuntimeError('fresh source destination already contains capture evidence')
 attempts_directory=directory/'source_capture_attempts'
 attempts_directory.mkdir(exist_ok=False)
 capture=base.capture if capture is None else capture
 trace=dict(kind='bounded_fresh_source_quality_capture',max_attempts=MAX_SOURCE_CAPTURE_ATTEMPTS,
            retry_reason='ValueError: support_plane_inlier_ratio_low',attempts=[],
            selected_attempt=None,selected_source_stamp_ns=None,
            original_source_fit_and_execution_gates_unchanged=True)
 trace_path=directory/'source_quality_retry.json'
 base.write(trace_path,trace)
 previous_stamp=None
 for index in range(1,MAX_SOURCE_CAPTURE_ATTEMPTS+1):
  attempt=attempts_directory/('attempt_%02d'%index);attempt.mkdir()
  item=dict(attempt=index,directory=str(attempt),status='capturing',source_stamp_ns=None)
  trace['attempts'].append(item);base.write(trace_path,trace)
  try:
   snapshot,xml=capture(attempt)
  except BaseException as error:
   item.update(status='capture_failed',error_type=type(error).__name__,reason=str(error),retry=False)
   eligible=isinstance(error,ValueError) and str(error)=='support_plane_inlier_ratio_low'
   if eligible:
    try:
     for name in ('source_rgb.png','source_raw.npz','source_binding.json'):
      if not (attempt/name).is_file():raise RuntimeError('failed plane capture raw evidence missing: '+name)
     stamp=_captured_source_stamp(attempt);item['source_stamp_ns']=stamp
     if previous_stamp is not None and stamp<=previous_stamp:
      raise RuntimeError('fresh quality retry source did not advance')
     previous_stamp=stamp
    except BaseException as evidence_error:
     item['evidence_error']=str(evidence_error)
     base.write(attempt/'source_capture_result.json',item);base.write(trace_path,trace)
     raise
   item['retry']=eligible and index<MAX_SOURCE_CAPTURE_ATTEMPTS
   base.write(attempt/'source_capture_result.json',item);base.write(trace_path,trace)
   if not item['retry']:raise
   continue
  # A completed producer must still contain its whole matching source bundle.
  # Integrity failures cannot be made eligible by acquiring more frames.
  try:
   stamp=_captured_source_stamp(attempt);item['source_stamp_ns']=stamp
   if previous_stamp is not None and stamp<=previous_stamp:
    raise RuntimeError('fresh quality retry source did not advance')
   for name in SOURCE_CAPTURE_FILES:
    path=attempt/name
    if not path.is_file() or path.stat().st_size==0:
     raise RuntimeError('completed capture evidence missing: '+name)
   saved=json.loads((attempt/'snapshot.json').read_text())
   if snapshot['source_stamp_ns']!=stamp or saved['source_stamp_ns']!=stamp:
    raise RuntimeError('completed capture source stamp disagreement')
   with np.load(str(attempt/'source_raw.npz'),allow_pickle=False) as raw:
    if int(raw['source_stamp_ns'])!=stamp:
     raise RuntimeError('completed raw source stamp disagreement')
   if (attempt/'robot_description.urdf').read_text()!=xml or hashlib.sha256(xml.encode()).hexdigest()!=snapshot['urdf_sha256']:
    raise RuntimeError('completed capture robot model disagreement')
   bundle_sha256={name:hashlib.sha256((attempt/name).read_bytes()).hexdigest()
                  for name in SOURCE_CAPTURE_FILES}
   for name in SOURCE_CAPTURE_FILES:shutil.copy2(str(attempt/name),str(directory/name))
   for name,digest in bundle_sha256.items():
    if hashlib.sha256((directory/name).read_bytes()).hexdigest()!=digest:
     raise RuntimeError('selected source copy verification failed: '+name)
  except BaseException as error:
   item.update(status='completed_capture_rejected',error_type=type(error).__name__,reason=str(error),retry=False)
   base.write(attempt/'source_capture_result.json',item);base.write(trace_path,trace)
   raise
  item.update(status='selected_complete_capture',bundle_sha256=bundle_sha256,retry=False)
  trace.update(selected_attempt=index,selected_source_stamp_ns=stamp)
  base.write(attempt/'source_capture_result.json',item);base.write(trace_path,trace)
  return snapshot,xml
 raise RuntimeError('bounded source quality capture exhausted without selection')


def build_sequence(reference):
 reference=high.vector(reference)
 if reference.shape!=(7,):raise RuntimeError('seven-component source SDK required')
 sequence=[];current=reference.copy()
 # Each sign is repeated in the opposite order in a second pass. Time alone
 # must not be confused with the effect of changing the approach direction.
 for repeat,signs in ((1,(-1,1)),(2,(1,-1))):
  for axis in (1,4):
   for sign in signs:
    goal=reference.copy();goal[axis]+=sign*high.STEP_COUNTS*base.QUANTUM
    stem='repeat%d_j%d_%s3'%(repeat,axis+1,'negative' if sign<0 else 'positive')
    for target,label,phase in ((goal,stem,'probe'),(reference.copy(),stem+'_return','probe_return')):
     before=base.words(current);after=base.words(target)
     if [i for i in range(6) if before[i]!=after[i]]!=[axis] or abs(after[axis]-before[axis])!=34:
      raise RuntimeError('probe must edit exactly one axis by34words')
     sequence.append(dict(index=len(sequence),axis=axis,label=label,phase=phase,
                          before_sdk=current.tolist(),target_sdk=target.tolist(),before_words=before,target_words=after))
     current=target.copy()
 if not np.array_equal(current,reference):raise RuntimeError('original SDK not restored')
 return sequence


class OrthogonalProbeAudit(high.HighPostureAudit):
 def __init__(self,snapshot,xml,validity,audit_factory=None):
  super().__init__(snapshot,xml,validity,audit_factory=audit_factory)
  self.sequence=build_sequence(self.reference)
  targets=np.array([self.reference[:6],self.source_q]+[s['target_sdk'][:6] for s in self.sequence])
  self.absolute_lo=targets.min(0)-math.radians(high.EXTRA_DEG)
  self.absolute_hi=targets.max(0)+math.radians(high.EXTRA_DEG)
  self.contract.pop('cycles');self.contract.pop('j2_probe_sign')
  self.contract.update(kind='current_posture_independent_J2_J5_signed_repeat_probes',sequence=self.sequence,
                       absolute_actual_bounds_rad=[self.absolute_lo.tolist(),self.absolute_hi.tolist()],
                       purpose='Separate direction/history and J2/J5 conditional responses; no unique physical component attribution assumed.')


def prepare_executor_audit(directory,cycles=7,j2_probe_sign=1):
 import rospy
 from sensor_msgs.msg import JointState
 from moveit_msgs.msg import RobotState
 from moveit_msgs.srv import GetStateValidity,GetStateValidityRequest
 if cycles!=7 or j2_probe_sign!=1:raise RuntimeError('orthogonal factory uses its fixed signed repeat protocol')
 directory=pathlib.Path(directory);directory.mkdir(parents=True,exist_ok=True)
 snapshot,xml=capture_with_source_quality_retry(directory)
 if snapshot['task_active'] or snapshot['protection_latched'] or not snapshot['feedback_ready'] or not snapshot['gui_direct_mode'] or not snapshot['actuation_status'].startswith('CONFIRMED:'):
  raise RuntimeError('fresh task/protection/feedback/manual/CONFIRMED evidence unavailable')
 rospy.wait_for_service('/check_state_validity',timeout=3);proxy=rospy.ServiceProxy('/check_state_validity',GetStateValidity)
 def valid(q,g):
  request=GetStateValidityRequest(group_name='alicia');request.robot_state=RobotState();request.robot_state.joint_state=JointState(name=base.NAMES,position=list(q)+[g]);return proxy(request).valid
 obj=OrthogonalProbeAudit(snapshot,xml,valid);proof=obj.audit_nominal_sequence()
 base.write(directory/'factory_audit_proof.json',proof);base.write(directory/'contract.json',obj.contract)
 (directory/'factory_source.py').write_bytes(pathlib.Path(__file__).read_bytes())
 base.write(directory/'dependencies.json',dict(high_plan_sha256=hashlib.sha256(pathlib.Path(high.__file__).read_bytes()).hexdigest(),base_plan_sha256=hashlib.sha256(pathlib.Path(base.__file__).read_bytes()).hexdigest()))
 return obj
