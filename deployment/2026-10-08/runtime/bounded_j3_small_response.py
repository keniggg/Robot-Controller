#!/usr/bin/env python3
"""One fresh +8 SDK-word J3 diagnostic; original failed J4 anchor remains binding.

Default read-only. No return, retry, calibration change, fault clearing, or
historical action plan input. Success concerns only this smaller input.
"""
import argparse
from datetime import datetime,timezone
import hashlib,json,math,os,signal
from pathlib import Path
import numpy as np
import bounded_board_centering as board
from diagnostic_failure_tail import record_failure_tail

HERE=Path(__file__).resolve().parent
PINNED_EPOCH=1790645893464309304
INITIAL_SDK_WORDS=(853,2270,1991,2048,1912,2045)
INITIAL_SDK_GRIPPER_M=.04975
OLD_J4_ANCHOR_RAD=board.motor.QUANTUM
OLD_J4_BOUND_RAD=math.radians(.4)
PRIOR_SESSION=board.wrist.ARCHIVE/'bounded_shoulder_elbow_validation_20260929T025136Z_167696'
PRIOR_SHA256={'terminal.json':'348ee928b6d6c05155ee5613c3574366b07ebb29397d222cf1e1ae5674eccd43',
              'plan.json':'baf99f9f8ff6e767d2f47105cd4e193b39dff85ce3ce1081f79e7c87e3e82c99'}


def prior_failure_record():
    data={}
    for name,expected in PRIOR_SHA256.items():
        raw=(PRIOR_SESSION/name).read_bytes()
        if hashlib.sha256(raw).hexdigest()!=expected:raise RuntimeError('original failed trial artifact changed: '+name)
        data[name]=json.loads(raw)
    terminal,plan=data['terminal.json'],data['plan.json']
    if (terminal['status']!='failed' or terminal['commands_published']!=2 or
        plan['frozen_epoch_ns']!=PINNED_EPOCH or abs(plan['initial_measured_rad'][3]-OLD_J4_ANCHOR_RAD)>1e-15 or
        tuple(plan['commands'][-1]['words'])!=INITIAL_SDK_WORDS):
        raise RuntimeError('original failure identity/anchor/SDK metadata differs')
    return dict(session=str(PRIOR_SESSION),artifact_sha256=dict(PRIOR_SHA256),terminal=terminal,
                original_failure_cleared=False,original_failed_command_completed=False,
                old_J4_anchor_rad=OLD_J4_ANCHOR_RAD,permanent_J4_bound_rad=OLD_J4_BOUND_RAD)


def require_original_j4_anchor(current):
    q=board.wrist.vector(current)
    if abs(q[3]-OLD_J4_ANCHOR_RAD)>OLD_J4_BOUND_RAD:
        raise RuntimeError('J4 escaped permanent original failed-trial anchor +/-0.4 degrees')
    return q


def require_initial_sdk(current):
    q=board.wrist.vector(current)
    if tuple(board.words(q))!=INITIAL_SDK_WORDS or abs(q[6]-INITIAL_SDK_GRIPPER_M)>1e-10:
        raise RuntimeError('SDK differs from pinned original failed-trial endpoint; no following external changes')
    return q


class SmallResponseContract(board.motor.CenteringContract):
    def __init__(self,measured,reference,epoch):
        if epoch!=PINNED_EPOCH:raise RuntimeError('driver epoch differs from pinned original failed trial')
        require_initial_sdk(reference)
        super().__init__(measured,reference,epoch,(0,0,8))
        self.labels=['small_J3_positive']
        if self.indices!=[2] or len(self.targets)!=1:raise RuntimeError('single J3 contract changed')
    def assert_feedback(self,current,epoch):
        try:require_original_j4_anchor(current)
        except RuntimeError as error:self.fail(str(error))
        return super().assert_feedback(current,epoch)


def make_diagnostic_class(base):
    collector=board.make_diagnostic_class(base)
    class Diagnostic(collector):
        def __init__(self,args,directory):
            self.prior_failure=prior_failure_record()
            super().__init__(args,directory)
            base.write_json(directory/'prior_failure_record.json',self.prior_failure)
        def guard(self):
            super().guard()
            with self.lock:
                try:
                    if self.epoch!=PINNED_EPOCH:raise RuntimeError('driver epoch differs from pinned original failed trial')
                    require_original_j4_anchor(base.positions(self.accepted[-1]))
                    if self.contract is None or self.contract.count==0:require_initial_sdk(base.positions(self.reference[-1]))
                except RuntimeError as error:
                    self.fault=self.fault or str(error)
                    raise
        def receive(self,topic,msg):
            # Check every raw accepted callback, including an excursion which
            # could return before the next synchronous guard. Still archive it.
            with self.pair_changed:
                if self.closed:return
                try:
                    if topic=='/alicia_d/accepted_joint_states':require_original_j4_anchor(base.positions(msg))
                    elif topic=='/alicia_d/control_reference' and (self.contract is None or self.contract.count==0):require_initial_sdk(base.positions(msg))
                    elif topic=='/alicia_d/control_reference_epoch' and msg.stamp.to_nsec()!=PINNED_EPOCH:
                        raise RuntimeError('driver epoch differs from pinned original failed trial')
                except RuntimeError as error:
                    self.fault=self.fault or str(error)
                    if self.contract is not None:self.contract.failed=self.contract.failed or str(error)
                    self.pair_changed.notify_all()
            return super().receive(topic,msg)
        def run(self):
            self.await_initial();self.stable_hold_recording(3.)
            geometry=self.snapshot('baseline')
            with self.lock:
                self.guard()
                measured=base.positions(self.accepted[-1]);reference=base.positions(self.reference[-1])
                if np.max(abs(measured[:6]-geometry['q'][:6]))>math.radians(.15):raise RuntimeError('fresh board image differs from settled start')
                board.wrist.validate_source_binding(base.rospy.Time.now().to_nsec(),self.frozen_epoch,
                    self.accepted[-1].header.stamp.to_nsec(),self.reference[-1].header.stamp.to_nsec(),geometry['source_stamp_ns'])
                geometry['q']=measured.copy();self.contract=SmallResponseContract(measured,reference,self.frozen_epoch)
            target=self.contract.targets[0]
            plan=dict(schema=1,purpose='single smaller J3 input diagnostic, not retry or completion of failed +22 input',execute=self.args.execute,
                maximum_commands=1,initial_measured_rad=measured.tolist(),starting_sdk_rad=reference.tolist(),starting_sdk_words=board.words(reference),frozen_epoch_ns=self.frozen_epoch,
                source_stamp_ns=geometry['source_stamp_ns'],board_observation=geometry['board_observation'],baseline_recording_seconds=3.,
                increment_words=8,increment_deg=math.degrees(8*board.motor.QUANTUM),
                actual_J3_start_word=board.words(measured)[2],SDK_J3_start_word=board.words(reference)[2],SDK_J3_target_word=board.words(target)[2],
                nominal_J3_target_minus_actual_deg=math.degrees(target[2]-measured[2]),SDK_J3_bias_deg=math.degrees(reference[2]-measured[2]),
                actual_response_bound_is_not_eight_words=True,prior_failure=self.prior_failure,
                permanent_J4_interval_deg=np.degrees([OLD_J4_ANCHOR_RAD-OLD_J4_BOUND_RAD,OLD_J4_ANCHOR_RAD+OLD_J4_BOUND_RAD]).tolist(),
                original_J4_constraint_only_tightens_feedback_not_replaces_full_geometry=True,
                robot_description_sha256=self.archived_urdf_sha256,frozen_T_tool_camera=self.frozen_observation_x.tolist(),camera_model_fingerprint=self.frozen_observation_camera,
                commands=[dict(label=self.contract.labels[0],joint=3,target_rad=target.tolist(),words=board.words(target))],
                automatic_return=False,automatic_retry=False,bootstrap_pending=False,old_failure_cleared=False,
                source='fresh board/measured/SDK; epoch and original SDK identity pinned; original fault immutable',archive_policy='local_only',upload_enabled=False,delete_enabled=False)
            base.write_json(self.out/'plan.json',plan)
            if not self.args.execute:
                plan['first_step_geometry']=self.path(measured,target,geometry,reference)
                base.write_json(self.out/'plan.json',plan);self.event('prepared_no_motion',plan=str(self.out/'plan.json'));return
            observed=self.execute_step(self.contract.labels[0],target)
            self.event('single_small_J3_observation_complete',commands_published=self.publish_count,
                actual_latest_board_observation=observed['board_observation'],original_failed_trial_remains_failed=True,automatic_return=False,automatic_retry=False)
    return Diagnostic


def parse_args(argv=None):
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--execute',action='store_true');a=p.parse_args(argv)
    a.bootstrap_pending=False;a.bootstrap_sync_bound_deg=4.8;a.joint=3
    return a


def main():
    a=parse_args()
    import pyrealsense2 as _sdk_import_check
    if not os.path.ismount(board.wrist.MOUNT) or not board.wrist.ARCHIVE.is_dir():
        raise RuntimeError('SanStand unavailable; no fallback')
    out=board.wrist.ARCHIVE/('bounded_j3_small_response_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')+'_'+str(os.getpid()))
    out.mkdir();(out/'ros').mkdir();os.environ['ROS_LOG_DIR']=str(out/'ros')
    manifest={}
    for name in ('bounded_j3_small_response.py','bounded_board_centering.py','bounded_motor_centering.py',
                 'bounded_wrist_response_diagnostic.py','bounded_joint_response_diagnostic.py',
                 'bounded_async_bag_writer.py','bootstrap_mixed_geometry.py','coupled_joint23_geometry.py',
                 'diagnostic_failure_tail.py'):
        data=(HERE/name).read_bytes();(out/name).write_bytes(data);manifest[name]=hashlib.sha256(data).hexdigest()
    (out/'source_sha256.json').write_text(json.dumps(manifest,indent=2)+'\n')
    base=board.motor.load_module('small_j3_response_base',HERE/'bounded_joint_response_diagnostic.py')
    base.rospy.init_node('bounded_j3_small_response',anonymous=True,disable_signals=True)
    terminal=dict(status='failed',execute=a.execute,commands_published=0);experiment=None
    def alarm(*_):raise RuntimeError('180-second single small J3 observation deadline exhausted')
    signal.signal(signal.SIGALRM,alarm);signal.setitimer(signal.ITIMER_REAL,180.)
    try:
        experiment=make_diagnostic_class(base)(a,out)
        experiment.run();terminal['status']='completed' if a.execute else 'prepared_no_motion'
    except BaseException as error:
        terminal['reason']=str(error)
        if experiment is not None:
            experiment.event('no_further_commands',reason=str(error),recovery_attempted=False,stop_or_disable_sent=False)
            terminal['passive_failure_tail']=record_failure_tail(experiment,str(error),is_shutdown=base.rospy.is_shutdown)
        raise
    finally:
        signal.setitimer(signal.ITIMER_REAL,0)
        if experiment is not None:
            terminal['commands_published']=experiment.publish_count
            try:experiment.close()
            except BaseException as error:terminal.update(status='failed',reason='archive close failed: '+str(error))
            terminal['archive_writer']=experiment.writer_summary or experiment.archive_writer.summary()
            if not terminal['archive_writer']['archive_complete']:terminal.update(status='failed',reason='archive incomplete')
        base.write_json(out/'terminal.json',terminal);print('SESSION '+str(out),flush=True)
    if terminal['status']=='failed':raise RuntimeError(terminal.get('reason','archive failed'))

if __name__=='__main__':main()
