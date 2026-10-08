#!/usr/bin/env python3
"""Two finite shoulder/elbow observations under the original runtime model.

Default read-only. One J2 -22 SDK-word command, then one J3 +22 command.
No return, retry, centering solver, calibration edits, sync or enable command.
"""
import argparse
from datetime import datetime, timezone
import hashlib,json,math,os,signal
from pathlib import Path
import numpy as np
import bounded_board_centering as board
from diagnostic_failure_tail import record_failure_tail

HERE=Path(__file__).resolve().parent
FIXED_INCREMENTS=(0,-22,22)


def fixed_contract(measured,reference,epoch):
    contract=board.board_contract(measured,reference,epoch,FIXED_INCREMENTS)
    if contract.indices!=[1,2] or len(contract.targets)!=2:
        raise RuntimeError('fixed two-axis validation protocol changed')
    contract.labels=['validation_J2_negative','validation_J3_positive']
    return contract


def make_diagnostic_class(base):
    collector=board.make_diagnostic_class(base)
    class Diagnostic(collector):
        def run(self):
            self.await_initial()
            self.stable_hold_recording(3.)
            geometry=self.snapshot('baseline')
            with self.lock:
                self.guard()
                measured=base.positions(self.accepted[-1]);reference=base.positions(self.reference[-1])
                if np.max(abs(measured[:6]-geometry['q'][:6]))>math.radians(.15):
                    raise RuntimeError('fresh board image differs from settled validation start')
                board.wrist.validate_source_binding(base.rospy.Time.now().to_nsec(),self.frozen_epoch,
                    self.accepted[-1].header.stamp.to_nsec(),self.reference[-1].header.stamp.to_nsec(),geometry['source_stamp_ns'])
                geometry['q']=measured.copy()
                self.contract=fixed_contract(measured,reference,self.frozen_epoch)
            plan=dict(schema=1,purpose='prospective original-model shoulder/elbow validation',execute=self.args.execute,
                maximum_commands=2,initial_measured_rad=measured.tolist(),starting_sdk_rad=reference.tolist(),frozen_epoch_ns=self.frozen_epoch,
                source_stamp_ns=geometry['source_stamp_ns'],board_observation=geometry['board_observation'],baseline_recording_seconds=3.,
                fixed_increment_words=list(FIXED_INCREMENTS),fixed_increment_deg=np.degrees(np.array(FIXED_INCREMENTS)*board.motor.QUANTUM).tolist(),
                robot_description_sha256=self.archived_urdf_sha256,frozen_T_tool_camera=self.frozen_observation_x.tolist(),camera_model_fingerprint=self.frozen_observation_camera,
                commands=[dict(label=label,joint=index+1,target_rad=target.tolist(),words=board.words(target))
                    for label,index,target in zip(self.contract.labels,self.contract.indices,self.contract.targets)],
                bootstrap_pending=False,automatic_return=False,automatic_retry=False,solver_used=False,calibration_changed=False,
                source='new exact board observation and current measured/SDK state; no old-plan input',
                archive_policy='local_only',upload_enabled=False,delete_enabled=False)
            base.write_json(self.out/'plan.json',plan)
            if not self.args.execute:
                self.args.joint=2
                plan['first_step_geometry']=self.path(measured,self.contract.targets[0],geometry,reference)
                base.write_json(self.out/'plan.json',plan)
                self.event('prepared_no_motion',plan=str(self.out/'plan.json'));return
            for index,label,target in zip(self.contract.indices,self.contract.labels,self.contract.targets):
                self.args.joint=index+1
                observed=self.execute_step(label,target)
            self.event('finite_shoulder_elbow_validation_complete',commands_published=self.publish_count,
                actual_latest_board_observation=observed['board_observation'],automatic_return=False,automatic_retry=False)
    return Diagnostic


def parse_args(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute',action='store_true')
    args=parser.parse_args(argv)
    args.bootstrap_pending=False;args.bootstrap_sync_bound_deg=4.8;args.joint=2
    return args


def main():
    a=parse_args()
    import pyrealsense2 as _sdk_import_check
    if not os.path.ismount(board.wrist.MOUNT) or not board.wrist.ARCHIVE.is_dir():
        raise RuntimeError('SanStand unavailable; no fallback')
    out=board.wrist.ARCHIVE/('bounded_shoulder_elbow_validation_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')+'_'+str(os.getpid()))
    out.mkdir();(out/'ros').mkdir();os.environ['ROS_LOG_DIR']=str(out/'ros')
    manifest={}
    for name in ('bounded_shoulder_elbow_validation.py','bounded_board_centering.py','bounded_motor_centering.py',
                 'bounded_wrist_response_diagnostic.py','bounded_joint_response_diagnostic.py',
                 'bounded_async_bag_writer.py','bootstrap_mixed_geometry.py','coupled_joint23_geometry.py',
                 'diagnostic_failure_tail.py'):
        data=(HERE/name).read_bytes();(out/name).write_bytes(data);manifest[name]=hashlib.sha256(data).hexdigest()
    (out/'source_sha256.json').write_text(json.dumps(manifest,indent=2)+'\n')
    base=board.motor.load_module('shoulder_elbow_validation_base',HERE/'bounded_joint_response_diagnostic.py')
    base.rospy.init_node('bounded_shoulder_elbow_validation',anonymous=True,disable_signals=True)
    terminal=dict(status='failed',execute=a.execute,commands_published=0);experiment=None
    def alarm(*_):raise RuntimeError('180-second finite shoulder/elbow observation deadline exhausted')
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
