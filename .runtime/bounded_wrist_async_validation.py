#!/usr/bin/env python3
"""Original finite wrist protocol with the reviewed asynchronous board recorder.

No calibration fitting or model writes. The original WristContract and path
proof are used unchanged, including the full symmetric initial response box.
"""
import argparse
from datetime import datetime, timezone
import hashlib,json,os,signal,time
from pathlib import Path
import bounded_board_centering as board

HERE=Path(__file__).resolve().parent

def make_diagnostic_class(base):
    collector=board.make_diagnostic_class(base)
    original=board.wrist.make_diagnostic_class(base)
    class Diagnostic(collector):
        def __init__(self,args,directory):
            super().__init__(args,directory)
            self.deadline=time.monotonic()+85.
        # Keep the original symmetric wrist geometry and two-command protocol.
        path=original.path
        def run(self):
            self.await_initial()
            self.stable_hold_recording(3.)
            return original.run(self)
    return Diagnostic

def parse_args(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--execute',action='store_true')
    p.add_argument('--joint',type=int,choices=(4,6),required=True)
    p.add_argument('--direction',type=int,choices=(-1,1),default=1)
    a=p.parse_args(argv)
    a.bootstrap_pending=False;a.bootstrap_sync_bound_deg=4.8
    return a

def main():
    a=parse_args()
    import pyrealsense2 as _sdk_import_check
    if not os.path.ismount(board.wrist.MOUNT) or not board.wrist.ARCHIVE.is_dir():
        raise RuntimeError('SanStand unavailable; no fallback')
    out=board.wrist.ARCHIVE/('bounded_wrist_async_validation_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')+'_'+str(os.getpid()))
    out.mkdir();(out/'ros').mkdir();os.environ['ROS_LOG_DIR']=str(out/'ros')
    manifest={}
    for name in ('bounded_wrist_async_validation.py','bounded_board_centering.py','bounded_motor_centering.py',
                 'bounded_wrist_response_diagnostic.py','bounded_joint_response_diagnostic.py',
                 'bounded_async_bag_writer.py','bootstrap_mixed_geometry.py','coupled_joint23_geometry.py'):
        data=(HERE/name).read_bytes();(out/name).write_bytes(data);manifest[name]=hashlib.sha256(data).hexdigest()
    (out/'source_sha256.json').write_text(json.dumps(manifest,indent=2)+'\n')
    base=board.motor.load_module('async_wrist_validation_base',HERE/'bounded_joint_response_diagnostic.py')
    base.rospy.init_node('bounded_wrist_async_validation',anonymous=True,disable_signals=True)
    terminal=dict(status='failed',execute=a.execute,commands_published=0);experiment=None
    def alarm(*_):raise RuntimeError('180-second finite wrist observation deadline exhausted')
    signal.signal(signal.SIGALRM,alarm);signal.setitimer(signal.ITIMER_REAL,180.)
    try:
        experiment=make_diagnostic_class(base)(a,out)
        experiment.run();terminal['status']='completed' if a.execute else 'prepared_no_motion'
    except BaseException as error:
        terminal['reason']=str(error)
        if experiment is not None:
            experiment.event('no_further_commands',reason=str(error),recovery_attempted=False,stop_or_disable_sent=False)
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
