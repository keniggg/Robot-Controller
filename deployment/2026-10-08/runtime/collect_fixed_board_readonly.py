#!/usr/bin/env python3
"""Finite passive recording after a completed/failed diagnostic; no publisher.

Keeps prior fault identity. Does not clear faults, return, plan or issue commands.
"""
from datetime import datetime,timezone
from pathlib import Path
import hashlib,json,os,signal
from types import SimpleNamespace
import bounded_board_centering as board
HERE=Path(__file__).resolve().parent
PREVIOUS='bounded_shoulder_elbow_validation_20260929T025136Z_167696'

def main():
    if not os.path.ismount(board.wrist.MOUNT):raise RuntimeError('SanStand unavailable')
    out=board.wrist.ARCHIVE/('fixed_board_passive_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ'))
    out.mkdir();(out/'ros').mkdir();os.environ['ROS_LOG_DIR']=str(out/'ros')
    manifest={}
    for name in ('collect_fixed_board_readonly.py','bounded_board_centering.py','bounded_motor_centering.py','bounded_wrist_response_diagnostic.py','bounded_joint_response_diagnostic.py','bounded_async_bag_writer.py','bootstrap_mixed_geometry.py','coupled_joint23_geometry.py'):
        data=(HERE/name).read_bytes();(out/name).write_bytes(data);manifest[name]=hashlib.sha256(data).hexdigest()
    (out/'source_sha256.json').write_text(json.dumps(manifest,indent=2)+'\n')
    base=board.motor.load_module('fixed_board_passive_base',HERE/'bounded_joint_response_diagnostic.py')
    base.rospy.init_node('fixed_board_passive',anonymous=True,disable_signals=True)
    args=SimpleNamespace(execute=False,bootstrap_pending=False,bootstrap_sync_bound_deg=4.8,joint=2)
    d=None;terminal={'status':'failed','execute':False,'commands_published':0,'previous_diagnostic':str(board.wrist.ARCHIVE/PREVIOUS),'previous_fault_preserved':True,'fault_cleared':False}
    try:
        d=board.make_diagnostic_class(base)(args,out)
        assert d.publisher is None and d.contract is None
        _,stationary=d.await_initial()
        d.stable_hold_recording(3.)
        g=d.snapshot('baseline')
        d.event('passive_recording_started',source_stamp_ns=g['source_stamp_ns'],previous_diagnostic=terminal['previous_diagnostic'],stationary=stationary)
        d.stable_hold_recording(6.)
        g=d.snapshot('final')
        d.event('passive_recording_finished',source_stamp_ns=g['source_stamp_ns'],commands_published=0,fault_cleared=False)
        terminal['status']='readonly_complete'
    except BaseException as e:
        terminal['reason']=str(e)
        raise
    finally:
        if d is not None:
            d.close();terminal['archive_writer']=d.writer_summary
            if not d.writer_summary['archive_complete']:terminal['status']='failed'
        base.write_json(out/'terminal.json',terminal)
        print('SESSION '+str(out),flush=True)
if __name__=='__main__':main()
