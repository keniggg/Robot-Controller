"""Finite passive RGB-D/TF/feedback record. Creates no command publisher."""
import argparse
import datetime
import json
import signal
import time
import numpy as np
import rospy
from collect_high_posture import BoundRecorder
from collect_single_axis import MOUNT,OUTROOT


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--record',action='store_true')
    args=parser.parse_args()
    if not args.record or not MOUNT.is_mount():raise RuntimeError('explicit record and mounted SanStand required')
    stamp=datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    out=OUTROOT/('static_hold_'+stamp);out.mkdir()
    rospy.init_node('support_static_hold_readonly',anonymous=True,disable_signals=True)
    rec=None
    def interrupted(sig,_):raise RuntimeError('passive record interrupted signal%d'%sig)
    for sig in (signal.SIGINT,signal.SIGTERM,signal.SIGALRM):signal.signal(sig,interrupted)
    signal.alarm(40)
    try:
        rec=BoundRecorder(out);rec.snapshot('hold_start')
        initial_q,initial_sdk,_=rec.joint_and_sdk();epoch=rec.epoch_ns;rec.armed=True
        def guard():
            rec.guard()
            if rec.epoch_ns!=epoch:raise RuntimeError('hold reference epoch changed')
            q,sdk,_=rec.joint_and_sdk()
            if not np.array_equal(sdk,initial_sdk):raise RuntimeError('hold SDK target changed')
            if np.max(abs(q[:6]-initial_q[:6]))>np.deg2rad(.2):raise RuntimeError('hold actual posture changed')
        start=time.monotonic()
        for seconds,label in [(10.,'hold_10sec'),(20.,'hold_20sec')]:
            while time.monotonic()-start<seconds:guard();time.sleep(.05)
            rec.snapshot(label)
        guard();q,sdk,_=rec.joint_and_sdk()
        (out/'result.json').write_text(json.dumps(dict(
            passive_record_completed=True,no_robot_commands_published=True,
            initial_q_rad=initial_q.tolist(),final_q_rad=q.tolist(),sdk_q_rad=sdk.tolist(),
            actual_change_deg=np.rad2deg(q[:6]-initial_q[:6]).tolist(),epoch_ns=epoch,
            latest_camera_profile=rospy.get_param('/camera/runtime_profile'),
            purpose='late static exposure/geometry evidence; does not claim unique hardware attribution'),indent=2)+'\n')
        rec.mark('passive_record_complete',no_robot_commands_published=True)
    except BaseException as exc:
        if rec is not None:rec.mark('passive_record_failed',reason=str(exc))
        raise
    finally:
        signal.alarm(0)
        if rec is not None:rec.close()


if __name__=='__main__':main()
