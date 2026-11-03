"""Read-only full RGB-D sidecar. A local stop file seals only this recorder."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import time


def wait_for_bag_finalize(bag, timeout_sec=30.):
    """The rosbag wrapper can exit before its writer finishes the final rename."""
    bag = Path(bag)
    active = Path(str(bag) + '.active')
    deadline = time.monotonic() + timeout_sec
    while not (bag.is_file() and not active.exists()):
        if time.monotonic() >= deadline:
            raise TimeoutError('Owned rosbag writer did not finalize the bag before the close deadline')
        time.sleep(.05)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--duration', type=int, default=600)
    args = parser.parse_args()
    mount = Path('/media/zhuyupei/SanStand')
    root = mount / '实验记录/Robot-Controller'
    assert mount.is_mount() and root / 'ros_log' in args.output.resolve().parents
    assert 1 <= args.duration <= 600
    assert shutil.disk_usage(root).free >= 10 * 2**30
    args.output.mkdir(exist_ok=False)
    os.environ['ROS_LOG_DIR'] = str(args.output / 'ros')
    topics = [
        '/supervisor/camera/color/image_raw', '/supervisor/camera/depth/image_raw',
        '/tf', '/tf_static', '/alicia_d/accepted_joint_states',
        '/alicia_d/control_reference_epoch', '/gui/joint_direct_mode',
        '/grasp_mode/selection', '/grasp_mode/status', '/grasp/state',
        '/grasp/near_field_phase', '/perception/unknown/detector_status',
        '/perception/unknown/object', '/perception/unknown/object_mask',
        '/grasp_mode/observe_return_measurement_request',
        '/grasp_mode/observe_return_measurement_ack',
        '/grasp_6d/preview_plan_enriched',
        '/alicia_controller/follow_joint_trajectory/status',
        '/alicia_controller/follow_joint_trajectory/goal',
        '/alicia_controller/follow_joint_trajectory/result',
        '/hand_controller/follow_joint_trajectory/status',
        '/hand_controller/follow_joint_trajectory/goal',
        '/hand_controller/follow_joint_trajectory/result',
    ]
    command = ['rosbag', 'record', '--duration=%d' % args.duration, '--buffsize=256',
               '-O', str(args.output / 'full_frames.bag'), *topics]
    result = dict(recording_closed=False, topics=topics, command=command,
                  started_ns=time.time_ns(), motion_commands_sent=False,
                  temperature_queried=False, full_frame_sidecar_only=True)
    def save():
        temp = args.output / 'result.json.tmp'
        temp.write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n')
        temp.replace(args.output / 'result.json')
    stop = args.output / 'stop_requested'
    with (args.output / 'recorder.log').open('x') as log:
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        stat = Path('/proc/%d/stat' % process.pid).read_text().rsplit(')', 1)[1].split()
        result.update(pid=process.pid, start_ticks=stat[19])
        save()
        try:
            deadline = time.monotonic() + args.duration + 20
            while process.poll() is None:
                if stop.exists() or time.monotonic() >= deadline:
                    # Popen owns this child, so no PID-name matching or unrelated signal.
                    result['stop_requested_ns'] = time.time_ns()
                    process.send_signal(signal.SIGINT)
                    break
                time.sleep(.2)
            result['returncode'] = process.wait(timeout=30)
        except BaseException:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                process.wait(timeout=30)
            result['interrupted'] = True
            save()
            raise
    bag = args.output / 'full_frames.bag'
    result['recorder_parent_exited_ns'] = time.time_ns()
    save()
    close_started = time.monotonic()
    try:
        wait_for_bag_finalize(bag)
    except Exception as exc:
        result['close_verification_error'] = repr(exc)
        save()
        raise
    result['finalize_wait_sec'] = time.monotonic() - close_started
    digest = hashlib.sha256()
    with bag.open('rb') as stream:
        for block in iter(lambda: stream.read(8 * 2**20), b''):
            digest.update(block)
    result.update(recording_closed=True, closed_ns=time.time_ns(), path=str(bag),
                  size_bytes=bag.stat().st_size, sha256=digest.hexdigest())
    save()
    print(json.dumps(result, ensure_ascii=False), flush=True)


if __name__ == '__main__':
    main()
