"""Authorized finite RGB-board experiment using the reviewed bounded executor.

One direct-mode ownership request retains existing SDK hold. No torque command,
enable/sync recovery, calibration change or automatic return after failure.
"""
import argparse
import datetime
import hashlib
import json
import signal
import sys
import time
import numpy as np
import rospy
from std_msgs.msg import Bool
import collect_high_posture as executor
from collect_single_axis import MOUNT, OUTROOT, Recorder
from board_source_capture import BoardRecorder, base


class OwnedBoardRecorder(BoardRecorder):
    def __init__(self, out):
        super().__init__(out)
        # The user explicitly authorized autonomous adjustment. Freeze the last
        # transmitted hold before ownership and verify it survives the switch.
        deadline = time.monotonic()+4.
        required = ['/alicia_d/sdk_command', '/alicia_d/control_reference', '/alicia_d/control_reference_epoch',
                    '/alicia_d/actuation_status', '/alicia_d/feedback_ready', '/alicia_d/motion_enabled',
                    '/alicia_d/protection_latched', '/gui/joint_direct_mode', '/grasp/state']
        while time.monotonic() < deadline:
            self.guard()
            with self.lock:
                if all(k in self.latest_inputs for k in required):
                    before = self.latest_inputs.copy()
                    break
            time.sleep(.025)
        else:
            raise RuntimeError('current control evidence unavailable before ownership')
        if (before['/grasp/state'][0].active or before['/alicia_d/protection_latched'][0].data
                or not before['/alicia_d/feedback_ready'][0].data or not before['/alicia_d/motion_enabled'][0].data
                or not before['/alicia_d/actuation_status'][0].data.startswith('CONFIRMED:')):
            raise RuntimeError('existing enabled CONFIRMED hold unavailable; no recovery permitted')
        sdk_message = before['/alicia_d/sdk_command'][0]
        sdk_values = dict(zip(sdk_message.name, sdk_message.position))
        initial_sdk = np.asarray([sdk_values[n] for n in base.NAMES[:6]] + [base.qmsg(before['/alicia_d/control_reference'][0])[6]], float)
        epoch = self.epoch_ns
        self.owner = rospy.Publisher('/gui/joint_direct_mode', Bool, queue_size=1, latch=True)
        deadline = time.monotonic()+3.
        while not self.owner.get_num_connections() and time.monotonic() < deadline:
            self.guard()
            time.sleep(.025)
        if not self.owner.get_num_connections():
            raise RuntimeError('direct ownership receiver unavailable')
        rospy.set_param('/gui/joint_direct_mode', True)
        self.owner.publish(Bool(data=True))
        self.mark('authorized_direct_ownership_request', mode=True, original_sdk_words=base.words(initial_sdk), epoch_ns=epoch,
                  torque_commands_published=0)
        deadline = time.monotonic()+4.
        while time.monotonic() < deadline:
            self.guard()
            with self.lock:
                measured_seen = bool(self.states)
            if not measured_seen:
                time.sleep(.025)
                continue
            try:
                q, sdk, _ = self.joint_and_sdk()
                with self.lock:
                    mode = self.latest_inputs.get('/gui/joint_direct_mode')
                if mode is not None and mode[0].data:
                    if self.epoch_ns != epoch or base.words(sdk) != base.words(initial_sdk) or sdk[6] != initial_sdk[6]:
                        raise RuntimeError('existing SDK hold / epoch changed on ownership; no motion')
                    self.mark('ownership_retained_sdk_hold', original_sdk_words=base.words(sdk), epoch_ns=epoch, q_rad=q.tolist())
                    held_q = q.copy()
                    self.armed = True
                    def frozen_hold_guard():
                        Recorder.guard(self)
                        current_q, current_sdk, _ = self.joint_and_sdk()
                        if self.epoch_ns != epoch or base.words(current_sdk) != base.words(initial_sdk) or current_sdk[6] != initial_sdk[6]:
                            raise RuntimeError('original SDK / epoch changed during board source preparation')
                        if np.max(abs(current_q[:6]-held_q[:6])) > np.deg2rad(.2):
                            raise RuntimeError('actual held posture changed during board source preparation')
                        with self.lock:
                            inputs = self.latest_inputs.copy()
                        if (not inputs['/gui/joint_direct_mode'][0].data or inputs['/alicia_d/protection_latched'][0].data
                                or not inputs['/alicia_d/motion_enabled'][0].data or not inputs['/alicia_d/feedback_ready'][0].data
                                or not inputs['/alicia_d/actuation_status'][0].data.startswith('CONFIRMED:')):
                            raise RuntimeError('existing direct enabled CONFIRMED hold lost during source preparation')
                    self.wait_guard_callback = frozen_hold_guard
                    frozen_hold_guard()
                    return
            except RuntimeError as error:
                if 'provenance or manual ownership' not in str(error):
                    raise
            time.sleep(.025)
        raise RuntimeError('driver did not acknowledge unchanged direct hold')


def passive_main(audit=False):
    if not MOUNT.is_mount():
        raise RuntimeError('SanStand unavailable; no fallback')
    out = OUTROOT/('board_hold_'+datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%SZ'))
    out.mkdir()
    rospy.init_node('fixed_board_exact_source_hold', anonymous=True, disable_signals=True)
    recorder = None
    signal.signal(signal.SIGALRM, lambda *_: (_ for _ in ()).throw(RuntimeError('passive board deadline')))
    signal.alarm(120 if audit else 45)
    try:
        recorder = OwnedBoardRecorder.__new__(OwnedBoardRecorder)
        OwnedBoardRecorder.__init__(recorder, out)
        if audit:
            import gen4_board_probe_plan as factory
            expected = '094d9d1a1bcda8e57ea54446932c5faa7d3e2d364ef878b9aa14e7fbe00137fe'
            if hashlib.sha256(base.ROOT.joinpath('.runtime/gen4_board_probe_plan.py').read_bytes()).hexdigest() != expected:
                raise RuntimeError('read-only stress factory differs from reviewed SHA')
            plan = factory.prepare_executor_audit(out/'fresh_plan')
            recorder.mark('readonly_geometry_stress_complete',steps=len(plan.sequence),robot_commands_published=0)
        stamps = []
        for label in ('board_hold_start', 'board_hold_repeat1', 'board_hold_repeat2'):
            stamps.append(recorder.snapshot(label, stamps[-1] if stamps else 0))
            executor.guarded_delay(1., recorder.guard)
        recorder.guard()
        base.write(out/'result.json', dict(passive_record_completed=True, no_robot_commands_published=True, source_stamps_ns=stamps))
        print(json.dumps(dict(directory=str(out), source_stamps_ns=stamps)), flush=True)
    except BaseException as error:
        if recorder is not None:
            recorder.mark('board_passive_failed', reason=str(error), robot_commands_published=0)
        raise
    finally:
        signal.alarm(0)
        if recorder is not None:
            recorder.close()


if __name__ == '__main__':
    if sys.argv[1:] == ['--record-only']:
        passive_main()
    elif sys.argv[1:] == ['--audit-only']:
        passive_main(audit=True)
    else:
        executor.BoundRecorder = OwnedBoardRecorder
        executor.main()
