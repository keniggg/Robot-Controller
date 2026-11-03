"""One explicit maintenance retry, retaining the original user authorization.

Run ONLY as the child of run_recorded_grasp.py, after recording is ready.
No replayed target/source, gate changes, recursive retries or torque commands.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import time

WORK = Path(__file__).resolve().parents[1]
MOUNT = Path('/media/zhuyupei/SanStand')


def save(path, value):
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n')
    temporary.replace(path)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--prior-selection', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    out = args.output.resolve()
    record = Path(os.environ['GRASP_RECORD_DIR']).resolve()
    if not MOUNT.is_mount() or any(MOUNT not in p.parents for p in (out, record)):
        raise RuntimeError('SanStand recording required; no fallback')
    if not (record / 'recorder_ready.json').is_file():
        raise RuntimeError('Full recording must precede selection')
    prior = json.loads(args.prior_selection.read_text())
    receipt = prior['alignment_receipt']
    receipt_bytes = Path(prior['alignment_receipt_file']).read_bytes()
    if (hashlib.sha256(receipt_bytes).hexdigest() != prior['alignment_receipt_sha256']
            or json.loads(receipt_bytes) != receipt):
        raise RuntimeError('Original receipt changed')
    event_file = Path(receipt['event_file'])
    if hashlib.sha256(event_file.read_bytes()).hexdigest() != receipt['event_file_sha256']:
        raise RuntimeError('Original confirmation evidence changed')
    event = json.loads(event_file.read_text())
    if (receipt['kind'] != 'explicit_user_target_alignment'
            or receipt['target_aligned'] is not True
            or receipt['confirmation_source'] != event['confirmation_source']
            or receipt['confirmed_ns'] != event['confirmed_ns']):
        raise RuntimeError('Original user confirmation required')
    out.mkdir(exist_ok=False)
    import rospy
    from sensor_msgs.msg import JointState, Image
    from std_msgs.msg import Header
    from alicia_flexible_grasp_supervisor.msg import GraspState
    from alicia_grasp_modes.srv import SetGraspMode
    spec = importlib.util.spec_from_file_location('retry_context', WORK / 'tools/diagnostics/prepare_rgb_acquisition_retry.py')
    guard = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(guard)
    rospy.init_node('recorded_alignment_maintenance_retry', anonymous=True, disable_signals=True)
    begin = rospy.Time.now().to_nsec()
    expected = guard.parse_selection(prior['new_selection'])

    def current_context():
        epoch = rospy.wait_for_message('/alicia_d/control_reference_epoch', Header, timeout=4)
        if epoch.frame_id != 'sdk_reference_epoch':
            raise RuntimeError('SDK epoch required')
        context = guard.retry_source_context(rospy.get_param('/grasp_mode/selection'), epoch.stamp.to_nsec(),
            rospy.get_param('/camera'), rospy.get_param('/handeye'), rospy.get_param('/robot_description'))
        if context['selection'] != expected:
            raise RuntimeError('Selection changed; do not retry')
        for key in ('epoch_ns', 'camera', 'handeye', 'model_sha256'):
            if context[key] != event['source_context'][key]:
                raise RuntimeError('Source context changed: ' + key)
        q = rospy.wait_for_message('/alicia_d/accepted_joint_states', JointState, timeout=4)
        positions = dict(zip(q.name, q.position))
        feedback = dict(stamp_ns=str(q.header.stamp.to_nsec()), frame_id=q.header.frame_id,
            positions=[positions[name] for name in guard.JOINT_NAMES])
        guard._sample(feedback)
        age_ns = rospy.Time.now().to_nsec() - q.header.stamp.to_nsec()
        if q.header.stamp.to_nsec() < begin or not 0 <= age_ns <= 200_000_000:
            raise RuntimeError('Fresh SDK feedback required')
        if max(abs(a-b) for a, b in zip(feedback['positions'], receipt['bound_feedback']['positions'])) > guard.POSE_TOLERANCE_RAD:
            raise RuntimeError('Pose changed since actual user alignment')
        if rospy.wait_for_message('/grasp/state', GraspState, timeout=4).active:
            raise RuntimeError('Task is active')
        if rospy.get_param('/rgb_acquisition/ledger', {}) or rospy.get_param('/gui/joint_direct_mode') is not False:
            raise RuntimeError('Acquisition or manual control active')
        if rospy.get_param('/grasp/unknown_lift_after_close_enabled') is not False:
            raise RuntimeError('Close-and-hold policy required')
        return context, feedback

    context, feedback = current_context()
    image = rospy.wait_for_message('/supervisor/camera/color/image_raw', Image, timeout=4)
    if image.header.stamp.to_nsec() < begin:
        raise RuntimeError('New image required')
    raw_out = out / 'raw_support'
    raw_log = (out / 'raw_recorder.log').open('w')
    raw = subprocess.Popen(['/usr/bin/python3', str(WORK / '.runtime/record_support_gap_15s.py'),
        '--output', str(raw_out), '--seconds', '15'], stdout=raw_log, stderr=subprocess.STDOUT)
    report = dict(preview_only=False, alignment_receipt=receipt,
        alignment_receipt_file=prior['alignment_receipt_file'],
        alignment_receipt_sha256=prior['alignment_receipt_sha256'],
        previous_selection=expected, new_selection=None,
        selection_kind='explicit_maintenance_retry_from_unchanged_user_alignment',
        confirmation_continues_from_original_user_message=True,
        no_new_user_confirmation_claimed=True, original_transition=str(args.prior_selection),
        original_transition_sha256=hashlib.sha256(args.prior_selection.read_bytes()).hexdigest(),
        measured_at_utc=datetime.now(timezone.utc).isoformat(), source_context=context,
        bound_feedback=feedback, current_rgb_stamp_ns=image.header.stamp.to_nsec(),
        reason='Prior attempt did not enter motion; independent original support audits recovered. '
               'One explicit recorded retry after orchestration correction, not revival of an old target.',
        record_dir=str(record), raw_recorder_pid=raw.pid,
        motion_commands_sent=False, torque_commands_sent=False)
    path = out / 'new_target_selection.json'
    try:
        deadline = time.monotonic() + 6
        while not (raw_out / 'ready.json').is_file():
            if raw.poll() is not None or time.monotonic() > deadline:
                raise RuntimeError('Raw recorder not ready; no selection')
            time.sleep(.05)
        ready = json.loads((raw_out / 'ready.json').read_text())
        if ready['initial_generation'] != expected['generation'] or ready['initial_selection_stamp_ns'] != expected['stamp_ns']:
            raise RuntimeError('Raw recorder selection mismatch')
        rospy.wait_for_service('/grasp_mode/select', timeout=4)
        report['source_context'], report['bound_feedback'] = current_context()
        save(path, report)
        if raw.poll() is not None or time.monotonic() >= ready['deadline_monotonic'] - 8:
            raise RuntimeError('Insufficient diagnostic window remaining; no selection')
        response = rospy.ServiceProxy('/grasp_mode/select', SetGraspMode)('unknown', 'observe_return')
        report['response'] = dict(success=response.success, message=response.message)
        if not response.success:
            raise RuntimeError(response.message)
        # Select responds SWITCHING before its worker publishes the new generation.
        # Await acknowledgement of this ONE request; never issue another request.
        deadline = min(time.monotonic() + 5, ready['deadline_monotonic'] - 3)
        while True:
            selected = guard.parse_selection(rospy.get_param('/grasp_mode/selection'))
            if selected != expected:
                break
            if time.monotonic() >= deadline or raw.poll() is not None:
                raise RuntimeError('Selection acknowledgement timeout; no retry')
            time.sleep(.025)
        if (selected['generation'] != expected['generation'] + 1 or selected['stamp_ns'] <= expected['stamp_ns']
                or selected['mode'] != 'unknown' or selected['strategy'] != 'observe_return'):
            raise RuntimeError('Unexpected new selection; no retry')
        report['new_selection'] = selected
        save(path, report)
        print('RECORDED_NEW_SELECTION ' + json.dumps(selected), flush=True)
        return subprocess.call(['/usr/bin/python3', '-u', str(WORK / '.runtime/run_confirmed_unknown_grasp_20261004.py'),
            '--selection-result', str(path)], cwd=str(WORK))
    except Exception as exc:
        report['failure'] = repr(exc)
        save(path, report)
        raise
    finally:
        # This bounded diagnostic subscriber finishes itself; never signal the grasp runner.
        raw.wait()
        raw_log.close()


if __name__ == '__main__':
    raise SystemExit(main())
