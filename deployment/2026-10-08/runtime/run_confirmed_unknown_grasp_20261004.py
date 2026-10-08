"""Run the existing hybrid grasp runner under the current real user confirmation."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import threading
import time

WORK = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(WORK/'tools/diagnostics'))
from confirmed_unknown_dispatch import (
    EntryDecision, RGB_WAIT_SEC, RGB_MAX_PLANS, context_parameters,
    acquisition_result, dispatch_attempt)

def main():
    import rospy
    from std_msgs.msg import Bool, Header, String
    from alicia_grasp_modes.selection import parse_selection
    parser=argparse.ArgumentParser()
    parser.add_argument('--selection-result',required=True,type=Path)
    args=parser.parse_args()
    transition=json.loads(args.selection_result.read_text())
    if (transition.get('preview_only') is not False or transition.get('failure')
            or transition.get('response',{}).get('success') is not True):
        raise RuntimeError('successful current selection transition required')
    receipt=transition['alignment_receipt']
    if (receipt.get('kind')!='explicit_user_target_alignment'
            or receipt.get('target_aligned') is not True
            or receipt.get('confirmation_source',{}).get('kind')!='user_message'):
        raise RuntimeError('actual user confirmation receipt required')
    selected=parse_selection(transition['new_selection'])
    if selected['mode']!='unknown' or selected['strategy']!='observe_return':
        raise RuntimeError('current hybrid unknown grasp selection required')
    mount=Path('/media/zhuyupei/SanStand')
    out=Path(os.environ['GRASP_RECORD_DIR']).resolve()
    if not mount.is_mount() or mount.resolve() not in out.parents or not out.is_dir():
        raise RuntimeError('active SanStand recording required; no fallback')
    rospy.init_node('confirmed_unknown_grasp_handoff',anonymous=True)
    condition = threading.Condition()
    decision = EntryDecision(selected, rospy.get_param('/unknown_perception/max_frame_age_sec', .8))
    def status_cb(message):
        try:
            value = json.loads(message.data)
        except (ValueError, TypeError):
            return
        with condition:
            decision.observe(value)
            condition.notify_all()
    def epoch_cb(message):
        if message.frame_id == 'sdk_reference_epoch' and message.stamp.to_nsec() > 0:
            with condition:
                decision.observe_epoch(message.stamp.to_nsec())
                condition.notify_all()
    # Start listening before file I/O and ownership preparation. A transient
    # ready/lost or epoch change during preparation must not be missed.
    subscriptions = [
        rospy.Subscriber('/perception/unknown/detector_status', String, status_cb, queue_size=8),
        rospy.Subscriber('/alicia_d/control_reference_epoch', Header, epoch_cb, queue_size=2)]
    if parse_selection(rospy.get_param('/grasp_mode/selection'))!=selected:
        raise RuntimeError('selection changed before runner handoff')
    record=dict(confirmation_id=receipt['confirmation_id'],
        confirmation_source=receipt['confirmation_source'],
        selection=selected,selection_result=str(args.selection_result),
        selection_result_sha256=hashlib.sha256(args.selection_result.read_bytes()).hexdigest(),
        operator_authorized_autonomous_grasp=True,
        strategy='current measured target first; bounded RGB acquisition only before target lock',
        budget_policy='RGB acquisition and observe_return retain separate original budgets',
        mass_policy='operator-declared <=20g upper bound; not a measured target mass',
        lift_policy=dict(
            physical_lift_after_close=rospy.get_param('/grasp/unknown_lift_after_close_enabled', True),
            simulation_lift_gate=rospy.get_param('/mujoco_digital_twin/unknown_lift_gate_enabled', True)),
        physical_success='unverified',torque_disable_authorized=False,
        old_plan_reused=False,recording_scope='local_only')
    (out/'authorization.json').write_text(json.dumps(record,ensure_ascii=False,indent=2)+'\n')
    # The actual user has finished manual adjustment. This changes command
    # ownership to the existing automatic pipeline; it never disables joints.
    publisher=rospy.Publisher('/gui/joint_direct_mode',Bool,queue_size=1,latch=True)
    rospy.sleep(.2)
    if parse_selection(rospy.get_param('/grasp_mode/selection'))!=selected:
        raise RuntimeError('selection changed during runner preparation')
    rospy.set_param('/gui/joint_direct_mode',False)
    publisher.publish(Bool(data=False))
    print('CURRENT_CONFIRMED_UNKNOWN_RUN '+json.dumps(record,ensure_ascii=False),flush=True)
    events = []
    def save_event(event):
        if not mount.is_mount():
            raise RuntimeError('SanStand unavailable during dispatch; no fallback')
        events.append(dict(event, observed_stamp_ns=rospy.Time.now().to_nsec()))
        (out/'dispatch.json').write_text(json.dumps(dict(selection=selected, events=events),
            ensure_ascii=False, indent=2)+'\n')
        print('CONFIRMED_DISPATCH '+json.dumps(events[-1], ensure_ascii=False), flush=True)
    def run_rgb(expected):
        log = out/'rgb_acquisition_dispatch.log'
        command = ['/usr/bin/python3', str(WORK/'tools/diagnostics/run_rgb_acquisition.py'),
            '--wait-sec', str(RGB_WAIT_SEC), '--max-plans', str(RGB_MAX_PLANS),
            '--expected-selection-json', json.dumps(expected, sort_keys=True)]
        # The gateway owns a potentially executing RPC until its real response.
        # No subprocess timeout, kill, or parallel grasp entry is permitted.
        with log.open('w') as stream:
            code = subprocess.call(command, cwd=str(WORK), stdout=stream, stderr=subprocess.STDOUT)
        report, artifact = acquisition_result(log, mount/'实验记录/Robot-Controller/ros_log')
        artifact['terminal_log'] = str(log)
        return code, report, artifact
    def run_metric(expected):
        return subprocess.call(['rosrun','alicia_grasp_modes','mode_aware_grasp_runner.py',
            '--mode','unknown','--strategy','observe_return',
            '--expected-selection-json', json.dumps(expected, sort_keys=True)], cwd=str(WORK))
    try:
        return dispatch_attempt(decision, condition=condition,
            selection=lambda: rospy.get_param('/grasp_mode/selection'),
            now_ns=lambda: rospy.Time.now().to_nsec(), monotonic=time.monotonic,
            parameters=lambda: context_parameters(rospy.get_param_cached),
            run_rgb=run_rgb, run_metric=run_metric, record=save_event,
            is_shutdown=rospy.is_shutdown)
    except Exception as exc:
        save_event(dict(phase='failed', reason=str(exc), automatic_new_selection=False))
        return 2
    finally:
        for subscription in subscriptions:
            subscription.unregister()

if __name__=='__main__':
    raise SystemExit(main())
