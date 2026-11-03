"""Reinitialize measurement after this recorded camera restart; never start motion.

The real user confirmation is retained verbatim, not reissued. This path only
applies to the unexecuted selection that was invalidated by this camera outage.
"""
import argparse
from copy import deepcopy
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import time

W = Path(__file__).resolve().parents[1]
RUN = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_ros_resume_20261006T024224Z')
MOUNT = Path('/media/zhuyupei/SanStand')
LOAD = RUN.parent/'regional_production_20261006T030007.949808Z/result.json'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def require(value, reason):
    if not value:
        raise RuntimeError(reason)


def module(name, relative):
    spec = importlib.util.spec_from_file_location(name, W/relative)
    value = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(value)
    return value


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--validation-result', required=True, type=Path)
    args = parser.parse_args()
    require(MOUNT.is_mount(), 'SanStand unavailable')
    validation = json.loads(args.validation_result.read_text())
    require(validation.get('validation_passed') is True, 'post-load live validation has not passed')
    require(validation.get('load_result_sha256') == sha(LOAD), 'validation refers to another load')
    load = json.loads(LOAD.read_text())
    transition = json.loads((RUN/'new_target_selection.json').read_text())
    receipt = transition['alignment_receipt']
    require(sha(transition['alignment_receipt_file']) == transition['alignment_receipt_sha256'], 'alignment receipt changed')
    require(sha(receipt['event_file']) == receipt['event_file_sha256'], 'alignment event changed')
    event = json.loads(Path(receipt['event_file']).read_text())
    require(receipt['confirmation_source'] == event['confirmation_source'], 'actual confirmation changed')
    os.environ['ROS_LOG_DIR'] = str(RUN/'ros')
    import rospy
    from std_msgs.msg import String
    from alicia_grasp_modes.srv import SetGraspMode
    from alicia_grasp_modes.selection import parse_selection
    rospy.init_node('refresh_measurement_after_camera_maintenance', anonymous=True, disable_signals=True)
    h = module('maintenance_camera_owner', '.runtime/gen4_root_cause/reload_camera_visual_gate.py')
    ab = module('maintenance_feedback', '.runtime/camera_preset_ab_observer.py')
    alignment = module('maintenance_alignment', 'tools/diagnostics/prepare_rgb_acquisition_retry.py')
    selected = parse_selection(transition['new_selection'])
    require(selected == parse_selection(load['current_selection']), 'camera load selection mismatch')
    ev = ab.RosEvidence(rospy)
    report = dict(kind='camera_maintenance_measurement_reinitialization', previous_selection=selected,
        load_result=str(LOAD), load_result_sha256=sha(LOAD), live_validation=str(args.validation_result),
        live_validation_sha256=sha(args.validation_result), alignment_receipt=receipt,
        alignment_receipt_sha256=transition['alignment_receipt_sha256'],
        original_confirmation_reissued=False, motion_commands_sent=False, torque_commands_sent=False,
        new_motion_authority_created=False, old_plan_reused=False, new_selection=None)
    output = RUN/'post_camera_measurement_selection.json'
    require(not output.exists(), 'maintenance selection already attempted')
    baseline = event['source_context']
    def guard(expect):
        require(MOUNT.is_mount(), 'SanStand unavailable')
        require(ev.guard.check(ev.now_ns(), freeze=True), 'real stationary feedback unavailable')
        snap = ev.guard.snapshot()
        require(snap['epoch'] == int(receipt['epoch_ns']), 'control epoch changed')
        verified_feedback = validation['final_feedback']
        require(snap['epoch'] == verified_feedback['epoch']
            and snap['accepted']['q'] == verified_feedback['accepted']['q']
            and snap['sdk']['q'] == verified_feedback['sdk']['q']
            and snap['accepted']['source_stamp_ns'] >= verified_feedback['accepted']['source_stamp_ns']
            and snap['sdk']['source_stamp_ns'] >= verified_feedback['sdk']['source_stamp_ns'],
            'feedback differs from the validated new window')
        monitor = alignment.AlignmentPoseMonitor(receipt)
        monitor.observe(snap['accepted']['source_stamp_ns'], snap['accepted']['q'][:6])
        monitor.require_stationary()
        require(parse_selection(rospy.get_param('/grasp_mode/selection')) == expect, 'selection changed')
        require(not rospy.get_param('/rgb_acquisition/ledger', {}), 'observation episode already exists')
        require(rospy.get_param('/handeye') == baseline['handeye'], 'handeye changed')
        require(hashlib.sha256(rospy.get_param('/robot_description').encode()).hexdigest() == baseline['model_sha256'], 'model changed')
        require(h.alive(load['new_camera']) and h.node_pid(rospy, '/supervisor_camera') == load['new_camera']['pid'], 'camera owner changed')
        require(h.stable_camera_configuration(rospy.get_param('/camera')) ==
            h.stable_camera_configuration(load['desired_camera_configuration']), 'loaded camera policy changed')
        for name, owner in load['core_preserved'].items():
            require(h.alive(owner) and h.node_pid(rospy, name) == owner['pid'], 'core owner changed: '+name)
        status = json.loads(rospy.wait_for_message('/grasp_mode/status', String, timeout=2).data)
        require(status.get('active') is False and status.get('start_reserved') is False, 'task reserved or active')
        return snap
    try:
        h.wait(lambda: ev.guard.check(ev.now_ns()), 4, 'feedback')
        report['feedback_before'] = guard(selected)
        status = json.loads(rospy.wait_for_message('/perception/unknown/detector_status', String, timeout=2).data)
        loss = status.get('first_loss') or {}
        events = load['events']
        prepared = next(e['observed_ns'] for e in events if e['event'] == 'prepared')
        exited = next(e['observed_ns'] for e in events if e['event'] == 'old_camera_exited')
        require(status.get('generation') == selected['generation'] and status.get('target_lost') is True
            and loss.get('reason') == 'exact_rgbd_stream_stale'
            and prepared <= loss.get('observed_stamp_ns', 0) <= exited, 'loss is not this recorded camera outage')
        report['retired_target_status'] = status
        rospy.wait_for_service('/grasp_mode/select', timeout=3)
        guard(selected)
        report['request_stamp_ns'] = rospy.Time.now().to_nsec()
        response = rospy.ServiceProxy('/grasp_mode/select', SetGraspMode)('unknown', 'observe_return')
        report['response'] = dict(success=response.success, message=response.message)
        require(response.success, response.message)
        def acknowledged():
            value = parse_selection(rospy.get_param('/grasp_mode/selection'))
            if value == selected:
                return False
            require(value['generation'] == selected['generation']+1 and value['stamp_ns'] > report['request_stamp_ns']
                and value['mode'] == selected['mode'] and value['strategy'] == selected['strategy'], 'unexpected new selection')
            return value
        report['new_selection'] = h.wait(acknowledged, 5, 'maintenance selection acknowledgement')
        report['feedback_after'] = guard(report['new_selection'])
    except BaseException as exc:
        report['error'] = repr(exc)
        raise
    finally:
        ev.close()
        require(MOUNT.is_mount(), 'SanStand unavailable')
        output.write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
        print(json.dumps(dict(path=str(output), new_selection=report['new_selection'], error=report.get('error')), ensure_ascii=False), flush=True)


if __name__ == '__main__':
    main()
