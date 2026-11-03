"""Load a tested perception repair, retire gen5, preserve the current camera/core.

Only the unknown perception producer and selection are changed. Original user
alignment is retained as provenance, never reissued as a new confirmation.
"""
import argparse
from copy import deepcopy
from datetime import datetime, timezone
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import signal
import subprocess
import threading
import time

W = Path(__file__).resolve().parents[1]
M = Path('/media/zhuyupei/SanStand')
R = M/'实验记录/Robot-Controller/ros_log/full_ros_resume_20261005T012259Z'
PREVIOUS = M/'实验记录/Robot-Controller/ros_log/saved_baseline_trial_20261005T033848.688318Z/continuation_20261005T034614Z/result.json'
MANIFEST = W/'.runtime/support_gap_validation_20261005.json'


def load(name, relative):
    spec = importlib.util.spec_from_file_location(name, W/relative)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--execute', action='store_true')
    if not parser.parse_args().execute:
        raise RuntimeError('explicit --execute required')
    import rospy
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Header, String
    from alicia_flexible_grasp_supervisor.msg import GraspState
    from alicia_grasp_modes.srv import SetGraspMode
    h = load('support_process', '.runtime/gen4_root_cause/reload_camera_visual_gate.py')
    p = load('support_process_env', '.runtime/load_pair_rectification_20261004.py')
    g = load('support_alignment', 'tools/diagnostics/prepare_rgb_acquisition_retry.py')
    assert M.is_mount(), 'SanStand unavailable; no fallback'
    manifest = json.loads(MANIFEST.read_text())
    assert manifest['tests_passed'] and manifest['reviewed']
    frozen = dict(manifest['files'])
    def verify_files():
        for path, sha in frozen.items():
            assert digest(W/path) == sha, path
    verify_files()
    previous = json.loads(PREVIOUS.read_text())
    assert previous['trial_loaded'] and previous['visual_nodes_restored']
    selection = g.parse_selection(previous['selection'])
    assert selection['generation'] == 5
    alignment_path = R/'alignment_receipt_20261005T012931Z.json'
    receipt = json.loads(alignment_path.read_text())
    assert receipt['confirmation_id'] == 'call_miGeToTAr9eAhD7cnx0dgXKW:0'
    assert digest(receipt['event_file']) == receipt['event_file_sha256']
    prior_trial = json.loads(Path(previous['prior_failed_trial']).read_text())
    expected_sdk = prior_trial['sdk_hold']
    out = R/('support_gap_load_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ'))
    out.mkdir(); (out/'ros').mkdir()
    os.environ['ROS_LOG_DIR'] = str(out/'ros')
    rospy.init_node('load_support_gap_perception_only', anonymous=True, disable_signals=True)
    monitor = g.AlignmentPoseMonitor(receipt)
    latest, faults, lock = {}, [], threading.RLock()
    def feedback(msg):
        with lock:
            try:
                assert msg.header.frame_id == 'sdk_measured'
                values = dict(zip(msg.name, msg.position))
                monitor.observe(msg.header.stamp.to_nsec(), [values[n] for n in g.JOINT_NAMES])
                latest['accepted'] = msg.header.stamp.to_nsec()
            except Exception as exc:
                faults.append(repr(exc))
    def remember(key, value):
        with lock:
            latest[key] = value
            if key == 'epoch' and value != int(receipt['epoch_ns']): faults.append('epoch_changed')
            if key == 'active' and value: faults.append('task_became_active')
            if key == 'sdk' and value[0] != expected_sdk: faults.append('sdk_hold_changed')
    subs = [rospy.Subscriber('/alicia_d/accepted_joint_states', JointState, feedback, queue_size=100),
        rospy.Subscriber('/alicia_d/control_reference_epoch', Header,
            lambda msg: remember('epoch', msg.stamp.to_nsec() if msg.frame_id == 'sdk_reference_epoch' else 0)),
        rospy.Subscriber('/grasp/state', GraspState, lambda msg: remember('active', msg.active)),
        rospy.Subscriber('/alicia_d/sdk_command', JointState,
            lambda msg: remember('sdk', (dict(zip(msg.name, msg.position)), time.monotonic()))),
        rospy.Subscriber('/grasp_mode/status', String, lambda msg: remember('mode', json.loads(msg.data)))]
    report = dict(preview_only=False, loaded=False, failure=None, response={},
        alignment_receipt=receipt, alignment_receipt_file=str(alignment_path),
        alignment_receipt_sha256=digest(alignment_path), previous_selection=deepcopy(selection),
        new_selection=None, previous_runtime_receipt=str(PREVIOUS), previous_sha256=digest(PREVIOUS),
        maintenance_retirement=True, confirmation_reissued=False,
        motion_commands_sent=False, torque_commands_sent=False, temperature_queries_sent=False,
        manifest_sha256=digest(MANIFEST), events=[])
    def save(event):
        assert M.is_mount(), 'SanStand unavailable; no fallback'
        report['events'].append(dict(event=event, observed_ns=rospy.Time.now().to_nsec()))
        (out/'result.json').write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
    try:
        h.wait(lambda: all(k in latest for k in ('accepted','epoch','active','sdk','mode')), 5, 'live state')
        core = dict(previous['core_preserved'])
        core.update({'/supervisor_camera': previous['owners']['camera'],
                     '/remote_grasp6d_node': previous['owners']['remote']})
        old = previous['owners']['unknown']
        assert h.alive(old) and h.node_pid(rospy, '/unknown_tabletop_perception') == old['pid']
        p.validate_relay('unknown')
        allowed = tuple(k for k in h.ENV_KEYS+p.EXTRA_ENV if k not in ('ROS_LOG_DIR','PYTHONUNBUFFERED'))
        environment = p.read_environment(old['pid'], allowed)
        context = g.retry_source_context(selection, latest['epoch'], rospy.get_param('/camera'),
            rospy.get_param('/handeye'), rospy.get_param('/robot_description'))
        assert rospy.get_param('/camera/mode_depth_preset/unknown_preset') == 'saved_baseline'
        def guard():
            assert M.is_mount()
            with lock:
                assert not faults, faults
                monitor.require_stationary()
                g.require_fresh_source(latest['accepted'], rospy.Time.now().to_nsec())
                assert latest['active'] is False and time.monotonic()-latest['sdk'][1] < 1.
                assert latest['mode'].get('active') is False and latest['mode'].get('start_reserved') is False
            assert g.parse_selection(rospy.get_param('/grasp_mode/selection')) == selection
            assert not rospy.get_param('/rgb_acquisition/ledger', {})
            assert rospy.get_param('/camera/mode_depth_preset/unknown_preset') == 'saved_baseline'
            preset = rospy.get_param('/camera/runtime_profile/mode_depth_preset')
            expected_preset = previous['camera_after']['runtime_profile']['mode_depth_preset']
            assert preset == expected_preset, 'camera preset changed'
            current = g.retry_source_context(context['selection'], latest['epoch'],
                rospy.get_param('/camera'), rospy.get_param('/handeye'), rospy.get_param('/robot_description'))
            assert g.digest(current) == g.digest(context), 'source_context_changed'
            for node, identity in core.items():
                assert h.alive(identity) and h.node_pid(rospy, node) == identity['pid'], node
        guard(); verify_files()
        report.update(preserved_processes=core, old_producer=old, source_context=context)
        save('prepared')
        assert h.alive(old) and h.node_pid(rospy, '/unknown_tabletop_perception') == old['pid']
        os.kill(old['pid'], signal.SIGINT)
        h.wait(lambda: not h.alive(old), 15, 'old producer exit', guard)
        save('old_producer_exited')
        guard()
        response = rospy.ServiceProxy('/grasp_mode/select', SetGraspMode)('unknown', 'observe_return')
        report['response'] = dict(success=response.success, message=response.message)
        assert response.success, response.message
        retired = deepcopy(selection)
        def transition():
            value = g.parse_selection(rospy.get_param('/grasp_mode/selection'))
            if value == retired: return False
            assert value['generation'] == retired['generation']+1
            assert value['stamp_ns'] > retired['stamp_ns']
            assert value['mode'] == 'unknown' and value['strategy'] == 'observe_return'
            return value
        selection = h.wait(transition, 8, 'new selection')
        report['new_selection'] = selection
        def acknowledged():
            with lock: value = deepcopy(latest['mode'])
            return (all(value.get(k) == v for k,v in selection.items())
                and value.get('state') == 'waiting_for_target'
                and value.get('active') is False and value.get('start_reserved') is False)
        h.wait(acknowledged, 10, 'reset acknowledgements', guard)
        save('retired_before_restart'); guard(); verify_files()
        unit = 'alicia-support-gap-perception-'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ').lower()
        command = p.service_argv('unknown', old, environment, out, unit, allowed)
        report.update(unit=unit, command=command); save('start_requested')
        subprocess.run(command, check=True, timeout=15, capture_output=True, text=True)
        def registered():
            unit_pid = int(subprocess.check_output(['systemctl','--user','show',unit,
                '--property=MainPID','--value'], text=True, timeout=3).strip() or '0')
            if not unit_pid: return False
            try: pid = h.node_pid(rospy, '/unknown_tabletop_perception')
            except (h.CameraRegistrationPending, ConnectionRefusedError): return False
            if pid == old['pid']: return False
            assert pid == unit_pid, 'foreign producer registration'
            identity = h.identity(pid)
            assert identity['argv'] == command[command.index('/usr/bin/python3'):]
            assert identity['exe_sha256'] == old['exe_sha256'] and identity['cwd'] == old['cwd']
            return identity
        report['new_producer'] = h.wait(registered, 20, 'new producer registration', guard)
        report['new_source_context'] = g.retry_source_context(selection, latest['epoch'],
            rospy.get_param('/camera'), rospy.get_param('/handeye'), rospy.get_param('/robot_description'))
        guard(); verify_files(); report['loaded'] = True; save('loaded')
    except BaseException as exc:
        report['failure'] = repr(exc); save('failed'); raise
    finally:
        for sub in subs: sub.unregister()
        print(json.dumps(dict(result=str(out/'result.json'), loaded=report['loaded'],
            new_selection=report['new_selection'], failure=report['failure']),ensure_ascii=False),flush=True)


if __name__ == '__main__':
    main()
