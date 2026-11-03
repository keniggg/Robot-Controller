"""Run the existing grasp runner after recorder/import readiness, then select once."""
import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import sys
import threading
import time


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run', type=Path, required=True)
    p.add_argument('--evidence', type=Path, required=True)
    a = p.parse_args()
    w = Path(__file__).resolve().parents[1]
    record = Path(os.environ['GRASP_RECORD_DIR']).resolve()
    mount = Path('/media/zhuyupei/SanStand')
    assert mount.is_mount() and all(mount in x.resolve().parents for x in (record, a.run, a.evidence))
    assert (record / 'recorder_ready.json').is_file()
    os.environ['ROS_LOG_DIR'] = str(record / 'runner_ros')

    def sha(path):
        return hashlib.sha256(Path(path).read_bytes()).hexdigest()

    def module(name, relative, digest):
        path = w / relative
        assert sha(path) == digest, relative + ' changed'
        spec = importlib.util.spec_from_file_location(name, path)
        value = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(value)
        return value

    source_sha = {
        'src/alicia_grasp_modes/src/alicia_grasp_modes/station_compute_wait_task.py': '79d50191c5cd2866f08526f0a3a86924dbea82b77fae254221a98afa36ad4fbe',
        'src/alicia_flexible_grasp_supervisor/scripts/motion_gateway_node.py': '79a9683766b6b75d6676b05c37bea84f9ea06b4b10251b4785dd434087a16250',
        'src/alicia_grasp_modes/src/alicia_grasp_modes/reachable_observation_views.py': 'b4ddc6c77b305ef355cd7c275853c498c411b145e57b503ee3f34228eaadb4dd',
        'src/alicia_grasp_modes/scripts/mode_aware_grasp_runner.py': '192b2195de054dd0aa7af42a4d985c813b84f081bede639235e5ca58b19fa011',
        'src/alicia_grasp_modes/scripts/unknown_tabletop_perception.py': 'c50450969bedfd4e11e3df2da86c867592f8724f99b568f66078b9fe9a957cce',
        'src/alicia_grasp_modes/src/alicia_grasp_modes/source_connected_owner.py': 'a458fa22b4a1a56c397d94ce7a6e48c060e651e7f88979369d093f65f50a074f',
        'src/alicia_grasp_modes/src/alicia_grasp_modes/station_compute_wait_remote.py': 'b06b741d47da07fbd275b6cf961c65e35f77c0605212087b533f6b3843c10c8d',
    }
    source_sha.update({'src/alicia_grasp_modes/src/alicia_grasp_modes/reachable_observation_views.py': 'b4ddc6c77b305ef355cd7c275853c498c411b145e57b503ee3f34228eaadb4dd', 'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot/observation_path_guard.py': '462316a22d104fb94a4a8431890726fa677f893486d9a1b34034da7e12553238', 'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot/observation_tracking_contract.py': 'e3c5a070559cfa7ab2cd0383b82a080b007c6703c9eab4bcfe7bc9deb1e2ccd5', 'src/alicia_flexible_grasp_supervisor/scripts/remote_grasp6d_node.py': '32b53d8b78dc96a7df6e06f9f9327df99d79f0053560633f287be54e0de582ed'})
    source_sha.update({'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot/pregrasp_feedforward.py': '920c637fd3a2001f822a4f83f512c8b7691f79f43ea215dcc17803ac647ba5bf', 'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot/observation_endpoint_compensation.py': '1b9de0f9704a5ae6fd38c8e210cce781a9228205518edd0e9175c1b1e68615cf', 'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot/observation_compensation_gateway.py': 'b9a436eef39fa738345ffac79461f08debff042f700c95c7433c81fa619cd66f'})
    source_sha.update({'src/alicia_grasp_modes/src/alicia_grasp_modes/reachable_observation_views.py': 'b28607c4609d6bcb1be47e43d0e6823c994314677365d7e21608dd214a5e67e7', 'src/alicia_grasp_modes/scripts/mode_aware_remote_grasp6d.py': 'b8f0285a6f2de1b8ca685038300d663d74b6173b9d1023a2fa2ff8951bfc7808', 'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot/observation_arrival_reserve.py': '89e4e65ec6cba5ca539cc10ef4f2cbb3e053a0b1c3fb21ac5633f5bfd29d886e'})
    assert all(sha(w / f) == digest for f, digest in source_sha.items())
    idle = module('idle_primitive', '.runtime/restore_gen62_after_handoff_20261022.py', 'f1d5ad558dfd40cf5f7e59820b6ab84b55b40933236014debe41229c155ab61d')
    health = module('health_primitive', '.runtime/verify_active_remote_readiness_20261020.py', 'eaa51eaf5b636b74d9957798bbf3b0407a0d25d5c6b11097c02b22a5e97948e5')
    selection = module('selection_primitive', '.runtime/run_reboot_realigned_grasp_20261010.py', 'cb7996419e32954b683fc4b148f54c9e467f2a8b6f259664fa2a5bb1d0d5ed11')
    warm = module('prewarm_primitive', '.runtime/prewarm_mode_grasp_runner_20261012.py', '3e5a0798bccaf49ea15cb8eac977d2621cfe48a78e8ae141bf324f4154e2fa49')

    import rospy
    import rosgraph
    from std_msgs.msg import Bool, Header, String
    from sensor_msgs.msg import JointState
    from actionlib_msgs.msg import GoalStatusArray
    from alicia_flexible_grasp_supervisor.msg import GraspState
    from alicia_grasp_modes.srv import SetGraspMode
    from alicia_grasp_modes.selection import parse_selection

    rospy.init_node('current_recorded_selection', anonymous=True, disable_rosout=True)
    master = rosgraph.Master(rospy.get_name())
    lock, states, samples = threading.RLock(), {}, []
    topics = [('joint', '/alicia_d/accepted_joint_states', JointState),
        ('epoch', '/alicia_d/control_reference_epoch', Header), ('manual', '/gui/joint_direct_mode', Bool),
        ('task', '/grasp/state', GraspState), ('mode', '/grasp_mode/status', String),
        ('arm', '/alicia_controller/follow_joint_trajectory/status', GoalStatusArray),
        ('hand', '/hand_controller/follow_joint_trajectory/status', GoalStatusArray),
        ('actuation', '/alicia_d/actuation_status', String), ('ready', '/alicia_d/feedback_ready', Bool),
        ('enabled', '/alicia_d/motion_enabled', Bool)]

    def receive(key, message):
        if message._connection_header.get('callerid') != idle.OWNERS[key]:
            return
        with lock:
            now = rospy.Time.now().to_nsec()
            states[key] = (message, now)
            if key == 'joint':
                samples.append(dict(stamp_ns=message.header.stamp.to_nsec(), received_ns=now,
                    names=list(message.name), positions=list(message.position), frame_id=message.header.frame_id,
                    caller=message._connection_header['callerid']))
                samples[:] = samples[-40:]

    subs = [rospy.Subscriber(topic, cls, lambda msg, key=key: receive(key, msg), queue_size=40)
            for key, topic, cls in topics]
    instruction = json.loads((a.evidence / 'user_instruction.json').read_text())
    assert instruction['new_scene_confirmation_required'] is False
    epoch_reference = None

    def snapshot():
        nonlocal epoch_reference
        end = time.monotonic() + 5
        while (len(samples) < 20 or len(states) < len(topics)) and time.monotonic() < end:
            time.sleep(.02)
        for key, topic, cls in topics:
            if key not in ('joint', 'arm', 'hand'):
                receive(key, rospy.wait_for_message(topic, cls, timeout=3))
        with lock:
            if epoch_reference is None:
                epoch_reference = states['epoch'][0].stamp.to_nsec()
            evidence = idle.validate_idle_snapshot(list(samples[-20:]), dict(states),
                rospy.Time.now().to_nsec(), epoch_reference)
        publishers = dict(master.getSystemState()[0])
        assert all(publishers.get(topic) == [idle.OWNERS[key]] for key, topic, cls in topics)
        return evidence

    def set_manual(value):
        # A verified already-current GUI latch requires no second transition.
        current = rospy.wait_for_message('/gui/joint_direct_mode', Bool, timeout=3)
        if (current._connection_header.get('callerid') == idle.GUI and current.data is value
                and rospy.get_param('/gui/joint_direct_mode') is value):
            return dict(value=value, already_current=True, observed_ns=time.time_ns())
        pub = rospy.Publisher('/gui/joint_direct_mode', Bool, queue_size=1, latch=True)
        try:
            end = time.monotonic() + 3
            while not pub.get_num_connections() and time.monotonic() < end:
                time.sleep(.02)
            assert pub.get_num_connections() > 0
            requested = rospy.Time.now().to_nsec()
            rospy.set_param('/gui/joint_direct_mode', value)
            pub.publish(Bool(data=value))
            end = time.monotonic() + 5
            while time.monotonic() < end:
                row = states.get('manual')
                if row and row[1] >= requested and row[0].data is value:
                    assert rospy.get_param('/gui/joint_direct_mode') is value
                    return dict(value=value, requested_ns=requested, gui_ack_ns=row[1])
                time.sleep(.02)
            raise RuntimeError('GUI-own mode acknowledgement missing')
        finally:
            pub.unregister()

    report = dict(started_ns=time.time_ns(), source_sha256=source_sha,
        user_instruction=str(a.evidence / 'user_instruction.json'),
        scene_review_skipped_by_user=True, record_dir=str(record), supplemental_observation_allowed=True,
        physical_grasp_verified=False, temperature_queried=False, driver_restarted=False)

    def save():
        (record / 'authorization.json').write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')

    automatic = False
    code = None
    try:
        with warm.PrewarmedModeRunner(w / 'src/alicia_grasp_modes/scripts/mode_aware_grasp_runner.py') as prepared:
            report['prewarm'] = prepared.ready
            report['prewarm_ready_ns'] = time.time_ns()
            binding = health.load_binding(a.run / 'high_accuracy_grasp_r1/arrival_margin_repair_r1/view_arrival_reserve_r1/online_reload_r1/remote_startup_binding.json',
                '8a67ffa519c525a7a21d2b2bf196d418ff8e950665e19d92176d8dbafd206704')
            report['remote_readiness'] = health.evaluate(binding, record)
            assert report['remote_readiness']['passed']
            report['before'] = snapshot()
            assert states['manual'][0].data is True and rospy.get_param('/gui/joint_direct_mode') is True
            assert rospy.get_param('/camera/mode_depth_preset')['unknown_preset'] == 'high_accuracy'
            assert rospy.get_param('/grasp/unknown_lift_after_close_enabled') is False
            services = dict(master.getSystemState()[2])
            assert services.get('/grasp_mode/select') == ['/grasp_mode_router']
            assert services.get('/grasp/start') == ['/grasp_mode_router']
            prior = parse_selection(rospy.get_param('/grasp_mode/selection'))
            automatic = True
            report['automatic_ack'] = set_manual(False)
            assert all(sha(w / f) == digest for f, digest in source_sha.items())
            report['selection_requested_ns'] = time.time_ns()
            response = rospy.ServiceProxy('/grasp_mode/select', SetGraspMode)('unknown', 'observe_return')
            assert response.success, response.message
            selected, ack = selection.wait_for_selection_commit(rospy, prior)
            report.update(selection=selected, selection_ack=ack, selection_committed_ns=time.time_ns())
            save()
            print('CURRENT_RECORDED_SELECTION ' + json.dumps(selected), flush=True)
            code = prepared.run_once(selected)
            report.update(runner_returncode=code, runner_finished_ns=time.time_ns())
    except BaseException as exc:
        report['launcher_error'] = repr(exc)
        raise
    finally:
        try:
            # An ambiguous service response remains with the task owner.
            text = (record / 'runner.log').read_text(errors='replace')
            if 'execution_response_unknown=True' in text:
                report['manual_restore_held'] = 'task owner must resolve ambiguous execution response'
            else:
                report['after'] = snapshot()
                if automatic:
                    report['manual_ack'] = set_manual(True)
                report['final_idle'] = snapshot()
                report['manual_after'] = rospy.get_param('/gui/joint_direct_mode')
        except Exception as exc:
            report['restore_error'] = repr(exc)
        report['finished_ns'] = time.time_ns()
        save()
        for sub in subs:
            sub.unregister()
        rospy.signal_shutdown('current recorded attempt ended')
    return code


if __name__ == '__main__':
    sys.exit(main())
