"""Reload idle perception and its offline geometry consumer after tested repair."""
import hashlib
import json
import os
from pathlib import Path
import subprocess
import time
import xmlrpc.client

import rosgraph
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header, String
from std_srvs.srv import Trigger
from alicia_flexible_grasp_supervisor.msg import GraspState
from alicia_flexible_grasp_supervisor.srv import TriggerZero

WORK = Path(__file__).resolve().parents[1]
EVIDENCE = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/user_grasp_20261008T090751Z/support_plane_repair')
TARGETS = ('/remote_grasp6d_node', '/unknown_tabletop_perception')
PRESERVE = ('/alicia_d_driver_node', '/motion_gateway', '/grasp_task_node',
            '/supervisor_camera', '/bessica_d_hw_interface',
            '/move_group', '/grasp_mode_router', '/compliant_gripper_controller')


class BoundedTransport(xmlrpc.client.Transport):
    def make_connection(self, host):
        connection = super().make_connection(host)
        connection.timeout = 4.
        return connection


def main():
    assert Path('/media/zhuyupei/SanStand').is_mount()
    report = json.loads((EVIDENCE / 'real_frame_replay.json').read_text())
    source = WORK / 'src/alicia_grasp_modes/src/alicia_grasp_modes/tabletop.py'
    expected = report['source_sha256'][str(source)]
    assert hashlib.sha256(source.read_bytes()).hexdigest() == expected
    out = EVIDENCE / 'deployment'
    out.mkdir(exist_ok=False)
    os.environ['ROS_LOG_DIR'] = str(out / 'observer_ros')
    rospy.init_node('load_support_plane_perception', anonymous=True)
    master = rosgraph.Master(rospy.get_name())

    def proxy(name):
        return xmlrpc.client.ServerProxy(master.lookupNode(name), transport=BoundedTransport())

    def identity(name):
        pid = proxy(name).getPid(rospy.get_name())[2]
        ticks = (Path('/proc') / str(pid) / 'stat').read_text().rsplit(')', 1)[1].split()[19]
        return dict(pid=pid, start_ticks=ticks)

    def idle():
        assert not rospy.wait_for_message('/grasp/state', GraspState, timeout=5).active
        status = json.loads(rospy.wait_for_message('/grasp_mode/status', String, timeout=5).data)
        assert status.get('active') is False and status.get('start_reserved') is False
        assert not status.get('pending')
        assert rospy.get_param('/gui/joint_direct_mode') is False
        assert not rospy.get_param('/rgb_acquisition/ledger', {})

    def measured():
        msg = rospy.wait_for_message('/alicia_d/accepted_joint_states', JointState, timeout=5)
        assert msg.header.frame_id == 'sdk_measured'
        assert 0 <= (rospy.Time.now() - msg.header.stamp).to_sec() < .2
        return dict(names=list(msg.name), positions=list(msg.position))

    def epoch():
        msg = rospy.wait_for_message('/alicia_d/control_reference_epoch', Header, timeout=5)
        assert msg.frame_id == 'sdk_reference_epoch'
        return msg.stamp.to_nsec()

    result = dict(deployment_passed=False, tested_source_sha256=expected,
                  motion_commands_sent=False, joint_disable_commands_sent=False,
                  temperature_queried=False, restarted_nodes=list(TARGETS))
    try:
        idle()
        selection = rospy.get_param('/grasp_mode/selection')
        before = {n: identity(n) for n in TARGETS + PRESERVE}
        q_before, epoch_before = measured(), epoch()
        result.update(before=before, selection=selection, measured_before=q_before, epoch_before=epoch_before)
        response = rospy.ServiceProxy('/grasp_6d/request_plan', TriggerZero)(False)
        assert response.success, response.message
        idle()
        assert rospy.get_param('/grasp_mode/selection') == selection
        for name in TARGETS:
            idle()
            assert rospy.get_param('/grasp_mode/selection') == selection
            assert epoch() == epoch_before
            assert identity(name) == before[name]
            reply = proxy(name).shutdown(rospy.get_name(), 'load validated support-plane provenance repair while idle')
            assert reply[0] == 1, reply
            old_stat = Path('/proc') / str(before[name]['pid']) / 'stat'
            deadline = time.monotonic() + 15.
            while old_stat.exists() and old_stat.read_text().rsplit(')', 1)[1].split()[0] != 'Z':
                assert time.monotonic() < deadline, 'Old consumer did not exit: ' + name
                time.sleep(.1)
        unit = 'alicia-support-plane-perception-20261008'
        command = ['systemd-run', '--user', '--collect', '--unit=' + unit,
                   '--property=Type=exec', '--property=Restart=no',
                   '--property=WorkingDirectory=' + str(WORK),
                   '--property=StandardOutput=append:' + str(out / 'perception.log'),
                   '--property=StandardError=append:' + str(out / 'perception.log'),
                   '--setenv=ROS_LOG_DIR=' + str(out / 'perception_ros'),
                   '/bin/bash', str(WORK / '.runtime/start_support_plane_perception_20261008.sh')]
        result.update(unit=unit, command=command)
        subprocess.run(command, check=True, timeout=15)
        deadline = time.monotonic() + 25.
        while True:
            try:
                current = {n: identity(n) for n in TARGETS}
                if all(current[n] != before[n] for n in TARGETS):
                    break
            except Exception:
                pass
            assert time.monotonic() < deadline, 'New perception not available'
            time.sleep(.2)
        rospy.wait_for_service('/grasp_mode/reset_remote', timeout=25.)
        reset = rospy.ServiceProxy('/grasp_mode/reset_remote', Trigger)()
        assert reset.success, reset.message
        result['remote_reset'] = dict(success=reset.success, message=reset.message)
        status = json.loads(rospy.wait_for_message('/perception/unknown/detector_status', String, timeout=10).data)
        after = {n: identity(n) for n in TARGETS + PRESERVE}
        assert all(after[n] == before[n] for n in PRESERVE)
        assert all(after[n] != before[n] for n in TARGETS)
        idle()
        assert rospy.get_param('/grasp_mode/selection') == selection
        assert epoch() == epoch_before
        q_after = measured()
        assert q_before == q_after, 'Measured posture changed during deployment'
        assert hashlib.sha256(source.read_bytes()).hexdigest() == expected
        result.update(deployment_passed=True, after=after, measured_after=q_after,
                      first_status=dict(state=status.get('state'), detail=status.get('detail')))
    except Exception as exc:
        result['error'] = repr(exc)
        raise
    finally:
        (out / 'result.json').write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n')
        print(json.dumps(result, ensure_ascii=False, indent=2), flush=True)


if __name__ == '__main__':
    main()
