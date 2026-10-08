"""Load the tested perception and observation planner; preserve all motion consumers."""
import argparse
import datetime
import hashlib
import json
from pathlib import Path
import subprocess
import time
import xmlrpc.client

import rosgraph
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Header, String
from std_srvs.srv import Trigger
from alicia_flexible_grasp_supervisor.msg import GraspState
from alicia_flexible_grasp_supervisor.srv import TriggerZero

WORK = Path(__file__).resolve().parents[1]
ARCHIVE = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log')
CONTRACT = WORK/'src/alicia_grasp_modes/src/alicia_grasp_modes/tabletop.py'
TARGETS = ('/remote_grasp6d_node', '/unknown_tabletop_perception')
PRESERVE = ('/grasp_task_node', '/alicia_d_driver_node', '/motion_gateway', '/bessica_d_hw_interface',
            '/move_group', '/supervisor_camera',
            '/grasp_mode_router', '/compliant_gripper_controller')

class BoundedTransport(xmlrpc.client.Transport):
    def make_connection(self, host):
        connection = super().make_connection(host)
        connection.timeout = 4.
        return connection

def node_proxy(uri):
    return xmlrpc.client.ServerProxy(uri, transport=BoundedTransport())

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--tested-sha256', required=True)
    args = parser.parse_args()
    out = args.output.resolve()
    if not Path('/media/zhuyupei/SanStand').is_mount() or out.parent != ARCHIVE:
        raise RuntimeError('SanStand mounted archive required; no fallback')
    if hashlib.sha256(CONTRACT.read_bytes()).hexdigest() != args.tested_sha256:
        raise RuntimeError('contract differs from tested source')
    out.mkdir(exist_ok=False)
    rospy.init_node('load_tested_support_fit', anonymous=True)
    master = rosgraph.Master(rospy.get_name())
    def pids(names):
        return {n: node_proxy(master.lookupNode(n)).getPid(rospy.get_name())[2]
                for n in names}
    def feedback():
        msg = rospy.wait_for_message('/alicia_d/accepted_joint_states', JointState, timeout=3)
        if msg.header.frame_id != 'sdk_measured' or len(msg.position) != 7:
            raise RuntimeError('measured seven-joint state required')
        return list(msg.position)
    def epoch():
        return rospy.wait_for_message('/alicia_d/control_reference_epoch', Header, timeout=3).stamp.to_nsec()
    def idle():
        if rospy.wait_for_message('/grasp/state', GraspState, timeout=3).active:
            raise RuntimeError('task active; reload refused')
        status = json.loads(rospy.wait_for_message('/grasp_mode/status', String, timeout=3).data)
        if status.get('active') is not False or status.get('start_reserved') is not False:
            raise RuntimeError('router owns an active request')
        if rospy.get_param('/grasp/unknown_lift_after_close_enabled', True) is not False:
            raise RuntimeError('explicit close-hold policy required')
        if rospy.get_param('/gui/joint_direct_mode', True):
            raise RuntimeError('manual owner active; reload deferred')
    report = dict(started_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
                  source_sha256=args.tested_sha256, robot_commands_sent=False,
                  torque_commands_sent=False, deployment_passed=False)
    faults = []
    subscriptions = []
    def record_fault(reason):
        if reason not in faults:
            faults.append(reason)
    try:
        idle()
        selection = rospy.get_param('/grasp_mode/selection')
        before = pids(TARGETS+PRESERVE)
        start_ticks = {n: (Path('/proc')/str(pid)/'stat').read_text().rsplit(')', 1)[1].split()[19]
                       for n,pid in before.items()}
        q = feedback(); reference_epoch = epoch()
        subscriptions = [
            rospy.Subscriber('/grasp/state', GraspState,
                lambda m: record_fault('task became active') if m.active else None, queue_size=10),
            rospy.Subscriber('/gui/joint_direct_mode', Bool,
                lambda m: record_fault('manual owner changed') if m.data else None, queue_size=10),
            rospy.Subscriber('/alicia_d/control_reference_epoch', Header,
                lambda m: record_fault('reference epoch changed') if m.stamp.to_nsec()!=reference_epoch else None,
                queue_size=10)]
        def context_unchanged():
            idle()
            if faults or rospy.get_param('/grasp_mode/selection') != selection or epoch()!=reference_epoch:
                raise RuntimeError('maintenance context changed: '+str(faults))
        def owner_unchanged(name):
            context_unchanged()
            uri = master.lookupNode(name)
            proxy = node_proxy(uri)
            if proxy.getPid(rospy.get_name())[2] != before[name]:
                raise RuntimeError('consumer owner changed: '+name)
            current_ticks = (Path('/proc')/str(before[name])/'stat').read_text().rsplit(')',1)[1].split()[19]
            if current_ticks != start_ticks[name] or faults:
                raise RuntimeError('consumer identity or task changed')
            return proxy
        report.update(before_pids=before, selection=selection, accepted_before=q,
                      epoch_before=reference_epoch, before_start_ticks=start_ticks)
        stopped = rospy.ServiceProxy('/grasp_6d/request_plan', TriggerZero)(False)
        if not stopped.success:
            raise RuntimeError(stopped.message)
        idle()
        for name in TARGETS:
            proxy = owner_unchanged(name)
            try:
                reply = proxy.shutdown(
                    rospy.get_name(), 'load tested measured-support fitting while idle')
                if reply[0] != 1:
                    raise RuntimeError(str(reply))
            except (ConnectionError, OSError) as exc:
                report.setdefault('shutdown_transport', {})[name] = repr(exc)
            deadline = time.monotonic()+12.
            stat = Path('/proc')/str(before[name])/'stat'
            while stat.exists() and stat.read_text().split()[2] != 'Z':
                if time.monotonic() >= deadline:
                    raise RuntimeError('old consumer did not exit: '+name)
                time.sleep(.1)
        unit = 'alicia-support-fit-'+datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dt%H%M%Sz').lower()
        command = ['systemd-run', '--user', '--collect', '--unit='+unit,
                   '--property=Type=exec', '--property=Restart=no',
                   '--property=WorkingDirectory='+str(WORK),
                   '--property=StandardOutput=append:'+str(out/'nodes.log'),
                   '--property=StandardError=append:'+str(out/'nodes.log'),
                   '--setenv=ROS_LOG_DIR='+str(out/'ros'), '/bin/bash',
                   str(WORK/'.runtime/start_support_fit_20261006.sh')]
        report.update(unit=unit, command=command)
        context_unchanged()
        subprocess.run(command, check=True, timeout=15)
        for service in ('/grasp_mode/reset_remote',):
            rospy.wait_for_service(service, timeout=25.)
            reply = rospy.ServiceProxy(service, Trigger)()
            report.setdefault('reset_ack', {})[service] = dict(success=reply.success, message=reply.message)
            if not reply.success:
                raise RuntimeError(reply.message)
        after = pids(TARGETS+PRESERVE)
        if any(after[n] != before[n] for n in PRESERVE) or any(after[n] == before[n] for n in TARGETS):
            raise RuntimeError('unexpected process replacement')
        idle()
        if faults or rospy.get_param('/grasp_mode/selection') != selection or epoch() != reference_epoch:
            raise RuntimeError('source/epoch changed during deployment')
        final_q = feedback()
        report.update(after_pids=after, accepted_after=final_q,
                      max_arm_delta_rad=max(abs(a-b) for a,b in zip(q[:6], final_q[:6])))
        if any(a != b for a,b in zip(q, final_q)):
            raise RuntimeError('measured posture changed; new plan handoff deferred')
        if hashlib.sha256(CONTRACT.read_bytes()).hexdigest() != args.tested_sha256:
            raise RuntimeError('source changed during deployment')
        context_unchanged()
        report['deployment_passed'] = True
    except Exception as exc:
        report['error'] = repr(exc)
        raise
    finally:
        if report['deployment_passed']:
            try:
                context_unchanged()
            except Exception as exc:
                report['deployment_passed'] = False
                report['error'] = repr(exc)
        report['context_faults'] = faults
        for sub in subscriptions:
            sub.unregister()
        report['completed_utc'] = datetime.datetime.now(datetime.timezone.utc).isoformat()
        (out/'result.json').write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
        print(json.dumps(report, ensure_ascii=False, indent=2), flush=True)
        if not report['deployment_passed']:
            raise RuntimeError(report.get('error', 'deployment context changed'))

if __name__ == '__main__':
    main()
