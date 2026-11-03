"""Reload three reviewed compute consumers, preserving the eight hardware/GUI nodes."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import time
import xmlrpc.client

W = Path(__file__).resolve().parents[1]
M = Path('/media/zhuyupei/SanStand')
E = M / '实验记录/Robot-Controller'
TARGETS = ('/grasp_task_node', '/remote_grasp6d_node', '/unknown_tabletop_perception')
PRESERVE = ('/alicia_d_driver_node', '/bessica_d_hw_interface', '/move_group',
            '/motion_gateway', '/supervisor_camera', '/compliant_gripper_controller',
            '/grasp_mode_router', '/alicia_supervisor_gui')

def read(path):
    return json.loads(Path(path).read_text())

def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def require(value, reason):
    if not value:
        raise RuntimeError(reason)

class Transport(xmlrpc.client.Transport):
    def make_connection(self, host):
        connection = super().make_connection(host)
        connection.timeout = 3.
        return connection

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--previous-deployment', type=Path, required=True)
    parser.add_argument('--release', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    out = args.output.resolve()
    require(M.is_mount() and not os.statvfs(M).f_flag & os.ST_RDONLY, 'mounted writable SanStand required')
    require(E / 'ros_log' in out.parents and out.parent.stat().st_dev == M.stat().st_dev,
            'external session output required')
    previous, release = read(args.previous_deployment), read(args.release)
    require(previous['deployment_passed'] is True, 'previous deployment not accepted')
    require(all(release.get(k) is True for k in ('passed', 'reviewed', 'installed')), 'release not sealed')
    summary = release['test_summary']
    require(summary['tests'] > 0 and all(summary[k] == 0 for k in ('failures', 'errors', 'skipped')),
            'release regression incomplete')
    expected = dict(previous['after'], **previous['preserved_after'])
    require(set(expected) == set(TARGETS + PRESERVE), 'prior node scope differs')

    def sources():
        for name, digest in release['source_sha256'].items():
            require(not Path(name).is_absolute() and '..' not in Path(name).parts and sha(W/name) == digest,
                    'source differs: ' + name)
        for path, digest in release['evidence_sha256'].items():
            require(sha(path) == digest, 'release evidence differs: ' + path)
    sources()
    out.mkdir(exist_ok=False)
    os.environ['ROS_LOG_DIR'] = str(out / 'observer_ros')
    import rosgraph
    import rospy
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    from actionlib_msgs.msg import GoalStatusArray
    from alicia_flexible_grasp_supervisor.msg import GraspState
    from alicia_grasp_modes.selection import parse_selection
    rospy.init_node('gen19_compute_reload', anonymous=True, disable_rosout=True)
    master = rosgraph.Master(rospy.get_name())

    def rpc(name):
        return xmlrpc.client.ServerProxy(master.lookupNode(name), transport=Transport())

    def identity(name):
        code, _, pid = rpc(name).getPid(rospy.get_name())
        require(code == 1, 'node PID unavailable: ' + name)
        fields = (Path('/proc') / str(pid) / 'stat').read_text().rsplit(')',1)[1].split()
        require(fields[0] != 'Z', 'zombie node: ' + name)
        return dict(pid=pid, start_ticks=fields[19], parent_pid=int(fields[1]))

    def exited(row):
        try:
            fields = (Path('/proc')/str(row['pid'])/'stat').read_text().rsplit(')',1)[1].split()
            return fields[0] == 'Z' or fields[19] != row['start_ticks']
        except FileNotFoundError:
            return True

    selected = parse_selection(rospy.get_param('/grasp_mode/selection'))
    def idle(require_task=True):
        require(rospy.get_param('/gui/joint_direct_mode') is True, 'GUI is not manual')
        require(parse_selection(rospy.get_param('/grasp_mode/selection')) == selected, 'selection changed')
        status = json.loads(rospy.wait_for_message('/grasp_mode/status', String, timeout=4).data)
        require(not any(status.get(k) for k in ('active', 'start_reserved', 'pending')), 'router busy')
        if require_task:
            require(not rospy.wait_for_message('/grasp/state', GraspState, timeout=4).active, 'task active')
        for topic in ('/alicia_controller/follow_joint_trajectory/status', '/hand_controller/follow_joint_trajectory/status'):
            statuses = rospy.wait_for_message(topic, GoalStatusArray, timeout=4)
            require(not any(g.status in (0,1,6,7) for g in statuses.status_list), 'active action')
        require(rospy.get_param('/alicia_d_driver_node/software_temperature_monitoring_enabled') is False
                and rospy.get_param('/alicia_d_driver_node/temperature_poll_rate_hz') == 0., 'temperature policy changed')

    result = dict(deployment_passed=False, whole_patch_deployed=False,
        source_manifest=str(args.release.resolve()), source_manifest_sha256=sha(args.release),
        source_sha256=release['source_sha256'], test_summary=summary,
        previous_deployment=str(args.previous_deployment.resolve()), previous_deployment_sha256=sha(args.previous_deployment),
        motion_commands_sent=False, torque_commands_sent=False, temperature_queried=False,
        new_selection_required_before_automatic_grasp=True, selection=selected,
        deployment_kind='three_compute_consumers_after_idle_identity_verification',
        reload_script_sha256=sha(__file__), shutdown_requests=[])
    def save():
        require(M.is_mount() and out.stat().st_dev == M.stat().st_dev, 'storage disappeared')
        temporary = out / 'result.json.tmp'
        with temporary.open('w') as stream:
            json.dump(result, stream, ensure_ascii=False, indent=2)
            stream.write('\n'); stream.flush(); os.fsync(stream.fileno())
        temporary.replace(out/'result.json')

    try:
        idle()
        result['before'] = {n:identity(n) for n in TARGETS}
        result['preserved_before'] = {n:identity(n) for n in PRESERVE}
        require(dict(result['before'], **result['preserved_before']) == expected, 'prior node identity changed')
        sources(); save()
        for name in TARGETS:
            idle(require_task=False)
            require(identity(name) == result['before'][name], 'target identity changed')
            row = dict(name=name, requested_ns=time.time_ns(), attempted=True)
            result['shutdown_requests'].append(row); save()
            try:
                code, message, _ = rpc(name).shutdown(rospy.get_name(), 'load reviewed feedback handoff clock repair')
                row.update(return_code=code, message=message)
            except Exception as exc:
                # Some rospy servers close their connection before replying.
                # This does not prove success: exact process exit is required below.
                row['rpc_error'] = repr(exc)
            save()
        deadline = time.monotonic()+25.
        while time.monotonic() < deadline:
            if all(exited(row) for row in result['before'].values()):
                break
            time.sleep(.1)
        require(all(exited(row) for row in result['before'].values()), 'old consumers remain; no retry or duplicate launch')
        result['old_processes_exited'] = True
        require({n:identity(n) for n in PRESERVE} == result['preserved_before'], 'preserved node changed')
        idle(require_task=False); sources()
        unit = 'alicia-observation-measurement-' + str(time.time_ns())
        command = ['systemd-run','--user','--collect','--unit='+unit,
            '--property=Type=exec','--property=Restart=no','--property=WorkingDirectory='+str(W),
            '--property=StandardOutput=append:'+str(out/'consumers.log'),
            '--property=StandardError=append:'+str(out/'consumers.log'),
            '--setenv=ROS_LOG_DIR='+str(out/'consumers_ros'),
            '/bin/bash',str(W/'.runtime/start_gen30_compute_consumers_20261016.sh')]
        result.update(unit=unit, command=command, new_launch_attempted=True); save()
        subprocess.run(command, check=True, timeout=15.)
        for service in ('/grasp_mode/reset_remote', '/grasp_mode/reset_task'):
            rospy.wait_for_service(service, timeout=40.)
            response = rospy.ServiceProxy(service, Trigger)()
            require(response.success, service+': '+response.message)
            result.setdefault('reset_ack',{})[service] = response.message
        idle()
        result['after'] = {n:identity(n) for n in TARGETS}
        result['preserved_after'] = {n:identity(n) for n in PRESERVE}
        parent = int(subprocess.check_output(['systemctl','--user','show',unit,'--property=MainPID','--value'],text=True).strip())
        require(parent > 1 and all(r['parent_pid'] == parent for r in result['after'].values()), 'new unit parent mismatch')
        require(all(result['after'][n] != result['before'][n] for n in TARGETS), 'old process reused')
        require(result['preserved_after'] == result['preserved_before'], 'hardware/GUI identity changed')
        sources()
        result.update(deployment_passed=True, whole_patch_deployed=True, compute_reloaded=True,
                      hardware_processes_preserved=True, recorded_ns=time.time_ns())
    except Exception as exc:
        result['error'] = repr(exc)
        raise
    finally:
        save()
        print(json.dumps(dict(deployment_passed=result['deployment_passed'], output=str(out),error=result.get('error')),ensure_ascii=False))

if __name__ == '__main__':
    main()
