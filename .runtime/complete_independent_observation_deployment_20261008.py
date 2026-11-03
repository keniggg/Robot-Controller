"""Complete the verified partial reload without restarting its new gateway."""
import hashlib
import json
import os
from pathlib import Path
import runpy
import subprocess

WORK = Path(__file__).resolve().parents[1]
lib = runpy.run_path(str(WORK/'.runtime/load_independent_observation_consumers_20261008.py'))
SESSION = lib['SESSION']
rpc, process_identity, wait_until = (lib[k] for k in ('rpc', 'process_identity', 'wait_until'))


def main():
    assert Path('/media/zhuyupei/SanStand').is_mount()
    before = json.loads((SESSION/'independent_observation_deployment_retry1/result.json').read_text())
    assert before['deployment_passed'] is False and before['new_launch_started'] is False
    assert before['shutdown_requests_sent'] == list(lib['TARGETS'])
    aliases = {n:p for n,p in before['preserved_before'].items() if p in before['before'].values()}
    assert aliases == {'/move_group_commander_wrappers_1791266183679893389': before['before']['/motion_gateway']}
    preserve = {n:p for n,p in before['preserved_before'].items() if n not in aliases}
    expired_observer = '/grasp_attempt_watch_readonly_492499_1791460373042'
    observer_identity = preserve.pop(expired_observer)
    assert not (Path('/proc')/str(observer_identity['pid'])).exists()
    assert set(lib['REQUIRED_PRESERVE']).issubset(preserve)
    out = SESSION/'independent_observation_deployment_completion_retry2'
    out.mkdir(exist_ok=False)
    os.environ['ROS_LOG_DIR'] = str(out/'observer_ros')
    import rospy, rosgraph
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Header, String
    from actionlib_msgs.msg import GoalStatusArray
    from std_srvs.srv import Trigger
    from alicia_flexible_grasp_supervisor.msg import GraspState
    rospy.init_node('complete_independent_observation_deployment', anonymous=True, disable_rosout=True)
    master = rosgraph.Master(rospy.get_name())
    def identity(name):
        reply = rpc(master.lookupNode(name)).getPid(rospy.get_name())
        assert reply[0] == 1
        return process_identity(reply[2])
    def check():
        for name,p in preserve.items(): assert identity(name) == p, name
        assert identity('/motion_gateway') == before['respawned_gateway']
        assert rospy.get_param('/grasp_mode/selection') == before['selection']
        assert rospy.get_param('/gui/joint_direct_mode') is False
        assert not rospy.get_param('/rgb_acquisition/ledger', {})
        e = rospy.wait_for_message('/alicia_d/control_reference_epoch', Header, timeout=5.)
        assert e.stamp.to_nsec() == before['epoch_before'] and e.frame_id == 'sdk_reference_epoch'
        q = rospy.wait_for_message('/alicia_d/accepted_joint_states', JointState, timeout=5.)
        assert q.header.frame_id == 'sdk_measured' and 0 <= (rospy.Time.now()-q.header.stamp).to_sec() < .2
        assert dict(names=list(q.name),positions=list(q.position)) == before['measured_before']
        status = json.loads(rospy.wait_for_message('/grasp_mode/status', String, timeout=5.).data)
        assert status['active'] is False and status['start_reserved'] is False and not status.get('pending')
        for topic, kind in master.getPublishedTopics('/'):
            if kind != 'actionlib_msgs/GoalStatusArray': continue
            m = rospy.wait_for_message(topic, GoalStatusArray, timeout=4.)
            assert 0 <= (rospy.Time.now()-m.header.stamp).to_sec() < 2.
            assert not any(g.status in (0,1,6,7) for g in m.status_list), topic
        for path, digest in before['validated_source_sha256'].items():
            assert hashlib.sha256((WORK/path).read_bytes()).hexdigest() == digest, path
    result = dict(deployment_passed=False, gateway_restarted_again=False, excluded_process_aliases=aliases,
                  expired_readonly_observer={expired_observer:observer_identity},
                  motion_commands_sent=False, temperature_queried=False, joint_disable_commands_sent=False,
                  prior_partial_result='independent_observation_deployment_retry1/result.json')
    try:
        check()
        for name in lib['NEW_LAUNCH_NODES']:
            try: process_identity(before['before'][name]['pid'])
            except (OSError, RuntimeError): pass
            else: raise AssertionError('old process still active: '+name)
            # A terminated rospy node can leave a stale master registration.
            # Require its process absent and its registered endpoint refused;
            # a name alone is not evidence of a surviving replacement.
            try: identity(name)
            except (rosgraph.MasterError, ConnectionRefusedError): pass
            else: raise AssertionError('node endpoint still active: '+name)
        command = ['systemd-run','--user','--collect','--unit=alicia-independent-observation-consumers-20261008',
            '--property=Type=exec','--property=Restart=no','--property=WorkingDirectory='+str(WORK),
            '--property=StandardOutput=append:'+str(out/'consumers.log'),
            '--property=StandardError=append:'+str(out/'consumers.log'),
            '--setenv=ROS_LOG_DIR='+str(out/'consumers_ros'),'/bin/bash',
            str(WORK/'.runtime/start_independent_observation_consumers_20261008.sh')]
        result['command'] = command
        subprocess.run(command, check=True, timeout=15.)
        def available():
            try: return all(identity(n) != before['before'][n] for n in lib['NEW_LAUNCH_NODES'])
            except Exception: return False
        wait_until(available, 45., 'new consumers unavailable')
        for service in ('/grasp_mode/reset_task','/grasp_mode/reset_remote'):
            rospy.wait_for_service(service, timeout=20.)
            reply = rospy.ServiceProxy(service,Trigger)()
            assert reply.success, (service,reply.message)
            result.setdefault('reset_ack',{})[service] = reply.message
        check()
        state = rospy.wait_for_message('/grasp/state',GraspState,timeout=5.)
        assert not state.active
        result.update(deployment_passed=True, after={n:identity(n) for n in lib['TARGETS']},
                      preserved_after={n:identity(n) for n in preserve},
                      measured_after=before['measured_before'],epoch_after=before['epoch_before'])
    except Exception as exc:
        result['error'] = repr(exc)
        raise
    finally:
        (out/'result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
        print(json.dumps(result,ensure_ascii=False),flush=True)


if __name__ == '__main__': main()
