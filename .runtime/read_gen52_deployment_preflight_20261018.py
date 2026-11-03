"""Read-only GEN52 idle reload preflight against the actual accepted GEN51 runtime."""
import argparse
import os
from pathlib import Path
import time
import xmlrpc.client
from gen52_idle_reload_common_20261018 import *


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('output', 'release', 'previous', 'control'):
        parser.add_argument('--'+name, type=Path, required=True)
    args = parser.parse_args()
    output = args.output.resolve(); storage(output)
    require(not output.exists(), 'output already exists')
    pinned = {str(path.resolve()): path.read_bytes() for path in (args.release, args.previous, args.control)+HELPERS}
    release = validate_release(args.release)
    previous, expected = validate_previous(args.previous)
    control = validate_control(args.control)
    output.mkdir()
    os.environ['ROS_LOG_DIR'] = str(output/'ros')
    import rosgraph
    import rospy
    from actionlib_msgs.msg import GoalStatusArray
    from std_msgs.msg import Header, String
    from alicia_flexible_grasp_supervisor.msg import GraspState
    rospy.init_node('gen52_readonly_reload_preflight', anonymous=True, disable_rosout=True)
    master = rosgraph.Master(rospy.get_name())
    def identity(name):
        code, _, pid = xmlrpc.client.ServerProxy(master.lookupNode(name), transport=Transport()).getPid(rospy.get_name())
        require(code == 1, 'node PID unavailable: '+name)
        return process(pid)
    def idle():
        require(rospy.get_param('/gui/joint_direct_mode') is True, 'GUI is not manual')
        require(not rospy.wait_for_message('/grasp/state', GraspState, timeout=5.).active, 'task active')
        mode = json.loads(rospy.wait_for_message('/grasp_mode/status', String, timeout=5.).data)
        require(not any(mode.get(k) for k in ('active', 'start_reserved', 'pending')), 'router busy')
        for topic in ('/alicia_controller/follow_joint_trajectory/status', '/hand_controller/follow_joint_trajectory/status'):
            msg = rospy.wait_for_message(topic, GoalStatusArray, timeout=5.)
            require(not any(goal.status in (0, 1, 6, 7) for goal in msg.status_list), 'active action')
    result = dict(passed=False, deployment_performed=False, scene_permission=False,
        motion_commands_sent=False, torque_disable_commands_sent=False, temperature_queried=False)
    try:
        actual = {name: identity(name) for name in expected}
        require(actual == expected, 'current 11 nodes differ from accepted GEN51 identities')
        selection = rospy.get_param('/grasp_mode/selection')
        require(type(selection.get('generation')) is int and selection['generation'] == 12
            and selection.get('mode') == 'unknown' and selection.get('strategy') == 'observe_return',
            'expected unchanged unknown observe_return generation 12')
        epoch = rospy.wait_for_message('/alicia_d/control_reference_epoch', Header, timeout=5.).stamp.to_nsec()
        require(epoch == previous['epoch_ns'] == 1792286012067798399, 'accepted GEN51 epoch changed')
        parameters = {name: rospy.get_param(name) for name in PARAMETERS}
        require(parameters[PARAMETERS[0]] == 1 and parameters[PARAMETERS[1]] is False, 'minimal observation policy changed')
        require(parameters[PARAMETERS[-2]] is False and parameters[PARAMETERS[-1]] == 0., 'temperature polling policy changed')
        idle(); sdk = feedback(rospy)
        require(sdk['epoch_ns'] == epoch, 'SDK epoch changed')
        require({name: identity(name) for name in expected} == actual, 'node identity changed during preflight')
        idle(); validate_sources(release)
        require(all(Path(path).read_bytes() == raw for path, raw in pinned.items()), 'input/helper bytes changed')
        require(rospy.get_param('/grasp_mode/selection') == selection, 'selection changed')
        require(rospy.wait_for_message('/alicia_d/control_reference_epoch', Header, timeout=5.).stamp.to_nsec() == epoch, 'epoch changed')
        require({name: rospy.get_param(name) for name in PARAMETERS} == parameters, 'runtime parameters changed')
        result.update(passed=True, recorded_ns=time.time_ns(), source_manifest=str(args.release.resolve()),
            source_manifest_sha256=sha(args.release), previous_deployment=str(args.previous.resolve()),
            previous_deployment_sha256=sha(args.previous), input_sha256={path: hashlib.sha256(raw).hexdigest() for path, raw in pinned.items()},
            helper_sha256={str(path): sha(path) for path in HELPERS}, node_identities=actual, selection=selection,
            manual_mode=True, epoch_ns=epoch, feedback=sdk, names=sdk['names'], positions=sdk['positions'], sample_count=sdk['sample_count'],
            task_active=False, actions_active=False, control_path=str(args.control.resolve()), control_snapshot=control,
            control_sha256=sha(args.control), runtime_parameters=parameters,
            gateway_original_parent=process(actual['/motion_gateway']['parent_pid']),
            replaced_nodes=list(TARGETS), preserved_nodes=list(PRESERVE),
            gateway_launch_policy='original_bringup_respawn_only_no_second_gateway')
    except Exception as exc:
        result['error'] = repr(exc)
        raise
    finally:
        write(output/'result.json', result)
        print(json.dumps(dict(passed=result['passed'], output=str(output), error=result.get('error')), ensure_ascii=False), flush=True)


if __name__ == '__main__':
    main()
