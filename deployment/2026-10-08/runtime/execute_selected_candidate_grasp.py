#!/usr/bin/env python3
"""One fresh runner invocation; importing this module has no ROS side effects."""
import hashlib
import json
import os
from pathlib import Path
import subprocess
import time

MODEL_SHA = '7812d6b69e854ed9f37b4e3cbd813ecd53dab3504065b2d04d287abc6fd852e0'
HANDEYE_SHA = '637701accca5eb5042075c213c5e00e9b189d3cd7788d916c8c06bf89f7b1a0c'
RUNNER = ('rosrun', 'alicia_grasp_modes', 'mode_aware_grasp_runner.py',
          '--mode', 'unknown', '--strategy', 'observe_return')
INACTIVE_STAGES = {'IDLE': 0, 'SUCCESS': 6, 'FAILED': 7, 'HOLDING': 10}
STOPPED_MESSAGES = {'continuous remote 6D inference stopped',
                    'continuous remote 6D inference already stopped'}
ACTION_TOPICS = ('/alicia_controller/follow_joint_trajectory/status',
                 '/hand_controller/follow_joint_trajectory/status')


def same_selection(actual, expected):
    return (all(actual.get(k) == expected[k] for k in ('mode', 'strategy', 'generation'))
            and int(actual.get('stamp_ns', -1)) == int(expected['stamp_ns']))


def validate_admission(task, router, manual, actions, selection, expected):
    """Only idle/terminal computation may be retired; this grants no motion."""
    if task.active is not False or task.state not in INACTIVE_STAGES or task.stage != INACTIVE_STAGES[task.state]:
        raise RuntimeError('task is not idle/terminal')
    if not same_selection(selection, expected) or not same_selection(router, expected):
        raise RuntimeError('selection changed or router has not acknowledged it')
    if (router.get('active') is not False or router.get('start_reserved') is not False
            or router.get('pending') or router.get('pending_strategy')
            or router.get('state') not in ('ready', 'task_returned')):
        raise RuntimeError('router is active, reserved, switching, or unavailable')
    if manual is not False:
        raise RuntimeError('manual control active or invalid; handoff does not take slider ownership')
    if len(actions) != len(ACTION_TOPICS):
        raise RuntimeError('both controller action statuses required')
    for statuses, age_ns in actions:
        if not 0 <= age_ns <= 2_000_000_000:
            raise RuntimeError('controller action status stale')
        if any(type(s) is not int or s not in range(10) or s in (0, 1, 6, 7) for s in statuses):
            raise RuntimeError('controller goal active or status invalid')


def fresh_phase_handoff(read_context, request_plan, launch, save_phase, now_ns=time.time_ns):
    """Stop old inference once; child creates its timestamp/subscribers BEFORE True.

    A False request retires remote inference only, not a trajectory/controller.
    No retry, compensating restart, mode write or physical command is performed.
    The production runner/task retain their own final admission checks.
    """
    context = read_context()
    selection = context['selection']
    if selection.get('mode') != 'unknown' or selection.get('strategy') not in ('observe_return', 'two_stage', 'direct'):
        raise RuntimeError('unsupported selected grasp strategy')
    command = list(RUNNER)
    command[-1] = selection['strategy']
    phase = dict(reason='Retire previous preview computation before runner establishes its fresh window.',
                 selection=context['selection'], status='admitted', time_ns=now_ns(),
                 hardware_stop_or_disable_command_sent=False, old_plan_reused=False,
                 start_delegated_to_runner_after_minimum_stamp=True)
    save_phase(phase)
    try:
        response = request_plan(False)
        phase['stop_compute'] = [response.success, response.message]
        phase['status'] = 'old_computation_stop_acknowledged'
        save_phase(phase)
        if response.success is not True or response.message not in STOPPED_MESSAGES:
            raise RuntimeError('candidate computation stop was not acknowledged')
        # Recheck task, reservation, selection, manual ownership, target and pair.
        current = read_context()
        if current != context:
            raise RuntimeError('bound handoff context changed')
        phase['status'] = 'runner_handoff'
        save_phase(phase)
        result = launch(command)
        phase.update(status='runner_returned', runner_exit_code=result)
        save_phase(phase)
        return result
    except Exception as exc:
        phase.update(status='failed_no_further_action', error=str(exc))
        save_phase(phase)
        raise


def main():
    # Keep all live imports and work behind main, so offline tests cannot call ROS.
    import rospy
    from actionlib_msgs.msg import GoalStatusArray
    from std_msgs.msg import String
    from alicia_flexible_grasp_supervisor.msg import GraspState
    from alicia_flexible_grasp_supervisor.srv import TriggerZero

    output = Path(os.environ['GRASP_RECORD_DIR']).resolve()
    mount = Path('/media/zhuyupei/SanStand')
    storage = mount / '实验记录/Robot-Controller'
    if not mount.is_mount() or storage.resolve() not in output.parents or not output.is_dir():
        raise RuntimeError('existing SanStand experiment directory required')
    ros_log = output / 'ros'
    ros_log.mkdir(exist_ok=True)
    os.environ['ROS_LOG_DIR'] = str(ros_log)
    rospy.init_node('candidate_current_motor_handoff', anonymous=True)
    context = Path(json.loads(Path('.runtime/current_moved_motor.json').read_text())['directory']) / 'context.json'
    context_data = json.loads(context.read_text())
    expected = context_data['new_selection']
    user_change = context_data['user_change']
    if not isinstance(user_change, str) or not user_change.strip():
        raise RuntimeError('current context user_change required')
    rospy.wait_for_service('/grasp_6d/request_plan', timeout=15.)
    request_plan = rospy.ServiceProxy('/grasp_6d/request_plan', TriggerZero)

    def read_context():
        # GraspState is latched/event-driven: its old header is not a stale heartbeat.
        task = rospy.wait_for_message('/grasp/state', GraspState, timeout=5.)
        router = json.loads(rospy.wait_for_message('/grasp_mode/status', String, timeout=5.).data)
        action_messages = [rospy.wait_for_message(topic, GoalStatusArray, timeout=5.) for topic in ACTION_TOPICS]
        now = rospy.Time.now().to_nsec()
        actions = [([s.status for s in m.status_list], now - m.header.stamp.to_nsec()) for m in action_messages]
        validate_admission(task, router, rospy.get_param('/gui/joint_direct_mode', False), actions,
                           rospy.get_param('/grasp_mode/selection'), expected)
        model_sha = hashlib.sha256(rospy.get_param('/robot_description').encode()).hexdigest()
        if model_sha != MODEL_SHA:
            raise RuntimeError('candidate URDF changed')
        handeye = rospy.get_param('/handeye')
        x_sha = hashlib.sha256(Path(handeye['calibration_file']).read_bytes()).hexdigest()
        if x_sha != HANDEYE_SHA:
            raise RuntimeError('paired handeye changed')
        target = json.loads(rospy.wait_for_message('/perception/unknown/detector_status', String, timeout=5.).data)
        if (target.get('state') != 'ready' or target.get('target_locked') is not True or target.get('target_lost')
                or target.get('generation') != expected['generation']
                or int(target.get('selection_stamp_ns', 0)) != int(expected['stamp_ns'])):
            raise RuntimeError('new current motor is not locked')
        if not 0 <= rospy.Time.now().to_nsec() - target['source_stamp_ns'] <= 800_000_000:
            raise RuntimeError('current target stale')
        # Source stamp advances naturally. Bind the selection and model, not an old image.
        return dict(selection=expected, model_sha256=model_sha, handeye_sha256=x_sha)

    def save_phase(phase):
        (output / 'fresh_computation_phase.json').write_text(json.dumps(phase, ensure_ascii=False, indent=2) + '\n')

    record = dict(authorization=user_change,
                  selection=expected, model_sha256=MODEL_SHA, handeye_sha256=HANDEYE_SHA,
                  mass_policy='operator-declared <=20g upper bound, not measured mass',
                  physical_success='unverified', torque_disable_authorized=False,
                  old_plan_reused=False, recording_scope='local_only')
    (output / 'authorization.json').write_text(json.dumps(record, ensure_ascii=False, indent=2) + '\n')
    return fresh_phase_handoff(read_context, request_plan, subprocess.call, save_phase)


if __name__ == '__main__':
    raise SystemExit(main())
