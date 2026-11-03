"""Two bounded GetMotionPlan calculations; no execution or motion authority."""
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import time

ANALYSIS = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_frozen_no_skin_20261012T021050Z/gen8_analysis')
EXPECTED = '7812d6b69e854ed9f37b4e3cbd813ecd53dab3504065b2d04d287abc6fd852e0'


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def wire(value):
    if hasattr(value, '__slots__'):
        return {name: wire(getattr(value, name)) for name in value.__slots__}
    if isinstance(value, (list, tuple)):
        return [wire(item) for item in value]
    return value


def save(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + '\n')


def main():
    assert Path('/media/zhuyupei/SanStand').is_mount()
    out = ANALYSIS / 'known_fk_joint_plans_current_scene'
    out.mkdir(exist_ok=False)  # A duplicate invocation cannot repeat queries.
    os.environ['ROS_LOG_DIR'] = str(out / 'ros')
    import rospy
    import rosservice
    from moveit_msgs.msg import Constraints, JointConstraint, RobotState
    from moveit_msgs.srv import GetMotionPlan, GetMotionPlanRequest
    rospy.init_node('gen8_known_fk_joint_plans_readonly', anonymous=True, disable_rosout=True)
    model = rospy.get_param('/robot_description')
    assert hashlib.sha256(model.encode()).hexdigest() == EXPECTED
    assert rosservice.get_service_type('/plan_kinematic_path') == 'moveit_msgs/GetMotionPlan'
    source_path = ANALYSIS / 'fk_replay_inputs/source.json'
    replay_path = ANALYSIS / 'fk_replay_inputs/original_fk_replay.json'
    source, replay = [json.loads(path.read_text()) for path in (source_path, replay_path)]
    assert source['model_sha256'] == EXPECTED and len(replay['candidates']) == 2
    assert sha(Path(source['model_path'])) == EXPECTED
    names = ['Joint%d' % index for index in range(1, 7)]
    historical = source['current_joint_positions']
    assert len(historical) == 6 and all(math.isfinite(q) for q in historical)
    keys = ['/robot/unknown_observation_timing', '/robot/strict_pose_planning_time',
            '/motion_gateway/strict_pose_planning_time', '/motion_gateway_node/strict_pose_planning_time',
            '/robot/planning_attempts', '/robot/strict_execution_max_joint_velocity_rad_s',
            '/robot/observation_max_joint_velocity_rad_s', '/robot_description_planning',
            '/move_group/default_planning_pipeline', '/move_group/planning_pipelines/ompl']
    config = {key: rospy.get_param(key, None) for key in keys}
    timing = config['/robot/unknown_observation_timing']
    assert timing == {'enabled': True, 'velocity_scaling': .3, 'acceleration_scaling': .2}
    assert config['/move_group/default_planning_pipeline'] == 'ompl'
    pipeline = config['/move_group/planning_pipelines/ompl']
    assert pipeline['planning_plugin'] == 'ompl_interface/OMPLPlanner'
    planner = pipeline['alicia']['default_planner_config']
    assert planner == 'RRTConnectkConfigDefault'
    assert config['/motion_gateway/strict_pose_planning_time'] is None
    assert config['/motion_gateway_node/strict_pose_planning_time'] is None
    # Preserve the configured strict-pose budget when it is below the delegated .8 s ceiling.
    allowed = min(.8, float(config['/robot/strict_pose_planning_time']))
    assert 0.0 < allowed <= .8
    start_state = RobotState()
    start_state.is_diff = False
    start_state.joint_state.name = names + ['right_finger']
    start_state.joint_state.position = historical + [.04975]
    metadata = {
        'created_at': datetime.now(timezone.utc).isoformat(),
        'script': str(Path(__file__).resolve()), 'script_sha256': sha(Path(__file__)),
        'model_sha256_before': EXPECTED,
        'service': '/plan_kinematic_path', 'service_type': 'moveit_msgs/GetMotionPlan',
        'source': {'path': str(source_path), 'sha256': sha(source_path)},
        'replay': {'path': str(replay_path), 'sha256': sha(replay_path)},
        'parameters_before': config,
        'scope': 'Current MoveIt collision scene, historical GEN8 first-view start, two exact known FK joint goals.',
        'request_semantics': {
            'allowed_planning_time_sec': allowed, 'num_planning_attempts': 1,
            'joint_goal_tolerance_above_below': 0.,
            'joint_goal_reason': 'Original _plan_observation_with_branch_hint requests exact checked q with zero joint-goal tolerance.',
            'velocity_acceleration_scaling': [.3, .2],
            'scaling_reason': 'Existing unknown observation timing preference, applied here to calculation. Original strict retiming/contract not invoked.',
            'workspace': 'Unspecified request default; existing FixWorkspaceBounds adapter resolves it; no custom workspace introduced.',
            'collision_scene': 'Current MoveIt service scene; no historical scene reconstruction or mutation.',
        },
        'physical_actions': False, 'motion_authority': False,
        'not_checked': ['original strict envelope and continuous path guards',
                        'following/driver reference and retiming contract',
                        'visibility and measured information gain',
                        'live source identity, ticket, approval, admission or execution',
                        'historical collision scene equivalence'],
    }
    save(out / 'inputs.json', metadata)
    rospy.wait_for_service('/plan_kinematic_path', timeout=5.)
    service = rospy.ServiceProxy('/plan_kinematic_path', GetMotionPlan, persistent=True)
    rows = []
    for index, candidate in enumerate(replay['candidates']):
        assert candidate['joint_names'] == names and candidate['model_sha256'] == EXPECTED
        goal = candidate['q']
        assert len(goal) == 6 and all(math.isfinite(q) for q in goal)
        assert rospy.get_param('/robot_description') == model
        request = GetMotionPlanRequest()
        plan = request.motion_plan_request
        plan.group_name = 'alicia'
        plan.pipeline_id = 'ompl'
        plan.planner_id = planner
        plan.start_state = start_state
        plan.num_planning_attempts = 1
        plan.allowed_planning_time = allowed
        plan.max_velocity_scaling_factor = .3
        plan.max_acceleration_scaling_factor = .2
        constraints = Constraints()
        constraints.name = 'exact_recorded_fk_candidate_%d' % index
        for name, value in zip(names, goal):
            constraints.joint_constraints.append(JointConstraint(
                joint_name=name, position=value, tolerance_above=0., tolerance_below=0., weight=1.))
        plan.goal_constraints = [constraints]
        save(out / ('candidate_%d_request.json' % index), wire(request))
        started = time.monotonic()
        response = service(request)
        elapsed = time.monotonic() - started
        response_path = out / ('candidate_%d_response.json' % index)
        save(response_path, wire(response))
        result = response.motion_plan_response
        trajectory = result.trajectory.joint_trajectory
        points = list(trajectory.points)
        row = {'candidate_index': index, 'error_code': result.error_code.val,
               'success': result.error_code.val == 1,
               'wall_elapsed_sec': elapsed, 'reported_planning_time_sec': result.planning_time,
               'joint_names': list(trajectory.joint_names), 'point_count': len(points),
               'request': str(out / ('candidate_%d_request.json' % index)),
               'response': str(response_path), 'response_sha256': sha(response_path)}
        if points:
            ordered_start = dict(zip(trajectory.joint_names, points[0].positions))
            ordered_end = dict(zip(trajectory.joint_names, points[-1].positions))
            row.update(start_max_abs_error_rad=max(abs(ordered_start[n] - q) for n, q in zip(names, historical)),
                       goal_max_abs_error_rad=max(abs(ordered_end[n] - q) for n, q in zip(names, goal)),
                       duration_sec=points[-1].time_from_start.to_sec(),
                       maximum_sampled_velocity_rad_s=max([abs(v) for p in points for v in p.velocities] or [0.]),
                       maximum_sampled_acceleration_rad_s2=max([abs(v) for p in points for v in p.accelerations] or [0.]))
        rows.append(row)
        save(out / 'partial.json', rows)
    service.close()
    config_after = {key: rospy.get_param(key, None) for key in keys}
    model_after = hashlib.sha256(rospy.get_param('/robot_description').encode()).hexdigest()
    result = dict(metadata, queries=rows, service_call_count=len(rows),
                  success_count=sum(row['success'] for row in rows),
                  model_sha256_after=model_after, parameters_after=config_after,
                  model_and_parameters_unchanged=model_after == EXPECTED and config_after == config)
    save(out / 'result.json', result)
    print(json.dumps({'output': str(out / 'result.json'), 'sha256': sha(out / 'result.json'),
                      'queries': rows, 'unchanged': result['model_and_parameters_unchanged']}, indent=2))


if __name__ == '__main__':
    main()
