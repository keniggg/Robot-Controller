"""Only MoveIt FK, state validity and IK calculation calls; no path authority."""
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import time

ANALYSIS = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_frozen_no_skin_20261012T021050Z/gen8_analysis')
EXPECTED = '7812d6b69e854ed9f37b4e3cbd813ecd53dab3504065b2d04d287abc6fd852e0'


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def wire(value):
    if hasattr(value, '__slots__'):
        return {name:wire(getattr(value,name)) for name in value.__slots__}
    if isinstance(value, (list,tuple)):
        return [wire(item) for item in value]
    return value


def main():
    assert Path('/media/zhuyupei/SanStand').is_mount()
    out = ANALYSIS / 'known_fk_current_scene_diagnostic'
    out.mkdir(exist_ok=False)
    os.environ['ROS_LOG_DIR'] = str(out / 'ros')
    import numpy as np
    import rospy
    from geometry_msgs.msg import PoseStamped
    from moveit_msgs.msg import RobotState
    from moveit_msgs.srv import GetPositionFK, GetPositionFKRequest, GetPositionIK, GetPositionIKRequest, GetStateValidity, GetStateValidityRequest
    from tf.transformations import quaternion_from_matrix, quaternion_matrix
    from alicia_flexible_grasp.robot.observation_path_guard import SerialUrdfFk
    rospy.init_node('gen8_known_fk_readonly',anonymous=True,disable_rosout=True)
    description = rospy.get_param('/robot_description')
    assert hashlib.sha256(description.encode()).hexdigest() == EXPECTED
    source_path = ANALYSIS / 'fk_replay_inputs/source.json'
    replay_path = ANALYSIS / 'fk_replay_inputs/original_fk_replay.json'
    prior_path = ANALYSIS / 'added_endpoint_current_scene_ik/queries.jsonl'
    source, replay = [json.loads(path.read_text()) for path in (source_path,replay_path)]
    assert len(replay['candidates']) == 2 and source['model_sha256'] == EXPECTED
    assert sha(Path(source['model_path'])) == EXPECTED
    prior = [json.loads(line) for line in prior_path.read_text().splitlines()]
    names = ['Joint%d' % index for index in range(1,7)]
    local_fk = SerialUrdfFk(description,names)
    services = {}
    for path,kind in (('/compute_fk',GetPositionFK),('/compute_ik',GetPositionIK),('/check_state_validity',GetStateValidity)):
        rospy.wait_for_service(path,timeout=5.)
        services[path] = rospy.ServiceProxy(path,kind,persistent=True)

    def robot_state(q):
        state = RobotState()
        state.is_diff = False
        state.joint_state.name = names+['right_finger']
        state.joint_state.position = list(q)+[.04975]
        return state

    def pose_from_matrix(matrix):
        message = PoseStamped()
        message.header.frame_id = 'base_link'
        message.header.stamp = rospy.Time(0)
        message.pose.position.x,message.pose.position.y,message.pose.position.z = matrix[:3,3]
        quaternion = quaternion_from_matrix(matrix)
        message.pose.orientation.x,message.pose.orientation.y,message.pose.orientation.z,message.pose.orientation.w = quaternion
        return message

    def query_ik(matrix,seed,avoid):
        request = GetPositionIKRequest()
        ik = request.ik_request
        ik.group_name,ik.ik_link_name,ik.avoid_collisions = 'alicia','tool0',avoid
        ik.robot_state,ik.pose_stamped,ik.timeout = robot_state(seed),pose_from_matrix(matrix),rospy.Duration(.02)
        started = time.monotonic()
        response = services['/compute_ik'](request)
        elapsed = time.monotonic()-started
        solution = dict(zip(response.solution.joint_state.name,response.solution.joint_state.position))
        row = dict(avoid_collisions=avoid,timeout_sec=.02,seed=list(seed),error_code=response.error_code.val,
            elapsed_sec=elapsed,solution=solution if response.error_code.val==1 else None)
        if response.error_code.val==1:
            solved = local_fk([solution[name] for name in names])
            row.update(solution_matrix_max_abs_error=float(np.max(np.abs(solved-matrix))),
                seed_delta_max_rad=max(abs(solution[name]-value) for name,value in zip(names,seed)))
        return row

    known = []
    for index,candidate in enumerate(replay['candidates']):
        assert candidate['model_sha256']==EXPECTED and candidate['joint_names']==names
        q = candidate['q']
        saved = np.asarray(candidate['T_base_tool0'])
        local = local_fk(q)
        assert np.max(np.abs(saved-local)) < 1e-12
        state = robot_state(q)
        fk_request = GetPositionFKRequest()
        fk_request.header.frame_id = 'base_link'
        fk_request.fk_link_names = ['tool0']
        fk_request.robot_state = state
        fk_response = services['/compute_fk'](fk_request)
        assert fk_response.error_code.val==1 and len(fk_response.pose_stamped)==1
        pose = fk_response.pose_stamped[0].pose
        moveit_matrix = quaternion_matrix([pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w])
        moveit_matrix[:3,3] = [pose.position.x,pose.position.y,pose.position.z]
        validity_request = GetStateValidityRequest(robot_state=state,group_name='alicia')
        validity = services['/check_state_validity'](validity_request)
        known.append(dict(index=index,q=q,stored_T_base_tool0=saved.tolist(),
            local_T_base_tool0=local.tolist(),moveit_T_base_tool0=moveit_matrix.tolist(),
            local_saved_max_abs_error=float(np.max(np.abs(saved-local))),
            moveit_local_max_abs_error=float(np.max(np.abs(moveit_matrix-local))),
            moveit_position_error_m=float(np.linalg.norm(moveit_matrix[:3,3]-local[:3,3])),
            state_validity=wire(validity),
            exact_fk_seed_ik=[query_ik(local,q,avoid) for avoid in (True,False)],
            historical_seed_ik=[query_ik(local,source['current_joint_positions'],avoid) for avoid in (True,False)]))
    indices = sorted({round(index*(len(prior)-1)/11) for index in range(12)})
    low_controls = []
    for index in indices:
        previous = prior[index]
        low_controls.append(dict(prior_query_index=index,preference_index=previous['preference_index'],
            distance_m=previous['distance_m'],tilt_deg=previous['tilt_deg'],roll_deg=previous['roll_deg'],
            prior_collision_aware_error_code=previous['error_code'],
            collision_disabled_query=query_ik(np.asarray(previous['base_from_tool']),source['current_joint_positions'],False)))
    after = hashlib.sha256(rospy.get_param('/robot_description').encode()).hexdigest()
    report = dict(scope='Current scene diagnostic calculation only: FK and state validity for 2 offline joint-space proposals; IK exact seeds and collision-disabled controls. No path or execution authority.',
        audited_at=datetime.now(timezone.utc).isoformat(),
        source_files={str(path):sha(path) for path in (source_path,replay_path,prior_path)},
        robot_description_sha256=EXPECTED,robot_description_sha256_after=after,model_unchanged=after==EXPECTED,
        group_name='alicia',tool_link='tool0',right_finger=.04975,known_fk_candidates=known,
        low_angle_collision_disabled_controls=low_controls,
        summary=dict(known_candidates=2,known_state_valid=sum(row['state_validity']['valid'] for row in known),
            moveit_fk_matches_local=sum(row['moveit_local_max_abs_error']<1e-10 for row in known),
            exact_seed_collision_aware_ik_success=sum(row['exact_fk_seed_ik'][0]['error_code']==1 for row in known),
            exact_seed_collision_disabled_ik_success=sum(row['exact_fk_seed_ik'][1]['error_code']==1 for row in known),
            historical_seed_collision_aware_ik_success=sum(row['historical_seed_ik'][0]['error_code']==1 for row in known),
            low_angle_collision_disabled_ik_success=sum(row['collision_disabled_query']['error_code']==1 for row in low_controls),
            low_angle_control_count=len(low_controls)),
        collision_scene_frozen=False,historical_scene_reconstructed=False,full_path_checked=False,
        online_collision_configuration_changed=False,motion_authorized=False,motion_commands_sent=False,temperature_queried=False)
    (out/'result.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(dict(output=str(out/'result.json'),sha256=sha(out/'result.json'),summary=report['summary']),ensure_ascii=False))


if __name__ == '__main__':
    main()
