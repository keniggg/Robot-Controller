#!/usr/bin/env python3
"""Check a frozen observation candidate through original gates, without motion."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
from types import SimpleNamespace

import numpy as np

ROOT = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
OLD = ROOT / 'ros_log/user_grasp_20261008T090751Z/source_snapshot_review'
SESSION = ROOT / 'ros_log/supplement_grasp_20261008T103117Z'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('candidate', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    assert Path('/media/zhuyupei/SanStand').is_mount()
    assert ROOT in args.output.resolve().parents
    args.output.mkdir(exist_ok=False, parents=True)
    os.environ['ROS_LOG_DIR'] = str(args.output / 'ros')
    import rospy
    from geometry_msgs.msg import PoseStamped
    from sensor_msgs.msg import JointState
    from moveit_msgs.msg import RobotState
    from moveit_msgs.srv import GetStateValidity, GetStateValidityRequest
    from alicia_flexible_grasp_supervisor.srv import SetTargetPose
    from alicia_flexible_grasp.robot.observation_path_guard import SerialUrdfFk
    from alicia_flexible_grasp.robot.observation_preview import decode_path_evidence, candidate_scene
    from alicia_flexible_grasp.robot.observation_tracking_contract import qualify_contract_path
    from alicia_flexible_grasp.robot.observation_view_guard import validate_observation_view_trajectory
    from alicia_flexible_grasp.grasp.gripper_geometry import GripperGeometry
    from alicia_grasp_modes.observe_return_visual import missing_side_view_gain
    from tf.transformations import quaternion_from_matrix

    rospy.init_node('complete_observation_candidate_readonly', anonymous=True, disable_rosout=True)
    candidate = json.loads(args.candidate.read_text())
    audit_bytes = (OLD / 'gen28_observation_frozen_audit.json').read_bytes()
    audit = json.loads(audit_bytes)
    assert hashlib.sha256(audit_bytes).hexdigest() == candidate['source_audit_sha256']
    frozen = json.loads((OLD / 'gen28_endpoints_prepared.jsonl').open().readline())
    source = json.loads((SESSION / 'frozen_view_source/source.json').read_text())
    arrays = np.load(str(SESSION / 'frozen_view_source/source.npz'))
    report = dict(execution_authorized=False, candidate=str(args.candidate),
        source_audit_sha256=candidate['source_audit_sha256'],
        candidate_sha256=hashlib.sha256(args.candidate.read_bytes()).hexdigest(),
        complete_candidate_checks_passed=False)
    def save():
        (args.output / 'result.json').write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
    try:
        assert rospy.get_param('/gui/joint_direct_mode') is False
        description = rospy.get_param('/robot_description')
        fk = SerialUrdfFk(description, ['Joint%d' % i for i in range(1, 7)])
        assert fk.model_sha256 == candidate['model_sha256']
        measured = rospy.wait_for_message('/alicia_d/accepted_joint_states', JointState, timeout=5.)
        assert measured.header.frame_id == 'sdk_measured'
        assert 0 <= (rospy.Time.now()-measured.header.stamp).to_sec() <= .2
        actual = dict(zip(measured.name, measured.position))
        assert all(abs(actual[n]-q) < 1e-12 for n,q in zip(frozen['seed']['name'], frozen['seed']['position']))
        state = RobotState(); state.joint_state = measured
        state.joint_state.position = list(state.joint_state.position)
        for name,value in zip(fk.names,candidate['q']):
            state.joint_state.position[state.joint_state.name.index(name)] = value
        rospy.wait_for_service('/check_state_validity', timeout=5.)
        valid = rospy.ServiceProxy('/check_state_validity', GetStateValidity)(
            GetStateValidityRequest(robot_state=state, group_name='alicia'))
        report['state_validity'] = dict(valid=valid.valid, contacts=[dict(body1=c.contact_body_1,
            body2=c.contact_body_2, depth=c.depth) for c in valid.contacts]); save()
        if not valid.valid:
            return
        target = np.asarray(candidate['T_base_tool0'])
        assert np.max(np.abs(fk(candidate['q'])-target)) < 1e-8
        pose = PoseStamped(); pose.header.frame_id = 'base_link'
        ns = audit['snapshot_stamp_ns']; pose.header.stamp = rospy.Time(ns//10**9, ns%10**9)
        for key,value in zip('xyz',target[:3,3]): setattr(pose.pose.position,key,float(value))
        for key,value in zip('xyzw',quaternion_from_matrix(target)): setattr(pose.pose.orientation,key,float(value))
        rospy.wait_for_service('/supervisor/check_observation_pose_strict',timeout=5.)
        response = rospy.ServiceProxy('/supervisor/check_observation_pose_strict',SetTargetPose)(pose,False)
        report['strict'] = dict(success=response.success,message=response.message); save()
        if not response.success:
            return
        tokens = re.findall(r'(?:^|\s)observation_path_evidence=(\S+)',response.message)
        assert len(tokens)==1
        evidence,trajectory,hold = decode_path_evidence(tokens[0],pose,fk)
        keys = ['/grasp_6d/remote/gripper_geometry', '/alicia_controller/constraints',
                '/alicia_controller/stop_trajectory_duration', '/robot/observation_tracking_contract_enabled']
        configuration = {k:rospy.get_param(k,None) for k in keys}
        assert configuration['/robot/observation_tracking_contract_enabled'] is True
        report['configuration'] = configuration
        cfg = configuration['/grasp_6d/remote/gripper_geometry']
        gripper = GripperGeometry(cfg.get('max_inner_gap_m',.05),cfg.get('width_safety_margin_per_side_m',.002),
            np.asarray(cfg.get('finger_box_xyz_m',[.0434,.0286,.06])),
            np.asarray(cfg.get('palm_box_xyz_m',[.1175,.155,.0774])),cfg.get('support_clearance_m',.003))
        g = audit['replay_geometry']
        geometry = SimpleNamespace(support_normal_base=g['support_normal_base'],support_offset_m=g['support_offset_m'],
            center_base=g['obb_center_base_m'],axes_base=g['R_base_obb'],size_xyz_m=g['obb_size_xyz_m'])
        scene = candidate_scene(geometry,ns)
        report['following_path'] = qualify_contract_path(trajectory,hold,scene,fk,gripper,
            evidence['opening'],configuration['/alicia_controller/constraints'],
            configuration['/alicia_controller/stop_trajectory_duration'])
        camera = source['camera']
        report['view_path'] = validate_observation_view_trajectory(trajectory,fk,arrays['tool_from_camera'],
            arrays['corners_base'],arrays['center_base'],arrays['points_base'],
            [camera[k] for k in ('fx','fy','cx','cy')],camera['width'],camera['height'],hold,
            **source['view_limits']['effective_visibility_limits'])
        goal = fk(trajectory.joint_trajectory.points[-1].positions) @ arrays['tool_from_camera']
        report['actual_ik_goal_gain'] = missing_side_view_gain(audit['observe_return']['measured_missing_sides'],
            goal[:3,3],candidate['endpoint_checks']['missing_side_view_gain']['depth_uncertainty_m'])
        assert rospy.get_param('/robot_description') == description
        assert {k:rospy.get_param(k,None) for k in keys} == configuration
        report['complete_candidate_checks_passed'] = bool(report['following_path']['ok']
            and report['view_path']['ok'] and report['actual_ik_goal_gain']['ok'])
    except Exception as exc:
        report['error'] = dict(type=type(exc).__name__,reason=str(exc),audit=getattr(exc,'audit',None))
        raise
    finally:
        save()
        print(json.dumps(dict(output=str(args.output/'result.json'),
            strict_success=report.get('strict',{}).get('success'),
            following_ok=report.get('following_path',{}).get('ok'),
            view_ok=report.get('view_path',{}).get('ok'),
            passed=report['complete_candidate_checks_passed'],error=report.get('error')),ensure_ascii=False))


if __name__ == '__main__': main()
