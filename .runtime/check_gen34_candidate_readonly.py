#!/usr/bin/env python3
"""Gen34 reconstructed-input diagnosis; strict planning and pure gates, no motion."""
import argparse
import copy
import hashlib
import json
import os
from pathlib import Path
import re
from types import SimpleNamespace

import numpy as np

ROOT = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
SESSION = ROOT / 'ros_log/supplement_grasp_20261008T103117Z'
SOURCE = SESSION / 'gen34_frozen_source'


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


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
    import tf2_ros
    from geometry_msgs.msg import PoseStamped
    from sensor_msgs.msg import JointState
    from moveit_msgs.msg import RobotState
    from moveit_msgs.srv import GetStateValidity, GetStateValidityRequest
    from alicia_flexible_grasp_supervisor.srv import SetTargetPose
    from alicia_flexible_grasp.robot.observation_path_guard import SerialUrdfFk
    from alicia_flexible_grasp.robot.observation_preview import decode_path_evidence, candidate_scene
    from alicia_flexible_grasp.robot.observation_tracking_contract import qualify_contract_path
    from alicia_flexible_grasp.robot.observation_view_guard import validate_observation_view_trajectory
    from alicia_flexible_grasp.robot.observation_view_gateway import ObservationViewGateway
    from alicia_flexible_grasp.grasp.gripper_geometry import GripperGeometry
    from alicia_grasp_modes.observe_return_visual import missing_side_view_gain
    from tf.transformations import quaternion_from_matrix

    rospy.init_node('gen34_observation_candidate_readonly', anonymous=True, disable_rosout=True)
    report = dict(execution_authorized=False,
        purpose='reconstructed_recorded_input_not_original_authority',
        all_original_request_inputs_available=False,
        candidate=str(args.candidate), candidate_sha256=sha(args.candidate),
        script_sha256=sha(Path(__file__)), complete_candidate_checks_passed=False,
        original_live_gates_and_fresh_source_required_for_any_execution=True,
        service_calls=[])

    def save():
        (args.output / 'result.json').write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')

    try:
        candidate = json.loads(args.candidate.read_text())
        source = json.loads((SOURCE / 'source.json').read_text())
        assert sha(SOURCE / 'source.json') == candidate['source_json_sha256']
        assert sha(SOURCE / 'source.npz') == source['source_npz_sha256']
        audit_path = Path(source['source_provenance']['audit_path'])
        assert sha(audit_path) == candidate['source_audit_sha256']
        audit = json.loads(audit_path.read_text())
        arrays = np.load(str(SOURCE / 'source.npz'))
        frozen = source['measured_joint_seed']
        report.update(source_audit_sha256=sha(audit_path), source_json_sha256=sha(SOURCE/'source.json'),
            source_npz_sha256=sha(SOURCE/'source.npz'),
            depth_uncertainty_scope=candidate['depth_uncertainty_scope'])
        assert rospy.get_param('/gui/joint_direct_mode') is False
        description = rospy.get_param('/robot_description')
        fk = SerialUrdfFk(description, ['Joint%d' % i for i in range(1, 7)])
        assert fk.model_sha256 == candidate['model_sha256'] == source['source_provenance']['model_sha256']
        measured = rospy.wait_for_message('/alicia_d/accepted_joint_states', JointState, timeout=5.)
        age = (rospy.Time.now()-measured.header.stamp).to_sec()
        assert measured.header.frame_id == 'sdk_measured'
        assert 0 <= age <= .2
        actual = dict(zip(measured.name, measured.position))
        assert all(abs(actual[n]-q) < 1e-12 for n,q in zip(frozen['names'], frozen['q']))
        assert actual['right_finger'] == .04975
        report['measured_seed'] = dict(stamp_ns=measured.header.stamp.to_nsec(), age_sec=age,
            names=list(measured.name), q=list(measured.position), frozen_seven_joints_match=True)

        keys = ['/grasp_6d/remote/gripper_geometry', '/alicia_controller/constraints',
                '/alicia_controller/stop_trajectory_duration', '/robot/observation_tracking_contract_enabled']
        configuration = {k:rospy.get_param(k,None) for k in keys}
        assert configuration['/robot/observation_tracking_contract_enabled'] is True
        report['configuration'] = configuration
        g = audit['replay_geometry']
        ns = audit['snapshot_stamp_ns']
        geometry = SimpleNamespace(support_normal_base=g['support_normal_base'],support_offset_m=g['support_offset_m'],
            center_base=g['obb_center_base_m'],axes_base=g['R_base_obb'],size_xyz_m=g['obb_size_xyz_m'])
        scene = candidate_scene(geometry,ns)
        gateway = ObservationViewGateway()
        gateway._observation_view_tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.))
        listener = tf2_ros.TransformListener(gateway._observation_view_tf_buffer)
        # Uses the original calibration/visibility parser and rigid-mount consistency gate.
        view_configuration = gateway._unknown_observation_view_configuration(scene)
        camera = source['camera']
        assert all(view_configuration['camera'][k] == camera[k] for k in ('fx','fy','cx','cy','width','height'))
        assert np.max(np.abs(np.asarray(view_configuration['tool_from_camera'])-arrays['tool_from_camera'])) < 1e-9
        report['view_configuration'] = view_configuration
        report['recorded_vs_live_mount_max_abs'] = float(np.max(np.abs(np.asarray(view_configuration['tool_from_camera'])-arrays['tool_from_camera'])))
        assert view_configuration['limits'] == source['view_limits']['effective_visibility_limits']
        state = RobotState(); state.joint_state = copy.deepcopy(measured)
        state.joint_state.position = list(state.joint_state.position)
        for name,value in zip(fk.names,candidate['q']):
            state.joint_state.position[state.joint_state.name.index(name)] = value
        rospy.wait_for_service('/check_state_validity', timeout=5.)
        report['service_calls'].append(dict(name='/check_state_validity', pure_computation=True)); save()
        valid = rospy.ServiceProxy('/check_state_validity', GetStateValidity)(
            GetStateValidityRequest(robot_state=state, group_name='alicia'))
        report['state_validity'] = dict(valid=valid.valid, contacts=[dict(body1=c.contact_body_1,
            body2=c.contact_body_2, depth=c.depth) for c in valid.contacts]); save()
        if not valid.valid:
            return
        target = np.asarray(candidate['T_base_tool0'])
        assert np.max(np.abs(fk(candidate['q'])-target)) < 1e-8
        pose = PoseStamped(); pose.header.frame_id = 'base_link'
        pose.header.stamp = rospy.Time(ns//10**9, ns%10**9)
        for key,value in zip('xyz',target[:3,3]): setattr(pose.pose.position,key,float(value))
        for key,value in zip('xyzw',quaternion_from_matrix(target)): setattr(pose.pose.orientation,key,float(value))
        rospy.wait_for_service('/supervisor/check_observation_pose_strict',timeout=5.)
        report['service_calls'].append(dict(name='/supervisor/check_observation_pose_strict', execute=False)); save()
        response = rospy.ServiceProxy('/supervisor/check_observation_pose_strict',SetTargetPose)(pose,False)
        report['strict'] = dict(success=response.success,message=response.message); save()
        if not response.success:
            return
        tokens = re.findall(r'(?:^|\s)observation_path_evidence=(\S+)',response.message)
        assert len(tokens)==1
        evidence,trajectory,hold = decode_path_evidence(tokens[0],pose,fk)
        assert np.max(np.abs(np.asarray(hold.positions)-np.asarray(frozen['q'][:6]))) < 1e-12
        cfg = configuration['/grasp_6d/remote/gripper_geometry']
        gripper = GripperGeometry(cfg.get('max_inner_gap_m',.05),cfg.get('width_safety_margin_per_side_m',.002),
            np.asarray(cfg.get('finger_box_xyz_m',[.0434,.0286,.06])),
            np.asarray(cfg.get('palm_box_xyz_m',[.1175,.155,.0774])),cfg.get('support_clearance_m',.003))
        goal_q = list(trajectory.joint_trajectory.points[-1].positions)
        goal_tool = fk(goal_q)
        goal_camera = goal_tool @ arrays['tool_from_camera']
        report['actual_trajectory'] = dict(joint_names=list(trajectory.joint_trajectory.joint_names),
            hold_q=list(hold.positions), goal_q=goal_q, actual_goal_T_base_tool0=goal_tool.tolist(),
            actual_goal_T_base_camera_link=goal_camera.tolist(),
            proposal_goal_joint_max_delta_rad=float(np.max(np.abs(np.asarray(goal_q)-candidate['q']))),
            points=len(trajectory.joint_trajectory.points),
            duration_sec=trajectory.joint_trajectory.points[-1].time_from_start.to_sec(),
            evidence_scope=evidence['scope'], trajectory_sha256=evidence['trajectory_sha256'])
        report['following_path'] = qualify_contract_path(trajectory,hold,scene,fk,gripper,
            evidence['opening'],configuration['/alicia_controller/constraints'],
            configuration['/alicia_controller/stop_trajectory_duration'])
        save()
        report['view_path'] = validate_observation_view_trajectory(trajectory,fk,arrays['tool_from_camera'],
            arrays['corners_base'],arrays['center_base'],arrays['points_base'],
            [camera[k] for k in ('fx','fy','cx','cy')],camera['width'],camera['height'],hold,
            **view_configuration['limits'])
        report['actual_ik_goal_gain'] = missing_side_view_gain(audit['observe_return']['measured_missing_sides'],
            goal_camera[:3,3],candidate['endpoint_checks']['missing_side_view_gain']['depth_uncertainty_m'])
        assert rospy.get_param('/robot_description') == description
        assert {k:rospy.get_param(k,None) for k in keys} == configuration
        assert gateway._unknown_observation_view_configuration(scene) == view_configuration
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
            gain_ok=report.get('actual_ik_goal_gain',{}).get('ok'),
            passed=report['complete_candidate_checks_passed'],error=report.get('error')),ensure_ascii=False))


if __name__ == '__main__': main()
