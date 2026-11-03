"""Frozen observation endpoints through read-only MoveIt IK, never execution."""
from collections import Counter
import argparse
import hashlib
import json
import os
from pathlib import Path
import time

ROOT = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
SESSION = ROOT / 'ros_log/full_latest_ros_20261009T003407Z'
AUDIT = ROOT / ('grasp_records/20261009T031019.431802Z_cf2238ea63164fd2a86f610925c32457/'
    'keyframes_audits/18452e4dea5750147f6da04b6122b9f605e6b39e98ef10c966855962b624f3d3.json')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output-name', default='gen10_observation_ik_probe')
    parser.add_argument('--timeout', type=float, default=.02)
    parser.add_argument('--indices', type=int, nargs='+')
    parser.add_argument('--diagnostic-ignore-collisions', action='store_true')
    args = parser.parse_args()
    assert args.output_name.startswith('gen10_') and '/' not in args.output_name
    assert 0 < args.timeout <= .5
    assert Path('/media/zhuyupei/SanStand').is_mount()
    out = SESSION / args.output_name
    out.mkdir(exist_ok=False)
    os.environ['ROS_LOG_DIR'] = str(out / 'ros')
    import numpy as np
    import rospy
    from sensor_msgs.msg import JointState
    from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest
    from tf.transformations import quaternion_from_matrix
    data = json.loads(AUDIT.read_text())
    assert hashlib.sha256(AUDIT.read_bytes()).hexdigest() == AUDIT.stem
    rospy.init_node('gen10_observation_ik_readonly', anonymous=True, disable_rosout=True)
    seed = rospy.wait_for_message('/alicia_d/accepted_joint_states', JointState, timeout=5)
    assert seed.header.frame_id == 'sdk_measured'
    assert 0 <= (rospy.Time.now() - seed.header.stamp).to_sec() < .3
    expected = data['observation_failure_diagnostics']['moveit_input_joint_state']
    mapping = dict(zip(seed.name, seed.position))
    assert all(abs(mapping[n]-q) < .004 for n,q in zip(expected['name'], expected['position_rad']))
    seed.velocity = []; seed.effort = []
    model_sha = hashlib.sha256(rospy.get_param('/robot_description').encode()).hexdigest()
    rospy.wait_for_service('/compute_ik', timeout=3)
    service = rospy.ServiceProxy('/compute_ik', GetPositionIK, persistent=True)
    rows = []
    for index, variant in enumerate(data['observation_failure_diagnostics']['observation_candidates']):
        if args.indices is not None and index not in args.indices:
            continue
        assert variant['frame_id'] == 'base_link'
        transform = np.array(variant['base_from_tool'])
        req = GetPositionIKRequest()
        req.ik_request.group_name = 'alicia'; req.ik_request.ik_link_name = 'tool0'
        req.ik_request.robot_state.joint_state = seed
        req.ik_request.pose_stamped.header.frame_id = 'base_link'
        pose = req.ik_request.pose_stamped.pose
        for field,value in zip('xyz',transform[:3,3]):setattr(pose.position,field,float(value))
        for field,value in zip('xyzw',quaternion_from_matrix(transform)):
            setattr(pose.orientation,field,float(value))
        req.ik_request.avoid_collisions = not args.diagnostic_ignore_collisions
        req.ik_request.timeout = rospy.Duration(args.timeout)
        started = time.monotonic()
        reply = service(req)
        row = dict(index=index, candidate_index=variant['candidate_index'],
            roll=variant['camera_roll_offset_deg'], distance=variant['camera_target_distance_m'],
            tilt=variant['camera_tilt_offset_deg'], code=reply.error_code.val,
            elapsed_sec=time.monotonic()-started, execution_authority=False)
        if reply.error_code.val == 1:
            row['solution'] = dict(zip(reply.solution.joint_state.name,reply.solution.joint_state.position))
        rows.append(row)
        with (out / 'rows.jsonl').open('a') as handle:handle.write(json.dumps(row)+'\n')
        if reply.error_code.val == 1 or index % 30 == 0: print(json.dumps(row),flush=True)
    end = rospy.wait_for_message('/alicia_d/accepted_joint_states', JointState, timeout=3)
    end_map = dict(zip(end.name,end.position))
    unchanged = all(abs(end_map[n]-q) < .004 for n,q in mapping.items())
    result = dict(audit=str(AUDIT), audit_sha256=AUDIT.stem, model_sha256=model_sha,
        scene='live MoveIt scene, historical frozen poses, same current accepted joint seed',
        start_seed=dict(stamp_ns=seed.header.stamp.to_nsec(),positions=mapping),
        source_pose_is_current=False, joint_seed_unchanged=unchanged,
        calls=len(rows), error_counts=dict(Counter(r['code'] for r in rows)),
        ik_timeout_sec=args.timeout, avoid_collisions=not args.diagnostic_ignore_collisions,
        found_indices=[r['index'] for r in rows if r['code']==1], motion_commands_sent=False,
        execution_authority=False, full_path_or_arrival_proven=False, rows=rows)
    (out/'result.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k!='rows'}),flush=True)


if __name__ == '__main__':main()
