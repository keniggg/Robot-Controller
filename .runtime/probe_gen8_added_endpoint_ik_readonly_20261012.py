"""Current-scene, collision-aware endpoint IK only; no motion authority."""
from collections import Counter, defaultdict
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import time

BASE = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
ANALYSIS = BASE / 'ros_log/full_latest_frozen_no_skin_20261012T021050Z/gen8_analysis'
EXPECTED = '7812d6b69e854ed9f37b4e3cbd813ecd53dab3504065b2d04d287abc6fd852e0'


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def pose_key(row):
    return tuple(tuple(values) for values in row['base_from_tool'])


def main():
    assert Path('/media/zhuyupei/SanStand').is_mount()
    out = ANALYSIS / 'added_endpoint_current_scene_ik'
    out.mkdir(exist_ok=False)
    os.environ['ROS_LOG_DIR'] = str(out / 'ros')
    import numpy as np
    import rospy
    from geometry_msgs.msg import PoseStamped
    from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest
    from tf.transformations import quaternion_from_matrix
    rospy.init_node('gen8_added_endpoint_ik_readonly', anonymous=True, disable_rosout=True)
    description = rospy.get_param('/robot_description')
    assert hashlib.sha256(description.encode()).hexdigest() == EXPECTED
    baseline_path = ANALYSIS / 'full_generator_baseline/result.json'
    candidate_path = ANALYSIS / 'full_generator_candidate/result.json'
    input_path = ANALYSIS / 'fk_replay_inputs/source.json'
    baseline, candidate, source = [json.loads(p.read_text()) for p in (baseline_path, candidate_path, input_path)]
    audit_path = Path(source['source_audit'])
    assert sha(audit_path) == source['source_audit_sha256'] == candidate['source_audit_sha256']
    recorded = json.loads(audit_path.read_text())['observation_failure_diagnostics']['observation_candidates']
    original_keys = {pose_key(row) for row in baseline['candidates']}
    candidate_keys = {pose_key(row) for row in candidate['candidates']}
    assert len(original_keys) == len(recorded) == 106
    assert original_keys == {pose_key(row) for row in recorded}
    assert original_keys <= candidate_keys and len(candidate_keys) == 218
    added = [row for row in candidate['candidates'] if pose_key(row) not in original_keys]
    assert len(added) == 112
    groups = defaultdict(list)
    roll_order = {value:index for index,value in enumerate((0.,-15.,15.,-30.,30.,-45.,45.,-60.,60.,-75.,75.,-90.,90.,180.))}
    for row in added:
        groups[(round(row['camera_target_distance_m'], 6), round(row['camera_tilt_offset_deg'], 6))].append(row)
    queues = [sorted(groups[key], key=lambda row:roll_order[row['camera_roll_offset_deg']]) for key in sorted(groups)]
    ordered = [queue[index] for index in range(max(map(len,queues))) for queue in queues if index < len(queue)]
    assert len(ordered) == 112 and len({pose_key(row) for row in ordered}) == 112
    rospy.wait_for_service('/compute_ik', timeout=5.)
    service = rospy.ServiceProxy('/compute_ik', GetPositionIK, persistent=True)
    names = ['Joint%d' % index for index in range(1,7)]
    seed = list(source['current_joint_positions'])
    assert len(seed) == 6 and all(math.isfinite(value) for value in seed)
    rows = []
    started = time.monotonic()
    with (out / 'queries.jsonl').open('x', buffering=1) as stream:
        for index, candidate_row in enumerate(ordered):
            request = GetPositionIKRequest()
            ik = request.ik_request
            ik.group_name = 'alicia'
            ik.ik_link_name = 'tool0'
            ik.avoid_collisions = True
            ik.timeout = rospy.Duration(.02)
            ik.robot_state.is_diff = False
            ik.robot_state.joint_state.name = names + ['right_finger']
            ik.robot_state.joint_state.position = seed + [.04975]
            ik.pose_stamped = PoseStamped()
            ik.pose_stamped.header.frame_id = 'base_link'
            ik.pose_stamped.header.stamp = rospy.Time(0)
            matrix = np.asarray(candidate_row['base_from_tool'], dtype=float)
            quaternion = quaternion_from_matrix(matrix)
            pose = ik.pose_stamped.pose
            pose.position.x, pose.position.y, pose.position.z = matrix[:3,3]
            pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w = quaternion
            before = time.monotonic()
            bounded_start = index < 96 and before-started < 3.
            row = dict(query_index=index, preference_index=candidate_row['preference_index'],
                distance_m=candidate_row['camera_target_distance_m'],
                tilt_deg=candidate_row['camera_tilt_offset_deg'], roll_deg=candidate_row['camera_roll_offset_deg'],
                base_from_tool=candidate_row['base_from_tool'], started_elapsed_sec=before-started,
                query_timeout_sec=.02, within_96_query_3s_start_budget=bounded_start)
            try:
                response = service(request)
                finished = time.monotonic()
                solution = dict(zip(response.solution.joint_state.name, response.solution.joint_state.position))
                success = (response.error_code.val == 1 and all(name in solution and math.isfinite(solution[name]) for name in names))
                row.update(error_code=response.error_code.val, collision_aware_endpoint_solution=success,
                    elapsed_sec=finished-before, finished_elapsed_sec=finished-started,
                    within_96_query_3s_completed_budget=bounded_start and finished-started <= 3.,
                    solution_joint_positions=solution if success else None,
                    max_joint_delta_from_historical_seed_rad=max(abs(solution[name]-value) for name,value in zip(names,seed)) if success else None)
            except Exception as exc:
                row.update(error=str(exc),collision_aware_endpoint_solution=False,
                    elapsed_sec=time.monotonic()-before, within_96_query_3s_completed_budget=False)
                rows.append(row)
                stream.write(json.dumps(row,ensure_ascii=False)+'\n')
                break
            rows.append(row)
            stream.write(json.dumps(row,ensure_ascii=False)+'\n')
    description_after = hashlib.sha256(rospy.get_param('/robot_description').encode()).hexdigest()
    bounded = [row for row in rows if row['within_96_query_3s_completed_budget']]
    result = dict(scope='Read-only /compute_ik endpoint query against current collision scene; historical measured seed, not historical scene replay or full path validation.',
        audited_at=datetime.now(timezone.utc).isoformat(), robot_description_sha256=EXPECTED,
        robot_description_sha256_after=description_after, model_unchanged=description_after==EXPECTED,
        input_sources={str(path):sha(path) for path in (baseline_path,candidate_path,input_path,audit_path)},
        replay_crosscheck=dict(recorded_original_count=106,baseline_exact_recorded_poses=True,
            candidate_count=218,all_original_106_exact_poses_preserved=True,new_endpoint_count=112),
        query=dict(service='/compute_ik',group='alicia',ik_link='tool0',avoid_collisions=True,
            timeout_sec=.02,historical_seed_joint_names=names,historical_seed_positions=seed,right_finger=.04975,
            ordering='round-robin distance/tilt groups; roll 0,-15,+15,-30,+30,...,-90,+90,180',
            repeated_seeds_or_retries=False),
        original_budget_style_subset=dict(max_queries=96,max_elapsed_sec=3.,completed_queries=len(bounded),
            solutions=sum(row['collision_aware_endpoint_solution'] for row in bounded),
            limitations='Bounded diagnostic ordering, not a replay of the production scheduler; actual RPC overhead is included.'),
        wider_diagnostic=dict(queries=len(rows),solutions=sum(row['collision_aware_endpoint_solution'] for row in rows),
            elapsed_sec=time.monotonic()-started,not_online_budget=True,
            error_codes=dict(Counter(str(row.get('error_code','exception')) for row in rows))),
        successful_query_indices=[row['query_index'] for row in rows if row['collision_aware_endpoint_solution']],
        queries_file=str(out/'queries.jsonl'),queries_sha256=sha(out/'queries.jsonl'),
        current_scene_frozen=False,historical_scene_reconstructed=False,full_path_checked=False,
        motion_authorized=False,motion_commands_sent=False,temperature_queried=False)
    (out / 'result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(dict(output=str(out/'result.json'),sha256=sha(out/'result.json'),
        original_budget_style_subset=result['original_budget_style_subset'],wider_diagnostic=result['wider_diagnostic']),ensure_ascii=False))


if __name__ == '__main__':
    main()
