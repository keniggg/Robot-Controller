"""Save read-only exact audit linkage and implementation parity review."""
import hashlib
import json
import math
from pathlib import Path

W = Path(__file__).resolve().parents[1]
N = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_restart_20261010T002347Z')
O = N / 'gen20_analysis/held_approach_candidates'
R = N.parents[1] / 'grasp_records/20261010T084609.133186Z_f832fa3e55eb45a59d41fecf2246af73'
A = R / 'keyframes_audits/ff26e079f8ad2986b8ca3ed71a06571f866f4bf8f807d7ad5281e17beccc47d4.json'
def sha(p): return hashlib.sha256(p.read_bytes()).hexdigest()
def distance(a,b): return math.sqrt(sum((x-y)**2 for x,y in zip(a,b)))
def main():
    best=json.loads((O/'best_candidate.json').read_text())
    c=best['candidate']; audit=json.loads(A.read_text())
    assert audit['request_id']==13 and audit['snapshot_stamp_ns']==best['source_stamp_ns']
    matched=[(i,r) for i,r in enumerate(audit['lineage']) if c['plan_id'] in json.dumps(r)]
    assert len(matched)==1
    i,row=matched[0]; profile=row['adaptive_stage_profile']
    poses=c['contact']['poses_xyz_xyzw']
    failure=c['height_retry']['original_failure']; hp=failure['held_approach_prediction']
    pp=hp['predicted_pregrasp']; ap=hp['predicted_approach']
    assert pp['predicted_positions_rad']==ap['virtual_start_positions_rad']
    assert distance(pp['fixed_goal_pose'],poses[0])<1e-12
    assert distance(ap['fixed_goal_pose'],poses[1])<1e-12
    profile_result=dict(schema='gen20_exact_best_candidate_profile_v1',
       plan_id=c['plan_id'],audit_path=str(A),audit_sha256=sha(A),lineage_index=i,
       adaptive_stage_profile=profile, execution_sequence=row['execution_sequence'],
       moveit_input_joint_state=row['moveit_input_joint_state'],
       poses_xyz_xyzw=poses, pregrasp_to_approach_distance_m=distance(poses[0][:3],poses[1][:3]),
       computed_minimum_pregrasp_from_source_defaults_m=max(.025,profile['approach_offset_m']+.005+profile['execution_position_error_m']),
       source_defaults=dict(pregrasp_min_m=.025,pregrasp_approach_gap_min_m=.005),
       historical_remote_configuration_limitation='The record captures /grasp and multiview but not the whole /grasp_6d/remote/adaptive_stage_generation dictionary. The source-default formula exactly reproduces the logged generated distance; the .005 value is not separately claimed to be a recorded ROS parameter.',
       captured_grasp_configuration=str(R/'perception_execution_configuration.json'))
    out=O/'best_candidate_stage_profile.json';out.write_text(json.dumps(profile_result,indent=2)+'\n')
    paths=[
        'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot/contact_sequence_precheck.py',
        'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot/held_axes_prediction.py',
        'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot/pregrasp_compensation.py',
        'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot/pregrasp_feedforward.py',
        'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot/pregrasp_gateway.py',
        'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/grasp/adaptive_stage_profiles.py',
        'src/alicia_flexible_grasp_supervisor/scripts/grasp_task_node.py',
        'src/alicia_flexible_grasp_supervisor/scripts/remote_grasp6d_node.py']
    report=dict(schema='gen20_held_approach_readonly_independent_review_v1',
      source_sha256={p:sha(W/p) for p in paths}, best_fixture=dict(path=str(O/'best_candidate.json'),sha256=sha(O/'best_candidate.json')),
      profile=dict(path=str(out),sha256=sha(out)), findings=[
       'The contact precheck propagates predicted pregrasp command and actual endpoints into approach. The logged predicted pregrasp actual vector exactly equals the approach virtual-start vector.',
       'Precheck and execution share predict_held_axes_endpoint and remaining_budget_guide; the execution wrapper passes its fresh episode guide with zero steps and unit response gains. Both hold Joint2/Joint5, bound total delta to 32 SDK counts, and require approach <=6 mm / <=5 degrees.',
       'remaining_budget_guide is eight-pass fixed-order coordinate descent on an even-count lattice. It explicitly does not prove global infeasibility. A failing 9.543 mm guide is not a proof that no legal pregrasp or same-budget coupled solution exists.',
       'The logged pregrasp distance equals the source-default minimum approach +5 mm + measured execution error. A shorter pregrasp would change that geometric policy and cannot silently be described as retaining all original bounds.',
       'Planning predicts constant current SDK-minus-encoder bias at virtual future poses. Execution must recapture actual stationary feedback and validate the complete command path and measured arrival; offline endpoint success would not establish executable or physically successful grasp.'
      ],diagnostic=dict(approach_position_error_m=ap['predicted_position_error_m'],approach_orientation_error_rad=ap['predicted_orientation_error_rad'],predicted_pregrasp_actual_equals_approach_start=True),
      no_production_edits=True,no_ros_or_hardware_access=True,no_collision_or_execution_success_claim=True)
    path=O/'algorithm_independent_review.json';path.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({'report':str(path),'sha256':sha(path),'profile':str(out),'profile_sha256':sha(out),'pregrasp_to_approach_m':profile_result['pregrasp_to_approach_distance_m']}))
if __name__=='__main__':main()
