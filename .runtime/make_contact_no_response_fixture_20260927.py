#!/usr/bin/env python3
"""Produce a small source fixture from already-extracted, original bag evidence."""
import hashlib
import json
from pathlib import Path

HERE=Path(__file__).resolve().parents[1]
OUT=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/unknown_grasp_20260923/approach_failure_20260927')
RECORD=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/grasp_records/20260927T063422.563534Z_094d509c116648378bf54b272bcc3489')
d=json.loads((OUT/'bag_forensics.json').read_text())
fk=json.loads((OUT/'fk_replay.json').read_text())
summary=json.loads((OUT/'bag_summary.json').read_text())
manifest=json.loads((RECORD/'manifest.json').read_text())
goals=d['events']['/alicia_controller/follow_joint_trajectory/goal']
goal=goals[-1]
result=d['events']['/alicia_controller/follow_joint_trajectory/result'][-1]
series=d['series']
start,end=goal['t'],result['t']
def state(at):
    value={}
    for key,topic in [('sdk','/alicia_d/sdk_command'),('accepted','/alicia_d/accepted_joint_states')]:
        row=min(series[topic],key=lambda r:abs(r['t']-at))
        value[key+'_stamp_ns']=round(row['stamp']*1e9)
        value[key+'_bag_time_sec']=row['t']
        value[key+'_positions_rad']=row['position'][:6]
    return value

fixture={'schema':1,
 'evidence_kind':'recorded_hardware_command_and_accepted_encoder_samples_not_simulated_response',
 'source':{'record_id':manifest['record_id'],'bag_path':str(RECORD/'keyframes.bag'),
           'bag_sha256_from_manifest':next(f['sha256'] for f in manifest['files'] if f['path']=='keyframes.bag'),
           'manifest_sha256':hashlib.sha256((RECORD/'manifest.json').read_bytes()).hexdigest(),
           'offline_extraction':'No ROS services, publications, or hardware motion used.'},
 'plan_id':fk['plan_id'],'model_sha256':fk['model_sha256'],
 'robot_description_fixture':'pregrasp_following_20260919.json',
 'joint_names':['Joint'+str(i) for i in range(1,7)],
 'epoch_ns':summary['epochs'][0]['msg']['stamp']['ns'],
 'epoch_count_in_record':len(summary['epochs']),
 'fixed_goal_pose_index':1,'fixed_goal_pose':fk['goal_poses'][1],
 'contact_pose_index':2,'contact_pose':fk['goal_poses'][2],
 'action_goal_id':goal['msg']['goal_id']['id'],
 'action_goal_stamp_ns':goal['msg']['goal_id']['stamp']['ns'],
 'action_result_stamp_ns':result['msg']['header']['stamp']['ns'],
 'trajectory_duration_sec':goal['msg']['goal']['trajectory']['points'][-1]['time_from_start']['sec'],
 'action_elapsed_sec':end-start,
 'command_start_positions_rad':goal['msg']['goal']['trajectory']['points'][0]['positions'],
 'command_goal_positions_rad':goal['msg']['goal']['trajectory']['points'][-1]['positions'],
 'goal_tolerance_rad':.035,'path_tolerance_defaults_rad':.12,'goal_time_sec':8.,
 'before':state(start-.1),'after':state(end-.05),
 'controller_result':result['msg']['result'],
 'observed_response':{'joint2_command_delta_counts':15,'joint2_accepted_delta_counts':0,
                       'joint2_max_sdk_minus_accepted_rad':.04908738521234046,
                       'accepted_sample_count':summary['/alicia_d/accepted_joint_states']['count'],
                       'joint_states_sample_count':summary['/joint_states']['count'],
                       'accepted_joint2_min_rad':summary['/alicia_d/accepted_joint_states']['min_j2'],
                       'accepted_joint2_max_rad':summary['/alicia_d/accepted_joint_states']['max_j2']},
 'fk_after':{'xyz_m':fk['states']['approach_failed']['measured_fk_xyz'],
             'quaternion_xyzw':fk['states']['approach_failed']['measured_fk_quaternion_xyzw'],
             'approach_goal_position_error_m':fk['states']['approach_failed']['residuals']['1']['position_m'],
             'approach_goal_orientation_error_rad':fk['states']['approach_failed']['residuals']['1']['orientation_rad'],
             'contact_goal_position_error_m':fk['states']['approach_failed']['residuals']['2']['position_m'],
             'following_contract_rejection':'PREGRASP_FOLLOWING_OUTSIDE_CONTRACT'},
 'health_evidence':{'protection_latched_values':summary['protection_values'],
                    'motion_enabled_values':summary['motion_enabled_values'],
                    'hardware_root_cause_established':False,
                    'sdk_transport_semantics':'crc_valid_serial_response_not_motion_ack'},
 'limits':'Encoder plateau demonstrates no observed Joint2 response to a reversing command; it does not establish whether the cause is mechanical deadband, actuator electronics, firmware, contact, or temperature. No physical recovery is proven.'}
target=HERE/'src/alicia_flexible_grasp_supervisor/tests/fixtures/contact_no_response_20260927.json'
target.write_text(json.dumps(fixture,indent=2)+'\n')
print(target, target.stat().st_size)
