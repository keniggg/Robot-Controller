"""Read-only current IK scheduling replay; no path or execution qualification."""
import json,copy,hashlib
from pathlib import Path
from types import SimpleNamespace as NS
import rospy
import numpy as np
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_matrix
from alicia_grasp_modes.silhouette_observation_planning import SilhouetteObservationPlanningMixin
from alicia_grasp_modes.silhouette_observation_order import transform_fingerprint,endpoint_schedule_metrics
OUT=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/resumed_observe_return_grasp_20260930T040740Z')
source=json.loads((OUT/'gen23_missing_side_observation_ik_probe.json').read_text())
rospy.init_node('endpoint_schedule_replay_readonly',anonymous=True,disable_signals=True)
def pose_matrix(p):
    q=p.pose.orientation;pos=p.pose.position
    m=quaternion_matrix([q.x,q.y,q.z,q.w]);m[:3,3]=[pos.x,pos.y,pos.z];return m
class Probe(SilhouetteObservationPlanningMixin):
    def _require_unknown_observation_prepared(self,p):
        assert p is prepared and source['selection']['generation']==23
node=Probe();node._observe_return_original=NS(rospy=rospy,pose_matrix=pose_matrix)
node.grasp_config={'observation_camera_target_nominal_distance_m':.26}
node.latest_joint_state=rospy.wait_for_message('/alicia_d/accepted_joint_states',JointState,timeout=4)
prepared=NS(ticket=NS(generation=23,target_epoch=3,request_id=246),snapshot=NS(stamp_ns=source['snapshot_stamp_ns'],target_identity=NS(track_id='gen23-diagnostic-replay')))
variants=[]
for i,row in enumerate(source['rows']):
    pose=PoseStamped();pose.header.frame_id='base_link';pose.header.stamp=rospy.Time.from_sec(source['snapshot_stamp_ns']/1e9)
    for k,v in zip('xyz',row['tool_position']):setattr(pose.pose.position,k,v)
    for k,v in zip('xyzw',row['tool_quaternion']):setattr(pose.pose.orientation,k,v)
    variants.append(dict(sequence=NS(pregrasp=pose),camera_roll_offset_deg=row['roll_deg'],camera_target_distance_m=row['distance_m'],camera_tilt_offset_deg=row['tilt_deg'],preference_index=i,observation_envelope=NS(minimum_support_clearance_m=0.)))
hints,report=node._observation_endpoint_search_hints(variants,prepared)
winners=[]
for v in variants:
    h=hints.get(transform_fingerprint(pose_matrix(v['sequence'].pregrasp)),{})
    if h.get('status')=='endpoint_found':winners.append(dict(roll=v['camera_roll_offset_deg'],distance=v['camera_target_distance_m'],tilt=v['camera_tilt_offset_deg'],**h))
winners.sort(key=lambda h:(h['joint_max_delta_rad'],h['joint_l1_delta_rad']))
result=dict(read_only=True,planning_authority=False,motion_requested=False,endpoint_CAD_not_replayed=True,path_verified=False,diagnostic_source_generation=23,report=report,ranked_hints=winners)
(OUT/'gen23_endpoint_hint_live_replay.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
print('PROBE_SUMMARY',report['checked_count'],report['hinted_count'],report['used_sec'],report['status'])
print('TOP_HINTS',json.dumps([{k:r[k] for k in ['roll','distance','tilt','joint_max_delta_rad','joint_l1_delta_rad']} for r in winners[:4]]))
