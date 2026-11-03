"""Read-only endpoint IK evidence; never a motion request or path certificate."""
import json,copy,time,hashlib
from pathlib import Path
import numpy as np
import rospy
import tf
from alicia_flexible_grasp_supervisor.msg import GraspState
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionIK,GetPositionIKRequest
from moveit_msgs.msg import RobotState
from tf.transformations import quaternion_matrix,quaternion_from_matrix
from alicia_grasp_modes.observe_return_visual import directed_missing_side_rotations,missing_side_view_gain
from alicia_flexible_grasp.robot.observation_path_guard import SerialUrdfFk
OUT=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/resumed_observe_return_grasp_20260930T040740Z')
p=OUT/'gen23_live_no_plan_audit.json';raw=p.read_bytes();d=json.loads(raw)
(OUT/'gen23_ik_source_audit.json').write_bytes(raw)
rospy.init_node('missing_side_observation_ik_readonly',anonymous=True,disable_signals=True)
assert not rospy.wait_for_message('/grasp/state',GraspState,timeout=4).active
s=rospy.wait_for_message('/alicia_d/accepted_joint_states',JointState,timeout=4)
fk=SerialUrdfFk(rospy.get_param('/robot_description'),['Joint%d'%i for i in range(1,7)])
listener=tf.TransformListener();listener.waitForTransform('tool0','camera_link',rospy.Time(0),rospy.Duration(3))
t,q=listener.lookupTransform('tool0','camera_link',rospy.Time(0));tc=quaternion_matrix(q);tc[:3,3]=t;ct=np.linalg.inv(tc)
gaps=d['observe_return']['measured_missing_sides'];geom=d['replay_geometry'];center=np.asarray(geom['obb_center_base_m']);normal=np.asarray(geom['support_normal_base'])
rotation=np.asarray(d['snapshot_transform']['T_base_camera_link'])[:3,:3]
dirs=directed_missing_side_rotations(gaps,rotation,normal,.003,.22)
service=rospy.ServiceProxy('/compute_ik',GetPositionIK,persistent=True)
rows=[];started=time.monotonic()
for index,(r,metadata) in enumerate(dirs):
 for distance in [.26,.22,.30]:
  for roll in rospy.get_param('/grasp_6d/remote/observation_camera_roll_offsets_deg', [0.,15.,-15.,30.,-30.,45.,-45.,60.,-60.,75.,-75.,90.,-90.,180.]):
   rad=np.radians(roll);roll_m=np.array([[1,0,0],[0,np.cos(rad),-np.sin(rad)],[0,np.sin(rad),np.cos(rad)]])
   cam=np.eye(4);cam[:3,:3]=r@roll_m;cam[:3,3]=center-distance*r[:,0];tool=cam@ct
   req=GetPositionIKRequest();req.ik_request.group_name='alicia';req.ik_request.ik_link_name='tool0';req.ik_request.avoid_collisions=True;req.ik_request.timeout=rospy.Duration(.035);req.ik_request.pose_stamped.header.frame_id='base_link'
   for a,v in zip('xyz',tool[:3,3]):setattr(req.ik_request.pose_stamped.pose.position,a,float(v))
   for a,v in zip('xyzw',quaternion_from_matrix(tool)):setattr(req.ik_request.pose_stamped.pose.orientation,a,float(v))
   success=None;codes=[]
   for seed_name in ['current','joint_midpoints']:
    req.ik_request.robot_state=RobotState();req.ik_request.robot_state.joint_state=copy.deepcopy(s)
    if seed_name=='joint_midpoints':
     mids=dict(zip(['Joint%d'%i for i in range(1,7)],np.mean(fk.limits,axis=1)))
     req.ik_request.robot_state.joint_state.position=[mids.get(n,v) for n,v in zip(s.name,s.position)]
    res=service(req);codes.append(res.error_code.val)
    if res.error_code.val==1:
     success={'seed':seed_name,'names':list(res.solution.joint_state.name),'positions':list(res.solution.joint_state.position)};break
   row=dict(direction=index,tilt_deg=metadata['camera_rotation_offset_deg'],distance_m=distance,roll_deg=roll,tool_position=tool[:3,3].tolist(),tool_quaternion=quaternion_from_matrix(tool).tolist(),codes=codes,solution=success)
   rows.append(row)
   if success and sum(x['solution'] is not None for x in rows)<=12:print('IK_SUCCESS',index,round(row['tilt_deg'],3),distance,roll,success['seed'],flush=True)
report=dict(tool_from_camera=tc.tolist(),robot_description_sha256=fk.model_sha256,source_sha256=hashlib.sha256(raw).hexdigest(),selection=d['grasp_mode_selection'],snapshot_stamp_ns=d['snapshot_stamp_ns'],collision_aware_ik_only=True,motion_requested=False,path_verified=False,elapsed_sec=time.monotonic()-started,success_count=sum(x['solution'] is not None for x in rows),rows=rows)
(OUT/'gen23_missing_side_observation_ik_probe.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
print('READONLY_IK_COMPLETE',len(rows),report['success_count'],report['elapsed_sec'],flush=True)
