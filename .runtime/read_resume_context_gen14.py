"""Read current joint/GUI/image context; never certify a new user reply."""
import json
import os
from pathlib import Path
import time

root = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_no_skin_20261011T000625Z')
assert Path('/media/zhuyupei/SanStand').is_mount()
out = root / 'gen14_resume_1032'
out.mkdir(exist_ok=True)
os.environ['ROS_LOG_DIR'] = str(out / 'read_ros')
import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import JointState, Image
from std_msgs.msg import Header
from alicia_flexible_grasp_supervisor.msg import GraspState
rospy.init_node('resume_context_readonly', anonymous=True, disable_rosout=True)
q = rospy.wait_for_message('/alicia_d/accepted_joint_states', JointState, timeout=4)
epoch = rospy.wait_for_message('/alicia_d/control_reference_epoch', Header, timeout=4)
task = rospy.wait_for_message('/grasp/state', GraspState, timeout=4)
image = rospy.wait_for_message('/supervisor/camera/color/image_raw', Image, timeout=4)
old = json.loads((root / 'realigned_20261011T071421Z/alignment_receipt.json').read_text())
ref = old['topics']['/alicia_d/accepted_joint_states']
previous = dict(zip(ref['names'], ref['positions']))
result = dict(recorded_ns=time.time_ns(), names=list(q.name), positions=list(q.position),
    maximum_joint_delta_rad=max(abs(v-previous[n]) for n,v in zip(q.name,q.position) if n in previous),
    previous_epoch_ns=old['epoch_ns'], epoch_ns=epoch.stamp.to_nsec(),
    manual_mode=rospy.get_param('/gui/joint_direct_mode'), task_active=task.active,
    task_state=str(task.state), selection=rospy.get_param('/grasp_mode/selection'),
    joint_frame=q.header.frame_id, joint_stamp_ns=q.header.stamp.to_nsec(),
    image_stamp_ns=image.header.stamp.to_nsec(), new_confirmation_claimed=False,
    motion_sent=False, temperature_queried=False)
assert cv2.imwrite(str(out / 'current_scene.png'), CvBridge().imgmsg_to_cv2(image,'bgr8'))
(out / 'current_context.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
print(json.dumps(result,ensure_ascii=False))
