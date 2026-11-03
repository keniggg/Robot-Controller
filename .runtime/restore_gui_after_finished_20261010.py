"""Return the GUI to manual after a finished recorded generation, without motion."""
import argparse,json,os,time
from pathlib import Path
ap=argparse.ArgumentParser();ap.add_argument('--generation',type=int,required=True);ap.add_argument('--output',type=Path,required=True);a=ap.parse_args()
mount=Path('/media/zhuyupei/SanStand');assert mount.is_mount() and mount in a.output.resolve().parents
a.output.mkdir(exist_ok=True);os.environ['ROS_LOG_DIR']=str(a.output/'manual_ros')
import rospy,cv2
from std_msgs.msg import Bool,String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from alicia_flexible_grasp_supervisor.msg import GraspState
rospy.init_node('restore_gui_after_finished_record',anonymous=True,disable_rosout=True)
assert rospy.get_param('/grasp_mode/selection')['generation']==a.generation
state=rospy.wait_for_message('/grasp/state',GraspState,timeout=5)
mode=json.loads(rospy.wait_for_message('/grasp_mode/status',String,timeout=5).data)
assert not state.active and not any(mode.get(k) for k in ('active','start_reserved','pending'))
p=rospy.Publisher('/gui/joint_direct_mode',Bool,queue_size=1,latch=True)
deadline=time.monotonic()+5
while not p.get_num_connections() and time.monotonic()<deadline:time.sleep(.05)
assert p.get_num_connections()
rospy.set_param('/gui/joint_direct_mode',True);p.publish(Bool(data=True));time.sleep(.3)
r=dict(generation=a.generation,verified_inactive=True,manual_restored=True,at_ns=rospy.Time.now().to_nsec(),motion_command_sent=False,torque_command_sent=False,temperature_queried=False)
(a.output/'manual_restored.json').write_text(json.dumps(r,indent=2)+'\n')
msg=rospy.wait_for_message('/supervisor/camera/color/image_raw',Image,timeout=5)
assert cv2.imwrite(str(a.output/'ended_scene.png'),CvBridge().imgmsg_to_cv2(msg,'bgr8'))
print(json.dumps(r))
