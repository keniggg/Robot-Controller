"""Initialize the observation planner on the current measured pose, without execution."""
import argparse,json,os,time
from pathlib import Path

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--output',type=Path,required=True);a=ap.parse_args()
    assert Path('/media/zhuyupei/SanStand').is_mount();a.output.mkdir(exist_ok=False);os.environ['ROS_LOG_DIR']=str(a.output/'ros')
    import rospy,tf2_ros
    from geometry_msgs.msg import PoseStamped
    from alicia_flexible_grasp_supervisor.msg import GraspState
    from alicia_flexible_grasp_supervisor.srv import SetTargetPose
    rospy.init_node('warm_current_observation_planner_readonly',anonymous=True,disable_rosout=True)
    assert not rospy.wait_for_message('/grasp/state',GraspState,timeout=5).active
    buffer=tf2_ros.Buffer();listener=tf2_ros.TransformListener(buffer)
    tf=buffer.lookup_transform('base_link','tool0',rospy.Time(0),rospy.Duration(3))
    pose=PoseStamped();pose.header=tf.header;pose.pose.position.x=tf.transform.translation.x;pose.pose.position.y=tf.transform.translation.y;pose.pose.position.z=tf.transform.translation.z;pose.pose.orientation=tf.transform.rotation
    rospy.wait_for_service('/supervisor/check_observation_pose_strict',timeout=10)
    start=time.monotonic();reply=rospy.ServiceProxy('/supervisor/check_observation_pose_strict',SetTargetPose)(pose,False)
    result=dict(execution_requested=False,goal_kind='current_measured_tool_pose',source_stamp_ns=pose.header.stamp.to_nsec(),elapsed_sec=time.monotonic()-start,service_success=reply.success,message=reply.message,authorizes_grasp=False)
    (a.output/'result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n');print(json.dumps({k:v for k,v in result.items() if k!='message'},ensure_ascii=False),flush=True)
if __name__=='__main__':main()
