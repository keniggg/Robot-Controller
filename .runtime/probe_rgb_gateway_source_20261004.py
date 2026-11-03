"""Read-only live source/API compatibility probe; no gateway init or actuator API."""
from collections import deque, OrderedDict
from datetime import datetime, timezone
import json
from pathlib import Path
import threading
import time

import rospy
import tf2_ros
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Header, String
from alicia_flexible_grasp.robot.rgb_acquisition_gateway import RgbAcquisitionGateway

OUT=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_ros_resume_20261004T002930Z')
if not Path('/media/zhuyupei/SanStand').is_mount():
    raise RuntimeError('SanStand unavailable')
rospy.init_node('rgb_gateway_source_readonly_probe',anonymous=True,disable_signals=True)
node=RgbAcquisitionGateway()
node._rgb_lock=threading.RLock(); node._endpoint_feedback_lock=threading.RLock()
node._rgb_pairs=OrderedDict(); node._rgb_status=None; node._rgb_ready_status=None
node._rgb_reference_ack=None; node._rgb_bridge=CvBridge(); node._control_reference_epoch_ns=None
node._endpoint_feedback_history={'accepted':deque(maxlen=128)}
node._observation_view_tf_buffer=tf2_ros.Buffer(rospy.Duration(10))
listener=tf2_ros.TransformListener(node._observation_view_tf_buffer)
positions=[]
node._current_arm_positions_snapshot=lambda:list(positions)
def joint(message):
    named=dict(zip(message.name,message.position))
    if message.header.frame_id!='sdk_measured' or any('Joint%d'%i not in named for i in range(1,7)):
        return
    with node._endpoint_feedback_lock:
        node._endpoint_feedback_history['accepted'].append(message)
        positions[:]=[named['Joint%d'%i] for i in range(1,7)]
def epoch(message):
    if message.frame_id=='sdk_reference_epoch':
        node._control_reference_epoch_ns=message.stamp.to_nsec()
subs=[rospy.Subscriber('/supervisor/camera/'+stream+'/image_raw',Image,
    lambda msg,kind=kind:node._rgb_acquisition_image_cb(msg,kind),queue_size=2,buff_size=2**24)
    for stream,kind in (('color','rgb'),('depth','depth'))]
subs += [rospy.Subscriber('/perception/unknown/detector_status',String,node._rgb_acquisition_status_cb,queue_size=2),
    rospy.Subscriber('/alicia_d/accepted_joint_states',JointState,joint,queue_size=50),
    rospy.Subscriber('/alicia_d/control_reference_epoch',Header,epoch,queue_size=2)]
deadline=time.monotonic()+15; reasons={}; accepted=[]; seen=set()
while time.monotonic()<deadline and len(accepted)<3:
    time.sleep(.05)
    with node._rgb_lock:
        status=node._rgb_ready_status
    if not status:
        continue
    interest=status['observation_interest']
    if interest['source_stamp_ns'] in seen:
        continue
    try:
        started=time.monotonic()
        value=node._rgb_capture(interest['source_id'])
        accepted.append(dict(source_stamp_ns=value['source_stamp_ns'],source_id=value['source_id'],
            source_rgb_sha256=value['source_rgb_sha256'],source_depth_sha256=value['source_depth_sha256'],
            capture_elapsed_sec=time.monotonic()-started))
        seen.add(value['source_stamp_ns'])
    except Exception as exc:
        reasons[str(exc)]=reasons.get(str(exc),0)+1
report=dict(read_only=True,robot_commands_sent=False,accepted=accepted,reasons=reasons)
path=OUT/('rgb_gateway_source_probe_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')+'.json')
path.write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(dict(report,path=str(path)),indent=2),flush=True)
raise SystemExit(0 if len(accepted)>=3 else 2)
