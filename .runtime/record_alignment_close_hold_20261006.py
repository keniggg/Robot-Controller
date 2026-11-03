"""Bind the actual latest user reply to newly received, read-only SDK feedback."""
from datetime import datetime, timezone
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import sys
import time

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

WORK=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('retry_guard',WORK/'tools/diagnostics/prepare_rgb_acquisition_retry.py')
guard=importlib.util.module_from_spec(spec);spec.loader.exec_module(guard)
MOUNT=Path('/media/zhuyupei/SanStand')
OUT=MOUNT/'实验记录/Robot-Controller/ros_log'/('manual_alignment_close_hold_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ'))
if not MOUNT.is_mount():
    raise RuntimeError('SanStand unavailable; no fallback')
OUT.mkdir(exist_ok=False)
os.environ["ROS_LOG_DIR"] = str(OUT/"ros")
rospy.init_node('record_actual_user_alignment',anonymous=True,disable_signals=True)
recorded=rospy.Time.now().to_nsec()
stamp=datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
# This is the exact reply received in the current conversation, not a generated
# confirmation or a new grant of motion permission.
origin=dict(kind='user_message',message_id=json.dumps(
    ['request_user_input_async','call_28FuJxiklpvrxcxtAWkYAXpX',0]),
    text='已换好新目标并对准，可以抓取')
deadline=time.monotonic()+5
while True:
    message=rospy.wait_for_message('/alicia_d/accepted_joint_states',JointState,timeout=3)
    values=dict(zip(message.name,message.position))
    if message.header.frame_id=='sdk_measured' and message.header.stamp.to_nsec()>=recorded:
        feedback=dict(stamp_ns=str(message.header.stamp.to_nsec()),frame_id='sdk_measured',
            positions=[values[name] for name in guard.JOINT_NAMES])
        guard._sample(feedback)
        break
    if time.monotonic()>deadline:
        raise RuntimeError('new SDK feedback after reply unavailable')
epoch=rospy.wait_for_message('/alicia_d/control_reference_epoch',Header,timeout=3)
if epoch.frame_id!='sdk_reference_epoch':
    raise RuntimeError('unexpected reference epoch')
context=guard.retry_source_context(rospy.get_param('/grasp_mode/selection'),epoch.stamp.to_nsec(),
    rospy.get_param('/camera'),rospy.get_param('/handeye'),rospy.get_param('/robot_description'))
event=dict(confirmation_source=origin,confirmed_ns=str(recorded),
    confirmation_time_semantics='local capture of the actual reply; not a claimed chat transport timestamp',
    target_aligned=True,source_context=context,bound_feedback=feedback,
    robot_commands_sent=False,torque_commands_sent=False)
event_path=OUT/('explicit_alignment_event_'+stamp+'.json')
event_path.write_text(json.dumps(event,ensure_ascii=False,indent=2)+'\n')
receipt=dict(version=1,kind='explicit_user_target_alignment',target_aligned=True,
    confirmation_id='call_28FuJxiklpvrxcxtAWkYAXpX:0',confirmation_source=origin,
    confirmed_ns=str(recorded),selection=context['selection'],epoch_ns=str(context['epoch_ns']),
    bound_feedback=feedback,source_context_sha256=guard.digest(context),
    event_file=str(event_path),event_file_sha256=hashlib.sha256(event_path.read_bytes()).hexdigest(),
    recorded_at_utc=datetime.now(timezone.utc).isoformat())
receipt_path=OUT/('alignment_receipt_'+stamp+'.json')
receipt_path.write_text(json.dumps(receipt,ensure_ascii=False,indent=2)+'\n')
(WORK/'.runtime/current_alignment_close_hold_20261006.json').write_text(json.dumps(
    dict(receipt=str(receipt_path),confirmation_id=receipt['confirmation_id']),indent=2)+'\n')
print(json.dumps(dict(receipt=str(receipt_path),selection=context['selection'],
    epoch_ns=context['epoch_ns'],feedback=feedback),ensure_ascii=False,indent=2))
