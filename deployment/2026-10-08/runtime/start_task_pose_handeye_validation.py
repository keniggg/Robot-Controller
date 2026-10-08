from pathlib import Path
from datetime import datetime,timezone
import json,os,subprocess,rospy,hashlib
root=Path.cwd();mount=Path('/media/zhuyupei/SanStand');assert os.path.ismount(str(mount))
tag=datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ');session=mount/'实验记录/Robot-Controller/ros_log'/('handeye_task_pose_validation_'+tag);session.mkdir();(session/'ros').mkdir()
rospy.init_node('start_task_pose_handeye_validation',anonymous=True,disable_signals=True)
ns='/d405_cc200_task_verify_'+tag+'_eye_on_hand';board='charuco_task_verify_'+tag
for k,v in dict(move_group_namespace='/',move_group='alicia',eye_on_hand=True,robot_base_frame='base_link',robot_effector_frame='tool0',tracking_base_frame='camera_link',tracking_marker_frame=board,freehand_robot_movement=True).items():rospy.set_param(ns+'/'+k,v)
env=os.environ.copy();env.update(ROS_LOG_DIR=str(session/'ros'),OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',PYTHONUNBUFFERED='1')
wrapper=str(root/'tools/with_local_realsense_sdk.sh')
tracker=[wrapper,'/usr/bin/python3','-u',str(root/'src/real-arm/alicia_d_calibration/scripts/charuco_tracker.py'),'__name:=charuco_task_pose_verification','_board_size:=[12,9]','_square_length:=0.015','_marker_length:=0.01125','_dictionary_id:=DICT_5X5_100','_camera_frame:=camera_link','_board_frame:='+board,'_image_topic:=/supervisor/camera/color/image_raw','_intrinsics_source:=realsense_runtime','_camera_parameter_namespace:=/camera','_quality_topic:=/charuco/quality','_max_input_age_sec:=0.35','_detection_scale:=1.5']
server=[wrapper,'/usr/bin/python3','-u',str(root/'tools/handeye_exact_sample_server.py'),'__ns:='+ns,'__name:=exact_sample_server','_session_dir:='+str(session),'_gui_alias:=/d405_cc200_recalib_20260809_eye_on_hand','_pose_change_source:=assistant_automatic','_sample_role:=validation']
processes={}
for name,args in [('tracker',tracker),('sampler',server)]:
 with (session/(name+'_stdout.log')).open('ab') as h:p=subprocess.Popen(args,env=env,stdout=h,stderr=subprocess.STDOUT,start_new_session=True)
 processes[name]=dict(pid=p.pid,args=args)
p=Path('/home/zhuyupei/.ros/easy_handeye/d405_cc200_auto_20260927_validated_eye_on_hand.yaml')
config=dict(session=str(session),namespace=ns,board_frame=board,processes=processes,pose_changes_authorized=True,positive_enable_only=True,validation_only=True,frozen_handeye=rospy.get_param('/handeye'),frozen_yaml_sha256=hashlib.sha256(p.read_bytes()).hexdigest(),training_session='/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/handeye_auto_20260927T102700Z',purpose='Independent task-pose invariance and depth-vs-board consistency; no refit or replacement')
(session/'automation_setup.json').write_text(json.dumps(config,ensure_ascii=False,indent=2)+'\n');(root/'.runtime/current_handeye_task_pose_validation.json').write_text(json.dumps(config,ensure_ascii=False,indent=2)+'\n')
print(json.dumps(config,ensure_ascii=False,indent=2),flush=True)
