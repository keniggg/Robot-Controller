from pathlib import Path
from datetime import datetime,timezone
import json,os,subprocess,rospy
ROOT=Path.cwd();mount=Path('/media/zhuyupei/SanStand');assert os.path.ismount(str(mount))
tag=datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ');session=mount/'实验记录/Robot-Controller/ros_log'/('handeye_auto_'+tag);session.mkdir();(session/'ros').mkdir()
rospy.init_node('start_auto_handeye_session',anonymous=True,disable_signals=True)
namespace='/d405_cc200_auto_'+tag+'_eye_on_hand';board='charuco_auto_'+tag
common=dict(move_group_namespace='/',move_group='alicia',eye_on_hand=True,robot_base_frame='base_link',robot_effector_frame='tool0',tracking_base_frame='camera_link',tracking_marker_frame=board,freehand_robot_movement=True)
for k,v in common.items():rospy.set_param(namespace+'/'+k,v)
env=os.environ.copy();env['ROS_LOG_DIR']=str(session/'ros');env['OPENBLAS_NUM_THREADS']='1';env['OMP_NUM_THREADS']='1';env['PYTHONUNBUFFERED']='1'
wrapper=str(ROOT/'tools/with_local_realsense_sdk.sh')
tracker=[wrapper,'/usr/bin/python3','-u',str(ROOT/'src/real-arm/alicia_d_calibration/scripts/charuco_tracker.py'),'__name:=charuco_current_sdk_auto','_board_size:=[12,9]','_square_length:=0.015','_marker_length:=0.01125','_dictionary_id:=DICT_5X5_100','_camera_frame:=camera_link','_board_frame:='+board,'_image_topic:=/supervisor/camera/color/image_raw','_intrinsics_source:=realsense_runtime','_camera_parameter_namespace:=/camera','_quality_topic:=/charuco/quality','_max_input_age_sec:=0.35','_detection_scale:=1.0']
server=[wrapper,'/usr/bin/python3','-u',str(ROOT/'tools/handeye_exact_sample_server.py'),'__ns:='+namespace,'__name:=exact_sample_server','_session_dir:='+str(session),'_gui_alias:=/d405_cc200_recalib_20260809_eye_on_hand','_pose_change_source:=assistant_automatic','_sample_role:=training']
processes={}
for name,args in [('tracker',tracker),('sampler',server)]:
 with (session/(name+'_stdout.log')).open('ab') as log:
  p=subprocess.Popen(args,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
 processes[name]={'pid':p.pid,'args':args}
config={'session':str(session),'namespace':namespace,'board_frame':board,'processes':processes,'pose_changes_authorized':True,'positive_enable_only':True,'holdout_protocol':'Training group set first; fit and freeze candidate; collect separate validation group session before deployment. No thresholds relaxed; no post-hoc sample exclusions for pass.'}
(session/'automation_setup.json').write_text(json.dumps(config,ensure_ascii=False,indent=2)+'\n')
(ROOT/'.runtime/current_handeye_autocalibration.json').write_text(json.dumps(config,ensure_ascii=False,indent=2)+'\n')
print(json.dumps(config,ensure_ascii=False,indent=2))
