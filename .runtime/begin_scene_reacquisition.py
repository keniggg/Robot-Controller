"""Explicit board-removal transition; target reacquisition only, no motion."""
import os,json,time,hashlib
from datetime import datetime,timezone
from pathlib import Path
root=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
if not os.path.ismount('/media/zhuyupei/SanStand'):raise RuntimeError('SanStand absent')
out=root/'ros_log'/('board_removed_scene_transition_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ'))
out.mkdir();(out/'ros').mkdir();os.environ['ROS_LOG_DIR']=str(out/'ros')
import rospy
from std_msgs.msg import String
from alicia_grasp_modes.srv import SetGraspMode
from alicia_flexible_grasp_supervisor.msg import GraspState
rospy.init_node('explicit_board_removed_reacquisition',anonymous=True,disable_signals=True)
state=rospy.wait_for_message('/grasp/state',GraspState,timeout=3)
if state.active:raise RuntimeError('task must be idle')
old=rospy.get_param('/grasp_mode/selection')
status=json.loads(rospy.wait_for_message('/perception/unknown/detector_status',String,timeout=3).data)
record={'reason':'operator_confirmed_board_removed_only','operator_confirmation':'只移走了标定板，其余未动','motor_unchanged':True,'base_camera_unchanged':True,'prior_selection':old,'prior_detector_status':status,'prior_failures':['grasp_diagnostic_20260928T084245Z','grasp_two_stage_diagnostic_20260928T093247Z'],'old_hover_error_resolved':False,'new_board_observation_available':False,'old_board_pose_reuse_authorized':False,'motor_move_requested':False,'motion_requested':False,'source_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),'prior_centering':'bounded_motor_centering_20260928T141708Z_351496','started_ns':rospy.Time.now().to_nsec()}
(out/'transition.json').write_text(json.dumps(record,ensure_ascii=False,indent=2))
rospy.wait_for_service('/grasp_mode/select',timeout=3)
r=rospy.ServiceProxy('/grasp_mode/select',SetGraspMode)('unknown',old['strategy'])
record['response']={'success':r.success,'message':r.message}
record['new_selection']=rospy.get_param('/grasp_mode/selection')
(out/'transition.json').write_text(json.dumps(record,ensure_ascii=False,indent=2))
print('SESSION',out,flush=True);print(json.dumps({k:record[k] for k in ['response','prior_selection','new_selection','motion_requested','old_hover_error_resolved']},ensure_ascii=False),flush=True)
if not r.success:raise RuntimeError(r.message)
