from pathlib import Path
import importlib.util,json
import numpy as np
import rospy,rosgraph
from tf.transformations import quaternion_matrix
from alicia_flexible_grasp.grasp.gripper_geometry import GripperGeometry,_stage_boxes,_box_corners
root=Path.cwd();src=root/'src/real-arm/alicia_d_calibration/scripts/charuco_tracker.py'
spec=importlib.util.spec_from_file_location('proposal_camera',src);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
base=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/current_grasp_preview_20260929T014236Z')
plan=json.loads((base/'preview_1790646187377696275.json').read_text())
params=m.read_runtime_camera_parameters('/camera','/readonly_frozen_goal_geometry',rosgraph.get_master_uri());ev=m.runtime_color_model_evidence(params,'camera_link','/supervisor/camera/color/image_raw');model=m.RuntimeColorModel(ev)
he=rospy.get_param('/handeye');X=quaternion_matrix(he['rotation_xyzw']);X[:3,3]=he['translation_xyz']
cfg=rospy.get_param('/grasp_6d/remote/gripper_geometry');g=GripperGeometry(cfg['max_inner_gap_m'],cfg['width_safety_margin_per_side_m'],np.array(cfg['finger_box_xyz_m']),np.array(cfg['palm_box_xyz_m']),.003)
def mat(p):
 T=quaternion_matrix([p['orientation'][k] for k in 'xyzw']);T[:3,3]=[p['position'][k] for k in 'xyz'];return T
geo=plan['object_geometry'];O=mat(geo['pose_base']);size=np.array([geo['size_xyz_m'][k] for k in 'xyz']);corners=_box_corners(O[:3,3],O[:3,:3],size)
n=np.array([geo['support_normal_base'][k] for k in 'xyz']);d=geo['support_offset_m'];T=mat(plan['poses'][0]);rows=[]
for h in [0,.01,.02]:
 goal=T.copy();goal[:3,3]+=h*n;C=goal@X;pts=(corners-C[:3,3])@C[:3,:3];pc=(O[:3,3]-C[:3,3])@C[:3,:3];opt=pts[:,[1,2,0]]*[-1,-1,1];opc=pc[[1,2,0]]*[-1,-1,1];uv=model.project(opt,[0,0,0],[0,0,0]);centeruv=model.project([opc],[0,0,0],[0,0,0])[0]
 clear={}
 for opening in [.04975,plan['required_open_width_m']]:
  clear[str(opening)]={name:float(np.min(_box_corners(center,goal[:3,:3],sz)@n+d)) for name,center,sz in _stage_boxes(goal,g,opening,'y','z')}
 rows.append(dict(normal_raise_m=h,T_base_tool=goal.tolist(),T_base_camera=C.tolist(),camera_link_center_m=pc.tolist(),optical_depth_range_m=[float(min(opt[:,2])),float(max(opt[:,2]))],object_center_pixel=centeruv.tolist(),obb_corner_bbox_xyxy=[float(min(uv[:,0])),float(min(uv[:,1])),float(max(uv[:,0])),float(max(uv[:,1]))],corner_min_image_margin_px=float(min(np.min(uv[:,0]),np.min(uv[:,1]),639-np.max(uv[:,0]),479-np.max(uv[:,1]))),cad_support_clearance_m=clear,obb_corner_uv=uv.tolist()))
report=dict(scope='frozen_plan_proposal_only_not_current_target_or_motion_authority',plan_id=plan['plan_id'],plan_stamp=plan['header']['stamp'],handeye=he,T_tool_camera=X.tolist(),camera_model=ev,gripper_config=cfg,rows=rows,motion_authorized=False)
(base/'readonly_pregrasp_goal_geometry.json').write_text(json.dumps(report,indent=2,ensure_ascii=False)+'\n')
for row in rows:print(json.dumps({k:v for k,v in row.items() if k not in ('T_base_tool','T_base_camera','obb_corner_uv')},ensure_ascii=False))
print(base/'readonly_pregrasp_goal_geometry.json')
