import json,pathlib,numpy as np
from tf.transformations import quaternion_matrix
from alicia_flexible_grasp.robot.observation_path_guard import SerialUrdfFk
from alicia_flexible_grasp.grasp.gripper_geometry import GripperGeometry,_stage_boxes,_box_corners
config=json.load(open('.runtime/current_handeye_task_pose_validation.json'));out=pathlib.Path(config['session']);capture=json.load(open(out/'capture_001/capture.json'));s=capture['samples'][capture['selected_frame_index']]
X=quaternion_matrix(config['frozen_handeye']['rotation_xyzw']);X[:3,3]=config['frozen_handeye']['translation_xyz'];Y=np.array(s['T_base_tool'])@X@np.array(s['T_camera_board']);latest=json.load(open(sorted(out.glob('capture_*/capture.json'))[-1])); latest_sample=latest['samples'][latest['selected_frame_index']]; q=np.array(latest_sample['accepted_joint_positions'][:6]);K=capture['camera_parameters_before'];n=Y[:3,2];n=n if n[2]>0 else -n;d=-n@Y[:3,3]
model=json.load(open('src/alicia_flexible_grasp_supervisor/tests/fixtures/measured_contact_20260926.json'));fk=SerialUrdfFk(model['robot_description'],model['joint_names']);g=GripperGeometry(.05,.002,np.array([.0434,.0286,.06]),np.array([.1175,.155,.0774]),.003);gap=s['accepted_joint_positions'][6]
C=np.array([[x,y,0,1] for x in [0,.18] for y in [0,.135]])
def geometry(v):
 T=fk(v);clear=min(float(np.min(_box_corners(c,T[:3,:3],size)@n+d)) for _,c,size in _stage_boxes(T,g,gap,'y','z'));B=np.linalg.inv(T@X)@Y;P=(B@C.T).T
 if np.min(P[:,0])<.12:return False,dict(reason='board too close/behind camera',minimum_gripper_plane_clearance_m=clear)
 uv=np.column_stack([K['cx']-K['fx']*P[:,1]/P[:,0],K['cy']-K['fy']*P[:,2]/P[:,0]]);edge=float(min(uv[:,0].min(),639-uv[:,0].max(),uv[:,1].min(),479-uv[:,1].max()))
 return clear>=.04 and edge>=20,dict(minimum_gripper_plane_clearance_m=clear,predicted_full_board_edge_px=edge,board_depth_min_m=float(P[:,0].min()))
# Prospective arm-flexion family, selected only by geometry, never by fit error.
family=[]
for delta2,delta3 in [(-8,12),(-16,24),(-24,36),(-32,48),(-40,60)]:
 options=[]
 for delta5 in np.arange(-64,17,2):
  delta=np.array([0,delta2,delta3,0,delta5,0],float);v=q+np.radians(delta);ok,info=geometry(v)
  if ok:options.append((info['predicted_full_board_edge_px'],v,delta,info))
 if options:
  _,v,delta,info=max(options,key=lambda z:z[0]);family.append(dict(pose_id=len(family)+1,q_rad=v.tolist(),offset_deg=delta.tolist(),geometry_eligible=True,geometry=info))
 else:print('FAMILY_NO_FULL_VIEW',delta2,delta3,flush=True)
# Reserve the return before any independent observation is acquired.
family.append(dict(pose_id=len(family)+1,q_rad=q.tolist(),offset_deg=[0.]*6,geometry_eligible=geometry(q)[0],geometry=geometry(q)[1],role='return'))
start=dict(T_tool_camera_historical=X.tolist(),T_base_board_initial_estimate=Y.tolist(),camera_intrinsics={k:K[k] for k in ['fx','fy','cx','cy']},frozen_reference_sample=s,baseline_capture=str(out/'capture_001/capture.json'))
(out/'starting_geometry.json').write_text(json.dumps(start,indent=2)+'\n');plan=dict(seed_q_rad=q.tolist(),frozen_candidate_before_samples=True,selection_basis='full-board projection and unchanged 40mm coarse gripper-plane reservation; no observed residual used',poses=family)
(out/'prospective_task_pose_plan.json').write_text(json.dumps(plan,indent=2)+'\n');print(json.dumps(plan,indent=2),flush=True)
