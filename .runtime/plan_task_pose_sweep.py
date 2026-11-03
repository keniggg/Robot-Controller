import json,pathlib,numpy as np
from tf.transformations import quaternion_matrix
from alicia_flexible_grasp.robot.observation_path_guard import SerialUrdfFk,ObservationPathError
from alicia_flexible_grasp.grasp.gripper_geometry import GripperGeometry,_stage_boxes,_box_corners
ROOT=pathlib.Path(__file__).resolve().parents[1]
config=json.load(open(ROOT/'.runtime/current_handeye_task_pose_validation.json'));out=pathlib.Path(config['session']);start=json.load(open(out/'starting_geometry.json'))
X=np.array(start['T_tool_camera_historical']);Y=np.array(start['T_base_board_initial_estimate']);K=start['camera_intrinsics'];n=Y[:3,2].copy();n=n if n[2]>0 else -n;d=-n@Y[:3,3]
seed_capture=json.load(open(out/'capture_002/capture.json'));s=seed_capture['samples'][seed_capture['selected_frame_index']];seed=np.array(s['accepted_joint_positions'][:6]);gap=s['accepted_joint_positions'][6]
fk=SerialUrdfFk((out/'robot_description.urdf').read_text(),['Joint%d'%i for i in range(1,7)]);g=GripperGeometry(.05,.002,np.array([.0434,.0286,.06]),np.array([.1175,.155,.0774]),.003);C=np.array([[x,y,0,1] for x in [0,.18] for y in [0,.135]])
def geometry(q):
 try:T=fk(q)
 except ObservationPathError as exc:return False,dict(reason=str(exc),clearance_m=-1.,edge_px=-1000.)
 clear=min(float(np.min(_box_corners(c,T[:3,:3],size)@n+d)) for _,c,size in _stage_boxes(T,g,gap,'y','z'));P=(np.linalg.inv(T@X)@Y@C.T).T
 if np.min(P[:,0])<.12:return False,dict(reason='board too close/behind camera',clearance_m=clear,edge_px=-1000)
 uv=np.column_stack([K['cx']-K['fx']*P[:,1]/P[:,0],K['cy']-K['fy']*P[:,2]/P[:,0]]);edge=float(min(uv[:,0].min(),639-uv[:,0].max(),uv[:,1].min(),479-uv[:,1].max()))
 return clear>=.04 and edge>=20,dict(clearance_m=clear,edge_px=edge,depth_min_m=float(P[:,0].min()))
def segment(a,b):
 rows=[geometry(a*(1-t)+b*t) for t in np.linspace(0,1,17)]
 return all(v[0] for v in rows),dict(clearance_m=min(v[1]['clearance_m'] for v in rows),edge_px=min(v[1]['edge_px'] for v in rows))
if __name__=='__main__':
 poses=[]
 # Initial lift/tilt then increase elbow flexion while centering full board.
 for d2,d3 in [(8,0),(12,12),(4,24),(-4,36),(-12,48),(-20,60),(-28,72)]:
  opts=[]
  for d5 in range(-124,9,2):
   delta=np.array([0,d2,d3,0,d5,0]);q=seed+np.radians(delta);ok,info=geometry(q)
   if ok:opts.append((info['edge_px'],q,delta,info))
  if not opts: print('NO_FEASIBLE_FAMILY',d2,d3,flush=True);continue
  _,q,delta,info=max(opts,key=lambda v:v[0]);poses.append(dict(pose_id=len(poses)+1,q_rad=q.tolist(),offset_deg=delta.tolist(),geometry=info,role='task_flexion_validation'))
 # Return over the same prospective family, then independent wrist-axis views.
 for p in list(reversed(poses[:-1])):
  p=dict(p);p['pose_id']=len(poses)+1;p['role']='return_path_validation';poses.append(p)
 if poses:
  broad=np.array(poses[-1]['q_rad'])
  for j,degrees in [(3,-12),(3,12),(5,-12),(5,12),(0,-6)]:
   for return_base in [False,True]:
    q=broad.copy()
    if not return_base:q[j]+=np.radians(degrees)
    ok,info=geometry(q)
    if ok:poses.append(dict(pose_id=len(poses)+1,q_rad=q.tolist(),offset_deg=np.degrees(q-seed).tolist(),geometry=info,role='wrist_axis_validation' if not return_base else 'wrist_return'))
 ok,info=geometry(seed);poses.append(dict(pose_id=len(poses)+1,q_rad=seed.tolist(),offset_deg=[0.]*6,geometry=info,role='return_to_capture002'))
 plan=dict(seed_q_rad=seed.tolist(),frozen_handeye_sha256=config['frozen_yaml_sha256'],reference_capture='capture_001',selection_basis='prospective geometry only, before residual analysis; no refit or recentering',poses=poses)
 (out/'prospective_flexion_sweep.json').write_text(json.dumps(plan,indent=2)+'\n')
 print(json.dumps(plan,indent=2),flush=True)
