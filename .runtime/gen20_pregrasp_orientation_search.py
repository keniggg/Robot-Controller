"""Offline new pregrasp waypoint proposal; all fixed target approach gates kept."""
exec(open(__file__.replace('gen20_pregrasp_orientation_search.py','gen20_held_search.py')).read().split('for r in records[:3]:')[0])
from scipy.optimize import least_squares
from tf.transformations import quaternion_from_matrix, rotation_matrix
from alicia_flexible_grasp.robot.held_axes_prediction import predict_held_axes_endpoint,HeldAxesPredictionError
r=records[0]; h=r['precheck']['held_approach_prediction']; p=h['predicted_pregrasp']; a=h['predicted_approach']
pg=pose_matrix(p['fixed_goal_pose']); ag=pose_matrix(a['fixed_goal_pose']); q0=np.asarray(h['nominal_pregrasp_command_rad']); offset=np.asarray(h['sdk_minus_encoder_rad'])
def inverse(target,seed):
 def err(q):
  t=fk(q); dr=target[:3,:3].T@t[:3,:3]
  return np.r_[(t[:3,3]-target[:3,3])*10,(dr[2,1]-dr[1,2])/2,(dr[0,2]-dr[2,0])/2,(dr[1,0]-dr[0,1])/2]
 z=least_squares(err,seed,bounds=(fk.limits[:,0]+1e-9,fk.limits[:,1]-1e-9),max_nfev=100,ftol=1e-12,gtol=1e-12,xtol=1e-12)
 assert np.linalg.norm(err(z.x))<1e-6
 return z.x
rows=[]
for x,y,z in sorted(product([-4,-2,0,2,4],repeat=3),key=lambda v:sum(abs(x) for x in v)):
 if x*x+y*y+z*z>25:continue
 rot=rotation_matrix(math.radians(x),[1,0,0])@rotation_matrix(math.radians(y),[0,1,0])@rotation_matrix(math.radians(z),[0,0,1])
 target=pg.copy();target[:3,:3]=pg[:3,:3]@rot[:3,:3]
 goal=list(target[:3,3])+list(quaternion_from_matrix(target))
 command=np.rint(inverse(target,q0)/Q)*Q
 row=dict(delta_euler_deg=[x,y,z],pregrasp_pose=goal,nominal_command=command.tolist())
 try:
  _, pe=predict_held_axes_endpoint(fk,command,command-offset,goal,position_tolerance_m=.010,orientation_tolerance_rad=math.radians(5),allow_already_converged=True)
  row['pregrasp']=pe
  _, ae=predict_held_axes_endpoint(fk,pe['predicted_command_positions_rad'],pe['predicted_positions_rad'],a['fixed_goal_pose'],position_tolerance_m=.006,orientation_tolerance_rad=math.radians(5),allow_already_converged=True)
  row['approach']=ae; row['success']=True
 except HeldAxesPredictionError as exc:row['failure']=str(exc);row['evidence']=exc.evidence;row['success']=False
 rows.append(row)
 if row['success']:print('QUALIFIED',json.dumps(row),flush=True)
 else:print([x,y,z],row['failure'],row['evidence'].get('predicted_position_error_m'),flush=True)
(OUT/'pregrasp_orientation_search.json').write_text(json.dumps(dict(plan_id=r['plan_id'],scope='conditional_endpoint_only_no_path_authority',rows=rows),indent=2)+'\n')
