exec(open(__file__.replace('gen20_tolerance_pregrasp_search.py','gen20_held_search.py')).read().split('for r in records[:3]:')[0])
from scipy.optimize import least_squares
from alicia_flexible_grasp.robot.held_axes_prediction import predict_held_axes_endpoint,HeldAxesPredictionError
def inverse(target,seed):
 def err(q):
  t=fk(q); dr=target[:3,:3].T@t[:3,:3]
  return np.r_[(t[:3,3]-target[:3,3])*10,(dr[2,1]-dr[1,2])/2,(dr[0,2]-dr[2,0])/2,(dr[1,0]-dr[0,1])/2]
 z=least_squares(err,seed,bounds=(fk.limits[:,0]+1e-9,fk.limits[:,1]-1e-9),max_nfev=100,ftol=1e-12,gtol=1e-12,xtol=1e-12)
 assert np.linalg.norm(err(z.x))<1e-6
 return z.x
rows=[]
for r in records[:5]:
 h=r['precheck']['held_approach_prediction'];p=h['predicted_pregrasp'];a=h['predicted_approach'];offset=np.asarray(h['sdk_minus_encoder_rad'])
 pg=pose_matrix(p['fixed_goal_pose']);ag=pose_matrix(a['fixed_goal_pose']);nominal=np.asarray(h['nominal_pregrasp_command_rad'])
 direction=ag[:3,3]-pg[:3,3];direction/=np.linalg.norm(direction)
 for distance in [.004,.006,.008,.009,.0095]:
  target=pg.copy();target[:3,3]+=direction*distance
  actual=inverse(target,nominal);cmd=np.rint((actual+offset)/Q)*Q;actual=cmd-offset
  row=dict(plan_id=r['plan_id'],distance=distance,command=cmd.tolist(),actual=actual.tolist())
  try:
   _,pe=predict_held_axes_endpoint(fk,cmd,actual,p['fixed_goal_pose'],position_tolerance_m=.010,orientation_tolerance_rad=math.radians(5),allow_already_converged=True)
   row['pregrasp']=pe
   _,ae=predict_held_axes_endpoint(fk,pe['predicted_command_positions_rad'],pe['predicted_positions_rad'],a['fixed_goal_pose'],position_tolerance_m=.006,orientation_tolerance_rad=math.radians(5),allow_already_converged=True)
   row['approach']=ae;row['success']=True
  except HeldAxesPredictionError as exc:row.update(success=False,failure=str(exc),evidence=exc.evidence)
  rows.append(row);print(r['plan_id'],distance,row['success'],row.get('approach',row.get('evidence'))['predicted_position_error_m'],flush=True)
(OUT/'tolerance_pregrasp_search.json').write_text(json.dumps(dict(scope='conditional_existing_pregrasp_tolerance_no_path_authority',rows=rows),indent=2)+'\n')
