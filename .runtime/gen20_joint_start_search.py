exec(open(__file__.replace('gen20_joint_start_search.py','gen20_held_search.py')).read().split('for r in records[:3]:')[0])
from alicia_flexible_grasp.robot.held_axes_prediction import predict_held_axes_endpoint,HeldAxesPredictionError
rows=[]
for r in records[:3]:
 h=r['precheck']['held_approach_prediction'];p=h['predicted_pregrasp'];a=h['predicted_approach'];offset=np.asarray(h['sdk_minus_encoder_rad']);nominal=np.asarray(h['nominal_pregrasp_command_rad'])
 pg=pose_matrix(p['fixed_goal_pose']);ag=pose_matrix(a['fixed_goal_pose'])
 def err(q,g):
  t=fk(q);return np.linalg.norm(t[:3,3]-g[:3,3]),math.acos(float(np.clip((np.trace(t[:3,:3].T@g[:3,:3])-1)/2,-1,1)))
 def after(x):
  q=x[:6].copy();q[free]+=x[6:]*Q;return q
 def con(x):
  pe,pa=err(x[:6],pg);ae,aa=err(after(x),ag)
  return np.array([.009-pe,math.radians(4)-pa,math.radians(4)-err(x[:6],ag)[1],math.radians(4)-aa])
 for seed in [np.r_[nominal,np.asarray(a['proposed_delta_counts'])[free]],np.r_[nominal,np.zeros(4)]]:
  result=minimize(lambda x:err(after(x),ag)[0]**2,seed,method='SLSQP',bounds=list(zip(np.maximum(nominal-.25,fk.limits[:,0]),np.minimum(nominal+.25,fk.limits[:,1])))+[(-32,32)]*4,constraints=[dict(type='ineq',fun=con)],options={'maxiter':150,'ftol':1e-13})
  cmd=np.rint((result.x[:6]+offset)/Q)*Q;actual=cmd-offset
  row=dict(plan_id=r['plan_id'],seed=seed.tolist(),solver_x=result.x.tolist(),solver_success=bool(result.success),constraints=con(result.x).tolist(),continuous_approach=list(err(after(result.x),ag)),command=cmd.tolist(),actual=actual.tolist())
  try:
   _,pe=predict_held_axes_endpoint(fk,cmd,actual,p['fixed_goal_pose'],position_tolerance_m=.010,orientation_tolerance_rad=math.radians(5),allow_already_converged=True)
   row['pregrasp']=pe
   _,ae=predict_held_axes_endpoint(fk,pe['predicted_command_positions_rad'],pe['predicted_positions_rad'],a['fixed_goal_pose'],position_tolerance_m=.006,orientation_tolerance_rad=math.radians(5),allow_already_converged=True)
   row['approach']=ae;row['success']=True
  except HeldAxesPredictionError as exc:row.update(success=False,failure=str(exc),evidence=exc.evidence)
  rows.append(row);print(r['plan_id'],row['continuous_approach'],row['success'],row.get('approach',row.get('evidence')).get('predicted_position_error_m'),flush=True)
(OUT/'joint_start_search.json').write_text(json.dumps(dict(scope='conditional_joint_start_and_held_approach_no_path_authority',rows=rows),indent=2)+'\n')
