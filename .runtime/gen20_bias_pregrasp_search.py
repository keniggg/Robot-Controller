exec(open(__file__.replace('gen20_bias_pregrasp_search.py','gen20_held_search.py')).read().split('for r in records[:3]:')[0])
from alicia_flexible_grasp.robot.held_axes_prediction import predict_held_axes_endpoint,HeldAxesPredictionError
rows=[]
for r in records:
 h=r['precheck']['held_approach_prediction'];p=h['predicted_pregrasp'];a=h['predicted_approach'];offset=np.asarray(h['sdk_minus_encoder_rad'])
 nominal=np.asarray(h['nominal_pregrasp_command_rad']);cmd=np.rint((nominal+offset)/Q)*Q;actual=cmd-offset
 row=dict(plan_id=r['plan_id'],new_pregrasp_command=cmd.tolist(),new_pregrasp_actual=actual.tolist())
 try:
  _,pe=predict_held_axes_endpoint(fk,cmd,actual,p['fixed_goal_pose'],position_tolerance_m=.010,orientation_tolerance_rad=math.radians(5),allow_already_converged=True)
  row['pregrasp']=pe
  _,ae=predict_held_axes_endpoint(fk,pe['predicted_command_positions_rad'],pe['predicted_positions_rad'],a['fixed_goal_pose'],position_tolerance_m=.006,orientation_tolerance_rad=math.radians(5),allow_already_converged=True)
  row['approach']=ae;row['success']=True
 except HeldAxesPredictionError as exc:row.update(success=False,failure=str(exc),evidence=exc.evidence)
 rows.append(row)
print('qualified',sum(r['success'] for r in rows),'total',len(rows))
print('best',min([r for r in rows if not r['success']],key=lambda v:v['evidence'].get('predicted_position_error_m',100)),flush=True)
(OUT/'bias_pregrasp_search.json').write_text(json.dumps(dict(scope='conditional_bias_aware_pregrasp_no_path_authority',rows=rows),indent=2)+'\n')
