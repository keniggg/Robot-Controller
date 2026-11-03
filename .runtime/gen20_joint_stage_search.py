"""Offline coupled guide experiment, never path/execution authorization."""
exec(open(__file__.replace('gen20_joint_stage_search.py','gen20_held_search.py')).read().split('for r in records[:3]:')[0])
results=[]
for r in records[:3]:
 h=r['precheck']['held_approach_prediction']; p=h['predicted_pregrasp']; a=h['predicted_approach']
 q=np.asarray(p['virtual_start_positions_rad']); counts=np.asarray(p['initial_sdk_counts'])
 pg=pose_matrix(p['fixed_goal_pose']); ag=pose_matrix(a['fixed_goal_pose'])
 def err(j,g):
  t=fk(j);return np.linalg.norm(t[:3,3]-g[:3,3]),math.acos(float(np.clip((np.trace(t[:3,:3].T@g[:3,:3])-1)/2,-1,1)))
 def state(x):
  j=q.copy();j[free]+=np.asarray(x)*Q;return j
 def constraints(x):
  pe,pa=err(state(x[:4]),pg); ae,aa=err(state(x[:4]+x[4:]),ag)
  return np.array([.010-pe,math.radians(5)-pa,.025-err(state(x[:4]),ag)[0],math.radians(5)-err(state(x[:4]),ag)[1],math.radians(5)-aa])
 orig=np.r_[np.asarray(p['proposed_delta_counts'])[free],np.asarray(a['proposed_delta_counts'])[free]]
 runs=[]
 for start in [orig,np.zeros(8)]+list(np.random.RandomState(20).uniform(-32,32,(4,8))):
  z=minimize(lambda x:err(state(x[:4]+x[4:]),ag)[0]**2,start,method='SLSQP',bounds=[(-32,32)]*8,constraints=[dict(type='ineq',fun=constraints)],options={'maxiter':150,'ftol':1e-13})
  d=np.rint(z.x); runs.append(dict(x=z.x.tolist(),quantized=d.tolist(),pre_error=list(err(state(d[:4]),pg)),approach_error=list(err(state(d[:4]+d[4:]),ag)),constraint=constraints(d).tolist(),success=bool(z.success),iterations=int(z.nit)))
 best=min(runs,key=lambda v:v['approach_error'][0]);print(r['plan_id'],json.dumps(best),flush=True)
 results.append(dict(plan_id=r['plan_id'],runs=runs))
(OUT/'coupled_held_stage_search.json').write_text(json.dumps(results,indent=2)+'\n')
