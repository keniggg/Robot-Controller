from pathlib import Path
helpers=Path(__file__).with_name('gen16_jaw_placement_probe.py').read_text().split('data=json.loads(INPUT.read_text());cases=[];start=time.monotonic()')[0]
exec(compile(helpers,'gen16_jaw_placement_probe_helpers','exec'))
p=NEW/'gen16_analysis/existing_candidate_endpoint_hypothesis.json';data=json.loads(p.read_text());scene=data['object_geometry'];opening=data['opening_m'];bias=np.array(data['sdk_minus_encoder_rad']);cases=[];start=time.monotonic();indices=[0,1,10,11,16,17,22,23,28,29]
# Bounded deterministic IK branch hypotheses, never a replacement for full path checks.
for i in indices:
 c=data['cases'][i];goal=np.array(c['goal']);q0=np.array(c['nominal_q']);seeds=[q0]
 for axes in ((1,2),(3,4,5),(0,1,2),(1,2,3,4)):
  s=q0.copy();s[list(axes)] *= -1;seeds.append(s)
 for sign in (-1,1):
  s=q0.copy();s[3]+=sign*math.pi;s[4]=-s[4];s[5]+=sign*math.pi;seeds.append(s)
 rng=np.random.RandomState(1600+i)
 for _ in range(5):seeds.append(rng.uniform(fk.limits[:,0],fk.limits[:,1]))
 solutions=[];attempts=[]
 for k,seed in enumerate(seeds):
  q,sol=solve(goal,seed);entry=dict(seed_index=k,ik=sol)
  if sol['position_error_m']<1e-6 and sol['orientation_error_rad']<1e-5:
   duplicate=next((j for j,v in enumerate(solutions) if np.max(abs(q-np.array(v['q'])))<1e-5),None)
   if duplicate is None:
    row=dict(q=q.tolist(),ik=sol,seed_index=k,command_within_joint_limits=bool(np.all(q+bias>=fk.limits[:,0]) and np.all(q+bias<=fk.limits[:,1])))
    if row['command_within_joint_limits']:
     row.update(nominal=endpoint(q,scene,opening),predicted_command=endpoint(q+bias,scene,opening));row['command_jaw_shift_m']=float((fk(q+bias)[:3,3]-fk(q)[:3,3])@goal[:3,1]);row['both_pass']=row['nominal']['ok'] and row['predicted_command']['ok']
    else:row['both_pass']=False
    solutions.append(row);entry['solution_index']=len(solutions)-1
   else:entry['solution_index']=duplicate
  attempts.append(entry)
 cases.append(dict(row_index=i,attempt_count=len(attempts),attempts=attempts,solutions=solutions));print(i,'unique',len(solutions),'jawmm',[round(x.get('command_jaw_shift_m',float('nan'))*1000,3) for x in solutions],'passes',[j for j,x in enumerate(solutions) if x['both_pass']],flush=True)
report=dict(scope='bounded_reconstructed_ik_branch_endpoint_hypotheses',execution_authorized=False,source_input=str(p),source_input_sha256=SHA(p),model_sha256=SHA(MODEL),cases=cases,elapsed_sec=time.monotonic()-start,limitations=['No original failed path is recorded. All q values are newly reconstructed IK solutions for original recorded registered grasp targets.','Bias remains the one recorded stationary source sample, not a new measurement of hypothetical arm configurations.','Endpoint only; no full arm or trajectory validation, no new contact evidence or execution authority.'])
out=NEW/'gen16_analysis/existing_candidate_branch_hypothesis.json';assert not out.exists();out.write_text(json.dumps(report,indent=2,ensure_ascii=False)+'\n');print(out,SHA(out),flush=True)
