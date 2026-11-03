from pathlib import Path
helpers=Path(__file__).with_name('gen16_jaw_placement_probe.py').read_text().split('data=json.loads(INPUT.read_text());cases=[];start=time.monotonic()')[0]
exec(compile(helpers,'gen16_jaw_placement_probe_helpers','exec'))
p=NEW/'gen16_analysis/existing_candidate_endpoint_hypothesis.json';data=json.loads(p.read_text());scene=data['object_geometry'];opening=data['opening_m'];bias=np.array(data['sdk_minus_encoder_rad']);cases=[];start=time.monotonic()
for case in data['cases']:
 goal=np.array(case['goal']);jaw=goal[:3,1];q0=np.array(case['nominal_q']);trials=[]
 for delta in np.arange(-.005,.00501,.0005):
  target=goal.copy();target[:3,3]+=delta*jaw;q,sol=solve(target,q0);cmd=q+bias
  row=dict(jaw_translation_m=float(delta),ik=sol)
  if sol['position_error_m']<1e-6 and sol['orientation_error_rad']<1e-5 and np.all(cmd>=fk.limits[:,0]) and np.all(cmd<=fk.limits[:,1]):
   row.update(nominal=endpoint(q,scene,opening),predicted_command=endpoint(cmd,scene,opening));row['both_endpoint_geometries_pass']=row['nominal']['ok'] and row['predicted_command']['ok']
   if row['both_endpoint_geometries_pass']:row.update(nominal_q=q.tolist(),command_q=cmd.tolist(),goal=target.tolist())
  else:row['both_endpoint_geometries_pass']=False
  trials.append(row)
 passing=[r for r in trials if r['both_endpoint_geometries_pass']]
 c=dict(row_index=case['row_index'],original_tilt_deg=case['original_tilt_deg'],source_index=case['source_index'],variant_index=case['variant_index'],original_strict_checked=case['original_strict_checked'],trials=trials,passing_offsets_m=[r['jaw_translation_m'] for r in passing]);cases.append(c)
 if passing:print('PASS ENDPOINT ONLY',c['row_index'],'tilt',c['original_tilt_deg'],'offsetmm',[round(v*1000,3) for v in c['passing_offsets_m']],flush=True)
report=dict(scope='existing_candidate_limited_jaw_offset_endpoint_only',execution_authorized=False,source_input=str(p),source_input_sha256=SHA(p),model_sha256=SHA(MODEL),cases=cases,elapsed_sec=time.monotonic()-start,limitations=['No original failed trajectory is available; new local IK endpoints only.','Shifted candidate bilateral physical footprint requires original registered fused point source sets and full re-materialization; this evidence is not reconstructed here and no contact eligibility is granted.','No four-stage path or whole-arm collision verification. Original OBB, support and geometry thresholds are unchanged.'])
out=NEW/'gen16_analysis/existing_candidate_offset_hypothesis.json';assert not out.exists();out.write_text(json.dumps(report,indent=2,ensure_ascii=False)+'\n');print(out,SHA(out),flush=True)
