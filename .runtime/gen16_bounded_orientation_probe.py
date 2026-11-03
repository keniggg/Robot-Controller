from pathlib import Path
helpers=Path(__file__).with_name('gen16_jaw_placement_probe.py').read_text().split('data=json.loads(INPUT.read_text());cases=[];start=time.monotonic()')[0]
exec(compile(helpers,'gen16_jaw_placement_probe_helpers','exec'))
p=NEW/'gen16_analysis/existing_candidate_endpoint_hypothesis.json';data=json.loads(p.read_text());scene=data['object_geometry'];opening=data['opening_m'];bias=np.array(data['sdk_minus_encoder_rad']);pivot=np.array(scene['pose_xyz_xyzw'][:3]);normal=np.array(scene['support_normal_base']);cases=[];start=time.monotonic()
# A finite diagnostic grid only, not any production sampler or geometry authorization.
for i in [0,20,23]:
 c=data['cases'][i];original=np.array(c['goal']);q0=np.array(c['nominal_q'])
 for yaw in [-7.,0.,7.]:
  for tilt in [-5.,0.,5.]:
   goal=original.copy();Q=Rotation.from_rotvec(normal*math.radians(yaw)).as_dcm();goal[:3,:3]=Q@original[:3,:3];goal[:3,3]=pivot+Q@(original[:3,3]-pivot)
   jaw=goal[:3,1];P=Rotation.from_rotvec(jaw*math.radians(tilt)).as_dcm();goal[:3,:3]=P@goal[:3,:3];goal[:3,3]=pivot+P@(goal[:3,3]-pivot);trials=[]
   for delta in np.arange(-.005,.00501,.0005):
    target=goal.copy();target[:3,3]+=delta*jaw;q,sol=solve(target,q0);cmd=q+bias;r=dict(jaw_translation_m=float(delta),ik=sol)
    if sol['position_error_m']<1e-6 and sol['orientation_error_rad']<1e-5 and np.all(cmd>=fk.limits[:,0]) and np.all(cmd<=fk.limits[:,1]):
     r.update(nominal=endpoint(q,scene,opening),predicted_command=endpoint(cmd,scene,opening));r['both_endpoint_geometries_pass']=r['nominal']['ok'] and r['predicted_command']['ok']
     if r['both_endpoint_geometries_pass']:r.update(nominal_q=q.tolist(),command_q=cmd.tolist(),goal=target.tolist(),command_jaw_shift_m=float((fk(cmd)[:3,3]-fk(q)[:3,3])@jaw))
    else:r['both_endpoint_geometries_pass']=False
    trials.append(r)
   passing=[r for r in trials if r['both_endpoint_geometries_pass']];cases.append(dict(original_row=i,yaw_delta_deg=yaw,tilt_about_jaw_delta_deg=tilt,trials=trials,passing_offsets_m=[r['jaw_translation_m'] for r in passing]))
   if passing:print('ENDPOINT ONLY',i,yaw,tilt,'offsetmm',[round(r['jaw_translation_m']*1000,3) for r in passing],flush=True)
report=dict(scope='bounded_orientation_plus_placement_endpoint_hypothesis',execution_authorized=False,source_input=str(p),source_input_sha256=SHA(p),model_sha256=SHA(MODEL),cases=cases,elapsed_sec=time.monotonic()-start,limitations=['No original failed trajectory is available; newly solved local IK only.','Changed orientation and translation require original exact FusedTargetSurface and repeated bilateral physical footprint/contact-height/materialization checks. Those inputs are not reconstructed, so no reported pose is a qualified contact candidate.','No four-stage path or whole-arm collision verification; original object OBB/support/gripper and numerical geometry thresholds remain unchanged.'])
out=NEW/'gen16_analysis/bounded_orientation_endpoint_hypothesis.json';assert not out.exists();out.write_text(json.dumps(report,indent=2,ensure_ascii=False)+'\n');print(out,SHA(out),flush=True)
