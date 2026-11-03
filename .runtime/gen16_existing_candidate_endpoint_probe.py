from pathlib import Path
# Reuse this session's frozen pure numeric helpers, not a robot control module.
helpers=Path(__file__).with_name('gen16_jaw_placement_probe.py').read_text().split('data=json.loads(INPUT.read_text());cases=[];start=time.monotonic()')[0]
exec(compile(helpers,'gen16_jaw_placement_probe_helpers','exec'))
data=json.loads(INPUT.read_text());audit=json.loads(Path(data['audit_path']).read_text());reg=np.array(audit['multiview_surface']['latest_registration']['snapshot_to_reference_transform']);receipt=data['rejections'][0]['evidence'];scene=receipt['object_geometry'];opening=receipt['precheck']['opening_m'];sample=receipt['precheck']['source_sample'];bias=np.array(sample['sdk_positions_rad'])-sample['accepted_positions_rad'];cases=[]
# Confirm the transform on all three originally strict-checked families.
verified=[]
for i,j in ((23,0),(10,3),(17,6)):
 row=data['stable_evaluations'][i];source=next(p for p in row['execution_sequence']['stages'] if p['stage']=='grasp');pred=reg@matrix(source['position_m']+source['quaternion_xyzw']);actual=matrix(data['rejections'][j]['evidence']['poses_xyz_xyzw'][2]);assert np.max(abs(pred-actual))<3e-15;verified.append(dict(row=i,receipt_index=j,max_absolute_matrix_difference=float(np.max(abs(pred-actual)))))
start=time.monotonic()
for i,row in enumerate(data['stable_evaluations']):
 source=next(p for p in row['execution_sequence']['stages'] if p['stage']=='grasp');goal=reg@matrix(source['position_m']+source['quaternion_xyzw']);hints=(row.get('endpoint_ik_search_probe') or {}).get('stages',[]);grasp=next((x for x in hints if x['stage']=='grasp' and x.get('error_code')==1),None)
 if grasp:
  m=dict(zip(grasp['joint_solution']['name'],grasp['joint_solution']['position_rad']));seed=np.array([m[n] for n in ARM_NAMES]);seedkind='recorded_pre_registration_grasp_ik_hint'
 else:seed=np.array(sample['accepted_positions_rad']);seedkind='recorded_stationary_measured_q_no_hint'
 q,sol=solve(goal,seed);command=q+bias; result=dict(row_index=i,source_index=row['source_index'],variant_index=row['variant_index'],seed_kind=seedkind,seed=seed.tolist(),goal=goal.tolist(),nominal_q=q.tolist(),command_q=command.tolist(),ik=sol,original_contact_height_placement=row.get('contact_height_placement'),original_final_contact_height_margin=row.get('final_contact_height_margin'),original_tilt_deg=row['adaptive_stage_profile']['tilt_deg'],original_strict_checked=bool(row.get('contact_sequence_precheck')))
 if sol['position_error_m']<1e-6 and sol['orientation_error_rad']<1e-5 and np.all(command>=fk.limits[:,0]) and np.all(command<=fk.limits[:,1]):
  result.update(nominal=endpoint(q,scene,opening),predicted_command=endpoint(command,scene,opening));result['both_geometries_pass']=result['nominal']['ok'] and result['predicted_command']['ok'];result['command_jaw_shift_m']=float((fk(command)[:3,3]-fk(q)[:3,3])@goal[:3,1])
 else:result['both_geometries_pass']=False
 cases.append(result);print(i,'tilt',round(result['original_tilt_deg'],3),'ikpos',round(sol['position_error_m']*1000,4),'nom/cmd',result.get('nominal',{}).get('ok'),result.get('predicted_command',{}).get('ok'),'jawmm',round(result.get('command_jaw_shift_m',float('nan'))*1000,3),flush=True)
report=dict(scope='recorded_candidate_registration_then_reconstructed_endpoint_only',execution_authorized=False,source_input=str(INPUT),source_input_sha256=SHA(INPUT),model_sha256=SHA(MODEL),fixed_source_registration=reg.tolist(),registration_exact_original_receipt_checks=verified,sdk_minus_encoder_rad=bias.tolist(),object_geometry=scene,opening_m=opening,cases=cases,elapsed_sec=time.monotonic()-start,limitations=['Original grasp trajectory/terminal q absent; these are newly solved exact goal endpoints, not original MoveIt paths.','Original candidate pose, measured contact and placement audit are retained. No new orientation or contact evidence has been invented.','No full commanded curve, arm collision, held pregrasp selector or final registered contact-margin checks are performed for the unchecked candidates.'])
out=NEW/'gen16_analysis/existing_candidate_endpoint_hypothesis.json';assert not out.exists();out.write_text(json.dumps(report,indent=2,ensure_ascii=False)+'\n');print(out,SHA(out),flush=True)
