"""Post-verification diagnostic fit. Never deploys or claims independent validation."""
from pathlib import Path
import json,sys,hashlib,numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'src/real-arm/alicia_d_calibration/scripts'))
import handeye_sample_audit as audit
from handeye_invariance_metrics import mean_transform
c=json.load(open(ROOT/'.runtime/current_handeye_task_pose_validation.json'));out=Path(c['session']);folder=max((Path(seg['session']) for seg in c['sampling_segments']),key=lambda p:sum(bool(e.get('active')) for e in json.load(open(p/'session.json'))['captures']));manifest=json.load(open(folder/'session.json'));entries=[e for e in manifest['captures'] if e.get('active')];samples=[];pins=[]
for e in entries:
 p=folder/e['directory']/'capture.json';raw=p.read_bytes();d=json.loads(raw);r=d['samples'][d['selected_frame_index']];samples.append({k:np.array(r[k]) for k in ['T_base_tool','T_camera_board']});pins.append(dict(capture=e['directory'],capture_sha256=hashlib.sha256(raw).hexdigest(),source_stamp_ns=r['source_stamp_ns']))
X=np.array(json.load(open(out/'starting_geometry.json'))['T_tool_camera_historical']);th={'translation_rms_m':.003,'translation_max_m':.005,'orientation_rms_deg':1.,'orientation_max_deg':2.};result=dict(purpose='diagnostic only after frozen-X verification; same samples are not independent validation of any fitted candidate',fit_data_session=str(folder),selection_basis='largest single-epoch set by sample count, independent of fit residual',fit_data_epoch_ns=manifest['initial_epoch'],source_groups=len(samples),input_pins=pins,thresholds=th,deployed=False,independently_validated_new_candidate=False,candidates={})
B=audit.board_transforms(samples,X);result['historical_fixed_X_against_diagnostic_recentered_Y']=audit.error_summary(*audit.errors_against_reference(B,mean_transform(B)))
for name,method in audit.ALGORITHMS.items():
 try:
  r=audit.candidate_record(samples,method,th);C=np.array(r['T_tool_camera']);r['change_from_deployed_translation_mm']=(1000*(C[:3,3]-X[:3,3])).tolist();r['change_from_deployed_rotation_deg']=float(np.degrees(np.arccos(np.clip((np.trace(C[:3,:3].T@X[:3,:3])-1)/2,-1,1))));result['candidates'][name]=r;print(name,json.dumps(r['cross_validation']['summary']),r['cross_validation']['passes_thresholds'],'delta_mm',r['change_from_deployed_translation_mm'],'delta_deg',r['change_from_deployed_rotation_deg'],flush=True)
 except Exception as exc:result['candidates'][name]=dict(error=str(exc));print(name,'ERROR',str(exc),flush=True)
(out/'post_validation_fit_diagnostic.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
