#!/usr/bin/env python3
"""Freeze and evaluate every accepted automatic training group, no deployment."""
from pathlib import Path
from datetime import datetime,timezone
import json,sys,hashlib,os
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'src/real-arm/alicia_d_calibration/scripts'))
import handeye_sample_audit as audit
from handeye_invariance_metrics import mean_transform
c=json.loads((ROOT/'.runtime/current_handeye_autocalibration.json').read_text());session=Path(c['session']);assert os.path.ismount('/media/zhuyupei/SanStand')
raw=(session/'session.json').read_bytes();m=json.loads(raw);entries=[e for e in m['captures'] if e.get('active')]
assert len(entries)>=10 and all(e['role']=='training' for e in entries)
out=session/('fit_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ'));out.mkdir();(out/'session_snapshot.json').write_bytes(raw)
th={'translation_rms_m':.003,'translation_max_m':.005,'orientation_rms_deg':1.,'orientation_max_deg':2.}
protocol={'thresholds':th,'groups':len(entries),'selection':'all active training samples, no post-fit exclusion','folds':'five deterministic group folds k::5; one archived selected middle frame per group','test_reference':'mean base-board transform from train groups only','candidate_selection':'among candidates passing all four CV limits, choose minimum translation RMS; otherwise no accepted candidate','independent_validation_required':True,'handeye_deployment':False}
(out/'protocol.json').write_text(json.dumps(protocol,indent=2)+'\n');samples=[];pins=[]
for e in entries:
 data=(session/e['directory']/'capture.json').read_bytes();d=json.loads(data);r=d['samples'][d['selected_frame_index']];assert d['accepted'] and d['model_fingerprint']==m['camera_model_fingerprint']
 samples.append({k:np.array(r[k]) for k in ['T_base_tool','T_camera_board']});(out/(e['directory']+'.json')).write_bytes(data);pins.append({'capture':e['directory'],'capture_sha256':hashlib.sha256(data).hexdigest(),'bag_sha256':e['bag_sha256'],'source_stamp_ns':r['source_stamp_ns']})
results={}
for name,method in audit.ALGORITHMS.items():
 try:results[name]=audit.candidate_record(samples,method,th)
 except Exception as e:results[name]={'error':str(e)}
valid=[name for name,r in results.items() if r.get('cross_validation',{}).get('passes_thresholds')]
best=min(valid,key=lambda name:results[name]['cross_validation']['summary']['translation_rms_m']) if valid else None
report={'created_at':datetime.now(timezone.utc).isoformat(),'protocol':protocol,'inputs':pins,'model_fingerprint':m['camera_model_fingerprint'],'algorithms':results,'selected_algorithm':best,'calibration_passed':False,'independent_validation_done':False,'deployed':False}
if best:
 X=np.array(results[best]['T_tool_camera']);report['frozen_candidate']={'algorithm':best,'T_tool_camera':X.tolist(),'T_base_board_training_reference':mean_transform(audit.board_transforms(samples,X)).tolist(),'training_groups':len(samples),'scope':'candidate awaiting independent validation'}
(out/'results.json').write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False)+'\n');(ROOT/'.runtime/current_handeye_auto_fit.json').write_text(json.dumps({'directory':str(out),'selected_algorithm':best},indent=2)+'\n')
print('FIT',out,'GROUPS',len(samples),'SELECTED',best)
for name,r in results.items():print(name,json.dumps(r.get('cross_validation',{}).get('summary',r)),r.get('cross_validation',{}).get('passes_thresholds'))
