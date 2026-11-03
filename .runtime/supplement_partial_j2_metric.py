#!/usr/bin/env python3
"""Single-source-pair supplement, explicitly not a multiframe holdout."""
import hashlib,json,os
from pathlib import Path
import numpy as np
import yaml
from tf.transformations import quaternion_matrix
import evaluate_frozen_metric_local_holdouts as L
import evaluate_frozen_metric_j1 as E
p=L.RESUME/'frozen_metric_local_holdouts/report.json'
if not os.path.ismount(L.MOUNT):raise RuntimeError('SanStand required')
r=json.loads(p.read_text());prior=L.RESUME/'joint_metric_identifiability/fit_report.json'
fit=next(f for f in json.loads(prior.read_text())['fits'] if f['orientation_unit_deg']==.1)
rank=json.loads((prior.parent/'rank_report.json').read_text())
yp=Path('/home/zhuyupei/.ros/easy_handeye/d405_cc200_auto_20260927_validated_eye_on_hand.yaml')
if hashlib.sha256(yp.read_bytes()).hexdigest()!=rank['handeye_sha256']:raise RuntimeError('baseline handeye changed since prior fit')
y=yaml.safe_load(yp.read_text())['transformation'];X=quaternion_matrix([y[k] for k in ('qx','qy','qz','qw')]);X[:3,3]=[y[k] for k in ('x','y','z')]
for s in r['sessions'].values():
    session=Path(s['session']);bf=session/('fresh_return_baseline_geometry.json' if 'fresh_return' in session.name else 'baseline_geometry.json')
    np.testing.assert_allclose(np.asarray(json.loads(bf.read_text())['T_tool_camera']),X,atol=1e-12,rtol=0)
r['baseline_handeye_verified_sha256']=rank['handeye_sha256']
s=r['sessions']['partial_J2_return'];session=Path(s['session']);rows=[];hashes={}
for label,name in [('baseline','explicit_original_sdk_return_j2_precommand_0_geometry.json'),('partial_J2_return','explicit_original_sdk_return_j3_precommand_0_geometry.json')]:
    source=session/name;g=json.loads(source.read_text());hashes[name]=hashlib.sha256(source.read_bytes()).hexdigest()
    rows.append(dict(g,offline_stationary_phase=label,exact_rgb_archived=True))
xml=(session/'robot_description.urdf').read_text();extra=dict(scope='one archived precommand RGB/quality/TF snapshot each, not sufficient for multiframe after-window',
    reason='bag ended before a complete centred +/-0.4 second accepted window for the post-J2 source',
    source_stamp_ns=[x['source_stamp_ns'] for x in rows],input_sha256=hashes,models={})
for label,z in [('runtime',np.zeros(10)),('prior_training_only_candidate',np.array(fit['z']))]:
    rr=E.evaluated(rows,xml,X,z)
    extra['models'][label]=L.comparison(rr[0],rr[1],np.asarray(rr[0]['T_tool_camera']))
s['snapshot_only_supplement']=extra
r['supplement_source_sha256']=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
p.write_text(json.dumps(r,indent=2,ensure_ascii=False)+'\n');(p.parent/Path(__file__).name).write_bytes(Path(__file__).read_bytes())
print(json.dumps(extra,ensure_ascii=False))
