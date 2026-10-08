#!/usr/bin/env python3
"""Additional closed-centering holdout only; no fit, live ROS, or parameter writes."""
import argparse,hashlib,json,os
from pathlib import Path
import numpy as np
import evaluate_frozen_metric_local_holdouts as L
import evaluate_frozen_metric_j1 as E

p=argparse.ArgumentParser();p.add_argument('session',type=Path);p.add_argument('output',type=Path);a=p.parse_args()
if not os.path.ismount(L.MOUNT) or L.MOUNT not in a.output.resolve().parents:raise ValueError('SanStand output required')
a.output.mkdir(parents=True,exist_ok=True)
template=json.loads((a.session/'baseline_geometry.json').read_text())
geos,audit=L.J.D.collect_paired_geometries(a.session,template,maximum_per_window=24)
rows=[dict(r,exact_rgb_archived=True) for r in geos.values()]
prior=L.RESUME/'joint_metric_identifiability/fit_report.json';fit=next(f for f in json.loads(prior.read_text())['fits'] if f['orientation_unit_deg']==.1)
xml=(a.session/'robot_description.urdf').read_text();X=np.asarray(template['T_tool_camera'])
reference=json.loads((L.RESUME/'j1_board_axis_140754/selected_geometries.json').read_text());np.testing.assert_allclose(X,next(iter(reference.values()))['T_tool_camera'],atol=1e-12,rtol=0)
result=dict(diagnostic_only=True,refitted=False,runtime_changed=False,session=str(a.session),selection=audit,
    source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
    frozen_prior_fit_sha256=hashlib.sha256(prior.read_bytes()).hexdigest(),frozen_parameters=fit['parameters'],
    original_model_sha256=hashlib.sha256(xml.encode()).hexdigest(),models={},
    limitations=['Comparisons assume board stayed fixed within the qualified windows of this CLOSED bag.',
        'Board was subsequently observed removed; do not compare absolute board position to other sessions or use these frames live.',
        'Only previously frozen 22-training-frame 0.1-degree-weight candidate evaluated. No new fit or parameter selection.',
        'Original task16 maximum pairwise position spread of 5.449 mm and adverse local heldouts remain.'])
for label,z in [('runtime',np.zeros(10)),('prior_training_only_candidate',np.asarray(fit['z']))]:
    rr=E.evaluated(rows,xml,X,z);result['models'][label]=L.describe(rr)
(a.output/'report.json').write_text(json.dumps(result,indent=2,ensure_ascii=False)+'\n')
(a.output/'selected_geometries.json').write_text(json.dumps(geos,indent=2)+'\n')
(a.output/Path(__file__).name).write_bytes(Path(__file__).read_bytes())
print(json.dumps({'selection':audit,'models':{k:v['comparisons'] for k,v in result['models'].items()}},ensure_ascii=False))
