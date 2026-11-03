#!/usr/bin/env python3
"""Bounded offline normal placement inside recorded unchanged contact margin."""
import importlib.util
import json
from pathlib import Path
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

spec=importlib.util.spec_from_file_location('gen15quantifier',Path(__file__).with_name('quantify_gen15_contact_prediction.py'))
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)

data=json.loads(m.AUDIT.read_text());previous=json.loads((m.OUT/'endpoint_quantification.json').read_text())
geom=data['replay_geometry'];reg=np.array(previous['registration'])
normal=np.array(geom['support_normal_base']);offset=geom['support_offset_m']
center=reg[:3,:3]@geom['obb_center_base_m']+reg[:3,3]
rotation=reg[:3,:3]@geom['R_base_obb'];size=np.array(geom['obb_size_xyz_m'])
fk=m.SerialUrdfFk(m.MODEL.read_text(),tuple('Joint%d'%i for i in range(1,7)))
gripper=m.GripperGeometry(.05,.002,np.array([.0434,.0286,.06]),np.array([.1175,.155,.0774]),.003)
rows=[]
for item in previous['rows']:
    entry=data['stable_evaluations'][item['index']]
    margin=entry['final_contact_height_margin'];limit=margin['translation_evidence_limit_m']
    intervals=[(a+limit,b-limit) for a,b in margin['allowed_normal_translation_intervals_m'] if a+limit<=b-limit]
    trials=[]
    original=np.array(item['nominal_T']);q0=np.array(item['offline_ik']['q']);bias=np.array(item['bias_sdk_minus_encoder_rad'])
    for low,high in intervals:
        values=sorted(set([0.,float(low+1e-6),float(high-1e-6)]+[float(x) for x in np.arange(.0005,high,.0005)]))
        for delta in values:
            if not low<=delta<=high:continue
            goal=original.copy();goal[:3,3]+=delta*normal
            def residual(q):
                T=fk(q);return np.r_[T[:3,3]-goal[:3,3],.2*Rotation.from_dcm(goal[:3,:3].T@T[:3,:3]).as_rotvec()]
            fit=least_squares(residual,q0,bounds=(fk.limits[:,0],fk.limits[:,1]),max_nfev=60,ftol=1e-12,xtol=1e-12,gtol=1e-12)
            if np.linalg.norm(residual(fit.x))>1e-7:continue
            if np.any(fit.x+bias<fk.limits[:,0]) or np.any(fit.x+bias>fk.limits[:,1]):continue
            checks={}
            for label,q in (('nominal',fit.x),('predicted_command',fit.x+bias)):
                T=fk(q);bodies=[]
                for name,c,s in m._stage_boxes(T,gripper,.04975,'y','z'):
                    bodies.append(dict(body=name,overlap=bool(m._obb_overlap(c,T[:3,:3],s,center,rotation,size)),sat_separating_gap_m=m.sat_gap(c,T[:3,:3],s,center,rotation,size),support_clearance_m=float(np.min(m._box_corners(c,T[:3,:3],s)@normal+offset))))
                checks[label]=bodies
            ok=not any(b['overlap'] or b['support_clearance_m']<.003 for bs in checks.values() for b in bs)
            trials.append(dict(normal_translation_m=delta,remaining_symmetric_contact_margin_m=min(delta-low+limit,high-delta+limit),original_evidence_limit_m=limit,endpoint_checks=checks,ok=ok,q=fit.x.tolist(),position_rotation_weighted_residual=float(np.linalg.norm(residual(fit.x)))))
    rows.append(dict(index=item['index'],source_index=item['source_index'],variant_index=item['variant_index'],original_contact_intervals=margin['allowed_normal_translation_intervals_m'],allowed_center_shift_preserving_entire_original_evidence_interval_m=intervals,trials=trials))
passing=[dict(index=r['index'],source_index=r['source_index'],variant_index=r['variant_index'],first_passing_trial=next(t for t in r['trials'] if t['ok'])) for r in rows if any(t['ok'] for t in r['trials'])]
report=dict(scope='pure endpoint counterfactual inside recorded original normal contact/support interval after retaining full original evidence radius; no execution authority',original_report='endpoint_quantification.json',candidate_count=len(rows),passing_candidate_count=len(passing),passing=passing,rows=rows,limitations=['Exact failed full trajectory was not recorded; independent endpoint IK only.','Original contact interval is used with full original evidence radius, not reduced to nominal point.','New placement is not a promoted plan: original full geometry, registration displacement bound, motion following/whole-arm and complete strict path remain required.'])
out=m.OUT/'normal_margin_counterfactual.json';out.write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(dict(result=str(out),candidate_count=len(rows),passing_count=len(passing),first_passing=passing[:1]),indent=2))
