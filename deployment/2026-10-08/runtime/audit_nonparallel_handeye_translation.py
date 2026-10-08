#!/usr/bin/env python3
"""Diagnostic fixed-X translation fit; J6 held out, no parameter writes."""
from pathlib import Path
import json,importlib.util,hashlib
import numpy as np
root=Path(__file__).resolve().parents[1];spec=importlib.util.spec_from_file_location('transaudit',root/'.runtime/analyze_fixed_handeye_translation.py');G=importlib.util.module_from_spec(spec);spec.loader.exec_module(G)
out=Path(json.loads((root/'.runtime/current_grasp_repair_resume.json').read_text())['directory']);sessions=G.load()
for joint in (4,6):
 p=out/('wrist_multiframe_j%d'%joint)/'samples.json';sessions.append(dict(joint=joint,rows=json.loads(p.read_text()),source=str(p),sha256=hashlib.sha256(p.read_bytes()).hexdigest()))
report=dict(diagnostic_only=True,runtime_changed=False,fit_joint_sessions=[2,3,4,5],heldout_joint=6,board_positions_never_compared_between_sessions=True,source=[{k:s[k] for k in ('joint','source','sha256')} for s in sessions],fits={})
for label,point in [('origin',np.zeros(3)),('center',G.CENTER)]:
 cs,rs,windows=G.build(sessions,point);train=[c for c in cs if c['joint']!=6];test=[c for c in cs if c['joint']==6];fit=G.fit(train)
 rr=dict(delta_tool_translation_mm=fit['correction']*1000,rank=fit['rank'],condition=fit['condition'],singular=fit['singular'],training=G.evaluate(train,fit['correction']),heldout_J6=G.evaluate(test,fit['correction']),returns=G.evaluate(rs,fit['correction']),leave_one_axis=[])
 for c in train:
  ff=G.fit([x for x in train if x is not c]);rr['leave_one_axis'].append(dict(heldout_axis=c['joint'],delta_tool_translation_mm=ff['correction']*1000,condition=ff['condition'],prediction=G.evaluate([c],ff['correction'])))
 report['fits'][label]=rr
 print(label,'correction mm',rr['delta_tool_translation_mm'],'condition',rr['condition']);print('train',[(r['joint'],r['original_norm_mm'],r['corrected_norm_mm']) for r in rr['training']]);print('J6heldout',rr['heldout_J6'])
p=out/'nonparallel_translation_audit';p.mkdir(exist_ok=True);(p/'report.json').write_text(json.dumps(report,default=G.as_json,indent=2));(p/Path(__file__).name).write_bytes(Path(__file__).read_bytes())
