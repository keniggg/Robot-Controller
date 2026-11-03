#!/usr/bin/env python3
"""Offline within-session rotation-cancellation and fixed-translation sensitivity."""
import argparse
import importlib.util
import json
import math
import os
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
s=importlib.util.spec_from_file_location('geometry',ROOT/'.runtime/analyze_fixed_handeye_translation.py')
G=importlib.util.module_from_spec(s);s.loader.exec_module(G)

def run(directory):
    rows=json.loads((directory/'samples.json').read_text())
    by={k:[r for r in rows if r['window']==k and r['exact_fresh_rgb_quality_tf']] for k in ['baseline','positive_stationary']}
    result={'kind':'counterrotation_time_block_invariants','window_frames':{k:len(v) for k,v in by.items()},'points':{},'limitations':['Per-window half-second blocks, no return movement, no simultaneous-motion assumption.','100 mm translation perturbation is a sensitivity scenario, not a measured calibration uncertainty.','Only measured stationary frames and within-session fixed board comparison.','Conditional block bootstrap does not include camera/board/model systematic error.']}
    for name,point in [('origin',np.zeros(3)),('center',G.CENTER)]:
        a,b=[G.summarize(by[k],point) for k in ['baseline','positive_stationary']]
        c=G.contrast(a,b,'positive',23);f=G.fit([c]);rng=np.random.default_rng(2809);deltas=[]
        for _ in range(200):
            x,y=[G.summarize(by[k],point,rng) for k in ['baseline','positive_stationary']]
            deltas.append((y['position']-x['position'])*1000)
        result['points'][name]={'block_counts':[a['block_count'],b['block_count']],'base_board_delta_mm':c['delta']*1000,'base_board_delta_norm_mm':float(np.linalg.norm(c['delta'])*1000),'rank_translation_response':f['rank'],'singular_values':f['singular'],'unconstrained_minimum_norm_translation_correction_mm':f['correction']*1000,'best_possible_residual_mm':float(np.linalg.norm(f['residual'])*1000),'conditional_delta_bootstrap_mm':G.percentile(deltas)}
    x=G.inv(np.asarray(rows[0]['T_base_tool_fk']))@np.asarray(rows[0]['T_base_board_fk'])@G.inv(np.asarray(rows[0]['T_camera_board']))
    rel=G.inv(G.average_transform([r['T_base_tool_fk'] for r in by['baseline']]))@G.average_transform([r['T_base_tool_fk'] for r in by['positive_stationary']]);bc=a['camera']@G.inv(b['camera']);bf=G.inv(x)@rel@x
    angle=float(np.linalg.norm(G.log_so3(rel[:3,:3])));leak=2*math.sin(angle/2)
    result['mean_motion']={'camera_rotation_vector_deg':np.degrees(G.log_so3(bc[:3,:3])),'fk_camera_rotation_vector_deg':np.degrees(G.log_so3(bf[:3,:3])),'camera_translation_mm':bc[:3,3]*1000,'fk_camera_translation_mm':bf[:3,3]*1000,'camera_angle_deg':float(np.degrees(np.linalg.norm(G.log_so3(bc[:3,:3])))),'fk_angle_deg':math.degrees(angle),'camera_tnorm_mm':float(np.linalg.norm(bc[:3,3])*1000),'fk_tnorm_mm':float(np.linalg.norm(bf[:3,3])*1000),'translation_error_mm':float(np.linalg.norm(bc[:3,3]-bf[:3,3])*1000),'fixed_translation_gain':leak,'max_leakage_100mm_translation_error_mm':leak*100}
    (directory/'counterrotation_invariants.json').write_text(json.dumps(result,ensure_ascii=False,indent=2,default=G.as_json)+'\n')
    (directory/'invariants_analysis_source.py').write_bytes(Path(__file__).read_bytes())
    return result

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('directory',type=Path);a=p.parse_args()
    if not os.path.ismount(G.MOUNT) or G.MOUNT not in a.directory.resolve().parents:raise RuntimeError('Mounted SanStand required')
    print(json.dumps(run(a.directory),default=G.as_json))
