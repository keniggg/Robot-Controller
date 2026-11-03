#!/usr/bin/env python3
"""Offline counterfactual from frozen original RGB function; no ROS connections."""
import argparse, hashlib, inspect, json
from pathlib import Path
import alicia_grasp_modes.rgb_instance as rgb_instance

p=argparse.ArgumentParser();p.add_argument('--production',action='store_true');p.add_argument('--record',choices=('gen9','gen6'),required=True);p.add_argument('--policy',choices=('directional','symmetric'),required=True);a=p.parse_args()
baseline_path=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_ros_20261009T003407Z/verification/arrival_rgb_directional_retry/before_rgb_instance.py')
baseline=baseline_path.read_text()
source=baseline[baseline.index('def instance_mask_from_seed('):baseline.index('\ndef refine_candidate(')].rstrip()
assert 'result, evidence = attempt(3 * margin)' in source
sha=hashlib.sha256(baseline_path.read_bytes()).hexdigest()
source=source.replace('    def attempt(search_margin):\n','    last_boundary = {}\n    def attempt(search_margin):\n        nonlocal last_boundary\n        margins = dict.fromkeys(("left", "top", "right", "bottom"), search_margin) if isinstance(search_margin, int) else search_margin\n')
source=source.replace('int(xs.min()) - search_margin', 'int(xs.min()) - margins["left"]').replace('int(xs.max()) + search_margin + 1','int(xs.max()) + margins["right"] + 1').replace('int(ys.min()) - search_margin','int(ys.min()) - margins["top"]').replace('int(ys.max()) + search_margin + 1','int(ys.max()) + margins["bottom"] + 1')
source=source.replace('        if (np.any(chosen[3]) or np.any(chosen[-4])','        last_boundary = dict(top=bool(np.any(chosen[3])), bottom=bool(np.any(chosen[-4])), left=bool(np.any(chosen[:, 3])), right=bool(np.any(chosen[:, -4])))\n        if (np.any(chosen[3]) or np.any(chosen[-4])')
source=source.replace('rgb_search_margin_px=int(search_margin),','rgb_search_margin_px=max(margins.values()), rgb_search_margins_px=margins.copy(),')
third='4 * margin' if a.policy=='symmetric' else '{side: (4 if touched else 3) * margin for side, touched in last_boundary.items()}'
source=source.replace('result, evidence = attempt(3 * margin)','result, evidence = attempt('+third+')')
namespace=dict(rgb_instance.__dict__);exec(compile(source,'<offline_counterfactual_rgb_function>','exec'),namespace)
if not a.production:rgb_instance.instance_mask_from_seed=namespace['instance_mask_from_seed']
else:sha=hashlib.sha256(Path(rgb_instance.__file__).read_bytes()).hexdigest()
base=Path('.runtime/replay_current_gen9_arrival_rgb_boundary.py' if a.record=='gen9' else '.runtime/replay_gen6_arrival_rgb_boundary.py').read_text()
base=base.replace("OUTPUT=ROOT/'ros_log/full_latest_ros_20261009T003407Z/gen9_arrival_analysis/rgb_boundary_replay'", "OUTPUT=ROOT/'ros_log/full_latest_ros_20261009T003407Z/gen9_arrival_analysis/"+a.record+'_'+a.policy+"'")
base=base.replace("OUTPUT=ROOT/'ros_log/full_latest_ros_20261009T003407Z/gen6_arrival_rgb_boundary'", "OUTPUT=ROOT/'ros_log/full_latest_ros_20261009T003407Z/gen9_arrival_analysis/"+a.record+'_'+a.policy+"'")
old='''            margin=16*(len(attempts)+1)  # Current 653x489 image rounds to16.
            x0=max(0,int(xs.min())-margin);x1=min(rgb.shape[1],int(xs.max())+margin+1)
            y0=max(0,int(ys.min())-margin);y1=min(rgb.shape[0],int(ys.max())+margin+1)'''
new='''            margin=16*(len(attempts)+1)
            margins=dict.fromkeys(('left','top','right','bottom'),margin)
            if len(attempts)==2:
                margins={side:(64 if POLICY=='symmetric' or attempts[-1]['boundary_pixels'].get(side,0) else 48) for side in margins}
                margin=max(margins.values())
            x0=max(0,int(xs.min())-margins['left']);x1=min(rgb.shape[1],int(xs.max())+margins['right']+1)
            y0=max(0,int(ys.min())-margins['top']);y1=min(rgb.shape[0],int(ys.max())+margins['bottom']+1)'''
assert old in base;base=base.replace(old,new)
base=base.replace('evidence=dict(margin_px=margin,box_xyxy=', 'evidence=dict(margin_px=margin,margins_px=margins,box_xyxy=')
base=base.replace("scope='offline same-frame diagnostic; original segmentation and RGB guards unchanged'", "scope='offline counterfactual bounded third crop; all original segmentation and acceptance guards unchanged',counterfactual_policy=POLICY,original_rgb_source_sha256=ORIGINAL_RGB_SHA")
if a.production:
 base=base.replace('/'+a.record+'_'+a.policy+"'", '/production_'+a.record+'_'+a.policy+"'")
 base=base.replace('offline counterfactual bounded third crop', 'actual production bounded third crop')
ns={'__name__':'replay_counterfactual','__file__':__file__,'POLICY':a.policy,'ORIGINAL_RGB_SHA':sha}
exec(compile(base,'<offline_counterfactual_replay>','exec'),ns);ns['main']()
