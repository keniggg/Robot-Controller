#!/usr/bin/env python3
"""Offline label ablation on sealed frames: same hard core, current probable FG."""
from pathlib import Path
import importlib.util,inspect,hashlib,json
from alicia_grasp_modes import rgb_instance
path=Path(rgb_instance.__file__);before=hashlib.sha256(path.read_bytes()).hexdigest()
original=inspect.getsource(rgb_instance.instance_mask_from_seed)
needle='result, evidence = attempt(margin)';assert original.count(needle)==1
counterfactual=original.replace(needle,'result, evidence = attempt((3 if observation_arrival_retry else 1) * margin)')
needle='        near = cv2.dilate(measured_now, np.ones((15, 15), np.uint8)) > 0'
assert counterfactual.count(needle)==1
added='''
        if observation_arrival_retry and geometry is not None:
            sl = np.s_[ay0:ay1, ax0:ax1]
            current_foreground = (np.isfinite(depth[sl])
                & (depth[sl] >= config.depth_min_m) & (depth[sl] <= config.depth_max_m)
                & np.all(np.isfinite(points[sl]), axis=-1)
                & (heights[sl] >= config.foreground_min_m)
                & (heights[sl] <= config.foreground_max_m))
            near |= current_foreground
'''
counterfactual=counterfactual.replace(needle,needle+added)
namespace=dict(rgb_instance.__dict__);exec(compile(counterfactual,'<same_core_probable_fg_counterfactual>','exec'),namespace)
rgb_instance.instance_mask_from_seed=namespace['instance_mask_from_seed']
spec=importlib.util.spec_from_file_location('gen12_identity_replay',Path(__file__).with_name('replay_gen12_arrival_identity.py'))
replay=importlib.util.module_from_spec(spec);spec.loader.exec_module(replay)
replay.OUT=replay.OUT.parent/'crop48_current_measured_probable_fg'
replay.main()
result_path=replay.OUT/'result.json';result=json.loads(result_path.read_text())
result['counterfactual']=dict(initial_search_margin_factor=3,probable_foreground='original dilated selected seed union all valid current measured foreground heights inside current ROI',first_measured_hard_core_unchanged=True,measured_support_hard_background_unchanged=True,depth_unchanged=True,other_guards_unchanged=True,production_source_sha256_before=before,production_source_sha256_after=hashlib.sha256(path.read_bytes()).hexdigest(),not_production_behavior=True)
result['limitations'].append('Diagnostic only: adding probable foreground from other current components does not establish their identity or authorize a generic production retry. Other-object negatives are not covered here.')
result_path.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
assert before==result['counterfactual']['production_source_sha256_after']
