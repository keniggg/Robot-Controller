#!/usr/bin/env python3
"""Offline alternate same-core first crop; never changes production files."""
from pathlib import Path
import argparse,importlib.util,inspect,hashlib,json
from alicia_grasp_modes import rgb_instance
parser=argparse.ArgumentParser();parser.add_argument('--factor',type=int,choices=(2,3),required=True);args=parser.parse_args()
path=Path(rgb_instance.__file__);before=hashlib.sha256(path.read_bytes()).hexdigest()
original=inspect.getsource(rgb_instance.instance_mask_from_seed)
needle='result, evidence = attempt(margin)'
assert original.count(needle)==1
counterfactual=original.replace(needle,f'result, evidence = attempt(({args.factor} if observation_arrival_retry else 1) * margin)')
namespace=dict(rgb_instance.__dict__);exec(compile(counterfactual,'<same_first_core_crop_counterfactual>','exec'),namespace)
rgb_instance.instance_mask_from_seed=namespace['instance_mask_from_seed']
spec=importlib.util.spec_from_file_location('gen12_identity_replay',Path(__file__).with_name('replay_gen12_arrival_identity.py'))
replay=importlib.util.module_from_spec(spec);spec.loader.exec_module(replay)
replay.OUT=replay.OUT.parent/f'crop_counterfactual_{16*args.factor}'
replay.main()
result_path=replay.OUT/'result.json';result=json.loads(result_path.read_text())
result['counterfactual']=dict(initial_search_margin_factor=args.factor,first_measured_core_unchanged=True,depth_unchanged=True,other_guards_unchanged=True,production_source_sha256_before=before,production_source_sha256_after=hashlib.sha256(path.read_bytes()).hexdigest(),code_change=needle+' -> '+f'result, evidence = attempt(({args.factor} if observation_arrival_retry else 1) * margin)',not_production_behavior=True)
result['limitations'].append('This diagnostic replaces the first search crop only in an in-memory function; actual production still starts at 16 px. It does not establish a safe generic retry trigger.')
result_path.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
assert before==result['counterfactual']['production_source_sha256_after']
