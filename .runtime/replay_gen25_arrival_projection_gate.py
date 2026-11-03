"""Real sealed endpoint counterfactual through the new production prism gate."""
import copy
import argparse
import hashlib
import json
from pathlib import Path
import time
import numpy as np
from alicia_grasp_modes import observation_arrival_projection as module

BASE=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_ros_20261009T003407Z')
D=BASE/'gen25_handoff_analysis'
parser=argparse.ArgumentParser()
parser.add_argument('--output-subdir',default='gen25_gate_replay')
args=parser.parse_args()
if not args.output_subdir.replace('_','').isalnum():
    raise ValueError('output must be a simple verification subdirectory')
OUT=BASE/'verification/observation_arrival_projection'/args.output_subdir
OUT.mkdir(parents=True,exist_ok=True)
def sha(p): return hashlib.sha256(Path(p).read_bytes()).hexdigest()
source_path=Path(module.__file__)
source_before=sha(source_path)
projection=json.loads((D/'planned_actual_projection.json').read_text())
manifest=json.loads((D/'source_images/manifest.json').read_text())
arrival=json.loads((D/'arrival_and_terminal.json').read_text())
statuses=json.loads((D/'detector_status.json').read_text())
camera=manifest['camera']
K=[camera[k] for k in ('fx','fy','cx','cy')]
rows=[]
begin=time.monotonic()
for v in projection['views']:
    idx=v['view_index']; stamp=v['source_stamp_ns']
    originals=[s for s in statuses if s['message'].get('source_stamp_ns')==stamp and s['message'].get('state')=='ready']
    assert len(originals)==1
    original=originals[0]
    status=copy.deepcopy(original['message'])
    assert 'reference_object_height_m' not in status
    # Old recordings predate this explicit field. Bind only the original fixed
    # acquired height; this is an offline schema counterfactual, never a receipt.
    status['reference_object_height_m']=manifest['anchor_status']['geometry_metrics']['median_height_m']
    selection=arrival['views'][idx-1]['arrival_request']['selection']
    contract=module.capture_arrival_projection(status,selection,stamp,camera)
    assert contract['audit']['footprint_sha256']==projection['frozen_prism']['footprint_sha256']
    assert contract['audit']['support_plane_sha256']==projection['frozen_prism']['support_plane_sha256']
    np.testing.assert_array_equal(contract['prism_base'],projection['frozen_prism']['corners_base'])
    results={}
    for kind,T in v['camera_transforms'].items():
        start=time.monotonic()
        try:
            value=module.validate_arrival_projection(contract,T,K,camera['width'],camera['height'],roi_margin_px=36.,minimum_depth_m=.07,maximum_depth_m=.5)
            results[kind]=dict(accepted=True,evidence=value)
        except ValueError as exc:
            results[kind]=dict(accepted=False,reason=str(exc))
        results[kind]['wall_sec']=time.monotonic()-start
    expected=dict(nominal_proposal=idx<3,strict_goal=idx<3,measured_exact_source=idx==1)
    assert {k:r['accepted'] for k,r in results.items()}==expected
    rows.append(dict(view_index=idx,request_id=v['request_id'],plan_id=v['plan_id'],planning_source_stamp_ns=stamp,ready_status_bag_received_ns=original['received_ns'],exact_status_source=True,measured_camera_source_stamp_ns=v['measured_source_stamp_ns'],source_audit=contract['audit'],original_audit_path=v['audit_path'],original_audit_file_sha256=v['audit_file_sha256'],results=results,expected=expected))
source_after=sha(source_path)
assert source_before==source_after
out=dict(passed=True,production_source_path=str(source_path),production_sha256_before=source_before,production_sha256_after=source_after,source_unchanged=True,script_sha256=sha(__file__),wall_sec=time.monotonic()-begin,provenance_files={str(D/name):sha(D/name) for name in ('planned_actual_projection.json','source_images/manifest.json','detector_status.json','arrival_and_terminal.json')},camera_parameters=projection['camera_parameters'],rows=rows,limits=dict(image_margin_px=36.,minimum_depth_m=.07,maximum_depth_m=.5),scope='Nine endpoint checks only; real saved geometry/transforms, no ROS, no generation, no path planning or motion',limitations=['Old status lacked reference_object_height_m; explicitly added original acquired anchor median_height_m, not a new live permission or source.', 'Measured poses are diagnostic endpoint counterfactuals, not a claim that the new planned-endpoint gate checks future actual feedback.', 'View2 actual margin21.557px fails the36px endpoint gate although original measured arrival RGB/identity passed; those are different gates.', 'No first16 future seed is invented; the production endpoint validator checks frozen prism plus its original16px observed background. Original arrival checks remain necessary.'])
dest=OUT/'result.json';dest.write_text(json.dumps(out,ensure_ascii=False,indent=2)+'\n')
print(json.dumps(dict(path=str(dest),sha256=sha(dest),production_sha256=source_after,wall_sec=out['wall_sec'],rows=[dict(view_index=x['view_index'],results={k:dict(accepted=y['accepted'],reason=y.get('reason')) for k,y in x['results'].items()}) for x in rows]),ensure_ascii=False,indent=2))
