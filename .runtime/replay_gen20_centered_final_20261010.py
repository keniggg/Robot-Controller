from pathlib import Path
import hashlib,json,tarfile,time,types,runpy
import numpy as np
from alicia_grasp_modes import reachable_observation_views as new
from alicia_grasp_modes.observation_arrival_projection import validate_arrival_projection
WORK=Path(__file__).resolve().parents[1];STAGE=WORK/'.runtime/gen20_centered_observation_20261010';OUT=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_restart_20261010T002347Z/verification/gen20_centered_observation')
prod='src/alicia_grasp_modes/src/alicia_grasp_modes/reachable_observation_views.py';test='src/alicia_grasp_modes/tests/test_observation_prism_centre_preference.py'
fixture=runpy.run_path(str(WORK/test));args,error=fixture['gen20_source'].__wrapped__()
with tarfile.open(str(STAGE/'source_before.tar')) as archive:raw=archive.extractfile(prod).read()
old=types.ModuleType('alicia_grasp_modes._gen20_original');old.__package__='alicia_grasp_modes';exec(compile(raw,'source_before.tar:'+prod,'exec'),old.__dict__)
records=[]
for label,module in [('exact_baseline',old),('final_patch',new)]:
 started=time.monotonic();cpu=time.thread_time();rows,audit=module.generate_reachable_observation_views(**args);record=dict(label=label,wall_sec=time.monotonic()-started,thread_cpu_sec=time.thread_time()-cpu,audit=audit,candidates=[])
 for row in rows:
  camera=args['source']['camera']
  try:perturbed=validate_arrival_projection(args['source']['arrival_projection'],np.asarray(row['T_base_camera_link'])@error,[camera[k] for k in ('fx','fy','cx','cy')],camera['width'],camera['height'])
  except ValueError as e:perturbed={'ok':False,'reason':str(e)}
  record['candidates'].append(dict(proposal=row,reapplied_recorded_relative_camera_error=perturbed))
 records.append(record);print(label,len(rows),record['wall_sec'],[(r['proposal']['endpoint_checks']['arrival_projection_endpoint']['minimum_image_edge_distance_px'],r['reapplied_recorded_relative_camera_error']['ok']) for r in record['candidates']])
assert records[0]['candidates'] and records[1]['candidates'];assert not records[0]['candidates'][0]['reapplied_recorded_relative_camera_error']['ok'];assert all(r['reapplied_recorded_relative_camera_error']['ok'] for r in records[1]['candidates'])
result=dict(source_sha256={name:hashlib.sha256((WORK/name).read_bytes()).hexdigest() for name in (prod,test)},baseline_source_sha256=hashlib.sha256(raw).hexdigest(),records=records,limitations=['No hardware action, new MoveIt scene/path checks, execution or grasp success is established by this offline endpoint-proposal replay.','The recorded relative camera error is reused only as a sensitivity regression; no future following or calibration guarantee.','Current visible box silhouette differs from the frozen historical prism and required rectangular domain; ROI validation remains unchanged.','Proposal objective uses centering only; existing joint-error box is unchanged and still used for support clearance, not the new soft objective.','Depth and mask exactly match recorded SHA; support cloud camera-coordinate arithmetic has different final floating bits from original optical conversion; pixel membership count matches.'])
p=OUT/'sealed_candidate_replay.json';p.write_text(json.dumps(result,indent=2)+'\n');print(p,hashlib.sha256(p.read_bytes()).hexdigest())
