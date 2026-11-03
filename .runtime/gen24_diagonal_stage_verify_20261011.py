from pathlib import Path
import json,hashlib,subprocess,sys,os,time
W=Path.cwd();S=W/'.runtime/gen24_diagonal_connectivity_staging_20261011';O=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_restart_20261010T002347Z/gen24_analysis/fragment_ownership');assert Path('/media/zhuyupei/SanStand').is_mount()
names=['test_bound_diagonal_depth_connectivity.py','test_tabletop.py','test_tabletop_plane_consensus.py','test_tabletop_support_visibility_recovery.py','test_observation_measurement_unknown.py','test_observation_arrival_station.py','test_arrival_matching_fragments.py','test_arrival_same_instance_confirmation.py','test_arrival_matching_fragment_tracking.py','test_arrival_fragment_recovery_guards.py','test_station_compute_wait_producer.py','test_contact_station_retirement.py','test_silhouette_snapshot_diagnostic.py']
tests=[S/'src/alicia_grasp_modes/tests'/n for n in names]
production=[S/p for p in json.loads((S/'baseline.json').read_text())]
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest();sources={str(p):sha(p)for p in production+tests}
env=os.environ.copy();env['PYTHONPATH']=str(S/'src/alicia_grasp_modes/src')+':'+str(W/'src/alicia_flexible_grasp_supervisor/tests')+':'+env.get('PYTHONPATH','');env.update(PYTHONNOUSERSITE='1',PYTHONDONTWRITEBYTECODE='1',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1')
command=['/usr/bin/python3','-m','pytest','-q',*[str(x)for x in tests], '--junitxml='+str(O/'combined_r1.xml'),'-o','cache_dir=/tmp/gen24-diagonal-final']
start=time.time()
with (O/'combined_r1.log').open('w')as f:r=subprocess.run(command,stdout=f,stderr=subprocess.STDOUT,env=env)
after={str(p):sha(p)for p in production+tests}
manifest=dict(command=command,test_scope=[str(x)for x in tests],source_sha256=sources,source_after_sha256=after,source_unchanged=sources==after,exit_code=r.returncode,wall_sec=time.time()-start,xml=str(O/'combined_r1.xml'),log=str(O/'combined_r1.log'),production_unchanged_from_baseline={p:sha(W/p)==h for p,h in json.loads((S/'baseline.json').read_text()).items()})
(O/'combined_r1_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n');print(json.dumps(dict(exit_code=r.returncode,source_unchanged=sources==after,wall_sec=manifest['wall_sec'])));sys.exit(r.returncode)
