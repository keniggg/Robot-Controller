from pathlib import Path
import hashlib,json,os,subprocess,time,sys
W=Path.cwd();S=W/'.runtime/gen24_diagonal_connectivity_staging_20261011';O=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_restart_20261010T002347Z/gen24_analysis/fragment_ownership');assert Path('/media/zhuyupei/SanStand').is_mount()
paths=['src/alicia_grasp_modes/src/alicia_grasp_modes/tabletop.py','src/alicia_grasp_modes/src/alicia_grasp_modes/arrival_fragment_group.py','src/alicia_grasp_modes/tests/test_diagonal_complete_fragment_owner.py'];sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest();before={str(S/p):sha(S/p)for p in paths}
env=os.environ.copy();env['PYTHONPATH']=str(S/'src/alicia_grasp_modes/src')+':'+str(W/'src/alicia_flexible_grasp_supervisor/tests')+':'+env.get('PYTHONPATH','');env.update(PYTHONNOUSERSITE='1',PYTHONDONTWRITEBYTECODE='1',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1')
cmd=['/usr/bin/python3','-m','pytest','-q',str(S/paths[-1]),'--junitxml='+str(O/'owner_targeted_final.xml'),'-o','cache_dir=/tmp/gen24-owner-final'];start=time.time()
with (O/'owner_targeted_final.log').open('w')as f:r=subprocess.run(cmd,stdout=f,stderr=subprocess.STDOUT,env=env)
with (O/'final_owner_replay_r2.log').open('w')as f:replay=subprocess.run(['/usr/bin/python3',str(W/'.runtime/gen24_final_owner_replay_20261011.py')],stdout=f,stderr=subprocess.STDOUT,env=env)
after={str(S/p):sha(S/p)for p in paths}
m=dict(command=cmd,test_scope=[str(S/paths[-1])],source_sha256=before,source_after_sha256=after,source_unchanged=before==after,exit_code=r.returncode,replay_exit_code=replay.returncode,xml=str(O/'owner_targeted_final.xml'),log=str(O/'owner_targeted_final.log'),wall_sec=time.time()-start)
(O/'owner_targeted_final_manifest.json').write_text(json.dumps(m,indent=2)+'\n');print(json.dumps(dict(exit_code=r.returncode,replay_exit_code=replay.returncode,source_unchanged=before==after,wall_sec=m['wall_sec'])));sys.exit(r.returncode)
