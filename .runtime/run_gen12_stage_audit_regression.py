from pathlib import Path
import subprocess,json,hashlib,sys
w=Path(__file__).resolve().parents[1]
stage=w/'.runtime/gen12_fused_enclosure_20261011'
ext=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_no_skin_20261011T000625Z/verification/gen12_fused_scene_audit')
assert Path('/media/zhuyupei/SanStand').is_mount()
ext.mkdir(exist_ok=True,parents=True)
prefix=ext/sys.argv[1]
assert not prefix.with_suffix('.log').exists()
files=['src/alicia_flexible_grasp_supervisor/scripts/remote_grasp6d_node.py','src/alicia_flexible_grasp_supervisor/tests/test_fused_scene_request_audit.py']
hashes=lambda:{x:hashlib.sha256((stage/x).read_bytes()).hexdigest() for x in files}
before=hashes()
cmd=['/usr/bin/python3','-m','pytest','-q',*sys.argv[2:],files[1],'-p','no:cacheprovider','--junitxml='+str(prefix.with_suffix('.xml'))]
with prefix.with_suffix('.log').open('x') as f:r=subprocess.run(cmd,cwd=stage,stdout=f,stderr=subprocess.STDOUT)
after=hashes()
prefix.with_suffix('.json').write_text(json.dumps(dict(source_sha256=before,source_after_sha256=after,source_unchanged=before==after,returncode=r.returncode,command=cmd),indent=2)+'\n')
print('\n'.join(prefix.with_suffix('.log').read_text().splitlines()[-65:]))
raise SystemExit(r.returncode if before==after else 2)
