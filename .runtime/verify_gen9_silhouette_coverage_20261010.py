from pathlib import Path
import os,sys,json,hashlib
WORK=Path(__file__).resolve().parents[1]
STAGE=WORK/'.runtime/gen9_silhouette_coverage_staging_20261010'
OUT=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_restart_20261010T002347Z/verification/gen9_silhouette_coverage_targeted_r4')
assert Path('/media/zhuyupei/SanStand').is_mount()
OUT.mkdir(exist_ok=False)
os.environ.update(PYTHONDONTWRITEBYTECODE='1',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',ROS_LOG_DIR=str(OUT/'ros'))
sys.dont_write_bytecode=True
os.environ['ROS_PACKAGE_PATH']=str(STAGE/'src')+':'+os.environ.get('ROS_PACKAGE_PATH','')
sys.path[:0]=[str(STAGE/'src/alicia_flexible_grasp_supervisor/tests'),str(STAGE/'src/alicia_grasp_modes/tests'),str(STAGE/'src/alicia_grasp_modes/src'),str(STAGE/'src/alicia_flexible_grasp_supervisor/src')]
paths=['src/alicia_grasp_modes/src/alicia_grasp_modes/silhouette_observation_order.py','src/alicia_grasp_modes/src/alicia_grasp_modes/silhouette_observation_planning.py','src/alicia_flexible_grasp_supervisor/scripts/remote_grasp6d_node.py','src/alicia_grasp_modes/tests/test_silhouette_observation_coverage.py','src/alicia_grasp_modes/tests/fixtures/gen9_silhouette_coverage.json']
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
before={p:sha(STAGE/p) for p in paths}
import pytest
with (OUT/'pytest.log').open('w') as log:
 a,b=sys.stdout,sys.stderr
 try:
  sys.stdout=sys.stderr=log
  rc=pytest.main(['-q','--disable-warnings','-p','no:cacheprovider','--basetemp',str(OUT/'pytest_tmp'),str(STAGE/paths[3]),str(STAGE/'src/alicia_grasp_modes/tests/test_silhouette_observation_order.py'),str(STAGE/'src/alicia_grasp_modes/tests/test_silhouette_observation_planning.py'),'--junitxml',str(OUT/'tests.xml')])
 finally:sys.stdout,sys.stderr=a,b
assert before=={p:sha(STAGE/p) for p in paths}
from test_remote_modes import module,base
from alicia_grasp_modes import station_compute_wait_remote
from alicia_flexible_grasp.vision import latest_only_inference
sources={n:dict(path=m.__file__,sha256=sha(Path(m.__file__))) for n,m in [('mode',module),('base',base),('station',station_compute_wait_remote),('coordinator',latest_only_inference)]}
assert all(str(STAGE) in row['path'] for row in sources.values())
import xml.etree.ElementTree as ET
cases=list(ET.parse(OUT/'tests.xml').getroot().iter('testcase'))
result=dict(passed=rc==0,tests=len(cases),files=before,module_sources=sources,physical_actions=False,live_source_edited=False,
 limitations=['No recorded untested endpoint is asserted reachable; no timeout or collision gate changes.',
 'Exact recorded endpoint poses and existing ordering metadata; controlled synthetic strict responses exercise original Mode loop and promotion.',
 'Real IK, whole-path requalification and physical grasp remain unverified.'])
(OUT/'result.json').write_text(json.dumps(result,indent=2,sort_keys=True))
print(json.dumps(result,indent=2))
raise SystemExit(rc)
