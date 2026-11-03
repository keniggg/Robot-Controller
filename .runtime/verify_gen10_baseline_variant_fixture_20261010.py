import json,hashlib,os,sys,time
from pathlib import Path
import xml.etree.ElementTree as ET
w=Path(__file__).resolve().parents[1]
n=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_restart_20261010T002347Z')
out=n/'verification/gen10_baseline_variant_fixture';assert Path('/media/zhuyupei/SanStand').is_mount();out.mkdir(exist_ok=False);(out/'tmp').mkdir()
os.environ.update(TMPDIR=str(out/'tmp'),ROS_LOG_DIR=str(out/'ros'),PYTHONDONTWRITEBYTECODE='1',PYTHONNOUSERSITE='1')
sys.dont_write_bytecode=True
paths=[w/'src/alicia_grasp_modes/src',w/'src/alicia_flexible_grasp_supervisor/src',w/'src/alicia_grasp_modes/tests',w/'src/alicia_flexible_grasp_supervisor/tests']
sys.path[:0]=list(map(str,paths));os.environ['ROS_PACKAGE_PATH']=str(w/'src')+':'+os.environ.get('ROS_PACKAGE_PATH','')
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
base=json.loads((n/'silhouette_coverage_deployment/result.json').read_text())['source_sha256'];assert len(base)==150;assert all(sha(w/p)==h for p,h in base.items())
import pytest
args=['-q','-p','no:cacheprovider','--basetemp',str(out/'pytest_tmp'),str(w/'src/alicia_grasp_modes/tests/test_observation_variant_local_reuse.py'),'--junitxml',str(out/'tests.xml')]
with (out/'pytest.log').open('w') as f:
 oldout,olderr=sys.stdout,sys.stderr
 try:sys.stdout=sys.stderr=f;rc=pytest.main(args)
 finally:sys.stdout,sys.stderr=oldout,olderr
import test_remote_modes
from alicia_grasp_modes import silhouette_observation_planning
mods={'mode_remote':test_remote_modes.module,'base_remote':test_remote_modes.base,'planning':silhouette_observation_planning}
audit={k:dict(path=str(Path(v.__file__).resolve()),sha256=sha(Path(v.__file__).resolve())) for k,v in mods.items()};assert all(x['path'].startswith(str(w/'src')+'/') for x in audit.values())
cases=list(ET.parse(str(out/'tests.xml')).getroot().iter('testcase'))
failures=[dict(c.attrib,detail=(c.find('failure').text if c.find('failure') is not None else c.find('error').text)) for c in cases if c.find('failure') is not None or c.find('error') is not None]
result=dict(baseline=150,actual_test_count=len(cases),returncode=int(rc),failures=failures,source_audit=audit,live_unchanged=all(sha(w/p)==h for p,h in base.items()),production_edits=False,physical_actions=False)
(out/'result.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(dict(baseline150=True,tests=len(cases),failures=len(failures),live_unchanged=result['live_unchanged'],report=str(out/'result.json'))))
