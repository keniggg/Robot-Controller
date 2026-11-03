from pathlib import Path
import hashlib,json,sys,os,time
import xml.etree.ElementTree as ET
W=Path(__file__).resolve().parents[1]
S=W/'.runtime/gen10_supplement_handoff_authority_20261012'
OUT=Path(sys.argv[1]);OUT.mkdir(parents=True,exist_ok=False)
os.environ['ROS_LOG_DIR']=str(OUT/'ros')
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
paths=[S/'src/alicia_grasp_modes/tests',S/'src/alicia_flexible_grasp_supervisor/tests',S/'src/alicia_grasp_modes/src',S/'src/alicia_flexible_grasp_supervisor/src']
sys.path[:0]=list(map(str,paths))
import test_task_modes
from alicia_grasp_modes import station_compute_wait_task as task
imports={k:str(v) for k,v in dict(mode=Path(test_task_modes.module.__file__).resolve(),base=Path(test_task_modes.base.__file__).resolve(),mixin=Path(task.__file__).resolve()).items()}
assert all(str(S)+'/' in p for p in imports.values()),imports
imports={k:dict(path=p,sha256=sha(p)) for k,p in imports.items()}
(OUT/'imports.json').write_text(json.dumps(imports,indent=2)+'\n')
keywords=('station','observe_return','initial_preflight','authority','measurement','supplement','task_modes','gen10')
scope=sorted(p for p in (S/'src/alicia_grasp_modes/tests').glob('test_*.py') if any(k in p.name for k in keywords))
scope += sorted(p for p in (S/'src/alicia_flexible_grasp_supervisor/tests').glob('test_*.py') if 'authority' in p.name or 'execution_plan' in p.name)
if len(sys.argv)>2 and sys.argv[2]=='--owner':
 scope=[S/'src/alicia_grasp_modes/tests'/name for name in ('test_gen10_station_handoff_authority.py','test_station_compute_wait_task.py')]
scope=list(dict.fromkeys(scope));rels=[str(p.relative_to(S)) for p in scope]
(OUT/'scope.json').write_text(json.dumps(rels,indent=2)+'\n')
print('IMPORTS',json.dumps(imports),flush=True)
print('SCOPE_FILES',len(scope),flush=True)
import pytest
started=time.time();result=pytest.main(['-q','-p','no:cacheprovider','--junitxml='+str(OUT/'pytest.xml'),'--basetemp='+str(OUT/'tmp'),*map(str,scope)])
counts=dict(tests=0,failures=0,errors=0,skipped=0)
if (OUT/'pytest.xml').exists():
 for node in ET.parse(OUT/'pytest.xml').getroot().iter('testsuite'):
  for k in counts:counts[k]+=int(node.attrib.get(k,0))
unchanged=all(sha(v['path'])==v['sha256'] for v in imports.values())
record=dict(passed=result==0 and unchanged,exit_code=int(result),counts=counts,elapsed_sec=time.time()-started,imports=imports,scope=rels,imports_unchanged=unchanged)
(OUT/'result.json').write_text(json.dumps(record,indent=2)+'\n')
print('RESULT',json.dumps(record),flush=True)
sys.exit(int(result))
