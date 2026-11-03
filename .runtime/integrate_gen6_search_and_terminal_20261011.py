"""Integrate frozen offline deliveries after reviews; no ROS commands."""
import ast
import hashlib
import json
from pathlib import Path
import shutil
import time

W=Path(__file__).resolve().parents[1]
N=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_no_skin_20261011T000625Z')
sha=lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
stages=[W/'.runtime/gen6_terminal_reason_20261011',W/'.runtime/gen6_observation_search_coverage_20261011']
files={}; tests={}; bases={}; scopes=[]; deliveries=[]
for stage in stages:
 p=stage/'delivery.json'; d=json.loads(p.read_text()); assert d['ready'] and d['source_frozen']
 deliveries.append(dict(path=str(p),sha256=sha(p)))
 for key,target in [('files',files),('test_files',tests)]:
  for f,h in d[key].items():
   assert f not in target and sha(stage/f)==h
   expected=d['base_sha256'][f]
   assert ((not (W/f).exists()) if expected is None else sha(W/f)==expected), f
   if f.endswith('.py'): ast.parse((stage/f).read_text())
   target[f]=h; bases[f]=expected
 scopes.extend(d['test_scope'])
backup=W/'.runtime'/('gen6_integration_before_'+str(time.time_ns()))
backup.mkdir()
for stage in stages:
 d=json.loads((stage/'delivery.json').read_text())
 for f in {**d['files'],**d['test_files']}:
  if (W/f).exists():
   (backup/f).parent.mkdir(parents=True,exist_ok=True);shutil.copy2(W/f,backup/f)
  (W/f).parent.mkdir(parents=True,exist_ok=True);shutil.copy2(stage/f,W/f)
for f,h in {**files,**tests}.items(): assert sha(W/f)==h
out=W/'.runtime/gen6_search_terminal_integrated_delivery.json'
assert not out.exists()
out.write_text(json.dumps(dict(ready=True,source_frozen=True,files=files,test_files=tests,
 base_sha256=bases,test_scope=list(dict.fromkeys(scopes)),components=deliveries,
 backup=str(backup),deployed=False),indent=2)+'\n')
print(json.dumps(dict(delivery=str(out),backup=str(backup),files=len(files),tests=len(tests))))
