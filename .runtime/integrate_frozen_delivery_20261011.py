"""Copy an explicitly frozen delivery after baseline verification; no ROS."""
import argparse
import ast
import hashlib
import json
from pathlib import Path
import shutil
import time

ap=argparse.ArgumentParser()
ap.add_argument('--delivery',type=Path,required=True)
a=ap.parse_args()
w=Path(__file__).resolve().parents[1]
d=json.loads(a.delivery.read_text())
assert d.get('ready',d.get('delivery_ready')) and d['source_frozen']
s=Path(d['stage']).resolve();assert w in s.parents
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
files={**d['files'],**d['test_files']}
for f,h in files.items():
 assert not Path(f).is_absolute() and '..' not in Path(f).parts
 assert sha(s/f)==h,('staging drift',f)
 b=d['base_sha256'][f]
 assert ((not (w/f).exists()) if b is None else sha(w/f)==b),('baseline drift',f)
 if f.endswith('.py'):ast.parse((s/f).read_text())
backup=w/'.runtime'/('before_'+s.name+'_'+str(time.time_ns()));backup.mkdir()
for f,h in files.items():
 if (w/f).exists():
  (backup/f).parent.mkdir(parents=True,exist_ok=True);shutil.copy2(w/f,backup/f)
 (w/f).parent.mkdir(parents=True,exist_ok=True);shutil.copy2(s/f,w/f)
 assert sha(w/f)==h
receipt=dict(delivery=str(a.delivery.resolve()),delivery_sha256=sha(a.delivery),
 backup=str(backup),source_sha256=files,deployed=False)
(backup/'integration_receipt.json').write_text(json.dumps(receipt,indent=2)+'\n')
print(json.dumps(dict(files=len(files),backup=str(backup),delivery_sha256=sha(a.delivery))))
