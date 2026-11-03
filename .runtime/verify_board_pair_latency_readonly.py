#!/usr/bin/env python3
"""Read-only current exact-pair verification: never run/execute/bootstrap/publish."""
import importlib.util,json,os,time,hashlib
from pathlib import Path
from types import SimpleNamespace
from datetime import datetime,timezone
root=Path(__file__).resolve().parents[1]
out=Path(json.loads((root/'.runtime/current_grasp_repair_resume.json').read_text())['directory'])/('pair_readonly_'+datetime.now(timezone.utc).strftime('%H%M%S'))
out.mkdir();(out/'ros').mkdir();os.environ['ROS_LOG_DIR']=str(out/'ros')
spec=importlib.util.spec_from_file_location('readonly_diagnostic',root/'.runtime/bounded_joint_response_diagnostic.py');mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod)
mod.rospy.init_node('readonly_exact_board_pair_verification',anonymous=True,disable_signals=True)
# bootstrap_pending only permits reading an initially pending state in guard().
# No run(), bootstrap_once(), execute_step(), or publisher exists in this program.
d=mod.Diagnostic(SimpleNamespace(execute=False,bootstrap_pending=True),out)
assert d.publisher is None
rows=[]
try:
 start=time.monotonic()
 while time.monotonic()-start<3:
  try:d.guard();break
  except RuntimeError:time.sleep(.02)
 for i in range(6):
  begin=time.monotonic()
  try:
   g=d.snapshot('readonly_%d'%i)
   q=d.quality[g['source_stamp_ns']]
   rows.append(dict(index=i,success=True,acquisition_sec=time.monotonic()-begin,source_stamp_ns=g['source_stamp_ns'],age_at_return_ms=(mod.rospy.Time.now().to_nsec()-g['source_stamp_ns'])/1e6,quality=q))
  except Exception as e:
   rows.append(dict(index=i,success=False,acquisition_sec=time.monotonic()-begin,error=str(e)))
  time.sleep(.22)
finally:d.close()
result=dict(readonly=True,publisher_created=False,motion_methods_called=False,source_sha256=hashlib.sha256((root/'.runtime/bounded_joint_response_diagnostic.py').read_bytes()).hexdigest(),rows=rows)
(out/'result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2))
print(json.dumps(dict(directory=str(out),success_count=sum(r['success'] for r in rows),attempt_count=len(rows),rows=[{k:v for k,v in r.items() if k!='quality'} for r in rows]),ensure_ascii=False,indent=2))
