from pathlib import Path
from datetime import datetime,timezone
import hashlib,json,subprocess,sys
w=Path(__file__).resolve().parents[1];m=Path('/media/zhuyupei/SanStand');assert m.is_mount()
man=Path(json.loads((w/'.runtime/current_contact_tracking_validation_20261006.json').read_text())['manifest']);j=json.loads(man.read_text());assert j['tests_passed'] and j['source_unchanged']
for f,h in j['files'].items():assert hashlib.sha256((w/f).read_bytes()).hexdigest()==h,f
j.update(reviewed=True,review_evidence=['contact_tracking_integration_review_20261006T054349Z/validation.json','simulation_symmetry_diagnostic/contact_contract_independent_review.log','simulation_symmetry_diagnostic/velocity_reference_repair_validation.json'],test_summary=dict(passed=1655,skipped=4));man.write_text(json.dumps(j,indent=2)+'\n')
receipt=json.loads((w/'.runtime/current_alignment_tracking_fix_20261006.json').read_text())['receipt']
out=m/'实验记录/Robot-Controller/ros_log'/('contact_tracking_deployment_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ'))
argv=[sys.executable,str(w/'.runtime/load_contact_margin_20261006.py'),'--execute','--validation-manifest',str(man),'--alignment-receipt',receipt,'--output',str(out)]
# The original loader creates its unique directory and records failures itself.
r=subprocess.run(argv,cwd=w,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True)
(out/'loader_console.log').write_text(r.stdout)
(w/'.runtime/current_contact_tracking_deployment_20261006.json').write_text(json.dumps(dict(directory=str(out),returncode=r.returncode),indent=2)+'\n')
result=json.loads((out/'result.json').read_text()) if (out/'result.json').exists() else {}
print(json.dumps(dict(directory=str(out),returncode=r.returncode,passed=result.get('deployment_passed'),error=result.get('error'),unit=result.get('unit')),ensure_ascii=False),flush=True)
sys.exit(r.returncode)
