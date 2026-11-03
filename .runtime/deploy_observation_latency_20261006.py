from pathlib import Path
from datetime import datetime,timezone
import hashlib,json,subprocess,sys,os
w=Path(__file__).resolve().parents[1];m=Path('/media/zhuyupei/SanStand');assert m.is_mount()
man=Path(json.loads((w/'.runtime/current_observation_latency_validation_20261006.json').read_text())['manifest']);j=json.loads(man.read_text());assert j['tests_passed'] and j['source_unchanged']
for f,h in j['files'].items(): assert hashlib.sha256((w/f).read_bytes()).hexdigest()==h,f
assert j.get('reviewed') is True, 'independent review required before loading'
out=m/'实验记录/Robot-Controller/ros_log'/('observation_latency_deployment_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ'))
env=dict(os.environ,ROS_LOG_DIR=str(out/'ros'))
argv=[sys.executable,str(w/'.runtime/load_observation_latency_20261006.py'),'--tested-sha256',j['files']['src/alicia_grasp_modes/scripts/mode_aware_remote_grasp6d.py'],'--output',str(out)]
r=subprocess.run(argv,cwd=w,env=env,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True)
assert out.is_dir()
(out/'loader_console.log').write_text(r.stdout);result=json.loads((out/'result.json').read_text()) if (out/'result.json').exists() else {}
unchanged=all(hashlib.sha256((w/f).read_bytes()).hexdigest()==h for f,h in j['files'].items())
summary=dict(directory=str(out),returncode=r.returncode,passed=result.get('deployment_passed'),error=result.get('error'),unit=result.get('unit'),tested_source_unchanged=unchanged)
(w/'.runtime/current_observation_latency_deployment_20261006.json').write_text(json.dumps(summary,indent=2)+'\n');print(json.dumps(summary,ensure_ascii=False),flush=True)
sys.exit(r.returncode or (0 if unchanged else 2))
