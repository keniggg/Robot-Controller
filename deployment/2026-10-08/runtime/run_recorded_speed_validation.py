"""Record one new user-authorized grasp using the existing mode runner."""
import os,json,time,subprocess
from pathlib import Path
from datetime import datetime,timezone
if not os.path.ismount('/media/zhuyupei/SanStand'):raise RuntimeError('SanStand absent')
out=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log')/('feedforward_speed_grasp_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ'));out.mkdir();(out/'ros').mkdir()
os.environ['ROS_LOG_DIR']=str(out/'ros')
Path('.runtime/current_grasp_diagnostic.json').write_text(json.dumps({'directory':str(out)},ensure_ascii=False)+'\n')
scene_log=(out/'scene_recorder.log').open('w')
scene=subprocess.Popen(['/usr/bin/python3','-u','.runtime/record_grasp_scene_diagnostic.py'],stdout=scene_log,stderr=subprocess.STDOUT,start_new_session=True)
for i in range(100):
 if (out/'scene_recorder_ready.json').exists():break
 if scene.poll() is not None:raise RuntimeError('scene recorder failed')
 time.sleep(.1)
else:raise RuntimeError('scene recorder not ready')
record=dict(directory=str(out),scene_recorder_pid=scene.pid,started_ns=time.time_ns(),purpose='user-authorized fresh selected object grasp with .008 feedforward timing')
Path('.runtime/latest_candidate_real_grasp.json').write_text(json.dumps(record,ensure_ascii=False,indent=2)+'\n')
print('DIAGNOSTIC_DIRECTORY',out,flush=True)
with (out/'launch.log').open('w') as log:
 attempt=subprocess.Popen(['/usr/bin/python3','-u','tools/run_recorded_grasp.py','--duration','900','--','/usr/bin/python3','-u','.runtime/execute_selected_candidate_grasp.py'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
 record['attempt_pid']=attempt.pid
 Path('.runtime/latest_candidate_real_grasp.json').write_text(json.dumps(record,ensure_ascii=False,indent=2)+'\n')
 print('ATTEMPT_PID',attempt.pid,flush=True)
 rc=attempt.wait()
print('RECORDED_ATTEMPT_EXIT',rc,flush=True)
