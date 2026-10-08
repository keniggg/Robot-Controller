"""Bounded, read-only resource telemetry correlated with the active attempt."""
import json,time,os
from pathlib import Path
p=Path(json.loads(Path('.runtime/latest_candidate_real_grasp.json').read_text())['directory'])
assert Path('/media/zhuyupei/SanStand').is_mount()
os.environ['ROS_LOG_DIR']=str(p/'ros')
import rospy
from alicia_flexible_grasp_supervisor.msg import GraspState
rospy.init_node('capture_resource_observer',anonymous=True,disable_signals=True)
state={'active_seen':False,'terminal':False}
def cb(m):
 if m.active:state['active_seen']=True
 elif state['active_seen']:state['terminal']=True
rospy.Subscriber('/grasp/state',GraspState,cb,queue_size=5)
def read(path):
 try:return Path(path).read_text().strip()
 except OSError as e:return str(e)
paths=['/proc/pressure/cpu','/proc/pressure/io','/proc/pressure/memory','/proc/loadavg','/proc/diskstats','/proc/stat',
 '/sys/bus/usb/devices/4-1/power/control','/sys/bus/usb/devices/4-1/power/runtime_status','/sys/bus/usb/devices/3-2/power/control']
with (p/'capture_resources.jsonl').open('w',buffering=1) as f:
 start=time.monotonic()
 while time.monotonic()-start<240 and not state['terminal'] and not rospy.is_shutdown():
  row=dict(wall_ns=time.time_ns(),monotonic_ns=time.monotonic_ns(),values={v:read(v) for v in paths})
  f.write(json.dumps(row)+'\n');time.sleep(1)
print(json.dumps(state))
