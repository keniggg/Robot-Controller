"""Load reviewed timing while idle; refresh only the Python gateway."""
import os, json, time, hashlib, xmlrpc.client, socket
from pathlib import Path
from datetime import datetime, timezone
socket.setdefaulttimeout(4)
root=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
if not os.path.ismount('/media/zhuyupei/SanStand'):raise RuntimeError('SanStand unavailable')
out=root/'ros_log'/('feedforward_speed_load_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ'))
out.mkdir();(out/'ros').mkdir();os.environ['ROS_LOG_DIR']=str(out/'ros')
import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from alicia_flexible_grasp_supervisor.msg import GraspState
rospy.init_node('load_reviewed_feedforward_timing',anonymous=True)
master=rospy.get_master()
def node(name):
 r=master.lookupNode(name)
 if r[0]!=1:raise RuntimeError('node absent: '+name)
 p=xmlrpc.client.ServerProxy(r[2]).getPid(rospy.get_name())
 if p[0]!=1:raise RuntimeError('node PID unavailable: '+name)
 return dict(uri=r[2],pid=p[2])
def hold():
 state=rospy.wait_for_message('/grasp/state',GraspState,timeout=4)
 if state.active or rospy.get_param('/gui/joint_direct_mode',False):raise RuntimeError('task or manual control active')
 q=rospy.wait_for_message('/alicia_d/sdk_command',JointState,timeout=4)
 ref=rospy.wait_for_message('/alicia_d/control_reference',JointState,timeout=4)
 epoch=rospy.wait_for_message('/alicia_d/control_reference_epoch',Header,timeout=4)
 return dict(sdk=dict(zip(q.name,q.position)),reference=dict(zip(ref.name,ref.position)),epoch_ns=epoch.stamp.to_nsec())
state=dict(before=hold(),gateway_before=node('/motion_gateway'),hardware_commands_sent=0,disable_commands_sent=0,scope='two timing parameters and idle Python gateway reload only')
state['protected']={n:node(n) for n in ('/alicia_d_driver_node','/bessica_d_hw_interface','/move_group')}
state['model_sha256']=hashlib.sha256(rospy.get_param('/robot_description').encode()).hexdigest()
if state['model_sha256']!='7812d6b69e854ed9f37b4e3cbd813ecd53dab3504065b2d04d287abc6fd852e0':raise RuntimeError('paired model changed')
owner=False
for uri in rospy.get_param('/roslaunch/uris').values():
 try:
  r=xmlrpc.client.ServerProxy(uri).list_processes()
  for name,unused in r[2][0]:
   if name.startswith('motion_gateway-'):
    info=xmlrpc.client.ServerProxy(uri).process_info(name)[2]
    if info.get('pid')==state['gateway_before']['pid'] and info.get('respawn'):owner=True
 except Exception:pass
if not owner:raise RuntimeError('gateway respawn owner absent')
sources=[Path('src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot')/n for n in ('pregrasp_gateway.py','feedforward_timing.py')]
state['sources']={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources}
state['old_parameters']={k:rospy.get_param('/grasp/'+k,None) for k in ('unknown_feedforward_timing_enabled','unknown_feedforward_max_joint_velocity_rad_s')}
def save():(out/'result.json').write_text(json.dumps(state,ensure_ascii=False,indent=2)+'\n')
save()
for key,value in (('unknown_feedforward_max_joint_velocity_rad_s',.008),('unknown_feedforward_timing_enabled',True)):
 rospy.set_param('/grasp/'+key,value)
 if rospy.get_param('/grasp/'+key)!=value:raise RuntimeError('parameter write mismatch')
state['parameters_loaded']=True;save()
try:xmlrpc.client.ServerProxy(state['gateway_before']['uri']).shutdown(rospy.get_name(),'load reviewed unknown feedforward timing while task inactive')
except (ConnectionError, socket.timeout) as exc:state['shutdown_reply']=str(exc);save()
deadline=time.monotonic()+30
while time.monotonic()<deadline:
 try:
  new=node('/motion_gateway')
  if new['pid']!=state['gateway_before']['pid']:
   rospy.wait_for_service('/supervisor/compensate_pregrasp',timeout=2)
   state['gateway_after']=new;break
 except Exception:pass
 time.sleep(.2)
else:raise RuntimeError('gateway did not become available')
state['after']=hold()
if state['after']!=state['before']:raise RuntimeError('hold changed during reload')
if any(node(n)!=identity for n,identity in state['protected'].items()):raise RuntimeError('protected process changed')
state['status']='loaded';save()
Path('.runtime/latest_feedforward_speed_load.json').write_text(json.dumps({'directory':str(out)},ensure_ascii=False)+'\n')
print(json.dumps(state,ensure_ascii=False))
