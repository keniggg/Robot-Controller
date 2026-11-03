import json, os, socket, subprocess, time, xmlrpc.client
from pathlib import Path
import xml.etree.ElementTree as ET
workspace=Path.cwd(); mount=Path('/media/zhuyupei/SanStand'); assert mount.is_mount()
out=mount/'实验记录/Robot-Controller/ros_log/idle_detector_repair_20260929T142135Z'
os.environ['ROS_LOG_DIR']=str(out/'ros'); (out/'ros').mkdir(exist_ok=True)
source=ET.parse('.runtime/model_pair_reload/perception_consumers.launch').getroot()
root=ET.Element('launch'); root.append(next(n for n in source.findall('node') if n.get('name')=='perception_node'))
launch=workspace/'.runtime/model_pair_reload/carton_only.launch'; ET.ElementTree(root).write(launch,encoding='unicode')
import rospy, rosgraph
from alicia_flexible_grasp_supervisor.msg import GraspState
from std_msgs.msg import Header
socket.setdefaulttimeout(4)
rospy.init_node('load_idle_detector_cache',anonymous=True,disable_signals=True)
master=rosgraph.Master(rospy.get_name())
def rpc(name):return xmlrpc.client.ServerProxy(master.lookupNode(name))
def ident(pid):
 p=Path('/proc')/str(pid); st=(p/'stat').read_text().rsplit(')',1)[1].split(); return dict(pid=pid,start_ticks=int(st[19]),argv=(p/'cmdline').read_bytes().decode().split('\0')[:-1])
def node(n):return ident(rpc(n).getPid(rospy.get_name())[2])
def ticks(pid):
 s=(Path('/proc')/str(pid)/'stat').read_text().rsplit(')',1)[1].split(); return int(s[11])+int(s[12])
def cpu(pid):
 t=time.monotonic(); a=ticks(pid); time.sleep(5); return 100*(ticks(pid)-a)/os.sysconf('SC_CLK_TCK')/(time.monotonic()-t)
protected=['/alicia_d_driver_node','/bessica_d_hw_interface','/motion_gateway','/accepted_robot_state_publisher','/supervisor_camera','/unknown_tabletop_perception','/move_group','/grasp_task_node','/remote_grasp6d_node','/grasp_mode_router']
result=dict(status='preparing',hardware_commands_sent=0,joint_disable_commands_sent=0,events=[])
try:
 assert not rospy.wait_for_message('/grasp/state',GraspState,timeout=4).active
 result['before']={n:node(n) for n in protected};result['root_before']=ident(8526)
 result['perception_before']=node('/perception_node'); assert result['perception_before']['pid']==457738
 result['parameters_before']={k:rospy.get_param(k,None) for k in ['/gui/joint_direct_mode','/grasp_mode/selection','/perception']}
 result['epoch_before']=rospy.wait_for_message('/alicia_d/control_reference_epoch',Header,timeout=4).stamp.to_nsec()
 result['cpu_before_one_core_percent_5sec']=cpu(457738)
 (out/'load_before.json').write_text(json.dumps(result,ensure_ascii=False,indent=2))
 result['events'].append(dict(event='perception_reload_begin',wall_time=time.time(),monotonic=time.monotonic()))
 ack=rpc('/perception_node').shutdown(rospy.get_name(),'load validated detector configuration cache only'); assert ack[0]==1, ack
 for i in range(100):
  if not Path('/proc/457738').exists(): break
  time.sleep(.1)
 else: raise RuntimeError('old perception did not exit')
 unit='alicia-idle-detector-cache-20260929.service'
 subprocess.run(['systemd-run','--user','--unit='+unit,'--property=PartOf=alicia-ros-live-20260928.service','--property=After=alicia-ros-live-20260928.service','/bin/bash',str(workspace/'.runtime/model_pair_reload/start_consumers.sh'),str(launch),str(out/'ros')],check=True,timeout=15)
 for i in range(200):
  try:
   new=node('/perception_node')
   if new['pid']!=457738:break
  except Exception:pass
  time.sleep(.1)
 else:raise RuntimeError('new perception not registered')
 result['perception_after']=new
 result['events'].append(dict(event='perception_reload_end',wall_time=time.time(),monotonic=time.monotonic()))
 time.sleep(3)
 result['cpu_after_one_core_percent_5sec']=cpu(new['pid'])
 result['after']={n:node(n) for n in protected};result['root_after']=ident(8526)
 result['parameters_after']={k:rospy.get_param(k,None) for k in result['parameters_before']}
 result['epoch_after']=rospy.wait_for_message('/alicia_d/control_reference_epoch',Header,timeout=4).stamp.to_nsec()
 assert result['before']==result['after']; assert result['root_before']==result['root_after']
 assert result['parameters_before']==result['parameters_after']; assert result['epoch_before']==result['epoch_after']
 assert not rospy.wait_for_message('/grasp/state',GraspState,timeout=4).active
 result['bus_info']=rpc('/perception_node').getBusInfo(rospy.get_name())[2]
 result['status']='loaded_idle_no_motion'
except Exception as exc:
 result.update(status='load_failed_no_motion',error=str(exc));raise
finally:
 (out/'load_result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
 Path('.runtime/latest_idle_detector_load.json').write_text(json.dumps(dict(directory=str(out),status=result['status']),ensure_ascii=False)+'\n')
 print(json.dumps({k:v for k,v in result.items() if k in ['status','error','cpu_before_one_core_percent_5sec','cpu_after_one_core_percent_5sec','events','perception_after']},ensure_ascii=False))
