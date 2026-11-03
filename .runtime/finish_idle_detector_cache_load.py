import json,os,socket,subprocess,time,xmlrpc.client
from pathlib import Path
out=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/idle_detector_repair_20260929T142135Z');assert Path('/media/zhuyupei/SanStand').is_mount()
result=json.loads((out/'load_result.json').read_text());(out/'load_attempt1.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
assert result['error']=='old perception did not exit'; assert not Path('/proc/457738').exists()
os.environ['ROS_LOG_DIR']=str(out/'ros')
import rospy,rosgraph
from alicia_flexible_grasp_supervisor.msg import GraspState
from std_msgs.msg import Header
socket.setdefaulttimeout(4);rospy.init_node('finish_idle_detector_cache_load',anonymous=True,disable_signals=True);master=rosgraph.Master(rospy.get_name())
def rpc(n):return xmlrpc.client.ServerProxy(master.lookupNode(n))
def ident(pid):
 p=Path('/proc')/str(pid);s=(p/'stat').read_text().rsplit(')',1)[1].split();return dict(pid=pid,start_ticks=int(s[19]),argv=(p/'cmdline').read_bytes().decode().split('\0')[:-1])
def node(n):return ident(rpc(n).getPid(rospy.get_name())[2])
def ticks(pid):
 s=(Path('/proc')/str(pid)/'stat').read_text().rsplit(')',1)[1].split();return int(s[11])+int(s[12])
try:
 assert not rospy.wait_for_message('/grasp/state',GraspState,timeout=4).active
 assert {n:node(n) for n in result['before']}==result['before']
 assert ident(8526)==result['root_before']
 w=Path.cwd(); unit='alicia-idle-detector-cache-20260929.service'
 subprocess.run(['systemd-run','--user','--unit='+unit,'--property=PartOf=alicia-ros-live-20260928.service','--property=After=alicia-ros-live-20260928.service','/bin/bash',str(w/'.runtime/model_pair_reload/start_consumers.sh'),str(w/'.runtime/model_pair_reload/carton_only.launch'),str(out/'ros')],check=True,timeout=15)
 for i in range(250):
  try:
   new=node('/perception_node')
   if new['pid']!=457738:break
  except Exception:pass
  time.sleep(.1)
 else:raise RuntimeError('new perception not registered')
 result['perception_after']=new;result['events'].append(dict(event='perception_reload_end',wall_time=time.time(),monotonic=time.monotonic()))
 time.sleep(3);t=time.monotonic();c=ticks(new['pid']);time.sleep(5);result['cpu_after_one_core_percent_5sec']=100*(ticks(new['pid'])-c)/os.sysconf('SC_CLK_TCK')/(time.monotonic()-t)
 result['after']={n:node(n) for n in result['before']};result['root_after']=ident(8526)
 result['parameters_after']={k:rospy.get_param(k,None) for k in result['parameters_before']};result['epoch_after']=rospy.wait_for_message('/alicia_d/control_reference_epoch',Header,timeout=4).stamp.to_nsec()
 assert result['before']==result['after'];assert result['root_before']==result['root_after'];assert result['parameters_before']==result['parameters_after'];assert result['epoch_before']==result['epoch_after']
 assert not rospy.wait_for_message('/grasp/state',GraspState,timeout=4).active
 result['bus_info']=rpc('/perception_node').getBusInfo(rospy.get_name())[2]
 result['status']='loaded_idle_no_motion';result['recovered_from']=result.pop('error')
except Exception as e:
 result.update(status='load_failed_no_motion',error=str(e));raise
finally:
 (out/'load_result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n');Path('.runtime/latest_idle_detector_load.json').write_text(json.dumps(dict(directory=str(out),status=result['status']),ensure_ascii=False)+'\n')
 print(json.dumps({k:v for k,v in result.items() if k in ['status','error','cpu_before_one_core_percent_5sec','cpu_after_one_core_percent_5sec','perception_after']},ensure_ascii=False))
