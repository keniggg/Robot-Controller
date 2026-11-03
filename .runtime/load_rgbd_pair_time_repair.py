import hashlib,json,os,socket,subprocess,time,xmlrpc.client
from pathlib import Path
mount=Path('/media/zhuyupei/SanStand');assert mount.is_mount()
out=mount/'实验记录/Robot-Controller/ros_log/rgbd_pair_time_repair_20260929T142631Z'
manifest=json.loads((out/'source_manifest.json').read_text())
for e in manifest:assert hashlib.sha256(Path(e['path']).read_bytes()).hexdigest()==e['sha256_after'],e['path']
os.environ['ROS_LOG_DIR']=str(out/'ros');(out/'ros').mkdir(exist_ok=True)
import rospy,rosgraph
from alicia_flexible_grasp_supervisor.msg import GraspState
from std_msgs.msg import Header
from sensor_msgs.msg import Image
socket.setdefaulttimeout(4);rospy.init_node('load_rgbd_pair_time_repair',anonymous=True,disable_signals=True);master=rosgraph.Master(rospy.get_name())
def rpc(n):return xmlrpc.client.ServerProxy(master.lookupNode(n))
def ident(pid):
 p=Path('/proc')/str(pid);s=(p/'stat').read_text().rsplit(')',1)[1].split();return dict(pid=pid,start_ticks=int(s[19]),argv=(p/'cmdline').read_bytes().decode().split('\0')[:-1])
def node(n):return ident(rpc(n).getPid(rospy.get_name())[2])
protected=['/alicia_d_driver_node','/bessica_d_hw_interface','/motion_gateway','/accepted_robot_state_publisher','/perception_node','/unknown_tabletop_perception','/move_group','/grasp_task_node','/remote_grasp6d_node','/grasp_mode_router']
result=dict(status='preparing',source_manifest=manifest,hardware_motion_commands_sent=0,joint_disable_commands_sent=0,events=[])
try:
 assert not rospy.wait_for_message('/grasp/state',GraspState,timeout=4).active
 result['before']={n:node(n) for n in protected};result['root_before']=ident(8526);result['camera_before']=node('/supervisor_camera');assert result['camera_before']['pid']==606029
 keys=['/gui/joint_direct_mode','/grasp_mode/selection','/camera']
 result['parameters_before']={k:rospy.get_param(k,None) for k in keys};result['epoch_before']=rospy.wait_for_message('/alicia_d/control_reference_epoch',Header,timeout=4).stamp.to_nsec()
 (out/'load_before.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
 result['events'].append(dict(event='intentional_camera_reload_begin',wall_time=time.time(),monotonic=time.monotonic()))
 subprocess.run(['systemctl','--user','restart','alicia-camera-timing-fix-20260929.service'],check=True,timeout=40)
 deadline=time.monotonic()+30
 while time.monotonic()<deadline:
  try:
   new=node('/supervisor_camera')
   if new['pid']!=606029:break
  except Exception:pass
  time.sleep(.2)
 else:raise RuntimeError('new camera node not registered')
 result['camera_after']=new
 msg=rospy.wait_for_message('/supervisor/camera/color/image_raw',Image,timeout=30)
 assert time.time()-msg.header.stamp.to_sec()<1.0
 result['first_new_frame']=dict(stamp_ns=msg.header.stamp.to_nsec(),width=msg.width,height=msg.height,encoding=msg.encoding)
 depth=rospy.wait_for_message('/supervisor/camera/depth/image_raw',Image,timeout=4)
 result['first_new_depth']=dict(stamp_ns=depth.header.stamp.to_nsec(),width=depth.width,height=depth.height,encoding=depth.encoding)
 result['events'].append(dict(event='intentional_camera_reload_end',wall_time=time.time(),monotonic=time.monotonic()))
 result['after']={n:node(n) for n in protected};result['root_after']=ident(8526);result['parameters_after']={k:rospy.get_param(k,None) for k in keys};result['epoch_after']=rospy.wait_for_message('/alicia_d/control_reference_epoch',Header,timeout=4).stamp.to_nsec()
 assert result['before']==result['after'];assert result['root_before']==result['root_after'];assert result['epoch_before']==result['epoch_after']
 for k in keys[:2]:assert result['parameters_before'][k]==result['parameters_after'][k],k
 camera_a=result['parameters_before']['/camera'];camera_b=result['parameters_after']['/camera']
 result['camera_parameter_changed_keys']=[k for k in set(camera_a)|set(camera_b) if camera_a.get(k)!=camera_b.get(k)]
 for k in ['width','height','fps','publish_fps','fx','fy','cx','cy','depth_scale','color_projection','projection_frame_convention']:
  assert camera_a.get(k)==camera_b.get(k),k
 assert not rospy.wait_for_message('/grasp/state',GraspState,timeout=4).active
 result['status']='loaded_with_live_rgbd_no_motion'
except Exception as e:
 result.update(status='load_failed_no_motion',error=str(e));raise
finally:
 (out/'load_result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n');Path('.runtime/latest_rgbd_pair_repair_load.json').write_text(json.dumps(dict(directory=str(out),status=result['status']),ensure_ascii=False)+'\n')
 print(json.dumps({k:v for k,v in result.items() if k in ['status','error','camera_after','camera_parameter_changed_keys','events']},ensure_ascii=False))
