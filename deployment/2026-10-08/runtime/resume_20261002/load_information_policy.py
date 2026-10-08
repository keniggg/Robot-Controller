"""Load tested software nodes only; never publishes a robot motion/enable command."""
import argparse,datetime,hashlib,json,pathlib,subprocess,time,xmlrpc.client
import rospy,rosgraph
from std_msgs.msg import Bool
from alicia_flexible_grasp_supervisor.msg import GraspState
ROOT=pathlib.Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
OUT=pathlib.Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_ros_latest_start_20261002')
SOURCES=['src/alicia_grasp_modes/src/alicia_grasp_modes/observation_policy.py',
'src/alicia_grasp_modes/src/alicia_grasp_modes/runner_contract.py',
'src/alicia_grasp_modes/src/alicia_grasp_modes/active_observation_policy.py',
'src/alicia_grasp_modes/src/alicia_grasp_modes/observe_return_session.py',
'src/alicia_grasp_modes/src/alicia_grasp_modes/observe_return_task.py',
'src/alicia_grasp_modes/src/alicia_grasp_modes/observe_return_remote.py',
'src/alicia_grasp_modes/scripts/mode_aware_grasp_task.py',
'src/alicia_grasp_modes/scripts/mode_aware_remote_grasp6d.py',
'src/alicia_flexible_grasp_supervisor/scripts/grasp_task_node.py',
'src/alicia_flexible_grasp_supervisor/scripts/remote_grasp6d_node.py',
'src/alicia_flexible_grasp_supervisor/scripts/motion_gateway_node.py',
'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/grasp/observation_working_volume.py',
'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot/observe_return_gateway.py',
'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot/observation_path_guard.py',
'src/alicia_flexible_grasp_supervisor/config/grasp_params.yaml']
NODES=['/alicia_d_driver_node','/supervisor_camera','/motion_gateway','/remote_grasp6d_node','/grasp_task_node']

def master():return rosgraph.Master('/information_policy_reload')
def node(name):
 uri=master().lookupNode(name);pid=xmlrpc.client.ServerProxy(uri).getPid('/information_policy_reload')[2]
 return dict(pid=pid,uri=uri,command=(pathlib.Path('/proc')/str(pid)/'cmdline').read_bytes().decode().replace('\0',' '))
def hashes():return {p:hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in SOURCES}
def run(*argv):subprocess.run(list(argv),check=True,timeout=15.)
def shutdown(name):
 info=node(name)
 xmlrpc.client.ServerProxy(info['uri']).shutdown('/information_policy_reload','load tested observation policy software')
 print('software shutdown requested '+name,flush=True)
 return info

def main():
 parser=argparse.ArgumentParser();parser.add_argument('--apply',action='store_true');args=parser.parse_args()
 if not pathlib.Path('/media/zhuyupei/SanStand').is_mount():raise RuntimeError('SanStand unavailable')
 rospy.init_node('information_policy_reload',anonymous=True,disable_signals=True)
 before={n:node(n) for n in NODES}
 state=rospy.wait_for_message('/grasp/state',GraspState,timeout=4.)
 if state.active:raise RuntimeError('active grasp cannot be hot reloaded')
 report=dict(recorded_at_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
  source_sha256=hashes(),before=before,apply=args.apply,hardware_commands_sent=False,
  joint_disable_commands_sent=False,real_motion_verified=False)
 if not args.apply:
  print(json.dumps(report,ensure_ascii=False,indent=2));return
 (OUT/'information_policy_deployment_started.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
 shutdown('/remote_grasp6d_node')
 run('systemctl','--user','restart','alicia-observation-margin-task-20261002.service')
 shutdown('/motion_gateway') # Parent roslaunch has respawn=true for this software node.
 run('systemd-run','--user','--unit=alicia-information-observation-remote-20261002',
     '--property=PartOf=alicia-ros-live-20260928.service','--property=Restart=no','--collect',
     '/bin/bash',str(ROOT/'.runtime/start_information_observation_remote_20261002.sh'))
 deadline=time.monotonic()+25.
 after={}
 while time.monotonic()<deadline:
  try:
   after={n:node(n) for n in NODES}
   if all(after[n]['pid']!=before[n]['pid'] for n in NODES[2:]):break
  except Exception:pass
  time.sleep(.3)
 else:raise RuntimeError('new software node PIDs unavailable')
 rospy.wait_for_service('/supervisor/admit_observe_return_view',timeout=6.)
 for n in NODES[:2]:
  if after[n]['pid']!=before[n]['pid']:raise RuntimeError('preserved hardware owner changed: '+n)
 if hashes()!=report['source_sha256']:raise RuntimeError('source changed during load')
 enabled=rospy.wait_for_message('/alicia_d/motion_enabled',Bool,timeout=4.).data
 feedback=rospy.wait_for_message('/alicia_d/feedback_ready',Bool,timeout=4.).data
 report.update(after=after,source_unchanged_during_load=True,driver_and_camera_preserved=True,
     motion_enabled=enabled,feedback_ready=feedback,admission_service_registered=True,
     complete=True)
 (OUT/'information_policy_deployment.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
 print(json.dumps({k:v for k,v in report.items() if k!='source_sha256'},ensure_ascii=False,indent=2))
if __name__=='__main__':main()
