from pathlib import Path
import json,os,time,subprocess,xmlrpc.client,rospy,sys
from std_msgs.msg import Header
from easy_handeye_msgs.srv import TakeSample
root=Path.cwd();cp=root/'.runtime/current_handeye_task_pose_validation.json';c=json.loads(cp.read_text());out=Path(c['session'])
rospy.init_node('continue_task_validation_sampling',anonymous=True,disable_signals=True)
epoch=rospy.wait_for_message('/alicia_d/control_reference_epoch',Header,timeout=3).stamp.to_nsec();segment=out/('epoch_'+str(epoch));segment.mkdir(exist_ok=False);(segment/'ros').mkdir()
oldns=c['namespace'];name=oldns+'/exact_sample_server';code,_,uri=rospy.get_master().lookupNode(name)
if code==1:xmlrpc.client.ServerProxy(uri).shutdown(rospy.get_name(),'Start separate frozen-parameter validation epoch; sampler has no arm commands')
ns='/d405_cc200_task_verify_epoch_'+str(epoch)+'_eye_on_hand'
for k,v in dict(move_group_namespace='/',move_group='alicia',eye_on_hand=True,robot_base_frame='base_link',robot_effector_frame='tool0',tracking_base_frame='camera_link',tracking_marker_frame=c['board_frame'],freehand_robot_movement=True).items():rospy.set_param(ns+'/'+k,v)
env=os.environ.copy();env.update(ROS_LOG_DIR=str(segment/'ros'),OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',PYTHONUNBUFFERED='1')
args=[str(root/'tools/with_local_realsense_sdk.sh'),'/usr/bin/python3','-u',str(root/'tools/handeye_exact_sample_server.py'),'__ns:='+ns,'__name:=exact_sample_server','_session_dir:='+str(segment),'_gui_alias:=/d405_cc200_recalib_20260809_eye_on_hand','_pose_change_source:=assistant_automatic','_sample_role:=validation']
with (segment/'sampler_stdout.log').open('ab') as f:p=subprocess.Popen(args,env=env,stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
segments=c.setdefault('sampling_segments',[dict(session=str(out),namespace=oldns,reason='initial fixed reference')]);segments.append(dict(session=str(segment),namespace=ns,epoch_ns=epoch,reason='positive-enable recovery changed control-reference epoch; no cross-epoch calibration fit'))
c['namespace']=ns;c['active_sample_session']=str(segment);c['processes']['sampler']=dict(pid=p.pid,args=args);cp.write_text(json.dumps(c,ensure_ascii=False,indent=2)+'\n');(out/'sampling_segments.json').write_text(json.dumps(segments,indent=2)+'\n');(segment/'automation_setup.json').write_text(json.dumps(c,ensure_ascii=False,indent=2)+'\n')
rospy.wait_for_service(ns+'/take_sample',timeout=15);time.sleep(1);r=rospy.ServiceProxy(ns+'/take_sample',TakeSample)();row=dict(event='validation_sample_accepted',time=time.time(),pose_id=int(sys.argv[1]) if len(sys.argv)>1 else 2,role='epoch_reference_before_future_samples',count=len(r.samples.hand_world_samples),sample_session=str(segment))
with (out/'automatic_motion.jsonl').open('a') as f:f.write(json.dumps(row)+'\n')
print(json.dumps(row),flush=True)
