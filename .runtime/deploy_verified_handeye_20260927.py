import os,json,hashlib,sys,time,subprocess,xmlrpc.client
from pathlib import Path
from datetime import datetime,timezone
import yaml,numpy as np,rospy,rosgraph,rosnode,tf2_ros
from alicia_flexible_grasp_supervisor.msg import GraspState
ROOT=Path.cwd(); final=Path(json.loads((ROOT/'.runtime/current_handeye_auto_final.json').read_text())['directory']); validated=json.loads((final/'validated_handeye.json').read_text()); result=json.loads((final/'result.json').read_text())
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
assert os.path.ismount('/media/zhuyupei/SanStand') and validated['validated'] and result['verdict']=='PASS'
assert sha(final/'validated_handeye.yaml')==validated['yaml_sha256'] and sha(final/'result.json')==validated['result_sha256']
rospy.init_node('deploy_verified_handeye',anonymous=True,disable_signals=True)
assert not rospy.wait_for_message('/grasp/state',GraspState,timeout=5).active
stamp=datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ'); out=final.parent/('deployment_'+stamp);out.mkdir()
config=ROOT/'src/alicia_flexible_grasp_supervisor/config/handeye.yaml';grasp=ROOT/'src/alicia_flexible_grasp_supervisor/config/grasp_params.yaml';oldcfg=config.read_text();oldgrasp=grasp.read_text();cfg=yaml.safe_load(oldcfg)['handeye'];previous=rospy.get_param('/handeye');oldcal=Path(os.path.expanduser(cfg['calibration_file']))
(out/'previous_handeye.yaml').write_text(oldcfg);(out/'previous_grasp_params.yaml').write_text(oldgrasp);(out/'previous_calibration.yaml').write_bytes(oldcal.read_bytes());(out/'previous_runtime_handeye.json').write_text(json.dumps(previous,indent=2)+'\n')
dest=Path.home()/'.ros/easy_handeye/d405_cc200_auto_20260927_validated_eye_on_hand.yaml'
assert not dest.exists(), 'Deployment destination already exists; inspect before replacing'
dest.write_bytes((final/'validated_handeye.yaml').read_bytes())
xyz=validated['translation_xyz_m'];quat=validated['quaternion_xyzw'];cfg['calibration_file']=str(dest);cfg['translation_xyz']=xyz;cfg['rotation_xyzw']=quat
newcfg='''# Current-model hand-eye calibration: 2026-09-27 Tsai-Lenz.
# 22 training groups, 7 independent validation groups, 1 return pose.
# CV max 2.677 mm / 0.648 deg; independent max 2.244 mm / 0.591 deg.
# Evidence: '''+str(final/'result.json')+'\n'+yaml.safe_dump({'handeye':cfg},sort_keys=False,allow_unicode=True)
config.write_text(newcfg)
limit=max(result['training_cross_validation']['translation_max_m'],result['all_validation_summary']['translation_max_m'])
oldcomment='''  # 2026-08-10 CC200 pruned-27 OpenCV/Park hand-eye result passes the formal
  # five-fold contract: 2.812/4.606 mm translation and 0.863/1.351 deg
  # rotation. The centering-margin gate remains active and still compares the
  # object-specific jaw margin against this measured maximum translation error.
'''
assert oldcomment in oldgrasp
newgrasp=oldgrasp.replace(oldcomment,'''  # 2026-09-27 current-intrinsics Tsai-Lenz: 22 groups, five-fold CV,
  # then 7 independent groups and 1 return pose with the candidate frozen.
  # Use the larger observed translation maximum from CV and held-out data.
  # This local empirical stress scale is not a certified workspace error bound.
''').replace('  handeye_translation_max_error_m: 0.00460616416406','  handeye_translation_max_error_m: '+repr(limit)).replace('calibration_interlock_reason: "HAND_EYE_CROSS_VALIDATION_PASSED"','calibration_interlock_reason: "HAND_EYE_CURRENT_INTRINSICS_CV_AND_INDEPENDENT_VALIDATION_PASSED"')
assert newgrasp!=oldgrasp;grasp.write_text(newgrasp)
rospy.set_param('/handeye',cfg);rospy.set_param('/grasp/handeye_translation_max_error_m',limit);rospy.set_param('/grasp/calibration_interlock_reason','HAND_EYE_CURRENT_INTRINSICS_CV_AND_INDEPENDENT_VALIDATION_PASSED')
master=rosgraph.Master(rospy.get_name());uri=master.lookupNode('/handeye_transform');node=xmlrpc.client.ServerProxy(uri);code,msg,pid=node.getPid(rospy.get_name());assert code==1
cmd=Path('/proc/%d/cmdline'%pid).read_bytes();assert b'handeye_transform_node.py' in cmd
shutdown=node.shutdown(rospy.get_name(),'Load user-authorized verified hand-eye parameters only');print('TF_PUBLISHER_RELOAD',pid,shutdown,flush=True)
for _ in range(60):
 if '/handeye_transform' not in rosnode.get_node_names():break
 time.sleep(.1)
else:raise RuntimeError('Previous TF publisher did not unregister')
args=['/usr/bin/python3','-u',str(ROOT/'src/alicia_flexible_grasp_supervisor/scripts/handeye_transform_node.py'),'__name:=handeye_transform']
with (out/'handeye_publisher.log').open('w') as logfile:
 p=subprocess.Popen(args,stdout=logfile,stderr=subprocess.STDOUT,start_new_session=True)
buf=tf2_ros.Buffer();listener=tf2_ros.TransformListener(buf);expected=np.array(xyz+quat);observed=None
for _ in range(100):
 try:
  t=buf.lookup_transform('tool0','camera_link',rospy.Time(0));v=t.transform;observed=[v.translation.x,v.translation.y,v.translation.z,v.rotation.x,v.rotation.y,v.rotation.z,v.rotation.w]
  if np.allclose(observed,expected,rtol=0,atol=1e-12):break
 except Exception:pass
 time.sleep(.1)
else:raise RuntimeError('Live TF did not match deployed candidate: '+str(observed))
assert p.poll() is None and rospy.get_param('/handeye')==cfg and sha(dest)==validated['yaml_sha256']
r={'created_at':datetime.now(timezone.utc).isoformat(),'source_validation':str(final),'validated_yaml_sha256':sha(dest),'calibration_file':str(dest),'new_runtime_handeye':cfg,'empirical_translation_error_m':limit,'empirical_error_source':'max(training five-fold CV maximum, frozen independent+return maximum)','publisher_pid':p.pid,'publisher_command':args,'runtime_tf_xyz_xyzw':observed,'previous_tf_publisher_pid':pid,'deployed':True,'driver_restarted':False,'torque_disable_commands_sent':False,'configuration_sha256':sha(config),'grasp_configuration_sha256':sha(grasp)}
(out/'deployment.json').write_text(json.dumps(r,ensure_ascii=False,indent=2)+'\n');(ROOT/'.runtime/current_handeye_deployment.json').write_text(json.dumps({'directory':str(out),'deployed':True},ensure_ascii=False,indent=2)+'\n')
print('DEPLOYMENT_VERIFIED',json.dumps(r,ensure_ascii=False),flush=True)
