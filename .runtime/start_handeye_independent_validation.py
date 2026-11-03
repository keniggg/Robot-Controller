from pathlib import Path
from datetime import datetime,timezone
import json,os,subprocess,rospy,hashlib
ROOT=Path.cwd();c=json.loads((ROOT/'.runtime/current_handeye_autocalibration.json').read_text());training=Path(c['session']);f=json.loads((ROOT/'.runtime/current_handeye_auto_fit.json').read_text());fitdir=Path(f['directory']);result=json.loads((fitdir/'results.json').read_text());assert result['selected_algorithm'], 'no training candidate passed'
assert os.path.ismount('/media/zhuyupei/SanStand')
folder=training/('independent_validation_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ'));folder.mkdir();(folder/'ros').mkdir();(folder/'starting_geometry.json').write_bytes((training/'starting_geometry.json').read_bytes())
rospy.init_node('start_handeye_validation_session',anonymous=True,disable_signals=True)
namespace=c['namespace']+'_validation';common=dict(move_group_namespace='/',move_group='alicia',eye_on_hand=True,robot_base_frame='base_link',robot_effector_frame='tool0',tracking_base_frame='camera_link',tracking_marker_frame=c['board_frame'],freehand_robot_movement=True)
for k,v in common.items():rospy.set_param(namespace+'/'+k,v)
args=[str(ROOT/'tools/with_local_realsense_sdk.sh'),'/usr/bin/python3','-u',str(ROOT/'tools/handeye_exact_sample_server.py'),'__ns:='+namespace,'__name:=exact_sample_server','_session_dir:='+str(folder),'_pose_change_source:=assistant_automatic','_sample_role:=validation']
env=os.environ.copy();env['ROS_LOG_DIR']=str(folder/'ros');env['OPENBLAS_NUM_THREADS']='1';env['OMP_NUM_THREADS']='1'
with (folder/'sampler_stdout.log').open('ab') as log:p=subprocess.Popen(args,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
config={'session':str(folder),'namespace':namespace,'board_frame':c['board_frame'],'training_session':str(training),'frozen_candidate_directory':str(fitdir),'frozen_results_sha256':hashlib.sha256((fitdir/'results.json').read_bytes()).hexdigest(),'sampler':{'pid':p.pid,'args':args},'validation_plan':str(training/'prospective_independent_validation_plan.json')}
(folder/'validation_setup.json').write_text(json.dumps(config,indent=2)+'\n');(ROOT/'.runtime/current_handeye_independent_validation.json').write_text(json.dumps(config,indent=2)+'\n');print(json.dumps(config,indent=2))
