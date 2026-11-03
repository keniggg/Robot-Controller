import json,pathlib,sys,runpy,numpy as np,rospy,time
from std_msgs.msg import String,Bool
from easy_handeye_msgs.srv import TakeSample
sys.path.insert(0,str(pathlib.Path('.runtime').resolve()))
from handeye_auto_motion import AutoMotion
m=runpy.run_path('.runtime/prepare_task_pose_validation.py');c=m['config'];out=m['out'];geometry=m['geometry']
rospy.init_node('task_pose_board_retreat',anonymous=True,disable_signals=True)
# User explicitly delegated posture control. Select the GUI direct protocol;
# this ownership flag is not a joint torque/disable command.
rospy.set_param('/gui/joint_direct_mode',True)
ownership=rospy.Publisher('/gui/joint_direct_mode',Bool,queue_size=1,latch=True)
time.sleep(.4);ownership.publish(Bool(data=True));time.sleep(.4)
motion=AutoMotion(out);q=np.array(motion.q[:6]);assert max(abs(q-m['q']))<.02,'Operator pose changed since baseline'
quality=json.loads(rospy.wait_for_message('/charuco/quality',String,timeout=3).data);assert quality['pose_ok'] and quality['charuco_count']>=35
_,initial=geometry(q);baseline=initial['minimum_gripper_plane_clearance_m']
def guard(target):
    previous=baseline
    for a in np.linspace(0,1,25):
        _,data=geometry(q*(1-a)+np.array(target[:6])*a)
        clearance=data.get('minimum_gripper_plane_clearance_m',-1.)
        if clearance+1e-10<previous or data.get('predicted_full_board_edge_px',-1)<20:
            raise RuntimeError('Retreat must increase clearance and retain full board')
        previous=clearance
    if previous<.04:raise RuntimeError('Retreat does not reach reserved observation clearance')
motion.command_guard=guard
(out/'bootstrap_retreat.json').write_text(json.dumps(dict(joint=2,delta_deg=4.,initial_clearance_m=baseline,policy='single bounded retreat; monotonic increasing board clearance; no contact target',positive_enable_only=True),indent=2)+'\n')
motion.move(1,4.)
time.sleep(.5)
r=rospy.ServiceProxy(c['namespace']+'/take_sample',TakeSample)();motion.emit('bootstrap_sample_accepted',count=len(r.samples.hand_world_samples))
