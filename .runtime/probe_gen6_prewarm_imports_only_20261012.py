"""Exercise actual runner imports and unreleased EOF cleanup; never call run_once."""
import hashlib,importlib.util,json,os,time
from pathlib import Path
w=Path(__file__).resolve().parents[1]
s=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_frozen_no_skin_20261012T021050Z')
out=s/'verification/gen6_prewarm_imports_only_r2';assert Path('/media/zhuyupei/SanStand').is_mount();out.mkdir(exist_ok=False)
os.environ['ROS_LOG_DIR']=str(out/'ros')
import rospy
from rosgraph import Master
from sensor_msgs.msg import JointState
from std_msgs.msg import Header,String
from actionlib_msgs.msg import GoalStatusArray
from alicia_flexible_grasp_supervisor.msg import GraspState
rospy.init_node('prewarm_imports_only_evidence',anonymous=True,disable_rosout=True)
master=Master(rospy.get_name())
def snapshot():
    selection=rospy.get_param('/grasp_mode/selection')
    assert selection['generation']==6
    manual=rospy.get_param('/gui/joint_direct_mode');assert type(manual) is bool
    assert not rospy.wait_for_message('/grasp/state',GraspState,timeout=4).active
    mode=json.loads(rospy.wait_for_message('/grasp_mode/status',String,timeout=4).data)
    assert not any(mode.get(k) for k in ('active','start_reserved','pending'))
    for t in ('/alicia_controller/follow_joint_trajectory/status','/hand_controller/follow_joint_trajectory/status'):
        assert not any(x.status in (0,1,6,7) for x in rospy.wait_for_message(t,GoalStatusArray,timeout=4).status_list)
    q=rospy.wait_for_message('/alicia_d/accepted_joint_states',JointState,timeout=4)
    assert q.header.frame_id=='sdk_measured' and 0<=(rospy.Time.now()-q.header.stamp).to_sec()<.2
    epoch=rospy.wait_for_message('/alicia_d/control_reference_epoch',Header,timeout=4).stamp.to_nsec()
    systems=master.getSystemState();nodes=sorted(set(n for group in systems for _,ns in group for n in ns))
    return dict(selection=selection,epoch_ns=epoch,joints=dict(zip(q.name,q.position)),nodes=nodes,manual=manual)
helper=w/'.runtime/prewarm_mode_grasp_runner_20261012.py'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
expected='3e5a0798bccaf49ea15cb8eac977d2621cfe48a78e8ae141bf324f4154e2fa49';assert sha(helper)==expected
spec=importlib.util.spec_from_file_location('actual_prewarm_probe',str(helper));module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
before=snapshot();start=time.monotonic()
with module.PrewarmedModeRunner(w/'src/alicia_grasp_modes/scripts/mode_aware_grasp_runner.py') as child:
    duration=time.monotonic()-start
    ready=dict(child.ready);assert not child.released
    during=snapshot()
assert child.process.poll()==75 and not child.released
after=snapshot();assert before==during==after
assert sha(helper)==expected
result=dict(passed=True,imports_ready_elapsed_sec=duration,ready=ready,child_exitcode=child.process.returncode,selection_released=False,original_runner_run_called=False,no_ros_graph_change=True,before=before,after=after,helper_sha256=expected,motion_commands_sent=False,temperature_queried=False,limitation='Import and no-release cleanup verified; actual post-selection latency and grasp still require a newly authorized run')
(out/'result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
print(json.dumps(dict(passed=True,imports_ready_elapsed_sec=duration,child_exitcode=child.process.returncode,selection_released=False,result=str(out/'result.json'))),flush=True)
