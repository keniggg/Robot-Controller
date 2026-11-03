"""Exercise new planning RPC on a historical scene; never start a grasp."""
from pathlib import Path
import io,json,time
import rospy
from std_msgs.msg import Header,Bool
from control_msgs.msg import FollowJointTrajectoryActionGoal
from alicia_flexible_grasp_supervisor.msg import Grasp6DPlan,GraspState
from alicia_flexible_grasp_supervisor.srv import CheckContactSequence,CheckContactSequenceRequest
root=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_ros_latest_start_20260930T014325Z')
assert Path('/media/zhuyupei/SanStand').is_mount()
rospy.init_node('verify_contact_candidate_precheck_readonly',anonymous=True)
if rospy.wait_for_message('/grasp/state',GraspState,timeout=5).active:raise RuntimeError('task active')
started_ns=rospy.Time.now().to_nsec();actions=[]
sub=rospy.Subscriber('/alicia_controller/follow_joint_trajectory/goal',FollowJointTrajectoryActionGoal,
    lambda m:actions.append(m.header.stamp.to_nsec()) if m.header.stamp.to_nsec()>=started_ns else None)
selection=rospy.get_param('/grasp_mode/selection')
epoch=rospy.wait_for_message('/alicia_d/control_reference_epoch',Header,timeout=5).stamp.to_nsec()
service=rospy.ServiceProxy('/supervisor/check_contact_sequence',CheckContactSequence)
rospy.wait_for_service('/supervisor/check_contact_sequence',timeout=4)
invalid=service(CheckContactSequenceRequest())
assert not invalid.success
plan=Grasp6DPlan();plan.deserialize((root/'contact_collision_replay/plan.roswire').read_bytes())
before=io.BytesIO();plan.serialize(before)
start=time.monotonic();result=service(CheckContactSequenceRequest(plan=plan,deadline=rospy.Time.from_sec(rospy.get_time()+12.)))
first_elapsed=time.monotonic()-start
repeat_start=time.monotonic()
repeat=service(CheckContactSequenceRequest(plan=plan,deadline=rospy.Time.from_sec(rospy.get_time()+12.)))
repeat_elapsed=time.monotonic()-repeat_start
repeat_evidence=json.loads(repeat.evidence_json)
end=io.BytesIO();plan.serialize(end)
assert before.getvalue()==end.getvalue()
evidence=json.loads(result.evidence_json)
assert evidence['plan_id']==plan.plan_id and evidence['execution_authorized'] is False
state=rospy.wait_for_message('/grasp/state',GraspState,timeout=5)
after=rospy.wait_for_message('/alicia_d/control_reference_epoch',Header,timeout=5).stamp.to_nsec()
report=dict(scope='live_readonly_rpc_historical_scene_current_feedback_not_an_executable_plan',
 started_ns=started_ns,elapsed_sec=first_elapsed,repeat_elapsed_sec=repeat_elapsed,
 repeat_success=repeat.success,repeat_cache_hit=repeat_evidence.get('prediction_cache_hit',False),
 repeat_evidence=repeat_evidence,source_plan_id=plan.plan_id,
 invalid_request_rejected=True,success=result.success,failure_code=result.failure_code,
 failed_stage=result.failed_stage,message=result.message,evidence=evidence,
 selection_before=selection,selection_after=rospy.get_param('/grasp_mode/selection'),
 epoch_before_ns=epoch,epoch_after_ns=after,task_active=state.active,action_goals=actions,
 motion_enabled=rospy.wait_for_message('/alicia_d/motion_enabled',Bool,timeout=5).data,
 actual_current_target_validated=False,new_physical_grasp_verified=False)
assert not state.active and not actions and epoch==after
assert report['selection_after']==selection
(root/'contact_precheck_live_readonly.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
print(json.dumps({k:v for k,v in report.items() if k not in ('evidence','repeat_evidence')},ensure_ascii=False))
