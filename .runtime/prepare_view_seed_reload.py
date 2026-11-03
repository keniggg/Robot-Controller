"""Read-only live binding plus frozen deployment package; no robot commands."""
from pathlib import Path
import hashlib,json,os,types,sys,time
W=Path(__file__).resolve().parents[1]
RUN=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z/gen58_resume_r1/observation_pose_fix/held_lattice_fix_r1/seed_wait_fix_r1/contact_feedback_fix_r1/actual_gen63_baseline_validation_20261023T041149Z')
R=RUN/'high_accuracy_grasp_r1/arrival_margin_repair_r1'
V=R/'view_search_repair_r1'
assert Path('/media/zhuyupei/SanStand').is_mount()
out=V/'preparation';out.mkdir()
os.environ['ROS_LOG_DIR']=str(out/'ros')
os.environ['ROS_PACKAGE_PATH']=str(W/'src')+':/opt/ros/noetic/share'
sys.path[:0]=[str(W/'src/alicia_grasp_modes/src'),str(W/'src/alicia_flexible_grasp_supervisor/src'),str(W/'devel/lib/python3/dist-packages'),'/opt/ros/noetic/lib/python3/dist-packages']
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def save(p,x):p.write_text(json.dumps(x,ensure_ascii=False,indent=2)+'\n')
def ref(p):return dict(path=str(p),sha256=sha(p))
def module(p,name):
    m=types.ModuleType(name);m.__file__=str(p);exec(compile(p.read_bytes(),str(p),'exec'),m.__dict__);return m
reload=module(W/'.runtime/reload_idle_view_remote.py','reviewed_view_reload')
hp=W/'.runtime/verify_active_remote_readiness_20261020.py'
assert sha(hp)==reload.PRIMITIVES[str(hp.relative_to(W))]
health=module(hp,'health_primitive')
binding_path=RUN/'repair_deployment_r1/remote_startup_binding.json'
binding=health.load_binding(binding_path,'19e6f75f9c4e29abca1a4872f2ee176433f492b42c9736e81da6395414ee16d6')
ready=health.evaluate(binding,out);assert ready['passed'],ready
import rosgraph,rospy
from std_msgs.msg import Header,Bool
rospy.init_node('view_seed_reload_preparation',anonymous=True,disable_rosout=True)
master=rosgraph.Master(rospy.get_name())
old=json.loads((R/'online_reload/result.json').read_text())
nodes={name:reload.node(master,name) for name in old['after_nodes']}
assert {n:{k:v for k,v in r.items() if k!='uri'} for n,r in nodes.items()}==old['after_nodes'],'live runtime differs from preserved successful gateway reload'
manual=rospy.wait_for_message('/gui/joint_direct_mode',Bool,timeout=3)
assert manual.data is True and manual._connection_header['callerid']=='/alicia_supervisor_gui'
assert rospy.get_param('/gui/joint_direct_mode') is True
epoch=rospy.wait_for_message('/alicia_d/control_reference_epoch',Header,timeout=3)
selection=rospy.get_param('/grasp_mode/selection')
runtime=dict(nodes=nodes,epoch_ns=epoch.stamp.to_nsec(),selection=selection,observed_ns=time.time_ns(),manual=True)
save(out/'runtime.json',runtime)
reg=R/'view_seed_full_regression_r2/result.json'
test=json.loads(reg.read_text());assert test['passed'] and test['source_unchanged']
assert all(sha(W/p)==h for p,h in test['source_sha256'].items())
relative='src/alicia_grasp_modes/src/alicia_grasp_modes/reachable_observation_views.py'
candidate=out/'reachable_observation_views.reviewed.py';candidate.write_bytes((W/relative).read_bytes())
baseline=V/'reachable_observation_views.before.py'
assert sha(baseline)=='7736b592f243ad709afe8ccd2f667c35a5c93c14037f5218769b51c61d631c2e'
plan=dict(workspace=str(W),output=str(V/'online_reload_r1'),launcher_sha256=sha(W/'.runtime/reload_idle_view_remote.py'),runtime=ref(out/'runtime.json'),remote_binding=ref(binding_path),changes=[dict(relative=relative,before_sha256=sha(baseline),after_sha256=sha(candidate),candidate=str(candidate))],validation=[ref(reg),ref(V/'timed_replay/result.json')])
save(out/'manifest.json',plan)
save(out/'preparation_result.json',dict(passed=True,manifest=ref(out/'manifest.json'),runtime=runtime,source_restoration_pending=True))
print(json.dumps(dict(passed=True,manifest=ref(out/'manifest.json'),nodes=len(nodes),selection=selection,epoch_ns=runtime['epoch_ns'])))
rospy.signal_shutdown('read-only preparation finished')
