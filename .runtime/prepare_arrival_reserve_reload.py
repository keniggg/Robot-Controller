"""Freeze validated sources and the actual idle runtime for remote replacement."""
from pathlib import Path
import hashlib,json,os,sys,time,types
W=Path(__file__).resolve().parents[1]
R=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z/gen58_resume_r1/observation_pose_fix/held_lattice_fix_r1/seed_wait_fix_r1/contact_feedback_fix_r1/actual_gen63_baseline_validation_20261023T041149Z/high_accuracy_grasp_r1/arrival_margin_repair_r1')
V=R/'view_arrival_reserve_r1';assert Path('/media/zhuyupei/SanStand').is_mount()
out=V/'preparation';out.mkdir()
os.environ['ROS_LOG_DIR']=str(out/'ros')
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def read(p):return json.loads(Path(p).read_text())
def save(p,x):p.write_text(json.dumps(x,ensure_ascii=False,indent=2)+'\n')
def ref(p):return dict(path=str(p),sha256=sha(p))
def module(p,name):
    m=types.ModuleType(name);m.__file__=str(p);exec(compile(p.read_bytes(),str(p),'exec'),m.__dict__);return m
helper=W/'.runtime/reload_idle_arrival_reserve.py';reload=module(helper,'feasible_view_reload')
hp=W/'.runtime/verify_active_remote_readiness_20261020.py'
assert sha(hp)==reload.PRIMITIVES[str(hp.relative_to(W))];health=module(hp,'health_primitive')
binding_path=R/'view_joint_reserve_r1/online_reload_r1/remote_startup_binding.json'
binding=health.load_binding(binding_path,'7fedb0f134d72930e1b2addb7bf575883e71f68f400d270e2650b0ecb8548a65')
ready=health.evaluate(binding,out);assert ready['passed'],ready
import rosgraph,rospy
from std_msgs.msg import Header,Bool
rospy.init_node('feasible_view_reload_preparation',anonymous=True,disable_rosout=True)
master=rosgraph.Master(rospy.get_name())
old=read(R/'observation_held_gateway_reload_r1/result.json');assert old['passed']
nodes={name:reload.node(master,name) for name in old['after_nodes']}
assert all({k:nodes[n][k] for k in expected}==expected for n,expected in old['after_nodes'].items()),'runtime changed since verified replacement'
manual=rospy.wait_for_message('/gui/joint_direct_mode',Bool,timeout=3)
assert manual.data is True and manual._connection_header['callerid']=='/alicia_supervisor_gui'
assert rospy.get_param('/gui/joint_direct_mode') is True
epoch=rospy.wait_for_message('/alicia_d/control_reference_epoch',Header,timeout=3)
runtime=dict(nodes=nodes,epoch_ns=epoch.stamp.to_nsec(),selection=rospy.get_param('/grasp_mode/selection'),observed_ns=time.time_ns(),manual=True)
save(out/'runtime.json',runtime)
reg=R/'arrival_reserve_full_r2/result.json';test=read(reg)
assert test['passed'] and test['source_unchanged'] and all(sha(W/p)==h for p,h in test['source_sha256'].items())
timing=V/'probe_r2/result.json';assert read(timing)['passed']
previous=read(V/'source_before/manifest.json');changes=[]
for relative in sorted(reload.ALLOWED):
    candidate=out/(Path(relative).name+'.reviewed.py');candidate.write_bytes((W/relative).read_bytes())
    if relative in previous:assert sha(V/'source_before'/relative)==previous[relative]
    changes.append(dict(relative=relative,before_sha256=sha(W/relative),after_sha256=sha(candidate),candidate=str(candidate)))
plan=dict(workspace=str(W),output=str(V/'online_reload_r1'),launcher_sha256=sha(helper),runtime=ref(out/'runtime.json'),remote_binding=ref(binding_path),changes=changes,validation=[ref(reg),ref(timing)],previous_loaded_source_evidence=ref(V/'source_before/manifest.json'),source_already_edited_on_disk=True)
save(out/'manifest.json',plan)
print(json.dumps(dict(manifest=ref(out/'manifest.json'),nodes=len(nodes),selection=runtime['selection'])))
rospy.signal_shutdown('preparation finished')
