#!/usr/bin/env python3
"""Two request-only negative controls; never publishes or changes scene/state."""
from pathlib import Path
import hashlib,io,json,os,struct,time
from datetime import datetime,timezone
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
STORE=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
if not Path('/media/zhuyupei/SanStand').is_mount():raise SystemExit('SanStand unavailable')
SOURCE=STORE/'ros_log/grasp_repair_resume_20260928T120606Z/lift_path_continuity/native_readonly_20260928T143048Z'
OUT=SOURCE/('negative_controls_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ'));OUT.mkdir()
os.environ['ROS_LOG_DIR']=str(OUT/'roslog')
import rospy,rospkg,yaml
from moveit_msgs.srv import GetStateValidity,GetStateValidityRequest
from alicia_flexible_grasp.robot.lift_moveit_adapter import ClosedLiftMoveItAdapter,_pose
from alicia_flexible_grasp.robot.stationary_following import ARM_NAMES
rospy.init_node('lift_native_readonly_negative_controls',anonymous=True,disable_signals=True)
source_report=json.loads((SOURCE/'native_moveit_report.json').read_text())
model=rospy.get_param('/robot_description')
assert hashlib.sha256(model.encode()).hexdigest()==source_report['current_model_sha256']
a=ClosedLiftMoveItAdapter.connect(model)
mesh=Path(rospkg.RosPack().get_path('alicia_d_descriptions'))/'meshes/base_link.STL'
raw=mesh.read_bytes();count=struct.unpack_from('<I',raw,80)[0]
assert len(raw)==84+50*count,'expected binary STL'
center=None
for i in range(count):
 vertices=np.array(struct.unpack_from('<9f',raw,84+50*i+12)).reshape(3,3)
 if np.linalg.norm(np.cross(vertices[1]-vertices[0],vertices[2]-vertices[0]))>1e-10:
  center=vertices.mean(axis=0);break
assert center is not None
before=a.scene_digest();rows=[]
for index,frame in [(2,'base_link'),(3,'tool0')]:
 req=GetStateValidityRequest();req.deserialize((SOURCE/('%03d_request.roswire'%index)).read_bytes())
 body=req.robot_state.attached_collision_objects[-1]
 assert body.object.header.frame_id==frame
 q=np.array([dict(zip(req.robot_state.joint_state.name,req.robot_state.joint_state.position))[n] for n in ARM_NAMES])
 world=np.eye(4);world[:3,3]=center
 body.object.pose=_pose(world if frame=='base_link' else np.linalg.inv(a.fk(q))@world)
 body.object.id += '_negative_control'
 stream=io.BytesIO();req.serialize(stream);(OUT/(frame+'_request.roswire')).write_bytes(stream.getvalue())
 t=time.monotonic();response=a.check_state(req);elapsed=time.monotonic()-t
 stream=io.BytesIO();response.serialize(stream);(OUT/(frame+'_response.roswire')).write_bytes(stream.getvalue())
 rows.append(dict(frame=frame,expected_valid=False,valid=response.valid,elapsed_seconds=elapsed,
  contacts=[yaml.safe_load(str(c)) for c in response.contacts]))
after=a.scene_digest()
report=dict(scope='request_local_only_native_body_negative_controls',parent_report_sha256=hashlib.sha256((SOURCE/'native_moveit_report.json').read_bytes()).hexdigest(),
 model_sha256=source_report['current_model_sha256'],base_mesh_path=str(mesh),base_mesh_sha256=hashlib.sha256(raw).hexdigest(),
 test_object_center_on_known_base_mesh_triangle_m=center.tolist(),object_size_unchanged=True,
 expected_world_pose_identical_in_both_frame_requests=True,results=rows,scene_before=before,scene_after=after,
 scene_unchanged=before==after,motion_authorized=False,actual_lift_verified=False,issued_motion_commands=0,
 source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
(OUT/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
print(json.dumps(report,ensure_ascii=False,indent=2));print('ARCHIVE',OUT)
