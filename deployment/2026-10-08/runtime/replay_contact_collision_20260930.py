"""Read-only Cartesian reconstruction from one closed grasp bag; no execution API."""
from copy import deepcopy
import argparse, hashlib, io, json, os
from pathlib import Path
import numpy as np
import rosbag, rospy
from moveit_msgs.srv import GetCartesianPath, GetCartesianPathRequest
from alicia_flexible_grasp.robot.observation_path_guard import FrozenObservationScene, SerialUrdfFk, wire_digest
from alicia_flexible_grasp.robot.contact_linear_compensation import compensate_cartesian_path
from alicia_flexible_grasp.robot.stationary_following import ARM_NAMES
from alicia_flexible_grasp.grasp.gripper_geometry import GripperGeometry, _stage_boxes, _box_corners, _obb_overlap


def plain(x):
    if hasattr(x, '__slots__'): return {k: plain(getattr(x, k)) for k in x.__slots__}
    if isinstance(x, (list, tuple)): return [plain(a) for a in x]
    return x


def sat_gap(center, rotation, size, scene):
    axes = [*rotation.T, *scene.rotation.T]
    axes += [np.cross(a, b) for a in rotation.T for b in scene.rotation.T]
    gaps=[]
    for axis in axes:
        if np.linalg.norm(axis)<1e-10: continue
        axis=axis/np.linalg.norm(axis)
        gaps.append(abs(float((center-scene.center)@axis)) - float(abs(rotation.T@axis)@(size/2)) - float(abs(scene.rotation.T@axis)@(scene.size/2)))
    return max(gaps)


def main():
    a=argparse.ArgumentParser();a.add_argument('record',type=Path);a.add_argument('output',type=Path);args=a.parse_args()
    archive=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
    if not Path('/media/zhuyupei/SanStand').is_mount() or archive not in args.output.resolve().parents:
        raise ValueError('mounted external output required')
    if not (args.record/'recorder_outcome.json').exists(): raise ValueError('closed record required')
    args.output.mkdir(exist_ok=False);(args.output/'ros').mkdir();os.environ['ROS_LOG_DIR']=str(args.output/'ros')
    latest={};plan=None;failure=None;samples=None
    topics=['/grasp_6d/plan_enriched','/joint_states','/alicia_d/accepted_joint_states','/alicia_d/sdk_command','/rosout_agg']
    with rosbag.Bag(str(args.record/'keyframes.bag')) as bag:
        for t,m,rt in bag.read_messages(topics=topics):
            if t=='/grasp_6d/plan_enriched' and m.valid:plan=deepcopy(m)
            if t in topics[1:4]:latest[t]=deepcopy(m)
            if t=='/rosout_agg' and 'Contact linear compensation rejected: ' in m.msg:
                failure=json.loads(m.msg.split('Contact linear compensation rejected: ',1)[1]);samples=deepcopy(latest);break
    if failure is None or plan.plan_id!=failure['plan_id']:raise ValueError('bound failure missing')
    rospy.init_node('readonly_contact_collision_replay',anonymous=True,disable_signals=True)
    urdf=rospy.get_param('/robot_description');fk=SerialUrdfFk(urdf,ARM_NAMES)
    geometry=rospy.get_param('/grasp_6d/remote/gripper_geometry')
    gripper=GripperGeometry(geometry['max_inner_gap_m'],geometry['width_safety_margin_per_side_m'],np.array(geometry['finger_box_xyz_m']),np.array(geometry['palm_box_xyz_m']),geometry.get('support_clearance_m',.003))
    scene=FrozenObservationScene.from_contact_plan(plan)
    def qs(topic):
        m=samples[topic];d=dict(zip(m.name,m.position));return [d[k] for k in ARM_NAMES]
    sample=dict(model_sha256=fk.model_sha256,joint_names=list(ARM_NAMES),sdk_positions_rad=qs('/alicia_d/sdk_command'),accepted_positions_rad=qs('/alicia_d/accepted_joint_states'))
    state=deepcopy(samples['/joint_states']);opening=dict(zip(state.name,state.position))['right_finger']
    values=dict(zip(ARM_NAMES,sample['accepted_positions_rad']));state.position=[values.get(k,v) for k,v in zip(state.name,state.position)]
    req=GetCartesianPathRequest();req.header.frame_id='base_link';req.start_state.joint_state=state;req.start_state.is_diff=False
    req.group_name='alicia';req.link_name='tool0';req.waypoints=[deepcopy(plan.poses[failure['goal_index']])]
    req.max_step=.003;req.jump_threshold=0.;req.avoid_collisions=True
    rospy.wait_for_service('/compute_cartesian_path',timeout=3)
    res=rospy.ServiceProxy('/compute_cartesian_path',GetCartesianPath)(req)
    if res.fraction < .999:raise ValueError('reconstruction partial: %s'%res.fraction)
    nominal=res.solution
    shifted, derivation=compensate_cartesian_path(nominal,fk,sample,plan.poses[failure['goal_index']])
    paths={}
    for label,tr in [('nominal',nominal),('full_offset_command',shifted)]:
        rows=[]
        for i,pt in enumerate(tr.joint_trajectory.points):
            T=fk(pt.positions);boxes=[]
            for name,center,size in _stage_boxes(T,gripper,opening,'y','z'):
                boxes.append(dict(name=name,overlap=bool(_obb_overlap(center,T[:3,:3],size,scene.center,scene.rotation,scene.size)),sat_best_separating_gap_m=sat_gap(center,T[:3,:3],size,scene),support_clearance_m=float(np.min(_box_corners(center,T[:3,:3],size)@scene.normal+scene.offset))))
            rows.append(dict(waypoint=i,q=list(pt.positions),tool_xyz=T[:3,3].tolist(),boxes=boxes))
        paths[label]=rows
        buf=io.BytesIO();tr.serialize(buf);(args.output/(label+'.roswire')).write_bytes(buf.getvalue())
    report=dict(scope='recorded_endpoint_and_readonly_MoveIt_reconstruction_not_executed',source_record=str(args.record),failure=failure,model_sha256=fk.model_sha256,plan_id=plan.plan_id,sample=sample,opening_m=opening,geometry=geometry,scene=dict(center=scene.center.tolist(),size=scene.size.tolist(),rotation=scene.rotation.tolist(),normal=scene.normal.tolist(),offset=scene.offset),cartesian_request=plain(req),fraction=res.fraction,paths=paths,derivation=derivation,full_retimed_failed_trajectory_not_recorded=True,no_motion_api_called=True)
    (args.output/'report.json').write_text(json.dumps(report,indent=2)+'\n');(args.output/'robot_description.urdf').write_text(urdf)
    (args.output/'source.py').write_bytes(Path(__file__).read_bytes());buf=io.BytesIO();plan.serialize(buf);(args.output/'plan.roswire').write_bytes(buf.getvalue())
    print(json.dumps(dict(directory=str(args.output),opening_m=opening,sdk_minus_encoder_rad=derivation['sdk_minus_encoder_rad'],waypoints=len(nominal.joint_trajectory.points),collisions={k:[dict(waypoint=r['waypoint'],boxes=[b for b in r['boxes'] if b['overlap']]) for r in rows if any(b['overlap'] for b in r['boxes'])] for k,rows in paths.items()},terminal={k:rows[-1] for k,rows in paths.items()}),indent=2))
if __name__=='__main__':main()
