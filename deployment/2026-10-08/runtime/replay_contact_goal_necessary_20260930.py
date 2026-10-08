"""Offline necessary-goal geometry replay; no ROS master or service calls."""
import importlib.util,json,hashlib,time
from pathlib import Path
import numpy as np
import rosbag
from tf.transformations import quaternion_from_matrix
from alicia_flexible_grasp_supervisor.msg import Grasp6DPlan

root=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
out=root/'ros_log/reoriented_observe_return_grasp_20260930T030005Z'
assert Path('/media/zhuyupei/SanStand').is_mount()
source=Path('src/alicia_flexible_grasp_supervisor/scripts/remote_grasp6d_node.py')
spec=importlib.util.spec_from_file_location('offline_remote_contact_goal',source)
remote=importlib.util.module_from_spec(spec);spec.loader.exec_module(remote)
node=remote.RemoteGrasp6DNode.__new__(remote.RemoteGrasp6DNode)
g=remote.GripperGeometry(.05,.002,np.array([.0434,.0286,.06]),np.array([.1175,.155,.0774]),.003)
auditpath=out/'planning_audit_generation7_timeout.json';data=json.loads(auditpath.read_text())
geometry=data['replay_geometry']; rows=[]
started=time.monotonic()
for candidate in data['stable_evaluations']:
    execution=candidate.get('execution_sequence') or {}
    if not execution.get('available'):continue
    plan=Grasp6DPlan();plan.header.frame_id='base_link';plan.header.stamp=remote.rospy.Time.from_sec(data['snapshot_stamp_sec'])
    plan.poses=[remote.make_pose_stamped('base_link', s['position_m'],s['quaternion_xyzw']).pose for s in execution['stages']]
    obj=plan.object_geometry;obj.header=plan.header;obj.valid=True;obj.source_mode='instance_mask';obj.target_track_id=data['observe_return']['target_track_id']
    for axis,value in zip('xyz',geometry['obb_center_base_m']):setattr(obj.pose_base.position,axis,value)
    matrix=np.eye(4);matrix[:3,:3]=geometry['R_base_obb'];q=quaternion_from_matrix(matrix)
    for axis,value in zip('xyzw',q):setattr(obj.pose_base.orientation,axis,value)
    for axis,value in zip('xyz',geometry['obb_size_xyz_m']):setattr(obj.size_xyz_m,axis,value)
    # Used only to satisfy geometry parsing; the necessary predicate checks
    # object overlap, not support. Do not infer a new plane or clear path.
    obj.support_normal_base.z=1.
    report=node._contact_goal_inevitable_collision(plan,g,.05,.001)
    rows.append(dict(track_id=candidate['tracking']['track_id'],variant_index=candidate['variant_index'],
        recorded_moveit=candidate.get('moveit'),necessary_goal_report=report))
known= root/'grasp_records/20260930T025045.665508Z_168dd92796ac4fccaa783e6785f676ba/keyframes.bag'
found={}
with rosbag.Bag(str(known)) as bag:
    for topic,plan,stamp in bag.read_messages(topics=['/grasp_6d/plan_enriched','/grasp_6d/preview_plan_enriched']):
        if plan.plan_id=='3aa80edb0650427434a203ee' and plan.valid:
            found[plan.plan_id]=dict(source=str(known),exact_recorded_plan=True,
                refinement_translation_m=plan.refinement_translation_m,
                refinement_rotation_deg=plan.refinement_rotation_deg,
                necessary_goal_report=node._contact_goal_inevitable_collision(plan,g,.05,.001))
assert found and all(not r['necessary_goal_report']['rejected'] for r in found.values())
summary=dict(policy='offline_necessary_goal_geometry_not_a_full_candidate_replay',
    no_service_or_hardware_calls=True,source_audit=str(auditpath),source_sha256=hashlib.sha256(auditpath.read_bytes()).hexdigest(),
    source_code_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
    candidate_count=len(rows),inevitable_overlap_count=sum(r['necessary_goal_report']['rejected'] for r in rows),
    resolver_timeouts_rejected=sum(r['necessary_goal_report']['rejected'] and (r['recorded_moveit'] or {}).get('failure_code')=='MOVEIT_TIMEOUT' for r in rows),
    elapsed_sec=time.monotonic()-started,known_grip_plans=found,candidates=rows,
    limitations=['Audit rows contain recorded post-placement contact goals, but this replay does not reconstruct the full runtime final-plan entry path.',
                 'Nonidentity registration and support restoration are excluded in the production entry path.',
                 'No collision-free trajectory, physical response, new view, or new grasp is attested.'])
(out/'contact_goal_necessary_replay.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n')
print(json.dumps({k:v for k,v in summary.items() if k not in ('candidates','known_grip_plans')},ensure_ascii=False))
print('successful_grip_rejected',found['3aa80edb0650427434a203ee']['necessary_goal_report']['rejected'])
