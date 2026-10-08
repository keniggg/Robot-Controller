"""Offline replay of the exact recorded lift; never imports a ROS node."""
from pathlib import Path
from copy import deepcopy
import hashlib,json,sys
from datetime import datetime, timezone
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src/alicia_flexible_grasp_supervisor/tests'))
from test_lift_path_continuity import recorded,point
from alicia_flexible_grasp.robot.lift_path_continuity import derive_lift_path,validate_lift_trajectory,occupancy_at,_intersection_in_pad
from alicia_flexible_grasp.robot.observation_path_guard import segment_coefficients,_polynomial,wire_digest
from alicia_flexible_grasp.grasp.gripper_geometry import GripperGeometry,_box_corners,_stage_boxes,_obb_overlap

root=Path(__file__).resolve().parents[1]
out=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/grasp_repair_resume_20260928T120606Z/lift_path_continuity')
out=out/('reviewed_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ'))
if not Path('/media/zhuyupei/SanStand').is_mount():raise SystemExit('SanStand unavailable; no fallback')
out.mkdir(parents=True,exist_ok=False)
data,fk,plan,tr,sample,opening=recorded()
shift,context,audit=derive_lift_path(tr,plan,fk,sample,opening)
bias=context.sdk-context.measured
cfg=data['parameters']['/grasp_6d']['remote']['gripper_geometry']
g=GripperGeometry(cfg['max_inner_gap_m'],cfg['width_safety_margin_per_side_m'],np.asarray(cfg['finger_box_xyz_m']),np.asarray(cfg['palm_box_xyz_m']),cfg.get('support_clearance_m',.003))
results={}
for name,trajectory,held in [('historical_nominal_with_SDK_bridge',tr,context.sdk),('nominal_measured_bridge',tr,context.measured),('complete_shift_with_SDK_bridge',shift,context.sdk)]:
 try:results[name]={'known_geometry_result':validate_lift_trajectory(trajectory,context,fk,g,point(held))}
 except Exception as e:results[name]={'known_geometry_unresolved':str(e),'passed':False}
first=next(i for i,p in enumerate(tr.joint_trajectory.points) if p.time_from_start.to_sec()>0)
duration=tr.joint_trajectory.points[first].time_from_start.to_sec()
hold=point(context.sdk)
old_coeff=segment_coefficients(hold,tr.joint_trajectory.points[first],duration,6)
first_only=deepcopy(tr);first_only.joint_trajectory.points[0].positions=context.sdk.tolist()
first_coeff=segment_coefficients(hold,first_only.joint_trajectory.points[first],duration,6)
new_coeff=segment_coefficients(hold,shift.joint_trajectory.points[first],duration,6)
nominal_coeff=segment_coefficients(point(context.measured),tr.joint_trajectory.points[first],duration,6)
fractions=np.linspace(0,1,101)
bridge_rows=[]
for f in fractions:
 old=_polynomial(old_coeff,f);new=_polynomial(new_coeff,f);expected=_polynomial(nominal_coeff,f)
 bridge_rows.append(dict(seconds=float(f*duration),historical_sdk=old.tolist(),full_shift_sdk=new.tolist(),
    hypothetical_full_shift_encoder=(new-bias).tolist(),nominal_encoder=expected.tolist()))
T=fk(context.measured);body=[]
for name,center,size in _stage_boxes(T,g,opening,'y','z'):
 body.append(dict(name=name,initial_target_obb_overlap=bool(_obb_overlap(center,T[:3,:3],size,context.scene.center,context.scene.rotation,context.scene.size)),
  initial_contact_intersection_proven=bool(_intersection_in_pad(T,context.scene.center,context.scene.rotation,context.scene.size,name,g,opening)) if 'finger' in name else False,
  minimum_support_m=float(np.min(_box_corners(center,T[:3,:3],size)@context.scene.normal+context.scene.offset))))
terminal=np.asarray(tr.joint_trajectory.points[-1].positions)
old_hyp=fk(terminal-bias);goal=fk(terminal)
artifact=dict(schema_version=1,scope='closed_bag_offline_replay_no_ROS_IO_no_execution',
 source_bag=data['source_bag'],source_goal_id=data['source_goal_id'],goal_receipt=data['goal_receipt'],
 source_fixture_sha256=hashlib.sha256((root/'src/alicia_flexible_grasp_supervisor/tests/fixtures/lift_following_20260928.json').read_bytes()).hexdigest(),
 source_sha256={str(p.relative_to(root)):hashlib.sha256(p.read_bytes()).hexdigest() for p in [root/'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot/lift_path_continuity.py',Path(__file__).resolve()]},
 derivation=audit,exact_original_trajectory_sha256=wire_digest(tr),waypoints=len(tr.joint_trajectory.points),
 original_duration_seconds=tr.joint_trajectory.points[-1].time_from_start.to_sec(),
 accepted_opening_m=opening, sdk_minus_accepted_deg=np.degrees(bias).tolist(),
 first_point_only_Noetic_bridge_identical=bool(np.array_equal(old_coeff,first_coeff)),
 constant_bias_hypothesis_old_terminal_error_m=float(np.linalg.norm(old_hyp[:3,3]-goal[:3,3])),
 constant_bias_hypothesis_complete_shift_terminal_error_m=float(np.linalg.norm(fk(np.asarray(shift.joint_trajectory.points[-1].positions)-bias)[:3,3]-goal[:3,3])),
 maximum_full_shift_bridge_encoder_difference_rad=max(float(np.max(np.abs(np.array(r['hypothetical_full_shift_encoder'])-r['nominal_encoder']))) for r in bridge_rows),
 first_positive_waypoint_seconds=duration,bridge_rows=bridge_rows,
 initial_geometry=body,geometry_checks=results,
 missing_production_inputs=['runtime_tool0_to_camera_collision_body_transform_and_sourced_body_geometry','mount_collision_geometry','current_whole_arm_MoveIt_scene_validation_for_all_derived_curves','fresh_stationary_runtime_sample_and_execution_epoch_binding'],
 interpretation=['First-point-only change is discarded at zero-stamped Noetic trajectory start.',
  'Full constant offset fixes command-path continuity only under an explicitly hypothetical constant following-bias model.',
  'Old CAD-envelope/pad rejection is unresolved conservative model evidence, not proof of actual illegal finger contact.',
  'World target stays present; carried OBB is an alternative hypothesis, never attachment confirmation.',
  'No source changes are wired to production, no new lift or ROS command was issued.'],
 motion_authorized=False,hardware_response_verified=False,actual_grasp_success=False)
(out/'historical_lift_replay.json').write_text(json.dumps(artifact,indent=2,ensure_ascii=False)+'\n')
for name,msg in [('historical_lift_trajectory',tr),('derived_lift_trajectory',shift)]:
 import io
 buf=io.BytesIO();msg.serialize(buf);(out/(name+'.roswire')).write_bytes(buf.getvalue())
print(json.dumps({k:artifact[k] for k in ['waypoints','accepted_opening_m','sdk_minus_accepted_deg','first_point_only_Noetic_bridge_identical','constant_bias_hypothesis_old_terminal_error_m','constant_bias_hypothesis_complete_shift_terminal_error_m','maximum_full_shift_bridge_encoder_difference_rad','geometry_checks']},indent=2))
print(out)
