"""Carry unchanged alignment after a recorded transient stream outage.

No new user reply is claimed. Preserve original confirmation time and source;
record the parent receipt and the new measured context separately.
"""
from pathlib import Path
from datetime import datetime,timezone
import json,hashlib,os,importlib.util
import numpy as np
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from alicia_flexible_grasp_supervisor.msg import GraspState
w=Path(__file__).resolve().parents[1];m=Path('/media/zhuyupei/SanStand');assert m.is_mount()
index=w/'.runtime/current_alignment_supplement_view_20261006.json';old=Path(json.loads(index.read_text())['receipt']);r=json.loads(old.read_text());selection=json.loads((old.parent/'new_target_selection.json').read_text())['new_selection']
deployment=json.loads((w/'.runtime/current_observation_expansion_deployment_20261006.json').read_text());assert deployment['passed'] is True and deployment['tested_source_unchanged'] is True
origin=json.loads(Path(r['event_file']).read_text());assert hashlib.sha256(Path(r['event_file']).read_bytes()).hexdigest()==r['event_file_sha256']
assert r['confirmation_id']=='call_bxnlc8gAuMDZZirhSJaHItcz:0'
continuity=m/'实验记录/Robot-Controller/ros_log/rgbd_continuity_20261006T070546Z/report.json'
v=json.loads(continuity.read_text()); assert v['motion_commands_sent'] is False and v['configuration_changed'] is False
assert v['selection']['generation']==10
for stream in ('rgb','depth'):
    assert v['summary'][stream]['count']>=200
    assert v['summary'][stream]['max_source_gap_sec']<.8 and v['summary'][stream]['max_receipt_gap_sec']<.8 and v['summary'][stream]['max_age_sec']<.8
assert v['summary']['exact_common_stamps']>=200
previous=m/'实验记录/Robot-Controller/grasp_records/20261006T065349.606099Z_446b1496b4cf4b1c85fb89bf752666ee'
manifest=json.loads((previous/'manifest.json').read_text()); assert manifest['state']=='complete' and manifest['recorder_closed'] is True
assert 'metric_runner_finished' in (previous/'runner.log').read_text()
assert not list((previous/'keyframes_audits').iterdir())

stamp=datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ');out=m/'实验记录/Robot-Controller/ros_log'/('revalidated_alignment_supplement_'+stamp);out.mkdir();os.environ['ROS_LOG_DIR']=str(out/'ros')
spec=importlib.util.spec_from_file_location('retry_guard',w/'tools/diagnostics/prepare_rgb_acquisition_retry.py');guard=importlib.util.module_from_spec(spec);spec.loader.exec_module(guard)
rospy.init_node('revalidate_existing_user_alignment_after_fix',anonymous=True,disable_signals=True)
assert not rospy.wait_for_message('/grasp/state',GraspState,timeout=4).active
assert rospy.get_param('/grasp/unknown_lift_after_close_enabled') is False
assert guard.parse_selection(rospy.get_param('/grasp_mode/selection'))==selection
assert not rospy.get_param('/gui/joint_direct_mode',True), 'new manual owner requires current confirmation'
epoch=rospy.wait_for_message('/alicia_d/control_reference_epoch',Header,timeout=3)
msg=rospy.wait_for_message('/alicia_d/accepted_joint_states',JointState,timeout=3);assert msg.header.frame_id=='sdk_measured';q=dict(zip(msg.name,msg.position));positions=[q[n] for n in guard.JOINT_NAMES]
guard.require_fresh_source(msg.header.stamp.to_nsec(),rospy.Time.now().to_nsec())
assert max(abs(a-b) for a,b in zip(positions,r['bound_feedback']['positions']))<=guard.POSE_TOLERANCE_RAD
ctx=guard.retry_source_context(selection,epoch.stamp.to_nsec(),rospy.get_param('/camera'),rospy.get_param('/handeye'),rospy.get_param('/robot_description'))
prior=dict(origin['source_context'],selection=selection);assert guard.digest(ctx)==guard.digest(prior)
feedback=dict(stamp_ns=str(msg.header.stamp.to_nsec()),frame_id='sdk_measured',positions=positions)
derivation=dict(kind='unchanged_alignment_after_stream_recovery',new_user_confirmation_received=False,parent_receipt=str(old),parent_receipt_sha256=hashlib.sha256(old.read_bytes()).hexdigest(),deployment=deployment['directory'],reason='generation10 failed before any preview or motion due to SDK stream outage; camera subsequently produced a continuous 20 second fresh exact-pair window; original user authority retained, old target identity not restored',continuity_report=str(continuity),continuity_sha256=hashlib.sha256(continuity.read_bytes()).hexdigest(),previous_record=str(previous),camera_outage_root_cause_solved=False,current_pose_matches_parent=True)
event=dict(confirmation_source=r['confirmation_source'],confirmed_ns=r['confirmed_ns'],confirmation_time_semantics='original explicit user reply retained; this is later measured revalidation',target_aligned=True,source_context=ctx,bound_feedback=feedback,derivation=derivation,robot_commands_sent=False,torque_commands_sent=False)
event_path=out/('alignment_revalidation_event_'+stamp+'.json');event_path.write_text(json.dumps(event,ensure_ascii=False,indent=2)+'\n')
r.update(selection=selection,bound_feedback=feedback,source_context_sha256=guard.digest(ctx),event_file=str(event_path),event_file_sha256=hashlib.sha256(event_path.read_bytes()).hexdigest(),derivation=derivation,recorded_at_utc=datetime.now(timezone.utc).isoformat())
receipt=out/('alignment_receipt_'+stamp+'.json');receipt.write_text(json.dumps(r,ensure_ascii=False,indent=2)+'\n');index.write_text(json.dumps(dict(receipt=str(receipt),confirmation_id=r['confirmation_id']),indent=2)+'\n');print(receipt,flush=True)
