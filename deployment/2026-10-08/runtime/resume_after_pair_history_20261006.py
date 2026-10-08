"""Preserve original operator alignment across the measured camera-only reload.

The activation edge is explicit. Old identity, pixels and plans are not reused.
This script then uses the existing selection and recorded-grasp entry points.
"""
from datetime import datetime, timezone
import copy
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys

import numpy as np
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from alicia_flexible_grasp_supervisor.msg import GraspState

W = Path(__file__).resolve().parents[1]
E = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
assert Path('/media/zhuyupei/SanStand').is_mount()
DEPLOY = E/'ros_log/regional_production_20261006T105318.899385Z/result.json'
D = json.loads(DEPLOY.read_text())
assert D['camera_loaded'] is True and D['static_validation_passed'] is True
assert D.get('error') is None and D['events'][-1]['event']=='finished'
v = json.loads(Path(D['manifest_path']).read_text())
assert v['tests_passed'] and v['reviewed'] and v['source_unchanged']
for name, expected in v['files'].items():
    assert hashlib.sha256((W/name).read_bytes()).hexdigest()==expected, name
prior_path=E/'ros_log/gen17_support_measurement_20261006/perception_selection/new_target_selection.json'
prior=json.loads(prior_path.read_text())
assert prior['response']['success'] and not prior.get('failure')
selection=prior['new_selection']
assert selection['generation']==17 and prior['preview_only'] is True
capture=E/'ros_log/gen17_support_measurement_20261006/raw_support_v2/summary.json'
closed=json.loads(capture.read_text())
assert closed['archive_closed'] and closed['complete_without_writer_error']
index=W/'.runtime/current_alignment_supplement_view_20261006.json'
old=Path(json.loads(index.read_text())['receipt'])
r=json.loads(old.read_text())
event_path=Path(r['event_file'])
assert hashlib.sha256(event_path.read_bytes()).hexdigest()==r['event_file_sha256']
origin=json.loads(event_path.read_text())
assert r['confirmation_id']=='call_bxnlc8gAuMDZZirhSJaHItcz:0'
spec=importlib.util.spec_from_file_location('alignment_guard',W/'tools/diagnostics/prepare_rgb_acquisition_retry.py')
guard=importlib.util.module_from_spec(spec);spec.loader.exec_module(guard)
stamp=datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
out=E/'ros_log'/('revalidated_alignment_pair_history_'+stamp)
out.mkdir();os.environ['ROS_LOG_DIR']=str(out/'ros')
rospy.init_node('resume_measured_alignment_after_camera_repair',anonymous=True,disable_signals=True)
assert not rospy.wait_for_message('/grasp/state',GraspState,timeout=4).active
assert not rospy.get_param('/gui/joint_direct_mode',True)
assert rospy.get_param('/grasp/unknown_lift_after_close_enabled') is False
assert not rospy.get_param('/rgb_acquisition/ledger',{})
assert guard.parse_selection(rospy.get_param('/grasp_mode/selection'))==selection
epoch=rospy.wait_for_message('/alicia_d/control_reference_epoch',Header,timeout=3).stamp.to_nsec()
msg=rospy.wait_for_message('/alicia_d/accepted_joint_states',JointState,timeout=3)
assert msg.header.frame_id=='sdk_measured'
guard.require_fresh_source(msg.header.stamp.to_nsec(),rospy.Time.now().to_nsec())
q=dict(zip(msg.name,msg.position));positions=[q[name] for name in guard.JOINT_NAMES]
assert np.max(np.abs(guard._q(positions)-guard._q(r['bound_feedback']['positions'])))<=guard.POSE_TOLERANCE_RAD
handeye=rospy.get_param('/handeye');model=rospy.get_param('/robot_description')
before=guard.retry_source_context(selection,epoch,D['camera_parameters_before'],handeye,model)
expected_before=dict(origin['source_context'],selection=selection)
assert guard.digest(before)==guard.digest(expected_before), 'unexpected pre-reload scene/context change'
after=guard.retry_source_context(selection,epoch,rospy.get_param('/camera'),handeye,model)
expected_after=guard.retry_source_context(selection,epoch,D['camera_parameters_after'],handeye,model)
assert guard.digest(after)==guard.digest(expected_after), 'context changed after verified deployment'
comparison_before=copy.deepcopy(before);comparison_after=copy.deepcopy(after)
old_activation=comparison_before['camera']['runtime_profile_geometry'].pop('geometry_activation_stamp_ns')
new_activation=comparison_after['camera']['runtime_profile_geometry'].pop('geometry_activation_stamp_ns')
assert int(new_activation)>int(old_activation)
assert guard.digest(comparison_before)==guard.digest(comparison_after), 'camera geometry changed beyond activation'
feedback=dict(stamp_ns=str(msg.header.stamp.to_nsec()),frame_id='sdk_measured',positions=positions)
derivation=dict(kind='unchanged_alignment_after_verified_camera_pair_history_reload',
    new_user_confirmation_received=False, parent_receipt=str(old),
    parent_receipt_sha256=hashlib.sha256(old.read_bytes()).hexdigest(),
    prior_diagnostic_selection=str(prior_path),
    prior_diagnostic_selection_sha256=hashlib.sha256(prior_path.read_bytes()).hexdigest(),
    deployment=str(DEPLOY),deployment_sha256=hashlib.sha256(DEPLOY.read_bytes()).hexdigest(),
    old_activation_stamp_ns=old_activation,new_activation_stamp_ns=new_activation,
    reason='Camera-only verified code reload; diagnostic gen16/17 sent no planning or motion; current measured alignment retained, all perception and plans rebuilt')
event=dict(confirmation_source=r['confirmation_source'],confirmed_ns=r['confirmed_ns'],
    confirmation_time_semantics='original actual user reply, later measured revalidation',
    target_aligned=True,source_context=after,bound_feedback=feedback,derivation=derivation,
    robot_commands_sent=False,torque_commands_sent=False)
event_path=out/'alignment_revalidation_event.json'
event_path.write_text(json.dumps(event,ensure_ascii=False,indent=2)+'\n')
r.update(selection=selection,bound_feedback=feedback,source_context_sha256=guard.digest(after),
    event_file=str(event_path),event_file_sha256=hashlib.sha256(event_path.read_bytes()).hexdigest(),
    derivation=derivation,recorded_at_utc=datetime.now(timezone.utc).isoformat())
receipt=out/'alignment_receipt.json'
receipt.write_text(json.dumps(r,ensure_ascii=False,indent=2)+'\n')
index.write_text(json.dumps(dict(receipt=str(receipt),confirmation_id=r['confirmation_id']),indent=2)+'\n')
print('REVALIDATED '+str(receipt),flush=True)
subprocess.run([sys.executable,str(W/'.runtime/select_pair_history_supplement_20261006.py')],cwd=W,check=True)
subprocess.run([sys.executable,str(W/'.runtime/launch_supplement_view_grasp_20261006.py')],cwd=W,check=True)
