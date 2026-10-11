"""Real source callbacks, fuser, coordinator, geometry activation and publishers.

Synthetic fixed-scene pixels use the measured GEN8 0.4s cadence / 0.42s
source latency. No network client, ROS graph or physical action is invoked.
"""
from copy import deepcopy
from types import SimpleNamespace as NS
import threading
import itertools
import numpy as np
import pytest
from test_station_compute_wait_remote import station, row_at, clock, feedback
from test_observation_measurement_remote import context, at
from test_unknown_pending_identity import deliver
from test_remote_modes import base


def configure(c):
 n=c.node; t=c.ready['status']['source_stamp_ns']
 n._near_field_phase_started_sec=(t-100000000)/1e9
 n._near_field_phase_deadline_sec=c.phase.deadline.to_nsec()/1e9
 n.last_submitted_stamp_ns=t-100000000
 n._configured_near_field_strategy=lambda:'single_snapshot_direct'
 n.near_field_planning_snapshot_frames=3; n.planning_snapshot_frames=5
 n.planning_snapshot_timeout_sec=0.;n.planning_snapshot_max_age_sec=.35
 n.planning_snapshot_max_span_sec=12.;n.planning_snapshot_max_inference_latency_sec=3.5
 n.planning_mask_min_iou=.85;n.planning_mask_max_centroid_shift_px=5.
 n.planning_max_joint_delta_rad=.01;n.mask_erosion_px=0
 n.depth_mad_scale=3.5;n.depth_mad_absolute_floor_m=.002
 n.mask_internal_hole_max_area_px=0;n.rate_hz=3.
 n.target_projection_frame_convention='opencv_optical'
 n.camera_visibility_gate_enabled=False;n.camera_visibility_diagnostic_enabled=False
 # Same shipped continuous result budget; the small fixture defaults to 1.2s.
 n.inference_coordinator._result_max_age_sec=15.0
 n.frames.update_joints(np.zeros(6))
 return t


def rgbd(c,row):
 n=c.node; msg=row['mask']; ns=msg.header.stamp.to_nsec()
 mask=n.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
 n.frames.update_color(np.zeros((*mask.shape,3),dtype=np.uint8),msg.header.stamp,msg.header.frame_id)
 n.frames.update_depth(np.full(mask.shape,350,dtype=np.uint16),msg.header.stamp,msg.header.frame_id)


def next_row(c,ns,previous,order=('mask','object','status'), partial=None):
 feedback(c, ns-10000000);feedback(c, ns+10000000)
 clock(c,ns+420000000)
 row=row_at(c,ns,recovery=previous)
 rgbd(c,row)
 for i,name in enumerate(order):
  deliver(c.node,row,(name,))
  if i<2 and partial: partial(c.node)
 assert not c.node._station_compute_pending
 assert not c.node._station_compute_retired
 return row


def submitted(c):
 n=c.node;t=configure(c)
 previous=None
 for i in range(3):
  row=next_row(c,t+i*400000000,previous)
  previous=row['status']['station_compute_wait']
  admitted=n._poll_direct_snapshot()
  if i<2: assert not admitted
 assert admitted,getattr(n,'_direct_snapshot_last_rejection',None)
 ticket=n._stream_worker_ticket
 assert ticket is not None and n._station_compute_work_certificate(ticket)
 assert n._pipeline_counters['submitted']==1
 assert ticket.payload[0].sample_stamp_ns==tuple(t+i*400000000 for i in range(3))
 return t,row,ticket


def prepared_for(n,ticket):
 snapshot=ticket.payload[0]
 geometry=NS(ok=True,center_base=np.array([0.,0.,.01]),axes_base=np.eye(3),
  size_xyz_m=np.array([.04,.03,.02]),support_normal_base=np.array([0.,0.,1.]),
  support_offset_m=0.,object_points_base=np.array([[0.,0.,.01],[.01,.01,.02]]),
  support_inlier_ratio=.95,source_mode='instance_mask')
 return NS(ticket=ticket,snapshot=snapshot,stamp=at(snapshot.stamp_ns),
  request_invalidation_generation=n._capture_geometry_generation(),geometry=geometry,
  pose_estimator=NS(T_base_optical=np.eye(4)),graspnet_input_audit={})


@pytest.mark.parametrize('order',list(itertools.permutations(('object','mask','status'))))
def test_actual_work_crosses_three_partial_handoffs_then_activates_and_publishes(station,order):
 c=station;n=c.node;t,row,ticket=submitted(c)
 prepared=prepared_for(n,ticket)
 generation=prepared.request_invalidation_generation
 checkpoints=[]
 def partial(node):
  assert node._station_compute_pending and node._station_compute_soft_wait
  assert not node.latest_object_geometry.valid
  assert not node.latest_preview_rich_plan.valid
  node._require_stream_ticket_current(ticket)
  assert node._capture_geometry_generation()==generation
  checkpoints.append(True)
 for i in range(3,6):
  row=next_row(c,t+i*400000000,row['status']['station_compute_wait'],order,partial)
  n._require_stream_ticket_current(ticket)
 assert len(checkpoints)==6
 assert n._pipeline_counters['submitted']==1
 # The original coordinator still correlates this exact original ticket.
 decision=n.inference_coordinator.complete(ticket,now_sec=c.clock[0],target_epoch=n.target_instance_epoch)
 assert decision.accepted,decision
 # Actual original activation and geometry ROS serializer, no bypass stub.
 assert n._activate_prepared_geometry(prepared)
 assert n.latest_object_geometry.valid
 rich=base.Grasp6DPlan();rich.valid=True;rich.header.stamp=prepared.stamp
 rich.header.frame_id='base_link';rich.plan_id='synthetic-computation-preview'
 rich.target_track_id=n._current_stream_target_identity().track_id
 rich.diagnostic='CONTACT_PLAN'
 legacy=base.PoseArray();legacy.header=deepcopy(rich.header)
 n._publish_preview_plan(rich,legacy,ticket=ticket)
 assert n.preview_rich_plan_pub.messages[-1].valid
 assert n._station_compute_work_published
 n._require_stream_ticket_current(ticket)  # Original post-publication worker/audit checkpoint.
 # First publication closes the exception permanently at this station.
 ns=row['status']['source_stamp_ns']
 clock(c,ns+810000000)
 assert n._station_compute_poll()
 assert not n._station_compute_soft_wait
 assert n._station_compute_ticket_code(ticket)=='TARGET_PROCESSING_PENDING'
 assert n._capture_geometry_generation()>generation


@pytest.mark.parametrize('fault',['false','explicit_pending','epoch','manual','moved','anchor'])
def test_real_invalid_measurement_permanently_cancels_unpublished_work(station,fault):
 c=station;n=c.node;t,row,ticket=submitted(c)
 ns=t+1200000000
 feedback(c,ns-10000000);feedback(c,ns+10000000);clock(c,ns+420000000)
 new=row_at(c,ns,recovery=row['status']['station_compute_wait'])
 if fault=='false':new['object'].detected=False
 elif fault=='explicit_pending':
  new['status']['state']='processing_deadline_pending'
  new['status']['station_compute_wait']=row['status']['station_compute_wait']
 elif fault=='epoch':n._measurement_remote_epoch_ns+=1
 elif fault=='manual':c.params['/gui/joint_direct_mode']=True
 elif fault=='moved':feedback(c,n._ros_source_clock_ns(),q=[.02,0,0,0,0,0])
 else:new['status']['anchor_position_base'][0]+=.01
 deliver(n,new)
 assert not n._station_compute_work_certificate(ticket)
 assert n._station_compute_ticket_code(ticket) in ('TARGET_LOST','TARGET_PROCESSING_PENDING')
 assert not n.latest_object_geometry.valid


def begin_partial(c,t,row,order=('mask','object','status')):
 ns=t+1200000000
 feedback(c,ns-10000000);feedback(c,ns+10000000);clock(c,ns+420000000)
 new=row_at(c,ns,recovery=row['status']['station_compute_wait']);rgbd(c,new)
 deliver(c.node,new,order[:1])
 assert c.node._station_compute_pending and c.node._station_compute_soft_wait
 return new


def wait_worker(n,prepared,monkeypatch):
 entered=threading.Event(); finished=threading.Event(); outcomes=[]
 wait=n._stream_condition.wait
 def tracked_wait(*a,**kw):
  entered.set();return wait(*a,**kw)
 monkeypatch.setattr(n._stream_condition,'wait',tracked_wait)
 def work():
  try:outcomes.append(n._activate_prepared_geometry(prepared))
  except Exception as exc:outcomes.append(exc)
  finally:finished.set()
 thread=threading.Thread(target=work,daemon=True);thread.start()
 assert entered.wait(2)
 assert not finished.is_set()
 return thread,finished,outcomes


def test_partial_three_source_recovery_unblocks_real_activation_without_input_lock_deadlock(station,monkeypatch):
 c=station;n=c.node;t,row,ticket=submitted(c);prepared=prepared_for(n,ticket)
 new=begin_partial(c,t,row)
 thread,finished,outcomes=wait_worker(n,prepared,monkeypatch)
 assert not n.latest_object_geometry.valid
 deliver(n,new,('object','status'))
 assert finished.wait(2);thread.join()
 assert outcomes==[True]
 assert n.latest_object_geometry.valid
 n._require_stream_ticket_current(ticket)


@pytest.mark.parametrize('fault',['manual','epoch','feedback_stale','phase','clock_back','deadline'])
def test_waiting_computation_exits_on_original_owner_feedback_and_clock_failures(station,monkeypatch,fault):
 c=station;n=c.node;t,row,ticket=submitted(c);prepared=prepared_for(n,ticket)
 begin_partial(c,t,row)
 thread,finished,outcomes=wait_worker(n,prepared,monkeypatch)
 if fault=='manual':c.params['/gui/joint_direct_mode']=True
 elif fault=='epoch':n._measurement_remote_epoch_ns+=1
 elif fault=='feedback_stale':c.clock[0]+=.81
 elif fault=='phase':n._station_compute_task_state='MOVE_PREGRASP'
 elif fault=='clock_back':c.clock[0]-=.2
 else:c.clock[0]=n._station_compute_reference['contract']['identity_deadline_ns']/1e9+.001
 with n._stream_condition:n._stream_condition.notify_all()
 assert finished.wait(2);thread.join()
 assert len(outcomes)==1 and isinstance(outcomes[0],base.StreamResultCancelled),outcomes
 assert not n.latest_object_geometry.valid


@pytest.mark.parametrize('fault',['object_false','mask_changed','processing_pending','retained_mask'])
def test_soft_recovery_never_hides_changed_historical_source(station,fault):
 from dataclasses import replace
 c=station;n=c.node;t,row,ticket=submitted(c)
 snapshot=ticket.payload[0]
 if fault=='retained_mask':
  bad=replace(snapshot,original_sample=replace(snapshot.original_sample,
      object_mask=np.zeros_like(snapshot.original_sample.object_mask)))
  assert n._station_compute_snapshot_work(bad) is None
  return
 row=next_row(c,t+1200000000,row['status']['station_compute_wait'])
 if fault=='object_false':
  msg=deepcopy(n._station_compute_objects[t]);msg.detected=False;n.object_cb(msg)
 elif fault=='mask_changed':
  msg=deepcopy(n._station_compute_masks[t][2]);msg.data=bytes(len(msg.data));n.mask_cb(msg)
 else:
  from std_msgs.msg import String
  import json
  status=deepcopy(n._station_compute_status[t]);status['state']='processing_deadline_pending'
  n._silhouette_source_status_cb(String(data=json.dumps(status)))
 assert not n._station_compute_work_certificate(ticket)
 assert n._station_compute_ticket_code(ticket) in ('TARGET_LOST','TARGET_PROCESSING_PENDING')


@pytest.mark.parametrize('failure',['clock_after_legacy','rich_send_exception'])
def test_first_publication_attempt_cannot_reopen_soft_computation(station,monkeypatch,failure):
 c=station;n=c.node;t,row,ticket=submitted(c);prepared=prepared_for(n,ticket)
 assert n._activate_prepared_geometry(prepared)
 rich=base.Grasp6DPlan();rich.valid=True;rich.header.stamp=prepared.stamp
 rich.header.frame_id='base_link';rich.plan_id='synthetic-publication';rich.diagnostic='CONTACT_PLAN'
 legacy=base.PoseArray();legacy.header=deepcopy(rich.header)
 if failure=='clock_after_legacy':
  original=n.preview_plan_pub.publish
  def legacy_send(msg):
   original(msg)
   ns=row['status']['station_compute_wait']['geometry_deadline_ns']+1000000
   clock(c,ns)
  monkeypatch.setattr(n.preview_plan_pub,'publish',legacy_send)
 else:
  def fail(msg):
   if msg.valid:raise RuntimeError('synthetic publisher failure')
  monkeypatch.setattr(n.preview_rich_plan_pub,'publish',fail)
 with pytest.raises((base.StreamResultCancelled,RuntimeError)):
  n._publish_preview_plan(rich,legacy,ticket=ticket)
 assert n._station_compute_publication_started
 assert not n._station_compute_soft_wait
 assert not n._station_compute_work_certificate(ticket)
 assert not n._station_compute_work_published


def test_original_worker_and_mode_wrappers_finish_one_request_across_three_source_handoffs(station,monkeypatch):
 """Control only CPU/remote-solver work; actual lifecycle/activation/publication.

 The synthetic solver output is not a real grasp or collision test. The
 unchanged physical-candidate regression suite covers those gates separately.
 """
 c=station;n=c.node;t,row,ticket=submitted(c)
 prepared=prepared_for(n,ticket)
 entered=threading.Event(); release=threading.Event(); metrics_done=threading.Event()
 def cpu_prediction(node,actual_ticket):
  assert node is n and actual_ticket is ticket
  entered.set();assert release.wait(3)
  node._require_stream_ticket_current(ticket)
  return base.PreparedPrediction(ticket=ticket,snapshot=prepared.snapshot,
    stamp=prepared.stamp,geometry=prepared.geometry,pose_estimator=prepared.pose_estimator,
    graspnet_input=None,candidates=(),remote_diagnostics={},remote_performance={},
    request_invalidation_generation=prepared.request_invalidation_generation,
    near_field=True,near_field_phase_id=n._near_field_phase_id)
 def selected_result(node,result):
  assert node._activate_prepared_geometry(result)
  rich=base.Grasp6DPlan();rich.valid=True;rich.header.stamp=result.stamp
  rich.header.frame_id='base_link';rich.plan_id='synthetic-worker-lifecycle-only'
  rich.diagnostic='CONTACT_PLAN';legacy=base.PoseArray();legacy.header=deepcopy(rich.header)
  node._publish_preview_plan(rich,legacy,ticket=result.ticket)
  node._require_stream_ticket_current(result.ticket)
  return dict(status='PREVIEW_READY',funnel={})
 monkeypatch.setattr(base.RemoteGrasp6DNode,'_prepare_and_predict',cpu_prediction)
 monkeypatch.setattr(base.RemoteGrasp6DNode,'_accept_prediction',selected_result)
 original_metrics=n.pipeline_metrics_pub.publish
 def record_metrics(msg):original_metrics(msg);metrics_done.set()
 monkeypatch.setattr(n.pipeline_metrics_pub,'publish',record_metrics)
 n._stream_worker=threading.Thread(target=n._stream_worker_main,daemon=True)
 n._stream_worker.start()
 try:
  assert entered.wait(2)
  for i in range(3,6):
   row=next_row(c,t+i*400000000,row['status']['station_compute_wait'],
     partial=lambda node:node._require_stream_ticket_current(ticket))
  release.set();assert metrics_done.wait(3)
  assert len(n.pipeline_metrics)==1
  metrics=n.pipeline_metrics[-1]
  assert metrics['request_id']==ticket.request_id
  assert metrics['status']=='PREVIEW_READY',metrics
  assert metrics['submitted']==metrics['started']==metrics['completed']==metrics['accepted']==1
  assert metrics['failed']==0
  assert n._station_compute_work_published
 finally:
  release.set();n.shutdown_streaming_worker()
