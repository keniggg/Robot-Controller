#!/usr/bin/env python3
"""Actual producer.process BAD-frame negative; original pixels/TF/SDK and clocks.

The accepted GOOD state is an explicitly reconstructed seed. Early phase and
the optional timely-identity schedule are counterfactuals, never real receipts.
Only ROS transport, deterministic clock and publisher sinks are substituted.
"""
import argparse
from copy import deepcopy
import importlib.util
import json
from pathlib import Path
import threading
import time
from types import SimpleNamespace
from unittest.mock import Mock, patch
import cv2
import numpy as np
import rosbag
import rospy
import tf2_py
from cv_bridge import CvBridge
from alicia_flexible_grasp_supervisor.msg import NearFieldPlanningPhase, GraspState
from alicia_grasp_modes.acquisition import InitialAcquisition
from alicia_grasp_modes.rgb_observation_source import digest
from alicia_flexible_grasp.robot.observation_view_gateway import _camera_configuration_contract

W=Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
def load(name,path):
    spec=importlib.util.spec_from_file_location(name,path);v=importlib.util.module_from_spec(spec);spec.loader.exec_module(v);return v
m=load('original_inputs',W/'.runtime/replay_attempt1_first_phase_sources_20261013.py')
module=load('original_unknown_process',W/'src/alicia_grasp_modes/scripts/unknown_tabletop_perception.py')
N=m.ROOT/'ros_log/full_latest_contact_admission_20261013T011103Z'
F=N/'verification/attempt1_first_phase_independent_v2'

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--output',required=True);a=ap.parse_args();out=Path(a.output);out.mkdir(parents=True,exist_ok=False)
    cv2.setNumThreads(1);fixture=json.loads((F/'fixture.json').read_text());camera=fixture['camera'];config=m.table.Config(**fixture['config']);k=[camera[x]for x in ('fx','fy','cx','cy')]
    context=json.loads((N/'readonly_full_frame_probe_002/context.json').read_text())
    parameters=deepcopy(context['parameters']);parameters.update({'/camera':camera,'/unknown_perception':json.loads((m.RECORD/'perception_execution_configuration.json').read_text())['parameters']['/unknown_perception'],'/gui/joint_direct_mode':False})
    qualified=next(x['message']for x in fixture['wire']if x['topic'].endswith('measurement_request')and x['message']['state']=='qualified')
    parameters['/grasp_mode/selection']=qualified['selection']
    model=parameters['/robot_description'];deployment=json.loads((N/'full_restart_acceptance/result.json').read_text());assert __import__('hashlib').sha256(model.encode()).hexdigest()==deployment['robot_description_sha256']
    exact={};buffer=tf2_py.BufferCore(rospy.Duration(600));static=[];dynamic=[]
    with rosbag.Bag(str(m.RECORD/'keyframes.bag')) as bag:
        for topic,msg,received in bag.read_messages(topics=['/tf','/tf_static','/supervisor/camera/color/image_raw','/supervisor/camera/depth/image_raw']):
            if topic=='/tf':dynamic.extend(msg.transforms)
            elif topic=='/tf_static':static.extend(msg.transforms)
            elif msg.header.stamp.to_nsec()in(m.GOOD,m.BAD,m.NEXT):exact.setdefault(msg.header.stamp.to_nsec(),{})['rgb'if'/color/'in topic else'depth']=msg
    for msg in static:buffer.set_transform_static(msg,'original_bag')
    for msg in sorted(dynamic,key=lambda x:x.header.stamp.to_nsec()):buffer.set_transform(msg,'original_bag')
    def frame(ns):
        f=np.load(F/f'{ns}_source.npz');return f['rgb'],f['depth'],f['T_base_camera'],({'rgb_valid':f['rgb_valid']}if'rgb_valid'in f else{}),f
    anchor_ns=fixture['frames'][0]['source_stamp_ns'];rgb,depth,T,kw,_=frame(anchor_ns);seg=m.table.segment_with_support_fallback(depth,k,T,config);tracker=m.table.TargetTracker(config);anchor,_=tracker.choose(seg,depth.shape);plane=seg.plane_base.copy();spatial=m.freeze_observation_spatial_reference(anchor,plane)
    rgb,depth,T,kw,fd=frame(m.PRE);hist,contour=m._appearance(rgb,m._mask(fd['recorded_mask'],rgb.shape[:2]));frozen=m.FrozenObservationAppearance(rgb,fd['recorded_mask'],kw.get('rgb_valid'),m.PRE,hist,contour)
    seg=m.table.segment_with_support_fallback(depth,k,T,config,anchor,anchor,plane);last=m.table._matching_candidates(seg.candidates,anchor,anchor,config)[0][1]
    rgb,depth,T,kw,fd=frame(m.GOOD);seg=m.table.segment_with_support_fallback(depth,k,T,config,anchor,last,plane,prefer_current_support=True);seg,pair=m.recover_arrival_matching_fragments(seg,rgb,depth,k,T,config,anchor,last,spatial,hist,contour,.5,appearance_context=dict(reference=frozen,source_ns=m.GOOD,deadline_monotonic=time.monotonic()+5,check_current=lambda:None),**kw);assert pair is not None and np.array_equal(pair[1].mask,fd['recorded_mask']);good_raw,good=pair
    good_status=next(x['message']for x in fixture['statuses']if x['message'].get('source_stamp_ns')==m.GOOD)
    bad_status=next(x['message']for x in fixture['statuses']if x['message'].get('source_stamp_ns')==m.BAD)
    original_phase=next(x for x in fixture['wire']if x['topic']=='/grasp/near_field_phase'and x['message'].get('active'))
    qualified_acks=[x for x in fixture['wire']if x['topic'].endswith('measurement_ack')and x['message'].get('state')=='qualified'and x['message'].get('accepted')]
    assert len(qualified_acks)==2
    early_ns=max(x['received_ns']for x in qualified_acks)+1
    start=bad_status['frame_timing']['processing_start_ros_ns'];stage_order=['input_age_check','decode','before_tf','tf_lookup','post_tf','segmentation','post_segment','tracking','metric_ambiguity_rgb_identity','failure_publish']
    ends=bad_status['frame_timing']['stage_end_elapsed_sec'];begin={stage:start+round(ends[stage_order[i-1]]*1e9)for i,stage in enumerate(stage_order)if i and stage_order[i-1]in ends};begin['metric_ambiguity_rgb_identity']=m.FAIL;begin['failure_publish']=bad_status['first_loss']['observed_stamp_ns']
    rows=[]
    for mode in ('original_phase_original_clock','early_phase_original_clock','early_phase_timely_identity_diagnostic'):
        now=[good_status['frame_timing']['status_build_ros_ns']]
        node=module.UnknownTabletopPerception.__new__(module.UnknownTabletopPerception);node.lock=threading.RLock();node.input_lock=threading.RLock();node.mode='unknown';node.generation=2;node.selection_ns=qualified['selection']['stamp_ns'];node.max_age=.8;node.tf_wait=.15;node.minimum_quality=.5;node.base_frame='base_link';node.camera_frame='camera_link';node.config=config;node.rgb_instance_refinement=True;node.bridge=CvBridge();node.last_shape=(camera['height'],camera['width']);node.anchor_source_stamp_ns=anchor_ns;node.latest=None;node.last_consumed_ns=m.GOOD;node.last_receive_wall=now[0]*1e-9;node.first_loss=None;node.last_good_source_stamp_ns=m.GOOD;node.tracker=m.table.TargetTracker(config);node.tracker.anchor=anchor;node.tracker.last=good_raw;node.tracker.plane_base=plane.copy();node.acquisition=InitialAcquisition(config);node._initialize_rgb_interest(parameters['/unknown_perception']);node._rgb_interest_epoch=qualified['control_epoch_ns'];node.pubs={key:Mock()for key in('object','object_mask','object_detected','raw_object_detected','object_pose_camera','object_pose_base','detector_status','rgb_reference_status')};node.buffer=SimpleNamespace(lookup_transform=lambda target,src,stamp,duration:buffer.lookup_transform_core(target,src,stamp))
        def feedback():
            node._rgb_interest_feedback.clear()
            for item in fixture['sdk']:
                if item['received_ns']<=now[0]:
                    values=dict(zip(item['name'],item['position']));node._rgb_interest_feedback.append((item['stamp_ns'],np.array([values[key]for key in module.JOINT_NAMES])))
        phase_delivered=[False]
        def stage_clock(stage):
            if stage in begin:
                changed=begin[stage]
                if mode.endswith('timely_identity_diagnostic')and stage in('metric_ambiguity_rgb_identity','failure_publish'):changed=begin['tracking']
                now[0]=max(now[0],changed);feedback()
                if (not mode.startswith('early') and not phase_delivered[0]
                        and now[0]>=original_phase['received_ns']):
                    node._station_compute_phase_cb(phase)
                    node._station_compute_state_cb(GraspState(state='PLAN_PREGRASP',active=True))
                    phase_delivered[0]=True
            original_stage(stage)
        with patch.object(module.rospy,'get_param',lambda key,default=None:parameters.get(key,default)),patch.object(module.rospy,'get_param_cached',lambda key,default=None:parameters.get(key,default)),patch.object(module.rospy,'get_master',lambda:SimpleNamespace(getParam=lambda key:(1,'sealed offline context',parameters.get(key)))),patch.object(module.rospy.Time,'now',staticmethod(lambda:module.rospy.Time(now[0]//10**9,now[0]%10**9))),patch.object(module.time,'monotonic',lambda:now[0]*1e-9),patch.object(module.rospy,'logwarn_throttle',Mock()),patch.object(module.rospy,'logwarn',Mock()),patch.object(module.rospy,'loginfo',Mock()):
            feedback();source=tuple(exact[m.GOOD][key]for key in('rgb','depth'));reference=node._prepare_locked_rgb_hold_reference(source,good,camera);assert reference is not None,'real measured seed reference failed'
            marker=dict(measurement_binding={key:deepcopy(qualified[key])for key in('selection','lease_id','session_id','view_plan_id','view_index','target_epoch','target_track_id','control_epoch_ns')},lease_id=qualified['lease_id'],qualified_source_ns=qualified['source_stamps_ns'][-1],qualified_source_stamps_ns=qualified['source_stamps_ns'],arrival_q=np.array(qualified['settle']['accepted_positions_rad']),epoch_ns=qualified['control_epoch_ns'],source_id=reference['source_id'],config_sha256=reference['config_sha256'],camera_configuration_sha256=digest(_camera_configuration_contract(camera)),anchor_fingerprint=reference['anchor_fingerprint'],reference_plane=plane.copy(),spatial_reference=spatial,fragment_histogram=hist,fragment_contour=contour,fragment_appearance_reference=frozen,session_deadline_ns=qualified['deadline_ns'],session_deadline_monotonic=qualified['deadline_ns']*1e-9)
            reference['observation_arrival_retry']=marker;node._locked_rgb_hold_reference=reference
            phase=NearFieldPlanningPhase();phase.active=True;phase.phase_id=1;phase.planning_stage='contact_replan';phase.reference_target_track_id=qualified['target_track_id'];ref_phase=qualified['source_stamps_ns'][-1]if mode.startswith('early')else original_phase['message']['reference_source_stamp'];phase.reference_source_stamp=module.rospy.Time(ref_phase//10**9,ref_phase%10**9);phase_ns=early_ns if mode.startswith('early')else original_phase['message']['header']['stamp'];phase.header.stamp=module.rospy.Time(phase_ns//10**9,phase_ns%10**9);phase.deadline=module.rospy.Time((phase_ns+90000000000)//10**9,(phase_ns+90000000000)%10**9)
            if mode.startswith('early'):
                node._station_compute_phase_cb(phase);node._station_compute_state_cb(GraspState(state='PLAN_PREGRASP',active=True))
            else:
                node._station_compute_phase_cb(NearFieldPlanningPhase(active=False))
                prior_state=max((item for item in fixture['wire']if item['topic']=='/grasp/state'and item['received_ns']<=now[0]),key=lambda x:x['received_ns'])['message']
                node._station_compute_state_cb(GraspState(state=prior_state['state'],active=prior_state['active']))
            node.status('ready','tracked',source,good)
            registered=getattr(node,'_station_compute_registration',None)is not None
            assert registered==mode.startswith('early'),(mode,registered)
            node.pubs={key:Mock()for key in node.pubs};now[0]=start;feedback();node.latest=tuple(exact[m.BAD][key]for key in('rgb','depth'));node.last_receive_wall=now[0]*1e-9
            original_stage=node._frame_timing_stage;node._frame_timing_stage=stage_clock
            node.process(None)
            status=json.loads(node.pubs['detector_status'].publish.call_args[0][0].data)
            row=dict(mode=mode,phase_ns=phase_ns,phase_schedule_counterfactual=mode.startswith('early'),processing_clock_counterfactual=mode.endswith('timely_identity_diagnostic'),registered_good=registered,state=status['state'],detail=status['detail'],last_good_ns=node.last_good_source_stamp_ns,first_loss=node.first_loss,detected=node.pubs['object'].publish.call_args[0][0].detected,frame_timing=status.get('frame_timing'))
            assert node.last_good_source_stamp_ns==m.GOOD and node.tracker.last is good_raw
            assert not row['detected'] and node.first_loss is not None
            assert status['detail']=='perception_failure: '+('metric_ambiguity_no_unique_complete_rgb_identity'if mode.endswith('timely_identity_diagnostic')else'source_exceeds_existing_frame_limit'),row
            # Continue with the next actually saved source. Failure cannot be
            # erased by a later visually good frame or a source stamp refresh.
            next_status=next(item['message']for item in fixture['statuses']if item['message'].get('source_stamp_ns')==m.NEXT)
            next_timing=next_status['frame_timing'];next_start=next_timing['processing_start_ros_ns'];next_ends=next_timing['stage_end_elapsed_sec']
            def next_stage_clock(stage):
                i=stage_order.index(stage)if stage in stage_order else 0
                if i and stage_order[i-1]in next_ends:now[0]=max(now[0],next_start+round(next_ends[stage_order[i-1]]*1e9));feedback()
                original_stage(stage)
            node._frame_timing_stage=next_stage_clock;now[0]=next_start;feedback();node.latest=tuple(exact[m.NEXT][key]for key in('rgb','depth'));node.pubs={key:Mock()for key in node.pubs};first_loss=deepcopy(node.first_loss)
            node.process(None)
            follow=json.loads(node.pubs['detector_status'].publish.call_args[0][0].data)
            assert node.last_good_source_stamp_ns==m.GOOD and node.first_loss==first_loss and not node.pubs['object'].publish.call_args[0][0].detected
            row['following_actual_saved_frame']=dict(source_ns=m.NEXT,processing_start_ns=next_start,state=follow['state'],detail=follow['detail'],last_good_ns=node.last_good_source_stamp_ns,first_loss_preserved=True,detected=False)
            rows.append(row);print(json.dumps({key:value for key,value in row.items()if key!='frame_timing'}),flush=True)
    report=dict(scope='actual process negative from explicitly seeded original GOOD state, original BAD RGB-D/TF/SDK and production gates; no rejected frame refresh',rows=rows,seed_reference='GOOD exact recorded mask reproduced by original geometry; acknowledged marker reconstructed from original qualified wire receipt',parameters='original camera/config/selection and same deployed model plus handeye from later readonly snapshot; no live ROS calls',source_sha256={str(p):m.sha(p)for p in[Path(__file__),W/'src/alicia_grasp_modes/scripts/unknown_tabletop_perception.py',W/'src/alicia_grasp_modes/src/alicia_grasp_modes/station_compute_wait_producer.py']},original_bad_appearance_preserved=True,source_restamped=False,no_ros_master=True,motion_authorized=False)
    (out/'result.json').write_text(json.dumps(m.plain(report),ensure_ascii=False,indent=2)+'\n');print('RESULT '+str(out/'result.json'))
if __name__=='__main__':main()
