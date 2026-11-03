#!/usr/bin/env python3
"""Actual qualified callback and serial source replay with elapsed CPU charged.

No ROS master. Accepted third-frame state is seeded only from its sealed receipt.
Shared recorded stages are conservative time floors; all newly executed stages
continue advancing by real perf_counter_ns. No stopped computation clock.
"""
import argparse
from copy import deepcopy
from dataclasses import asdict
import importlib.util
import json
from pathlib import Path
import sys
import time
from unittest.mock import Mock, patch
from types import SimpleNamespace
import cv2
import numpy as np
from std_msgs.msg import String
from alicia_flexible_grasp_supervisor.msg import NearFieldPlanningPhase, GraspState
from alicia_grasp_modes.observation_arrival_confirmation import ObservationArrivalConfirmation
from alicia_grasp_modes.rgb_observation_source import digest
from alicia_grasp_modes.rgb_observation_handoff import _candidate_fingerprint
from alicia_flexible_grasp.robot.observation_view_gateway import _camera_configuration_contract
import alicia_grasp_modes.observation_appearance as appearance_module
import alicia_grasp_modes.station_compute_wait_producer as producer_module

W=Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
S=W/'.runtime/gen13_station_fragment_identity_20261012'
N=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_contact_admission_20261013T011103Z')
QUAL=1791854971572257757
SOURCES=(1791854972104432344,1791854972438854694,1791854972907250881,1791854973041301727,1791854973238121509)
def load(name,path):
    spec=importlib.util.spec_from_file_location(name,path);mod=importlib.util.module_from_spec(spec);sys.modules[name]=mod;spec.loader.exec_module(mod);return mod
def sha(path):return __import__('hashlib').sha256(path.read_bytes()).hexdigest()

def main():
    p=argparse.ArgumentParser();p.add_argument('--output',required=True);args=p.parse_args();out=Path(args.output)
    assert Path(appearance_module.__file__).resolve()==(S/'src/alicia_grasp_modes/src/alicia_grasp_modes/observation_appearance.py').resolve(),'candidate package PYTHONPATH required'
    paths=[S/'src/alicia_grasp_modes/scripts/unknown_tabletop_perception.py',Path(producer_module.__file__),Path(appearance_module.__file__)]
    hashes={str(path):sha(path)for path in paths}
    harness=W/'.runtime/replay_attempt1_full_process_negative_20261013.py';text=harness.read_text();prefix=text[:text.index('    rows=[]\n    for mode')]
    prefix=prefix.replace('in(m.GOOD,m.BAD,m.NEXT)',f'in{(QUAL,)+SOURCES!r}')+'    return locals()\n'
    namespace={'__name__':'offline_preparation','__file__':str(harness)};exec(compile(prefix,str(harness),'exec'),namespace)
    namespace['F']=N/'verification/attempt1_first_phase_with_qualified_source'
    data=namespace['main']();m=namespace['m'];module=load('gen13_actual_unknown',paths[0])
    assert producer_module.StationComputeWaitProducerMixin in module.UnknownTabletopPerception.__mro__
    assert module.FrozenObservationAppearance is appearance_module.FrozenObservationAppearance
    camera=data['camera'];config=data['config'];qualified=data['qualified'];parameters=data['parameters'];fixture=data['fixture'];buffer=data['buffer'];anchor=data['anchor'];plane=data['plane'];spatial=data['spatial'];k=data['k'];exact=data['exact']
    qrgb,qdepth,qT,qkwargs,qfile=data['frame'](QUAL)
    qseg=m.table.segment_with_support_fallback(qdepth,k,qT,config,anchor,data['last'],plane,prefer_current_support=True)
    qseg,qpair=m.recover_arrival_matching_fragments(qseg,qrgb,qdepth,k,qT,config,anchor,data['last'],spatial,data['hist'],data['contour'],.5,appearance_context=dict(reference=data['frozen'],source_ns=QUAL,deadline_monotonic=time.monotonic()+5,check_current=lambda:None),**qkwargs)
    assert qpair is not None and np.array_equal(qpair[1].mask,qfile['recorded_mask'])
    qraw,qrefined=qpair
    request_row=next(x for x in fixture['wire']if x['topic'].endswith('measurement_request')and x['message']['state']=='qualified')
    now=[request_row['received_ns']];running=[False];start_ns=[now[0]];perf_start=[0];padding_ns=[0]
    node_setup=next(line for line in text.splitlines()if line.lstrip().startswith('node=module.UnknownTabletopPerception.__new__'))
    setup_scope={**namespace,**data,'module':module,'now':now,'QUAL':QUAL,'good_raw':qraw}
    exec(node_setup.strip().replace('m.GOOD','QUAL'),setup_scope);node=setup_scope['node']
    node._observation_measurement_ack_pub=Mock()
    def feedback(at):
        node._rgb_interest_feedback.clear()
        for item in fixture['sdk']:
            if item['received_ns']<=at:
                values=dict(zip(item['name'],item['position']));node._rgb_interest_feedback.append((item['stamp_ns'],np.array([values[key]for key in module.JOINT_NAMES])))
    def clock_ns():
        value=start_ns[0]+time.perf_counter_ns()-perf_start[0]+padding_ns[0]if running[0]else now[0]
        feedback(value)
        return value
    def ros_now():
        value=clock_ns();return module.rospy.Time(value//10**9,value%10**9)
    source_status={x['message']['source_stamp_ns']:x['message']for x in fixture['statuses']if x['message'].get('source_stamp_ns')}
    rows=[]
    with patch.object(module.rospy,'get_param',lambda key,default=None:parameters.get(key,default)),patch.object(module.rospy,'get_param_cached',lambda key,default=None:parameters.get(key,default)),patch.object(module.rospy,'get_master',lambda:SimpleNamespace(getParam=lambda key:(1,'sealed offline parameter snapshot',parameters.get(key)))),patch.object(module.rospy.Time,'now',staticmethod(ros_now)),patch.object(module.time,'monotonic',lambda:clock_ns()*1e-9),patch.object(module.rospy,'logwarn_throttle',Mock()),patch.object(module.rospy,'logwarn',Mock()),patch.object(module.rospy,'loginfo',Mock()):
        feedback(now[0]);qsource=tuple(exact[QUAL][key]for key in('rgb','depth'));hold=node._prepare_locked_rgb_hold_reference(qsource,qrefined,camera);assert hold is not None
        node._locked_rgb_hold_reference=hold
        previous=next(x['message']for x in fixture['wire']if x['topic'].endswith('measurement_request')and x['message']['state']=='requalifying')
        confirmation=ObservationArrivalConfirmation(config,anchor,data['last'],plane,data['hist'],data['contour'],.5,appearance_reference=data['frozen'])
        qstatus=source_status[QUAL]['observation_measurement'];confirmation.last_appearance_evidence=deepcopy(qstatus['appearance_identity'])
        node._observation_measurement=dict(request=deepcopy(previous),anchor=anchor,anchor_fingerprint=_candidate_fingerprint(anchor),last_fingerprint=_candidate_fingerprint(qraw),plane=plane.copy(),source_id=hold['source_id'],reference_stamp_ns=m.PRE,config_sha256=digest(asdict(config)),camera_configuration_sha256=digest(_camera_configuration_contract(camera)),deadline_monotonic=previous['capture_deadline_ns']*1e-9,session_deadline_monotonic=qualified['deadline_ns']*1e-9,confirmation=confirmation,arrival_q=np.array(qualified['settle']['accepted_positions_rad']),qualified_stamps=qualified['source_stamps_ns'],support_audit=qstatus['support_audit'])
        # The successful third-frame/three-frame state is a sealed seed. The
        # qualified callback itself is the unmodified candidate implementation.
        node._observation_measurement_request_cb(String(data=json.dumps(qualified)))
        ack=json.loads(node._observation_measurement_ack_pub.publish.call_args[0][0].data);assert ack['accepted'],ack
        marker=hold['observation_arrival_retry'];station=marker['station_appearance'];assert station.source_ns==QUAL;template_sha=station.mask_sha256;assert template_sha=='2bd3be2be89c32d3d5d7789d69580c3de6218cf4c343059c1638ca804ac8fdcd'
        phase_ns=data['early_ns'];now[0]=phase_ns;phase=NearFieldPlanningPhase(active=True,phase_id=1,planning_stage='contact_replan',reference_target_track_id=qualified['target_track_id']);phase.header.stamp=module.rospy.Time(phase_ns//10**9,phase_ns%10**9);phase.reference_source_stamp=module.rospy.Time(QUAL//10**9,QUAL%10**9);phase.deadline=module.rospy.Time((phase_ns+90000000000)//10**9,(phase_ns+90000000000)%10**9);node._station_compute_phase_cb(phase);node._station_compute_state_cb(GraspState(state='PLAN_PREGRASP',active=True))
        baseline_stage=node._frame_timing_stage
        for ns in SOURCES:
            recorded=source_status[ns]['frame_timing'];original_entry=recorded['processing_start_ros_ns'];start_ns[0]=max(now[0],original_entry);padding_ns[0]=0;perf_start[0]=time.perf_counter_ns();running[0]=True;stage_offsets=[]
            # A stage begins when the immediately previous recorded stage
            # ended. The ordering below is selected by original elapsed time.
            prior_order=sorted(recorded['stage_end_elapsed_sec'],key=lambda s:recorded['stage_end_elapsed_sec'][s]);floors={prior_order[i+1]:recorded['stage_end_elapsed_sec'][stage]for i,stage in enumerate(prior_order[:-1])}
            def stage(stage):
                if stage in floors:
                    target=original_entry+round(floors[stage]*1e9);before=clock_ns()
                    if target>before:padding_ns[0]+=target-before;stage_offsets.append(dict(stage=stage,added_recorded_stage_floor_ns=target-before))
                baseline_stage(stage)
            node._frame_timing_stage=stage;node.latest=tuple(exact[ns][key]for key in('rgb','depth'));node.pubs={key:Mock()for key in node.pubs};node.last_receive_wall=clock_ns()*1e-9;node.process(None)
            end=clock_ns();wall=(time.perf_counter_ns()-perf_start[0])*1e-9;running[0]=False;now[0]=end
            status=json.loads(node.pubs['detector_status'].publish.call_args[0][0].data);published=node.pubs['object'].publish.call_args[0][0];current_ref=node._locked_rgb_hold_reference;current_marker=(current_ref or{}).get('observation_arrival_retry');current_station=(current_marker or{}).get('station_appearance')
            row=dict(source_ns=ns,original_processing_entry_ns=original_entry,replay_processing_entry_ns=start_ns[0],replay_completion_ns=end,measured_wall_sec=wall,recorded_stage_floor_added_sec=padding_ns[0]*1e-9,source_age_at_completion_sec=(end-ns)*1e-9,stage_floors=stage_offsets,state=status['state'],detail=status['detail'],detected=published.detected,published_source_ns=published.header.stamp.to_nsec(),last_good_ns=node.last_good_source_stamp_ns,template_source_ns=None if current_station is None else current_station.source_ns,template_mask_sha256=None if current_station is None else current_station.mask_sha256,template_object_unchanged=current_station is station,marker_object_unchanged=current_marker is marker,registered=getattr(node,'_station_compute_registration',None)is not None,first_loss=node.first_loss,frame_timing=status.get('frame_timing'))
            rows.append(row);print(json.dumps({key:value for key,value in row.items()if key!='frame_timing'}),flush=True)
            if status['state']!='ready':break
            assert published.detected and published.header.stamp.to_nsec()==ns and node.last_good_source_stamp_ns==ns
            assert current_station is station and current_marker is marker and current_station.source_ns==QUAL and current_station.mask_sha256==template_sha
            assert 0<=end-ns<800000000
    after={str(path):sha(path)for path in paths};unchanged=after==hashes
    report=dict(passed=unchanged and len(rows)==len(SOURCES)and all(x['state']=='ready'for x in rows),scope='candidate actual qualified callback then full process; sealed qualified state seed; original pixels/source/TF/SDK plus measured offline runtime and conservative recorded stage floors',rows=rows,source_sha256=hashes,source_unchanged=unchanged,qualified_callback_ack=ack,qualified_source_ns=QUAL,template_mask_sha256=template_sha,phase_schedule_counterfactual=True,phase_ns=phase_ns,processing_times_are_counterfactual_replay_not_historical=True,all_added_post_refine_publish_time_counted=True,source_restamped=False,old_metric_geometry_reused=False,motion_authorized=False,limitations=['The first two qualification images were not preserved; their successful three-frame receipt is explicitly seeded, not re-enacted or invented.','The qualified callback and all subsequent full process calculations are real candidate code; publishers and parameter/TF/SDK delivery are offline transport.','Each source timestamp stays original; serialized replay entry may be later than original when actual new-branch computation overlaps it.','Recorded shared-stage times are lower bounds and actual elapsed time always advances, including new commit/publish stages absent from original failed trace.'])
    (out/'result.json').write_text(json.dumps(m.plain(report),ensure_ascii=False,indent=2)+'\n');print('RESULT '+str(out/'result.json'));assert report['passed'],report['rows'][-1]
if __name__=='__main__':main()
