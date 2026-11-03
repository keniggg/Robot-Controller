#!/usr/bin/env python3
"""Actual producer, sealed available RGBD, measured completion-clock counterfactual."""
from pathlib import Path
from copy import deepcopy
from types import SimpleNamespace as NS
from unittest.mock import Mock
import argparse
import importlib.util
import json
import time
import hashlib
import numpy as np
import cv2
import rosbag
import rospy
import tf2_py
from _pytest.monkeypatch import MonkeyPatch
from std_msgs.msg import Header,String
import test_unknown_stream as fixture
from alicia_grasp_modes import tabletop as table
from alicia_grasp_modes.rgb_instance import refine_candidate
from alicia_flexible_grasp.vision.rgbd_rectification import rectification_rgb_valid_mask
from tf.transformations import quaternion_matrix

ROOT=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
SESSION=ROOT/'ros_log/full_latest_ros_20261009T003407Z'
RECORD=ROOT/'grasp_records/20261009T034234.183696Z_9e74e994513542879c9e556c6ccd2180'
OUT=SESSION/'gen11_arrival_analysis'

def stamp(ns):return rospy.Time(ns//10**9,ns%10**9)
def matrix(msg):
    t=msg.transform;q=t.rotation;m=quaternion_matrix([q.x,q.y,q.z,q.w])
    m[:3,3]=[t.translation.x,t.translation.y,t.translation.z];return m

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--baseline',action='store_true');args=parser.parse_args()
    if args.baseline:
        path=SESSION/'verification/arrival_confirmation_freshness/before_unknown_tabletop_perception.py'
        spec=importlib.util.spec_from_file_location('sealed_unknown_baseline',path)
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module);fixture.module=module
    else:module=fixture.module
    label='baseline' if args.baseline else 'fixed'
    cv2.setNumThreads(1)
    assert json.loads((RECORD/'manifest.json').read_text())['recorder_closed'] is True
    source_paths=[Path(module.__file__),Path('src/alicia_grasp_modes/src/alicia_grasp_modes/rgb_instance.py'),
        Path('src/alicia_grasp_modes/src/alicia_grasp_modes/observation_arrival_confirmation.py')]
    hashes={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in source_paths}
    summary=json.loads((OUT/'analysis.json').read_text())
    requests={r['message']['state']:r['message'] for r in summary['requests']}
    first_status=[r for r in summary['arrival_statuses'] if summary['arrival_ns']<=r['received_ns']<=summary['capture_deadline_ns']][:3]
    completion_ns=[r['received_ns'] for r in first_status]
    online_sources=[r['message']['source_stamp_ns'] for r in first_status]
    images={};masks={};static=[];dynamic=[];cameras=[];feedback=[];statuses=[]
    with rosbag.Bag(str(RECORD/'keyframes.bag')) as bag:
        for topic,msg,received in bag.read_messages(topics=['/tf','/tf_static','/grasp_record/camera_runtime_metadata',
            '/alicia_d/accepted_joint_states','/perception/unknown/detector_status',
            '/supervisor/camera/color/image_raw','/supervisor/camera/depth/image_raw',
            '/perception/object_mask','/experiment/observation_rgbd/color','/experiment/observation_rgbd/depth']):
            if topic=='/tf':dynamic.extend(msg.transforms)
            elif topic=='/tf_static':static.extend(msg.transforms)
            elif topic.endswith('camera_runtime_metadata'):
                d=json.loads(msg.data)
                if d.get('parameters'):cameras.append(d['parameters'])
            elif topic.endswith('accepted_joint_states'):
                if msg.header.frame_id=='sdk_measured':feedback.append(msg)
            elif topic.endswith('detector_status'):
                d=json.loads(msg.data)
                if d.get('generation')==11:statuses.append((received.to_nsec(),d))
            else:
                ns=msg.header.stamp.to_nsec()
                if topic.endswith('object_mask'):masks[ns]=msg
                elif ns<requests['executing']['stamp_ns'] or summary['arrival_ns']<ns<completion_ns[-1]:
                    images.setdefault(ns,{})['rgb' if '/color/' in topic or topic.endswith('/color') else 'depth']=msg
    acquired=next(d for _,d in statuses if d.get('detail')=='acquired')
    initial=acquired['anchor_source_stamp_ns']
    last_ns=max(d['source_stamp_ns'] for recv,d in statuses if recv<requests['executing']['stamp_ns']
        and d['state']=='ready' and d['source_stamp_ns'] in masks and set(images.get(d['source_stamp_ns'],{}))=={'rgb','depth'})
    available=sorted(ns for ns,value in images.items() if summary['arrival_ns']<ns and set(value)=={'rgb','depth'})
    sources=available[:3]
    assert len(sources)==3 and all(ns<now for ns,now in zip(sources,completion_ns))
    assert all(0.<(now-ns)*1e-9<.8 for ns,now in zip(sources,completion_ns))
    assert (completion_ns[-1]-sources[0])*1e-9>.8
    buffer=tf2_py.BufferCore(rospy.Duration(600.))
    for t in static:buffer.set_transform_static(t,'sealed_completion_replay')
    for t in sorted(dynamic,key=lambda t:t.header.stamp.to_nsec()):buffer.set_transform(t,'sealed_completion_replay')
    monkeypatch=MonkeyPatch()
    with monkeypatch.context() as mp:
        n,_,_,_,params=fixture.rgb_interest_node(mp)
        now=[requests['executing']['stamp_ns']]
        mp.setattr(module.rospy.Time,'now',staticmethod(lambda:stamp(now[0])))
        mp.setattr(module.time,'monotonic',lambda:now[0]*1e-9)
        mp.setattr(module,'transform_matrix',matrix)
        n.buffer=NS(lookup_transform=lambda parent,child,when,*a:buffer.lookup_transform_core(parent,child,when))
        selection=requests['executing']['selection'];params['/grasp_mode/selection']=deepcopy(selection)
        camera=cameras[0];keys=('fx','fy','cx','cy','width','height','depth_scale')
        assert all(all(c[k]==camera[k] for k in keys) for c in cameras)
        params['/camera']=camera
        params['/robot_description']=Path('.runtime/candidate_models/expanded_metric_20260929/alicia_expanded_metric_candidate.urdf').read_text()
        n.generation=11;n.selection_ns=selection['stamp_ns'];n.last_shape=(camera['height'],camera['width'])
        n._rgb_interest_epoch_cb(Header(stamp=stamp(requests['executing']['control_epoch_ns']),frame_id='sdk_reference_epoch'))
        n._observation_measurement_ack_pub=Mock();k=[camera[v] for v in ('fx','fy','cx','cy')]
        def pair(ns):return images[ns]['rgb'],images[ns]['depth']
        def decoded(ns):
            rgb=n.bridge.imgmsg_to_cv2(images[ns]['rgb'],'bgr8');raw=n.bridge.imgmsg_to_cv2(images[ns]['depth'],'passthrough')
            depth=np.asarray(raw,dtype=np.float32)*camera['depth_scale']
            T=matrix(buffer.lookup_transform_core('base_link','camera_link',stamp(ns)))
            valid=rectification_rgb_valid_mask(camera,depth.shape)
            return rgb,depth,T,({} if valid is None else {'rgb_valid':valid})
        rgb,depth,T,kw=decoded(initial);seg=table.segment_with_support_fallback(depth,k,T,n.config)
        anchor,reason=n.tracker.choose(seg,depth.shape);assert anchor is not None,reason
        assert np.allclose(anchor.position_base,acquired['anchor_position_base'],atol=1e-10,rtol=0)
        n.tracker.plane_base=np.asarray(acquired['reference_support_plane_base'])
        rgb,depth,T,kw=decoded(last_ns)
        seg=table.segment_with_support_fallback(depth,k,T,n.config,anchor,anchor,n.tracker.plane_base)
        raw,reason=n.tracker.choose(seg,depth.shape);assert raw is not None,reason
        refined=refine_candidate(rgb,depth,k,T,raw,n.config,**kw)
        assert np.array_equal(refined.mask,n.bridge.imgmsg_to_cv2(masks[last_ns],'mono8'))
        n.anchor_source_stamp_ns=initial;n.last_good_source_stamp_ns=last_ns
        def feedback_until(end):
            for message in feedback:
                ns=message.header.stamp.to_nsec()
                if feedback_until.last<ns<=end:n._rgb_interest_joint_cb(message)
            feedback_until.last=end
        feedback_until.last=last_ns-700_000_000;feedback_until(now[0])
        n._locked_rgb_hold_reference=n._prepare_locked_rgb_hold_reference(pair(last_ns),refined,camera)
        assert n._locked_rgb_hold_reference is not None
        def send(request):
            n._observation_measurement_request_cb(String(data=json.dumps(request)))
            ack=json.loads(n._observation_measurement_ack_pub.publish.call_args[0][0].data)
            assert ack['accepted'],ack
            return ack
        send(requests['executing'])
        now[0]=requests['requalifying']['stamp_ns']+1_000_000;feedback_until(now[0]);send(requests['requalifying'])
        output=[];original_anchor=n.tracker.anchor;original_plane=n.tracker.plane_base.copy()
        for index,(ns,completed) in enumerate(zip(sources,completion_ns)):
            now[0]=completed;feedback_until(now[0]);n.latest=pair(ns)
            count=n.pubs['object'].publish.call_count;began=time.perf_counter();n.process(None);wall=time.perf_counter()-began
            status=json.loads(n.pubs['detector_status'].publish.call_args[0][0].data)
            delta=n.pubs['object'].publish.call_count-count
            assert n.tracker.anchor is original_anchor and np.array_equal(n.tracker.plane_base,original_plane)
            assert not n.tracker.lost and n.first_loss is None
            output.append(dict(source_stamp_ns=ns,recorded_completion_receipt_ns=completed,
                current_source_age_sec=(completed-ns)*1e-9,oldest_source_age_sec=(completed-sources[0])*1e-9,
                offline_process_wall_sec=wall,new_object_publications=delta,state=status['state'],detail=status['detail'],
                observation_measurement=status.get('observation_measurement')))
            if index<2:assert delta==0 and status['state']=='pending',status
            elif args.baseline:
                assert delta==0 and status['observation_measurement']['reason']=='source_exceeds_existing_frame_limit',status
            else:
                assert delta==1 and status['state']=='ready',status
                ack=json.loads(n._observation_measurement_ack_pub.publish.call_args[0][0].data)
                assert ack['qualified'] and ack['source_stamps_ns']==sources,ack
                send(dict(requests['requalifying'],state='qualified',request_id='sealed-completion-receipt',
                          stamp_ns=now[0],source_stamps_ns=sources))
                assert n._observation_measurement is None
        assert hashes=={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in source_paths}
        result=dict(label=label,passed=True,record=str(RECORD),source_sha256=hashes,
            original_anchor_source_ns=initial,pre_motion_source_ns=last_ns,available_replay_sources=sources,
            first_online_sources=online_sources,online_raw_available=[ns in images for ns in online_sources],
            first_online_status_receipt_ns=completion_ns,frames=output,original_anchor_plane_unchanged=True,
            clock_provenance='Each offline process call holds now at the corresponding actual first-group detector-status bag receipt time. These are completion-style counterfactual clocks, not internal check timestamps.',
            limitations=['First two online source images were not saved at diagnostic 5 Hz. Replay uses the first three available post-arrival exact RGBD pairs from this same sealed record, not the exact online confirmation trio.',
                'Actual TF, joint feedback, source headers and camera values are preserved. No image is processed before its exposure time.',
                'Offline process wall time is measured separately; clock does not advance with offline CPU work. No 100ms source-age or live timing claim.',
                'Transport and parameter reads are mocked; this is not execution authority or physical success.'],
            ros_actions=False,physical_grasp_verified=False)
        with (OUT/('actual_producer_completion_clock_'+label+'.json')).open('x') as f:json.dump(result,f,indent=2,ensure_ascii=False)
        print(json.dumps(result,ensure_ascii=False),flush=True)

if __name__=='__main__':main()
