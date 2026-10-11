#!/usr/bin/env python3
"""Offline real-frame arrival -> receipt -> normal tracking; fake transport only."""
from pathlib import Path
from copy import deepcopy
from types import SimpleNamespace as NS
from unittest.mock import Mock
import json
import hashlib
import numpy as np
import cv2
import rosbag
import rospy
import tf2_py
from _pytest.monkeypatch import MonkeyPatch
from std_msgs.msg import Header, String
from test_unknown_stream import rgb_interest_node, module
from alicia_grasp_modes import tabletop as table
from alicia_grasp_modes.rgb_instance import refine_candidate
from alicia_flexible_grasp.vision.rgbd_rectification import rectification_rgb_valid_mask
from tf.transformations import quaternion_matrix

ROOT=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
SESSION=ROOT/'ros_log/full_latest_ros_20261009T003407Z'
RECORD=ROOT/'grasp_records/20261009T015130.939636Z_053c5f63c0a5405da2a53aa19e654e4c'
OUTPUT=SESSION/'verification/observation_arrival_rgb_search'

def stamp(ns):return rospy.Time(ns//10**9,ns%10**9)
def matrix(msg):
    t=msg.transform;q=t.rotation;m=quaternion_matrix([q.x,q.y,q.z,q.w])
    m[:3,3]=[t.translation.x,t.translation.y,t.translation.z];return m

def main():
    cv2.setNumThreads(1)
    previous=json.loads((SESSION/'gen6_arrival_rgb_boundary/production_replay_result.json').read_text())
    initial=previous['anchor_source_ns'];last_ns=previous['pre_motion_source_ns']
    sources=[r['source_stamp_ns'] for r in previous['selected_frames']]
    needed={initial,last_ns,*sources};images={};masks={};static=[];dynamic=[];cameras=[];requests={};feedback=[];acquired=None
    with rosbag.Bag(str(RECORD/'keyframes.bag')) as bag:
        for topic,msg,received in bag.read_messages(topics=['/tf','/tf_static','/grasp_record/camera_runtime_metadata','/grasp_mode/observe_return_measurement_request','/alicia_d/accepted_joint_states','/perception/unknown/detector_status','/supervisor/camera/color/image_raw','/supervisor/camera/depth/image_raw','/perception/object_mask','/experiment/observation_rgbd/color','/experiment/observation_rgbd/depth']):
            if topic=='/tf':dynamic.extend(msg.transforms)
            elif topic=='/tf_static':static.extend(msg.transforms)
            elif topic.endswith('camera_runtime_metadata'):
                d=json.loads(msg.data)
                if d.get('parameters'):cameras.append(d['parameters'])
            elif topic.endswith('measurement_request'):
                d=json.loads(msg.data);requests[d['state']]=d
            elif topic.endswith('accepted_joint_states'):
                if msg.header.frame_id=='sdk_measured':feedback.append(msg)
            elif topic.endswith('detector_status'):
                d=json.loads(msg.data)
                if d.get('generation')==6 and d.get('detail')=='acquired':acquired=d
            else:
                ns=msg.header.stamp.to_nsec()
                if ns not in needed:continue
                if topic.endswith('object_mask'):masks[ns]=msg
                else:images.setdefault(ns,{})['rgb' if '/color/' in topic or topic.endswith('/color') else 'depth']=msg
    buffer=tf2_py.BufferCore(rospy.Duration(600.))
    for t in static:buffer.set_transform_static(t,'sealed_station_replay')
    for t in sorted(dynamic,key=lambda t:t.header.stamp.to_nsec()):buffer.set_transform(t,'sealed_station_replay')
    monkeypatch=MonkeyPatch()
    with monkeypatch.context() as mp:
        n,clock,_,_,params=rgb_interest_node(mp)
        now=[requests['executing']['stamp_ns']]
        mp.setattr(module.rospy.Time,'now',staticmethod(lambda:stamp(now[0])))
        mp.setattr(module.time,'monotonic',lambda:now[0]*1e-9)
        mp.setattr(module,'transform_matrix',matrix)
        n.buffer=NS(lookup_transform=lambda parent,child,when,*args:buffer.lookup_transform_core(parent,child,when))
        selection=requests['executing']['selection'];params['/grasp_mode/selection']=deepcopy(selection)
        params['/camera']=cameras[0];params['/robot_description']=(Path.cwd()/'.runtime/candidate_models/expanded_metric_20260929/alicia_expanded_metric_candidate.urdf').read_text()
        n.generation=selection['generation'];n.selection_ns=selection['stamp_ns'];n.last_shape=(cameras[0]['height'],cameras[0]['width'])
        n._rgb_interest_epoch_cb(Header(stamp=stamp(requests['executing']['control_epoch_ns']),frame_id='sdk_reference_epoch'))
        n._observation_measurement_ack_pub=Mock();k=[cameras[0][v] for v in ('fx','fy','cx','cy')]
        def pair(ns):return images[ns]['rgb'],images[ns]['depth']
        def decoded(ns):
            rgb=n.bridge.imgmsg_to_cv2(images[ns]['rgb'],'bgr8');raw=n.bridge.imgmsg_to_cv2(images[ns]['depth'],'passthrough')
            depth=np.asarray(raw,dtype=np.float32)*cameras[0]['depth_scale'];T=matrix(buffer.lookup_transform_core('base_link','camera_link',stamp(ns)))
            valid=rectification_rgb_valid_mask(cameras[0],depth.shape)
            return rgb,depth,T,({} if valid is None else {'rgb_valid':valid})
        rgb,depth,T,kw=decoded(initial);seg=table.segment_with_support_fallback(depth,k,T,n.config)
        anchor,reason=n.tracker.choose(seg,depth.shape);assert anchor is not None,reason
        assert np.allclose(anchor.position_base,acquired['anchor_position_base'],atol=1e-10,rtol=0)
        n.tracker.plane_base=np.asarray(acquired['reference_support_plane_base'])
        rgb,depth,T,kw=decoded(last_ns);seg=table.segment_with_support_fallback(depth,k,T,n.config,anchor,anchor,n.tracker.plane_base)
        raw,reason=n.tracker.choose(seg,depth.shape);assert raw is not None,reason
        refined=refine_candidate(rgb,depth,k,T,raw,n.config,**kw)
        assert np.array_equal(refined.mask,n.bridge.imgmsg_to_cv2(masks[last_ns],'mono8'))
        n.anchor_source_stamp_ns=initial;n.last_good_source_stamp_ns=last_ns
        def feedback_until(t):
            # Preserve the recorded sequence including actual intermediate motion.
            for msg in feedback:
                ns=msg.header.stamp.to_nsec()
                if feedback_until.last<ns<=t:n._rgb_interest_joint_cb(msg)
            feedback_until.last=t
        feedback_until.last=last_ns-700_000_000;feedback_until(now[0])
        n._locked_rgb_hold_reference=n._prepare_locked_rgb_hold_reference(pair(last_ns),refined,cameras[0]);assert n._locked_rgb_hold_reference is not None
        def send(req):
            n._observation_measurement_request_cb(String(data=json.dumps(req)))
            result=json.loads(n._observation_measurement_ack_pub.publish.call_args[0][0].data)
            assert result['accepted'],result
            return result
        send(requests['executing'])
        now[0]=requests['requalifying']['stamp_ns']+1_000_000;feedback_until(now[0]);send(requests['requalifying'])
        output=[];terminal=None;fixed=None;fixed_values=None;original_anchor=n.tracker.anchor;original_plane=n.tracker.plane_base.copy()
        for index,ns in enumerate(sources):
            now[0]=ns+100_000_000;feedback_until(now[0]);n.latest=pair(ns)
            before=n.pubs['object'].publish.call_count;n.process(None)
            status=json.loads(n.pubs['detector_status'].publish.call_args[0][0].data)
            added=n.pubs['object'].publish.call_count-before
            assert n.tracker.anchor is original_anchor and not n.tracker.lost and n.first_loss is None,status
            assert np.array_equal(n.tracker.plane_base,original_plane)
            if index<2:assert added==0 and status['state']=='pending',status
            else:assert added==1 and status['state']=='ready' and n.last_good_source_stamp_ns==ns,status
            if index==2:
                ack=json.loads(n._observation_measurement_ack_pub.publish.call_args[0][0].data)
                assert ack['qualified'] and ack['source_stamps_ns']==sources[:3],ack
                terminal=dict(requests['requalifying'],state='qualified',request_id='sealed-offline-qualified',stamp_ns=now[0],source_stamps_ns=sources[:3])
                send(terminal);assert n._observation_measurement is None
            marker=(n._locked_rgb_hold_reference or {}).get('observation_arrival_retry')
            if index>=2:
                assert marker is not None, 'qualified same-station capability not retained'
                values=json.dumps(marker,sort_keys=True,default=lambda v:v.tolist() if isinstance(v,np.ndarray) else str(v))
                if fixed is None:fixed=marker;fixed_values=values
                assert marker is fixed and values==fixed_values,'fixed station evidence drifted'
            output.append(dict(source_stamp_ns=ns,phase='arrival_confirmation' if index<3 else 'normal_tracking',state=status['state'],detail=status['detail'],new_object_publications=added,hold_source_ns=None if n._locked_rgb_hold_reference is None else n._locked_rgb_hold_reference['source_stamp_ns'],station_marker_present=marker is not None))
        result=dict(status='passed',record=str(RECORD),scope='actual unknown producer callbacks and process with sealed RGB-D/TF/accepted joints; offline transport and source-age clock',synthetic_now_offset_from_frame_sec=.1,arrival_confirmation_sources=sources[:3],subsequent_normal_frames=len(sources)-3,frames=output,original_anchor_and_plane_unchanged=True,fixed_station_marker_unchanged=True,first_two_frames_unpublished=True,qualified_receipt_processed=True,ros_actions=False,limitations=['Offline fake clock uses 100ms source age; this does not measure future live scheduling latency.','Archived source context supplies actual camera, TF and URDF; parameter transport is mocked and no new execution authority is granted.'])
        OUTPUT.mkdir(parents=True,exist_ok=True);(OUTPUT/'gen6_actual_station_process.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
        print(json.dumps(result,ensure_ascii=False),flush=True)

if __name__=='__main__':main()
