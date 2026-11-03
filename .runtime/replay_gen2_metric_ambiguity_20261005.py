#!/usr/bin/env python3
"""Conditional production replay using sealed exact RGB-D/TF/joint evidence.

The original private last-good cache was not recorded. Its image/pose fields
are reconstructed from real records under an explicit fixed replay context.
This tests the new code on actual pixels; it does not prove original online
cache availability, unchanged unrecorded context, or physical grasp success.
ROS transport/time/parameter access are mocked; no master or devices are used.
"""
from dataclasses import replace
import hashlib
import json
from pathlib import Path
import sys
import time

import numpy as np
from _pytest.monkeypatch import MonkeyPatch

WORK=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(WORK/'src/alicia_grasp_modes/tests'),str(WORK/'src/alicia_grasp_modes/src'),
             str(WORK/'src/alicia_flexible_grasp_supervisor/src')]
from test_unknown_stream import module,rgb_interest_node
from alicia_grasp_modes import tabletop
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

ROOT=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/grasp_records/'
          '20261005T013130.144638Z_f95c4f0498e54f8a922171b982c2df69/first_ambiguity_analysis')


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def stamp(ns):return module.rospy.Time(ns//10**9,ns%10**9)


def main():
    assert Path('/media/zhuyupei/SanStand').is_mount()
    report=json.loads((ROOT/'report.json').read_text());good=report['first_loss']['last_good_source_stamp_ns']
    loss=report['first_loss']['source_stamp_ns'];records={r['source_stamp_ns']:r for r in report['source_frames']}
    manifest={r['path']:r['sha256'] for r in report['files']}
    for name,digest in manifest.items():assert sha(ROOT/name)==digest
    keys=[good]+[ns for ns in sorted(records) if ns>=loss][:3]
    arrays={ns:dict(np.load(ROOT/(str(ns)+'.npz'))) for ns in keys}
    epoch=json.loads((ROOT/'no_motion_recording_receipt.json').read_text())['epochs'][0]['stamp_ns']
    patch=MonkeyPatch()
    n,clock,_,_,parameters=rgb_interest_node(patch)
    n.generation=report['first_loss']['generation'];n.selection_ns=report['first_loss']['selection_stamp_ns']
    parameters['/grasp_mode/selection']=dict(mode='unknown',strategy='observe_return',
        generation=n.generation,stamp_ns=n.selection_ns)
    profile=report['camera_metadata'];intrinsics=profile['geometry_intrinsics']
    camera=dict(parameters['/camera'],runtime_profile=profile,
        width=intrinsics['width'],height=intrinsics['height'],
        depth_scale=profile['depth_scale_m_per_unit'],
        **{k:intrinsics[k] for k in ('fx','fy','cx','cy')})
    parameters['/camera']=camera;n.last_shape=(camera['height'],camera['width'])
    n._rgb_interest_epoch_cb(Header(stamp=stamp(epoch),frame_id='sdk_reference_epoch'))
    phase=[records[good]['status']['frame_timing']['status_build_ros_ns'],time.perf_counter()]
    def now():return stamp(phase[0]+int((time.perf_counter()-phase[1])*1e9))
    patch.setattr(module.rospy.Time,'now',staticmethod(now))
    patch.setattr(module,'transform_matrix',lambda args:arrays[args[2].to_nsec()]['T_base_camera'].copy()
        if args[0]=='base_link' else np.eye(4))
    def feedback(until_ns):
        for row in report['accepted_feedback']:
            if row['stamp_ns']<=until_ns:
                msg=JointState();msg.header.stamp=stamp(row['stamp_ns']);msg.header.frame_id='sdk_measured'
                msg.name,msg.position=row['names'],row['positions'];n._rgb_interest_joint_cb(msg)
    feedback(phase[0])
    base=np.array(report['first_loss']['reference_support_plane_base'])
    recorded=report['first_loss_status'];data=arrays[good];K=data['K'];T=data['T_base_camera']
    anchor=tabletop.Candidate(data['published_mask'],np.array([1.,1.]),(1,1,2,2),np.array([.1,0,0]),
        np.array(recorded['anchor_position_base']),np.array(recorded['anchor_extent_m']),.1,.8,{},
        np.array(recorded['reference_support_footprint_base']))
    last=replace(anchor,position_base=np.array(report['first_loss']['candidate_evidence']['last_position_base']))
    segmentation=tabletop.segment_with_support_fallback(data['depth_raw'].astype(np.float32)*camera['depth_scale'],
        K,T,n.config,anchor,last,base)
    matches=[c for c in segmentation.candidates if np.max(np.abs(c.position_base-last.position_base))<1e-9]
    assert len(matches)==1
    n.tracker.anchor,n.tracker.last,n.tracker.plane_base=anchor,matches[0],base
    n.anchor_source_stamp_ns=recorded['anchor_source_stamp_ns'];n.last_good_source_stamp_ns=good
    def source(ns):
        d=arrays[ns];pair=(n.bridge.cv2_to_imgmsg(d['rgb'],'bgr8'),n.bridge.cv2_to_imgmsg(d['depth_raw'],'16UC1'))
        for msg in pair:msg.header.stamp=stamp(ns);msg.header.frame_id='camera_link'
        return pair
    # Cache structure follows production; original initial anchor mask and
    # unrecorded source-context fields are explicitly modeled, never asserted real.
    measured=replace(matches[0],mask=data['published_mask'])
    n._locked_rgb_hold_reference=n._prepare_locked_rgb_hold_reference(source(good),measured,camera)
    assert n._locked_rgb_hold_reference is not None
    old_reference=n._locked_rgb_hold_reference;old_last=n.tracker.last
    output=dict(scope=__doc__,source_report_sha256=sha(ROOT/'report.json'),
        script_sha256=sha(__file__),private_reference_reconstructed=True,
        original_live_cache_eligibility_proven=False,source_context_scope='Fixed test context; recorded camera and selection/epoch, modeled URDF/handeye parameters and tool-camera lookup.',
        source_implementation_sha256={str(Path(p).resolve()):sha(p) for p in
            [module.__file__,WORK/'src/alicia_grasp_modes/src/alicia_grasp_modes/metric_ambiguity_identity.py']},rows=[])
    for ns in keys[1:]:
        phase[:]=[records[ns]['status']['frame_timing']['status_build_ros_ns'],time.perf_counter()]
        clock[0]=phase[0]*1e-9;feedback(phase[0]);n.latest=source(ns)
        n.process(None)
        status=json.loads(n.pubs['detector_status'].publish.call_args[0][0].data)
        detected=n.pubs['object'].publish.call_args[0][0].detected
        mask_pixels=int(np.count_nonzero(n.bridge.imgmsg_to_cv2(n.pubs['object_mask'].publish.call_args[0][0])))
        row=dict(source_stamp_ns=ns,status=status,detected=detected,published_mask_pixels=mask_pixels,
            live_last_unchanged=n.tracker.last is old_last,old_reference_retained=n._locked_rgb_hold_reference is old_reference,
            last_good_source_stamp_ns=n.last_good_source_stamp_ns,lost=n.tracker.lost,
            runtime_seconds=time.perf_counter()-phase[1])
        output['rows'].append(row)
        if ns==loss:
            assert not detected and mask_pixels==0 and not n.tracker.lost
            assert n.tracker.last is old_last and n._locked_rgb_hold_reference is old_reference
            assert n.last_good_source_stamp_ns==good
        else:
            assert detected and not n.tracker.lost
            assert n.last_good_source_stamp_ns==ns
    output['passed']=True
    destination=ROOT/'conditional_production_hold_replay.json'
    destination.write_text(json.dumps(output,ensure_ascii=False,indent=2)+'\n')
    patch.undo()
    print(json.dumps(dict(path=str(destination),sha256=sha(destination),passed=True,
        decisions=[(r['source_stamp_ns'],r['detected'],r['status']['detail']) for r in output['rows']])))


if __name__=='__main__':main()
