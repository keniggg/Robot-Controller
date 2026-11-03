#!/usr/bin/env python3
"""Sealed RGB-D/TF/SDK replay; never initializes ROS or connects to a master."""
import argparse
import hashlib
import importlib.util
import json
import time
from dataclasses import asdict, fields
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import cv2
import numpy as np
import rosbag
import rospy
import tf2_py
from cv_bridge import CvBridge
from tf.transformations import quaternion_matrix
from alicia_grasp_modes import tabletop as table
from alicia_grasp_modes.rgb_instance import refine_candidate, freeze_observation_spatial_reference
from alicia_grasp_modes.rgb_observation_handoff import _appearance, _mask
from alicia_grasp_modes.observation_appearance import FrozenObservationAppearance
from alicia_grasp_modes.arrival_fragment_group import recover_arrival_matching_fragments, recover_arrival_fragment_group
from alicia_grasp_modes.rgb_observation_source import require_fresh_source
from alicia_flexible_grasp.vision.rgbd_rectification import rectification_rgb_valid_mask

ROOT = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
RECORD = ROOT/'grasp_records/20261013T012841.933029Z_8537e8805ca34fa9be2d153cdbc37b25'
W = Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
GOOD = 1791854972104432344
BAD = 1791854972438854694
NEXT = 1791854972907250881
PRE = 1791854935437230348
FAIL = 1791854972904886245

def plain(value):
    if isinstance(value, np.ndarray): return value.tolist()
    if isinstance(value, np.generic): return value.item()
    if hasattr(value, 'to_nsec'): return value.to_nsec()
    if hasattr(value, '__slots__'): return {key: plain(getattr(value, key)) for key in value.__slots__}
    if isinstance(value, dict): return {key: plain(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)): return [plain(item) for item in value]
    return value

def sha(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024*1024), b''): digest.update(chunk)
    return digest.hexdigest()

def transform_matrix(msg):
    t=msg.transform; q=t.rotation
    result=quaternion_matrix([q.x,q.y,q.z,q.w]); result[:3,3]=[t.translation.x,t.translation.y,t.translation.z]
    return result

def capture(call):
    try: value=call(); return dict(passed=True, value=plain(value))
    except (ValueError, TypeError, KeyError, AttributeError) as exc: return dict(passed=False, reason=str(exc), exception=type(exc).__name__)

def main():
    ap=argparse.ArgumentParser(); ap.add_argument('--output',required=True); args=ap.parse_args()
    out=Path(args.output); out.mkdir(parents=True,exist_ok=False); cv2.setNumThreads(1)
    manifest=json.loads((RECORD/'manifest.json').read_text())
    assert manifest['recorder_closed'] is True and manifest['state']=='complete' and manifest['result']=='failure'
    bag_sha=sha(RECORD/'keyframes.bag')
    assert bag_sha==next(x['sha256'] for x in manifest['files'] if x['path']=='keyframes.bag')
    images={}; masks={}; statuses=[]; static=[]; dynamic=[]; cameras=[]; wire=[]; sdk=[]
    topics=['/tf','/tf_static','/perception/unknown/detector_status','/grasp_record/camera_runtime_metadata',
        '/supervisor/camera/color/image_raw','/supervisor/camera/depth/image_raw','/perception/object_mask',
        '/experiment/observation_rgbd/color','/experiment/observation_rgbd/depth',
        '/alicia_d/accepted_joint_states','/grasp/near_field_phase','/grasp/state',
        '/grasp_mode/observe_return_measurement_request','/grasp_mode/observe_return_measurement_ack']
    with rosbag.Bag(str(RECORD/'keyframes.bag')) as bag:
        for topic,msg,received in bag.read_messages(topics=topics):
            ns=received.to_nsec()
            if topic=='/tf': dynamic.extend(msg.transforms)
            elif topic=='/tf_static': static.extend(msg.transforms)
            elif topic.endswith('camera_runtime_metadata'):
                data=json.loads(msg.data)
                if data.get('parameters'): cameras.append((ns,data['parameters']))
            elif topic.endswith('detector_status'):
                data=json.loads(msg.data)
                if data.get('generation')==2: statuses.append((ns,data))
            elif topic.endswith('accepted_joint_states'):
                # Whitelisted measured feedback only, never temperature/actuation diagnostics.
                sdk.append(dict(received_ns=ns,stamp_ns=msg.header.stamp.to_nsec(),frame_id=msg.header.frame_id,name=list(msg.name),position=list(msg.position)))
            elif '/grasp/' in topic or '/grasp_mode/' in topic:
                data=json.loads(msg.data) if hasattr(msg,'data') else plain(msg)
                wire.append(dict(topic=topic,received_ns=ns,message=data))
            else:
                stamp=msg.header.stamp.to_nsec()
                if topic.endswith('object_mask'): masks[stamp]=msg
                else:
                    role='rgb' if '/color/' in topic or topic.endswith('/color') else 'depth'
                    pair=images.setdefault(stamp,{})
                    if role in pair:
                        assert pair[role].data==msg.data
                    pair[role]=msg
    acquired=next(row for _,row in statuses if row.get('detail')=='acquired')
    anchor_ns=acquired['anchor_source_stamp_ns']
    selected=[anchor_ns,PRE,1791854971572257757,GOOD,BAD,NEXT,1791854973041301727,1791854973238121509]
    assert all(set(images.get(ns,{}))=={'rgb','depth'} for ns in selected)
    camera=cameras[0][1]; camera_keys=('fx','fy','cx','cy','width','height','depth_scale')
    assert all(all(c[k]==camera[k] for k in camera_keys) for _,c in cameras)
    camera=json.loads(json.dumps(camera)); intrinsics=[camera[x] for x in ('fx','fy','cx','cy')]
    bridge=CvBridge(); buffer=tf2_py.BufferCore(rospy.Duration(600))
    for msg in static: buffer.set_transform_static(msg,'sealed_original_bag')
    for msg in sorted(dynamic,key=lambda x:x.header.stamp.to_nsec()): buffer.set_transform(msg,'sealed_original_bag')
    frames={}; input_rows=[]
    for ns in selected:
        pair=images[ns]; rgb=bridge.imgmsg_to_cv2(pair['rgb'],'bgr8'); raw=bridge.imgmsg_to_cv2(pair['depth'],'passthrough')
        depth=np.asarray(raw,dtype=np.float32)*camera['depth_scale'] if pair['depth'].encoding in ('16UC1','mono16') else np.asarray(raw,dtype=np.float32)
        tf=buffer.lookup_transform_core('base_link',pair['rgb'].header.frame_id,rospy.Time(ns//10**9,ns%10**9)); T=transform_matrix(tf)
        valid=rectification_rgb_valid_mask(camera,depth.shape)
        payload=dict(rgb=rgb,raw_depth=raw,depth=depth,T_base_camera=T)
        if ns in masks: payload['recorded_mask']=bridge.imgmsg_to_cv2(masks[ns],'mono8')
        if valid is not None: payload['rgb_valid']=valid
        path=out/f'{ns}_source.npz'; np.savez_compressed(path,**payload)
        frames[ns]=(rgb,depth,T,{} if valid is None else {'rgb_valid':valid})
        before=max((x for x in sdk if x['stamp_ns']<=ns),key=lambda x:x['stamp_ns'],default=None)
        after=min((x for x in sdk if x['stamp_ns']>=ns),key=lambda x:x['stamp_ns'],default=None)
        assert before and after
        input_rows.append(dict(source_stamp_ns=ns,path=path.name,sha256=sha(path),rgb_header=plain(pair['rgb'].header),depth_header=plain(pair['depth'].header),depth_encoding=pair['depth'].encoding,exact_tf=plain(tf),sdk_before=before,sdk_after=after))
    configuration=json.loads((RECORD/'perception_execution_configuration.json').read_text())
    known={field.name for field in fields(table.Config)}
    config=table.Config(**{key:value for key,value in configuration['parameters']['/unknown_perception'].items() if key in known})
    status_by_source={row['source_stamp_ns']:row for _,row in statuses if row.get('source_stamp_ns')}
    retained_statuses=[dict(received_ns=ns,message=row) for ns,row in statuses if row.get('source_stamp_ns') in selected or row.get('observation_measurement',{}).get('state') in ('requalified','qualified')]
    fixture=dict(record=str(RECORD),bag_sha256=bag_sha,scope='sealed original measured inputs; no ROS master, no execution authority',camera=camera,config=asdict(config),frames=input_rows,wire=wire,sdk=[x for x in sdk if 1791854969000000000<=x['stamp_ns']<=1791854974000000000],statuses=retained_statuses,available_pairs_after_arrival=[ns for ns,pair in sorted(images.items()) if ns>=1791854970360903024 and set(pair)=={'rgb','depth'}],source_sha256={str(path):sha(path) for path in [Path(__file__),Path(table.__file__),W/'src/alicia_grasp_modes/scripts/unknown_tabletop_perception.py']})
    (out/'fixture.json').write_text(json.dumps(plain(fixture),ensure_ascii=False,indent=2)+'\n')
    print('FIXTURE '+str(out/'fixture.json'),flush=True)

    rgb,depth,T,kw=frames[anchor_ns]
    seg=table.segment_with_support_fallback(depth,intrinsics,T,config)
    tracker=table.TargetTracker(config); anchor,reason=tracker.choose(seg,depth.shape)
    assert anchor is not None,reason
    assert np.allclose(anchor.position_base,acquired['anchor_position_base'],atol=1e-10,rtol=0)
    assert np.allclose(anchor.extent_m,acquired['anchor_extent_m'],atol=1e-10,rtol=0)
    plane=np.asarray(acquired['reference_support_plane_base'])
    assert np.allclose(seg.plane_base,plane,atol=1e-10,rtol=0)
    spatial=freeze_observation_spatial_reference(anchor,plane)
    # PRE is the actual immutable executing appearance. Its recorded mask is
    # transparently seeded; anchor and each new frame still use real geometry.
    rgb,depth,T,kw=frames[PRE]
    premask=bridge.imgmsg_to_cv2(masks[PRE],'mono8')
    hist,contour=_appearance(rgb,_mask(premask,rgb.shape[:2]))
    frozen=FrozenObservationAppearance(rgb,premask,kw.get('rgb_valid'),PRE,hist,contour)
    seg=table.segment_with_support_fallback(depth,intrinsics,T,config,anchor,anchor,plane)
    matches=table._matching_candidates(seg.candidates,anchor,anchor,config)
    assert len(matches)==1,len(matches)
    last=matches[0][1]
    pre_refined=refine_candidate(rgb,depth,intrinsics,T,last,config,**kw)
    pre_reconstruction_equal=bool(np.array_equal(pre_refined.mask,premask))
    rows=[]; good_raw=None; good_refined=None
    for ns in [GOOD,BAD,NEXT,1791854973041301727,1791854973238121509]:
        rgb,depth,T,kw=frames[ns]; started=time.perf_counter(); appearances=[]
        row=dict(source_stamp_ns=ns,scope='geometry diagnostic only; clock authority evaluated separately; no rejected frame commits',last_good_input_source_ns=GOOD if good_raw is not None else PRE)
        try:
            result=table.segment_with_support_fallback(depth,intrinsics,T,config,anchor,last,plane,prefer_current_support=True)
            matches=table._matching_candidates(result.candidates,anchor,last,config)
            row.update(candidate_count=len(result.candidates),original_metric_matches=len(matches),support_audit=result.metrics)
            # This pure geometry diagnostic has no authority. A 5 s wall budget
            # only bounds replay CPU; production clock gates are checked below.
            context=dict(reference=frozen,source_ns=ns,deadline_monotonic=time.monotonic()+5.,check_current=lambda:None,record=appearances.append)
            pair=None
            recovery=recover_arrival_matching_fragments if len(matches)>1 else recover_arrival_fragment_group if not matches else None
            if recovery:
                try:
                    result,pair=recovery(result,rgb,depth,intrinsics,T,config,anchor,last,spatial,hist,contour,.5,appearance_context=context,**kw)
                    row['fragment_accepted']=pair is not None
                except ValueError as exc: row.update(fragment_accepted=False,fragment_reason=str(exc))
            if pair is not None:
                raw,refined=pair
            elif len(matches)==1:
                raw=matches[0][1]
                refined=refine_candidate(rgb,depth,intrinsics,T,raw,config,observation_arrival_retry=True,observation_spatial_reference=spatial,**kw)
            else:
                raw=refined=None
            if refined is not None:
                row.update(refined_bbox=list(refined.bbox),refined_quality=refined.quality,refined_position_base=refined.position_base,refined_mask_sha256=hashlib.sha256(np.ascontiguousarray(refined.mask>0,dtype=np.uint8).tobytes()).hexdigest())
                if ns in masks: row['recorded_mask_bit_equal']=np.array_equal(refined.mask,bridge.imgmsg_to_cv2(masks[ns],'mono8'))
                if ns==GOOD:
                    assert row['recorded_mask_bit_equal'] is True,'GOOD must reproduce exactly before seeding next frame'
                    good_raw,good_refined,last=raw,refined,raw
            if len(matches)>1 and good_refined is not None:
                from alicia_grasp_modes.metric_ambiguity_identity import metric_seeded_identity_evidence
                row['metric_identity_against_actual_good']=capture(lambda:metric_seeded_identity_evidence(rgb,[x[1] for x in matches],frames[GOOD][0],good_refined.mask,minimum_quality=.5,rgb_valid=kw.get('rgb_valid'),refine=lambda c:refine_candidate(rgb,depth,intrinsics,T,c,config,observation_arrival_retry=True,**kw))[1])
        except Exception as exc:
            row.update(error=str(exc),exception=type(exc).__name__)
            if ns==GOOD: raise
        row['appearance_audits']=appearances; row['wall_sec']=time.perf_counter()-started; rows.append(row)
        print(json.dumps(plain({k:v for k,v in row.items() if k not in ('support_audit','appearance_audits')})),flush=True)

    # Invoke actual producer rejection entry, keeping original nanosecond clocks.
    script=W/'src/alicia_grasp_modes/scripts/unknown_tabletop_perception.py'
    spec=importlib.util.spec_from_file_location('sealed_unknown_phase_replay',script); module=importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    node=module.UnknownTabletopPerception.__new__(module.UnknownTabletopPerception)
    node.max_age=.8; node.generation=2; node.selection_ns=1791854923890747785; node._rgb_interest_epoch=1791854771428479964
    node.last_good_source_stamp_ns=GOOD; node.tracker=SimpleNamespace(lost=False,anchor=anchor,last=good_raw,plane_base=plane)
    ref=dict(source_stamp_ns=GOOD,generation=node.generation,selection_ns=node.selection_ns,epoch_ns=node._rgb_interest_epoch)
    node._locked_rgb_hold_reference=ref
    registered=[]
    for flag in (False,True):
        node._station_compute_registration={'reference':ref} if flag else None
        with patch.object(module.rospy.Time,'now',staticmethod(lambda:module.rospy.Time(FAIL//10**9,FAIL%10**9))):
            result=capture(node._check_metric_ambiguity_reference)
        assert result.get('reason')=='source_exceeds_existing_frame_limit',result
        registered.append(dict(registration_present=flag,original_method='_check_metric_ambiguity_reference',now_ns=FAIL,reference_source_ns=GOOD,current_source_ns=BAD,**result))
    clocks=[]
    for label,source,now in [('original_failure_reference',GOOD,FAIL),('original_failure_current',BAD,FAIL),('next_original_entry_reference',GOOD,1791854972921741008),('next_original_entry_current',NEXT,1791854972921741008),('reference_boundary',GOOD,GOOD+800000000),('reference_boundary_plus_1ns',GOOD,GOOD+800000001)]:
        clocks.append(dict(label=label,source_ns=source,now_ns=now,age_ns=now-source,**capture(lambda:require_fresh_source(source,now,.8))))
    report=dict(scope='independent original measured continuous-frame geometry plus original producer clock-entry replay; not a full successful task simulation',fixture=str(out/'fixture.json'),geometry_rows=rows,actual_clock_entry=registered,freshness=clocks,last_good_preserved_ns=GOOD,original_anchor_reproduced=True,pre_motion_mask_bit_equal=pre_reconstruction_equal,pre_motion_reference_seed='original recorded RGB and published mask; not a reconstructed-state claim',good_mask_bit_equal=True,limitations=['Only task phase scheduling may be counterfactual in downstream replay. Input source timestamps, SDK, TF and original processing clocks remain recorded.','Pixels between BAD2438 and NEXT2907 were not saved. No 2572/2705 replacement pixels or synthetic clocks may be claimed.','Appearance helper geometry diagnostic uses a bounded wall budget without runtime owner mutation; it provides no lifetime or motion authority.','Rejected frames never replace last_good in this replay. Subsequent frames are geometry diagnostics against actual GOOD, not falsely committed successful recovery.'])
    (out/'result.json').write_text(json.dumps(plain(report),ensure_ascii=False,indent=2)+'\n')
    print('RESULT '+str(out/'result.json'),flush=True)

if __name__=='__main__': main()
