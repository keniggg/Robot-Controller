#!/usr/bin/env python3
"""Offline per-experiment fixed-board comparisons; no ROS node, I/O or fit deployment.

Raw RGB snapshots and quality/TF-only observations are reported separately.
No absolute board coordinates are compared across recording directories.
"""
import argparse
import bisect
from collections import Counter
import hashlib
import json
import math
import os
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/alicia_flexible_grasp_supervisor/src'))
import numpy as np
import rosbag
import rospy
import tf2_py
from tf.transformations import quaternion_matrix
from alicia_flexible_grasp.robot.observation_path_guard import SerialUrdfFk

NAMES=['Joint%d'%i for i in range(1,7)]
QUANTUM=2*math.pi/4096


def transform(m):
    q=m.transform.rotation;p=m.transform.translation
    a=quaternion_matrix([q.x,q.y,q.z,q.w]);a[:3,3]=[p.x,p.y,p.z]
    return a


def angle(a,b):
    return math.degrees(math.acos(float(np.clip((np.trace(a[:3,:3].T@b[:3,:3])-1)/2,-1,1))))


def stats(x):
    a=np.asarray(x,dtype=float)
    if not len(a):return None
    return dict(count=len(a),minimum=np.min(a,axis=0).tolist(),maximum=np.max(a,axis=0).tolist(),
        median=np.median(a,axis=0).tolist(),mean=np.mean(a,axis=0).tolist(),
        p05=np.percentile(a,5,axis=0).tolist(),p95=np.percentile(a,95,axis=0).tolist())


def words(q):return tuple(int(math.floor((v+math.pi)/QUANTUM+.5)) for v in q)


def compare(a,b):
    if not a or not b:return dict(pair_count=0,reason='one or both windows have no qualified samples')
    rows=[]
    for x in a:
        for y in b:
            d=(np.array(y['T_base_board_fk'])[:3,3]-np.array(x['T_base_board_fk'])[:3,3])*1000
            d_tf=(np.array(y['T_base_board_tf'])[:3,3]-np.array(x['T_base_board_tf'])[:3,3])*1000
            fa=angle(np.array(x['T_base_tool_fk']),np.array(y['T_base_tool_fk']))
            ca=angle(np.array(x['T_camera_board']),np.array(y['T_camera_board']))
            same=x['sdk_words']==y['sdk_words'] and x['epoch_ns']==y['epoch_ns']
            ax=np.asarray(x['T_base_tool_fk']);ay=np.asarray(y['T_base_tool_fk'])
            cx=np.asarray(x['T_camera_board']);cy=np.asarray(y['T_camera_board'])
            handeye=np.linalg.inv(ax)@np.asarray(x['T_base_board_fk'])@np.linalg.inv(cx)
            bc=cx@np.linalg.inv(cy);bf=np.linalg.inv(handeye)@np.linalg.inv(ax)@ay@handeye
            camera_translation=bc[:3,3]*1000;fk_translation=bf[:3,3]*1000
            rows.append(dict(delta=d,tf_delta=d_tf,norm=float(np.linalg.norm(d)),fk_angle=fa,
                camera_angle=ca,angle_difference=ca-fa,same_sdk=same,
                camera_translation=camera_translation,fk_translation=fk_translation,
                translation_difference=camera_translation-fk_translation,
                relative_rotation_mismatch=angle(bc,bf),
                accepted_delta=np.degrees(np.array(y['accepted_rad'])-np.array(x['accepted_rad']))))
    return dict(pair_count=len(rows),delta_base_board_xyz_mm=stats([r['delta'] for r in rows]),
        delta_base_board_tf_xyz_mm=stats([r['tf_delta'] for r in rows]),
        displacement_base_board_mm=stats([r['norm'] for r in rows]),
        relative_fk_angle_deg=stats([r['fk_angle'] for r in rows]),
        relative_camera_angle_deg=stats([r['camera_angle'] for r in rows]),
        camera_minus_fk_angle_deg=stats([r['angle_difference'] for r in rows]),
        accepted_delta_deg=stats([r['accepted_delta'] for r in rows]),
        relative_camera_translation_xyz_mm=stats([r['camera_translation'] for r in rows]),
        relative_fk_camera_translation_xyz_mm=stats([r['fk_translation'] for r in rows]),
        relative_camera_translation_norm_mm=stats([np.linalg.norm(r['camera_translation']) for r in rows]),
        relative_fk_camera_translation_norm_mm=stats([np.linalg.norm(r['fk_translation']) for r in rows]),
        camera_minus_fk_translation_xyz_mm=stats([r['translation_difference'] for r in rows]),
        camera_minus_fk_translation_norm_mm=stats([np.linalg.norm(r['translation_difference']) for r in rows]),
        relative_rotation_vector_mismatch_deg=stats([r['relative_rotation_mismatch'] for r in rows]),
        same_sdk_and_epoch_pairs=sum(r['same_sdk'] for r in rows),
        all_pairs_same_sdk_and_epoch=all(r['same_sdk'] for r in rows),
        interpretation='All cross-window pairs are descriptive, correlated observations; not independent confidence intervals. Rotation-angle equality is invariant to a fixed hand-eye transform.')


def analyze(directory,output,historical=False):
    events=[json.loads(s) for s in (directory/'events.jsonl').read_text().splitlines() if s.strip()]
    plan=json.loads((directory/'plan.json').read_text())
    base=json.loads((directory/'baseline_geometry.json').read_text())
    urdf=(directory/'robot_description.urdf').read_text();fk=SerialUrdfFk(urdf,NAMES)
    quality={};board={};rgb={};accepted={};sdk={};epochs=[];counts=Counter();tfkeys=set()
    buffer=tf2_py.BufferCore(rospy.Duration(600))
    topics=['/charuco/quality','/tf','/tf_static','/bounded_response/snapshot_rgb',
        '/supervisor/camera/color/image_raw','/bounded_wrist/paired_rgb','/bounded_joint23/paired_rgb','/alicia_d/accepted_joint_states',
        '/alicia_d/sdk_command','/alicia_d/control_reference_epoch']
    with rosbag.Bag(str(directory/'evidence.bag')) as bag:
        end_ns=int(bag.get_end_time()*1e9)
        for topic,msg,t in bag.read_messages(topics=topics):
            received=t.to_nsec();counts[topic]+=1
            if topic=='/charuco/quality':
                q=json.loads(msg.data);ns=q.get('source_stamp_ns')
                if isinstance(ns,int):quality[ns]=(q,received)
            elif topic in ('/tf','/tf_static'):
                for m in msg.transforms:
                    key=(m.header.frame_id,m.child_frame_id,m.header.stamp.to_nsec())
                    if key in tfkeys:continue
                    tfkeys.add(key)
                    if topic=='/tf_static':buffer.set_transform_static(m,'offline_bag')
                    else:buffer.set_transform(m,'offline_bag')
                    if m.header.frame_id=='camera_link':board[(m.header.stamp.to_nsec(),m.child_frame_id)]=(transform(m),received)
            elif topic.endswith('rgb') or topic.endswith('/image_raw'):
                rgb[msg.header.stamp.to_nsec()]=received
            elif topic.endswith('control_reference_epoch'):epochs.append((received,msg.stamp.to_nsec()))
            else:
                d=dict(zip(msg.name,msg.position));q=[float(d[n]) for n in NAMES]
                (sdk if topic.endswith('sdk_command') else accepted)[msg.header.stamp.to_nsec()]=q
    ats=sorted(accepted);sts=sorted(sdk);epochs.sort();ets=[x[0] for x in epochs]
    cmds=[e for e in events if e['event']=='command']
    coupled=bool(plan.get('logical_legs'))
    outbound=(cmds[0] if coupled and cmds else next((e for e in cmds if e['label']=='outbound'),None))
    if outbound is None:raise ValueError('No executed outbound command')
    windows=[dict(name='baseline',start_ns=plan['stationary']['stationary_window_start_ns'],
        end_ns=outbound['command_stamp_ns'])]
    for e in events:
        intermediate=(coupled and e['event']=='one_hot_sdk_confirmed_and_stationary' and e.get('edited_joint')==2)
        if e['event']!='settled' and not intermediate:continue
        following=[c['command_stamp_ns'] for c in cmds if c['command_stamp_ns']>e['ros_stamp_ns']]
        if intermediate and not following:continue
        label=(e['label']+'_joint2_only' if intermediate else
               e['label']+'_stationary' if coupled else
               'outbound_stationary' if e['label']=='outbound' else 'return_stationary')
        windows.append(dict(name=label,start_ns=e['stationary']['stationary_window_start_ns'],
            event_settled_ns=e['ros_stamp_ns'],end_ns=min(following) if following else end_ns))
    rejected=Counter();rows=[]
    expected=base['quality'];x_expected=np.asarray(base['T_tool_camera'])
    for ns,(q,received) in sorted(quality.items()):
        window=next((w for w in windows if w['start_ns']<=ns<w['end_ns']),None)
        if window is None:continue
        try:
            if not (q.get('pose_ok') is True and q.get('charuco_count',0)>=35 and q.get('edge_clearance_px',-1)>=20
                    and isinstance(q.get('reprojection_rms_px'),(int,float)) and 0<=q['reprojection_rms_px']<=1.5
                    and not q.get('camera_model_change_latched') and not q.get('camera_model_error')
                    and q.get('camera_model_fingerprint')==expected['camera_model_fingerprint']
                    and q.get('board_frame')==expected['board_frame'] and q.get('intrinsics_source')=='realsense_runtime'
                    and q.get('board_size')==[12,9] and q.get('dictionary_id')=='DICT_5X5_100'
                    and abs(q.get('square_length_m',0)-.015)<1e-10 and abs(q.get('marker_length_m',0)-.01125)<1e-10):
                raise ValueError('quality_or_model_contract')
            match=board.get((ns,q['board_frame']))
            if match is None:raise ValueError('missing_exact_quality_tf')
            b,tf_received=match
            ages=[(received-ns)*1e-6,(tf_received-ns)*1e-6]
            if min(ages)<0 or max(ages)>350:raise ValueError('source_stale_or_future')
            ai=bisect.bisect_left(ats,ns);near=min(ats[max(0,ai-1):ai+1],key=lambda t:abs(t-ns))
            if abs(near-ns)>100_000_000:raise ValueError('encoder_separation_over_100ms')
            lo=bisect.bisect_right(ats,ns-400_000_000)-1;hi=bisect.bisect_left(ats,ns+400_000_000)
            if lo<0 or hi>=len(ats):raise ValueError('full_centered_stationary_window_missing')
            ts=ats[lo:hi+1];qq=np.asarray([accepted[t] for t in ts])
            if len(ts)<6 or max(np.diff(ts))>250_000_000 or np.max(np.ptp(qq,axis=0))>math.radians(.15):
                raise ValueError('accepted_not_independently_stationary')
            si=bisect.bisect_right(sts,ns)-1
            if si<0 or ns-sts[si]>300_000_000:raise ValueError('fresh_sdk_source_missing')
            ei=bisect.bisect_right(ets,received)-1
            if ei<0 or epochs[ei][1]!=plan['frozen_epoch_ns']:raise ValueError('epoch_mismatch')
            stamp=rospy.Time(ns//1_000_000_000,ns%1_000_000_000)
            a_tf=transform(buffer.lookup_transform_core('base_link','tool0',stamp))
            x=transform(buffer.lookup_transform_core('tool0','camera_link',stamp))
            if not np.allclose(x,x_expected,rtol=0,atol=1e-10):raise ValueError('handeye_changed')
            a=fk(accepted[near]);match_mm=float(np.linalg.norm(a[:3,3]-a_tf[:3,3])*1000)
            if match_mm>2.:raise ValueError('TF_accepted_FK_mismatch_over_2mm')
            raw_ns=rgb.get(ns);raw_age=(raw_ns-ns)*1e-6 if raw_ns is not None else None
            rows.append(dict(window=window['name'],source_stamp_ns=ns,quality_received_age_ms=ages[0],
                tf_received_age_ms=ages[1],raw_rgb_present=raw_ns is not None,raw_rgb_received_age_ms=raw_age,
                exact_fresh_rgb_quality_tf=raw_ns is not None and 0<=raw_age<=350,
                charuco_count=q['charuco_count'],reprojection_rms_px=q['reprojection_rms_px'],edge_clearance_px=q['edge_clearance_px'],
                accepted_stamp_ns=near,accepted_source_separation_ms=(near-ns)*1e-6,accepted_rad=accepted[near],
                stationary_count=len(ts),stationary_span_sec=(ts[-1]-ts[0])*1e-9,
                stationary_peak_to_peak_deg=np.degrees(np.ptp(qq,axis=0)).tolist(),
                sdk_source_age_ms=(ns-sts[si])*1e-6,sdk_words=list(words(sdk[sts[si]])),epoch_ns=epochs[ei][1],
                T_base_tool_fk=a.tolist(),T_camera_board=b.tolist(),T_base_board_fk=(a@x@b).tolist(),
                T_base_board_tf=(a_tf@x@b).tolist(),TF_accepted_FK_translation_difference_mm=match_mm,
                TF_accepted_FK_orientation_difference_deg=angle(a,a_tf)))
        except (ValueError,tf2_py.TransformException) as exc:rejected[str(exc)]+=1
    populations={}
    for population,subset in [('quality_tf',rows),('exact_rgb_quality_tf',[r for r in rows if r['exact_fresh_rgb_quality_tf']])]:
        summaries={};bywindow={}
        for w in windows:
            rr=[r for r in subset if r['window']==w['name']];bywindow[w['name']]=rr
            summaries[w['name']]=dict(window=w,count=len(rr),raw_rgb_exact_count=sum(r['exact_fresh_rgb_quality_tf'] for r in rr),
                quality_age_ms=stats([r['quality_received_age_ms'] for r in rr]),TF_age_ms=stats([r['tf_received_age_ms'] for r in rr]),
                accepted_association_ms=stats([r['accepted_source_separation_ms'] for r in rr]),
                base_board_xyz_mm=stats([np.array(r['T_base_board_fk'])[:3,3]*1000 for r in rr]),
                accepted_rad=stats([r['accepted_rad'] for r in rr]),unique_sdk_words=sorted(set(tuple(r['sdk_words']) for r in rr)))
        comparisons={}
        comparison_windows=([(w['name']+'_minus_baseline','baseline',w['name']) for w in windows if w['name']!='baseline'] + [(windows[i]['name']+'_minus_'+windows[i-1]['name'],windows[i-1]['name'],windows[i]['name']) for i in range(2,len(windows))] if coupled else [('outbound_minus_baseline','baseline','outbound_stationary'),('return_minus_baseline','baseline','return_stationary'),('return_minus_outbound','outbound_stationary','return_stationary')])
        for name,a,b in comparison_windows:
            comparisons[name]=compare(bywindow.get(a,[]),bywindow.get(b,[]))
        populations[population]=dict(windows=summaries,comparisons=comparisons)
    hold_windows=[]
    for w in windows:
        if 'event_settled_ns' not in w:continue
        start=w['event_settled_ns'];end=min(start+3_000_000_000,w['end_ns'])
        aa=[(t,accepted[t]) for t in ats if start<=t<=end];ss=[(t,sdk[t]) for t in sts if start<=t<=end]
        if not aa:continue
        qa=np.asarray([q for t,q in aa]);delta=np.diff(qa,axis=0)
        # Count genuine nonzero sign reversals per axis, ignoring repeated quantized samples.
        rev=[]
        for j in range(6):
            signs=np.sign(delta[:,j][abs(delta[:,j])>QUANTUM*.25]);rev.append(int(np.count_nonzero(np.diff(signs))) if len(signs)>1 else 0)
        hold_windows.append(dict(name=w['name'],start_ns=start,end_ns=end,accepted_samples=len(aa),sdk_samples=len(ss),
            sdk_words_constant=len(set(words(q) for t,q in ss))==1 if ss else False,
            accepted_net_delta_deg=np.degrees(qa[-1]-qa[0]).tolist(),accepted_peak_to_peak_deg=np.degrees(np.ptp(qa,axis=0)).tolist(),
            accepted_nonzero_direction_reversals=rev))
    result=dict(schema=1,kind='offline_within_session_fixed_board_multiframe',historical_logic_validation_only=historical,
        directory=str(directory),output=str(output),source_model_sha256=hashlib.sha256(urdf.encode()).hexdigest(),
        camera_model_fingerprint=expected['camera_model_fingerprint'],joint=plan.get('joint',plan.get('edited_joints',[None])[0]),edited_joints=plan.get('edited_joints',[plan.get('joint')]),counts=dict(counts),
        rejected_counts=dict(rejected),qualified_quality_tf_samples=len(rows),populations=populations,
        fixed_sdk_post_settle_3sec=hold_windows,
        limitations=['No comparison to an earlier board position.','Quality/TF-only rows have no exact archived raw RGB; these are separated from exact RGB triplets.',
            'FK and base TF are the robot model, not independent physical pose metrology.','Angle invariance diagnoses fixed-handeye inconsistency only when the board remains fixed; it does not isolate camera/PnP versus joint mechanics.',
            'No hand-eye refit, parameter write or motion command.'])
    output.mkdir(parents=True,exist_ok=True)
    (output/'samples.json').write_text(json.dumps(rows,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    (output/'report.json').write_text(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    (output/'analysis_source.py').write_bytes(Path(__file__).read_bytes())
    print(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False))


if __name__=='__main__':
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('directory',type=Path);ap.add_argument('--output',required=True,type=Path)
    ap.add_argument('--historical-validation',action='store_true');args=ap.parse_args()
    mount=Path('/media/zhuyupei/SanStand')
    if not os.path.ismount(str(mount)) or mount not in args.output.resolve().parents:raise RuntimeError('SanStand output required; no fallback')
    analyze(args.directory,args.output,args.historical_validation)
