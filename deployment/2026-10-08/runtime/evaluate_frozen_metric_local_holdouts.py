#!/usr/bin/env python3
"""Frozen training-only model on independent local sessions; no fitting or ROS node."""
import argparse,bisect,hashlib,json,math,os
from collections import defaultdict,Counter
from pathlib import Path
import numpy as np
import rosbag
from tf.transformations import quaternion_matrix
import audit_j1_board_axis as J
import evaluate_frozen_metric_j1 as E
G=J.G
MOUNT=Path('/media/zhuyupei/SanStand')
ARCH=MOUNT/'实验记录/Robot-Controller/ros_log'
RESUME=ARCH/'grasp_repair_resume_20260928T120606Z'
EPOCH=1790597312141115461


def partial_rows(session):
    template=json.loads((session/'fresh_return_baseline_geometry.json').read_text())
    events=[json.loads(x) for x in (session/'events.jsonl').read_text().splitlines()]
    command=next(e for e in events if e['event']=='command')
    stop=next(e for e in events if e['event']=='one_hot_sdk_confirmed_and_stationary')
    quality={};boards={};accepted={};paired=set();epochs=set();rejected=Counter()
    with rosbag.Bag(str(session/'evidence.bag')) as bag:
        for topic,msg,t in bag.read_messages(topics=['/charuco/quality','/tf','/alicia_d/accepted_joint_states','/alicia_d/control_reference_epoch','/bounded_joint23/pair_provenance']):
            if topic=='/charuco/quality':
                q=json.loads(msg.data);quality[q['source_stamp_ns']]=(q,t.to_nsec())
            elif topic=='/tf':
                for tf in msg.transforms:
                    if tf.header.frame_id!='camera_link':continue
                    p=tf.transform.translation;r=tf.transform.rotation
                    C=quaternion_matrix([r.x,r.y,r.z,r.w]);C[:3,3]=[p.x,p.y,p.z]
                    boards[(tf.header.stamp.to_nsec(),tf.child_frame_id)]=(C,t.to_nsec())
            elif topic.endswith('accepted_joint_states'):
                values=dict(zip(msg.name,msg.position));accepted[msg.header.stamp.to_nsec()]=[values['Joint%d'%i] for i in range(1,7)]
            elif topic.endswith('pair_provenance'):
                p=json.loads(msg.data)
                if p.get('exact_rgb_board_quality'):paired.add(p['source_stamp_ns'])
            else:epochs.add(msg.stamp.to_nsec())
    if epochs!={EPOCH}:raise ValueError('partial-return epoch mismatch')
    stamps=sorted(accepted);rows=[]
    for ns,(q,received) in sorted(quality.items()):
        phase='baseline' if ns<command['command_stamp_ns'] else 'partial_J2_return' if ns>=stop['stationary']['stationary_window_start_ns'] else None
        if phase is None:continue
        match=boards.get((ns,q.get('board_frame')))
        if match is None or ns not in paired:rejected['no_exact_RGB_TF']+=1;continue
        C,tr=match
        if not(q.get('pose_ok') and q.get('charuco_count',0)>=35 and q.get('reprojection_rms_px',10)<=1.5 and q.get('camera_model_fingerprint')==template['quality']['camera_model_fingerprint'] and not q.get('camera_model_error') and not q.get('camera_model_change_latched')):rejected['quality']+=1;continue
        if not all(0<=t-ns<=350000000 for t in (received,tr)):rejected['source_age']+=1;continue
        i=bisect.bisect_left(stamps,ns);near=min(stamps[max(0,i-1):i+1],key=lambda t:abs(t-ns))
        lo=bisect.bisect_right(stamps,ns-400000000)-1;hi=bisect.bisect_left(stamps,ns+400000000)
        if lo<0 or hi>=len(stamps) or abs(near-ns)>100000000:rejected['full_window']+=1;continue
        tt=stamps[lo:hi+1];qq=np.asarray([accepted[t] for t in tt])
        if len(tt)<6 or max(np.diff(tt))>250000000 or np.max(np.ptp(qq,axis=0))>math.radians(.15):rejected['stationary']+=1;continue
        rows.append(dict(source_stamp_ns=ns,offline_stationary_phase=phase,accepted_positions_rad=accepted[near],T_camera_board=C.tolist(),quality=q,exact_rgb_archived=True))
    return template,rows,dict(rejected)


def load_old(session,path):
    template=json.loads((session/'baseline_geometry.json').read_text());rows=[]
    for r in json.loads(path.read_text()):
        if r['epoch_ns']!=EPOCH:raise ValueError('old multiframe epoch mismatch')
        rows.append(dict(source_stamp_ns=r['source_stamp_ns'],offline_stationary_phase=r['window'],accepted_positions_rad=r['accepted_rad'],
            T_camera_board=r['T_camera_board'],quality=dict(reprojection_rms_px=r['reprojection_rms_px'],charuco_count=r['charuco_count']),
            exact_rgb_archived=r['exact_fresh_rgb_quality_tf']))
    return template,rows,{}


def comparison(a,b,X):
    C0,C1=np.asarray(a['T_camera_board']),np.asarray(b['T_camera_board'])
    A0,A1=np.asarray(a['T_base_tool']),np.asarray(b['T_base_tool'])
    Y0,Y1=np.asarray(a['T_base_board']),np.asarray(b['T_base_board'])
    Ro=C1[:3,:3].T@C0[:3,:3];Rp=Y0[:3,:3].T@A1[:3,:3]@A0[:3,:3].T@Y0[:3,:3]
    vo,vp=G.log_so3(Ro),G.log_so3(Rp)
    observed=G.inv(C1)[:3,3]-G.inv(C0)[:3,3]
    expected=(G.inv(Y0)@A1@X)[:3,3]-(G.inv(Y0)@A0@X)[:3,3]
    p=np.array([.09,.0675,0.]);drift=Y1[:3,:3]@p+Y1[:3,3]-Y0[:3,:3]@p-Y0[:3,3]
    return dict(board_origin_drift_mm=float(np.linalg.norm(Y1[:3,3]-Y0[:3,3])*1000),board_center_drift_mm=float(np.linalg.norm(drift)*1000),
        relative_rotation_residual_deg=math.degrees(np.linalg.norm(G.log_so3(Rp.T@Ro))),
        observed_rotation_deg=math.degrees(np.linalg.norm(vo)),predicted_rotation_deg=math.degrees(np.linalg.norm(vp)),
        camera_travel_vector_error_mm=float(np.linalg.norm(expected-observed)*1000),
        observed_to_model_axis_deg=J.angle(vo,vp,True) if min(np.linalg.norm(vo),np.linalg.norm(vp))>math.radians(.2) else None,
        actual_joint_delta_deg=np.degrees(np.asarray(b['accepted_positions_rad'])[:6]-np.asarray(a['accepted_positions_rad'])[:6]).tolist())


def describe(rows):
    phases=defaultdict(list)
    for r in rows:phases[r['offline_stationary_phase']].append(r)
    ww={k:J.average(v) for k,v in phases.items()};order=sorted(ww,key=lambda k:min(r['source_stamp_ns'] for r in phases[k]))
    X=np.asarray(rows[0]['T_tool_camera']);comp={}
    for i in range(1,len(order)):
        for j in sorted(set([0,i-1])):
            name=order[i]+'__minus__'+order[j];comp[name]=comparison(ww[order[j]],ww[order[i]],X)
    return dict(windows={k:dict(count=len(v),exact_rgb_archived=sum(r['exact_rgb_archived'] for r in v),
        within_window_max_orientation_from_mean_deg=max(math.degrees(np.linalg.norm(G.log_so3(np.asarray(ww[k]['T_camera_board'])[:3,:3].T@np.asarray(r['T_camera_board'])[:3,:3]))) for r in v)) for k,v in phases.items()},comparisons=comp)


def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);args=p.parse_args()
    if not os.path.ismount(MOUNT) or MOUNT not in args.output.resolve().parents:raise ValueError('SanStand output required')
    args.output.mkdir(parents=True,exist_ok=True)
    prior=RESUME/'joint_metric_identifiability/fit_report.json';fit=next(f for f in json.loads(prior.read_text())['fits'] if f['orientation_unit_deg']==.1)
    configs=[('J5',ARCH/'bounded_joint_response_20260928T122338Z_47349',ARCH/'bounded_joint_response_20260928T122338Z_47349/multiframe_analysis/samples.json'),
        ('J4',ARCH/'bounded_wrist_response_diagnostic_20260928T125306Z_126028',RESUME/'wrist_multiframe_j4/samples.json'),
        ('J6',ARCH/'bounded_wrist_response_diagnostic_20260928T125421Z_130257',RESUME/'wrist_multiframe_j6/samples.json'),
        ('J2_J3',ARCH/'bounded_joint23_counterrotation_20260928T131913Z_197737',RESUME/'counterrotation_multiframe_131913/samples.json'),
        ('partial_J2_return',ARCH/'bounded_joint23_fresh_return_20260928T134632Z_269602',None)]
    result=dict(diagnostic_only=True,refitted=False,runtime_changed=False,epoch_ns=EPOCH,candidate_orientation_unit_deg=.1,
        frozen_prior_fit_sha256=hashlib.sha256(prior.read_bytes()).hexdigest(),frozen_parameters=fit['parameters'],
        source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),sessions={},
        limitations=['Independent local sessions are compared only within each session; no absolute positions across board layouts are compared.',
            'J5 quality/TF population may lack archived exact RGB frames; that distinction is explicitly counted.',
            'No new fit/weight selection. Original task16 max pairwise drift 5.449 mm remains a generalization limitation.',
            'Mean pose differences are descriptive; repeated frames are correlated and physical pose is not independently metrologized.'])
    for label,session,path in configs:
        template,rows,rej=partial_rows(session) if path is None else load_old(session,path)
        xml=(session/'robot_description.urdf').read_text();X=np.asarray(template['T_tool_camera'])
        record=dict(session=str(session),source_samples=str(path) if path else 'closed exact paired RGB/quality/TF bag recovery',
            input_sha256=hashlib.sha256(path.read_bytes()).hexdigest() if path else None,source_model_sha256=hashlib.sha256(xml.encode()).hexdigest(),
            quality_selection_rejections=rej,models={})
        for kind,z in [('runtime',np.zeros(10)),('prior_training_only_candidate',np.asarray(fit['z']))]:
            rr=E.evaluated(rows,xml,X,z);record['models'][kind]=describe(rr)
            raw=[r for r in rr if r['exact_rgb_archived']]
            record['models'][kind]['exact_RGB_only']=describe(raw) if raw else None
            good=[r for r in rr if r['quality']['reprojection_rms_px']<=.15]
            record['models'][kind]['RMS_le_0p15px']=describe(good) if good else None
        result['sessions'][label]=record
        (args.output/(label+'_input_rows.json')).write_text(json.dumps(rows,indent=2)+'\n')
    (args.output/'report.json').write_text(json.dumps(result,indent=2,ensure_ascii=False)+'\n')
    (args.output/Path(__file__).name).write_bytes(Path(__file__).read_bytes())
    print(json.dumps({k:{m:r['comparisons'] for m,r in s['models'].items()} for k,s in result['sessions'].items()},ensure_ascii=False))

if __name__=='__main__':main()
