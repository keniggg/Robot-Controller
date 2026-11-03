#!/usr/bin/env python3
"""Closed-bag prospective evaluation of an immutable URDF/X pair; NEVER fit/control.

All validation poses use the original new-board baseline Y, not a per-bag or
per-phase recentered Y. Accepted encoder windows independently prove stationarity;
command/settled labels alone do not prove it. RGB/quality/board TF must share an
exact source stamp. Overlapping frames describe repeatability, not independent trials.
"""
import argparse
from bisect import bisect_left,bisect_right
from collections import Counter,defaultdict
import hashlib,json,math,os
from pathlib import Path
import numpy as np
import audit_joint_metric_identifiability as M

MOUNT=Path('/media/zhuyupei/SanStand')
FROZEN_MODEL='7812d6b69e854ed9f37b4e3cbd813ecd53dab3504065b2d04d287abc6fd852e0'
FROZEN_X='637701accca5eb5042075c213c5e00e9b189d3cd7788d916c8c06bf89f7b1a0c'
ORIGINAL_MODEL='890592a24b8ad818d995e69beb670ea4254bf84f589615bfac3c8562f62449b7'
ORIGINAL_X='6ab0e8c902fe87bad1935a69b72044930b79ce1b239728cfce6dda8e94533468'
FROZEN_FIT='37e558bd3749c13c5bff30c6250624b1e7cd0c21dbaf8a938d335b3dd070e672'
CAMERA='e5231b8a08edf9c055b7dc3eb06296eab90660bde2861781aa19ec1d0be06765'
NAMES=['Joint%d'%i for i in range(1,7)]
RGB_TOPICS=['/bounded_wrist/paired_rgb','/bounded_joint23/paired_rgb','/bounded_response/snapshot_rgb','/bounded_motor_centering/paired_rgb']
QUALITY='/charuco/quality';ACCEPTED='/alicia_d/accepted_joint_states';EPOCH='/alicia_d/control_reference_epoch'
DEFAULT_MANIFEST=MOUNT/'实验记录/Robot-Controller/ros_log/grasp_repair_resume_20260928T120606Z/expanded_metric_training_once/candidate_model/manifest.json'


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def validate_model_pins(manifest):
    expected={'candidate_model_sha256':FROZEN_MODEL,'candidate_handeye_sha256':FROZEN_X,
              'original_urdf_sha256':ORIGINAL_MODEL,'original_handeye_sha256':ORIGINAL_X,'frozen_fit_sha256':FROZEN_FIT}
    if any(manifest.get(k)!=v for k,v in expected.items()):raise ValueError('frozen model/fit identity mismatch')
    files={'candidate_urdf_path':FROZEN_MODEL,'candidate_handeye_path':FROZEN_X,'frozen_fit_path':FROZEN_FIT}
    for key,value in files.items():
        if sha(manifest[key])!=value:raise ValueError('candidate file hash mismatch: '+key)
    originals={Path(x['copy']).name:Path(x['copy']) for x in manifest['rollback_sources']}
    old_urdf=originals['original_robot_description.urdf'];old_x=originals['original_validated_handeye.yaml']
    if sha(old_urdf)!=ORIGINAL_MODEL or sha(old_x)!=ORIGINAL_X:raise ValueError('rollback model identity mismatch')
    return {'original':(M.MetricFk(old_urdf.read_text()),M.handeye(old_x)[0]),
            'candidate':(M.MetricFk(Path(manifest['candidate_urdf_path']).read_text()),M.handeye(Path(manifest['candidate_handeye_path']))[0])}


def stationary_window(ns,accepted,commands,epoch):
    stamps=sorted(accepted);lo=bisect_right(stamps,ns-400_000_000)-1;hi=bisect_left(stamps,ns+400_000_000)
    if lo<0 or hi>=len(stamps):return None,'full_800ms_stationary_window_missing'
    ts=stamps[lo:hi+1];qq=np.asarray([accepted[t] for t in ts],dtype=float)
    if ts[0]<=epoch:return None,'window_before_epoch'
    if any(ts[0]<=t<=ts[-1] for t in commands):return None,'window_contains_command'
    if len(ts)<6 or max(np.diff(ts))>250_000_000:return None,'accepted_sample_gap'
    if qq.shape!=(len(ts),6) or not np.all(np.isfinite(qq)):return None,'invalid_accepted_positions'
    ptp=np.ptp(qq,axis=0)
    if np.max(ptp)>math.radians(.15)+1e-14:return None,'encoder_not_stationary'
    near=min(ts,key=lambda t:abs(t-ns))
    if abs(near-ns)>100_000_000:return None,'encoder_source_separation'
    return dict(start_ns=ts[0],end_ns=ts[-1],sample_count=len(ts),maximum_gap_ns=int(max(np.diff(ts))),
                joint_peak_to_peak_deg=np.degrees(ptp).tolist(),accepted_stamp_ns=near,q=accepted[near]),None


def image_record(msg):
    if msg.encoding not in ('rgb8','bgr8') or (msg.width,msg.height)!=(640,480):raise ValueError('RGB dimensions/encoding mismatch')
    if msg.step<msg.width*3 or len(msg.data)!=msg.step*msg.height:raise ValueError('RGB payload/stride mismatch')
    return dict(source_stamp_ns=msg.header.stamp.to_nsec(),frame_id=msg.header.frame_id,
                width=msg.width,height=msg.height,encoding=msg.encoding,step=msg.step,
                raw_sha256=hashlib.sha256(bytes(msg.data)).hexdigest())


def quality_ok(q):
    rms=q.get('reprojection_rms_px');count=q.get('charuco_count')
    return (q.get('pose_ok') is True and isinstance(count,(int,float)) and count>=35
            and isinstance(rms,(int,float)) and math.isfinite(rms) and 0<=rms<=1.5
            and q.get('camera_model_fingerprint')==CAMERA and not q.get('camera_model_change_latched')
            and not q.get('camera_model_error') and q.get('camera_frame')=='camera_link'
            and q.get('board_frame')=='charuco_board' and q.get('board_size')==[12,9]
            and q.get('dictionary_id')=='DICT_5X5_100'
            and abs(q.get('square_length_m',-1)-.015)<1e-12
            and abs(q.get('marker_length_m',-1)-.01125)<1e-12)


def mean_transform(transforms):
    T=np.asarray(transforms,dtype=float);out=np.eye(4)
    out[:3,3]=T[:,:3,3].mean(axis=0);out[:3,:3]=M.matrix(M.rotation_mean(M.from_matrix(T[:,:3,:3])))
    return out


def fixed_reference_metrics(transforms,Y):
    T=np.asarray(transforms,dtype=float)
    if not len(T):return None
    d=np.linalg.norm(T[:,:3,3]-Y[:3,3],axis=1)*1000
    r=np.degrees(np.linalg.norm(M.from_matrix(np.einsum('ij,njk->nik',Y[:3,:3].T,T[:,:3,:3])).as_rotvec(),axis=1))
    values=dict(count=len(T),position_rms_mm=float(np.sqrt(np.mean(d*d))),position_max_mm=float(d.max()),
                rotation_rms_deg=float(np.sqrt(np.mean(r*r))),rotation_max_deg=float(r.max()))
    values['historical_3mm_5mm_1deg_2deg_numeric_pass']=bool(values['position_rms_mm']<=3 and values['position_max_mm']<=5 and values['rotation_rms_deg']<=1 and values['rotation_max_deg']<=2)
    values['local_4mm_4deg_numeric_pass']=bool(values['position_max_mm']<=4 and values['rotation_max_deg']<=4)
    return values


def validation_status(baseline_rows,validation_rows):
    if len(baseline_rows)<2:return 'baseline_insufficient_multiframe'
    if not validation_rows:return 'baseline_only_not_validation'
    groups=defaultdict(list)
    for r in validation_rows:groups[(r['session'],r['phase'])].append(r)
    if any(len(v)<2 for v in groups.values()):return 'validation_phase_insufficient_multiframe'
    baseline_q=np.mean([r['q'] for r in baseline_rows],axis=0)
    if not any(np.max(np.abs(np.mean([r['q'] for r in v],axis=0)-baseline_q))>math.radians(.15) for v in groups.values()):
        return 'no_distinct_posture_not_validation'
    return 'multi_posture_prospective_numeric_evaluation'



def archive_status(summary):
    if summary is None:return dict(available=False,status='unavailable_legacy_synchronous_archive',complete=None)
    complete=(summary.get('archive_complete') is True and summary.get('failure') is None
              and summary.get('accepted')==summary.get('written')
              and all(summary.get(k)==0 for k in ('failed','rejected','pending','inflight','retained_failed_payloads')))
    return dict(available=True,status='complete' if complete else 'incomplete',complete=bool(complete),summary=summary)


def add_coverage(report,audits):
    phases=[]
    for audit in audits:
        counts=audit['selected_per_phase']
        for i,(_stamp,label) in enumerate(audit['commands']):
            phase='%02d_%s'%(i+1,label);count=counts.get(phase,0)
            phases.append(dict(session=audit['session'],phase=phase,qualified_frame_count=count,
                               multiframe_coverage_complete=count>=2))
    incomplete=[p for p in phases if not p['multiframe_coverage_complete']]
    archive_incomplete=any(a.get('archive_evidence',{}).get('complete') is False for a in audits)
    report['measurement_status']=report['status']
    report['coverage']=dict(command_phases=phases,all_command_phases_have_multiframe_evidence=not incomplete,
                           explicitly_incomplete_archive=archive_incomplete,
                           archives=[dict(session=a['session'],evidence=a.get('archive_evidence',archive_status(None))) for a in audits])
    if incomplete:report['status']='partial_command_phase_coverage'
    if archive_incomplete:report['status']='incomplete_archive_diagnostic_only'
    return report


def collect_session(directory,models,expected_epoch=None,maximum_per_phase=24):
    # rosbag import is deliberately confined to this offline reader; no rospy.
    import rosbag
    directory=Path(directory);urdf=directory/'robot_description.urdf'
    if sha(urdf)!=ORIGINAL_MODEL:raise ValueError('session used incompatible original URDF')
    paths=list(directory.glob('*geometry.json'));preferred=directory/'baseline_geometry.json'
    if preferred not in paths:preferred=next(iter(paths),None)
    if preferred is None:raise ValueError('session baseline geometry absent')
    template=json.loads(preferred.read_text())
    if not np.allclose(template['T_tool_camera'],models['original'][1],atol=1e-12,rtol=0):raise ValueError('session X differs from original frozen X')
    if template['quality']['camera_model_fingerprint']!=CAMERA:raise ValueError('session camera model mismatch')
    events=[json.loads(x) for x in (directory/'events.jsonl').read_text().splitlines() if x.strip()]
    command_events=[e for e in events if e.get('event')=='command']
    commands=sorted((int(e['command_stamp_ns']),e.get('label','command_%d'%i)) for i,e in enumerate(command_events))
    command_stamps=[c[0] for c in commands];accepted={};quality={};board={};rgb={};depth={};epochs=set();reasons=Counter()
    topics=RGB_TOPICS+[QUALITY,ACCEPTED,EPOCH,'/tf','/supervisor/camera/depth/image_raw']
    with rosbag.Bag(str(directory/'evidence.bag')) as bag:
        for topic,msg,t in bag.read_messages(topics=topics):
            received=t.to_nsec()
            if topic==ACCEPTED:
                if len(msg.name)!=len(msg.position) or len(set(msg.name))!=len(msg.name):raise ValueError('malformed accepted joint state')
                q=dict(zip(msg.name,msg.position));values=[q[n] for n in NAMES];ns=msg.header.stamp.to_nsec()
                if ns in accepted and accepted[ns]!=values:raise ValueError('conflicting accepted state for same source')
                accepted[ns]=values
            elif topic==EPOCH:epochs.add(msg.stamp.to_nsec())
            elif topic==QUALITY:
                q=json.loads(msg.data);ns=q.get('source_stamp_ns')
                if isinstance(ns,int) and ns not in quality:quality[ns]=(q,received)
            elif topic=='/tf':
                for tf in msg.transforms:
                    if tf.header.frame_id!='camera_link' or tf.child_frame_id!='charuco_board':continue
                    p=tf.transform.translation;r=tf.transform.rotation;quat=np.array([r.x,r.y,r.z,r.w]);xyz=np.array([p.x,p.y,p.z])
                    if not np.all(np.isfinite(quat)) or not np.all(np.isfinite(xyz)) or abs(np.linalg.norm(quat)-1)>1e-6:raise ValueError('invalid board transform')
                    T=np.eye(4);T[:3,:3]=M.matrix(M.Rotation.from_quat(quat));T[:3,3]=xyz;key=tf.header.stamp.to_nsec()
                    if key not in board:board[key]=(T,received)
            elif topic.endswith('depth/image_raw'):
                ns=msg.header.stamp.to_nsec();depth[ns]=dict(encoding=msg.encoding,raw_sha256=hashlib.sha256(bytes(msg.data)).hexdigest())
            else:
                try:r=image_record(msg)
                except ValueError as exc:reasons[str(exc)]+=1;continue
                ns=r['source_stamp_ns']
                if ns in rgb and rgb[ns]['raw_sha256']!=r['raw_sha256']:raise ValueError('conflicting RGB payload for same exact source')
                rgb[ns]=r
    if len(epochs)!=1:raise ValueError('exactly one archived control-reference epoch required')
    epoch=next(iter(epochs))
    if expected_epoch is not None and epoch!=expected_epoch:raise ValueError('control-reference epoch changed across fixed-board evaluation')
    good=defaultdict(list)
    for ns in sorted(rgb):
        if ns<=epoch:reasons['source_before_epoch']+=1;continue
        if ns not in quality or ns not in board:reasons['missing_exact_quality_or_board_tf']+=1;continue
        q,qr=quality[ns];C,tr=board[ns]
        if not quality_ok(q):reasons['quality_or_camera_or_board_model']+=1;continue
        if rgb[ns]['frame_id']!=q['camera_frame']:reasons['RGB_board_frame_mismatch']+=1;continue
        if not all(0<=age<=350_000_000 for age in (qr-ns,tr-ns)):reasons['quality_or_tf_source_age_above_350ms']+=1;continue
        window,error=stationary_window(ns,accepted,command_stamps,epoch)
        if error:reasons[error]+=1;continue
        index=bisect_left(command_stamps,ns)-1;phase=('baseline' if index<0 else '%02d_%s'%(index+1,commands[index][1]))
        row=dict(session=str(directory.resolve()),phase=phase,source_stamp_ns=ns,epoch_ns=epoch,q=window['q'],stationarity=window,
                 rgb=rgb[ns],exact_depth=depth.get(ns),quality=q,quality_received_age_ms=(qr-ns)/1e6,tf_received_age_ms=(tr-ns)/1e6,
                 T_camera_board=C.tolist(),T_base_board={name:(fk(window['q'])@X@C).tolist() for name,(fk,X) in models.items()})
        good[phase].append(row)
    archive_file=directory/'archive_writer_summary.json'
    archive=archive_status(json.loads(archive_file.read_text()) if archive_file.exists() else None)
    if archive_file.exists():archive['summary_sha256']=sha(archive_file)
    selected=[]
    for phase,rows in good.items():
        indices=np.unique(np.linspace(0,len(rows)-1,min(maximum_per_phase,len(rows))).round().astype(int))
        selected.extend(rows[int(i)] for i in indices)
    return selected,dict(session=str(directory.resolve()),epoch_ns=epoch,command_count=len(commands),commands=commands,archive_evidence=archive,
        available_qualified_per_phase={k:len(v) for k,v in good.items()},selected_per_phase=dict(Counter(r['phase'] for r in selected)),
        rejected=dict(reasons),exact_RGB_count=len(rgb),accepted_count=len(accepted),baseline_geometry_sha256=sha(preferred),
        input_urdf_sha256=sha(urdf),events_sha256=sha(directory/'events.jsonl'),bag_size_bytes=(directory/'evidence.bag').stat().st_size)


def partition_initial_session(rows):
    """Freeze only pre-command Y; keep all same-bag post-command observations."""
    return [r for r in rows if r['phase']=='baseline'],[r for r in rows if r['phase']!='baseline']


def evaluate(baseline_rows,validation_rows):
    report=dict(status=validation_status(baseline_rows,validation_rows),baseline_frame_count=len(baseline_rows),validation_frame_count=len(validation_rows),models={})
    if len(baseline_rows)<2:return report
    for name in ('original','candidate'):
        Y=mean_transform([r['T_base_board'][name] for r in baseline_rows]);phases=defaultdict(list)
        for r in validation_rows:phases[(r['session'],r['phase'])].append(r)
        report['models'][name]=dict(fixed_baseline_T_base_board=Y.tolist(),baseline_repeatability=fixed_reference_metrics([r['T_base_board'][name] for r in baseline_rows],Y),
            all_validation=fixed_reference_metrics([r['T_base_board'][name] for r in validation_rows],Y),
            phases=[dict(session=s,phase=p,metrics=fixed_reference_metrics([r['T_base_board'][name] for r in rows],Y),
                         accepted_mean_joint_deg=np.degrees(np.mean([r['q'] for r in rows],axis=0)).tolist()) for (s,p),rows in phases.items()])
    return report


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--baseline-dir',type=Path,required=True);p.add_argument('--bag-dirs',type=Path,nargs='*',default=[])
    p.add_argument('--candidate-manifest',type=Path,default=DEFAULT_MANIFEST);p.add_argument('--board-layout-id',required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    if not os.path.ismount(MOUNT) or MOUNT not in a.output.resolve().parents:raise ValueError('SanStand output required; no fallback')
    if a.output.exists():raise ValueError('refusing to overwrite prospective evidence directory')
    if a.baseline_dir.resolve() in {x.resolve() for x in a.bag_dirs} or len({x.resolve() for x in a.bag_dirs})!=len(a.bag_dirs):raise ValueError('duplicate baseline or validation bag')
    manifest=json.loads(a.candidate_manifest.read_text());models=validate_model_pins(manifest)
    initial_rows,audit=collect_session(a.baseline_dir,models);baseline,validation=partition_initial_session(initial_rows);audits=[audit]
    for path in a.bag_dirs:
        rows,extra=collect_session(path,models,expected_epoch=audit['epoch_ns']);validation.extend(rows);audits.append(extra)
    result=add_coverage(evaluate(baseline,validation),audits)
    result.update(schema=1,diagnostic_only=True,fit_performed=False,ros_parameters_written=False,motor_commands_sent=0,
        board_layout_id=a.board_layout_id,board_layout_assumption='Caller declares this one new fixed board layout unchanged across all listed bags; image data cannot establish absence of arbitrary world board motion.',
        candidate_model_sha256=FROZEN_MODEL,candidate_handeye_sha256=FROZEN_X,frozen_fit_sha256=FROZEN_FIT,
        candidate_manifest_sha256=sha(a.candidate_manifest),evaluator_source_sha256=sha(__file__),session_audits=audits,
        limits=['Baseline Y is frozen from the same declared initial raw sources for each model; validation bags and phases never recenter it.',
                'Multiple overlapping stationary frames are correlated, not independent trials or external metrology.',
                'Exact archived board pose is evaluated; no new calibration, camera fitting, or best-model selection is performed.',
                'Threshold passes are numerical evidence only, not deployment or motion authorization. Single posture is not validation.'])
    a.output.mkdir(parents=True);(a.output/'report.json').write_text(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    (a.output/'selected_frames.json').write_text(json.dumps(dict(baseline=baseline,validation=validation),ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    (a.output/'evaluator_source.py').write_bytes(Path(__file__).read_bytes())
    print(json.dumps(dict(output=str(a.output),status=result['status'],baseline_frame_count=len(baseline),validation_frame_count=len(validation),models=result['models']),ensure_ascii=False))

if __name__=='__main__':main()
