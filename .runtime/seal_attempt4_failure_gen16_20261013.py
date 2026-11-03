"""Seal the actual R4 failure, reusing already completed full-byte hash evidence."""
from pathlib import Path
import hashlib,json,os,time
W=Path(__file__).resolve().parents[1]
E=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
N=E/'ros_log/full_latest_contact_admission_20261013T011103Z'
R=E/'grasp_records/20261013T094326.672604Z_0b39e4a7c8514b43b66ae320b8aaeea8'
O=N/'attempt_4_seal'
read=lambda p:json.loads(Path(p).read_text())
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
def save(p,d):
    with p.open('x') as f:f.write(json.dumps(d,ensure_ascii=False,indent=2)+'\n')
def update(p,d):
    q=p.with_name(p.name+'.seal4_tmp')
    with q.open('x') as f:f.write(json.dumps(d,ensure_ascii=False,indent=2)+'\n');f.flush();os.fsync(f.fileno())
    q.replace(p)
assert Path('/media/zhuyupei/SanStand').is_mount() and not O.exists()
m=read(R/'manifest.json');outcome=read(R/'recorder_outcome.json')
reason='OBSERVE_RETURN_SUPPLEMENTAL_FAILED: OBSERVE_RETURN_REMOTE_ACK_TIMEOUT'
assert m['state']=='complete' and m['result']=='failure' and m['failure_reason']==reason
assert m['recorder_closed'] and outcome['bag_closed']
assert m['result_evidence']['runner_returncode']==6
assert 'GRASP_RESULT success=False message='+reason in (R/'runner.log').read_text()
a=read(R/'authorization.json');assert a['selection']['generation']==5
extract_path=N/'verification/attempt4_producer_timeline_r1/result.json';extract=read(extract_path)
assert extract['extraction_complete'] and extract['source_bag_unchanged']
assert extract['original_manifest_sha256']==sha(R/'manifest.json')
rows=[]
for row in m['files']:
    p=R/row['path'];assert R in p.resolve().parents and p.is_file() and not p.is_symlink()
    assert p.stat().st_size==row['size_bytes']
    if row['path']=='keyframes.bag':
        assert extract['source_bag']==str(p) and extract['source_bag_sha256']==row['sha256']
        evidence=dict(hash_provenance=str(extract_path),hash_provenance_sha256=sha(extract_path))
    else:
        assert sha(p)==row['sha256'],row['path'];evidence=dict(hash_computed_now=True)
    rows.append(dict(row,verified=True,**evidence))
topics=extract['all_bag_topics']
assert topics['/alicia_controller/follow_joint_trajectory/goal']['count']==1
assert topics['/alicia_controller/follow_joint_trajectory/result']['count']==1
assert not any('hand_controller' in name and name.endswith('/goal') for name in topics)
wire_path=N/'verification/attempt4_producer_timeline_r1/wire.json'
last=[r for r in read(wire_path) if r['topic']=='/grasp/state'][-1]
assert last['message']['state']=='FAILED' and last['message']['active'] is False
assert sha(Path(last['file']))==last['sha256']
side_path=N/'attempt_4_full_frames/result.json';side=read(side_path);bag=Path(side['path'])
assert side['recording_closed'] and side['returncode']==0
assert bag==N/'attempt_4_full_frames/full_frames.bag' and bag.stat().st_size==side['size_bytes']
assert not list(R.glob('*.bag.active')) and not list(bag.parent.glob('*.bag.active'))
assert not (Path('/proc')/str(side['pid'])).exists()
writer_matches=[];inaccessible=0
paths={str(bag),str(R/'keyframes.bag')}
for proc in Path('/proc').iterdir():
    if not proc.name.isdigit():continue
    try: entries=list((proc/'fd').iterdir())
    except (PermissionError,FileNotFoundError):inaccessible+=1;continue
    for fd in entries:
        try:
            if os.readlink(fd) not in paths:continue
            lines=(proc/'fdinfo'/fd.name).read_text().splitlines()
            flags=int(next(x.split(':',1)[1].strip() for x in lines if x.startswith('flags:')),8)
            if flags&3:writer_matches.append(dict(pid=proc.name,fd=fd.name))
        except (FileNotFoundError,PermissionError,ProcessLookupError):continue
assert not writer_matches,writer_matches
deployment_path=N/'gen15_deployment.json';dep=read(deployment_path)
assert dep['deployment_passed'] and dep['whole_patch_deployed'] and len(dep['source_sha256'])==908
assert all(sha(W/p)==h for p,h in dep['source_sha256'].items())
protection=read(E/'maintenance/failed_record_space_guard_protected.json')['protected_paths']
assert all(any(x['path']==str(p.relative_to(E)) for x in protection) for p in (R,bag.parent))
O.mkdir()
verification=dict(passed=True,recorded_ns=time.time_ns(),record=str(R),result='failure',failure_reason=reason,
    manifest_sha256=sha(R/'manifest.json'),files=rows,main_recorder_closed=True,
    sidecar=dict(receipt=str(side_path),receipt_sha256=sha(side_path),bag=str(bag),size_bytes=side['size_bytes'],sha256=side['sha256'],
        hash_provenance='Owned reviewed recorder hashes every byte after finalization before writing recording_closed=true',recording_closed=True),
    no_accessible_open_writer=True,inaccessible_proc_fd_directories=inaccessible,selection=a['selection'],
    action_goal_counts=dict(arm=1,hand=0),last_task_state=last,
    source_count=908,source_unchanged=True,deployment=str(deployment_path),deployment_sha256=sha(deployment_path),
    physical_grasp_verified=False,manual_restored=False,manual_mode=None,
    manual_note='Restore attempt lost true during final check; user confirmed currently operating mode switch. No successful restoration is claimed.',
    operator_mode_confirmation='call_7addf96b971e4ec6a7fafc7d9fd8ce1b:0',no_recording_deleted=True)
save(O/'manifest_verification.json',verification)
for i,p in enumerate((N/'current_progress.json',W/'.runtime/latest_full_ros_start.json')):
    before=p.read_bytes();(O/('progress_before_%d.json'%i)).write_bytes(before);d=json.loads(before)
    assert d['last_record']==str(R) and d['automatic_grasp_running'] is False
    d.update(last_record_sealed=True,main_recorder_closed=True,sidecar_seal_pending=False,
        closed_record_verification=str(O/'manifest_verification.json'),
        full_frame_sidecar=str(side_path),full_frame_sidecar_bag=str(bag),full_frame_sidecar_sha256=side['sha256'],
        updated_ns=time.time_ns())
    update(p,d)
save(O/'result.json',dict(passed=True,record=str(R),manifest_files=len(rows),protected_count=len(protection),
    verification_sha256=sha(O/'manifest_verification.json'),source_count=908,physical_grasp_verified=False,
    manual_restored=False,no_recording_deleted=True,script_sha256=sha(Path(__file__))))
print(json.dumps(dict(passed=True,record=str(R),manifest_files=len(rows),protected_count=len(protection))))
