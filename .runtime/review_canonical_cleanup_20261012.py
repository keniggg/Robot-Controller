"""Read-only classification; write review evidence only, never delete data."""
import datetime
import hashlib
import json
import os
from pathlib import Path
import rosbag

ROOT = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
OUT = ROOT/'maintenance/user_failed_cleanup_review_20261012_canonical'
assert Path('/media/zhuyupei/SanStand').is_mount()
OUT.mkdir(exist_ok=False)
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
prior = ROOT/'maintenance/user_failed_cleanup_20261007T134426Z/success_conflicts_review.json'
protected_ids = set(json.loads(prior.read_text())['protected_record_ids'])
opened = set()
unreadable = 0
for proc in Path('/proc').glob('[0-9]*'):
    try:
        for fd in (proc/'fd').iterdir():
            try:
                target = os.readlink(fd)
                if target.startswith(str(ROOT)+'/'):
                    opened.add(target[:-10] if target.endswith(' (deleted)') else target)
            except OSError:
                pass
    except OSError:
        unreadable += 1
report = dict(schema=1, read_only=True, recorded_at=datetime.datetime.now(datetime.timezone.utc).isoformat(),
    old_success_review=dict(path=str(prior.relative_to(ROOT)),sha256=sha(prior)),
    open_fd_check=dict(unreadable_process_fd_directories=unreadable,external_open_paths=sorted(opened)),
    records=[], protected=[])
for d in sorted((ROOT/'grasp_records').iterdir()):
    bag = d/'keyframes.bag'
    if not bag.is_file():
        continue
    manifest = json.loads((d/'manifest.json').read_text())
    if (d.name in protected_ids or manifest.get('result') != 'failure'
            or d.name < '20261006T110552' or d.name >= '20261011'):
        report['protected'].append(dict(path=str(d.relative_to(ROOT)),reason=(
            'current repair evidence 20261011 or later' if d.name >= '20261011' else
            'physical success/unknown/previous protected or outside reviewed age range')))
        continue
    summary = json.loads((d/'result_summary.json').read_text())
    outcome = json.loads((d/'recorder_outcome.json').read_text())
    assert manifest['state']=='complete' and manifest['recorder_closed'] is True
    assert summary['result']=='failure' and outcome['bag_closed'] is True
    assert not any(p.startswith(str(d)+'/') for p in opened), d
    raw = next(f for f in manifest['files'] if f['path']=='keyframes.bag')
    st = bag.stat()
    assert st.st_size == raw['size_bytes'] and len(raw['sha256']) == 64
    states=[]
    if outcome['active_seen']:
        with rosbag.Bag(str(bag),'r') as stream:
            for topic,message,stamp in stream.read_messages(topics=['/grasp/state']):
                state=dict(state=message.state,active=message.active,success=message.success,
                    message=message.message,stamp_ns=stamp.to_nsec())
                if not states or (state['state'],state['active'],state['message']) != (
                        states[-1]['state'],states[-1]['active'],states[-1]['message']):
                    states.append(state)
        assert states, ('no state evidence',d)
    evidence=[dict(path=str((d/name).relative_to(ROOT)),sha256=sha(d/name))
        for name in ('manifest.json','result_summary.json','recorder_outcome.json','command_result.json','runner.log')]
    row=dict(record_path=str(d.relative_to(ROOT)),started_at=manifest['started_at'],
        completed_at=manifest['completed_at'],failure_reason=summary['failure_reason'],
        active_seen=outcome['active_seen'],states=states,evidence=evidence,
        raw_files=[dict(path='keyframes.bag',kind='bag',size_bytes=raw['size_bytes'],sha256=raw['sha256'])],
        allocated_bytes=st.st_blocks*512,identity=dict(device=st.st_dev,inode=st.st_ino,
            size_bytes=st.st_size,mtime_ns=st.st_mtime_ns),manifest_hash_is_historical=True)
    report['records'].append(row)
    (OUT/'classification_evidence.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(dict(record=d.name,active=outcome['active_seen'],states=sorted(set(s['state'] for s in states)),
        raw_gib=round(st.st_blocks*512/2**30,3)),ensure_ascii=False),flush=True)
report['complete']=True
report['candidate_count']=len(report['records'])
report['candidate_allocated_bytes']=sum(r['allocated_bytes'] for r in report['records'])
(OUT/'classification_evidence.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
print('COMPLETE '+str(OUT),flush=True)
