"""Read-only external metadata inventory; classification artifact stays in workspace.
No unlink, ROS calls, device queries or recursive bulk-file hashing.
"""
from collections import Counter
from datetime import datetime,timezone
import hashlib,json,os
from pathlib import Path

EXT=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
HERE=Path(__file__).resolve().parent
ROOTS=[EXT/'grasp_records',EXT/'ros_log/unknown_grasp_20260923']
PRIOR=EXT/'maintenance/user_failed_cleanup_20261004T065532Z/storage_cleanup_retention_review.json'
RAW_EXT={'.bag','.png','.jpg','.jpeg','.mp4','.avi','.npz','.npy','.pcd','.ply','.pkl'}

def sha(raw):return hashlib.sha256(raw).hexdigest()
def time_parse(value):
    result=datetime.fromisoformat(value.replace('Z','+00:00'))
    if result.tzinfo is None:raise ValueError('naive_start_time')
    return result.astimezone(timezone.utc)
def identity(p):
    s=p.lstat();return dict(size_bytes=s.st_size,allocated_bytes=s.st_blocks*512,mtime_ns=s.st_mtime_ns,device=s.st_dev,inode=s.st_ino)
def main():
    prior=json.loads(PRIOR.read_text());protect={r['record_id']:r for r in prior['protect_records']}
    agent_path=HERE/'success_conflicts_review.json';independent=None
    if agent_path.exists():independent=json.loads(agent_path.read_text())
    rows=[];errors=[]
    for root in ROOTS:
        for directory in sorted(root.iterdir()):
            if not directory.is_dir() or directory.is_symlink():continue
            mp=directory/'manifest.json'
            if not mp.is_file():continue
            try:
                mr=mp.read_bytes();m=json.loads(mr);started=time_parse(m['started_at']);reasons=[]
                row=dict(record_id=directory.name,path=str(directory),root=str(root),started_at=m['started_at'],started_utc=started.isoformat(),
                    completed_at=m.get('completed_at'),state=m.get('state'),result=m.get('result'),recorder_closed=m.get('recorder_closed'),
                    failure_reason=m.get('failure_reason'),manifest_sha256=sha(mr),manifest_identity=identity(mp),
                    current_20261007_or_later=started.date().isoformat()>='2026-10-07',old_cleanup_receipt_present=(directory/'user_cleanup_receipt.json').exists(),
                    evidence=[dict(path=str(mp),sha256=sha(mr))],raw_present=[],candidate_bags=[],protection_reasons=reasons,
                    result_evidence=m.get('result_evidence'),metadata=m.get('metadata'),owner=m.get('owner'))
                for name in ('result_summary.json','recorder_outcome.json','command_result.json','physical_result_user_confirmed.json'):
                    f=directory/name
                    if f.is_file() and not f.is_symlink():
                        raw=f.read_bytes();row['evidence'].append(dict(path=str(f),sha256=sha(raw)))
                        if name=='result_summary.json':
                            value=json.loads(raw);row['result_summary_outcome']=value.get('result')
                            if value.get('result') not in (None,m.get('result')):reasons.append('manifest_result_summary_conflict')
                        elif name=='physical_result_user_confirmed.json':
                            value=json.loads(raw);row['explicit_physical_observation']=value
                            if value.get('physical_grip_confirmed_by_user') is True:reasons.append('explicit_user_physical_success')
                for info in m.get('files',[]):
                    name=info.get('path','');file=directory/name
                    if not name or file.suffix.lower() not in RAW_EXT or not file.exists():continue
                    if file.is_symlink() or directory not in file.resolve().parents:
                        reasons.append('manifest_raw_path_not_canonical');continue
                    if not file.is_file():continue
                    f=dict(path=str(file),manifest_path=name,manifest_sha256=info.get('sha256'),manifest_size_bytes=info.get('size_bytes'),identity=identity(file))
                    row['raw_present'].append(f)
                    if f['identity']['size_bytes']!=info.get('size_bytes'):reasons.append('raw_size_manifest_mismatch')
                    # Prefer closed bulky bag only; compact images/fixtures remain retained.
                    if file.suffix.lower()=='.bag':row['candidate_bags'].append(f)
                if row['current_20261007_or_later']:reasons.append('current_task_20261007_or_later_keep_full')
                if m.get('state')!='complete' or m.get('recorder_closed') is not True:reasons.append('not_closed_complete')
                if m.get('result')!='failure':reasons.append('not_explicit_automated_failure')
                if row['old_cleanup_receipt_present']:reasons.append('existing_cleanup_receipt_do_not_overwrite')
                if directory.name in protect:
                    reasons.append('prior_physical_success_or_overlap');row['prior_retention_evidence']=protect[directory.name]
                if any('calib' in v.lower() or 'handeye' in v.lower() for v in directory.parts):reasons.append('calibration_keep_full')
                row['candidate_bag_allocated_bytes']=sum(f['identity']['allocated_bytes'] for f in row['candidate_bags'])
                row['candidate_bag_size_bytes']=sum(f['identity']['size_bytes'] for f in row['candidate_bags'])
                row['preliminary_eligible']=bool(row['candidate_bags']) and not reasons
                row['eligible_for_cleanup']=False
                row['pending_independent_success_conflict_review']=row['preliminary_eligible']
                rows.append(row)
            except Exception as exc:errors.append(dict(path=str(directory),error=repr(exc)))
    rows.sort(key=lambda r:(r['started_utc'],r['path']))
    independent_rows={} if independent is None else {v['record_id']:v for v in independent['records']}
    for r in rows:
        check=independent_rows.get(r['record_id'])
        if check is not None:
            r['independent_success_review']=dict(disposition=check['review_disposition'],protect=check['protect'],
                evidence=check.get('evidence',[]),overlap_review=check.get('overlap_review'))
            original=next((e for e in check['metadata_sources'] if e['path']==str(Path(r['path'])/'manifest.json')),None)
            if original is None or original['sha256']!=r['manifest_sha256']:
                r['protection_reasons'].append('independent_review_manifest_hash_mismatch')
            if check['protect']:r['protection_reasons'].append('independent_review_physical_success_overlap_or_uncertain_outcome')
        if r['preliminary_eligible'] and check is None:r['protection_reasons'].append('independent_success_review_missing')
        r['eligible_for_cleanup']=bool(r['candidate_bags']) and not r['protection_reasons']
        r['pending_independent_success_conflict_review']=r['preliminary_eligible'] and check is None
    totals=Counter();eligible=Counter()
    for r in rows:
        if r['preliminary_eligible']:totals[r['started_utc'][:10]]+=r['candidate_bag_allocated_bytes']
        if r['eligible_for_cleanup']:eligible[r['started_utc'][:10]]+=r['candidate_bag_allocated_bytes']
    out=dict(schema=1,kind='read_only_cleanup60_classification_review',created_utc=datetime.now(timezone.utc).isoformat(),
        scope='Formal manifests only; legacy standalone data not independently reclassified. No deletion and no expensive full disk hashing.',
        target_available_bytes=60*2**30,prior_review_path=str(PRIOR),prior_review_sha256=sha(PRIOR.read_bytes()),
        criteria=['state=complete; recorder_closed=true; explicit result=failure; original result summary agreement',
                  'Preserve unknown/interrupted/calibration/current20261007 and later',
                  'Physical success and overlapping mixed success recordings override automated failure',
                  'Only listed regular bag files candidates; retain summaries, validation, metadata, logs, source and compact diagnostic fixtures',
                  'Sort by parsed original started_at UTC then canonical path, not file modification time',
                  'Real-process open file checks, mount identity and final source binding must be performed by executor before deletion'],
        independent_success_review_present=independent is not None,records=rows,errors=errors,
        counts=dict(total=len(rows),preliminary_candidates=sum(r['preliminary_eligible'] for r in rows),eligible=sum(r['eligible_for_cleanup'] for r in rows)),
        preliminary_bag_allocated_bytes=sum(totals.values()),preliminary_by_day_bytes=dict(totals),
        classification_final=independent is not None and not errors,independent_success_review_path=str(agent_path),
        independent_success_review_sha256=sha(agent_path.read_bytes()) if independent is not None else None,
        eligible_bag_allocated_bytes=sum(eligible.values()),eligible_by_day_bytes=dict(eligible),
        available_bytes_observed=os.statvfs(EXT).f_bavail*os.statvfs(EXT).f_frsize,source_sha256=sha(Path(__file__).read_bytes()))
    (HERE/'classification_review.json').write_text(json.dumps(out,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(dict(rows=len(rows),preliminary_candidates=out['counts']['preliminary_candidates'],preliminary_GiB=out['preliminary_bag_allocated_bytes']/2**30,by_day_GiB={k:round(v/2**30,3) for k,v in totals.items()},eligible_count=out['counts']['eligible'],eligible_GiB=out['eligible_bag_allocated_bytes']/2**30,errors=errors),ensure_ascii=False))

if __name__=='__main__':main()
