"""Bind independently classified failures and additional closed scene records."""
import datetime as dt
import hashlib
import json
import re
from pathlib import Path

ROOT=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
MAINT=ROOT/'maintenance'
assert Path('/media/zhuyupei/SanStand').is_mount()
review_path=MAINT/'user_failed_cleanup_review_20261012_canonical/classification_evidence.json'
read=lambda p:json.loads(p.read_text())
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
review=read(review_path)
assert review['complete'] is True
records=[]
protected=list(review['protected'])


def evidence(path,finding):
    return dict(path=str(path.relative_to(ROOT)),sha256=sha(path),finding=finding)


def make_row(directory,started,reason,proof,raw,formal):
    return dict(record_path=str(directory.relative_to(ROOT)),started_at=started,
        classification='failure',closed_verified=True,non_success_verified=True,
        operator_success_conflict_checked=True,calibration=False,current_investigation_required=False,
        formal_record=formal,evidence=proof,raw_files=raw,
        summary=dict(problem=reason,method='保留原失败日志、身份/时间绑定及阶段证据；按原问题报告复核RGB-D时效、几何和执行确认。本次仅清理封口原始录制，不将既有失败改记为成功。',
                     validation='原记录明确失败且录制已关闭；已复核实物成功冲突并排除当前定位保护。具体原结果和后续验证见保留日志/审计；本摘要不声称完成实物抓取。'))


for original in review['records']:
    directory=ROOT/original['record_path']
    assert not any(s['success'] or s['state'] in ('CLOSE_GRIPPER','LIFT','SUCCEEDED') for s in original['states'])
    assert not original['active_seen'] or original['states']
    proof=[dict(p,finding='原始失败/封口证据，独立分类已核对') for p in original['evidence']]
    proof.append(evidence(review_path,'独立读取本实验/grasp/state，未出现闭爪、提升或成功；既有实物成功冲突已排除'))
    records.append(make_row(directory,original['started_at'],original['failure_reason'],proof,original['raw_files'],True))

# Independent scene recordings bind their actual closed receipt and exact
# canonical failure. Interrupted and successful/uncertain recordings remain.
legacy_dirs=sorted((ROOT/'ros_log').glob('feedforward_speed_grasp_*'))+[
    ROOT/'ros_log/grasp_diagnostic_20260928T084245Z',ROOT/'ros_log/grasp_two_stage_diagnostic_20260928T093247Z']
legacy_review=[]
for directory in legacy_dirs:
    bag=directory/'scene_diagnostic.bag'
    closed=directory/'scene_recorder_result.json'
    launch=next((p for p in (directory/'launch.log',directory/'recorded_attempt_stdout.log') if p.exists()),None)
    if not bag.is_file() or not closed.is_file() or launch is None:
        continue
    outcome=read(closed)
    matched=re.search(r'^GRASP_RECORD_DIR=(.+)$',launch.read_text(),re.M)
    if outcome.get('closed') is not True or not matched:
        protected.append(dict(path=str(directory.relative_to(ROOT)),reason='scene closure or exact result binding missing'))
        continue
    canonical=Path(matched[1]);manifest=read(canonical/'manifest.json')
    canonical_outcome=read(canonical/'recorder_outcome.json')
    if manifest.get('result')!='failure' or manifest.get('state')!='complete' or manifest.get('recorder_closed') is not True or canonical_outcome.get('bag_closed') is not True:
        protected.append(dict(path=str(directory.relative_to(ROOT)),reason='canonical result is interrupted/unknown or unclosed'))
        continue
    operator=directory/'operator_result.json'
    explicit_empty=operator.exists() and read(operator).get('physical_grasp_success') is False
    text=(canonical/'runner.log').read_text(errors='replace')
    physical_success=canonical/'physical_result_user_confirmed.json'
    if physical_success.exists() or (not explicit_empty and any(re.search(r'\b'+stage+r'\b',text) for stage in ('CLOSE_GRIPPER','LIFT','SUCCEEDED'))):
        protected.append(dict(path=str(directory.relative_to(ROOT)),reason='physical result successful/uncertain or closure reached without empty-grasp proof'))
        continue
    reason=manifest['failure_reason']
    proof=[evidence(closed,'独立场景录制器 closed=true'),evidence(launch,'精确指向本次canonical记录及failure结果'),
           evidence(canonical/'manifest.json','canonical complete/failure且recorder_closed=true'),
           evidence(canonical/'recorder_outcome.json','canonical bag_closed=true'),
           evidence(canonical/'runner.log','原阶段失败日志；未闭爪或有用户明确空抓结果')]
    if explicit_empty:proof.append(evidence(operator,'用户明确物体仍在桌面，没有夹住，physical_grasp_success=false'))
    st=bag.stat()
    raw=[dict(path=bag.name,kind='bag',size_bytes=st.st_size,sha256=None,
              reviewed_identity=dict(device=st.st_dev,inode=st.st_ino,size_bytes=st.st_size,mtime_ns=st.st_mtime_ns))]
    records.append(make_row(directory,manifest['started_at'],reason,proof,raw,False))
    legacy_review.append(dict(path=str(directory.relative_to(ROOT)),canonical=str(canonical.relative_to(ROOT)),
        empty_grasp_explicit=explicit_empty,evidence=proof,raw_identity=raw[0]['reviewed_identity']))

# Early independent attempts qualify only with an explicit no-action summary
# and the recorder's completed footer, not a folder name or process exit.
old_roots=[ROOT/'ros_log/ros_live_20260921_session',ROOT/'ros_log/ros_live_20260922_session']
for base in old_roots:
    for directory in sorted(base.iterdir()):
        bag=directory/'keyframes.bag'
        runner=directory/'runner.log'
        if not bag.is_file() or not runner.is_file():continue
        log=next((p for p in (directory/'keyframes.log',directory/'recorder.log') if p.is_file()),None)
        result=next((p for p in (directory/'recorded_action_summary.json',directory/'result.json') if p.is_file()),None)
        if log is None or result is None:continue
        value=read(result);text=runner.read_text(errors='replace');recorder=log.read_text(errors='replace')
        no_action=value.get('active_task_messages')==0 and (value.get('automatic_grasp_completed') is False or value.get('actual_grasp_completed') is False)
        if not no_action or 'NO_FRESH_MODE_PREVIEW' not in text or 'KEYFRAME_RECORDING_FINISHED' not in recorder:continue
        authorization=directory/'authorization.json'
        if not authorization.is_file():continue
        timestamp=read(authorization).get('handoff_unix')
        if not isinstance(timestamp,(float,int)):continue
        started=dt.datetime.fromtimestamp(timestamp,dt.timezone.utc).isoformat()
        reason=next(line for line in text.splitlines() if 'NO_FRESH_MODE_PREVIEW' in line)
        proof=[evidence(result,'明确无active task且未完成自动抓取'),evidence(runner,'预览失败，没有执行资格'),
               evidence(log,'KEYFRAME_RECORDING_FINISHED封口'),evidence(authorization,'真实实验开始时间')]
        st=bag.stat();raw=[dict(path=bag.name,kind='bag',size_bytes=st.st_size,sha256=None,
            reviewed_identity=dict(device=st.st_dev,inode=st.st_ino,size_bytes=st.st_size,mtime_ns=st.st_mtime_ns))]
        records.append(make_row(directory,started,reason,proof,raw,False))

protected.append(dict(path='ros_log/full_latest_frozen_no_skin_20261012T021050Z',reason='当前修复、授权、ROS与定位证据'))
for pattern in ('candidate_real_grasp_*','bounded_*','alignment_*','fixed_board_*','handeye*','full_ros_latest_start_20261002'):
    protected.extend(dict(path=str(p.relative_to(ROOT)),reason='实物成功冲突、标定或未分类几何定位证据') for p in (ROOT/'ros_log').glob(pattern))
catalog=dict(schema=1,review_id='20261012_canonical_states_and_bound_legacy_no_success',records=records)
policy=dict(schema=1,enabled=True,mount_uuid='4899-D74E',trigger_free_bytes=10*2**30,target_free_bytes=70*2**30,
    authorization='用户2026-10-12：如果sanstand空间不够，就按之前我的要求记录在文档中的，清理70G出来，以后少于10G时就清理70G出来；沿用文档可用空间GiB口径。',
    unclassified_records_protected=True)
for name,value in (('catalog',catalog),('protected',dict(schema=1,protected_paths=protected)),('policy',policy)):
    target=MAINT/('failed_record_space_guard_'+name+'.json')
    assert not target.exists()
    target.write_text(json.dumps(value,ensure_ascii=False,indent=2)+'\n')
(MAINT/'failed_record_space_guard_legacy_review_20261012.json').write_text(json.dumps(legacy_review,ensure_ascii=False,indent=2)+'\n')
print(json.dumps(dict(canonical_records=67,total_records=len(records),legacy_scene_records=len(legacy_review),
    candidate_gib=sum(i['size_bytes'] for r in records for i in r['raw_files'])/2**30,protected=len(protected))))
