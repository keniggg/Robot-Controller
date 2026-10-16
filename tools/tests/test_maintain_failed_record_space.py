import importlib.util
import json
import os
from pathlib import Path
import pytest

spec = importlib.util.spec_from_file_location('space_guard', Path(__file__).parents[1] / 'maintain_failed_record_space.py')
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)


@pytest.fixture
def configured(tmp_path, monkeypatch):
    root = tmp_path / 'records'
    (root / 'maintenance').mkdir(parents=True)
    directory = root / 'grasp_records/failed'
    directory.mkdir(parents=True)
    raw = directory / 'keyframes.bag'
    raw.write_bytes(b'closed raw experiment')
    evidence = directory / 'failure.json'
    evidence.write_text('{"result":"failure","closed":true}')
    other = directory / 'runner.log'
    other.write_text('FAILED before starting a motion')
    row = dict(record_path='grasp_records/failed', started_at='2020-01-01T00:00:00+00:00',
               classification='failure', closed_verified=True, non_success_verified=True,
               operator_success_conflict_checked=True, calibration=False,
               current_investigation_required=False, formal_record=False,
               evidence=[dict(path=str(p.relative_to(root)), sha256=m.sha(p), finding='failed/closed') for p in (evidence, other)],
               raw_files=[dict(path='keyframes.bag', kind='bag', size_bytes=raw.stat().st_size, sha256=m.sha(raw))],
               summary=dict(problem='test failure', method='retain original evidence', validation='failed, not success'))
    policy = dict(schema=1, enabled=True, authorization='explicit test fixture', mount_uuid='4899-D74E',
                  trigger_free_bytes=10*m.GIB, target_free_bytes=70*m.GIB)
    for filename, value in ((m.PREFIX+'policy.json', policy), (m.PREFIX+'catalog.json', dict(schema=1, review_id='test', records=[row])),
                            (m.PREFIX+'protected.json', dict(schema=1, protected_paths=[])),
                            ('storage_control.json',dict(state='resumed', archive_policy='local_only'))):
        (root/'maintenance'/filename).write_text(json.dumps(value))
    (root/'grasp_records/archive_config.json').write_text(json.dumps(dict(archive_policy='local_only',upload_enabled=False,delete_enabled=False)))
    guard = m.Guard(root, tmp_path)
    guard.device = raw.stat().st_dev
    monkeypatch.setattr(guard,'mounted',lambda policy: None)
    monkeypatch.setattr(guard,'available',lambda: None)
    monkeypatch.setattr(guard,'free',lambda: 5*m.GIB if raw.exists() else 71*m.GIB)
    return guard, raw, evidence, row


def test_closed_raw_deleted_but_all_original_metadata_kept(configured):
    guard, raw, evidence, _ = configured
    old = m.sha(evidence)
    result = guard.run(True)
    assert result['state']=='complete' and not raw.exists() and m.sha(evidence)==old
    receipt=json.loads(Path(result['receipt']).read_text())
    assert receipt['files'][0]['retained_hashes_unchanged']
    assert receipt['files'][0]['sha256']==m.hashlib.sha256(b'closed raw experiment').hexdigest()
    assert list(raw.parent.glob('空间清理摘要*'))


def test_kernel_lease_rejects_existing_open_file(configured):
    guard, raw, _, _ = configured
    with raw.open('rb'):
        result=guard.run(True)
    assert raw.exists() and result['state']=='eligible_exhausted'
    receipt=json.loads(Path(result['receipt']).read_text())
    assert not receipt['files'] and receipt['skipped']


def test_changed_bound_evidence_prevents_any_deletion(configured):
    guard, raw, evidence, _ = configured
    evidence.write_text('{"result":"success"}')
    with pytest.raises(AssertionError):guard.run(True)
    assert raw.exists()


def test_protected_record_prevents_any_deletion(configured):
    guard, raw, _, _ = configured
    (guard.maintenance/(m.PREFIX+'protected.json')).write_text(json.dumps(dict(schema=1,protected_paths=[dict(path='grasp_records/failed',reason='current investigation')])))
    with pytest.raises(ValueError):guard.run(True)
    assert raw.exists()


@pytest.mark.parametrize('setting',['closed_verified','non_success_verified','operator_success_conflict_checked'])
def test_uncertain_review_does_not_qualify(configured,setting):
    guard, raw, _, row=configured
    row[setting]=False
    (guard.maintenance/(m.PREFIX+'catalog.json')).write_text(json.dumps(dict(schema=1,review_id='test',records=[row])))
    with pytest.raises(AssertionError):guard.run(True)
    assert raw.exists()


def test_symlink_cannot_redirect_deletion(configured):
    guard, raw, _, _=configured
    outside=guard.root.parent/'outside.bag';outside.write_bytes(b'closed raw experiment')
    raw.unlink();raw.symlink_to(outside)
    with pytest.raises(ValueError):guard.run(True)
    assert outside.exists()


def test_high_space_never_cleans(configured,monkeypatch):
    guard,raw,_,_=configured
    monkeypatch.setattr(guard,'free',lambda:11*m.GIB)
    assert guard.run(True)['state']=='above_trigger' and raw.exists()


def test_wrong_raw_hash_preserves_file(configured):
    guard,raw,_,row=configured
    row['raw_files'][0]['sha256']='0'*64
    (guard.maintenance/(m.PREFIX+'catalog.json')).write_text(json.dumps(dict(schema=1,review_id='test',records=[row])))
    with pytest.raises(AssertionError):guard.run(True)
    assert raw.exists()


def test_user_physical_success_overrides_automated_failure(configured):
    guard,raw,_,_=configured
    (raw.parent/'physical_result_user_confirmed.json').write_text('{"physical_grip_confirmed_by_user":true}')
    with pytest.raises(ValueError):guard.run(True)
    assert raw.exists()


def test_new_opener_during_hash_cancels_deletion(configured,monkeypatch):
    guard,raw,_,_=configured
    original=m.os.read
    attempted=[False]
    def read_with_competing_opener(descriptor,size):
        if not attempted[0] and size==8*1024*1024:
            attempted[0]=True
            code='import os,sys\ntry: os.open(sys.argv[1],os.O_RDONLY|os.O_NONBLOCK)\nexcept BlockingIOError: sys.exit(0)\nsys.exit(2)'
            reply=m.subprocess.run(['/usr/bin/python3','-c',code,str(raw)],timeout=3)
            assert reply.returncode==0
        return original(descriptor,size)
    monkeypatch.setattr(m.os,'read',read_with_competing_opener)
    with pytest.raises(RuntimeError,match='requested by another process'):guard.run(True)
    assert attempted[0] and raw.exists()


@pytest.mark.parametrize('disposition',['no_motion','active','unknown','overlap_success'])
def test_only_unambiguous_mature_prestart_failure_can_self_classify(configured,disposition):
    guard,raw,_,_=configured
    (guard.root/'ros_log').mkdir()
    manifest=dict(started_at='2020-01-01T00:00:00+00:00',completed_at='2020-01-01T00:10:00+00:00',
                  state='complete',result='failure',recorder_closed=True,failure_reason='NO_FRESH_MODE_PREVIEW: stale',
                  result_evidence=dict(observations=[]),
                  files=[dict(path=raw.name,size_bytes=raw.stat().st_size,sha256=m.sha(raw))])
    outcome=dict(bag_closed=True,active_seen=False)
    if disposition=='active':outcome['active_seen']=True
    if disposition=='unknown':manifest['result']='unknown'
    for name,value in (('manifest.json',manifest),('result_summary.json',dict(result=manifest['result'])),
                       ('recorder_outcome.json',outcome),('command_result.json',dict(runner_returncode=3))):
        (raw.parent/name).write_text(json.dumps(value))
    (raw.parent/'runner.log').write_text('NO_FRESH_MODE_PREVIEW: stale')
    if disposition=='overlap_success':
        directory=raw.parent.parent/'successful';directory.mkdir()
        (directory/'manifest.json').write_text(json.dumps(dict(manifest,result='success')))
    catalog=dict(records=[])
    added=guard.discover_pre_motion(catalog,dict(protected_paths=[]))
    assert added==(disposition=='no_motion')
    assert len(catalog['records'])==(1 if added else 0)


def test_unmounted_storage_fails_before_reading_policy(tmp_path):
    guard=m.Guard(tmp_path/'missing',tmp_path/'not_mounted')
    with pytest.raises(RuntimeError,match='EXPERIMENT_STORAGE_UNAVAILABLE'):guard.run(True)


def test_invalid_enabled_string_cannot_enable_deletion(configured):
    guard,raw,_,_=configured
    path=guard.maintenance/(m.PREFIX+'policy.json')
    value=json.loads(path.read_text());value['enabled']='false';path.write_text(json.dumps(value))
    with pytest.raises(AssertionError):guard.run(True)
    assert raw.exists()


def _closed_prestart_record(raw):
    manifest = dict(started_at='2020-01-01T00:00:00+00:00',
                    completed_at='2020-01-01T00:10:00+00:00', state='complete',
                    result='failure', recorder_closed=True,
                    failure_reason='NO_FRESH_MODE_PREVIEW: stale',
                    result_evidence=dict(observations=[]),
                    files=[dict(path=raw.name, size_bytes=raw.stat().st_size, sha256=m.sha(raw))])
    for name, value in (
            ('manifest.json', manifest), ('result_summary.json', dict(result='failure')),
            ('recorder_outcome.json', dict(bag_closed=True, active_seen=False)),
            ('command_result.json', dict(runner_returncode=3))):
        (raw.parent / name).write_text(json.dumps(value))
    (raw.parent / 'runner.log').write_text('NO_FRESH_MODE_PREVIEW: stale')
    return manifest


@pytest.mark.parametrize('field', ['started_at', 'completed_at'])
@pytest.mark.parametrize('bad_value', ['missing', None, 42, {}, ''])
@pytest.mark.parametrize('context', ['unknown_conflict', 'failure_candidate'])
def test_unclosed_or_malformed_interval_never_self_classifies(configured, field, bad_value, context):
    guard, raw, _, _ = configured
    manifest = _closed_prestart_record(raw)
    if context == 'unknown_conflict':
        path = raw.parent.parent / 'currently_recording' / 'manifest.json'
        path.parent.mkdir()
        manifest.update(state='recording', result=None, recorder_closed=False)
    else:
        path = raw.parent / 'manifest.json'
    if bad_value == 'missing':
        manifest.pop(field)
    else:
        manifest[field] = bad_value
    path.write_text(json.dumps(manifest))
    catalog = dict(records=[])
    assert guard.discover_pre_motion(catalog, dict(protected_paths=[])) is False
    assert catalog['records'] == [] and raw.exists()


def test_active_null_completion_reports_exhausted_without_deleting_unknown_records(configured):
    guard, raw, _, _ = configured
    manifest = _closed_prestart_record(raw)
    current = raw.parent.parent / 'currently_recording'
    current.mkdir()
    manifest.update(state='recording', result=None, completed_at=None, recorder_closed=False)
    (current / 'manifest.json').write_text(json.dumps(manifest))
    policy_path = guard.maintenance / (m.PREFIX + 'policy.json')
    policy = json.loads(policy_path.read_text())
    policy['auto_catalog_pre_motion_failures'] = True
    policy_path.write_text(json.dumps(policy))
    catalog_path = guard.maintenance / (m.PREFIX + 'catalog.json')
    catalog_path.write_text(json.dumps(dict(schema=1, review_id='test-empty-reviewed-catalog', records=[])))
    result = guard.run(True)
    receipt = json.loads(Path(result['receipt']).read_text())
    assert result['state'] == 'eligible_exhausted' and result['deleted_bytes'] == 0
    assert receipt['files'] == [] and raw.exists() and (current / 'manifest.json').exists()
    assert json.loads(catalog_path.read_text())['records'] == []


def local_only80(guard):
    path=guard.maintenance/(m.PREFIX+'policy.json')
    value=json.loads(path.read_text())
    value.update(schema=3,target_free_bytes=80*m.GIB,metadata_retention='local_only',
        metadata_github=dict(enabled=False,delete_local_metadata_after_verified=False))
    path.write_text(json.dumps(value))
    return path


def test_local80_deletes_only_closed_raw_and_never_calls_cloud(configured,monkeypatch):
    guard,raw,evidence,_=configured
    local_only80(guard)
    import migrate_failed_record_metadata
    monkeypatch.setattr(migrate_failed_record_metadata.Cleanup,'run',
        lambda *_:pytest.fail('revoked cloud upload must not run'))
    monkeypatch.setattr(guard,'free',lambda:5*m.GIB if raw.exists() else 81*m.GIB)
    old=m.sha(evidence)
    result=guard.run(True)
    assert result['state']=='complete' and not raw.exists() and m.sha(evidence)==old


def test_local80_does_not_resume_previous_cloud_receipt_upload(configured,monkeypatch):
    guard,raw,evidence,_=configured
    local_only80(guard)
    old=guard.maintenance/'user_failed_metadata_cleanup_previous';old.mkdir()
    (old/'progress.json').write_text(json.dumps(dict(state='postdelete_upload_pending')))
    import migrate_failed_record_metadata
    monkeypatch.setattr(migrate_failed_record_metadata.Cleanup,'run',
        lambda *_:pytest.fail('previous cloud uploads were revoked'))
    guard.run(True)
    assert evidence.exists() and not raw.exists()
    assert json.loads((old/'progress.json').read_text())['state']=='postdelete_upload_pending'


def test_local80_rejects_accidental_cloud_reenable(configured):
    guard,raw,evidence,_=configured
    path=local_only80(guard)
    v=json.loads(path.read_text());v['metadata_github']['enabled']=True;path.write_text(json.dumps(v))
    with pytest.raises(AssertionError):guard.run(True)
    assert raw.exists() and evidence.exists()


def test_local80_exhaustion_keeps_all_metadata(configured,monkeypatch):
    guard,raw,evidence,_=configured
    local_only80(guard);raw.unlink();monkeypatch.setattr(guard,'free',lambda:5*m.GIB)
    old=m.sha(evidence)
    result=guard.run(True)
    assert result['state']=='eligible_exhausted' and result['deleted_bytes']==0
    assert evidence.exists() and m.sha(evidence)==old
