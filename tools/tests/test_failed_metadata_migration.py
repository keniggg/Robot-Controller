"""Offline cloud-bound cleanup tests; real leases and source checks remain active."""
import hashlib
import json
from pathlib import Path
import shutil
import tarfile

import pytest

from test_maintain_failed_record_space import configured, m
from migrate_failed_record_metadata import Cleanup, INDEX


class Cloud:
    def __init__(self, destination, raw, evidence):
        self.destination=destination
        self.raw=raw
        self.evidence=evidence
        self.calls=[]
        self.fail_on=None
        self.unverified_on=None
        self.mutate_on=None
        destination.mkdir()

    def __call__(self, repository, tag):
        assert repository=='keniggg/Robot-Controller'
        self.tag=tag
        return self

    def upload(self, source, record_id):
        self.calls.append(source.name)
        if len(self.calls)<=3:
            assert self.raw.exists() and self.evidence.exists(), 'deleted before remote verification'
        if len(self.calls)==self.fail_on:
            raise RuntimeError('controlled upload failure')
        shutil.copyfile(source,self.destination/source.name)
        result=dict(verified=True,readback_verified=len(self.calls)!=self.unverified_on,
            repository='keniggg/Robot-Controller',release_tag=self.tag,
            release_url='https://github.com/keniggg/Robot-Controller/releases/tag/'+self.tag,
            size=source.stat().st_size,sha256=hashlib.sha256(source.read_bytes()).hexdigest())
        if len(self.calls)==self.mutate_on:
            self.evidence.write_text('{"result":"success"}')
        return result


@pytest.fixture
def cloud_context(configured, monkeypatch):
    guard,raw,evidence,row=configured
    p=guard.maintenance/(m.PREFIX+'policy.json')
    policy=json.loads(p.read_text())
    policy.update(schema=2,target_free_bytes=80*m.GIB,metadata_github=dict(
        enabled=True,repository='keniggg/Robot-Controller',predelete_verification_required=True,
        postdelete_receipt_upload_required=True,delete_local_metadata_after_verified=True))
    p.write_text(json.dumps(policy))
    cloud=Cloud(guard.root.parent/'fake_cloud',raw,evidence)
    monkeypatch.setattr(guard,'free',lambda:5*m.GIB if raw.exists() or evidence.exists() else 81*m.GIB)
    return guard,raw,evidence,row,policy,cloud


def test_verified_cloud_copy_precedes_raw_and_metadata_deletion(cloud_context):
    guard,raw,evidence,row,policy,cloud=cloud_context
    original=evidence.read_bytes()
    result=Cleanup(guard,cloud).run(guard.controls(),True)
    assert result['state']=='complete' and not raw.exists() and not evidence.exists()
    assert cloud.calls==['metadata.tar.gz','metadata_manifest.json','predelete_plan.json',
        'cleanup_receipt.json','github_recovery_index.json']
    with tarfile.open(cloud.destination/'metadata.tar.gz') as archive:
        assert archive.extractfile(str(evidence.relative_to(guard.root))).read()==original
        assert not any(member.name.endswith('.bag') for member in archive)
    receipt=json.loads((cloud.destination/'cleanup_receipt.json').read_text())
    assert len(receipt['files'])==3 and all(f['state']=='deleted' for f in receipt['files'])
    assert receipt['deleted_bytes']==sum(f['size_bytes'] for f in receipt['files'])
    assert (raw.parent/INDEX).is_file()
    assert not (Path(result['receipt']).parent/'metadata.tar.gz').exists()
    assert json.loads(Path(result['recovery_index']).read_text())['cleanup_receipt_asset']['readback_verified']


@pytest.mark.parametrize('failure_at',[1,2,3])
def test_any_predelete_upload_failure_preserves_all_local_sources(cloud_context,failure_at):
    guard,raw,evidence,row,policy,cloud=cloud_context
    cloud.fail_on=failure_at
    with pytest.raises(RuntimeError,match='upload failure'):
        Cleanup(guard,cloud).run(policy,True)
    assert raw.exists() and evidence.exists() and (raw.parent/'runner.log').exists()


def test_unverified_remote_reply_never_authorizes_deletion(cloud_context):
    guard,raw,evidence,row,policy,cloud=cloud_context
    cloud.unverified_on=3
    with pytest.raises(RuntimeError,match='verified readback'):
        Cleanup(guard,cloud).run(policy,True)
    assert raw.exists() and evidence.exists()


def test_predelete_upload_retry_reuses_backup_and_rechecks_original_sources(cloud_context):
    guard,raw,evidence,row,policy,cloud=cloud_context
    cloud.fail_on=2
    with pytest.raises(RuntimeError,match='upload failure'):
        Cleanup(guard,cloud).run(policy,True)
    assert raw.exists() and evidence.exists()
    runs=list(guard.maintenance.glob('user_failed_metadata_cleanup_*'))
    before=(runs[0]/'metadata.tar.gz').read_bytes()
    cloud.fail_on=None
    result=Cleanup(guard,cloud).run(policy,True)
    assert result['state']=='complete' and len(list(guard.maintenance.glob('user_failed_metadata_cleanup_*')))==1
    assert (cloud.destination/'metadata.tar.gz').read_bytes()==before


def test_success_or_source_change_after_upload_preserves_record(cloud_context):
    guard,raw,evidence,row,policy,cloud=cloud_context
    cloud.mutate_on=3
    with pytest.raises(RuntimeError):
        Cleanup(guard,cloud).run(policy,True)
    assert raw.exists() and evidence.exists()
    receipt=json.loads((cloud.destination/'cleanup_receipt.json').read_text())
    assert receipt['state']=='stopped_on_error' and not receipt['files']


def test_open_metadata_file_protects_entire_record_including_raw(cloud_context):
    guard,raw,evidence,row,policy,cloud=cloud_context
    with evidence.open('rb'):
        result=Cleanup(guard,cloud).run(policy,True)
    assert result['state']=='eligible_exhausted' and raw.exists() and evidence.exists()
    receipt=json.loads(Path(result['receipt']).read_text())
    assert receipt['skipped'] and receipt['deleted_bytes']==0


def test_undeclared_binary_file_is_not_deleted_or_uploaded(cloud_context):
    guard,raw,evidence,row,policy,cloud=cloud_context
    extra=raw.parent/'unclassified.npz';extra.write_bytes(b'unknown auxiliary data')
    Cleanup(guard,cloud).run(policy,True)
    assert extra.read_bytes()==b'unknown auxiliary data'
    with tarfile.open(cloud.destination/'metadata.tar.gz') as archive:
        assert not any(member.name.endswith('.npz') for member in archive)


def test_receipt_upload_failure_is_finished_before_new_cleanup(cloud_context):
    guard,raw,evidence,row,policy,cloud=cloud_context
    cloud.fail_on=4
    with pytest.raises(RuntimeError,match='upload failure'):
        Cleanup(guard,cloud).run(policy,True)
    assert not raw.exists() and not evidence.exists()
    cloud.fail_on=None
    result=Cleanup(guard,cloud).run(policy,True)
    assert result['state']=='above_trigger'
    assert cloud.calls.count('cleanup_receipt.json')==2
    assert all(json.loads(p.read_text())['state']=='complete'
               for p in guard.maintenance.glob('user_failed_metadata_cleanup_*/progress.json'))


def test_metadata_only_record_with_previously_deleted_raw_is_migrated(cloud_context):
    guard,raw,evidence,row,policy,cloud=cloud_context
    raw.unlink()
    old_upload=cloud.upload
    def upload(source,record_id):
        # No new raw deletion is claimed for a historical completed cleanup.
        if len(cloud.calls)<3:
            assert evidence.exists()
        was_raw=cloud.raw
        cloud.raw=evidence
        try:
            return old_upload(source,record_id)
        finally:
            cloud.raw=was_raw
    cloud.upload=upload
    result=Cleanup(guard,cloud).run(policy,True)
    assert result['state']=='complete' and not evidence.exists()
    receipt=json.loads(Path(result['receipt']).read_text())
    assert all(not f['raw_recording'] for f in receipt['files'])


def test_missing_local_deletion_authorization_fails_closed(cloud_context):
    guard,raw,evidence,row,policy,cloud=cloud_context
    p=guard.maintenance/(m.PREFIX+'policy.json')
    policy['metadata_github']['delete_local_metadata_after_verified']=False
    p.write_text(json.dumps(policy))
    with pytest.raises(AssertionError):
        guard.run(True)
    assert raw.exists() and evidence.exists()


def test_new_uploads_use_small_verified_parts(tmp_path):
    from test_grasp_archive_transport import FakeGitHub
    from migrate_failed_record_metadata import Publisher
    fake=FakeGitHub()
    source=tmp_path/'metadata.tar.gz';source.write_bytes(b'm'*5*1024*1024)
    publisher=Publisher('keniggg/Robot-Controller','record-1')
    publisher.transport._opener=type('Opener',(),{'open':staticmethod(fake.open)})()
    publisher.transport._token='fake-test-token'
    result=publisher.upload(source,'record-1')
    assert [part['size'] for part in result['parts']]==[4*1024*1024,1024*1024]
    assert result['readback_verified'] and fake.uploads==2 and fake.readbacks>=2


def test_lost_response_whole_asset_is_reused_without_duplicate_upload(tmp_path):
    from test_grasp_archive_transport import FakeGitHub
    from migrate_failed_record_metadata import Publisher
    from grasp_archive_transport import GithubReleaseStore
    fake=FakeGitHub()
    source=tmp_path/'metadata.tar.gz';source.write_bytes(b'm'*5*1024*1024)
    old=GithubReleaseStore('keniggg','Robot-Controller','record-1',token='fake-test-token',opener=fake)
    old.upload_file('record-1',source,source.name)
    publisher=Publisher('keniggg/Robot-Controller','record-1')
    publisher.transport._opener=type('Opener',(),{'open':staticmethod(fake.open)})()
    publisher.transport._token='fake-test-token'
    result=publisher.upload(source,'record-1')
    assert result['readback_verified'] and len(result['parts'])==1 and fake.uploads==1
