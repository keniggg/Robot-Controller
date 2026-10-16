"""Verified GitHub metadata migration before evidence-bound failed-record cleanup."""
import datetime as dt
import fcntl
import hashlib
import json
import os
from pathlib import Path
import signal
import stat
import tarfile
import time
from urllib.parse import quote

from grasp_archive_transport import GithubReleaseStore, HTTPStatusError

TEXT_SUFFIXES = {'.json', '.jsonl', '.md', '.log', '.txt', '.yaml', '.yml',
                 '.csv', '.tsv', '.xml', '.py', '.urdf'}
INDEX = 'github_cleanup_index.json'


class MetadataTransport(GithubReleaseStore):
    def _prepare_tag(self, tag, create=False):
        try:
            return self._json('GET', self.repo_url+'/releases/tags/'+quote(tag, safe=''))
        except HTTPStatusError as error:
            if error.status != 404 or not create:
                raise
        return self._json('POST', self.repo_url+'/releases', dict(
            tag_name=tag, name='Failed experiment metadata '+tag,
            body='Evidence-approved failed experiments: summaries, original results, '
                 'authorization, logs, audits and precise cleanup lists. Original raw '
                 'recordings are not archived here. Failure remains failure. '
                 'The metadata manifest contains original paths and SHA256 hashes; '
                 'the final cleanup receipt records actual deletions.',
            draft=False, prerelease=False, make_latest='false'))


class Publisher:
    def __init__(self, repository, tag):
        owner, repo = repository.split('/')
        self.transport = MetadataTransport(owner, repo, tag,
            chunk_size=4*1024*1024, timeout=120)

    def upload(self, path, record_id):
        chunk_size=self.transport.chunk_size
        try:
            # A lost upload response may leave a complete older whole-file
            # asset. Reuse and reverify it instead of uploading duplicate parts.
            self.transport.prepare_release(create=True)
            self.transport.chunk_size=1024*1024*1024
            whole=self.transport.plan_file(record_id,path,Path(path).name)
            assets=self.transport._assets()
            if not all(part['asset_name'] in assets and
                    assets[part['asset_name']].get('state')=='uploaded' for part in whole['parts']):
                self.transport.chunk_size=chunk_size
            entry = self.transport.upload_file(record_id, path, Path(path).name)
            # Full streamed remote readback also works when GitHub omits a digest.
            self.transport.verify_file(entry, readback=True)
        finally:
            self.transport.chunk_size=chunk_size
        entry['readback_verified'] = True
        return entry


class Cleanup:
    def __init__(self, guard, publisher_factory=Publisher):
        self.g = guard
        self.publisher_factory = publisher_factory
        self.broken = False

    def inventory(self, row):
        directory = self.g.path(row['record_path'])
        raw = {str(Path(item['path'])):item for item in row['raw_files']}
        files = []
        for path in sorted(directory.rglob('*')):
            if path.is_symlink():
                raise ValueError('symlink in candidate experiment: '+str(path))
            if not path.is_file() or path.name in ('.lock', INDEX):
                continue
            relative = str(path.relative_to(directory))
            is_raw = relative in raw
            if not is_raw and path.suffix not in TEXT_SUFFIXES:
                continue  # Undeclared binary data is not eligible for deletion.
            info = path.stat()
            if not stat.S_ISREG(info.st_mode) or info.st_nlink != 1:
                raise ValueError('candidate file is not a single regular file')
            if is_raw and (raw[relative]['kind'] != 'bag' or path.suffix != '.bag'
                    or info.st_size != raw[relative]['size_bytes']):
                raise ValueError('raw recording differs from reviewed cleanup scope')
            digest = raw[relative]['sha256'] if is_raw else self.g.sha_file(path)
            if is_raw and digest is None:
                digest = self.g.sha_file(path)
            files.append(dict(path=str(path.relative_to(self.g.root)),
                size_bytes=info.st_size, allocated_bytes=info.st_blocks*512,
                sha256=digest, raw_recording=is_raw,
                identity=[info.st_dev,info.st_ino,info.st_size,info.st_mtime_ns]))
        return files

    def require_bindings(self, bindings):
        for name, digest in bindings.items():
            if self.g.sha_file(name) != digest:
                raise RuntimeError('cleanup policy/catalog/protection changed')

    def delete_record(self, row, files, run, ledger, bindings):
        import maintain_failed_record_space as common
        self.require_bindings(bindings)
        self.g.record(row, json.loads(self.g.path('maintenance/'+common.PREFIX+'protected.json').read_text()))
        descriptors = []
        self.broken = False
        try:
            for item in files:
                path = self.g.path(item['path'])
                fd = os.open(path, os.O_RDONLY|os.O_NONBLOCK|os.O_NOFOLLOW)
                info = os.fstat(fd)
                descriptors.append((fd,path,item,info))
                if ([info.st_dev,info.st_ino,info.st_size,info.st_mtime_ns] != item['identity']
                        or info.st_nlink != 1 or not stat.S_ISREG(info.st_mode)):
                    raise RuntimeError('source identity changed after GitHub backup')
                fcntl.fcntl(fd, fcntl.F_SETOWN, os.getpid())
                fcntl.fcntl(fd, fcntl.F_SETLEASE, fcntl.F_WRLCK)
                digest = hashlib.sha256()
                while True:
                    block = os.read(fd,8*1024*1024)
                    if self.broken:
                        raise RuntimeError('candidate requested by another process')
                    if not block:
                        break
                    digest.update(block)
                if digest.hexdigest() != item['sha256']:
                    raise RuntimeError('source hash changed after GitHub backup')
                if common.identity(os.fstat(fd)) != common.identity(info):
                    raise RuntimeError('source identity changed while hashing under lease')
            if self.inventory_paths(row) != {item['path'] for item in files}:
                raise RuntimeError('experiment file set changed after GitHub backup')
            self.require_bindings(bindings)
            self.g.mounted(ledger['policy'])
            if self.g.controls() is None:
                raise RuntimeError('maintenance paused')
            quarantine = run/'quarantine'
            quarantine.mkdir(exist_ok=True)
            for fd,path,item,original in descriptors:
                if self.broken or fcntl.fcntl(fd,fcntl.F_GETLEASE) != fcntl.F_WRLCK:
                    raise RuntimeError('file lease broken; stop deletion')
                if not (common.identity(os.fstat(fd)) == common.identity(original)
                        == common.identity(path.stat())):
                    raise RuntimeError('source identity changed under lease')
                self.require_bindings(bindings)
                self.g.mounted(ledger['policy'])
                if self.g.controls() is None:
                    raise RuntimeError('maintenance paused')
                entry = dict(item,state='intent',record_path=row['record_path'])
                ledger['files'].append(entry)
                common.atomic(run/'deletion_ledger.json',ledger)
                moved = quarantine/(str(len(ledger['files']))+'.file')
                os.rename(path,moved)
                try:
                    if self.broken or fcntl.fcntl(fd,fcntl.F_GETLEASE) != fcntl.F_WRLCK:
                        raise RuntimeError('file requested during deletion')
                    self.require_bindings(bindings)
                    self.g.mounted(ledger['policy'])
                    moved.unlink()
                except Exception:
                    if moved.exists() and not path.exists():
                        os.rename(moved,path)
                    entry['state']='retained'
                    common.atomic(run/'deletion_ledger.json',ledger)
                    raise
                entry.update(state='deleted',deleted_ns=time.time_ns())
                ledger['deleted_bytes'] += item['size_bytes']
                ledger['deleted_allocated_bytes'] += item['allocated_bytes']
                common.atomic(run/'deletion_ledger.json',ledger)
        finally:
            for fd,_path,_item,_original in reversed(descriptors):
                try:
                    fcntl.fcntl(fd,fcntl.F_SETLEASE,fcntl.F_UNLCK)
                finally:
                    os.close(fd)

    def inventory_paths(self, row):
        directory = self.g.path(row['record_path'])
        raw = {str(Path(i['path'])) for i in row['raw_files']}
        result = set()
        for path in directory.rglob('*'):
            if path.is_symlink():
                raise ValueError('new symlink in experiment')
            if path.is_file() and path.name not in ('.lock',INDEX):
                if str(path.relative_to(directory)) in raw or path.suffix in TEXT_SUFFIXES:
                    result.add(str(path.relative_to(self.g.root)))
        return result

    def finish(self, run, publisher, state, backups):
        import maintain_failed_record_space as common
        if self.g.controls() is None:
            raise RuntimeError('maintenance paused before receipt upload')
        final = publisher.upload(run/'cleanup_receipt.json',run.name)
        self.require_upload(final,run/'cleanup_receipt.json')
        index = dict(schema=1,kind='github_verified_failed_experiment_cleanup',
            state=state,repository='keniggg/Robot-Controller',
            release_url=final['release_url'],metadata_assets=backups,
            cleanup_receipt_asset=final,updated_ns=time.time_ns())
        common.atomic(run/'github_recovery_index.json',index)
        remote_index = publisher.upload(run/'github_recovery_index.json',run.name)
        self.require_upload(remote_index,run/'github_recovery_index.json')
        common.atomic(run/'remote_recovery_index_asset.json',remote_index)
        common.atomic(run/'progress.json',dict(state='complete',result_state=state))
        # Only staging payloads already verified in GitHub are removed here.
        for filename in ('metadata.tar.gz','metadata_manifest.json','predelete_plan.json'):
            path=run/filename
            if path.exists():
                path.unlink()
        return index

    def require_upload(self, entry, path):
        if (entry.get('verified') is not True or entry.get('readback_verified') is not True
                or entry.get('repository') != 'keniggg/Robot-Controller'
                or entry.get('sha256') != self.g.sha_file(path)
                or entry.get('size') != path.stat().st_size):
            raise RuntimeError('GitHub upload lacks complete verified readback')

    def run(self, policy, apply=False):
        import maintain_failed_record_space as common
        with (self.g.maintenance/(common.PREFIX+'lock')).open('a') as lock:
            fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
            # A verified metadata backup survives even if the final receipt
            # upload was interrupted. Finish that receipt before any new work.
            resume = None
            for pending in sorted(self.g.maintenance.glob('user_failed_metadata_cleanup_*/progress.json')):
                progress=json.loads(pending.read_text())
                if progress['state']=='postdelete_upload_pending':
                    if not apply:
                        return dict(state='pending_receipt_upload',run=str(pending.parent))
                    run=pending.parent
                    publisher=self.publisher_factory(policy['metadata_github']['repository'],progress['tag'])
                    self.finish(run,publisher,progress['result_state'],progress['backups'])
                elif progress['state']=='predelete_upload_pending':
                    if not apply:
                        return dict(state='pending_predelete_upload',run=str(pending.parent))
                    if resume is not None:
                        raise RuntimeError('multiple pending metadata batches require review')
                    resume=pending.parent
                elif progress['state']=='running':
                    raise RuntimeError('interrupted deletion requires intent review: '+str(pending))
            initial=self.g.free()
            if resume is None and initial>=policy['trigger_free_bytes']:
                return dict(state='above_trigger',free_bytes=initial)
            catalog_path=self.g.path('maintenance/'+common.PREFIX+'catalog.json')
            protected_path=self.g.path('maintenance/'+common.PREFIX+'protected.json')
            catalog=json.loads(catalog_path.read_text())
            protected=json.loads(protected_path.read_text())
            if resume is None and apply and policy.get('auto_catalog_pre_motion_failures',False):
                if self.g.discover_pre_motion(catalog,protected):
                    common.atomic(catalog_path,catalog)
            records=[]
            if resume is not None:
                plan=json.loads((resume/'predelete_plan.json').read_text())
                self.require_bindings(plan['binding_sha256'])
                manifest=json.loads((resume/'metadata_manifest.json').read_text())
                if self.g.sha_file(resume/'metadata.tar.gz')!=manifest['bundle_sha256']:
                    raise RuntimeError('pending metadata bundle changed')
                for group in plan['records']:
                    self.g.record(group['row'],protected)
                    if self.inventory(group['row'])!=group['files']:
                        raise RuntimeError('pending backup source changed')
                    records.append(group)
            else:
                rows=sorted(catalog['records'],key=lambda r:(r['started_at'],r['record_path']))
                for row in rows:
                    files=self.inventory(row)
                    if not files:
                        continue
                    self.g.record(row,protected)
                    records.append(dict(row=row,files=files))
            amount=sum(item['size_bytes'] for r in records for item in r['files'])
            if not apply:
                return dict(state='reviewed_plan',candidate_records=len(records),
                    candidate_bytes=amount,free_bytes=initial,target_free_bytes=policy['target_free_bytes'])
            if not records:
                result=dict(state='eligible_exhausted',candidate_records=0,deleted_bytes=0,free_bytes=initial)
                common.atomic(self.g.maintenance/(common.PREFIX+'status.json'),result)
                return result
            run=resume or self.g.maintenance/('user_failed_metadata_cleanup_'+str(time.time_ns()))
            if resume is None:
                run.mkdir()
            tag='failed-cleanup-metadata-'+run.name.split('_')[-1]
            bindings={str(p):self.g.sha_file(p) for p in (catalog_path,protected_path,
                self.g.path('maintenance/'+common.PREFIX+'policy.json'),
                self.g.path('maintenance/storage_control.json'),
                self.g.path('grasp_records/archive_config.json'))}
            plan=dict(schema=1,policy=policy,binding_sha256=bindings,records=records,
                source_sha256=self.g.sha_file(__file__),raw_recordings_uploaded=False)
            if resume is None:
                common.atomic(run/'predelete_plan.json',plan)
            members=[]
            with tarfile.open(run/('metadata.tar.gz' if resume is None else 'resume_check.tar.gz'),'w:gz') as archive:
                for group in records:
                    row=group['row']
                    text='\n\n'.join(label+'：'+row['summary'][key] for label,key in (
                        ('问题发现','problem'),('修复方法','method'),('验证摘要','validation')))+'\n\n原实验结果：失败。\n'
                    import io
                    content=text.encode('utf-8')
                    name='summaries/'+hashlib.sha256(row['record_path'].encode()).hexdigest()+'.md'
                    info=tarfile.TarInfo(name);info.size=len(content);archive.addfile(info,io.BytesIO(content))
                    members.append(dict(path=name,sha256=hashlib.sha256(content).hexdigest(),size_bytes=len(content),generated_summary=True))
                    for item in group['files']:
                        if item['raw_recording']:
                            continue
                        source=self.g.path(item['path'])
                        if self.g.sha_file(source)!=item['sha256']:
                            raise RuntimeError('metadata source changed during packaging')
                        archive.add(source,arcname=item['path'],recursive=False)
                        if self.g.sha_file(source)!=item['sha256']:
                            raise RuntimeError('metadata source changed during packaging')
                        members.append(dict(path=item['path'],sha256=item['sha256'],size_bytes=item['size_bytes']))
            if resume is None:
                common.atomic(run/'metadata_manifest.json',dict(schema=1,members=members,
                    bundle_sha256=self.g.sha_file(run/'metadata.tar.gz'),
                    raw_recordings_included=False,record_count=len(records)))
            else:
                (run/'resume_check.tar.gz').unlink()
            expected={item['path']:item for item in members}
            with tarfile.open(run/'metadata.tar.gz','r:gz') as archive:
                seen=set()
                for member in archive:
                    if not member.isfile() or member.name in seen or member.name not in expected:
                        raise RuntimeError('invalid or duplicate metadata archive member')
                    seen.add(member.name)
                    digest=hashlib.sha256()
                    with archive.extractfile(member) as stream:
                        for block in iter(lambda:stream.read(1024*1024),b''):
                            digest.update(block)
                    if (member.size!=expected[member.name]['size_bytes']
                            or digest.hexdigest()!=expected[member.name]['sha256']):
                        raise RuntimeError('metadata archive does not preserve source bytes')
                if seen!=set(expected):
                    raise RuntimeError('metadata archive is incomplete')
            publisher=self.publisher_factory(policy['metadata_github']['repository'],tag)
            common.atomic(run/'progress.json',dict(state='predelete_upload_pending',tag=tag))
            backups={}
            for name in ('metadata.tar.gz','metadata_manifest.json','predelete_plan.json'):
                backups[name]=publisher.upload(run/name,run.name)
                self.require_upload(backups[name],run/name)
            # All three remote assets have passed streamed SHA256 readback.
            self.require_bindings(bindings)
            common.atomic(run/'progress.json',dict(state='running',tag=tag,backups=backups))
            ledger=dict(schema=1,state='running',policy=policy,initial_free_bytes=initial,
                execution_source_sha256=self.g.sha_file(__file__),
                files=[],skipped=[],deleted_bytes=0,deleted_allocated_bytes=0,
                github_predelete_backup=backups,motion_commands_sent=False,temperature_queried=False)
            previous=signal.signal(signal.SIGIO,lambda *_:setattr(self,'broken',True))
            error=None
            try:
                for group in records:
                    if self.g.free()>=policy['target_free_bytes']:
                        break
                    row=group['row']
                    try:
                        self.delete_record(row,group['files'],run,ledger,bindings)
                        common.atomic(self.g.path(row['record_path'])/INDEX,dict(schema=1,
                            original_result='failure',record_path=row['record_path'],
                            metadata_moved_to_github=True,raw_recording_uploaded=False,
                            release_url=backups['metadata.tar.gz']['release_url'],
                            recovery_index=str(run/'github_recovery_index.json'),
                            predelete_manifest_sha256=backups['predelete_plan.json']['sha256']))
                    except BlockingIOError as exc:
                        ledger['skipped'].append(dict(record_path=row['record_path'],reason=str(exc)))
                    print(json.dumps(dict(record=row['record_path'],free_gib=round(self.g.free()/common.GIB,3))),flush=True)
                state='complete' if self.g.free()>=policy['target_free_bytes'] else 'eligible_exhausted'
            except Exception as exc:
                error=str(exc);state='stopped_on_error'
            finally:
                signal.signal(signal.SIGIO,previous)
            ledger.update(state=state,final_free_bytes=self.g.free(),completed_ns=time.time_ns(),error=error)
            common.atomic(run/'cleanup_receipt.json',ledger)
            common.atomic(run/'progress.json',dict(state='postdelete_upload_pending',tag=tag,
                backups=backups,result_state=state))
            index=self.finish(run,publisher,state,backups)
            result=dict(state=state,receipt=str(run/'cleanup_receipt.json'),
                recovery_index=str(run/'github_recovery_index.json'),release_url=index['release_url'],
                deleted_bytes=ledger['deleted_bytes'],free_bytes=self.g.free(),candidate_records=len(records))
            common.atomic(self.g.maintenance/(common.PREFIX+'status.json'),result)
            if error:
                raise RuntimeError(error)
            return result
