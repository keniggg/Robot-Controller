#!/usr/bin/env python3
"""Maintain SanStand space using only evidence-bound, reviewed failed records."""
import argparse
import datetime as dt
import fcntl
import hashlib
import json
import os
from pathlib import Path
import signal
import stat
import subprocess
import time

MOUNT = Path('/media/zhuyupei/SanStand')
ROOT = MOUNT / '实验记录/Robot-Controller'
GIB = 1 << 30
PREFIX = 'failed_record_space_guard_'


def identity(value):
    return (value.st_dev, value.st_ino, value.st_size, value.st_mtime_ns,
            value.st_mode, value.st_nlink)


def sha(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for data in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(data)
    return digest.hexdigest()


def atomic(path, value):
    path = Path(path)
    data = value if isinstance(value, str) else json.dumps(value, ensure_ascii=False, indent=2) + '\n'
    temp = path.with_name('.' + path.name + '.' + str(os.getpid()) + '.tmp')
    with temp.open('x', encoding='utf-8') as stream:
        stream.write(data)
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temp, path)
    fd = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY)
    try:
        os.fsync(fd)
    finally:
        os.close(fd)


class Guard:
    def __init__(self, root=ROOT, mount=MOUNT):
        self.root, self.mount = Path(root), Path(mount)
        self.maintenance = self.root / 'maintenance'
        self.broken = False
        self.device = None

    def path(self, relative):
        relative = Path(relative)
        if relative.is_absolute() or not relative.parts or any(p in ('.', '..') for p in relative.parts):
            raise ValueError('noncanonical relative path')
        path = self.root / relative
        if any(p.is_symlink() for p in (path, *path.parents)) or path.resolve() != path:
            raise ValueError('symlink or noncanonical path')
        return path

    def free(self):
        values = os.statvfs(self.mount)
        return values.f_bavail * values.f_frsize

    sha_file = staticmethod(sha)

    def mounted(self, policy):
        if not self.mount.is_mount():
            raise RuntimeError('EXPERIMENT_STORAGE_UNAVAILABLE: SanStand not mounted')
        fields = subprocess.check_output(['findmnt', '-n', '-o', 'UUID,FSTYPE,OPTIONS',
                                         '-T', str(self.mount)], text=True).split()
        if len(fields) != 3 or fields[0] != policy['mount_uuid'] or fields[1] != 'exfat' or 'rw' not in fields[2].split(','):
            raise RuntimeError('SanStand mount identity or write mode changed')
        device = self.mount.stat().st_dev
        if self.device is not None and device != self.device:
            raise RuntimeError('SanStand device changed')
        self.device = device

    def available(self):
        if not self.mount.is_mount():
            raise RuntimeError('EXPERIMENT_STORAGE_UNAVAILABLE: SanStand not mounted')

    def controls(self):
        self.available()
        policy = json.loads(self.path('maintenance/' + PREFIX + 'policy.json').read_text())
        assert policy['schema'] in (1, 2, 3)
        assert type(policy['enabled']) is bool
        assert policy['trigger_free_bytes'] == 10 * GIB
        if policy['schema'] == 3:
            assert policy['target_free_bytes'] == 80 * GIB
            assert policy['metadata_retention'] == 'local_only'
            assert policy['metadata_github']['enabled'] is False
            assert policy['metadata_github']['delete_local_metadata_after_verified'] is False
        elif policy['schema'] == 2:
            assert policy['target_free_bytes'] == 80 * GIB
            cloud = policy['metadata_github']
            assert cloud['enabled'] is True and cloud['repository'] == 'keniggg/Robot-Controller'
            assert all(cloud[k] is True for k in ('predelete_verification_required',
                'postdelete_receipt_upload_required', 'delete_local_metadata_after_verified'))
        else:
            # Old isolated/historical fixtures keep their original semantics.
            # Mounted production storage must use the latest authorized policy.
            assert self.root != ROOT and policy['target_free_bytes'] == 70 * GIB
        assert policy['mount_uuid'] == '4899-D74E' and policy['authorization'].strip()
        self.mounted(policy)
        control = json.loads(self.path('maintenance/storage_control.json').read_text())
        archive = json.loads(self.path('grasp_records/archive_config.json').read_text())
        assert control['archive_policy'] == archive['archive_policy'] == 'local_only'
        assert archive['upload_enabled'] is False and archive['delete_enabled'] is False
        if not policy['enabled'] or control['state'] != 'resumed':
            return None
        return policy

    def record(self, row, protection):
        assert row['classification'] == 'failure'
        assert all(row[k] is True for k in ('closed_verified', 'non_success_verified', 'operator_success_conflict_checked'))
        assert row['calibration'] is False and row['current_investigation_required'] is False
        directory = self.path(row['record_path'])
        assert directory.is_dir()
        for entry in protection['protected_paths']:
            protected = self.path(entry['path'])
            if directory == protected or protected in directory.parents:
                raise ValueError('record is protected: ' + row['record_path'])
        assert len(row['evidence']) >= 2
        for item in row['evidence']:
            assert item['finding'].strip()
            evidence = self.path(item['path'])
            assert evidence.is_file() and sha(evidence) == item['sha256'], item['path']
        assert all(row['summary'][k].strip() for k in ('problem', 'method', 'validation'))
        if row['formal_record']:
            manifest = json.loads((directory / 'manifest.json').read_text())
            outcome = json.loads((directory / 'recorder_outcome.json').read_text())
            summary = json.loads((directory / 'result_summary.json').read_text())
            assert manifest['state'] == 'complete' and manifest['recorder_closed'] is True
            assert manifest['result'] == summary['result'] == 'failure' and outcome['bag_closed'] is True
        # Independently recorded user success always wins over automated failure.
        physical = directory / 'physical_result_user_confirmed.json'
        if physical.exists():
            observation = json.loads(physical.read_text())
            if observation.get('physical_grip_confirmed_by_user') is not False:
                raise ValueError('user physical result is successful or uncertain')
        return directory

    def discover_pre_motion(self, catalog, protection):
        """Only mature, explicitly rejected pre-start failures can self-classify.

        Active attempts still require independent review and an explicit entry.
        Recent records and the current progress record remain protected.
        """
        known = {r['record_path'] for r in catalog['records']}
        protected = [self.path(p['path']) for p in protection['protected_paths']]
        recent = dt.datetime.now(dt.timezone.utc) - dt.timedelta(days=7)
        last_records = set()
        blocked_intervals = []
        for manifest_path in (self.root/'grasp_records').glob('*/manifest.json'):
            try:
                value=json.loads(manifest_path.read_text())
                observation=manifest_path.parent/'physical_result_user_confirmed.json'
                physical=observation.exists() and json.loads(observation.read_text()).get('physical_grip_confirmed_by_user') is not False
                if value.get('result')!='failure' or physical:
                    start_text, end_text = value.get('started_at'), value.get('completed_at')
                    # An active/unknown record commonly has completed_at=null.
                    # Its conflict interval cannot be closed: decline all new
                    # classifications, while retaining the reviewed catalog.
                    if not isinstance(start_text, str) or not isinstance(end_text, str):return False
                    start=dt.datetime.fromisoformat(start_text.replace('Z','+00:00'))
                    end=dt.datetime.fromisoformat(end_text.replace('Z','+00:00'))
                    if start.tzinfo is None or end.tzinfo is None:return False
                    blocked_intervals.append((start,end))
            except (ValueError,KeyError,OSError):return False
        for progress in (self.root/'ros_log').glob('*/current_progress.json'):
            try:
                value=json.loads(progress.read_text())
                for key in ('last_record','record_dir'):
                    if value.get(key):last_records.add(Path(value[key]))
            except (ValueError,OSError):
                # Missing/corrupt progress cannot establish a new safe scope.
                return False
        added = False
        for directory in sorted((self.root/'grasp_records').iterdir()):
            relative=str(directory.relative_to(self.root))
            if (not directory.is_dir() or directory.is_symlink() or relative in known
                    or directory in last_records
                    or any(directory==p or p in directory.parents for p in protected)):
                continue
            names=('manifest.json','result_summary.json','recorder_outcome.json','command_result.json','runner.log')
            if not all((directory/n).is_file() and not (directory/n).is_symlink() for n in names):continue
            try:
                manifest=json.loads((directory/'manifest.json').read_text())
                outcome=json.loads((directory/'recorder_outcome.json').read_text())
                summary=json.loads((directory/'result_summary.json').read_text())
                start_text, end_text = manifest.get('started_at'), manifest.get('completed_at')
                if not isinstance(start_text, str) or not isinstance(end_text, str):continue
                started=dt.datetime.fromisoformat(start_text.replace('Z','+00:00'))
                if started.tzinfo is None or started>recent:continue
                ended=dt.datetime.fromisoformat(end_text.replace('Z','+00:00'))
                if ended.tzinfo is None or any(started<=b and a<=ended for a,b in blocked_intervals):continue
                if (manifest.get('state')!='complete' or manifest.get('result')!='failure'
                        or manifest.get('recorder_closed') is not True or summary.get('result')!='failure'
                        or outcome.get('bag_closed') is not True or outcome.get('active_seen') is not False
                        or not manifest.get('failure_reason','').startswith('NO_FRESH_MODE_PREVIEW')):continue
                if manifest.get('result_evidence',{}).get('observations')!=[]:continue
                if any(directory.glob('*physical*')) or any(directory.glob('*operator*result*')):continue
                info=next(i for i in manifest['files'] if i['path']=='keyframes.bag')
                raw=directory/'keyframes.bag'
                if not raw.is_file() or raw.is_symlink() or raw.stat().st_size!=info['size_bytes']:continue
                runner=(directory/'runner.log').read_text(errors='replace')
                if 'NO_FRESH_MODE_PREVIEW' not in runner:continue
                catalog['records'].append(dict(record_path=relative,started_at=manifest['started_at'],
                    classification='failure',closed_verified=True,non_success_verified=True,
                    operator_success_conflict_checked=True,calibration=False,current_investigation_required=False,
                    formal_record=True,evidence=[dict(path=str((directory/n).relative_to(self.root)),
                        sha256=sha(directory/n),finding='完整封口、NO_FRESH_MODE_PREVIEW且active_seen=false，无自动动作或实物结果冲突标记') for n in names],
                    raw_files=[dict(path='keyframes.bag',kind='bag',size_bytes=info['size_bytes'],sha256=info['sha256'])],
                    summary=dict(problem=manifest['failure_reason'],method='核对输入时效、selection及预览接纳；未获得任务启动资格。',
                        validation='原结果failure、两个封口凭证一致，active_seen=false。仅归类启动前失败，不声称修复或夹持成功。')))
                added=True
            except (ValueError,KeyError,StopIteration,OSError):continue
        return added

    def delete(self, row, item, protection, run, receipt, binding):
        self.mounted(receipt['policy'])
        assert self.controls() is not None, 'maintenance paused'
        for path, digest in binding.items():
            assert sha(path) == digest, 'catalog or protection changed'
        directory = self.record(row, protection)
        path = self.path(str(Path(row['record_path']) / item['path']))
        assert directory in path.parents and item['kind'] == 'bag' and path.suffix == '.bag'
        expected = item['sha256']
        if expected is None:
            assert row['formal_record'] is False and item['reviewed_identity']
        else:
            assert len(expected) == 64 and all(c in '0123456789abcdef' for c in expected)
        if not path.exists():
            return
        descriptor = os.open(path, os.O_RDONLY | os.O_NONBLOCK | os.O_NOFOLLOW)
        leased = False
        moved = None
        self.broken = False
        try:
            original = os.fstat(descriptor)
            assert stat.S_ISREG(original.st_mode) and original.st_nlink == 1
            assert original.st_size == item['size_bytes'] and original.st_dev == self.device
            if item.get('reviewed_identity'):
                bound = item['reviewed_identity']
                assert (original.st_dev, original.st_ino, original.st_size, original.st_mtime_ns) == (
                    bound['device'], bound['inode'], bound['size_bytes'], bound['mtime_ns'])
            fcntl.fcntl(descriptor, fcntl.F_SETOWN, os.getpid())
            # A write lease cannot coexist with any other open description,
            # including descriptors hidden by /proc permissions. SIGIO cancels
            # deletion when a new opener requests a lease break.
            fcntl.fcntl(descriptor, fcntl.F_SETLEASE, fcntl.F_WRLCK)
            leased = True
            digest = hashlib.sha256()
            while True:
                data = os.read(descriptor, 8 * 1024 * 1024)
                if self.broken:
                    raise RuntimeError('raw file requested by another process')
                if not data:
                    break
                digest.update(data)
            actual_sha = digest.hexdigest()
            assert expected is None or actual_sha == expected, 'raw SHA256 differs from reviewed evidence'
            assert identity(os.fstat(descriptor)) == identity(original) == identity(path.stat())
            self.record(row, protection)
            for source, value in binding.items():
                assert sha(source) == value
            self.mounted(receipt['policy'])
            assert self.controls() is not None
            text = ('# 失败实验空间清理摘要\n\n原实验结果：失败。\n\n'
                    '问题发现：' + row['summary']['problem'] + '\n\n'
                    '解决思路与方法：' + row['summary']['method'] + '\n\n'
                    '验证结果：' + row['summary']['validation'] + '\n\n'
                    '原结果、授权、日志、审计及manifest保留；原始录制缺失由本次精确回执解释。\n')
            summary_path = directory / ('空间清理摘要_' + run.name + '.md')
            if not summary_path.exists():
                atomic(summary_path, text)
            retained = {str(p.relative_to(self.root)): sha(p) for p in directory.rglob('*')
                        if p.is_file() and not p.is_symlink() and p.suffix in
                        ('.json', '.jsonl', '.md', '.log', '.txt', '.yaml', '.yml')}
            quarantine = run / 'quarantine'
            quarantine.mkdir(exist_ok=True)
            destination = quarantine / (str(len(receipt['files'])) + '.bag')
            assert not destination.exists()
            entry = dict(record_path=row['record_path'], path=str(path.relative_to(self.root)),
                         sha256=actual_sha, expected_historical_sha256=expected, size_bytes=original.st_size,
                         allocated_bytes=original.st_blocks * 512, inode=original.st_ino,
                         device=original.st_dev, state='intent', quarantine=str(destination.relative_to(self.root)),
                         retained_sha256=retained, evidence=row['evidence'])
            receipt['files'].append(entry)
            atomic(run / 'receipt.json', receipt)
            record_receipt = directory / ('空间清理回执_' + run.name + '_' + str(len(receipt['files'])) + '.json')
            atomic(record_receipt, entry)
            assert not self.broken and fcntl.fcntl(descriptor, fcntl.F_GETLEASE) == fcntl.F_WRLCK
            assert identity(path.stat()) == identity(original)
            os.rename(path, destination)
            moved = destination
            assert identity(destination.stat()) == identity(original) and not self.broken
            assert fcntl.fcntl(descriptor, fcntl.F_GETLEASE) == fcntl.F_WRLCK
            self.mounted(receipt['policy'])
            assert self.controls() is not None
            for source, value in binding.items():
                assert sha(source) == value
            destination.unlink()
            moved = None
            entry.update(state='deleted', deleted_at=dt.datetime.now(dt.timezone.utc).isoformat())
            receipt['deleted_bytes'] += original.st_size
            receipt['deleted_allocated_bytes'] += original.st_blocks * 512
            atomic(record_receipt, entry)
            atomic(run / 'receipt.json', receipt)
            assert all(sha(self.path(name)) == value for name, value in retained.items())
            entry['retained_hashes_unchanged'] = True
            atomic(record_receipt, entry)
            atomic(run / 'receipt.json', receipt)
            print(json.dumps(dict(deleted=entry['path'], free_gib=round(self.free() / GIB, 3))), flush=True)
        except Exception:
            if moved is not None and moved.exists() and not path.exists():
                os.rename(moved, path)
            raise
        finally:
            if leased:
                fcntl.fcntl(descriptor, fcntl.F_SETLEASE, fcntl.F_UNLCK)
            os.close(descriptor)

    def run(self, apply=False):
        policy = self.controls()
        if policy is None:
            return dict(state='paused')
        if policy['schema'] == 2:
            from migrate_failed_record_metadata import Cleanup
            return Cleanup(self).run(policy, apply)
        with (self.maintenance / (PREFIX + 'lock')).open('a') as lock:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            initial = self.free()
            if initial >= policy['trigger_free_bytes']:
                return dict(state='above_trigger', free_bytes=initial)
            catalog_path = self.path('maintenance/' + PREFIX + 'catalog.json')
            protected_path = self.path('maintenance/' + PREFIX + 'protected.json')
            catalog, protection = (json.loads(p.read_text()) for p in (catalog_path, protected_path))
            assert catalog['schema'] == protection['schema'] == 1
            assert catalog['review_id'].strip()
            if apply and policy.get('auto_catalog_pre_motion_failures',False):
                if self.discover_pre_motion(catalog,protection):
                    atomic(catalog_path,catalog)
            records = sorted(catalog['records'], key=lambda r: (dt.datetime.fromisoformat(
                r['started_at'].replace('Z', '+00:00')), r['record_path']))
            raw_paths = [str(Path(r['record_path']) / i['path']) for r in records for i in r['raw_files']]
            assert len(raw_paths) == len(set(raw_paths)), 'duplicate raw paths'
            records = [r for r in records if any(self.path(str(Path(r['record_path'])/i['path'])).exists()
                                                for i in r['raw_files'])]
            for old in self.maintenance.glob('user_failed_cleanup_space_*/receipt.json'):
                prior = json.loads(old.read_text())
                if prior.get('state') == 'running' or any(i['state'] == 'intent' for i in prior.get('files', [])):
                    raise RuntimeError('unresolved cleanup intent requires recovery: ' + str(old))
            for row in records:
                self.record(row, protection)
            available = sum(i['size_bytes'] for r in records for i in r['raw_files']
                            if self.path(str(Path(r['record_path']) / i['path'])).exists())
            if not apply:
                return dict(state='reviewed_plan', candidate_records=len(records),
                            candidate_bytes=available, free_bytes=initial,
                            target_free_bytes=policy['target_free_bytes'])
            timestamp = dt.datetime.now(dt.timezone.utc).strftime('%Y%m%dT%H%M%SZ')
            run = self.maintenance / ('user_failed_cleanup_space_' + timestamp + '_' + str(os.getpid()))
            run.mkdir()
            binding = {p: sha(p) for p in (catalog_path, protected_path,
                self.path('maintenance/' + PREFIX + 'policy.json'))}
            receipt = dict(schema=1, state='running', policy=policy, review_id=catalog['review_id'],
                           source_sha256=sha(Path(__file__)), binding_sha256={str(p): h for p, h in binding.items()},
                           initial_free_bytes=initial, target_free_bytes=policy['target_free_bytes'],
                           deleted_bytes=0, deleted_allocated_bytes=0, files=[], skipped=[],
                           motion_commands_sent=False, temperature_queried=False)
            atomic(run / 'plan.json', dict(records=records, protection=protection))
            atomic(run / 'receipt.json', receipt)
            previous_handler = signal.signal(signal.SIGIO, lambda *_: setattr(self, 'broken', True))
            try:
                for row in records:
                    for item in row['raw_files']:
                        if self.free() >= policy['target_free_bytes']:
                            break
                        try:
                            self.delete(row, item, protection, run, receipt, binding)
                        except BlockingIOError as error:
                            receipt['skipped'].append(dict(record_path=row['record_path'], path=item['path'], reason=str(error)))
                            atomic(run / 'receipt.json', receipt)
                    if self.free() >= policy['target_free_bytes']:
                        break
                receipt['state'] = 'complete' if self.free() >= policy['target_free_bytes'] else 'eligible_exhausted'
            except Exception as error:
                receipt.update(state='stopped_on_error', error=repr(error))
                raise
            finally:
                signal.signal(signal.SIGIO, previous_handler)
                receipt.update(final_free_bytes=self.free(), completed_at=dt.datetime.now(dt.timezone.utc).isoformat())
                atomic(run / 'receipt.json', receipt)
                atomic(self.maintenance / (PREFIX + 'status.json'), dict(
                    state=receipt['state'], receipt=str(run / 'receipt.json'),
                    free_bytes=receipt['final_free_bytes'], deleted_bytes=receipt['deleted_bytes']))
            return dict(state=receipt['state'], receipt=str(run / 'receipt.json'),
                        deleted_bytes=receipt['deleted_bytes'], free_bytes=receipt['final_free_bytes'])


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--apply', action='store_true')
    args = parser.parse_args()
    print(json.dumps(Guard().run(args.apply), ensure_ascii=False), flush=True)
