#!/usr/bin/env python3
"""Seal the user's local-only policy and remove only completed synthetic fixtures."""
import argparse
import fcntl
import hashlib
import json
import os
from pathlib import Path
import signal
import stat
import subprocess
import time

WORK = Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
MOUNT = Path('/media/zhuyupei/SanStand')
ROOT = MOUNT / '实验记录/Robot-Controller'
MAINT = ROOT / 'maintenance'
VALIDATIONS = (
    '1791953263886197699', '1791953295601802237', '1791953429649887704',
    '1791953917851751563', '1791955641947397401',
)
LATEST = MAINT / 'failed_metadata_migration_validation_1791955641947397401'
USER_LOCAL = '先不上传到github，就保存在sanstand吧，能清理多少使用空间就清理多少使用空间'
USER_EXISTING = '已经迁移的就不管了'


def sha(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1048576), b''):
            digest.update(chunk)
    return digest.hexdigest()


def atomic(path, value):
    temporary = path.with_name('.' + path.name + '.' + str(os.getpid()))
    with temporary.open('x', encoding='utf-8') as stream:
        stream.write(json.dumps(value, ensure_ascii=False, indent=2) + '\n')
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary, path)
    descriptor = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY)
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def free():
    values = os.statvfs(MOUNT)
    return values.f_bavail * values.f_frsize


def mounted():
    assert MOUNT.is_mount()
    fields = subprocess.check_output(
        ['findmnt', '-n', '-o', 'UUID,FSTYPE,OPTIONS', '-T', str(MOUNT)], text=True).split()
    assert fields[:2] == ['4899-D74E', 'exfat'] and 'rw' in fields[2].split(',')
    assert ROOT.stat().st_dev == MOUNT.stat().st_dev


def identity(value):
    return [value.st_dev, value.st_ino, value.st_size, value.st_mtime_ns,
            value.st_mode, value.st_nlink]


def baseline():
    paths = [MAINT / name for name in (
        'failed_record_space_guard_catalog.json', 'failed_record_space_guard_protected.json',
        'storage_control.json')]
    paths += [ROOT / 'grasp_records/archive_config.json',
              ROOT / 'ros_log/unknown_grasp_20260923/archive_config.json']
    # Existing migrations, restored material and their staging are outside the deletion scope.
    paths += [p for p in (MAINT / 'user_failed_metadata_cleanup_1791953606120759822').rglob('*')
              if p.is_file()]
    restore_receipt = MAINT / 'local_metadata_restore_1791955414686771130/receipt.json'
    paths += [restore_receipt]
    restored = json.loads(restore_receipt.read_text())
    for row in restored['restored'] + restored['already_present']:
        path = ROOT / row['path']
        assert path.resolve().is_relative_to(ROOT) if hasattr(Path, 'is_relative_to') else ROOT in path.resolve().parents
        assert not path.is_symlink() and path.stat().st_size == row['size_bytes']
        assert sha(path) == row['sha256']
        paths.append(path)
    for suffix in VALIDATIONS:
        directory = MAINT / ('failed_metadata_migration_validation_' + suffix)
        paths += [p for p in directory.iterdir() if p.is_file()]
    return {str(p): sha(p) for p in paths}


def activate():
    mounted()
    result_path = LATEST / 'result.json'
    result = json.loads(result_path.read_text())
    assert result['passed'] is True and result['source_unchanged'] is True
    assert result['summary'] == dict(tests=100, failures=0, errors=0, skipped=0)
    for name, digest in result['source_sha256'].items():
        assert sha(WORK / name) == digest, name
    for name, digest in result['artifact_sha256'].items():
        assert sha(LATEST / name) == digest, name
    policy_path = MAINT / 'failed_record_space_guard_policy.json'
    policy = json.loads(policy_path.read_text())
    assert policy['schema'] == 2 and policy['enabled'] is False and policy.get('user_hold')
    control = json.loads((MAINT / 'storage_control.json').read_text())
    assert control['state'] == 'resumed' and control['archive_policy'] == 'local_only'
    for config in (ROOT / 'grasp_records/archive_config.json',
                   ROOT / 'ros_log/unknown_grasp_20260923/archive_config.json'):
        value = json.loads(config.read_text())
        assert value['upload_enabled'] is False and value['delete_enabled'] is False
        assert value['archive_policy'] == 'local_only'
    run = MAINT / ('user_failed_cleanup_local_only_final_' + str(time.time_ns()))
    run.mkdir()
    atomic(run / 'policy_before.json', policy)
    preserved = baseline()
    atomic(run / 'preserved_sha256.json', preserved)
    policy.update(schema=3, enabled=True, metadata_retention='local_only',
                  existing_migration_handling='leave_current_state', updated_ns=time.time_ns(),
                  authorization=USER_LOCAL + '；' + USER_EXISTING + '。沿用低于10 GiB触发、80 GiB可用空间目标与合格记录不足即停止的规则。',
                  authorization_source=dict(source='actual_user_message', platform_message_id=None,
                      text=USER_LOCAL, clarification=USER_EXISTING,
                      recorded_time_kind='locally_recorded_receipt_time'),
                  validated_implementation=dict(result=str(result_path), result_sha256=sha(result_path),
                      source_sha256=result['source_sha256'], tests=100, recorded_ns=time.time_ns()))
    previous_clarification = policy.pop('authorization_clarification', None)
    if previous_clarification is not None:
        policy['withdrawn_upload_authorization_clarification'] = previous_clarification
    policy.pop('user_hold')
    policy['metadata_github'].update(enabled=False, delete_local_metadata_after_verified=False,
        predelete_verification_required=False, postdelete_receipt_upload_required=False,
        scope='No further uploads or local metadata deletion; existing migrated material left unchanged.')
    atomic(policy_path, policy)
    atomic(run / 'activation.json', dict(state='local_only_activated', policy=str(policy_path),
        policy_sha256=sha(policy_path), validation=str(result_path), validation_sha256=sha(result_path),
        source_sha256=result['source_sha256'], preserved_sha256=str(run / 'preserved_sha256.json'),
        user_text=[USER_LOCAL, USER_EXISTING], initial_free_bytes=free(),
        github_contacted=False, motion_commands_sent=False, temperature_queried=False))
    atomic(MAINT / 'failed_record_local_only_release_20261014.json', dict(
        state='validated_local_only', run=str(run), tests=100, policy_sha256=sha(policy_path),
        source_sha256=result['source_sha256'], validation=str(result_path),
        validation_sha256=sha(result_path), existing_migration_handling='leave_current_state'))
    print(json.dumps(dict(run=str(run), state='local_only_activated'), ensure_ascii=False))


def fixtures(run):
    mounted()
    assert run.parent == MAINT and run.name.startswith('user_failed_cleanup_local_only_final_')
    saved = json.loads((run / 'preserved_sha256.json').read_text())
    for name, digest in saved.items():
        assert sha(Path(name)) == digest, name
    records = []
    directories = []
    for suffix in VALIDATIONS:
        base = MAINT / ('failed_metadata_migration_validation_' + suffix)
        assert (base / 'result.json').is_file()
        for name in ('pytest_tmp', 'tmp'):
            directory = base / name
            if not directory.exists():
                continue
            assert not directory.is_symlink() and directory.is_dir()
            directories.append(directory)
            for path in sorted(directory.rglob('*')):
                value = path.lstat()
                assert not path.is_symlink() and path.resolve() == path
                assert value.st_dev == MOUNT.stat().st_dev and value.st_uid == os.getuid()
                if stat.S_ISDIR(value.st_mode):
                    continue
                assert stat.S_ISREG(value.st_mode) and value.st_nlink == 1, str(path)
                records.append(dict(path=str(path), identity=identity(value),
                    sha256=sha(path), size_bytes=value.st_size, allocated_bytes=value.st_blocks * 512))
    identities = {(r['identity'][0], r['identity'][1]) for r in records}
    open_files = []
    for process in Path('/proc').iterdir():
        if not process.name.isdigit():
            continue
        try:
            for descriptor in (process / 'fd').iterdir():
                try:
                    value = descriptor.stat()
                    if (value.st_dev, value.st_ino) in identities:
                        open_files.append(dict(pid=int(process.name), fd=descriptor.name))
                except (OSError, PermissionError):
                    pass
        except (OSError, PermissionError):
            pass
    assert not open_files, open_files
    atomic(run / 'synthetic_fixture_plan.json', dict(scope='Only completed unit-test synthetic fixtures',
        directories=[str(d) for d in directories], files=records, open_files=open_files,
        protected_logs_and_existing_migrations=str(run / 'preserved_sha256.json')))
    broken = [False]
    prior = signal.signal(signal.SIGIO, lambda *_: broken.__setitem__(0, True))
    removed, logical, allocated = 0, 0, 0
    initial = free()
    try:
        with (run / 'synthetic_fixture_deletions.jsonl').open('x', encoding='utf-8') as journal:
            for row in records:
                mounted()
                path = Path(row['path'])
                descriptor = os.open(path, os.O_RDWR | os.O_NOFOLLOW)
                try:
                    fcntl.fcntl(descriptor, fcntl.F_SETLEASE, fcntl.F_WRLCK)
                    assert not broken[0] and identity(os.fstat(descriptor)) == row['identity']
                    assert identity(path.lstat()) == row['identity']
                    with os.fdopen(os.dup(descriptor), 'rb') as stream:
                        hasher = hashlib.sha256()
                        for chunk in iter(lambda: stream.read(1048576), b''):
                            hasher.update(chunk)
                        digest = hasher.hexdigest()
                    assert digest == row['sha256'] and not broken[0]
                    journal.write(json.dumps(dict(state='intent', **row), ensure_ascii=False) + '\n')
                    journal.flush()
                    os.fsync(journal.fileno())
                    assert not broken[0] and identity(path.lstat()) == row['identity']
                    path.unlink()
                    directory_descriptor = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY)
                    try:
                        os.fsync(directory_descriptor)
                    finally:
                        os.close(directory_descriptor)
                    journal.write(json.dumps(dict(state='deleted', **row), ensure_ascii=False) + '\n')
                    journal.flush()
                    os.fsync(journal.fileno())
                    removed += 1
                    logical += row['size_bytes']
                    allocated += row['allocated_bytes']
                finally:
                    os.close(descriptor)
            for directory in directories:
                for child in sorted((p for p in directory.rglob('*') if p.is_dir()),
                                    key=lambda p: len(p.parts), reverse=True):
                    child.rmdir()
                directory.rmdir()
        for name, digest in saved.items():
            assert sha(Path(name)) == digest, name
        receipt = dict(state='complete', initial_free_bytes=initial, final_free_bytes=free(),
            deleted_files=removed, deleted_bytes=logical, deleted_allocated_bytes=allocated,
            preserved_sha256_unchanged=True, experiments_deleted=False, github_contacted=False,
            existing_migrations_unchanged=True, completed_ns=time.time_ns())
        atomic(run / 'synthetic_fixture_receipt.json', receipt)
        print(json.dumps(dict(run=str(run), **receipt), ensure_ascii=False))
    finally:
        signal.signal(signal.SIGIO, prior)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('phase', choices=('activate', 'fixtures'))
    parser.add_argument('--run', type=Path)
    args = parser.parse_args()
    if args.phase == 'activate':
        activate()
    else:
        assert args.run is not None
        fixtures(args.run)
