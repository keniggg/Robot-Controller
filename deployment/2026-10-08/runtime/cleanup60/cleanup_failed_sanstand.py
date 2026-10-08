#!/usr/bin/env python3
"""One explicitly authorized, bounded cleanup; no ROS or archive worker calls."""
import argparse
import datetime as dt
import hashlib
import json
import os
from pathlib import Path
import stat

MOUNT = Path('/media/zhuyupei/SanStand')
ROOT = MOUNT / '实验记录/Robot-Controller'
BULK = {'.bag'}  # This round needs only old failed bags; retain all images and fixtures.
GIB = 1 << 30


def now():
    return dt.datetime.now(dt.timezone.utc).isoformat()


def digest(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024*1024), b''):
            h.update(block)
    return h.hexdigest()


def free_bytes():
    s = os.statvfs(str(MOUNT))
    return s.f_bavail * s.f_frsize


def checked_path(path):
    p = Path(path)
    if not p.is_absolute() or ROOT not in p.parents:
        raise RuntimeError('path outside authorized experiment root: ' + str(p))
    if any(x.is_symlink() for x in (p, *p.parents)):
        raise RuntimeError('symlink rejected: ' + str(p))
    if p.resolve() != p:
        raise RuntimeError('noncanonical path rejected: ' + str(p))
    return p


def open_paths():
    paths = set()
    for f in Path('/proc').glob('[0-9]*/fd/*'):
        try:
            target = os.readlink(str(f))
            if target.startswith(str(ROOT) + '/'):
                paths.add(target.removesuffix(' (deleted)') if hasattr(target, 'removesuffix') else target.replace(' (deleted)', ''))
        except OSError:
            pass
    return paths


def identity(path):
    s = Path(path).lstat()
    if not stat.S_ISREG(s.st_mode) or s.st_nlink != 1:
        raise RuntimeError('not a regular file: ' + str(path))
    return dict(size=s.st_size, allocated=s.st_blocks * 512, mtime_ns=s.st_mtime_ns, device=s.st_dev, inode=s.st_ino)


def atomic(path, value):
    data = value if isinstance(value, str) else json.dumps(value, ensure_ascii=False, indent=2) + '\n'
    p = Path(path)
    t = p.with_name('.' + p.name + '.cleanup-tmp')
    with t.open('x', encoding='utf-8') as f:
        f.write(data)
        f.flush()
        os.fsync(f.fileno())
    os.replace(str(t), str(p))
    fd = os.open(str(p.parent), os.O_RDONLY | os.O_DIRECTORY)
    try:
        os.fsync(fd)
    finally:
        os.close(fd)


def validate_record(record, opened, prepared=False):
    d = checked_path(record['path'])
    if not d.is_dir() or record['classification'] != 'failed' or not record['evidence']:
        raise RuntimeError('record is not independently classified failed')
    if any(p == str(d) or p.startswith(str(d) + '/') for p in opened):
        raise RuntimeError('record has an open descriptor: ' + str(d))
    if not prepared and any((d / n).exists() for n in ('实验简要记录.md', 'user_cleanup_receipt.json')):
        raise RuntimeError('existing cleanup/summary record will not be overwritten')
    for evidence in record['bound_evidence']:
        p = checked_path(evidence['path'])
        if digest(p) != evidence['sha256']:
            raise RuntimeError('classification evidence changed: ' + str(p))
    for item in record['delete_files']:
        p = checked_path(item['path'])
        if d not in p.parents or p.suffix.lower() not in BULK:
            raise RuntimeError('not an allowlisted raw file in this record: ' + str(p))
        if identity(p) != item['identity']:
            raise RuntimeError('raw file changed: ' + str(p))
    for item in record['keep_files']:
        if identity(checked_path(item['path'])) != item['identity']:
            raise RuntimeError('retained file changed before cleanup')
    if not record.get('summary_markdown'):
        raise RuntimeError('missing problem/method/validation summary')
    if record.get('formal_record'):
        m = json.loads((d / 'manifest.json').read_text())
        if m.get('result') != 'failure':
            raise RuntimeError('formal recording result is not failure')
        if m.get('state') != 'complete' or m.get('recorder_closed') is not True:
            raise RuntimeError('formal recording is not closed and complete')


def check_mount(device):
    if not os.path.ismount(str(MOUNT)) or MOUNT.stat().st_dev != device:
        raise RuntimeError('SanStand mount changed or disconnected')


def execute(plan_path, expected_sha):
    if not os.path.ismount(str(MOUNT)):
        raise RuntimeError('SanStand is not mounted')
    if digest(plan_path) != expected_sha:
        raise RuntimeError('reviewed plan SHA mismatch')
    plan = json.loads(Path(plan_path).read_text())
    check_mount(plan['mount_device'])
    for path, sha in plan.get('input_sha256', {}).items():
        if digest(path) != sha:
            raise RuntimeError('reviewed input changed: ' + path)
    if plan['target_available_bytes'] != 60 * GIB or plan['max_available_bytes'] != 63 * GIB:
        raise RuntimeError('cleanup bound changed')
    if plan['records'] != sorted(plan['records'], key=lambda r: (r['started_at'], r['path'])):
        raise RuntimeError('records not oldest first')
    if any(r['path'] in plan.get('protect_records', []) for r in plan['records']):
        raise RuntimeError('protected record in cleanup plan')
    paths = [i['path'] for r in plan['records'] for i in r['delete_files']]
    if len(paths) != len(set(paths)):
        raise RuntimeError('duplicate deletion path')
    opened = open_paths()
    for record in plan['records']:
        validate_record(record, opened)
    run = checked_path(plan['external_run_path'])
    run.mkdir(parents=True, exist_ok=False)
    atomic(run / 'plan.json', plan)
    receipt = dict(schema=1, authorization=plan['authorization'], started_at=now(), plan_sha256=expected_sha,
                   initial_available_bytes=free_bytes(), target_available_bytes=60*GIB,
                   deleted_allocated_bytes=0, deleted_file_count=0, records=[], state='running')
    atomic(run / 'receipt.json', receipt)
    try:
        for record in plan['records']:
            check_mount(plan['mount_device'])
            if free_bytes() >= 60 * GIB:
                break
            validate_record(record, open_paths())
            d = Path(record['path'])
            atomic(d / '实验简要记录.md', record['summary_markdown'])
            retained = [dict(path=item['path'], sha256=digest(item['path'])) for item in record['keep_files']]
            r = dict(record_id=d.name, path=str(d), started_at=record['started_at'], state='prepared',
                     summary_sha256=digest(d / '实验简要记录.md'), retained=retained, deleted=[],
                     source_evidence=record['evidence'], note='Raw data deletion explicitly authorized by user; historical manifests remain unchanged.')
            atomic(d / 'user_cleanup_receipt.json', r)
            receipt['records'].append(r)
            atomic(run / 'receipt.json', receipt)
            pending = record['delete_files']
            for batch_start in range(0, len(pending), 25):
                if free_bytes() >= 60 * GIB:
                    break
                batch = pending[batch_start:batch_start + 25]
                r['pending_delete'] = batch
                r['state'] = 'deleting'
                atomic(d / 'user_cleanup_receipt.json', r)
                atomic(run / 'receipt.json', receipt)
                for item in batch:
                    check_mount(plan['mount_device'])
                    if free_bytes() >= 60 * GIB:
                        break
                    p = checked_path(item['path'])
                    if free_bytes() + item['identity']['allocated'] > 63 * GIB:
                        raise RuntimeError('next raw file exceeds bounded cleanup')
                    if any(q == str(d) or q.startswith(str(d) + '/') for q in open_paths()):
                        raise RuntimeError('record became active')
                    if identity(p) != item['identity']:
                        raise RuntimeError('file changed immediately before deletion')
                    p.unlink()
                    r['deleted'].append(item)
                    receipt['deleted_allocated_bytes'] += item['identity']['allocated']
                    receipt['deleted_file_count'] += 1
                r.pop('pending_delete', None)
                atomic(d / 'user_cleanup_receipt.json', r)
                atomic(run / 'receipt.json', receipt)
            for item in retained:
                if digest(item['path']) != item['sha256']:
                    raise RuntimeError('retained metadata digest changed')
            r['state'] = 'complete'
            r['completed_at'] = now()
            r['available_bytes_after'] = free_bytes()
            atomic(d / 'user_cleanup_receipt.json', r)
            atomic(run / 'receipt.json', receipt)
            print(json.dumps(dict(record=d.name, deleted_files=len(r['deleted']), available_gib=round(free_bytes()/GIB,3)), ensure_ascii=False), flush=True)
        receipt['state'] = 'complete' if free_bytes() >= 60 * GIB else 'eligible_plan_exhausted'
    except Exception as exc:
        receipt['state'] = 'stopped_on_error'
        receipt['error'] = str(exc)
        raise
    finally:
        receipt['completed_at'] = now()
        receipt['final_available_bytes'] = free_bytes()
        atomic(run / 'receipt.json', receipt)
        index = ['# 旧失败实验清理记录', '', '按用户本次指令清理；自动删除及上传策略均保持关闭。', '',
                 '保留各实验的“实验简要记录.md”、原始结果/授权/验证元数据及文本记录。原始录制数据的缺失见每个目录的 user_cleanup_receipt.json。', '',
                 '释放 %.3f GiB；剩余 %.3f GiB；状态 %s。' % (receipt['deleted_allocated_bytes']/GIB, receipt['final_available_bytes']/GIB, receipt['state']), '']
        index += ['- [%s](%s/实验简要记录.md)' % (r['record_id'], r['path']) for r in receipt['records']]
        atomic(run / '清理索引.md', '\n'.join(index) + '\n')
    print(json.dumps(dict(state=receipt['state'], deleted_gib=receipt['deleted_allocated_bytes']/GIB,
                          available_gib=receipt['final_available_bytes']/GIB, run_path=str(run)), ensure_ascii=False), flush=True)


if __name__ == '__main__':
    a = argparse.ArgumentParser()
    a.add_argument('--plan', required=True)
    a.add_argument('--sha256', required=True)
    args = a.parse_args()
    execute(args.plan, args.sha256)
