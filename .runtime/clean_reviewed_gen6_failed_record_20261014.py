"""Apply the reviewed single GEN6 failure classification and existing space guard."""
import fcntl
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import time

W = Path(__file__).resolve().parents[1]
M = Path('/media/zhuyupei/SanStand')
E = M/'实验记录/Robot-Controller'
REVIEW = W/'.runtime/gen20_gen6_cleanup_review_20261014'

def read(path):
    return json.loads(Path(path).read_text())

def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def main():
    assert M.is_mount() and not os.statvfs(E).f_flag & os.ST_RDONLY
    assert sha(REVIEW/'review_manifest.json') == '16ef047d94ec67d9453694e305b8f96abc74f75cba5fe193768b7c5176f15a54'
    review = read(REVIEW/'review_manifest.json')
    for name, digest in review['files'].items():
        assert sha(REVIEW/name) == digest
    row = read(REVIEW/'catalog_entry.json')
    change = read(REVIEW/'protected_change.json')
    target = E/row['record_path']
    raw = target/'keyframes.bag'
    assert raw.is_file() and raw.stat().st_size == 3824608857
    # The guard obtains an exclusive kernel lease before hashing/deleting.
    # This preliminary scan also records that no visible process is using it.
    opened = []
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit():
            continue
        try:
            for fd in (proc/'fd').iterdir():
                try:
                    if fd.resolve(strict=True) == raw:
                        opened.append(str(fd))
                except OSError:
                    pass
        except OSError:
            pass
    assert not opened, opened
    spec = importlib.util.spec_from_file_location('reviewed_space_guard', W/'tools/maintain_failed_record_space.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    guard = module.Guard()
    policy = guard.controls()
    assert policy and guard.free() < policy['trigger_free_bytes']
    maintenance = E/'maintenance'
    catalog_path = maintenance/'failed_record_space_guard_catalog.json'
    protected_path = maintenance/'failed_record_space_guard_protected.json'
    audit = maintenance/('user_failed_cleanup_gen6_classification_'+str(time.time_ns()))
    audit.mkdir()
    unchanged_controls = {str(p):sha(p) for p in (
        maintenance/'failed_record_space_guard_policy.json',
        maintenance/'storage_control.json', E/'grasp_records/archive_config.json')}
    with (maintenance/'failed_record_space_guard_lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        assert sha(protected_path) == change['expected_before_sha256']
        catalog = read(catalog_path)
        protected = read(protected_path)
        assert len(protected['protected_paths']) == change['before_count']
        assert all(not (E/r['record_path']/item['path']).exists()
                   for r in catalog['records'] for item in r['raw_files'])
        assert not any(r['record_path'] == row['record_path'] for r in catalog['records'])
        for name, path in [('catalog.before.json',catalog_path),('protected.before.json',protected_path)]:
            (audit/name).write_bytes(path.read_bytes())
        for name in ('review_manifest.json','catalog_entry.json','protected_change.json','summary.md'):
            (audit/name).write_bytes((REVIEW/name).read_bytes())
        entries = protected['protected_paths']
        for remove in change['remove_entries']:
            assert entries.count(remove) == 1
            entries.remove(remove)
        assert len(entries) == change['expected_after_count']
        guard.record(row, protected)
        catalog['records'].append(row)
        catalog['review_id'] += ';20261014_gen6_startup_failure_independently_closed'
        summary = (REVIEW/'summary.md').read_text().replace('（执行前草稿）','（独立审阅完成，删除以回执为准）')
        module.atomic(target/'空间清理独立复核_GEN6_20261014.md', summary)
        # All other protected records and archive controls remain byte-bound.
        module.atomic(catalog_path, catalog)
        module.atomic(protected_path, protected)
        after = {str(p):sha(p) for p in (catalog_path, protected_path)}
        module.atomic(audit/'classification_receipt.json', dict(
            recorded_ns=time.time_ns(), record=row['record_path'],
            source_script_sha256=sha(__file__), controls_sha256=unchanged_controls,
            before_sha256={str(catalog_path):sha(audit/'catalog.before.json'),
                           str(protected_path):sha(audit/'protected.before.json')},
            after_sha256=after, other_protected_records_preserved=True,
            visible_open_fds=opened, raw_deleted=False))

    print(json.dumps(dict(classification=str(audit), candidate_bytes=3824608857,
        deletion_executed=False, next_action='Apply schema-2 verified GitHub metadata migration and 80GiB space guard')),flush=True)

if __name__ == '__main__':
    main()
