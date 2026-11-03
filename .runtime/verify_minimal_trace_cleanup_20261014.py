#!/usr/bin/env python3
"""Verify the completed, finite local cleanup without touching hardware."""
import hashlib
import json
import os
from pathlib import Path
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from minimal_experiment_trace import TraceStore, atomic, sha_file

ROOT = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
RUN = ROOT / 'maintenance/minimal_trace_cleanup_1791956657642651189'


def main():
    receipt = json.loads((RUN / 'receipt.json').read_text())
    assert receipt['state'] == 'complete'
    plan = json.loads((RUN / 'plan.json').read_text())
    source = json.loads((RUN / 'execution_source.json').read_text())
    assert source['plan_sha256'] == sha_file(RUN / 'plan.json')
    assert source['tool_sha256'] == plan['source_tool_sha256']
    validation = json.loads(Path(plan['validation_result']).read_text())
    assert validation['passed'] and validation['source_unchanged']
    assert sha_file(plan['validation_result']) == plan['validation_result_sha256']
    assert validation['source_sha256']['tools/minimal_experiment_trace.py'] == source['tool_sha256']
    rows = {r['path']: r for r in plan['text_files'] + plan['raw_files']}
    store = TraceStore(RUN / 'trace.sqlite3', readonly=True)
    try:
        assert store.db.execute("SELECT count(*) FROM deletions WHERE state!='deleted'").fetchone()[0] == 0
        totals = store.deletion_totals()
        assert all(receipt[k] == v for k, v in totals.items())
        logical_bytes, raw_bytes, text_bytes, allocated_raw, allocated_text = 0, 0, 0, 0, 0
        references, examples = [], []
        for name, value in store.db.execute("SELECT path,receipt FROM deletions ORDER BY path"):
            item = json.loads(value)
            row = rows[name]
            path = ROOT / name
            assert not path.exists(), name
            assert row['identity'] == item['original_identity']
            assert row['allocated_bytes'] == item['allocated_bytes']
            assert not any(name == p or name.startswith(p + '/') for p in plan['excluded_prefixes'])
            reference = item['trace_reference']
            info = store.info(reference)
            logical_bytes += row['identity'][2]
            if row['kind'] == 'raw_bag':
                raw_bytes += row['identity'][2]
                allocated_raw += row['allocated_bytes']
                assert info['metadata']['original_sha256'] == item['original_sha256']
                assert item['original_result'] == row['original_result']
                assert info['metadata']['all_non_visual_messages_retained'] is True
                assert item['full_raw_recoverable'] is False
                references.append(reference)
            else:
                text_bytes += row['identity'][2]
                allocated_text += row['allocated_bytes']
                assert info['sha256'] == item['original_sha256']
                if len(examples) < 5 or ('authorization' in name and not any('authorization' in e for e in examples)):
                    examples.append(reference)
        # Independent exact readback samples; the executor already verified the
        # entire archive before writing its completion receipt.
        examples += ['@index/experiment_results.json', '@project/tools/minimal_experiment_trace.py']
        for name in examples:
            store.verify(name)
        baseline = json.loads((RUN / 'protected_before.json').read_text())['files']
        changed, missing = [], []
        for name, expected in baseline.items():
            try:
                st = (ROOT / name).stat()
                if [st.st_dev, st.st_ino, st.st_size, st.st_mtime_ns] != expected:
                    changed.append(name)
            except FileNotFoundError:
                missing.append(name)
        values = os.statvfs(ROOT)
        result = dict(state='verified' if not changed and not missing else 'protected_changes_require_review',
                      plan_sha256=source['plan_sha256'], tool_sha256=source['tool_sha256'],
                      validation=validation['summary'], totals=totals,
                      deleted_logical_bytes=logical_bytes, raw_logical_bytes=raw_bytes,
                      text_logical_bytes=text_bytes, raw_allocated_bytes=allocated_raw,
                      text_allocated_bytes=allocated_text, retained_trace=store.stats(),
                      deleted_raw_trace_references=references, exact_readback_examples=examples,
                      protected_checked=len(baseline), protected_changed=changed, protected_missing=missing,
                      actual_free_bytes=values.f_bavail * values.f_frsize,
                      stop_reason=receipt['stop_reason'], skipped=receipt['skipped'],
                      github_contacted=False, motion_commands_sent=False, temperature_queried=False,
                      verified_ns=time.time_ns())
        atomic(RUN / 'final_verification.json', result)
        print(json.dumps({k: v for k, v in result.items() if k not in
                          ('deleted_raw_trace_references', 'exact_readback_examples', 'skipped')}, ensure_ascii=False))
        assert result['state'] == 'verified', 'protected changes require explicit review'
    finally:
        store.close()


if __name__ == '__main__':
    main()
