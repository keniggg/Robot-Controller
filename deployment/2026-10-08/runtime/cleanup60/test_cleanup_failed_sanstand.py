#!/usr/bin/env python3
"""Deletion safety checks using temporary files only; never touch SanStand."""
import contextlib
import copy
import hashlib
import importlib.util
import io
import json
import os
from pathlib import Path
import tempfile
import unittest
from unittest import mock


SPEC = importlib.util.spec_from_file_location(
    'bounded_cleanup_under_test', Path(__file__).with_name('cleanup_failed_sanstand.py'))
CLEANUP = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(CLEANUP)


class CleanupSafetyTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='sanstand-cleanup-safety-', dir='/tmp')
        self.addCleanup(self.temp.cleanup)
        self.mount = Path(self.temp.name) / 'simulated-mount'
        self.root = self.mount / '实验记录/Robot-Controller'
        self.record = self.root / 'grasp_records/20200101-failed'
        self.record.mkdir(parents=True)
        self.manifest = self.record / 'manifest.json'
        self.manifest.write_text(json.dumps(dict(state='complete', recorder_closed=True, result='failure')))
        self.log = self.record / 'runner.log'
        self.log.write_text('GRASP_RESULT success=False message=failed\n')
        self.raw = [self.record / 'keyframes.bag', self.record / 'second.bag']
        for p in self.raw:
            p.write_bytes(b'failed experiment raw payload\n' * 20)
        self.unit = 4096
        self.target = 60 * self.unit
        self.initial_free = self.target - CLEANUP.identity(self.raw[0])['allocated']
        self.raw_allocated = {str(p): CLEANUP.identity(p)['allocated'] for p in self.raw}
        self.plan_path = Path(self.temp.name) / 'reviewed-plan.json'
        self.run = self.root / 'maintenance/one-bounded-cleanup'
        self.record_plan = dict(
            path=str(self.record), classification='failed', started_at='2020-01-01T00:00:00Z',
            formal_record=True, evidence=[dict(file=str(self.log), excerpt='explicit failed result')],
            bound_evidence=[dict(path=str(p), sha256=CLEANUP.digest(p)) for p in (self.manifest, self.log)],
            delete_files=[dict(path=str(p), identity=CLEANUP.identity(p)) for p in self.raw],
            keep_files=[dict(path=str(p), identity=CLEANUP.identity(p)) for p in (self.manifest, self.log)],
            summary_markdown='发现：历史抓取失败。\n修复：未证实。\n方法：原始结果复核。\n验证：只确认历史失败。\n')
        self.plan = dict(mount_device=self.mount.stat().st_dev, target_available_bytes=self.target,
                         max_available_bytes=63 * self.unit, records=[self.record_plan],
                         external_run_path=str(self.run), authorization='Temporary-file safety test only')
        self.write_plan()

    def write_plan(self):
        self.plan_path.write_text(json.dumps(self.plan, ensure_ascii=False, indent=2))
        self.plan_sha = CLEANUP.digest(self.plan_path)

    def simulated_free(self):
        return self.initial_free + sum(n for p, n in self.raw_allocated.items() if not Path(p).exists())

    @contextlib.contextmanager
    def simulation(self, opened=None, mount_check=None):
        with contextlib.ExitStack() as stack:
            stack.enter_context(mock.patch.object(CLEANUP, 'MOUNT', self.mount))
            stack.enter_context(mock.patch.object(CLEANUP, 'ROOT', self.root))
            stack.enter_context(mock.patch.object(CLEANUP, 'GIB', self.unit))
            stack.enter_context(mock.patch.object(CLEANUP, 'free_bytes', side_effect=self.simulated_free))
            stack.enter_context(mock.patch.object(CLEANUP, 'open_paths', return_value=set(opened or [])))
            stack.enter_context(mock.patch.object(CLEANUP.os.path, 'ismount',
                                                side_effect=mount_check or (lambda p: p == str(self.mount))))
            stack.enter_context(contextlib.redirect_stdout(io.StringIO()))
            yield

    def execute(self):
        CLEANUP.execute(self.plan_path, self.plan_sha)

    def assert_no_deletion(self):
        self.assertTrue(all(p.exists() for p in self.raw))

    def test_summary_and_durable_intent_exist_before_unlink_and_metadata_is_preserved(self):
        original_hashes = {str(p): CLEANUP.digest(p) for p in (self.manifest, self.log)}
        original_unlink = Path.unlink
        inspected = []

        def inspect_before_unlink(path, *args, **kwargs):
            if str(path) in self.raw_allocated:
                self.assertEqual((self.record / '实验简要记录.md').read_text(), self.record_plan['summary_markdown'])
                per_record = json.loads((self.record / 'user_cleanup_receipt.json').read_text())
                global_receipt = json.loads((self.run / 'receipt.json').read_text())
                self.assertIn(str(path), [i['path'] for i in per_record['pending_delete']])
                self.assertIn(str(path), [i['path'] for i in global_receipt['records'][0]['pending_delete']])
                self.assertEqual({i['path']: i['sha256'] for i in per_record['retained']}, original_hashes)
                inspected.append(str(path))
            return original_unlink(path, *args, **kwargs)

        with self.simulation(), mock.patch.object(Path, 'unlink', new=inspect_before_unlink):
            self.execute()
        self.assertEqual(inspected, [str(self.raw[0])])
        self.assertEqual({str(p): CLEANUP.digest(p) for p in (self.manifest, self.log)}, original_hashes)
        receipt = json.loads((self.run / 'receipt.json').read_text())
        self.assertEqual(receipt['state'], 'complete')
        self.assertEqual(receipt['deleted_file_count'], 1)
        self.assertTrue(self.raw[1].exists(), 'must stop as soon as free space reaches the target')

    def test_already_at_target_deletes_no_file_or_record_metadata(self):
        self.initial_free = self.target
        with self.simulation():
            self.execute()
        self.assert_no_deletion()
        self.assertFalse((self.record / '实验简要记录.md').exists())
        self.assertEqual(json.loads((self.run / 'receipt.json').read_text())['deleted_file_count'], 0)

    def test_changed_plan_is_rejected_before_any_external_action(self):
        self.plan_path.write_text(self.plan_path.read_text() + '\n')
        with self.simulation(), self.assertRaisesRegex(RuntimeError, 'reviewed plan SHA mismatch'):
            self.execute()
        self.assert_no_deletion()
        self.assertFalse(self.run.exists())

    def test_changed_outcome_evidence_is_rejected(self):
        self.log.write_text('GRASP_RESULT success=True message=grip confirmed\n')
        with self.simulation(), self.assertRaisesRegex(RuntimeError, 'classification evidence changed'):
            self.execute()
        self.assert_no_deletion()
        self.assertFalse(self.run.exists())

    def test_open_record_is_rejected(self):
        with self.simulation(opened=[str(self.raw[0])]), self.assertRaisesRegex(RuntimeError, 'open descriptor'):
            self.execute()
        self.assert_no_deletion()
        self.assertFalse(self.run.exists())

    def test_explicitly_protected_record_is_never_cleaned_even_if_failed_and_old(self):
        # A newly protected record outranks both the failed classification and
        # the oldest-first selection. Rebind the plan SHA so it is the actual
        # protection check, not the reviewed-plan digest, which rejects it.
        self.plan['protect_records'] = [str(self.record)]
        self.write_plan()
        before = {str(p): CLEANUP.digest(p) for p in (*self.raw, self.manifest, self.log)}
        with self.simulation(), self.assertRaisesRegex(RuntimeError, 'protected record'):
            self.execute()
        self.assertEqual({str(p): CLEANUP.digest(p) for p in (*self.raw, self.manifest, self.log)}, before)
        self.assertFalse(self.run.exists())
        self.assertFalse((self.record / '实验简要记录.md').exists())
        self.assertFalse((self.record / 'user_cleanup_receipt.json').exists())

    def test_closed_formal_record_requires_exact_failure_result_after_evidence_rebinding(self):
        # Closed/complete plus a failed runner line cannot overrule the formal
        # outcome. Exercise fresh hashes/identities to reach the result guard.
        for result in ('success', 'unknown', None, False, 'Failure', 'missing'):
            with self.subTest(result=result):
                manifest = dict(state='complete', recorder_closed=True)
                if result != 'missing':
                    manifest['result'] = result
                self.manifest.write_text(json.dumps(manifest))
                self.record_plan['bound_evidence'][0]['sha256'] = CLEANUP.digest(self.manifest)
                self.record_plan['keep_files'][0]['identity'] = CLEANUP.identity(self.manifest)
                self.write_plan()
                with self.simulation(), self.assertRaisesRegex(RuntimeError, 'result is not failure'):
                    self.execute()
                self.assert_no_deletion()
                self.assertFalse(self.run.exists())
                self.assertFalse((self.record / '实验简要记录.md').exists())

    def test_duplicate_deletion_inside_one_record_is_rejected_before_writes(self):
        self.record_plan['delete_files'].append(copy.deepcopy(self.record_plan['delete_files'][0]))
        self.write_plan()
        with self.simulation(), self.assertRaisesRegex(RuntimeError, 'duplicate deletion path'):
            self.execute()
        self.assert_no_deletion()
        self.assertFalse(self.run.exists())

    def test_duplicate_deletion_across_plan_records_is_rejected_before_writes(self):
        self.plan['records'].append(copy.deepcopy(self.record_plan))
        self.write_plan()
        with self.simulation(), self.assertRaisesRegex(RuntimeError, 'duplicate deletion path'):
            self.execute()
        self.assert_no_deletion()
        self.assertFalse(self.run.exists())

    def test_hardlinked_raw_file_is_rejected_even_with_unchanged_bytes_and_inode(self):
        alias = Path(self.temp.name) / 'retained-experiment-alias.bag'
        os.link(self.raw[0], alias)
        self.assertEqual(self.raw[0].stat().st_ino, alias.stat().st_ino)
        with self.simulation(), self.assertRaisesRegex(RuntimeError, 'not a regular file'):
            self.execute()
        self.assert_no_deletion()
        self.assertTrue(alias.exists())
        self.assertEqual(self.raw[0].stat().st_nlink, 2)
        self.assertFalse(self.run.exists())

    def test_hardlinked_retained_metadata_also_fails_preflight(self):
        alias = Path(self.temp.name) / 'retained-manifest-alias.json'
        os.link(self.manifest, alias)
        with self.simulation(), self.assertRaisesRegex(RuntimeError, 'not a regular file'):
            self.execute()
        self.assert_no_deletion()
        self.assertTrue(alias.exists())
        self.assertFalse(self.run.exists())

    def test_hardlink_created_after_preflight_is_caught_immediately_before_unlink(self):
        alias = Path(self.temp.name) / 'concurrent-retained-alias.bag'
        checks = []

        def open_paths_with_concurrent_link():
            checks.append(True)
            # Initial preflight, record preparation, then per-file active
            # writer check. Simulate a link appearing just before identity()
            # is checked again; the original size/mtime/inode have not changed.
            if len(checks) == 3:
                os.link(self.raw[0], alias)
            return set()

        with self.simulation(), mock.patch.object(CLEANUP, 'open_paths', side_effect=open_paths_with_concurrent_link):
            with self.assertRaisesRegex(RuntimeError, 'not a regular file'):
                self.execute()
        self.assert_no_deletion()
        self.assertTrue(alias.exists())
        receipt = json.loads((self.run / 'receipt.json').read_text())
        self.assertEqual(receipt['state'], 'stopped_on_error')
        self.assertEqual(receipt['deleted_file_count'], 0)
        self.assertFalse(receipt['records'][0]['deleted'])

    def test_raw_file_from_another_record_is_rejected(self):
        other = self.root / 'grasp_records/another-experiment/keyframes.bag'
        other.parent.mkdir()
        other.write_bytes(b'another experiment')
        self.record_plan['delete_files'].append(dict(path=str(other), identity=CLEANUP.identity(other)))
        self.write_plan()
        with self.simulation(), self.assertRaisesRegex(RuntimeError, 'not an allowlisted raw file'):
            self.execute()
        self.assert_no_deletion()
        self.assertTrue(other.exists())

    def test_non_bulk_metadata_is_rejected_as_raw_deletion(self):
        self.record_plan['delete_files'].append(dict(path=str(self.manifest), identity=CLEANUP.identity(self.manifest)))
        self.write_plan()
        with self.simulation(), self.assertRaisesRegex(RuntimeError, 'not an allowlisted raw file'):
            self.execute()
        self.assert_no_deletion()
        self.assertTrue(self.manifest.exists())

    def test_incomplete_formal_record_is_rejected_even_with_rebound_evidence(self):
        self.manifest.write_text(json.dumps(dict(state='incomplete', recorder_closed=False, result='failure')))
        self.record_plan['bound_evidence'][0]['sha256'] = CLEANUP.digest(self.manifest)
        self.record_plan['keep_files'][0]['identity'] = CLEANUP.identity(self.manifest)
        self.write_plan()
        with self.simulation(), self.assertRaisesRegex(RuntimeError, 'not closed and complete'):
            self.execute()
        self.assert_no_deletion()

    def test_existing_summary_is_not_overwritten(self):
        prior = self.record / '实验简要记录.md'
        prior.write_text('original experiment note')
        with self.simulation(), self.assertRaisesRegex(RuntimeError, 'will not be overwritten'):
            self.execute()
        self.assertEqual(prior.read_text(), 'original experiment note')
        self.assert_no_deletion()

    def test_mount_disconnection_before_unlink_preserves_payload(self):
        calls = []

        def mounted(path):
            calls.append(path)
            return len(calls) < 4

        with self.simulation(mount_check=mounted), self.assertRaisesRegex(RuntimeError, 'mount changed or disconnected'):
            self.execute()
        self.assert_no_deletion()
        receipt = json.loads((self.run / 'receipt.json').read_text())
        self.assertEqual(receipt['state'], 'stopped_on_error')
        self.assertEqual(receipt['deleted_file_count'], 0)

    def test_changed_raw_identity_is_rejected(self):
        self.raw[0].write_bytes(self.raw[0].read_bytes() + b'new recorder output')
        with self.simulation(), self.assertRaisesRegex(RuntimeError, 'raw file changed'):
            self.execute()
        self.assert_no_deletion()

    def test_oversize_first_file_cannot_exceed_free_space_bound(self):
        self.raw[0].write_bytes(b'x' * (4 * self.unit))
        self.record_plan['delete_files'][0]['identity'] = CLEANUP.identity(self.raw[0])
        self.write_plan()
        self.initial_free = self.target - 1
        with self.simulation(), self.assertRaisesRegex(RuntimeError, 'exceeds bounded cleanup'):
            self.execute()
        self.assert_no_deletion()


if __name__ == '__main__':
    unittest.main(verbosity=2)
