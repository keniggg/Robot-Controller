import json
import hashlib
from pathlib import Path
import tempfile
import unittest
from restore_after_host_exit import restore_owned_settings, process_start_ticks, restore, verify_active_extension

class RestoreTests(unittest.TestCase):
    def active_guard(self, root):
        extension = root / 'sun-0.1.1'; extension.mkdir(exist_ok=True)
        package = extension / 'package.json'; package.write_text('{"version":"0.1.1"}')
        registry = root / 'extensions.json'
        registry.write_text(json.dumps([{'identifier': {'id': 'sunyingkai.codex-usage-monitor'},
            'version': '0.1.1', 'location': {'path': str(extension)}}]))
        return {'registry_path': str(registry), 'id': 'SunYingkai.codex-usage-monitor',
                'version': '0.1.1', 'path': str(extension),
                'package_sha256': hashlib.sha256(package.read_bytes()).hexdigest()}
    def manifest(self):
        return {'temporary_values': {'home': 'empty', 'period': 86400},
                'original_values': {'home': {'present': False, 'value': None},
                                    'period': {'present': True, 'value': 60}}}
    def test_restore_only_owned_keys(self):
        updated, restored, kept = restore_owned_settings({'home': 'empty', 'period': 86400, 'affinity': 1}, self.manifest())
        self.assertEqual(updated, {'period': 60, 'affinity': 1})
        self.assertEqual(set(restored), {'home', 'period'})
        self.assertEqual(kept, [])
    def test_preserve_later_user_changes(self):
        updated, restored, kept = restore_owned_settings({'home': 'user/path', 'period': 86400, 'new': True}, self.manifest())
        self.assertEqual(updated, {'home': 'user/path', 'period': 60, 'new': True})
        self.assertEqual(kept, ['home'])
    def test_do_not_recreate_user_removed_override(self):
        updated, restored, kept = restore_owned_settings({}, self.manifest())
        self.assertEqual(updated, {})
        self.assertEqual(restored, [])
    def test_pid_missing_and_start_time_with_space_in_comm(self):
        with tempfile.TemporaryDirectory() as root:
            root = Path(root)
            self.assertIsNone(process_start_ticks(42, root))
            (root / '42').mkdir()
            (root / '42/stat').write_text('42 (code helper) ' + ' '.join(['S'] + ['0'] * 18 + ['12345'] + ['0'] * 3))
            self.assertEqual(process_start_ticks(42, root), '12345')

    def test_patch_hash_mismatch_never_restores_settings(self):
        with tempfile.TemporaryDirectory() as root:
            root = Path(root); source = root / 'reader.js'; source.write_text('changed')
            settings = root / 'settings.json'; settings.write_text('{"home":"empty","period":86400}')
            saved = self.manifest(); saved['required_active_extension'] = self.active_guard(root); saved.update(settings_path=str(settings), required_patch_sha256={str(source): 'wrong'})
            before = settings.read_bytes()
            self.assertEqual(restore(saved, root / 'state.json'), 2)
            self.assertEqual(settings.read_bytes(), before)
            self.assertEqual(json.loads((root / 'state.json').read_text())['status'], 'blocked_patch_changed')
    def test_verified_restore_preserves_unrelated_settings(self):
        with tempfile.TemporaryDirectory() as root:
            root = Path(root); source = root / 'reader.js'; source.write_text('fixed')
            settings = root / 'settings.json'; settings.write_text('{"home":"empty","period":86400,"affinity":1}')
            saved = self.manifest(); saved['required_active_extension'] = self.active_guard(root); saved.update(settings_path=str(settings), required_patch_sha256={str(source): hashlib.sha256(source.read_bytes()).hexdigest()})
            self.assertEqual(restore(saved, root / 'state.json'), 0)
            self.assertEqual(json.loads(settings.read_text()), {'period': 60, 'affinity': 1})

    def test_matching_active_registry_and_manifest_pass(self):
        with tempfile.TemporaryDirectory() as root:
            saved = {'required_active_extension': self.active_guard(Path(root))}
            self.assertEqual(verify_active_extension(saved), (True, None))
    def test_missing_active_guard_fails_closed(self):
        self.assertEqual(verify_active_extension({}), (False, 'missing_active_extension_guard'))
    def test_new_registered_version_blocks_even_if_old_patch_files_remain(self):
        with tempfile.TemporaryDirectory() as root:
            root = Path(root); saved = self.manifest(); guard = self.active_guard(root)
            saved['required_active_extension'] = guard
            old = root / 'sun-0.1.1/reader.js'; old.write_text('fixed old version')
            settings = root / 'settings.json'; settings.write_text('{"home":"empty","period":86400}')
            saved.update(settings_path=str(settings), required_patch_sha256={str(old): hashlib.sha256(old.read_bytes()).hexdigest()})
            registry = Path(guard['registry_path']); entry = json.loads(registry.read_text())[0]
            entry['version'] = '0.1.2'; entry['location']['path'] = str(root / 'sun-0.1.2')
            registry.write_text(json.dumps([entry])); before = settings.read_bytes()
            self.assertEqual(restore(saved, root / 'state.json'), 2)
            self.assertEqual(settings.read_bytes(), before)
            self.assertEqual(json.loads((root / 'state.json').read_text())['reason'], 'registered_extension_version_changed')
    def test_same_version_registered_in_new_directory_blocks(self):
        with tempfile.TemporaryDirectory() as root:
            guard = self.active_guard(Path(root)); registry = Path(guard['registry_path']); entries = json.loads(registry.read_text())
            entries[0]['location']['path'] += '-replacement'; registry.write_text(json.dumps(entries))
            self.assertEqual(verify_active_extension({'required_active_extension': guard}), (False, 'registered_extension_path_changed'))
    def test_modified_package_manifest_blocks(self):
        with tempfile.TemporaryDirectory() as root:
            guard = self.active_guard(Path(root)); (Path(guard['path']) / 'package.json').write_text('{"version":"other"}')
            self.assertEqual(verify_active_extension({'required_active_extension': guard}), (False, 'registered_extension_manifest_changed'))
    def test_duplicate_registry_entries_block(self):
        with tempfile.TemporaryDirectory() as root:
            guard = self.active_guard(Path(root)); registry = Path(guard['registry_path']); entries = json.loads(registry.read_text())
            registry.write_text(json.dumps(entries * 2))
            self.assertEqual(verify_active_extension({'required_active_extension': guard}), (False, 'registered_extension_missing_or_ambiguous'))
    def test_missing_registry_entry_blocks(self):
        with tempfile.TemporaryDirectory() as root:
            guard = self.active_guard(Path(root)); Path(guard['registry_path']).write_text('[]')
            self.assertEqual(verify_active_extension({'required_active_extension': guard}), (False, 'registered_extension_missing_or_ambiguous'))
    def test_invalid_registry_blocks(self):
        with tempfile.TemporaryDirectory() as root:
            guard = self.active_guard(Path(root)); Path(guard['registry_path']).write_text('not json')
            self.assertEqual(verify_active_extension({'required_active_extension': guard}), (False, 'active_extension_registry_unreadable_or_invalid'))

if __name__ == '__main__':
    unittest.main()
