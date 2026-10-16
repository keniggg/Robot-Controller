#!/usr/bin/env python3
"""Restore two temporary statistics settings after the old host exits.

This process never signals VS Code. PID start time defeats PID reuse. Source
hashes and compare-and-swap checks prevent resuming an unverified old reader or
reverting later user edits. State/log paths are supplied on the external disk.
"""
import argparse
import hashlib
import json
from pathlib import Path
import time
from repair_usage_monitor import atomic_write


def process_start_ticks(pid, proc_root=Path('/proc')):
    try:
        stat = (proc_root / str(pid) / 'stat').read_text()
        return stat[stat.rfind(')') + 2:].split()[19]
    except (FileNotFoundError, ProcessLookupError):
        return None


def restore_owned_settings(current, saved):
    result = dict(current)
    restored, preserved = [], []
    for key, expected in saved['temporary_values'].items():
        if key not in current or current[key] != expected:
            preserved.append(key)
            continue
        prior = saved['original_values'][key]
        if prior['present']:
            result[key] = prior['value']
        else:
            result.pop(key, None)
        restored.append(key)
    return result, restored, preserved



def verify_active_extension(saved):
    expected = saved.get('required_active_extension')
    if not isinstance(expected, dict):
        return False, 'missing_active_extension_guard'
    try:
        registry = json.loads(Path(expected['registry_path']).read_text())
        entries = [entry for entry in registry
                   if entry.get('identifier', {}).get('id', '').lower() == expected['id'].lower()]
        if len(entries) != 1:
            return False, 'registered_extension_missing_or_ambiguous'
        entry = entries[0]
        if entry.get('version') != expected['version']:
            return False, 'registered_extension_version_changed'
        registered_path = entry.get('location', {}).get('path')
        if not registered_path or Path(registered_path).resolve() != Path(expected['path']).resolve():
            return False, 'registered_extension_path_changed'
        package = Path(registered_path) / 'package.json'
        if hashlib.sha256(package.read_bytes()).hexdigest() != expected['package_sha256']:
            return False, 'registered_extension_manifest_changed'
    except (OSError, ValueError, KeyError, TypeError, AttributeError):
        return False, 'active_extension_registry_unreadable_or_invalid'
    return True, None


def restore(saved, state_path):
    verified, reason = verify_active_extension(saved)
    if not verified:
        state = {'status': 'blocked_active_extension_changed', 'reason': reason,
                 'settings_restored': False, 'editor_restarted': False}
        atomic_write(state_path, json.dumps(state, indent=2) + '\n')
        return 2
    for path, expected in saved['required_patch_sha256'].items():
        p = Path(path)
        actual = hashlib.sha256(p.read_bytes()).hexdigest() if p.is_file() else None
        if actual != expected:
            state = {'status': 'blocked_patch_changed', 'path': path,
                     'settings_restored': False, 'editor_restarted': False}
            atomic_write(state_path, json.dumps(state, indent=2) + '\n')
            return 2
    path = Path(saved['settings_path'])
    before = path.read_bytes()
    current = json.loads(before)
    updated, restored, preserved = restore_owned_settings(current, saved)
    # Refuse to overwrite a concurrent settings edit between read and write.
    if path.read_bytes() != before:
        return None
    if restored:
        atomic_write(path, json.dumps(updated, ensure_ascii=False, indent=4) + '\n')
    state = {'status': 'restored_after_old_host_exit', 'restored_keys': restored,
             'preserved_user_changed_keys': preserved, 'editor_restarted': False,
             'time_unix': time.time()}
    atomic_write(state_path, json.dumps(state, indent=2) + '\n')
    return 0


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--manifest', type=Path, required=True)
    parser.add_argument('--state', type=Path, required=True)
    args = parser.parse_args()
    saved = json.loads(args.manifest.read_text())
    atomic_write(args.state, json.dumps({'status': 'waiting_for_old_host_exit',
        'pid': saved['old_host_pid'], 'start_ticks': saved['old_host_start_ticks'],
        'editor_restarted': False}, indent=2) + '\n')
    while process_start_ticks(saved['old_host_pid']) == saved['old_host_start_ticks']:
        time.sleep(5)
    for _ in range(3):
        result = restore(saved, args.state)
        if result is not None:
            raise SystemExit(result)
        time.sleep(1)
    atomic_write(args.state, json.dumps({'status': 'blocked_concurrent_settings_edit',
        'settings_restored': False, 'editor_restarted': False}, indent=2) + '\n')
    raise SystemExit(2)


if __name__ == '__main__':
    main()
