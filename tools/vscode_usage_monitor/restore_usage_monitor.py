#!/usr/bin/env python3
"""Validate or restore this local reader repair; never reload VS Code."""
import argparse
import hashlib
import json
from pathlib import Path
from repair_usage_monitor import atomic_write


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--extension-dir', type=Path, required=True)
    parser.add_argument('--backup-dir', type=Path, required=True)
    parser.add_argument('--affinity-report', type=Path)
    parser.add_argument('--apply', action='store_true')
    args = parser.parse_args()
    manifest = json.loads((args.backup_dir / 'manifest.json').read_text())
    package = args.extension_dir / 'package.json'
    reader = args.extension_dir / 'out/codexUsage.js'
    helper = args.extension_dir / 'out/bounded_jsonl_reader.js'
    assert sha(package) == manifest['before']['package.json'], 'Extension version changed; re-review rollback'
    assert sha(reader) == manifest['after']['codexUsage.js'], 'Reader changed after repair; do not overwrite'
    assert sha(helper) == manifest['after']['bounded_jsonl_reader.js'], 'Helper changed; do not delete'
    original = args.backup_dir / 'codexUsage.js'
    assert sha(original) == manifest['before']['codexUsage.js'], 'Backup digest mismatch'
    settings = None
    if args.affinity_report:
        report = json.loads(args.affinity_report.read_text())
        settings = Path(report['path'])
        data = json.loads(settings.read_text())
        affinity = data.get('extensions.experimental.affinity', {})
        assert affinity.get(report['extension']) == report['affinity'], 'Affinity changed; do not overwrite'
        backup = Path(report['backup'])
        assert sha(backup) == report['before_sha256']
        old = json.loads(backup.read_text()).get('extensions.experimental.affinity', {})
        if report['extension'] in old:
            affinity[report['extension']] = old[report['extension']]
        else:
            del affinity[report['extension']]
        if not affinity:
            data.pop('extensions.experimental.affinity', None)
    if args.apply:
        atomic_write(reader, original.read_text())
        helper.unlink()
        if settings:
            atomic_write(settings, json.dumps(data, ensure_ascii=False, indent=4) + '\n')
    print(json.dumps({'validation_passed': True, 'restored': args.apply, 'window_reloaded': False}))


if __name__ == '__main__':
    main()
