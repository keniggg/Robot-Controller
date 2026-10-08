#!/usr/bin/env python3
"""Stage/apply a narrow, backed-up repair to SunYingkai's usage reader.

Never reloads VS Code. An extension update can replace this local repair.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import tempfile

REPLACEMENT = '''    for (const file of files) {
        const summary = await boundedUsageReader.read(file, (line) => {
            if (!line.includes('"token_count"')) return undefined;
            const parsed = parseTokenCountLine(line);
            if (!parsed) return undefined;
            return {
                timestampMs: parsed.timestamp ? Date.parse(parsed.timestamp) : 0,
                value: {
                    latestTimestamp: parsed.timestamp,
                    latestFile: file,
                    planType: parsed.payload?.rate_limits?.plan_type,
                    primary: normalizeWindow(parsed.payload?.rate_limits?.primary),
                    secondary: normalizeWindow(parsed.payload?.rate_limits?.secondary),
                    totalTokenUsage: normalizeTokenUsage(parsed.payload?.info?.total_token_usage),
                    lastTokenUsage: normalizeTokenUsage(parsed.payload?.info?.last_token_usage),
                    modelContextWindow: parsed.payload?.info?.model_context_window,
                },
            };
        });
        if (!summary) continue;
        latest.tokenEventsSeen += summary.eventCount;
        if (!summary.latest || summary.latestMs < latestMs) continue;
        latestMs = summary.latestMs;
        latest = { ...latest, ...summary.latest };
    }
'''
PRELUDE = '''const { createReader, singleFlight } = require("./bounded_jsonl_reader");
const boundedUsageReader = createReader();
const readUsageSingleFlight = singleFlight(readCodexUsageImpl, (options) => JSON.stringify([
    expandHome(options.codexHome?.trim() || path.join(os.homedir(), '.codex')),
    Math.max(1, options.scanDays),
]));
async function readCodexUsage(options) { return readUsageSingleFlight(options); }
async function readCodexUsageImpl(options) {'''


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def repaired(text):
    if 'const boundedUsageReader = createReader();' in text:
        raise RuntimeError('Reader already repaired; no blind reapplication')
    begin = text.index('    for (const file of files) {\n')
    end = text.index('    return latest;\n', begin)
    old = text[begin:end]
    assert "await fs.readFile(file, 'utf8')" in old and 'text.split(/\\r?\\n/)' in old
    assert text.count('async function readCodexUsage(options) {') == 1
    result = text[:begin] + REPLACEMENT + text[end:]
    return result.replace('async function readCodexUsage(options) {', PRELUDE)


def atomic_write(path, content):
    old_mode = path.stat().st_mode & 0o777 if path.exists() else 0o644
    fd, temp = tempfile.mkstemp(prefix=path.name + '.', dir=path.parent)
    try:
        with os.fdopen(fd, 'w') as out:
            out.write(content)
        os.chmod(temp, old_mode)
        os.replace(temp, path)
    finally:
        if os.path.exists(temp):
            os.unlink(temp)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--extension-dir', type=Path, required=True)
    ap.add_argument('--stage-dir', type=Path)
    ap.add_argument('--backup-dir', type=Path)
    ap.add_argument('--apply', action='store_true')
    args = ap.parse_args()
    package = args.extension_dir / 'package.json'
    metadata = json.loads(package.read_text())
    assert metadata['publisher'] == 'SunYingkai' and metadata['name'] == 'codex-usage-monitor'
    assert metadata['version'] == '0.1.1', 'Re-review another extension version before patching'
    reader = args.extension_dir / 'out/codexUsage.js'
    helper = Path(__file__).with_name('bounded_jsonl_reader.js')
    content = repaired(reader.read_text())
    if args.stage_dir:
        args.stage_dir.mkdir(parents=True, exist_ok=True)
        (args.stage_dir / reader.name).write_text(content)
        shutil.copy2(helper, args.stage_dir / helper.name)
    if args.apply:
        assert args.backup_dir and os.path.ismount('/media/zhuyupei/SanStand')
        args.backup_dir.mkdir(parents=True, exist_ok=False)
        originals = {'codexUsage.js': reader, 'extension.js': args.extension_dir / 'out/extension.js', 'package.json': package}
        manifest = {'extension': 'SunYingkai.codex-usage-monitor', 'version': metadata['version'], 'runtime_reloaded': False,
                    'before': {name: digest(path) for name, path in originals.items()}}
        for name, path in originals.items():
            shutil.copy2(path, args.backup_dir / name)
        atomic_write(args.extension_dir / 'out' / helper.name, helper.read_text())
        atomic_write(reader, content)
        manifest['after'] = {'codexUsage.js': digest(reader), helper.name: digest(args.extension_dir / 'out' / helper.name)}
        (args.backup_dir / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
        print(json.dumps({'applied': str(reader), 'backup': str(args.backup_dir), 'manifest': manifest}))


if __name__ == '__main__':
    main()
