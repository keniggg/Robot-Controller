"""Assemble reviewed test-only corrections; do not change installed sources."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import time


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read(path):
    return json.loads(Path(path).read_text())


def save(path, value):
    with path.open('x') as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2)
        stream.write('\n')


def main():
    ap = argparse.ArgumentParser()
    for name in ('prior', 'original', 'original-review', 'output'):
        ap.add_argument('--' + name, type=Path, required=True)
    ap.add_argument('--pair', nargs=2, type=Path, action='append', required=True)
    args = ap.parse_args()
    work = Path(__file__).resolve().parents[1]
    out = args.output.resolve()
    assert work / '.runtime' in out.parents and not out.exists()
    prior, original, original_review = map(read, (args.prior, args.original, args.original_review))
    assert all(x['passed'] for x in (prior, original, original_review))
    assert original_review['delivery_sha256'] == sha(args.original)
    for name, digest in prior['source_sha256'].items():
        assert sha(work / name) == digest, name
    scope = list(prior['test_scope'])
    parents = {str(p.resolve()): sha(p) for p in (args.prior, args.original, args.original_review)}
    changes = []
    to_copy = {}
    for manifest_path, review_path in args.pair:
        delivery, reviewed = read(manifest_path), read(review_path)
        assert reviewed['passed'] and reviewed['delivery_sha256'] == sha(manifest_path)
        assert delivery.get('passed') or delivery.get('status') == 'candidate_verified_not_installed'
        stage = Path(delivery.get('candidate_root') or delivery.get('stage') or manifest_path.resolve().parent)
        for row in delivery['files']:
            name = row.get('path') or row['relative_path']
            relative = Path(name)
            assert not relative.is_absolute() and '..' not in relative.parts and 'tests' in relative.parts
            assert name not in to_copy
            base = row.get('base_sha256') or row['baseline_sha256']
            digest = row.get('candidate_sha256') or row['sha256']
            assert sha(work / name) == base and sha(stage / name) == digest
            to_copy[name] = stage / name
            changes.append(dict(path=name, base_sha256=base, candidate_sha256=digest))
        scope.extend(delivery.get('test_scope', []))
        scope.extend(reviewed.get('test_scope', []))
        for p in (manifest_path, review_path):
            parents[str(p.resolve())] = sha(p)
    scope = list(dict.fromkeys(scope))
    assert all(not Path(name).is_absolute() and (work / name).is_file() for name in scope)
    old_stage = Path(original['candidate_root'])
    assert not set(to_copy) & {row['path'] for row in original['files']}
    out.mkdir()
    for row in original['files']:
        assert sha(old_stage / row['path']) == row['candidate_sha256'] == sha(work / row['path'])
        to_copy[row['path']] = old_stage / row['path']
    for name, source in to_copy.items():
        target = out / name
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, target)
    common = dict(passed=True, candidate_root=str(out), test_scope=scope,
        parent_sha256=parents, motion_commands_sent=False, physical_grasp_verified=False,
        recorded_ns=time.time_ns())
    save(out / 'delta_manifest.json', dict(common, files=changes,
        kind='GEN16_TEST_ONLY_COMBINATION_CORRECTIONS', production_modified=False))
    save(out / 'delivery_manifest_r3.json', dict(common, files=original['files'] + changes,
        kind='GEN16_COMPLETE_BUNDLE_WITH_TEST_CORRECTIONS', production_delta_from_r2=False))
    for name, digest in prior['source_sha256'].items():
        assert sha(work / name) == digest, name
    print(json.dumps(dict(output=str(out), delta_files=len(changes),
        bundle_files=len(to_copy), test_scope_files=len(scope),
        delta_sha256=sha(out / 'delta_manifest.json'),
        delivery_sha256=sha(out / 'delivery_manifest_r3.json'))))


if __name__ == '__main__':
    main()
