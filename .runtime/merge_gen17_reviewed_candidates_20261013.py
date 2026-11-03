"""Merge independent GEN17 deliveries against one verified deployment base.

Only creates an isolated candidate and evidence; never installs or calls ROS.
"""
import argparse
import difflib
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import time


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read(path):
    return json.loads(Path(path).read_text())


def relative(value):
    path = Path(value)
    assert not path.is_absolute() and '..' not in path.parts, value
    return path


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--prior', type=Path, required=True)
    ap.add_argument('--delivery', type=Path, action='append', required=True)
    ap.add_argument('--candidate', type=Path, required=True)
    ap.add_argument('--evidence', type=Path, required=True)
    args = ap.parse_args()
    workspace = Path(__file__).resolve().parents[1]
    mount = Path('/media/zhuyupei/SanStand')
    assert mount.is_mount() and mount in args.evidence.resolve().parents
    assert workspace / '.runtime' in args.candidate.resolve().parents
    prior = read(args.prior)
    assert prior['passed'] and not args.candidate.exists() and not args.evidence.exists()
    baseline = prior['source_sha256']
    for name, digest in baseline.items():
        assert sha(workspace / relative(name)) == digest, name
    deliveries = []
    scope = list(prior['test_scope'])
    parents = {str(args.prior.resolve()): sha(args.prior)}
    # Validate every parent file before creating the candidate.
    for path in args.delivery:
        delivery = read(path)
        if 'passed' not in delivery:
            # Two author packages use an evidence-backed delivery schema;
            # normalize in memory without rewriting their sealed manifests.
            assert delivery['package'] in ('gen16_success_first_contact', 'gen16_contact_budget_checkpoint_v2')
            assert delivery['status'] == 'candidate_verified_not_installed'
            proof = delivery['verification']
            assert sha(proof['path']) == proof['sha256']
            verified = read(proof['path'])
            assert verified['passed'] and verified['returncode'] == 0
            assert verified['summary']['tests'] > 0
            assert all(verified['summary'][k] == 0 for k in ('failures', 'errors', 'skipped'))
            stage = path.resolve().parent
            scope_files = []
            for selected in verified['selected']:
                source = Path(selected.split('::', 1)[0])
                try:
                    name = source.relative_to(stage)
                except ValueError:
                    name = source.relative_to(workspace)
                scope_files.append(str(relative(name)))
            delivery = dict(delivery, passed=True, candidate_root=str(stage),
                test_scope=list(dict.fromkeys(scope_files)),
                files=[dict(path=row['relative_path'], base_sha256=row['baseline_sha256'],
                            candidate_sha256=row['sha256']) for row in delivery['files']])
            parents[str(Path(proof['path']).resolve())] = proof['sha256']
        assert delivery['passed']
        stage = Path(delivery.get('candidate_root') or delivery['stage'])
        for row in delivery['files']:
            dest = relative(row.get('destination', row['path']))
            source = stage / relative(row['path'])
            assert sha(source) == (row.get('candidate_sha256') or row['sha256'])
            current = workspace / dest
            assert (sha(current) if current.exists() else None) == row['base_sha256'], str(dest)
        deliveries.append((path, stage, delivery))
        scope.extend(delivery.get('test_scope', []))
        parents[str(path.resolve())] = sha(path)
    scope = list(dict.fromkeys(scope))
    args.candidate.mkdir()
    args.evidence.mkdir()
    copied = set(baseline) | {str(relative(p)) for p in scope if (workspace / relative(p)).is_file()}
    for name in sorted(copied):
        target = args.candidate / name
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(workspace / name, target)
    changes = {}
    merges = []
    for manifest, stage, delivery in deliveries:
        for row in delivery['files']:
            name = str(relative(row.get('destination', row['path'])))
            target = args.candidate / name
            source = stage / relative(row['path'])
            target.parent.mkdir(parents=True, exist_ok=True)
            if name not in changes:
                shutil.copy2(source, target)
                changes[name] = {'base_sha256': row['base_sha256'], 'parents': []}
            elif target.read_bytes() != source.read_bytes():
                assert row['base_sha256'] is not None, 'conflicting newly added file: ' + name
                command = ['git', 'merge-file', '-p', str(target), str(workspace / name), str(source)]
                result = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
                merges.append({'path': name, 'command': command, 'returncode': result.returncode,
                               'stderr': result.stderr.decode()})
                if result.returncode != 0:
                    (args.evidence / 'merge_conflict.txt').write_bytes(result.stdout)
                    raise RuntimeError('Unresolved candidate merge: ' + name)
                target.write_bytes(result.stdout)
            changes[name]['parents'].append(str(manifest.resolve()))
    patch = []
    files = []
    for name, row in sorted(changes.items()):
        original = workspace / name
        target = args.candidate / name
        before = original.read_text().splitlines(True) if original.exists() else []
        patch.extend(difflib.unified_diff(before, target.read_text().splitlines(True),
                                        fromfile='a/' + name, tofile='b/' + name))
        files.append(dict(path=name, base_sha256=row['base_sha256'],
                          candidate_sha256=sha(target), parent_deliveries=row['parents']))
    patch_path = args.candidate / 'delivery.patch'
    patch_path.write_text(''.join(patch))
    for name, digest in baseline.items():
        assert sha(workspace / name) == digest, name
    for name in scope:
        assert (args.candidate / relative(name)).is_file(), name
    result = dict(passed=True, kind='GEN17_COMBINED_CANDIDATE_NOT_INSTALLED',
                  candidate_root=str(args.candidate.resolve()), files=files,
                  test_scope=scope, parent_sha256=parents, patch_sha256=sha(patch_path),
                  merges=merges, baseline_sources_unchanged=True,
                  motion_commands_sent=False, deployed=False, physical_grasp_verified=False,
                  recorded_ns=time.time_ns())
    out = args.candidate / 'delivery_manifest.json'
    out.write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n')
    (args.evidence / 'result.json').write_text(json.dumps(dict(result,
        delivery_path=str(out), delivery_sha256=sha(out)), ensure_ascii=False, indent=2) + '\n')
    print(json.dumps(dict(delivery=str(out), sha256=sha(out), files=len(files), test_files=len(scope))))


if __name__ == '__main__':
    main()
