"""Seal the installed GEN17 bundle with its actual regression and reviews."""
import argparse
import hashlib
import json
from pathlib import Path
import time


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read(path):
    return json.loads(Path(path).read_text())


def main():
    ap = argparse.ArgumentParser()
    for name in ('install', 'tests', 'delivery', 'review', 'output'):
        ap.add_argument('--' + name, type=Path, required=True)
    args = ap.parse_args()
    for name, value in vars(args).items():
        setattr(args, name, value.resolve())
    work = Path(__file__).resolve().parents[1]
    mount = Path('/media/zhuyupei/SanStand')
    assert mount.is_mount() and mount in args.output.resolve().parents
    installed, tested, delivery, reviewed = map(read,
        (args.install, args.tests, args.delivery, args.review))
    assert all(d['passed'] for d in (installed, tested, delivery, reviewed))
    assert tested['source_unchanged'] and tested['production_worktree_tested']
    assert tested['prior_sha256'] == sha(args.install)
    assert reviewed['delivery_sha256'] == sha(args.delivery)
    assert installed['source_sha256'] == tested['source_sha256']
    assert tested['summary']['tests'] > 0
    assert all(tested['summary'][k] == 0 for k in ('failures', 'errors', 'skipped'))
    scopes = [d['test_scope'] for d in (installed, delivery, reviewed)]
    assert all(len(scope) == len(set(scope)) for scope in scopes)
    assert set(scopes[0]) == set(scopes[1]) == set(scopes[2])
    argv_tests = [p for p in tested['command'] if p in set(installed['test_scope'])]
    assert argv_tests == installed['test_scope']
    for name, digest in installed['source_sha256'].items():
        assert sha(work / name) == digest, name
    for row in delivery['files']:
        assert sha(work / row['path']) == row['candidate_sha256'], row['path']
    evidence = {}

    def bind(name, digest):
        path = Path(name)
        if not path.is_absolute():
            path = work / path
        assert sha(path) == digest, str(path)
        evidence[str(path.resolve())] = digest

    item = installed
    while True:
        for name, digest in item['evidence_sha256'].items():
            bind(name, digest)
        if 'prior' not in item:
            break
        item = read(item['prior'])
    for name, digest in delivery['parent_sha256'].items():
        bind(name, digest)
    for name, digest in reviewed['evidence_sha256'].items():
        bind(name, digest)
    for path in (args.install, args.tests, args.delivery, args.review, Path(__file__)):
        bind(str(path), sha(path))
    result = dict(passed=True, reviewed=True, installed=True,
        kind='gen17_registered_current_expiry_wait_across_contact_admission',
        source_sha256=installed['source_sha256'], source_count=len(installed['source_sha256']),
        test_scope=installed['test_scope'], test_summary=tested['summary'],
        test_scope_files=len(installed['test_scope']), evidence_sha256=evidence,
        installed_receipt=str(args.install), actual_regression=str(args.tests),
        candidate_delivery=str(args.delivery), combined_review=str(args.review),
        policy_changes=[
            'Already registered current geometry expiration waits for a genuinely fresh independently joined successor',
            'Task provenance and final pre-freeze metadata commit share bounded pending semantics',
            'Remote receipt issuance revalidates current geometry before issuance and retries only before commitment',
            'Remote wait wakes for hard quarantine and preserved-identity generation cancellation',
            'Registration clock pairing preserves the original monotonic geometry bound',
            'Existing task plan lock keeps current-record inspection and quarantine atomic across copy and requalification status entry points',
            'Pure clock expiration preserves only immutable audited proposal evidence for an exactly matching station retirement request, never planning or motion authority',
            'Original candidate and immutable historical receipt witnesses are preserved',
            'No phase/session/identity renewal and no physical action replay'],
        execution_requirements_preserved=[
            'Actual target, control owner, SDK and accepted feedback',
            'Fresh publication/commit evidence and exact current phase ownership',
            'Geometry, continuous collision, IK and motion limits',
            'Registration correction and contact uncertainty evidence limits',
            'Original published phase/session deadlines; automatic lift disabled'],
        physical_grasp_verified=False, motion_commands_sent=False,
        temperature_queried=False, recorded_ns=time.time_ns())
    with args.output.open('x') as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2)
        stream.write('\n')
    print(json.dumps(dict(passed=True, release=str(args.output), sha256=sha(args.output),
                         sources=result['source_count'], tests=result['test_summary'])))


if __name__ == '__main__':
    main()
