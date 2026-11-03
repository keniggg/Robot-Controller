"""Freeze the deployed GEN47 sources for isolated GEN48 offline work."""
import hashlib
import json
import os
from pathlib import Path
import time

W = Path(__file__).resolve().parents[1]
E = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
V = E/'ros_log/full_latest_gen41_restart_20261018T011222Z/verification'
RELEASE = V/'gen47_release_manifest.json'
RELEASE_SHA = '69e63cc71c4c63f1bbeaa2693dd88829363f06c878117f5f6129f38eaaf43e5a'
BASE = W/'.runtime/gen48_baseline_source'
CANDIDATE = W/'.runtime/gen48_visibility_candidate'
OUT = V/'gen48_source_baseline_r1'


def sha(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()


def write(p, value):
    raw = (json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False)+'\n').encode()
    with p.open('xb') as f:
        f.write(raw)
        f.flush()
        os.fsync(f.fileno())
    assert p.read_bytes() == raw
    return sha(p)


def main():
    assert E.parent.parent.is_mount(), 'SanStand is unavailable'
    assert sha(RELEASE) == RELEASE_SHA
    release = json.loads(RELEASE.read_bytes())
    sources = release['source_sha256']
    assert len(sources) == 1052 and release['passed'] and release['reviewed']
    assert not any(p.exists() for p in (BASE, CANDIDATE, OUT))
    for rel, digest in sources.items():
        assert not Path(rel).is_absolute() and '..' not in Path(rel).parts
        assert sha(W/rel) == digest, rel
    OUT.mkdir()
    for root in (BASE, CANDIDATE):
        root.mkdir()
        for rel, digest in sources.items():
            raw = (W/rel).read_bytes()
            assert hashlib.sha256(raw).hexdigest() == digest
            target = root/rel
            target.parent.mkdir(parents=True, exist_ok=True)
            with target.open('xb') as f:
                f.write(raw)
            assert sha(target) == digest
    manifest = dict(kind='GEN48_baseline_source', workspace_root=str(BASE),
        release_path=str(RELEASE), release_sha256=RELEASE_SHA,
        source_sha256=sources, source_count=len(sources),
        historical_test_files=release['test_scope_files'],
        actual_regression=release['actual_regression'],
        actual_regression_sha256=release['actual_regression_sha256'],
        source_bytes=sum((BASE/rel).stat().st_size for rel in sources),
        recorded_ns=time.time_ns())
    baseline_sha = write(BASE/'manifest.json', manifest)
    assert all(sha(W/rel) == digest for rel, digest in sources.items())
    receipt = dict(passed=True, baseline_path=str(BASE),
        baseline_manifest_sha256=baseline_sha, candidate_path=str(CANDIDATE),
        source_count=len(sources), original_sources_unchanged=True,
        production_installed=False, motion_commands_sent=False,
        helper_sha256=sha(Path(__file__)), recorded_ns=time.time_ns())
    receipt_sha = write(OUT/'result.json', receipt)
    print(json.dumps(dict(result=str(OUT/'result.json'), sha256=receipt_sha,
        baseline_manifest_sha256=baseline_sha, source_count=len(sources))))


if __name__ == '__main__':
    main()
