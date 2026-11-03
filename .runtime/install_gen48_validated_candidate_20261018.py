"""Install exactly the independently reviewed GEN48 module and new tests.

This is a source installation, not a running-node deployment or motion grant.
The complete regression must run against these installed bytes before reload.
"""
import hashlib
import json
import os
from pathlib import Path
import time

W = Path(__file__).resolve().parents[1]
E = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
N = E/'ros_log/full_latest_gen41_restart_20261018T011222Z'
V = N/'verification'
C = W/'.runtime/gen48_visibility_candidate'
OUT = V/'gen48_installation_r1'
PROD = 'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/cuboid_rgb_interval.py'
TEST = 'src/alicia_flexible_grasp_supervisor/tests/test_gen48_cuboid_visibility_family.py'
EXPECTED = {PROD: 'f724adea2782b414cccac9154bbd8166ffd498e6f3d4742a68712aa62835ade5',
            TEST: 'a407c66bd9f79c874e714e58da24fca10db61e70e3edc42684548c5294f1cdf4'}


def sha(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()


def read(p):
    return json.loads(p.read_bytes())


def main():
    assert E.parent.parent.is_mount() and not OUT.exists()
    refs = {
        V/'gen47_release_manifest.json': '69e63cc71c4c63f1bbeaa2693dd88829363f06c878117f5f6129f38eaaf43e5a',
        V/'gen48_candidate_focus_r1/result.json': 'c4b8ffc0444069623b80eb75b5a523e1ae0041100f0fd417e95297427ce9dd32',
        V/'gen48_visibility_family_integration_review_r2/review.json': '92298990c8472aa2d047b09b932832ca04e97c7a7e645fb31e92917d4b192d27',
        V/'gen48_g47_candidate_qualification_r1/result.json': '03b6c0ed6b37b2c6292aa96f18006877cef0529ac2693fde09df258f21579b1c',
    }
    assert all(sha(p) == digest for p, digest in refs.items())
    prior = read(V/'gen47_release_manifest.json')['source_sha256']
    focus = read(V/'gen48_candidate_focus_r1/result.json')
    assert focus['passed'] and focus['summary'] == dict(tests=94, failures=0, errors=0, skipped=0)
    sources = focus['source_sha256']
    assert len(prior) == 1052 and len(sources) == 1053
    assert set(sources)-set(prior) == {TEST}
    assert {p for p in prior if prior[p] != sources[p]} == {PROD}
    assert all(sources[p] == digest for p, digest in EXPECTED.items())
    assert all(sha(C/p) == digest for p, digest in sources.items())
    assert all(sha(W/p) == digest for p, digest in prior.items())
    assert not (W/TEST).exists()
    ready_path = N/'gen48_preinstall_readonly_r1/result.json'
    ready = read(ready_path)
    assert ready['passed'] and ready['manual_mode'] and not ready['task_active']
    assert ready['deployment_processes_unchanged'] and ready['selection']['generation'] == 7
    assert 0 <= time.time_ns()-ready['recorded_ns'] < 120*10**9
    control_path = E/'ros_log/full_latest_frozen_no_skin_20261012T021050Z/continuous_authorization_20261012/control.json'
    control = read(control_path)
    assert control['state'] == 'active' and control['revision'] == 52
    refs[ready_path] = sha(ready_path)
    refs[control_path] = sha(control_path)
    OUT.mkdir()
    for rel in (PROD, TEST):
        target = W/rel
        temporary = target.with_name(target.name+'.gen48.install.tmp')
        assert not temporary.exists()
        mode = target.stat().st_mode & 0o777 if target.exists() else 0o644
        with temporary.open('xb') as f:
            f.write((C/rel).read_bytes())
            f.flush()
            os.fsync(f.fileno())
        assert sha(temporary) == EXPECTED[rel]
        os.chmod(temporary, mode)
        os.replace(temporary, target)
    assert all(sha(W/p) == digest for p, digest in sources.items())
    assert all(sha(p) == digest for p, digest in refs.items())
    value = dict(passed=True, source_count=1053, changed_files=EXPECTED,
        source_sha256=sources, original_baseline=W.joinpath('.runtime/gen48_baseline_source').as_posix(),
        evidence_sha256={str(p): digest for p, digest in refs.items()},
        full_regression_passed=False, runtime_reloaded=False, motion_commands_sent=False,
        recorded_ns=time.time_ns(), helper_sha256=sha(Path(__file__)))
    p = OUT/'result.json'
    with p.open('x') as f:
        json.dump(value, f, ensure_ascii=False, indent=2, allow_nan=False)
        f.write('\n')
    print(json.dumps(dict(passed=True, result=str(p), sha256=sha(p), source_count=1053)))


if __name__ == '__main__':
    main()
