"""Bind independent GEN22 review to the completed frozen regression."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import time
import xml.etree.ElementTree as ET

W = Path(__file__).resolve().parents[1]
E = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
N = E/'ros_log/full_latest_gen18_restart_20261014T012216Z'
START = E/'ros_log/full_latest_gen22_restart_20261015T003700Z/startup_review.json'
SOURCES = {
    'src/alicia_grasp_modes/scripts/unknown_tabletop_perception.py': '11c77f485e433ad22cca9d22f3225f834909ecf6e8b892908ab15bbf8c0f58f3',
    'src/alicia_grasp_modes/src/alicia_grasp_modes/station_compute_wait_producer.py': 'a1e5fd29a57ddf71ecb77644f8e480e6edee4f66644e38f31d6206e28483830f',
    'src/alicia_grasp_modes/src/alicia_grasp_modes/station_compute_wait_remote.py': '8a7a36c3b23b297585620894bec00ea3091b9bc6220a57ad1900dd8314afd72e',
    'src/alicia_grasp_modes/src/alicia_grasp_modes/observe_return_remote.py': '6c497ca57f10c9c91f27ea10e1432b0c6ad95a9cb23635dfc9ee36b39888a5a4',
    'src/alicia_grasp_modes/scripts/mode_aware_remote_grasp6d.py': '3ae6f738aa0eeb8d6c5fbee89519cb51b529cbdf7857bdf811fb51b1662b274e',
    'src/alicia_grasp_modes/tests/test_gen22_ingress_recheck.py': '1216309f05f6f802ee58c7ce176fe758300649af293f0be00606bc2411db9e51',
    'src/alicia_grasp_modes/tests/test_gen22_qualified_rgbd_remote.py': 'f07e3d078d73dc3a4e260b3b061daad77e20a9a0ee176014cb500385e2ec6cb3',
    'src/alicia_grasp_modes/tests/test_gen22_station_sdk_refresh_independent.py': '76ee3c4ce7e0cc18fbd5611e95238aa59a099d2ca5d8d4032b5810501cfb36a3',
    'src/alicia_grasp_modes/tests/test_gen22_commit_boundary_independent.py': '5f4dffd25f35bf34790cd09957ae0d3eee4f8136fc616e5fc11519c004bdea88',
}


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read(path):
    return json.loads(Path(path).read_text())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--combined', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    mount = Path('/media/zhuyupei/SanStand')
    assert mount.is_mount() and not os.statvfs(mount).f_flag & os.ST_RDONLY
    assert N/'verification' in args.output.resolve().parents
    evidence = {}

    def bind(path, expected=None):
        path = Path(path).resolve(strict=True)
        digest = sha(path)
        assert expected is None or digest == expected, str(path)
        assert str(path) not in evidence or evidence[str(path)] == digest
        evidence[str(path)] = digest
        return path

    combined = read(bind(args.combined))
    assert combined['passed'] and combined['source_unchanged']
    assert combined['returncode'] == 0 and not combined['live_ros_contacted']
    assert combined['production_worktree_tested']
    assert combined['source_count'] == 941 and combined['test_scope_files'] == 204
    assert len(combined['source_sha256']) == 941 and len(combined['test_scope']) == 204
    assert all(combined['summary'][k] == 0 for k in ('failures', 'errors', 'skipped'))
    assert combined['summary']['tests'] >= 6149 + 98 + 15
    for name, digest in combined['source_sha256'].items():
        assert not Path(name).is_absolute() and '..' not in Path(name).parts
        assert sha(W/name) == digest, name
    for name, digest in SOURCES.items():
        assert combined['source_sha256'][name] == digest, name
    for name, key in [('pytest.log', 'log_sha256'), ('pytest.xml', 'xml_sha256')]:
        bind(args.combined.parent/name, combined[key])
    suites = ET.parse(args.combined.parent/'pytest.xml').getroot()
    assert {k: sum(int(t.get(k, 0)) for t in suites.iter('testsuite'))
            for k in combined['summary']} == combined['summary']
    baseline = read(bind(combined['baseline'], combined['baseline_sha256']))
    production = {n for n in SOURCES if '/tests/' not in n}
    assert {n for n, digest in baseline['source_sha256'].items()
            if combined['source_sha256'][n] != digest} == production
    assert set(combined['source_sha256']) - set(baseline['source_sha256']) == set(SOURCES) - production
    bind(combined['patch'], combined['patch_sha256'])
    bind(W/'.runtime/verify_gen22_combined_20261014.py', combined['runner_sha256'])
    bind(__file__)

    producer = read(bind(N/'verification/gen22_ingress_recheck/delivery.json',
        '53819d5c0b8956453d178d8797f687dbdf42ba4f4588979ac276168d0769fddb'))
    for path, digest in producer['evidence'].items():
        bind(path, digest)
    sdk = read(bind(N/'verification/gen22_independent_sdk_r2/review.json',
        'd6edab56fbb0c510ed0dd14464418d870db0017b7d642007a01b7a3610c4fcc8'))
    assert sdk['reviewed'] and not sdk['blocking_findings']
    for path, digest in sdk['evidence_sha256'].items():
        bind(N/path, digest)
    for directory in ('gen22_independent_commit_boundary_red',
                      'gen22_independent_commit_boundary_green_r1'):
        parent = N/'verification'/directory
        receipt = read(bind(parent/'receipt.json'))
        bind(parent/'pytest.log', receipt['log_sha256'])
        bind(parent/'pytest.xml', receipt['xml_sha256'])
        assert receipt['source_unchanged']
        if directory.endswith('green_r1'):
            assert receipt['returncode'] == 0
            assert all(SOURCES[k] == v for k, v in receipt['source_sha256'].items())
    extra_parent = N/'verification/gen22_qualified_rgbd_remote/r11_resume_extra'
    extra = read(bind(extra_parent/'receipt.json'))
    assert extra['returncode'] == 0
    bind(extra_parent/'pytest.log', extra['log_sha256'])
    bind(extra_parent/'pytest.xml')
    extra_suites = ET.parse(extra_parent/'pytest.xml').getroot()
    assert {k: sum(int(t.get(k, 0)) for t in extra_suites.iter('testsuite'))
            for k in ('tests', 'failures', 'errors', 'skipped')} == {
                'tests': 98, 'failures': 0, 'errors': 0, 'skipped': 0}
    for name in ('src/alicia_flexible_grasp_supervisor/tests/test_rgbd_snapshot.py',
                 'src/alicia_grasp_modes/tests/test_unknown_pending_identity.py'):
        assert name in combined['test_scope']
        assert combined['source_sha256'][name] == baseline['source_sha256'][name]
    bind(N/'verification/gen22_camera_gap_evidence/findings.json',
        '4043251b7af9236988a347187c9adf635cf1b2bcb0bd2601362b796cf89f38fe')
    startup = read(bind(START,
        '0bc9ff55c126690bc80702407f15e7703e65cfea24dbf3910e0bbd4de8f2cab2'))
    assert startup['reviewed'] and not startup['blocking_findings_for_local_cold_start']
    for path, digest in startup['evidence_sha256'].items():
        bind(path, digest)
    bind(W/'.runtime/finalize_gen22_full_restart_20261014.py',
        '1c7a477f673f3d827a6d041f615877777492bd357ad5a3c4c59b3b0b41869bdd')
    reload_path = '.runtime/reload_gen19_compute_20261014.py'
    bind(W/reload_path)
    result = dict(schema_version=1, reviewed=True, blocking_findings=[],
        recorded_ns=time.time_ns(), reviewer='/root/gen22_review_resume',
        review_scope='GEN22 five production files and four new tests, including independent producer review, actual remote MRO/cache and final registration concurrency boundaries; full frozen regression and separately reviewed cold-start scripts.',
        reviewed_source_sha256=SOURCES, evidence_sha256=evidence,
        validation=dict(combined=str(args.combined.resolve()), summary=combined['summary'],
            source_count=941, test_scope_files=204, independent_commit_tests=15,
            producer_targeted_tests=139, producer_independent_tests=12,
            supplemental_98_tests='Separately passed before r2; both files are included in r2, not additional unique tests.',
            live_ros_contacted=False, production_edits_by_reviewer=False),
        findings_resolved=[
            'Empty RGB-D snapshot is rechecked against latest ingress under the input lock before irreversible loss.',
            'Only typed provenance from an actual stale accepted-feedback failure allows one bounded SDK refresh retry; fresh measurement remains required.',
            'A qualified observation awaiting its exact original RGB-D can retry historical registration within unchanged source/cache/phase/session clocks.',
            'Paused reached-frame retry permits only existing validated three-topic station identity registration, including READY cached during the first wait.',
            'Typed pending preserves a registered identity in either callback order; historical registration does not release current geometry or publish an executable plan.',
            'Runtime age/latency/mask settings and original source, phase, request/session identity and contents remain pinned; duplicate phase messages cannot renew or bypass a revoked retry.',
            'All local owner/configuration and dual-clock checks run again after the potentially waiting context call; final commit repeats the local predicate.',
            'Actual phase and task callbacks serialize their short state changes with final registration. Reconstruction and inactive-task cleanup release the short locks first.',
            'Manual takeover consumes the pending retry while preserving its seen key, so later automatic mode or phase redelivery cannot resurrect it.'
        ],
        limits=[
            'No offline result proves physical grasp success. Runtime acceptance and fresh site checks remain pending.',
            'The original GEN21 record does not identify the exact producer-local scope exception; SDK race reproduction is not proof that it was the historical first cause.',
            'Transport and scheduler gaps are not eliminated, and truly expired sources, changed owners or invalid geometry remain rejected.',
            'Initial r10 and combined r1 were interrupted and are not treated as passed.',
            'The newly added final context call runs outside commit locks; existing selection checks remain unchanged. No claim is made that all legacy RPCs are lock-free.'
        ],
        reload_review=dict(path=reload_path, sha256=sha(W/reload_path), blockers=[],
            compatibility_only=True,
            actual_deployment='Full cold start requested by user; this compatibility field is not a claim of three-compute reload.'),
        cold_start_review=dict(path=str(START), sha256=sha(START),
            blockers=[], actual_runtime_acceptance_pending=True),
        motion_commands_sent=False, temperature_queried=False,
        physical_grasp_verified=False)
    args.output.parent.mkdir(exist_ok=False)
    assert args.output.parent.stat().st_dev == mount.stat().st_dev
    with args.output.open('x') as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2)
        stream.write('\n'); stream.flush(); os.fsync(stream.fileno())
    print(json.dumps(dict(review=str(args.output), sha256=sha(args.output),
                         summary=combined['summary']), ensure_ascii=False))


if __name__ == '__main__':
    main()
