"""Seal reviewed GEN28 source binding and corrected real-frame offline replay."""
import argparse
import hashlib
import json
from pathlib import Path
import time
import xml.etree.ElementTree as ET

from verify_gen28_combined_20261015 import W, N, BASE, BASE_SHA, PRODUCTION, FIXTURE, new_tests, read, guard, write


OFFLINE_REPLAY = '.runtime/replay_gen28_actual_raw_box_entry.py'
OFFLINE_REPLAY_SHA = '497171f0f0801a71077b385238d8af876e60efb41036e43a95b30d4b947d15b1'


def sha(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('combined', 'review', 'entry-replay', 'output'):
        parser.add_argument('--'+name, required=True, type=Path)
    parser.add_argument('--review-sha256', required=True)
    args = parser.parse_args()
    guard(args.output)
    assert not args.output.exists()
    assert sha(BASE) == BASE_SHA
    prior = read(BASE)
    evidence = {}

    def bind(path, expected=None):
        path = Path(path).resolve(strict=True)
        digest = sha(path)
        assert expected is None or digest == expected, str(path)
        assert str(path) not in evidence or evidence[str(path)] == digest
        evidence[str(path)] = digest
        return path

    bind(BASE, BASE_SHA)
    # Existing runtime reload and motion admission helpers remain exactly the
    # reviewed GEN27 versions; this source fix adds no new hardware entrypoint.
    for path, digest in prior['evidence_sha256'].items():
        bind(path, digest)
    combined = read(bind(args.combined))
    assert combined['passed'] and combined['source_unchanged'] and combined['scope_complete']
    assert combined['baseline_file_test_counts_retained']
    assert combined['returncode'] == 0 and combined['regression_scope'] == 'full_combined'
    assert combined['production_worktree_tested'] and not combined['live_ros_contacted']
    assert combined['baseline'] == str(BASE) and combined['baseline_sha256'] == BASE_SHA
    sources, scope, summary = combined['source_sha256'], combined['test_scope'], combined['summary']
    assert len(sources) == combined['source_count'] and len(scope) == combined['test_scope_files']
    assert set(prior['test_scope']) <= set(scope) and set(scope) == set(prior['test_scope']+new_tests())
    assert summary['tests'] > prior['test_summary']['tests']
    assert all(summary[key] == 0 for key in ('failures', 'errors', 'skipped'))
    assert all(sha(W/name) == digest for name, digest in sources.items())
    assert set(combined['test_case_counts_by_file']) == set(scope)
    assert sum(combined['test_case_counts_by_file'].values()) == summary['tests']
    assert all(combined['test_case_counts_by_file'][name] >= count
               for name, count in prior['test_case_counts_by_file'].items())
    patch = read(bind(combined['patch'], combined['patch_sha256']))
    modified = PRODUCTION | {FIXTURE} | set(new_tests())
    assert set(patch['files']) == modified and not patch['new_resources']
    for name, row in patch['files'].items():
        assert row == dict(baseline_sha256=prior['source_sha256'].get(name), sha256=sources[name])
    for path, digest in patch['evidence_sha256'].items():
        bind(path, digest)
    bind(args.combined.parent/'pytest.log', combined['log_sha256'])
    xml = bind(args.combined.parent/'pytest.xml', combined['xml_sha256'])
    actual = {key: sum(int(s.get(key, 0)) for s in ET.parse(xml).getroot().iter('testsuite'))
              for key in ('tests', 'failures', 'errors', 'skipped')}
    assert actual == summary
    bind(W/'.runtime/verify_gen28_combined_20261015.py', combined['runner_sha256'])
    review_path = bind(args.review, args.review_sha256)
    review = read(review_path)
    assert review['reviewed'] is True and not review['blocking_findings']
    required = modified | {'.runtime/verify_gen28_combined_20261015.py', '.runtime/seal_gen28_release_20261015.py', OFFLINE_REPLAY}
    assert required <= set(review['reviewed_source_sha256'])
    for name, digest in review['reviewed_source_sha256'].items():
        assert not Path(name).is_absolute() and '..' not in Path(name).parts
        bind(W/name, digest)
        if name in sources:
            assert sources[name] == digest
    assert review['evidence_sha256'].get(str(args.combined.resolve())) == sha(args.combined)
    assert review['evidence_sha256'].get(str(args.entry_replay.resolve())) == sha(args.entry_replay)
    for path, digest in review['evidence_sha256'].items():
        bind(path, digest)
    replay_path = bind(args.entry_replay)
    replay = read(replay_path)
    # The independent replay publishes these explicit claims only after it
    # invokes the actual repaired remote entry, not a memory-cloned prototype.
    assert replay['actual_production_entry_called'] is True
    assert replay['corrected_bbox_xywh'] is True
    assert replay['raw_model_qualified'] is True
    assert replay['fused_pixel_baseline_qualified'] is False
    assert replay['fused_geometry_preserved'] is True
    assert replay['raw_source_hashes_match'] is True
    assert replay['source_unchanged'] is True
    assert replay['actual_production_generator_called'] is True
    assert replay['physical_grasp_verified'] is False
    assert replay['source_sha256'].get(OFFLINE_REPLAY) == OFFLINE_REPLAY_SHA
    assert review['reviewed_source_sha256'].get(OFFLINE_REPLAY) == OFFLINE_REPLAY_SHA
    for name, digest in replay['source_sha256'].items():
        assert not Path(name).is_absolute() and '..' not in Path(name).parts
        if name == OFFLINE_REPLAY:
            bind(W/name, OFFLINE_REPLAY_SHA)
        else:
            assert sources.get(name) == digest
            bind(W/name, digest)
    for path, digest in replay['input_sha256'].items():
        bind(path, digest)
    previous = bind(N/'gen27_deployment/result.json')
    deployment = read(previous)
    assert deployment['deployment_passed'] and deployment['whole_patch_deployed']
    assert deployment['source_manifest_sha256'] == BASE_SHA
    assert len(dict(deployment['after'], **deployment['preserved_after'])) == 11
    bind(__file__)
    release = dict(schema_version=1, kind='gen28_retained_original_frame_box_qualification',
        sealed_ns=time.time_ns(), passed=True, reviewed=True, installed=True,
        installed_semantics='Exact local source passed offline tests and corrected real-frame replay; runtime reload remains pending.',
        source_sha256=sources, source_count=len(sources), test_scope=scope,
        test_scope_files=len(scope), test_summary=summary,
        test_case_counts_by_file=combined['test_case_counts_by_file'],
        regression_scope='full_combined', baseline_release=str(BASE), baseline_release_sha256=BASE_SHA,
        actual_regression=str(args.combined.resolve()), combined_review=str(review_path),
        actual_entry_replay=str(replay_path), evidence_sha256=evidence,
        previous_deployment=str(previous), runtime_deployment_pending=True,
        physical_grasp_verified=False, motion_commands_sent=False, temperature_queried=False)
    write(args.output, release)
    print(json.dumps(dict(release=str(args.output), sha256=sha(args.output), summary=summary)), flush=True)


if __name__ == '__main__':
    main()
