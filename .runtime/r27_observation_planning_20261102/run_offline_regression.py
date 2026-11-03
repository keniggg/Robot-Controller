#!/usr/bin/python3
"""Use the existing isolated verifier with an explicit reviewed R27 manifest.

Historical inventory lookup uses the byte-identical SSD aliases already
reviewed for R26. Extra runtime sources/tests are declared and SHA-pinned in
the review; the original sandbox, reconciliation and test functions remain.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import sys
import time
import types

sys.dont_write_bytecode = True
W = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(W / 'tools'))
from experiment_storage import select


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def checked(reference):
    path = Path(reference['path'])
    assert sha(path) == reference['sha256'], 'reviewed input changed: ' + str(path)
    return json.loads(path.read_bytes())


def main():
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument('--source-review', type=Path, required=True)
    parser.add_argument('--source-review-sha256', required=True)
    args, remaining = parser.parse_known_args()
    assert not sys.flags.optimize and sys.flags.no_user_site
    review_ref = dict(path=str(args.source_review.resolve()), sha256=args.source_review_sha256)
    review = checked(review_ref)
    assert review['reviewed'] is True and review['kind'] == 'r27_offline_source_review'
    baseline = checked(review['baseline_release'])
    assert review['baseline_release']['sha256'] == '9e483c892acfa1b9361a74873e90c972e7be0bfdb634f105b2b25fffaa92449a'
    entry = W / '.runtime/verify_integrated_failure_repair.py'
    assert sha(entry) == baseline['source_sha256'][str(entry.relative_to(W))]
    runner = types.ModuleType('_r27_original_isolated_regression')
    runner.__file__ = str(entry)
    sys.modules[runner.__name__] = runner
    exec(compile(entry.read_bytes(), str(entry), 'exec'), runner.__dict__)
    storage = select('ssd')
    root = storage.maintenance_root / 'r27_observation_planning_20261102'
    binding = storage.validate(root)
    aliases = storage.maintenance_root / 'physical_source_admission_repair_r26_20261030/validation_inputs_r1'
    receipt = checked(dict(path=str(aliases / 'alias_receipt.json'),
        sha256='746cefdd6e2f2c78a91aaf76b58c012d188de475d9f35b51385aab3ba85385ae'))
    for row in receipt['aliases']:
        assert sha(row['copy_path']) == sha(row['alias_path']) == row['sha256']
        assert aliases in Path(row['alias_path']).resolve().parents
    runner.BASE = runner.RUN = aliases
    original_scope, original_manifest, original_write = runner.scope, runner.source_manifest, runner.write
    extra_tests = tuple(review.get('additional_test_files', ()))
    extra_sources = tuple(review.get('additional_source_files', ()))
    assert len(set(extra_tests)) == len(extra_tests)
    assert set(extra_tests) <= set(extra_sources)
    assert all(Path(p).is_absolute() is False and '..' not in Path(p).parts for p in extra_sources)

    def declared_scope():
        tests, exclusions = original_scope()
        assert not set(tests).intersection(extra_tests)
        return sorted(tests + list(extra_tests)), exclusions

    def declared_manifest(tests):
        result = original_manifest(tests)
        for path in extra_sources:
            result[path] = sha(W / path)
        return result

    def declared_write(path, data):
        # The original verifier declares its historical explicit test in the
        # final result. Record the additional tests that this adapter actually
        # dispatched, so release readers can reconstruct the same full scope.
        if Path(path).name == 'result.json' and 'explicit_test_files' in data:
            assert set(extra_tests) <= set(data['declared_test_files'])
            selected_extra = sorted(set(extra_tests).intersection(data['test_files']))
            if data['complete_scope_requested']:
                assert set(extra_tests) <= set(data['test_files'])
            case_files = {row['nodeid'].split('::', 1)[0] for row in data['case_results']}
            expected_files = {node.split('::', 1)[0] for node in data['expected_case_ids']}
            verified = set(selected_extra) <= case_files == expected_files
            if data['passed']:
                assert verified
            assert {row['path'] for row in data['files']} == set(data['test_files'])
            data = dict(data, explicit_test_files=[*data['explicit_test_files'], *selected_extra],
                explicit_scope_extension_verified_against_actual_cases=verified)
        return original_write(path, data)

    runner.scope, runner.source_manifest = declared_scope, declared_manifest
    runner.write = declared_write
    tests, exclusions = declared_scope()
    current = declared_manifest(tests)
    assert current == review['source_sha256'], 'reviewed source inventory changed'
    changes = {p: dict(before_sha256=baseline['source_sha256'].get(p), after_sha256=current.get(p))
        for p in set(current) | set(baseline['source_sha256'])
        if current.get(p) != baseline['source_sha256'].get(p)}
    assert changes == review['changed_sources'], 'source delta differs from explicit review'
    assert '--name' in remaining and '--storage-backend' in remaining
    assert remaining[remaining.index('--storage-backend') + 1] == 'ssd'
    name = remaining[remaining.index('--name') + 1]
    assert Path(name).name == name and name not in ('.', '..')
    destination = root / 'regression_entry_receipts'
    destination.mkdir(exist_ok=True)
    output = destination / (name + '.json')
    raw = (json.dumps(dict(recorded_ns=time.time_ns(), source_review=review_ref,
        entry=dict(path=str(Path(__file__).resolve()), sha256=sha(__file__)),
        original_runner=dict(path=str(entry), sha256=sha(entry)),
        storage_binding=binding, argv=remaining, remapped_lookup_globals=['BASE', 'RUN'],
        manifest_extensions=extra_sources, scope_extensions=extra_tests,
        result_metadata_extension='actual extra case files also recorded as explicit_test_files',
        declared_test_file_count=len(tests), source_count=len(current),
        sandbox_and_reconciliation_unchanged=True, all_original_test_files_preserved=True,
        hardware_or_network_operations_authorized=False), ensure_ascii=False, indent=2) + '\n').encode()
    with output.open('xb') as stream:
        stream.write(raw)
        stream.flush()
        os.fsync(stream.fileno())
    assert output.read_bytes() == raw
    assert checked(review_ref) == review
    storage.validate(output, expected=binding)
    return runner.main(remaining)


if __name__ == '__main__':
    sys.exit(main())
