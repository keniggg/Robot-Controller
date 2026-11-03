"""Write the bounded GEN28 independent review after checking frozen evidence."""
import ast
import hashlib
import json
import os
from pathlib import Path
import time
import xml.etree.ElementTree as ET

import numpy as np

W = Path(__file__).resolve().parents[1]
M = Path('/media/zhuyupei/SanStand')
N = M / '实验记录/Robot-Controller/ros_log/full_latest_gen22_restart_20261015T003700Z'
V = N / 'verification'
BASE = V / 'gen27_release_manifest.json'
BASE_SHA = '3f041d7644e0de907ed97fc90ddffaa47d6c93a8f858c1c4763140ddc123e789'
COMBINED = V / 'gen28_combined_r1/result.json'
ENTRY = V / 'gen28_actual_raw_box_entry_r3/result.json'
ENTRY_SHA = '74e002d57bbfc792994acc6281aabbf98073331a9cf274bc1d4b83674fabf7b2'
SEAL_SHA = 'ae0e5774eba84be81ec6bb59c1c3a84ba4ee3e86bc3ac868b7a9f4d59b8ba4e0'
OUT = V / 'gen28_final_independent_review/review.json'
PRODUCTION = {
    'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/regular_box_inference.py': '9006b9b3b416cb7b5a43b58e9ab420e8d7294442235eaf7a95673f6188729c0f',
    'src/alicia_flexible_grasp_supervisor/scripts/remote_grasp6d_node.py': '27f33fc0079fa54a8d905966d192ba22cba424d4cc00d5bdf076953948ffc275',
}
FIXTURE = 'src/alicia_flexible_grasp_supervisor/tests/test_gen26_inferred_box_remote_pipeline.py'
NEW_TESTS = {
    'src/alicia_flexible_grasp_supervisor/tests/test_gen28_retained_box_source.py',
    'src/alicia_flexible_grasp_supervisor/tests/test_gen28_raw_box_source_independent.py',
}
SCRIPTS = {
    '.runtime/verify_gen28_combined_20261015.py',
    '.runtime/seal_gen28_release_20261015.py',
    '.runtime/replay_gen28_actual_raw_box_entry.py',
}


def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as file:
        for block in iter(lambda: file.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def read(path):
    return json.loads(Path(path).read_text())


def canonical_hash(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'),
        allow_nan=False, ensure_ascii=True).encode()).hexdigest()


def main():
    assert M.is_mount() and not os.statvfs(M).f_flag & os.ST_RDONLY
    assert not OUT.parent.exists()
    assert sha(BASE) == BASE_SHA and sha(ENTRY) == ENTRY_SHA
    assert sha(W / '.runtime/seal_gen28_release_20261015.py') == SEAL_SHA
    prior, combined, entry = read(BASE), read(COMBINED), read(ENTRY)
    assert combined['passed'] is True and combined['source_unchanged'] is True
    assert combined['scope_complete'] and combined['baseline_file_test_counts_retained']
    assert combined['returncode'] == 0 and combined['regression_scope'] == 'full_combined'
    assert combined['summary'] == dict(tests=7067, failures=0, errors=0, skipped=0)
    assert combined['source_count'] == 965 and combined['test_scope_files'] == 230
    sources, scope = combined['source_sha256'], combined['test_scope']
    assert set(sources) == set(prior['source_sha256']) | NEW_TESTS
    assert set(scope) == set(prior['test_scope']) | NEW_TESTS
    assert all(sha(W / name) == digest for name, digest in sources.items())
    changed = {name for name, digest in prior['source_sha256'].items() if sources[name] != digest}
    assert changed == set(PRODUCTION) | {FIXTURE}
    assert all(sources[name] == digest for name, digest in PRODUCTION.items())
    assert all(combined['test_case_counts_by_file'][name] >= count
        for name, count in prior['test_case_counts_by_file'].items())
    evidence = {}

    def bind(path, expected=None):
        path = Path(path).resolve(strict=True)
        digest = sha(path)
        assert expected is None or digest == expected, str(path)
        evidence[str(path)] = digest
        return path

    bind(BASE, BASE_SHA)
    bind(COMBINED)
    bind(ENTRY, ENTRY_SHA)
    patch = read(bind(combined['patch'], combined['patch_sha256']))
    assert set(patch['files']) == changed | NEW_TESTS and not patch['new_resources']
    for path, digest in patch['evidence_sha256'].items():
        bind(path, digest)
    bind(COMBINED.parent / 'pytest.log', combined['log_sha256'])
    xml = ET.parse(bind(COMBINED.parent / 'pytest.xml', combined['xml_sha256'])).getroot()
    actual = {key: sum(int(s.get(key, 0)) for s in xml.iter('testsuite'))
        for key in ('tests', 'failures', 'errors', 'skipped')}
    assert actual == combined['summary']
    counts = dict.fromkeys(scope, 0)
    for case in xml.iter('testcase'):
        assert case.get('file') in counts
        counts[case.get('file')] += 1
    assert counts == combined['test_case_counts_by_file']
    assert sha(W / '.runtime/verify_gen28_combined_20261015.py') == combined['runner_sha256']
    for name, digest in entry['source_sha256'].items():
        assert sha(W / name) == digest
        if name in sources:
            assert sources[name] == digest
        else:
            assert name == '.runtime/replay_gen28_actual_raw_box_entry.py'
    for path, digest in entry['input_sha256'].items():
        bind(path, digest)
    for flag in ('actual_production_entry_called', 'corrected_bbox_xywh', 'raw_model_qualified',
                 'fused_geometry_preserved', 'raw_source_hashes_match', 'source_unchanged',
                 'actual_production_generator_called', 'snapshot_reconstructed_with_actual_fuser',
                 'fusion_arrays_match_corrected_replay'):
        assert entry[flag] is True, flag
    assert entry['fused_pixel_baseline_qualified'] is False
    assert entry['physical_grasp_verified'] is False
    assert entry['candidate_count'] == 0
    assert entry['candidate_generation_failure_code'] == 'BOX_INFERENCE_PRECISION_VIEW_REQUIRED'
    assert entry['original_bbox_xywh'] == [264, 202, 111, 74]
    assert entry['measured_points_sha256_before'] == entry['measured_points_sha256_after']
    model = entry['model']
    assert model['model_sha256'] == canonical_hash({k: v for k, v in model.items() if k != 'model_sha256'})
    raw_path = next(Path(p) for p in entry['input_sha256'] if p.endswith('source_1792050907106353521.npz'))
    with np.load(raw_path, allow_pickle=False) as raw:
        for key, field in [('color', 'color_sha256'), ('depth_raw', 'depth_sha256'), ('object_mask', 'mask_sha256')]:
            assert hashlib.sha256(np.ascontiguousarray(raw[key]).tobytes()).hexdigest() == model['source'][field]
    geometry_path = next(Path(p) for p in entry['input_sha256'] if p.endswith('initial_three_frame_geometry.json'))
    g = read(geometry_path)
    payload = dict(center=g['obb_center_base_m'], axes=g['R_base_obb'], size=g['obb_size_xyz_m'],
        support_normal=g['support_normal_base'], support_offset=g['support_offset_m'],
        support_inlier_ratio=g['support_inlier_ratio'], measured_points_sha256=hashlib.sha256(
            np.ascontiguousarray(g['object_points_base_m'], dtype=float).tobytes()).hexdigest())
    assert canonical_hash(payload) == model['source']['geometry_sha256']
    for relative in PRODUCTION:
        old = ast.parse((W / '.runtime/gen28_baseline_source' / relative).read_text())
        new = ast.parse((W / relative).read_text())
        if relative.endswith('regular_box_inference.py'):
            definitions = lambda tree: {n.name: ast.dump(n, include_attributes=False)
                for n in tree.body if isinstance(n, (ast.FunctionDef, ast.ClassDef))}
            before, after = definitions(old), definitions(new)
            assert all(after.get(name) == body for name, body in before.items())
            assert set(after) - set(before) == {'qualify_regular_box_snapshot'}
    reviewed = {name: sha(W / name) for name in set(PRODUCTION) | {FIXTURE} | NEW_TESTS | SCRIPTS}
    result = dict(schema_version=1, reviewed=True, blocking_findings=[],
        reviewed_ns=time.time_ns(), reviewer='independent_gen26_review',
        reviewed_source_sha256=reviewed, evidence_sha256=evidence,
        full_regression=combined['summary'], source_count=965, test_scope_files=230,
        assessment=[
            'Only the box source adapter and remote call changed; all existing qualifier functions, thresholds and uncertainty formulas remain unchanged.',
            'Raw RGB/depth/mask bind to retained final sample, current observation identity/stamp and normalized final input mask hash; unavailable/mismatched raw inputs cannot borrow fused pixels.',
            'The unchanged fused measured geometry has a separate digest; uncertainty enclosure, signed plan, execution and close consumers preserve their existing contracts.',
            'Combined XML counts and all 965 current source hashes independently matched; all 228 prior files retain at least their prior case count.',
            'Corrected xywh replay calls the actual production adapter and generator; raw model qualifies while old fused-pixel qualification fails.',
        ], limitations=[
            'GEN28 fixes input provenance only. The actual replay generates zero candidates and retains BOX_INFERENCE_PRECISION_VIEW_REQUIRED.',
            'No physical grasp, robot motion, deployment or complete user objective is certified by this review.',
            'Old initial three-frame geometry used an incorrect xyxy bbox; retained only as diagnostic history, never as proof of corrected geometry.',
            *entry['limitations'],
        ], physical_grasp_verified=False, live_ros_contacted=False,
        motion_commands_sent=False, temperature_queried=False)
    assert all(sha(W / name) == digest for name, digest in sources.items())
    assert all(sha(W / name) == digest for name, digest in reviewed.items())
    OUT.parent.mkdir()
    encoded = json.dumps(result, indent=2, ensure_ascii=False, allow_nan=False) + '\n'
    with OUT.open('x') as file:
        file.write(encoded)
        file.flush()
        os.fsync(file.fileno())
    assert OUT.read_text() == encoded
    print(json.dumps(dict(review=str(OUT), sha256=sha(OUT), reviewed=True,
        summary=combined['summary'], candidate_count=entry['candidate_count']), ensure_ascii=False))


if __name__ == '__main__':
    main()
