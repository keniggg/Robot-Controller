"""GEN55: manifest-bound configurable offline pytest; plan-only default.

This helper is outside the current release. It does not install/deploy anything
or run on import. Focus scope comes from the hash-bound candidate manifest.
Full mode requires its explicit full_scope_ready flag and closed case inventory.
Every child has separate temporary, ROS-log, Python guard, import-provenance and
XML files. The coordinator requires exact testcase identities, not just totals.
It reuses the frozen serial runner's source/compatibility/guard contracts.
"""
import argparse
from collections import Counter
from copy import deepcopy
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import signal
import resource
import subprocess
import sys
import time
import traceback
import xml.etree.ElementTree as ET

PROJECT = Path(__file__).resolve().parents[1]
W = PROJECT
MANIFEST_PATH = None
MANIFEST_SHA = None
SERIAL = W/'.runtime/verify_gen43_offline_tests_20261018_r2.py'
SERIAL_SHA = 'de1b3727a78644ade82690dbfd748745c4e03969eda18556bd1192b54c55698f'
E = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
V = E/'ros_log/full_latest_gen41_restart_20261018T011222Z/verification'
HISTORY = V/'gen53_full_parallel_r1/result.json'
HISTORY_SHA = '13a428ac36f300a18a80adb00603c09f6ee70c90342d6a847afeac125883ff8a'
CALIBRATION = V/'gen43_two_process_pilot_monitor_r1/verification.json'
CALIBRATION_SHA = 'e8361d8a905351de2be64cd259008c3bc97193f4a0ec1cb16bf26306ceb3066b'
PRIOR_TWO_WORKER = W/'.runtime/verify_gen43_two_process_draft_20261018.py'
PRIOR_TWO_WORKER_SHA = '82c6efbf6947577c561ea7427d4dd8fd346b16f1a7ee0f5302d384751564be3b'
GEN54_BASELINE = PROJECT/'.runtime/gen54_baseline_source/runner_manifest.json'
GEN54_BASELINE_SHA = 'e0ea9529ff6afdff8a6794578f1e64d996cdb124ee1adf3e6732e38e44984748'
GEN53_FULL_XML_SHA = '3ce8c6f4e6de0be68745042e113f2e727f7b8f3cdc862e4728dca0523b8ebb78'
GEN54_RELEASE = V/'gen54_release_manifest.json'
GEN54_RELEASE_SHA = 'ec0d1c1d2ebdb88fc9c77174790190dfff71024e58fea6958b356cb9ebf5d3f8'
GEN55_REVIEWED_TEST = 'src/alicia_flexible_grasp_supervisor/tests/test_gen54_collision_contract.py'
GEN55_REVIEWED_TEST_BASELINE_SHA = '61aa4b25bd6e8b4d7103607eced1e3eda266e016b5ba44af9408d943b358d1f5'
GEN55_REVIEWED_TERMINAL_TEST = 'src/alicia_flexible_grasp_supervisor/tests/test_gen52_contact_terminal_collision_screen.py'
GEN55_REVIEWED_SEQUENCE_TEST = 'src/alicia_flexible_grasp_supervisor/tests/test_contact_sequence_precheck.py'
GEN55_REVIEWED_STAGING_TEST = 'src/alicia_flexible_grasp_supervisor/tests/test_gen38_contact_replay_staging.py'
GEN55_TEST_REVIEW_CONTRACTS = {
    GEN55_REVIEWED_TEST: (GEN55_REVIEWED_TEST_BASELINE_SHA, 7, 17),
    GEN55_REVIEWED_TERMINAL_TEST: ('e4596d78aa4d19fc07f5c72519902c126190390408ce9b8f742942363be080ad', 2, 11),
    GEN55_REVIEWED_SEQUENCE_TEST: ('9a5fab072320da83879a5a3d285801016777c288b0dbedee3abc9f47f00cb7f9', 1, 32),
    GEN55_REVIEWED_STAGING_TEST: ('9b220d41ff37de9988dd57b5d7d739310200ee8715a060c1aa1a9c8c90600a24', 1, 10),
}
BALANCE_HISTORY = V/'gen54_full_parallel_r1/result.json'
BALANCE_HISTORY_SHA = '6963ef9c30e17a4b94c0fc0aee62476bce29af4641c3966ad561c0ec05728adb'
SUPPLEMENTAL_CAD_INPUTS = {
    'src/arm-mujoco/synriard/urdf/Alicia_D_v5_6/Alicia_D_v5_6_gripper_50mm.urdf': '7e08a01f1a6cf3b45ed78a26d3a9f5939e2bb6bc33f6bc01032db097882edaeb',
    'src/arm-mujoco/synriard/meshes/Alicia_D_v5_6/gripper_50mm/Link6.STL': 'cff46c5522d1a869b0397965adb885a7f4c62c5b137b409d8df605aab3cc3d5d',
    'src/arm-mujoco/synriard/meshes/Alicia_D_v5_6/gripper_50mm/Link7.STL': 'd546b7ab908c74281a41261412089a2f83f7519c2444739afe8ffff46450d19d',
    'src/arm-mujoco/synriard/meshes/Alicia_D_v5_6/gripper_50mm/Link8.STL': '5e5b548734269b731c1d60502c742e3d70d5a97a1e8316db523e2232b4c5043b',
    'src/real-arm/alicia_d_descriptions/urdf/alicia_duo_with_gripper.urdf': '890592a24b8ad818d995e69beb670ea4254bf84f589615bfac3c8562f62449b7',
    'src/real-arm/alicia_d_descriptions/meshes/Link7.STL': 'ff006151147d25c431d868fc6a0f46b96275690d30526d13c741ffe51ac6fcea',
    'src/real-arm/alicia_d_descriptions/meshes/Link8.STL': 'fd49b1a4c9ecf39854d0de1ee3645239aae2427cca6ce7a24b195bf041c6098e',
    'src/real-arm/alicia_d_moveit/config/alicia_d_descriptions.srdf': 'ac14d9a4de083d30444b3c8e762bc166a61382ec334ac455a93ac792ae6a42c2',
}
REVIEWED_OLD_TEST_CHANGES = [
    dict(path='src/alicia_flexible_grasp_supervisor/tests/test_gripper_geometry.py',
         baseline_sha256='0b2b44da8962476a81a4cdfcd9655b5e94cd20160eb6a334cd0766d9ddd3d45f',
         candidate_sha256='4fe13a1336b611b4417ea2944ad1020d052b7586d994ed829f562415116dd06b',
         review_evidence=dict(path=str(V/'gen54_collision_geometry_r1/near_limit_versioned_expectation_review.json'),
                              sha256='431c590a5d7304a29a2a9c80316a7122529ac6c7381d41297c275b28e19b8a2a')),
    dict(path='src/alicia_flexible_grasp_supervisor/tests/test_remote_grasp6d_streaming.py',
         baseline_sha256='6957b2d808da254dface00bd025791362a44c98e93765980de25a74bb1e68638',
         candidate_sha256='b6b4c8004caadc7826969568bd81a5964d65b87299d5836898cd72ea4f1d9c77',
         review_evidence=dict(path=str(V/'gen54_old_expectation_review_r1/recorded_expectation_review.json'),
                              sha256='f1884d032e508c43e268d3a2ba2929b9127199a2643ac1bf7661b5066b52a1de')),
    dict(path='src/alicia_flexible_grasp_supervisor/tests/test_contact_height_window_proposals.py',
         baseline_sha256='331388555feb1323fdb8013e4745113826662d4013796d0462e2c0393be87a11',
         candidate_sha256='d8ed14b42ccb3a1a8681e7791490097143f7a14d97ca7bb392fbf3f4e4b52201',
         review_evidence=dict(path=str(V/'gen54_geometry_pipeline_r1/height_cardinality_only_old_test_review_r1.json'),
                              sha256='55df9ba96aed22b77be4db213859b41c59cdea3d90b0ffc853a85471109cbf1b')),
]


def bind_manifest(path, expected_sha):
    global MANIFEST_PATH,MANIFEST_SHA,W
    MANIFEST_PATH=Path(path).resolve(strict=True);MANIFEST_SHA=expected_sha
    assert sha(MANIFEST_PATH)==MANIFEST_SHA
    manifest=read(MANIFEST_PATH)
    assert manifest['kind']=='GEN55_frozen_offline_candidate'
    W=Path(manifest['workspace_root']).resolve(strict=True)
    assert W==PROJECT or PROJECT/'.runtime' in W.parents
    return manifest


def reference(row):
    path=Path(row['path']).resolve(strict=True)
    assert sha(path)==row['sha256']
    return path


def resource_budget(args):
    assert 1<=args.worker_cap<=8 and args.reserve_cpus>=0
    assert args.memory_per_worker_mib>0 and args.reserve_memory_mib>=0
    affinity=sorted(os.sched_getaffinity(0))
    quota_cores=None;quota={}
    v2=Path('/sys/fs/cgroup/cpu.max')
    v1=Path('/sys/fs/cgroup/cpu/cpu.cfs_quota_us')
    if v2.exists():
        value,period=v2.read_text().split();quota=dict(path=str(v2),value=value,period=int(period))
        if value!='max':quota_cores=int(value)/int(period)
    elif v1.exists():
        value=int(v1.read_text());period=int(v1.with_name('cpu.cfs_period_us').read_text());quota=dict(path=str(v1),value=value,period=period)
        if value>0:quota_cores=value/period
    effective=len(affinity) if quota_cores is None else min(len(affinity),math.floor(quota_cores))
    cpu_workers=max(0,effective-args.reserve_cpus)
    mem={k:int(v.split()[0]) for k,v in (l.split(':',1) for l in Path('/proc/meminfo').read_text().splitlines())}
    available_mib=mem['MemAvailable']/1024.
    memory_workers=max(0,math.floor((available_mib-args.reserve_memory_mib)/args.memory_per_worker_mib))
    capacity=min(args.worker_cap,cpu_workers,memory_workers)
    selected=max(1,capacity) if args.workers=='auto' else int(args.workers)
    assert 1<=selected<=args.worker_cap
    calibration=read(CALIBRATION);assert sha(CALIBRATION)==CALIBRATION_SHA
    observed=max(float(p['peak_rss_kib'].split()[0])/1024. for key in ('first_sample','second_sample')
        for p in calibration['monitoring'][key]['processes'] if 'peak_rss_kib' in p)
    return dict(requested_workers=args.workers,selected_workers=selected,worker_cap=args.worker_cap,
        affinity_cpus=affinity,os_cpu_count=os.cpu_count(),cgroup_quota=quota,effective_cpu_count=effective,
        reserved_cpu_count=args.reserve_cpus,cpu_worker_capacity=cpu_workers,
        mem_total_mib=mem['MemTotal']/1024.,mem_available_mib=available_mib,
        reserved_memory_mib=args.reserve_memory_mib,memory_per_worker_mib=args.memory_per_worker_mib,
        memory_worker_capacity=memory_workers,resource_worker_capacity=capacity,
        execution_resources_sufficient=(selected<=capacity),load_average=list(os.getloadavg()),
        measured_pilot_peak_rss_mib=observed,calibration_path=str(CALIBRATION),calibration_sha256=CALIBRATION_SHA,
        budget_reason='One CPU per worker; leave four default CPU slots for online control. Per-worker 768MiB default exceeds observed pilot peak about315MiB; reserve an additional2GiB of current MemAvailable. These are planning margins, not a full-suite peak guarantee.')


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read(path):
    return json.loads(Path(path).read_bytes())


def save(path, value):
    with Path(path).open('x') as f:
        json.dump(value, f, ensure_ascii=False, indent=2, allow_nan=False)
        f.write('\n'); f.flush(); os.fsync(f.fileno())


def load_serial():
    assert sha(SERIAL) == SERIAL_SHA
    spec = importlib.util.spec_from_file_location('gen43_frozen_serial_contract', SERIAL)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)  # Definitions only: guarded main is not called.
    return module


def bootstrap_sources(serial):
    guard, audit = serial.bootstrap_sources()
    original = "'remote_grasp6d' in name or name.startswith('alicia_grasp_modes')"
    assert audit.count(original) == 1
    audit = audit.replace(original,
        "'remote_grasp6d' in name or Path(path).name in ('grasp_task_node.py', 'mode_aware_remote_grasp6d.py', 'unknown_tabletop_perception.py') or name.startswith('alicia_grasp_modes')")
    return guard, audit


def current_context(serial):
    assert MANIFEST_PATH is not None and sha(MANIFEST_PATH)==MANIFEST_SHA
    manifest=read(MANIFEST_PATH)
    assert reference(manifest['baseline_manifest'])==GEN54_BASELINE and sha(GEN54_BASELINE)==GEN54_BASELINE_SHA
    baseline=read(GEN54_BASELINE);original=baseline['source_sha256']
    assert baseline['kind']=='GEN54_exact_GEN53_baseline_source'
    assert baseline['baseline_generation']=='GEN53'
    assert Path(baseline['workspace_root']).resolve(strict=True)==GEN54_BASELINE.parent
    assert Path(baseline['live_workspace']).resolve(strict=True)==PROJECT
    assert baseline['source_count']==len(original)==1069 and baseline['test_scope_files']==322
    assert baseline['all_source_bytes_exact'] is True
    assert Path(baseline['original_full_regression']).resolve(strict=True)==HISTORY and baseline['original_full_regression_sha256']==HISTORY_SHA
    assert all(sha(GEN54_BASELINE.parent/p)==h for p,h in original.items())
    assert sha(HISTORY)==HISTORY_SHA
    release=Path(baseline['baseline_release'])
    assert release.resolve(strict=True)==(V/'gen53_release_manifest.json').resolve(strict=True)
    assert sha(release)==baseline['baseline_release_sha256']=='e9f07af55bfac1ae9fd7562d93f04748b6b2e68139f7bab3e0f645cc46ef255f'
    sealed=read(release);assert sealed['source_sha256']==original
    history=read(HISTORY);xml=HISTORY.parent/'pytest.xml'
    assert history['passed'] and history['source_sha256']==original and sha(xml)==history['xml_sha256']==GEN53_FULL_XML_SHA
    summary,old_cases,_=inventory(xml)
    counts=dict(Counter(c[0] for c in old_cases))
    assert summary==dict(tests=9010,failures=0,errors=0,skipped=0) and len(counts)==322
    assert history['test_case_counts_by_file']==baseline['test_case_counts_by_file']==counts
    assert baseline['test_summary']==summary
    assert len(baseline['test_scope'])==len(set(baseline['test_scope']))==322 and set(baseline['test_scope'])==set(counts)
    baseline_scope=list(history['test_case_counts_by_file'])
    assert set(baseline_scope)<=set(original)
    variant=manifest.get('variant','candidate');assert variant in ('candidate','baseline')
    allowed=set(manifest['allowed_production_paths'])
    assert (allowed or variant=='baseline') and all(p in original and '/tests/' not in p and p.startswith('src/') and p.endswith('.py') for p in allowed)
    expected=dict(original);changes=manifest['production_source_changes'];seen=set()
    for row in changes:
        path=row['path'];assert path in allowed and path not in seen
        seen.add(path);assert row['baseline_sha256']==original[path]
        assert row['candidate_sha256']!=original[path]
        if variant=='candidate':expected[path]=row['candidate_sha256']
    assert seen==allowed
    # Explicit new production modules are pinned and reviewed independently
    # of old-path changes; they cannot replace old tests or fixtures.
    new_production = manifest.get('new_production_sources', [])
    new_production_paths = set()
    for row in new_production:
        path = row['path']; parsed = Path(path)
        assert variant == 'candidate'
        assert path.startswith('src/') and '/src/' in path and path.endswith('.py')
        assert '/tests/' not in path and not parsed.is_absolute() and '..' not in parsed.parts
        assert path not in original and path not in expected and path not in new_production_paths
        assert len(row['sha256']) == 64 and row.get('review_evidence')
        evidence = reference(row['review_evidence'])
        review = read(evidence)
        assert review['passed'] is True and review['owned_source_sha256'][path] == row['sha256']
        new_production_paths.add(path); expected[path] = row['sha256']
    changed_or_new = allowed | new_production_paths
    assert not manifest.get('approved_fixture_changes'), 'Existing fixture bytes must remain unchanged.'
    fixtures={};dependencies={str(release):sha(release)}
    # Only these exact reviewed test-contract updates are permitted: two
    # unsafe old expectations and the explicit three-to-four proposal budget.
    # Their original case identities remain in scope;
    # all other old test bytes and all original input fixtures stay exact.
    old_test_changes=manifest.get('approved_old_test_changes', [])
    assert not old_test_changes or old_test_changes == REVIEWED_OLD_TEST_CHANGES
    for row in old_test_changes:
        path=row['path']
        assert variant == 'candidate' and row['baseline_sha256'] == original[path]
        assert path not in allowed and path not in fixtures
        evidence=reference(row['review_evidence'])
        dependencies[str(evidence)]=row['review_evidence']['sha256']
        fixtures[path]=row
        expected[path]=row['candidate_sha256']
    new=manifest.get('new_tests',[])
    supplements=manifest.get('supplemental_existing_tests',[])
    new_paths=[];supplement_paths=[];new_cases=[];expected_counts=dict(counts)
    for row in new_production:
        evidence=reference(row['review_evidence']);dependencies[str(evidence)]=row['review_evidence']['sha256']
    for row in new+supplements:
        path=row['path'];assert path not in new_paths+supplement_paths
        if row in new:
            assert path not in original
            assert any(path.startswith((root+'/test_gen54_', root+'/test_gen55_')) and path.endswith('.py') for root in serial.ROOTS)
            new_paths.append(path);expected[path]=row['sha256']
        else:
            assert path in original and path not in counts and row['sha256']==expected[path]
            assert any(path.startswith(root+'/test_') and path.endswith('.py') for root in serial.ROOTS)
            supplement_paths.append(path)
        cases=row.get('case_identities')
        if cases is not None:
            assert len(cases)==row['expected_count']>0 and all(c[0]==path and len(c)==3 for c in cases)
            assert len(cases)==len(set(map(tuple,cases)))
            expected_counts[path]=row['expected_count'];new_cases+=cases
            evidence=reference(row['collection_evidence']);dependencies[str(evidence)]=row['collection_evidence']['sha256']
            collection=read(evidence)
            assert collection['mode']=='collect' and collection['passed'] and collection['source_unchanged']
            assert collection['runner_sha256']==sha(__file__)
            assert collection['source_sha256']==manifest['source_sha256']
            assert not collection['test_bodies_executed'] and not collection['full_regression_passed']
            assert not collection['external_boundary_attempts'] and not collection['foreign_or_inconsistent_imports']
            assert sorted(map(tuple,cases))==sorted(tuple(c) for c in collection['collected_case_identities'] if c[0]==path)
    data_fixtures=manifest.get('new_fixture_sources',[]);seen_fixtures=set()
    for row in data_fixtures:
        path=row['path'];parsed=Path(path)
        assert any(path.startswith(root+'/fixtures/') for root in serial.ROOTS)
        assert not parsed.is_absolute() and '..' not in parsed.parts
        assert parsed.suffix.lower() not in ('.py','.pyc','.pyo','.so','.sh','.bash')
        assert path not in original and path not in expected and path not in seen_fixtures
        seen_fixtures.add(path);expected[path]=row['sha256']
    # GEN55 permits only the exact test files and case counts listed above.
    # The reviewed hash pairs cannot authorize changes to another old test,
    # rename old cases, or reclassify any historical physical failure.
    gen55_reviews = manifest.get('approved_gen55_test_changes', [])
    assert len(gen55_reviews) <= len(GEN55_TEST_REVIEW_CONTRACTS)
    gen55_reviews_by_path = {}
    for row in gen55_reviews:
        path = row['path'];assert path in GEN55_TEST_REVIEW_CONTRACTS and path not in gen55_reviews_by_path
        baseline_digest, changed_count, original_count = GEN55_TEST_REVIEW_CONTRACTS[path]
        assert row['baseline_sha256'] == baseline_digest
        assert row['candidate_sha256'] != baseline_digest and len(row['candidate_sha256']) == 64
        if path in original:
            assert original[path] == expected[path] == baseline_digest
            expected[path] = row['candidate_sha256']
        else:
            assert expected[path] == row['candidate_sha256']
        evidence = reference(row['review_evidence']); review = read(evidence)
        assert review['passed'] is True and review['path'] == row['path']
        assert review['baseline_sha256'] == row['baseline_sha256']
        assert review['candidate_sha256'] == row['candidate_sha256']
        reviewed_cases = review['changed_case_identities']
        assert len(reviewed_cases) == len(set(map(tuple, reviewed_cases))) == changed_count
        assert all(len(c) == 3 and c[0] == row['path'] for c in reviewed_cases)
        assert review['unchanged_case_identities_preserved'] is True
        assert review['historical_failures_reclassified'] is False
        assert review['original_case_count'] == original_count
        assert len(review['all_original_case_identities']) == original_count
        gen55_reviews_by_path[path] = (row,review)
        dependencies[str(evidence)] = row['review_evidence']['sha256']
    for row in manifest.get('additional_review_evidence', []):
        evidence = reference(row); dependencies[str(evidence)] = row['sha256']
    for row in new_production:
        review = read(reference(row['review_evidence']))
        for mapping in ('owned_source_sha256','reviewed_related_sources'):
            assert all(expected.get(path) == digest for path,digest in review[mapping].items())
        for asset in review['required_CAD_assets']:
            relative = asset['path']
            assert not Path(relative).is_absolute() and '..' not in Path(relative).parts
            for root in (PROJECT,W):
                path=root/relative;assert path.resolve(strict=True)==path and sha(path)==asset['sha256']
                dependencies[str(path)]=asset['sha256']
    # Existing CAD tests read these eight non-code artifacts directly. Pin
    # both the project originals and separate candidate copies; never expose
    # live src as an import fallback or silently expand the code source map.
    if ('src/alicia_flexible_grasp_supervisor/tests/test_gripper_geometry.py' in supplement_paths
            or 'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot/finger_cad_separation.py' in new_production_paths):
        for relative, digest in SUPPLEMENTAL_CAD_INPUTS.items():
            assert relative not in expected
            for root in (PROJECT, W):
                path = root/relative
                assert path.resolve(strict=True) == path
                assert sha(path) == digest
                dependencies[str(path)] = digest
    assert manifest['source_sha256']==expected
    assert all(not Path(p).is_absolute() and '..' not in Path(p).parts for p in expected)
    sources={p:sha(W/p) for p in expected};assert sources==expected
    scope=baseline_scope+supplement_paths+new_paths;assert len(scope)==len(set(scope))
    all_cases=old_cases+new_cases;assert len(all_cases)==len(set(map(tuple,all_cases)))
    # Preserve every GEN54 test/fixture byte and completed case identity too.
    # Its interrupted launcher remains historical; no missing exit is recast.
    assert sha(GEN54_RELEASE) == GEN54_RELEASE_SHA
    prior_release = read(GEN54_RELEASE)
    assert prior_release['source_count'] == len(prior_release['source_sha256']) == 1075
    assert sha(BALANCE_HISTORY) == BALANCE_HISTORY_SHA
    prior_full = read(BALANCE_HISTORY)
    prior_xml = BALANCE_HISTORY.parent/'pytest.xml'
    assert sha(prior_xml) == prior_full['xml_sha256']
    prior_summary, prior_cases, _ = inventory(prior_xml)
    assert prior_summary == dict(tests=9255, failures=0, errors=0, skipped=0)
    assert prior_full['source_sha256'] == prior_release['source_sha256']
    assert prior_full['full_regression_passed'] is True
    for path, digest in prior_release['source_sha256'].items():
        assert path in expected
        if path in gen55_reviews_by_path:
            row,review = gen55_reviews_by_path[path]
            prior_file_cases = {tuple(c) for c in prior_cases if c[0] == path}
            assert digest == row['baseline_sha256'] and expected[path] == row['candidate_sha256']
            assert set(map(tuple,review['changed_case_identities'])) <= prior_file_cases
            assert set(map(tuple,review['all_original_case_identities'])) == prior_file_cases
            continue
        if path not in allowed or '/tests/' in path:
            assert expected[path] == digest, 'GEN54 baseline input changed: '+path
    if manifest.get('full_scope_ready'):
        assert set(map(tuple, prior_cases)) <= set(map(tuple, all_cases))
    for path in (GEN54_RELEASE, BALANCE_HISTORY, prior_xml):
        dependencies[str(path)] = sha(path)
    required=manifest.get('required_changed_sources',sorted(changed_or_new));assert set(required)==changed_or_new
    per_file=manifest.get('required_imports_by_test_file',{})
    assert set(per_file)<=set(scope) and all(set(v)<=changed_or_new for v in per_file.values())
    # Existing source loaders may import generated ROS messages. Bind their
    # exact historical module bytes; an isolated workspace cannot import live
    # PROJECT/src as a fallback.
    provenance=V/'gen43_combined_r2/module_provenance.json'
    historical_provenance_result=V/'gen43_combined_r2/result.json'
    assert sha(historical_provenance_result)=='ac2a76cc400ffc644c02ad6175557fa905da94f54d72abbe413e838f5a50e299'
    assert sha(provenance)==read(historical_provenance_result)['module_provenance_sha256']
    dependencies[str(historical_provenance_result)]=sha(historical_provenance_result)
    generated={r['path']:r['sha256'] for r in read(provenance).values() if serial.inside(Path(r['path']),PROJECT/'devel')}
    dependencies[str(provenance)]=sha(provenance)
    # Historical AST comparator files used by the unchanged GEN38 replay
    # tests are read-only fixtures, not replacement production modules.
    historical_ast_inputs = {'.runtime/gen37_baseline_source/src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot/observation_path_guard.py': 'f3af52527035ad8211e785e3aeb14ecd596570bf236a989662547fd9ce73de9a', '.runtime/gen37_baseline_source/src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot/contact_sequence_precheck.py': 'dba703313e5bde18ef74b300f01d15f69132bc0ccbae2a3855eaf51e7e976f88'}
    for relative, digest in historical_ast_inputs.items():
        path = W/relative
        assert sha(path) == digest
        dependencies[str(path.resolve(strict=True))] = digest
    guard, audit = bootstrap_sources(serial)
    return dict(scope=scope, sources=sources, expected_counts=expected_counts,
        case_identities=all_cases,old_case_identities=old_cases,old_expected_counts=counts,
        new_fixture_sources=data_fixtures,
        required_changed_sources=required,required_imports_by_test_file=per_file,generated_imports=generated,
        compatibility=dict(variant=variant,production_source_changes=changes,new_production_sources=new_production,
            approved_old_test_changes=fixtures,approved_gen55_test_changes=gen55_reviews),
        compatibility_dependencies=dependencies,full_scope_ready=manifest.get('full_scope_ready',False),
        original=original,baseline_sha256=GEN54_BASELINE_SHA,
        guard_sha256=hashlib.sha256(guard.encode()).hexdigest(),
        audit_sha256=hashlib.sha256(audit.encode()).hexdigest())


def case_identity(case):
    return [case.get('file',''), case.get('classname',''), case.get('name','')]


def inventory(path):
    tree = ET.parse(path).getroot()
    summary = dict.fromkeys(('tests','failures','errors','skipped'),0)
    for suite in tree.iter('testsuite'):
        for key in summary: summary[key] += int(suite.get(key,0))
    cases = [case_identity(c) for c in tree.iter('testcase')]
    durations = {}
    for case in tree.iter('testcase'):
        seconds = float(case.get('time','0'))
        assert math.isfinite(seconds) and seconds >= 0
        durations[case.get('file','')] = durations.get(case.get('file',''),0.) + seconds
    assert summary['tests'] == len(cases)
    assert len(cases) == len({tuple(c) for c in cases}), 'duplicate testcase identities'
    for key,tag in (('failures','failure'),('errors','error'),('skipped','skipped')):
        assert summary[key] == sum(len(c.findall(tag)) for c in tree.iter('testcase')), 'XML child outcome mismatch: '+key
    return summary,cases,durations


def make_plan(serial, output, mode, resources):
    context = current_context(serial)
    manifest=read(MANIFEST_PATH);history=read(BALANCE_HISTORY);xml=BALANCE_HISTORY.parent/'pytest.xml'
    _,_,durations=inventory(xml)
    assert mode in ('focus','full','collect')
    if mode=='full':
        assert manifest.get('full_scope_ready') is True, 'GEN55 full scope is not yet frozen/approved.'
        assert manifest.get('new_tests') and all('case_identities' in r for r in manifest['new_tests']+manifest.get('supplemental_existing_tests',[]))
        actual_new={str(p.relative_to(W)) for root in serial.ROOTS for prefix in ('test_gen54_*.py','test_gen55_*.py') for p in (W/root).glob(prefix)}
        assert actual_new=={r['path'] for r in manifest['new_tests']}
        scope=context['scope']
    elif mode=='focus':
        scope=manifest['focus_scope']
        assert scope and len(scope)==len(set(scope)) and set(scope)<=set(context['scope'])
    else:
        scope=[r['path'] for r in manifest['new_tests']+manifest.get('supplemental_existing_tests',[])];assert scope
    assert mode=='collect' or all(p in context['expected_counts'] for p in scope), 'Guarded collection evidence is missing.'
    expected_counts = {} if mode=='collect' else {p:context['expected_counts'][p] for p in scope}
    scope_cases = [] if mode=='collect' else [c for c in context['case_identities'] if c[0] in expected_counts]
    assert len(scope_cases) == sum(expected_counts.values())
    # A three-file focus uses at most three workers; this never changes the
    # explicitly recorded 8-worker resource cap or full-scope semantics.
    worker_count=min(resources['selected_workers'],len(scope))
    resources=dict(resources,selected_workers=worker_count,
        execution_resources_sufficient=(worker_count<=resources['resource_worker_capacity']))
    # Every whole file is assigned once by historical LPT. Only shards owning a
    # changed-module contract test require that module to load. Every import in
    # every shard still has strict provenance, and the aggregate must load it.
    assigned = [[] for _ in range(worker_count)]
    loads = [0. for _ in range(worker_count)]
    rest = set(scope)
    missing_weight=max(1.,sum(durations.values())/len(durations))
    for name in sorted(rest,key=lambda n:(-durations.get(n,missing_weight),n)):
        index = min(range(worker_count),key=lambda i:(loads[i],i))
        assigned[index].append(name); loads[index] += durations.get(name,missing_weight)
    shards = []
    for index, names in enumerate(assigned):
        chosen = set(names)
        shard_scope = [n for n in scope if n in chosen]
        shards.append(dict(index=index,scope=shard_scope,test_files=len(shard_scope),
            required_changed_sources=[] if mode=='collect' else sorted({p for test in shard_scope for p in context['required_imports_by_test_file'].get(test,[])}),
            expected_count=0 if mode=='collect' else sum(expected_counts[p] for p in shard_scope),
            historical_case_time_sec=loads[index],
            case_identities=[c for c in scope_cases if c[0] in chosen]))
    all_scope=[p for s in shards for p in s['scope']]
    assert len(all_scope)==len(set(all_scope)) and set(all_scope)==set(scope)
    assert sum(s['expected_count'] for s in shards) == len(scope_cases)
    required=set(context['required_changed_sources']) if mode=='full' else {
        p for test in scope for p in context['required_imports_by_test_file'].get(test,[])}
    if not required:required=set(context['required_changed_sources'])
    if mode=='collect':required=set()
    return dict(schema=1,kind='GEN55_parallel_'+mode+'_offline_plan',
        mode=mode,candidate_manifest_path=str(MANIFEST_PATH),candidate_manifest_sha256=MANIFEST_SHA,
        required_changed_sources=sorted(required),
        worker_count=worker_count,resource_budget=resources,
        prior_verified_two_worker_runner=str(PRIOR_TWO_WORKER),prior_verified_two_worker_runner_sha256=PRIOR_TWO_WORKER_SHA,
        scope_kind=mode,test_scope=scope,expected_file_count=len(scope),
        expected_case_count=len(scope_cases),expected_counts_by_file=expected_counts,
        full_regression_scope=(mode=='full'),
        runner_path=str(Path(__file__).resolve()),runner_sha256=sha(__file__),
        frozen_serial_contract=str(SERIAL),frozen_serial_contract_sha256=SERIAL_SHA,
        historical_result=str(BALANCE_HISTORY),historical_result_sha256=BALANCE_HISTORY_SHA,
        historical_xml=str(xml),historical_xml_sha256=sha(xml),context=context,
        expected_case_identities=scope_cases,shards=shards,output=str(output),
        maximum_workers=resources['worker_cap'],historical_result_elapsed_sec=history['elapsed_sec'],
        historical_worker_count=history['worker_count'], historical_scope_files=history['test_scope_files'],
        duration_estimate_scope='historical testcase execution time only; not a measured speedup or wall-time promise',
        tests_executed=False,approved_for_current_release=False,ROS_or_hardware_used=False)


def group_exists(group):
    try: os.killpg(group,0); return True
    except ProcessLookupError: return False


def signal_group(group, sig, records):
    # The coordinator passes only the exact PGIDs it created using
    # start_new_session=True. Worker leader exit does not change that ownership.
    try:
        os.killpg(group,sig)
        records.append(dict(pgid=group,signal=signal.Signals(sig).name))
    except ProcessLookupError: pass


def cleanup_group(group, records, process):
    """Bounded cleanup, including children after their worker leader exits."""
    def exists():
        process.poll()  # Reap our leader even if only descendants still exist.
        return group_exists(group)
    if not exists(): return False
    signal_group(group,signal.SIGTERM,records)
    deadline=time.monotonic()+5.
    while exists() and time.monotonic()<deadline: time.sleep(.05)
    if exists():
        signal_group(group,signal.SIGKILL,records)
        deadline=time.monotonic()+2.
        while exists() and time.monotonic()<deadline: time.sleep(.05)
    return exists()


def process_stat(pid):
    """Read only a proc identity; never infer ownership from command text."""
    try:
        text = (Path('/proc')/str(pid)/'stat').read_text()
        tail = text[text.rfind(')')+2:].split()
        return dict(pid=int(pid), state=tail[0], ppid=int(tail[1]),
            pgid=int(tail[2]), session=int(tail[3]), start_ticks=int(tail[19]),
            user_ticks=int(tail[11]), system_ticks=int(tail[12]), rss_pages=int(tail[21]))
    except (FileNotFoundError, ProcessLookupError, PermissionError):
        return None


class WorkloadMonitor:
    """Measured offline process groups, kept separate from host-wide context."""
    def __init__(self, output):
        self.path = output/'workload_samples.jsonl'
        self.stream = self.path.open('x')
        self.clock_ticks = os.sysconf('SC_CLK_TCK')
        self.page_kib = os.sysconf('SC_PAGE_SIZE')/1024.
        self.next_sample = 0.
        self.samples = 0
        self.max_group_rss_kib = 0.
        self.peak_active_process_count = 0
        self.minimum_host_available_mib = None
        self.observed = {}

    def sample(self, processes, force=False):
        now = time.monotonic()
        if not force and now < self.next_sample:
            return
        self.next_sample = now+2.
        groups = {p.pid for p in processes if p.poll() is None}
        rows = []
        for entry in Path('/proc').iterdir():
            if not entry.name.isdecimal():
                continue
            row = process_stat(entry.name)
            if row is None or row['pgid'] not in groups:
                continue
            row['cpu_sec'] = (row['user_ticks']+row['system_ticks'])/self.clock_ticks
            row['rss_kib'] = row['rss_pages']*self.page_kib
            try:
                io = (entry/'io').read_text()
                row['io_bytes'] = {k:int(v) for k,v in (line.split(':',1) for line in io.splitlines())
                    if k in ('read_bytes','write_bytes','rchar','wchar')}
            except (FileNotFoundError, ProcessLookupError, PermissionError):
                row['io_bytes'] = None
            key = (row['pid'],row['start_ticks'])
            self.observed[key] = max(row['cpu_sec'],self.observed.get(key,0.))
            rows.append(row)
        memory = {k:int(v.split()[0]) for k,v in
            (line.split(':',1) for line in Path('/proc/meminfo').read_text().splitlines())}
        available = memory['MemAvailable']/1024.
        self.minimum_host_available_mib = available if self.minimum_host_available_mib is None else min(available,self.minimum_host_available_mib)
        rss = sum(row['rss_kib'] for row in rows)
        self.max_group_rss_kib = max(self.max_group_rss_kib,rss)
        self.peak_active_process_count = max(self.peak_active_process_count,len(rows))
        record = dict(monotonic_sec=now,at_ns=time.time_ns(),owned_worker_pgids=sorted(groups),
            offline_process_groups=dict(processes=rows,simultaneous_rss_kib=rss,
                sampled_accumulated_cpu_sec=sum(self.observed.values())),
            coordinator=process_stat(os.getpid()),
            host_context=dict(load_average=list(os.getloadavg()),mem_available_mib=available,
                mem_total_mib=memory['MemTotal']/1024.,cpu_stat=Path('/proc/stat').read_text().splitlines()[0]))
        self.stream.write(json.dumps(record,allow_nan=False)+'\n');self.stream.flush()
        self.samples += 1

    def close(self):
        self.stream.flush();os.fsync(self.stream.fileno());self.stream.close()
        return dict(path=str(self.path),sha256=sha(self.path),sample_count=self.samples,
            sample_interval_sec=2.,peak_sampled_group_rss_kib=self.max_group_rss_kib,
            peak_sampled_offline_process_count=self.peak_active_process_count,
            observed_process_cpu_sec=sum(self.observed.values()),
            minimum_host_mem_available_mib=self.minimum_host_available_mib,
            cpu_scope='Per-process CPU for exact worker PGIDs, accumulated by PID/start identity; sampled lower bound, separate from worker RUSAGE_CHILDREN totals.',
            rss_scope='Simultaneous sampled RSS sum for exact offline worker PGIDs; shared pages may be counted more than once.',
            host_scope='Host load/memory/CPU context includes online work; it is not attributed to offline tests.')


COLLECTION_PLUGIN = '''import json, os
from pathlib import Path
from _pytest.junitxml import mangle_test_address

def pytest_collection_finish(session):
    rows = []
    for item in session.items:
        names = mangle_test_address(item.nodeid)
        rows.append({"case_identity": [str(item.location[0]), ".".join(names[:-1]), names[-1]], "nodeid": item.nodeid})
    with Path(os.environ["GEN54_COLLECTED_ITEMS"]).open("x") as f:
        json.dump(rows, f, ensure_ascii=False, indent=2)
'''


def stable(serial, plan):
    return (sha(__file__) == plan['runner_sha256'] and sha(SERIAL) == SERIAL_SHA
        and sha(PRIOR_TWO_WORKER)==PRIOR_TWO_WORKER_SHA and sha(CALIBRATION)==CALIBRATION_SHA
        and sha(HISTORY) == HISTORY_SHA and sha(plan['historical_xml']) == plan['historical_xml_sha256']
        and sha(MANIFEST_PATH)==MANIFEST_SHA==plan['candidate_manifest_sha256']
        and current_context(serial) == plan['context'])


def worker(plan_path,index):
    serial = load_serial(); plan = read(plan_path); plan_sha = sha(plan_path)
    assert index in range(plan['worker_count']) and stable(serial,plan)
    assert os.getsid(0)==os.getpgrp()==os.getpid(), 'worker must own its coordinator-created session'
    assert Path(plan['output']).resolve() == plan_path.parent.resolve()
    shard = plan['shards'][index]; context = plan['context']
    output = plan_path.parent/('worker_%d'%index)
    serial.storage(output); assert not output.exists(); output.mkdir()
    for name in ('tmp','ros','ros_home','bootstrap','pycache','cache'): (output/name).mkdir()
    guard,audit = bootstrap_sources(serial)
    (output/'bootstrap/sitecustomize.py').write_text(guard)
    (output/'bootstrap/gen42_import_audit.py').write_text(audit)
    if plan['mode']=='collect':(output/'bootstrap/gen54_collect_audit.py').write_text(COLLECTION_PLUGIN)
    generated = {str(p.relative_to(output)):sha(p) for p in (output/'bootstrap').iterdir()}
    env = dict(os.environ,PYTHONDONTWRITEBYTECODE='1',PYTHONNOUSERSITE='1',
        PYTHONPYCACHEPREFIX=str(output/'pycache'),XDG_CACHE_HOME=str(output/'cache'),
        PYTEST_DISABLE_PLUGIN_AUTOLOAD='1',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',
        MKL_NUM_THREADS='1',NUMEXPR_NUM_THREADS='1',
        LD_LIBRARY_PATH='/opt/ros/noetic/lib:/opt/ros/noetic/lib/x86_64-linux-gnu',
        ROS_DISTRO='noetic',ROS_VERSION='1',ROS_PYTHON_VERSION='3',
        ROS_MASTER_URI='http://127.0.0.1:1',ROS_LOG_DIR=str(output/'ros'),
        ROS_HOME=str(output/'ros_home'),TMPDIR=str(output/'tmp'),TMP=str(output/'tmp'),TEMP=str(output/'tmp'),
        OFFLINE_BOUNDARY_LOG=str(output/'boundary_attempts.jsonl'),
        GEN54_COLLECTED_ITEMS=str(output/'collected_items.json'),
        GEN42_MODULE_PROVENANCE=str(output/'module_provenance.json'))
    for key in ('PYTEST_ADDOPTS','PYTEST_PLUGINS'): env.pop(key,None)
    paths = [output/'bootstrap']+[W/p for p in (*serial.ROOTS,
        'src/alicia_grasp_modes/src','src/alicia_flexible_grasp_supervisor/src')]
    paths += [PROJECT/'devel/lib/python3/dist-packages',Path('/opt/ros/noetic/lib/python3/dist-packages')]
    env['PYTHONPATH'] = ':'.join(map(str,paths))
    env['ROS_PACKAGE_PATH'] = str(W/'src')+':/opt/ros/noetic/share'
    command = [sys.executable,'-m','pytest','-q','--rootdir='+str(W),*shard['scope'],
        '-p','no:cacheprovider','-p','gen42_import_audit','--tb=short',
        '--basetemp='+str(output/'pytest_tmp'),'--junitxml='+str(output/'pytest.xml'),
        '-o','junit_family=legacy']
    if plan['mode']=='collect':command+=['--collect-only','-p','gen54_collect_audit']
    result = dict(passed=False,worker_index=index,plan_sha256=plan_sha,command=command,
        mode=plan['mode'],test_bodies_executed=(plan['mode']!='collect'),
        scope_kind=plan['scope_kind'],full_regression_scope=plan['full_regression_scope'],
        required_changed_sources=shard['required_changed_sources'],
        test_scope=shard['scope'],expected_count=shard['expected_count'],interrupted=False)
    started=time.monotonic(); cpu_before=resource.getrusage(resource.RUSAGE_CHILDREN); returncode=None; process=None; cleanup=[]
    try:
        with (output/'pytest.log').open('x') as log:
            process=subprocess.Popen(command,cwd=W,env=env,stdout=log,stderr=subprocess.STDOUT)
            try: returncode=process.wait()
            except KeyboardInterrupt:
                result['interrupted']=True
                try: returncode=process.wait(timeout=20.)
                except subprocess.TimeoutExpired:
                    process.terminate();cleanup.append(dict(pid=process.pid,signal='SIGTERM'))
                    try: returncode=process.wait(timeout=5.)
                    except subprocess.TimeoutExpired:
                        process.kill();cleanup.append(dict(pid=process.pid,signal='SIGKILL'));returncode=process.wait()
        xml=output/'pytest.xml'; provenance_path=output/'module_provenance.json'
        summary,cases,_=inventory(xml)
        provenance=read(provenance_path); invalid={}; oracle_rows={}
        for label,row in provenance.items():
            p=Path(row['path'])
            if serial.inside(p,W/'src'):
                relative=str(p.relative_to(W)); valid=relative in context['sources'] and row['sha256']==context['sources'][relative]
            elif serial.inside(p,PROJECT/'devel'):
                valid=context['generated_imports'].get(str(p))==row['sha256'] and sha(p)==row['sha256']
            else:
                # This is exactly the serial full-run oracle exception, bound
                # to its original test in the UNION scope, path and bytes. A
                # helper import in the other process gains no wider whitelist.
                valid=(label==serial.ORACLE_LABEL and p==serial.ORACLE and row['sha256']==serial.ORACLE_SHA
                    and serial.ORACLE_TEST in plan['test_scope'] and context['sources'][serial.ORACLE_TEST]==serial.ORACLE_TEST_SHA
                    and sha(serial.ORACLE)==serial.ORACLE_SHA)
                if valid: oracle_rows[label]=row
            if not valid: invalid[label]=row
        loaded_changed={path:any(r['path']==str(W/path) and r['sha256']==context['sources'][path]
            for r in provenance.values()) for path in context['required_changed_sources']}
        source_loaded=any(loaded_changed.values())
        attempts_path=output/'boundary_attempts.jsonl'
        attempts=attempts_path.read_text().splitlines() if attempts_path.exists() else []
        expected=sorted(map(tuple,shard['case_identities']))
        actual=sorted(map(tuple,cases))
        counts=dict(Counter(c[0] for c in cases))
        expected_counts={} if plan['mode']=='collect' else {p:plan['expected_counts_by_file'][p] for p in shard['scope']}
        unchanged=(stable(serial,plan) and sha(plan_path)==plan_sha
            and all(sha(output/p)==h for p,h in generated.items()))
        complete=(actual==expected and counts==expected_counts and summary['tests']==shard['expected_count'])
        collected=[];collection_sha=None;collection_nodes=[]
        if plan['mode']=='collect':
            collected_rows=read(output/'collected_items.json');collection_sha=sha(output/'collected_items.json')
            collected=[r['case_identity'] for r in collected_rows];collection_nodes=[r['nodeid'] for r in collected_rows]
            assert len(collected)==len(set(map(tuple,collected)))==len(set(collection_nodes))
            complete=complete and set(c[0] for c in collected)==set(shard['scope']) and bool(collected)
        result.update(summary=summary,case_identities=cases,test_case_counts_by_file=counts,
            scope_complete=complete,source_unchanged=unchanged,changed_source_loaded=source_loaded,
            loaded_changed_sources=loaded_changed,
            foreign_or_inconsistent_imports=invalid,accepted_exact_historical_oracle=oracle_rows,
            external_boundary_attempts=attempts,generated_bootstrap_sha256=generated,
            module_provenance_sha256=sha(provenance_path),xml_sha256=sha(xml),log_sha256=sha(output/'pytest.log'))
        result.update(mode=plan['mode'],test_bodies_executed=(plan['mode']!='collect'),
            collected_case_identities=collected,collected_nodeids=collection_nodes,collection_sha256=collection_sha)
        result['passed']=(returncode==0 and not result['interrupted'] and complete and unchanged
            and all(loaded_changed[p] for p in shard['required_changed_sources']) and not invalid and not attempts
            and all(summary[k]==0 for k in ('failures','errors','skipped')))
    except BaseException as exc:
        if isinstance(exc,KeyboardInterrupt): result['interrupted']=True
        result.update(error=repr(exc),traceback=traceback.format_exc())
    finally:
        # Every exceptional path reaps the direct pytest child. Its descendants
        # inherit this worker's PGID; the coordinator always checks that entire
        # owned group, including when this worker exits before its descendants.
        if process is not None:
            if process.poll() is None:
                result['passed']=False
                process.terminate();cleanup.append(dict(pid=process.pid,signal='SIGTERM'))
                try: process.wait(timeout=5.)
                except subprocess.TimeoutExpired:
                    process.kill();cleanup.append(dict(pid=process.pid,signal='SIGKILL'));process.wait()
            returncode=process.wait()
        cpu_after=resource.getrusage(resource.RUSAGE_CHILDREN)
        result.update(returncode=returncode,elapsed_sec=time.monotonic()-started,
            worker_pgid=os.getpgrp(),direct_child_cleanup_signals=cleanup,
            direct_child_reaped=(process is not None and process.returncode is not None),
            pytest_cpu_user_sec=cpu_after.ru_utime-cpu_before.ru_utime,
            pytest_cpu_system_sec=cpu_after.ru_stime-cpu_before.ru_stime,
            reaped_child_max_rss_kib=cpu_after.ru_maxrss,
            reaped_child_max_rss_scope='RUSAGE_CHILDREN high-water maximum in this fresh worker; not simultaneous RSS sum',
            live_ROS_contacted=False,hardware_contacted=False,motion_commands_sent=False,temperature_queried=False)
        serial.storage(output);save(output/'result.json',result)
    return 0 if result['passed'] else 1


def execute(serial,output,plan):
    assert stable(serial,plan)
    if plan['mode'] == 'full':
        assert os.environ.get('GEN55_DURABLE_UNIT') and os.environ.get('INVOCATION_ID'), 'Full regression requires its recorded durable user unit.'
    coordinator = process_stat(os.getpid())
    save(output/'coordinator_identity.json',dict(pid=os.getpid(),identity=coordinator,
        durable_unit=os.environ.get('GEN55_DURABLE_UNIT'),systemd_invocation_id=os.environ.get('INVOCATION_ID'),
        plan_sha256=sha(output/'plan.json'),runner_sha256=sha(__file__),at_ns=time.time_ns()))
    assert plan['resource_budget']['execution_resources_sufficient'], 'Selected workers exceed the recorded CPU/memory budget; no test started.'
    budget=plan['resource_budget']
    launch_budget=resource_budget(argparse.Namespace(workers=str(plan['worker_count']),worker_cap=budget['worker_cap'],
        reserve_cpus=budget['reserved_cpu_count'],memory_per_worker_mib=budget['memory_per_worker_mib'],
        reserve_memory_mib=budget['reserved_memory_mib']))
    save(output/'launch_resource_budget.json',launch_budget)
    assert launch_budget['execution_resources_sufficient'], 'Resources changed before launch; no test started.'
    plan_path=output/'plan.json'; plan_sha=sha(plan_path)
    processes=[]; logs=[]; interrupted=False; signals=[]; started=time.monotonic()
    closed_groups=set();residual_groups=[];cleanup_required=False;launcher_error=None
    monitor=WorkloadMonitor(output);workload=None
    def close_owned_group(process):
        nonlocal cleanup_required
        if process.pid in closed_groups:return
        # Check our exact group even when poll() already reaped its leader.
        # Closing promptly reduces the lifetime of a completed group identity.
        if group_exists(process.pid):cleanup_required=True
        if cleanup_group(process.pid,signals,process):residual_groups.append(process.pid)
        closed_groups.add(process.pid)
    try:
        for index in range(plan['worker_count']):
            log=(output/('worker_%d_launcher.log'%index)).open('x'); logs.append(log)
            command=[sys.executable,'-B',str(Path(__file__).resolve()),'--worker-plan',str(plan_path),'--worker-index',str(index)]
            processes.append(subprocess.Popen(command,cwd=W,stdout=log,stderr=subprocess.STDOUT,start_new_session=True))
        save(output/'owned_workers.json',dict(plan_sha256=plan_sha,coordinator_pid=os.getpid(),
            workers=[dict(index=i,pid=p.pid,pgid=p.pid,identity=process_stat(p.pid)) for i,p in enumerate(processes)]))
        try:
            while True:
                monitor.sample(processes)
                running=False
                for process in processes:
                    if process.poll() is None:running=True
                    else:close_owned_group(process)
                if not running:break
                time.sleep(.2)
        except KeyboardInterrupt:
            interrupted=True
            for p in processes:
                signal_group(p.pid,signal.SIGINT,signals)
            deadline=time.monotonic()+30.
            while any(p.poll() is None for p in processes) and time.monotonic()<deadline:time.sleep(.1)
    except BaseException as exc:
        interrupted=interrupted or isinstance(exc,KeyboardInterrupt)
        launcher_error=dict(error=repr(exc),traceback=traceback.format_exc())
    finally:
        # Only these explicit offline children are eligible for cleanup. No
        # runtime ROS process group, user service or hardware PID is touched.
        for p in processes:
            close_owned_group(p)
            if p.poll() is None:
                try:p.wait(timeout=2.)
                except subprocess.TimeoutExpired:residual_groups.append(p.pid)
        for log in logs:log.close()
        workload=monitor.close()
    results=[]; missing=[]; all_cases=[]; all_collected=[]; all_nodes=[]; summary=dict.fromkeys(('tests','failures','errors','skipped'),0)
    combined=ET.Element('testsuites'); bindings={}
    for index in range(plan['worker_count']):
        folder=output/('worker_%d'%index);path=folder/'result.json'
        if not path.exists():missing.append(index);continue
        row=read(path);results.append(row);bindings[str(path)]=sha(path)
        assert row['plan_sha256']==plan_sha
        assert row['worker_index']==index and row['test_scope']==plan['shards'][index]['scope']
        if row.get('passed'):
            # Independently bind the child's evidence after BOTH children exit;
            # no summary silently trusts a green child JSON with changed XML,
            # provenance, bootstrap or appended boundary evidence.
            assert sha(folder/'module_provenance.json')==row['module_provenance_sha256']
            assert sha(folder/'pytest.log')==row['log_sha256']
            assert sha(folder/'pytest.xml')==row['xml_sha256']
            assert all(sha(folder/p)==h for p,h in row['generated_bootstrap_sha256'].items())
            boundary=folder/'boundary_attempts.jsonl'
            assert not boundary.exists() or not boundary.read_text().splitlines()
            assert row['source_unchanged'] and row['scope_complete']
            assert row['required_changed_sources']==plan['shards'][index]['required_changed_sources']
            assert all(row['loaded_changed_sources'][p] for p in row['required_changed_sources'])
            assert row['direct_child_reaped'] and not row['direct_child_cleanup_signals']
            assert row['scope_kind']==plan['scope_kind'] and row['full_regression_scope']==plan['full_regression_scope']
            assert not row['foreign_or_inconsistent_imports'] and not row['external_boundary_attempts']
            child_summary,child_cases,_=inventory(folder/'pytest.xml')
            assert child_summary==row['summary'] and child_cases==row['case_identities']
            if plan['mode']=='collect':
                assert sha(folder/'collected_items.json')==row['collection_sha256']
                collected_rows=read(folder/'collected_items.json')
                assert [r['case_identity'] for r in collected_rows]==row['collected_case_identities']
                assert [r['nodeid'] for r in collected_rows]==row['collected_nodeids']
        if 'summary' in row:
            for key in summary:summary[key]+=row['summary'][key]
        all_cases+=row.get('case_identities',[])
        all_collected+=row.get('collected_case_identities',[]);all_nodes+=row.get('collected_nodeids',[])
        xml=folder/'pytest.xml'
        if row.get('xml_sha256') and sha(xml)==row['xml_sha256']:
            parsed=ET.parse(xml).getroot()
            for suite in parsed.iter('testsuite'):combined.append(deepcopy(suite))
    duplicates=[list(k) for k,n in Counter(map(tuple,all_cases)).items() if n>1]
    expected=set(map(tuple,plan['expected_case_identities']));actual=set(map(tuple,all_cases))
    counts=dict(Counter(c[0] for c in all_cases))
    complete=(not missing and not duplicates and expected==actual and counts==plan['expected_counts_by_file']
        and len(all_cases)==summary['tests']==plan['expected_case_count'])
    if plan['mode']=='collect':
        complete=(complete and bool(all_collected) and len(all_collected)==len(set(map(tuple,all_collected)))==len(set(all_nodes))
            and set(c[0] for c in all_collected)==set(plan['test_scope']))
    unchanged=stable(serial,plan) and sha(plan_path)==plan_sha
    globally_loaded={path:any(r.get('loaded_changed_sources',{}).get(path,False) for r in results)
        for path in plan['required_changed_sources']}
    passed=(not interrupted and launcher_error is None and not cleanup_required and not residual_groups
        and len(processes)==len(results)==plan['worker_count'] and all(p.returncode==0 for p in processes)
        and all(r['passed'] for r in results) and complete and unchanged
        and all(globally_loaded.values())
        and all(summary[k]==0 for k in ('failures','errors','skipped')))
    ET.ElementTree(combined).write(output/'pytest.xml',encoding='utf-8',xml_declaration=True)
    combined_summary,combined_cases,_=inventory(output/'pytest.xml')
    combined_matches=(combined_summary==summary and sorted(map(tuple,combined_cases))==sorted(map(tuple,all_cases)))
    passed=passed and combined_matches
    result=dict(passed=passed,summary=summary,scope_complete=complete,source_unchanged=unchanged,
        kind='GEN55_parallel_'+plan['scope_kind']+'_offline_result',scope_kind=plan['scope_kind'],
        mode=plan['mode'],candidate_manifest_path=str(MANIFEST_PATH),candidate_manifest_sha256=MANIFEST_SHA,
        full_regression_scope=plan['full_regression_scope'],
        full_regression_passed=(passed and plan['full_regression_scope'] and plan['context']['compatibility']['variant']=='candidate'),
        test_bodies_executed=(plan['mode']!='collect'),
        collected_case_identities=all_collected,collected_nodeids=all_nodes,collected_case_count=len(all_collected),
        external_boundary_attempts=[x for r in results for x in r.get('external_boundary_attempts',[])],
        foreign_or_inconsistent_imports={str(i):r['foreign_or_inconsistent_imports'] for i,r in enumerate(results) if r.get('foreign_or_inconsistent_imports')},
        elapsed_sec=time.monotonic()-started,worker_count=plan['worker_count'],resource_budget=plan['resource_budget'],
        measured_workload=workload,
        pytest_cpu_user_sec=sum(r.get('pytest_cpu_user_sec',0.) for r in results),
        pytest_cpu_system_sec=sum(r.get('pytest_cpu_system_sec',0.) for r in results),
        pytest_cpu_scope='Sum of RUSAGE_CHILDREN deltas from disjoint fresh workers; observed pytest process trees only.',
        launch_resource_budget=launch_budget,launch_resource_budget_sha256=sha(output/'launch_resource_budget.json'),
        loaded_changed_sources_global=globally_loaded,test_scope_files=plan['expected_file_count'],source_count=len(plan['context']['sources']),
        test_case_counts_by_file=counts,duplicate_case_identities=duplicates,
        missing_case_identities=[list(c) for c in sorted(expected-actual)],
        unexpected_case_identities=[list(c) for c in sorted(actual-expected)],missing_worker_results=missing,
        worker_result_sha256=bindings,worker_returncodes=[p.returncode for p in processes],
        interrupted=interrupted,launcher_error=launcher_error,
        offline_child_cleanup_required=cleanup_required,offline_child_cleanup_signals=signals,
        residual_offline_process_groups=sorted(set(residual_groups)),plan_sha256=plan_sha,
        combined_xml_matches_worker_results=combined_matches,
        xml_sha256=sha(output/'pytest.xml'),runner_sha256=sha(__file__),
        source_sha256=plan['context']['sources'],source_compatibility=plan['context']['compatibility'],
        accepted_for_current_release=False,live_ROS_contacted=False,hardware_contacted=False,
        motion_commands_sent=False,temperature_queried=False,deployed=False,physical_grasp_verified=False)
    save(output/'result.json',result)
    print(json.dumps(dict(passed=passed,summary=summary,output=str(output))),flush=True)
    return 0 if passed else 1


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path)
    p.add_argument('--manifest',type=Path)
    p.add_argument('--manifest-sha256')
    p.add_argument('--mode',choices=('focus','full','collect'),default='focus')
    p.add_argument('--workers',default='auto',help='auto or an explicit positive count; plans record whether the resource budget permits execution.')
    p.add_argument('--worker-cap',type=int,default=8,help='Safety cap, at most8 for this helper.')
    p.add_argument('--reserve-cpus',type=int,default=4)
    p.add_argument('--memory-per-worker-mib',type=int,default=768)
    p.add_argument('--reserve-memory-mib',type=int,default=2048)
    p.add_argument('--execute',action='store_true',help='Explicitly execute the budgeted complete-file shards; default is plan only.')
    p.add_argument('--execute-plan',type=Path,help='Execute a previously written exact plan from its persistent coordinator.')
    p.add_argument('--plan-sha256')
    p.add_argument('--worker-plan',type=Path,help=argparse.SUPPRESS)
    p.add_argument('--worker-index',type=int,help=argparse.SUPPRESS)
    a=p.parse_args()
    def terminate_as_interrupt(signum, frame):
        raise KeyboardInterrupt('offline coordinator received '+signal.Signals(signum).name)
    signal.signal(signal.SIGTERM,terminate_as_interrupt)
    if a.execute_plan:
        assert a.output is None and a.manifest is None and not a.execute and a.worker_plan is None
        plan_path=a.execute_plan.resolve(strict=True)
        assert a.plan_sha256 and sha(plan_path)==a.plan_sha256
        plan=read(plan_path);bind_manifest(plan['candidate_manifest_path'],plan['candidate_manifest_sha256'])
        serial=load_serial();output=plan_path.parent;serial.storage(output)
        assert plan_path.name=='plan.json' and Path(plan['output']).resolve()==output
        assert not (output/'result.json').exists() and not (output/'coordinator_identity.json').exists()
        assert not list(output.glob('worker_*')) and stable(serial,plan)
        return execute(serial,output,plan)
    assert a.plan_sha256 is None
    if a.worker_plan:
        assert a.output is None and not a.execute and a.worker_index is not None
        plan=read(a.worker_plan)
        bind_manifest(plan['candidate_manifest_path'],plan['candidate_manifest_sha256'])
        return worker(a.worker_plan.resolve(strict=True),a.worker_index)
    assert a.worker_index is None and a.output is not None
    assert a.manifest is not None and a.manifest_sha256 is not None
    bind_manifest(a.manifest,a.manifest_sha256)
    serial=load_serial();output=a.output.resolve();serial.storage(output);assert not output.exists()
    assert a.workers=='auto' or (a.workers.isdecimal() and int(a.workers)>0)
    assert sha(PRIOR_TWO_WORKER)==PRIOR_TWO_WORKER_SHA
    plan=make_plan(serial,output,a.mode,resource_budget(a));output.mkdir();save(output/'plan.json',plan)
    if not a.execute:
        print(json.dumps(dict(plan_only=True,tests_executed=False,plan=str(output/'plan.json'),
            worker_count=plan['worker_count'],execution_resources_sufficient=plan['resource_budget']['execution_resources_sufficient'],
            plan_sha256=sha(output/'plan.json'),shards=[dict(index=s['index'],files=s['test_files'],cases=s['expected_count'],historical_case_time_sec=s['historical_case_time_sec']) for s in plan['shards']])),flush=True)
        return 0
    return execute(serial,output,plan)


if __name__=='__main__':
    raise SystemExit(main())
