"""Seal completed GEN19 evidence without contacting ROS or altering prior releases."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import time
import xml.etree.ElementTree as ET

W = Path(__file__).resolve().parents[1]
M = Path('/media/zhuyupei/SanStand')
E = M / '实验记录/Robot-Controller'
N = E / 'ros_log/full_latest_gen18_restart_20261014T012216Z'
V = N / 'verification'

def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def read(path):
    return json.loads(Path(path).read_text())

def write_new(path, value):
    assert M.is_mount() and not os.statvfs(M).f_flag & os.ST_RDONLY
    assert path.parent.stat().st_dev == M.stat().st_dev
    with path.open('x') as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2)
        stream.write('\n'); stream.flush(); os.fsync(stream.fileno())

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--review-sha256', required=True)
    args = parser.parse_args()
    assert M.is_mount() and N.is_dir()
    assert read(E/'maintenance/storage_control.json')['archive_policy'] == 'local_only'
    config = read(E/'grasp_records/archive_config.json')
    assert config['upload_enabled'] is False and config['delete_enabled'] is False
    evidence = {}
    def bind(path, expected=None):
        path = Path(path).resolve(strict=True)
        digest = sha(path)
        assert expected is None or digest == expected, str(path)
        evidence[str(path)] = digest
        return path
    combined_path = bind(V/'gen19_combined_r2/result.json')
    combined = read(combined_path)
    assert combined['passed'] and combined['source_unchanged'] and combined['returncode'] == 0
    assert combined['production_worktree_tested'] and not combined['live_ros_contacted']
    summary = combined['summary']
    assert summary['tests'] >= 5871 and all(summary[k] == 0 for k in ('failures','errors','skipped'))
    for name, key in [('pytest.log','log_sha256'),('pytest.xml','xml_sha256')]:
        bind(combined_path.parent/name, combined[key])
    suites = ET.parse(combined_path.parent/'pytest.xml').getroot()
    assert {k:sum(int(s.get(k,0)) for s in suites.iter('testsuite')) for k in summary} == summary
    assert len(combined['source_sha256']) == combined['source_count'] == 927
    assert len(combined['test_scope']) == combined['test_scope_files'] == 187
    for name, digest in combined['source_sha256'].items():
        assert not Path(name).is_absolute() and '..' not in Path(name).parts
        assert sha(W/name) == digest, name
    for key in ('baseline','patch'):
        bind(combined[key], combined[key+'_sha256'])
    bind(W/'.runtime/verify_gen19_combined_20261014.py', combined['runner_sha256'])
    review_path = bind(W/'.runtime/gen19_independent_source_review_r2_20261014.json',
        args.review_sha256)
    review = read(review_path)
    assert review['reviewed'] and not review['blocking_findings']
    for name, digest in review['reviewed_source_sha256'].items():
        assert combined['source_sha256'][name] == digest and sha(W/name) == digest
    for name, digest in review['evidence_sha256'].items():
        bind(name, digest)
    related = read(review['validation']['related_regression']['result'])
    assert related['passed'] and related['source_unchanged'] and related['returncode'] == 0
    assert related['summary']['tests'] >= 565
    assert all(related['summary'][k] == 0 for k in ('failures','errors','skipped'))
    for name, digest in related['source_sha256'].items():
        assert sha(W/name) == digest, name
    reload_review = review['reload_review']
    assert not reload_review['blockers']
    bind(W/reload_review['path'], reload_review['sha256'])
    for name, digest in reload_review['required_release_evidence_sha256'].items():
        bind(name, digest)
    bind(__file__)
    bind(N/'cold_restart_acceptance/result.json',
        '6c84d3b810e851ae0d22e314a1975232d7e95351670f0b249fc74ab583bc4f41')
    external_review = V/'gen19_independent_source_review.json'
    write_new(external_review, review); bind(external_review)
    release = dict(schema_version=1, kind='gen19_feedback_and_measurement_ack_clock_order',
        sealed_ns=time.time_ns(), passed=True, reviewed=True, installed=True,
        installed_semantics='Operational worktree source tested; three-consumer runtime reload pending.',
        source_sha256=combined['source_sha256'], source_count=combined['source_count'],
        test_scope=combined['test_scope'], test_scope_files=combined['test_scope_files'],
        test_summary=summary, actual_regression=str(combined_path),
        focused_regression=review['validation']['related_regression']['result'],
        candidate_delivery=combined['patch'], combined_review=str(external_review),
        evidence_sha256=evidence, runtime_deployment_pending=True, physical_grasp_verified=False,
        motion_commands_sent=False, temperature_queried=False,
        current_user_scope='Continue unfinished repair and recorded grasp until success under existing continuous authorization; verify current idle/site before starting.')
    path = V/'gen19_release_manifest.json'
    write_new(path, release)
    print(json.dumps(dict(release=str(path),sha256=sha(path),summary=summary,
        source_count=release['source_count'],evidence_files=len(evidence)),ensure_ascii=False))

if __name__ == '__main__':
    main()
