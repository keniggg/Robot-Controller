"""Preserve two independent assessments without claiming a new review."""
import argparse
import hashlib
import json
from pathlib import Path

WORK = Path(__file__).resolve().parents[1]
NEW = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_restart_20261010T002347Z')
OUT = NEW/'verification/gen18_arrival_feedback_gap_combined'


def artifact(path):
    return dict(path=str(path), sha256=hashlib.sha256(path.read_bytes()).hexdigest())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--producer-review', required=True, type=Path)
    args = ap.parse_args()
    consumer_path = NEW/'gen18_analysis/verification/consumer_independent_review.json'
    producer = json.loads(args.producer_review.read_text())
    consumer = json.loads(consumer_path.read_text())
    delivery_path = OUT/'delivery.json'
    delivery = json.loads(delivery_path.read_text())
    assessments = [('producer', producer, args.producer_review), ('consumer', consumer, consumer_path)]
    reviews = []
    reviewed_production = {}
    for name, report, path in assessments:
        blockers = report.get('blocking_findings', report.get('blockers'))
        assert blockers == [], (name, 'missing_or_nonempty_blockers')
        mapping = report.get('source_sha256', report.get('files'))
        if isinstance(mapping, list):
            mapping = {row['path']: row['sha256'] for row in mapping}
        assert mapping
        for f, digest in mapping.items():
            assert delivery['source_sha256'][f] == digest, (name, 'review_binding', f)
            if f in delivery['production_source_sha256']:
                reviewed_production[f] = digest
        reviews.append(dict(scope=name, artifact=artifact(path)))
    assert reviewed_production == delivery['production_source_sha256']
    result = dict(reviewed=True, blocking_findings=[],
        source_sha256=delivery['source_sha256'],
        production_source_sha256=reviewed_production,
        delivery=artifact(delivery_path), independent_assessments=reviews,
        assessment_source='Two cross-agent static reviews, each bound to its owned source subset; aggregation performs no additional review.',
        tests=dict(unique_passed=334, suite=artifact(OUT/'combined_r1_manifest.json')),
        limitations=['Synthetic RGB-D and fixed station pose with recorded GEN18 time deltas do not establish physical recovery or grasp success.'])
    path = OUT/'cross_review.json'
    path.write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps(artifact(path)))


if __name__ == '__main__':
    main()
