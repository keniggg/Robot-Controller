"""Assemble already-frozen producer/consumer source with the final suite."""
import argparse
import hashlib
import json
from pathlib import Path
import runpy

WORK = Path(__file__).resolve().parents[1]
OUT = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_restart_20261010T002347Z/verification/gen18_arrival_feedback_gap_combined')
DRIVER = runpy.run_path(str(WORK/'.runtime/run_gen18_arrival_feedback_combined_20261010.py'), run_name='metadata_only')
FILES = DRIVER['PRODUCTION'] + [DRIVER['FIXTURE']] + [
    'src/alicia_grasp_modes/tests/test_arrival_feedback_bracket_gap.py',
    'src/alicia_grasp_modes/tests/test_arrival_feedback_gap_late_consumers.py',
    'src/alicia_grasp_modes/tests/test_arrival_feedback_gap_integration.py',
]


def sha(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--stem', required=True)
    ap.add_argument('--consumer-delivery', type=Path, required=True)
    args = ap.parse_args()
    producer_path = WORK/'.runtime/gen18_arrival_feedback_gap_20261010/producer_delivery.json'
    producer = json.loads(producer_path.read_text())
    consumer = json.loads(args.consumer_delivery.read_text())
    suite_path = OUT/(args.stem+'_manifest.json')
    suite = json.loads(suite_path.read_text())
    assert suite['source_unchanged'] and suite['exit_code'] == 0
    sources = {f: sha(WORK/f) for f in FILES}
    assert producer.get('ready') or producer.get('delivery_ready')
    for f, digest in producer['files'].items():
        assert sources[f] == digest
    consumer_sources = consumer.get('source_sha256', consumer.get('files'))
    if isinstance(consumer_sources, list):
        consumer_sources = {row['path']: row['sha256']
                            for row in consumer_sources + consumer.get('test_files', [])}
    assert consumer_sources
    for f, digest in consumer_sources.items():
        assert sources[f] == digest
    for f, digest in sources.items():
        assert suite['source_sha256'][f] == digest
    result = dict(delivery_ready=True, all_delivery_files_frozen=True, source_sha256=sources,
        production_source_sha256={f: sources[f] for f in DRIVER['PRODUCTION']},
        producer_delivery=dict(path=str(producer_path), sha256=sha(producer_path)),
        consumer_delivery=dict(path=str(args.consumer_delivery), sha256=sha(args.consumer_delivery)),
        final_suite=dict(path=str(suite_path), sha256=sha(suite_path)),
        deployment_performed=False,
        limitations=['Recorded feedback timing with synthetic RGB-D and fixed pose; no completed physical grasp claimed.',
                    'Only the previously registered fixed arrival station can retain identity during an oversized two-sided feedback bracket.'])
    path = OUT/'delivery.json'
    path.write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps({'path': str(path), 'sha256': sha(path), 'files': len(sources)}))


if __name__ == '__main__':
    main()
