#!/usr/bin/env python3
"""Offline extraction of the gen11 cancelled request; no ROS/network/control."""
import argparse
from collections import Counter
import hashlib
import json
import os
from pathlib import Path
import re

STORE = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
ATTEMPT = STORE / 'ros_log/feedforward_speed_grasp_20260929T075717Z'
FORMAL = STORE / 'grasp_records/20260929T075718.486796Z_922422a152f34960b1c988a01016dc93'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-log', type=Path, default=STORE/'ros_log/unknown_grasp_20260923/supervision_continuous.log')
    parser.add_argument('--output', type=Path, default=ATTEMPT/'near_field_wait_analysis')
    args = parser.parse_args()
    if not os.path.ismount('/media/zhuyupei/SanStand'):
        raise RuntimeError('SanStand not mounted')
    args.output.resolve().relative_to(STORE.resolve())
    raw, failures, samples, metrics = [], [], {}, []
    with args.source_log.open(errors='replace') as stream:
        for line in stream:
            try: stamp = float(line.split(' ', 1)[0])
            except ValueError: continue
            if not 1790668656 <= stamp <= 1790668680:
                continue
            if not any(tag in line for tag in ('/remote_grasp6d_node ', '/supervisor_camera ',
                    '/grasp_task_node ', '/unknown_tabletop_perception ', 'GRASP_STATE ')):
                continue
            raw.append(line)
            if 'pipeline metrics: ' in line:
                item = json.loads(line.split('pipeline metrics: ', 1)[1])
                metrics.append(dict(log_receipt_sec=stamp, **item))
            if stamp < 1790668660:
                continue
            match = re.search(r'stable variant recheck failed: track=(\d+) variant=(\d+).*maximum symmetric contact/support margin ([.0-9]+) mm', line)
            if match:
                failures.append(dict(log_receipt_sec=stamp, track=int(match[1]),
                    variant=int(match[2]), margin_mm=float(match[3])))
            for kind in ('GraspNet', 'tabletop'):
                marker = '%s candidate rejection samples: ' % kind
                if marker in line:
                    rows = json.loads(line.split(marker, 1)[1])
                    samples[kind] = dict(sample_count=len(rows),
                        sample_counts=dict(Counter(r['code'] for r in rows)), rows=rows)
    request = next(m for m in metrics if m['request_id'] == 32)
    audits = []
    for path in sorted((FORMAL/'keyframes_audits').glob('*.json')):
        data = path.read_bytes(); value = json.loads(data)
        audits.append(dict(path=str(path), sha256=hashlib.sha256(data).hexdigest(),
            request_id=value.get('request_id'), source_ns=value.get('snapshot_stamp_ns'),
            selection=value.get('grasp_mode_selection'), outcome=value.get('outcome')))
    raw_bytes = ''.join(raw).encode()
    report = dict(schema=1, actual_motion_started=False,
        conclusion='Near-field request32 was submitted; target loss cancelled it before a fresh contact preview. No handoff defect is shown.',
        request32=request, recheck_failure_count=len(failures),
        recheck_failed_track_count=len({r['track'] for r in failures}),
        normal_translation_evidence_limit_mm=2.676768,
        recheck_failures=failures, bounded_rejection_samples=samples,
        final_request32_audit_present=any(a['request_id']==32 for a in audits), audits=audits,
        limits=['Logged rejection samples are bounded samples, not the complete candidate population.',
                'No complete request32 strict-contact audit survived its lifecycle cancellation; unchecked count and track24 result are unknown.',
                'Normal translation evidence is a contact/support-height condition, not an aperture-width margin.',
                'No global ungraspability or successful contact plan follows from these records.'],
        raw_excerpt_sha256=hashlib.sha256(raw_bytes).hexdigest(),
        source_script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    args.output.mkdir(parents=True, exist_ok=False)
    (args.output/'source_excerpt.log').write_bytes(raw_bytes)
    (args.output/'report.json').write_text(json.dumps(report, indent=2, ensure_ascii=False)+'\n')
    print(json.dumps(dict(output=str(args.output), recheck_failures=len(failures),
        request32_status=request['status'], final_request32_audit_present=report['final_request32_audit_present'])))


if __name__ == '__main__':
    main()
