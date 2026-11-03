#!/usr/bin/env python3
"""Offline GEN11 closed-bag audit. Explicit non-temperature topic allowlist."""
import base64
import gzip
import hashlib
import io
import json
import pathlib
import re

import rosbag

W = pathlib.Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
E = pathlib.Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
S = E / 'ros_log/full_latest_frozen_no_skin_20261012T021050Z'
R = E / 'grasp_records/20261012T140905.069509Z_022d4230fcb4456d8808a52b6c1db9ca'
O = S / 'gen11_analysis/contact_promotion_evidence'
TOPICS = ['/grasp/near_field_phase', '/grasp/state', '/grasp_6d/gate_audit',
    '/grasp_6d/object_geometry', '/grasp_6d/plan_enriched',
    '/grasp_6d/preview_plan_enriched', '/grasp_6d/status',
    '/grasp_mode/observe_return_session', '/grasp_mode/observe_return_measurement_request',
    '/grasp_mode/observe_return_measurement_ack', '/perception/object',
    '/perception/object_mask', '/perception/unknown/detector_status',
    '/gui/joint_direct_mode']


def sha(path):
    h = hashlib.sha256()
    with path.open('rb') as f:
        for data in iter(lambda: f.read(8 * 1024 * 1024), b''):
            h.update(data)
    return h.hexdigest()


def main():
    manifest = json.loads((R / 'manifest.json').read_text())
    assert manifest['state'] == 'complete' and manifest['recorder_closed'] is True
    bag = R / 'keyframes.bag'
    expected = next(x['sha256'] for x in manifest['files'] if x['path'] == bag.name)
    assert sha(bag) == expected
    O.mkdir(parents=True, exist_ok=True)
    events = []
    counts = {}
    with rosbag.Bag(str(bag)) as b, gzip.open(str(O / 'wire.jsonl.gz'), 'wt', encoding='utf-8') as wire:
        for topic, m, t in b.read_messages(topics=TOPICS):
            counts[topic] = counts.get(topic, 0) + 1
            ns = t.to_nsec()
            # Complete wire messages remain available for genuine callback replay.
            buf = io.BytesIO(); m.serialize(buf)
            raw = buf.getvalue()
            row = dict(topic=topic, bag_ns=ns, msg_type=m._type,
                md5sum=m._md5sum, serialized_sha256=hashlib.sha256(raw).hexdigest(),
                serialized_base64=base64.b64encode(raw).decode('ascii'))
            wire.write(json.dumps(row, separators=(',', ':')) + '\n')
            if not 1791814232500000000 <= ns <= 1791814324500000000:
                continue
            d = dict(topic=topic, bag_ns=ns)
            if topic == '/perception/unknown/detector_status':
                v = json.loads(m.data); c = v.get('station_compute_wait') or {}
                d.update({k:v.get(k) for k in ('state','detail','source_stamp_ns','target_lost','first_loss')})
                d['station_contract'] = c
                d['recovery_chain_sources'] = [x.get('source_ns') for x in v.get('station_compute_recovery_chain', [])]
            elif topic == '/perception/object_mask':
                d.update(source_ns=m.header.stamp.to_nsec(), height=m.height, width=m.width,
                    frame_id=m.header.frame_id, encoding=m.encoding, data_sha256=hashlib.sha256(bytes(m.data)).hexdigest())
            elif topic == '/perception/object':
                d.update(source_ns=m.header.stamp.to_nsec(), detected=m.detected)
            elif topic in ('/grasp_6d/plan_enriched','/grasp_6d/preview_plan_enriched'):
                d.update(source_ns=m.header.stamp.to_nsec(), plan_id=m.plan_id, valid=m.valid,
                    diagnostic=m.diagnostic, candidate_source=m.candidate_source)
            elif topic == '/grasp_6d/object_geometry':
                d.update(source_ns=m.header.stamp.to_nsec(), valid=m.valid, failure_reason=m.failure_reason)
            elif topic == '/grasp_6d/gate_audit':
                v=json.loads(m.data); d.update(report_sha256=v.get('report_sha256'),
                    report_path=v.get('report_path'), promotion=v.get('summary',{}).get('promotion'),
                    status=v.get('summary',{}).get('status'))
            elif hasattr(m, 'data'):
                try: d['value']=json.loads(m.data)
                except (TypeError, ValueError): d['value']=m.data
            else:
                d['message']=str(m)
            events.append(d)
    (O / 'events.json').write_text(json.dumps(events, ensure_ascii=False, indent=2)+'\n')
    log = S / 'gen10_fix_compute_deployment/consumers.log'
    height=[]; metrics=[]
    for line in log.read_text(errors='replace').splitlines():
        match = re.match(r'\[\w+\] \[([0-9.]+)\]', line)
        if not match or not 1791814233 <= float(match[1]) <= 1791814324: continue
        for marker, target in [('remote 6D contact height retry: ',height),('remote 6D pipeline metrics: ',metrics)]:
            if marker in line:
                target.append(dict(log_ros_sec=float(match[1]),data=json.loads(line.split(marker,1)[1])))
    (O / 'height_receipts.json').write_text(json.dumps(height,ensure_ascii=False,indent=2)+'\n')
    (O / 'metrics.json').write_text(json.dumps(metrics,ensure_ascii=False,indent=2)+'\n')
    result=dict(scope='sealed GEN11 read-only allowlisted offline extraction; no ROS service, action, publish or temperature data',
        record=str(R), bag_sha256=expected, manifest_sha256=sha(R/'manifest.json'),
        topic_counts=counts, artifacts={p.name:sha(p) for p in O.iterdir() if p.is_file() and p.name!='receipt.json'},
        script_sha256=sha(pathlib.Path(__file__)))
    (O/'receipt.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(dict(output=str(O),topic_counts=counts,receipt_sha256=sha(O/'receipt.json')),ensure_ascii=False))


if __name__ == '__main__':
    main()
