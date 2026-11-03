#!/usr/bin/env python3
"""Offline metadata-only audit; no ROS imports, images, or device access."""
import argparse
import collections
import hashlib
import json
from pathlib import Path
import zipfile


def sha(b):
    return hashlib.sha256(b).hexdigest()


def stats(a):
    if not a:
        return {'count': 0, 'missing': True}
    a = sorted(a)
    def q(p):
        i = (len(a)-1)*p; j = int(i)
        return a[j] + (a[min(j+1, len(a)-1)]-a[j])*(i-j)
    return {'count': len(a), 'min': a[0], 'p50': q(.5), 'p95': q(.95), 'max': a[-1]}


def run(capture, output):
    verified = {}
    with zipfile.ZipFile(capture/'diagnostic.zip') as z:
        manifest = {e['name']: e for e in json.loads(z.read('entry_manifest.json'))}
        def read(name):
            b = z.read(name); expected = manifest[name]
            assert len(b) == expected['bytes'] and sha(b) == expected['sha256'], name
            verified[name] = expected
            return json.loads(b)
        metadata = [read(n) for n in z.namelist() if n.startswith('metadata/') and n.endswith('.json')]
        wrappers = [read(n) for n in sorted(z.namelist()) if n.startswith('status/') and n.endswith('.json')]
        arrivals = read('raw_image_header_arrivals.json')
    sdk = {}
    camera_sizes = []
    for m in metadata:
        camera = m['parameters']['/camera']
        camera_sizes.append(len(json.dumps(camera).encode()))
        p = json.loads(m['parameters']['/camera_read_diagnostics']['payload_json'])
        for e in p['entries']:
            ns = int(e['source_stamp_ns'])
            if ns in sdk:
                assert sdk[ns] == e, 'same-source SDK metadata changed'
            sdk[ns] = e
    raw = collections.defaultdict(list)
    for a in arrivals:
        raw[(int(a['source_stamp_ns']), a['stream'])].append(a)
    rows = []
    status_counts = collections.Counter()
    stale_rows = []
    for w in wrappers:
        s = json.loads(w['original_status_data'])
        if s.get('generation') != 25:
            continue
        status_counts[s['detail']] += 1
        t = s.get('frame_timing', {})
        ns = s.get('source_stamp_ns')
        if ns is None:
            continue
        e = sdk.get(int(ns))
        row = {'source_stamp_ns': ns, 'detail': s['detail'], 'frame_timing': t,
               'sdk_exact_match': e is not None, 'streams': {}}
        if e:
            ev = e['evidence']
            row.update(sdk_read_completed_monotonic=e['read_completed_monotonic'],
                       sdk_host_wait=ev.get('host_wait'), sdk_stages_sec=ev.get('stages_sec'),
                       sdk_raw_frames=ev.get('raw_frames'), sdk_total_sec=ev.get('total_sec'),
                       sdk_source_clock=e.get('image_header_clock'),
                       sdk_exposure_clock_calibrated=e.get('exposure_clock_calibrated'))
        for stream in ('color', 'depth'):
            u = t.get('raw_arrivals', {}).get(stream)
            rr = raw.get((int(ns), stream), [])
            item = {'unknown_arrival': u, 'recorder_arrivals': rr,
                    'same_source_stream_recorder_count': len(rr)}
            if u and e:
                item['unknown_after_sdk_complete_sec'] = u['arrival_monotonic_sec']-e['read_completed_monotonic']
                item['unknown_ros_monotonic_age_disagreement_sec'] = u['source_age_at_arrival_sec']-item['unknown_after_sdk_complete_sec']
            if len(rr) == 1 and e:
                item['recorder_after_sdk_complete_sec'] = rr[0]['arrival_monotonic']-e['read_completed_monotonic']
                item['recorder_ros_monotonic_age_disagreement_sec'] = (rr[0]['receipt_ros_ns']-ns)*1e-9-item['recorder_after_sdk_complete_sec']
                if u:
                    item['unknown_minus_recorder_arrival_sec'] = u['arrival_monotonic_sec']-rr[0]['arrival_monotonic']
                    item['original_sequence_equal'] = rr[0]['source_seq'] == u['source_seq']
            row['streams'][stream] = item
        rows.append(row)
        if 'source_frame_stale_or_future' in s['detail']:
            stale_rows.append(row)
    streams_summary = {}
    for stream in ('color', 'depth'):
        streams_summary[stream] = {key: stats([r['streams'][stream][key] for r in rows if key in r['streams'][stream]]) for key in (
            'unknown_after_sdk_complete_sec', 'recorder_after_sdk_complete_sec',
            'unknown_minus_recorder_arrival_sec', 'unknown_ros_monotonic_age_disagreement_sec',
            'recorder_ros_monotonic_age_disagreement_sec')}
    source_paths = [
        Path('src/alicia_grasp_modes/scripts/unknown_tabletop_perception.py'),
        Path('src/alicia_flexible_grasp_supervisor/scripts/camera_node.py'),
        Path('/opt/ros/noetic/lib/python3/dist-packages/rospy/impl/tcpros_base.py'),
        Path('/opt/ros/noetic/lib/python3/dist-packages/rospy/msg.py')]
    report = {'schema': 1, 'capture': str(capture), 'scope': 'sealed gen25 metadata only; no pixels read',
        'archive_entry_hashes_verified': verified, 'sdk_visible_unique_count': len(sdk),
        'sdk_history_is_not_same_as_selection_window': True,
        'status_counts': dict(status_counts), 'source_rows': len(rows),
        'camera_parameter_serialized_json_bytes': stats(camera_sizes),
        'streams_summary': streams_summary, 'critical_stale_rows': stale_rows,
        'all_source_rows': rows, 'static_source_sha256': {str(p.resolve()): sha(p.read_bytes()) for p in source_paths},
        'interpretation': {
            'proved': [
                'For first stale source, the independently recorded same-sequence RGB and depth arrived about 17/19 ms after SDK completion while unknown callbacks arrived about 2.245/2.121 s later.',
                'This localizes the large extra delay to the unknown connection/receive/deserialize/callback scheduling path; a common publisher or source-clock stall does not explain the same-source difference.',
                'Recorded exact synchronization wait and input_lock wait are microseconds for these stale sources; input residence is not the dominant 2 s delay.',
                'No threshold or source timestamp was altered. Frame-age rejection remains correct.'
            ],
            'not_proved': [
                'No per-receive socket/deserialize trace, unknown thread scheduler samples or stack was captured. GIL, scheduling, page faults, TCP flow control and callback transport cannot be distinguished.',
                'No specific corrective production code defect has been established by these records.',
                'The raw recorder also has its own receipt delays; archival status receipt must not be confused with unknown internal callback time.',
                'The current on-disk source hashes are audit context, not by themselves proof of the loaded process implementation.'
            ],
            'code_audit': [
                'unknown subscriptions already use queue_size=2 and buff_size=2**24; raw arrival runs before TimeSynchronizer.add.',
                'frame callback only stores newest exact pair under input_lock; timer owns expensive processing.',
                'unknown does not read /camera_read_diagnostics; the large root diagnostics payload is not nested under /camera.',
                'rospy deserialize queue_size prunes only complete messages already present in its receive buffer, not an end-to-end source-age deadline.'
            ],
            'minimal_next_evidence_if_authorized': 'Bounded numeric receive_once entry / recv return / deserialize completion stamps plus fixed unknown RGB/depth and Timer TID stat/schedstat/wchan/page-fault evidence, aligned by monotonic time to existing SDK and arrival records; no extra image recording.',
            'production_change_recommended_now': False,
            'quality_or_freshness_gate_change_recommended': False
        }}
    output.mkdir(parents=True, exist_ok=False)
    b = (json.dumps(report, ensure_ascii=False, sort_keys=True, indent=2)+'\n').encode()
    (output/'report.json').write_bytes(b)
    script_sha = sha(Path(__file__).read_bytes())
    (output/'SHA256SUMS').write_text(sha(b)+'  report.json\n'+script_sha+'  '+str(Path(__file__).resolve())+'\n')
    print(json.dumps({'report': str(output/'report.json'), 'sha256': sha(b), 'status_counts': dict(status_counts), 'source_rows': len(rows), 'streams_summary': streams_summary}, ensure_ascii=False))


if __name__ == '__main__':
    ap = argparse.ArgumentParser(); ap.add_argument('--capture', type=Path, required=True); ap.add_argument('--output', type=Path, required=True)
    a = ap.parse_args(); run(a.capture.resolve(), a.output.resolve())
