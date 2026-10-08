#!/usr/bin/env python3
"""Metadata-only first-loss audit of one sealed exact-RGBD archive; no ROS imports."""
import argparse
import collections
import hashlib
import json
from pathlib import Path
import zipfile


def sha(data):
    return hashlib.sha256(data).hexdigest()


def stats(values):
    a = sorted(values)
    if not a:
        return {'count': 0, 'missing': True}
    def q(p):
        i = (len(a)-1)*p; j = int(i)
        return a[j]+(a[min(j+1,len(a)-1)]-a[j])*(i-j)
    return {'count': len(a), 'min': a[0], 'p50': q(.5), 'p95': q(.95), 'max': a[-1]}


def main(case, output):
    capture = case/'selection/raw_support'
    summary_bytes = (capture/'summary.json').read_bytes()
    summary = json.loads(summary_bytes)
    assert summary['archive_closed'] and summary['complete_without_writer_error'] and not summary['error']
    selection_bytes = (case/'selection/new_target_selection.json').read_bytes()
    selection = json.loads(selection_bytes)['new_selection']
    generation = selection['generation']; selection_ns = int(selection['stamp_ns'])
    verified = {}
    with zipfile.ZipFile(capture/'diagnostic.zip') as z:
        manifest_bytes = z.read('entry_manifest.json')
        entries = json.loads(manifest_bytes)
        manifest = {e['name']: e for e in entries}
        assert len(manifest) == len(entries)
        def read(name):
            b = z.read(name); m = manifest[name]
            assert len(b)==m['bytes'] and sha(b)==m['sha256'], name
            verified[name] = m
            return json.loads(b)
        metadata = [read(n) for n in z.namelist() if n.startswith('metadata/') and n.endswith('.json')]
        wrappers = [read(n) for n in sorted(z.namelist()) if n.startswith('status/') and n.endswith('.json')]
        arrivals = read('raw_image_header_arrivals.json')
    sdk = {}; metadata_errors = []
    for m in metadata:
        if m.get('errors'):
            metadata_errors.append({'sequence':m.get('sequence'),'errors':m['errors']})
        env = m['parameters'].get('/camera_read_diagnostics')
        if not env:
            metadata_errors.append({'sequence':m.get('sequence'),'error':'missing_root_diagnostics'}); continue
        p = json.loads(env['payload_json'])
        for e in p['entries']:
            ns = int(e['source_stamp_ns'])
            if ns in sdk:
                assert sdk[ns] == e, 'different diagnostics for same source'
            sdk[ns] = e
    statuses = []
    for w in wrappers:
        b = w['original_status_data'].encode()
        assert sha(b)==w['original_status_utf8_sha256']
        s = json.loads(b)
        if s.get('generation') == generation and int(s.get('selection_stamp_ns',0)) == selection_ns:
            statuses.append((w,s))
    lost_pair = next((w,s) for w,s in statuses if s.get('first_loss') and s.get('target_lost'))
    loss = lost_pair[1]['first_loss']
    assert loss['generation']==generation and int(loss['selection_stamp_ns'])==selection_ns
    last_good = int(loss['last_good_source_stamp_ns']); lost_ns = int(loss['source_stamp_ns']); observed = int(loss['observed_stamp_ns'])
    acquired = next(s for w,s in statuses if s['detail']=='acquired')
    acquired_ns = int(acquired['source_stamp_ns'])
    before = [(w,s) for w,s in statuses if w['status_index'] <= lost_pair[0]['status_index']]
    raw = collections.defaultdict(list)
    for a in arrivals:
        raw[(int(a['source_stamp_ns']),a['stream'])].append(a)
    rows = []
    for w,s in before:
        ns = s.get('source_stamp_ns'); t=s.get('frame_timing',{})
        row = {'status_index':w['status_index'],'detail':s['detail'],'state':s.get('state'),
               'source_stamp_ns':ns,'frame_timing':t,'source_qualification':s.get('source_qualification'),
               'geometry_metrics':s.get('geometry_metrics'),'status_receipt_ros_ns':w.get('receipt_ros_ns'),
               'status_receipt_monotonic':w.get('receipt_monotonic'),'streams':{}}
        e=sdk.get(int(ns)) if ns else None
        row['sdk_exact_source_match'] = e is not None
        row['exact_pair_saved'] = ns in summary['saved_pair_source_stamps_ns'] if ns else False
        if e:
            row['sdk_evidence']=e
        for stream in ('color','depth'):
            u=t.get('raw_arrivals',{}).get(stream)
            arr=raw.get((int(ns),stream),[]) if ns else []
            item={'unknown_raw_arrival':u,'independent_recorder_arrivals':arr}
            if u and e:
                item['unknown_after_sdk_complete_sec']=u['arrival_monotonic_sec']-e['read_completed_monotonic']
                item['unknown_ros_monotonic_age_disagreement_sec']=u['source_age_at_arrival_sec']-item['unknown_after_sdk_complete_sec']
            if len(arr)==1 and e:
                item['recorder_after_sdk_complete_sec']=arr[0]['arrival_monotonic']-e['read_completed_monotonic']
                if u:
                    item['unknown_minus_recorder_arrival_sec']=u['arrival_monotonic_sec']-arr[0]['arrival_monotonic']
                    item['same_sequence']=u['source_seq']==arr[0]['source_seq']
            row['streams'][stream]=item
        rows.append(row)
    ordered=sorted(sdk.values(),key=lambda e:e['read_completed_monotonic'])
    gaps=[]
    for prev,cur in zip(ordered,ordered[1:]):
        pns=int(prev['source_stamp_ns']); ns=int(cur['source_stamp_ns'])
        if not(acquired_ns <= pns < ns <= lost_ns): continue
        ev=cur['evidence']; hw=ev.get('host_wait',{}); enter=hw.get('enter',{}).get('monotonic_sec')
        g={'previous_source_stamp_ns':pns,'source_stamp_ns':ns,
           'read_completed_gap_sec':cur['read_completed_monotonic']-prev['read_completed_monotonic'],
           'source_stamp_gap_sec':(ns-pns)*1e-9,
           'previous_complete_to_wait_enter_sec':enter-prev['read_completed_monotonic'] if enter is not None else None,
           'host_wait':hw,'read_total_sec':ev.get('total_sec'),'stages_sec':ev.get('stages_sec'),
           'within_last_good_to_loss_source_interval':last_good <= pns < ns <= lost_ns,
           'streams':{}}
        for stream in ('color','depth'):
            pr=prev['evidence'].get('raw_frames',{}).get(stream,{})
            cr=ev.get('raw_frames',{}).get(stream,{})
            item={'previous_original':pr,'current_original':cr}
            for key in ('frame_number','device_timestamp_ms'):
                if key in pr and key in cr: item[key+'_delta']=cr[key]-pr[key]
            pm=pr.get('metadata_raw',{}); cm=cr.get('metadata_raw',{})
            item['metadata_deltas']={k:cm[k]-pm[k] for k in ('frame_timestamp','backend_timestamp','time_of_arrival') if k in pm and k in cm}
            g['streams'][stream]=item
        gaps.append(g)
    critical=[r for r in rows if r['source_stamp_ns'] and last_good<=r['source_stamp_ns']<=lost_ns]
    native_window=[e for e in ordered if last_good<=int(e['source_stamp_ns'])<=lost_ns]
    center_summary=collections.Counter()
    for e in native_window:
        r=e['evidence'].get('regional_stationary_evidence',{})
        c=next((v for v in r.get('regions',[]) if v.get('name')=='center'),{})
        center_summary[c.get('visual',{}).get('reason','missing')]+=1
    report={
      'schema':1,'scope':'sealed gen26 metadata-only first-loss diagnosis; no pixels, no active bags, no ROS',
      'case':str(case),'generation':generation,'selection_stamp_ns':selection_ns,'capture_summary':summary,
      'source_integrity':{'summary_sha256':sha(summary_bytes),'new_target_selection_sha256':sha(selection_bytes),
         'entry_manifest_sha256':sha(manifest_bytes),'verified_entries':verified,
         'all_read_status_original_utf8_sha256_verified':True,
         'pixel_entries_not_read_or_hash_verified':True,'whole_archive_not_rehashed':True,
         'metadata_errors':metadata_errors,'sdk_unique_exact_source_count':len(sdk)},
      'first_loss_original':loss,'first_loss_original_status_data':lost_pair[0]['original_status_data'],
      'first_loss_status_index':lost_pair[0]['status_index'],
      'first_loss_current_frame_age_sec':(observed-lost_ns)*1e-9,
      'first_loss_last_good_age_sec':(observed-last_good)*1e-9,
      'first_loss_after_selection_sec':(observed-selection_ns)*1e-9,
      'status_counts_until_first_loss':dict(collections.Counter(s['detail'] for w,s in before)),
      'critical_status_rows':critical,'all_status_rows_until_first_loss':rows,
      'sdk_acquired_to_first_loss_gaps':gaps,
      'largest_sdk_gaps':sorted(gaps,key=lambda x:x['read_completed_gap_sec'],reverse=True)[:8],
      'last_good_to_loss_visible_sdk_reads':len(native_window),'last_good_to_loss_center_reasons':dict(center_summary),
      'ingress_until_first_loss':{stream:{k:stats([r['streams'][stream][k] for r in rows if k in r['streams'][stream]]) for k in ('unknown_after_sdk_complete_sec','recorder_after_sdk_complete_sec','unknown_minus_recorder_arrival_sec','unknown_ros_monotonic_age_disagreement_sec')} for stream in ('color','depth')},
      'interpretation':{
        'established':[
         'The first loss rejects an 851.191 ms old last-good support reference; its current exact RGBD source is only 33.235 ms old. This is not a replay of the historical two-second current-frame receive delay.',
         'Three intervening frames explicitly report support_plane_inlier_ratio_low with support_audit null and no measured support. They do not refresh the old reference. None contains a support_gap_measured_revalidation stage.',
         'Last-good processing took 288.881 ms including 157.284 ms TF wait; the last pending processing took 143.825 ms including 71.644 ms TF wait. These wall-time costs reduce the time available before the unchanged expiry.',
         'A genuine 742.508 ms SDK wait/native time gap occurred earlier, followed by four tracked statuses; it is real but does not alone explain the later first-loss transition.',
         'The last-good-to-loss interval also contains a 222.472 ms read gap dominated by 195.176 ms outside SDK read, and a separate 93.230 ms SDK wait. Native timestamps progress discontinuously on both streams in these gaps.',
         'All three pending sources lack qualified center temporal filtering (warming_up, visual_motion, visual_motion). Last-good itself was unfiltered yet its current support audit passed, so lack of filtering alone is not a demonstrated cause.'
        ],
        'limitations':[
         'Image source headers denote host read completion, not calibrated exposure time. Native metadata and frame counts do not isolate hardware, USB, VM, scheduling, or SDK causes.',
         'The early source-rejection status has no frame_timing: its unknown callback delay is not invented from the separate recorder receipt.',
         'This analysis does not refit saved image rings or prove why the three support measurements fail; possible noise, real motion or multiple planes remain unresolved.',
         'No CPU or scheduler trace was captured for the 195 ms read-external interval; publication, locks, allocation, diagnostics and scheduling cannot be separated here.',
         'The metadata-only integrity check verifies each accessed ZIP entry and original status string, not all pixel entries or the whole large archive.',
         'No gates were changed, no retired target revived, no new sample or motion was requested. The still-recording full bag was not accessed.'
        ],
        'production_fix_established':False,'freshness_or_quality_relaxation_recommended':False,
        'next_focused_work':'Investigate current support measurement quality and bounded TF/processing cost using the sealed exact-source evidence; do not treat a fresh current frame as refreshing unmeasured support.'
      },
      'analysis_source_sha256':sha(Path(__file__).read_bytes())}
    output.mkdir(parents=True,exist_ok=False)
    b=(json.dumps(report,ensure_ascii=False,sort_keys=True,indent=2)+'\n').encode()
    (output/'first_loss_report.json').write_bytes(b)
    (output/'SHA256SUMS').write_text(sha(b)+'  first_loss_report.json\n'+report['analysis_source_sha256']+'  '+str(Path(__file__).resolve())+'\n')
    print(json.dumps({'report':str(output/'first_loss_report.json'),'sha256':sha(b),'generation':generation,
      'current_frame_age_sec':report['first_loss_current_frame_age_sec'],'last_good_age_sec':report['first_loss_last_good_age_sec'],
      'counts':report['status_counts_until_first_loss'],'visible_center':report['last_good_to_loss_center_reasons'],
      'ingress':report['ingress_until_first_loss']},ensure_ascii=False))


if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('--case',type=Path,required=True);ap.add_argument('--output',type=Path,required=True)
    a=ap.parse_args();main(a.case.resolve(),a.output.resolve())
