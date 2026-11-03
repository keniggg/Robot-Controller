"""Summarize sealed source-window stage costs without further image loading."""
from collections import Counter
import hashlib
import json
from pathlib import Path
import numpy as np

P=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/aligned_grasp_20261007T112309.952794Z/recorded_retry_20261007T120118Z/raw_support/analysis')


def stats(values):
    values=[x for x in values if isinstance(x,(int,float)) and np.isfinite(x)]
    return None if not values else dict(count=len(values),p50_ms=float(np.percentile(values,50))*1000,
        p95_ms=float(np.percentile(values,95))*1000,max_ms=float(max(values))*1000)


def main():
    report=json.loads((P/'report.json').read_text()); groups={}
    for row in report['timeline']:
        if row['detail']=='support_measurement_pending_revalidation': group='pending'
        elif (row.get('source_qualification') or {}).get('state')=='recovered_unique_measured_frame': group='recovered'
        elif row['state']=='ready': group='ordinary_ready'
        else:group='initial_unavailable'
        groups.setdefault(group,[]).append(row)
    result=dict(report_sha256=hashlib.sha256((P/'report.json').read_bytes()).hexdigest(),
        diagnostic_only=True,motion_authorized=False,stages={},source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    for group,rows in groups.items():
        timings=[r.get('frame_timing') or {} for r in rows]
        summary=dict(status_count=len(rows))
        for key in ('processing_wall_sec','processing_thread_cpu_sec','status_source_age_sec',
                    'processing_start_source_age_sec','ingress_to_processing_sec','exact_pair_source_age_sec'):
            summary[key]=stats([t.get(key) for t in timings])
        for section in ('stage_wall_sec','stage_thread_cpu_sec'):
            keys=sorted({k for t in timings for k in t.get(section,{})})
            summary[section]={k:stats([t.get(section,{}).get(k) for t in timings]) for k in keys}
        result['stages'][group]=summary
    result['pending_replay']=[dict(source_stamp_ns=r['source_stamp_ns'],
        returned_tf_delta_ns=r.get('tf_return_delta_ns'),original_audit=r.get('original_locked_audit'),
        sampled_ratios=[q['fit']['original_locals'].get('ratio') for q in r['rings']],
        all_ring_ratios=[q.get('all_actual_ring_inlier_ratio_diagnostic') for q in r['rings']],
        center_visual_reason=(r.get('center_filter') or {}).get('visual',{}).get('reason')) for r in report['pending_replay']]
    result['conclusions']=[
        'Nine pending sources all reproduce support_plane_inlier_ratio_low on all original rings; this is not merely a wrong status reason.',
        'The same original models evaluated on full rings also stay below 0.7 for all 27 pending ring fits, so deterministic sampling alone does not explain these failures.',
        'Eight recovery transitions in the window succeed before the original deadline. No gen23 loss occurs inside the 15-second archive.',
        'Source arrival is fresh; on-CPU image and revalidation work plus TF wall wait consume a substantial part of the last-good age budget.',
        'These timing values are producer diagnostics, not recorder arrival latency. They do not establish cause of the later outside-window loss.']
    raw=(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False)+'\n').encode()
    (P/'timing_and_interpretation.json').write_bytes(raw)
    (P/'timing_and_interpretation.sha256').write_text(hashlib.sha256(raw).hexdigest()+'  timing_and_interpretation.json\n')
    print(json.dumps(dict(sha256=hashlib.sha256(raw).hexdigest(),groups={k:{n:v for n,v in d.items() if n in ('status_count','processing_wall_sec','processing_thread_cpu_sec','status_source_age_sec')} for k,d in result['stages'].items()})))


if __name__=='__main__':main()
