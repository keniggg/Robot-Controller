#!/usr/bin/env python3
"""Read only sealed status/metadata ZIP entries; never decode RGB-D pixels."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import zipfile
import numpy as np

W=Path(__file__).resolve().parents[1]
EXT=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def distribution(values):
 a=np.asarray([v for v in values if v is not None],float)
 return None if not len(a) else dict(count=len(a),minimum=float(a.min()),p50=float(np.percentile(a,50)),p95=float(np.percentile(a,95)),maximum=float(a.max()))
def main():
 p=argparse.ArgumentParser();p.add_argument('--run',type=Path,default=EXT/'ros_log/gen19_support_measurement_20261006');a=p.parse_args()
 run=json.loads((a.run/'run.json').read_text());summary=json.loads((a.run/'raw_support/summary.json').read_text())
 assert run['complete'] and run['source_unchanged'] and not run.get('error')
 assert summary['archive_closed'] and summary['complete_without_writer_error']
 out=a.run/'status_review.json';assert not out.exists() and EXT.resolve() in out.resolve().parents
 rows=[];metadata=[]
 with zipfile.ZipFile(a.run/'raw_support/diagnostic.zip') as z:
  for name in z.namelist():
   if name.startswith('status/'):
    r=json.loads(z.read(name));s=json.loads(r['original_status_data'])
    assert hashlib.sha256(r['original_status_data'].encode()).hexdigest()==r['original_status_utf8_sha256']
    if s.get('generation')==19:
     t=s.get('frame_timing',{});q=s.get('source_qualification') or {};g=s.get('geometry_metrics') or {}
     rows.append(dict(entry=name,receipt_ros_ns=r['receipt_ros_ns'],source_stamp_ns=s.get('source_stamp_ns'),
      state=s['state'],detail=s['detail'],target_lost=s.get('target_lost'),first_loss=s.get('first_loss'),
      anchor_source_stamp_ns=s.get('anchor_source_stamp_ns'),selection_stamp_ns=s.get('selection_stamp_ns'),
      exact_rgbd=s.get('exact_rgbd'),color_stamp_ns=s.get('color_stamp_ns'),depth_stamp_ns=s.get('depth_stamp_ns'),
      source_qualification=q,status_source_age_sec=t.get('status_source_age_sec'),
      ingress_source_age_sec=t.get('ingress_source_age_sec'),processing_wall_sec=t.get('processing_wall_sec'),
      processing_thread_cpu_sec=t.get('processing_thread_cpu_sec'),
      raw_arrival_difference_sec=t.get('raw_arrival_difference_sec'),
      support_audit={k:v for k,v in g.items() if k.startswith('support_current_audit') or k.startswith('support_reference')},
      original_status_sha256=r['original_status_utf8_sha256']))
   elif name.startswith('metadata/'):
    r=json.loads(z.read(name));params=r['parameters'];s=params['/camera'].get('runtime_profile',{}).get('stationary_temporal',{})
    metadata.append(dict(entry=name,read_start_ros_ns=r['read_start_ros_ns'],read_end_ros_ns=r['read_end_ros_ns'],
     errors=r.get('errors'),selection=params['/grasp_mode/selection'],filtered_fraction=s.get('filtered_fraction'),
     center=next((x for x in s.get('regions',[]) if x.get('name')=='center'),None),
     pair_history=s.get('pair_history'),diagnostic_association='parameter read interval only; not exact status/image source proof'))
 rows.sort(key=lambda r:r['receipt_ros_ns']);episodes=[];pending=[];last_ready=None
 for row in rows:
  if row['detail']=='support_measurement_pending_revalidation':pending.append(row)
  elif row['state']=='ready':
   if pending:
    episodes.append(dict(pending_count=len(pending),first_pending_source_ns=pending[0]['source_stamp_ns'],
     last_pending_source_ns=pending[-1]['source_stamp_ns'],reference_source_ns=pending[0]['source_qualification']['reference_source_stamp_ns'],
     max_reported_elapsed_since_measured_sec=max(x['source_qualification']['elapsed_since_measured_sec'] for x in pending),
     recovered=True,recovery_source_ns=row['source_stamp_ns'],recovery_status_age_sec=row['status_source_age_sec'],
     recovery_from_reference_source_sec=(row['source_stamp_ns']-pending[0]['source_qualification']['reference_source_stamp_ns'])/1e9,
     recovery_kind=row['source_qualification'].get('state'),
     recovery_support_audit=row['source_qualification'].get('support_audit'),
     recovery_rgb_identity_verified=row['source_qualification'].get('identity_verified')))
    pending=[]
   last_ready=row
 if pending:
  episodes.append(dict(pending_count=len(pending),first_pending_source_ns=pending[0]['source_stamp_ns'],last_pending_source_ns=pending[-1]['source_stamp_ns'],
   recovered=False,max_reported_elapsed_since_measured_sec=max(x['source_qualification']['elapsed_since_measured_sec'] for x in pending)))
 violations=[]
 for r in rows:
  if r['source_stamp_ns'] is not None and (not r['exact_rgbd'] or r['source_stamp_ns']!=r['color_stamp_ns'] or r['source_stamp_ns']!=r['depth_stamp_ns']):violations.append(dict(entry=r['entry'],reason='source_pair_not_exact'))
  if r['detail']=='support_measurement_pending_revalidation':
   q=r['source_qualification']
   if q.get('metric_geometry_authority') is not False or q.get('motion_authorized') is not False or q.get('support_measurement_available') is not False:
    violations.append(dict(entry=r['entry'],reason='pending_retained_geometry_authority'))
 sources=[x['source_stamp_ns'] for x in rows if x['source_stamp_ns'] is not None]
 report=dict(schema=1,run=str(a.run),input_run_sha256=sha(a.run/'run.json'),input_summary_sha256=sha(a.run/'raw_support/summary.json'),
  script_sha256=sha(Path(__file__)),status_count=len(rows),status_counts=dict(Counter(x['detail'] for x in rows)),
  unique_sources=len(set(sources)),statuses_without_source=len(rows)-len(sources),anchor_sources=sorted({x['anchor_source_stamp_ns'] for x in rows if x['anchor_source_stamp_ns'] is not None}),
  statuses_without_anchor=sum(x['anchor_source_stamp_ns'] is None for x in rows),
  source_span_sec=(max(sources)-min(sources))/1e9,
  target_lost_count=sum(bool(x['target_lost'] or x['first_loss']) for x in rows),
  timing={key:distribution(x[key] for x in rows) for key in ('status_source_age_sec','ingress_source_age_sec','processing_wall_sec','processing_thread_cpu_sec','raw_arrival_difference_sec')},
  pending_episodes=episodes,last_state={k:rows[-1][k] for k in ('state','detail','source_stamp_ns','status_source_age_sec')},
  pending_reasons=dict(Counter(x['source_qualification'].get('reason') for x in rows if x['detail']=='support_measurement_pending_revalidation')),
  metadata_count=len(metadata),metadata=metadata,rows=rows,violations=violations,
  limitations=['No image entries decoded or read; no independent support refit.','Parameter snapshots have read intervals, not exact per-source SDK gate association.',
   'Only this 15-second perception window showed no retirement; persistent support pending remains, no root-cause cure or grasp success claim.',
   'Record budget omitted some images explicitly; 58 image pairs saved out of status associations, not a complete raw stream.'])
 out.write_text(json.dumps(report,indent=2,ensure_ascii=False,allow_nan=False)+'\n')
 print(json.dumps({k:report[k] for k in ('status_count','status_counts','unique_sources','source_span_sec','target_lost_count','timing','pending_episodes','last_state','violations')},ensure_ascii=False))
if __name__=='__main__':main()
