"""Read-only extraction of exact rejected SDK pair and asynchronous filter status."""
import json,hashlib
from pathlib import Path
import zipfile
BASE=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/gen17_support_measurement_20261006')
LOG=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/unknown_grasp_20260923/supervision_continuous.log')

def main():
 with zipfile.ZipFile(BASE/'raw_support_v2/diagnostic.zip')as z:
  rows=sorted([json.loads(z.read(n))for n in z.namelist()if n.startswith('metadata/')],key=lambda r:r['sequence'])
  timeline=[]
  for r in rows:
   s=r['parameters']['/camera']['runtime_profile']['stationary_temporal'];regions=[]
   for region in s.get('regions',[]):
    v=region.get('visual',{});item=dict(region)
    if 'quiet_span_sec'in v:
     item['derived_anchor_device_timestamp_ms']=v['device_timestamp_ms']-1000*v['quiet_span_sec']
     item['derived_anchor_frame_number']=v['frame_number']-v['continuous_frames']+1
    regions.append(item)
   timeline.append(dict(sequence=r['sequence'],read_start_ros_ns=r['read_start_ros_ns'],read_end_ros_ns=r['read_end_ros_ns'],filtered_fraction=s['filtered_fraction'],regions=regions))
 with LOG.open('rb')as f:
  offset=max(0,LOG.stat().st_size-5000000);f.seek(offset);data=f.read().decode(errors='replace')
 logs=[]
 for line in data.splitlines():
  try:when=float(line.split(' ',1)[0])
  except ValueError:continue
  if not(1791282094<=when<=1791282097 and '/supervisor_camera'in line):continue
  item=dict(supervision_receipt_wall_sec=when,original_line=line)
  prefix='Camera RGB-D pair unavailable: '
  if prefix in line:item['read_diagnostic']=json.loads(line.split(prefix,1)[1])
  logs.append(item)
 bad=next(r['read_diagnostic']for r in logs if r.get('read_diagnostic',{}).get('raw_frames',{}).get('color',{}).get('frame_number')==778405)
 prior=bad['previous_completed_raw_frames']['raw_frames'];current=bad['raw_frames']
 relation={stream:dict(previous_complete_frame_number=prior[stream]['frame_number'],rejected_frame_number=current[stream]['frame_number'],
   frame_number_delta=current[stream]['frame_number']-prior[stream]['frame_number'],
   device_timestamp_delta_ms=current[stream]['device_timestamp_ms']-prior[stream]['device_timestamp_ms'],
   raw_evidence_identical=current[stream]==prior[stream])for stream in ('color','depth')}
 report=dict(schema=1,source_log=str(LOG),bounded_tail_offset_bytes=offset,log_rows=logs,stationary_parameter_timeline=timeline,
   rejected_vs_previous_complete=relation,
   direct_findings=['The rejected color metadata is exactly identical to the last completed read: frame778405 at device5634.056ms.',
    'The rejected depth is the immediately next depth frame780420, +33.338ms (rounded) after prior780419.',
    'Pair skew33.0341796875ms exceeds unchanged16.666666666666668ms threshold; rejection occurs before SDK align, any depth filter, and regional processing.',
    'RealSenseManager.read finally resets histories for every non-complete result, including this pre-filter rejected pair.',
    'The subsequent sampled center window has 5 continuous RGB frames ending778410/device5800.7468ms, deriving a new anchor778406/device5667.394ms immediately after the rejected pair.'],
   proof_limits=['No per-read successful raw SDK metadata was recorded after this rejected pair. Next accepted COLOR anchor is derivable; next accepted DEPTH identity/timestamp is not available. Do not assert proven two-stream continuity for this historical event.',
    'The later center anchor changes again to device5934.0981ms/frame778413. Successful snapshots are asynchronous and warn logs are throttled; second reset cause cannot be conclusively assigned from this capture.',
    'Joint feedback/TF unchanged at sampled instants does not exclude every intervening feedback freshness failure or physical micro-motion.',
    'Regional parameter summaries omit the raw global_history_reset_reason. They report current permission, not a complete reset event history.'],
   minimum_contract_proposal=['Reject the unusable SDK pair and publish no pair exactly as before.',
    'Only a fully identified timestamp_skew_exceeded packet rejected before all align/filter stages may put history into private quarantine; all other failures hard-reset.',
    'The next usable pair can resume only when BOTH raw RGB and raw DEPTH are exactly +1 relative to the last complete accepted pair, with same profile/domain/fps, original device/host interval bounds, and unchanged geometry/filter/pipeline/mode provenance.',
    'Re-evaluate existing accepted+SDK stationary evidence and existing regional visual permission before using history; no extension of .35s or host-gap bounds, no restoration after any independent reset.',
    'Any duplicate/backward/skipped stream frame, long wait, missing or malformed metadata, changed profile, mode, owner or feedback invalidates quarantine and creates fresh history; persistence0/current-hole checks remain unchanged.',
    'Record both previous/next raw source identities and explicit resume/reset reason, including reset before any filtered output; no global maximum-angle or grasp authority changes.'],
   necessary_counterexamples=['Repeated oldcolor/newdepth then next RGB+1/depth+1 is the only basic recoverable pattern; identical output to uninterrupted accepted sequence must be tested.',
    'RGB+1 but depth+2, depth duplicate, either reversed counter/time/domain/fps, or host interval>.1s must reset.',
    'Good color tracking cannot compensate for missing depth provenance.',
    'Any read failure after an SDK spatial/temporal filter has run must reset; do not assume filter state untouched.',
    'Multiple bad packets cannot extend the last accepted host deadline, increment stationary sample count, publish stale data, or refresh target identity.',
    'Manual/SDK motion, accepted feedback expiry, visual motion/insufficient tracks, and preset/pipeline/alignment changes must reset despite otherwise consecutive frames.'],
   production_files_read={str(p):hashlib.sha256(p.read_bytes()).hexdigest()for p in [Path('src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision')/n for n in ('realsense_manager.py','regional_stationary_depth.py','visual_stationary.py','stationary_depth.py')]},
   scope='Read-only analysis; no proposed behavior implemented, no camera or robot operations.')
 out=BASE/'stationary_reset_evidence_v2.json';assert not out.exists();out.write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
 print(json.dumps(dict(path=str(out),rejected_vs_previous_complete=relation,sha256=hashlib.sha256(out.read_bytes()).hexdigest()),ensure_ascii=False))
if __name__=='__main__':main()
