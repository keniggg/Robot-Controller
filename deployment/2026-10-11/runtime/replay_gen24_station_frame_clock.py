#!/usr/bin/env python3
"""Saved-pixel gate replay plus isolated real-producer recorded-clock probe."""
from pathlib import Path
import json,hashlib,argparse
import numpy as np,cv2
from _pytest.monkeypatch import MonkeyPatch
from alicia_grasp_modes import tabletop as table,rgb_instance as rgbmod
from alicia_grasp_modes.observation_arrival_confirmation import ObservationArrivalConfirmation
from alicia_grasp_modes.arrival_fragment_group import recover_arrival_fragment_group
from alicia_flexible_grasp.vision.rgbd_rectification import rectification_rgb_valid_mask

ROOT=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_ros_20261009T003407Z/gen24_handoff_analysis')
parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True)
parser.add_argument('--producer-source',type=Path)
args=parser.parse_args();OUT=args.output;OUT.mkdir(exist_ok=True)
IN=ROOT/'source_images';meta=json.loads((IN/'manifest.json').read_text());camera=meta['camera']
timing=json.loads((ROOT/'first_loss_timing_and_station.json').read_text())
K=[camera[k] for k in ('fx','fy','cx','cy')];cfg=table.Config();cv2.setNumThreads(1)
def row_for(role=None,ns=None):
 return next(r for r in meta['files'] if (role is None or role in r['roles']) and (ns is None or r['source_stamp_ns']==ns))
def frame(row):
 a=np.load(IN/row['npz']);dep=np.asarray(a['depth_raw'],np.float32)*camera['depth_scale']
 return a,a['rgb'],dep,a['T_base_camera'],rectification_rgb_valid_mask(camera,dep.shape)
a,rgb,dep,T,valid=frame(row_for('anchor'));seg=table.segment_with_support_fallback(dep,K,T,cfg)
t=table.TargetTracker(cfg);anchor,reason=t.choose(seg,dep.shape);assert anchor is not None,reason
expected=meta['anchor_status'];plane=np.asarray(expected['reference_support_plane_base'])
assert np.allclose(anchor.position_base,expected['anchor_position_base'],atol=1e-10,rtol=0)
assert np.allclose(seg.plane_base,plane,atol=1e-10,rtol=0)
a,rgb,dep,T,valid=frame(row_for('view1_last_pre_motion'))
seg=table.segment_with_support_fallback(dep,K,T,cfg,anchor,anchor,plane,prefer_current_support=True)
matches=table._matching_candidates(seg.candidates,anchor,anchor,cfg);assert len(matches)==1
last=matches[0][1];refined=rgbmod.refine_candidate(rgb,dep,K,T,last,cfg,rgb_valid=valid)
assert np.array_equal(refined.mask,a['refined_mask'])
helper=ObservationArrivalConfirmation(cfg,anchor,last,plane,a['histogram'],a['contour'],.5)
qualified_ns=1791543687167207002;failed_ns=1791543687661621093
a,rgb,dep,T,valid=frame(row_for(ns=qualified_ns))
seg=table.segment_with_support_fallback(dep,K,T,cfg,anchor,last,plane,prefer_current_support=True)
_,candidate,staged,evidence=helper.observe(seg,rgb,dep,K,T,qualified_ns,rgb_valid=valid)
assert np.array_equal(candidate.mask,a['refined_mask'])
actual_last=staged.last
a,rgb,dep,T,valid=frame(row_for(ns=failed_ns))
seg=table.segment_with_support_fallback(dep,K,T,cfg,anchor,actual_last,plane,prefer_current_support=True)
raw_matches=len(table._matching_candidates(seg.candidates,anchor,actual_last,cfg))
seg,pair=recover_arrival_fragment_group(seg,rgb,dep,K,T,cfg,anchor,actual_last,helper.spatial_reference,
 helper.histogram,helper.contour,.5,rgb_valid=valid)
assert pair is not None and len(table._matching_candidates(seg.candidates,anchor,actual_last,cfg))==1
cv2.imwrite(str(OUT/'current_original_gates_mask.png'),pair[1].mask)
pixel_result=dict(qualified_mask_bitexact=True,raw_matches=raw_matches,fragment_pair=True,
 original_group_geometry_rgb_identity_passed=True,metrics=pair[1].metrics,support=seg.metrics,
 limitation='One saved qualified frame reconstructs its current last; its preceding online three-frame qualification is recorded separately, not recreated here.')

from test_arrival_fragment_group import completed_fragment_arrival,split_depth
from test_locked_rgb_hold import custom_frame,latest_status,module
if args.producer_source is not None:
 module.__file__=str(args.producer_source)
 exec(compile(args.producer_source.read_text(),module.__file__,'exec'),module.__dict__)
clock_results=[]
for finish_age in (.79,timing['old_reference_age_at_failure_sec']):
 with MonkeyPatch().context() as mp:
  n,clock,feed,joint,params,marker=completed_fragment_arrival(mp)
  old=n._locked_rgb_hold_reference;old_ns=old['source_stamp_ns'];old_last=n.tracker.last
  # Build the next source, then start processing at the recorded relative age.
  process=n.process;n.process=lambda event:None
  clock[0]=old_ns*1e-9+timing['capture_gap_sec']+.1
  source=custom_frame(n,clock,joint,target_depth=split_depth());n.process=process
  clock[0]=old_ns*1e-9+timing['old_reference_age_at_processing_start_sec']
  original=module.recover_arrival_fragment_group;completed=[]
  def measured(*args,**kwargs):
   value=original(*args,**kwargs);completed.append(value[1] is not None)
   clock[0]=old_ns*1e-9+finish_age
   return value
  mp.setattr(module,'recover_arrival_fragment_group',measured)
  n.process(None)
  clock_results.append(dict(finish_reference_age_sec=finish_age,group_completed=completed,
   status=latest_status(n),marker_invalidated_reason=marker.get('invalidated_reason'),
   last_changed=n.tracker.last is not old_last,source_renewed=n.last_good_source_stamp_ns!=old_ns,
   final_detected=n.pubs['object'].publish.call_args[0][0].detected))
report=dict(scope='OFFLINE_ACTUAL_PIXEL_GATES_AND_SYNTHETIC_TRANSPORT_RECORDED_RELATIVE_CLOCK',
 record=meta['record'],current_source_stamp_ns=failed_ns,reference_source_stamp_ns=qualified_ns,
 pixel_result=pixel_result,producer_clock_probe=clock_results,
 source_sha256={module.__file__:hashlib.sha256(Path(module.__file__).read_bytes()).hexdigest()},
 limitations=['Pixels/TF/anchor and qualified reference are recorded; producer timing probe separately uses an existing synthetic ROS-transport fixture with the real producer and exact recorded relative ages.',
 'No production edit, ROS request, or execution authority; this does not assert an unrecorded private runtime exception verbatim.'])
(OUT/'result.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps({'pixel_gates_passed':True,'clock_results':[{k:r[k] for k in ('finish_reference_age_sec','group_completed','marker_invalidated_reason','last_changed','source_renewed','final_detected')} for r in clock_results]}))
print('RESULT',OUT/'result.json')
