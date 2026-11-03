#!/usr/bin/env python3
"""Reconstruct original arrival proposals only; never turns rejected masks valid."""
from pathlib import Path
from dataclasses import fields
import hashlib,json,time,inspect
import numpy as np,cv2
from alicia_grasp_modes import tabletop as table
from alicia_grasp_modes.rgb_instance import refine_candidate
from alicia_grasp_modes.observation_appearance import FrozenObservationAppearance
from alicia_grasp_modes.rgb_observation_handoff import _appearance,_mask
from alicia_grasp_modes.observation_arrival_confirmation import ObservationArrivalConfirmation
from alicia_flexible_grasp.vision.rgbd_rectification import rectification_rgb_valid_mask
import alicia_grasp_modes.arrival_fragment_group as fragment
O=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_contact_admission_20261013T011103Z/verification/gen14_arrival_independent')
ANCHOR=1791860167282787322;PRE=1791860176945733070

def plain(x):
 if isinstance(x,np.ndarray):return x.tolist()
 if isinstance(x,np.generic):return x.item()
 if isinstance(x,dict):return {k:plain(v)for k,v in x.items()}
 if isinstance(x,(tuple,list)):return [plain(v)for v in x]
 return x

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
 cv2.setNumThreads(1);f=json.loads((O/'fixture.json').read_text());c=f['camera'];K=[c[k]for k in('fx','fy','cx','cy')];names={x.name for x in fields(table.Config)};config=table.Config(**{k:v for k,v in f['unknown_parameters'].items()if k in names})
 def load(ns):
  d=np.load(O/f'{ns}_source.npz');valid=rectification_rgb_valid_mask(c,d['depth'].shape);return d,{}if valid is None else dict(rgb_valid=valid)
 d,kw=load(ANCHOR);r=table.segment_with_support_fallback(d['depth'],K,d['T_base_camera'],config);t=table.TargetTracker(config);anchor,reason=t.choose(r,d['depth'].shape);plane=r.plane_base.copy()
 d,kw=load(PRE);r=table.segment_with_support_fallback(d['depth'],K,d['T_base_camera'],config,anchor,anchor,plane);matches=table._matching_candidates(r.candidates,anchor,anchor,config);assert len(matches)==1;last=matches[0][1];refined=refine_candidate(d['rgb'],d['depth'],K,d['T_base_camera'],last,config,**kw)
 hist,contour=_appearance(d['rgb'],_mask(d['recorded_mask'],d['rgb'].shape[:2]));frozen=FrozenObservationAppearance(d['rgb'],d['recorded_mask'],kw.get('rgb_valid'),PRE,hist,contour)
 seed=dict(premotion_last_metrics=last.metrics,premotion_refined_metrics=refined.metrics,premotion_last_support_footprint=last.support_footprint_base,anchor_position_base=anchor.position_base,anchor_extent_m=anchor.extent_m,reference_plane_base=plane,premotion_raw_bbox=last.bbox,premotion_recorded_mask_sha256=hashlib.sha256(d['recorded_mask'].tobytes()).hexdigest(),premotion_refinement_exactly_matches_recorded_mask=bool(np.array_equal(refined.mask,d['recorded_mask'])),premotion_accepted_recorded_mask_seed=True)
 original=fragment.match_observation_appearance;rows=[]
 for status in json.loads((O/'arrival_statuses.json').read_text()):
  ns=status['message']['source_stamp_ns'];d,kw=load(ns);r=table.segment_with_support_fallback(d['depth'],K,d['T_base_camera'],config,anchor,last,plane,prefer_current_support=True);captured=[]
  def capture(rgb,mask,*args,**kwargs):
   caller=inspect.currentframe().f_back.f_locals
   summaries={}
   for key in ('proposal','refined'):
    value=caller.get(key)
    if value is not None:summaries[key]=dict(position_base=value.position_base,extent_m=value.extent_m,bbox=value.bbox,quality=value.quality,metrics=value.metrics,support_footprint_base=value.support_footprint_base)
   evidence=original(rgb,mask,*args,**kwargs);index=len(captured);path=O/f'{ns}_rejected_proposal_{index}_mask.png';cv2.imwrite(str(path),mask)
   cv2.imwrite(str(O/f'{ns}_rejected_proposal_{index}_mask_visible.png'),(mask>0).astype(np.uint8)*255)
   captured.append(dict(geometry=summaries,mask_path=str(path),mask_sha256=hashlib.sha256(mask.tobytes()).hexdigest(),original_evidence=evidence));return evidence
  fragment.match_observation_appearance=capture
  conf=ObservationArrivalConfirmation(config,anchor,last,plane,hist,contour,.5,appearance_reference=frozen);started=time.perf_counter()
  try:
   result=conf.observe(r,d['rgb'],d['depth'],K,d['T_base_camera'],ns,appearance_deadline_monotonic=time.monotonic()+5,appearance_check_current=lambda:None,**kw);reason='qualified'if result[0]else result[3]['reason']
  except ValueError as e:reason=str(e)
  finally:fragment.match_observation_appearance=original
  row=dict(source_ns=ns,original_recorded_reason=status['message']['observation_measurement']['reason'],reconstructed_geometry_reason=reason,candidate_count=len(r.candidates),original_metric_matching_count=len(table._matching_candidates(r.candidates,anchor,last,config)),measured_wall_sec=time.perf_counter()-started,proposals=captured,original_clock_replayed=False,diagnostic_geometry_only=True)
  rows.append(row);print(json.dumps(dict(source_ns=ns,reason=reason,masks=len(captured))),flush=True)
 report=dict(scope='Actual original geometry/native refinement and original premotion appearance; diagnostic generous local time only, not actual source clock replay or qualification success',seed=seed,rows=rows,source_sha256={str(Path(m.__file__)):sha(Path(m.__file__))for m in(table,fragment)},no_ros_master=True,motion_commands_sent=False)
 (O/'original_geometry_reconstruction_v2.json').write_text(json.dumps(plain(report),ensure_ascii=False,indent=2)+'\n')
if __name__=='__main__':main()
