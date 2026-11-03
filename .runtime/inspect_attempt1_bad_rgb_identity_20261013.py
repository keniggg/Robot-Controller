#!/usr/bin/env python3
"""Original RGB proposals and native refinements from sealed GOOD/BAD/NEXT."""
import argparse
from dataclasses import asdict, replace
import importlib.util
import json
from pathlib import Path
import time
from unittest.mock import patch
import cv2
import numpy as np
from alicia_grasp_modes.rgb_instance import _FrameRefinement
from alicia_grasp_modes.rgb_observation_region import CenterSearchPolicy, propose_rgb_observation_regions
from alicia_grasp_modes.rgb_observation_handoff import HandoffPolicy
from alicia_grasp_modes.observation_appearance import match_observation_appearance
import alicia_grasp_modes.arrival_fragment_group as fragment_module

W=Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
spec=importlib.util.spec_from_file_location('input_replay',W/'.runtime/replay_attempt1_first_phase_sources_20261013.py');m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
N=m.ROOT/'ros_log/full_latest_contact_admission_20261013T011103Z';F=N/'verification/attempt1_first_phase_with_qualified_source'
def main():
    p=argparse.ArgumentParser();p.add_argument('--output',required=True);args=p.parse_args();out=Path(args.output);out.mkdir(parents=True,exist_ok=False);cv2.setNumThreads(1)
    fixture=json.loads((F/'fixture.json').read_text());camera=fixture['camera'];cfg=m.table.Config(**fixture['config']);K=[camera[x]for x in('fx','fy','cx','cy')]
    def frame(ns):
        f=np.load(F/f'{ns}_source.npz');return f['rgb'],f['depth'],f['T_base_camera'],({'rgb_valid':f['rgb_valid']}if'rgb_valid'in f else{}),f
    rgb,depth,T,kw,_=frame(fixture['frames'][0]['source_stamp_ns']);seg=m.table.segment_with_support_fallback(depth,K,T,cfg);tracker=m.table.TargetTracker(cfg);anchor,_=tracker.choose(seg,depth.shape);plane=seg.plane_base.copy();spatial=m.freeze_observation_spatial_reference(anchor,plane)
    rgb,depth,T,kw,data=frame(m.PRE);hist,contour=m._appearance(rgb,m._mask(data['recorded_mask'],rgb.shape[:2]));frozen=m.FrozenObservationAppearance(rgb,data['recorded_mask'],kw.get('rgb_valid'),m.PRE,hist,contour);seg=m.table.segment_with_support_fallback(depth,K,T,cfg,anchor,anchor,plane);last=m.table._matching_candidates(seg.candidates,anchor,anchor,cfg)[0][1]
    rgb,depth,T,kw,data=frame(m.GOOD);seg=m.table.segment_with_support_fallback(depth,K,T,cfg,anchor,last,plane,prefer_current_support=True);seg,pair=m.recover_arrival_matching_fragments(seg,rgb,depth,K,T,cfg,anchor,last,spatial,hist,contour,.5,appearance_context=dict(reference=frozen,source_ns=m.GOOD,deadline_monotonic=time.monotonic()+5,check_current=lambda:None),**kw);assert pair is not None and np.array_equal(pair[1].mask,data['recorded_mask']);last,good=pair;ref_mask=good.mask>0;ref_rgb=rgb.copy();rh,rc=m._appearance(ref_rgb,ref_mask.astype(np.uint8));rows=[]
    def identity(rgb,mask):
        current=mask>0;h,c=m._appearance(rgb,current.astype(np.uint8));iou=float(np.count_nonzero(current&ref_mask)/np.count_nonzero(current|ref_mask));colour=float(np.sqrt(max(0.,1.-np.sum(np.sqrt(h*rh)))));shape=float(cv2.matchShapes(rc,c,cv2.CONTOURS_MATCH_I1,0.));return dict(iou=iou,colour=colour,shape=shape,original_hold_gate_passed=iou>=.85 and colour<=.4 and shape<=.4)
    def output_mask(ns,name,rgb,mask,native=None):
        path=out/f'{ns}_{name}.png';overlay=rgb.copy()
        if native is not None:cv2.drawContours(overlay,cv2.findContours((native>0).astype(np.uint8),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2],-1,(0,0,255),1)
        cv2.drawContours(overlay,cv2.findContours((mask>0).astype(np.uint8),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2],-1,(0,255,0),1);cv2.imwrite(str(path),overlay);return str(path)
    qns=1791854971572257757;qrgb,qdepth,qT,qkw,qdata=frame(qns);qmask=qdata['recorded_mask'];qh,qc=m._appearance(qrgb,m._mask(qmask,qrgb.shape[:2]));qfrozen=m.FrozenObservationAppearance(qrgb,qmask,qkw.get('rgb_valid'),qns,qh,qc)
    for ns in(m.GOOD,m.BAD,m.NEXT):
        rgb,depth,T,kw,data=frame(ns);cv2.imwrite(str(out/f'{ns}_original_rgb.png'),rgb);row=dict(source_ns=ns,reference_source_ns=m.GOOD)
        seg=m.table.segment_with_support_fallback(depth,K,T,cfg,anchor,last,plane,prefer_current_support=True);matches=m.table._matching_candidates(seg.candidates,anchor,last,cfg);row['native_matches']=len(matches);fitter=_FrameRefinement(depth,K,T,spatial,cfg);refinements=[]
        for index,(_,native)in enumerate(matches):
            item=dict(index=index,native_bbox=list(native.bbox),native_position=native.position_base)
            for kind,refine in [('spatial_optional',lambda:fitter.refine(rgb,native,**kw)),('independent_native',lambda:m.refine_candidate(rgb,depth,K,T,native,cfg,observation_arrival_retry=True,**kw))]:
                try:
                    current=refine();idv=identity(rgb,current.mask);audit=match_observation_appearance(rgb,(current.mask>0).astype(np.uint8),hist,contour,rgb_valid=kw.get('rgb_valid'),appearance_context=dict(reference=frozen,source_ns=ns,deadline_monotonic=time.monotonic()+5,check_current=lambda:None))
                    item[kind]=dict(identity_against_good=idv,original_appearance_against_premotion=audit,mask_sha256=__import__('hashlib').sha256(np.ascontiguousarray(current.mask>0,dtype=np.uint8).tobytes()).hexdigest(),rgb_metrics=current.metrics,overlay=output_mask(ns,f'native{index}_{kind}',rgb,current.mask,native.mask))
                    np.savez_compressed(out/f'{ns}_native{index}_{kind}.npz',mask=current.mask,native_mask=native.mask)
                except ValueError as exc:item[kind]=dict(error=str(exc))
            refinements.append(item)
        row['refinements']=refinements
        # Explicit new-lineage candidate, never substituted into baseline rows:
        # only the appearance parent changes to the already measured GOOD.
        # Spatial anchor/plane, native candidates and every original gate stay.
        good_frozen=m.FrozenObservationAppearance(ref_rgb,good.mask,kw.get('rgb_valid'),m.GOOD,rh,rc)
        lineage_audit=[]
        try:
            lineage_seg,lineage_pair=m.recover_arrival_matching_fragments(seg,rgb,depth,K,T,cfg,anchor,last,spatial,rh,rc,.5,appearance_context=dict(reference=good_frozen,source_ns=ns,deadline_monotonic=time.monotonic()+5,check_current=lambda:None,record=lineage_audit.append),**kw)
            row['candidate_registered_good_appearance_lineage']=dict(passed=lineage_pair is not None,source_parent_ns=m.GOOD,current_source_ns=ns,original_premotion_spatial_anchor_preserved=True,existing_thresholds_unchanged=True,appearance_audits=lineage_audit)
            if lineage_pair is not None:
                raw_candidate,current=lineage_pair
                row['candidate_registered_good_appearance_lineage'].update(mask_sha256=__import__('hashlib').sha256(np.ascontiguousarray(current.mask>0,dtype=np.uint8).tobytes()).hexdigest(),position_base=current.position_base,extent_m=current.extent_m,quality=current.quality,metrics=current.metrics,overlay=output_mask(ns,'candidate_registered_good_lineage',rgb,current.mask),refined_is_existing_independent_mask=any(item.get('spatial_optional',{}).get('mask_sha256')==__import__('hashlib').sha256(np.ascontiguousarray(current.mask>0,dtype=np.uint8).tobytes()).hexdigest()for item in refinements))
                np.savez_compressed(out/f'{ns}_candidate_registered_good_lineage.npz',mask=current.mask,raw_mask=raw_candidate.mask,position_base=current.position_base,extent_m=current.extent_m)
        except ValueError as exc:
            row['candidate_registered_good_appearance_lineage']=dict(passed=False,reason=str(exc),appearance_audits=lineage_audit)
        narrow=[]
        for name,alt,altmask,ah,ac,final_origin in [('native_good_final_origin',good_frozen,good.mask,rh,rc,True),('all_frozen_qualified',qfrozen,qmask,qh,qc,False),('native_frozen_qualified_final_origin',qfrozen,qmask,qh,qc,True)]:
            calls=[0];audits=[];overlap_rows=[]
            def appearance(rgb,mask,histogram,outline,*,rgb_valid=None,appearance_context=None):
                calls[0]+=1
                use_parent=not final_origin or calls[0]<=len(matches)
                if use_parent:
                    iou=float(np.count_nonzero((mask>0)&(altmask>0))/np.count_nonzero((mask>0)|(altmask>0)));overlap_rows.append(dict(call=calls[0],iou=iou))
                    if final_origin and iou<.85:raise ValueError('diagnostic_native_station_iou_below_original_hold_gate')
                    context=dict(appearance_context or {},reference=alt,record=audits.append)
                    return match_observation_appearance(rgb,mask,ah,ac,rgb_valid=rgb_valid,appearance_context=context)
                return match_observation_appearance(rgb,mask,histogram,outline,rgb_valid=rgb_valid,appearance_context=dict(appearance_context or {},record=audits.append))
            try:
                with patch.object(fragment_module,'match_observation_appearance',appearance):
                    _,candidate=m.recover_arrival_matching_fragments(seg,rgb,depth,K,T,cfg,anchor,last,spatial,hist,contour,.5,appearance_context=dict(reference=frozen,source_ns=ns,deadline_monotonic=time.monotonic()+5,check_current=lambda:None),**kw)
                detail=dict(name=name,passed=candidate is not None,origin_final_preserved=final_origin,station_reference_ns=alt.source_ns,appearance_calls=calls[0],overlap_rows=overlap_rows,appearance_audits=audits)
                if candidate is not None:
                    c=candidate[1];detail.update(mask_sha256=__import__('hashlib').sha256(np.ascontiguousarray(c.mask>0,dtype=np.uint8).tobytes()).hexdigest(),metrics=c.metrics,overlay=output_mask(ns,name,rgb,c.mask))
            except ValueError as exc:detail=dict(name=name,passed=False,reason=str(exc),origin_final_preserved=final_origin,station_reference_ns=alt.source_ns,appearance_calls=calls[0],overlap_rows=overlap_rows,appearance_audits=audits)
            narrow.append(detail)
        row['narrow_origin_preserving_and_frozen_qualified_diagnostics']=narrow
        x,y,w,h=cv2.boundingRect(ref_mask.astype(np.uint8));policy=CenterSearchPolicy(((x+.5*w)/rgb.shape[1],(y+.5*h)/rgb.shape[0]),radius_fraction=cfg.center_radius_fraction)
        proposals=propose_rgb_observation_regions(rgb,policy=policy,**kw);current=[]
        for index,proposal in enumerate(proposals.candidates):
            current.append(dict(index=index,bbox=list(proposal.bbox_xywh),**identity(rgb,proposal.mask),overlay=output_mask(ns,f'independent_rgb{index}',rgb,proposal.mask)))
        matches_rgb=sorted((.65*x['colour']/.4+.35*x['shape']/.4,x['index'])for x in current if x['original_hold_gate_passed'])
        row['independent_rgb']=dict(reason=proposals.reason,total_candidates=proposals.total_candidate_count,rows=current,original_unique_hold_predicate_passed=bool(matches_rgb and(len(matches_rgb)==1 or matches_rgb[1][0]-matches_rgb[0][0]>HandoffPolicy().ambiguity_margin)))
        rows.append(row);print(json.dumps(dict(source_ns=ns,independent_rgb_summary={k:v for k,v in row['independent_rgb'].items()if k!='rows'},native=[{kind:item.get(kind,{}).get('identity_against_good')for kind in('spatial_optional','independent_native')}for item in refinements])),flush=True)
    report=dict(scope='original raw RGB/geometry diagnostic; no old lease refresh or motion authority',rows=rows,original_fixture=str(F),source_sha256={str(Path(__file__)):m.sha(Path(__file__))},visual_legend='green=current original algorithm outline; red=full native measured depth seed; untouched original RGB saved separately',historical_source_replaced=False,source_restamped=False,motion_authorized=False)
    (out/'result.json').write_text(json.dumps(m.plain(report),ensure_ascii=False,indent=2)+'\n');print('RESULT '+str(out/'result.json'))
if __name__=='__main__':main()
