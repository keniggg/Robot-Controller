#!/usr/bin/env python3
"""Offline exact-source gate diagnosis; no ROS, device access, or parameter writes."""
import argparse
from collections import Counter
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import sys

import cv2
import numpy as np

WORK=Path(__file__).resolve().parent.parent
for path in (WORK/'src/alicia_grasp_modes/src',WORK/'src/alicia_flexible_grasp_supervisor/src'):
    sys.path.insert(0,str(path))
from alicia_grasp_modes import tabletop,rgb_instance,measured_mask_seeds,acquisition
from alicia_grasp_modes.rgb_observation_region import CenterSearchPolicy,propose_rgb_observation_regions
from alicia_grasp_modes.rgb_observation_source import bracket_feedback
from alicia_grasp_modes.rgb_observation_handoff import _appearance,HandoffPolicy
from alicia_flexible_grasp.vision.rgbd_rectification import rectification_rgb_valid_mask,require_rectified_source_after_activation

BASE=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/rgb_observation_bootstrap_20261004T141247Z')


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def arrsha(a):return hashlib.sha256(np.ascontiguousarray(a).tobytes()).hexdigest()
def percentiles(a):
    return dict(zip(('min','p10','p50','p90','max'),np.percentile(a,[0,10,50,90,100]).tolist())) if len(a) else None

def refine_with_seed_trace(rgb,depth,K,T,candidate,cfg,valid_rgb):
    """Observe actual production seed locals without altering its operations."""
    traces=[];old=sys.gettrace()
    def trace(frame,event,arg):
        if frame.f_code is measured_mask_seeds.confidence_labels.__code__ and event in ('exception','return'):
            local=frame.f_locals;item={'event':event}
            for key in ('measured','core','foreground','top','side','reliable','background'):
                if key in local:item[key+'_pixels']=int(np.count_nonzero(local[key]))
            for key in ('upper_height','upper_surface_band_used','original_top_pixels','separated'):
                if key in local:
                    value=local[key];item[key]=value.item() if isinstance(value,np.generic) else value
            if event=='exception':item['exception']=str(arg[1])
            traces.append(item)
        return trace
    try:
        sys.settrace(trace)
        result=rgb_instance.refine_candidate(rgb,depth,K,T,candidate,cfg,rgb_valid=valid_rgb)
        return result,dict(passed=True,trusted_seed_execution=traces,metrics=result.metrics)
    except ValueError as exc:
        return None,dict(passed=False,reason=str(exc),trusted_seed_execution=traces)
    finally:sys.settrace(old)


def main():
    args=argparse.ArgumentParser(description=__doc__)
    args.add_argument('--output',type=Path,default=BASE/'measurement_gate_diagnostic.json')
    options=args.parse_args()
    cv2.setNumThreads(1)
    source=BASE/'result.json';original=json.loads(source.read_text());records=original['rows']
    policy=CenterSearchPolicy(center_xy=(.5,.5),radius_fraction=.125)
    prepared=[]
    for row in records:
        assert sha(row['source_file'])==row['source_file_sha256']
        with np.load(row['source_file'],allow_pickle=False) as z:raw={k:z[k] for k in z.files}
        ns=int(raw['source_stamp_ns']);assert ns==row['source_stamp_ns']
        rgb,z16=raw['color_bgr'],raw['depth_raw']
        assert arrsha(rgb)==row['source_rgb_sha256'] and arrsha(z16)==row['source_depth_sha256']
        assert np.array_equal(raw['q_source'],row['q_source'])
        assert np.array_equal(raw['T_base_camera'],row['T_base_camera'])
        K=raw['K'];cam=row['source_context']['camera']
        assert np.array_equal(K,[cam[k] for k in ('fx','fy','cx','cy')])
        require_rectified_source_after_activation(cam,ns)
        rgb_valid=rectification_rgb_valid_mask(cam,z16.shape)
        assert rgb_valid is not None and rgb.shape[:2]==z16.shape==(cam['height'],cam['width'])
        feedback=bracket_feedback((row['accepted_before'],row['accepted_after']),ns,row['epoch_ns'])
        assert np.array_equal(np.asarray(feedback[0]),raw['q_source'])
        cfg=tabletop.Config(**row['config']);depth=z16.astype(np.float32)*float(raw['depth_scale'])
        valid=np.isfinite(depth)&(depth>=cfg.depth_min_m)&(depth<=cfg.depth_max_m)
        proposals=propose_rgb_observation_regions(rgb,policy=policy,depth_valid=valid,rgb_valid=rgb_valid)
        assert [(list(c.bbox_xywh),c.area_px) for c in proposals.candidates]==[(c['bbox_xywh'],c['area_px']) for c in row['rgb_candidates']]
        prepared.append((row,raw,rgb,depth,valid,rgb_valid,cfg,proposals))
    # Reference is an actual generic candidate from an archived confirmed RGB
    # interest. No hand-selected colour, physical dimensions or mask is added.
    reference=prepared[2];ref_region=reference[-1].selected
    assert reference[0]['observation_interest_ready'] and ref_region is not None
    assert list(ref_region.bbox_xywh)==reference[0]['interest_volume']['bbox_xywh']
    ref_hist,ref_contour=_appearance(reference[2],ref_region.mask.astype(np.uint8))
    gates=HandoffPolicy()
    report=dict(schema=1,scope=__doc__,ros_io=False,device_access=False,motion_authorized=False,
        source_result=str(source),source_result_sha256=sha(source),script_sha256=sha(__file__),
        reference=dict(row_index=2,source_stamp_ns=reference[0]['source_stamp_ns'],
            region_bbox=list(ref_region.bbox_xywh),mask_sha256=arrsha(ref_region.mask),
            selection='generic recorded RGB-only interest; offline appearance association, not renewed object authority'),
        policy=asdict(policy),config=asdict(reference[6]),original_handoff_gates=asdict(gates),
        implementation_sha256={str(Path(m.__file__).resolve()):sha(m.__file__) for m in
            (tabletop,rgb_instance,measured_mask_seeds,acquisition)},
        limitations=[
            'Archived native uint16 is published aligned/filter-processed and centrally rectified depth, not sensor pre-align matcher output.',
            'Attached detector status may refer to another source. It is reported separately and never assigned as the exact saved-frame outcome.',
            'Initial depth selection/refinement is replayed independently per exact frame. Eight sparse captures do not reproduce all online acquisition history or task selection freshness.',
            'Target mask is a generic same-reference appearance hypothesis. It does not provide metric geometry or revive the expired gen13 selection.',
            'Image connected-component sizes are a diagnostic topology statistic, not replacement for the production depth-edge split.',
            'Saved q/TF/source are retained unchanged. No source clock, camera pose, missing pixel or surface is synthesized.'
        ],rows=[])
    sampled_acquisition=acquisition.InitialAcquisition(reference[6])
    for index,(row,raw,rgb,depth,valid,valid_rgb,cfg,proposals) in enumerate(prepared):
        ns=row['source_stamp_ns'];K=raw['K'];T=raw['T_base_camera']
        result=dict(index=index,source_stamp_ns=ns,source_file_sha256=row['source_file_sha256'],
                    q_source=row['q_source'],accepted_span_ns=row['accepted_after'][0]-row['accepted_before'][0],
                    detector_source_stamp_ns=row.get('detector',{}).get('source_stamp_ns'),
                    detector_detail_not_exact=row.get('detector',{}).get('detail'),
                    detector_is_exact_source=row.get('detector',{}).get('source_stamp_ns')==ns,
                    recorded_interest_ready=row['observation_interest_ready'],
                    recorded_rgb_track_reason=row['rgb_track']['reason'])
        report['rows'].append(result)
        associated=[];proposals_report=[]
        for j,c in enumerate(proposals.candidates):
            hist,contour=_appearance(rgb,c.mask.astype(np.uint8))
            colour=float(np.sqrt(max(0.,1.-np.sum(np.sqrt(hist*ref_hist)))))
            shape=float(cv2.matchShapes(ref_contour,contour,cv2.CONTOURS_MATCH_I1,0.))
            passed=colour<=gates.max_colour_distance and shape<=gates.max_contour_distance
            proposals_report.append(dict(index=j,bbox=list(c.bbox_xywh),pixels=int(c.area_px),colour_distance=colour,contour_distance=shape,appearance_pass=passed))
            if passed:associated.append(j)
        result['rgb_proposals']=proposals_report
        if len(associated)!=1:
            result['target_diagnostic_error']='appearance hypothesis not unique';continue
        region=proposals.candidates[associated[0]];mask=region.mask>0
        result['associated_rgb_index']=associated[0]
        fitted=tabletop.segment(depth,K,T,cfg)
        assert all((abs(v-row['support_metrics'][k]) <= 1e-12 if isinstance(v,(int,float))
                    else v==row['support_metrics'][k]) for k,v in fitted.metrics.items())
        result['support_metrics']=fitted.metrics;result['support_plane_base']=fitted.plane_base.tolist()
        points=tabletop._project(depth,K);plane_camera=T.T@fitted.plane_base
        heights=np.einsum('ijk,k->ij',points,plane_camera[:3])+plane_camera[3]
        yy,xx=np.indices(depth.shape);radius=np.hypot(xx-depth.shape[1]/2,yy-depth.shape[0]/2)
        roi=radius<=155*min(depth.shape[1]/640.,depth.shape[0]/480.)
        raw_fg=valid&roi&(heights>=cfg.foreground_min_m)&(heights<=cfg.foreground_max_m)
        opened=cv2.morphologyEx(raw_fg.astype(np.uint8),cv2.MORPH_OPEN,np.ones((3,3),np.uint8))>0
        opened &= valid&(heights>=cfg.foreground_min_m)
        _,components=cv2.connectedComponents((opened&mask).astype(np.uint8),connectivity=8)
        areas=np.bincount(components.ravel())[1:]
        observed=mask&valid
        result['target_depth']=dict(mask_pixels=int(mask.sum()),valid_depth_pixels=int(observed.sum()),
            valid_depth_fraction=float(observed.sum()/mask.sum()),unobserved_rgb_pixels=int(np.count_nonzero(mask&~valid_rgb)),
            roi_fraction=float(np.count_nonzero(mask&roi)/mask.sum()),
            raw_foreground_4_to_120mm_pixels=int(np.count_nonzero(mask&raw_fg)),
            after_original_3x3_open_pixels=int(np.count_nonzero(mask&opened)),
            image_connected_component_sizes_after_open=sorted(areas.tolist(),reverse=True),
            optical_z_mm=percentiles(depth[observed]*1000),height_to_current_support_mm=percentiles(heights[observed]*1000))
        chosen,reason=tabletop.TargetTracker(cfg).choose(fitted,depth.shape)
        chosen_index=next((j for j,c in enumerate(fitted.candidates) if c is chosen),None)
        result.update(actual_depth_candidate_count=len(fitted.candidates),initial_choice_reason=reason,
                      initial_chosen_index=chosen_index,candidates=[])
        selected_refined=None
        for j,c in enumerate(fitted.candidates):
            cm=c.mask>0
            iou=float(np.count_nonzero(mask&cm)/np.count_nonzero(mask|cm))
            candidate=dict(index=j,bbox=list(c.bbox),quality=c.quality,quality_pass=c.quality>=.5,
                mask_pixels=int(cm.sum()),target_intersection_pixels=int(np.count_nonzero(mask&cm)),
                target_mask_iou=iou,handoff_measured_iou_pass=iou>=gates.minimum_measured_mask_iou,
                metrics=c.metrics,initial_selected=j==chosen_index)
            if candidate['quality_pass']:
                refined,trace=refine_with_seed_trace(rgb,depth,K,T,c,cfg,valid_rgb)
                candidate['refinement']=trace
                if refined is not None:
                    rm=refined.mask>0;candidate['refined_target_mask_iou']=float(np.count_nonzero(mask&rm)/np.count_nonzero(mask|rm))
                if j==chosen_index:selected_refined=refined
            result['candidates'].append(candidate)
        if chosen is None:
            initial_result=reason
        elif chosen.quality<.5:
            initial_result='insufficient_geometry_quality'
        elif selected_refined is None:
            initial_result='initial_instance_'+result['candidates'][chosen_index]['refinement']['reason']
        else:initial_result='single_frame_initial_refinement_passed'
        result['initial_exact_frame_outcome']=initial_result
        if selected_refined is None:
            sampled_acquisition.reset();result['sampled_acquisition_ready']=False
        else:
            ready,detail=sampled_acquisition.observe(chosen,selected_refined,ns)
            result['sampled_acquisition_ready']=ready;result['sampled_acquisition_reason']=detail
        result['target_best_measured_candidate_iou']=max((v['target_mask_iou'] for v in result['candidates']),default=0.)
    measured=[r for r in report['rows'] if 'target_depth' in r]
    report['summary']=dict(frames=len(records),appearance_unique_frames=len(measured),
        recorded_rgb_interest_ready=sum(r['observation_interest_ready'] for r in records),
        exact_initial_outcomes=dict(Counter(r.get('initial_exact_frame_outcome','not_reached') for r in report['rows'])),
        raw_foreground_counts=[r['target_depth']['raw_foreground_4_to_120mm_pixels'] for r in measured],
        opened_target_counts=[r['target_depth']['after_original_3x3_open_pixels'] for r in measured],
        valid_depth_fraction=[r['target_depth']['valid_depth_fraction'] for r in measured],
        maximum_actual_candidate_target_iou=max((r['target_best_measured_candidate_iou'] for r in measured),default=0.),
        any_sampled_sequence_initial_ready=any(r.get('sampled_acquisition_ready') for r in report['rows']),
        attached_detector_exact_source_count=sum(r['detector_is_exact_source'] for r in report['rows']))
    if not Path('/media/zhuyupei/SanStand').is_mount():raise RuntimeError('SanStand unavailable; no fallback')
    payload=(json.dumps(report,ensure_ascii=False,allow_nan=False,indent=2)+'\n').encode()
    with options.output.open('xb') as stream:stream.write(payload)
    print(json.dumps(dict(output=str(options.output),sha256=sha(options.output),summary=report['summary']),ensure_ascii=False,indent=2))


if __name__=='__main__':main()
