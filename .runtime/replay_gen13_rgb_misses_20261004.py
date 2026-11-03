"""Replay exact gen13 RGB failures against the immutable original reference."""
from collections import Counter
from dataclasses import asdict, fields
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np

from alicia_grasp_modes import tabletop
from alicia_grasp_modes.rgb_observation_handoff import (
    HandoffPolicy, RgbHandoffFrame, _appearance, associate_rgb_interest, freeze_rgb_interest)
from alicia_grasp_modes.rgb_observation_region import CenterSearchPolicy, propose_rgb_observation_regions
from alicia_grasp_modes.rgb_observation_projection import projected_rgb_search_policy
from alicia_grasp_modes.rgb_observation_source import measured_support_coverage
from alicia_grasp_modes.rgb_observation_volume import RGBObservationVolume, SupportPlaneProvenance

BASE = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log')
ARCHIVE = BASE/'gen13_rgb_support_exact_20261004T122355Z/report.json'
REFERENCE = BASE/'rgb_acquisition_gateway_20261004T114259.948391Z_1541ec9e/result.json'
RUN = BASE/'rgb_acquisition_run_20261004T114249Z/result.json'
OUT = ARCHIVE.parent/'rgb_replay.json'


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def camera_matrix(K):return np.array([[K[0],0.,K[2]],[0.,K[1],K[3]],[0.,0.,1.]])


def original_reference():
    archived=json.loads(REFERENCE.read_text());meta=archived['frames']['initial']
    context=meta['source_context'];interest=meta['interest'];raw=np.load(meta['source_file'])
    volume_raw=dict(meta['volume'])
    volume_raw['support_provenance']=SupportPlaneProvenance(**volume_raw['support_provenance'])
    volume=RGBObservationVolume(**{f.name:volume_raw[f.name] for f in fields(RGBObservationVolume) if f.init})
    depth=raw['depth_raw'].astype(np.float32)*float(raw['depth_scale'])
    source=int(raw['source_stamp_ns'])
    frame=RgbHandoffFrame(raw['color_bgr'],depth,camera_matrix(raw['K']),raw['T_base_camera'],
        volume.support_provenance,meta['source_id'],context['selection']['generation'],context['epoch_ns'],
        context['selection']['stamp_ns'],source,source,source+200_000_000,
        tuple((int(meta[k][0]),tuple(meta[k][1])) for k in ('accepted_before','accepted_after')))
    policy=CenterSearchPolicy(**interest['rgb_search_policy'])
    proposed=propose_rgb_observation_regions(raw['color_bgr'],policy=policy)
    matching=[c for c in proposed.candidates if np.array_equal(c.mask,raw['region_mask'])]
    assert len(matching)==1
    reference=freeze_rgb_interest(frame,matching[0],volume,
        support_ring_outer_radius_px=interest['support_metrics']['support_ring_outer_radius_px'])
    return reference,context,policy


def main():
    cv2.setNumThreads(1)
    evidence=json.loads(ARCHIVE.read_text());run=json.loads(RUN.read_text())
    reference,context,base_policy=original_reference()
    config=tabletop.Config(**{f.name:context['perception'][f.name] for f in fields(tabletop.Config) if f.name in context['perception']})
    policy=HandoffPolicy()
    records=[]
    for row in evidence['raw_frames']:
        assert sha(row['file'])==row['sha256']
        raw=np.load(row['file']);source=row['source_stamp_ns'];status=row['status']['message']
        recorded=status['rgb_acquisition_handoff'];observation=recorded['observation_index']
        rgb=raw['color_bgr'];depth=raw['depth_raw'].astype(np.float32)*float(raw['depth_scale'])
        T=raw['T_base_camera'];K=raw['K'];seg=tabletop.segment(depth,K,T,config)
        coverage=measured_support_coverage(depth,K,T,seg.plane_base,config,seg.metrics['support_ring_outer_radius_px'])
        metrics=seg.metrics
        provenance=SupportPlaneProvenance(source,tuple(seg.plane_base),
            'rgb_handoff:%s:%d'%(evidence['reference_id'],source),
            int(round(metrics['support_sample_count']*metrics['support_inlier_ratio'])),
            metrics['support_inlier_ratio'],metrics['support_residual_m'],coverage['spatial_quadrants'])
        frame=RgbHandoffFrame(rgb,depth,camera_matrix(K),T,provenance,reference.source_id,
            reference.generation,reference.epoch_ns,reference.selection_stamp_ns,source,source,
            status.get('frame_timing',{}).get('status_build_ros_ns',row['status']['bag_arrival_ns']),
            tuple((int(row[k][0]),tuple(row[k][1])) for k in ('accepted_before','accepted_after')))
        audit=associate_rgb_interest(reference,frame,(),observation_index=observation)
        result=dict(source_stamp_ns=source,tags=row['tags'],observation_index=observation,
            recorded_reason=recorded['reason'],support_audit=dict(audit.support_audit),
            source_npz=row['file'],source_sha256=row['sha256'],q_source=row['q_source'])
        nearby=[a['arm_position'] for a in evidence['accepted_feedback'] if a['arm_position'] is not None
            and source-300_000_000<=a['source_stamp_ns']<=source+100_000_000]
        result['local_sdk_joint_span_rad']=(np.ptp(np.asarray(nearby),axis=0).tolist() if nearby else None)
        settled=int(run['requests'][observation-1]['ledger']['settled_ns'])
        result['source_after_settled_ms']=(source-settled)*1e-6
        if audit.reason!='no_cross_view_rgb_match':
            result['replayed_reason']=audit.reason
            records.append(result);continue
        projected=np.asarray(audit.projected_volume_pixels)
        expected=np.asarray(recorded.get('projected_volume_pixels',[]))
        result['projection_max_abs_difference_px']=(float(np.max(abs(projected-expected))) if expected.shape==projected.shape else None)
        search=projected_rgb_search_policy(base_policy,projected,depth.shape)
        proposals=propose_rgb_observation_regions(rgb,policy=search,
            depth_valid=np.isfinite(depth)&(depth>=config.depth_min_m)&(depth<=config.depth_max_m))
        association=associate_rgb_interest(reference,frame,proposals,seg.candidates,observation_index=observation)
        hull=cv2.convexHull(projected.astype(np.float32)).reshape(-1,2)
        projected_mask=np.zeros(depth.shape,np.uint8)
        cv2.fillConvexPoly(projected_mask,np.rint(hull).astype(np.int32),1)
        candidates=[]
        for i,candidate in enumerate(proposals.candidates):
            mask=candidate.mask.astype(np.uint8)
            hist,contour=_appearance(rgb,mask)
            color=float(np.sqrt(max(0.,1.-np.sum(np.sqrt(hist*reference.histogram)))))
            shape=float(cv2.matchShapes(reference.contour,contour,cv2.CONTOURS_MATCH_I1,0.))
            inside=int(np.count_nonzero(mask&projected_mask));total=int(np.count_nonzero(mask))
            candidates.append(dict(index=i,bbox_xywh=list(candidate.bbox_xywh),mask_pixels=total,
                mask_sha256=hashlib.sha256(mask.tobytes()).hexdigest(),projected_inside_pixels=inside,
                projected_fraction=inside/total,colour_distance=color,contour_distance=shape,
                projection_passes=inside/total>=policy.min_projected_fraction,
                colour_passes=color<=policy.max_colour_distance,shape_passes=shape<=policy.max_contour_distance,
                score=.65*color/policy.max_colour_distance+.35*shape/policy.max_contour_distance,
                channels=list(candidate.channels),valid_depth_fraction=candidate.valid_depth_fraction))
        result.update(replayed_reason=association.reason,proposal_reason=proposals.reason,
            proposal_count=len(candidates),proposal_total_count=proposals.total_candidate_count,
            proposal_evaluated_regions=proposals.evaluated_regions,
            search_policy=asdict(search),projected_volume_pixels=projected.tolist(),
            proposal_candidates=candidates,associated_index=association.rgb_region_index,
            measured_candidate_index=association.measured_candidate_index,
            alternatives=list(association.alternatives))
        records.append(result)
    result=dict(scope=__doc__,source_report=str(ARCHIVE),source_report_sha256=sha(ARCHIVE),
        reference_report=str(REFERENCE),reference_report_sha256=sha(REFERENCE),
        source_context=context,config=asdict(config),policy=asdict(policy),
        reference_mask_fingerprint=reference.mask_fingerprint,
        original_measured_support_footprint_base=reference.measured_support_footprint_base.tolist(),
        script_sha256=sha(__file__),module_hashes={name:sha(__import__('alicia_grasp_modes.'+name,fromlist=['x']).__file__)
            for name in ('rgb_observation_region','rgb_observation_handoff','rgb_observation_projection','tabletop')},
        records=records,ros_io=False,production_changes=False,robot_commands_sent=False,
        authority='offline reconstruction only; no live reference or measured-geometry permission')
    OUT.write_text(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    failures=[r for r in records if r['recorded_reason']=='no_cross_view_rgb_match']
    print(json.dumps(dict(output=str(OUT),sha256=sha(OUT),rows=len(records),rgb_failures=len(failures),
        replayed_rgb_failure_reasons=dict(Counter(r['replayed_reason'] for r in failures)),
        projection_max_abs_difference_px=max(r.get('projection_max_abs_difference_px') or 0 for r in records),
        failure_candidate_digest=[dict(stamp=r['source_stamp_ns'],obs=r['observation_index'],
            reason=r['replayed_reason'],proposal_reason=r.get('proposal_reason'),
            appearance_matches=[c for c in r.get('proposal_candidates',[]) if c['colour_passes'] and c['shape_passes']]) for r in failures]),
        ensure_ascii=False,indent=2),flush=True)


if __name__=='__main__':main()
