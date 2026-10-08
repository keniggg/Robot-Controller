"""Read native trial4 RGB-D only; no ROS, threshold edits or geometry authority."""
from dataclasses import fields
from pathlib import Path
from collections import Counter
import hashlib
import json
import os
import sys

import cv2
import numpy as np

from alicia_grasp_modes.tabletop import Config, _project, segment, project_base_to_pixel
from alicia_grasp_modes.rgb_instance import refine_candidate
from alicia_grasp_modes.measured_mask_seeds import local_surface_normals, confidence_labels
from alicia_grasp_modes.rgb_observation_region import CenterSearchPolicy, propose_rgb_observation_regions
from alicia_grasp_modes.rgb_observation_volume import SupportPlaneProvenance, build_rgb_observation_volume, project_rgb_observation_volume
from alicia_grasp_modes.rgb_observation_handoff import RgbHandoffFrame, freeze_rgb_interest, _raw_support_audit, _appearance
from alicia_grasp_modes.rgb_observation_source import measured_support_coverage
from alicia_flexible_grasp.vision.object_geometry import _voxel_centroids, _neighbor_distance_filter

ROOT=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log')
BEFORE=ROOT/'rgb_acquisition_gateway_20261004T044244.594925Z_cd1dada6/result.json'
AFTER=ROOT/'rgb_observation_bootstrap_20261004T044428Z/result.json'


def sha(value):return hashlib.sha256(value).hexdigest()
def span(values):return [float(min(values)),float(np.median(values)),float(max(values))] if values else None
def quant(values):
    return dict(count=len(values),p05_p25_p50_p75_p90_p95_mm=np.percentile(values*1000.,[5,25,50,75,90,95]).tolist()) if len(values) else None


def voxel(points):
    if not len(points):return dict(native=0,voxel=0,cleaned=0,at_least_120=False)
    vs=_voxel_centroids(points,.0015)
    cleaned=_neighbor_distance_filter(vs,16,2.,cell_size_m=.0015) if len(vs) else vs
    return dict(native=len(points),voxel=len(vs),cleaned=len(cleaned),at_least_120=len(cleaned)>=120)


def load(record):
    path=Path(record['source_file'])
    assert sha(path.read_bytes())==record['source_file_sha256']
    raw=np.load(path,allow_pickle=False)
    rgb,native=raw['color_bgr'],raw['depth_raw']
    assert sha(rgb.tobytes())==record['source_rgb_sha256']
    assert sha(native.tobytes())==record['source_depth_sha256']
    intr=np.asarray(raw['K'])
    if intr.shape==(3,3):intr=np.array([intr[0,0],intr[1,1],intr[0,2],intr[1,2]])
    K=np.array([[intr[0],0.,intr[2]],[0.,intr[1],intr[3]],[0.,0.,1.]])
    return raw,rgb,native.astype(np.float32)*float(raw['depth_scale']),intr,K,raw['T_base_camera']


def make_frame(record,rgb,depth,K,T,seg,cfg):
    ns=int(record['source_stamp_ns']); context=record['source_context']; selection=context['selection']
    coverage=measured_support_coverage(depth,(K[0,0],K[1,1],K[0,2],K[1,2]),T,seg.plane_base,cfg,
        seg.metrics['support_ring_outer_radius_px'])
    m=seg.metrics
    p=SupportPlaneProvenance(ns,tuple(seg.plane_base),'trial4_raw_diagnostic:%d'%ns,
        int(round(m['support_sample_count']*m['support_inlier_ratio'])),m['support_inlier_ratio'],
        m['support_residual_m'],coverage['spatial_quadrants'])
    age=record.get('source_age_at_end_sec',record.get('interest',{}).get('source_age_at_status_sec',.1))
    return RgbHandoffFrame(rgb,depth,K,T,p,record['source_id'],selection['generation'],
        context['epoch_ns'],selection['stamp_ns'],ns,ns,ns+int(round(age*1e9)),
        tuple(record[k] for k in ('accepted_before','accepted_after')))


def main():
    cv2.setNumThreads(1)
    before=json.loads(BEFORE.read_text());after=json.loads(AFTER.read_text())
    refs=[]; outputs=[]; pools={'before':{},'after':{}}
    records=[('before',name,r) for name,r in before['frames'].items()]+[
        ('after',str(i),r) for i,r in enumerate(after['rows'])]
    for stage,label,r in records:
        raw,rgb,depth,intr,K,T=load(r);cfg=Config(**r.get('config',{}))
        points=_project(depth,intr);base=points@T[:3,:3].T+T[:3,3]
        seg=segment(depth,intr,T,cfg)
        plane_camera=T.T@seg.plane_base
        height=np.einsum('ijk,k->ij',points,plane_camera[:3])+plane_camera[3]
        valid=np.isfinite(depth)&(depth>=cfg.depth_min_m)&(depth<=cfg.depth_max_m)
        frame=make_frame(r,rgb,depth,K,T,seg,cfg)
        if not refs:
            policy=CenterSearchPolicy(**r['interest']['rgb_search_policy'])
            proposals=propose_rgb_observation_regions(rgb,policy=policy,depth_valid=valid)
            for member in r['interest']['rgb_interest_members']:
                regions=[p for p in proposals.candidates if sha(p.mask.tobytes())==member['mask_sha256']]
                assert len(regions)==1
                region=regions[0]
                volume=build_rgb_observation_volume(bbox_xywh=region.bbox_xywh,image_shape=depth.shape,
                    camera_matrix=K,T_base_camera=T,support_plane_base=seg.plane_base,
                    support_provenance=frame.support_provenance,foreground_max_m=cfg.foreground_max_m,
                    source_stamp_ns=frame.source_stamp_ns,working_depth_range_m=(.07,.5))
                refs.append(freeze_rgb_interest(frame,region,volume,
                    support_ring_outer_radius_px=seg.metrics['support_ring_outer_radius_px']))
        # Match the producer's frozen-envelope projection search, not current centre rank.
        projected=np.concatenate([project_rgb_observation_volume(ref.volume,T_base_camera=T,
            camera_matrix=K,image_shape=depth.shape,working_depth_range_m=ref.volume.working_depth_range_m)
            for ref in refs])
        midpoint=(projected.min(axis=0)+projected.max(axis=0))/2.
        radius=float(np.max(np.linalg.norm(projected-midpoint,axis=1)))/min(depth.shape)
        policy=(CenterSearchPolicy(**r['interest']['rgb_search_policy']) if stage=='before' else
            CenterSearchPolicy((float(midpoint[0]/depth.shape[1]),float(midpoint[1]/depth.shape[0])),
                radius_fraction=min(1.,max(radius,.01))))
        proposals=propose_rgb_observation_regions(rgb,policy=policy,depth_valid=valid)
        fg=valid&(height>=cfg.foreground_min_m)&(height<=cfg.foreground_max_m)
        opened=cv2.morphologyEx(fg.astype(np.uint8),cv2.MORPH_OPEN,np.ones((3,3),np.uint8)).astype(bool)&fg
        normals,reliable=local_surface_normals(points,depth,valid)
        cosine=np.abs(np.einsum('ijk,k->ij',normals,plane_camera[:3]/np.linalg.norm(plane_camera[:3])))
        row=dict(stage=stage,label=label,source_stamp_ns=int(r['source_stamp_ns']),
            source_file=r['source_file'],source_file_sha256=r['source_file_sha256'],
            source_id=r['source_id'],q_source=list(raw['q_source']),support_metrics=seg.metrics,
            support_plane_base=seg.plane_base.tolist(),support_reference_fixed=True,
            support_alignment_applied=False,candidates=[],segmented_candidates=[],reference_diagnostics=[])
        for region in proposals.candidates:
            measured=region.mask&valid
            hc=height[measured]
            row['candidates'].append(dict(bbox_xywh=region.bbox_xywh,centroid_xy=region.centroid_xy,
                mask_sha256=sha(region.mask.tobytes()),area_px=region.area_px,
                valid_points=int(measured.sum()),valid_fraction=float(measured.sum()/region.area_px),
                optical_depth_m=float(np.median(depth[measured])) if np.any(measured) else None,
                actual_height=quant(hc),height_ge_original_4mm=int(np.count_nonzero(measured&fg)),
                opened_height_ge_original_4mm=int(np.count_nonzero(region.mask&opened)),
                native_valid_voxel_diagnostic=voxel(base[measured]),
                original_foreground_voxel=voxel(base[measured&fg])))
        for candidate in seg.candidates:
            measured=(candidate.mask!=0)&fg
            core=cv2.erode(measured.astype(np.uint8),np.ones((3,3),np.uint8)).astype(bool)
            upper=float(np.percentile(height[measured],90))
            top=core&(height>=upper);side=core&reliable&(cosine<=np.sqrt(.5))
            item=dict(bbox_xywh=candidate.bbox,valid_points=candidate.metrics['valid_points'],
                quality=candidate.quality,extent_m=candidate.extent_m.tolist(),
                position_base=candidate.position_base.tolist(),top_pixels=int(top.sum()),
                side_pixels=int(side.sum()),pre_colour_core_pixels=int(np.count_nonzero(top|side)),
                upper_seed_height_mm=upper*1000.,actual_height=quant(height[measured]),
                foreground_fraction_in_rgb_regions=[float(np.count_nonzero(measured&region.mask)/np.count_nonzero(measured))
                    for region in proposals.candidates])
            try:
                _,_,evidence=confidence_labels(candidate.mask!=0,points,depth,height,plane_camera[:3],cfg,bgr=rgb)
                item['seed_evidence']=evidence
            except ValueError as exc:item['seed_error']=str(exc)
            try:
                refined=refine_candidate(rgb,depth,intr,T,candidate,cfg)
                item['refined_valid_points']=refined.metrics['valid_points']
                item['refined_bbox_xywh']=refined.bbox
                item['refined_original_foreground_voxel']=voxel(base[(refined.mask!=0)&fg])
            except ValueError as exc:item['refine_error']=str(exc)
            row['segmented_candidates'].append(item)
        for member,ref in enumerate(refs):
            audit,error=_raw_support_audit(ref,frame)
            pix=np.asarray(project_rgb_observation_volume(ref.volume,T_base_camera=T,camera_matrix=K,
                image_shape=depth.shape,working_depth_range_m=ref.volume.working_depth_range_m))
            projected_mask=np.zeros(depth.shape,np.uint8)
            cv2.fillConvexPoly(projected_mask,np.rint(cv2.convexHull(pix.astype(np.float32))).astype(np.int32),1)
            matches=[]
            for idx,region in enumerate(proposals.candidates):
                fraction=float(np.count_nonzero(region.mask&projected_mask)/region.area_px)
                h,c=_appearance(rgb,region.mask.astype(np.uint8))
                colour=float(np.sqrt(max(0.,1.-np.sum(np.sqrt(h*ref.histogram)))))
                shape=float(cv2.matchShapes(ref.contour,c,cv2.CONTOURS_MATCH_I1,0.))
                if fraction>=.8 and colour<=.4 and shape<=.4:
                    matches.append(dict(candidate_index=idx,score=.65*colour/.4+.35*shape/.4,
                        colour=colour,shape=shape,projected_fraction=fraction))
            matches.sort(key=lambda x:x['score'])
            unique=bool(matches) and (len(matches)==1 or matches[1]['score']-matches[0]['score']>.12)
            selected=matches[0]['candidate_index'] if unique else None
            if stage=='before':
                exact=[idx for idx,p in enumerate(proposals.candidates) if sha(p.mask.tobytes())==
                    r['interest']['rgb_interest_members'][member]['mask_sha256']]
                assert len(exact)==1
                selected=exact[0]
            diagnostic=dict(member=member,original_bbox_xywh=ref.volume.bbox_xywh,
                raw_fixed_footprint_support_audit=audit,support_error=error,
                appearance_only_unique=unique,candidate_index=selected,appearance_matches=matches,
                diagnostic_selection_basis='archived_exact_member_mask' if stage=='before' else 'original_appearance_and_volume_only',
                source_owner_association_executed=False)
            row['reference_diagnostics'].append(diagnostic)
            if selected is not None:
                pools[stage].setdefault(str(member),[]).append((row['candidates'][selected],
                    height[proposals.candidates[selected].mask&valid]))
        detector=r.get('detector')
        if detector:
            row['recorded_detector_summary']={k:detector.get(k) for k in ('state','detail','generation',
                'target_locked','target_lost','anchor_source_stamp_ns','anchor_position_base','anchor_extent_m')}
            if detector.get('anchor_position_base'):
                row['recorded_anchor_projected_uv']=project_base_to_pixel(detector['anchor_position_base'],T,intr).tolist()
        outputs.append(row)
    summary={}
    for stage,groups in pools.items():
        summary[stage]={}
        for member,matched in groups.items():
            rows=[r for r,_ in matched]
            summary[stage][member]=dict(matched_frames=len(rows),
                valid_point_min_median_max=span([r['valid_points'] for r in rows]),
                valid_fraction_min_median_max=span([r['valid_fraction'] for r in rows]),
                rgb_area_min_median_max=span([r['area_px'] for r in rows]),
                optical_depth_min_median_max_m=span([r['optical_depth_m'] for r in rows]),
                pooled_height=quant(np.concatenate([h for _,h in matched])),
                original_4mm_point_min_median_max=span([r['height_ge_original_4mm'] for r in rows]),
                opened_4mm_point_min_median_max=span([r['opened_height_ge_original_4mm'] for r in rows]),
                raw_voxel_clean_min_median_max=span([r['native_valid_voxel_diagnostic']['cleaned'] for r in rows]),
                foreground_voxel_clean_min_median_max=span([r['original_foreground_voxel']['cleaned'] for r in rows]))
    supports=[r['reference_diagnostics'][0] for r in outputs if r['stage']=='after']
    seed_summary={}
    for stage in ('before','after'):
        selected_rows=[r for r in outputs if r['stage']==stage]
        relevant=[]
        for row in selected_rows:
            ci=row['reference_diagnostics'][0]['candidate_index']
            candidates=[s for s in row['segmented_candidates'] if ci is not None
                and s['foreground_fraction_in_rgb_regions'][ci]>=.5]
            relevant.append(candidates)
        flat=[s for group in relevant for s in group]
        seed_summary[stage]=dict(frames_with_related_depth_candidate=sum(bool(group) for group in relevant),
            trusted_top_min_median_max=span([s['top_pixels'] for s in flat]),
            trusted_side_min_median_max=span([s['side_pixels'] for s in flat]),
            refine_success_frames=[selected_rows[i]['label'] for i,group in enumerate(relevant)
                if any('refine_error' not in s for s in group)],
            refine_error_counts=dict(Counter(s['refine_error'] for s in flat if 'refine_error' in s)),
            refined_voxel_clean_counts=[s['refined_original_foreground_voxel']['cleaned']
                for s in flat if 'refined_original_foreground_voxel' in s])
    report=dict(before_source=str(BEFORE),before_source_sha256=sha(BEFORE.read_bytes()),
        after_source=str(AFTER),after_source_sha256=sha(AFTER.read_bytes()),script_sha256=sha(Path(__file__).read_bytes()),
        summary=summary,seed_summary=seed_summary,raw_support_summary=dict(
            shift_min_median_max_mm=span([s['raw_fixed_footprint_support_audit']['support_reference_max_distance_m']*1000 for s in supports]),
            normal_min_median_max_deg=span([s['raw_fixed_footprint_support_audit']['support_reference_angle_deg'] for s in supports]),
            rejected=sum(bool(s['support_error']) for s in supports)),
        recorded_after_first_loss=after['rows'][0]['detector'].get('first_loss'),
        source_contexts_identical=before['frames']['initial']['source_context']==after['rows'][0]['source_context'],
        robot_commands_sent=False,thresholds_modified=False,geometry_authorized=False,
        caveats=['RGB appearance matching labels diagnostics only; source-context archive schemas differ and no owner handoff is asserted.',
            'All valid native RGB-mask points are optimistic sampling counts, not certified measured foreground.',
            'No support alignment is applied; raw comparison uses the original frozen measured support footprint, never D subtraction.',
            'Original segment is independently replayed on each raw frame; recorded detector status may describe a nearby earlier frame.'],rows=outputs)
    print(json.dumps({k:report[k] for k in ('summary','seed_summary','raw_support_summary')},indent=2))
    if '--write' in sys.argv:
        if not os.path.ismount('/media/zhuyupei/SanStand'):raise RuntimeError('SanStand_not_mounted')
        target=AFTER.parent/'trial4_before_after_native_geometry.json'
        target.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
        print('report',target,'sha256',sha(target.read_bytes()))


if __name__=='__main__':main()
