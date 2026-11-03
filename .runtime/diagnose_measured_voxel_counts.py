"""Count native archived samples under unchanged production voxel operations."""
from pathlib import Path
import hashlib
import json
import os
import sys

import cv2
import numpy as np

from alicia_grasp_modes.tabletop import Config, _project, segment
from alicia_grasp_modes.rgb_observation_region import CenterSearchPolicy, propose_rgb_observation_regions
from alicia_flexible_grasp.vision.object_geometry import _voxel_centroids, _neighbor_distance_filter


def sha(value):return hashlib.sha256(value).hexdigest()


def span(values):
    return [float(min(values)), float(np.median(values)), float(max(values))] if values else None


def main():
    cv2.setNumThreads(1)
    archive=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/rgb_observation_bootstrap_20261004T034110Z')
    source=archive/'result.json'; document=json.loads(source.read_text())
    local_file=archive/'independent_local_support_diagnostic.json'
    local=json.loads(local_file.read_text())
    rows=[]; reference_regions=None; local_groups=[]
    for frame_index,row in enumerate(document['rows']):
        path=Path(row['source_file']); assert sha(path.read_bytes())==row['source_file_sha256']
        raw=np.load(path,allow_pickle=False);rgb=raw['color_bgr'];native=raw['depth_raw']
        assert sha(rgb.tobytes())==row['source_rgb_sha256'];assert sha(native.tobytes())==row['source_depth_sha256']
        config=Config(**row['config']);depth=native.astype(np.float32)*float(raw['depth_scale'])
        transform=raw['T_base_camera'];points=_project(depth,raw['K'])
        base=points@transform[:3,:3].T+transform[:3,3]
        segmented=segment(depth,raw['K'],transform,config)
        heights=np.einsum('ijk,k->ij',base,segmented.plane_base[:3])+segmented.plane_base[3]
        valid=np.isfinite(depth)&(depth>=config.depth_min_m)&(depth<=config.depth_max_m)
        proposals=propose_rgb_observation_regions(rgb,
            policy=CenterSearchPolicy((.5,.5),radius_fraction=config.center_radius_fraction),depth_valid=valid)
        if reference_regions is None:
            reference_regions=proposals.candidates
            local_groups=[dict(reference_bbox=ref.bbox_xywh,reference_mask_sha256=sha(ref.mask.tobytes()),
                               matches=[]) for ref in reference_regions]
        current=dict(frame_index=frame_index,source_stamp_ns=row['source_stamp_ns'],
                     source_file_sha256=row['source_file_sha256'],candidates=[])
        for region in proposals.candidates:
            item=dict(bbox_xywh=region.bbox_xywh,area_px=region.area_px,
                      mask_sha256=sha(region.mask.tobytes()),filters=[])
            for selection in ('all_native_valid_rgb_pixels_diagnostic_only','legacy_global_height_at_least_4mm'):
                mask=region.mask&valid
                if selection=='legacy_global_height_at_least_4mm':
                    mask&=(heights>=config.foreground_min_m)&(heights<=config.foreground_max_m)
                measured=base[mask]
                for spacing in (.0015,.0025):
                    voxel=_voxel_centroids(measured,spacing)
                    cleaned=_neighbor_distance_filter(voxel,16,2.,cell_size_m=spacing) if len(voxel) else voxel
                    item['filters'].append(dict(selection=selection,voxel_size_m=spacing,
                        native_count=len(measured),voxel_count=len(voxel),cleaned_count=len(cleaned),
                        below_existing_120_points=len(cleaned)<120,
                        metric_object_classification_established=False))
            current['candidates'].append(item)
        rows.append(current)
        for group,ref in zip(local_groups,reference_regions):
            ious=[np.count_nonzero(ref.mask&r.mask)/np.count_nonzero(ref.mask|r.mask) for r in proposals.candidates]
            ranked=sorted(range(len(ious)),key=lambda j:ious[j],reverse=True)
            if not ranked or ious[ranked[0]]<.65:
                continue
            # Diagnostic identity labels only; do not silently select between
            # similarly overlapping alternatives of the same frozen patch.
            if len(ranked)>1 and ious[ranked[0]]-ious[ranked[1]]<=.10:
                continue
            j=ranked[0]
            group['matches'].append(dict(frame_index=frame_index,candidate_index=j,iou=ious[j]))
    summary=[]
    for group in local_groups:
        group['local_ring_summary']=[]
        for radius in (8,16,24):
            folds=[fold for match in group['matches']
                for ring in local['rows'][match['frame_index']]['candidates'][match['candidate_index']]['rings']
                if ring['outer_distance_px']==radius for fold in ring['folds']
                if fold.get('fit_succeeded')
                and fold['training_quadrants']['at_least_20_pixels']>=3
                and fold['held_out_quadrants']['at_least_20_pixels']>=3]
            group['local_ring_summary'].append(dict(radius_px=radius,fold_count=len(folds),
                median_difference_min_median_max_mm=span([f['median_delta_from_heldout_mm'] for f in folds]),
                heldout_scaled_mad_min_median_max_mm=span([f['held_out_residual']['mad_normal_scaled_mm'] for f in folds]),
                local_global_angle_max_deg=max([f['support_normal_angle_deg'] for f in folds],default=None),
                local_global_distance_max_mm=max([f['maximum_local_global_plane_distance_on_object_mm'] for f in folds],default=None)))
        vox=[]
        for spacing in (.0015,.0025):
            selected=[f for match in group['matches']
                for f in rows[match['frame_index']]['candidates'][match['candidate_index']]['filters']
                if f['voxel_size_m']==spacing and f['selection']=='all_native_valid_rgb_pixels_diagnostic_only']
            vox.append(dict(voxel_size_m=spacing,native_counts=span([f['native_count'] for f in selected]),
                voxel_counts=span([f['voxel_count'] for f in selected]),
                cleaned_counts=span([f['cleaned_count'] for f in selected]),
                count_below_existing_120=sum(f['below_existing_120_points'] for f in selected)))
        summary.append(dict(reference_bbox=group['reference_bbox'],matched_frames=len(group['matches']),
                            voxel_results=vox))
    report=dict(source=str(source),source_sha256=sha(source.read_bytes()),
        local_support_diagnostic=str(local_file),local_support_diagnostic_sha256=sha(local_file.read_bytes()),
        diagnostic_script_sha256=sha(Path(__file__).read_bytes()),
        implementation=dict(python=sys.version,numpy=np.__version__,opencv=cv2.__version__),
        spacing_provenance={'0.0015':'runtime ROS parameter read by parent and tracked grasp_params.yaml:425',
                            '0.0025':'comparison only: fallback code default; not current runtime setting'},
        method='actual same-source native points transformed to base, existing voxel centroids and neighbor16 std2 filter',
        all_native_counts_are_optimistic_sampling_diagnostics_not_foreground_certification=True,
        expected_existing_minimum_points=120,thresholds_modified=False,robot_commands_sent=False,
        geometry_authorized=False,summary=summary,local_regions_matched_to_first_frame=local_groups,rows=rows)
    print(json.dumps(summary[:3],ensure_ascii=False))
    if '--write' in sys.argv:
        if not os.path.ismount('/media/zhuyupei/SanStand'):
            raise RuntimeError('SanStand_not_mounted_no_fallback')
        target=archive/'native_voxel_and_fixed_patch_comparison.json'
        if target.exists():raise RuntimeError('diagnostic_report_already_exists')
        target.write_text(json.dumps(report,indent=2,ensure_ascii=False,allow_nan=False)+'\n')
        print('saved',target,'sha256',sha(target.read_bytes()))


if __name__=='__main__':main()
