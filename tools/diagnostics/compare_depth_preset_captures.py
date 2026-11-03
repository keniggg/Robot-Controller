#!/usr/bin/env python3
"""Finite offline A/B/A coverage and temporal-repeatability comparison.

Reuses the stage analyser's sealed-file, pixel and geometry verification. Native
and aligned pixels are never identified by their array indices. All differences
are descriptive; temporal repeatability is not absolute depth accuracy.
"""
import argparse
import hashlib
import io
import json
import resource
import time
from pathlib import Path

import analyze_depth_stage_probe as base
import numpy as np


def quantiles(values):
    values = np.asarray(values, dtype=float)
    values = values[np.isfinite(values)]
    return dict(count=int(values.size), **{
        'p%d' % p: float(np.percentile(values, p)) if values.size else None
        for p in (50, 95, 99)})


def medians(values):
    # Do not emit all-NaN warnings, or silently count unsupported pixels.
    output = np.full(values.shape[1], np.nan)
    present = np.any(values > 0, axis=0)
    masked = np.where(values[:, present] > 0, values[:, present], np.nan)
    output[present] = np.nanmedian(masked, axis=0)
    return output


def repeatability(values, mask, scale):
    selected = values[:, mask]
    count = (selected > 0).sum(axis=0)
    usable = selected[:, count >= 2]
    centers = medians(usable)
    deviations = np.where(usable > 0, np.abs(usable - centers), np.nan) * scale
    pair_differences = []
    for previous, current in zip(selected, selected[1:]):
        both = (previous > 0) & (current > 0)
        pair_differences.append(np.abs(current[both].astype(float) - previous[both]) * scale)
    return dict(selected_pixels=int(mask.sum()), pixels_with_at_least_two_measurements=int(usable.shape[1]),
        abs_deviation_from_own_pixel_temporal_median_m=quantiles(deviations),
        per_pixel_temporal_mad_m=quantiles(np.nanmedian(deviations, axis=0)),
        adjacent_abs_difference_m=quantiles(np.concatenate(pair_differences) if pair_differences else []))


def compare_values(values, scale):
    base.require(len(values) == 3 and len({v.shape for v in values}) == 1, 'equal A/B/A sample grids required')
    valid = [v > 0 for v in values]
    always = [v.all(axis=0) for v in valid]
    absent = [~v.any(axis=0) for v in valid]
    masks = dict(all_roi_pixels=np.ones(values[0].shape[1], dtype=bool),
        valid_in_every_frame_all_three=always[0] & always[1] & always[2],
        B_always_valid_but_each_A_has_a_hole=always[1] & ~always[0] & ~always[2],
        B_always_valid_but_both_A_always_zero=always[1] & absent[0] & absent[2])
    result = dict(support_counts={name:int(mask.sum()) for name,mask in masks.items()},
        segments={label:dict(zero_fractions=(~v).mean(axis=1).tolist(),
            always_valid_pixels=int(a.sum()), always_zero_pixels=int(z.sum()),
            intermittent_pixels=int((~a & ~z).sum()))
            for label,v,a,z in zip(('A1','B','A2'),valid,always,absent)},
        repeatability={name:{label:repeatability(v,mask,scale)
            for label,v in zip(('A1','B','A2'),values)} for name,mask in masks.items()})
    common = masks['valid_in_every_frame_all_three']
    center = [medians(v[:,common]) for v in values]
    result['common_support_between_segment_median_depth_differences_m'] = {
        name:dict(signed=quantiles(center[j]-center[i]), absolute=quantiles(np.abs(center[j]-center[i])))
        for name,i,j in (('B_minus_A1',0,1),('A2_minus_A1',0,2),('A2_minus_B',1,2))}
    for row in result['common_support_between_segment_median_depth_differences_m'].values():
        for record in row.values():
            for key in ('p50','p95','p99'):
                if record[key] is not None:
                    record[key] *= scale
    return result


def stationary_stage_evidence(rows, scale):
    return [dict(pair=row['name'],
        change=base.transition(row['arrays']['after_filter_0_spatial'],
                               row['arrays']['after_stationary_depth'],scale),
        exact_read_regional_evidence=row['metadata']['original_read'].get('regional_stationary_evidence'))
        for row in rows]


def load_color_layers(folder, inputs, rois, reference, rectifier):
    folder=Path(folder).resolve()
    record=json.loads(base.read_file(folder/'result.json',inputs))
    base.require(record.get('kind')=='RGB_only_frozen_table_color_layers' and
                 record.get('depth_used_to_choose_threshold') is False, 'RGB-only frozen color layer specification required')
    base.require(record['source_rgb']==reference['source_rgb'] and
                 record['table_bounds_xyxy']==rois['table']['bounds_xyxy'], 'color layers belong to another ROI reference')
    raw=base.read_file(folder/record['mask_archive']['file'],inputs,record['mask_archive'])
    shape=(rectifier.height,rectifier.width)
    table=np.zeros(shape,dtype=bool);x0,y0,x1,y1=record['table_bounds_xyxy'];table[y0:y1,x0:x1]=True
    combined=np.zeros(shape,dtype=np.uint8);scopes={}
    with np.load(io.BytesIO(raw),allow_pickle=False) as saved:
        base.require(set(saved.files)==set(record['classes']) and len(saved.files)==2, 'two frozen color layers required')
        for name in saved.files:
            mask=saved[name];expected=record['classes'][name]
            base.require(mask.dtype==bool and mask.shape==shape and int(mask.sum())==expected['pixels'] and
                         base.sha256(mask.tobytes())==expected['mask_pixel_sha256'], 'color layer pixels differ')
            combined+=mask.astype(np.uint8)
            selected=mask.ravel()[rectifier.target_indices]
            base.require(selected.any(), 'color layer contains no mapped depth samples')
            scopes[name]=(rectifier.source_indices[selected],rectifier.target_indices[selected])
    base.require(np.array_equal(combined,table.astype(np.uint8)), 'color layers must partition the declared table rectangle exactly')
    return record,scopes


def self_test():
    a1=np.array([[1,0,0,1],[1,1,0,1]],dtype=np.uint16)
    b=np.array([[2,2,2,0],[2,2,2,0]],dtype=np.uint16)
    a2=np.array([[1,1,0,1],[1,0,0,1]],dtype=np.uint16)
    result=compare_values([a1,b,a2],.001)
    base.require(result['support_counts'] == dict(all_roi_pixels=4,
        valid_in_every_frame_all_three=1,B_always_valid_but_each_A_has_a_hole=2,
        B_always_valid_but_both_A_always_zero=1), 'support masks differ')
    delta=result['common_support_between_segment_median_depth_differences_m']['B_minus_A1']['signed']
    base.require(delta['p50'] == .001, 'unit conversion differs')
    empty=result['repeatability']['B_always_valid_but_both_A_always_zero']['A1']
    base.require(empty['pixels_with_at_least_two_measurements'] == 0 and
                 empty['adjacent_abs_difference_m']['count'] == 0, 'missing support was treated as measured')
    row=dict(name='pair_01',arrays=dict(after_filter_0_spatial=np.array([[0,10,20]],dtype=np.uint16),
        after_stationary_depth=np.array([[0,11,20]],dtype=np.uint16)),
        metadata=dict(original_read=dict(regional_stationary_evidence=dict(active=True,filtered_pixel_count=2))))
    observation=stationary_stage_evidence([row],.001)[0]
    base.require(observation['change']['changed_positive_values']==1 and
                 observation['exact_read_regional_evidence']['active'] is True, 'active regional evidence lost')
    return dict(passed=True,checks=['common and recovered support masks','unit conversion',
        'absent support remains absent','active stationary changes are retained separately'])


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--captures',type=Path,nargs=3,metavar=('A1','B','A2'))
    roi_args=parser.add_mutually_exclusive_group()
    roi_args.add_argument('--roi-burst',type=Path)
    roi_args.add_argument('--roi-spec',type=Path)
    parser.add_argument('--color-layers',type=Path,
        help='Frozen RGB-only layer directory; reused exactly, never re-thresholded on an A/B/A frame.')
    parser.add_argument('--sdk-root',type=Path,default=base.WORKTREE/'.runtime/realsense-2.56.5')
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--self-test',action='store_true')
    args=parser.parse_args();started=time.monotonic();cpu=time.process_time()
    if args.self_test:
        result=self_test()
    else:
        base.require(args.captures and (args.roi_burst or args.roi_spec), 'three captures and explicit ROI source required')
        inputs={};reference_contract=None
        if args.roi_spec:
            rois,_,reference_source,reference_contract=base.load_roi_spec(args.roi_spec,inputs)
        else:
            rois=base.load_rois(args.roi_burst,inputs);reference_source=dict(kind='sealed_historical_burst')
        verified=[base.verify_capture(path,inputs) for path in args.captures]
        rows=[entry[1] for entry in verified]
        base.require(len({len(v) for v in rows}) == 1, 'A/B/A frame counts differ')
        rectifier=base.make_rectifier(sum(rows,[]),args.sdk_root,inputs)
        if reference_contract is not None:
            base.require(reference_contract == rectifier.contract, 'current ROI reference/capture rectification differs')
        profiles=[segment[0]['metadata']['context']['runtime_profile'] for segment in rows]
        controls=('device','sdk_version','color_intrinsics','native_depth_intrinsics',
                  'depth_to_color_extrinsics','alignment','depth_filters','color_projection_correction','rectification')
        for key in controls:
            base.require(all(p.get(key) == profiles[0].get(key) for p in profiles), 'A/B/A geometry/filter change: '+key)
        for segment,expected in zip(rows,('high_accuracy','saved_baseline','high_accuracy')):
            for row in segment:
                meta=row['metadata'];preset=meta['context']['runtime_profile']['mode_depth_preset']
                base.require(preset.get('mode') == 'unknown' and preset.get('preset') == expected,
                             'A/B/A task mode or preset differs')
        scale=rows[0][0]['metadata']['context']['depth_scale_m']
        base.require(all(r['metadata']['context']['depth_scale_m'] == scale for segment in rows for r in segment),
                     'depth scale differs')
        scope_indices={name:base.source_roi(rectifier,record['bounds_xyxy']) for name,record in rois.items()}
        canvas_counts={name:(record['bounds_xyxy'][2]-record['bounds_xyxy'][0])*
            (record['bounds_xyxy'][3]-record['bounds_xyxy'][1]) for name,record in rois.items()}
        color_layers=None
        if args.color_layers:
            base.require(args.roi_spec is not None, 'frozen current color layers require --roi-spec')
            color_layers,color_scopes=load_color_layers(args.color_layers,inputs,rois,reference_source,rectifier)
            base.require(not set(color_scopes).intersection(scope_indices), 'duplicate comparison scope')
            scope_indices.update(color_scopes)
            canvas_counts.update({name:entry['pixels'] for name,entry in color_layers['classes'].items()})
        comparisons={};downstream={}
        for name,(source,target) in scope_indices.items():
            comparisons[name]={};downstream[name]={}
            for stage in ('after_align_depth','after_filter_0_spatial','after_stationary_depth',
                          'after_clip_depth','after_rectification_depth'):
                indices=target if stage=='after_rectification_depth' else source
                values=[np.stack([r['arrays'][stage].ravel()[indices] for r in segment]) for segment in rows]
                destination=comparisons if stage in ('after_align_depth','after_filter_0_spatial') else downstream
                destination[name][stage]=compare_values(values,scale)
        snapshots=[segment[0]['metadata']['owner'].get('preset_snapshot') for segment in rows]
        result=dict(schema=1,kind='finite_preset_ABA_coverage_repeatability',passed=True,diagnostic_only=True,
            input_files=inputs,captures=[str(v.resolve()) for v in args.captures],rois=rois,roi_reference=reference_source,
            configured_presets=['high_accuracy','saved_baseline','high_accuracy'],preset_snapshots=snapshots,
            geometry_and_filter_contracts_equal=True,
            depth_scale_m_per_unit=scale,comparisons=comparisons,downstream_output_comparisons=downstream,
            stationary_stage_evidence={label:stationary_stage_evidence(segment,scale)
                for label,segment in zip(('A1','B','A2'),rows)},
            primary_preset_comparison_stages=['after_align_depth','after_filter_0_spatial'],
            color_layers=color_layers,
            scope_mapped_depth_pixels={name:len(indices[0]) for name,indices in scope_indices.items()},
            scope_canvas_pixels=canvas_counts,
            scope_fixed_unmapped_pixels={name:canvas_counts[name]-len(indices[0]) for name,indices in scope_indices.items()},
            interpretation_limits=[
                'Primary preset comparisons precede regional temporal filtering; downstream output also depends on its measured eligibility and history.',
                'Fixed ROI labels do not prove current scene occupancy, planar truth, or no motion.',
                'Temporal repeatability is not absolute accuracy; permanently biased points can look stable.',
                'New B-only measurements have no A measurement reference and need independent geometric validation.',
                'Frozen RGB layers describe association; color, brightness, texture, material and reflection are not independently controlled.',
                'Consecutive frames and neighboring pixels are correlated; these are descriptive statistics.',
                'Preset snapshots may precede warmup; missing per-frame exposure/gain evidence remains a limitation.'],
            ROS_contacted=False,SDK_device_access=False,parameters_changed=False,grasp_verified=False)
    result.update(source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        elapsed_sec=time.monotonic()-started,process_cpu_sec=time.process_time()-cpu,
        peak_rss_kib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss)
    hashes=base.write_results(args.output,result)
    print(json.dumps(dict(output=str(args.output),files=hashes,elapsed_sec=result['elapsed_sec']),ensure_ascii=False))


if __name__ == '__main__':
    main()
