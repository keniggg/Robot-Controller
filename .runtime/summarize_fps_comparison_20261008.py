#!/usr/bin/env python3
"""Offline three-leg FPS comparison of sealed evidence; never reads ROS/devices."""
import argparse
from collections import Counter, defaultdict
import hashlib
import importlib.util
import json
from pathlib import Path

HERE=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('sealed_sdk_analysis',HERE/'analyze_sdk_memory_pressure_20261007.py')
memory=importlib.util.module_from_spec(spec);spec.loader.exec_module(memory)
LEGS=(('baseline30','baseline30_passive_v2'),('trial15','trial15_passive'),('restore30','restore30_passive'))


def sha(raw): return hashlib.sha256(raw).hexdigest()
def require(value,reason):
    if not value: raise ValueError(reason)


def successful_read_summary(entries,start,end,coverage):
    selected=sorted((entry for entry in entries if start<=entry['read_completed_monotonic']<end),
                    key=lambda entry:entry['read_completed_monotonic'])
    processing=[];invalid_processing=0;center_known=0;qualified=0;applied=0
    reasons=Counter();visual=Counter();missing_reset=0;stages=defaultdict(list)
    source_intervals=[];complete_intervals=[];deltas={stream:defaultdict(list) for stream in ('color','depth')}
    for i,entry in enumerate(selected):
        evidence=entry['evidence'];require(evidence.get('result')=='complete','noncomplete_success_history')
        timing=evidence.get('stages_sec',{})
        total=evidence.get('total_sec');wait=timing.get('wait_for_frames')
        if memory.finite(total) and memory.finite(wait) and 0<=wait<=total:
            processing.append(total-wait)
        else: invalid_processing+=1
        for key,value in timing.items():
            if key!='wait_for_frames' and memory.finite(value): stages[key].append(value)
        regional=evidence.get('regional_stationary_evidence',{})
        if 'global_history_reset_reason' in regional: reasons[str(regional['global_history_reset_reason'])]+=1
        else: missing_reset+=1
        centers=[region for region in regional.get('regions',[]) if region.get('name')=='center']
        if len(centers)==1 and type(centers[0].get('qualified')) is bool and type(centers[0].get('filter_applied')) is bool:
            center_known+=1;qualified+=int(centers[0]['qualified']);applied+=int(centers[0]['filter_applied'])
            visual[str((centers[0].get('visual') or {}).get('reason','unavailable'))]+=1
        if i:
            before=selected[i-1]
            if memory.entry_key(before)[:2]!=memory.entry_key(entry)[:2]: continue
            source_intervals.append((entry['source_stamp_ns']-before['source_stamp_ns'])/1e9)
            complete_intervals.append(entry['read_completed_monotonic']-before['read_completed_monotonic'])
            changes=memory.source_delta(before['evidence'].get('raw_frames',{}),evidence.get('raw_frames',{}))
            for stream,change in changes.items():
                if change['same_timestamp_domain']:
                    for key,value in change['delta'].items(): deltas[stream][key].append(value)
    valid=[row for row in coverage if row.get('valid')]
    return dict(successful_reads_completed_inside_capture=len(selected),
        capture_start_monotonic=start,capture_end_monotonic=end,capture_duration_sec=end-start,
        excluded_outside=len(entries)-len(selected),
        global_history_reset_reasons=dict(reasons),global_reset_count=sum(v for k,v in reasons.items() if k!='None'),
        global_reset_field_unavailable=missing_reset,
        center=dict(known=center_known,unavailable=len(selected)-center_known,qualified=qualified,filter_applied=applied,
                    qualified_fraction_among_known=qualified/center_known if center_known else None,
                    filter_applied_fraction_among_known=applied/center_known if center_known else None,
                    visual_reasons=dict(visual)),
        processing_excluding_wait_stage_seconds=memory.stats(processing),processing_unavailable=invalid_processing,
        processing_definition='original total_sec minus original stages_sec.wait_for_frames; excludes SDK wait stage, not all camera-node/observer costs',
        nonwait_stages_seconds={key:memory.stats(values) for key,values in stages.items()},
        successful_source_interval_seconds=memory.stats(source_intervals),
        successful_read_completion_interval_seconds=memory.stats(complete_intervals),
        raw_stream_deltas_between_visible_successes={stream:{key:memory.stats(values) for key,values in row.items()}
            for stream,row in deltas.items()},
        raw_clock_semantics='device_timestamp_ms is SDK timestamp; metadata frame_timestamp/backend/arrival deltas retain raw units. API counters and actual_fps are not independent sensor-production evidence.',
        metadata_coverage=dict(rpc_rows=len(coverage),valid_rpc_rows=len(valid),invalid_rpc_rows=len(coverage)-len(valid),
            proven_adjacent_history_overlap_count=sum(row.get('no_successful_history_gap_proven') is True for row in valid),
            explicit_zero_overlap_count=sum(row.get('previous_same_owner_overlap_count')==0 for row in valid),
            copy_error_or_missing_count=sum(type(row.get('producer_copy_errors')) is not int or row['producer_copy_errors']!=0 for row in valid),
            first_read_after_start_sec=selected[0]['read_completed_monotonic']-start if selected else None,
            tail_after_last_read_sec=end-selected[-1]['read_completed_monotonic'] if selected else None,
            entire_capture_coverage_proven=False,
            limitation='Only retained successful reads; rejected SDK returns, unobserved edges and uncovered RPC gaps are not reconstructed.'))


def passive_summary(folder):
    # Reuse the existing checksum, owner, reference, boot and time validation.
    analysis=memory.analyze(folder)
    raw=(folder/'sdk_entries.jsonl').read_bytes()
    require(sha(raw)==analysis['input_sha256']['sdk_entries.jsonl'],'SDK_file_changed_after_validation')
    entries=[json.loads(line)['original_entry'] for line in raw.splitlines()]
    window=analysis['window']
    result=successful_read_summary(entries,window['start_monotonic'],window['end_monotonic'],analysis['rpc_history_coverage'])
    result.update(input_sha256=analysis['input_sha256'],integrity=analysis['integrity'],owners=window['owners'],
        wait_seconds_reads_completed_inside_capture=analysis['summary']['wait_seconds'],
        wait_seconds_wholly_inside_capture=analysis['summary']['wait_seconds_wholly_inside_capture'],
        long_wait_count=analysis['summary']['long_wait_count'],
        long_wait_count_wholly_inside_capture=analysis['summary']['long_wait_count_wholly_inside_capture'],
        wait_scope=analysis['summary']['wait_seconds_scope'])
    return result


def quality_summary(folder,capture):
    sums={name:checksum for checksum,name in (line.split(None,1)
        for line in (folder/'SHA256SUMS').read_text().splitlines() if line.strip())}
    raw=(folder/'report.json').read_bytes();require(sha(raw)==sums.get('report.json'),'quality_report_SHA_mismatch')
    report=json.loads(raw)
    require(report.get('camera_plane_only') is True and report.get('quality_thresholds_changed') is False,
            'camera_plane_only_quality_contract_missing')
    require(report.get('target_identity') is None and report.get('motion_authorized') is False,'quality_report_has_target_authority')
    require(Path(report['capture']).resolve()==capture.resolve(),'quality_capture_path_mismatch')
    require(sha((capture/'summary.json').read_bytes())==report['capture_summary_sha256'],'quality_capture_summary_changed')
    script=(folder/'script_at_analysis.py').read_bytes()
    require(sha(script)==sums.get('script_at_analysis.py')==report['analysis_source_sha256'],'quality_analyzer_SHA_mismatch')
    rows=report['rows'];require(0<len(rows)<=6,'quality_rows_not_one_to_six')
    require([row['source_stamp_ns'] for row in rows]==report['selected_sources'],'quality_selected_sources_mismatch')
    camera=report['camera_parameters']
    slim=[]
    for row in rows:
        rings=row['analysis']['rings']
        require([ring['outer_base_px'] for ring in rings]==[170,140,110],'quality_ring_sequence_changed')
        slim.append(dict(source_stamp_ns=row['source_stamp_ns'],support_pass=row['analysis']['support_available'],
            reason=row['analysis'].get('support_reason'),selected_outer_base_px=row['analysis'].get('selected_outer_base_px'),
            exact_sdk_read_available=row['exact_read_history_unique'],center=row['same_source_center'],
            global_reset=row['same_source_global_history_reset_reason'],
            rings=[dict(outer_base_px=r['outer_base_px'],valid_points=r['valid_points'],valid_fraction=r['valid_fraction'],
                        original_fit=r['fit'],full_ring_diagnostic=r.get('original_model_full_ring_diagnostic')) for r in rings]))
    return dict(report_sha256=sha(raw),analyzer_sha256=report['analysis_source_sha256'],
        production_tabletop_sha256=report['production_tabletop_sha256'],
        total_original_saved_pairs=report['total_saved_pairs'],analyzed_pairs=len(rows),
        support_pass_count=sum(row['support_pass'] for row in slim),
        exact_sdk_read_count=sum(row['exact_sdk_read_available'] for row in slim),
        original_config=report['config'],camera_intrinsics=report['camera_intrinsics'],depth_scale_m=report['depth_scale_m'],
        observed_fps=camera.get('fps'),observed_publish_fps=camera.get('publish_fps'),
        camera_config_except_fps_and_runtime={k:v for k,v in camera.items() if k not in ('fps','publish_fps','runtime_profile')},
        original_runtime_geometry={k:camera.get('runtime_profile',{}).get(k) for k in
            ('geometry_intrinsics','geometry_intrinsics_source','rectification')},
        rows=slim,sampling=report['sampling'])


def summarize(folder):
    result=dict(schema=1,offline_only=True,problem_resolved=False,motion_authorized=False,legs={},pending_legs=[],invalid_legs=[],
        limitations=['A/B/A includes camera restarts and different observation times; it does not isolate FPS from all scene/host changes.',
            'Passive timing and RGB-D quality are separate windows, not assumed to be concurrent.',
            'At most six quality-blind samples per leg do not establish continuous support acceptance or target graspability.',
            'All statistics exclude successful read completions outside each passive capture; outside-window waits are separately labelled.',
            'No clock interpolation, missing-frame reconstruction, threshold change or physical-success claim is made.'])
    for leg,passive in LEGS:
        p=folder/passive;q=folder/(leg+'_rgbd_analysis');capture=folder/(leg+'_rgbd')
        missing=[str(path) for path in (p/'outcome.json',p/'SHA256SUMS',q/'report.json',q/'SHA256SUMS',capture/'summary.json') if not path.is_file()]
        if missing:
            result['legs'][leg]=dict(status='pending',missing=missing);result['pending_legs'].append(leg);continue
        try:
            ps=passive_summary(p);qs=quality_summary(q,capture)
            require(ps['integrity']['observer_integrity_passed'] is True and ps['integrity']['same_boot'] is True,
                    'passive_observer_integrity_not_passed')
            result['legs'][leg]=dict(status='complete',passive=ps,quality=qs)
        except Exception as exc:
            result['legs'][leg]=dict(status='invalid',error=type(exc).__name__+': '+str(exc));result['invalid_legs'].append(leg)
    complete=not result['pending_legs'] and not result['invalid_legs']
    result['all_three_legs_complete']=complete
    result['config_comparability']='pending' if not complete else 'consistent_except_requested_fps_and_runtime_owner'
    if complete:
        base=result['legs']['baseline30']['quality'];differences=[]
        for leg,_ in LEGS[1:]:
            q=result['legs'][leg]['quality']
            for key in ('analyzer_sha256','production_tabletop_sha256','original_config','camera_intrinsics','depth_scale_m',
                        'camera_config_except_fps_and_runtime','original_runtime_geometry'):
                if q[key]!=base[key]: differences.append(dict(leg=leg,field=key))
        result['config_differences']=differences
        if differences: result['config_comparability']='differences_require_review'
    result['summary_source_sha256']=sha(Path(__file__).read_bytes())
    result['passive_analyzer_source_sha256']=sha((HERE/'analyze_sdk_memory_pressure_20261007.py').read_bytes())
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input',type=Path,required=True);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();folder=args.input.resolve();out=args.output.resolve()
    require(memory.MOUNT.is_mount() and memory.EXTERNAL in folder.parents and memory.EXTERNAL in out.parents,
            'mounted_external_experiment_paths_required')
    require(out.parent.is_dir() and not out.exists(),'new_output_in_existing_external_directory_required')
    result=summarize(folder)
    with out.open('xb') as handle: handle.write((json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False)+'\n').encode())
    print(json.dumps(dict(output=str(out),sha256=sha(out.read_bytes()),all_three_legs_complete=result['all_three_legs_complete'],
        pending_legs=result['pending_legs'],invalid_legs=result['invalid_legs'])))
    return 0 if result['all_three_legs_complete'] else 2


if __name__=='__main__': raise SystemExit(main())
