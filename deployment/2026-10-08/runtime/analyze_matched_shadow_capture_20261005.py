#!/usr/bin/env python3
"""Read-only, same-frame camera-local support comparison; never target authority.

Only new capture files are inputs. No old object reference, mask, ROS, physical
device, image completion, TF tolerance or production configuration mutation.
"""
import argparse
from collections import Counter
from dataclasses import asdict, fields
import hashlib
import json
from pathlib import Path
import re
import sys
import zipfile

import cv2
import numpy as np

WORK = Path(__file__).resolve().parents[1]
for relative in ('src/alicia_grasp_modes/src', 'src/alicia_flexible_grasp_supervisor/src'):
    sys.path.insert(0, str(WORK / relative))
from alicia_grasp_modes import tabletop
from alicia_flexible_grasp.vision.rgbd_rectification import PairedPinholeRectification
from alicia_flexible_grasp.vision.stationary_depth import StationaryDepthWindow

MOUNT = Path('/media/zhuyupei/SanStand')
EXT = MOUNT / '实验记录/Robot-Controller'
STAGES = ('pre_stationary_depth', 'matched_shadow_depth', 'post_stationary_depth', 'final_depth')


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def array_sha(value):
    return hashlib.sha256(np.ascontiguousarray(value).tobytes()).hexdigest()


def digest(value):
    # Exact encoding used by the frozen observer's feedback-window audit.
    raw = (json.dumps(value, ensure_ascii=False, allow_nan=False, sort_keys=True, indent=2)+'\n').encode()
    return hashlib.sha256(raw).hexdigest()


def quantiles(value):
    values = np.asarray(value, dtype=float).ravel()
    if not np.isfinite(values).all():
        raise ValueError('nonfinite diagnostic statistic')
    if not values.size:
        return None
    return dict(zip(('min', 'p10', 'p50', 'p90', 'max'), map(float, np.percentile(values, (0, 10, 50, 90, 100)))))


def verify_artifacts(folder):
    """Verify every committed sample and refuse orphan/extra sample files."""
    summary = json.loads((folder/'summary.json').read_text())
    listed = {}
    for item in summary['files']:
        name = item['name']
        if (type(name) is not str or not re.fullmatch(r'sample_\d{2}\.(json|npz)', name)
                or name in listed or not 0 < item['bytes'] <= 8*1024**2):
            raise ValueError('invalid or duplicate sample manifest entry')
        path = folder/name
        if path.is_symlink() or path.stat().st_size != item['bytes'] or sha(path) != item['sha256']:
            raise ValueError('sample file integrity failure: '+name)
        listed[name] = item
    samples = sorted({Path(name).stem for name in listed})
    if not 1 <= len(samples) <= 24 or any({s+'.json', s+'.npz'}-listed.keys() for s in samples):
        raise ValueError('sample manifest missing paired files or invalid count')
    if {p.name for p in folder.glob('sample_*')} != set(listed):
        raise ValueError('uncommitted sample file present')
    total = sum(p.stat().st_size for p in folder.iterdir() if p.is_file())
    if total > min(summary['maximum_artifact_bytes'], 159000000):
        raise ValueError('capture artifact bound exceeded')
    return summary, samples, dict(summary_sha256=sha(folder/'summary.json'),
        samples=len(samples), actual_capture_bytes=total, manifest_sample_files_verified=True,
        authentication_scope='Internal recorded hashes only; not a cryptographic signature.')


def read_sample(folder, stem):
    info = json.loads((folder/(stem+'.json')).read_text())
    with zipfile.ZipFile(folder/(stem+'.npz')) as archive:
        if any(item.file_size > 2*1024**2+8192 for item in archive.infolist()):
            raise ValueError('sample array exceeds bounded decoded size')
    with np.load(folder/(stem+'.npz'), allow_pickle=False) as archive:
        if set(archive.files) != set(info['arrays']) or len(archive.files) != len(set(archive.files)):
            raise ValueError('sample array keyset mismatch')
        arrays = {}
        for key in archive.files:
            array = archive[key]
            recorded = info['arrays'][key]
            color = key.endswith('rgb')
            if (array.dtype != (np.uint8 if color else np.uint16)
                    or array.ndim != (3 if color else 2) or color and array.shape[2] != 3
                    or not 0 < array.nbytes <= 2*1024**2
                    or str(array.dtype) != recorded['dtype'] or list(array.shape) != recorded['shape']
                    or array.nbytes != recorded['bytes'] or array_sha(array) != recorded['sha256']):
                raise ValueError('sample array integrity failure: '+key)
            arrays[key] = array.copy()
    if info['sample_index'] != int(stem.rsplit('_', 1)[1]):
        raise ValueError('sample index mismatch')
    return info, arrays


def source_classification(info, summary):
    evidence = info.get('source_evidence', {})
    feedback = evidence.get('feedback', {})
    tf, audit = evidence.get('exact_tf'), evidence.get('tf_stamp_audit', {})
    anchor = info.get('anchor_ros_stamp_ns')
    original = info.get('production_read_diagnostics', {})
    pair = bool(info.get('complete') is True and original.get('result') == 'complete'
                and original.get('pair_timing', {}).get('accepted') is True
                and not any(info.get(k) for k in ('capture_error', 'context_incomplete', 'metadata_error')))
    reasons = []
    if not pair:
        reasons.append('original_pair_or_metadata_incomplete')
    if evidence.get('evidence_complete') is not True:
        reasons.append('recorded_source_evidence_incomplete')
    try:
        frozen = summary['frozen_guard']
        lo, hi = feedback['accepted_before'], feedback['accepted_after']
        q = np.asarray(feedback['q_at_anchor'], dtype=float)
        if not (type(anchor) is int and anchor > 0 and feedback['qualified'] is True
                and feedback['anchor_ros_stamp_ns'] == anchor and not feedback['faults']
                and frozen['epoch'] <= lo['source_stamp_ns'] <= anchor <= hi['source_stamp_ns']
                and hi['source_stamp_ns']-lo['source_stamp_ns'] == feedback['span_ns'] <= 200000000
                and feedback['maximum_span_ns'] == 200000000
                and q.shape == (7,) and np.isfinite(q).all()
                and lo['q'] == hi['q'] == feedback['q_at_anchor'] == frozen['accepted']['q']):
            raise ValueError('feedback contract')
    except (KeyError, TypeError, ValueError):
        reasons.append('feedback_bracket_not_qualified')
    try:
        if not (audit['requested_stamp_ns'] == audit['returned_stamp_ns'] == anchor
                and audit['delta_ns'] == 0 and audit['exact_match'] is True
                and tf['source_stamp_ns'] == anchor and tf['parent'] and tf['child']
                and tf['parent'] != tf['child'] and len(tf['translation']) == 3
                and len(tf['quaternion_xyzw']) == 4
                and np.isfinite(tf['translation']+tf['quaternion_xyzw']).all()
                and abs(sum(v*v for v in tf['quaternion_xyzw'])-1.) <= 1e-6):
            raise ValueError('strict TF contract')
    except (KeyError, TypeError, ValueError):
        reasons.append('strict_tf_not_qualified')
    if summary.get('guard_faults'):
        reasons.append('capture_guard_fault')
    return dict(original_pair_complete=pair, source_qualified=not reasons, rejection_reasons=reasons,
                tf_stamp_audit=audit, recorded_tf_error=evidence.get('tf_error'),
                scope='Original strict host read-completion anchor only; never exposure-time pose or target authority.')


def shadow_classification(info, arrays):
    record = info.get('matched_stationary_shadow', {})
    reasons = []
    pre, shadow = arrays.get('pre_stationary_depth'), arrays.get('matched_shadow_depth')
    if pre is None or shadow is None or pre.shape != shadow.shape:
        return dict(eligible=False, history_filtered=False, kind='unavailable', contradictions=['shadow_arrays_missing'])
    if np.any(shadow[pre == 0]):
        reasons.append('shadow_filled_current_hole')
    active, first = record.get('active') is True, record.get('first_history_frame') is True
    if record.get('error') or record.get('pair_complete') is False:
        reasons.append('shadow_or_pair_error')
    if record.get('original_output_replaced') is not False or record.get('geometry_authority') is not False:
        reasons.append('shadow_scope_not_recorded')
    if (record.get('alpha'), record.get('delta_depth_units'), record.get('persistence')) != (.4, 100, 0):
        reasons.append('shadow_parameters_differ')
    raw = info.get('production_read_diagnostics', {}).get('raw_frames', {})
    if record.get('color_source') != raw.get('color') or record.get('depth_source') != raw.get('depth'):
        reasons.append('shadow_pair_source_mismatch')
    encoder = record.get('encoder', {})
    try:
        window = encoder['source_window']
        if set(window) != {'sdk', 'accepted'} or any(len(rows) > 24 for rows in window.values()):
            raise ValueError('encoder window bounds')
        if digest(window) != encoder['source_window_sha256']:
            raise ValueError('encoder window digest')
        saved = StationaryDepthWindow()
        saved.history = window
        qualified = saved.stationary(encoder['evaluation_ros_stamp_ns']*1e-9)
        if qualified != encoder['qualified']:
            raise ValueError('encoder qualification differs from original algorithm')
    except (KeyError, TypeError, ValueError, IndexError):
        reasons.append('encoder_window_not_reproducible')
    if active:
        visual = record.get('visual', {})
        if (encoder.get('qualified') is not True or visual.get('qualified') is not True
                or visual.get('reason') != 'stationary' or record.get('output_kind') != 'filtered'
                or type(record.get('history_id')) is not int or record['history_id'] < 1):
            reasons.append('active_qualification_inconsistent')
    elif record.get('output_kind') != 'current_frame_passthrough':
        reasons.append('inactive_kind_inconsistent')
    if (not active or first) and not np.array_equal(pre, shadow):
        reasons.append('passthrough_or_first_frame_changed')
    kind = 'history_filtered' if active and not first else 'first_history_frame' if active else 'inactive_passthrough'
    return dict(eligible=active and not reasons, history_filtered=active and not first and not reasons,
        kind=kind, contradictions=reasons, recorded=record,
        visual_replay_possible=False, full_hash_chain_replay_possible=False)


def fit_audit(points, cfg):
    """Call unmodified original fit; keep its rejected model as diagnostic only."""
    state = {}
    def trace(frame, event, arg):
        if frame.f_code is tabletop._fit_plane.__code__ and event in ('exception', 'return'):
            local = frame.f_locals
            if all(k in local for k in ('normal', 'offset', 'ratio')):
                state.update(plane=np.r_[local['normal'], local['offset']].copy(), ratio=float(local['ratio']))
        return trace
    old, passed, error, residual = sys.gettrace(), False, None, None
    try:
        sys.settrace(trace)
        plane, ratio, residual, count = tabletop._fit_plane(points, cfg)
        state.update(plane=plane, ratio=ratio)
        passed = True
    except ValueError as exc:
        error = str(exc)
    finally:
        sys.settrace(old)
    plane = state.get('plane')
    if plane is not None and plane[3] < 0:
        plane = -plane
    audit = dict(fit_passed=passed, reason=error, original_sampled_inlier_ratio=state.get('ratio'),
                 points=len(points), inlier_rmse_mm=None if residual is None else residual*1000,
                 diagnostic_rejected_model=not passed and plane is not None,
                 plane_camera_link=None if plane is None else plane.tolist(), geometry_authority=False)
    if plane is not None:
        distance = points@plane[:3]+plane[3]
        audit.update(abs_residual_mm=quantiles(abs(distance)*1000), signed_residual_mm=quantiles(distance*1000),
                     all_points_inlier_ratio=float(np.mean(abs(distance) <= cfg.support_threshold_m)))
    return plane, audit


def initial_support(points, valid, radius, u, v, cfg):
    """Original initial support rings/thresholds/coverage, without fabricated TF."""
    height, width = valid.shape
    if np.count_nonzero(valid) < cfg.support_min_points:
        return dict(passed=False, reason='insufficient_valid_depth', attempts=[])
    factor = min(width/640., height/480.)
    attempts = []
    for outer in (170, 140, 110):
        ring = valid & (radius >= 70*factor) & (radius <= outer*factor)
        plane, audit = fit_audit(points[ring], cfg)
        audit['outer_radius_px'] = outer*factor
        attempts.append(audit)
        if audit['fit_passed']:
            supporting = ring & (abs(np.einsum('ijk,k->ij', points, plane[:3])+plane[3]) <= cfg.support_threshold_m)
            covered = tabletop._support_quadrants(supporting, u, v, np.array([width/2., height/2.]))
            audit['covered_quadrants'] = covered
            reason = ('support_plane_residual_too_large' if audit['inlier_rmse_mm'] > cfg.support_threshold_m*1000
                      else 'insufficient_support_spatial_coverage' if covered < 3 else None)
            return dict(passed=reason is None, reason=reason, attempts=attempts)
        if audit['reason'] != 'support_plane_inlier_ratio_low':
            break
    return dict(passed=False, reason=attempts[-1]['reason'], attempts=attempts)


def clip_raw(depth, scale, interval):
    if len(interval) != 2 or not np.isfinite(interval).all() or not 0 < interval[0] < interval[1]:
        raise ValueError('recorded camera clip interval invalid')
    lo, hi = int(np.ceil(interval[0]/scale-1e-9)), int(np.floor(interval[1]/scale+1e-9))
    result = depth.copy()
    result[(result < lo) | (result > hi)] = 0
    return result


def mapped_stages(info, arrays, mapper):
    profile = info['runtime_profile']
    if (profile['rectification'] != mapper.contract or profile['geometry_intrinsics'] != mapper.output_intrinsics
            or profile['color_intrinsics'] != mapper.source_intrinsics
            or info['published_camera_parameters']['runtime_profile']['geometry_intrinsics'] != mapper.output_intrinsics):
        raise ValueError('captured rectification contract differs')
    scale = info['depth_scale_m']
    if not np.isfinite(scale) or scale <= 0 or scale != profile['depth_scale_m_per_unit']:
        raise ValueError('captured depth scale differs')
    interval = profile['depth_filters']['range_m']
    rgb, final = mapper.apply(arrays['raw_rgb'], clip_raw(arrays['post_stationary_depth'], scale, interval))
    if not np.array_equal(rgb, arrays['final_rgb']) or not np.array_equal(final, arrays['final_depth']):
        raise ValueError('original final RGB-D not reproducible from recorded aligned pair')
    result = {'final_depth': arrays['final_depth']}
    for stage in STAGES[:-1]:
        clipped = clip_raw(arrays[stage], scale, interval)
        if clipped.shape != (mapper.source_height, mapper.source_width):
            raise ValueError('aligned array geometry mismatch')
        out = np.zeros((mapper.height, mapper.width), np.uint16)
        out.ravel()[mapper.target_indices] = clipped.ravel()
        result[stage] = out
    return result


def compare_stages(rectified, scale, intrinsics, cfg):
    shape = rectified['final_depth'].shape
    if any(depth.shape != shape for depth in rectified.values()):
        raise ValueError('common raster required')
    v, u = np.indices(shape)
    center = (shape[1]/2., shape[0]/2.)
    radius = np.hypot(u-center[0], v-center[1])
    factor = min(shape[1]/640., shape[0]/480.)
    ring = (radius >= 70*factor) & (radius <= 140*factor)
    depths = {name:array.astype(np.float32)*scale for name, array in rectified.items()}
    valid = {name:np.isfinite(depth)&(depth >= cfg.depth_min_m)&(depth <= cfg.depth_max_m) for name, depth in depths.items()}
    common = ring & np.logical_and.reduce(list(valid.values()))
    points = {name:tabletop._project(depth, intrinsics) for name, depth in depths.items()}
    out = dict(fixed_ring=dict(center_xy=center, inner_radius_px=70*factor, outer_radius_px=140*factor,
               pixels=int(ring.sum()), mask_sha256=array_sha(ring)),
               common_valid_pixels=int(common.sum()), common_mask_sha256=array_sha(common), stages={}, deltas={})
    out['common_scope']='Intersection of all four stages in this same frame, not all-frame temporal coverage.'
    reference = None
    for name in STAGES:
        point = points[name]
        _, own = fit_audit(point[ring & valid[name]], cfg)
        model, matched = fit_audit(point[common], cfg)
        if name == 'pre_stationary_depth':
            reference = model
        current = dict(valid_ring_pixels=int(np.count_nonzero(ring & valid[name])),
            valid_ring_fraction=float(np.mean(valid[name][ring])), original_initial_support=initial_support(point,valid[name],radius,u,v,cfg),
            fixed_ring_own_coverage_fit=own, fixed_ring_common_coverage_fit=matched)
        if reference is not None:
            current['common_residual_to_pre_model_mm'] = quantiles(abs(point[common]@reference[:3]+reference[3])*1000)
        out['stages'][name] = current
    pre = rectified['pre_stationary_depth']
    for name in STAGES[1:]:
        other = rectified[name]
        out['deltas']['pre__'+name] = dict(
            common_pixels=int(common.sum()), common_signed_z_delta_mm=quantiles((other[common].astype(float)-pre[common])*scale*1000),
            ring_created_nonzero=int(np.count_nonzero(ring & (pre == 0) & (other > 0))),
            ring_removed_nonzero=int(np.count_nonzero(ring & (pre > 0) & (other == 0))))
    return out


def continuity_audit(summary, rows):
    """Check stored cumulative counters without inventing unsaved reads."""
    phases = {phase['label']:phase for phase in summary['matched_stationary_shadow']['phases']}
    if len(phases) != len(summary['matched_stationary_shadow']['phases']):
        raise ValueError('duplicate shadow phase labels')
    for phase in phases.values():
        if (sum(phase['reasons'].values()) != phase['reads']
                or not 0 <= phase['eligible_reads'] <= phase['reads']
                or any(type(v) is not int or v < 0 for field in ('reasons','resets') for v in phase[field].values())
                or not re.fullmatch('[0-9a-f]{64}', phase['chain_sha256'])):
            raise ValueError('shadow phase count or hash format contradiction')
    previous={};selected=[]
    for row in rows:
        record=row['shadow'].get('recorded',{});snapshot=record.get('phase')
        if snapshot is None:
            continue
        label=row['phase'];final=phases[label]
        if (snapshot['label'] != label or not 0 < snapshot['reads'] <= final['reads']
                or sum(snapshot['reasons'].values()) != snapshot['reads']
                or not 0 <= snapshot['eligible_reads'] <= snapshot['reads']
                or not re.fullmatch('[0-9a-f]{64}',snapshot['chain_sha256'])):
            raise ValueError('selected shadow phase snapshot contradiction')
        for field in ('reasons','resets'):
            if any(v < 0 or v > final[field].get(k,0) for k,v in snapshot[field].items()):
                raise ValueError('selected cumulative counter exceeds final phase')
        prior=previous.get(label);delta=None
        if prior:
            if (snapshot['reads'] <= prior['phase']['reads'] or record['history_id'] < prior['history_id']
                    or any(snapshot[field].get(k,0) < v for field in ('reasons','resets') for k,v in prior['phase'][field].items())):
                raise ValueError('selected cumulative shadow audit moved backwards')
            delta=dict(successful_shadow_reads=snapshot['reads']-prior['phase']['reads'],
                sampled_depth_frame_number_difference=record['depth_source']['frame_number']-prior['depth_source']['frame_number'],
                history_id_change=record['history_id']-prior['history_id'],
                reason_increments={k:v-prior['phase']['reasons'].get(k,0) for k,v in snapshot['reasons'].items()},
                reset_increments={k:v-prior['phase']['resets'].get(k,0) for k,v in snapshot['resets'].items()})
        selected.append(dict(index=row['index'],phase=label,history_id=record['history_id'],kind=row['shadow']['kind'],since_previous_saved_sample=delta))
        previous[label]=record
    return dict(phase_counters=phases,selected_sample_changes=selected,full_chain_replayed=False,
        limitation='Per-read hash links and intermediate RGB are not saved. Sample frame-number differences are not acquisition drop counts. Visual rejection can drop temporal history without a reset counter; see reasons/history IDs. Phase-boundary reset belongs to the departing phase.')


def phase_summaries(rows):
    result = {}
    scopes = {'all_complete_camera_local_diagnostics':lambda r:True,
              'strict_source_qualified_diagnostics':lambda r:r['source']['source_qualified'],
              'shadow_history_eligible_diagnostics':lambda r:r['shadow']['history_filtered'],
              'strict_source_and_shadow_history':lambda r:r['source']['source_qualified'] and r['shadow']['history_filtered']}
    for phase in ('A1', 'B', 'A2'):
        phase_rows = [r for r in rows if r['phase'] == phase]
        item = dict(saved_rows=len(phase_rows), source_qualified=sum(r['source']['source_qualified'] for r in phase_rows),
            source_rejections=dict(Counter(reason for r in phase_rows for reason in r['source']['rejection_reasons'])),
            shadow_kinds=dict(Counter(r['shadow']['kind'] for r in phase_rows)), comparisons={})
        for scope, accepts in scopes.items():
            selected = [r for r in phase_rows if 'comparison' in r and accepts(r)]
            item['comparisons'][scope] = dict(rows=len(selected), stages={})
            for stage in STAGES:
                stats = [r['comparison']['stages'][stage] for r in selected]
                item['comparisons'][scope]['stages'][stage] = dict(
                    initial_support_passes=sum(s['original_initial_support']['passed'] for s in stats),
                    fixed_ring_fit_passes=sum(s['fixed_ring_own_coverage_fit']['fit_passed'] for s in stats),
                    valid_fraction=quantiles([s['valid_ring_fraction'] for s in stats]),
                    own_residual_p90_mm=quantiles([s['fixed_ring_own_coverage_fit']['abs_residual_mm']['p90'] for s in stats
                        if s['fixed_ring_own_coverage_fit'].get('abs_residual_mm') is not None]),
                    common_residual_p90_mm=quantiles([s['fixed_ring_common_coverage_fit']['abs_residual_mm']['p90'] for s in stats
                        if s['fixed_ring_common_coverage_fit'].get('abs_residual_mm') is not None]),
                    all_samples_common_residual_p90_mm=quantiles([r['all_samples_common_ring'][stage]['abs_residual_mm']['p90'] for r in selected
                        if r.get('all_samples_common_ring',{}).get(stage,{}).get('abs_residual_mm') is not None]))
        result[phase] = item
    return result


def analyze(capture, config_path=None):
    cv2.setNumThreads(1)
    folder = capture if (capture/'summary.json').exists() else capture/'stages'
    root = folder.parent
    summary, samples, integrity = verify_artifacts(folder)
    outer = json.loads((root/'result.json').read_text()) if (root/'result.json').exists() else None
    if outer is not None and 'sensor_summary' in outer and outer['sensor_summary'] != summary:
        raise ValueError('runner summary differs from capture summary')
    loaded = json.loads((folder/'loaded_modules.json').read_text())
    archived = root/'camera_preset_ab_observer.py'
    if not archived.is_file() or sha(archived) != loaded['wrapper_sha256']:
        raise ValueError('archived wrapper differs from loaded manifest')
    raw_config = json.loads(config_path.read_text()) if config_path else {}
    cfg = tabletop.Config(**{f.name:raw_config[f.name] for f in fields(tabletop.Config) if f.name in raw_config})
    if cfg.support_threshold_m != .002 or cfg.support_min_ratio != .70:
        raise ValueError('expected unchanged 2mm / 70 percent support gates')
    report = dict(schema=1, capture=str(capture), script_sha256=sha(__file__), artifact_integrity=integrity,
        runner_receipt=dict(available=outer is not None,sha256=sha(root/'result.json') if outer is not None else None,
            captured_summary_crosschecked=bool(outer is not None and 'sensor_summary' in outer),
            experiment_complete=outer.get('experiment_complete') if outer else None,
            visual_nodes_restored=outer.get('visual_nodes_restored') if outer else None,
            error=outer.get('error') if outer else None,
            scope='Runner failure does not erase independently saved wrapper evidence or establish consumer recovery.'),
        loaded_modules_sha256=sha(folder/'loaded_modules.json'), archived_wrapper_sha256=sha(archived),
        source_implementation_sha256={str(Path(module.__file__).resolve()):sha(module.__file__) for module in (tabletop,sys.modules[PairedPinholeRectification.__module__])},
        config=asdict(cfg), config_source=str(config_path) if config_path else 'Current code defaults, not a captured online parameter snapshot',
        config_sha256=sha(config_path) if config_path else None,
        protocol=dict(experiment_valid=summary['experiment_valid'],capture_complete=summary['capture_complete'],
            counts=summary['counts'],errors=summary['errors'],guard_faults=summary['guard_faults'],
            restoration=summary['restoration'],shadow_phase_audits=summary['matched_stationary_shadow']['phases']),
        geometry_authority=False,target_identity_or_grasp_geometry_evaluated=False,rows=[],
        limitations=['No old object reference or target mask is loaded. Fixed image rings can include current clutter; no fitted plane is independently established as the true tabletop.',
            'Each stage is mapped with this capture\'s immutable SDK paired rectification into the identical ideal pinhole canvas. No same-index comparison between 640x480 aligned and 653x489 final rasters.',
            'Original camera clipping is reproduced in integer depth units and applied equally before stage comparison. SDK aligned axial depth convention remains unchanged.',
            'Strict TF/feedback source rejection is preserved separately from camera-local diagnostic measurements. No exposure-time synchronization or physical depth ground truth is inferred.',
            'A stored first-history frame is a passthrough, not temporal smoothing evidence. Inactive shadow output is also current-frame passthrough.',
            'Only selected frames and cumulative counters/hashes were saved. Missing intermediate RGB frames prevent full replay or hash-chain verification; sample frame-number gaps do not count as acquisition drops.',
            'Common valid pixels are conditional on surviving all compared stages within that frame; own coverage is separately reported. No per-pixel temporal median or filled geometry is created.',
            'New target/view results concern only this capture; they cannot retroactively qualify old A/B records. These summaries are not online support/target lifecycle or grasp authorization.'])
    mapper = None
    all_frame_common=None
    compared_stems=[]
    for stem in samples:
        info, arrays = read_sample(folder, stem)
        row = dict(index=info['sample_index'],phase=info['phase'],source=source_classification(info,summary),
                   shadow=shadow_classification(info,arrays),production_temporal=info.get('runtime_profile',{}).get('stationary_temporal'))
        report['rows'].append(row)
        if not row['source']['original_pair_complete'] or any(name not in arrays for name in (*STAGES,'raw_rgb','final_rgb')):
            row['comparison_excluded']='Original complete same-frame stage set unavailable'
            continue
        if row['shadow']['contradictions']:
            row['comparison_excluded']='Shadow provenance or value contradiction; not treated as matched output'
            continue
        if mapper is None:
            sys.path.insert(0,str(WORK/'.runtime/realsense-2.56.5'))
            import pyrealsense2 as rs
            mapper=PairedPinholeRectification(info['runtime_profile']['color_intrinsics'],sdk=rs)
            report['map_contract']=mapper.contract
        rectified=mapped_stages(info,arrays,mapper)
        k=[mapper.output_intrinsics[key] for key in ('fx','fy','cx','cy')]
        row['original_final_rgb_depth_reconstructed_exactly']=True
        row['comparison']=compare_stages(rectified,info['depth_scale_m'],k,cfg)
        valid=[(depth*info['depth_scale_m'] >= cfg.depth_min_m)&(depth*info['depth_scale_m'] <= cfg.depth_max_m) for depth in rectified.values()]
        common=np.logical_and.reduce(valid)
        all_frame_common=common if all_frame_common is None else all_frame_common & common
        compared_stems.append((stem,row))
    # A second bounded pass matches A/B pixel coverage without retaining all
    # depth frames in memory. This is conditional diagnostic coverage only.
    if all_frame_common is not None:
        h,w=all_frame_common.shape;v,u=np.indices((h,w));factor=min(w/640.,h/480.)
        radius=np.hypot(u-w/2.,v-h/2.)
        all_frame_common &= (radius>=70*factor)&(radius<=140*factor)
        report['all_compared_samples_common_ring']=dict(pixels=int(all_frame_common.sum()),mask_sha256=array_sha(all_frame_common),
            included_sample_indices=[row['index'] for _,row in compared_stems],
            scope='Same rectified pixel positions positive/in-range in every compared stage and complete sample. Conditional surviving subset; not representative of B-only depth or independent scene stationarity proof.')
        for stem,row in compared_stems:
            info,arrays=read_sample(folder,stem)
            rectified=mapped_stages(info,arrays,mapper)
            row['all_samples_common_ring']={}
            for name,depth in rectified.items():
                points=tabletop._project_pixels(depth[all_frame_common].astype(np.float32)*info['depth_scale_m'],
                    u[all_frame_common],v[all_frame_common],k)
                _,row['all_samples_common_ring'][name]=fit_audit(points,cfg)
    report['continuous_shadow_audit']=continuity_audit(summary,report['rows'])
    report['phase_summary']=phase_summaries(report['rows'])
    return report


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--capture',type=Path,required=True)
    parser.add_argument('--config',type=Path,help='Optional captured /unknown_perception JSON; never old target records')
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args(argv)
    if not MOUNT.is_mount() or EXT not in args.output.resolve().parents:
        raise ValueError('analysis output must be on mounted SanStand; no fallback')
    if args.output.exists():
        raise ValueError('new output path required; do not overwrite evidence')
    report=analyze(args.capture,args.config)
    with args.output.open('x') as stream:
        json.dump(report,stream,ensure_ascii=False,allow_nan=False,indent=2)
        stream.write('\n')
    print(json.dumps(dict(output=str(args.output),sha256=sha(args.output),phase_summary=report['phase_summary']),ensure_ascii=False))


if __name__=='__main__':
    main()
