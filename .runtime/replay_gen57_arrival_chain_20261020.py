"""Four exact GEN57 recorded sources through frozen pure arrival/station code.

This is an offline counterfactual, never a ROS owner/ACK/clock qualification.
Only after three real geometric confirmations may the fourth source exercise
the qualified-station pure recovery branch against an immutable third image.
No acknowledgement or node state is fabricated. The production source tree is
copied before imports, hash checked, and never changed by this script.
"""
import argparse
import ast
from dataclasses import asdict, fields
import hashlib
import json
import os
import platform
from pathlib import Path
import resource
import sys
import time
import traceback
from types import SimpleNamespace

W = Path(__file__).resolve().parents[1]
E = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
DEFAULT_INPUT = E/'ros_log/full_latest_gen41_restart_20261018T011222Z/gen57_resume_r3/actual_attempt_r1_geometry/rgbd_inputs_r4'
INPUT_MANIFEST_SHA = 'e2af878e098dd68953f9663f2d0070fce04250ed34b50e9ef78199882bdb86bb'
ANCHOR = 1792548314546731710
REFERENCE = 1792548329816028356
ARRIVALS = (1792548361880352258, 1792548362147817611, 1792548362414543151)
STATION = 1792548362680734872
REFERENCE_MASK_SHA = '6c93eef7e7277329810d798d30b410c08ea0dd92a6e7fedff406afc02fb61b24'
PACKAGES = ('src/alicia_grasp_modes/src/alicia_grasp_modes',
            'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp')
NODE = 'src/alicia_grasp_modes/scripts/unknown_tabletop_perception.py'
OUTPUT = None
BOUNDARY_ATTEMPTS = []


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def digest_bytes(value):
    return hashlib.sha256(value).hexdigest()


def inside(path, root):
    try:
        Path(path).resolve().relative_to(root)
        return True
    except (ValueError, TypeError):
        return False


def audit(event, args):
    def relative_path(path, fd=None):
        path = Path(os.fsdecode(path))
        if not path.is_absolute() and isinstance(fd, int) and fd >= 0:
            path = Path('/proc/self/fd')/str(fd)/path
        return path
    reason = None
    if event.startswith('socket.') or event in ('subprocess.Popen', 'os.system',
            'os.posix_spawn', 'os.posix_spawnp', 'os.fork', 'os.forkpty',
            'os.exec', 'pty.spawn'):
        reason = 'external process/network boundary'
    elif event == 'open' and isinstance(args[0], (str, bytes, os.PathLike)):
        path, mode, flags = args
        write_flags = os.O_WRONLY | os.O_RDWR | os.O_CREAT | os.O_TRUNC | os.O_APPEND
        writable = (isinstance(mode, str) and any(c in mode for c in 'wax+')) or (
            isinstance(flags, int) and flags & write_flags)
        if writable and not inside(os.fsdecode(path), OUTPUT):
            reason = 'write outside output'
    elif event in ('os.remove', 'os.rmdir', 'os.mkdir', 'os.chmod', 'os.chown',
                    'os.utime', 'os.truncate'):
        directory_fd = args[2] if event == 'os.mkdir' and len(args) > 2 else (
            args[1] if event in ('os.remove', 'os.rmdir') and len(args) > 1 else None)
        if not inside(relative_path(args[0], directory_fd), OUTPUT):
            reason = 'mutation outside output'
    elif event in ('os.rename', 'os.link', 'os.symlink'):
        if not all(inside(os.fsdecode(p), OUTPUT) for p in args[:2]):
            reason = 'cross-output mutation'
    if reason:
        BOUNDARY_ATTEMPTS.append(dict(event=event, reason=reason))
        raise RuntimeError('offline boundary rejected: ' + event + ': ' + reason)


def plain(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, dict):
        return {str(k): plain(v) for k, v in value.items()}
    if isinstance(value, (tuple, list)):
        return [plain(v) for v in value]
    return value


def save(name, value):
    path = OUTPUT/name
    with path.open('x') as stream:
        json.dump(plain(value), stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write('\n'); stream.flush(); os.fsync(stream.fileno())
    return dict(path=str(path), sha256=sha(path))


def array_sha(value):
    return digest_bytes(np.ascontiguousarray(value).tobytes())


def mask_summary(mask):
    selected = (np.asarray(mask) > 0).astype(np.uint8)
    return dict(pixels=int(np.count_nonzero(selected)), binary_sha256=array_sha(selected),
                raw_sha256=array_sha(mask), bbox_xywh=list(cv2.boundingRect(selected)))


def candidate_summary(candidate):
    return dict(position_base=candidate.position_base, extent_m=candidate.extent_m,
        quality=candidate.quality, bbox=candidate.bbox, metrics=candidate.metrics,
        support_footprint_base=candidate.support_footprint_base, mask=mask_summary(candidate.mask))


def namespace(value):
    return SimpleNamespace(**{k: namespace(v) if isinstance(v, dict) else v
                              for k, v in value.items()})


def immutable(value):
    array = np.ascontiguousarray(value)
    return np.frombuffer(array.tobytes(), dtype=array.dtype).reshape(array.shape)


def freeze_sources(root):
    names = [NODE]
    names += [str(p.relative_to(root)) for folder in PACKAGES for p in sorted((root/folder).rglob('*.py'))]
    names = sorted(set(names))
    contents = {name: (root/name).read_bytes() for name in names}
    pins = {name: digest_bytes(data) for name, data in contents.items()}
    if not all(sha(root/name) == value for name, value in pins.items()):
        raise RuntimeError('source changed during snapshot; run again with a stable source')
    snapshot = OUTPUT/'source_snapshot'
    for name, content in contents.items():
        target = snapshot/name; target.parent.mkdir(parents=True, exist_ok=True)
        with target.open('xb') as stream:
            stream.write(content)
        assert sha(target) == pins[name]
    return snapshot, pins, contents[NODE]


def import_frozen(snapshot):
    global np, cv2, table, refine_candidate, _appearance, _mask, FrozenObservationAppearance
    global ObservationArrivalConfirmation, fragment, FrozenMeasuredFaceSource
    global MeasuredFaceUnavailable, freeze_measured_face
    # Python 3.8 platform.uname otherwise shells out to `uname -p` while
    # importing NumPy's test helpers through SciPy. Fill only platform metadata
    # from the actual kernel uname syscall; no subprocess is needed or allowed.
    kernel = os.uname()
    platform._uname_cache = platform.uname_result(kernel.sysname, kernel.nodename,
        kernel.release, kernel.version, kernel.machine, kernel.machine)
    sys.path[:0] = [str(snapshot/str(Path(folder).parent)) for folder in PACKAGES]
    import numpy as np
    import cv2
    from alicia_grasp_modes import tabletop as table
    from alicia_grasp_modes.rgb_instance import refine_candidate
    from alicia_grasp_modes.rgb_observation_handoff import _appearance, _mask
    from alicia_grasp_modes.observation_appearance import FrozenObservationAppearance
    from alicia_grasp_modes.observation_arrival_confirmation import ObservationArrivalConfirmation
    import alicia_grasp_modes.arrival_fragment_group as fragment
    from alicia_grasp_modes.observation_measured_face import (
        FrozenMeasuredFaceSource, MeasuredFaceUnavailable, freeze_measured_face)
    cv2.setNumThreads(1)


def main(args):
    global OUTPUT
    OUTPUT = args.output.resolve()
    assert Path('/media/zhuyupei/SanStand').is_mount(), 'SanStand is not mounted'
    assert sys.flags.no_user_site, 'Run with PYTHONNOUSERSITE=1 (same dependency isolation as formal regression)'
    assert inside(OUTPUT, E) and OUTPUT != E and not OUTPUT.exists()
    OUTPUT.mkdir(parents=True, exist_ok=False)
    sys.dont_write_bytecode = True
    sys.addaudithook(audit)
    (OUTPUT/'runner_source.py').write_bytes(Path(__file__).read_bytes())
    started = time.monotonic(); usage = resource.getrusage(resource.RUSAGE_SELF)
    snapshot, source_pins, node_bytes = freeze_sources(args.source_root.resolve(strict=True))
    import_frozen(snapshot)
    source_manifest = save('source_manifest.json', dict(source_root=str(args.source_root.resolve()),
        snapshot_root=str(snapshot), source_sha256=source_pins,
        runner_path=str(Path(__file__).resolve()), runner_sha256=sha(__file__),
        runtime_dependencies={name: dict(version=getattr(sys.modules[name], '__version__', None),
            path=getattr(sys.modules[name], '__file__', None)) for name in ('numpy', 'cv2', 'scipy')},
        python_no_user_site=True))
    inputs = args.inputs.resolve(strict=True)
    assert sha(inputs/'manifest.json') == INPUT_MANIFEST_SHA
    manifest = json.loads((inputs/'manifest.json').read_text())
    rows = {r['source_stamp_ns']: r for r in manifest['rows']}
    input_paths = [inputs/name for name in ('manifest.json', 'anchor_status.json',
        'frozen_reference_status.json', 'unknown_perception_configuration.json')]
    for stamp in (ANCHOR, REFERENCE) + ARRIVALS + (STATION,):
        row = rows[stamp]; path = Path(row['path']).resolve(strict=True)
        assert inside(path, inputs) and sha(path) == row['npz_sha256']
        input_paths.append(path)
        if stamp in ARRIVALS + (STATION,):
            assert row['online_evaluated'] is True
    input_pins = {str(path): sha(path) for path in input_paths}
    save('input_manifest.json', dict(source_stamps_ns=[ANCHOR, REFERENCE, *ARRIVALS, STATION],
        input_sha256=input_pins, input_arrays_immutable=True,
        exact_recorded_pairs=True, nearest_source_substitution=False))
    tree = ast.parse(node_bytes)
    transform_node = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'transform_matrix')
    scope = {'np': np}
    exec(compile(ast.Module(body=[transform_node], type_ignores=[]), str(snapshot/NODE), 'exec'), scope)
    transform_matrix = scope['transform_matrix']; transform_deltas = {}
    def load(stamp):
        row = rows[stamp]
        with np.load(row['path'], allow_pickle=False) as archive:
            data = {k: immutable(archive[k]) for k in archive.files}
        transform = transform_matrix(namespace(row['exact_tf']))
        transform_deltas[str(stamp)] = float(np.max(np.abs(transform-data['T_base_camera'])))
        data['T_base_camera'] = immutable(transform)
        data['depth'] = immutable(np.asarray(data['depth_raw'], dtype=np.float32)*float(data['depth_scale']))
        return data, ({} if 'rgb_valid' not in data else dict(rgb_valid=data['rgb_valid']))
    params = json.loads((inputs/'unknown_perception_configuration.json').read_text())
    config = table.Config(**{k: v for k, v in params.items() if k in {f.name for f in fields(table.Config)}})
    minimum = float(params.get('minimum_geometry_quality', .5))
    anchor_status = json.loads((inputs/'anchor_status.json').read_text())['data']
    reference_status = json.loads((inputs/'frozen_reference_status.json').read_text())['data']
    data, kwargs = load(ANCHOR)
    segmentation = table.segment_with_support_fallback(data['depth'], data['K'], data['T_base_camera'], config)
    anchor, reason = table.TargetTracker(config).choose(segmentation, data['depth'].shape)
    assert anchor is not None, reason
    plane = immutable(segmentation.plane_base)
    anchor_checks = {}
    for name, actual, expected in (
            ('position', anchor.position_base, anchor_status['anchor_position_base']),
            ('extent', anchor.extent_m, anchor_status['anchor_extent_m']),
            ('plane', plane, anchor_status['reference_support_plane_base']),
            ('footprint', anchor.support_footprint_base, anchor_status['reference_support_footprint_base'])):
        anchor_checks[name] = dict(max_abs_delta=float(np.max(np.abs(actual-expected))),
            equal_at_1e_10=bool(np.allclose(actual, expected, atol=1e-10, rtol=0)))
    assert all(row['equal_at_1e_10'] for row in anchor_checks.values())
    save('anchor_reconstruction.json', dict(checks=anchor_checks, candidate=candidate_summary(anchor), config=asdict(config)))
    data, kwargs = load(REFERENCE)
    assert reference_status['geometry_metrics']['support_refit_reason'] == 'qualified_current_support_measurement'
    segmentation = table.segment_with_support_fallback(data['depth'], data['K'], data['T_base_camera'],
        config, anchor, anchor, plane, prefer_current_support=True)
    matching = table._matching_candidates(segmentation.candidates, anchor, anchor, config)
    assert len(matching) == 1
    last = matching[0][1]
    refined = refine_candidate(data['color'], data['depth'], data['K'], data['T_base_camera'], last, config, **kwargs)
    recorded = data['recorded_refined_mask']
    reference_recomputation_bit_equal = bool(np.array_equal(refined.mask, recorded))
    # The online hold stores _mask's normalized uint8 representation.
    historical_mask = immutable(_mask(recorded, data['color'].shape[:2]))
    whole_mask = immutable(_mask(recorded if args.reference_mode == 'recorded' else refined.mask,
                                 data['color'].shape[:2]))
    histogram, contour = _appearance(data['color'], whole_mask)
    measurement = reference_status['initial_station_measurement']
    assert measurement['source_stamp_ns'] == REFERENCE
    assert array_sha(historical_mask) == measurement['refined_mask_sha256'] == REFERENCE_MASK_SHA
    frozen_mask_sha = array_sha(whole_mask)
    measured_source = FrozenMeasuredFaceSource(data['depth_raw'], rows[REFERENCE]['depth_encoding'],
        float(data['depth_scale']), data['K'], data['T_base_camera'], plane, config, REFERENCE,
        measurement['source_id'], array_sha(data['color']), array_sha(whole_mask))
    frozen_data_pins = {key: array_sha(value) for key, value in data.items()}
    def check_reference_inputs():
        assert all(array_sha(data[key]) == value for key, value in frozen_data_pins.items())
    # Real monotonic watchdog bounds offline work; it is not the historical source lifetime.
    measured_face = None; face_error = None; face_start = time.monotonic()
    try:
        measured_face = freeze_measured_face(measured_source, data['color'], whole_mask,
            kwargs.get('rgb_valid', np.ones(whole_mask.shape, bool)), check_current=check_reference_inputs,
            deadline_monotonic=face_start+args.compute_timeout_seconds, clock=time.monotonic)
    except MeasuredFaceUnavailable as exc:
        face_error = str(exc)
    frozen = FrozenObservationAppearance(data['color'], whole_mask, kwargs.get('rgb_valid'),
        REFERENCE, histogram, contour, measured_face=measured_face)
    assert frozen.mask_sha256 == frozen_mask_sha
    reference_report = dict(reference_source_stamp_ns=REFERENCE,
        reference_mode=args.reference_mode,
        reference_mask_frozen_from_original_recorded_bytes=args.reference_mode == 'recorded',
        reconstructed_reference_is_counterfactual=args.reference_mode == 'reconstructed',
        current_source_recomputed_reference_bit_equal=reference_recomputation_bit_equal,
        recorded_mask=mask_summary(recorded), normalized_mask=mask_summary(whole_mask),
        raw_last=candidate_summary(last), refined=candidate_summary(refined),
        measured_face_available=measured_face is not None, measured_face_unavailable_reason=face_error,
        measured_face_source_binding_sha256=measured_source.binding_sha256,
        measured_face_audit=None if measured_face is None else json.loads(measured_face.audit_json),
        measured_face_wall_seconds=time.monotonic()-face_start,
        historical_reference_lifetime_not_replayed=True)
    save('reference_reconstruction.json', reference_report)
    conf = ObservationArrivalConfirmation(config, anchor, last, plane, histogram, contour, minimum,
                                           appearance_reference=frozen)
    original_anchor = json.dumps(plain(candidate_summary(anchor)), sort_keys=True)
    records = []; final_candidate = None; final_staged = None; third_data = None
    stage_profile = []
    watched = {'refine_candidate', 'recover_arrival_fragment_group', 'recover_arrival_matching_fragments'}
    def profile(frame, event, returned):
        if event != 'return' or frame.f_code.co_name not in watched or not inside(frame.f_code.co_filename, snapshot):
            return
        entry = dict(function=frame.f_code.co_name, returned_normally=returned is not None)
        for key in ('proposal', 'refined', 'candidate'):
            value = frame.f_locals.get(key)
            if isinstance(value, table.Candidate):
                entry[key] = candidate_summary(value)
        if isinstance(returned, table.Candidate):
            entry['result'] = candidate_summary(returned)
        stage_profile.append(entry)
    def compute_arrival(stamp, role):
        nonlocal final_candidate, final_staged, third_data
        frame, rgb_kwargs = load(stamp); begin = time.monotonic(); stage_profile.clear()
        segmentation = table.segment_with_support_fallback(frame['depth'], frame['K'], frame['T_base_camera'],
            config, anchor, conf.last, plane, prefer_current_support=True)
        entry = dict(role=role, source_stamp_ns=stamp, exact_original_online_source=True,
            source_npz_sha256=rows[stamp]['npz_sha256'], support_metrics=segmentation.metrics,
            native_candidates=[candidate_summary(c) for c in segmentation.candidates],
            metric_match_count=len(table._matching_candidates(segmentation.candidates, anchor, conf.last, config)))
        def check_frame():
            assert frame['color'].flags.writeable is False and frame['depth_raw'].flags.writeable is False
            assert conf.anchor is anchor and conf.appearance_reference is frozen
        sys.setprofile(profile)
        try:
            confirmed, candidate, staged, evidence = conf.observe(segmentation, frame['color'], frame['depth'],
                frame['K'], frame['T_base_camera'], stamp,
                appearance_deadline_monotonic=begin+args.compute_timeout_seconds,
                appearance_check_current=check_frame, **rgb_kwargs)
            entry.update(completed=True, confirmed=confirmed, evidence=evidence, candidate=candidate_summary(candidate))
            if stamp == ARRIVALS[-1] and confirmed:
                final_candidate, final_staged, third_data = candidate, staged, frame
        except ValueError as exc:
            entry.update(completed=False, confirmed=False, rejection=str(exc))
        finally:
            sys.setprofile(None)
        entry.update(appearance_identity=conf.last_appearance_evidence,
            confirmation_source_stamps_ns=list(conf.source_stamps_ns),
            pure_stage_returns=list(stage_profile), wall_seconds=time.monotonic()-begin)
        records.append(entry); save(str(stamp)+'_result.json', entry)
        print(json.dumps(dict(source_stamp_ns=stamp, confirmed=entry['confirmed'],
            rejection=entry.get('rejection'), confirmation_sources=entry['confirmation_source_stamps_ns'])), flush=True)
    for stamp in ARRIVALS:
        compute_arrival(stamp, 'arrival_confirmation')
    station_report = dict(source_stamp_ns=STATION, attempted=False, pure_geometry_passed=False,
        runtime_qualified=False, ack_handoff_executed=False, runtime_marker_created=False)
    if final_candidate is None:
        station_report['reason'] = 'three_frame_confirmation_not_completed'
        compute_arrival(STATION, 'arrival_diagnostic_only_station_prerequisite_failed')
    else:
        assert conf.source_stamps_ns == list(ARRIVALS)
        third_mask = _mask(final_candidate.mask, third_data['color'].shape[:2])
        station_hist, station_contour = _appearance(third_data['color'], third_mask)
        template = FrozenObservationAppearance(third_data['color'], third_mask,
            third_data.get('rgb_valid'), ARRIVALS[-1], station_hist, station_contour)
        frame, rgb_kwargs = load(STATION); station_evidence = []; stage_profile.clear()
        station_report.update(attempted=True, template_source_stamp_ns=template.source_ns,
            template_mask_sha256=template.mask_sha256,
            template_is_offline_counterfactual=True, premise='Three pure geometry confirmations; no runtime qualified ACK exists.')
        begin = time.monotonic()
        def check_station_inputs():
            assert template.source_ns == ARRIVALS[-1] and final_staged.anchor is anchor
        context = dict(reference=template, source_ns=STATION, check_current=check_station_inputs,
            record=lambda evidence: station_evidence.append(evidence),
            deadline_monotonic=begin+args.compute_timeout_seconds,
            same_station_mask_iou_minimum=.85, reference_role='qualified_station_identity_template')
        sys.setprofile(profile)
        try:
            segmentation = table.segment_with_support_fallback(frame['depth'], frame['K'], frame['T_base_camera'],
                config, anchor, final_staged.last, plane, prefer_current_support=True)
            matches = table._matching_candidates(segmentation.candidates, anchor, final_staged.last, config)
            station_report['original_metric_matches'] = len(matches)
            station_report['support_metrics'] = segmentation.metrics
            recovery = (fragment.recover_arrival_fragment_group if not matches else
                fragment.recover_arrival_matching_fragments if len(matches) > 1 else None)
            pair = None
            if recovery is not None:
                segmentation, pair = recovery(segmentation, frame['color'], frame['depth'], frame['K'],
                    frame['T_base_camera'], config, anchor, final_staged.last, conf.spatial_reference,
                    station_hist, station_contour, minimum, appearance_context=context, **rgb_kwargs)
            staged = table.TargetTracker(config)
            staged.anchor, staged.last, staged.plane_base = anchor, final_staged.last, plane.copy()
            raw, reason = staged.choose(segmentation, frame['depth'].shape)
            if raw is None or raw.quality < minimum:
                raise ValueError('station_metric_choice_rejected: '+reason)
            refined_station = pair[1] if pair is not None and raw is pair[0] else refine_candidate(
                frame['color'], frame['depth'], frame['K'], frame['T_base_camera'], raw, config,
                observation_arrival_retry=True, observation_spatial_reference=conf.spatial_reference, **rgb_kwargs)
            if not np.isfinite(refined_station.quality) or refined_station.quality < minimum:
                raise ValueError('station_refined_quality_insufficient')
            station_report.update(pure_geometry_passed=True, raw_candidate=candidate_summary(raw),
                refined_candidate=candidate_summary(refined_station), recovery_branch=None if recovery is None else recovery.__name__)
        except ValueError as exc:
            station_report['rejection'] = str(exc)
        finally:
            sys.setprofile(None)
        station_report.update(appearance_identity=station_evidence, pure_stage_returns=list(stage_profile),
            wall_seconds=time.monotonic()-begin)
    save('station_result.json', station_report)
    assert json.dumps(plain(candidate_summary(anchor)), sort_keys=True) == original_anchor
    assert np.array_equal(conf.plane, plane) and frozen.mask_sha256 == frozen_mask_sha
    assert all(sha(path) == value for path, value in input_pins.items())
    assert all(sha(snapshot/name) == value for name, value in source_pins.items())
    imports = {}
    for name, module in tuple(sys.modules.items()):
        if name.startswith(('alicia_grasp_modes', 'alicia_flexible_grasp')) and getattr(module, '__file__', None):
            path = Path(module.__file__).resolve()
            assert inside(path, snapshot), 'foreign project import: '+str(path)
            imports[name] = dict(path=str(path), sha256=sha(path))
    save('module_provenance.json', imports)
    save('boundary_audit.json', dict(attempts=BOUNDARY_ATTEMPTS, audit_hook_enabled=True,
        input_writes_blocked=True, output_only_writes=True, network_and_process_creation_blocked=True,
        python_audit_is_not_a_kernel_sandbox=True))
    after = resource.getrusage(resource.RUSAGE_SELF)
    changed = [name for name, value in source_pins.items() if sha(args.source_root/name) != value]
    report = dict(scope='Exact recorded four-source pure computation; no ROS, runtime clocks, ACK or motion.',
        completed=True, source_manifest=source_manifest, input_sha256=input_pins,
        source_snapshot_verified=True, original_workspace_changed_since_snapshot=changed,
        anchor_unchanged=True, reference_mask_unchanged=True, input_hashes_unchanged=True,
        reference=reference_report, rows=records, station=station_report,
        three_frame_geometry_confirmed=final_candidate is not None,
        full_runtime_handoff_verified=False, runtime_qualification_verified=False,
        all_boundary_attempts=BOUNDARY_ATTEMPTS, transform_max_abs_deltas=transform_deltas,
        original_clocks_replayed=False, original_capture_deadline_verified=False,
        offline_watchdog_seconds=args.compute_timeout_seconds,
        wall_seconds=time.monotonic()-started, user_cpu_seconds=after.ru_utime-usage.ru_utime,
        system_cpu_seconds=after.ru_stime-usage.ru_stime, max_rss_kib=after.ru_maxrss,
        limitations=['Only four original online sources have complete exact RGB-D here; eight other online sources are absent.',
            'No synthetic ACK, source timestamp, owner receipt, TF lookup, stationarity certificate or runtime marker is issued.',
            'The fourth-source station computation is conditional on a counterfactual completed handoff, not proof of that handoff.',
            'The pure station branch does not run node publication, station registration, remote prediction, task or contact commit.',
            'The monotonic watchdog bounds this offline computation and does not replay historical source freshness.'])
    print(json.dumps(save('result.json', report)), flush=True)
    return 0


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--inputs', type=Path, default=DEFAULT_INPUT)
    parser.add_argument('--source-root', type=Path, default=W)
    parser.add_argument('--reference-mode', choices=('recorded', 'reconstructed'), default='recorded')
    parser.add_argument('--compute-timeout-seconds', type=float, default=120.)
    arguments = parser.parse_args()
    assert 0. < arguments.compute_timeout_seconds <= 300.
    try:
        raise SystemExit(main(arguments))
    except Exception as exc:
        if OUTPUT is not None and OUTPUT.exists() and 'np' in globals():
            save('failure.json', dict(error=repr(exc), traceback=traceback.format_exc(),
                boundary_attempts=BOUNDARY_ATTEMPTS, full_runtime_handoff_verified=False))
        raise
