"""One finite offline comparison of three original initial-view RGB-D sources.

Isolated function clones change rectangle estimation, never production code or
gate thresholds. The two predeclared alternatives use endpoint least squares,
and optionally exact fractional pixel-cell area for the same 5x5 corner gate.
"""
import difflib
import hashlib
import inspect
import json
import math
import os
from pathlib import Path
import sys
import time
from types import SimpleNamespace as NS

W = Path(__file__).resolve().parents[1]
M = Path('/media/zhuyupei/SanStand')
N = M/'实验记录/Robot-Controller/ros_log/full_latest_gen22_restart_20261015T003700Z'
O = N/'verification/gen30_initial_visual_direction_full_r1'
IDS = (1792062749691942453, 1792062749893290758, 1792062750092055082)
assert M.is_mount() and not os.statvfs(M).f_flag & os.ST_RDONLY
assert not O.exists()
os.environ.update(ROS_MASTER_URI='http://127.0.0.1:1', PYTHONDONTWRITEBYTECODE='1',
                  OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1')
for relative in ('src/alicia_flexible_grasp_supervisor/src', 'devel/lib/python3/dist-packages'):
    sys.path.insert(0, str(W/relative))

import numpy as np
from alicia_flexible_grasp.vision import regular_box_inference as box
from alicia_flexible_grasp.vision.rgbd_snapshot import RgbdSample, fuse_stable_samples
from alicia_flexible_grasp.vision.object_geometry import estimate_object_geometry
from alicia_flexible_grasp.vision.target_observation import TargetTrackIdentity


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def arrsha(value):
    return hashlib.sha256(np.ascontiguousarray(value).tobytes()).hexdigest()


def serial(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    raise TypeError(type(value).__name__)


inputs = {}
def read(path):
    inputs[str(path)] = sha(path)
    return json.loads(Path(path).read_text())


prior = read(N/'verification/gen30_initial_view_box_replay_r1/result.json')
assert inputs[str(N/'verification/gen30_initial_view_box_replay_r1/result.json')] == '8334b360c1af652b735c4bf51b89e55a5a5b26ddd0ebc5ba9ce1d783c81f56ce'
axes_diagnosis = read(N/'verification/gen30_initial_visual_axes_r1/result.json')
assert tuple(prior['selected_original_sources']) == IDS
assert tuple(row['source_stamp_ns'] for row in axes_diagnosis['rows']) == IDS
source_paths = [Path(inspect.getsourcefile(module)) for module in
    (box, sys.modules[fuse_stable_samples.__module__], sys.modules[estimate_object_geometry.__module__])]
sources = {str(path): sha(path) for path in source_paths}
assert sha(source_paths[0]) == '9006b9b3b416cb7b5a43b58e9ab420e8d7294442235eaf7a95673f6188729c0f'
identity = TargetTrackIdentity(**prior['source_identity'])
arrays, samples = [], []
for ns, binding in zip(IDS, prior['frame_bindings']):
    assert binding['source_stamp_ns'] == ns
    path = Path(binding['input_npz'])
    inputs[str(path)] = sha(path)
    assert inputs[str(path)] == binding['input_sha256']
    with np.load(path, allow_pickle=False) as archive:
        a = {key: archive[key].copy() for key in archive.files}
    assert all(arrsha(a[key]) == digest for key, digest in binding['array_sha256'].items())
    assert int(a['source_stamp_ns']) == ns
    bbox = tuple(binding['bbox_xywh'])
    # Geometry does not inspect the ROS object message. Keep its original
    # detected/bbox facts without importing ROS or opening the archived bag.
    message = NS(detected=True, bbox_x=bbox[0], bbox_y=bbox[1],
                 bbox_width=bbox[2], bbox_height=bbox[3])
    q = np.asarray(binding['accepted_joint_bracket']['before']['positions_rad'])
    samples.append(RgbdSample(a['color_bgr'], a['depth_raw'], a['object_mask'], bbox,
        message, ns/1e9, 'camera_link', q, ns, identity.epoch, identity))
    arrays.append(a)
scale = float(arrays[-1]['depth_scale_m'])
k = arrays[-1]['intrinsics']
K = NS(fx=k[0], fy=k[1], cx=k[2], cy=k[3],
       width=arrays[-1]['depth_raw'].shape[1], height=arrays[-1]['depth_raw'].shape[0])
optical_to_link = np.eye(4)
optical_to_link[:3, :3] = [[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]]
T = arrays[-1]['T_base_camera_link'] @ optical_to_link
snapshot = fuse_stable_samples(samples, **prior['snapshot_arguments'])
assert snapshot.ok and tuple(snapshot.sample_stamp_ns) == IDS
geometry = estimate_object_geometry(snapshot.depth_raw, snapshot.target_depth_raw,
    snapshot.object_mask, snapshot.bbox, K, scale, T, snapshot.source_mode,
    **prior['geometry_arguments'])
assert geometry.ok
assert arrsha(geometry.object_points_base) == prior['geometry']['object_points_sha256']
for key in ('center_base', 'axes_base', 'size_xyz_m', 'support_normal_base'):
    np.testing.assert_array_equal(getattr(geometry, key), prior['geometry'][key])


def fractional_corner_coverage(mask, corner):
    """Area of original occupied pixel cells in the same centered 5x5 square."""
    x, y = map(float, corner)
    xs = np.arange(max(0, int(math.floor(x-3.))), min(mask.shape[1], int(math.ceil(x+3.))+1))
    ys = np.arange(max(0, int(math.floor(y-3.))), min(mask.shape[0], int(math.ceil(y+3.))+1))
    wx = np.maximum(0., np.minimum(xs+.5, x+2.5)-np.maximum(xs-.5, x-2.5))
    wy = np.maximum(0., np.minimum(ys+.5, y+2.5)-np.maximum(ys-.5, y-2.5))
    area = float(np.sum(mask[np.ix_(ys, xs)]*wy[:, None]*wx[None, :]))
    return min(1., area/(25.*.25))


def fitted_rectangle(selected, on_plane, original_axes):
    """Orthogonal LS to measured segment endpoints, independent edge offsets.

    Each line has a free normal offset. Minimizing squared perpendicular
    endpoint residuals leaves a 2x2 principal-axis problem. Equal endpoint
    measurement weights produce squared-length direction information; neither
    foreground occupancy nor any downstream pass/fail enters this fit.
    """
    segments = [on_plane(row[2]) @ original_axes[:, :2] for row in selected]
    directions = [points[1]-points[0] for points in segments]
    aligned = np.asarray([delta if i % 2 == 0 else [delta[1], -delta[0]]
                          for i, delta in enumerate(directions)])
    _, vectors = np.linalg.eigh(aligned.T @ aligned)
    direction = vectors[:, -1]
    if direction[0] < 0.:
        direction = -direction
    rotation = np.column_stack((direction, [-direction[1], direction[0]]))
    axes = original_axes.copy()
    axes[:, :2] = original_axes[:, :2] @ rotation
    rotated = [points @ rotation for points in segments]
    lo = np.array([rotated[3][:, 0].mean(), rotated[0][:, 1].mean()])
    hi = np.array([rotated[1][:, 0].mean(), rotated[2][:, 1].mean()])
    residuals, slopes = [], []
    for i, points in enumerate(rotated):
        across = 1 if i % 2 == 0 else 0
        delta = points[1]-points[0]
        residuals.extend(np.abs(points[:, across]-points[:, across].mean()).tolist())
        slopes.append(abs(float(delta[across]/delta[1-across])))
    audit = dict(rotation_from_depth_axis_deg=math.degrees(math.atan2(direction[1], direction[0])),
        selected_measured_lines_pixels=[row[2].tolist() for row in selected],
        endpoint_normal_residual_max_m=max(residuals),
        endpoint_normal_residual_rms_m=float(np.sqrt(np.mean(np.square(residuals)))),
        fitted_line_angular_residual_deg=np.degrees(np.arctan(slopes)).tolist(),
        uncertainty_not_less_than_original_selected_line_discrepancy=True)
    return axes, lo, hi, max(max(row[3] for row in selected), max(slopes)), max(residuals), audit


visual_original = inspect.getsource(box._visual_rectangle)
qualifier_original = inspect.getsource(box.qualify_regular_box_top)
def substitute(text, old, new):
    assert text.count(old) == 1, old
    return text.replace(old, new)


def clone_variant(fractional):
    visual = substitute(visual_original, '    candidates=[]\n',
        '    original_axes=axes.copy()\n    candidates=[]\n')
    visual = substitute(visual, '''        lo=np.array([selected[3][0],selected[0][0]])
        hi=np.array([selected[1][0],selected[2][0]])''', '''        axes,lo,hi,fit_slope,fit_residual,fit_audit=fitted_rectangle(selected,on_plane,original_axes)
        silhouette=on_plane(np.column_stack((xs,ys)).astype(float))@axes[:,:2]
        silhouette_lo,silhouette_hi=silhouette.min(axis=0),silhouette.max(axis=0)''')
    visual = substitute(visual, '                               max(row[3] for row in selected),unobserved_extension))',
        '                               fit_slope,unobserved_extension,axes.copy(),fit_audit,fit_residual))')
    if fractional:
        visual = substitute(visual, '            corner_cov.append(float(min(1.,np.mean(patch)/.25)))',
            '            corner_cov.append(fractional_corner_coverage(mask,corner))')
    qualifier = substitute(qualifier_original, '            lower,upper=visual[1],visual[2]',
        '            lower,upper=visual[1],visual[2]\n            axes=visual[7]\n            coord=top@axes[:,:2]')
    qualifier = substitute(qualifier, '        edge_uncertainty = max(.001, 2.*rms+scale) + .5*pixel_size + unobserved_extension',
        '        edge_uncertainty = max(.001, 2.*rms+scale) + .5*pixel_size + unobserved_extension + (visual[9] if visual is not None else 0.)')
    namespace = dict(vars(box), fitted_rectangle=fitted_rectangle,
        fractional_corner_coverage=fractional_corner_coverage)
    exec(compile(visual, '<isolated_visual_rectangle>', 'exec'), namespace)
    exec(compile(qualifier, '<isolated_regular_box_qualifier>', 'exec'), namespace)
    return namespace['qualify_regular_box_top'], namespace['_visual_rectangle'], visual, qualifier


def evaluate(qualifier, visual, a, ns):
    diagnostics = dict(combinations=[], visual_found=False)
    line_map = {}
    if visual is box._visual_rectangle:
        lines, first = inspect.getsourcelines(visual)
    else:
        text = variants[active_variant][2]
        lines, first = text.splitlines(), 1
    for key, needle in (('corner', 'if not good_candidate or min(corner_cov)<.5:'),
                        ('edge', 'if (visual_support < .70 or'),
                        ('accepted', 'if good_candidate and mask_boundary_count>=3:'),
                        ('start', 'if np.any(hi-lo < .005):')):
        line_map[key] = first + next(i for i, line in enumerate(lines) if line.strip() == needle)
    current = None
    def visual_trace(frame, event, value):
        nonlocal current
        v, line = frame.f_locals, frame.f_lineno
        if event == 'return':
            diagnostics['visual_found'] = value is not None
            if value is not None:
                diagnostics['selected_area_m2'] = value[0]
                if len(value) > 7:
                    diagnostics['selected_fit'] = value[8]
        if event != 'line':
            return visual_trace
        if line == line_map['start']:
            current = dict(index=len(diagnostics['combinations'])+1, edges=[], accepted=False)
            if 'fit_audit' in v:
                current['fit'] = v['fit_audit']
            diagnostics['combinations'].append(current)
        elif line == line_map['corner']:
            current.update(projected_corners_px=v['projected'].tolist(),
                corner_coverage=list(v['corner_cov']), corner_image_margin_ok=v['good_candidate'])
        elif line == line_map['edge']:
            row = {key: float(v[key]) for key in ('i', 'visual_support', 'valid_inside', 'supported',
                'valid_outside', 'continued', 'mask_border_fraction', 'mask_boundary_count')}
            row['higher_outside_fraction'] = float(np.mean(v['outside'][0] & (v['outside'][2] > .0025)))
            current['edges'].append(row)
        elif line == line_map['accepted']:
            current.update(accepted=bool(v['good_candidate'] and v['mask_boundary_count']>=3),
                mask_boundary_count=v['mask_boundary_count'])
        return visual_trace
    def trace(frame, event, value):
        if frame.f_code is visual.__code__:
            return visual_trace(frame, event, value)
        if frame.f_code is qualifier.__code__:
            if event == 'return':
                diagnostics['qualifier_has_visual'] = frame.f_locals.get('visual') is not None
            return trace
        return None
    previous = sys.gettrace()
    try:
        sys.settrace(trace)
        result = qualifier(a['depth_raw'], a['object_mask'], K, scale,
            a['T_base_camera_link'] @ optical_to_link, geometry, identity, ns, color_bgr=a['color_bgr'])
    finally:
        sys.settrace(previous)
    return dict(ok=result.ok, code=result.code, audit=dict(result.audit),
        model=result.model.to_evidence_dict() if result.model is not None else None,
        visual_diagnostics=diagnostics)


variants = {'original': (box.qualify_regular_box_top, box._visual_rectangle,
                         visual_original, qualifier_original),
    'endpoint_orthogonal_ls': clone_variant(False),
    'endpoint_orthogonal_ls_fractional_corner': clone_variant(True)}
started = time.monotonic()
rows = []
for ns, a, baseline in zip(IDS, arrays, prior['raw_frames_with_same_fused_geometry']):
    row = dict(source_stamp_ns=ns, variants={})
    for active_variant, variant in variants.items():
        result = evaluate(variant[0], variant[1], a, ns)
        row['variants'][active_variant] = result
        if active_variant == 'original':
            assert all(result[key] == baseline['qualification'][key] for key in ('ok', 'code', 'audit', 'model'))
        expected = len(next(item for item in axes_diagnosis['rows'] if item['source_stamp_ns']==ns)['combinations'])
        assert len(result['visual_diagnostics']['combinations']) == expected
    rows.append(row)
assert all(sha(path) == digest for path, digest in sources.items())
assert all(sha(path) == digest for path, digest in inputs.items())
assert arrsha(geometry.object_points_base) == prior['geometry']['object_points_sha256']
O.mkdir()
for name, variant in variants.items():
    if name == 'original':
        continue
    diff = ''.join(difflib.unified_diff(visual_original.splitlines(True), variant[2].splitlines(True),
        fromfile='production/_visual_rectangle', tofile=name+'/_visual_rectangle'))
    diff += ''.join(difflib.unified_diff(qualifier_original.splitlines(True), variant[3].splitlines(True),
        fromfile='production/qualify_regular_box_top', tofile=name+'/qualify_regular_box_top'))
    (O/(name+'.diff')).write_text(diff)
report = dict(classification='finite_isolated_full_qualifier_comparison_initial_three_sources',
    recorded_ns=time.time_ns(), selected_sources=list(IDS), rows=rows,
    method=dict(direction='orthogonal endpoint least squares with independent edge offsets',
        search='unchanged original top-three per-edge Hough choices, exactly 12/6/12 combinations',
        uncertainty='original angular discrepancy retained as floor, fitted endpoint normal residual added to existing positional budget',
        fractional_corner='exact original occupied-pixel-cell area within same 5x5 square; normalization and threshold unchanged',
        selection='unchanged largest area among candidates passing every original visual gate'),
    original_qualification_reproduced=True, reconstructed_geometry_exactly_matches_prior=True,
    input_sha256=inputs, source_sha256=sources, input_unchanged=True, source_unchanged=True,
    script_sha256=sha(__file__), elapsed_sec=time.monotonic()-started,
    production_modified=False, no_ros_calls=True, no_hardware_access=True,
    thresholds_changed=False, full_original_qualifier_gates_executed=True,
    motion_authorized=False, physical_grasp_verified=False,
    limitations=prior['limitations']+[
        'Isolated estimation alternatives have not been regression-tested or deployed.',
        'Corner foreground occupancy is a completeness consistency check, not a calibrated metric location-error bound.',
        'Full qualification is not a collision-path, physical opening, execution or real grasp validation.'])
(O/'result.json').write_text(json.dumps(report,ensure_ascii=False,indent=2,default=serial,allow_nan=False)+'\n')
print(json.dumps(dict(result=str(O/'result.json'),sha256=sha(O/'result.json'),
    rows=[dict(source=r['source_stamp_ns'],variants={k:dict(ok=v['ok'],code=v['code'],
        visual=v['visual_diagnostics']['visual_found'],accepted=sum(c['accepted'] for c in v['visual_diagnostics']['combinations']))
        for k,v in r['variants'].items()}) for r in rows]),ensure_ascii=False))
