"""Offline interval audit; no output can qualify a box or authorize motion.

An in-memory AST probe continues one rejected calculation past its early
mask-extension guard. This is a numerical counterfactual, not a proposed patch.
All production sources and source image arrays are checked before and after.
"""
import ast
import hashlib
import inspect
import json
import math
import os
from pathlib import Path
import sys

W = Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
E = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
D = E/'ros_log/full_latest_gen36_restart_20261017T012037Z'
O = D/'verification/gen37_interval_budget_review_r1'
S = D/'verification/gen37_box_source_replay_r1'
I = D/'verification/gen37_actual_attempt1_source_images_r1'
G = E/'grasp_records/20261017T032113.476278Z_a84cc89fc53e4c2b82680749d78b9b9f'
assert Path('/media/zhuyupei/SanStand').is_mount()
assert 'sitecustomize' in sys.modules
assert os.environ['OFFLINE_BOUNDARY_LOG'] == str(O/'boundary.jsonl')
sys.path.insert(0, str(W/'src/alicia_flexible_grasp_supervisor/src'))

def sha(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()

def read(p):
    return json.loads(Path(p).read_text())

def serial(v):
    return v.tolist() if hasattr(v, 'tolist') else v.item()

def save(name, value):
    (O/name).write_text(json.dumps(value, ensure_ascii=False, indent=2,
                                 allow_nan=False, default=serial)+'\n')

release = read(D/'verification/gen37_release_manifest.json')
assert sha(D/'verification/gen37_release_manifest.json') == '1ef29fa3c200b139fb3b0fabaa3d6d34b12df76606047e24f61efcfaf6074487'
before = {p: sha(W/p) for p in release['source_sha256']}
assert before == release['source_sha256'] and len(before) == 1005
save('source_sha256_before.json', before)

import cv2
import numpy as np
from alicia_flexible_grasp.vision import outer_rgb_rectangle as outer
cv2.setNumThreads(1)

evidence = read(S/'original_fragment_family_diagnostic.json')
assert evidence['diagnostic_only'] and evidence['all_candidates_rejected']
assert evidence['qualified_model'] is None
audit = read(G/'keyframes_audits/0e5ebbd3c562cfab2617b338045f554ffc0920a23626fbc2a49cc474cc9d798d.json')
camera = read(D/'gen37_runtime/gen37_grasp_scene_r2/camera_configuration.json')
stamp = audit['snapshot_stamp_ns']
K = np.array([camera[k] for k in ('fx', 'fy', 'cx', 'cy')])
scale = camera['depth_scale']
T = np.asarray(audit['snapshot_transform']['T_base_optical'])
paths = [I/(str(stamp)+suffix) for suffix in ('_color.png', '_mask.png', '_depth.npy')]
image_sha = {str(p): sha(p) for p in paths}
extraction = read(I/'receipt.json')
assert all(v == extraction['output_sha256'][Path(p).name] for p, v in image_sha.items())
color = cv2.imread(str(paths[0]))
mask = cv2.imread(str(paths[1]), 0) > 0
depth = np.load(paths[2], allow_pickle=False)
state = np.load(S/'exact_current_top_fit.npz', allow_pickle=False)
origin, tn, axes = state['origin'], state['tn'], state['axes']
current = read(S/'current_qualification_actual.json')['audit']
uvtop = state['uv'][state['keep']]
pixel = float(np.median(depth[uvtop[:, 1], uvtop[:, 0]])*scale)/min(K[:2])/current['camera_top_cosine']
rms = current['top_plane_rms_m']
array_sha_before = [hashlib.sha256(v.tobytes()).hexdigest() for v in (color, mask, depth)]

tree = ast.parse(inspect.getsource(outer._infer_outer_rgb_rectangle))
removed = []
for node in ast.walk(tree):
    if isinstance(node, ast.If) and ast.get_source_segment(inspect.getsource(outer._infer_outer_rgb_rectangle), node.test) == 'proposal is not None and mask_extension > position_error+sampling':
        removed.append(ast.dump(node.test))
        node.body = [ast.Pass()]
assert len(removed) == 1
tree.body[0].name = '_diagnostic_continue_rejected_interval'
ast.fix_missing_locations(tree)
namespace = dict(math=math, cv2=cv2, np=np, _trig_range=outer._trig_range)
exec(compile(tree, '<diagnostic-only-in-memory-mask-interval>', 'exec'), namespace)
probe = namespace['_diagnostic_continue_rejected_interval']
members, rows = [], []
reference = None
for row in evidence['candidate_family']:
    offsets = row['edge_offsets_m']
    lo, hi = np.array([offsets[3], offsets[0]]), np.array([offsets[1], offsets[2]])
    original = outer._infer_outer_rgb_rectangle(color, mask, depth, scale, T, K,
        origin, tn, axes, pixel, rms, proposal=(lo, hi), resolved_sides=row['resolved_sides'])
    assert not original['ok'] and original['reason'] == 'unobserved mask band exceeds the RGB boundary localization interval'
    result = probe(color, mask, depth, scale, T, K, origin, tn, axes, pixel,
        rms, proposal=(lo, hi), resolved_sides=row['resolved_sides'])
    summary = dict(diagnostic_only=True, qualified_model=None, motion_authorized=False,
        diagnostic_later_gates_completed=bool(result['ok']),
        later_reason=result.get('reason'), interval=result.get('audit'))
    if result['ok']:
        A = result['axes']
        center = A[:, :2]@((result['lower']+result['upper'])/2.)
        if reference is None:
            reference = result, A, center
        _, RA, RC = reference
        angle = math.atan2(float(RA[:, 1]@A[:, 0]), float(RA[:, 0]@A[:, 0]))
        member = dict(center_offset_reference_m=((center-RC)@RA[:, :2]).tolist(),
            size_xy_m=(result['upper']-result['lower']).tolist(),
            nominal_yaw_offset_rad=angle, interval=result['audit'])
        members.append(member)
        summary.update(size_xy_m=member['size_xy_m'], edge_budget_m=result['edge_uncertainty'],
                       yaw_budget_rad=result['yaw_uncertainty'])
    rows.append(summary)

family = None
if len(members) == len(rows):
    edge, yaw = outer._family_budget(members)
    size = np.asarray(members[0]['size_xy_m'])
    narrow = int(np.argmin(size)); other = 1-narrow
    turn = min(yaw, math.atan2(size[other], size[narrow]))
    widest = size[narrow]*math.cos(turn)+size[other]*math.sin(turn)
    U = widest-size[narrow]+2*edge
    family = dict(diagnostic_only=True, qualified_model=None, motion_authorized=False,
        member_count=len(members), reference_size_xy_m=size, family_edge_budget_m=edge,
        family_yaw_budget_rad=yaw, centered_principal_narrow_nominal_m=float(size[narrow]),
        centered_principal_narrow_uncertainty_m=float(U),
        nominal_fit_clearance_each_side_m=.0005,
        centered_principal_required_opening_m=float(widest+2*edge+.001),
        physical_opening_m=.05, configured_reserve_m=.002,
        required_plus_reserve_m=float(widest+2*edge+.001+.002),
        note='Numeric feasibility only. No authentic outer-side/yaw qualification, contact-face proof, full 3D model, registration union or path check is produced.')

# A complete-mask spatial enclosure does not constrain the physical face yaw
# to the yaw inferred from internal printed lines.
angle = math.radians(10)
R = np.array([[math.cos(angle), -math.sin(angle)], [math.sin(angle), math.cos(angle)]])
true_size, print_size = np.array([.052, .035]), np.array([.044, .027])
signs = np.array([[-1., -1.], [-1., 1.], [1., -1.], [1., 1.]])
true_corners = signs*true_size/2 @ R.T
print_corners = signs*print_size/2
print_in_true = print_corners@R
assert np.all(np.abs(print_in_true) < true_size/2)
outer_half = np.max(np.abs(true_corners), axis=0)
extensions = outer_half-print_size/2
extension = float(extensions.max())
small_yaw = math.radians(.25)
nominal_widest = print_size[1]*math.cos(small_yaw)+print_size[0]*math.sin(small_yaw)
counterexample = dict(kind='analytic_complete_mask_internal_print_counterexample',
    actual_image_reproduction=False, physical_experiment=False,
    actual_top_size_m=true_size, actual_top_yaw_rad=angle,
    internal_print_size_m=print_size, internal_print_yaw_rad=0.,
    print_inside_true_margin_min_m=float(np.min(true_size/2-np.abs(print_in_true))),
    complete_true_mask_nominal_axis_extensions_m=extensions,
    extension_max_m=extension, nominal_print_yaw_budget_rad=small_yaw,
    hypothetical_plane_translation_m=.0005,
    hypothetical_edge_budget_m=extension+.0005,
    hypothetical_required_opening_m=float(nominal_widest+2*(extension+.0005)+.001),
    hypothetical_required_plus_reserve_m=float(nominal_widest+2*(extension+.0005)+.001+.002),
    actual_face_yaw_exceeds_existing_8deg=True,
    relevance='Both sides of printed lines can measure the same top plane; full Canny and a complete mask do not make those printed lines true prism faces. Expanding a fixed-axis spatial envelope alone does not enclose face orientation in the claimed small yaw interval.',
    limitations='This is an identifiability/math counterexample, not a full raster/plane/exterior gate pass. It refutes treating spatial enclosure alone as physical contact-face qualification.')

after = {p: sha(W/p) for p in before}
assert after == before
assert {str(p): sha(p) for p in paths} == image_sha
assert [hashlib.sha256(v.tobytes()).hexdigest() for v in (color, mask, depth)] == array_sha_before
boundary = O/'boundary.jsonl'
attempts = boundary.read_text().splitlines() if boundary.exists() else []
assert not attempts
save('source_sha256_after.json', after)
save('diagnostic_counterfactual.json', dict(
    diagnostic_only=True, qualified_model=None, motion_authorized=False,
    production_modified=False, source_stamp_ns=stamp, source_images_sha256=image_sha,
    source_count=len(before), source_unchanged=True, source_arrays_unchanged=True,
    removed_in_memory_ast_guard_only=removed, candidates=rows,
    family_numeric_feasibility=family, complete_mask_counterexample=counterexample,
    external_boundary_attempts=attempts, script_sha256=sha(__file__)))
print(json.dumps(dict(candidates=len(rows), later_gates_completed=[r['diagnostic_later_gates_completed'] for r in rows],
    family_numeric_feasibility=family, complete_mask_counterexample=counterexample), default=serial))
