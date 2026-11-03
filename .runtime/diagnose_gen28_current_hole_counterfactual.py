"""Bounded offline counterfactual; never produces qualified execution evidence.

Only the qualifier's internal-hole rejection is removed in a private copy.
The real model constructor must still reject the resulting hole evidence.
The unchanged opening evaluator bytecode is then used with a diagnostic-only
value type to disclose later numerical obstacles, not to authorize contact.
"""
import ast
from copy import deepcopy
import hashlib
import inspect
import json
import os
from pathlib import Path
import sys
import types

W = Path(__file__).resolve().parents[1]
M = Path('/media/zhuyupei/SanStand')
N = M / '实验记录/Robot-Controller/ros_log/full_latest_gen22_restart_20261015T003700Z'
R = N / 'verification/gen28_current_initial_readonly_replay_r1/result.json'
O = N / 'verification/gen28_current_internal_hole_counterfactual_r2'
assert M.is_mount() and not os.statvfs(M).f_flag & os.ST_RDONLY
assert not O.exists()
O.mkdir()
(O / 'tmp').mkdir()
(O / 'ros').mkdir()
os.environ.update(ROS_MASTER_URI='http://127.0.0.1:1',
    PYTHONDONTWRITEBYTECODE='1', ROS_LOG_DIR=str(O / 'ros'), TMPDIR=str(O / 'tmp'),
    OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1')
sys.path.insert(0, str(W / 'src/alicia_flexible_grasp_supervisor/src'))

import numpy as np
from alicia_flexible_grasp.vision import regular_box_inference as box
from alicia_flexible_grasp.grasp import gripper_geometry as gg
from alicia_flexible_grasp.vision.object_geometry import GeometryEstimate
from alicia_flexible_grasp.vision.target_observation import TargetTrackIdentity


def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def read(path):
    return json.loads(Path(path).read_text())


def serial(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    raise TypeError(type(value).__name__)


class DiagnosticOnlyBox:
    """Explicitly not a RegularBoxInference; production admission must refuse."""
    def __init__(self, body):
        self.body = deepcopy(body)

    def to_evidence_dict(self):
        return deepcopy(self.body)

    def __getattr__(self, key):
        value = self.body[key]
        return np.asarray(value, dtype=float) if key in (
            'center_base', 'axes_base', 'size_xyz_m', 'support_normal_base') else value


replay = read(R)
assert sha(R) == '4a70af98a6a8fa90a79cdbc44f771e3a5d51545656b6598c1ffdc4dd9f2b16d7'
assert replay['raw_qualification']['code'] == 'BOX_TOP_CONTOUR_INCOMPLETE'
assert replay['raw_qualification']['audit']['reason'] == 'missing or holed top occupancy'
sources = dict(replay['source_sha256'])
sources[str(Path(__file__).resolve().relative_to(W))] = sha(__file__)
inputs = dict(replay['input_sha256'])
inputs[str(R)] = sha(R)
inputs[replay['measured_geometry']['path']] = replay['measured_geometry']['sha256']
assert all(sha(W / p) == h for p, h in sources.items())
assert all(sha(p) == h for p, h in inputs.items())
capture = read(replay['capture'])
frame = capture['frames'][-1]
raw = np.load(frame['npz'], allow_pickle=False)
geometry = GeometryEstimate(ok=True, failure_code='', failure_reason='',
    **read(replay['measured_geometry']['path']))
identity = TargetTrackIdentity(**replay['diagnostic_identity'])
params_path = next(p for p in inputs if p.endswith('/parameters.json'))
values = read(params_path)['before']['parameters']
physical = float(values['/gripper']['open_position_m'])
gripper = gg.GripperGeometry(**replay['effective_gripper_geometry'])
fit = float(replay['effective_tabletop_geometry_config']['opening_fit_clearance_each_side_m'])
effective = min(physical, gripper.max_inner_gap_m,
    replay['effective_tabletop_geometry_config']['max_inner_gap_m'], gg.ANALYTICAL_MAX_INNER_GAP_M)
args = (raw['depth_raw'], raw['object_mask'], raw['intrinsics'],
        float(raw['depth_scale_m']), raw['T_base_optical'], geometry, identity,
        int(raw['source_stamp_ns']))
baseline = box.qualify_regular_box_top(*args, color_bgr=raw['color'])
assert not baseline.ok and baseline.code == replay['raw_qualification']['code']
assert dict(baseline.audit)['maximum_internal_hole_fraction'] == replay['raw_qualification']['audit']['maximum_internal_hole_fraction']

original_source = inspect.getsource(box.qualify_regular_box_top)
tree = ast.parse(original_source)
original_tree = ast.dump(tree, include_attributes=False)
removed = []
for node in ast.walk(tree):
    if not isinstance(node, ast.If) or not isinstance(node.test, ast.BoolOp):
        continue
    test = node.test
    if (isinstance(test.op, ast.Or) and len(test.values) == 2
            and ast.dump(test.values[1], include_attributes=False) == ast.dump(
                ast.parse('hole_fraction > 0.02', mode='eval').body, include_attributes=False)):
        removed.append('occupancy < (.65 if visual is not None else .88) or hole_fraction > .02')
        node.test = test.values[0]
assert len(removed) == 1
ast.fix_missing_locations(tree)
namespace = dict(vars(box))
exec(compile(tree, '<diagnostic_only_skip_internal_hole>', 'exec'), namespace)
qualifier = namespace['qualify_regular_box_top']
captured = {}


def tracer(frame, event, arg):
    if frame.f_code is qualifier.__code__:
        if event == 'return':
            for name in ('model', 'edge_uncertainty', 'height_uncertainty',
                         'yaw_uncertainty', 'unobserved_extension', 'excess'):
                if name in frame.f_locals:
                    captured[name] = deepcopy(frame.f_locals[name])
        return tracer
    return None


old_trace = sys.gettrace()
try:
    sys.settrace(tracer)
    counterfactual = qualifier(*args, color_bgr=raw['color'])
finally:
    sys.settrace(old_trace)
assert not counterfactual.ok, 'No diagnostic run may yield production-qualified evidence'
assert counterfactual.model is None
opening_rows = []
production_validation_error = None
if 'model' in captured:
    body = captured['model']
    try:
        box.validate_box_model_evidence(body)
    except ValueError as exc:
        production_validation_error = str(exc)
    assert production_validation_error == 'maximum_internal_hole_fraction outside inference limits'
    assert counterfactual.code == 'BOX_INFERENCE_INPUT_INVALID'
    diagnostic = DiagnosticOnlyBox(body)
    assert not isinstance(diagnostic, box.RegularBoxInference)
    production_type_rejection = gg.evaluate_inferred_box_contact(
        diagnostic, diagnostic.center_base, diagnostic.axes_base[:, 1],
        -diagnostic.support_normal_base, gripper,
        opening_fit_clearance_each_side_m=fit)
    assert not production_type_rejection.ok and production_type_rejection.code == 'BOX_INFERENCE_INPUT_INVALID'
    globals_copy = dict(gg.evaluate_inferred_box_contact.__globals__, RegularBoxInference=DiagnosticOnlyBox)
    evaluate = types.FunctionType(gg.evaluate_inferred_box_contact.__code__, globals_copy,
        'diagnostic_only_opening_math', gg.evaluate_inferred_box_contact.__defaults__)
    assert evaluate.__code__ is gg.evaluate_inferred_box_contact.__code__
    for axis_index in (0, 1):
        ev = evaluate(diagnostic, diagnostic.center_base, diagnostic.axes_base[:, axis_index],
            -diagnostic.support_normal_base, gripper,
            support_normal_base=diagnostic.support_normal_base,
            opening_fit_clearance_each_side_m=fit)
        targeted = {key: value for key, value in ev.targeted_view.items()
            if key not in ('model_sha256', 'target_edges_or_corners_base')}
        required = ev.inferred_width_m + ev.width_uncertainty_m + 2. * fit
        opening_rows.append(dict(diagnostic_axis_index=axis_index,
            diagnostic_math_ok=ev.ok, diagnostic_math_code=ev.code,
            nominal_width_m=ev.inferred_width_m, width_uncertainty_m=ev.width_uncertainty_m,
            required_open_upper_m=required, effective_opening_m=effective,
            remaining_aperture_m=effective-required,
            precision_reserve_m=max(gripper.jaw_clearance_each_side_m,ev.width_uncertainty_m),
            source_typed_production_admission=False, contact_height_bounds_m=ev.contact_height_bounds_m,
            targeted_numeric_diagnostic=targeted))
    parameters = {key: body[key] for key in ('size_xyz_m', 'center_base', 'axes_base',
        'empirical_edge_uncertainty_m', 'empirical_height_uncertainty_m',
        'empirical_yaw_uncertainty_rad')}
    parameters['qualification_numeric_diagnostic'] = body['qualification']
else:
    parameters = {key: value for key,value in captured.items() if key != 'model'}

assert all(sha(W / p) == h for p, h in sources.items())
assert all(sha(p) == h for p, h in inputs.items())
report = dict(schema_version=1,
    classification='rejected_input_counterfactual_numeric_diagnostic_only',
    qualification_gate_bypassed_diagnostic_only=True,
    exact_removed_qualifier_predicate='hole_fraction > 0.02',
    original_combined_condition=removed[0], occupancy_gate_retained=True,
    other_qualifier_checks_unchanged=True,
    original_qualifier_ast_sha256=hashlib.sha256(original_tree.encode()).hexdigest(),
    cloned_qualifier_ast_sha256=hashlib.sha256(ast.dump(tree,include_attributes=False).encode()).hexdigest(),
    opening_evaluator_body_unchanged=True,
    opening_type_binding='private globals: RegularBoxInference -> DiagnosticOnlyBox; real production type guard separately rejects',
    production_qualification=dict(ok=baseline.ok,code=baseline.code,audit=dict(baseline.audit)),
    cloned_qualifier_terminal=dict(ok=counterfactual.ok,code=counterfactual.code,audit=dict(counterfactual.audit)),
    production_validator_rejection=production_validation_error,
    remaining_qualifier_reached_model_validation='model' in captured,
    diagnostic_parameters=parameters, diagnostic_opening_rows=opening_rows,
    current_physical_opening_m=physical, opening_fit_clearance_each_side_m=fit,
    source_sha256=sources,input_sha256=inputs,source_and_inputs_unchanged=True,
    qualified_production_model_emitted=False,executable_plan_emitted=False,
    execution_authority=False,motion_authority=False,physical_grasp_verified=False,
    no_ros_calls=True,no_hardware_access=True,no_temperature_reads=True,
    limitations=['This bypass is solely a counterfactual diagnosis. Production still rejects the measured internal hole.',
      'No collision path, IK, execution, or closing contract was evaluated.',
      'Any numeric downstream acceptance would still be non-authorizing and would not establish that the hole is physically harmless.'])
out = O / 'result.json'
out.write_text(json.dumps(report,ensure_ascii=False,indent=2,default=serial,allow_nan=False)+'\n')
print(json.dumps(dict(path=str(out),sha256=sha(out),
    production_validator_rejection=production_validation_error,
    remaining_qualifier_reached_model_validation='model' in captured,
    diagnostic_opening_rows=opening_rows),ensure_ascii=False,default=serial),flush=True)
