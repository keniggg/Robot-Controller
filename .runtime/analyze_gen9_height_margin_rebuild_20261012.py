#!/usr/bin/env python3
"""Read text/closed JSON evidence only; never imports ROS or queries services."""
import ast
import hashlib
import json
from pathlib import Path
import re

W = Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
E = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
S = E / 'ros_log/full_latest_frozen_no_skin_20261012T021050Z'
R = E / 'grasp_records/20261012T122920.925646Z_9a08734d03f247ac895e5b762b04c615'
OUT = S / 'gen9_analysis/height_margin_rebuild'
OUT.mkdir(parents=True, exist_ok=False)

def sha(data):
    return hashlib.sha256(data).hexdigest()

def bound(path):
    data = path.read_bytes()
    return dict(path=str(path), sha256=sha(data), bytes=len(data))

def write(name, value):
    p = OUT / name
    p.write_text(json.dumps(value, ensure_ascii=False, indent=2, sort_keys=True, allow_nan=False)+'\n')
    return bound(p)

log = S / 'gen8_fix_compute_deployment/consumers.log'
log_bytes = log.read_bytes()
warnings, retries, extracts = [], [], []
for lineno, line in enumerate(log_bytes.decode().splitlines(), 1):
    stamp = re.search(r'\[(179\d+\.\d+)\]', line)
    context = dict(line=lineno, ros_time_sec=float(stamp.group(1)) if stamp else None)
    if 'unchanged translation evidence limit' in line:
        warnings.append(dict(context, message=line))
        extracts.append(line)
    prefix = 'remote 6D contact height retry: '
    if prefix in line:
        d = json.loads(line.split(prefix, 1)[1])
        extracts.append(line)
        search = d.get('search') or {}
        proposals = []
        for trial in search.get('proposals', []):
            placement = trial.get('placement') or {}
            height = trial.get('final_contact_height_margin') or {}
            precheck = trial.get('contact_sequence_precheck') or {}
            proposals.append(dict(
                accepted=trial.get('accepted'), plan_id=trial.get('plan_id'),
                normal_translation_m=placement.get('normal_translation_m'),
                displacement_m=trial.get('registration_plus_placement_displacement_m'),
                result=trial.get('result'), reason=trial.get('reason'),
                height={k: height.get(k) for k in (
                    'ok', 'translation_evidence_limit_m', 'maximum_symmetric_normal_translation_m',
                    'allowed_normal_translation_intervals_m', 'nominal')},
                precheck={k: precheck.get(k) for k in (
                    'plan_id', 'success', 'failure_code', 'failed_stage', 'scope',
                    'execution_authorized', 'future_following_verified', 'real_grasp_success',
                    'prediction_cache_hit', 'checked_stage_names', 'model_sha256',
                    'epoch_ns', 'planning_scene_digest', 'source_sample', 'completion_sample')}))
        retries.append(dict(context, request=d.get('request'), source_stamp_ns=d.get('source_stamp_ns'),
            original_plan_id=d.get('plan_id'), eligible=d.get('eligible'),
            deadline_sec=d.get('deadline_sec'), original_failure_code=d.get('original_failure_code'),
            search_reason=search.get('reason'), proposals=proposals))

extract_path = OUT / 'exact_height_log_lines.log'
extract_path.write_text('\n'.join(extracts)+'\n')
calibration_path = E / 'ros_log/handeye_auto_20260927T102700Z/final_validation_20260927T110130Z/result.json'
calibration = json.loads(calibration_path.read_text())
remote_path = W / 'src/alicia_flexible_grasp_supervisor/scripts/remote_grasp6d_node.py'
remote = remote_path.read_text()
tree = ast.parse(remote)
methods = {}
for node in ast.walk(tree):
    if isinstance(node, ast.FunctionDef) and node.name in {
        '_contact_height_window_retry', '_strict_check_final_near_field_plan',
        '_build_selected_preview_bundle', '_check_direct_registered_candidate',
        '_publish_selected_preview', '_prepare_final_registered_contact_plan',
        '_final_contact_sequence_evaluation'}:
        methods[node.name] = dict(first_line=node.lineno, last_line=node.end_lineno,
            source_sha256=sha(ast.get_source_segment(remote, node).encode()))
source_paths = [remote_path,
    W/'src/alicia_grasp_modes/scripts/mode_aware_remote_grasp6d.py',
    W/'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/grasp/contact_height_margin.py',
    W/'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/grasp/contact_height_placement.py',
    W/'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot/contact_sequence_precheck.py',
    W/'src/alicia_flexible_grasp_supervisor/config/grasp_params.yaml']
audits, first_example = [], None
for p in sorted((R/'keyframes_audits').glob('*.json')):
    data = json.loads(p.read_text())
    audits.append(dict(bound(p), request_id=data.get('request_id'), generation=data.get('generation')))
    if data.get('request_id') == 1:
        for index, row in enumerate(data.get('lineage', [])):
            placement = row.get('contact_height_placement') or {}
            if placement.get('after', {}).get('ok') and first_example is None:
                first_example = dict(path=str(p), field='lineage[%d].contact_height_placement'%index,
                    placement=placement)

result = dict(schema=1, scope='GEN9_read_only_height_margin_and_rebuild_source_review',
    execution_authorized=False, rpc_performed=False, bag_scanned=False,
    physical_grasp_success_claimed=False, production_modified=False,
    log_read=dict(path=str(log), read_bytes=len(log_bytes), read_prefix_sha256=sha(log_bytes),
        semantics='Exact bytes read at analysis; live file may subsequently append.'),
    exact_log_extract=bound(extract_path), sources=[bound(p) for p in source_paths],
    immutable_record_audits=audits,
    calibration=dict(source=bound(calibration_path),
        training_cross_validation=calibration['training_cross_validation'],
        independent_validation=calibration['independent_validation'],
        input_limit_m=0.0026767676132463574,
        interpretation='Historical observed translation maximum, used as unchanged empirical stress radius; not certified future 3D or normal error bound.'),
    warning_semantics=dict(
        tiny_margin_mm=0.000075,
        tiny_margin_metres=0.000000075,
        definition='min(-lo, hi) for the connected valid contact/support translation interval containing the current pose; this is remaining geometric slack, not observed calibration precision or contact overlap.',
        placement_rule='A failed current symmetric margin may be recentered only within the same connected interval if its full width is at least 2*limit; actual contact and support are rechecked after shifting.',
        no_placement_warning='The candidate connected interval cannot contain the unchanged full +/- limit after centering. This candidate rejection alone does not establish that the target or all candidates are infeasible.',
        algorithm='Finite measured contact band versus nonconvex CAD finger pad, intersected with 3 mm support clearance half-line; nominal geometry and both +/- endpoints retained.',
        unproven='No independent exact-input numerical reproduction of the track29/30 tiny interval: the early exception does not persist its complete geometry report in the completed audit rows read here.',
        finding='No evidence from these messages of a changed or erroneous calibration input; later accepted same-limit candidates disprove a global margin obstruction.'),
    warning_events=warnings, accepted_placement_example=first_example, height_retry_events=retries,
    rebuild_source_methods=methods,
    rebuild_finding=dict(
        status='independent_source_dataflow_confirmation_and_repeated_log_identity',
        sequence='Base direct candidate evaluation builds a preview; mode-specific MuJoCo evaluation builds it again; publication builds it again.',
        original_geometry='Each build starts with runtime.sequence and runtime.grasp_pose, applies registration and final support restoration again to a newly constructed local plan.',
        accepted_retry='Retry mutates the local plan from replacement and updates replacement_runtime. It records final_execution_sequence and final_strict_plan_id, but does not replace the raw runtime.sequence or runtime.grasp_pose from which the next build starts.',
        consequence='Accepted height may be discarded on the next build, so original plan failure and the identical height search can repeat and spend the unchanged near-field budget.',
        unsafe_naive_fix='Do not write already registered/lifted final poses into pre-registration runtime as if raw: rebuilding may apply registration/translation a second time.',
        preservation_direction='Preserve only exact context-bound selected geometry/placement intent, then reapply once and rehash/recheck all original final geometry, margin, continuity, and current strict precheck rules.',
        evidence_not_safe_to_cache_unconditionally=[
            'SDK/encoder freshness and independent stationary samples', 'enable epoch and mode selection',
            'accepted gripper opening', 'model, geometry and grasp/gripper configuration',
            'MoveIt scene and frozen target/registration/source identity',
            'request ticket, near-field phase and original deadline',
            'execution response, real contact/closure/lift or measured following'],
        existing_cache='Supervisor already reuses exact predictions for <=15 seconds only under unchanged bound plan/model/epoch/selection/config/feedback/opening/scene and reacquires fresh samples/checks context and scene afterward; it never stores an executable trajectory. Retain this rule.'))
artifact = write('result.json', result)
seal = write('seal.json', dict(schema=1, result=artifact, script=bound(Path(__file__)),
    exact_log_extract=bound(extract_path), no_production_mutation=True, no_rpc=True))
print(json.dumps(dict(result=artifact, seal=seal, warnings=len(warnings), retries=len(retries),
    accepted=[dict(time=r['ros_time_sec'], original=r['original_plan_id'],
        plan=p['plan_id'], shift=p['normal_translation_m'], cache=p['precheck']['prediction_cache_hit'])
        for r in retries for p in r['proposals'] if p['accepted']]), ensure_ascii=False))
