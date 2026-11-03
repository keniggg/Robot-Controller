#!/usr/bin/env python3
"""Read-only source/audit arithmetic; no ROS, network, hardware, or qualification."""
import collections
import hashlib
import json
import math
from pathlib import Path
import time

W = Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
E = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
D = E / 'ros_log/full_latest_gen36_restart_20261017T012037Z'
G = E / 'grasp_records/20261017T070447.445347Z_83343e1525b54994a6f491259995a100'
O = D / 'verification/gen39_actual_attempt1_budget_review_r1'
assert Path('/media/zhuyupei/SanStand').is_mount(), 'SanStand unavailable; no fallback'
O.mkdir(exist_ok=True)
inputs = {}


def read(path):
    path = Path(path)
    raw = path.read_bytes()
    digest = hashlib.sha256(raw).hexdigest()
    prior = inputs.setdefault(str(path), digest)
    assert prior == digest, 'input changed: ' + str(path)
    return raw


def load(path):
    return json.loads(read(path))


def write(name, value):
    path = O / name
    assert not path.exists(), 'immutable output already exists: ' + str(path)
    raw = (json.dumps(value, ensure_ascii=False, indent=2, sort_keys=True,
                      allow_nan=False) + '\n').encode()
    path.write_bytes(raw)
    assert path.read_bytes() == raw
    return hashlib.sha256(raw).hexdigest()


def counter(values):
    return dict(collections.Counter(values))


initial_path = G / 'keyframes_audits/b41daabc71c9f7122439704be7e44fb5e7921d11344c6ff43917726b1c380da2.json'
final_path = G / 'keyframes_audits/9455bc56d73cbdf2a04ffbf86128abde2c71866ad9b31abe61b477207c174f6f.json'
initial, final = load(initial_path), load(final_path)
assert inputs[str(initial_path)] == initial_path.stem
assert inputs[str(final_path)] == final_path.stem
release_path = D / 'verification/gen39_release_manifest.json'
release = load(release_path)
assert inputs[str(release_path)] == '47a2f5148d585dfa0d938868453e8293fffdefc35290f1b788bc013703257030'
configuration_path = G / 'perception_execution_configuration.json'
load(configuration_path)
load(G / 'result_summary.json')
load(G / 'command_result.json')

relative_sources = [
    'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/grasp/gripper_geometry.py',
    'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/grasp/tabletop_geometry_candidates.py',
    'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/grasp/contact_evidence.py',
    'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/grasp/measured_contact.py',
    'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/grasp/contact_execution.py',
    'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/grasp/contact_height_margin.py',
    'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/regular_box_inference.py',
    'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/cuboid_rgb_interval.py',
    'src/alicia_flexible_grasp_supervisor/scripts/grasp_task_node.py',
    'src/alicia_flexible_grasp_supervisor/config/grasp_params.yaml',
    'src/real-arm/alicia_d_driver/src/alicia_d_driver_node.cpp',
]
source_checks = {}
release_bound_sources = []
additional_inspected_sources = []
for relative in relative_sources:
    read(W / relative)
    digest = inputs[str(W / relative)]
    if relative in release['source_sha256']:
        read(W / '.runtime/gen39_r2_source' / relative)
        assert digest == inputs[str(W / '.runtime/gen39_r2_source' / relative)]
        assert digest == release['source_sha256'][relative]
        release_bound_sources.append(relative)
    else:
        additional_inspected_sources.append(relative)
    source_checks[relative] = digest

tg = final['remote_diagnostics']['tabletop_geometry']
box = tg['regular_box_inference']
assert box['ok'] and box['code'] == 'REGULAR_BOX_INFERENCE_QUALIFIED'
assert box['cuboid_rgb_interval']['complete_raster_responses_used'] is True
assert final['snapshot_stamp_ns'] == 1792220759499861717
assert box['model_sha256'] == 'a52f80400af701553a7866f4791b6ff2913b70382e57ebb352bc37bea823e322'
directions = tg['surface_evidence_rejections']
assert len(directions) == 2
wide = next(x for x in directions if x['failure_code'] == 'NO_FIT_DIRECTION')
narrow = next(x for x in directions if x['failure_code'] == 'BOX_INFERENCE_PRECISION_VIEW_REQUIRED')
target = narrow['targeted_view']
L, B, H = box['inferred_size_xyz_m']
theta = target['empirical_yaw_uncertainty_rad']
e0, e1 = target['empirical_axis_uncertainty_m']
yaw_increment = L * math.sin(theta) + B * (math.cos(theta) - 1.)
axis_position = 2. * e1
required = target['required_open_upper_m']
fit_total = required - target['inferred_width_m'] - target['width_uncertainty_m']
assert abs(fit_total - .001) < 1e-12
assert abs(yaw_increment + axis_position - target['width_uncertainty_m']) < 1e-12
assert abs(target['inferred_width_m'] - B) < 1e-12
assert target['commanded_center_jaw_offset_m'] == 0.
assert abs(required - (B + yaw_increment + axis_position + fit_total)) < 1e-12
assert required < .05 and .05 - required < .002
assert tg['proposal_count'] == tg['materialized_count'] == 0
assert not final['stable_evaluations'] and final['moveit_selection']['checked_count'] == 0

cf = []
for opening in (.05, .0498):
    for reserve in (.002, 0.):
        cf.append(dict(opening_m=opening, reserve_m=reserve,
            complete_family_plus_fit_required_m=required,
            effective_required_m=required + reserve,
            aperture_margin_before_reserve_m=opening - required,
            margin_after_reserve_m=opening - required - reserve,
            scalar_aperture_inequality_passes=(required + reserve <= opening),
            measured_opening_semantics=('analytical maximum' if opening == .05 else
                'saved initial seed value / illustrative comparison; not a final accepted SDK sample'),
            candidate_materialized=False, path_evaluated=False,
            qualified_contact_or_motion_authorized=False))

evs = initial['stable_evaluations']
checked = [e for e in evs if (e.get('contact_goal_necessary_check') or {}).get('checked')]
assert len(evs) == 50 and len(checked) == 46
assert all(e['contact_goal_necessary_check']['rejected'] for e in checked)
assert all(e['contact_goal_necessary_check']['path_verified'] is False for e in checked)
assert all(e['contact_height_placement']['before']['code'] == 'CONTACT_HEIGHT_NORMAL_MARGIN_INSUFFICIENT' for e in checked)
assert all(e['contact_height_placement']['after']['ok'] for e in checked)
assert all(e['final_contact_height_margin']['code'] == 'CONTACT_HEIGHT_NORMAL_MARGIN_OK' for e in checked)
height = dict(
    scope='initial observation only; not final near-field candidate',
    empirical_normal_translation_evidence_m=.0026767676132463574,
    evaluated_unique_endpoints=len(checked),
    before_codes=counter(e['contact_height_placement']['before']['code'] for e in checked),
    after_codes=counter(e['contact_height_placement']['after']['code'] for e in checked),
    final_codes=counter(e['final_contact_height_margin']['code'] for e in checked),
    before_symmetric_margin_range_m=[min(e['contact_height_placement']['before']['maximum_symmetric_normal_translation_m'] for e in checked),max(e['contact_height_placement']['before']['maximum_symmetric_normal_translation_m'] for e in checked)],
    after_symmetric_margin_range_m=[min(e['contact_height_placement']['after']['maximum_symmetric_normal_translation_m'] for e in checked),max(e['contact_height_placement']['after']['maximum_symmetric_normal_translation_m'] for e in checked)],
    final_symmetric_margin_range_m=[min(e['final_contact_height_margin']['maximum_symmetric_normal_translation_m'] for e in checked),max(e['final_contact_height_margin']['maximum_symmetric_normal_translation_m'] for e in checked)],
    continuous_3d_path_or_grip_proven=False)

seed = initial['pipeline_funnel']['acceptance_diagnostics']['endpoint_ik_search_order']['joint_seed']
idx = seed['name'].index('right_finger')
assert abs(seed['position_rad'][idx] - .0498) < 1e-12
seed_summary = dict(pointer='/pipeline_funnel/acceptance_diagnostics/endpoint_ik_search_order/joint_seed',
    initial_stamp_ns=seed['stamp_ns'], frame_id=seed['frame_id'], source=seed['source'],
    right_finger_value_m=seed['position_rad'][idx], service_start_state_exact=seed['service_start_state_exact'],
    final_stage_accepted_sdk_measurement=False,
    limitation='Do not promote a saved initial remote seed into fresh accepted final-stage SDK evidence.')

# Exact released sources establish introduction independently of folder names.
old_base = E / 'ros_log/full_latest_gen22_restart_20261015T003700Z/verification'
rel = relative_sources[0]
history_spec = [
    ('GEN25', old_base/'gen25_release_manifest.json', W/'.runtime/gen10_supplement_handoff_authority_20261012'/rel),
    ('GEN26', old_base/'gen26_release_manifest.json', W/'.runtime/gen30_baseline_source'/rel),
    ('GEN31', old_base/'gen31_release_manifest.json', W/'.runtime/gen32_baseline_source'/rel),
    ('GEN36', E/'ros_log/full_latest_gen29_restart_20261016T000728Z/verification/gen36_release_manifest.json', W/'.runtime/gen36_baseline_source'/rel),
    ('GEN38', D/'verification/gen38_release_manifest.json', W/'.runtime/gen38_baseline_source'/rel),
]
history = []
history_text = {}
for version, manifest_path, source_path in history_spec:
    old = load(manifest_path)
    source = read(source_path).decode()
    assert old['source_sha256'][rel] == inputs[str(source_path)]
    history_text[version] = source
    history.append(dict(version=version, release_manifest=str(manifest_path),
        release_manifest_sha256=inputs[str(manifest_path)],
        exact_source_path=str(source_path), source_sha256=inputs[str(source_path)]))
assert 'def evaluate_inferred_box_contact' not in history_text['GEN25']
assert 'physical-required < max(float(finger_geometry.jaw_clearance_each_side_m),uncertainty)' in history_text['GEN26']
assert 'ANALYTICAL_JAW_CLEARANCE_EACH_SIDE_M = 0.002' in history_text['GEN26']
assert 'it is neither a measured localization error nor a physical gap change.' in history_text['GEN36']

selector_path = D / 'verification/gen39_actual_attempt1_source_replay_r1/exact_selector_replay.json'
selector = load(selector_path)
assert inputs[str(selector_path)] == 'de1dbd6781efc132179b14a376489959a2e6bbf326adf66e927c582ae9354034'
assert selector['actual_preappend_available'] is True
assert selector['selected_after_exact_cas'] is True
assert selector['final_same_source_full_fusion_model_sha256'] == box['model_sha256']
assert selector['raw_model_reused_for_final'] is False
assert selector['choice']['selected_hypothesis'] == 'identity'
selector_summary = dict(
    online_audit_has_registration_choice_receipt=False,
    online_fused_scene_geometry=final['multiview_surface']['fused_scene_geometry'],
    evidence_type='exact offline replay using retained actual preappend CAS and true current raw source',
    artifact_path=str(selector_path), artifact_sha256=inputs[str(selector_path)],
    actual_before_count=selector['before_count'], actual_raw_count=selector['raw_count'],
    selected_hypothesis=selector['choice']['selected_hypothesis'],
    selected_after_matches_actual_CAS=True,
    final_same_source_full_fusion_model_sha256=box['model_sha256'],
    original_icp=selector['choice']['hypotheses']['original_icp'],
    identity=selector['choice']['hypotheses']['identity'],
    raw_model_is_only_registration_choice_not_final_contact_authority=True,
    qualification_or_motion_authorized_by_replay=False)

report = dict(
    schema_version=1, kind='gen39_actual_attempt1_independent_budget_review',
    recorded_ns=time.time_ns(), diagnostic_only=True, offline_only=True,
    production_or_tests_changed=False, ros_used=False, network_used=False,
    hardware_accessed=False, motion_authorized=False, physical_grasp_verified=False,
    record=str(G), release_manifest=str(release_path), release_sha256=inputs[str(release_path)],
    final=dict(request_id=final['request_id'], generation=final['generation'],
        source_stamp_ns=final['snapshot_stamp_ns'], target_epoch=final['target_epoch'],
        outcome=final['outcome'], plan_id=final['plan_id'],
        box_qualification_code=box['code'], box_model_sha256=box['model_sha256'],
        inferred_size_xyz_m=box['inferred_size_xyz_m'],
        empirical_axis_uncertainty_m=[e0,e1], empirical_yaw_uncertainty_rad=theta,
        empirical_height_uncertainty_m=box['cuboid_rgb_interval']['empirical_height_uncertainty_m'],
        complete_raster_responses_used=True,
        top_occupancy_fraction=box['top_occupancy_fraction'],
        maximum_internal_hole_fraction=box['maximum_internal_hole_fraction'],
        maximum_observed_model_excess_m=box['maximum_observed_model_excess_m'],
        tabletop_failure_code=tg['failure_code'], tabletop_proposals=tg['proposal_count'],
        tabletop_materialized=tg['materialized_count'],
        source_counts=final['pipeline_funnel']['source_counts'],
        learned_local_rejections=dict(NO_FIT_DIRECTION=6,GRASPNET_STAGE_PROFILE_UNAVAILABLE=3,
            BOX_CONTACT_DIRECTION_UNSUPPORTED=2,BOX_CONTACT_CENTER_OUTSIDE=1),
        final_moveit_checked=final['moveit_selection']['checked_count'],
        final_stable_evaluations=len(final['stable_evaluations']),
        final_ik_or_path_failure_established=False),
    narrow_direction_budget=dict(angle_deg=narrow['angle_deg'],
        nominal_m=B, yaw_projection_increment_m=yaw_increment,
        twice_axis_position_budget_m=axis_position,
        normal_height_projection_m=0., commanded_center_offset_m=0.,
        width_uncertainty_m=target['width_uncertainty_m'], fit_clearance_each_side_m=fit_total/2.,
        required_m=required, physical_maximum_m=.05,
        margin_before_reserve_m=.05-required,
        precision_reserve_m=target['precision_reserve_m'],
        precision_reserve_source=target['precision_reserve_source'],
        effective_required_m=required+target['precision_reserve_m'],
        maximum_face_direction_error_deg=target['maximum_face_direction_error_deg'],
        uncertainty_is_certified_bound=False),
    wide_direction=wide, scalar_counterfactuals=cf,
    counterfactual_semantics='Only arithmetic on the original complete uncertainty family; no materialization, final union/CAD, contact, freshness, or path acceptance is claimed.',
    initial=dict(box_code=initial['remote_diagnostics']['tabletop_geometry']['regular_box_inference']['code'],
        measured_model_maximum_excess_m=initial['remote_diagnostics']['tabletop_geometry']['regular_box_inference']['maximum_observed_model_excess_m'],
        stable_evaluations=len(evs), unique_necessary_endpoint_checks=len(checked),
        duplicate_evaluations_not_independent_rejections=len(evs)-len(checked),
        endpoint_check_policy='inevitable_frozen_contact_goal_obb_overlap',
        endpoint_rejection_count=46, path_verified=False,
        reason='Endpoint necessary check proves unavoidable frozen OBB overlap within unchanged endpoint/opening tolerances; not 46 failed continuous paths or 46 failed IK solves.'),
    initial_height_margin=height, saved_initial_opening_seed=seed_summary,
    reserve_policy_review=dict(
        classification='Additional configured software aperture-margin policy, not measured localization error, hardware opening error, or physical gap change.',
        total_margin_not_two_mm_per_side=True,
        inherited_from_GEN26=True,
        original_GEN26_semantics='max(configured 2 mm, same width U) extra reserve; later extra-U duplication removed.',
        current_width_U_counted_once=True,
        fit_semantics='0.5 mm per side configured geometric opening-fit clearance. No independent evidence it bounds execution tracking or full handeye translation.',
        distinct_unsupported_claim='Neither current U plus fit nor the fixed 2 mm reserve certifies all 3D localization/execution errors.',
        known_external_error='Handeye empirical translation stress 2.676767613 mm from CV/held-out calibration; existing contact-height check covers support-normal 1D only. Existing operator centering override remains enabled.',
        downstream_formula='required=max(union_required,model_required); effective_required=max(union_required,model_required+reserve). U is not summed a second time with union.',
        collision_formula='Complete measured OBB plus complete inferred yaw/axis/height family union; fixed 2 mm reserve does not expand this inferred box family.',
        support_for_preference='Evidence supports treating this fixed extra reserve as a clearance preference or bounded supplementary-view preference rather than asserting physical impossibility. A coherent policy change must cover all consumers and retain original actual opening, full family, union, CAD/contact/support/path/freshness checks.',
        not_a_replacement='Do not treat 1 mm fit as a newly calibrated execution-error bound or remove real U; no new numerical threshold is proposed.',
        scalar_fit_does_not_authorize_motion=True,
        consumers=[
            dict(path=relative_sources[0],lines='199-214,251-282,331-363',role='contact evaluation and typed candidate recomputation'),
            dict(path=relative_sources[1],lines='421-434',role='additional explicit proposal precision_shortfall'),
            dict(path=relative_sources[2],lines='124-149',role='signed inferred evidence recomputation'),
            dict(path=relative_sources[3],lines='60-177',role='fresh measured-stage model/union aperture plus full finite CAD'),
            dict(path=relative_sources[8],lines='3938-4047',role='new accepted sdk_measured FK/opening and authority/freshness rechecks'),
        ]),
    reserve_history=history,
    sdk_opening_semantics=dict(
        analytical_maximum_m=.05,
        independent_hardware_opening_comment_m=.0499375,
        physical_open_gap_contract_tolerance_m=.0005,
        sdk_value_mapping='SDK raw/1000 times configured prismatic stroke; feedback is not an independent caliper measurement of maximum opening.',
        driver_lines='1427-1438,2348-2351,3291-3308',
        final_evaluation='Task samples new sdk_measured right_finger and actual FK; saved initial seed cannot authorize the final stage.'),
    selector_replay=selector_summary,
    source_checks=source_checks,
    release_bound_sources=release_bound_sources,
    additional_inspected_sources_not_claimed_in_GEN39_1025=additional_inspected_sources,
    first_uncompleted_handoff='Final near-field candidate producer did not generate an admissible contact candidate; no final plan exists to be rejected by executor/IK/path.',
    next_minimal_step='Review fixed clearance preference consistently across all contact consumers, then exact-source replay the entire producer-to-plan/CAD/path chain without lowering real U or dropping history. Only a new fresh on-site admission could authorize a later motion.',
    limitations=[
        'No final candidate materialized, so scalar aperture counterfactual does not establish path feasibility or actual contact.',
        'Uncertainty is an empirical common family, not a certified bound on all localization or execution errors.',
        'A exact replay is independently source/CAS-bound evidence; original online audit alone has no registration_choice receipt.',
        'A reports six ACK first-two-frame image files absent; do not manufacture a complete per-frame comparison.',
        'No old favorable frame was selected or substituted; no hardware action was performed.',
    ],
    evidence_sha256=dict(inputs))

for path, digest in inputs.items():
    assert hashlib.sha256(Path(path).read_bytes()).hexdigest() == digest, 'source changed during review: '+path
report['all_review_inputs_unchanged'] = True
report_sha = write('result.json', report)
md = f'''本次最终补视数据的盒形v3已经合格，完整历史最大几何超差0.902800mm通过原1mm门。第一处未完成交接是候选生产：窄向装入量49.129509mm小于50mm，但剩余0.870491mm低于固定2mm额外软件净空；tabletop未生成候选，12个learned候选也全部局部拒绝，最终MoveIt检查0条。

| 窄向预算 | mm |
|---|---:|
| 名义窄边 | {B*1000:.9f} |
| 完整yaw族投影增宽 | {yaw_increment*1000:.9f} |
| 两侧axisE位置预算 | {axis_position*1000:.9f} |
| 当前方向height投影/命令中心偏移 | 0 / 0 |
| 双侧fit | {fit_total*1000:.9f} |
| required | {required*1000:.9f} |
| 独立额外reserve | 2.000000000 |
| 当前策略effective required | {(required+.002)*1000:.9f} |

2mm是GEN26引入的固定软件margin策略。GEN25精确发布源码没有推断接触函数；GEN26发布绑定的源码291–305行已经额外要求margin≥max(2mm,U)。GEN31对v2、GEN36对v1去掉重复extra-U，当前源码199–214行明确它不是测得的定位误差或物理gap变化。没有在所审来源中找到它来自硬件/执行误差标定的独立依据。它比较的是总margin，不是再要求2mm/侧。

配置fit为0.5mm/侧的几何装入净空，不能冒称覆盖执行跟踪或三维手眼误差。当前U包含真实图像/高度/共同yaw族及命令中心偏移，仍明确为经验预算。独立±2.676768mm来自手眼CV/held-out经验应力；现有接触高度门只检验支撑法向1D，原操作员jaw-centering override仍启用。固定2mm本身也不能替代该三维误差证明。

支持将固定额外2mm按实际语义审议为净空排序或有界补视偏好，保留真实开口、完整family/测量union、CAD/有限接触、支撑面、路径、新鲜性与控制权检查。必须一致处理tabletop显式precision_shortfall、contact evaluator、typed/signed证据复核和最终measured_aperture，不能只在上游删一个if。当前最终消费者用max(union_required, model_required+reserve)，没有再次把U与union相加。碰撞包络已包含同一完整yaw/axis/height族，并未用reserve扩大模型。

仅数值反事实：reserve=0时49.129509mm小于50mm，亦小于保存的初始seed 49.8mm；前者仅余0.870491mm、后者0.670491mm。初始seed frame_id为空且不是最终新鲜accepted SDK测量，不能作为最终许可。驱动将SDK数值按stroke映射米；分析上限50mm、源码注释独立实测49.9375mm以及SDK反馈49.8mm是不同证据，不能互换。

初始另有46条冻结目标OBB必要终点拒绝，path_verified全部false；不是46条连续路径失败。±2.676768mm对before 0.5mm的警告随后已有contact-height自动平移，46条after/final均通过，法向余量范围3.523832–6.814202mm。该before警告不是最终近视首拒。

A的真实preappend1852点+当前raw931点精确回放选择I，after2610点与CAS逐项一致，最终model SHA同a52f8040…。原ICP虽NN RMSE较低，完整union最大超差1.049326mm仍超过原1mm；I为0.902800mm。线上audit单独没有choice回执，REGISTERED I本身不证明selector触发，结论依赖该精确回放。未使用raw model直接授权最终接触，未删除历史、未改U/阈值。

这是只读离线问题审查，没有生成接触计划、验证最终路径或发出运动。result SHA256 `{report_sha}`。完整输入、源码、历史发布与A精确回放SHA见result.json。
'''
md_path = O / 'review.md'
assert not md_path.exists()
md_path.write_text(md)
assert md_path.read_text() == md
seal = dict(schema_version=1, kind='gen39_actual_attempt1_budget_review_seal',
    review_completed=True, diagnostic_only=True, release_review_complete=False,
    production_or_tests_changed=False, motion_authorized=False,
    artifacts_sha256={'result.json':report_sha,'review.md':hashlib.sha256(md_path.read_bytes()).hexdigest()},
    source_sha256=source_checks)
seal_sha = write('seal.json',seal)
print(json.dumps(dict(output=str(O),result_sha256=report_sha,seal_sha256=seal_sha,
    source_checks=len(source_checks),diagnostic_only=True,motion_authorized=False),ensure_ascii=False))
