"""Read-only GEN38 design review artifact; no production/test modification."""
import hashlib
import json
import os
from pathlib import Path
import sys
import time

W = Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
D = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen36_restart_20261017T012037Z')
O = D/'verification/gen38_shape_contract_review_r1'
assert Path('/media/zhuyupei/SanStand').is_mount()
assert 'sitecustomize' in sys.modules
assert os.environ['OFFLINE_BOUNDARY_LOG'] == str(O/'boundary.jsonl')

def read(p):
    return json.loads(Path(p).read_text())

def sha(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()

def save(name, obj):
    (O/name).write_text(json.dumps(obj, ensure_ascii=False, indent=2,
                                 allow_nan=False)+'\n')

release = read(D/'verification/gen37_release_manifest.json')
assert sha(D/'verification/gen37_release_manifest.json') == '1ef29fa3c200b139fb3b0fabaa3d6d34b12df76606047e24f61efcfaf6074487'
before = read(O/'source_sha256_before.json')
assert before == release['source_sha256']
assert {p:sha(W/p) for p in before} == before and len(before) == 1005

review = dict(
    complete=True, kind='same_frame_cuboid_projection_contract_design_review',
    production_patch_written=False, tests_modified=False,
    actual_frame_fitting_performed=False, qualified_model=None,
    motion_authorized=False,
    recommendation='先做逐候选完整可见面投影核验；若现有候选集没有真外边，再用完整cuboid8角投影拟合提出同一套参数族。不能把side预测偷换成原resolved_sides真实深度回执。',
    problem=dict(
        confirmed='当前mask含顶面与barcode侧面，整mask射线投到顶面后构造的矩形不再是实际外轮廓；真实mask边界有完整Canny支持。',
        existing_helpers=[
            '_visual_rectangle只记录四条候选边±3px、21点的局部top/side深度，且需至少三面mask-boundary。',
            '_top_side_proposals的resolved=True严格来自inside valid>=.60且top>=.70、outside valid>=.60且top<=.30。',
            '_infer_outer_rgb_rectangle已有known measured side排除，但unknown边带仍全部可为顶面；没有逐像素可见面身份或首交证明。',
            '因此现有标记不足以把右侧未知barcode band从top约束移走；仅改过滤条件或置resolved=True会违反原资格合同。'],
        current_raw_top_tangent_span_m=[.05716151452254903,.04035101863],
        top_support_angle_deg=2.0994714435446626,
        note='raw top范围包括原拟合keep内全部9862点，仅用于提示数据约束；不授权按百分位删点或把模型拟合成功当抓取成功。'),
    minimum_same_frame_input_contract=dict(
        identity=['exact target epoch/track','exact retained source stamp/frame','generation/observation request lineage'],
        immutable_source=['unchanged BGR','unchanged raw depth','unchanged binary mask','same K/depth scale/T','same admitted geometry and support'],
        inherited_measurements=['top origin/normal/RMS','support normal/offset','top median height and tilt/slab','original depth-top axis as search seed'],
        ban=['no fused mask as current silhouette','no new frame substitution','no masked depth filling','no historical model cache in this patch']),
    shared_parameter_contract=dict(
        parameters='p=(theta,lx,ux,ly,uy,h) in one common support basis; center and size are derived from the same offsets.',
        geometry='A(theta)=[a0(theta),a1(theta),support_normal]; lower4 vertices on n·x+d=0, upper4 on a common top-height/slab family. All 8 vertices and all visible faces share p.',
        tilt='Current measured top is 2.099 degrees from support. Preserve the existing measured top-plane slab and vertical deviation. Either project the measured tilted top intersections with vertical sides, or bound all consistent parallel-h slab members. One best constant h must not silently replace this family.',
        candidate_directions='Depth-top axes are a seed and existing +/-8deg search domain, not measured physical side normals. Direction qualification comes from the jointly explained full external silhouette and its projected top/bottom/vertical edge assignments.',
        assignment='A silhouette horizontal edge can be top or bottom; a vertical edge follows support normal in 3D. Infer the assignment jointly with all 8 corners. A Hough print line inside the silhouette cannot independently choose theta.'),
    original_raster_constraints=dict(
        mask_boundary='Extract the external current binary-mask boundary. Compare it with the model external convex hull in BOTH directions using the original 5x5 neighbourhood (Chebyshev2 / Euclidean sqrt8), all edges and corner arms, not mean IoU or a percentile.',
        color='Require current original Canny in the same 5x5 neighbourhood of the external contour used as geometric evidence. Ignore interior printing as an orientation authority; it may remain a consistency observation.',
        segmentation='Every original boundary/unknown-band pixel stays in the evidence. A pixel may be explained as side only by a candidate whose COMPLETE cuboid projection agrees with the true mask/Canny contour. Do not crop it from the source mask.',
        depth='For each valid source depth used for qualification, compare its ray first-hit possibilities with the candidate top/side/support faces, retaining inherited plane/slab/scale budgets. Close-to-top points may be top-or-side; do not label them side merely because exclusion helps.',
        geometry='Retain existing all measured geometry conflict check, maximum excess, original OBB and registered fused union. A smaller silhouette fit may not discard raw/fused/history points to obtain aperture.',
        outside='Keep current image margins, complete outer envelope visibility, measured continuation and occluder contradictions. Unknown depth outside is unknown, not positive free-space evidence.'),
    smallest_candidate_validation_flow=[
        'Use current depth/Hough candidates unchanged initially, with original offsets; if fragment grouping is separately repaired, preserve all offsets and all original depth evidence.',
        'Build each full prism candidate from the SAME theta/offsets/height/support. Project all8 corners with the exact T/K and derive the visible hull/face first-intersection map.',
        'Reject a candidate if its complete projected outer hull fails bidirectional current-mask/Canny support. An internal printed top rectangle cannot pass merely by treating everything outside it as side.',
        'For a passing candidate, retain the full mask and attach an explicit top/side/ambiguous explanation per relevant boundary band. Unknown pixels explained by side are still recorded; the top constraint changes because the full 3D projection explains them, not because they vanished.',
        'Recompute the original rectangular/plane interval using top-compatible evidence, preserving all eligible explanations. After any edge refit, project the OUTPUT model again; validating a pre-fit guess does not authenticate the returned axes/size.',
        'If the candidate set lacks a jointly valid outer shape, add a full-prism silhouette proposal, not a new robot view. It must then pass the same shared family and existing contact/path pipeline.',
        'If an eligible explanation remains unresolved or solver/search coverage is incomplete, return localized precision-unresolved with evidence; do not emit a grasp-qualified model from a best fit alone.'
    ],
    finite_family_implementation_options=dict(
        fixed_theta='For fixed theta and fixed top-height member, each homogeneous camera-image half-plane applied to every projected vertex is a linear inequality in the common center/size offsets (positive optical depth required). This can bound parameter cells without unrelated endpoint choices.',
        theta_height='Retain interval cells for theta and the common top-plane/height family. Bound the full projected vertex/edge variation in each cell using interval arithmetic or proven conservative Lipschitz bounds. A sampled yaw/h grid alone is not a coverage proof.',
        search_coverage='For each possible visible top/bottom/vertical edge assignment retain all feasible components. Search caps or ambiguous unexamined cells remain precision-unresolved rather than silently omitted.',
        downstream='Compress the union to the existing scalar edge/yaw/height budgets conservatively as a first implementation; this preserves existing gripper/path consumers. Keep all member/cell evidence signed. Anisotropic optimization can wait; no need to shrink U to force a pass.',
        common_plane='The top slab induces a common camera homothety, not independently favorable shifts of each side/corner.'),
    evidence_version_contract=dict(
        do_not_relabel='Model-predicted side is not the existing measured top-to-side resolved_sides bool. Old schema1/2 semantics and parsers remain exact.',
        minimal_new_proof='A separately versioned cuboid-projection qualification contains source hashes, shared parameters/cells, visible-face assignment, complete-mask boundary support, current Canny support, raw-depth contradictions/unknown counts and family-derived budgets.',
        validation='Strictly validate fields, all8 shared corners, all feasible members, original source binding and budget arithmetic. Bind the proof into the immutable model digest and existing signed contact/plan lineage.',
        model_shape='RegularBoxInference schema3 (or an explicitly separate qualification kind) may retain center/axes/size/edge/yaw/height attributes so existing consumers need not gain an alternative execution route. Accepting schema3 without validating its new proof is not sufficient.'),
    required_negative_cases=[
        dict(case='GEN37 complete-mask internal-print counterexample',
             required='Remain rejected as a qualified contact model. Equal-gray true external boundary lacks required outer Canny, and theta0 printing does not jointly match true52x35@10deg silhouette. Do not pass by propagating only print yaw. Same saved procedural scene should be replayed unchanged.'),
        dict(case='side-only explanation for arbitrary unknown mask band',
             required='Reject if declared side is outside the full 8-corner hull or its ray first hit is top/ambiguous; full band is retained.'),
        dict(case='table shadow / color print / one long projected line',
             required='Cannot replace a complete shared visible hull; raw support/continuation and outside contradictions remain.'),
        dict(case='one/multiple feasible cuboids',
             required='Do not choose only the narrowest or smallest-yaw cuboid; all feasible cells contribute to one downstream budget.'),
        dict(case='crop/occluder/segmentation hole or spill',
             required='Preserve original margins/visibility/occupancy. A fit inside an incomplete mask is not proof of complete physical top.'),
        dict(case='tilted top / nearly top side points',
             required='Top slab and whole parameter family include tilt; uncertain points remain ambiguous, never reclassified to favor aperture.'),
        dict(case='real source order/delay/duplicate',
             required='Proof remains bound to one exact retained source; frame/order changes cannot overwrite a newer task or trigger repeated motion.'),
        dict(case='valid shape but old fused collision union remains obstructed',
             required='Keep the rejection. Correct shape inference does not authorize deleting admitted history or relaxing strict SAT/path checks.')],
    first_reviewable_implementation_scope=[
        'One same-frame helper for complete prism projection/visible-face qualification and a strict proof validator.',
        'One integration point in the existing regular-box fallback; preserve schema1/2 successes and all original guards.',
        'Existing observe_return plan/execution path consumes the result; no second planning strategy, new observation angle or repeated-view search.',
        'Replay the actual retained failure through candidate generation, full shape qualification, conservative collision enclosure and grasp candidate production. If no valid contact plan results, report that exact remaining producer boundary instead of claiming fixed.',
        'Replay unchanged GEN37 internal-print counterexample and existing GEN31 oblique/side/shadow/crop tests; then relevant handoff/order/end-to-end regressions. Hardware stays outside this offline review.'])
save('review.json', review)
after={p:sha(W/p) for p in before}
assert after==before
save('source_sha256_after.json',after)
boundary=O/'boundary.jsonl';attempts=boundary.read_text().splitlines() if boundary.exists() else []
assert not attempts
inputs=[D/'verification/gen37_release_manifest.json',
    D/'verification/gen37_box_source_replay_r1/actual_mask_contour_rgb_support.json',
    D/'verification/gen37_box_source_replay_r1/current_fit_support.json',
    D/'verification/gen37_interval_budget_review_r1/synthetic_internal_print_counterexample.json',
    W/'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/regular_box_inference.py',
    W/'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/outer_rgb_rectangle.py',
    W/'src/alicia_flexible_grasp_supervisor/tests/test_gen31_oblique_box_independent.py',Path(__file__)]
result=dict(complete=True,passed=True,kind='read_only_contract_review',
    completed_ns=time.time_ns(),source_count=len(before),source_unchanged=True,
    production_modified=False,tests_modified=False,qualified_model=None,
    motion_authorized=False,actual_fitting_performed=False,
    primary_conclusion='完整8角可见轮廓约束是纠正top/side错误归属的具体共同模型；最小实现可以先作为现有候选的联合验证器，不能仅删mask-band硬门或把未知side写成measured resolved。',
    input_sha256={str(p):sha(p) for p in inputs},review_sha256=sha(O/'review.json'),
    external_boundary_attempts=attempts,
    guard_scope='Python socket/dev guard; no ROS, network or hardware calls. Not OS/C isolation.')
save('result.json',result)
files={p.name:sha(p) for p in O.iterdir() if p.is_file() and p.name!='seal.json'}
save('seal.json',dict(complete=True,sealed_ns=time.time_ns(),source_count=1005,
    source_unchanged=True,result_sha256=sha(O/'result.json'),
    review_sha256=sha(O/'review.json'),artifact_sha256=files))
print(json.dumps(dict(result_sha256=sha(O/'result.json'),review_sha256=sha(O/'review.json'),
    seal_sha256=sha(O/'seal.json')),ensure_ascii=False))
