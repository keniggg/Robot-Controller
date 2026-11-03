"""Read-only independent contract and exact-point enclosure arithmetic."""
import hashlib
import itertools
import json
import math
from pathlib import Path
import time

import numpy as np

W = Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
E = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
D = E/'ros_log/full_latest_gen36_restart_20261017T012037Z'
G = E/'grasp_records/20261017T070447.445347Z_83343e1525b54994a6f491259995a100'
O = D/'verification/gen40_qualified_enclosure_contract_review_r1'
assert Path('/media/zhuyupei/SanStand').is_mount()
assert not O.exists()
O.mkdir()
inputs = {}


def read(path):
    path = Path(path)
    raw = path.read_bytes()
    digest = hashlib.sha256(raw).hexdigest()
    assert inputs.setdefault(str(path),digest)==digest
    return raw


def load(path):
    return json.loads(read(path))


def write(name,value):
    raw=(json.dumps(value,ensure_ascii=False,indent=2,sort_keys=True,allow_nan=False)+'\n').encode()
    path=O/name
    with path.open('xb') as f:
        f.write(raw)
    assert path.read_bytes()==raw
    return hashlib.sha256(raw).hexdigest()


base='src/alicia_flexible_grasp_supervisor/'
sources=[
    base+'src/alicia_flexible_grasp/vision/object_geometry.py',
    base+'src/alicia_flexible_grasp/vision/fused_scene_geometry.py',
    base+'src/alicia_flexible_grasp/vision/regular_box_inference.py',
    base+'src/alicia_flexible_grasp/grasp/contact_evidence.py',
    base+'src/alicia_flexible_grasp/grasp/contact_execution.py',
    base+'src/alicia_flexible_grasp/grasp/rich_plan_integrity.py',
    base+'msg/ObjectGeometry.msg',
    base+'scripts/remote_grasp6d_node.py',
    base+'tests/test_gen26_inferred_tabletop_candidates.py',
    base+'tests/test_gen31_box_interval_independent.py',
    base+'tests/test_fused_scene_recenter.py',
]
source_hashes={}
for name in sources:
    read(W/name)
    source_hashes[name]=inputs[str(W/name)]
release_path=D/'verification/gen39_release_manifest.json'
release=load(release_path)
source_release_matches={name:source_hashes[name]==release['source_sha256'].get(name)
    for name in sources}

a_path=G/'keyframes_audits/9455bc56d73cbdf2a04ffbf86128abde2c71866ad9b31abe61b477207c174f6f.json'
s_path=D/'verification/gen39_actual_attempt1_source_replay_r1/exact_selector_replay.json'
a=load(a_path);s=load(s_path)
assert inputs[str(a_path)]==a_path.stem
assert inputs[str(s_path)]=='de1dbd6781efc132179b14a376489959a2e6bbf326adf66e927c582ae9354034'
assert s['selected_after_exact_cas'] and not s['raw_model_reused_for_final']
box=a['remote_diagnostics']['tabletop_geometry']['regular_box_inference']
assert box['ok'] and s['final_same_source_full_fusion_model_sha256']==box['model_sha256']
r=a['replay_geometry'];m=s['raw_model']
P=np.asarray(r['object_points_base_m'],dtype=float)
assert P.shape==(2610,3) and np.all(np.isfinite(P))
assert hashlib.sha256(P.tobytes()).hexdigest()==r['object_points_sha256']
A=np.asarray(m['axes_base']);c=np.asarray(m['center_base']);size=np.asarray(m['size_xyz_m'])
assert np.allclose(size,box['inferred_size_xyz_m'],atol=0,rtol=0)
target=next(x['targeted_view'] for x in a['remote_diagnostics']['tabletop_geometry']['surface_evidence_rejections']
    if x['failure_code']=='BOX_INFERENCE_PRECISION_VIEW_REQUIRED')
assert np.allclose(np.mean(target['target_edges_or_corners_base'],axis=0),c,rtol=0,atol=1e-12)
assert np.allclose(A,np.asarray(r['R_base_obb']),rtol=0,atol=0)
e=np.asarray(target['empirical_axis_uncertainty_m']);hu=m['empirical_height_uncertainty_m'];yaw=m['empirical_yaw_uncertainty_rad']
spread=[]
for index,(x,y) in enumerate(((size[0],size[1]),(size[1],size[0]))):
    angle=min(yaw,math.atan2(y,x))
    spread.append(x*math.cos(angle)+y*math.sin(angle)+2.*e[index])
family_half=np.array([spread[0]/2.,spread[1]/2.,(size[2]+hu)/2.])
family_center=c+A[:,2]*hu/2.
signs=np.asarray(list(itertools.product((-1.,1.),repeat=3)))
family_vertices=family_center+(signs*family_half)@A.T
points_local=(P-family_center)@A
lower=np.minimum(-family_half,np.min(points_local,axis=0))
upper=np.maximum(family_half,np.max(points_local,axis=0))
new_center=family_center+A@((lower+upper)/2.)
new_size=upper-lower
assert np.all(np.abs((P-new_center)@A)<=new_size/2.+1e-14)
assert np.all(np.abs((family_vertices-new_center)@A)<=new_size/2.+1e-14)
center_local=(c-new_center)@A
line=(-new_size[1]/2.-center_local[1],new_size[1]/2.-center_local[1])
required=2.*max(-line[0]+.0005,line[1]+.0005)
assert abs(required-target['required_open_upper_m'])<1e-12
outside=np.maximum(points_local.max(axis=0)-family_half,-family_half-points_local.min(axis=0))
assert outside[0]>.0009 and outside[0]<.001
assert outside[1]<0. and outside[2]<0.

recorded=dict(point_count=len(P),point_order_plain_f64_sha256=r['object_points_sha256'],
    model_sha256=box['model_sha256'],source_model_evidence_role='Raw-qualified model shape parameters match final shape; final exact model SHA is independently matched by exact selector/fusion replay. This reviewer does not reuse raw model for final authority.',
    point_interval_model_coordinates_m=[((P-c)@A).min(axis=0).tolist(),((P-c)@A).max(axis=0).tolist()],
    collision_family_center_m=family_center.tolist(),collision_family_half_extent_m=family_half.tolist(),
    point_extension_beyond_collision_family_by_axis_m=outside.tolist(),
    proposed_exact_points_plus_complete_family_center_m=new_center.tolist(),
    proposed_exact_points_plus_complete_family_size_m=new_size.tolist(),
    same_model_center_narrow_jaw_line_interval_m=list(line),fit_each_side_m=.0005,
    proposed_union_scalar_required_m=required,old_union_scalar_required_m=.050884430837433874,
    all_2610_points_contained=True,complete_family_eight_extreme_vertices_contained=True,
    original_points_altered=False,gate_U_or_height_reduced=False,
    nominal_center_moved_to_fit_jaws=False,geometry_returned_to_production=False,
    materialization_typed_evidence_CAD_path_or_motion_proven=False)

result=dict(schema_version=1,kind='gen40_qualified_planning_enclosure_independent_contract_review',
    recorded_ns=time.time_ns(),reviewed=True,diagnostic_only=True,production_or_tests_changed=False,
    geometry_altered_in_runtime=False,ros_contacted=False,hardware_contacted=False,motion_authorized=False,
    release_review_complete=False,proposed_contract_implemented_or_tested=False,
    source_sha256=source_hashes,source_matches_gen39_release=source_release_matches,evidence_sha256=dict(inputs),
    facts=[
        dict(path=sources[0],lines='684-779',finding='The old OBB is not pure PCA: projected 2/98-percentile filtering, minAreaRect with PCA edge orientation selection, support-anchored 2/98-percentile height. No independently calibrated occupancy/error padding is introduced here.'),
        dict(path=sources[1],lines='1-6,63-81,128-178,186-188',finding='Generic fused v2 intentionally preserves old box and all exact admitted distinct points, fixes axes/support and outward height, and explicitly disclaims unseen-shape proof. Existing corner-retention tests codify this representation contract.'),
        dict(path=sources[2],lines='351-365,381-428,646-674',finding='Model source geometry hash includes original center/axes/size/support/ratio and measured point hash. Existing inferred enclosure unions old OBB corners with complete model family. Qualifier model geometry is inferred from raw pixels/support; old center/axes/size appear only in source payload, not fitting/shape uncertainty.'),
        dict(path=sources[6],lines='1-19',finding='ROS ObjectGeometry transports pose/size/support and counts, not complete measured points or their hash.'),
        dict(path=sources[3],lines='152-194',finding='Consumer binds plan source stamp/track/source support and verifies every complete model-family extreme vertex inside plan box. It cannot recompute containment of all measured source points because those points are absent.'),
        dict(path=sources[4],lines='97-125',finding='Final measured aperture freezes the plan OBB and typed model, and checks new measured FK/opening. It trusts the producer-supplied signed box to preserve measured occupancy; it has no separate point cloud.'),
        dict(path=sources[5],lines='358-386',finding='Plan content hash freezes the exact OBB and contact evidence. A content hash protects against later change; it does not prove the producer used all source points or that supplied extrema are correct.'),
    ],
    recorded_counterfactual=recorded,
    recommended_minimal_contract=dict(
        scope='A qualified regular-box planning representation only; generic measured-only/unqualified fused v2 remains unchanged.',
        immutable_inputs='Keep original same-source fused GeometryEstimate, all exact points/order/provenance, and qualified model. Require exact regular_box_geometry_sha256(original)==model.source.geometry_sha256 plus original source identity/stamp/coordinate/support binding.',
        coordinate_requirement='All model family and measured points must be in one validated coordinate frame. For transformed models use verified corresponding transformation; never assume source frame equals registered frame.',
        measured_gate='Re-evaluate original full measured-family consistency with all points and unchanged 1 mm allowance. Reject a true geometry conflict; do not trim it away.',
        construction='Take exact extrema of complete empirical yaw/axis/height family extreme vertices and every real admitted point in the qualified axes; use their midpoint and full extent. No old nominal OBB empty corners, no point deletion, no percentile refit, no numerical U shrink.',
        checks='Validate finite right-handed axes, support/source, unchanged original physical size/height limits, exact point equality/hash, every point and family extreme vertex contained at numerical tolerance only.',
        audit='Explicit new policy/version and original geometry hash/OBB retained for diagnosis; record new box and verified full point/family containment. Do not label this generic v2 outward-only or original_obb_preserved.',
        freeze='The same newly verified geometry must feed candidate reach, finite CAD/sweep, MoveIt/MuJoCo, signed plan, stage aperture/contact and close checks. Never only shrink the producer aperture representation.',
        trust_boundary='With present wire format, full-point proof is enforced at the trusted producer before freezing/signed handoff; downstream independently checks family and unchanged frozen OBB, not all original points.',
        independent_consumer_requirement='If downstream independent all-point verification is required, transport the complete immutable source-point payload or resolvable local CAS reference and verify raw identity/hash/geometry payload and all extrema. A count, digest, or producer-written bounds alone is insufficient.',
    ),
    alternative_review=dict(
        proposal='Refit a measured OBB in qualified axes using all exact measured points, then use original enclose and requalify.',
        possible=True,
        caveat='Changing source geometry center/axes/size changes model.source.geometry_sha256; rerun the real qualifier over retained original pixels/support/all points and bind a new model, never merely rehash old model.',
        source_lineage='Keep original admitted geometry and its digest plus explicit derived representation lineage; do not claim source geometry unchanged after refitting.',
        circularity='The initial qualified axes come from raw pixels/support independently of old geometry center/axes/size; a second real qualification must still confirm the final shape/family and source. No promise of identical model or a favorable fit.',
        drawback='More source/qualification handoff and compute than direct source-bound planning construction; still does not by itself add missing complete points to the ROS consumer.',
        preference='Direct qualified planning construction preserves original source geometry and is the smaller contract if trusted producer ownership is accepted.'),
    blocking_findings=[
        'Blocking only for a claim of current downstream-independent complete-point verification: current plan/wire lacks points/CAS; family-contained rehashed box can exclude a measured point outside family without that consumer detecting it.',
        'Changing generic measured fused v2 or silently discarding old corners without a qualified complete shape and explicit new representation contract is not justified.',
    ],
    nonblocking_findings=[
        'No independently calibrated unseen-occupancy/error contract for the old minAreaRect corners was found in the reviewed producer. Their preservation is an existing conservative representation policy, not additional raw measurement.',
        'Qualified complete-prism family plus all true points can form a conservative representation under the existing qualified-shape assumptions, with all existing uncertainties and physical gates preserved.',
        'Actual data require real-point extension along long axis by 0.9027995 mm; only using model family would lose that real occupancy.',
        'This arithmetic does not show that new geometry passes materialization, contact, continuous path, current-frame freshness, or actual physical grasp.',
    ],
    required_negative_validation=[
        'Reject wrong source geometry digest, wrong source stamp/identity/support or unaccounted coordinate/registration transform.',
        'Reject reordered/deleted/changed source points; all actual point extrema including the real long-axis extension must remain.',
        'Retain complete analytic yaw extrema, axis/height U and every unresolved source association; no favorable interval or frame selection.',
        'Keep original >1 mm full-measured-family conflict refusal and unqualified/generic-v2 old-corner contract.',
        'Freeze exact same representation across producer, typed signed plan, aperture/CAD/path consumers; test a plan box that contains model family but cuts a real point outside family.',
        'Replay actual record end-to-end offline plus adversarial source/geometry changes, then original comprehensive regression; no scalar pass is a motion token.',
    ])
for path,digest in inputs.items():
    assert hashlib.sha256(Path(path).read_bytes()).hexdigest()==digest,path
result['reviewed_inputs_unchanged']=True
result_sha=write('result.json',result)
seal_sha=write('seal.json',dict(schema_version=1,review_completed=True,release_review_complete=False,
    diagnostic_only=True,motion_authorized=False,artifacts_sha256={'result.json':result_sha},
    script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
    source_sha256=source_hashes))
print(json.dumps(dict(result=str(O/'result.json'),result_sha256=result_sha,seal_sha256=seal_sha,
    proposed_scalar_required_m=required,independent_consumer_gap=True),ensure_ascii=False))
