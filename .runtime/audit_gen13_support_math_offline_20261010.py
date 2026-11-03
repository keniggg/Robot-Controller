"""Read sealed GEN13 inputs; audit support math without ROS or device access."""
from pathlib import Path
from types import SimpleNamespace
from datetime import datetime, timezone
import hashlib
import json
import sys

import cv2
import numpy as np

WORK = Path(__file__).resolve().parents[1]
ROOT = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
NEW = ROOT / 'ros_log/full_latest_restart_20261010T002347Z'
OUT = NEW / 'gen13_handoff_analysis/offline_support_math'
REPLAY = NEW / 'gen13_handoff_analysis/support_plane_replay/report.json'
assert Path('/media/zhuyupei/SanStand').is_mount(), 'SanStand must be mounted'
sys.path.insert(0, str(WORK / 'src/alicia_grasp_modes/src'))
sys.path.insert(0, '/opt/ros/noetic/lib/python3/dist-packages')
from alicia_grasp_modes import tabletop
import rosbag  # File reader only. No ROS node, master or topic access.

cv2.setNumThreads(1)
sha = lambda p: hashlib.sha256(Path(p).read_bytes()).hexdigest()
replay = json.loads(REPLAY.read_text())
source = Path(tabletop.__file__)
assert sha(source) == replay['source_files']['tabletop_sha256']
cfg = tabletop.Config(**replay['config'])
reference = np.asarray(replay['reference']['plane_base'])
anchor = np.asarray(replay['reference']['anchor_base'])
footprint = np.asarray(replay['reference']['footprint_base'])
rng = np.random.RandomState(1313)


def normplane(p):
    p = np.asarray(p, dtype=float)
    return p / np.linalg.norm(p[:3])


def independent_metrics(ref, cur, point, polygon):
    ref, cur = normplane(ref), normplane(cur)
    if np.dot(ref[:3], cur[:3]) < 0:
        cur = -cur
    # Deliberately evaluate two distance fields independently; do not copy
    # the production implementation's coefficient-difference expression.
    ref_dist = polygon.dot(ref[:3]) + ref[3]
    cur_dist = polygon.dot(cur[:3]) + cur[3]
    point_dist = float(np.dot(cur[:3], point) + cur[3]
                       - np.dot(ref[:3], point) - ref[3])
    normal_dot = float(np.dot(ref[:3], cur[:3]))
    max_shift = max(abs(point_dist), float(np.max(np.abs(cur_dist-ref_dist))))
    angle = float(np.degrees(np.arctan2(np.linalg.norm(np.cross(ref[:3],cur[:3])),normal_dot)))
    return max_shift, angle, point_dist, cur_dist-ref_dist


def production_outcome(ref, cur, point, polygon):
    try:
        _, metrics = tabletop._validate_support_plane_update(ref,cur,point,cfg,polygon)
        return {'accepted': True, 'metrics': metrics}
    except ValueError as exc:
        return {'accepted': False, 'reason': str(exc)}


def rigid_plane(p, R, t):
    p = normplane(p)
    n = R.dot(p[:3])
    return np.r_[n, p[3]-n.dot(t)]


rows = []
initial_reconstruction = None
covariance_checks = 0
scale_sign_checks = 0
max_covariance_error = 0.
for frame_index, frame in enumerate(replay['frames']):
    path = Path(frame['source_file'])
    assert sha(path) == frame['source_file_sha256']
    with np.load(path, allow_pickle=False) as z:
        assert np.array_equal(z['reference_plane_base'], reference)
        assert np.array_equal(z['anchor_base'], anchor)
        assert np.array_equal(z['footprint_base'], footprint)
        depth = z['depth_raw'].astype(np.float32) * float(z['depth_scale'])
        T = z['T_base_camera'].copy()
        K = z['intrinsics'].copy()
    if frame_index == 0:
        acquired = tabletop.segment(depth, K, T, cfg)
        tracker = tabletop.TargetTracker(cfg)
        chosen, reason = tracker.choose(acquired, depth.shape)
        assert reason == 'acquired' and chosen is not None
        assert chosen.support_footprint_base.shape == footprint.shape
        errors = dict(plane_max_abs_error=float(np.max(np.abs(tracker.plane_base-reference))),
            anchor_max_abs_error=float(np.max(np.abs(chosen.position_base-anchor))),
            footprint_max_abs_error=float(np.max(np.abs(chosen.support_footprint_base-footprint))))
        assert max(errors.values()) < 1e-12
        initial_reconstruction = dict(reason=reason,candidate_count=len(acquired.candidates),
            selected_candidate_matches_frozen_reference=True,**errors)
    # Repeat the exact bounded ring fit, using its sealed input center. Pixel
    # projection/calibration review is assigned to a different agent.
    vv, uu = np.indices(depth.shape)
    center = np.asarray(frame['ring_center_px'])
    valid = np.isfinite(depth) & (depth >= cfg.depth_min_m) & (depth <= cfg.depth_max_m)
    radius = np.hypot(uu-center[0], vv-center[1])
    scale = min(depth.shape[1]/640.,depth.shape[0]/480.)
    pc, ratio, residual, count, _, _, _ = tabletop._fit_support_ring_sequence(
        depth,K,valid,radius,scale,cfg)
    assert np.max(np.abs(pc-np.asarray(frame['plane_camera']))) < 1e-12
    # Independent rigid-plane equation, versus production inverse transpose.
    cur = rigid_plane(pc,T[:3,:3],T[:3,3])
    via_inverse_transpose = normplane(np.linalg.solve(T.T,pc))
    plane_transform_error = float(np.max(np.abs(cur-via_inverse_transpose)))
    assert plane_transform_error < 1e-12
    maximum,angle,anchor_delta,distances = independent_metrics(reference,cur,anchor,footprint)
    expected = frame['reference_delta']['maximum_local_shift_mm'] / 1000.
    assert abs(maximum-expected) < 1e-12
    outcome = production_outcome(reference,cur,anchor,footprint)
    assert outcome['accepted'] == (frame_index < 2)
    if outcome['accepted']:
        assert abs(outcome['metrics']['support_reference_max_distance_m']-maximum) < 1e-12
    else:
        assert outcome['reason'] == frame['replayed_audit_rejection']
    ref = normplane(reference)
    if cur[:3].dot(ref[:3]) < 0: cur=-cur
    # On the original plane, intersect its normal ray with the current plane.
    along_reference_normal = -distances / np.dot(cur[:3],ref[:3])
    along_base_z = -distances / cur[2]
    anchor_height = float(anchor.dot(ref[:3])+ref[3])
    anchor_projection = anchor-anchor_height*ref[:3]
    projected_anchor_delta = float(anchor_projection.dot(cur[:3])+cur[3])
    for a in (.125,7.3,-.75,-11.):
        for b in (.5,3.25,-2.):
            scaled = production_outcome(a*reference,b*cur,anchor,footprint)
            assert scaled['accepted'] == outcome['accepted']
            m,_,_,_=independent_metrics(a*reference,b*cur,anchor,footprint)
            assert abs(m-maximum)<1e-12
            scale_sign_checks += 1
    # Common rigid-frame changes must transform both planes and all anchors.
    for _ in range(60):
        R,_=np.linalg.qr(rng.normal(size=(3,3)))
        if np.linalg.det(R)<0: R[:,0]*=-1
        t=rng.uniform(-10.,10.,3)
        r2,c2=rigid_plane(reference,R,t),rigid_plane(cur,R,t)
        a2=R.dot(anchor)+t
        f2=footprint.dot(R.T)+t
        m,_,_,_=independent_metrics(r2,c2,a2,f2)
        error=abs(m-maximum)
        assert error<1e-12
        assert production_outcome(r2,c2,a2,f2)['accepted']==outcome['accepted']
        max_covariance_error=max(max_covariance_error,error)
        covariance_checks+=1
    # The absolute value of an affine field over a convex polygon achieves
    # its maximum at a vertex. Interior/boundary combinations cannot exceed it.
    weights=rng.dirichlet(np.ones(len(footprint)),size=400)
    interior=weights.dot(footprint)
    dense=np.abs(interior.dot(cur[:3])+cur[3]-(interior.dot(ref[:3])+ref[3]))
    assert float(dense.max()) <= float(np.max(np.abs(distances)))+1e-12
    rows.append(dict(station='initial' if frame_index==0 else 'first_view' if frame_index==1 else 'second_view',
        source_stamp_ns=frame['source_stamp_ns'],source_file=str(path),source_file_sha256=sha(path),
        normal_angle_deg=angle,plane_transform_max_abs_error=plane_transform_error,
        independent_maximum_mm=maximum*1000,production_outcome=outcome,
        coefficient_offset_difference_mm=abs(cur[3]-ref[3])*1000,
        signed_anchor_distance_difference_mm=anchor_delta*1000,
        anchor_height_above_reference_mm=anchor_height*1000,
        anchor_projection_distance_mm=projected_anchor_delta*1000,
        off_plane_anchor_effect_mm=(anchor_delta-projected_anchor_delta)*1000,
        footprint_maximum_mm=float(np.max(np.abs(distances)))*1000,
        reference_normal_height_maximum_mm=float(np.max(np.abs(along_reference_normal)))*1000,
        base_z_height_maximum_mm=float(np.max(np.abs(along_base_z)))*1000,
        fitted_inlier_ratio=ratio,fitted_inlier_rms_mm=residual*1000,fit_samples=count))

# Threshold and intersecting-plane examples are independent geometric cases;
# production gates always retain their original Config and are never relaxed.
square=np.array([[-.02,-.02,0.],[.02,-.02,0.],[.02,.02,0.],[-.02,.02,0.]])
synthetic=[]
for separation in (0.,.0039999999,.004,.0040000001,.0057):
    p=np.array([0.,0.,1.,-separation])
    outcome=production_outcome([0,0,1,0],p,[0,0,.04],square)
    assert outcome['accepted'] == (separation <= .004+1e-12)
    synthetic.append(dict(name='parallel_plane_separation',separation_m=separation,outcome=outcome))
theta=np.deg2rad(3.)
tilted=np.array([np.sin(theta),0.,np.cos(theta),0.])
long_polygon=square.copy();long_polygon[:,0]*=5
outcome=production_outcome([0,0,1,0],tilted,[0,0,0],long_polygon)
assert not outcome['accepted'] and 'local_distance' in outcome['reason']
synthetic.append(dict(name='planes_cross_at_anchor_but_disagree_at_footprint_edge',
    anchor_distance_m=0.,edge_distance_m=float(.1*np.sin(theta)),outcome=outcome))

# Verify frozen provenance independently across EVERY stored GEN13 status.
rec=Path(replay['record'])
assert json.loads((rec/'manifest.json').read_text())['recorder_closed']
status_count=0
provenance_rows=[]
with rosbag.Bag(str(rec/'keyframes.bag'),'r') as bag:
    for _,msg,received in bag.read_messages(topics=['/perception/unknown/detector_status']):
        d=json.loads(msg.data)
        if d.get('generation')!=13: continue
        status_count+=1
        if 'reference_support_plane_base' not in d: continue
        same=(np.array_equal(reference,np.asarray(d['reference_support_plane_base']))
              and np.array_equal(anchor,np.asarray(d['anchor_position_base']))
              and np.array_equal(footprint,np.asarray(d['reference_support_footprint_base']))
              and d['anchor_source_stamp_ns']==replay['reference']['source_stamp_ns'])
        assert same, ('unexpected frozen provenance change',received.to_nsec())
        provenance_rows.append(dict(received_ns=received.to_nsec(),source_stamp_ns=d.get('source_stamp_ns'),
            state=d.get('state'),detail=d.get('detail'),all_four_reference_fields_bitwise_equal=True))
assert len(provenance_rows)>=5

report=dict(schema='gen13_offline_support_math_v1',created_utc=datetime.now(timezone.utc).isoformat(),
    all_checks_passed=True,script=str(Path(__file__).resolve()),script_sha256=sha(__file__),
    source_path=str(source),source_sha256=sha(source),replay_path=str(REPLAY),replay_sha256=sha(REPLAY),
    frozen_plane=reference.tolist(),frozen_anchor=anchor.tolist(),
    frozen_footprint_vertices=len(footprint),
    frozen_footprint_max_reference_residual_m=float(np.max(np.abs(footprint.dot(normplane(reference)[:3])+normplane(reference)[3]))),
    frames=rows,synthetic_cases=synthetic,initial_acquisition_reconstruction=initial_reconstruction,
    invariance=dict(sign_and_scale_cases=scale_sign_checks,common_rigid_frame_cases=covariance_checks,
        maximum_common_frame_error_m=max_covariance_error,convex_interior_checks=400*len(rows)),
    provenance=dict(total_generation13_statuses=status_count,with_frozen_reference=len(provenance_rows),
        every_frozen_reference_bitwise_unchanged=True,rows=provenance_rows),
    exact_metric='max(abs(signed_distance_current(anchor)-signed_distance_reference(anchor)), max(abs(signed_distance_current(footprint)-signed_distance_reference(footprint))))',
    metric_interpretation='Planes are co-normalized and sign-aligned. Footprint lies on reference plane, so footprint terms are current-plane perpendicular distances. At an elevated anchor it is a difference of signed distance fields, not simply distance to the current plane. This is not raw offset coefficient subtraction or literal base-z height.',
    reference_update_code=dict(acquisition='tabletop.py:581-594 freezes initial selected candidate and result plane; only initial acquisition writes these fields.',
        rgb_acquisition='unknown_tabletop_perception.py:3085 and 3225 alternate initialization requires tracker.anchor is None; it is not a later-view refresh.',
        arrival='observation_arrival_confirmation.py:103 copies original plane; unknown_tabletop_perception.py:2794 commits last only.',
        recovery='Staged recovery trackers copy original anchor and plane; no current measured plane is installed as frozen reference.'),
    findings=dict(deterministic_math_bug_reproduced=False,current_generation_wrong_reference_update_found=False,
        local_error_is_origin_invariant=True,raw_d_comparison_would_be_wrong=True,
        actual_gate_is_footprint_dominated=all(r['footprint_maximum_mm']>=abs(r['signed_anchor_distance_difference_mm'])-1e-9 for r in rows),
        second_station_failure_persists_with_literal_normal_or_z_height=True),
    limitations=['Only sealed GEN13 and named code paths verified; no claim of physical camera or robot calibration accuracy.',
        'Sub-millimeter fitted inlier RMS is precision around that fitted plane, not accuracy of its base-frame position.',
        'Fixed initial plane is a software invariant; its physical accuracy and whether a single plane adequately models the surface remain unproven.',
        'Distance-field and literal height metrics can differ at a threshold; in these actual frames all second-view variants exceed 4 mm. No threshold or runtime behavior changed.'],
    restrictions=dict(ros_master_or_devices_accessed=False,live_source_modified=False,temperature_accessed=False,thresholds_modified=False))
OUT.mkdir(parents=True,exist_ok=True)
(OUT/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
print(json.dumps(dict(report=str(OUT/'report.json'),sha256=sha(OUT/'report.json'),
    all_checks_passed=True,frozen_reference_statuses=len(provenance_rows),
    max_frame_error_m=max_covariance_error,frames=[{k:r[k] for k in ('station','independent_maximum_mm','off_plane_anchor_effect_mm','reference_normal_height_maximum_mm','base_z_height_maximum_mm')} for r in rows]),ensure_ascii=False))
