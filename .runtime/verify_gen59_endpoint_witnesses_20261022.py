"""Verify saved witnesses from all full padded CAD vertices, without optimizing."""
from pathlib import Path
import gzip
import hashlib
import itertools
import json
import sys
import time

W=Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
S=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z/gen58_resume_r1/observation_pose_fix/actual_attempt_r1')
O=S/'contact_endpoint_probe_review_r2'
O.mkdir(exist_ok=False)
sys.path[:0]=[str(W/'src/alicia_flexible_grasp_supervisor/src'),'/opt/ros/noetic/lib/python3/dist-packages']
import numpy as np
import scipy
from scipy.spatial import ConvexHull
from alicia_flexible_grasp.robot.observation_path_guard import SerialUrdfFk

start=time.monotonic()
source=S/'contact_endpoint_review_r2/result.json'
assert hashlib.sha256(source.read_bytes()).hexdigest()=='b04890157dbd0b2c809a208661de79e821c003508aceda1d06855716bdc23d3e'
data=json.loads(source.read_text())
probe=W/'.runtime/review_gen59_contact_endpoints_20261022.py'
assert hashlib.sha256(probe.read_bytes()).hexdigest()==data['source_sha256']
(O/'reviewed_probe.py').write_bytes(probe.read_bytes())
for name,expected in data['input_sha256'].items():
    assert hashlib.sha256(Path(name).read_bytes()).hexdigest()==expected
audit=json.loads((S/'geometry_side_review_r1/source_audits/27ac190088e2c5fb1bbacbd1ead32f26f62f7322218682bdc26ca7edc554f822.json').read_text())
review=[]
for row in data['rows']:
    index=row['stable_index']; label=row['proposal']
    window=audit['stable_evaluations'][index]['contact_height_window_search']
    checks={'original':window['original_failure']}
    checks.update({str(i):p['contact_sequence_precheck'] for i,p in zip(window['evaluated_proposal_indices'],window['proposals'])})
    check=checks[label]
    if row.get('replay_available') is False:
        assert index==32 and label=='0' and 'contact_failure_replay' not in check
        review.append(dict(stable_index=index,proposal=label,stored=False));continue
    ref=check['contact_failure_replay']['reference']
    packed=(Path(ref['store_root'])/ref['relative_path']).read_bytes()
    assert hashlib.sha256(packed).hexdigest()==ref['gzip_sha256']
    raw=gzip.decompress(packed)
    assert hashlib.sha256(raw).hexdigest()==row['receipt_sha256']==ref['object_sha256']
    receipt=json.loads(raw)
    assert receipt['binding']['plan_id']==check['plan_id']==row['plan_id']
    assert receipt['first_failed_sample']['curve']=='predicted_command'
    fk=SerialUrdfFk(receipt['model']['xml'],receipt['checked_curve']['joint_names'])
    assert fk.model_sha256==receipt['model']['sha256']
    assert fk.finger_cad_model.provenance==receipt['first_failed_sample']['cad_target_projection']['provenance']
    for path,expected in fk.finger_cad_model.provenance['assets'].items():
        assert hashlib.sha256((W/path).read_bytes()).hexdigest()==expected
    sample=receipt['following_sample']
    assert sample['joint_names']==receipt['checked_curve']['joint_names']
    bias=np.asarray(sample['sdk_positions_rad'])-np.asarray(sample['accepted_positions_rad'])
    assert np.array_equal(bias,np.asarray(row['measured_bias_rad']))
    command=np.asarray(receipt['checked_curve']['points'][-1]['positions'])
    nominal=command-bias
    assert np.max(np.abs(fk(command)-np.asarray(receipt['first_failed_sample']['T_base_tool0'])))<1e-12
    saved={v['kind']:v for v in row['endpoints']}
    for name,q in [('nominal_constant_bias_reconstruction',nominal),('recorded_predicted_command',command)]:
        assert np.max(np.abs(fk(q)-np.asarray(saved[name]['tool_pose_base'])))<1e-12
    cmd=saved['recorded_predicted_command']
    bodies={v['body']:v for v in cmd['bodies']}
    assert bodies['right_finger']['conservative_overlap'] is True
    witness=bodies['right_finger']['independent_convex_hull']
    assert witness['physical_mesh_collision_claimed'] is False
    assert witness['global_optimality_claimed'] is False
    assert witness['separation_proof_claimed'] is False
    x=np.asarray(witness['witness_tool_m'])
    radius=float(witness['intersection_inscribed_radius_m'])
    assert np.all(np.isfinite(x)) and np.isfinite(radius)
    opening=receipt['opening_m']; assert opening==row['opening_m']
    # Independent construction: include every original vertex and all eight
    # padding corners, rather than reducing to the first hull's vertices.
    parts=fk.finger_cad_model._components[1]
    raw_vertices=np.concatenate([p.vertices_at_zero+opening*p.velocity_per_opening for p in parts])
    padding=np.asarray(list(itertools.product((-.0006,.0006),repeat=3)))
    full_vertices=(raw_vertices[:,None,:]+padding).reshape((-1,3))
    full_hull=ConvexHull(full_vertices)
    hull_normals=full_hull.equations[:,:3]
    hull_rhs=-full_hull.equations[:,3]
    hull_margins=hull_rhs-hull_normals@x
    # Independently test the witness against the target in world coordinates.
    T=fk(command); scene=receipt['scene']
    world=T[:3,:3]@x+T[:3,3]
    target_local=np.asarray(scene['rotation']).T@(world-np.asarray(scene['center']))
    target_margins=np.asarray(scene['size'])/2.-np.abs(target_local)
    minimum=float(min(np.min(hull_margins),np.min(target_margins)))
    ball_margin=float(min(np.min(hull_margins-np.linalg.norm(hull_normals,axis=1)*radius),np.min(target_margins-radius)))
    assert radius>1e-8 and minimum>1e-8 and ball_margin>=-1e-9
    assert abs(minimum-witness['witness_minimum_constraint_margin_m'])<1e-10
    assert witness['conclusion']=='strict_convex_enclosure_intersection'
    review.append(dict(stable_index=index,proposal=label,plan_id=row['plan_id'],
        receipt_sha256=row['receipt_sha256'],model_sha256=fk.model_sha256,
        original_vertex_count=len(raw_vertices),full_padded_vertex_count=len(full_vertices),
        independent_hull_facets=len(full_hull.equations),radius_m=radius,
        full_vertex_hull_witness_margin_m=float(np.min(hull_margins)),
        world_target_obb_witness_margin_m=float(np.min(target_margins)),
        minimum_ball_slack_m=ball_margin,checked_witness_strictly_inside_both=True,
        nominal_tool_position_error_m=saved['nominal_constant_bias_reconstruction']['tool_position_error_m'],
        predicted_command_tool_position_error_m=cmd['tool_position_error_m'],
        saved_nominal_all_target_boxes_separate=all(v['conservative_overlap'] is False for v in saved['nominal_constant_bias_reconstruction']['bodies'])))
for name,expected in data['input_sha256'].items():
    assert hashlib.sha256(Path(name).read_bytes()).hexdigest()==expected
report=dict(scope='independent_saved_witness_validation_no_optimizer_no_ROS_no_network_no_bag_no_motion',
    result_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),probe_sha256=data['source_sha256'],
    verification_script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
    all_36_input_hashes_match=True,rows=review,numpy_version=np.__version__,scipy_version=scipy.__version__,
    limitations=['Strict interior witnesses prove conservative padded complete-union convex-hull intersection with target OBB; no physical nonconvex-mesh collision is established.',
        'No solver optimality, separation, full-path feasibility, future actuator following or physical grip is established.',
        'The 3-second objective callback is a cooperative per-solver bound, not a process hard timeout, and excludes hull construction. Actual recorded run completed in 0.163407835 seconds.',
        'Nominal endpoint is reconstructed using fixed SDK-minus-accepted bias; it is not a measured future endpoint.'],
    production_modified=False,elapsed_sec=time.monotonic()-start)
(O/'witness_verification.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
(O/Path(__file__).name).write_bytes(Path(__file__).read_bytes())
print(json.dumps(dict(verified=len([v for v in review if v.get('checked_witness_strictly_inside_both')]),
    unavailable=len([v for v in review if v.get('stored') is False]),elapsed_sec=report['elapsed_sec'],
    result_sha256=hashlib.sha256((O/'witness_verification.json').read_bytes()).hexdigest())))
