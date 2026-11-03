"""Offline endpoint geometry diagnostic; no motion or production edits."""
from pathlib import Path
import gzip
import hashlib
import itertools
import json
import sys
import time

W = Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
S = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z/gen58_resume_r1/observation_pose_fix/actual_attempt_r1')
A = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/grasp_records/20261022T044426.800138Z_c2cd1e6e1a8d4226ab4af1eaaa6646ff/keyframes_audits/27ac190088e2c5fb1bbacbd1ead32f26f62f7322218682bdc26ca7edc554f822.json')
RELEASE = S.parents[2]/'verification/gen59_release_manifest.json'
assert hashlib.sha256(RELEASE.read_bytes()).hexdigest() == '5fed4cbed7ea1e824eda3bb750657970dcc1ec2d711cd041eff1a3d482ef2c6c'
source_map = json.loads(RELEASE.read_text())['source_sha256']
for rel, sha in source_map.items():
    if rel.startswith('src/alicia_flexible_grasp_supervisor/src/'):
        assert hashlib.sha256((W/rel).read_bytes()).hexdigest() == sha
sys.path[:0] = [str(W/'src/alicia_flexible_grasp_supervisor/src'), '/opt/ros/noetic/lib/python3/dist-packages']
import numpy as np
from scipy.spatial import ConvexHull
from scipy.optimize import minimize
from tf.transformations import quaternion_matrix
from alicia_flexible_grasp.robot import observation_path_guard as guard
from alicia_flexible_grasp.grasp.gripper_geometry import GripperGeometry, _stage_boxes

start = time.monotonic()
assert hashlib.sha256(A.read_bytes()).hexdigest() == A.stem
audit = json.loads(A.read_text())
output = S/'contact_endpoint_review_r2'
output.mkdir(exist_ok=False)
references = {str(A): A.stem, str(RELEASE): hashlib.sha256(RELEASE.read_bytes()).hexdigest()}
for module in list(sys.modules.values()):
    p = getattr(module, '__file__', None)
    if p and str(W) in p and Path(p).is_file():
        references[p] = hashlib.sha256(Path(p).read_bytes()).hexdigest()
        relative = str(Path(p).relative_to(W))
        if relative.startswith('src/'):
            assert references[p] == source_map[relative]

def receipt(ref):
    p = Path(ref['store_root'])/ref['relative_path']
    compressed = p.read_bytes()
    assert hashlib.sha256(compressed).hexdigest() == ref['gzip_sha256']
    raw = gzip.decompress(compressed)
    assert hashlib.sha256(raw).hexdigest() == ref['object_sha256']
    references[str(p)] = ref['gzip_sha256']
    return json.loads(raw)

def independent_hull_overlap(fk, opening, body, transform, scene):
    # Preserve the complete active/reference union, original opening, and
    # original 0.6 mm tool-axis padding. This is a numerical diagnostic only.
    parts = fk.finger_cad_model._components[('left_finger','right_finger').index(body)]
    vertices = np.concatenate([p.vertices_at_zero+opening*p.velocity_per_opening for p in parts])
    initial_hull = ConvexHull(vertices)
    offsets = np.asarray(list(itertools.product((-.0006,.0006),repeat=3)))
    padded = (vertices[initial_hull.vertices,None,:]+offsets).reshape(-1,3)
    hull = ConvexHull(padded)
    target_center = transform[:3,:3].T@(scene.center-transform[:3,3])
    target_rotation = transform[:3,:3].T@scene.rotation
    normals = np.vstack((hull.equations[:,:3], target_rotation.T, -target_rotation.T))
    rhs = np.concatenate((-hull.equations[:,3],
        scene.size/2.+target_rotation.T@target_center,
        scene.size/2.-target_rotation.T@target_center))
    # Seek a strictly interior witness; no optimality/separation claim uses
    # the solver status. Independently verify against every original facet.
    constraints = np.column_stack((normals,np.linalg.norm(normals,axis=1)))
    initial = np.append(target_center, np.min(rhs-normals@target_center)-.001)
    optimize_start = time.monotonic()
    def objective(x):
        if time.monotonic()-optimize_start>3.:
            raise TimeoutError('bounded diagnostic witness solve exceeded 3 seconds')
        return -x[3]
    lp = minimize(objective,initial,jac=lambda x:np.asarray([0.,0.,0.,-1.]),method='SLSQP',
        constraints=[dict(type='ineq',fun=lambda x:rhs-constraints@x,jac=lambda x:-constraints)],
        options={'ftol':1e-11,'maxiter':80})
    result = dict(scope='independent_complete_padded_CAD_union_convex_hull_vs_target_OBB',
        solver_success=bool(lp.success), solver_message=str(lp.message), execution_authorized=False,
        physical_mesh_collision_claimed=False, facet_count=len(hull.equations),
        global_optimality_claimed=False, diagnostic_solver='bounded_SLSQP_witness_search',
        entire_original_union_and_padding_retained=True)
    if lp.success:
        assert np.all(np.isfinite(lp.x))
        ball_margin = float(np.min(rhs-normals@lp.x[:3]-np.linalg.norm(normals,axis=1)*lp.x[3]))
        witness_margin = float(np.min(rhs-normals@lp.x[:3]))
        strict = lp.x[3]>1e-8 and witness_margin>1e-8 and ball_margin>=-1e-9
        result.update(intersection_inscribed_radius_m=float(lp.x[3]),
            witness_tool_m=lp.x[:3].tolist(),
            witness_minimum_constraint_margin_m=witness_margin,
            inscribed_ball_minimum_constraint_margin_m=ball_margin,
            conclusion='strict_convex_enclosure_intersection' if strict else 'boundary_or_numerically_unresolved',
            separation_proof_claimed=False)
    return result

rows=[]
for index in (32,34,36):
    stable = audit['stable_evaluations'][index]
    window = stable['contact_height_window_search']
    indices = window['evaluated_proposal_indices']
    assert len(indices)==len(window['proposals']) and len(set(indices))==len(indices)
    checks = [('original',window['original_failure'])]+[(str(i),p['contact_sequence_precheck']) for i,p in zip(indices,window['proposals'])]
    for label,check in checks:
        replay = check.get('contact_failure_replay',{})
        if replay.get('status')!='stored':
            rows.append(dict(stable_index=index,proposal=label,reason=check.get('reason'),replay_available=False));continue
        ref = replay['reference'];d=receipt(ref)
        assert d['first_failed_sample']['curve']=='predicted_command'
        assert d['checked_curve']['joint_names']==d['following_sample']['joint_names']
        assert d['checked_curve']['points'][-1]['positions']==d['first_failed_sample']['joint_positions_rad']
        for name, sha in d['config']['replay_source_sha256'].items():
            p=W/'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot'/name
            assert hashlib.sha256(p.read_bytes()).hexdigest()==sha
            references[str(p)]=sha
        fk = guard.SerialUrdfFk(d['model']['xml'],d['checked_curve']['joint_names'])
        assert fk.model_sha256==d['model']['sha256'] and fk.finger_cad_model is not None
        assert fk.finger_cad_model.provenance == d['first_failed_sample']['cad_target_projection']['provenance']
        for rel,sha in fk.finger_cad_model.provenance['assets'].items():
            assert hashlib.sha256((W/rel).read_bytes()).hexdigest()==sha
            references[str(W/rel)]=sha
        scene=guard.FrozenObservationScene(**{k:np.asarray(v) if k in ('normal','center','rotation','size') else v for k,v in d['scene'].items()})
        gripper=GripperGeometry(**d['config']['evaluated_gripper'])
        sample=d['following_sample'];bias=np.asarray(sample['sdk_positions_rad'])-sample['accepted_positions_rad']
        command=np.asarray(d['checked_curve']['points'][-1]['positions']);nominal=command-bias
        T=fk(command)
        assert np.max(np.abs(T-np.asarray(d['first_failed_sample']['T_base_tool0'])))<1e-12
        pose=d['plan_data']['poses_xyz_xyzw'][2];goal=quaternion_matrix(pose[3:]);goal[:3,3]=pose[:3]
        endpoint=[]
        for kind,q in [('nominal_constant_bias_reconstruction',nominal),('recorded_predicted_command',command)]:
            transform=fk(q);bodies=[]
            for body,center,size in _stage_boxes(transform,gripper,d['opening_m'],'y','z'):
                overlap,proof=guard.gripper_target_overlap(fk,gripper,d['opening_m'],body,transform,center,size,scene)
                row=dict(body=body,conservative_overlap=bool(overlap),cad_proof_status=None if proof is None else proof['status'])
                if overlap and body in ('left_finger','right_finger'):
                    row['independent_convex_hull']=independent_hull_overlap(fk,d['opening_m'],body,transform,scene)
                bodies.append(row)
            endpoint.append(dict(kind=kind,tool_pose_base=transform.tolist(),
                tool_position_error_m=float(np.linalg.norm(transform[:3,3]-goal[:3,3])),
                displacement_in_goal_tool_axes_m=(goal[:3,:3].T@(transform[:3,3]-goal[:3,3])).tolist(),bodies=bodies))
        rows.append(dict(stable_index=index,track_id=stable['tracking']['track_id'],proposal=label,
            plan_id=d['binding']['plan_id'],receipt_sha256=ref['object_sha256'],opening_m=d['opening_m'],
            measured_bias_rad=bias.tolist(),nominal_reconstruction_not_measured=True,
            original_failure_kind=d['first_failed_sample']['validation_kind'],endpoints=endpoint))

for p,sha in references.items():
    assert hashlib.sha256(Path(p).read_bytes()).hexdigest()==sha
result=dict(scope='frozen_terminal_endpoint_geometry_no_ROS_no_motion_no_path_acceptance',
    source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),input_sha256=references,
    rows=rows,elapsed_sec=time.monotonic()-start,production_modified=False,physical_success=False)
(output/'result.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(dict(output=str(output/'result.json'),rows=len(rows),elapsed_sec=result['elapsed_sec'])))
